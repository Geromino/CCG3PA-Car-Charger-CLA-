/**
 * @file ccg_sync.c
 *
 * @brief @{Logic to synchronize state between CCG5s on an Ice Lake system}
 *
 *******************************************************************************
 *
 * Copyright (2014-2016), Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All rights reserved.
 *
 * This software, including source code, documentation and related materials
 * (“Software”), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software (“EULA”).
 *
 * If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress’s integrated circuit
 * products. Any reproduction, modification, translation, compilation, or
 * representation of this Software except as specified above is prohibited
 * without the express written permission of Cypress. Disclaimer: THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the
 * right to make changes to the Software without notice. Cypress does not
 * assume any liability arising out of the application or use of the Software
 * or any product or circuit described in the Software. Cypress does not
 * authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death (“High Risk Product”). By
 * including Cypress’s product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 */

#include <project.h>
#include <config.h>
#include <ccg_sync.h>
#include <pd.h>
#include <dpm.h>
#include <gpio.h>
#include <hpi.h>
#include <ucsi.h>
#include <pdss_hal.h>
#include <timer.h>
#include <ucsi_internal.h>
#include <alt_modes_mngr.h>

#if CCG_SYNC_ENABLE

/* Number of times to retry DPM commands */
#define CCG_SYNC_DPM_RETRY_CNT                (10)
/* 900mA definition in 10mA units */
#define I_900mA                               (90)
    
/* I2C slave addresses for both ports */
static uint8_t gl_local_addr = CCG_SYNC_D0_SLAVE_ADDR,
               gl_remote_addr = CCG_SYNC_D1_SLAVE_ADDR;

/* Data sync packets */
static volatile ccg_sync_pkt_t gl_remote_dev_pkt, gl_local_dev_pkt;
    
/* Remote port status */
static uint8_t gl_remote_port_en = 0x00,
               gl_remote_port_conn = 0x00;

#if CCG_SYNC_ALGORITHM == V1
static uint8_t gl_remote_high_pwr_port = 0x00;

/* Track the port that's advertising highest power */
static int8_t gl_local_high_pwr_port = -1;
#endif /* CCG_SYNC_ALGORITHM == V1 */

/* Flag to indicate if part should to sleep */
static uint8_t gl_sleep_allowed = true;

/* Flag to know if this CCG5 is a 'preferred' controller. See app_evt_handler for details */
static uint8_t gl_preferred_controller = false;

/* Retry count for DPM access */
static uint8_t gl_dpm_retry_count[NO_OF_TYPEC_PORTS] = { CCG_SYNC_DPM_RETRY_CNT
#if CCG_PD_DUALPORT_ENABLE    
    , 
    CCG_SYNC_DPM_RETRY_CNT 
#endif /* CCG_PD_DUALPORT_ENABLE */
};

/* Source PDO that gets modified during policy management */
static pd_do_t gl_src_pdo[NO_OF_TYPEC_PORTS];

/* Flag to control internal power policy */
static bool gl_cur_ctrl_enabled = true;
   
/* Flag to indicate if I2C is active */
static bool gl_sync_intf_active = false;

/* Local events */
static uint32_t gl_ccg_sync_evts = 0;

/* Alt Mode states */
static alt_mode_evt_t  gl_am_cmd_info;
static uint8_t         gl_trigger_mode_reentry[NO_OF_TYPEC_PORTS] = { 0
#if CCG_PD_DUALPORT_ENABLE    
    , 
    0 
#endif /* CCG_PD_DUALPORT_ENABLE */    
};
    
static void ccg_sync_dpm_cbk(uint8_t port, resp_status_t status, const pd_packet_t *rec_pkt);
static void sync_ilim_setting(uint8_t port);

#if CCG_SYNC_ALGORITHM == V1
static void set_port_cur_lim(uint8_t port, uint16_t current);
#endif /* CCG_SYNC_ALGORITHM == V1 */

#if CCG_UCSI_ENABLE_V2
/* External UCSI functions */
extern void ucsi_handle_ppm_reset(ucsi_ctrl_details_t* ctrl, ucsi_evt_t evt);
extern void ucsi_handle_set_ntfn_en(ucsi_ctrl_details_t* ctrl, ucsi_evt_t evt);
extern void ucsi_handle_set_pwr_level(ucsi_ctrl_details_t* ctrl, ucsi_evt_t evt);
#endif /* CCG_UCSI_ENABLE_V2 */

/* Signal an event within */
void ccg_sync_set_evt(ccg_sync_evt_t evt)
{
    uint8_t intr = CyEnterCriticalSection();
    gl_ccg_sync_evts |= (1 << evt);
    gl_sleep_allowed = false;
    CyExitCriticalSection(intr);
}

/* Clear an event within */
void ccg_sync_clr_evt(ccg_sync_evt_t evt)
{
    uint8_t intr = CyEnterCriticalSection();
    gl_ccg_sync_evts &= ~(1 << evt);
    CyExitCriticalSection(intr);
}    

void ccg_sync_intf_init()
{
    /* Do nothing */
}

/* This sets the preferred controller which has priority in allocating 3A to its ports.
 * If two devices are connected at the same time (or within tCCDebounce) on two different CCG5s,
 * the part with preferred_controller=true will allocate 3A to its port */
void attr_always_inline ccg_sync_set_preferred(uint8_t preferred) 
{
    gl_preferred_controller = preferred;
}

/* Set ILIM pin state */
static void sync_ilim_setting(uint8_t port)
{
    uint8_t is_supplying_3A = gl_src_pdo[port].fixed_src.max_current == I_3A;
    if(port)
        SRC_3A_LIM_P2_Write(is_supplying_3A);
    else
        SRC_3A_LIM_P1_Write(is_supplying_3A);
}

/* Source PDOs sent. Change ILIM pin state */
static void ccg_sync_dpm_cbk(uint8_t port, resp_status_t status, const pd_packet_t *rec_pkt)
{
    (void) rec_pkt;
    
    /* If the Source Caps were sent successfully, change ILIM
     * Ideally, this should be done after contract */
    if(status >= CMD_SENT)
        sync_ilim_setting(port);
}

static void ccg_sync_tmr_cbk(uint8_t port, timer_id_t id)
{
    switch(id)
    {
        case CCG_SYNC_DPM_RETRY_TIMER:
			/* Retry source pdo update */
            ccg_sync_set_evt(CCG_SYNC_EVT_UPDT_SRC_CAP_P0 + port);
            break;
            
        default:
            break;
    }
}

/* Set the I2C slave address of local CCG5 and other/remote CCG5 */
void attr_always_inline ccg_sync_set_slave_addr(uint8_t local_addr, uint8_t remote_addr)
{
    gl_local_addr = local_addr;
    gl_remote_addr = remote_addr;
}

#if CCG_SYNC_ALGORITHM == V2

/* We need to initialize local port current state to 0mA 
 * for the first PDO update to take effect */
uint16_t gl_local_port_current[NO_OF_TYPEC_PORTS] = { 0
#if CCG_PD_DUALPORT_ENABLE
            , 
            0 
#endif /* CCG_PD_DUALPORT_ENABLE */
        },
         gl_remote_port_current[NO_OF_TYPEC_PORTS] = { I_900mA
#if CCG_PD_DUALPORT_ENABLE            
            , 
            I_900mA
#endif /* CCG_PD_DUALPORT_ENABLE */    
        };
        
static uint8_t ccg_sync_set_port_current(uint8_t port, uint16_t current)
{
    if(gl_local_port_current[port] == current)
        return false;
    
    /* Set to 5V/3A by default */
    gl_src_pdo[port].val = 0x3E01912C;
    
    /* Change Source PDO current */
    gl_local_port_current[port] = current;
    gl_src_pdo[port].fixed_src.max_current = current;
    ccg_sync_set_evt(port + CCG_SYNC_EVT_CHNG_PORT_CUR_P0);
#if CCG_UCSI_ENABLE_V2    
    ucsi_notify(port, UCSI_NTFN_CAP_CHANGED);
#endif /* CCG_UCSI_ENABLE_V2 */    
    return true;
}

static void ccg_sync_execute_power_policies()
{
    uint8_t port;
    uint8_t ports_updated = false;
    uint8_t remote_port_high_pwr = (gl_remote_port_current[0] == I_3A)
                                    || (gl_remote_port_current[1] == I_3A);
    
    for(port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
        const dpm_status_t* stat = dpm_get_info(port);
        
        if(stat->attach && (stat->cur_port_role == PRT_ROLE_SOURCE) /* Attached as SRC */
            && ((gl_local_port_current[port] == I_3A)               /* AND Port is already supplying 3A */
                || ((gl_local_port_current[1 - port] != I_3A)       /*     OR | Other Port is not supplying 3A */
                    && (remote_port_high_pwr == false)              /*        | AND No remote port is supplying 3A */
                    && (gl_preferred_controller || !gl_remote_port_conn))))/* | AND | We're preferred controller */
        {                                                                  /*       | OR (non-preferred) Remote port is not about to attach */
            ports_updated |= ccg_sync_set_port_current(port, I_3A);
        }
        else
        {
            ports_updated |= ccg_sync_set_port_current(port, I_900mA);
        }
        
        /* If we're disconnecting, quickly update ILIM pin. Don't wait for events in the main loop */
        if(stat->attach == false)
        {
            sync_ilim_setting(port);
            ports_updated = true; /* Force update on disconnect */
        }
    }
    
    if (ports_updated)
        ccg_sync_set_evt(CCG_SYNC_EVT_UPDATE_REMOTE_PORT);
}
#endif /* CCG_SYNC_ALGORITHM == V2 */

void ccg_sync_task()
{
    uint32_t slaveStatus = CCGComm_I2CSlaveClearWriteStatus(),
             masterStatus = CCGComm_I2CMasterClearStatus();
    uint8_t port;
    ccg_sync_evt_t evt;
    
    if(slaveStatus & CCGComm_I2C_SSTAT_WR_CMPLT)
    {
        ccg_sync_set_evt(CCG_SYNC_EVT_PKT_RCVD);
    }
    else if(slaveStatus & (CCGComm_I2C_SSTAT_RD_BUSY | CCGComm_I2C_SSTAT_WR_BUSY))
    {
        /* Sleep is not allowed if slave transfer is in progress */
        gl_sleep_allowed = false;
    }
    
    if(masterStatus & CCGComm_I2C_MSTAT_ERR_XFER)
    {
		/* If I2C write failed, retry */
        ccg_sync_set_evt(CCG_SYNC_EVT_SEND_PKT);
    }
    else if(masterStatus & CCGComm_I2C_MSTAT_WR_CMPLT)
    {
        /* If write is done, go to sleep */
        gl_sleep_allowed = true;
    }

    evt = event_group_get_event(&gl_ccg_sync_evts, true);
        
    switch(evt)
    {
        case CCG_SYNC_EVT_VSYS_ADDED:
            /* VSYS is available. Start the I2C block */
            if(!gl_sync_intf_active)
            {
                CCGComm_I2CSlaveInitWriteBuf((uint8_t *)&gl_remote_dev_pkt, sizeof(ccg_sync_pkt_t));
                CCGComm_I2CSlaveInitReadBuf((uint8_t *)&gl_local_dev_pkt, sizeof(ccg_sync_pkt_t));
                CCGComm_Start();
                CCGComm_I2CSlaveSetAddress(gl_local_addr);
                CCGComm_I2CSlaveSetAddressMask(0xFE);
            }
            gl_sync_intf_active = true;
            gl_sleep_allowed = true;
            break;
        
        case CCG_SYNC_EVT_VSYS_REMOVED:
            if(gl_sync_intf_active)
                CCGComm_Stop();
            gl_ccg_sync_evts = 0; /* Unsafe clear of all events */
            gl_sync_intf_active = false;
            gl_sleep_allowed = true;
            break;
        
        case CCG_SYNC_EVT_PKT_RCVD:
            switch(gl_remote_dev_pkt.cmd)
            {
#if CCG_UCSI_ENABLE
                case UCSI_RESET_PPM:
                    /* PPM_RESET received on the other CCG. Reset our internal state too */
    #if CCG_UCSI_ENABLE_V2
                    ucsi_handle_ppm_reset(NULL, 0);
                    ucsi_regs.cci.indicators = 0; /* Don't raise a notification */
#endif /* CCG_UCSI_ENABLE_V2 */                    
                    ccg_sync_clr_evt(CCG_SYNC_EVT_SEND_PKT);
                    gl_sleep_allowed = true;
                    break;
                    
                case UCSI_SET_NTFN_ENABLE:
#if CCG_UCSI_ENABLE_V2                    
                    /* Other CCG's notifications were changed */
                    ucsi_regs.control.details.set_notification_enable.notifications = gl_remote_dev_pkt.data.val;
                    ucsi_handle_set_ntfn_en(&ucsi_regs.control.details, 0);
                    ucsi_regs.cci.indicators = 0; /* Don't raise a notification */
#endif /* CCG_UCSI_ENABLE_V2 */                    
                    ccg_sync_clr_evt(CCG_SYNC_EVT_SEND_PKT);
                    gl_sleep_allowed = true;
                    break;
                    
            #if CCG_UCSI_VERSION == UCSI_VERSION_1_1
                case UCSI_SET_PWR_LEVEL:
#if CCG_UCSI_ENABLE_V2                
                    /* Extract data from the packet and fill to the UCSI CONTROL register */
                    ucsi_regs.control.details.set_pwr_level.connector_number = 0;
                    ucsi_regs.control.details.set_pwr_level.set_src_level = gl_remote_dev_pkt.data.val & 0x01;
                    ucsi_regs.control.details.set_pwr_level.type_c_current = (gl_remote_dev_pkt.data.val >> 1) & 0x03;
                    ucsi_regs.control.details.set_pwr_level.max_power = gl_remote_dev_pkt.data.val >> 8;
                    /* Handle the command */
                    ucsi_handle_set_pwr_level(&ucsi_regs.control.details, 0);
                    ucsi_regs.cci.indicators = 0; /* Don't raise a notification */
                    ccg_sync_clr_evt(CCG_SYNC_EVT_SEND_PKT);
                    gl_sleep_allowed = true;
#endif /* CCG_UCSI_ENABLE_V2 */                    
                    break;
            #endif /* UCSI 1.1 */
#endif /* CCG_UCSI_ENABLE */

                case UPDATE_STATUS:
                    gl_remote_port_en = gl_remote_dev_pkt.data.val & 0x03;
                    gl_remote_port_conn = (gl_remote_dev_pkt.data.val >> 2) & 0x03;
#if CCG_SYNC_ALGORITHM == V2
                    gl_remote_port_current[0] = gl_remote_dev_pkt.data.port_sts.port0_is_high_pwr ? I_3A : I_900mA;
                    gl_remote_port_current[1] = gl_remote_dev_pkt.data.port_sts.port1_is_high_pwr ? I_3A : I_900mA;
                    if(gl_cur_ctrl_enabled)
                        ccg_sync_execute_power_policies();
#else
                    gl_remote_high_pwr_port = (gl_remote_dev_pkt.data.val >> 8) & 0x03;
                    gl_sleep_allowed = true;
                    /* If no ports on other CCG5 is attached, check if we can advertise 3A on 
					 * any of our local ports */
                    if((gl_remote_port_conn == 0) && (gl_local_high_pwr_port == -1))
                    {
                        for(port = 0; port < NO_OF_TYPEC_PORTS; port++)
                            if(dpm_get_info(port)->contract_exist)
                            {
                                gl_local_high_pwr_port = port;
                                set_port_cur_lim(port, I_3A);
                                break;
                            }
                    }
#endif /* CCG_SYNC_ALGORITHM */
#if CCG_UCSI_ENABLE_V2
                    /* Refresh the number of connectors on the system */
                    ucsi_refresh_conn_cnt();
#endif /* CCG_UCSI_ENABLE_V2 */                    
                    break;
                    
                default:
                    gl_sleep_allowed = true;
                    break;
            }
            CCGComm_I2CSlaveClearWriteBuf();
            break;
            
        case CCG_SYNC_EVT_CHNG_PORT_CUR_P0:
        case CCG_SYNC_EVT_CHNG_PORT_CUR_P1:
            /* Port's current advertisement changed. Sync state */
            if(!gl_cur_ctrl_enabled)
                break;
            
            port = evt - CCG_SYNC_EVT_CHNG_PORT_CUR_P0;
             
            /* Update local PDO list */
            dpm_update_src_cap(port, 1, (pd_do_t*)&gl_src_pdo[port]);
            dpm_update_src_cap_mask(port, 0x01);
                
            if(dpm_get_info(port)->attach && (dpm_get_info(port)->contract_exist == false))
            {                
                /* If the current requested is 3A, then change Rp */
                if(gl_src_pdo[port].fixed_src.max_current == I_3A)
                    dpm_typec_command(port, DPM_CMD_SET_RP_3A, NULL);
                else
                    dpm_typec_command(port, DPM_CMD_SET_RP_DFLT, NULL);
            }
            
            /* If there's a PD contract already, try re-negotiating it */
            if(dpm_get_info(port)->contract_exist && (dpm_get_info(port)->cur_port_role == PRT_ROLE_SOURCE))
            {
                gl_dpm_retry_count[port] = CCG_SYNC_DPM_RETRY_CNT;
                if(dpm_pd_command(port, DPM_CMD_SRC_CAP_CHNG, NULL, ccg_sync_dpm_cbk) != CCG_STAT_SUCCESS)
                {
                    timer_start(port, CCG_SYNC_DPM_RETRY_TIMER, CCG_SYNC_DPM_RETRY_WAIT_PERIOD, ccg_sync_tmr_cbk);
                    gl_dpm_retry_count[port]--;
                }
            }
            else /* If not, then just change the ILIM pins */
                sync_ilim_setting(port);
            gl_sleep_allowed = true;
            break;
            
        case CCG_SYNC_EVT_UPDT_SRC_CAP_P0:
        case CCG_SYNC_EVT_UPDT_SRC_CAP_P1:
            /* We need to update source caps advertised on a port */
            if(!gl_cur_ctrl_enabled)
                break;
            
            port = evt - CCG_SYNC_EVT_UPDT_SRC_CAP_P0;
            if(dpm_pd_command(port, DPM_CMD_SRC_CAP_CHNG, NULL, ccg_sync_dpm_cbk) != CCG_STAT_SUCCESS)
            {
                if(gl_dpm_retry_count[port] != 0)
                    timer_start(port, CCG_SYNC_DPM_RETRY_TIMER, CCG_SYNC_DPM_RETRY_WAIT_PERIOD, ccg_sync_tmr_cbk);
                gl_dpm_retry_count[port]--;
            }
            gl_sleep_allowed = true;
            break;
                      
            
        case CCG_SYNC_EVT_UPDATE_REMOTE_PORT:
            if(!gl_sync_intf_active)
                break;
            
            /* Fill the update packet */
            gl_local_dev_pkt.cmd = UPDATE_STATUS;
            /* Init */
            gl_local_dev_pkt.data.val = hpi_get_port_enable();
            
			/* We use 'connect' vs 'attached' because connect gives pre-debounce status.
		     * The first physically connected device gets 3A */
            if (dpm_get_info(0)->connect
                && !(dpm_get_info(0)->attach)
                && (dpm_get_info(0)->cur_port_role == PRT_ROLE_SOURCE))
            {
                gl_local_dev_pkt.data.port_sts.port0_connected = 1;
            }
            
            if (dpm_get_info(1)->connect 
                && !(dpm_get_info(1)->attach)
                && (dpm_get_info(1)->cur_port_role == PRT_ROLE_SOURCE))
            {
                gl_local_dev_pkt.data.port_sts.port1_connected = 1;
            }
            
#if CCG_SYNC_ALGORITHM == V1
            gl_local_dev_pkt.data.port_sts.port0_is_high_pwr = gl_local_high_pwr_port == 0;
        #if CCG_PD_DUALPORT_ENABLE
            gl_local_dev_pkt.data.port_sts.port1_enabled = (hpi_get_port_enable() >> 1) & 0x01;
            gl_local_dev_pkt.data.port_sts.port1_connected = dpm_get_info(1)->connect ? 1 : 0;
            gl_local_dev_pkt.data.port_sts.port1_is_high_pwr = gl_local_high_pwr_port == 1;
        #endif /* CCG_PD_DUALPORT_ENABLE */
#else
            if(gl_local_port_current[0] == I_3A)
                gl_local_dev_pkt.data.port_sts.port0_is_high_pwr = 1;
            if(gl_local_port_current[1] == I_3A)
                gl_local_dev_pkt.data.port_sts.port1_is_high_pwr = 1;
#endif /* CCG_SYNC_ALGORITHM */
            /* Intentional fall-through to send the status update packet */
            
        case CCG_SYNC_EVT_SEND_PKT:
            if(!gl_sync_intf_active)
                break;
            
            timer_stop(0, CCG_SYNC_BUSY_TIMER);
            /* Send the packet over */
            if(CCGComm_I2CMasterWriteBuf(gl_remote_addr,
                    (uint8_t*)&gl_local_dev_pkt, sizeof(ccg_sync_pkt_t), CCGComm_I2C_MODE_COMPLETE_XFER) != CCGComm_I2C_MSTR_NO_ERROR)
            {
                ccg_sync_set_evt(CCG_SYNC_EVT_SEND_PKT);
            }
            
            timer_start(0, CCG_SYNC_BUSY_TIMER, CCG_SYNC_BUSY_TIMER_PERIOD, NULL);
            break;  
            
        default:
            break;
    }
}

#if CCG_SYNC_ALGORITHM == V1
static void set_port_cur_lim(uint8_t port, uint16_t current)
{
    /* Set to 5V/3A by default */
    gl_src_pdo[port].val = 0x3E01912C;
    
    /* Change Source PDO current */
    gl_src_pdo[port].fixed_src.max_current = current;
    ccg_sync_set_evt(port + CCG_SYNC_EVT_CHNG_PORT_CUR_P0);
}
#endif /* CCG_SYNC_ALGORITHM == V1 */

/* Enable or disable local power policy management. We do this if EC changes PDOs via HPI */
void attr_always_inline ccg_sync_en_pdo_ctrl(uint8_t enable)
{
    gl_cur_ctrl_enabled = enable;
}

void ccg_sync_pd_event_handler(uint8_t port, app_evt_t evt, const void *data)
{
    (void)data;
    
#if CCG_SYNC_ALGORITHM == V1
    uint8_t other_port = 1 - port;
#endif /* CCG_SYNC_ALGORITHM == V1 */
    switch(evt)
    {
        case APP_EVT_TYPEC_ATTACH_WAIT:
            /* On first attach, fall back to default current */
            ccg_sync_set_port_current(port, I_900mA);
            
            /* We notify the other controller of an intent to supply 3A */
            ccg_sync_set_evt(CCG_SYNC_EVT_UPDATE_REMOTE_PORT);
            break;

#if CCG_SYNC_ALGORITHM == V2
        case APP_EVT_DISCONNECT:
            /* On disconnect, fall back to default current */
            ccg_sync_set_port_current(port, I_900mA);
            /* Fall through to handle power policy update */
            
        case APP_EVT_CONNECT:
        case APP_EVT_PR_SWAP_COMPLETE:
            if(gl_cur_ctrl_enabled)
                ccg_sync_execute_power_policies();
            break;
#else
        case APP_EVT_TYPEC_ATTACH:
            /* Run this logic only if we're a source */
            if(dpm_get_info(port)->cur_port_role == PRT_ROLE_SINK)
                break;
            
            /* Commit the intention (to supply 3A) here. The logic is derived via negativa
                LOGIC: 900mA is supplied when
                1. other local port is supplying 3A (or)
                2. when no local port is supplying 3A,
                    (a) [ remote port is supplying 3A (or)
                        (b) [ when no remote port is supplying 3A,
                                (i) some remote port is 'connected'  (and)
                                (ii) we're not the preferred controller ]]
             */
            if( (gl_local_high_pwr_port != -1) || /* Condition (1) */
                    (gl_remote_high_pwr_port || /* Condition (2)(a) */
                        (gl_remote_port_conn && !gl_preferred_controller)) /* Condition (2)(a)(i) and (2)(a)(ii) */
            )
                set_port_cur_lim(port, I_900mA);
            else
            {
                gl_local_high_pwr_port = port;
                set_port_cur_lim(port, I_3A);
            }
            ccg_sync_set_evt(CCG_SYNC_EVT_UPDATE_REMOTE_PORT);
            break;
            
        case APP_EVT_DISCONNECT: 
            gl_trigger_mode_reentry[port] = false;

            if (gl_src_pdo[port].fixed_src.max_current == I_3A)
            {
                /* Relinquish high power port status */
                gl_local_high_pwr_port = -1;
                set_port_cur_lim(port, I_900mA);
                /* If other local port is attached as source, re-assign high power */
                if(dpm_get_info(other_port)->contract_exist
                    && (dpm_get_info(other_port)->cur_port_role == PRT_ROLE_SOURCE))
                {
                    gl_local_high_pwr_port = other_port;
                    set_port_cur_lim(other_port, I_3A);
                }
            }
            
            sync_ilim_setting(port);
            
            timer_stop(port, CCG_SYNC_DPM_RETRY_TIMER);
            ccg_sync_set_evt(CCG_SYNC_EVT_UPDATE_REMOTE_PORT);
            ccg_sync_clr_evt(CCG_SYNC_EVT_CHNG_PORT_CUR_P0 + port);
            ccg_sync_clr_evt(CCG_SYNC_EVT_UPDT_SRC_CAP_P0 + port);
            break;
#endif /* CCG_SYNC_ALGORITHM */
        case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE:
            /* If we re-negotiated a PD contract with 3A, re-enter any scheduled alt modes */
            if ((((pd_contract_info_t*)data)->status == PD_CONTRACT_NEGOTIATION_SUCCESSFUL)
                && gl_trigger_mode_reentry[port]
                && (gl_src_pdo[port].fixed_src.max_current == I_3A))
            {
                eval_app_alt_mode_cmd(port, (uint8_t*)&gl_am_cmd_info, (uint8_t*)&gl_am_cmd_info);
                gl_trigger_mode_reentry[port] = false;
            }
            break;
            
        default:
            break;
    }
}

/* Check if CCG can go to sleep */
bool attr_always_inline ccg_sync_sleep_allowed()
{
    return !gl_ccg_sync_evts && gl_sleep_allowed 
                && !timer_is_running(0, CCG_SYNC_BUSY_TIMER);
}

/* Retrieve remote CCG status */
uint8_t attr_always_inline ccg_sync_get_remote_port_en(void)
{
    return gl_remote_port_en;
}

/* Trigger an event to update the other CCG5 */
void attr_always_inline ccg_sync_update_remote_port()
{
    ccg_sync_set_evt(CCG_SYNC_EVT_UPDATE_REMOTE_PORT);
}

/* Send a packet to the other CCG5 */
void ccg_sync_send_remote_pkt(ccg_sync_cmd_t cmd, uint16_t data)
{
    gl_local_dev_pkt.cmd = cmd;
    gl_local_dev_pkt.data.val = data;
    ccg_sync_set_evt(CCG_SYNC_EVT_SEND_PKT);
}

/* Schedule a re-entry of alt mode when the port advertises 3A */
void ccg_sync_schedule_mode_reentry(uint8_t port, uint16_t svid, uint8_t alt_mode_id)
{
    gl_trigger_mode_reentry[port] = true;
    gl_am_cmd_info.alt_mode_event.svid = svid;
    gl_am_cmd_info.alt_mode_event.alt_mode = alt_mode_id;
    gl_am_cmd_info.alt_mode_event.alt_mode_evt = AM_CMD_ENTER; 
    gl_am_cmd_info.alt_mode_event.data_role = PRT_TYPE_DFP;
}

#endif /* CCG_SYNC_ENABLE */

/* End of file */
