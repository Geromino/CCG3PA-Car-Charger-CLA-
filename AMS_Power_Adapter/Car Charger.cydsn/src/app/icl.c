/**
 * @file icl.c
 *
 * @brief @{Ice Lake specific logic@}
 *
 *******************************************************************************
 *
 * Copyright (2014-2018), Cypress Semiconductor Corporation or a subsidiary of
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

#include <config.h>
#include <pd.h>
#include <dpm.h>
#include <dpm_intern.h>
#include <timer.h>
#include <pdss_hal.h>
#if BB_RETIMER_ENABLE
#include <bb_retimer.h>
#endif /* BB_RETIMER_ENABLE */
#if CCG_SYNC_ENABLE
#include <ccg_sync.h>
#endif /* CCG_SYNC_ENABLE */
#include <hal_ccgx.h>
#include <hpi_internal.h>
#include <hpi.h>
#include <icl.h>
#include <intel_ridge_internal.h>
#include <intel_ridge.h>
#include <ridge_slave.h>
#include <alt_modes_mngr.h>
#include <app.h>

#if BB_RETIMER_ENABLE || CCG_SYNC_ENABLE
static void icl_supply_change_cb(uint8_t port, ccg_supply_t supply_id, bool present);
#endif /* BB_RETIMER_ENABLE || CCG_SYNC_ENABLE */

#if ICL_FORCE_TBT_MODE_SUPP
/* A special mux configuration to indicate invalid status */
#define MUX_CFG_INVALID_IDX             (-1)    
    
/* Custom HPI register to handle force tbt mode */
static uint8_t icl_hpi_reg_wr_handler(uint16_t reg_addr, uint8_t wr_size, uint8_t *wr_data);

/* State of the MUX on each port */
static int8_t gl_mux_seq_idx[NO_OF_TYPEC_PORTS] = { MUX_CFG_INVALID_IDX
#if CCG_PD_DUALPORT_ENABLE    
    ,  
    MUX_CFG_INVALID_IDX
#endif /* CCG_PD_DUALPORT_ENABLE */    
};

#if ICL_FORCE_TBT_MODE_SUPP & ICL_MAIN_FW
    
/* Sequence of states that the MUX and retimer goes through when "force tbt mode" is set */
/* Note: RIDGE_DEBUG_MODE_MASK is used as Safe State mask i.e Connected + USB 2.0 */
static const uint32_t gl_mux_state_seq[] = { RIDGE_DISCON_STATE_MASK, 
#if TBT_DFP_SUPP || TBT_UFP_SUPP    
    RIDGE_USB_STATE_MASK, RIDGE_DEBUG_MODE_MASK, RIDGE_TBT_MODE_MASK
#endif
};

/* Store number of states in the sequence list */
#define NUM_STATES_IN_SEQ       (sizeof(gl_mux_state_seq) / sizeof(uint32_t))

#endif /* ICL_FORCE_TBT_MODE_SUPP & ICL_MAIN_FW*/    

/* Custom HPI register data */
static uint8_t gl_icl_sts = 0;
#endif /* ICL_FORCE_TBT_MODE_SUPP */

#if ICL_MAIN_FW
static volatile uint32_t gl_icl_evts[NO_OF_TYPEC_PORTS] = {0
#if CCG_PD_DUALPORT_ENABLE
    , 
    0
#endif /* CCG_PD_DUALPORT_ENABLE */    
};
#endif /* ICL_MAIN_FW */

#if ICL_OCP_ENABLE
static uint8_t gl_ocp_enabled[NO_OF_TYPEC_PORTS] = {0
#if CCG_PD_DUALPORT_ENABLE    
    , 
    0
#endif /* CCG_PD_DUALPORT_ENABLE */    
};
static vbus_ocp_cbk_t gl_ocp_cbk = NULL;
static void icl_ocp_handler(void);
#endif /* ICL_OCP_ENABLE */

void barrel_state_change_handler(void);
extern void psnk_enable (uint8_t port);
extern void psnk_disable (uint8_t port, sink_discharge_off_cbk_t snk_discharge_off_handler);

static bool gl_vsys_ready = false;

#if ICL_MAIN_FW
void dp_dfp_dequeue_hpd(uint8_t port);
#endif /* ICL_MAIN_FW */

static uint8_t gl_adp_state = ADP_STATE_INIT;
static int8_t  gl_adp_edge  = ADP_EDGE_NONE;

/* Initialize ICL RVP platform-specific logic */
void icl_init()
{
#if BB_RETIMER_ENABLE || CCG_SYNC_ENABLE
    pd_hal_set_supply_change_evt_cb(icl_supply_change_cb);
#endif /* BB_RETIMER_ENABLE || CCG_SYNC_ENABLE */
#if ICL_FORCE_TBT_MODE_SUPP
    hpi_set_userdef_write_handler(icl_hpi_reg_wr_handler);
#endif /* ICL_FORCE_TBT_MODE_SUPP */
#if ICL_OCP_ENABLE    
    Port_OCP_ISR_StartEx(icl_ocp_handler);
#endif /* ICL_OCP_ENABLE */
    
    Barrel_Detect_ISR_StartEx(barrel_state_change_handler);
    barrel_state_change_handler();
}

#if ICL_OCP_ENABLE
/* Lightweight OCP config function */
void icl_ocp_cfg(uint8_t port, bool enable, vbus_ocp_cbk_t cbk)
{
    uint8_t intr_state = CyEnterCriticalSection();
    gl_ocp_enabled[port] = enable;
    gl_ocp_cbk = cbk;
    CyExitCriticalSection(intr_state);
}

/* Pin interrupt handler */
void icl_ocp_handler()
{
    uint8_t pinIntrSrc = SW_OC_FAULT_ClearInterrupt();
    
#if ICL_FORCE_TBT_MODE_SUPP
    /* If TBT mode was forced on the port, restart CC scans.
     * OCP is more urgent */
    if(gl_icl_sts & ICL_STS_REG_FORCE_TBT_MODE)
    {
        dpm_get_status(0)->skip_scan = false;
        dpm_get_status(1)->skip_scan = false;
    }
#endif /* ICL_FORCE_TBT_MODE_SUPP */
    
    if(gl_ocp_enabled[0] && (pinIntrSrc & (1 << SW_OC_FAULT_0_SHIFT)))
    {
        gl_ocp_cbk(0);
    }
    
    if(gl_ocp_enabled[1] && (pinIntrSrc & (1 << SW_OC_FAULT_1_SHIFT)))
    {
        gl_ocp_cbk(1);
    }
    
    Port_OCP_ISR_ClearPending();
}
#endif /* ICL_OCP_ENABLE */

void barrel_state_change_deb_cbk(uint8_t port, timer_id_t id);
static uint16_t gl_adp_detect_debounce[] = { 
    ICL_ADP_DETACH_DEBOUNCE_TIME, ICL_ADP_INIT_DEBOUNCE_TIME, ICL_ADP_ATTACH_DEBOUNCE_TIME 
};

void barrel_state_change_handler()
{
    gl_adp_edge = (ADP_DETECT_Read() && !gl_adp_state) ? ADP_POSEDGE :
                    (!ADP_DETECT_Read() && (gl_adp_state == 1)) ? ADP_NEGEDGE : ADP_EDGE_NONE;
                    
    if (ADP_DETECT_Read() != gl_adp_state)
    {
        gl_adp_state = ADP_DETECT_Read();
        timer_stop(0, ICL_ADP_DETECT_DEB_TIMER);
        timer_start(0, ICL_ADP_DETECT_DEB_TIMER, gl_adp_detect_debounce[gl_adp_edge + 1], barrel_state_change_deb_cbk);
          
        for(uint8_t port = 0; port < NO_OF_TYPEC_PORTS; port++)
        {
            if ((gl_adp_edge == ADP_NEGEDGE) /* If we lost DC barrel */
                && (dpm_get_info(port)->vconn_logical == true) /* If we're supplying VCONN */
                && (dpm_get_info(port)->cbl_vdo.std_cbl_vdo.cbl_term >= CBL_TERM_ONE_ACT_ONE_PAS_VCONN_REQ)) /* Active cable */
            {
                /* Send hard reset so that other device will become DFP/Source and reconfigures us */
                dpm_pd_command(port, DPM_CMD_SEND_HARD_RESET, NULL, NULL);
            }
        }
    }
    
    ADP_DETECT_ClearInterrupt();
    Barrel_Detect_ISR_ClearPending();
}

void barrel_state_change_deb_cbk(uint8_t port, timer_id_t id)
{    
    for(port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
        const dpm_status_t* dpm_stat = dpm_get_info(port);
        
        /* If we can sink current and DC barrel is removed, do so */
        if ((dpm_stat->dpm_enabled)
            && (dpm_stat->fault_active == false)
            && (dpm_stat->cur_port_role == PRT_ROLE_SINK)
            && (app_get_status(port)->psnk_cur >= I_1P5A)
            && (ADP_DETECT_Read() == 0))
        {
            psnk_enable(port);
        }
        else /* DC barrel attached. Stop sinking current */
            psnk_disable(port, NULL);
    }
}

#if ICL_FORCE_TBT_MODE_SUPP
/* Timer callback to trigger mux state change */
static void icl_mux_state_chng_cbk(uint8_t port, timer_id_t id)
{
#if BB_RETIMER_ENABLE  
    /* Wait for all retimer writes to finish before moving forward */
    if (retimer_update_is_pending(0) || retimer_update_is_pending(1))
        timer_start(port, ICL_MUX_STATE_CHANGE_TIMER, ICL_MUX_STATE_CHANGE_DURATION, icl_mux_state_chng_cbk);
    else
#endif /* BB_RETIMER_ENABLE */
#if ICL_MAIN_FW
        icl_set_evt(port, ICL_EVT_CHANGE_MUX_STATE);
#endif /* ICL_MAIN_FW */        
}
    
/* HPI Write Handler */
static uint8_t icl_hpi_reg_wr_handler(uint16_t reg_addr, uint8_t wr_size, uint8_t *wr_data)
{
    uint8_t port,
            force_tbt_mode = wr_data[0] == ICL_CTRL_CMD_FORCE_TBT_MODE;
            
    hpi_response_t return_status = HPI_RESPONSE_INVALID_COMMAND;
            
    switch(reg_addr)
    {
        case ICL_CTRL_REG:
        
            /* Expect 1 byte:
                Byte[0] -> Command: 0x01 -> Force TBT Mode
                                    0x00 -> Normal operation */
            if((wr_size != 1)
                || (force_tbt_mode && 
                        ((gl_mux_seq_idx[0] != MUX_CFG_INVALID_IDX)
                         || (gl_mux_seq_idx[1] != MUX_CFG_INVALID_IDX))))
            {
                return HPI_RESPONSE_INVALID_ARGUMENT;
            }
            
            for(port = 0; port < NO_OF_TYPEC_PORTS; port++) 
            {
                if(force_tbt_mode)
                {
                    /* Stop scanning for CC line changes. Reject all swaps. Exit all modes */
                    dpm_get_status(port)->skip_scan = true;
                    dpm_update_swap_response(port, APP_RESP_REJECT | (APP_RESP_REJECT << 2) | (APP_RESP_REJECT << 4));
                    alt_mode_mngr_exit_all (port);

                    /* Start the mux state change sequence sequence from the first state */
                    gl_mux_seq_idx[port] = 0;
                    
                    /* Wait a while to let the alt mode manager exit all modes */
                    timer_start(port, ICL_MUX_STATE_CHANGE_TIMER, ICL_MUX_STATE_CHANGE_DURATION, icl_mux_state_chng_cbk);
                    
                    return_status = HPI_RESPONSE_NO_RESPONSE;
                }
                else
                {
                    timer_stop(port, ICL_MUX_STATE_CHANGE_TIMER);
                    
                    /* Start scanning for CC line changes. Do Type-C error recovery to restart connection */
                    dpm_get_status(port)->skip_scan = false;
                    dpm_update_swap_response(port, get_pd_port_config(port)->swap_response);
                    dpm_typec_command(port, DPM_CMD_TYPEC_ERR_RECOVERY, NULL);
                    
                    /* Reset index and the status register */
                    gl_mux_seq_idx[port] = MUX_CFG_INVALID_IDX;
                    gl_icl_sts = 0;
                    hpi_init_userdef_regs(ICL_STS_REG, 1, &gl_icl_sts);
                    
                    /* Restart processing MUX changes on this port */
                    ignore_mux_changes(port, false);
                    
                    return_status = HPI_RESPONSE_SUCCESS;
                }
            }
            
            hpi_init_userdef_regs(ICL_CTRL_REG, wr_size, wr_data);
            break;
#if BB_RETIMER_ENABLE            
        case ICL_RT_PWR:
            /* Debug register to force power to retimer
             * Not used by Intel */
            if(wr_data[0])
            {
                retimer_enable(0);
                retimer_enable(1);
            }
            else
            {
                retimer_disable(0);
                retimer_disable(1);
            }
            return_status = HPI_RESPONSE_SUCCESS;
            break;
#endif /* BB_RETIMER_ENABLE */
    }
    
    return return_status;
}
#endif /* ICL_FORCE_TBT_MODE_SUPP */

#if ICL_MAIN_FW
void icl_set_evt(uint8_t port, icl_evt_t evt)
{
    uint8_t intr = CyEnterCriticalSection();
    gl_icl_evts[port] |= (1 << evt);
    CyExitCriticalSection(intr);
}
#endif /* ICL_MAIN_FW */

void icl_task(uint8_t port)
{
#if ICL_MAIN_FW    
    icl_evt_t evt;
    
    evt = event_group_get_event(&gl_icl_evts[port], true);
    
    switch(evt)
    {
        case ICL_EVT_DEQUEUE_HPD:
            /* We come here if the MUX was busy with a previous change.
             * Start popping the HPD queue */
            dp_dfp_dequeue_hpd(port);
            break;
            
        case ICL_EVT_CHANGE_MUX_STATE:
#if ICL_FORCE_TBT_MODE_SUPP            
            /* Run the retimer firmware update state machine */
            if(gl_mux_seq_idx[port] == MUX_CFG_INVALID_IDX)
                break;
            
            /* If we're done with the state sequence, then signal the EC */
            else if(gl_mux_seq_idx[port] == NUM_STATES_IN_SEQ)
            {
                gl_mux_seq_idx[port] = MUX_CFG_INVALID_IDX;
                /* Update HPI registers */
                gl_icl_sts |= ICL_STS_REG_FORCE_TBT_MODE;
                hpi_init_userdef_regs(ICL_STS_REG, 1, &gl_icl_sts);
                
                /* If other port is also done, then send SUCCESS */
                if(gl_mux_seq_idx[1 - port] == MUX_CFG_INVALID_IDX)
                    hpi_reg_enqueue_event(HPI_REG_SECTION_DEV, HPI_RESPONSE_SUCCESS, 0, 0);
                    
                /* Stop processing MUX changes on this port */
                ignore_mux_changes(port, true);
            }
            
            /* Run the MUX through a pre-determined sequence */
            else
            {   
                /* Force TBT status update so that we go through the exact sequence */
                ridge_force_status_update(port, true);
                /* Tell the PMC to change to the next state */
                set_mux(port, MUX_CONFIG_RIDGE_CUSTOM, gl_mux_state_seq[gl_mux_seq_idx[port]]);
                gl_mux_seq_idx[port] += 1; /* Move to the next index */
                timer_start(port, ICL_MUX_STATE_CHANGE_TIMER, ICL_MUX_STATE_CHANGE_DURATION, icl_mux_state_chng_cbk);

#if BB_RETIMER_ENABLE                
                /* Tell retimer to clear all bits in Debug_Mode register */
                retimer_set_compl_mode(false);
                retimer_set_evt(port, RT_EVT_FORCE_PWR_CHNG);
#endif /* BB_RETIMER_ENABLE */                
            }
#endif /* ICL_FORCE_TBT_MODE_SUPP */
            break;
            
        default:
            break;
    }
#endif /* ICL_MAIN_FW */
}

#if BB_RETIMER_ENABLE || CCG_SYNC_ENABLE
    
#if VESA_SCR_FIX
static alt_mode_evt_t gl_am_cmd_info;
static bool icl_try_enter_mode(uint8_t port, uint16_t svid, uint16_t alt_mode_id)
{
    gl_am_cmd_info.alt_mode_event.svid = svid;
    gl_am_cmd_info.alt_mode_event.alt_mode = alt_mode_id;
    gl_am_cmd_info.alt_mode_event.alt_mode_evt = AM_CMD_ENTER; 
    gl_am_cmd_info.alt_mode_event.data_role = PRT_TYPE_DFP;
    
    return eval_app_alt_mode_cmd(port, (uint8_t*)&gl_am_cmd_info, (uint8_t*)&gl_am_cmd_info);
}
#endif /*VESA_SCR_FIX*/

static void icl_vsys_stable_cbk(uint8_t port, timer_id_t id)
{
#if BB_RETIMER_ENABLE
    retimer_set_evt(port, RT_EVT_VSYS_ADDED);
#endif /* BB_RETIMER_ENABLE */
#if CCG_SYNC_ENABLE
    ccg_sync_set_evt(CCG_SYNC_EVT_VSYS_ADDED);
#endif /* CCG_SYNC_ENABLE */

#if VESA_SCR_FIX
    /* If vsys is back up and we were vconn source, re-enter alt mode */
    if (!gl_vsys_ready 
            && (dpm_get_info(port)->vconn_logical) 
            && (dpm_get_info(port)->cur_port_type == PRT_TYPE_DFP))
    {
        uint8_t idx;
        for(idx = 0; idx < 2; idx++)
        {
            if (icl_try_enter_mode (port, 
                    get_tbl_info(port, idx)->svid, get_tbl_info(port, idx)->alt_mode_id) == true)
            {
                break;
            }
        }
    }
#endif
    gl_vsys_ready = true;
}

static void icl_handle_vsys_change(uint8_t port, bool present)
{
    timer_stop(port, ICL_VSYS_STABLE_TIMER);
    
    if(present)
    {
        timer_start(port, ICL_VSYS_STABLE_TIMER, ICL_VSYS_STABLE_WAIT_TIME, icl_vsys_stable_cbk);
    }
    else
    {
#if BB_RETIMER_ENABLE
        retimer_set_evt(port, RT_EVT_VSYS_REMOVED);
#endif /* BB_RETIMER_ENABLE */
#if CCG_SYNC_ENABLE
        ccg_sync_set_evt(CCG_SYNC_EVT_VSYS_REMOVED);
#endif /* CCG_SYNC_ENABLE */

#if VESA_SCR_FIX
        /* Exit all modes when vsys is dropped and we're vconn source */
        if (gl_vsys_ready
            && (dpm_stat->vconn_logical) 
            && (dpm_stat->cur_port_type == PRT_TYPE_DFP))
        {
            alt_mode_mngr_exit_all (port);
        }
#endif       
        gl_vsys_ready = false;
    }
}

/* Power supply change handler. When VSYS/V5V is added or removed, we init or de-init any peripheral logic */
static void icl_supply_change_cb(uint8_t port, ccg_supply_t supply_id, bool present)
{
    switch(supply_id)
    {
        case CCG_SUPPLY_V5V:
        case CCG_SUPPLY_VSYS:
            icl_handle_vsys_change(0, present);
            icl_handle_vsys_change(1, present);
            break;
    }
}
#endif /* BB_RETIMER_ENABLE || CCG_SYNC_ENABLE */

bool icl_sleep_allowed()
{
#if ICL_FORCE_TBT_MODE_SUPP    
    #if CCG_PD_DUALPORT_ENABLE
        return (gl_mux_seq_idx[0] == MUX_CFG_INVALID_IDX) && (gl_mux_seq_idx[0] == MUX_CFG_INVALID_IDX);    
    #else
        return (gl_mux_seq_idx[0] == MUX_CFG_INVALID_IDX);
    #endif
#else
    return true;
#endif
}

/* Get VSYS state */
bool icl_vsys_is_present()
{
    return gl_vsys_ready;
}
/* End of File */
