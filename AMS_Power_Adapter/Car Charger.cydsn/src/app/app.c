/**
 * @file app.c
 *
 * @brief @{PD application handler source file.@}
 *
 *******************************************************************************
 *
 * Copyright (2014-2019), Cypress Semiconductor Corporation or a subsidiary of
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
#include <psource.h>
#include <psink.h>
#include <pdo.h>
#include <swap.h>
#include <vdm.h>
#include <app.h>
#include <vdm_task_mngr.h>
#include <timer.h>
#if CCG_HPI_ENABLE
#include <hpi.h>
#endif /* CCG_HPI_ENABLE */
#include <alt_mode_hw.h>
#include <alt_modes_mngr.h>
#include <hal_ccgx.h>
#include <gpio.h>
#include <system.h>
#if CCG_LOAD_SHARING_ENABLE
#include <loadsharing.h>
#endif /* CCG_LOAD_SHARING_ENABLE */
#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
#include <sensor_check.h>
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */
#if CCG_CABLE_COMP_ENABLE
#include <cable_comp.h>
#endif /* CCG_CABLE_COMP_ENABLE */

#if RIDGE_SLAVE_ENABLE
#include <ridge_slave.h>
#include <intel_ridge.h>
#endif /* RIDGE_SLAVE_ENABLE */
#if DP_UFP_SUPP
#include <dp_sid.h>
#endif /* DP_UFP_SUPP */
#if (CCG_BB_ENABLE != 0)
#include <billboard.h>
#endif /* (CCG_BB_ENABLE != 0) */

#if DP_UFP_SUPP
#include <hpd.h>
#endif /* DP_UFP_SUPP */

#if BATTERY_CHARGING_ENABLE
#include <battery_charging.h>
#endif /* BATTERY_CHARGING_ENABLE */

#if (CCG_TYPE_A_PORT_ENABLE == 1)
#include <type_a.h>
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */

#if CCG_UCSI_ENABLE
#include <ucsi.h>
#endif /* CCG_UCSI_ENABLE */

#if ICL_RVP_HW
#include <icl.h>
#endif /* ICL_RVP_HW */

#if BB_RETIMER_ENABLE
#include <bb_retimer.h>
#endif /* BB_RETIMER_ENABLE */

#if CCG_SYNC_ENABLE
#include <ccg_sync.h>
#endif

#if (CCG_LOAD_SHARING_ENABLE && CCG_REV3_HANDLE_BAD_SINK)
extern bool gl_stop_ls[NO_OF_TYPEC_PORTS];
extern bool gl_stop_power_throttle[NO_OF_TYPEC_PORTS];
bool gl_bad_device[NO_OF_TYPEC_PORTS] = {false};
#endif /* (CCG_LOAD_SHARING_ENABLE && CCG_REV3_HANDLE_BAD_SINK) */

#if ICL_RVP_HW
bool gl_block_swaps = false;
#endif

#if BB_RETIMER_ENABLE
uint8_t gl_system_state = SYSTEM_STATE_S0;
static void app_retimer_disable_cbk(uint8_t port, timer_id_t id)
{
    retimer_disable(port);
}
#endif /* BB_RETIMER_ENABLE */

#if APP_PPS_SINK_SUPPORT
uint8_t hpi_user_reg_handler(uint16_t addr, uint8_t size, uint8_t *data);
void app_pps_sink_disable(uint8_t port);
#endif /* APP_PPS_SINK_SUPPORT */

ovp_settings_t* pd_get_ptr_ovp_tbl(uint8_t port)
{
    /* Update the OVP settings from the configuration table. */
    return ((ovp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->ovp_tbl_offset));
}

ocp_settings_t* pd_get_ptr_ocp_tbl(uint8_t port)
{
    /* Update the VBus OCP settings from the configuration table */
    return ((ocp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->ocp_tbl_offset));
}

rcp_settings_t* pd_get_ptr_rcp_tbl(uint8_t port)
{
    /* Update the VBus RCP settings from the configuration table */
    return ((rcp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->rcp_tbl_offset));
}

uvp_settings_t* pd_get_ptr_uvp_tbl(uint8_t port)
{
    /* Update the VBus UVP settings from the configuration table */
    return ((uvp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->uvp_tbl_offset));
}

scp_settings_t* pd_get_ptr_scp_tbl(uint8_t port)
{
    /* Update the VBus SCP settings from the configuration table */
    return ((scp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->scp_tbl_offset));
}

vconn_ocp_settings_t* pd_get_ptr_vconn_ocp_tbl(uint8_t port)
{
    /* Update the Vcon OCP settings from the configuration table */
    return ((vconn_ocp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->vconn_ocp_tbl_offset));
}

otp_settings_t* pd_get_ptr_otp_tbl(uint8_t port)
{
    /* Update the OTP settings from the configuration table */
    return ((otp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->otp_tbl_offset));
}

pwr_params_t* pd_get_ptr_pwr_tbl(uint8_t port)
{
    /* Update the power parameters from the configuration table */
    return ((pwr_params_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->pwr_tbl_offset));
}

chg_cfg_params_t* pd_get_ptr_chg_cfg_tbl(uint8_t port)
{
#if CCG_TYPE_A_PORT_ENABLE
    if (port == TYPE_A_PORT_ID)
    {
        /* Return parameters for TYPE-A port. */
        return pd_get_ptr_type_a_chg_cfg_tbl (0);
    }
#endif /* CCG_TYPE_A_PORT_ENABLE */

    /* Update the legacy charging parameters from the configuration table */
    return ((chg_cfg_params_t *)((uint8_t *)(get_pd_config ()) +
        get_pd_port_config(port)->chg_cfg_tbl_offset));
}

bat_chg_params_t* pd_get_ptr_bat_chg_tbl(uint8_t port)
{
    /* Update the battery charging parameterss from the configuration table */
    return ((bat_chg_params_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->bat_chg_tbl_offset));
}

pwr_params_t* pd_get_ptr_type_a_pwr_tbl(uint8_t port)
{
    /* Update the power parameters of Type-A port from the configuration table */
    return ((pwr_params_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->type_a_pwr_tbl_offset));
}

typeA_chg_cfg_params_t* pd_get_ptr_type_a_chg_cfg_tbl(uint8_t port)
{
    /* Update the legacy charging parameters from the configuration table */
    return ((typeA_chg_cfg_params_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->type_a_chg_tbl_offset));
}

bb_settings_t* pd_get_ptr_bb_tbl(uint8_t port)
{
    /* Update the Billboard settings from the configuration table*/
    return ((bb_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->bb_tbl_offset));
}

auto_cfg_settings_t* pd_get_ptr_auto_cfg_tbl(uint8_t port)
{
    /* Update the Automotive charger settings from the configuration table*/
    return ((auto_cfg_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->auto_cfg_tbl_offset));
}

tbthost_cfg_settings_t* pd_get_ptr_tbthost_cfg_tbl(uint8_t port)
{
    /* Retrieve the thunderbolt host config parameters. */
    return ((tbthost_cfg_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->tbthost_cfg_tbl_offset));
}

custom_host_cfg_settings_t* pd_get_ptr_host_cfg_tbl(uint8_t port)
{
    /* Retrieve the custom host config parameters. */
    return ((custom_host_cfg_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->custom_host_tbl_offset));
}

#if (CCG_TYPE_A_PORT_ENABLE)
app_status_t app_status[2];
#else
app_status_t app_status[NO_OF_TYPEC_PORTS];
#endif /* CCG_TYPE_A_PORT_ENABLE */

/* Flag to indicate that activity timer timed out. */
static volatile bool ccg_activity_timer_timeout = false;

#if CCG_REV3_HANDLE_BAD_SINK
/* Bad sink timeout status */
volatile uint8_t gl_bad_sink_timeout_status[NO_OF_TYPEC_PORTS];
/* Bad PD sink connected status */
volatile uint8_t gl_bad_sink_pd_status[NO_OF_TYPEC_PORTS];
#endif /* CCG_REV3_HANDLE_BAD_SINK */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE)
extern bool gl_power_throttle_cmd_pending[NO_OF_TYPEC_PORTS];
extern bool gl_power_throttle_renegotiation_complete[NO_OF_TYPEC_PORTS];
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */

#if ICL_RVP_HW && ICL_MAIN_FW
static void auto_dr_swap_cbk(uint8_t port, timer_id_t id);

static void auto_dr_swap_resp_cb (uint8_t port, resp_status_t resp, const pd_packet_t *pkt_ptr)
{
    if(
        /* If a "WAIT" was received, retry */
        ((resp == RES_RCVD) && (pkt_ptr->hdr.hdr.msg_type == CTRL_MSG_WAIT))
        /* If any other error was seen, retry */
        || (resp < CMD_SENT)
      )
    {
        timer_start(port, APP_AUTO_DR_SWAP_TIMER, APP_AUTO_DR_SWAP_TRY_PERIOD, auto_dr_swap_cbk);
    }
}

static void auto_dr_swap_cbk(uint8_t port, timer_id_t id)
{
    const dpm_status_t* dpm_stat = dpm_get_info(port);
    const app_status_t* app_stat = app_get_status(port);
    
    dpm_pd_cmd_buf_t pd_cmd_buf;   
    pd_cmd_buf.cmd_sop = SOP;
    
    /* If port is UFP and no alt mode has been entered, try a DR_Swap to switch to DFP */
    if((dpm_stat->cur_port_type == PRT_TYPE_UFP)
        && (app_stat->alt_mode_entered == false)
        && dpm_stat->contract_exist)
    {
        if( 
            (icl_vsys_is_present() == false) /* If VSYS is not ready, try again later*/
            || (dpm_pd_command(port, DPM_CMD_SEND_DR_SWAP, &pd_cmd_buf, auto_dr_swap_resp_cb) != CCG_STAT_SUCCESS) /* If cmd failed, retry */
          )
        {
            timer_start(port, APP_AUTO_DR_SWAP_TIMER, APP_AUTO_DR_SWAP_TRY_PERIOD, auto_dr_swap_cbk);
        }   
    }
}   
#endif /* ICL_RVP_HW */

bool app_validate_configtable_offsets()
{
    uint8_t  port;
#if CCG_REG_SEC_CTRL
    uint32_t sil_id = 0;
    sys_get_silicon_id(&sil_id);
#endif /* CCG_REG_SEC_CTRL */
    for(port = TYPEC_PORT_0_IDX ; port < NO_OF_TYPEC_PORTS; port++)
    {
#if (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S))
        if ((get_pd_port_config(port)->pwr_tbl_offset == 0) || 
            (pd_get_ptr_pwr_tbl(port)->table_len < sizeof(pwr_params_t))
#if CCG_REG_SEC_CTRL
        || (sil_id != 0x2B01)
#endif /* CCG_REG_SEC_CTRL */
)
        {
            return false;
        }

        if(VBUS_CTRL_TYPE_P1 != pd_get_ptr_pwr_tbl(port)->fb_type)
        {
            return false;
        }
#endif /* (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S)) */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE)
        if ((get_pd_port_config(port)->auto_cfg_tbl_offset == 0) || 
            (pd_get_ptr_auto_cfg_tbl(port)->table_len < sizeof(auto_cfg_settings_t)))
        {
            return false;
        }
        /* No retries for Auto Table. */
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */

        /* Initialize the fault-handler variables. */
        if (fault_handler_init_vars (port) == false)
        {
            return false;
        }

#if VCONN_OCP_ENABLE
        if ((get_pd_port_config(port)->vconn_ocp_tbl_offset == 0) || 
            (pd_get_ptr_vconn_ocp_tbl(port)->table_len < sizeof(vconn_ocp_settings_t)))
        {
            return false;
        }

        /* No retries for VCONN OCP. */
#endif /* VCONN_OCP_ENABLE */

#if OTP_ENABLE
        if ((get_pd_port_config(port)->otp_tbl_offset == 0) || 
            (pd_get_ptr_otp_tbl(port)->table_len < sizeof(otp_settings_t)))
        {
            return false;
        }

        /* No retries for OTP. */
#endif /* OTP_ENABLE */

#if BATTERY_CHARGING_ENABLE
#if CCG_TYPE_A_PORT_ENABLE
        if (port == TYPE_A_PORT_ID)
        {
            if ((get_pd_port_config(0)->type_a_chg_tbl_offset == 0) || 
                (pd_get_ptr_type_a_chg_cfg_tbl(0)->table_len < sizeof(typeA_chg_cfg_params_t)))
            {
                return false;
            }
        }
        else
#endif /* CCG_TYPE_A_PORT_ENABLE */
        {
            if ((get_pd_port_config(port)->chg_cfg_tbl_offset == 0)  || 
                (pd_get_ptr_chg_cfg_tbl(port)->table_len < sizeof(chg_cfg_params_t)))
            {
                return false;
            }
        }
#endif /* BATTERY_CHARGING_ENABLE */

#if POWER_BANK
        if ((get_pd_port_config(port)->bat_chg_tbl_offset == 0)  || 
            (pd_get_ptr_bat_chg_tbl(port)->table_len < sizeof(bat_chg_params_t)))
        {
            return false;
        }
#endif /* POWER_BANK */

#if (CCG_TYPE_A_PORT_ENABLE == 1)
        if (get_pd_port_config(port)->type_a_enable)
        {
            if ((get_pd_port_config(port)->type_a_pwr_tbl_offset == 0)  || 
                (pd_get_ptr_type_a_pwr_tbl(port)->table_len < sizeof(pwr_params_t)))
            {
                return false;
            }
        }
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */

#if (CCG_BB_ENABLE != 0)
        if ((get_pd_port_config(port)->bb_tbl_offset == 0)  || 
            (pd_get_ptr_bb_tbl(port)->table_len < sizeof(bb_settings_t)))
        {
            return false;
        }
#endif /* (CCG_BB_ENABLE != 0) */
    }

    return true;
}

#if ((defined(CCG3PA)) || (defined(CCG3PA2)) || (defined(PAG1S)))
bool ccg_app_is_idle(void)
{
    /*
     * If activity timer timeout event is not pending, CCG is idle and system can
     * enter low power mode.
     */
    return !ccg_activity_timer_timeout;
}

void ccg_activity_timer_cb(uint8_t instance, timer_id_t id)
{
    (void)instance;
    (void)id;
    /*
     * Activity timer expired. Generate an event so that CCG periodic checks
     * can run.
     */
    ccg_activity_timer_timeout = true;
}

void ccg_app_task(uint8_t port)
{
#if CCG_CABLE_COMP_ENABLE
    ccg_cable_comp_task(port);
#endif /* CCG_CABLE_COMP_ENABLE */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
    ccg_sensor_debounce_task();
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */

#if CCG_LOAD_SHARING_ENABLE
    ccg_ls_task(port);
#endif /* CCG_LOAD_SHARING_ENABLE */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE)
    ccg_power_throttle_task(port);
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */

    /* Check VBATT, OTP and TYPE-A current consumption if activity timer has timed out. */
    if (ccg_activity_timer_timeout == true)
    {
#if (POWER_BANK == 1)
        pb_bat_monitor ();
#endif /* (POWER_BANK == 1) */

#if (CCG_TYPE_A_PORT_ENABLE == 1)
        if (get_pd_port_config(0)->type_a_enable)
        {
            type_a_detect_disconnect ();
        }
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */

#if (OTP_ENABLE == 1)
        app_otp_check_temp (port);
#endif /* OTP_ENABLE */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
        ccg_sensor_check();
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */

#if (defined(PAG1S) && CCG_REG_SEC_CTRL)
        pd_pasc_poll_task(port);
#endif /* (defined(PAG1S) && CCG_REG_SEC_CTRL) */

#if ((OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ENABLE == 1) || \
    (CCG_TEMP_BASED_VOLTAGE_THROTTLING == 1) || (CCG_VIN_BASED_VOLTAGE_THROTTLING == 1) || \
    (CCG_LOAD_SHARING_ENABLE == 1) || (CCG_PASC_VALLEY_FW_ALGO_ENABLE == 1))
        ccg_activity_timer_timeout = false;
        timer_start (0, CCG_ACTIVITY_TIMER_ID, CCG_ACTIVITY_TIMER_PERIOD,
                ccg_activity_timer_cb);
#endif /* (OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ ENABLE == 1) */
    }
}

void ccg_app_task_init(void)
{
#if (POWER_BANK == 1)
    pb_task_init ();
#endif /* (POWER_BANK == 1) */

#if OTP_ENABLE
    /* Enable OTP. */
    app_otp_enable (NO_OF_TYPEC_PORTS);
#endif /* OTP_ENABLE */

    /*
     * Start CCG activity timer. This timer is periodically used to monitor
     * battrey voltage (in power bank application), TYPE-A current consumption
     * and OTP.
     */
#if ((OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ENABLE == 1) || \
    (CCG_TEMP_BASED_VOLTAGE_THROTTLING == 1) || (CCG_VIN_BASED_VOLTAGE_THROTTLING == 1) ||\
    (CCG_LOAD_SHARING_ENABLE == 1) || (CCG_PASC_VALLEY_FW_ALGO_ENABLE == 1))
    ccg_activity_timer_timeout = false;
    timer_start (0, CCG_ACTIVITY_TIMER_ID, CCG_ACTIVITY_TIMER_PERIOD,
            ccg_activity_timer_cb);
#endif /* (OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ENABLE == 1) */
}
#endif /* (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S)) */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
static bool app_is_vdm_task_ready(uint8_t port)
{
    /* Assume cable discovery finished when device is UFP. */
    bool retval = true;

#if DFP_ALT_MODE_SUPP

    const dpm_status_t *dpm_stat = dpm_get_info (port);

    /* This check only makes sense for DFP. */
    if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
    {
#if (ROLE_PREFERENCE_ENABLE)
        /* Don't proceed with alternate mode if DR_SWAP is pending. */
        if ((app_status[port].app_pending_swaps & APP_DR_SWAP_PENDING) != 0)
        {
            return false;
        }
#endif /* (ROLE_PREFERENCE_ENABLE) */

        /*
         * Set the cable discovered flag if:
         * 1. Cable discovery is disabled.
         * 2. EMCA present flag in DPM is set.
         * 3. Cable discovery process not restarted.
         */
        if (
                (dpm_stat->cbl_dsc == false) || 
               ((dpm_stat->emca_present != false) && 
                (app_status[port].disc_cbl_pending == false))
            )
        {
            app_status[port].cbl_disc_id_finished = true;
        }

        /* Return the status of Cable discovered flag. */
        retval = app_status[port].cbl_disc_id_finished;
    }

#endif /* DFP_ALT_MODE_SUPP */

    return retval;
}
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */

ccg_status_t app_disable_pd_port(uint8_t port, dpm_typec_cmd_cbk_t cbk)
{
    ccg_status_t retval = CCG_STAT_SUCCESS;

    if (timer_is_running (port, APP_FAULT_RECOVERY_TIMER))
    {
        /* If the HPI Master is asking us to disable the port, make sure all fault protection state is cleared. */
        app_status[port].fault_status &= ~(
                APP_PORT_VBUS_DROP_WAIT_ACTIVE | APP_PORT_SINK_FAULT_ACTIVE | APP_PORT_DISABLE_IN_PROGRESS |
                APP_PORT_VCONN_FAULT_ACTIVE | APP_PORT_V5V_SUPPLY_LOST);
        timer_stop(port, APP_FAULT_RECOVERY_TIMER);
        pd_typec_dis_rd(port, CC_CHANNEL_1);
        pd_typec_dis_rd(port, CC_CHANNEL_2);
        cbk(port, DPM_RESP_SUCCESS);
    }
    else
    {
        /* Just pass the call on-to the stack. */
        if (dpm_get_info(port)->dpm_enabled)
        {
            retval = dpm_typec_command(port, DPM_CMD_PORT_DISABLE, cbk);
        }
        else
        {
            cbk(port, DPM_RESP_SUCCESS);
        }
    }

    return retval;
}

#if CCG_REV3_HANDLE_BAD_SINK
/* Bad sink timer callback */
void app_bad_sink_timeout_cbk(uint8_t port, timer_id_t id)
{
    (void)id;
    const dpm_status_t* dpm_stat = dpm_get_info(port);

    /* Take bad sink action only if PD connected device and PD is REV3. */
    if (((dpm_stat->pd_connected == true) ||
         (gl_bad_sink_pd_status[port] == true)) &&
         (gl_bad_sink_timeout_status[port] == false))
    {
        /* Set bad sink status and retain it until port detach or disable */
        gl_bad_sink_timeout_status[port] = true;
#if (CCG_LOAD_SHARING_ENABLE)
        gl_bad_device[port] = true;
        gl_stop_ls[port] = true;
        gl_stop_power_throttle[port] = true;
#endif /* (CCG_LOAD_SHARING_ENABLE) */
        /* Initiate Type-C error recovery. */
        dpm_typec_command(port, DPM_CMD_TYPEC_ERR_RECOVERY, NULL);
    }
}
#endif /* CCG_REV3_HANDLE_BAD_SINK */

extern void ccg_hal_check_if_vsys_up (void);

static void app_cbl_dsc_timer_cb (uint8_t port, timer_id_t id);
static void app_cbl_dsc_callback (uint8_t port, resp_status_t resp, const pd_packet_t *pkt_ptr)
{
    /* Keep repeating the DPM command until we succeed. */
    if (resp == SEQ_ABORTED)
    {
        timer_start (port, APP_CBL_DISC_TRIGGER_TIMER, APP_CBL_DISC_TIMER_PERIOD, app_cbl_dsc_timer_cb);
    }
}

static void app_cbl_dsc_timer_cb (uint8_t port, timer_id_t id)
{
    if (dpm_pd_command (port, DPM_CMD_INITIATE_CBL_DISCOVERY, NULL, app_cbl_dsc_callback) != CCG_STAT_SUCCESS)
    {
        timer_start (port, APP_CBL_DISC_TRIGGER_TIMER, APP_CBL_DISC_TIMER_PERIOD, app_cbl_dsc_timer_cb);
    }
}

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
        
bool app_vdm_layer_reset(uint8_t port)
{
    const dpm_status_t *dpm_stat = dpm_get_info(port);
    app_status_t *app = app_get_status(port);
    bool stat = true;
    
    if ((dpm_stat->contract_exist) && (gl_dpm_port_type[port] == PRT_TYPE_DFP))
    {

#if ICL_CONFIG_DISABLE     
        /*ToDo - updated ICL config xml & .c as per SDK v3.4*/
#else   
        /*
         * Reset the alternate mode state machine. The cable discovery complete flag is also cleared so
         * that alternate mode state machine can be started at the end of cable discovery.
         */
        alt_mode_layer_reset(port);
#endif /*ICL_CONFIG_DISABLE*/    
        app->cbl_disc_id_finished = false;
        app->disc_cbl_pending     = true;
        /* Ask PD stack to trigger cable discovery. */
        if (dpm_pd_command(port, DPM_CMD_INITIATE_CBL_DISCOVERY,
                    NULL, app_cbl_dsc_callback) != CCG_STAT_SUCCESS)
        {
            timer_start(port, APP_CBL_DISC_TRIGGER_TIMER, APP_CBL_DISC_TIMER_PERIOD, app_cbl_dsc_timer_cb);
        }
    }
    else
    {
        stat = false;
    }
    return stat;
}
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

uint8_t app_task(uint8_t port)
{
    fault_handler_task (port);

#if CCG_REV3_HANDLE_BAD_SINK
    /* 
     * Check for bad sink timeout.
     * If the PD contract was not completed for APP_BAD_SINK_TIMEOUT_TIMER_PERIOD,
     * a Type-C error recovery was initiated. Downgrade the PD revision from
     * REV3 to REV2 as expected by few bad sinks to complete the contract.
     * Bad sink status will be maintained until port disconnect/disable.
     */
    if((gl_bad_sink_timeout_status[port] == true) && (dpm_get_info(port)->pe_fsm_state == PE_FSM_SRC_STARTUP))
    {
        dpm_downgrade_pd_port_rev(port);
    }
#endif /* CCG_REV3_HANDLE_BAD_SINK */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    /* If VDM processing is allowed */
    if (app_status[port].vdm_task_en != false)
    {
        /* Wait for cable discovery completion before going on Alt. Modes. */
        if (app_is_vdm_task_ready (port))
        {
            vdm_task_mngr (port);
        }
    }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

#if NCP_CLIND_ENABLE
    if(port == TYPEC_PORT_0_IDX)
    {
        Clind_OCP_Check(port);
    }
#endif /* NCP_CLIND_ENABLE */

#if (CCG_BB_ENABLE != 0)
    if (bb_is_present(port) != false)
    {
        bb_task(port);
    }
#endif /* (CCG_BB_ENABLE != 0) */

#if BATTERY_CHARGING_ENABLE
    uint8_t i;
    for (i = 0; i < NO_OF_BC_PORTS; i++)
    {
        bc_fsm (i);
    }
#endif /* BATTERY_CHARGING_ENABLE */

#if RIDGE_SLAVE_ENABLE
    ridge_slave_task();
#endif /* RIDGE_SLAVE_ENABLE */

#if (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S))
    /* Run polling tasks of CCG. */
    ccg_app_task(port);
#endif /* (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S)) */

#if ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6)))
    ccg_hal_check_if_vsys_up ();
#endif /* ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6))) */

#ifdef PAG1S
  #if CCG_PASC_LP_ENABLE
    pd_pasc_lp_task(port);
  #endif /* CCG_PASC_LP_ENABLE */
#endif /* PAG1S */

    return true;
}

bool app_deepsleep_allowed(void)
{
    bool out = true;

#if (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S))
    const dpm_status_t *dpm;
    uint8_t i = 0;

    /*
     * Deepsleep mode operation can only be supported when un-attached.
     * When attached, the references and the CSA block requires to be
     * active.
     */
    for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
    {
        dpm = dpm_get_info(i);

        if (dpm->attach == true)
        {
            out = false;
            break;
        }
    }
#endif /* (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S)) */

#ifdef PAG1S
    /*
     * In case of secondary regulator control, we will lose regulation
     * if we go into deep sleep. To avoid this, we need to monitor VBUS_IN
     * level and based on the state of it, wake up and get regulation going
     * again. If this is not done, restricting to use of SLEEP mode only.
     */
#if ((CCG_REG_SEC_CTRL) && (!CCG_PASC_LP_ENABLE))
    out = false;
#else /* !((CCG_REG_SEC_CTRL) && (!CCG_PASC_LP_ENABLE)) */
    if (out == true)
    {
        for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
        {
            /*
             * For PAG1S, there is an additional sleep check required. Instead of adding
             * a seperate function, this function is overridden to do the sleep entry
             * request also from this call.
             */
#if CCG_PASC_LP_ENABLE
            if (pd_pasc_lp_is_active(i) == false)
            {
                /* We are idle and detached. We can now start the LP mode operation. */
                pd_pasc_lp_enable(i);
            }
            if (pd_pasc_lp_ds_allowed(i) == false)
            {
                out = false;
            }
#endif /* CCG_PASC_LP_ENABLE */
        }
    }
#endif /* ((CCG_REG_SEC_CTRL) && (!CCG_PASC_LP_ENABLE)) */

#endif /* PAG1S */

    return out;
}

bool app_sleep(void)
{
    bool stat = true;
    uint8_t port;

    for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
#if CCG_BB_ENABLE
        if (!bb_enter_deep_sleep(port))
        {
            stat = false;
            break;
        }
#endif /* CCG_BB_ENABLE */

#if (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S))
        /*
         * Check if CCG polling tasks are not pending to be serviced and system can enter
         * low power mode.
         */
        if (ccg_app_is_idle () == false)
        {
            stat = false;
            break;
        }
#endif /* (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S)) */

        /* Don't go to sleep while CC/SBU fault handling is pending. */
        if ((app_status[port].fault_status & APP_PORT_SINK_FAULT_ACTIVE) != 0)
        {
            stat = false;
            break;
        }

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
        if (!is_vdm_task_idle(port))
        {
            stat = false;
            break;
        }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

#if (DP_UFP_SUPP) && (CCG_HPD_RX_ENABLE)
        /* CDT 245126 workaround: Check if HPD RX Activity timer is running.
         * If yes, don't enter deep sleep. */
        if (!is_hpd_rx_state_idle (port))
        {
            stat = false;
            break;
        }
#endif /* DP_UFP_SUPP && CCG_HPD_RX_ENABLE */
    }

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    if (stat)
    {
        for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
        {
            /* Prepare for deep-sleep entry. */
            alt_mode_mngr_sleep(port);
        }
    }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

    return stat;
}

void app_wakeup(void)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    uint8_t port;

    for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
        alt_mode_mngr_wakeup (port);
    }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */
}

#if (CCG_BB_ENABLE != 0)
/* Alternate mode entry timeout callback function. */
static void ame_tmr_cbk(uint8_t port, timer_id_t id)
{
    (void)id;

    /* Alternate modes are reset in vdm_task_mngr_deinit(). */
    bb_enable(port, BB_CAUSE_AME_TIMEOUT);
}
#endif /* (CCG_BB_ENABLE != 0) */

#if CCG_PD_REV3_ENABLE

void extd_msg_cb(uint8_t port, resp_status_t resp, const pd_packet_t* pkt_ptr)
{
    static pd_ams_type ams_type[NO_OF_TYPEC_PORTS];
    (void)pkt_ptr;
    if(resp == RES_RCVD)
    {
        dpm_set_chunk_transfer_running(port, ams_type[port]);
    }
    if(resp == CMD_SENT)
    {
        ams_type[port] = dpm_get_info(port)->non_intr_response;
    }
}

/* Global variable used as dummy data buffer to send Chunk Request messages. */
static uint32_t gl_extd_dummy_data;

static bool app_extd_msg_handler(uint8_t port, pd_packet_extd_t *pd_pkt_p)
{
    /* If this is a chunked message which is not complete, send another chunk request. */
    if ((pd_pkt_p->hdr.hdr.chunked == true) && (pd_pkt_p->hdr.hdr.data_size >
               ((pd_pkt_p->hdr.hdr.chunk_no + 1) * MAX_EXTD_MSG_LEGACY_LEN)))
    {
        dpm_pd_cmd_buf_t extd_dpm_buf;

        extd_dpm_buf.cmd_sop = pd_pkt_p->sop;
        extd_dpm_buf.extd_type = pd_pkt_p->msg;
        extd_dpm_buf.extd_hdr.val = 0;
        extd_dpm_buf.extd_hdr.extd.chunked = true;
        extd_dpm_buf.extd_hdr.extd.request = true;
        extd_dpm_buf.extd_hdr.extd.chunk_no = pd_pkt_p->hdr.hdr.chunk_no + 1;
        extd_dpm_buf.dat_ptr = (uint8_t*)&gl_extd_dummy_data;
        extd_dpm_buf.timeout = PD_SENDER_RESPONSE_TIMER_PERIOD;

        /* Send next chunk request */
        dpm_pd_command_ec(port, DPM_CMD_SEND_EXTENDED,
                &extd_dpm_buf, extd_msg_cb);
    }
    else
    {
#if CCG_SLN_EXTN_MSG_HANDLER_ENABLE
        /* If macro is enabled - allow handling the requests from solution space. */
        return false;
#else
        /*
         * Don't send any response to response messages. Handling here instead of in the stack so that
         * these messages can be used for PD authentication implementation.
         */
        if ((pd_pkt_p->msg != EXTD_MSG_SECURITY_RESP) && (pd_pkt_p->msg != EXTD_MSG_FW_UPDATE_RESP))
        {
            /* Send Not supported message */
            dpm_pd_command_ec(port, DPM_CMD_SEND_NOT_SUPPORTED, NULL, NULL);
        }
#endif /* CCG_HANDLE_EXT_MSG_IN_SOL */
    }

    return true;
}
#endif /* CCG_PD_REV3_ENABLE */

void app_update_bc_src_support(uint8_t port, uint8_t enable)
{
#if BATTERY_CHARGING_ENABLE && (defined(CCG5C) || defined(CCG5) || defined(CCG6))
    app_status[port].bc_12_src_disabled = (bool)(!enable);
    if (!enable)
    {
        bc_stop(port);
    }
#endif /* BATTERY_CHARGING_ENABLE && (defined(CCG5C) || defined(CCG5) || defined(CCG6)) */
}

void app_update_sys_pwr_state(uint8_t state)
{
    
#if BB_RETIMER_ENABLE
    gl_system_state = state;
    /* We're not checking if any other retimer write is pending/about to happen.
     * No code space for any fine tuned checks. So do a raw retimer write */
    retimer_set_evt(0, RT_EVT_UPDATE_STATUS);
#if CCG_PD_DUALPORT_ENABLE
    retimer_set_evt(1, RT_EVT_UPDATE_STATUS);
#endif /* CCG_PD_DUALPORT_ENABLE */
#endif /* BB_RETIMER_ENABLE */

#if BATTERY_CHARGING_ENABLE && (defined(CCG5C) || defined(CCG5) || defined(CCG6))
#if (!CCG_SINK_ONLY)
    uint8_t i = 0;

    /*
       If BC 1.2 support is enabled and the system power state changed; proceed with further checks.
       If we are currently operating as CDP, don't make any changes.
       If we are currently operating as DCP and the new power state is still S3/S4/S5, don't make any changes.
       If we are currently operating as DCP and the new power state is S0, trigger error recovery to allow
       device enumeration (only if no alternate modes are active).
     */
    if (
            (app_status[i].bc_12_src_disabled == 0) &&
            (state == NB_SYS_PWR_STATE_S0)
       )
    {
#if CCG_PD_DUALPORT_ENABLE
        for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
#endif /* CCG_PD_DUALPORT_ENABLE */
        {
            if (
                    (dpm_get_info(i)->attach) &&
                    (dpm_get_info(i)->cur_port_role == PRT_ROLE_SOURCE) &&
                    (bc_port_is_cdp(i) == false) &&
                    ((alt_mode_get_status(i) & 0x7F) == 0)
               )
            {
                dpm_typec_command (i, DPM_CMD_TYPEC_ERR_RECOVERY, NULL);
            }
        }
    }
#endif /* (!CCG_SINK_ONLY) */
#endif /* BATTERY_CHARGING_ENABLE && (defined(CCG5C) || defined(CCG5) || defined(CCG6)) */
}

#if (BATTERY_CHARGING_ENABLE && ((defined(CCG5)) || (((defined(CCG3PA) || defined(CCG3PA2)) && defined(CCG_CDP_EN)))))

/* Function to start the CCG3PA/CCG5 BC 1.2 source state machine. */
void app_bc_12_sm_start(uint8_t port)
{

#if (((defined(CCG3PA) || defined(CCG3PA2)) && defined(CCG_CDP_EN)))
    /* Proceed with CCG3PA BC1.2 CDP source state machine */
    ccg_bc_cdp_en(port);
#else /* !(((defined(CCG3PA) || defined(CCG3PA2)) && defined(CCG_CDP_EN))) */

/* Proceed with CCG5 BC1.2 Source State Machine. 
 * Enable CDP is system power state is S0, otherwise enable DCP.
 */
#if CCG_HPI_ENABLE
    if (hpi_get_sys_pwr_state () == NB_SYS_PWR_STATE_S0)
#endif /* CCG_HPI_ENABLE */
    {
        ccg_bc_cdp_en(port);
    }
#if CCG_HPI_ENABLE
    else
    {
        ccg_bc_dcp_en(port);
    }
#endif /* CCG_HPI_ENABLE */
#endif /* (((defined(CCG3PA) || defined(CCG3PA2)) && defined(CCG_CDP_EN))) */
}

#endif /* (BATTERY_CHARGING_ENABLE && ((defined(CCG5)) || (((defined(CCG3PA) || defined(CCG3PA2)) && defined(CCG_CDP_EN))))) */

uint32_t get_bat_status[NO_OF_TYPEC_PORTS];

#if (ROLE_PREFERENCE_ENABLE)

/* Variable storing current preference for data role. */
volatile uint8_t app_pref_data_role[NO_OF_TYPEC_PORTS];

/* Variable storing current preference for power role. */
volatile uint8_t app_pref_power_role[NO_OF_TYPEC_PORTS];

/* Forward declaration of function to trigger swap operations. */
static void app_initiate_swap (uint8_t port, timer_id_t id);

static void app_role_swap_resp_cb (
        uint8_t            port,
        resp_status_t      resp,
        const pd_packet_t *pkt_ptr)
{
    app_status_t *app_stat = &app_status[port];
    bool next_swap = false;

    if (resp == RES_RCVD)
    {
        if (pkt_ptr->hdr.hdr.msg_type == CTRL_MSG_WAIT)
        {
            app_stat->actv_swap_count++;
            if (app_stat->actv_swap_count < APP_MAX_SWAP_ATTEMPT_COUNT)
            {
                timer_start (port, APP_INITIATE_SWAP_TIMER, app_stat->actv_swap_delay, app_initiate_swap);
            }
            else
            {
                /* Swap attempts timed out. Proceed with next swap. */
                next_swap = true;
            }
        }
        else
        {
            /* Swap succeeded or failed. Proceed with next swap. */
            next_swap = true;
        }
    }
    else if ((resp == CMD_FAILED) || (resp == SEQ_ABORTED) || (resp == RES_TIMEOUT))
    {
        timer_start (port, APP_INITIATE_SWAP_TIMER, app_stat->actv_swap_delay, app_initiate_swap);
    }

    if (next_swap)
    {
        /* Clear the LS bit in the app_pending_swaps flag as it has completed or failed. */
        app_stat->app_pending_swaps &= (app_stat->app_pending_swaps - 1);

        app_stat->actv_swap_type  = 0;
        app_stat->actv_swap_count = 0;
        timer_start (port, APP_INITIATE_SWAP_TIMER, APP_INITIATE_DR_SWAP_TIMER_PERIOD, app_initiate_swap);
    }
}

static void app_initiate_swap (uint8_t port, timer_id_t id)
{
    const dpm_status_t *dpm_stat = dpm_get_info(port);
    dpm_pd_cmd_buf_t pd_cmd_buf;
    uint8_t swaps_pending = app_status[port].app_pending_swaps;
    uint8_t actv_swap = app_status->actv_swap_type;

    /* Stop the timer that triggers swap operation. */
    timer_stop (port, APP_INITIATE_SWAP_TIMER);

    /* Nothing to do if we are not in PD contract. */
    if (!dpm_stat->contract_exist)
        return;

    if (actv_swap == 0)
    {
        /* No ongoing swap operation. Pick the next pending swap from the list. */
        if ((swaps_pending & APP_VCONN_SWAP_PENDING) != 0)
        {
            actv_swap = DPM_CMD_SEND_VCONN_SWAP;
            app_status[port].actv_swap_delay = APP_INITIATE_DR_SWAP_TIMER_PERIOD;
        }
        else
        {
            if ((swaps_pending & APP_DR_SWAP_PENDING) != 0)
            {
                actv_swap = DPM_CMD_SEND_DR_SWAP;
                app_status[port].actv_swap_delay = APP_INITIATE_DR_SWAP_TIMER_PERIOD;
            }
            else
            {
                if (swaps_pending != 0)
                {
                    actv_swap = DPM_CMD_SEND_PR_SWAP;
                    app_status[port].actv_swap_delay = APP_INITIATE_PR_SWAP_TIMER_PERIOD;
                }
            }
        }

        app_status[port].actv_swap_count = 0;
    }

    if (actv_swap != 0)
    {
        /* Check whether the selected swap is still valid. */
        switch (actv_swap)
        {
            case DPM_CMD_SEND_VCONN_SWAP:
                if (dpm_stat->vconn_logical)
                {
                    app_status[port].app_pending_swaps &= ~APP_VCONN_SWAP_PENDING;
                    actv_swap = 0;
                }
                break;

            case DPM_CMD_SEND_DR_SWAP:
                if (dpm_stat->cur_port_type == app_pref_data_role[port])
                {
                    app_status[port].app_pending_swaps &= ~APP_DR_SWAP_PENDING;
                    actv_swap = 0;
                }
                break;

            case DPM_CMD_SEND_PR_SWAP:
                if (dpm_stat->cur_port_role == app_pref_power_role[port])
                {
                    app_status[port].app_pending_swaps &= ~APP_PR_SWAP_PENDING;
                    actv_swap = 0;
                }
                break;

            default:
                actv_swap = 0;
                break;
        }

        if (actv_swap == 0)
        {
            /*
             * Currently selected SWAP is no longer relevant. Re-run function to identify the next swap to be
             * performed.
             */
            if (app_status[port].app_pending_swaps != 0)
            {
                app_status[port].actv_swap_type = 0;
                timer_start (port, APP_INITIATE_SWAP_TIMER, APP_INITIATE_DR_SWAP_TIMER_PERIOD, app_initiate_swap);
            }
        }
        else
        {
            /* Store the swap command for use in the callback. */
            app_status[port].actv_swap_type = actv_swap;

            /* Only packet type needs to be set when initiating swap operations. */
            pd_cmd_buf.cmd_sop = SOP;

            /* Try to trigger the selected swap operation. */
            if (dpm_pd_command(port, actv_swap, &pd_cmd_buf, app_role_swap_resp_cb) != CCG_STAT_SUCCESS)
            {
                /* Retries in case of AMS failure can always be done with a small delay. */
                timer_start (port, APP_INITIATE_SWAP_TIMER, APP_INITIATE_DR_SWAP_TIMER_PERIOD, app_initiate_swap);
            }
        }
    }
}

/* This function is called at the end of a PD contract to check whether any role swaps need to be triggered. */
static void app_contract_handler (uint8_t port)
{
    app_status_t *app_stat = &app_status[port];
    const dpm_status_t *dpm_stat = dpm_get_info(port);
    uint16_t delay_reqd = APP_INITIATE_PR_SWAP_TIMER_PERIOD;

#if (!CCG_SINK_ONLY)
    /* Check if we need to go ahead with PR-SWAP. */
    if (
            (app_pref_power_role[port] == PRT_DUAL) ||
            (dpm_get_info(port)->cur_port_role == app_pref_power_role[port])
       )
    {
        app_stat->app_pending_swaps &= ~APP_PR_SWAP_PENDING;
    }
    else
    {
        /* If we are about to swap to become source, ensure VConn Swap is done as required. */
        if ((app_pref_power_role[port] == PRT_ROLE_SOURCE) && (dpm_stat->vconn_logical == 0))
        {
            app_stat->app_pending_swaps |= APP_VCONN_SWAP_PENDING;

#if (DEFER_VCONN_SRC_ROLE_SWAP)
            /* Keep the VConn SWAP role until we have finished the required tasks. */
            app_stat->keep_vconn_src = true;
#endif /* (DEFER_VCONN_SRC_ROLE_SWAP) */
        }
    }
#endif /* (!CCG_SINK_ONLY) */

    /* Check if we need to go ahead with DR-SWAP. */
    if (
            (app_pref_data_role[port] == PRT_TYPE_DRP) ||
            (dpm_get_info(port)->cur_port_type == app_pref_data_role[port])
       )
    {
        app_stat->app_pending_swaps &= ~APP_DR_SWAP_PENDING;
    }
    else
    {
        /* DR-SWAPs need to be initiated as soon as possible. VConn swap will be triggered after DR_SWAP as needed. */
        delay_reqd = APP_INITIATE_DR_SWAP_TIMER_PERIOD;
    }

    /* Start a timer that will kick of the Swap state machine. */
    timer_start (port, APP_INITIATE_SWAP_TIMER, delay_reqd, app_initiate_swap);
}

static void app_connect_change_handler (uint8_t port)
{
    /* Stop all timers used to trigger swap operations. */
    timer_stop (port, APP_INITIATE_SWAP_TIMER);

    /*
     * Assume that PR_SWAP and DR_SWAP are pending. The actual status
     * will be updated on contract completion.
     */
    app_status[port].app_pending_swaps = APP_PR_SWAP_PENDING | APP_DR_SWAP_PENDING;
    app_status[port].actv_swap_type    = 0;
    app_status[port].actv_swap_count   = 0;
}

#endif /* (ROLE_PREFERENCE_ENABLE) */

static uint8_t gl_app_previous_polarity[NO_OF_TYPEC_PORTS];

void app_event_handler(uint8_t port, app_evt_t evt, const void* dat)
{
    const app_req_status_t* result;
    const pd_contract_info_t* contract_status;
    uint16_t dpm_voltage,dpm_current;
    bool  skip_soln_cb = false;
    bool  hardreset_cplt = false;
    bool  typec_only = false;
    const dpm_status_t *dpm_stat = dpm_get_info(port);

#if CCG_PD_REV3_ENABLE
    pd_do_t alert_ado;
#endif /* CCG_PD_REV3_ENABLE */

    switch(evt)
    {
        case APP_EVT_TYPEC_STARTED:
#if (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S))
#if (POWER_BANK == 1)
            /*
             * In power bank case, if device is powered by external VDDD (i.e not
             * in dead battery), disable internal VBUS regulator.
             */
            if (dpm_stat->dead_bat == false)
            {
                pd_hal_disable_vreg (port);
            }
#endif /* (POWER_BANK == 1) */
#endif /* (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S)) */

            /* Initialize the MUX to its default settings (isolate). */
            mux_ctrl_init (port);
            app_status[port].vdm_prcs_failed = false;
            break;

        case APP_EVT_TYPEC_ATTACH:
#if CCG_REV3_HANDLE_BAD_SINK
            /* Start bad sink timer */
            timer_stop(port, APP_BAD_SINK_TIMEOUT_TIMER);
            timer_start(port, APP_BAD_SINK_TIMEOUT_TIMER, APP_BAD_SINK_TIMEOUT_TIMER_PERIOD, app_bad_sink_timeout_cbk);
#if (CCG_LOAD_SHARING_ENABLE)
            if (true == gl_bad_device[port])
            {
                gl_stop_ls[port] = false;
                gl_stop_power_throttle[port] = false;
                gl_bad_device[port] = false;
            }
#endif /* (CCG_LOAD_SHARING_ENABLE) */
#endif /* CCG_REV3_HANDLE_BAD_SINK */
            /* This will also enable the USB (DP/DM) MUX where required. */
            set_mux (port, MUX_CONFIG_SS_ONLY, 0);

            /* Clear all fault counters if we have seen a change in polarity from previous connection. */
            if (dpm_stat->polarity != gl_app_previous_polarity[port])
            {
                fault_handler_clear_counts (port);
            }
            gl_app_previous_polarity[port] = dpm_stat->polarity;
            break;

        case APP_EVT_CONNECT:
            app_status[port].vdm_prcs_failed = false;
            app_status[port].cbl_disc_id_finished = false;
            app_status[port].disc_cbl_pending = false;

#if ICL_RVP_HW
            /* Debug Accessories don't raise the TYPEC_ATTACH event. So configuring the MUX here */
            if(dpm_stat->attached_dev == DEV_DBG_ACC)
            {
                /* Source or sink VBUS based on termination state */
                if (dpm_stat->cc_status.state == RP_CC1_RD_CC2_RD)
                    psrc_enable(port, NULL);
                else
                    psnk_enable(port);
                set_mux (port, MUX_CONFIG_SS_ONLY, 0);
                app_get_status(port)->debug_acc_attached = true;
            }
            
            /* If we're source/dfp, reject vconn swaps until cable discovery is done */
            if(dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
            {
                dpm_update_swap_response(port,
                    (dpm_stat->swap_response & ~DPM_VCONN_SWAP_RESP_MASK) | (APP_RESP_REJECT << DPM_VCONN_SWAP_RESP_POS));
                gl_block_swaps = true;
            }
            
#endif /* ICL_RVP_HW */            
            
#if (CCG_BB_ENABLE != 0)
            /* Enable the AME timer on attach if in sink mode. */
            if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
            {
                if (bb_bind_to_port(port) == CCG_STAT_SUCCESS)
                {
                    /* Start the AME timer in any case as the MUX needs to be setup in the callback in some cases. */
                    timer_start(port, APP_AME_TIMEOUT_TIMER, APP_AME_TIMEOUT_TIMER_PERIOD, ame_tmr_cbk);

                    /* Leave self power status flag cleared as we do not have a PD contract in place. */
                    bb_update_self_pwr_status (port, 0);
                }
            }
#endif /* (CCG_BB_ENABLE != 0) */

#if (ROLE_PREFERENCE_ENABLE)
            app_connect_change_handler (port);
#endif /* (ROLE_PREFERENCE_ENABLE) */
            break;

        case APP_EVT_HARD_RESET_COMPLETE:
            app_status[port].cbl_disc_id_finished = false;
            app_status[port].disc_cbl_pending = false;
            hardreset_cplt = true;
            /* Intentional fall-through. */

        case APP_EVT_HARD_RESET_SENT:
        case APP_EVT_PE_DISABLED:
            typec_only = ((dpm_stat->pd_connected == false) || (evt == APP_EVT_PE_DISABLED));
            /* Intentional fall-through. */

        case APP_EVT_HARD_RESET_RCVD:
        case APP_EVT_VBUS_PORT_DISABLE:
        case APP_EVT_DISCONNECT:
        case APP_EVT_TYPE_C_ERROR_RECOVERY:
#if ICL_RVP_HW
            gl_block_swaps = false;
            /* Intel: If debug accessory was supported and we disconnected, then disable power switches */
            if (app_get_status(port)->debug_acc_attached && (evt == APP_EVT_DISCONNECT))
            {
                psrc_disable(port, NULL);
                psnk_disable(port, NULL);
            }
#endif /*ICL_RVP_HW*/
            
#if CCG_REV3_HANDLE_BAD_SINK
            if ((evt == APP_EVT_DISCONNECT) || (evt == APP_EVT_VBUS_PORT_DISABLE))
            {
                /* Stop bad sink timer and clear bad sink status. */
                timer_stop(port, APP_BAD_SINK_TIMEOUT_TIMER);
                gl_bad_sink_timeout_status[port] = false;
                gl_bad_sink_pd_status[port] = false;
#if (CCG_LOAD_SHARING_ENABLE && CCG_REV3_HANDLE_BAD_SINK)
                gl_bad_device[port] = false;
#endif /* (CCG_LOAD_SHARING_ENABLE && CCG_REV3_HANDLE_BAD_SINK) */
            }
#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE)
            if((evt == APP_EVT_DISCONNECT) && (true == gl_power_throttle_cmd_pending[port]))
            {
                 gl_power_throttle_renegotiation_complete[port] = true;
            }
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */
            
            
#endif /* CCG_REV3_HANDLE_BAD_SINK */
#if (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S))
#if (POWER_BANK == 1)
            /*
             * In power bank case, if device is powered by external VDDD, VDDD gets
             * shorted to VBUS_IN line. This shall result connecting VDDD to the
             * Type-C VBUS line. This also includes cases where we start as dead
             * dead battery device and then get charged. So if any time VBUS has to
             * be removed in course of PD / Type-C state machine, ensure that internal
             * VBUS regulator is disabled. In event of dead battery, this shall lead
             * to device reset. This is the safest recovery path. CDT 276535.
             *
             * This code can be removed if the VBATT monitoring can be done
             * continously. But this code can still be in place to avoid any
             * corner case handling.
             */

            /* Do this only on disconnect and type-C error recovery. */
            if ((evt == APP_EVT_DISCONNECT) || (evt == APP_EVT_TYPE_C_ERROR_RECOVERY))
            {
                pd_hal_disable_vreg(port);
            }
#endif /* (POWER_BANK == 1) */
#endif /* (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S)) */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
            vdm_task_mngr_deinit (port);

#if ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6)))
            timer_stop (port, APP_CBL_DISC_TRIGGER_TIMER);
#endif /* ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6))) */

#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

            /*
             * Re-enable MUX in USB mode if hard reset has been completed.
             */
            if (hardreset_cplt)
            {
                set_mux (port, MUX_CONFIG_SS_ONLY, 0);
            }
            else
            {
                /*
                 * Isolate the data lines if this is a PD connection.
                 */
                if (!typec_only)
                {
                    set_mux (port, MUX_CONFIG_ISOLATE, 0);
                    timer_stop (port, APP_AME_TIMEOUT_TIMER);
                    
#if BB_RETIMER_ENABLE
                    /* Turn the retimer off after it's been written to */
                    CyDelayUs(10);
                    if(gl_system_state != SYSTEM_STATE_S0)
                        timer_start(port, APP_RETIMER_DISABLE_WAIT_TIMER, APP_RETIMER_DISABLE_DELAY, app_retimer_disable_cbk);
                    else
                        retimer_disable(port);
#endif /* BB_RETIMER_ENABLE */
                }
            }
#if (VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE | ICL_OCP_ENABLE)
            if(evt == APP_EVT_TYPE_C_ERROR_RECOVERY)
            {
                /* Clear port-in-fault flag if all fault counts are within limits. */
                if (!app_port_fault_count_exceeded(port))
                {
                    if ((app_status[port].fault_status & APP_PORT_DISABLE_IN_PROGRESS) == 0)
                    {
                        dpm_clear_fault_active(port);
                    }
                }
            }
#endif /* VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE */
            
            if ((evt == APP_EVT_DISCONNECT) || (evt == APP_EVT_VBUS_PORT_DISABLE))
            {
#if REGULATOR_REQUIRE_STABLE_ON_TIME
                /* Disable the regulator on port disconnect */
                REGULATOR_DISABLE();
#endif /* REGULATOR_REQUIRE_STABLE_ON_TIME */

                /* Cleanup the PD block states on disconnect. */
                pd_hal_cleanup(port);

#if VCONN_OCP_ENABLE
                /* Clear the VConn fault status. */
                app_status[port].fault_status &= ~APP_PORT_VCONN_FAULT_ACTIVE;
#endif /* VCONN_OCP_ENABLE */

#if CCG_BB_ENABLE
                if (evt == APP_EVT_DISCONNECT)
                {
                    bb_disable (port, true);

                    /* Clear self power status flag cleared on disconnect. */
                    bb_update_self_pwr_status (port, 0);
                }
#endif /* CCG_BB_ENABLE */       
            }

#if (ROLE_PREFERENCE_ENABLE)
            if (
                    (evt == APP_EVT_HARD_RESET_COMPLETE) ||
                    (evt == APP_EVT_TYPE_C_ERROR_RECOVERY) ||
                    (evt == APP_EVT_DISCONNECT)
               )
            {
                /* Stop the DR-Swap and PR-Swap trigger timers.  Assume that
                 * PR_SWAP and DR_SWAP are pending. The actual status will be
                 * updated on contract completion.
                 */
                app_connect_change_handler (port);
            }
#endif /* (ROLE_PREFERENCE_ENABLE) */

#if APP_PPS_SINK_SUPPORT
            /* Make sure the PPS re-negotiation task is stopped. */
            app_pps_sink_disable (port);
#endif /* APP_PPS_SINK_SUPPORT */
            break;

        case APP_EVT_EMCA_NOT_DETECTED:
#if ICL_RVP_HW
            /*ToDo */
            gl_block_swaps = false;
#endif /* ICL_RVP_HW */
        case APP_EVT_EMCA_DETECTED:
            app_status[port].cbl_disc_id_finished = true;
            app_status[port].disc_cbl_pending = false;
            app_status[port].vdm_prcs_failed = false;

#if DFP_ALT_MODE_SUPP
            /* If alt. mode state machine is inactive or idle, we can allow VConn swaps to proceed. */
            if (is_vdm_task_idle(port))
            {
#if (DEFER_VCONN_SRC_ROLE_SWAP)
                app_status[port].keep_vconn_src = false;
#endif /* (DEFER_VCONN_SRC_ROLE_SWAP) */
            }
#endif /* DFP_ALT_MODE_SUPP */

            /*
               Update the MUX settings with new cable information once EMCA detection is completed, only
               if we are still in USB mode.
             */
            if (get_mux_state(port) == MUX_CONFIG_SS_ONLY)
            {
                set_mux (port, MUX_CONFIG_SS_ONLY, 0);
            }
            break;

        case APP_EVT_VCONN_SWAP_COMPLETE:
#if ICL_RVP_HW            
            /* If we're now the VCONN supplier, reject more swaps until cable status can be determined */
            if(dpm_stat->vconn_logical == true)
            {
                dpm_update_swap_response(port,
                    (dpm_stat->swap_response & ~DPM_VCONN_SWAP_RESP_MASK) | (APP_RESP_REJECT << DPM_VCONN_SWAP_RESP_POS));
                gl_block_swaps = true;
            }
#endif /* ICL_RVP_HW */
            break;            
            
        case APP_EVT_DR_SWAP_COMPLETE:
            result = (const app_req_status_t*)dat ;
            if(*result == REQ_ACCEPT)
            {
#if (ROLE_PREFERENCE_ENABLE)
                app_status[port].app_pending_swaps &= ~APP_DR_SWAP_PENDING;
                if (app_status[port].actv_swap_type == DPM_CMD_SEND_DR_SWAP)
                {
                    timer_stop (port, APP_INITIATE_SWAP_TIMER);
                    app_contract_handler (port);
                }
#endif /* (ROLE_PREFERENCE_ENABLE) */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
#if ICL_CONFIG_DISABLE     
                /*ToDo - updated ICL config xml & .c as per SDK v3.4*/
#else     
                /* Device data role changed. Reset alternate mode layer. */
                alt_mode_layer_reset(port);
#endif /*ICL_CONFIG_DISABLE*/     
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

#if (CCG_BB_ENABLE != 0)
                /* Start tAME Timer to enable BB functionality */
                if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
                {
                    if (bb_bind_to_port(port) == CCG_STAT_SUCCESS)
                    {
                        timer_start(port, APP_AME_TIMEOUT_TIMER , APP_AME_TIMEOUT_TIMER_PERIOD, ame_tmr_cbk);
                    }
                }
                else
                {
                    timer_stop (port, APP_AME_TIMEOUT_TIMER);
                }
#endif /* (CCG_BB_ENABLE != 0) */
            }
            break;

        case APP_EVT_VENDOR_RESPONSE_TIMEOUT:
            /* If the APP layer is going to retry the VDM, do not send the event. */
            if (app_status[port].vdm_retry_pending)
                skip_soln_cb = true;
            break;

        case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE:
            contract_status = (pd_contract_info_t*)dat;
            
#if APP_DEBUG_DEVICE_TO_CHARGE_REQUEST
    
    		if(dpm_stat -> contract_exist == true)
			{
				dpm_voltage = dpm_stat->contract.max_volt / 100;
				dpm_current = dpm_stat->contract.cur_pwr / 5;
			}
#endif

            /* Set VDM version based on active PD revision. */
#if CCG_PD_REV3_ENABLE
            if (dpm_stat->spec_rev_sop_live >= PD_REV3)
            {
                app_status[port].vdm_version = STD_VDM_VERSION_REV3;
            }
            else
#endif /* CCG_PD_REV3_ENABLE */
            {
                app_status[port].vdm_version = STD_VDM_VERSION_REV2;
            }

            if ((contract_status->status == PD_CONTRACT_NEGOTIATION_SUCCESSFUL) ||
                    (contract_status->status == PD_CONTRACT_CAP_MISMATCH_DETECTED))
            {
#if CCG_REV3_HANDLE_BAD_SINK
                /* 
                 * Stop bad sink timer, but retain bad sink status until port
                 * detach or disable.
                 */
                timer_stop(port, APP_BAD_SINK_TIMEOUT_TIMER);
#endif /* CCG_REV3_HANDLE_BAD_SINK */
#if (ROLE_PREFERENCE_ENABLE)
                app_contract_handler (port);
#endif /* (ROLE_PREFERENCE_ENABLE) */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
                /*
                 * Contract established.  Enable VDM task manager for Alt. Mode support.
                 * This function will have no effect if the Alt. Modes are already running.
                 */
                if (
                        (gl_dpm_port_type[port] == PRT_TYPE_UFP) ||
                        (app_status[port].vdm_prcs_failed == false)
                   )
                {
                    enable_vdm_task_mngr(port);
                }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

#if ICL_RVP_HW && ICL_MAIN_FW
                /* See if DR_Swap is needed */
                auto_dr_swap_cbk(port, APP_AUTO_DR_SWAP_TIMER);
#endif /* ICL_RVP_HW */
            }

#if (CCG_BB_ENABLE != 0)
            /* Now that there is a PD contract in place, we can set the self powered status flag. */
            bb_update_self_pwr_status (port, 1);

            if (
                    (contract_status->status != PD_CONTRACT_NEGOTIATION_SUCCESSFUL) &&
                    (dpm_get_info(port)->cur_port_role == PRT_ROLE_SINK) &&
                    (gl_dpm_port_type[port] == PRT_TYPE_UFP)
               )
            {
                bb_enable(port, BB_CAUSE_PWR_FAILURE);
            }
#endif /* (CCG_BB_ENABLE != 0) */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE)
            if(true == gl_power_throttle_cmd_pending[port])
            {
                 gl_power_throttle_renegotiation_complete[port] = true;
            }
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */

            /* If we are not DFP, we can now allow the port partner to become VConn Source. */
            if (
                    (gl_dpm_port_type[port] == PRT_TYPE_UFP)
#if ROLE_PREFERENCE_ENABLE
                    &&
                    (app_pref_data_role[port] != PRT_TYPE_DFP)
#endif /* ROLE_PREFERENCE_ENABLE */
               )
            {
#if (DEFER_VCONN_SRC_ROLE_SWAP)
                app_status[port].keep_vconn_src = false;
#endif /* (DEFER_VCONN_SRC_ROLE_SWAP) */
            }
            break;
#if 0 /* Now covered by VCONN_OCP retry mechanism */
        case APP_EVT_VCONN_OCP_FAULT:
#if VCONN_OCP_ENABLE
            /* Store the VConn fault status. */
            app_status[port].fault_status |= APP_PORT_VCONN_FAULT_ACTIVE;

#if (DFP_ALT_MODE_SUPP)
            /* Exit any active alternate modes. */
            if (gl_dpm_port_type[port] == PRT_TYPE_DFP)
            {
                vdm_task_mngr_deinit(port);
            }
#endif /* (DFP_ALT_MODE_SUPP) */
#endif /* VCONN_OCP_ENABLE */
            break;
#endif /* 0 */

#if CCG_PD_REV3_ENABLE
        case APP_EVT_HANDLE_EXTENDED_MSG:
#if (CCG_HPI_PD_ENABLE)
            /* Handle the extended message locally if forwarding to EC is not enabled. */
            if (hpi_is_extd_msg_ec_ctrl_enabled (port) == false)
#endif
            {
                if(!(app_extd_msg_handler(port, (pd_packet_extd_t *)dat)))
                {
                    skip_soln_cb  = false;
                }
                else
                {
                    skip_soln_cb  = true;
                }
            }
            break;

        case APP_EVT_ALERT_RECEIVED:
            /* Respond to ALERT message only if there is the number of object is one. */
            if (((pd_packet_t*)dat)->len == 1)
            {
                alert_ado = ((pd_packet_t*)dat)->dat[0];
                if(alert_ado.ado_alert.bat_status_change == false)
                {
                    dpm_pd_command(port, DPM_CMD_GET_STATUS, NULL, NULL);
                }
                else
                {
                    uint8_t i = alert_ado.ado_alert.fixed_bats |
                        (alert_ado.ado_alert.hot_swap_bats << 4);
                    dpm_pd_cmd_buf_t cmd;

                    /* Identify the first battery for which the change is intended. */
                    get_bat_status[port] = 0;
                    while ((i != 0) && ((i & 0x01) == 0))
                    {
                        get_bat_status[port]++;
                        i >>= 1;
                    }

                    cmd.cmd_sop = SOP;
                    cmd.extd_hdr.val = 0x1;
                    cmd.timeout = PD_SENDER_RESPONSE_TIMER_PERIOD;
                    cmd.extd_type = EXTD_MSG_GET_BAT_STATUS;
                    cmd.dat_ptr = (uint8_t*)&get_bat_status[port];
                    dpm_pd_command(port, DPM_CMD_SEND_EXTENDED, &cmd, NULL);
                }
            }
            break;
#endif /* CCG_PD_REV3_ENABLE */




        case APP_EVT_TYPEC_ATTACH_WAIT:
#if REGULATOR_REQUIRE_STABLE_ON_TIME
            REGULATOR_ENABLE();
#endif /* REGULATOR_REQUIRE_STABLE_ON_TIME */

#if BB_RETIMER_ENABLE
            timer_stop(port, APP_RETIMER_DISABLE_WAIT_TIMER);
            retimer_enable(port);
#endif /* BB_RETIMER_ENABLE */

#if CCG_PASC_LP_ENABLE
            pd_pasc_lp_disable(port);
#endif /* CCG_PASC_LP_ENABLE */
            break;

#if REGULATOR_REQUIRE_STABLE_ON_TIME
        case APP_EVT_TYPEC_ATTACH_WAIT_TO_UNATTACHED:
            REGULATOR_DISABLE();
            break;
#endif /* REGULATOR_REQUIRE_STABLE_ON_TIME */


#if (ROLE_PREFERENCE_ENABLE)
        case APP_EVT_PR_SWAP_COMPLETE:
            app_status[port].app_pending_swaps &= ~APP_PR_SWAP_PENDING;
            if (app_status[port].actv_swap_type == DPM_CMD_SEND_PR_SWAP)
            {
                timer_stop (port, APP_INITIATE_SWAP_TIMER);
                app_contract_handler (port);
            }
            break;
#endif /* (ROLE_PREFERENCE_ENABLE) */

        case APP_EVT_ALT_MODE:
#if DFP_ALT_MODE_SUPP
            /* We can clear the keep VConn source flag once the alternate mode state machine has become idle. */
            if (is_vdm_task_idle (port))
            {
#if (DEFER_VCONN_SRC_ROLE_SWAP)
                app_status[port].keep_vconn_src = false;
#endif /* (DEFER_VCONN_SRC_ROLE_SWAP) */
            }
#endif /* DFP_ALT_MODE_SUPP */

#if (ROLE_PREFERENCE_ENABLE)
            /* Stop trying to initiate DR_SWAP once any alt. mode has been entered. */
            if (app_status[port].alt_mode_entered)
            {
                app_status[port].app_pending_swaps &= ~APP_DR_SWAP_PENDING;
                if (app_status[port].actv_swap_type == DPM_CMD_SEND_DR_SWAP)
                {
                    timer_stop (port, APP_INITIATE_SWAP_TIMER);
                    app_contract_handler (port);
                }
            }
#endif /* (ROLE_PREFERENCE_ENABLE) */
            break;

        case APP_EVT_PD_SINK_DEVICE_CONNECTED:
#if CCG_REV3_HANDLE_BAD_SINK
            gl_bad_sink_pd_status[port] = true;
#endif /* CCG_REV3_HANDLE_BAD_SINK */
            break;
        
#if 0
        /* Default handlers are sufficient for these cases. */
        case APP_EVT_UNEXPECTED_VOLTAGE_ON_VBUS:
        case APP_EVT_RP_CHANGE:
        case APP_EVT_PKT_RCVD:
        case APP_EVT_VCONN_SWAP_COMPLETE:
        case APP_EVT_SENDER_RESPONSE_TIMEOUT:
        case APP_EVT_SOFT_RESET_SENT:
        case APP_EVT_CBL_RESET_SENT:
#endif

        default:
            /* Nothing to do. */
            break;
    }

    /* Pass the event notification to the fault handler module. */
    if (fault_event_handler (port, evt, dat))
    {
        skip_soln_cb = true;
    }

#if BATTERY_CHARGING_ENABLE
    bc_pd_event_handler (port, evt);
#endif /* BATTERY_CHARGING_ENABLE */

#if POWER_BANK
    pb_event_handler (port, evt);
#endif /* POWER_BANK */

    if (!skip_soln_cb)
    {
        /* Send notifications to the solution */
        sln_pd_event_handler(port, evt, dat);
    }
}

app_resp_t* app_get_resp_buf(uint8_t port)
{
    return &app_status[port].app_resp;
}

app_status_t* app_get_status(uint8_t port)
{
    return &app_status[port];
}

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
bool app_is_port_enabled(uint8_t port)
{
    bool ret = true;

    if (port > NO_OF_TYPEC_PORTS)
        ret = false;

#if ((CCG_HPI_ENABLE) && (!CCG_LOAD_SHARING_ENABLE))
    /* Check if the port has been disabled through HPI. */
    if ((hpi_get_port_enable() & (1 << port)) == 0)
        ret = false;
#endif /* CCG_HPI_ENABLE */

    return ret;
}

#if CCGX_V5V_CHANGE_DETECT

#if DP_UFP_SUPP
static void app_ufp_5v_recov_cb(uint8_t port, timer_id_t id)
{
    (void)id;

    /* Create app alt mode command to initiate DP related Vconn Swap procedure */
    uint8_t dp_cmd[4]  = {0xA, 0x00, 0x01, 0xFF};
    uint8_t dp_data[4] = {0x00, 0x00, 0x00, DP_APP_VCONN_SWAP_CFG_CMD};

    if (eval_app_alt_mode_cmd(port, dp_cmd, dp_data) == false)
    {
        /* Init Hard reset if DP alt mode not entered */
        dpm_pd_command (port, DPM_CMD_SEND_HARD_RESET, NULL, NULL);
    }
}
#endif /* #if DP_UFP_SUPP */
#endif /* CCGX_V5V_CHANGE_DETECT */

/* Callback that will be called when there is any change to the V5V or VSYS supplies. */
void app_ccg5_supply_change_cb(uint8_t port, ccg_supply_t supply_id, bool present)
{
#if CCGX_V5V_CHANGE_DETECT
    const dpm_status_t *dpm_stat = dpm_get_info(port);

    /*
     * Currently we only handle V5V changes:
     * If V5V is removed, we exit active alternate modes if there is a cable which requires VConn.
     * If V5V is re-applied after being removed, we restart the alternate mode state machine.
     */
    if (supply_id == CCG_SUPPLY_V5V)
    {
        if (!present)
        {
            app_status[port].fault_status |= APP_PORT_V5V_SUPPLY_LOST;

            if (vconn_is_present (port))
            {
                /* Ensure that the VConn switch is turned off. */
                vconn_disable (port, dpm_stat->rev_pol);

#if (DFP_ALT_MODE_SUPP)
                /* If we are the DFP, the cable requires VConn and Alt. Modes are Active, exit alt. modes. */
                if (
                        (gl_dpm_port_type[port] == PRT_TYPE_DFP) &&
                        (dpm_stat->cbl_vdo.std_cbl_vdo.cbl_term != CBL_TERM_BOTH_PAS_VCONN_NOT_REQ) &&
                        (app_status[port].alt_mode_entered != 0)
                   )
                {
                    vdm_task_mngr_deinit(port);
                }
#endif /* (DFP_ALT_MODE_SUPP) */
            }
        }
        else
        {
            if ((app_status[port].fault_status & APP_PORT_V5V_SUPPLY_LOST) != 0)
            {
                app_status[port].fault_status &= ~APP_PORT_V5V_SUPPLY_LOST;

#if (DFP_ALT_MODE_SUPP)
                /*
                 * Alt. Mode operation was previously suspended due to V5V not being present.
                 * We can restart the alt. mode state machine so that mode discovery and operation
                 * can take place.
                 */
                if ((dpm_stat->contract_exist) && (gl_dpm_port_type[port] == PRT_TYPE_DFP))
                {
                    /* Ask PD stack to trigger cable discovery. */
                    if (dpm_pd_command(port, DPM_CMD_INITIATE_CBL_DISCOVERY,
                                NULL, app_cbl_dsc_callback) != CCG_STAT_SUCCESS)
                    {
                        timer_start(port, APP_CBL_DISC_TRIGGER_TIMER, APP_CBL_DISC_TIMER_PERIOD, app_cbl_dsc_timer_cb);
                    }
#if ICL_CONFIG_DISABLE     
                    /*ToDo - updated ICL config xml & .c as per SDK v3.4*/
#else
                    /*
                     * Reset the alternate mode state machine. The cable discovery complete flag is also cleared so
                     * that alternate mode state machine can be started at the end of cable discovery.
                     */
                    alt_mode_layer_reset(port);
#endif /*ICL_CONFIG_DISABLE*/    
                    app_status[port].cbl_disc_id_finished = false;
                }
#endif /* (DFP_ALT_MODE_SUPP) */
#if DP_UFP_SUPP
                if ((dpm_stat->contract_exist) && (gl_dpm_port_type[port] == PRT_TYPE_UFP))
                {
                    timer_start(port, APP_UFP_RECOV_VCONN_SWAP_TIMER, APP_UFP_RECOV_VCONN_SWAP_TIMER_PERIOD, app_ufp_5v_recov_cb);
                }
#endif /* DP_UFP_SUPP */
            }
        }
    }
#endif /* CCGX_V5V_CHANGE_DETECT */
}

/* Callback used to receive fault notification from the HAL and to pass it on to the event handler. */
static void app_cc_sbu_fault_handler(uint8_t port, bool fault_type)
{
    app_event_handler(port, (fault_type ? APP_EVT_SBU_OVP : APP_EVT_CC_OVP), 0);
}

#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

void app_init(void)
{
    uint8_t port;

    /* For now, only the VDM handlers require an init call. */
    for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
        /* Initialize the VDM responses from the configuration table. */
        vdm_data_init(port);

        /* Set alt mode trigger based on config */
        app_get_status(port)->alt_mode_trig_mask = get_pd_port_config(port)->alt_mode_trigger;

#if ((defined(CCG6)) && OTP_ENABLE)
        /* Init OT temperature tracking */
        app_otp_enable (port);
#endif /* ((defined(CCG6)) && OTP_ENABLE) */

#if (CCG_BB_ENABLE != 0)
        /*
         * Initialize the billboard interface. The billboard
         * interface shall not get initialized if it is not
         * enabled in configuration table.
         */
        bb_init(port);
#endif /* (CCG_BB_ENABLE != 0) */

#if BATTERY_CHARGING_ENABLE
        bc_init(port);
#endif /* BATTERY_CHARGING_ENABLE */

#if (ROLE_PREFERENCE_ENABLE)
        if (get_pd_port_config(port)->tbthost_cfg_tbl_offset != 0)
        {
            app_pref_power_role[port] = (pd_get_ptr_tbthost_cfg_tbl(port)->pref_pwr_role);
            app_pref_data_role[port] = (pd_get_ptr_tbthost_cfg_tbl(port)->pref_data_role);
        }
        else
        {
            /* Initialize with no preference by default. */
            app_pref_power_role[port] = PRT_DUAL;
            app_pref_data_role[port] = PRT_TYPE_DRP;
        }
#endif /* (ROLE_PREFERENCE_ENABLE) */

#if ICL_CONFIG_DISABLE     
        /*ToDo - updated ICL config xml & .c as per SDK v3.4*/
#else 
        #if DFP_ALT_MODE_SUPP 
            /* Save mask to enable DFP alt modes */
            app_get_status(port)->dfp_alt_mode_mask = get_alt_modes_config_mask(port, PRT_TYPE_DFP);
        #endif /* DFP_ALT_MODE_SUPP */
        #if UFP_ALT_MODE_SUPP
            /* Save mask to enable UFP alt modes */
            app_get_status(port)->ufp_alt_mode_mask = get_alt_modes_config_mask(port, PRT_TYPE_UFP);
        #endif /* DFP_ALT_MODE_SUPP */
#endif /*ICL_CONFIG_DISABLE*/
    }

#if (CCG_TYPE_A_PORT_ENABLE == 1)
    /* For systems with TYPEA port. */
    if (get_pd_port_config(0)->type_a_enable)
    {
        type_a_port_enable();
    }
    else
    {
        /* Ensure that the state machine variables are initialized correctly. */
        type_a_port_disable();
    }
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))

#if (CCG_CC_SBU_OVP_RETRY_LIMIT != 0)
    /* Set the retry limit where it is non-zero. */
    gl_app_fault_retry_limit[0][FAULT_TYPE_CC_OVP]  = CCG_CC_SBU_OVP_RETRY_LIMIT;
    gl_app_fault_retry_limit[0][FAULT_TYPE_SBU_OVP] = CCG_CC_SBU_OVP_RETRY_LIMIT;

#if CCG_PD_DUALPORT_ENABLE
    gl_app_fault_retry_limit[1][FAULT_TYPE_CC_OVP]  = CCG_CC_SBU_OVP_RETRY_LIMIT;
    gl_app_fault_retry_limit[1][FAULT_TYPE_SBU_OVP] = CCG_CC_SBU_OVP_RETRY_LIMIT;
#endif /* CCG_PD_DUALPORT_ENABLE */
#endif /* (CCG_CC_SBU_OVP_RETRY_LIMIT != 0) */

    /* Register a callback for notification of CC/SBU faults. */
    ccg_set_fault_cb(app_cc_sbu_fault_handler);

    /* Register a handler that will be notified when there is any change in V5V or VSYS state. */
    pd_hal_set_supply_change_evt_cb(app_ccg5_supply_change_cb);
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

#if APP_PPS_SINK_SUPPORT
    hpi_set_userdef_write_handler (hpi_user_reg_handler);
#endif /* APP_PPS_SINK_SUPPORT */
}

/* Implements CCG deep sleep functionality for power saving. */
bool system_sleep(void)
{
    uint8_t intr_state;
    bool dpm_slept = false;
    bool app_slept = false;
    bool retval = false;
#if BATTERY_CHARGING_ENABLE
    bool bc_slept = false;
#endif /* BATTERY_CHARGING_ENABLE */

    intr_state = CyEnterCriticalSection();

    /*
     * We have to check the application layer, HPI and the Device Policy
     * Manager (DPM) to see if all of these modules are ready for sleep.
     * CCG can only enter deep sleep if all of these blocks are in an idle
     * state.
     *
     * Note: The respective sleep functions might be performing some
     * state updates as part of the idle check function; and therefore
     * the corresponding wakeup function needs to be called if they have
     * returned true to indicate that sleep is allowed.
     */
    if (app_sleep())
    {
        app_slept = true;

#if BATTERY_CHARGING_ENABLE
        if(bc_sleep() == true)
        {
            bc_slept = true;
#endif /* BATTERY_CHARGING_ENABLE */

            if (
#if CCG_HPI_ENABLE
                    (hpi_sleep_allowed()) &&
#endif /* CCG_HPI_ENABLE */

#if CCG_UCSI_ENABLE
                    (ucsi_sleep_allowed()) &&
#endif /* CCG_UCSI_ENABLE */

#if ICL_RVP_HW
                    (icl_sleep_allowed()) &&
#endif /* ICL_RVP_HW */

#if BB_RETIMER_ENABLE
                    (retimer_sleep_allowed()) &&
#endif /* BB_RETIMER_ENABLE */

#if CCG_SYNC_ENABLE
                    (ccg_sync_sleep_allowed()) &&
#endif /* CCG_SYNC_ENABLE */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
                    ccg_sensor_is_idle() &&
#endif /*(CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */

#if CCG_LOAD_SHARING_ENABLE
                    ccg_ls_is_idle(0) &&
#endif /* CCG_LOAD_SHARING_ENABLE */
#if CCG_CABLE_COMP_ENABLE
                    ccg_cable_comp_is_idle(0) &&
#endif /* CCG_CABLE_COMP_ENABLE */
#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE)
                    ccg_power_throttle_sleep_allowed() &&
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */
                    (dpm_deepsleep())
               )
            {
                dpm_slept = true;
                
#if CCG_SYNC_ENABLE
                CCGComm_Sleep();
#endif /* CCG_SYNC_ENABLE */                
                
                /*
                 * Check connection status of TYPE-A and TYPE-C ports to determine if deepsleep
                 * entry is allowed. If not,enter sleep mode to save power.
                 */
                if (
#if CCG_TYPE_A_PORT_ENABLE
                    (type_a_is_idle() == true) &&
#endif /* CCG_TYPE_A_PORT_ENABLE */
                    (app_deepsleep_allowed() == true)
                    )
                {
                    timer_enter_sleep();

                    /*
                     * CDT 224642: The I2C IDLE check needs to be done as the last step
                     * before device enters into sleep. Otherwise, the device may fail
                     * to wake up when there is an address match on the I2C interface.
                     */
#if ((CCG_HPI_ENABLE) || (RIDGE_SLAVE_ENABLE))
                    if (
#if CCG_HPI_ENABLE
                            (hpi_sleep())
#else
                            (1)
#endif /* CCG_HPI_ENABLE */
                            &&
#if RIDGE_SLAVE_ENABLE
                            (ridge_slave_sleep())
#else
                            (1)
#endif /* RIDGE_SLAVE_ENABLE */
                       )
#endif /* ((CCG_HPI_ENABLE) || (RIDGE_SLAVE_ENABLE)) */
                    {
#if (defined(CCG3PA) || defined(CCG3PA2))
                        pd_hal_set_reference(0, true);
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */
                        /* Device sleep entry. */
                        CySysPmDeepSleep();
#if (defined(CCG3PA) || defined(CCG3PA2))
                        pd_hal_set_reference(0, false);
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */
                        retval = true;

                    }
                }
                else
                {
#if (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S))
                    /* Enter Sleep mode to save power. */
                    CySysPmSleep();
#endif /* (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S)) */
                }
#if CCG_SYNC_ENABLE
                CCGComm_Wakeup();
#endif /* CCG_SYNC_ENABLE */                
            }
#if BATTERY_CHARGING_ENABLE
        }
#endif /* BATTERY_CHARGING_ENABLE */
    }

    CyExitCriticalSection(intr_state);

    /* Call dpm_wakeup() if dpm_sleep() had returned true. */
    if(dpm_slept)
    {
        dpm_wakeup();
    }

    /* Call app_wakeup() if app_sleep() had returned true. */
    if(app_slept)
    {
        app_wakeup();
    }

#if BATTERY_CHARGING_ENABLE
    if(bc_slept)
    {
        bc_wakeup();
    }
#endif /* BATTERY_CHARGING_ENABLE */

    return retval;
}

#if VCONN_OCP_ENABLE
void app_vconn_ocp_cbk(uint8_t port, bool comp_out)
{
    /* Disable VConn since we hit a fault. */
    vconn_disable(port, dpm_get_info(port)->rev_pol);

    /* Notify application layer about fault. */
    app_event_handler(port, APP_EVT_VCONN_OCP_FAULT, NULL);
}
#endif /* VCONN_OCP_ENABLE */

bool vconn_enable(uint8_t port, uint8_t channel)
{
#if VCONN_OCP_ENABLE

    /* Do not attempt to enable VConn if fault was detected in present connection. */
    if ((app_status[port].fault_status & APP_PORT_VCONN_FAULT_ACTIVE) != 0)
    {
        return false;
    }

#if CCG_VCONN_MON_WITH_ADC

    /* If Vconn current is being monitored through ADC, configure the associated IOs. */
    hsiom_set_config (APP_VCONN_MON_PORT_PIN_P1,APP_VCONN_MON_AMUX_INPUT_P1);
#if CCG_PD_DUALPORT_ENABLE
    hsiom_set_config (APP_VCONN_MON_PORT_PIN_P2,APP_VCONN_MON_AMUX_INPUT_P2);
#endif /* CCG_PD_DUALPORT_ENABLE */

    /*
     * 120us delay required as a settling time after HSIOM config to get a stable
     * ADC reading.
     */
    CyDelayUs(120);

#endif /* CCG_VCONN_MON_WITH_ADC */

    system_vconn_ocp_en(port, app_vconn_ocp_cbk);
#endif /* VCONN_OCP_ENABLE */

    /* Reset RX Protocol for cable */
    dpm_prot_reset_rx(port, SOP_PRIME);
    dpm_prot_reset_rx(port, SOP_DPRIME);

    if (pd_vconn_enable(port, channel) != CCG_STAT_SUCCESS)
    {
#if ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6)))
        app_status[port].fault_status |= APP_PORT_V5V_SUPPLY_LOST;
#endif /* ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6))) */
        return false;
    }
    return true;
}

void vconn_disable(uint8_t port, uint8_t channel)
{
    pd_vconn_disable(port, channel);

#if VCONN_OCP_ENABLE
    system_vconn_ocp_dis(port);
#endif /* VCONN_OCP_ENABLE */
}

bool vconn_is_present(uint8_t port)
{
    return pd_is_vconn_present( port, dpm_get_info(port)->rev_pol);
}

bool vbus_is_present(uint8_t port, uint16_t volt, int8 per)
{
    uint8_t level;
    uint8_t retVal;

#ifdef CCG4
    /*
     * OVP comparator is multiplexed with VBUS polling.
     * To avoid false output on OVP Trip pin when VBUS is polled
     * OVP trip pin is disconnected from OVP comp output and last
     * value of OVP comp output is driven via GPIO.
     */
    system_disconnect_ovp_trip(port);
#endif /* CCG4*/

    /*
     * Re-run calibration every time to ensure that VDDD or the measurement
     * does not break.
     */
    pd_adc_calibrate (port, APP_VBUS_POLL_ADC_ID);
    level = pd_get_vbus_adc_level(port, APP_VBUS_POLL_ADC_ID, volt, per);
    retVal = pd_adc_comparator_sample (port, APP_VBUS_POLL_ADC_ID, APP_VBUS_POLL_ADC_INPUT, level);

#ifdef CCG4
    /* Connect OVP trip pin back to comparator output */
    system_connect_ovp_trip(port);
#endif /* CCG4*/

    return retVal;
}

uint16_t vbus_get_value(uint8_t port)
{
    uint16_t retVal;

#ifdef CCG4
    /*
     * OVP comparator is multiplexed with VBUS polling. To avoid false output
     * on OVP Trip pin when VBUS is polled OVP trip pin is disconnected from
     * OVP comp output and last value of OVP comp output is driven via GPIO.
     */
    system_disconnect_ovp_trip(port);
#endif /* CCG4 */

    /* Measure the actual VBUS voltage. */
    retVal = pd_hal_measure_vbus(port);

#ifdef CCG4
    /* Connect OVP trip pin back to comparator output */
    system_connect_ovp_trip(port);
#endif /* CCG4*/

    return retVal;
}

#if VCONN_OCP_ENABLE
bool system_vconn_ocp_en(uint8_t port, PD_ADC_CB_T cbk)
{
    if (cbk == NULL)
    {
        return false;
    }

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))

    /* Request the HAL to enable OCP detection with the appropriate debounce period. */
    pd_hal_vconn_ocp_enable(port, pd_get_ptr_vconn_ocp_tbl(port)->debounce, cbk);

#else /* !defined (CCG5) || defined(CCG6) || defined(CCG5C) */
#if CCG4_DOCK
    PPDSS_REGS_T gl_pdss[NO_OF_TYPEC_PORTS] =
    {
        PDSS0
#if CCG_PD_DUALPORT_ENABLE
            ,
        PDSS1
#endif /* CCG_PD_DUALPORT_ENABLE */
    };
    PPDSS_REGS_T pd = gl_pdss[port];
    pd->pfet300_ctrl |= PDSS_PFET300_CTRL_EN_COMP;
    pd->intr_1_cfg &= ~PDSS_INTR_1_CFG_V5V_CFG_MASK;
    pd->intr_1_cfg |= CYVAL_USBPD_V5V_CFG_POS_EDG_DIS_NEG_EDG_EN << PDSS_INTR_1_CFG_V5V_CFG_POS;
    pd->intr1_mask |= PDSS_INTR1_MASK_V5V_CHANGED_MASK;
#else /* !CCG4_DOCK */
    /* Configure ADC to detect VConn Over-Current condition. */
    pd_adc_comparator_ctrl(port, APP_VCONN_OCP_ADC_ID, APP_VCONN_OCP_ADC_INPUT,
            APP_VCONN_TRIGGER_LEVEL, PD_ADC_INT_RISING, cbk);
#endif /* CCG4_DOCK */
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */
    return true;
}

void system_vconn_ocp_dis(uint8_t port)
{
#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))

    /* Disable VConn OCP detection in the HAL. */
    pd_hal_vconn_ocp_disable(port);

#else /* !defined (CCG5) || defined(CCG6) || defined(CCG5C) */
#if CCG4_DOCK
    PPDSS_REGS_T gl_pdss[NO_OF_TYPEC_PORTS] =
    {
        PDSS0
#if CCG_PD_DUALPORT_ENABLE
            ,
        PDSS1
#endif /* CCG_PD_DUALPORT_ENABLE */
    };
    PPDSS_REGS_T pd = gl_pdss[port];
    pd->pfet300_ctrl &= ~PDSS_PFET300_CTRL_EN_COMP;
    pd->intr_1_cfg &= ~PDSS_INTR_1_CFG_V5V_CFG_MASK;
    pd->intr1_mask &= ~PDSS_INTR1_MASK_V5V_CHANGED_MASK;
#else /* !CCG4_DOCK */
    /* Disable the ADC used for VConn OCP detection. */
    pd_adc_comparator_ctrl(port, APP_VCONN_OCP_ADC_ID, 0, 0, 0, NULL);
#endif /* CCG4_DOCK */
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */
}

#endif /* VCONN_OCP_ENABLE */

/* End of File */
