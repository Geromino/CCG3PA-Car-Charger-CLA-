/**
 * @file fault_handlers.c
 *
 * @brief @{PD power related fault handling source file.@}
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
#include <alt_mode_hw.h>
#include <alt_modes_mngr.h>
#include <hal_ccgx.h>
#include <gpio.h>
#include <system.h>
#include <utils.h>

#if CCG_HPI_ENABLE
#include <hpi.h>
#endif /* CCG_HPI_ENABLE */

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

enum
{
    FAULT_TYPE_VBUS_OVP = 0,    /* 0 */
    FAULT_TYPE_VBUS_UVP,        /* 1 */
    FAULT_TYPE_VBUS_OCP,        /* 2 */
    FAULT_TYPE_VBUS_SCP,        /* 3 */
    FAULT_TYPE_CC_OVP,          /* 4 */
    FAULT_TYPE_VCONN_OCP,       /* 5 */
    FAULT_TYPE_SBU_OVP,         /* 6 */
    FAULT_TYPE_OTP,             /* 7 */
    FAULT_TYPE_VBUS_RCP,        /* 8 */
    FAULT_TYPE_COUNT            /* 9 */
};

/* Variable defined in app.c */
extern app_status_t app_status[];

#if (VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE | VBUS_UVP_ENABLE | VBUS_RCP_ENABLE | ICL_OCP_ENABLE | VCONN_OCP_ENABLE)
#define FAULT_HANDLER_ENABLE            (1)             /* Shorthand for any faults enabled. */
#endif /* (VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE | VBUS_RCP_ENABLE | VCONN_OCP_ENABLE) */

#if FAULT_HANDLER_ENABLE

/* 
 * If fault retry count in configuration table is set to this value, then
 * faults are not counted. That is, infinite fault recovery is enabled.
 */
#define FAULT_COUNTER_SKIP_VALUE        (255u)

/* Number of retries defined by user for each fault type. */
static uint8_t gl_app_fault_retry_limit[NO_OF_TYPEC_PORTS][FAULT_TYPE_COUNT] =
{
    {0}
};

/* Number of times each fault condition has been detected during current connection. */
static volatile uint8_t gl_app_fault_count[NO_OF_TYPEC_PORTS][FAULT_TYPE_COUNT] =
{
    {0}
};

#if CCG_HPI_AUTO_CMD_ENABLE
/*
 * Byte 0 is dynamic status and Byte 1 is sticky status.
 * Sticky status is set as soon as fault is hit and is maintained until
 * HPI read for fault status is requested. This status is clear on read.
 * Dynamic status is set as long as device is in the fault state.
 * Once the device is out of fault state, status is cleared.
 */
uint16_t gl_app_fault_status = 0;

uint16_t app_retrieve_fault_status(uint8_t port)
{
    (void)port;
    uint16_t status = gl_app_fault_status;
    /* Clear sticky fault status */
    gl_app_fault_status &= 0x00FF; 
    return status;
}
#endif /* CCG_HPI_AUTO_CMD_ENABLE */

/* Check whether any fault count has exceeded limit for the specified PD port. */
bool app_port_fault_count_exceeded(uint8_t port)
{
    uint32_t i;
    bool     retval = false;

    /*
     * Check whether the count for any fault type has exceeded the limit specified.
     */
    for (i = 0; i < FAULT_TYPE_COUNT; i++)
    {
        if (gl_app_fault_count[port][i] > gl_app_fault_retry_limit[port][i])
        {
            retval = true;
            break;
        }
    }

    return (retval);
}

/* This function stops PD operation and configures Type-C to look for detach of faulty device. */
void app_conf_for_faulty_dev_removal(uint8_t port)
{
    const dpm_status_t *dpm_stat = dpm_get_info(port);

    if ((!dpm_stat->attach) || (dpm_stat->cur_port_role == PRT_ROLE_SINK))
    {
        /* Set flag to trigger port disable sequence. */
        app_status[port].fault_status |= APP_PORT_SINK_FAULT_ACTIVE;
    }
    /* In VCONN fault Stop PE is not required */
    if ((app_status[port].fault_status & APP_PORT_VCONN_FAULT_ACTIVE) == 0)
    {
        /* Stop PE */
        dpm_pe_stop(port);
    }

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    /* Make sure any alternate mode related state is cleared. */
    vdm_task_mngr_deinit (port);
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */
}
#if (VBUS_OVP_ENABLE || VBUS_OCP_ENABLE || ICL_RVP_HW || VBUS_SCP_ENABLE || VBUS_RCP_ENABLE || VCONN_OCP_ENABLE)
/* Generic routine that notifies the stack about recovery actions for a fault. */
static void app_handle_fault(uint8_t port, uint32_t fault_type)
{
    uint8_t reason = PD_HARDRES_REASON_VBUS_OVP;

    if (fault_type == FAULT_TYPE_VBUS_OCP)
    {
        reason = PD_HARDRES_REASON_VBUS_OCP;
    }

    /* Not checking for validity of port or fault_type as all calls to this function are internal. */
    dpm_set_fault_active(port);

#if CCG_HPI_AUTO_CMD_ENABLE
    if (FAULT_TYPE_VBUS_OCP == fault_type)
    {
        gl_app_fault_status |= 0x01;
    }
    else if (FAULT_TYPE_VBUS_OVP == fault_type)
    {
        gl_app_fault_status |= 0x02;
    }
    else if (FAULT_TYPE_VBUS_UVP == fault_type)
    {
        gl_app_fault_status |= 0x04;
    }
    else
    {
        /* No code execution */
    }
#endif /* CCG_HPI_AUTO_CMD_ENABLE */

#if ((VBUS_UVP_ENABLE) && (CCG_PPS_SRC_ENABLE))
    /*
     * If current foldback mode is enabled, then we should recover from the
     * failure as the behaviour is expected. But we should still continue to
     * handle the fault with hard reset. So, we do not let the fault count
     * to be incremented.
     */
    if ((fault_type != FAULT_TYPE_VBUS_UVP) || (app_status[port].cur_fb_enabled == false))
#endif /* ((VBUS_UVP_ENABLE) && (CCG_PPS_SRC_ENABLE)) */
    {
#if CCG_REG_SEC_CTRL
        /*
         * If the line voltage is not available, do not register the count.
         * This is due to input removal. We still want to handle the fault
         * to disconnect the sink.
         */
        if (pd_hal_measure_line_volt(port) > PASC_LINE_FF_ON_THRESHOLD)
#endif /* CCG_REG_SEC_CTRL */
        {
            /* Update the fault count. */
            if(gl_app_fault_retry_limit[port][fault_type] == FAULT_COUNTER_SKIP_VALUE)
            {
                /* Do not count faults if infinite fault retry is set. */    
            }
            else
            {
                gl_app_fault_count[port][fault_type]++;
            }
        }
    }

    if (gl_app_fault_count[port][fault_type] < (gl_app_fault_retry_limit[port][fault_type] + 1))
    {
#if VCONN_OCP_ENABLE
        if(fault_type == FAULT_TYPE_VCONN_OCP)
        {
#if (DFP_ALT_MODE_SUPP)
            /* Make sure any alternate mode related state is cleared. */
            vdm_task_mngr_deinit (port);
            app_vdm_layer_reset(port);
#endif /* (DFP_ALT_MODE_SUPP) */
            return;
        }
#endif /* VCONN_OCP_ENABLE */
        
        dpm_clear_hard_reset_count(port);

        /*
         * Try a Hard Reset to recover from fault.
         * If not successful (not in PD contract), try Type-C error recovery.
         */
        if (dpm_send_hard_reset (port, reason) != CCG_STAT_SUCCESS)
        {
            dpm_typec_command(port, DPM_CMD_TYPEC_ERR_RECOVERY, NULL);
        }
#if ICL_RVP_HW
        if(fault_type == FAULT_TYPE_VBUS_OCP)
        {
            OC_FAULT_OUT_Write(1);
        }
#endif /* ICL_RVP_HW */        
    }
    else
    {
#if VCONN_OCP_ENABLE
        if(fault_type == FAULT_TYPE_VCONN_OCP)
        {
            app_status[port].fault_status |= APP_PORT_VCONN_FAULT_ACTIVE;
        }
#endif /* VCONN_OCP_ENABLE */
        app_conf_for_faulty_dev_removal(port);

#if ICL_RVP_HW
        if(fault_type == FAULT_TYPE_VBUS_OCP)
        {
            OC_FAULT_OUT_Write(0);
        }
        gl_app_fault_count[port][fault_type]++;
#endif /* ICL_RVP_HW */        
    }
}

#endif /* VBUS_OVP_ENABLE || VBUS_OCP_ENABLE || ICL_RVP_HW || VBUS_SCP_ENABLE || VBUS_RCP_ENABLE || VCONN_OCP_ENABLE */

/* Timer used to re-enable the PD port after a fault. */
static void fault_recovery_timer_cb(uint8_t port, timer_id_t id)
{
    uint16_t period = APP_FAULT_RECOVERY_TIMER_PERIOD;

    if (
            (vbus_is_present(port, VSAFE_0V, 0) == false)
       )
    {
        if ((app_status[port].fault_status & APP_PORT_VBUS_DROP_WAIT_ACTIVE) != 0)
        {
            app_status[port].fault_status &= ~APP_PORT_VBUS_DROP_WAIT_ACTIVE;

            /* VBus has already been removed. Enable the Rd termination to check for physical detach. */
            pd_typec_rd_enable (port);
            period = APP_FAULT_RECOVERY_MAX_WAIT;
        }
        else
        {
            /*
             * If VBus is not detected, we can re-enable the PD port.
             */
            app_status[port].fault_status &= ~APP_PORT_DISABLE_IN_PROGRESS;
            dpm_clear_fault_active(port);

            pd_typec_dis_rd(port, CC_CHANNEL_1);
            pd_typec_dis_rd(port, CC_CHANNEL_2);
            dpm_start(port);

            /* Return without restarting the timer. */
            return;
        }
    }

    /* Restart the timer to check VBus and Rp status again. */
    timer_start (port, APP_FAULT_RECOVERY_TIMER, period, fault_recovery_timer_cb);
}

/* Callback used to get notification that PD port disable has been completed. */
static void app_port_disable_cb(uint8_t port, dpm_typec_cmd_resp_t resp)
{
    uint16_t period = APP_FAULT_RECOVERY_TIMER_PERIOD;

    if (
            (vbus_is_present(port, VSAFE_0V, 0) == false)
       )
    {
        /* VBus has already been removed. Enable the Rd termination to check for physical detach. */
        pd_typec_rd_enable (port);
        period = APP_FAULT_RECOVERY_MAX_WAIT;
    }
    else
    {
        /* VBus has not been removed. Start a task which waits for VBus removal. */
        app_status[port].fault_status |= APP_PORT_VBUS_DROP_WAIT_ACTIVE;
    }

    /* Provide a delay to allow VBus turn-on by port partner and then enable the port. */
    timer_start (port, APP_FAULT_RECOVERY_TIMER, period, fault_recovery_timer_cb);
}

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
#if (!CCG_SINK_ONLY)
static void src_disable_cbk(uint8_t port)
{
    /* Dummy callback used to ensure VBus discharge happens on CC/SBU OVP. */
}
#endif /* (!CCG_SINK_ONLY) */
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

#endif /* FAULT_HANDLER_ENABLE */

/* Clear all fault counters associated with the specified port. */
void fault_handler_clear_counts (uint8_t port)
{
#if FAULT_HANDLER_ENABLE
    /* Clear all fault counters on disconnect. */
    memset ((uint8_t *)gl_app_fault_count[port], 0, FAULT_TYPE_COUNT);
#endif /* FAULT_HANDLER_ENABLE */
}

/* Fault-handling specific actions to be performed for various event callbacks. */
bool fault_event_handler(uint8_t port, app_evt_t evt, const void *dat)
{
    bool skip_soln_cb = false;

#if FAULT_HANDLER_ENABLE
#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    const dpm_status_t *dpm_stat = dpm_get_info(port);
#endif /* (defined (CCG5) || defined(CCG6) || defined(CCG5C)) */

    switch (evt)
    {
        case APP_EVT_TYPE_C_ERROR_RECOVERY:
            if (app_port_fault_count_exceeded(port))
            {
                break;
            }
            /* Fall-through to below case when fault counts are within limits. */

        case APP_EVT_DISCONNECT:
        case APP_EVT_VBUS_PORT_DISABLE:
        case APP_EVT_HARD_RESET_SENT:
            /* Clear the port-in-fault status. */
            if ((app_status[port].fault_status & APP_PORT_DISABLE_IN_PROGRESS) == 0)
            {
                dpm_clear_fault_active(port);
            }

            if ((evt == APP_EVT_DISCONNECT) || (evt == APP_EVT_VBUS_PORT_DISABLE))
            {
                /* Clear fault counters in cases where an actual disconnect has been detected. */
                fault_handler_clear_counts (port);

#if RIDGE_SLAVE_ENABLE
                /* Clear the error status. */
                ridge_slave_update_ocp_status(port, false);
#endif /* RIDGE_SLAVE_ENABLE */
            }

#if CCG_HPI_AUTO_CMD_ENABLE
            /* Clear only dynamic fault status, that is Byte 0 */
            gl_app_fault_status &= 0xFF00;
#endif /* CCG_HPI_AUTO_CMD_ENABLE */
            break;

        case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE:
#if (CCG_CC_SBU_OVP_RETRY_LIMIT != 0)
            /* Clear the CC/SBU OVP fault count if we have successfully negotiated a PD contract. */
            gl_app_fault_count[port][FAULT_TYPE_CC_OVP]  = 0;
            gl_app_fault_count[port][FAULT_TYPE_SBU_OVP] = 0;
#endif /* (CCG_CC_SBU_OVP_RETRY_LIMIT != 0) */
            break;

#if VBUS_OCP_ENABLE || ICL_RVP_HW
        case APP_EVT_VBUS_OCP_FAULT:
#if CCG_HPI_AUTO_CMD_ENABLE
            gl_app_fault_status |= 0x0100;
#endif /* CCG_HPI_AUTO_CMD_ENABLE */
#if RIDGE_SLAVE_ENABLE
            /* Update the OCP status. */
            ridge_slave_update_ocp_status(port, true);
#endif /* RIDGE_SLAVE_ENABLE */
            app_handle_fault(port, FAULT_TYPE_VBUS_OCP);          
            break;            
#endif /* VBUS_OCP_ENABLE || ICL_RVP_HW */

#if VBUS_SCP_ENABLE
        case APP_EVT_VBUS_SCP_FAULT:
            app_handle_fault(port, FAULT_TYPE_VBUS_SCP);         
            break;
#endif /* VBUS_SCP_ENABLE */

#if VBUS_RCP_ENABLE
        case APP_EVT_VBUS_RCP_FAULT:
            app_handle_fault(port, FAULT_TYPE_VBUS_RCP);         
            break;
#endif /* VBUS_RCP_ENABLE */

#if VBUS_OVP_ENABLE
        case APP_EVT_VBUS_OVP_FAULT:
#if CCG_HPI_AUTO_CMD_ENABLE
            gl_app_fault_status |= 0x0200;
#endif /* CCG_HPI_AUTO_CMD_ENABLE */
            app_handle_fault(port, FAULT_TYPE_VBUS_OVP);          
            break;
#endif /* VBUS_OVP_ENABLE */

#if VBUS_UVP_ENABLE
        case APP_EVT_VBUS_UVP_FAULT:
#if CCG_HPI_AUTO_CMD_ENABLE
            gl_app_fault_status |= 0x0400;
#endif /* CCG_HPI_AUTO_CMD_ENABLE */
            app_handle_fault(port, FAULT_TYPE_VBUS_UVP);          
            break;
#endif /* VBUS_UVP_ENABLE */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
        case APP_EVT_CC_OVP:
        case APP_EVT_SBU_OVP:
            {
                /* Make sure SOURCE/SINK FETs and VConn supply are turned OFF. */
                vconn_disable(port, dpm_stat->rev_pol);
#if (!CCG_SINK_ONLY)
                if ((dpm_stat->attach) && (dpm_stat->cur_port_role == PRT_ROLE_SOURCE))
                {
                    /* Remove the Rp termination and notify the HAL that OVP is pending. */
                    pd_typec_dis_rp(port, dpm_stat->polarity);
                    pd_hal_set_cc_ovp_pending(port);
                    psrc_disable(port, src_disable_cbk);
                }
                else
#endif /* (!CCG_SINK_ONLY) */
                {
#if CCG_HW_DRP_TOGGLE_ENABLE
                    /* Abort auto toggle if enabled. */
                    pd_hal_abort_auto_toggle(port);
#endif /* CCG_HW_DRP_TOGGLE_ENABLE */

#if (!(CCG_SOURCE_ONLY))
                    psnk_disable(port, 0);
#endif /* (!(CCG_SOURCE_ONLY)) */
            }

#if VBUS_OVP_ENABLE
                /* No need to take new action as long as previous fault handling is still pending. */
                if ((app_status[port].fault_status & (APP_PORT_SINK_FAULT_ACTIVE |
                                APP_PORT_DISABLE_IN_PROGRESS)) == 0)
                {
                    app_handle_fault(port, FAULT_TYPE_CC_OVP);
                }
                else
                {
                    skip_soln_cb = true;
                }
#endif /* VBUS_OVP_ENABLE */
            }
            break;
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

#if VCONN_OCP_ENABLE
            case APP_EVT_VCONN_OCP_FAULT:
                app_handle_fault(port, FAULT_TYPE_VCONN_OCP);
                break;
#endif /* VCONN_OCP_ENABLE */

        default:
            break;
    }
#endif /* FAULT_HANDLER_ENABLE */

    return skip_soln_cb;
}

bool fault_handler_init_vars (uint8_t port)
{
#if VBUS_OVP_ENABLE
    if (get_pd_port_config(port)->ovp_tbl_offset == 0)
    {
        return false;
    }

    gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_OVP] = pd_get_ptr_ovp_tbl(port)->retry_cnt;
#endif /* VBUS_OVP_ENABLE */

#if VBUS_OCP_ENABLE || ICL_OCP_ENABLE
    if (get_pd_port_config(port)->ocp_tbl_offset == 0)
    {
        return false;
    }

    gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_OCP] = pd_get_ptr_ocp_tbl(port)->retry_cnt;
#endif /* VBUS_OCP_ENABLE || ICL_OCP_ENABLE */

#if VBUS_RCP_ENABLE
    if (get_pd_port_config(port)->rcp_tbl_offset == 0)
    {
        return false;
    }

    gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_RCP] = pd_get_ptr_rcp_tbl(port)->retry_cnt;
#endif /* VBUS_RCP_ENABLE */

#if VBUS_UVP_ENABLE
    if (get_pd_port_config(port)->uvp_tbl_offset == 0)
    {
        return false;
    }

    gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_UVP] = pd_get_ptr_uvp_tbl(port)->retry_cnt;
#endif /* VBUS_UVP_ENABLE */

#if VBUS_SCP_ENABLE
    if (get_pd_port_config(port)->scp_tbl_offset == 0)
    {
        return false;
    }

    gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_SCP] = pd_get_ptr_scp_tbl(port)->retry_cnt;
#endif /* VBUS_SCP_ENABLE */

#if VCONN_OCP_ENABLE
    if (get_pd_port_config(port)->vconn_ocp_tbl_offset == 0)
    {
        return false;
    }

    gl_app_fault_retry_limit[port][FAULT_TYPE_VCONN_OCP] = pd_get_ptr_vconn_ocp_tbl(port)->retry_cnt;
#endif /* VCONN_OCP_ENABLE */

    return true;
}

void fault_handler_task(uint8_t port)
{
#if FAULT_HANDLER_ENABLE
    /*
     * If SINK fault handling is pending, queue a port disable command.
     */
    if ((app_status[port].fault_status & APP_PORT_SINK_FAULT_ACTIVE) != 0)
    {
        if (dpm_typec_command (port, DPM_CMD_PORT_DISABLE, app_port_disable_cb) != CCG_STAT_BUSY)
        {
            app_status[port].fault_status &= ~APP_PORT_SINK_FAULT_ACTIVE;
            app_status[port].fault_status |= APP_PORT_DISABLE_IN_PROGRESS;
        }
    }
#endif /* FAULT_HANDLER_ENABLE */
}

#if VBUS_OVP_ENABLE

#define MAX_OVP_DEBOUNCE_CYCLES         (0x20u)

/* Configure Over-Voltage Protection checks based on parameters in config table. */
void app_ovp_enable(uint8_t port, uint16_t volt_mV, bool pfet, PD_ADC_CB_T ovp_cb)
{
#if (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC)
    uint8_t level;
    uint8_t threshold;
#else
    uint8_t debounce = 0;
#endif /* (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC) */

    uint8_t intr_state;

    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_OVP_EN_MASK)
    {
        intr_state = CyEnterCriticalSection();

#if (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC)
        threshold = pd_get_ptr_ovp_tbl(port)->threshold;

#if ADC_FALSE_OVP_FIX
        /* Make sure threshold is set to a suitable value at low voltages to avoid false OVP trips. */
        if (apply_threshold(volt_mV, threshold) < ADC_VBUS_MIN_OVP_LEVEL)
        {
            volt_mV   = ADC_VBUS_MIN_OVP_LEVEL;
            threshold = 0;
        }
#endif /* ADC_FALSE_OVP_FIX */

        /* Set OVP threshold. */
        level = pd_get_vbus_adc_level(port, APP_OVP_ADC_ID, volt_mV, threshold);
        pd_adc_comparator_ctrl(port, APP_OVP_ADC_ID, APP_OVP_ADC_INPUT, level, PD_ADC_INT_FALLING, ovp_cb);

#else /* (VBUS_OVP_MODE != VBUS_OVP_MODE_ADC) */

        /* Convert debounce delay in us to filter clock cycles assuming the use of 500 KHz filter clock. */
        debounce = pd_get_ptr_ovp_tbl(port)->debounce;
        debounce = (debounce + 1) / 2;
        debounce = GET_MIN (debounce, MAX_OVP_DEBOUNCE_CYCLES);

#if (defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined (CCG5) ||\
        defined(CCG6) || defined(CCG5C) || defined(PAG1S))
        pd_internal_vbus_ovp_en(port, volt_mV, pd_get_ptr_ovp_tbl(port)->threshold, ovp_cb,
                pfet, VBUS_OVP_MODE, debounce);
#endif /* CCGx */

#endif /* (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC) */

        CyExitCriticalSection(intr_state);
    }
}

void app_ovp_disable(uint8_t port, bool pfet)
{
    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_OVP_EN_MASK)
    {
        /* Disable OVP. */
#if (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC)
        pd_adc_comparator_ctrl(port, APP_OVP_ADC_ID, 0, 0, 0, NULL);
#else
#if (defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined (CCG5) ||\
        defined(CCG6) || defined(CCG5C) || defined(PAG1S))
        pd_internal_vbus_ovp_dis(port, pfet);
#endif /* CCGx */
#endif /* (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC) */
    }
}

#endif /* VBUS_OVP_ENABLE */

#if VBUS_UVP_ENABLE

#define MAX_UVP_DEBOUNCE_CYCLES         (0x20u)

#if CCG4_DOCK
extern void dock_uvp_check (uint8_t port, timer_id_t id);
#endif /* CCG4_DOCK */

/* Configure Under-Voltage Protection checks based on parameters in config table. */
void app_uvp_enable(uint8_t port, uint16_t volt_mV, bool pfet, PD_ADC_CB_T uvp_cb)
{
    uint8_t intr_state;
    uint8_t debounce;

    /* Convert debounce delay in us to filter clock cycles assuming the use of 500 KHz filter clock. */
    debounce = pd_get_ptr_uvp_tbl(port)->debounce;
    debounce = (debounce + 1) / 2;
    debounce = GET_MIN (debounce, MAX_UVP_DEBOUNCE_CYCLES);

    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_UVP_EN_MASK)
    {
        intr_state = CyEnterCriticalSection ();

#if CCG_PPS_SRC_ENABLE
        if (dpm_get_info(port)->src_sel_pdo.fixed_src.supply_type == PDO_AUGMENTED)
        {
            /*
             * In PPS mode operation, UVP is not an unrecoverable event. It
             * needs to be dealt with a simple hardreset. Configure for non-
             * hardware cutoff operation. NOTE: The threshold for operation
             * can be overridden to set the cut-off based on system requirement.
             * Currently using the lowest UVP point for this.
             */
            volt_mV = dpm_get_info(port)->src_sel_pdo.pps_src.min_volt * PD_VOLT_PER_UNIT_PPS;
            pd_internal_vbus_uvp_en (port, volt_mV, pd_get_ptr_uvp_tbl(port)->threshold,
                    uvp_cb, pfet, VBUS_UVP_MODE_INT_COMP, debounce);
        }
        else
#endif /* CCG_PPS_SRC_ENABLE */
        {
#if CCG4_DOCK
            timer_start(port, VBUS_UVP_TIMER_ID, VBUS_UVP_TIMER_PERIOD, dock_uvp_check);
#else /* CCG4_DOCK */
            pd_internal_vbus_uvp_en (port, volt_mV, pd_get_ptr_uvp_tbl(port)->threshold,
                    uvp_cb, pfet, VBUS_UVP_MODE, debounce);
#endif /* CCG4_DOCK */
        }

        CyExitCriticalSection (intr_state);
    }
}

void app_uvp_disable(uint8_t port, bool pfet)
{
    /* Disable UVP. */
    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_UVP_EN_MASK)
    {
#if CCG4_DOCK
        timer_stop(port, VBUS_UVP_TIMER_ID);
#else /* CCG4_DOCK */
        pd_internal_vbus_uvp_dis (port, pfet);
#endif /* CCG4_DOCK */
    }
}

#endif /* VBUS_UVP_ENABLE */

#if OTP_ENABLE

#ifndef CCG6

/* Globals to keep track of OTP condition. */
static uint8_t g1_otp_therm_type[NO_OF_TYPEC_PORTS][THERMISTOR_COUNT] = {{APP_THERMISTOR_TYPE_ERROR}};
static bool gl_otp_port_disable[NO_OF_TYPEC_PORTS] = {false};
/* Global to keep track of thermistors in OTP condition */
static uint8_t gl_therm_in_otp = 0x00;
static uint16_t gl_otp_sys_debounce_count = 0;
static uint16_t gl_therm_debounce_count[THERMISTOR_COUNT] = {0};

static void app_otp_init(uint8_t port)
{
    uint8_t index, active_thermistors = THERMISTOR_COUNT;
    /* Initialization for port specific instance. */
#if (THERMISTOR_COUNT > 1)
    active_thermistors = (THERMISTOR_COUNT - 1) + pd_get_ptr_otp_tbl(port)->therm_1_enable;
#endif /* (THERMISTOR_COUNT > 1) */

    uint8_t therm_type = pd_get_ptr_otp_tbl(port)->therm_type;
    gl_otp_port_disable[port] = false;
    gl_otp_sys_debounce_count = 0;
    gl_therm_in_otp = 0x00;
    for(index = 0; index < active_thermistors; index++)
    {
        gl_therm_debounce_count[index] = 0;
    }

    if(0x00 == therm_type)
    {
        g1_otp_therm_type[port][0] = APP_THERMISTOR_TYPE_NTC;
    }
    else if(0x01 == (therm_type & 0x01))
    {
        g1_otp_therm_type[port][0] = APP_THERMISTOR_TYPE_PTC;
    }
    else if(0x02 == (therm_type & 0x02))
    {
        g1_otp_therm_type[port][0] = APP_THERMISTOR_TYPE_INTRNL;
    }
    else
    {
        g1_otp_therm_type[port][0] = APP_THERMISTOR_TYPE_ERROR;
    }
#if (THERMISTOR_COUNT > 1)

    if(pd_get_ptr_otp_tbl(port)->therm_1_enable)
    {
        therm_type = pd_get_ptr_otp_tbl(port)->therm_type_1;
        if(0x00 == therm_type)
        {
            g1_otp_therm_type[port][1] = APP_THERMISTOR_TYPE_NTC;
        }
        else if(0x01 == (therm_type & 0x01))
        {
            g1_otp_therm_type[port][1] = APP_THERMISTOR_TYPE_PTC;
        }
        else if(0x02 == (therm_type & 0x02))
        {
            g1_otp_therm_type[port][1] = APP_THERMISTOR_TYPE_INTRNL;
        }
        else
        {
            g1_otp_therm_type[port][1] = APP_THERMISTOR_TYPE_ERROR;
        }
    }
#endif /* (THERMISTOR_COUNT > 1) */
}

/* This function will return the current temperature of the system */
static uint16_t app_otp_get_sys_temp(uint8_t port, uint8_t therm_id)
{   
    (void)port;
    uint8_t level;
    uint8_t intr_state;
    uint16_t therm_volt;
    intr_state = CyEnterCriticalSection();
    /* We need to drive GPIO1 high in order to read the thermistor value */
#ifdef PAG1S
    uint8_t gpio = OTP_THERMISTOR_GPIO_0_P1;
#ifdef CY_PINS_THERM_DRIVE_H
    gpio_set_value(GPIO_PORT_0_PIN_1, true);
    if(therm_id == 1)
    {    
        gpio = OTP_THERMISTOR_GPIO_1_P1;
    }
#endif /* CY_PINS_THERM_DRIVE_H */

    /* Configure GPIO. */
    hsiom_set_config(gpio, HSIOM_MODE_AMUXA);
    CyDelayUs(20);
    /* Take ADC sample. */
    pd_adc_select_vref(port,PD_ADC_ID_0,PD_ADC_VREF_VDDD);
    level = pd_adc_sample(port, PD_ADC_ID_0, PD_ADC_INPUT_AMUX_A);
    /* 
     * Using custom level-voltage conversion logic - The entry to the config table for cut-off
     * and restart may have been calculated using a different nominal value for Vddd. We need to scale
     * the Vddd to match the voltage values mentioned via the configuration utility
     */
    therm_volt = ((level * OTP_VDDD_REFERENCE)/PD_ADC_NUM_LEVELS);
    pd_adc_select_vref(port,PD_ADC_ID_0,PD_ADC_VREF_PROG);
    hsiom_set_config(gpio, HSIOM_MODE_GPIO);
#ifdef CY_PINS_THERM_DRIVE_H
    /* We need to drive the GPIO 1 low in order to conserve power */    
    gpio_set_value(GPIO_PORT_0_PIN_1, false);
#endif /* CY_PINS_THERM_DRIVE_H */    

#else /* CCG3PA */ 
    hsiom_set_config(OTP_THERM_GPIO, HSIOM_MODE_AMUXA);
    level = pd_adc_sample (port, PD_ADC_ID_1, PD_ADC_INPUT_AMUX_A);
    therm_volt = pd_adc_level_to_volt (port, PD_ADC_ID_1, level);
#endif /* PAG1S */   
    CyExitCriticalSection(intr_state);
    return therm_volt;
}    

static bool otp_restart_port(uint8_t port, uint16_t therm_volt, uint8_t therm_id)
{
    uint16_t restart_val = pd_get_ptr_otp_tbl(port)->restart_val;
#if (THERMISTOR_COUNT > 1)
    if(therm_id == 0x01)
    {
        restart_val = pd_get_ptr_otp_tbl(port)->restart_val_1;
    }
#endif /* (THERMISTOR_COUNT > 1) */

    return (((g1_otp_therm_type[port][therm_id] == APP_THERMISTOR_TYPE_NTC) &&
                (therm_volt >= restart_val)) ||
            (((g1_otp_therm_type[port][therm_id] == APP_THERMISTOR_TYPE_PTC) &&
              (therm_volt <= restart_val))));
}

static bool otp_is_ot_exist(uint8_t port, uint16_t therm_volt, uint8_t therm_id)
{
    uint16_t cutoff_val = pd_get_ptr_otp_tbl(port)->cutoff_val;
#if (THERMISTOR_COUNT > 1)
    if(therm_id == 0x01)
    {
        cutoff_val = pd_get_ptr_otp_tbl(port)->cutoff_val_1;
    }    
#endif /* (THERMISTOR_COUNT > 1) */

    return ((g1_otp_therm_type[port][therm_id] == APP_THERMISTOR_TYPE_NTC) &&
            (therm_volt <= cutoff_val)) ||
        (((g1_otp_therm_type[port][therm_id] == APP_THERMISTOR_TYPE_PTC) &&
          (therm_volt >= pd_get_ptr_otp_tbl(port)->cutoff_val)));
}

static void otp_debounce_cb(uint8_t port, timer_id_t id)
{
    uint8_t index, active_thermistors = THERMISTOR_COUNT;
    uint16_t therm_volt;
#if (THERMISTOR_COUNT > 1)
    active_thermistors = (THERMISTOR_COUNT - 1) + pd_get_ptr_otp_tbl(port)->therm_1_enable;
#endif /* (THERMISTOR_COUNT > 1) */

    if(gl_otp_port_disable[port] == false)
    {
        for(index = 0; index < active_thermistors; index++)
        {
            therm_volt = app_otp_get_sys_temp(port, index);
            if(true == otp_is_ot_exist(port, therm_volt, index))
            {
                gl_therm_debounce_count[index] ++;
            }
        }
        if ((get_pd_port_config(port)->protection_enable & CFG_TABLE_OTP_EN_MASK))
        {
            gl_otp_sys_debounce_count++;
            if(gl_otp_sys_debounce_count > pd_get_ptr_otp_tbl(port)->debounce)
            {
                for(index = 0; index < active_thermistors; index++)
                {
                    if(gl_therm_debounce_count[index] > pd_get_ptr_otp_tbl(port)->debounce)
                    {
                        /* Here all thermistors which show OTP condition after debounce are set */
                        gl_therm_in_otp |= (0x01 << index);
                    }	 
                }
                if(gl_therm_in_otp != 0x00)
                {
                    dpm_stop(port);
                    /* Clear all the counts here */
                    for(index=0; index < active_thermistors; index++)
                    {
                        gl_therm_debounce_count[index] = 0;
                    }
                    gl_otp_sys_debounce_count = 0;
                    gl_otp_port_disable[port] = true;
                }
            }
            else
            {
                timer_start(port, OTP_DEBOUNCE_TIMER_ID, OTP_DEBOUNCE_PERIOD, otp_debounce_cb);
            } 
        }
    }
    else if(gl_otp_port_disable[port] == true)
    {
        /* We are currently in OTP condition - check if all thermistors are out of OTP */
        for(index = 0; index < active_thermistors; index++)
        {
            therm_volt = app_otp_get_sys_temp(port, index);
            if(true != otp_restart_port(port, therm_volt,index))
            {
                /* If atleast one thermistor still remains in otp - exit */
                break;
            }
            gl_therm_debounce_count[index]++;
        }
        if(index == active_thermistors)
        {
            if ((get_pd_port_config(port)->protection_enable & CFG_TABLE_OTP_EN_MASK))
            {
                gl_otp_sys_debounce_count++;
                if(gl_otp_sys_debounce_count > pd_get_ptr_otp_tbl(port)->debounce)
                {
                    for(index = 0; index < active_thermistors; index++)
                    {
                        if(gl_therm_debounce_count[index] > pd_get_ptr_otp_tbl(port)->debounce)
                        {
                            gl_therm_in_otp &= ~(index+1);
                        }
                    }
                }
                else
                {
                    timer_start(port, OTP_DEBOUNCE_TIMER_ID, OTP_DEBOUNCE_PERIOD, otp_debounce_cb);
                }

                if(gl_therm_in_otp == 0x00)
                {
                    dpm_start(port);
                    gl_otp_port_disable[port] = false;
                    app_otp_enable(port);
                }
            }
            else
            {
                /* OTP feature can get disabled while debouncing is in progress.
                 * There is no point continuing OTP checking in this case.
                 * This is equivalent to no OTP condition.
                 */
                app_otp_init(port);
            }
        }
    }
}

void app_otp_check_temp(uint8_t port)
{
    uint8_t ot_exist = 0;
    uint8_t index, active_thermistors = THERMISTOR_COUNT;
    uint16_t therm_volt;
#if (THERMISTOR_COUNT > 1)
    active_thermistors = (THERMISTOR_COUNT - 1) + pd_get_ptr_otp_tbl(port)->therm_1_enable;
#endif /* (THERMISTOR_COUNT > 1) */

    /* Check if OTP has been enabled in the config table */
    if((get_pd_port_config(port)->protection_enable & CFG_TABLE_OTP_EN_MASK))
    {
        if(gl_otp_port_disable[port] == false)
        {
            /* Check thermistors to see whether any are in OTP */
            for(index = 0; index < active_thermistors; index++)
            {
                therm_volt = app_otp_get_sys_temp(port, index);
                ot_exist |= otp_is_ot_exist(port, therm_volt, index);	
            }
            if(ot_exist != 0)
            {
                /* Initialize all counters to 0 and start debounce */
                for(index=0; index < active_thermistors; index++)
                {
                    gl_therm_debounce_count[index] = 0;
                }
                gl_otp_sys_debounce_count = 0;
                timer_start(port, OTP_DEBOUNCE_TIMER_ID, OTP_DEBOUNCE_PERIOD, otp_debounce_cb);
            }
        }
        else if (gl_otp_port_disable[port] == true)	
        {
            for(index = 0; index < active_thermistors; index++)
            {
                therm_volt = app_otp_get_sys_temp(port, index);
                if(true != otp_restart_port(port, therm_volt, index))
                {
                    break;
                }
            }
            if(index == active_thermistors)
            {
                /* All thermistors are out of OTP condition - start debounce */
                for(index=0; index < active_thermistors; index++)
                {
                    gl_therm_debounce_count[index] = 0;
                }
                gl_otp_sys_debounce_count = 0;
                timer_start(port, OTP_DEBOUNCE_TIMER_ID, OTP_DEBOUNCE_PERIOD, otp_debounce_cb);
            }
        }
    }
}

#else /* CCG6 */

static void ccg6f_port_disable_cb(uint8_t port, dpm_typec_cmd_resp_t resp)
{
    /* Nothing to do here. */
}

/* OT monitor callback function. */
static void ccg6f_get_temp_cbk(uint8_t port, timer_id_t id)
{
    uint8_t vbjt_code;
    app_status_t* app_stat = app_get_status(port);

    /* Get VBJT ADC code */
    vbjt_code = pd_adc_sample (port, PD_ADC_ID_0, PD_ADC_INPUT_BJT);

    /* Check if over-temperature condition. */
    if (
            (vbjt_code <= app_stat->turn_off_temp_limit) &&
            (app_stat->is_hot_shutdown == false)
       )
    {
        /* Turn off the FETs. */
        psrc_disable (port, NULL);
        psnk_disable (port, NULL);

        /* Send OT notifications to the solution. */
        app_event_handler (port, APP_EVT_TEMPERATURE_FAULT, 0);

        /* Set fault active flag and disable the port. */
        app_stat->is_hot_shutdown = true;
        dpm_set_fault_active (port);
        dpm_typec_command (port, DPM_CMD_PORT_DISABLE, ccg6f_port_disable_cb);
    }

    if (
            (app_stat->is_hot_shutdown == true) &&
            (vbjt_code > app_stat->turn_on_temp_limit)
       )
    {
        /* Clear the fault condition and re-enable the port. */
        app_stat->is_hot_shutdown = false;
        dpm_clear_fault_active (port);
        dpm_start (port);
    }

    /* Re-start timer to keep monitoring temperature of IC */
    timer_start(port, APP_OT_DETECTION_TIMER, APP_OT_DETECTION_TIMER_PERIOD, ccg6f_get_temp_cbk);
}

static uint8_t calc_3rd_point(int16_t x, int16_t x0, int16_t x1, int16_t y0, int16_t y1)
{
    int32_t val = 0;
    int32_t y1_y0 = (int32_t)(y1 - y0);
    int32_t x1_x0 = (int32_t)(x1 - x0);
    int32_t x_x0  = (int32_t)(x - x0);

    /* Multiplying by 10 for better accuracy during calculations. */
    val = y1_y0 * x_x0 * 10;
    val = val / x1_x0;
    val = val - 5;
    val = val / 10;
    val = val + y0;

    return ((uint8_t)val);
}

#endif /* CCG6 */

void app_otp_enable(uint8_t port)
{
#ifndef CCG6
    /*
     * If port is < NO_OF_TYPEC_PORTS, individual port specific data structure
     * instances are updated. Port = NO_OF_TYPEC_PORTS is used for updating
     * data structures for all ports at 1 shot. This is used during
     * initialization.
     */
    if (NO_OF_TYPEC_PORTS == port)
    {
        /* This has been called from initialization. We need to do the data
           structure initialization for all port instances. */
        for(port = TYPEC_PORT_0_IDX ; port < NO_OF_TYPEC_PORTS; port++)
        {
            app_otp_init(port);
        }
    }
    else
    {
        /* Initialization for port specific instance. */
        app_otp_init(port);
    }
#else /* CCG6 */

    otp_settings_t *ot_settings = pd_get_ptr_otp_tbl(port);

    uint8_t low_temp_vbjt_code  = *((uint8_t *)APP_OT_VBE_LOW_TEMP_ADDR);
    uint8_t high_temp_vbjt_code = *((uint8_t *)APP_OT_VBE_HIGH_TEMP_ADDR);
    uint8_t room_temp_vbjt_code = *((uint8_t *)APP_OT_VBE_25_C_TEMP_ADDR);

    /* Temporary: Apply default high/low temp values if not found of SFLASH. */
    if ((low_temp_vbjt_code == high_temp_vbjt_code))
    {
        high_temp_vbjt_code = room_temp_vbjt_code - ((APP_OT_HIGH_TEMP - APP_OT_ROOM_TEMP) / VBJT_LSB_TEMP_DELTA);
        low_temp_vbjt_code  = room_temp_vbjt_code + ((APP_OT_ROOM_TEMP - APP_OT_LOW_TEMP) / VBJT_LSB_TEMP_DELTA);
    }

    /* Check OTP parameters in the config table. */
    if (
            ((get_pd_port_config(port)->protection_enable & CFG_TABLE_OTP_EN_MASK) != 0) &&
            (ot_settings->therm_type == APP_THERMISTOR_TYPE_INTRNL)
       )
    {
        /*
         * Calculate hot device shutdown low/high thresholds VBJT code for given shutdown temp using linear dependence
         * between temperature and VBJT code using VBJT code for -40C and 125C which stored in sflash as input.
         */
        app_status[port].turn_off_temp_limit = calc_3rd_point (
                ot_settings->cutoff_val, APP_OT_LOW_TEMP, APP_OT_HIGH_TEMP, low_temp_vbjt_code, high_temp_vbjt_code);
        app_status[port].turn_on_temp_limit = calc_3rd_point (
                ot_settings->restart_val, APP_OT_LOW_TEMP, APP_OT_HIGH_TEMP, low_temp_vbjt_code, high_temp_vbjt_code);

        /* Start timer to monitor temperature of IC */
        app_status[port].is_hot_shutdown = false;
        timer_start(port, APP_OT_DETECTION_TIMER, APP_OT_DETECTION_TIMER_PERIOD, ccg6f_get_temp_cbk);
    }
#endif /* CCG6 */
}

#endif /* OTP_ENABLE */

/* End of File */
