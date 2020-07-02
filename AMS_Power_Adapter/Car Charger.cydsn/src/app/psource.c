/**
 * @file psource.c
 *
 * @brief @{Power source (Provider) manager source file.@}
 */

/*
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
#include <app.h>
#include <timer.h>
#include <hal_ccgx.h>
#include <pdss_hal.h>
#include <gpio.h>
#include <battery_charging.h>
#include <system.h>
#if CCG_CABLE_COMP_ENABLE
#include <cable_comp.h>
#endif /* CCG_CABLE_COMP_ENABLE */

#if (!CCG_SINK_ONLY)
#if (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S))
#include <vbus_ctrl.h>
#endif /* #if (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S)) */

#if ICL_RVP_HW
#include <icl.h>
#endif /* ICL_RVP_HW */

/* Type-C current levels in 10mA units. */
#define CUR_LEVEL_3A    300
#define CUR_LEVEL_1_5A  150
#define CUR_LEVEL_DEF   90

/* VBUS absolute maximum voltage in mV units */
#define VBUS_MAX_VOLTAGE  (21500u)

/* Allowed margin (%) over 5V before the provider FET is turned ON. */
#define PSOURCE_SAFE_FET_ON_MARGIN          (5u)

static void psrc_dis_ovp(uint8_t port);
void psrc_en_ovp(uint8_t port);
void psrc_en_uvp(uint8_t port);
static void psrc_dis_uvp(uint8_t port);
void psrc_en_rcp(uint8_t port);
static void psrc_dis_ocp(uint8_t port);
static void psrc_shutdown(uint8_t port, bool discharge_dis);
#if APP_PSOURCE_SAFE_FET_ON_ENABLE
static void psrc_fet_on_check_cbk(uint8_t port, timer_id_t id);
#endif /* APP_PSOURCE_SAFE_FET_ON_ENABLE */

void app_psrc_tmr_cbk(uint8_t port, timer_id_t id);
void app_psrc_vbus_ovp_cbk(uint8_t port, bool comp_out);
void app_psrc_vbus_ocp_cbk(uint8_t port);
void app_psrc_vbus_scp_cbk(uint8_t port);
void app_psrc_vbus_rcp_cbk(uint8_t port);

void psrc_select_voltage(uint8_t port);

#if NCP_MANAGEMENT
extern void ncp_manage_cbk(uint8_t port, timer_id_t id);
#endif /* NCP_MANAGEMENT */

#if CCG_CABLE_COMP_ENABLE
extern volatile bool gl_cable_comp_start[NO_OF_TYPEC_PORTS];
#endif /* CCG_CABLE_COMP_ENABLE */

static void vbus_fet_on(uint8_t port)
{
#if APP_PSOURCE_SAFE_FET_ON_ENABLE
    uint16_t vbus_in_volt = 0;
    uint16_t threshold = 0;
#endif /* APP_PSOURCE_SAFE_FET_ON_ENABLE */

    /* If fet is already On then no need to enable it again */    
    if(app_get_status(port)->is_vbus_on == false)
    {
#if APP_PSOURCE_SAFE_FET_ON_ENABLE
        threshold = apply_threshold(VSAFE_5V, PSOURCE_SAFE_FET_ON_MARGIN);
        
       /* Don't trun on the FET if Vbus-in voltage is not in safe range. */
        vbus_in_volt = pd_hal_measure_vbus_in(port);
        if(vbus_in_volt > threshold)
        {
            /*
             * Vbus-In is monitored with APP_PSOURCE_SAFE_FET_ON_MONITOR_TIMER_ID.
             * If Vbus-in is not brought into the safe range within
             * APP_PSOURCE_SAFE_FET_ON_TIMER_ID timeout, then a manual OVP is 
             * triggerred.
             */
            vbus_discharge_on(port);
            /* Start timer for Vbus internal discharge timeout */
            timer_start(port, APP_PSOURCE_SAFE_FET_ON_TIMER_ID,
                APP_PSOURCE_SAFE_FET_ON_TIMER_PERIOD, psrc_fet_on_check_cbk);
            /* Start timer to keep track of Vbus internal discharge. */
            timer_start(port, APP_PSOURCE_SAFE_FET_ON_MONITOR_TIMER_ID,
                APP_PSOURCE_SAFE_FET_ON_MONITOR_TIMER_PERIOD, psrc_fet_on_check_cbk);
        }
        else
#endif /* APP_PSOURCE_SAFE_FET_ON_ENABLE */
        {
            app_get_status(port)->is_vbus_on = true;
#if NCP_POWER_SAVE
#if NCP_MANAGEMENT
            timer_start(port,NCP_ENABLE_DELAY_ID,NCP_ENABLE_DELAY_PERIOD,ncp_manage_cbk);
#else
            PD_CTRL_EN(port);
#endif /* NCP_MANAGEMENT */
#endif /* NCP_POWER_SAVE */
            if(port == TYPEC_PORT_0_IDX)
            {
                /*
                 * In case of REGULATOR_REQUIRE_STABLE_ON_TIME, the regulator is
                 * already turned on. POWER_BANK implementation, uses a single
                 * regulator and FET control for both source and sink operation.
                 * Turning OFF sink FET here will cause the regulator to get wrongly
                 * shut down. We should not disable sink here in this case.
                 */
#if (!((REGULATOR_REQUIRE_STABLE_ON_TIME) && (POWER_BANK)))
                APP_VBUS_SNK_FET_OFF_P1();
                CyDelayUs(10);
#endif /* (!((REGULATOR_REQUIRE_STABLE_ON_TIME) && (POWER_BANK))) */
                APP_VBUS_SRC_FET_ON_P1();
            }
#if CCG_PD_DUALPORT_ENABLE
            if(port == TYPEC_PORT_1_IDX)
            {
                APP_VBUS_SNK_FET_OFF_P2();
                CyDelayUs(10);
                APP_VBUS_SRC_FET_ON_P2();
            }
#endif /* CCG_PD_DUALPORT_ENABLE */
        }
    }
}

static void vbus_fet_off(uint8_t port)
{
    app_get_status(port)->is_vbus_on = false;
#if NCP_POWER_SAVE
    PD_CTRL_DIS(port);
#endif /* NCP_POWER_SAVE */
    if(port == TYPEC_PORT_0_IDX)
    {
        APP_VBUS_SRC_FET_OFF_P1();
    }
#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        APP_VBUS_SRC_FET_OFF_P2();
    }
#endif /* CCG_PD_DUALPORT_ENABLE */
}

#endif /* (!CCG_SINK_ONLY) */

void vbus_discharge_on(uint8_t port)
{
    if(port == TYPEC_PORT_0_IDX)
    {
        APP_DISCHARGE_FET_ON_P1();
    }
#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        APP_DISCHARGE_FET_ON_P2();
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

}

void vbus_discharge_off(uint8_t port)
{
    if(port == TYPEC_PORT_0_IDX)
    {
        APP_DISCHARGE_FET_OFF_P1();
    }
#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        APP_DISCHARGE_FET_OFF_P2();
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

}

#if (!CCG_SINK_ONLY)
#if CCG_PPS_SRC_ENABLE

#if VBUS_CF_SOFT_START_ENABLE
uint16_t gl_cf_cur_new;
uint16_t gl_cf_cur_prev = 0;

void vbus_ctrl_timer_isr(void)
{
    uint16_t step = 0;

    if (gl_cf_cur_prev < I_1A)
    {
        gl_cf_cur_prev = I_1A;
    }

    if (gl_cf_cur_new > gl_cf_cur_prev)
    {
        step = gl_cf_cur_new - gl_cf_cur_prev;
        if (step > VBUS_CF_SOFT_START_STEP)
        {
            step = VBUS_CF_SOFT_START_STEP;
        }

        step += gl_cf_cur_prev;
    }
    else
    {
        step = gl_cf_cur_prev - gl_cf_cur_new;
        if (step > VBUS_CF_SOFT_START_STEP)
        {
            step = VBUS_CF_SOFT_START_STEP;
        }

        step = gl_cf_cur_prev - step;
    }

    /* Load the CF setting. */
    pd_cf_enable(0, step);
    gl_cf_cur_prev = step;

    /* Clear the interrupt */
    VBUS_CTRL_TIMER_ClearInterrupt(VBUS_CTRL_TIMER_INTR_MASK_TC);

    /* If there is pending transition, queue an interrupt. */
    if (step != gl_cf_cur_new)
    {
        VBUS_CTRL_TIMER_ISR_StartEx(vbus_ctrl_timer_isr);
        VBUS_CTRL_TIMER_Start();
    }
    else
    {
        VBUS_CTRL_TIMER_Stop();
    }
}
#endif /* VBUS_CF_SOFT_START_ENABLE */

#if CCG_CF_HW_DET_ENABLE
static void psrc_cf_cbk(uint8_t port, bool state)
{
    (void)state;

    if (app_get_status(port)->cur_fb_enabled)
    {
        dpm_pps_task(port);
    }
}
#endif /* CCG_CF_HW_DET_ENABLE */

static void psrc_en_cf(uint8_t port)
{
#if VBUS_CF_EN
    const dpm_status_t *dpm_stat = dpm_get_info(port);

    if (dpm_stat->src_last_rdo.val != dpm_stat->src_cur_rdo.val)
    {
        uint32_t op_cur = dpm_stat->src_cur_rdo.rdo_pps.op_cur * 5; /* In 10mA units */

        /* Minimum supported limit is 1A. */
        if(op_cur < I_1A)
        {
            op_cur = I_1A;
        }

        /*
         * The PDO is power limited, then we should limit the current to the
         * maximum allowed by PDP limit. The PDP information is retrieved from
         * the extended source cap information.
         */
        if (dpm_stat->src_sel_pdo.pps_src.pps_pwr_limited)
        {
            uint32_t limit = (dpm_stat->ext_src_cap[CCG_PD_EXT_SRCCAP_PDP_INDEX] * 1000);

            /*
             * To improve arithematic accuracy, the limit is calculated by
             * (mW * 100) / (mV) to get current in 10mA units.
             */
            limit = ((limit * 100) / app_get_status(port)->psrc_volt);
            op_cur = GET_MIN(op_cur, limit);
        }

#if !VBUS_CF_SOFT_START_ENABLE
        pd_cf_enable(port, op_cur);
#else /* VBUS_CF_SOFT_START_ENABLE */
        /*
         * Do soft start on CF only if we are already in PPS mode. Otherwise do a direct load.
         * Slowly starting CF setting can overloading as the CF setting may not have caught up
         * with the previously set load.
         */
        if (app_get_status(port)->cur_fb_enabled)
        {
            gl_cf_cur_new = op_cur;
            vbus_ctrl_timer_isr();
        }
        else
        {
            gl_cf_cur_prev = op_cur;
            pd_cf_enable(port, op_cur);
        }
#endif /* VBUS_CF_SOFT_START_ENABLE */
        app_get_status(port)->cur_fb_enabled = true;
    }
#endif /* VBUS_CF_EN */
}

static void psrc_dis_cf(uint8_t port)
{
#if VBUS_CF_EN
    app_status_t *app_stat = app_get_status(port);

    if (app_stat->cur_fb_enabled)
    {
        pd_cf_disable(port);
        dpm_set_cf(port, false);
        app_stat->cur_fb_enabled = false;
#if !CCG_CF_HW_DET_ENABLE
        timer_stop(port, APP_PSOURCE_CF_TIMER);
#endif /* !CCG_CF_HW_DET_ENABLE */
#if VBUS_CF_SOFT_START_ENABLE
        VBUS_CTRL_TIMER_Stop();
        gl_cf_cur_prev = 0;
#endif /* VBUS_CF_SOFT_START_ENABLE */
    }
#endif /* VBUS_CF_EN */
}
#endif /* CCG_PPS_SRC_ENABLE */

static void call_psrc_ready_cbk(uint8_t port)
{
    app_status_t* app_stat = app_get_status(port);

    if (app_stat->pwr_ready_cbk != NULL)
    {
        app_stat->pwr_ready_cbk (port);
        app_stat->pwr_ready_cbk = NULL;
    }
#if CCG_CABLE_COMP_ENABLE
    /* 
     * Enable cable compensation comparators for the first time when 
     * PS_RDY is sent.
     * Cable compensation task will take care of re-adjusting the comparators 
     * as and when comparator interrupts are received.
     */
    gl_cable_comp_start[port] = true;
#endif /* CCG_CABLE_COMP_ENABLE */
}

#if APP_PSOURCE_SAFE_FET_ON_ENABLE
static void psrc_fet_on_check_cbk(uint8_t port, timer_id_t id)
{
    uint16_t vbus_in_volt = 0;
    uint16_t threshold = 0;

    /* 
     * If it is Vbus internal discharge timeout and Vbus-in is not within the
     * safe range, manually raise an OVP.
     * If it is not timeout and Vbus-in is not in the range, turn on discharge.
     * If it is not timeout and Vbus-in is in the range, then turn on the FET.
     */
    if(id == APP_PSOURCE_SAFE_FET_ON_TIMER_ID)
    {
        /* Vbus internal discharge timeout */
        /* Make sure that monitor timer is OFF. */
        timer_stop(port, APP_PSOURCE_SAFE_FET_ON_MONITOR_TIMER_ID);
#if VBUS_OVP_ENABLE
        app_psrc_vbus_ovp_cbk(port, true);
#endif
    }
    else if(id == APP_PSOURCE_SAFE_FET_ON_MONITOR_TIMER_ID)
    {
        /* Vbus internal discharge monitor */
        threshold = apply_threshold(VSAFE_5V, pd_get_ptr_ovp_tbl(port)->threshold);
        vbus_in_volt = pd_hal_measure_vbus_in(port);

        if (vbus_in_volt > threshold)
        {
            vbus_discharge_on(port);
            timer_start(port, APP_PSOURCE_SAFE_FET_ON_MONITOR_TIMER_ID, 
                APP_PSOURCE_SAFE_FET_ON_MONITOR_TIMER_PERIOD, psrc_fet_on_check_cbk);
        }
        else
        {
            vbus_discharge_off(port);
            /* Make sure that timeout timer is off. */
            timer_stop(port, APP_PSOURCE_SAFE_FET_ON_TIMER_ID);
            vbus_fet_on(port);
        }
    }
}
#endif /* APP_PSOURCE_SAFE_FET_ON_ENABLE */

/*Timer Callback*/
void app_psrc_tmr_cbk(uint8_t port, timer_id_t id)
{
    app_status_t* app_stat = app_get_status(port);

    switch(id)
    {
        case APP_PSOURCE_EN_TIMER:
            /* Supply did not reach expected level. Turn off power and do error recovery. */
            timer_stop_range(port, APP_PSOURCE_EN_MONITOR_TIMER, APP_PSOURCE_EN_HYS_TIMER);
            app_stat->psrc_volt_old = VSAFE_0V;
            psrc_shutdown(port, true);

#if (VBUS_UVP_ENABLE)
            /*
             *If the VBUS does not reach VSAFE5V, then we need to treat this as an
             * under voltage condition. Since the UVP hardware detection cannot be
             * enabled when turning on the VBUS, this has to be manually triggered
             * from here by invoking the callback directly. Do this only if UVP is
             * enabled from the configuration table.
             */
            if (get_pd_port_config(port)->protection_enable & CFG_TABLE_UVP_EN_MASK)
            {
                app_psrc_vbus_ovp_cbk(port, false);
            }
#endif /* (VBUS_UVP_ENABLE) */
            break;

#if ((!defined(CCG3PA)) && (!defined(CCG3PA2)) && (!defined(PAG1S)))
        case APP_PSOURCE_EN_MONITOR_TIMER:
            if (((app_stat->psrc_rising == true) &&
                        (vbus_is_present(port, app_stat->psrc_volt, VBUS_TURN_ON_MARGIN) == true)) ||
                    ((app_stat->psrc_rising == false) &&
                     (vbus_is_present(port, app_stat->psrc_volt, VBUS_DISCHARGE_MARGIN) == false)))
            {
                /* Start Source enable hysteresis Timer */
                timer_start(port, APP_PSOURCE_EN_HYS_TIMER,
                        APP_PSOURCE_EN_HYS_TIMER_PERIOD, app_psrc_tmr_cbk);
                break;
            }

            /* Start Monitor Timer again */
            timer_start(port, APP_PSOURCE_EN_MONITOR_TIMER,
                    APP_PSOURCE_EN_MONITOR_TIMER_PERIOD, app_psrc_tmr_cbk);
            break;
#elif REGULATOR_REQUIRE_STABLE_ON_TIME
        case APP_PSOURCE_EN_MONITOR_TIMER:
            if (vbus_ctrl_set_is_idle(port))
            {
                /* Start Source enable hysteresis Timer */
                timer_start(port, APP_PSOURCE_EN_HYS_TIMER,
                        APP_PSOURCE_EN_HYS_TIMER_PERIOD, app_psrc_tmr_cbk);            
            }
            break;   
#endif /* ((!defined(CCG3PA)) && (!defined(CCG3PA2)) && (!defined(PAG1S))) */

        case APP_PSOURCE_EN_HYS_TIMER:
#if REGULATOR_REQUIRE_STABLE_ON_TIME
            if (timer_is_running(port, APP_PSOURCE_EN_MONITOR_TIMER))
            {
                return;
            }
#endif /* REGULATOR_REQUIRE_STABLE_ON_TIME */           
            timer_stop(port, APP_PSOURCE_EN_TIMER);
            app_stat->psrc_volt_old = app_stat->psrc_volt;
            vbus_discharge_off(port);

            if(app_stat->psrc_rising == false)
            {
                /* VBus voltage has stabilized at the new lower level. Update the OVP and RCP limits. */
                psrc_en_ovp (port);
                psrc_en_rcp (port);
            }
            else
            {
                psrc_en_uvp (port);
            }

#if CCG_PPS_SRC_ENABLE
            if (app_stat->cur_fb_enabled)
            {
#if !CCG_CF_HW_DET_ENABLE
                timer_start(port, APP_PSOURCE_CF_TIMER,
                        APP_PSOURCE_CF_TIMER_PERIOD, app_psrc_tmr_cbk);
#else /* CCG_CF_HW_DET_ENABLE */
                pd_cf_mon_enable(port, dpm_get_info(port)->cur_fb, psrc_cf_cbk);
#endif /* CCG_CF_HW_DET_ENABLE */
            }
#endif /* CCG_PPS_SRC_ENABLE && !CCG_CF_HW_DET_ENABLE */

#if CCG_SRGDRV_DISABLE_ON_TRANSITION
            pd_srgdrv_enable(port);
#endif /* CCG_SRGDRV_DISABLE_ON_TRANSITION */

            call_psrc_ready_cbk(port);
            break;

        case APP_PSOURCE_DIS_TIMER:
            /* Discharge operation timed out. */
            timer_stop(port, APP_PSOURCE_DIS_MONITOR_TIMER);
            psrc_shutdown(port, true);
            break;

        case APP_PSOURCE_DIS_MONITOR_TIMER:
            if (vbus_is_present(port, VSAFE_5V, VBUS_DISCHARGE_TO_5V_MARGIN) == false)
            {
                /* Voltage has dropped below 5 V. We can now turn off the FET and continue discharge. */
                psrc_shutdown(port, false);
            }

            if (vbus_is_present(port, VSAFE_0V, VBUS_TURN_ON_MARGIN) == false)
            {
                /* Start Extra discharge to allow proper discharge below Vsafe0V */
                timer_start(port, APP_PSOURCE_DIS_EXT_DIS_TIMER,
                        APP_PSOURCE_DIS_EXT_DIS_TIMER_PERIOD,
                        app_psrc_tmr_cbk);
            }
            else
            {
                /* Start Monitor Timer again */
                timer_start(port, APP_PSOURCE_DIS_MONITOR_TIMER,
                        APP_PSOURCE_DIS_MONITOR_TIMER_PERIOD,
                        app_psrc_tmr_cbk);
            }
            break;

        case APP_PSOURCE_DIS_EXT_DIS_TIMER:
            timer_stop(port, APP_PSOURCE_DIS_TIMER);
            vbus_discharge_off(port);

            /* Notify the caller that psrc_disable is complete. */
            call_psrc_ready_cbk(port);
            break;

#if CCG_PPS_SRC_ENABLE && !CCG_CF_HW_DET_ENABLE
        case APP_PSOURCE_CF_TIMER:
            if (app_stat->cur_fb_enabled)
            {
                /*
                 * This task needs to be invoked every 100ms when in PPS mode
                 * operation for the state machine to function correctly. Since
                 * the function expects the VBUS to have stabilized before
                 * calling, it is being called from this timer interrupt handler.
                 */
                dpm_pps_task(port);
                timer_start(port, APP_PSOURCE_CF_TIMER,
                        APP_PSOURCE_CF_TIMER_PERIOD, app_psrc_tmr_cbk);
            }
            break;
#endif /* CCG_PPS_SRC_ENABLE && !CCG_CF_HW_DET_ENABLE */

        default:
            break;
    }
}

#if VBUS_OCP_ENABLE || ICL_RVP_HW

#if HX3PD_DS1_OCP_PIN_WORKAROUND
CY_ISR (pwren_intr_handler)
{
    if(gpio_get_intr(HX3PD_DS1_OCP_PIN))
    {
        CyIntDisable (HX3PD_DS1_OCP_PIN >> 4);
        gpio_clear_intr (HX3PD_DS1_OCP_PIN);
        dpm_clear_fault_active(TYPEC_PORT_1_IDX);
        dpm_start(TYPEC_PORT_1_IDX);
    }
}
#endif /* HX3PD_DS1_OCP_PIN_WORKAROUND */

#if HX3PD_DS2_OCP_PIN_WORKAROUND
CY_ISR (pwren_0_intr_handler)
{
    if(gpio_get_intr(HX3PD_DS2_OCP_PIN))
    {
        CyIntDisable (HX3PD_DS2_OCP_PIN >> 4);
        gpio_clear_intr (HX3PD_DS2_OCP_PIN);
        dpm_clear_fault_active(TYPEC_PORT_0_IDX);
        dpm_start(TYPEC_PORT_0_IDX);
    }
}
#endif /* HX3PD_DS2_OCP_PIN_WORKAROUND */

void app_psrc_vbus_ocp_cbk(uint8_t port)
{
    /* 
     * Stop all psource trasition timers and notify stack about voltage
     * transition complete to process the SCP hard reset sequence. 
     * Also disable all other fault detection by calling psource shutdown.
     */
    timer_stop_range(port, APP_PSOURCE_EN_TIMER, APP_PSOURCE_EN_HYS_TIMER);
    if(port == TYPEC_PORT_0_IDX)
    {
#if HX3PD_DS2_OCP_PIN_WORKAROUND
        if(gpio_read_value(HX3PD_DS2_OCP_PIN))
        {
            /* Configure input from PWREN to handle events. */
            CyIntDisable (HX3PD_DS2_OCP_PIN >> 4);
            gpio_clear_intr (HX3PD_DS2_OCP_PIN);
            gpio_int_set_config (HX3PD_DS2_OCP_PIN, GPIO_INTR_FALLING);
            CyIntSetVector (HX3PD_DS2_OCP_PIN >> 4, pwren_0_intr_handler);
            CyIntEnable (HX3PD_DS2_OCP_PIN >> 4);
        }
#endif /* HX3PD_DS2_OCP_PIN_WORKAROUND */
    }
    else
    {
#if HX3PD_DS1_OCP_PIN_WORKAROUND 
        if(gpio_read_value(HX3PD_DS1_OCP_PIN))
        {
            /* Configure input from PWREN to handle events. */
            CyIntDisable (HX3PD_DS1_OCP_PIN >> 4);
            gpio_clear_intr (HX3PD_DS1_OCP_PIN);
            gpio_int_set_config (HX3PD_DS1_OCP_PIN, GPIO_INTR_FALLING);
            CyIntSetVector (HX3PD_DS1_OCP_PIN >> 4, pwren_intr_handler);
            CyIntEnable (HX3PD_DS1_OCP_PIN >> 4);
        }
#endif /* HX3PD_DS1_OCP_PIN_WORKAROUND */
    }
    call_psrc_ready_cbk(port);

#ifdef CCG6
    /* TODO: See if this is really required. */
    PDSS->pgdo_pu_1_cfg |= PDSS_PGDO_PU_1_CFG_PGDO_PU_IN_LV_OFF_VALUE | PDSS_PGDO_PU_1_CFG_PGDO_PU_EN_LV_OFF_VALUE;
#endif /* CCG6 */

    /* OCP fault. */
    psrc_shutdown(port, true);
    
#if ICL_RVP_HW
    OC_FAULT_OUT_Write(0);
#endif /* ICL_RVP_HW */    

    /* Set alert message */
    pd_do_t alert;
    alert.val = 0;
    alert.ado_alert.ocp = true;
    dpm_set_alert(port, alert);

    /* Enqueue HPI OVP fault event. */
    app_event_handler(port, APP_EVT_VBUS_OCP_FAULT, NULL);

}
#endif /* VBUS_OCP_ENABLE */


#if VBUS_SCP_ENABLE
void app_psrc_vbus_scp_cbk(uint8_t port)
{
    /* 
     * Stop all psource trasition timers and notify stack about voltage
     * transition complete to process the SCP hard reset sequence. 
     * Also disable all other fault detection by calling psource shutdown.
     */ 
    timer_stop_range(port, APP_PSOURCE_EN_TIMER, APP_PSOURCE_EN_HYS_TIMER);
    call_psrc_ready_cbk(port);

    /* SCP fault. */
    psrc_shutdown(port, true);

    /* Set alert message */
    pd_do_t alert;
    alert.val = 0;
    alert.ado_alert.ocp = true;
    dpm_set_alert(port, alert);

    /* Enqueue HPI SCP fault event. */
    app_event_handler(port, APP_EVT_VBUS_SCP_FAULT, NULL);
}
#endif /* VBUS_SCP_ENABLE */

#if VBUS_RCP_ENABLE
void app_psrc_vbus_rcp_cbk(uint8_t port)
{
    pd_do_t alert;

    /* RCP fault. */
    psrc_shutdown(port, true);

    /* Treat RCP as equivalent to OVP and send an alert post fault recovery. */
    alert.val = 0;
    alert.ado_alert.ovp = true;
    dpm_set_alert(port, alert);

    /* Enqueue HPI RCP fault event. */
    app_event_handler(port, APP_EVT_VBUS_RCP_FAULT, NULL);
}
#endif /* VBUS_RCP_ENABLE */

#if ((VBUS_OVP_ENABLE) || (VBUS_UVP_ENABLE))

static void ovp_pwr_ready_cbk(uint8_t port)
{
    /* Dummy callback to allow vbus discharge */
}

void app_psrc_vbus_ovp_cbk(uint8_t port, bool comp_out)
{
    app_status_t *app_stat = app_get_status(port);
    pd_do_t alert;

#if ((VBUS_UVP_ENABLE) && (CCG_PPS_SRC_ENABLE))
    bool delay_shutdown = false;

    /*
     * If this is UVP and we are in PPS mode, we should not increment the
     * fault count. Also, we should be disabling the port before sending the
     * hard reset.
     */
    if ((app_stat->cur_fb_enabled) && (comp_out == false))
    {
        delay_shutdown = true;
    }

    /* Drop source voltage to VSAFE_0V. */
    if (delay_shutdown == false)
#endif /* ((VBUS_UVP_ENABLE) && (CCG_PPS_SRC_ENABLE)) */
    {
        app_stat->psrc_volt = VSAFE_0V;
        psrc_select_voltage(port);

        /*OVP fault*/
        psrc_shutdown(port, true);
    }

#ifndef CCG4
    /* Enqueue HPI fault event */
    if (comp_out == true)
#endif /* CCG4 */
    {
        /* OVP fault condition. */

        /* Set alert message to be sent after fault recovery. */
        alert.val = 0;
        alert.ado_alert.ovp = true;
        dpm_set_alert(port, alert);

        app_event_handler(port, APP_EVT_VBUS_OVP_FAULT, NULL);
        psrc_disable(port, ovp_pwr_ready_cbk);
    }
#ifndef CCG4
    else
    {
        /* UVP fault condition. */

        /* 
         * UVP is a hardware cutoff in micro seconds and OCP is a software
         * debounce and cutoff in milli seconds. When there is a sudden change
         * in Vbus current, Vbus voltage dips and causes UVP to react first
         * rather than OCP. Sink expects an alert message for OCP, that will be
         * missed if UVP is received. Hence, mimic OCP alert in case of UVP as
         * well. This will be taken care only for non-PPS contracts.
         */
        if(dpm_get_info(port)->src_sel_pdo.src_gen.supply_type != PDO_AUGMENTED)
        {
            /* Set alert message */
            pd_do_t alert;
            alert.val = 0;
            alert.ado_alert.ocp = true;
            dpm_set_alert(port, alert);
        }

        app_event_handler(port, APP_EVT_VBUS_UVP_FAULT, NULL);
#if ((VBUS_UVP_ENABLE) && (CCG_PPS_SRC_ENABLE))
        if (delay_shutdown == true)
        {
            psrc_shutdown(port, true);
        }
#endif /* ((VBUS_UVP_ENABLE) && (CCG_PPS_SRC_ENABLE)) */
    }
#endif /* CCG4 */
}
#endif /* ((VBUS_OVP_ENABLE) || (VBUS_UVP_ENABLE)) */

void psrc_select_voltage(uint8_t port)
{
#if (!HIGHER_VOLTAGES_SUPP_DISABLE)
    app_status_t *app_stat = app_get_status(port);
#if VBUS_OFFSET_VOLTAGE_ENABLE
    pwr_params_t *ptr = pd_get_ptr_pwr_tbl(port);
#endif /* VBUS_OFFSET_VOLTAGE_ENABLE */

#if CCG_PROG_SOURCE_ENABLE
    uint32_t select_volt = app_stat->psrc_volt;

    /* Don't drop voltage below 5 V. */
    if (app_stat->psrc_volt == VSAFE_0V)
    {
        app_stat->psrc_volt = VSAFE_5V;
    }

#if VBUS_OFFSET_VOLTAGE_ENABLE
    /*
     * Add Vbus offset voltage.
     * Vbus offset is not applicable for PPS supply type.
     */
    if(dpm_get_info(port)->src_sel_pdo.src_gen.supply_type != PDO_AUGMENTED)
    {
        select_volt += ptr->vbus_offset_volt ;
    }
#endif /* VBUS_OFFSET_VOLTAGE_ENABLE */
    
#if CCG_CABLE_COMP_ENABLE
    if (cable_comp_is_enabled(port) == true)
    {
        select_volt += cable_comp_get_voltage(port);
    }
#endif /* CCG_CABLE_COMP_ENABLE */

    /* 
     * Cap the selected voltage to the absolute maximum voltage that can be
     * applied to the cable. 
     */ 
    if (select_volt > VBUS_MAX_VOLTAGE)
    {
        select_volt = VBUS_MAX_VOLTAGE;
    }

    if(port == TYPEC_PORT_0_IDX)
    {
#if NCP_MANAGEMENT
        timer_start(port,NCP_VOLTCHANGE_DELAY_ID,NCP_VOLTCHANGE_DELAY_PERIOD,ncp_manage_cbk);
#else
        APP_VBUS_SET_VOLT_P1(select_volt);
#endif /* NCP_MANAGEMENT */
    }
#if CCG_PD_DUALPORT_ENABLE
    else
    {
        APP_VBUS_SET_VOLT_P2(select_volt);
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

#else /* CCG_PROG_SOURCE_ENABLE */

    uint8_t intr_state = CyEnterCriticalSection();

    if (port == TYPEC_PORT_0_IDX)
    {
        switch (app_stat->psrc_volt)
        {
            case VSAFE_9V:
                APP_VBUS_SET_9V_P1();
                break;
            case VSAFE_12V:
                APP_VBUS_SET_12V_P1();
                break;
            case VSAFE_13V:
                APP_VBUS_SET_13V_P1();
                break;
            case VSAFE_15V:
                APP_VBUS_SET_15V_P1();
                break;
            case VSAFE_19V:
                APP_VBUS_SET_19V_P1();
                break;
            case VSAFE_20V:
                APP_VBUS_SET_20V_P1();
                break;
            default:
                app_stat->psrc_volt = VSAFE_5V;
                APP_VBUS_SET_5V_P1();
                break;
        }
    }
#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        switch (app_stat->psrc_volt)
        {
            case VSAFE_9V:
                APP_VBUS_SET_9V_P2();
                break;
            case VSAFE_12V:
                APP_VBUS_SET_12V_P2();
                break;
            case VSAFE_13V:
                APP_VBUS_SET_13V_P2();
                break;
            case VSAFE_15V:
                APP_VBUS_SET_15V_P2();
                break;
            case VSAFE_19V:
                APP_VBUS_SET_19V_P2();
                break;
            case VSAFE_20V:
                APP_VBUS_SET_20V_P2();
                break;
            default:
                app_stat->psrc_volt = VSAFE_5V;
                APP_VBUS_SET_5V_P2();
                break;
        }
    }
#endif /* CCG_PD_DUALPORT_ENABLE */
    CyExitCriticalSection(intr_state);
#endif  /* CCG_PROG_SOURCE_ENABLE */
#endif /* HIGHER_VOLTAGES_SUPP */
}

#if (APP_VBUS_LOAD_SWITCH_ENABLE)
void app_psrc_ld_sw_cbk(uint8_t port, bool state)
{
    uint16_t volt_mV = app_get_status(port)->psrc_volt;

    /* We need to ensure that we are still active to filter out race conditions. */
    if ((volt_mV < APP_VBUS_LD_SW_VOLT_MIN) && (volt_mV > VSAFE_0V))
    {
        /*
         * The load is above the threshold. Disable the load switch and configure
         * SR comparator to trigger on falling edge.
         */
        if (state)
        {
            APP_PSRC_LD_SW_OFF(port);
#if (!CCG_CABLE_COMP_ENABLE)
            pd_set_sr_comp(port, (APP_VBUS_LD_SW_CUR_MIN - APP_VBUS_LD_SW_CUR_THRES),
                    FILTER_CFG_POS_DIS_NEG_EN, app_psrc_ld_sw_cbk);
#endif /* (!CCG_CABLE_COMP_ENABLE) */
        }
        else
        {
            APP_PSRC_LD_SW_ON(port);
#if (!CCG_CABLE_COMP_ENABLE)
            pd_set_sr_comp(port, (APP_VBUS_LD_SW_CUR_MIN + APP_VBUS_LD_SW_CUR_THRES),
                    FILTER_CFG_POS_EN_NEG_DIS, app_psrc_ld_sw_cbk);
#endif /* (!CCG_CABLE_COMP_ENABLE) */
        }
    }
    else
    {
#if (!CCG_CABLE_COMP_ENABLE)
        pd_stop_sr_comp(port);
#endif /* (!CCG_CABLE_COMP_ENABLE) */
        APP_PSRC_LD_SW_OFF(port);
        app_get_status(port)->ld_sw_ctrl = false;
    }
}

void psrc_ld_sw_ctrl(uint8_t port, uint16_t volt_mV)
{
    uint8_t intr_state;
    app_status_t *app_stat = app_get_status(port);

    intr_state = CyEnterCriticalSection();
    /*
     * Turn ON the load switch if the requested voltage and current are below
     * the threshold specified. Since this voltage is always expected to be
     * below 5V, it is possible only when PPS is active.
     */
    if ((volt_mV < APP_VBUS_LD_SW_VOLT_MIN) && (volt_mV > VSAFE_0V))
    {
        /*
         * If the load switch is already active, then do nothing. If not, configure
         * the SR comparator to interrupt when the current goes below the threshold.
         */
        if (app_stat->ld_sw_ctrl == false)
        {
#if (!CCG_CABLE_COMP_ENABLE)
            pd_set_sr_comp(port, (APP_VBUS_LD_SW_CUR_MIN - APP_VBUS_LD_SW_CUR_THRES),
                    FILTER_CFG_POS_DIS_NEG_EN, app_psrc_ld_sw_cbk);
#else
            /* 
             * Use Vbus current change from cable compensation callback.
             * And enable load switch to add initial load on the Vbus.
             */
            APP_PSRC_LD_SW_ON(port);
#endif /* (!CCG_CABLE_COMP_ENABLE) */
            app_stat->ld_sw_ctrl = true;
        }
    }
    else
    {
        /*
         * Disable the SR comparator and disable the load switch.
         */
#if (!CCG_CABLE_COMP_ENABLE)
        pd_stop_sr_comp(port);
#endif /* (!CCG_CABLE_COMP_ENABLE) */
        APP_PSRC_LD_SW_OFF(port);
        app_stat->ld_sw_ctrl = false;
    }

    CyExitCriticalSection(intr_state);
}
#endif /* APP_VBUS_LOAD_SWITCH_ENABLE */

#if (CCG_CABLE_COMP_ENABLE)
/* 
 * This function will be called on every CABLE_COMP_HYS change in Vbus current.
 * vbus_cur is in 10mA units.
 */
void app_psrc_vbus_cur_cbk(uint8_t port, uint16_t vbus_cur)
{
#if APP_VBUS_LOAD_SWITCH_ENABLE 
    app_status_t *app_stat = app_get_status(port);

    if (app_stat->ld_sw_ctrl == true)
    {
        if(vbus_cur > (APP_VBUS_LD_SW_CUR_MIN + APP_VBUS_LD_SW_CUR_THRES))
        {
            app_psrc_ld_sw_cbk(port, true);
        }
        else if(vbus_cur < (APP_VBUS_LD_SW_CUR_MIN - APP_VBUS_LD_SW_CUR_THRES))
        {
            app_psrc_ld_sw_cbk(port, false);
        }
    }
#endif /* APP_VBUS_LOAD_SWITCH_ENABLE */
}
#endif /* (CCG_CABLE_COMP_ENABLE) */

void psrc_set_voltage(uint8_t port, uint16_t volt_mV)
{
    app_status_t *app_stat = app_get_status(port);
    const dpm_status_t *dpm_stat = dpm_get_info(port);

    app_stat->psrc_volt = volt_mV;
        
#if CCG_TYPE_A_PORT_ENABLE
    if (port == TYPE_A_PORT_ID)
    {
        APP_VBUS_SET_VOLT_P2 (volt_mV);
        return;
    }
#endif /* CCG_TYPE_A_PORT_ENABLE */

#if CCG_SRGDRV_DISABLE_ON_TRANSITION
    pd_srgdrv_disable(port);
#endif /* CCG_SRGDRV_DISABLE_ON_TRANSITION */

#if (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S))
    /* Ensure that the feedback control logic is turned on. */
    vbus_ctrl_fb_enable(port);
#endif /* (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S)) */

#if CCG_CABLE_COMP_ENABLE
    cable_comp_enable(port, app_psrc_vbus_cur_cbk);
    cable_comp_cfg(port, volt_mV);
#endif /* CCG_CABLE_COMP_ENABLE */

#if CCG_PPS_SRC_ENABLE

#if CCG_CF_HW_DET_ENABLE
    pd_cf_mon_disable(port);
#endif /* CCG_CF_HW_DET_ENABLE */

    if (dpm_get_info(port)->src_sel_pdo.fixed_src.supply_type == PDO_AUGMENTED)
    {
#if (CCG_CABLE_COMP_ENABLE) && (CCG_CABLE_COMP_IN_PPS_DISABLE)
        cable_comp_disable(port);
#endif /* (CCG_CABLE_COMP_ENABLE) && (CCG_CABLE_COMP_IN_PPS_DISABLE) */
        psrc_en_cf(port);
    }
    else
    {
        psrc_dis_cf(port);
    }
#endif /* CCG_PPS_SRC_ENABLE */

    /* Setup the load switch control. */
#if APP_VBUS_LOAD_SWITCH_ENABLE
    psrc_ld_sw_ctrl(port, volt_mV);
#endif /* APP_VBUS_LOAD_SWITCH_ENABLE */

#if VBUS_OCP_ENABLE
    /* Leave OCP detection disabled while doing the voltage transition. */
    psrc_dis_ocp (port);
#endif /* VBUS_OCP_ENABLE */

    if ((app_stat->psrc_volt >= app_stat->psrc_volt_old) && (volt_mV != VSAFE_0V))
    {
        /* If voltage is being increased, make sure that OVP and RCP limits are moved up. */
        psrc_en_ovp (port);
        psrc_en_rcp (port);
    }
    else if ((app_stat->psrc_volt < app_stat->psrc_volt_old) && (volt_mV != VSAFE_0V))
    {
        /*
         * Enable UVP only if port partner is attached. We need to ensure that
         * UVP does not get enabled if VBUS is not applied, like in case of
         * HARD_RESET.
         */
        if ((dpm_stat->attach == true) && (app_stat->is_vbus_on == true))
        {
            psrc_en_uvp (port);
        }
    }

    psrc_select_voltage(port);
}

uint32_t psrc_get_voltage (uint8_t port)
{
#if CCG_CABLE_COMP_ENABLE
    return app_get_status(port)->psrc_volt + cable_comp_get_voltage(port);
#else /* !CCG_CABLE_COMP_ENABLE */
    return app_get_status(port)->psrc_volt;
#endif /* CCG_CABLE_COMP_ENABLE */
}

#if (VBUS_OCP_ENABLE || VBUS_SCP_ENABLE)
static const uint32_t cc_rp_to_cur_map[] = {
    CUR_LEVEL_DEF,
    CUR_LEVEL_1_5A,
    CUR_LEVEL_3A
};
#endif /* (VBUS_OCP_ENABLE || VBUS_SCP_ENABLE) */

void psrc_set_current (uint8_t port, uint16_t cur_10mA)
{
#if (VBUS_OCP_ENABLE || VBUS_SCP_ENABLE)
    /* Update the OCP/SCP thresholds when required. */
    const dpm_status_t *dpm_stat = dpm_get_info(port);
    uint32_t ocp_cur;

#if (VBUS_CF_EN && VBUS_OCP_ENABLE)
    app_status_t* app_stat = app_get_status(port);
#endif /* VBUS_CF_EN */

    if (dpm_stat->contract_exist)
    {
#if CCG_PPS_SRC_ENABLE
        if (dpm_stat->src_sel_pdo.src_gen.supply_type == PDO_AUGMENTED)
        {
            /* Convert in 10mA units */
            ocp_cur = dpm_stat->src_sel_pdo.pps_src.max_cur * 5;
        }
        else
#endif /* CCG_PPS_SRC_ENABLE */
        {
            ocp_cur = dpm_stat->src_sel_pdo.src_gen.max_cur_power;
        }
    }
    else
    {
        ocp_cur = cc_rp_to_cur_map[dpm_stat->src_cur_level];

#if BATTERY_CHARGING_ENABLE
        /* Update current limit as per battery charging module */
        if (
                (bc_get_status(port)->cur_mode != BC_CHARGE_NONE) &&
                (bc_get_status(port)->cur_mode != BC_CHARGE_CDP)
           )
        {
            ocp_cur = bc_get_status(port)->cur_amp;
        }
#endif /* BATTERY_CHARGING_ENABLE */
    }

#if VBUS_OCP_ENABLE
    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_OCP_EN_MASK)
    {
#if VBUS_CF_EN
        /* Enable OCP only if not in current foldback mode. */
        if (app_stat->cur_fb_enabled == false)
#endif /* VBUS_CF_EN */
        {
            system_vbus_ocp_en(port, ocp_cur, app_psrc_vbus_ocp_cbk);
        }
    }
#endif /* VBUS_OCP_ENABLE */

#if VBUS_SCP_ENABLE
    /* Enable SCP. */
    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_SCP_EN_MASK)
    {
        system_vbus_scp_en (port, ocp_cur, app_psrc_vbus_scp_cbk);
    }
#endif /* VBUS_SCP_ENABLE */

#endif /* (VBUS_OCP_ENABLE || VBUS_SCP_ENABLE) */
}

void psrc_enable (uint8_t port, pwr_ready_cbk_t pwr_ready_handler)
{
#if CCG_TYPE_A_PORT_ENABLE
    if (port == TYPE_A_PORT_ID)
    {
        /* Turn on PSource FET for TYPE-A port. */
        vbus_fet_on(port);
        return;
    }
#endif /* CCG_TYPE_A_PORT_ENABLE */
 
    app_status_t* app_stat = app_get_status(port);
    const dpm_status_t *dpm_stat = dpm_get_info(port);
    uint8_t intr_state;

#if PROCHOT_SUPP
    if(!ADP_DETECT_Read())
        PROCHOT_Write(0);
    CyDelayUs(PROCHOT_TURN_ON_DELAY);
#endif /* PROCHOT_SUPP */

    intr_state = CyEnterCriticalSection();

    timer_stop_range(port, APP_PSOURCE_EN_TIMER, APP_PSOURCE_DIS_EXT_DIS_TIMER);

    /* Turn on FETs only if dpm is enabled and there is no active fault condition. */
    if ((dpm_stat->dpm_enabled) && (dpm_stat->fault_active == false))
    {
#if (VBUS_OCP_ENABLE || VBUS_SCP_ENABLE)
        /* Make sure OCP/SCP is enabled where required. The current parameter is not used. */
        psrc_set_current (port, CUR_LEVEL_3A);
#endif /* (VBUS_OCP_ENABLE || VBUS_SCP_ENABLE) */

#if ICL_RVP_HW && ICL_OCP_ENABLE
        /* Set the callback to be triggered when load switch indicates fault */
        icl_ocp_cfg(port, true, app_psrc_vbus_ocp_cbk);
#endif /* ICL_RVP_HW */

#if REGULATOR_REQUIRE_STABLE_ON_TIME  
    if(REGULATOR_STATUS() == false)
    {
        timer_start(port, APP_PSOURCE_EN_MONITOR_TIMER, REGULATOR_TURN_ON_DELAY, app_psrc_tmr_cbk);
    }
#endif /* REGULATOR_REQUIRE_STABLE_ON_TIME */    

        /* Turn off VBus Discharge by default. */
        vbus_discharge_off(port);

        /* Turn on PSource FET */
        vbus_fet_on(port);

        if (pwr_ready_handler != NULL)
        {
            app_stat->psrc_rising = true;

            /* If the VBus voltage is dropping, turn the discharge path on. */
            if(app_stat->psrc_volt_old > app_stat->psrc_volt)
            {
                app_stat->psrc_rising = false;
                vbus_discharge_on(port);
            }
            app_stat->pwr_ready_cbk = pwr_ready_handler;

            /* Start Power source enable and monitor timer */
            timer_start(port, APP_PSOURCE_EN_TIMER,
                    APP_PSOURCE_EN_TIMER_PERIOD, app_psrc_tmr_cbk);
            /* For CCG3PA/CCG3PA2/PAG1S APP_PSOURCE_EN_MONITOR_TIMER is not required */
#if ((!defined(CCG3PA)) && (!defined(CCG3PA2)) && (!defined(PAG1S)))
            timer_start(port, APP_PSOURCE_EN_MONITOR_TIMER,
                    APP_PSOURCE_EN_MONITOR_TIMER_PERIOD, app_psrc_tmr_cbk);
#endif /* ((!defined(CCG3PA)) && (!defined(CCG3PA2)) && (!defined(PAG1S))) */
        }
    }

    CyExitCriticalSection(intr_state);
}

void psrc_disable(uint8_t port, pwr_ready_cbk_t pwr_ready_handler)
{
    app_status_t* app_stat = app_get_status(port);
    uint8_t intr_state;

#if CCG_TYPE_A_PORT_ENABLE
    if (port == TYPE_A_PORT_ID)
    {
        /* Turn off PSource FET for TYPE-A port. */
        vbus_fet_off(port);
        return;
    }
#endif /* CCG_TYPE_A_PORT_ENABLE */

#if PROCHOT_SUPP
    if(!ADP_DETECT_Read())
        PROCHOT_Write(0);
#endif /* PROCHOT_SUPP */

#if ICL_RVP_HW && ICL_OCP_ENABLE
    /* Disable OCP */
    icl_ocp_cfg(port, false, app_psrc_vbus_ocp_cbk);
#endif /* ICL_RVP_HW */

    intr_state = CyEnterCriticalSection();

    timer_stop_range(port, APP_PSOURCE_EN_TIMER, APP_PSOURCE_DIS_EXT_DIS_TIMER);
#if CCG_PPS_SRC_ENABLE
    psrc_dis_cf(port);
#endif /* CCG_PPS_SRC_ENABLE */

#if VBUS_UVP_ENABLE
    psrc_dis_uvp(port);
#endif /* VBUS_UVP_ENABLE */

    if((app_stat->psrc_volt_old <= VSAFE_5V)
#if ((defined(CCG3PA)) || (defined(CCG3PA2)) || (defined(PAG1S)))
        && (false == vbus_is_present(port, VSAFE_5V, 5))
#endif /* ((!defined(CCG3PA)) && (!defined(CCG3PA2)) && (!defined(PAG1S))) */
        )
    {
        psrc_shutdown(port, false);
        CyDelayUs(20);
    }
    else
    {
#if ((defined(CCG3PA)) || (defined(CCG3PA2)) || (defined(PAG1S)))
        /*
         * If we are in the middle of transition, then we cannot
         * use the psrc_volt_old variable as the actual voltage
         * may be different. To avoid false UV/OV trips, the better
         * option is to use the actual VBUS voltage instead.
         */
        if (app_stat->psrc_volt != app_stat->psrc_volt_old)
        {
            app_stat->psrc_volt_old = vbus_get_value(port);
        }
#endif /* ((defined(CCG3PA)) || (defined(CCG3PA2)) || (defined(PAG1S))) */
        psrc_set_voltage(port, VSAFE_5V);
    }

    app_stat->psrc_volt_old = VSAFE_0V;

    if ((pwr_ready_handler != NULL) && (dpm_get_info(port)->dpm_enabled))
    {
        /* Turn on discharge to get the voltage to drop faster. */
        vbus_discharge_on(port);
        app_stat->pwr_ready_cbk = pwr_ready_handler;

        /*Start Power source enable and monitor timer*/
        timer_start(port, APP_PSOURCE_DIS_TIMER,
                APP_PSOURCE_DIS_TIMER_PERIOD, app_psrc_tmr_cbk);
        timer_start(port, APP_PSOURCE_DIS_MONITOR_TIMER,
                APP_PSOURCE_DIS_MONITOR_TIMER_PERIOD, app_psrc_tmr_cbk);
    }
    else
    {
        psrc_shutdown(port, true);
    }

    CyExitCriticalSection(intr_state);
}

static void psrc_dis_ovp(uint8_t port)
{
#if VBUS_OVP_ENABLE
#if (defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined (CCG5) || defined(CCG6) || defined(CCG5C) || defined(PAG1S))
    app_ovp_disable (port, CCG_SRC_FET);
#else /* CCG4 */
    app_ovp_disable (port, false);
#endif /* CCGx */
#endif /* VBUS_OVP_ENABLE */
}

static void psrc_dis_ocp(uint8_t port)
{
#if VBUS_OCP_ENABLE
    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_OCP_EN_MASK)
    {
        system_vbus_ocp_dis(port);
    }
#endif /* VBUS_OCP_ENABLE */
}

static void psrc_dis_scp(uint8_t port)
{
#if VBUS_SCP_ENABLE
    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_SCP_EN_MASK)
    {
        system_vbus_scp_dis (port);
    }
#endif /* VBUS_SCP_ENABLE */
}

static void psrc_dis_uvp(uint8_t port)
{
#if VBUS_UVP_ENABLE
#if CCG4_DOCK
    app_uvp_disable (port, true);
#else /* !CCG4_DOCK */
    app_uvp_disable (port, CCG_SRC_FET);
#endif /* CCG4_DOCK */
#endif /* VBUS_UVP_ENABLE */
}

static void psrc_dis_rcp(uint8_t port)
{
#if VBUS_RCP_ENABLE
    system_vbus_rcp_dis(port);
#endif /* VBUS_RCP_ENABLE */
}

static void psrc_shutdown(uint8_t port, bool discharge_dis)
{
    /*Turn Off Source FET*/
    vbus_fet_off(port);

    if(discharge_dis == true)
    {
        vbus_discharge_off(port);
    }

#if CCG_CABLE_COMP_ENABLE
    cable_comp_disable(port);
#endif /* CCG_CABLE_COMP_ENABLE */

#if (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S))
    /* Ensure that the feedback control logic is turned off. */
    vbus_ctrl_fb_disable(port);
#endif /* (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S)) */

#if (APP_VBUS_LOAD_SWITCH_ENABLE)
    /* Ensure that the load switch is disabled. */
    psrc_ld_sw_ctrl(port, VSAFE_5V);
#endif /* (APP_VBUS_LOAD_SWITCH_ENABLE) */

    /* Disable OVP/OCP/UVP */
    psrc_dis_ovp(port);
    psrc_dis_uvp(port);
    psrc_dis_ocp(port);
    psrc_dis_scp(port);
    psrc_dis_rcp(port);

#if CCG_PPS_SRC_ENABLE
    psrc_dis_cf(port);
#endif /* CCG_PPS_SRC_ENABLE */
}

void psrc_en_ovp(uint8_t port)
{
#if (VBUS_OVP_ENABLE)
#if (defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5) || defined(CCG5C) || defined(CCG6) || defined(PAG1S))
    app_ovp_enable(port, app_get_status(port)->psrc_volt,
            CCG_SRC_FET, app_psrc_vbus_ovp_cbk);
#else /* CCG4 */
    app_ovp_enable(port, app_get_status(port)->psrc_volt,
            true, app_psrc_vbus_ovp_cbk);
#endif /* CCGx */
#endif /* (VBUS_OVP_ENABLE) */
}

void psrc_en_rcp(uint8_t port)
{
#if VBUS_RCP_ENABLE
    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_RCP_EN_MASK)
    {
        system_vbus_rcp_en (port, app_psrc_vbus_rcp_cbk);
    }
#endif /* VBUS_RCP_ENABLE */
}

void psrc_en_uvp(uint8_t port)
{
#if VBUS_UVP_ENABLE
    /* Using the same callback as OVP as behavior is same. */
#if CCG4_DOCK
    app_uvp_enable(port, app_get_status(port)->psrc_volt, true, app_psrc_vbus_ovp_cbk);
#else
    app_uvp_enable(port, app_get_status(port)->psrc_volt, CCG_SRC_FET, app_psrc_vbus_ovp_cbk);
#endif /* CCG4_DOCK */
#endif /* VBUS_UVP_ENABLE */
}

#endif /* (!CCG_SINK_ONLY) */

/* End of File */
