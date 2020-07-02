/**
 * @file battery_charging.c
 *
 * @brief  @{Battery Charging source file.@}
 *
 *******************************************************************************
 * @copyright
 *
 * Copyright (2014-2019), Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All rights reserved.
 *
 * This software, including source code, documentation and related materials
 * (â€œSoftwareâ€), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries (â€œCypressâ€) and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software (â€œEULAâ€).
 *
 * If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypressâ€™s integrated circuit
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
 * significant property damage, injury or death (â€œHigh Risk Productâ€). By
 * including Cypressâ€™s product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 */
#include <project.h>
#include <config.h>
#include <dpm.h>
#include <chgb_hal.h>
#include <psource.h>
#include <psink.h>
#include <timer.h>
#include <app.h>
#include <utils.h>
#include <battery_charging.h>

#if CCG_TYPE_A_PORT_ENABLE
#include <type_a.h>
#endif /* CCG_TYPE_A_PORT_ENABLE */

#if ((defined CCG5) || (defined CCG5C) || (defined CCG6))

#if CCG_HPI_ENABLE
#include <hpi.h>
#endif /* CCG_HPI_ENABLE */

#include <system.h>

#endif /* ((defined CCG5) || (defined CCG5C) || (defined CCG6)) */

#if BATTERY_CHARGING_ENABLE

/**
 * @typedef bc_fsm_evt_t
 * @brief Enum to hold BC events id
 */
typedef enum
{
    BC_FSM_EVT_ENTRY = 0,           /*  0: BC Event: State entry. */
    BC_FSM_EVT_CMP1_FIRE,           /*  1: BC Event: CMP1 interrupt. */
    BC_FSM_EVT_CMP2_FIRE,           /*  2: BC Event: CMP2 interrupt. */
    BC_FSM_EVT_QC_CHANGE,           /*  3: BC Event: QC state change. */
    BC_FSM_EVT_QC_CONT,             /*  4: BC Event: QC continuous mode entry. */
    BC_FSM_EVT_AFC_RESET_RCVD,      /*  5: BC Event: AFC reset received. */
    BC_FSM_EVT_AFC_MSG_RCVD,        /*  6: BC Event: AFC message received. */
    BC_FSM_EVT_AFC_MSG_SENT,        /*  7: BC Event: AFC message sent. */
    BC_FSM_EVT_AFC_MSG_SEND_FAIL,   /*  8: BC Event: AFC message sending failed. */
    BC_FSM_EVT_TIMEOUT1,            /*  9: BC Event: Timer1 expiry interrupt. */
    BC_FSM_EVT_TIMEOUT2,            /* 10: BC Event: Timer2 expiry interrupt. */
    BC_FSM_EVT_DISCONNECT,          /* 11: BC Event: Device disconnect. */
    BC_FSM_MAX_EVTS                 /* 12: Number of events. */
}bc_fsm_evt_t;

#ifndef CCG5C
static const uint16_t apple_id_to_cur_map[] = {
    APPLE_AMP_1A,
    APPLE_AMP_2_1A,
    APPLE_AMP_2_4A
};
#endif /* CCG5C */

/* Battery Charger Configuration structure*/
bc_status_t gl_bc_status[NO_OF_BC_PORTS];
#if LEGACY_DYN_CFG_ENABLE
static chg_cfg_params_t gl_bc_cfg[NO_OF_BC_PORTS];
#endif /* LEGACY_DYN_CFG_ENABLE */

/* Function prototypes*/

/* Callback from BC Phy */
static void bc_phy_cbk_handler(uint8_t cport, uint32_t event);

/* Callback from timer module */
static void bc_tmr_cbk(uint8_t cport, timer_id_t id);

/* Callback from power module */
static void bc_pwr_ready_cbk(uint8_t cport);

/* BC FSM prototypes */
static void bc_fsm_off(uint8_t cport, bc_fsm_evt_t evt);

/* Hnadlers for SRC mode operation. */
static void bc_fsm_src_look_for_connect(uint8_t cport, bc_fsm_evt_t evt);
static void bc_fsm_src_initial_connect(uint8_t cport, bc_fsm_evt_t evt);
static void bc_fsm_src_apple_connected(uint8_t cport, bc_fsm_evt_t evt);

#if (defined (CCG5C) || defined (CCG6))
/* BC 1.2 CDP can only be supported on CCG5C and CCG6. */
static void bc_fsm_src_apply_cdp(uint8_t cport, bc_fsm_evt_t evt);
bool vdm_src_applied[NO_OF_BC_PORTS];
uint8_t vdm_src_lv_cnt[NO_OF_BC_PORTS];
bool detach_detect_en[NO_OF_BC_PORTS];
#endif /* (defined (CCG5C) || defined (CCG6)) */

#if (!QC_AFC_CHARGING_DISABLED)
static void bc_fsm_src_others_connected(uint8_t cport, bc_fsm_evt_t evt);
static void bc_fsm_src_qc_or_afc(uint8_t cport, bc_fsm_evt_t evt);
static void bc_fsm_src_qc_connected(uint8_t cport, bc_fsm_evt_t evt);
static void bc_fsm_src_afc_connected(uint8_t cport, bc_fsm_evt_t evt);
#endif /* (!QC_AFC_CHARGING_DISABLED) */

/* Handlers for Sink mode operation. */
#if (!(CCG_SOURCE_ONLY)) && (!BC_SOURCE_ONLY)
static void bc_fsm_sink_start(uint8_t cport, bc_fsm_evt_t evt);
static void bc_fsm_sink_apple_charger_detect(uint8_t c_port, bc_fsm_evt_t evt);
static void bc_fsm_sink_apple_brick_id_detect(uint8_t c_port, bc_fsm_evt_t evt);
static void bc_fsm_sink_primary_charger_detect(uint8_t c_port, bc_fsm_evt_t evt);
static void bc_fsm_sink_type_c_only_source_connected(uint8_t c_port, bc_fsm_evt_t evt);
static void bc_fsm_sink_secondary_charger_detect(uint8_t c_port, bc_fsm_evt_t evt);
static void bc_fsm_sink_dcp_connected(uint8_t port, bc_fsm_evt_t evt);
static void bc_fsm_sink_sdp_connected(uint8_t port, bc_fsm_evt_t evt);
static void bc_fsm_sink_cdp_connected(uint8_t port, bc_fsm_evt_t evt);
#endif /* (!(CCG_SOURCE_ONLY)) && (!BC_SOURCE_ONLY) */

void (*const bc_fsm_table [BC_FSM_MAX_STATES]) (uint8_t cport, bc_fsm_evt_t evt) =
{
    bc_fsm_off,                                 /*  0: BC_FSM_OFF */
    bc_fsm_src_look_for_connect,                /*  1: BC_FSM_SRC_LOOK_FOR_CONNECT */
    bc_fsm_src_initial_connect,                 /*  2: BC_FSM_SRC_INITIAL_CONNECT */
    bc_fsm_src_apple_connected,                 /*  3: BC_FSM_SRC_APPLE_CONNECTED */

#if (defined CCG5C) || (defined CCG6)
    bc_fsm_src_apply_cdp,                       /*  4: BC_FSM_SRC_CDP_CONNECTED */
#else
    bc_fsm_off,                                 /*  4: BC_FSM_SRC_CDP_CONNECTED */
#endif /* (defined CCG5C) || (defined CCG6) */

#if (!QC_AFC_CHARGING_DISABLED)
    bc_fsm_src_others_connected,                /*  5: BC_FSM_SRC_OTHERS_CONNECTED */
    bc_fsm_src_qc_or_afc,                       /*  6: BC_FSM_SRC_QC_OR_AFC */
    bc_fsm_src_qc_connected,                    /*  7: BC_FSM_SRC_QC_CONNECTED */
    bc_fsm_src_afc_connected,                   /*  8: BC_FSM_SRC_AFC_CONNECTED */
#else /* QC_AFC_CHARGING_DISABLED */
    bc_fsm_off,                                 /*  5: BC_FSM_SRC_OTHERS_CONNECTED */
    bc_fsm_off,                                 /*  6: BC_FSM_SRC_QC_OR_AFC */
    bc_fsm_off,                                 /*  7: BC_FSM_SRC_QC_CONNECTED */
    bc_fsm_off,                                 /*  8: BC_FSM_SRC_AFC_CONNECTED */
#endif /* (!QC_AFC_CHARGING_DISABLED) */

#if (!(CCG_SOURCE_ONLY)) && (!BC_SOURCE_ONLY)
    bc_fsm_sink_start,                          /*  9: BC_FSM_SINK_START */
    bc_fsm_sink_apple_charger_detect,           /* 10: BC_FSM_SINK_APPLE_CHARGER_DETECT */
    bc_fsm_sink_apple_brick_id_detect,          /* 11: BC_FSM_SINK_APPLE_BRICK_ID_DETECT */
    bc_fsm_sink_primary_charger_detect,         /* 12: BC_FSM_SINK_PRIMARY_CHARGER_DETECT */
    bc_fsm_sink_type_c_only_source_connected,   /* 13: BC_FSM_SINK_TYPE_C_ONLY_SOURCE_CONNECTED */
    bc_fsm_sink_secondary_charger_detect,       /* 14: BC_FSM_SINK_SECONDARY_CHARGER_DETECT */
    bc_fsm_sink_dcp_connected,                  /* 15: BC_FSM_SINK_DCP_CONNECTED */
    bc_fsm_sink_sdp_connected,                  /* 16: BC_FSM_SINK_SDP_CONNECTED */
    bc_fsm_sink_cdp_connected                   /* 17: BC_FSM_SINK_CDP_CONNECTED */
#endif /* (!(CCG_SOURCE_ONLY)) && (!BC_SOURCE_ONLY) */
};

static void bc_phy_cbk_handler(uint8_t cport, uint32_t event)
{
    bc_set_bc_evt(cport, event);
}

static void bc_tmr_cbk(uint8_t cport, timer_id_t id)
{
    if(id == APP_BC_GENERIC_TIMER1)
    {
        bc_set_bc_evt(cport, BC_EVT_TIMEOUT1);
    }
    else if(id == APP_BC_GENERIC_TIMER2)
    {
        bc_set_bc_evt(cport, BC_EVT_TIMEOUT2);
    }
}

static void bc_pwr_ready_cbk(uint8_t cport)
{
    /* Do nothing */
}

ccg_status_t bc_init(uint8_t cport)
{
    bc_status_t *bc_stat = &gl_bc_status[cport];

    chgb_init(cport, bc_phy_cbk_handler);
    bc_stat->bc_fsm_state = BC_FSM_OFF;

#if LEGACY_DYN_CFG_ENABLE
    /* Copy the configuration from the configuration table. */
    memcpy((uint8_t *)(&gl_bc_cfg[cport]), (uint8_t *)pd_get_ptr_chg_cfg_tbl(cport),
            sizeof (chg_cfg_params_t));
#endif /* LEGACY_DYN_CFG_ENABLE */

    return CCG_STAT_SUCCESS;
}

const chg_cfg_params_t *bc_get_config(uint8_t cport)
{
#if LEGACY_DYN_CFG_ENABLE
    return (const chg_cfg_params_t *)&gl_bc_cfg[cport];
#else /* !LEGACY_DYN_CFG_ENABLE */
    return pd_get_ptr_chg_cfg_tbl(cport);
#endif /* LEGACY_DYN_CFG_ENABLE */
}

bool bc_is_active(uint8_t cport)
{
    bc_status_t *bc_stat = &gl_bc_status[cport];

    if (bc_stat->bc_fsm_state == BC_FSM_OFF)
    {
        return false;
    }

    return true;
}

#if LEGACY_DYN_CFG_ENABLE
ccg_status_t bc_set_config(uint8_t cport, chg_cfg_params_t *chg_cfg)
{
    bc_status_t *bc_stat = &gl_bc_status[cport];

    if ((chg_cfg != NULL) && (bc_stat->bc_fsm_state == BC_FSM_OFF))
    {
        memcpy((uint8_t *)(&gl_bc_cfg[cport]), (uint8_t *)chg_cfg,
                sizeof (chg_cfg_params_t));

        return CCG_STAT_SUCCESS;
    }
    
    return CCG_STAT_FAILURE;
}
#endif /* LEGACY_DYN_CFG_ENABLE */

#if (!QC_AFC_CHARGING_DISABLED)
static uint8_t bc_afc_src_get_vi_count(uint8_t cport)
{
    return bc_get_config(cport)->afc_src_cap_cnt;
}

static uint8_t* bc_afc_src_get_vi_ptr(uint8_t cport)
{
    return (uint8_t *)bc_get_config(cport)->afc_src_caps;
}

#if (QC_CF_EN || LEGACY_DYN_CFG_ENABLE)    
uint8_t ccg_get_system_max_pdp(uint8_t cport)
{
    /* Function returns the max PDP value of the system 
     * This information will be used by the current limit function
     */
    uint8_t sys_max_pdp = CCG_MAX_PDP_VALUE;
    if(get_pd_port_config(cport)->ext_src_cap_length != 0)
    {
        /* If PD3.0 is supported, the PDP value will be read from the extended source cap */        
        const dpm_status_t *dpm_stat = dpm_get_info(cport);
        sys_max_pdp = dpm_stat->ext_src_cap[CCG_PD_EXT_SRCCAP_PDP_INDEX];
    }
    else
    {
        /* If not PD3.0 we will use the PDP value configured */    
    }
    return sys_max_pdp;
}
#endif /* (QC_CF_EN || LEGACY_DYN_CFG_ENABLE) */

#if LEGACY_DYN_CFG_ENABLE
    
/* This function forms the new VI code based on the port PDP */
void bc_afc_form_vi(uint8_t cport)
{
    uint8_t new_vi;
    uint8_t index;
    uint8_t port_pdp = ccg_get_system_max_pdp(cport);
    uint32_t new_current;

    for(index = 0; index < gl_bc_cfg[cport].afc_src_cap_cnt; index++)
    {
        new_current = GET_MIN(((port_pdp * 1000)/((AFC_BASE_VOLT/AFC_VOLT_STEP) + BYTE_GET_UPPER_NIBBLE(gl_bc_cfg[cport].afc_src_caps[index]))),
                                            (LEGACY_MAX_CABLE_RATING * 10));
        /* Convert current to 10mA units */
        new_current = new_current/10;
        if(new_current < AFC_BASE_AMP)
        {
            new_current = AFC_BASE_AMP;
        }
        new_vi = GET_MIN(((new_current - AFC_BASE_AMP)/AFC_AMP_STEP), AFC_MAX_AMP);
        gl_bc_cfg[cport].afc_src_caps[index] = ((gl_bc_cfg[cport].afc_src_caps[index] & 0xF0) | (new_vi & 0x0F));
    }
}
#endif /* LEGACY_DYN_CFG_ENABLE */

#endif /* (!QC_AFC_CHARGING_DISABLED) */

ccg_status_t bc_start(uint8_t cport, bc_port_role_t port_role)
{
#if ((defined CCG3PA) || (defined CCG3PA2) || (defined CCG6) || defined(PAG1S))
    bc_status_t *bc_stat = &gl_bc_status[cport];
    const chg_cfg_params_t *chg_cfg = bc_get_config(cport);
#if (((defined(CCG3PA) || (defined(CCG3PA2)))) && (CCG_CDP_EN))  
    /* For CDP only mode of operation, we will check if BC1.2 has been enabled in configuration table */
    if((chg_cfg->src_sel & BC_SRC_1_2_MODE_ENABLE_MASK) != 0)
    {
        app_bc_12_sm_start(cport);
    }
    return CCG_STAT_SUCCESS;
#endif /* (((defined(CCG3PA) || (defined(CCG3PA2)))) && (CCG_CDP_EN)) */

    if (((chg_cfg->src_sel & BATT_CHARGING_SRC_MASK) == 0) &&
            ((chg_cfg->snk_sel & BATT_CHARGING_SINK_MASK) == 0))
    {
        return CCG_STAT_SUCCESS;
    }

    bc_stat->connected = false;
    bc_stat->cur_mode = BC_CHARGE_NONE;

#if (!(CCG_SOURCE_ONLY))
    if (port_role == BC_PORT_SOURCE)
#endif /* (!(CCG_SOURCE_ONLY)) */
    {
        /* Start only if we are source and BC is not disabled */
#if (defined(CCG6))
        if (
                (dpm_get_info(cport)->cur_port_role == PRT_ROLE_SOURCE)
                && (!app_get_status(cport)->bc_12_src_disabled)
           )
#endif /* (defined(CCG6)) */
        {
#if ((defined (CCG6)) && (CCG_HPI_ENABLE))
            if (hpi_get_sys_pwr_state () == NB_SYS_PWR_STATE_S0)
            {
                bc_stat->bc_fsm_state = BC_FSM_SRC_CDP_CONNECTED;
            }
            else
#endif /* ((defined (CCG6)) && (CCG_HPI_ENABLE)) */
            {
                bc_stat->bc_fsm_state = BC_FSM_SRC_LOOK_FOR_CONNECT;
            }

            bc_stat->bc_evt = BC_EVT_ENTRY;

        }
    }
#if (!(CCG_SOURCE_ONLY))
    else
    {
        /* Move to start state for sink mode operation. */
        bc_stat->bc_fsm_state = BC_FSM_SINK_START;
        bc_stat->bc_evt = BC_EVT_ENTRY;
    }
#endif /* (!(CCG_SOURCE_ONLY)) */

    return CCG_STAT_SUCCESS;

#elif (defined(CCG5C) || defined(CCG5))
    /* Start only if we are source and BC is not disabled */
    if (
            (dpm_get_info(cport)->cur_port_role == PRT_ROLE_SOURCE) &&
            (!app_get_status(cport)->bc_12_src_disabled)
       )
    {
#ifdef CCG5C
        bc_status_t *bc_stat = &gl_bc_status[cport];
        const chg_cfg_params_t *chg_cfg = bc_get_config(cport);

        if ((chg_cfg->src_sel & BATT_CHARGING_SRC_MASK) == 0)
        {
            return CCG_STAT_SUCCESS;
        }

        bc_stat->connected = false;
        bc_stat->cur_mode = BC_CHARGE_NONE;

#if CCG_HPI_ENABLE
        if (hpi_get_sys_pwr_state () == NB_SYS_PWR_STATE_S0)
        {
            bc_stat->bc_fsm_state = BC_FSM_SRC_CDP_CONNECTED;
        }
        else
#endif /* CCG_HPI_ENABLE */
        {
            bc_stat->bc_fsm_state = BC_FSM_SRC_LOOK_FOR_CONNECT;
        }
        bc_stat->bc_evt = BC_EVT_ENTRY;
#else /* !CCG5C */
        app_bc_12_sm_start(cport);
#endif /* CCG5C */
    }
    return CCG_STAT_SUCCESS;
#else
    return CCG_STAT_SUCCESS;
#endif /* CCGx */
}

ccg_status_t bc_stop(uint8_t cport)
{
#if (defined(CCG3PA)) || (defined(CCG3PA2)) || (defined(CCG6) || defined(PAG1S))

#if (((defined(CCG3PA) || (defined(CCG3PA2)))) && (CCG_CDP_EN))
    ccg_bc_dis(cport);
    return CCG_STAT_SUCCESS;    
#endif /* (((defined(CCG3PA) || (defined(CCG3PA2)))) && (CCG_CDP_EN)) */    

    bc_status_t* bc_stat = &gl_bc_status[cport];

    /* Do nothing if we are already off. */
    if (bc_stat->bc_fsm_state == BC_FSM_OFF)
    {
        return CCG_STAT_SUCCESS;
    }

#if (LEGACY_APPLE_SRC_SLN_TERM_ENABLE)
    /* Disable external Apple source termination. */
    sln_remove_apple_src_term(cport);
#endif /* (LEGACY_APPLE_SRC_SLN_TERM_ENABLE) */

    chgb_stop_comp(cport, BC_CMP_0_IDX);
    timer_stop_range(cport, APP_BC_GENERIC_TIMER1, APP_BC_DP_DM_DEBOUNCE_TIMER);
    chgb_disable(cport);
    bc_stat->bc_fsm_state = BC_FSM_OFF;
    bc_stat->bc_evt = 0;
    bc_stat->connected = false;
    bc_stat->attach = false;
    bc_stat->cur_mode = BC_CHARGE_NONE;

#ifdef CCG6
    detach_detect_en[cport] = false;
#endif /* CCG6 */

    /* Clear sink specific flags and states. */
    bc_stat->cur_volt = VSAFE_0V;

#if ((defined(CCG6)) && (!CCG_SOURCE_ONLY) && (!BC_SOURCE_ONLY))
    if (dpm_get_info(cport)->cur_port_role == PRT_ROLE_SINK)
    {
        /* If mode is sink and there is no PD contract, ensure current limit is set to minimum. */
        if (!dpm_get_info(cport)->contract_exist)
        {
            psnk_set_current(cport, ISAFE_0A);
        }

        /* Enable DP/DM Mux if we are sink and still attached. */
        if (dpm_get_info(cport)->attach)
        {
            ccg_config_dp_dm_mux (cport,
                    dpm_get_info(cport)->polarity ? DPDM_MUX_CONN_USB_BOT_UART : DPDM_MUX_CONN_USB_TOP_UART);
        }
    }
#endif /* ((defined(CCG6)) && (!CCG_SOURCE_ONLY) && (!BC_SOURCE_ONLY)) */

    return CCG_STAT_SUCCESS;

#elif (defined(CCG5C))
    bc_status_t* bc_stat = &gl_bc_status[cport];

    chgb_stop_comp(cport, BC_CMP_0_IDX);
    timer_stop_range(cport, APP_BC_GENERIC_TIMER1, APP_BC_DP_DM_DEBOUNCE_TIMER);
    chgb_disable(cport);
    bc_stat->bc_fsm_state = BC_FSM_OFF;
    bc_stat->bc_evt = 0;
    bc_stat->connected = false;
    bc_stat->attach = false;
    bc_stat->cur_mode = BC_CHARGE_NONE;
    detach_detect_en[cport] = false;

    /* Clear sink specific flags and states. */
    bc_stat->cur_volt = VSAFE_0V;

    /* If mode is sink, ensure current limit is set to minimum. */
#if (!(CCG_SOURCE_ONLY)) && (!BC_SOURCE_ONLY)
    if (dpm_get_info(cport)->cur_port_role == PRT_ROLE_SINK)
    {
        psnk_set_current(cport, ISAFE_0A);
    }
#endif /* (!(CCG_SOURCE_ONLY)) && (!BC_SOURCE_ONLY) */

    return CCG_STAT_SUCCESS;

#elif (defined(CCG5))
    ccg_bc_dis(cport);
    return CCG_STAT_SUCCESS;

#else
    return CCG_STAT_SUCCESS;
#endif /* CCGx */
}

bool bc_port_is_cdp(uint8_t cport)
{
    bool ret = false;

#ifdef CCG5
    if (ccg_bc_is_cdp(cport))
    {
        ret = true;
    }
#endif /* CCG5 */

#if ((defined(CCG5C)) || (defined(CCG6)))
    if (gl_bc_status[cport].bc_fsm_state == BC_FSM_SRC_CDP_CONNECTED)
    {
        ret = true;
    }
#endif /* ((defined(CCG5C)) || (defined(CCG6))) */

    return ret;
}

#if (!QC_AFC_CHARGING_DISABLED)
void bc_debounce(uint8_t cport)
{
    uint32_t i;
    bc_dp_dm_state_t new_state;
    chgb_comp_pinput_t pinput = CHGB_COMP_P_DP;
    bc_status_t *bc_stat = &gl_bc_status[cport];
    const chg_cfg_params_t *chg_cfg = bc_get_config(cport);

    new_state.state = QC_MODE_RSVD;

    /* Get current status */
    for(i = 0 ; i < 2 ; i++)
    {
        if(i == 1)
        {
            pinput = CHGB_COMP_P_DM;
        }

        if (chgb_set_comp(cport, BC_CMP_0_IDX, pinput, CHGB_COMP_N_VREF,
                          CHGB_VREF_0_425V, CHGB_COMP_NO_INTR) == false)
        {
            new_state.d[i] = BC_D_GND;
        }
        else if(chgb_set_comp(cport, BC_CMP_0_IDX, pinput, CHGB_COMP_N_VREF,
                          CHGB_VREF_2V, CHGB_COMP_NO_INTR) == false)
        {
            new_state.d[i] = BC_D_0_6V;
        }
        else
        {
            new_state.d[i] = BC_D_3_3V;
        }
    }

    /* Do debounce */
    if(bc_stat->dp_dm_status.state == new_state.state)
    {
        bc_stat->old_dp_dm_status.state = bc_stat->dp_dm_status.state;
        timer_stop(cport, APP_BC_DP_DM_DEBOUNCE_TIMER);
        return;
    }

    if(bc_stat->old_dp_dm_status.state != new_state.state)
    {
        /*
         * Do not debounce DP/DM state if current mode is QC Continous mode and new state
         * translates to non-5V Fixed mode. Only transition out of continous mode is either
         * 5V fixed mode or disconnect.
         */
        if ((bc_stat->dp_dm_status.state != QC_MODE_CONT) || (new_state.state == QC_MODE_5V) ||
            (new_state.state == 0))
        {
            timer_start(cport, APP_BC_DP_DM_DEBOUNCE_TIMER, APP_BC_DP_DM_DEBOUNCE_TIMER_PERIOD, NULL);
            bc_stat->old_dp_dm_status.state = new_state.state;
            return;
        }
        if(bc_stat->dp_dm_status.state == QC_MODE_CONT)
        {
            return;
        }
    }

    if(timer_is_running(cport, APP_BC_DP_DM_DEBOUNCE_TIMER) == false)
    {
        bc_stat->dp_dm_status.state = bc_stat->old_dp_dm_status.state;
        /*
         * If both DP and DM are Hi-Z, generate CMP1_FIRE interrupt which translates
         * to device disconnect event. Otherwise, generate QC Mode change interrupt.
         */
        if (bc_stat->dp_dm_status.state == 0)
        {
            bc_set_bc_evt(cport, BC_EVT_DISCONNECT);
        }
        else
        {
            /* Proceed with QC state change only if QC is enabled. */
            if (chg_cfg->src_sel & BC_SRC_QC_MODE_ENABLE_MASK)
            {
                bc_set_bc_evt(cport, BC_EVT_QC_CHANGE);
            }
        }
    }
}
#endif /* (!QC_AFC_CHARGING_DISABLED) */

ccg_status_t bc_fsm(uint8_t cport)
{
#if (defined CCG5)
    /* Run the CDP state machine where required. */
    ccg_bc_cdp_sm(cport);
#else

    bc_status_t* bc_stat = &gl_bc_status[cport];
    uint8_t evt;
    
#if (defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S))

#if (((defined(CCG3PA) || (defined(CCG3PA2)))) && (CCG_CDP_EN))
    ccg_bc_cdp_sm(cport);
    return CCG_STAT_SUCCESS;
#endif /* (((defined(CCG3PA) || (defined(CCG3PA2)))) && (CCG_CDP_EN)) */

    /* Execute state machine only if the port is active. */
    if(cport == TYPEC_PORT_0_IDX)
    {
        if (dpm_get_info(cport)->connect == false)
        {
            bc_stop(cport);
            return CCG_STAT_SUCCESS;
        }
    }
#if (CCG_TYPE_A_PORT_ENABLE == 1)
    else
    {
        if (type_a_get_status()->type_a_enabled == false)
        {
            return CCG_STAT_SUCCESS;
        }
    }
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (!QC_AFC_CHARGING_DISABLED)
    if((bc_stat->bc_fsm_state == BC_FSM_SRC_QC_OR_AFC) || (bc_stat->bc_fsm_state == BC_FSM_SRC_QC_CONNECTED)
        || (bc_stat->bc_fsm_state == BC_FSM_SRC_AFC_CONNECTED))
    {
        bc_debounce(cport);
    }
#endif /* (!QC_AFC_CHARGING_DISABLED) */

    /* Get bc event */
    evt = event_group_get_event((uint32_t *)&(bc_stat->bc_evt), true);

    /* Check if any valid event pending, if not return */
    if(evt < BC_FSM_MAX_EVTS)
    {
        bc_fsm_table[bc_stat->bc_fsm_state] (cport, evt);
    }
#endif /* (defined CCG5) */

    return CCG_STAT_SUCCESS;
}

bool bc_sleep(void)
{
#if (defined CCG3PA) || (defined(CCG3PA2) || defined(PAG1S))

#if (((defined(CCG3PA) || (defined(CCG3PA2)))) && (CCG_CDP_EN))

    uint8_t port;
    for(port = 0; port < NO_OF_BC_PORTS; port++)
    if (ccg_is_cdp_sm_busy (port))
    {
            return false;
    }
    
#else
    uint8_t i;
    for(i = 0; i < NO_OF_BC_PORTS; i++)
    {
        bc_status_t* bc_stat = &gl_bc_status[i];

        if ((bc_stat->bc_evt != 0)  ||
            (timer_is_running(i, APP_BC_GENERIC_TIMER1) == true) ||
            (timer_is_running(i, APP_BC_GENERIC_TIMER2) == true) ||
            (timer_is_running(i, APP_BC_DP_DM_DEBOUNCE_TIMER) == true)
           ) 
        {
            return false;
        }

#if (!QC_AFC_CHARGING_DISABLED)
        if (bc_stat->bc_fsm_state == BC_FSM_SRC_QC_CONNECTED)
        {
            /* Enable master DP/DM sense */
            chgb_qc_src_master_sense_enable(i);
        }
#endif /* (!QC_AFC_CHARGING_DISABLED) */

        /* Configure QC RCVR interrupt as wakeup source from DP/DM activity. */
        chgb_deepsleep(i);

#if (!QC_AFC_CHARGING_DISABLED)
        /* 
         * Check the DP/DM status before going to sleep.
         * This is to make sure DP/DM debounce edges are not missed between
         * bc_fsm() and bc_sleep().
         */
        if(((chgb_dp_status(i) == true) ^ (bc_stat->dp_dm_status.d[0] == BC_D_3_3V)) ||
           ((chgb_dm_status(i) == true) ^ (bc_stat->dp_dm_status.d[1] == BC_D_3_3V)))
        {
            return false;
        }
#endif /* (!QC_AFC_CHARGING_DISABLED) */
    }

    for(i = 0; i < NO_OF_BC_PORTS; i++)
    {
#if (!QC_AFC_CHARGING_DISABLED)
        bc_status_t* bc_stat = &gl_bc_status[i];

        /*
         * Configure DP/DM Comparators to enable device wakeup in QC 2.0 mode.
         * When QC device updates QC2.0 mode, these interrupts will wake up the device.
         * QC2.0 mode request is then debounced in bc_debounce() routine.
         * Setting comparators based on the currrent QC2.0 mode.
         */
        if ((bc_stat->bc_fsm_state == BC_FSM_SRC_QC_OR_AFC) ||
            (bc_stat->bc_fsm_state == BC_FSM_SRC_QC_CONNECTED))
        {
            switch (bc_stat->dp_dm_status.state)
            {
                case QC_MODE_5V:
                    /* For < 0.6V transition on DP. */
                    chgb_set_comp(i, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                          CHGB_VREF_0_425V, CHGB_COMP_EDGE_FALLING);
                    /* For > 0V transition on DM. */
                    chgb_set_comp(i, BC_CMP_1_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
                          CHGB_VREF_0_425V, CHGB_COMP_EDGE_RISING);
                    /* QCOM RCVR interrupt will be used for > 0.6V transition on DP. */
                    break;

                case QC_MODE_9V:
                    /* For < 0.6V transition on DM. */
                    chgb_set_comp(i, BC_CMP_1_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
                              CHGB_VREF_0_425V, CHGB_COMP_EDGE_FALLING);
                    /*
                     * QCOM RCVR interrupts will be used for < 3.3V transition on DP
                     * and > 0.6V transition on DM.
                     */
                    break;

                case QC_MODE_12V:
                    /* For < 0.6V transition on DP. */
                    chgb_set_comp(i, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                              CHGB_VREF_0_425V, CHGB_COMP_EDGE_FALLING);
                     /* For < 0.6V transition on DM. */
                    chgb_set_comp(i, BC_CMP_1_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
                              CHGB_VREF_0_425V, CHGB_COMP_EDGE_FALLING);
                    /* QCOM RCVR interrupts will be used for > 0.6V transition on DP and DM. */
                    break;

                case QC_MODE_20V:
                    /* Nothing to do here beacuse QC RCVR interrupt will detect <3.3V
                     * transition on DP and DM. */
                    break;

                case QC_MODE_CONT:
                    /* For < 0.6V transition on DP. */
                    chgb_set_comp(i, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                              CHGB_VREF_0_425V, CHGB_COMP_EDGE_FALLING);
                    /*
                     * QCOM RCVR interrupt will be used for < 3.3V transition on DM and > 0.6V
                     * transition on DP.
                     */
                    break;

                default:
                    break;
            }
        }

#endif /* (!QC_AFC_CHARGING_DISABLED) */
    }

#endif /* (((defined(CCG3PA) || (defined(CCG3PA2)))) && (CCG_CDP_EN)) */

#elif (defined(CCG5C) || (defined(CCG6)))
    /* CCG5 or CCG5C*/
    uint8_t i;

    for(i = 0; i < NO_OF_BC_PORTS; i++)
    {
        bc_status_t* bc_stat = &gl_bc_status[i];
        if ((bc_stat->bc_evt != 0) ||
                (timer_is_running(i, APP_BC_GENERIC_TIMER1) == true) ||
                (timer_is_running(i, APP_BC_GENERIC_TIMER2) == true) ||
                (timer_is_running(i, APP_BC_DETACH_DETECT_TIMER) == true) ||
                (timer_is_running(i, APP_BC_DP_DM_DEBOUNCE_TIMER) == true))
        {
            return false;
        }
    }

    for (i = 0; i < NO_OF_BC_PORTS; i++)
    {
        bc_status_t* bc_stat = &gl_bc_status[i];
        if(bc_stat->cur_mode != BC_CHARGE_CDP)

        {
            /* Configure QC RCVR interrupt as wakeup source from DP/DM activity. */
            chgb_deepsleep (i);
        }
    }
#elif (defined(CCG5))
    uint8_t i;

    for (i = 0; i < NO_OF_BC_PORTS; i++)
    {
        /* Cannot go to sleep while we have VDM_SRC enabled. */
        if (ccg_is_cdp_sm_busy (i))
        {
            return false;
        }
    }
#endif /* CCGx */
    return true;
}

#if (defined CCG5C) || (defined CCG6)
/* This function checks whether the BC 1.2 sink connected to the port has been removed.
 * The CDP state machine is restarted, if so.
 */
void bc_cdp_detach_cbk(uint8_t cport, timer_id_t id)
{
    (void)id;

    bc_status_t* bc_stat = &gl_bc_status[cport];

    /* Stop the counter and read the count value */
    if (chgb_get_counter_value(cport) == 0)
    {
        /* Go to look for CDP connection state */
        bc_stat->bc_fsm_state = BC_FSM_SRC_CDP_CONNECTED;
        bc_stat->bc_evt = BC_EVT_ENTRY;
    }

    /* Power down the comparators */
    chgb_disable_cdp_comparators(cport);
}
#endif /* (defined CCG5C) || (defined CCG6) */

bool bc_wakeup(void)
{
#if (defined(CCG3PA)) || (defined(CCG3PA2) || defined(PAG1S))

#if (((defined(CCG3PA) || (defined(CCG3PA2)))) && (CCG_CDP_EN))
    /* CCG3PA CDP1.2 Do nothing */    
#else
    uint8_t i = 0;
    for(i = 0; i < NO_OF_BC_PORTS; i++)
    {

#if (!QC_AFC_CHARGING_DISABLED)

        /*
         * If in QC or AFC mode, we might have configured comparators to wakeup device
         * on DP/DM activity. Disable the comparators.
         */
        bc_status_t* bc_stat = &gl_bc_status[i];

        if ((bc_stat->bc_fsm_state == BC_FSM_SRC_QC_OR_AFC) ||
            (bc_stat->bc_fsm_state == BC_FSM_SRC_QC_CONNECTED))
        {
            chgb_stop_comp(i, BC_CMP_0_IDX);
            bc_clear_bc_evt(i, BC_EVT_CMP1_FIRE);
            chgb_stop_comp(i, BC_CMP_1_IDX);
            bc_clear_bc_evt(i, BC_EVT_CMP2_FIRE);
        }
#endif /* (!QC_AFC_CHARGING_DISABLED) */

        /* Disable QC RCVR interrupts. */
        chgb_wakeup(i);
    }
#endif /* (((defined(CCG3PA) || (defined(CCG3PA2)))) && (CCG_CDP_EN)) */    
    
#elif (defined(CCG6) || defined(CCG5C))
    uint8_t i = 0;
    for(i = 0; i < NO_OF_BC_PORTS; i++)
    {
        /* If the port is configured as CDP and a sink was previously connected, check whether it has been
         * removed.
         */
        if (detach_detect_en[i])
        {
            /* Configure the comparators and select the output */
            chgb_config_cdp_comparators(i);

            /* Configure and start the Counter */
            chgb_counter_start(i);

            /* Start 5ms timer and handle detach detection in cbk */
            timer_start(i, APP_BC_DETACH_DETECT_TIMER, 5, bc_cdp_detach_cbk);
        }

        chgb_wakeup(i);
    }
#else /* CCG5 */
    /* Do nothing. */
#endif /* CCGx */
    return true;
}

const bc_status_t* bc_get_status(uint8_t cport)
{
    return &gl_bc_status[cport];
}

void bc_pd_event_handler(uint8_t port, app_evt_t evt)
{
    const chg_cfg_params_t *chg_cfg = bc_get_config(port);
#if defined(CCG3PA) || defined(CCG3PA2) || defined(PAG1S)
    if(port != TYPEC_PORT_0_IDX)
    {
        /* Only Port0 is Type C port */
        return;
    }
#endif /* defined(CCG3PA) || defined(CCG3PA2) */

    /*
     * NOTE: The port index variable is multiplex for this usage model.
     * The application currently supports only one Type-C port. If this
     * gets modified, then the port index handling should be updated to match.
     */

    switch (evt)
    {
        case APP_EVT_DISCONNECT:
        case APP_EVT_VBUS_PORT_DISABLE:
            bc_stop(port);
            break;

        case APP_EVT_PE_DISABLED:
#if (!LEGACY_PD_PARALLEL_OPER)
            /* Start legacy state machine once PE is disabled. */
            if (dpm_get_info(port)->cur_port_role == PRT_ROLE_SOURCE)
            {
                /* Already started in case of parallel legacy and PD source operation. */
                if ((chg_cfg->src_sel & BATT_CHARGING_SRC_MASK) != 0)
                {
                    bc_start(port, BC_PORT_SOURCE);
                }
            }

#if ((!CCG_SOURCE_ONLY) && (!BC_SOURCE_ONLY))
            if (dpm_get_info(port)->cur_port_role == PRT_ROLE_SINK)
            {
                if ((chg_cfg->src_sel & BATT_CHARGING_SINK_MASK) != 0)
                {
                    bc_start(port, BC_PORT_SINK);
                }
            }
#endif /* ((!CCG_SOURCE_ONLY) && (!BC_SOURCE_ONLY)) */
#endif /* (!LEGACY_PD_PARALLEL_OPER) */
            break; 
 
#if (LEGACY_PD_PARALLEL_OPER)
        case APP_EVT_TYPEC_ATTACH:
            /* Start legacy state machine once PE is disabled. */
            if (dpm_get_info(port)->cur_port_role == PRT_ROLE_SOURCE)
            {
                /* Already started in case of parallel legacy and PD source operation. */
                if ((chg_cfg->src_sel & BATT_CHARGING_SRC_MASK) != 0)
                {
                    bc_start(port, BC_PORT_SOURCE);
                }
            }
#if ((!CCG_SOURCE_ONLY) && (!BC_SOURCE_ONLY))
            else
            {
                if ((chg_cfg->snk_sel & BATT_CHARGING_SINK_MASK) != 0)
                {
                    bc_start(port, BC_PORT_SINK);
                }
            }
#endif /* ((!CCG_SOURCE_ONLY) && (!BC_SOURCE_ONLY)) */
            break;
#endif /* (LEGACY_PD_PARALLEL_OPER) */

#if (LEGACY_PD_PARALLEL_OPER)

       case APP_EVT_PD_SINK_DEVICE_CONNECTED:
            if (
                    (dpm_get_info(port)->cur_port_role == PRT_ROLE_SOURCE) &&
                    ((chg_cfg->src_sel & BATT_CHARGING_SRC_MASK) != 0)
               )
            {
                /* Sink device responded to SRC CAP. Stop legacy state machine. */
#if (!CCG_BC_12_IN_PD_ENABLE)
                bc_stop(port);
#endif /* (!CCG_BC_12_IN_PD_ENABLE) */
            }
            break;

#if (!BC_SOURCE_ONLY)
       case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE:
            if (
                    (dpm_get_info(port)->cur_port_role == PRT_ROLE_SINK) &&
                    ((chg_cfg->snk_sel & BATT_CHARGING_SINK_MASK) != 0)
               )
            {
                /* PD message received. Stop BC 1.2 state machine. */
                bc_stop(port);
            }
            break;
#endif /* (!BC_SOURCE_ONLY) */

#endif /* (LEGACY_PD_PARALLEL_OPER) */

        default:
            break;
    }
}

void bc_set_bc_evt(uint8_t cport, uint32_t evt_mask)
{
    bc_status_t* bc_stat = &gl_bc_status[cport];

    uint8_t intr_state = CyEnterCriticalSection();

    bc_stat->bc_evt |= evt_mask;

    CyExitCriticalSection(intr_state);
}

void bc_clear_bc_evt(uint8_t cport, uint32_t evt_mask)
{
    bc_status_t* bc_stat = &gl_bc_status[cport];

    uint8_t intr_state = CyEnterCriticalSection();

    bc_stat->bc_evt &= ~evt_mask;

    CyExitCriticalSection(intr_state);
}

#if (!(CCG_SOURCE_ONLY)) && (!BC_SOURCE_ONLY)
static void bc_eval_apple_brick_id(uint8_t cport, bc_apple_brick_id brick_id)
{
    /* Handle detected Apple Brick IDs. */
    switch (brick_id)
    {
        case APPLE_BRICK_ID_3:
            psnk_set_current (cport, APPLE_AMP_1A);
            break;

        case APPLE_BRICK_ID_1:
            psnk_set_current (cport, APPLE_AMP_2_1A);
            break;

        case APPLE_BRICK_ID_4:
             psnk_set_current (cport, APPLE_AMP_2_4A);
            break;

        default:
            /* Rest of the Brick IDs reserved for future use. */
            break;
    }

    /* Ensure VBUS is set to 5V. */
    psnk_set_voltage (cport, VSAFE_5V);
}
#endif /* (!(CCG_SOURCE_ONLY)) */

/* FSM functions start */

static void bc_fsm_off(uint8_t cport, bc_fsm_evt_t evt)
{
    gl_bc_status[cport].cur_mode = BC_CHARGE_NONE;
}

static void bc_fsm_src_look_for_connect(uint8_t cport, bc_fsm_evt_t evt)
{
    bc_status_t *bc_stat = &gl_bc_status[cport];
#ifndef CCG5C
    const chg_cfg_params_t *chg_cfg = bc_get_config(cport);
#endif /* CCG5C */

    switch(evt)
    {
        case BC_FSM_EVT_ENTRY:
            /*
             * The detection logic varies based on the protocols selected.
             * If only Apple charging is selected, then different Apple source IDs
             * are supported. If Apple charging needs to be supported along with
             * BC 1.2, then only 2.4A Apple charger ID is supported.
             *
             * If Apple charging is selected, keep presenting the Apple terminations.
             * In case of Apple only charging mode, keep presenting the terminations.
             * There is no further action after this.
             *
             * If Apple charging along with BC 1.2 based detection is enabled,
             * then first start with Apple 2.4A termination. Also enable D+ comparator
             * to look for < 2.2V. If this is detected, then switch to BC termination
             * and proceed with BC 1.2 based connection detection logic.
             *
             * If Apple charging is not selected, then proceed directly to BC 1.2
             * terminations and subsequent detection logic.
             *
             * Detach detection for Apple and BC 1.2 DCP sink cannot be done as
             * sink terminations are not present after detection. Only re-attach
             * can be detected. So, In case of BC 1.2 DCP operation, this state
             * shall be re-entered to reapply Apple terminations as required.
             *
             * In case of QC and AFC mode of operation, detach can be detected.
             * When this happens, this state shall be re-entered. Detach handling
             * needs to be done for this.
             *
             * NOTE: There are two cases which are not currently dealt with:
             * 1. In case of Type-C port, when we enter legacy state machine for
             *    the first time, the VBUS may be already be present and the
             *    sink may already be attached and completed its detection logic.
             *    We may need to power cycle VBUS to restart sink's detection
             *    logic. This is not currently done as we start PD and Legacy
             *    together (LEGACY_PD_PARALLEL_OPER).
             *
             * 2. In case of Type-A port attached to a BC 1.2 sink or an Apple
             *    sink, there is no real detach detection. When Apple charging
             *    is enabled, there is also no re-attach detection.
             *
             *    The type-A port current consumption is monitored to switch to
             *    low power regulator when current drops below 300mA. This same
             *    logic moves back to high power regulator, but there is a polling
             *    delay as well as regulator turn on delay involved which can
             *    cause the 5V regulator (which also feeds the CCG3PA device) to
             *    shut-off due to over current.
             *
             *    There may be a work around; when we switch to low power regulator,
             *    also switch to BC 1.2 DCP attach wait state and stay there until
             *    there is an attach. In event of attach, switch to Apple mode of
             *    detection and proceed as usual.
             *
             *    Since the current systems are able to withstand the switch
             *    without disconnecting, no implementation is done currently.
             */
            chgb_stop_comp(cport, BC_CMP_0_IDX);
            chgb_stop_comp(cport, BC_CMP_1_IDX);
            timer_stop_range(cport, APP_BC_GENERIC_TIMER1, APP_BC_DETACH_DETECT_TIMER);
            bc_clear_bc_evt(cport, BC_EVT_ALL_MASK);
            chgb_enable(cport);

#ifndef CCG5C
            if (chg_cfg->src_sel & BC_SRC_APPLE_MODE_ENABLE_MASK)
            {
#if (LEGACY_APPLE_SRC_SLN_TERM_ENABLE)
                /*
                 * If Apple charging is enabled, then present Apple terminations.
                 * Since parallel operation requires external control, invoke
                 * the solution function instead of the HAL function. It is expected
                 * that the solution handler uses external termination for DP
                 * when parallel operation is required. The solution handler
                 * can choose to use internal termination when parallel operation
                 * is not required. This is useful when only one port requires
                 * parallel operation.
                 */
                sln_apply_apple_src_term(cport, (chgb_src_term_t)chg_cfg->apple_src_id);
                bc_stat->cur_amp = apple_id_to_cur_map[chg_cfg->apple_src_id];
                /*
                 * If parallel operation is expected, then setup CMP2 to
                 * detect D+ going below 2.2V. If it goes below this level,
                 * then it means that a BC 1.2 based sink is attached.
                 */
                if (chg_cfg->src_sel & BC_SRC_1_2_MODE_ENABLE_MASK)
                {
                    chgb_set_comp(cport, BC_CMP_1_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                            CHGB_VREF_2_2V, CHGB_COMP_EDGE_FALLING);
                }
#else /* (!LEGACY_APPLE_SRC_SLN_TERM_ENABLE) */
                chgb_apply_src_term(cport, (chgb_src_term_t)chg_cfg->apple_src_id);
                bc_stat->cur_amp = apple_id_to_cur_map[chg_cfg->apple_src_id];
#endif /* (LEGACY_APPLE_SRC_SLN_TERM_ENABLE) */

                /* Indicate connectivity. */
                bc_stat->attach = true;
                bc_stat->connected = true;
                bc_stat->cur_mode = BC_CHARGE_APPLE;

#if CCG_TYPE_A_PORT_ENABLE
                if (cport == TYPE_A_PORT_ID)
                {
                    type_a_update_status(true, true);
                }
#endif /* CCG_TYPE_A_PORT_ENABLE */
            }
            else
#endif /* CCG5C */
            {
                /*
                 * If in DCP mode, do not change the setting, else this is
                 * a detach. Indicate the same.
                 */
                if (bc_stat->cur_mode != BC_CHARGE_DCP)
                {
                    bc_stat->cur_mode = BC_CHARGE_NONE;
                    bc_stat->attach = false;

#if CCG_TYPE_A_PORT_ENABLE
                    if (cport == TYPE_A_PORT_ID)
                    {
                        type_a_update_status(false, false);
                    }
#endif /* CCG_TYPE_A_PORT_ENABLE */
                }
                /* Ensure DCP terminations are on by default. */
                chgb_apply_src_term(cport, CHGB_SRC_TERM_DCP);

                /* Set Comp1 to look for > 0.4V on D+ */
                chgb_set_comp(cport, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                        CHGB_VREF_0_425V, CHGB_COMP_EDGE_RISING);
            }

            /* Ensure that VBUS is 5V. But do this only if PD is disabled. */
            if (!((cport == TYPEC_PORT_0_IDX) &&
                       (dpm_get_info(cport)->pd_disabled == false)))
            {
                psrc_set_voltage(cport, VSAFE_5V);
                psrc_enable(cport, bc_pwr_ready_cbk);
            }
            break;

        case BC_FSM_EVT_CMP1_FIRE:
#if CCG_TYPE_A_PORT_ENABLE
            /* Switch to high power VBUS regulator as soon as connect is detected. */
            if (cport == TYPE_A_PORT_ID)
            {
                type_a_update_status (true, false);
            }
#endif /* CCG_TYPE_A_PORT_ENABLE */

            bc_stat->attach = true;
            bc_stat->cur_mode = BC_CHARGE_NONE;
            bc_stat->cur_amp = BC_AMP_LIMIT;
            chgb_stop_comp(cport, BC_CMP_0_IDX);
            bc_stat->bc_fsm_state = BC_FSM_SRC_INITIAL_CONNECT;
            bc_set_bc_evt(cport, BC_EVT_ENTRY);
            break;

#if (LEGACY_APPLE_SRC_SLN_TERM_ENABLE)
        case BC_FSM_EVT_CMP2_FIRE:
            if(bc_stat->cur_mode == BC_CHARGE_APPLE)
            {
                /*
                 * A BC 1.2 based sink has been attached. Need to switch the
                 * terminations to BC 1.2.
                 */
                chgb_stop_comp(cport, BC_CMP_1_IDX);
                bc_clear_bc_evt(cport, BC_EVT_ALL_MASK);

                /*
                 * This may be a glitch. Do a small debounce to ensure that
                 * we are attached to a BC device. Also, latest iphones are
                 * also causing a glitch on DP during connect. This need to
                 * be filtered out. There is no real debounce logic required.
                 * If a BC device is attached, the line would stay low beyond
                 * a fixed duration; if not the line shall revert back to 2.2V.
                 */
                timer_start(cport, APP_BC_GENERIC_TIMER1, APP_BC_APPLE_DETECT_TIMER_PERIOD, bc_tmr_cbk);
            }
            break;

        case BC_FSM_EVT_TIMEOUT1:
            if(bc_stat->cur_mode == BC_CHARGE_APPLE)
            {
                /*
                 * If the DP voltage continues below 2.2V, we are attached to
                 * BC device; proceed with BC detection. If not, go back to Apple
                 * mode and wait for DP to go down again.
                 */
                if (chgb_set_comp(cport, BC_CMP_1_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                            CHGB_VREF_2_2V, CHGB_COMP_NO_INTR) == false)
                {
                    sln_remove_apple_src_term(cport);

                    /* Ensure DCP terminations are on by default. */
                    chgb_apply_src_term(cport, CHGB_SRC_TERM_DCP);

                    bc_stat->attach = false;
                    bc_stat->connected = false;
                    bc_stat->cur_mode = BC_CHARGE_NONE;

                    /* Set Comp1 to look for > 0.4V on D+ */
                    chgb_set_comp(cport, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                            CHGB_VREF_0_425V, CHGB_COMP_EDGE_RISING);

#if CCG_TYPE_A_PORT_ENABLE
                    /*
                     * Switch to low power regulator for the time being.
                     */
                    if (cport == TYPE_A_PORT_ID)
                    {
                        type_a_update_status(false, false);
                    }
#endif /* CCG_TYPE_A_PORT_ENABLE */
                }
                else
                {
                    chgb_set_comp(cport, BC_CMP_1_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                            CHGB_VREF_2_2V, CHGB_COMP_EDGE_FALLING);
                }
            }
            break;
#endif /* (LEGACY_APPLE_SRC_SLN_TERM_ENABLE) */

        default:
            break;
    }
}

static void bc_fsm_src_initial_connect(uint8_t cport, bc_fsm_evt_t evt)
{
    bc_status_t *bc_stat = &gl_bc_status[cport];
    const chg_cfg_params_t *chg_cfg = bc_get_config(cport);
    switch(evt)
    {
        case BC_FSM_EVT_ENTRY:
            bc_stat->comp_rising = false;
            /* Set Comp1 to look for < 0.4V on D+ for 1 second */
            chgb_set_comp(cport, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                          CHGB_VREF_0_425V, CHGB_COMP_EDGE_FALLING);
            /* Set Comp2 to ensure DP does not go above 2.2V. */
            chgb_set_comp(cport, BC_CMP_1_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                          CHGB_VREF_2V, CHGB_COMP_EDGE_RISING);
            /* Start TGLITCH_BC_DONE timer to ascertain Apple or others */
            timer_start (cport, APP_BC_GENERIC_TIMER1, APP_BC_DCP_DETECT_TIMER_PERIOD, bc_tmr_cbk);
            break;

        case BC_FSM_EVT_CMP1_FIRE:
            if(bc_stat->comp_rising == false)
            {
                /*
                 * If AFC or BC1.2 mode is enabled, then we will
                 * have to determine those devices.
                 */
                if ((chg_cfg->src_sel & (BC_SRC_1_2_MODE_ENABLE_MASK
                                | BC_SRC_AFC_MODE_ENABLE_MASK)) != 0)
                {
                    bc_clear_bc_evt (cport, BC_EVT_TIMEOUT2);
                    timer_start (cport, APP_BC_GENERIC_TIMER2, 150, bc_tmr_cbk);
                }
                /*
                 * In QC only mode, stop DCP detect timer and wait for DP to rise
                 * above 0.6V again.
                 */
                else
                {
                    timer_stop (cport, APP_BC_GENERIC_TIMER1);
                    bc_clear_bc_evt (cport, BC_EVT_TIMEOUT1);
                }
                /* Set Comp1 to look for > 0.4V on D+. */
                chgb_set_comp(cport, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                        CHGB_VREF_0_425V, CHGB_COMP_EDGE_RISING);
                bc_stat->comp_rising = true;
            }
            else
            {
                /* The DP line has gone above 0.6V. Re-start detection. */
                timer_stop(cport, APP_BC_GENERIC_TIMER2);
                bc_clear_bc_evt(cport, BC_EVT_TIMEOUT2);
                chgb_stop_comp(cport, BC_CMP_0_IDX);
                bc_set_bc_evt(cport, BC_EVT_ENTRY);
            }
            break;


        case BC_FSM_EVT_CMP2_FIRE:
            /*
             * If TGLITCH_BC_DONE timer is running, that means DP went above 2V before
             * glitch filter timer expired. We will have to wait for DP to come back in 0.4 - 2V range
             * and then start device detection again. Till then we will stay in DCP only mode.
             */
            if (timer_is_running (cport, APP_BC_GENERIC_TIMER1))
            {
                /* DP went above 2V. We should stop DCP Detect timer. */
                timer_stop(cport, APP_BC_GENERIC_TIMER1);
                bc_clear_bc_evt(cport, BC_EVT_TIMEOUT1);
                /* Stop Apple device detect timer as well. */
                timer_stop(cport, APP_BC_GENERIC_TIMER2);
                bc_clear_bc_evt(cport, BC_EVT_TIMEOUT2);

                /* Stop Comp0. */
                chgb_stop_comp(cport, BC_CMP_0_IDX);
                bc_clear_bc_evt(cport, BC_EVT_CMP1_FIRE);

                /*
                 * From this point on we should wait for DP to fall below 2V. When it falls below
                 * 2V, we can start device detetcion again.
                 */
                chgb_set_comp(cport, BC_CMP_1_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                          CHGB_VREF_2V, CHGB_COMP_EDGE_FALLING);
            }
            else
            {
                /* DP is now below 2V. Start device detection again. */
                chgb_stop_comp(cport, BC_CMP_1_IDX);
                bc_stat->bc_fsm_state = BC_FSM_SRC_INITIAL_CONNECT;
                bc_set_bc_evt(cport, BC_EVT_ENTRY);
            }
            break;

#if (!QC_AFC_CHARGING_DISABLED)
        case BC_FSM_EVT_TIMEOUT1:
            chgb_stop_comp(cport, BC_CMP_0_IDX);
            bc_clear_bc_evt(cport, BC_EVT_CMP1_FIRE);
            chgb_stop_comp(cport, BC_CMP_1_IDX);
            bc_clear_bc_evt(cport, BC_EVT_CMP2_FIRE);
            timer_stop(cport, APP_BC_GENERIC_TIMER2);
            bc_clear_bc_evt(cport, BC_EVT_TIMEOUT2);
            bc_stat->bc_fsm_state = BC_FSM_SRC_OTHERS_CONNECTED;
            bc_set_bc_evt(cport, BC_EVT_ENTRY);
            break;
#endif /* (!QC_AFC_CHARGING_DISABLED) */

        case BC_FSM_EVT_TIMEOUT2:
            chgb_stop_comp(cport, BC_CMP_0_IDX);
            bc_clear_bc_evt(cport, BC_EVT_CMP1_FIRE);
            chgb_stop_comp(cport, BC_CMP_1_IDX);
            bc_clear_bc_evt(cport, BC_EVT_CMP2_FIRE);
            timer_stop(cport, APP_BC_GENERIC_TIMER1);
            bc_clear_bc_evt(cport, BC_EVT_TIMEOUT1);

            /* Treat this as a BC1.2 device. */
            /* Indicate BC1.2 device is connected so that current
             * monitoring can start. */
#if CCG_TYPE_A_PORT_ENABLE
            if (cport == TYPE_A_PORT_ID)
            {
                type_a_update_status (true, true);
            }
#endif /* CCG_TYPE_A_PORT_ENABLE */
            bc_stat->connected = true;
            bc_stat->cur_mode = BC_CHARGE_DCP;
            bc_stat->cur_amp = BC_AMP_LIMIT;
            /* Go back to init state and wait for DP attach event similar
             * to Apple mode. */
            bc_stat->bc_fsm_state = BC_FSM_SRC_LOOK_FOR_CONNECT;
            bc_set_bc_evt(cport, BC_EVT_ENTRY);
            break;

        default:
            break;
    }
}

static void bc_fsm_src_apple_connected(uint8_t cport, bc_fsm_evt_t evt)
{
    (void)cport;
    (void)evt;
}

#if (defined CCG5C) || (defined CCG6)

#define CDP_DX_VOLTAGE_CHECK_PERIOD     (15)    /* Frequency of D+/D- voltage checks in CDP state machine. */
#define CDP_VDMSRC_FAULT_CHECK_PERIOD   (2)     /* Frequency of D- voltage checks once a fault has been detected. */
#define MAX_CDP_VDMSRC_FAULT_COUNT      (3)     /* Max. number of faulty D- voltage readings allowed. */

/* Timer which periodically measures D- voltage to ensure that it is in the expected VDM_SRC range. */
void vdmsrc_check_timer_cb(uint8_t cport, timer_id_t id)
{
    uint16_t poll_interval = CDP_DX_VOLTAGE_CHECK_PERIOD;

    if (chgb_check_vdm_src_on (cport))
    {
        if (!chgb_set_comp(cport, BC_CMP_1_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF, CHGB_VREF_0_425V, CHGB_COMP_NO_INTR))
        {
            poll_interval = CDP_VDMSRC_FAULT_CHECK_PERIOD;
            vdm_src_lv_cnt[cport]++;

            if (vdm_src_lv_cnt[cport] >= MAX_CDP_VDMSRC_FAULT_COUNT)
            {
                /* DM voltage is too low despite VDM_SRC being applied. Turn off VDM_SRC. */
                chgb_remove_vdm_src(cport);
            }
        }
        else
        {
            /* Clear the fault count. */
            vdm_src_lv_cnt[cport] = 0;
        }

        /* Restart the timer so that we keep checking the DM voltage. */
        timer_start(cport, APP_BC_GENERIC_TIMER1, poll_interval, vdmsrc_check_timer_cb);

        /* Restore CMP_1 configuration to default. */
        chgb_set_comp(cport, BC_CMP_1_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                CHGB_VREF_0_85V, CHGB_COMP_EDGE_RISING);
    }
}

static void bc_fsm_src_apply_cdp(uint8_t cport, bc_fsm_evt_t evt)
{
    bc_status_t *bc_stat = &gl_bc_status[cport];

    switch(evt)
    {
        case BC_FSM_EVT_ENTRY:
            chgb_stop_comp(cport, BC_CMP_0_IDX);
            chgb_stop_comp(cport, BC_CMP_1_IDX);
            timer_stop_range(cport, APP_BC_GENERIC_TIMER1, APP_BC_DP_DM_DEBOUNCE_TIMER);
            bc_clear_bc_evt(cport, BC_EVT_ALL_MASK);
            detach_detect_en[cport] = false;
            chgb_enable(cport);

            bc_stat->cur_mode = BC_CHARGE_CDP;
            chgb_apply_src_term(cport, CHGB_SRC_TERM_CDP);

            /* Set comparator 1 to fire when D+ > 0.425V */
            chgb_set_comp(cport, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                        CHGB_VREF_0_425V, CHGB_COMP_EDGE_RISING);
            /* Set comparator 2 to fire when D+ > 0.85V */
            chgb_set_comp(cport, BC_CMP_1_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                        CHGB_VREF_0_85V, CHGB_COMP_EDGE_RISING);
            vdm_src_applied[cport] = false;
            bc_stat->comp_rising = false;
            break;

        case BC_FSM_EVT_CMP1_FIRE:
            if (!bc_stat->comp_rising)
            {
                /* Voltage in 0.4 to 0.8 range. Turn on VDM_SRC */
                if(!vdm_src_applied[cport])
                {
                    vdm_src_applied[cport] = true;
                    bc_stat->comp_rising = true;
                    chgb_apply_vdm_src(cport);

                    /* Start a timer which will measure DM voltage and ensure that it is in the valid range. */
                    timer_start (cport, APP_BC_GENERIC_TIMER1, CDP_DX_VOLTAGE_CHECK_PERIOD, vdmsrc_check_timer_cb);

                    /* Set comparator 1 to fire when D+ < 0.425V */
                    chgb_set_comp(cport, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                        CHGB_VREF_0_425V, CHGB_COMP_EDGE_FALLING);
                }
            }
            else
            {
                /* Voltage below 0.4V , turn off VDM_SRC */
                bc_stat->comp_rising = false;
                if (vdm_src_applied[cport])
                {
                    chgb_remove_vdm_src(cport);
                    vdm_src_applied[cport] = false;
                }
                chgb_stop_comp(cport, BC_CMP_0_IDX);
                chgb_stop_comp(cport, BC_CMP_1_IDX);
                bc_set_bc_evt(cport, BC_EVT_ENTRY);
            }
            break;

        case BC_FSM_EVT_CMP2_FIRE:
            /* Voltage gone above 0.8. Stop CDP detection */
            chgb_stop_comp(cport, BC_CMP_0_IDX);
            timer_stop_range(cport, APP_BC_GENERIC_TIMER1, APP_BC_DP_DM_DEBOUNCE_TIMER);
            if(vdm_src_applied[cport])
            {
                chgb_remove_vdm_src(cport);
                vdm_src_applied[cport] = false;
            }
            /* Isolate the DPDM lines from the charge detect block */
            detach_detect_en[cport] = true ;
            chgb_isolate_dpdm(cport);
            break;

        default:
            break;
    }
}
#endif /* (defined CCG5C)|| (defined CCG6) */

#if (!QC_AFC_CHARGING_DISABLED)
static void bc_stop_src_cap_on_detect(uint8_t cport)
{
    /*
     * The code assumes that there is only single Type-C port. This
     * needs to be updated for device families which has more than
     * one Type-C port.
     */
    if((cport == TYPEC_PORT_0_IDX) &&
            (false == dpm_get_info(cport)->pd_disabled))
    {
        dpm_disable(cport);
    }

}

static void bc_fsm_src_others_connected(uint8_t cport, bc_fsm_evt_t evt)
{
    bc_status_t *bc_stat = &gl_bc_status[cport];
    const chg_cfg_params_t *chg_cfg = bc_get_config(cport);
    switch(evt)
    {
        case BC_FSM_EVT_ENTRY:
            bc_stat->connected = true;

            if((chg_cfg->src_sel &
               (BC_SRC_AFC_MODE_ENABLE_MASK | BC_SRC_QC_MODE_ENABLE_MASK)) != 0)
            {
                bc_stat->dp_dm_status.state = QC_MODE_5V;
                bc_stat->old_dp_dm_status.state = QC_MODE_5V;

                /* Try detecting QC or AFC */
                chgb_apply_src_term(cport, CHGB_SRC_TERM_QC);

                /* Let voltage settle */
                CyDelayUs(100);
            }
            /* Set Comp1 to look for < 0.4V on D- to ensure no short on DP/DM */
            chgb_set_comp(cport, BC_CMP_0_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
                          CHGB_VREF_0_425V, CHGB_COMP_EDGE_FALLING);
            break;
        case BC_FSM_EVT_CMP1_FIRE:
            /* Move to next state */
            chgb_stop_comp(cport, BC_CMP_0_IDX);
            bc_clear_bc_evt(cport, BC_EVT_CMP1_FIRE);
            if((chg_cfg->src_sel &
               (BC_SRC_AFC_MODE_ENABLE_MASK | BC_SRC_QC_MODE_ENABLE_MASK)) != 0)
            {
                bc_stat->bc_fsm_state = BC_FSM_SRC_QC_OR_AFC;
            }
            else
            {
                bc_stat->bc_fsm_state = BC_FSM_SRC_LOOK_FOR_CONNECT;
            }
            bc_set_bc_evt(cport, BC_EVT_ENTRY);
            break;
        default:
            break;
    }
}

static void bc_fsm_src_qc_or_afc(uint8_t cport, bc_fsm_evt_t evt)
{
    bc_status_t *bc_stat = &gl_bc_status[cport];
    const chg_cfg_params_t *chg_cfg = bc_get_config(cport);

    switch(evt)
    {
        case BC_FSM_EVT_ENTRY:
            if((chg_cfg->src_sel & BC_SRC_AFC_MODE_ENABLE_MASK) != 0)
            {
                bc_stat->afc_src_msg_count = 0;
                bc_stat->afc_src_match_count = 0;
                bc_stat->afc_src_is_matched = false;
                chgb_afc_src_init(cport);
                chgb_afc_src_start(cport);
            }
            if((chg_cfg->src_sel & BC_SRC_QC_MODE_ENABLE_MASK) != 0)
            {
                chgb_qc_src_init(cport);
            }
            break;
        case BC_FSM_EVT_DISCONNECT:
            /* Detached */
            bc_stat->bc_fsm_state = BC_FSM_SRC_LOOK_FOR_CONNECT;
            bc_set_bc_evt(cport, BC_EVT_ENTRY);
            break;
        case BC_FSM_EVT_QC_CHANGE:
            /* Not AFC move to QC detected */
            chgb_afc_src_stop(cport);
            bc_stat->cur_mode = BC_CHARGE_QC2;
            bc_stat->bc_fsm_state = BC_FSM_SRC_QC_CONNECTED;
            bc_set_bc_evt(cport, BC_EVT_QC_CHANGE);
            break;
        case BC_FSM_EVT_AFC_MSG_RCVD:
            /* Not QC, move to AFC detected */
            chgb_qc_src_stop(cport);
            bc_stat->cur_mode = BC_CHARGE_AFC;
            bc_stat->cur_amp = BC_AMP_LIMIT;
            bc_stat->bc_fsm_state = BC_FSM_SRC_AFC_CONNECTED;
            bc_set_bc_evt(cport, BC_EVT_AFC_MSG_RCVD);
            bc_stop_src_cap_on_detect(cport);
            break;
        case  BC_FSM_EVT_AFC_MSG_SEND_FAIL:
            chgb_afc_src_start(cport);
            break;
        case BC_FSM_EVT_AFC_RESET_RCVD:
            chgb_afc_src_stop(cport);
            bc_stat->afc_src_msg_count = 0;
            bc_stat->afc_src_match_count = 0;
            bc_stat->afc_src_is_matched = false;
            bc_stat->afc_tx_active = false;
            chgb_afc_src_start(cport);
            break;
        default:
            break;
    }
}

#if (QC_CF_EN || LEGACY_DYN_CFG_ENABLE)
void qc_set_cf_limit(uint8_t cport)
{
    /* Get the current port PDP value */
    uint8_t port_pdp = ccg_get_system_max_pdp(cport);
    
    gl_bc_status[cport].cur_amp = GET_MIN(((port_pdp * CCG_POWER_PRECISION_MULT)/psrc_get_voltage(cport)),
                                    LEGACY_MAX_CABLE_RATING);
    pd_cf_enable(cport, gl_bc_status[cport].cur_amp);
    app_get_status(cport)->cur_fb_enabled = true;
}
#endif /* (QC_CF_EN || LEGACY_DYN_CFG_ENABLE) */

static void bc_fsm_src_qc_connected(uint8_t cport, bc_fsm_evt_t evt)
{
    int pulse_count;
#if (QC_CF_EN || LEGACY_DYN_CFG_ENABLE)    
    bool new_qc_state = false;
#endif /* (QC_CF_EN || LEGACY_DYN_CFG_ENABLE) */
    uint32_t new_volt = 0;
    bc_status_t* bc_stat = &gl_bc_status[cport];
    const chg_cfg_params_t *chg_cfg = bc_get_config(cport);

    switch(evt)
    {
        case BC_FSM_EVT_DISCONNECT:
            /* Detached */
            chgb_qc_src_cont_mode_stop(cport);
            bc_stat->bc_fsm_state = BC_FSM_SRC_LOOK_FOR_CONNECT;
#if (QC_CF_EN || LEGACY_DYN_CFG_ENABLE)            
            /* On QC detach, disable the current foldback on the port */
            pd_cf_disable(cport);
            dpm_set_cf(cport, false);
            app_get_status(cport)->cur_fb_enabled = false;
#endif /* (QC_CF_EN || LEGACY_DYN_CFG_ENABLE) */            
            bc_set_bc_evt(cport, BC_EVT_ENTRY);
            break;
        case BC_FSM_EVT_QC_CHANGE:
#if (QC_CF_EN || LEGACY_DYN_CFG_ENABLE)            
            /* We have detected a voltage change. Set this variable to update the CF value */
            new_qc_state = true;
#endif /* (QC_CF_EN || LEGACY_DYN_CFG_ENABLE) */    
            if(bc_stat->dp_dm_status.state != QC_MODE_CONT)
            {
                bc_stat->cur_mode = BC_CHARGE_QC2;
                /* Disable Continuous mode operation */
                chgb_qc_src_cont_mode_stop(cport);
                bc_clear_bc_evt(cport, BC_EVT_QC_CONT);
            }
            
            if(bc_stat->dp_dm_status.state != QC_MODE_CONT)
            {
                /*
                 * Here if we detect a voltage other than 5V in QC mode we stop sending source caps.
                 */
                bc_stop_src_cap_on_detect(cport); 
            }
            switch(bc_stat->dp_dm_status.state)
            {
                case QC_MODE_5V:
#if !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE))                
                    bc_stat->cur_amp = QC_AMP_5V;   
#endif /* !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE)) */
                    psrc_set_voltage(cport, VSAFE_5V);
                    psrc_enable(cport, bc_pwr_ready_cbk);
                    break;
                case QC_MODE_9V:
#if !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE))                    
                    bc_stat->cur_amp = QC_AMP_9V;
#endif /* !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE)) */
                    psrc_set_voltage(cport, VSAFE_9V);
                    psrc_enable(cport, bc_pwr_ready_cbk);
                    break;
                case QC_MODE_12V:
#if !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE))                    
                    bc_stat->cur_amp = QC_AMP_12V;
#endif /* !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE)) */
                    psrc_set_voltage(cport, VSAFE_12V);
                    psrc_enable(cport, bc_pwr_ready_cbk);
                    break;
                case QC_MODE_20V:
                    if((chg_cfg->qc_src_type == BC_SRC_QC_VER_2_CLASS_B_VAL) ||
                       (chg_cfg->qc_src_type == BC_SRC_QC_VER_3_CLASS_B_VAL)
                       )
                    {
#if !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE))
                        bc_stat->cur_amp = QC_AMP_20V;    
#endif /* !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE))*/
                        psrc_set_voltage(cport, VSAFE_20V);
                        psrc_enable(cport, bc_pwr_ready_cbk);
                    }
                    break;
                case QC_MODE_CONT:
                    if(chg_cfg->qc_src_type >= BC_SRC_QC_VER_3_CLASS_A_VAL)
                    {
#if !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE))                        
                        bc_stat->cur_amp = QC_AMP_CONT;
                        bc_stat->cur_mode = BC_CHARGE_QC3;
#endif /* (!((QC_CF_EN || LEGACY_DYN_CFG_ENABLE)) */    
                        /* Enable Continuous mode operation */
                        chgb_qc_src_cont_mode_start(cport);
                    }
                    break;
            }
            break;
        case BC_FSM_EVT_QC_CONT:
            pulse_count = chgb_get_qc_pulse_count(cport);
            if(pulse_count > 0)
            {
                /* Voltage change in mV units. Each pulse cause 200mV change */
                new_volt = pulse_count * QC_CONT_VOLT_CHANGE_PER_PULSE;
                new_volt = new_volt + app_get_status(cport)->psrc_volt;
            }
            else
            {
                new_volt = 0 - pulse_count;
                /* Voltage change in mV units. Each pulse cause 200mV change */
                new_volt = new_volt * QC_CONT_VOLT_CHANGE_PER_PULSE;
                if(new_volt <= (app_get_status(cport)->psrc_volt - QC3_MIN_VOLT))
                {
                    new_volt = app_get_status(cport)->psrc_volt - new_volt;
                }
                else
                {
                    new_volt = QC3_MIN_VOLT;
                }
            }

            /* Check for minimum and maximum voltage levels and limit to allowed range. */
            if(new_volt < QC3_MIN_VOLT)
            {
                new_volt = QC3_MIN_VOLT;
            }
            if ((chg_cfg->qc_src_type  == BC_SRC_QC_VER_3_CLASS_A_VAL) && (new_volt > VSAFE_12V))
            {
                new_volt = VSAFE_12V;
            }
            if ((chg_cfg->qc_src_type == BC_SRC_QC_VER_3_CLASS_B_VAL) && (new_volt > VSAFE_20V))
            {
                new_volt = VSAFE_20V;
            }
            /* If the voltage is higher than 5V, then we should stop PD. */
            if(new_volt > VSAFE_5V)
            {
                bc_stop_src_cap_on_detect(cport);
            }
            /* Update voltage only if there is a difference. */
            if(app_get_status(cport)->psrc_volt != new_volt)
            {
#if !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE))                        
                bc_stat->cur_amp = QC_AMP_CONT;    
#endif /* !((QC_CF_EN || LEGACY_DYN_CFG_ENABLE)) */
                psrc_set_voltage(cport, new_volt);
                psrc_enable(cport, bc_pwr_ready_cbk);
            }
            /* Clear out the handled pulse count. */
            chgb_update_qc_pulse_count(cport, pulse_count);
            break;
        default:
            break;
    }
#if (QC_CF_EN || LEGACY_DYN_CFG_ENABLE)
    
    if((app_get_status(cport)->cur_fb_enabled == false) || (new_qc_state == true))
    {
        /* If current foldback has not been enabled on the port or if there is a QC state change
         * We update the current foldback on the port
         */
        qc_set_cf_limit(cport);        
    }

#endif /* (QC_CF_EN || LEGACY_DYN_CFG_ENABLE) */    
}

static void bc_afc_src_evaluate_match(uint8_t cport)
{

    bc_status_t* bc_stat = &gl_bc_status[cport];
    if(bc_stat->afc_src_msg_count >= 3)
    {
        if(bc_stat->afc_src_match_count >= 2)
        {
            bc_stat->afc_src_msg_count = 0;
            bc_stat->afc_src_match_count = 0;
            psrc_set_voltage(cport, (AFC_BASE_VOLT + (BYTE_GET_UPPER_NIBBLE(bc_stat->afc_src_matched_byte) * AFC_VOLT_STEP)));
            bc_stat->cur_amp = (AFC_BASE_AMP + (BYTE_GET_LOWER_NIBBLE(bc_stat->afc_src_matched_byte) * AFC_AMP_STEP));
            psrc_enable(cport, bc_pwr_ready_cbk);
            chgb_afc_src_start(cport);
        }
        else
        {
            /* Enter default operation and clear AFC counters if 3 attempts fail  */
            bc_stat->afc_src_msg_count = 0;
            bc_stat->afc_src_match_count = 0;
            bc_stat->cur_volt = VSAFE_5V;
            psrc_set_voltage(cport, VSAFE_5V);
            bc_stat->cur_amp = BC_AMP_LIMIT;
            psrc_enable(cport, bc_pwr_ready_cbk);
        }
    }
    else
    {
        chgb_afc_src_start(cport);
    }
}

static void bc_afc_src_handle_rcvd_msg(uint8_t cport)
{
    bc_status_t* bc_stat = &gl_bc_status[cport];
    /* Match Data received and send proper response same byte if match else all */
    uint8_t rcvd_vi = chgb_afc_get_rx_data_ptr(cport)[0];
    uint8_t i;
    uint8_t *src_vi = bc_afc_src_get_vi_ptr(cport);
    uint8_t src_count = bc_afc_src_get_vi_count(cport);
    /* Set tx active flag. */
    bc_stat->afc_tx_active = true;
    for(i = 0; i < src_count; i++)
    {
        if(((rcvd_vi & 0xF0) == (src_vi[i] & 0xF0) ) &&
           ((rcvd_vi & 0xF) <= (src_vi[i] & 0xF)))
        {
            bc_stat->afc_src_cur_match_byte = rcvd_vi;
            bc_stat->afc_src_is_matched = true;
            chgb_afc_set_tx_data(cport, &rcvd_vi, 1);
            return;
        }
    }
    bc_stat->afc_src_is_matched = false;
    /* If there was no match, we clear both the AFC counters */
    bc_stat->afc_src_msg_count = 0;
    bc_stat->afc_src_match_count = 0;
    chgb_afc_set_tx_data(cport, src_vi, src_count);
}

static void bc_fsm_src_afc_connected(uint8_t cport, bc_fsm_evt_t evt)
{
    bc_status_t* bc_stat = &gl_bc_status[cport];

    switch(evt)
    {
        case BC_FSM_EVT_DISCONNECT:
            /* Detached */
            chgb_afc_src_stop(cport);
            bc_stat->bc_fsm_state = BC_FSM_SRC_LOOK_FOR_CONNECT;
            bc_set_bc_evt(cport, BC_EVT_ENTRY);
            break;
        case BC_FSM_EVT_AFC_MSG_RCVD:
            bc_afc_src_handle_rcvd_msg(cport);
            break;
        case BC_FSM_EVT_AFC_RESET_RCVD:
            chgb_afc_src_stop(cport);
            bc_stat->cur_volt = VSAFE_5V;
            psrc_set_voltage(cport, VSAFE_5V);
            psrc_enable(cport, bc_pwr_ready_cbk);
            bc_stat->afc_src_msg_count = 0;
            bc_stat->afc_src_match_count = 0;
            bc_stat->afc_src_is_matched = false;
            bc_stat->afc_tx_active = false;
            chgb_afc_src_start(cport);
            break;
        case BC_FSM_EVT_AFC_MSG_SENT:
            if(bc_stat->afc_src_is_matched == true)
            {
               /* Increment AFC counters on match only */ 
               bc_stat->afc_src_msg_count++;
               if(bc_stat->afc_src_cur_match_byte == bc_stat->afc_src_last_match_byte)
                {
                    bc_stat->afc_src_match_count++;
                    if(bc_stat->afc_src_match_count == 2)
                    {
                        bc_stat->afc_src_matched_byte = bc_stat->afc_src_cur_match_byte;
                    }
                }
            }
            bc_stat->afc_tx_active = false;
            bc_stat->afc_src_last_match_byte = bc_stat->afc_src_cur_match_byte;
            bc_afc_src_evaluate_match(cport);
            break;
        case BC_FSM_EVT_AFC_MSG_SEND_FAIL:

            /* If transmission was active, increment msg count. */
            if (bc_stat->afc_tx_active == true)
            {
                bc_stat->afc_tx_active = false;
                bc_stat->afc_src_msg_count++;
                bc_afc_src_evaluate_match(cport);
            }
            else
            {
                /* This is a timeout event. Restart hardware state machine. */
                chgb_afc_src_start(cport);
            }
            break;
        default:
            break;
    }
}
#endif /* (!QC_AFC_CHARGING_DISABLED) */

#if (!(CCG_SOURCE_ONLY)) && (!BC_SOURCE_ONLY)
static void bc_fsm_sink_start(uint8_t cport, bc_fsm_evt_t evt)
{
    bc_status_t *bc_stat = &gl_bc_status[cport];
    const chg_cfg_params_t *chg_cfg = bc_get_config(cport);
    if (evt == BC_FSM_EVT_ENTRY)
    {
        /* Set up CHGDET hardware block for operation. */
        chgb_disable (cport);
        chgb_enable (cport);

        bc_stat->cur_mode = BC_CHARGE_NONE;

        /* Move to Apple charger detection state, if enabled. */
        if (chg_cfg->snk_sel & BC_SINK_APPLE_MODE_ENABLE_MASK)
        {
            bc_stat->bc_fsm_state = BC_FSM_SINK_APPLE_CHARGER_DETECT;
            bc_set_bc_evt (cport, BC_EVT_ENTRY);
        }
        /* Start BC1.2, if enabled. */
        else if (chg_cfg->snk_sel & BC_SINK_1_2_MODE_ENABLE_MASK)
        {
            bc_stat->bc_fsm_state = BC_FSM_SINK_PRIMARY_CHARGER_DETECT;
            bc_set_bc_evt (cport, BC_EVT_ENTRY);
        }
        /* No legacy charging mode is enabled. Assume TYPE-C only source. */
        else
        {
            bc_stat->bc_fsm_state = BC_FSM_SINK_TYPE_C_ONLY_SOURCE_CONNECTED;
            bc_set_bc_evt (cport, BC_EVT_ENTRY);
        }
    }
}

static void bc_fsm_sink_apple_charger_detect(uint8_t c_port, bc_fsm_evt_t evt)
{
    bool apple_charger_detected = false;
    bc_status_t *bc_stat = &gl_bc_status[c_port];
    const chg_cfg_params_t *chg_cfg = bc_get_config(c_port);

    /*
     * CCG Sink needs to detect if it is connected to Apple charger or not.
     * Apple charger is expected to drive >1V on both D+/-. So measure D+/-
     * voltage and determine the type of charger.
     */
    if (evt == BC_FSM_EVT_ENTRY)
    {
        /* Apple RDAT_LKG resistors on D+ and D-. */
        chgb_apply_rdat_lkg_dp (c_port);
        chgb_apply_rdat_lkg_dm (c_port);

        /*
         * TODO: See if a timer shall be used here instead of just checking current
         * voltage on D+/-.
         */

        /* Check if D+ > 1V */
        if (chgb_set_comp (c_port, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
            CHGB_VREF_0_85V, CHGB_COMP_NO_INTR) == true)
        {
            /* Check if D- > 1V. */
            if (chgb_set_comp (c_port, BC_CMP_0_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
            CHGB_VREF_0_85V, CHGB_COMP_NO_INTR) == true)
            {
                /* Apple charger detected. */
                apple_charger_detected = true;
            }
        }

        if (apple_charger_detected == true)
        {
            /* Now we now that CCG is connected to Apple charger. Detect Brick ID */
            bc_stat->cur_mode = BC_CHARGE_APPLE;
            bc_stat->bc_fsm_state = BC_FSM_SINK_APPLE_BRICK_ID_DETECT;
            bc_set_bc_evt (c_port, BC_EVT_ENTRY);
        }
        /* Apple charger not detected. */
        else
        {
            /* Start BC1.2, if enabled. */
            if (chg_cfg->snk_sel & BC_SINK_1_2_MODE_ENABLE_MASK)
            {
                bc_stat->bc_fsm_state = BC_FSM_SINK_PRIMARY_CHARGER_DETECT;
                bc_set_bc_evt (c_port, BC_EVT_ENTRY);
            }
            /* No legacy charging mode is enabled. Assume TYPE-C only source. */
            else
            {
                bc_stat->bc_fsm_state = BC_FSM_SINK_TYPE_C_ONLY_SOURCE_CONNECTED;
                bc_set_bc_evt (c_port, BC_EVT_ENTRY);
            }
        }
    }
}

static void bc_fsm_sink_apple_brick_id_detect(uint8_t c_port, bc_fsm_evt_t evt)
{
    /* Detect Apple Brick ID here as required by Apple Brick ID spec. */

    /*
     * DP and DM can be connected to three terminations:
     * TERM1 : 1 - 2.22 V
     * TERM2 : 2.22 - 2.89 V
     * TERM3 : 2,.89+ V
     * Encoding user here is : TERM1 : 1, TERM2: 2, TERM3: 3.
     */
    bc_apple_term dp_term = APPLE_TERM1, dm_term = APPLE_TERM1;
    if (evt == BC_FSM_EVT_ENTRY)
    {
        /*
         * We already know that DP is greater than 1V. Check if DP is greater than
         * 2.9V. If yes, we have term3 on DP. If not, check if DP is greater than 2.2V.
         * If yes, we have term2 on DP. Else, DP has term1.
         */

        /* Need to enable 2.9V detection for Apple brick ID. */
        chgb_enable_apple_det (c_port);

        if (chgb_set_comp (c_port, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
            CHGB_VREF_2_9V, CHGB_COMP_NO_INTR) == true)
        {
            dp_term = APPLE_TERM3;
        }
        else
        {
            if (chgb_set_comp (c_port, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                CHGB_VREF_2_2V, CHGB_COMP_NO_INTR) == true)
            {
                dp_term = APPLE_TERM2;
            }
        }

        /* Similar test for DM. */
        if (chgb_set_comp (c_port, BC_CMP_0_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
            CHGB_VREF_2_9V, CHGB_COMP_NO_INTR) == true)
        {
            dm_term = APPLE_TERM3;
        }
        else
        {
            if (chgb_set_comp (c_port, BC_CMP_0_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
                CHGB_VREF_2_2V, CHGB_COMP_NO_INTR) == true)
            {
                dm_term = APPLE_TERM2;
            }
        }
        /* Disable 2.9V detection for Apple brick ID. */
        chgb_disable_apple_det (c_port);

        /* Evaluate Apple termination detected. */
        bc_eval_apple_brick_id (c_port, dp_term | (dm_term << 0x04));

    }
}

static void bc_fsm_sink_primary_charger_detect(uint8_t c_port, bc_fsm_evt_t evt)
{
    bc_status_t* bc_stat = &gl_bc_status[c_port];
    /* This state is for primary charger detect. Refer BC 1.2 spec for details. */
    if (evt == BC_FSM_EVT_ENTRY)
    {
        /* Apply terminations on D+/-. */
        chgb_apply_sink_term (c_port, CHGB_SINK_TERM_PCD);
        timer_start (c_port, APP_BC_GENERIC_TIMER1, APP_BC_VDP_DM_SRC_ON_PERIOD, bc_tmr_cbk);
    }
    else if(evt == BC_FSM_EVT_TIMEOUT1)
    {
        /* Now measure D- and see if D- is pulled up to VDP_SRC. */
        if (chgb_set_comp (c_port, BC_CMP_0_IDX, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
            CHGB_VREF_0_425V, CHGB_COMP_NO_INTR) == true)
        {
            /* Start timer for source to differenciate between primary and secondary detection */
            timer_start (c_port, APP_BC_GENERIC_TIMER2, APP_BC_VDMSRC_EN_DIS_PERIOD, bc_tmr_cbk);
        }
        else
        {
            /* TYPE-C only source connected. */
            bc_stat->bc_fsm_state = BC_FSM_SINK_TYPE_C_ONLY_SOURCE_CONNECTED;
            bc_set_bc_evt (c_port, BC_EVT_ENTRY);
        }
        /* Remove applied terminations */
        chgb_remove_term(c_port);
    }
    else if (evt == BC_FSM_EVT_TIMEOUT2)
    {
        /* Proceed to secondary detection for CDP/DCP detection. */
        bc_stat->bc_fsm_state = BC_FSM_SINK_SECONDARY_CHARGER_DETECT;
        bc_set_bc_evt (c_port, BC_EVT_ENTRY);
    }
}

static void bc_fsm_sink_type_c_only_source_connected(uint8_t c_port, bc_fsm_evt_t evt)
{
    /* Do Nothing */
    (void) c_port;
    (void) evt;

    chgb_disable(c_port);

#ifdef CCG6
    /* Enable DP/DM Mux if not done so far. */
    ccg_config_dp_dm_mux (c_port,
            dpm_get_info(c_port)->polarity ? DPDM_MUX_CONN_USB_BOT_UART : DPDM_MUX_CONN_USB_TOP_UART);
#endif /* CCG6 */
}

static void bc_fsm_sink_secondary_charger_detect(uint8_t c_port, bc_fsm_evt_t evt)
{
    bc_status_t* bc_stat = &gl_bc_status[c_port];

    /*
     * This state is used to perform secondary charger detect. Refer BC 1.2 spec
     * for details.
     */

    if (evt == BC_FSM_EVT_ENTRY)
    {
        /* Apply terminations on D+/-. */
        chgb_apply_sink_term (c_port, CHGB_SINK_TERM_SCD);
        /* Start timer to apply VDM_SRC for TVDM_SRC_ON */
        timer_start (c_port, APP_BC_GENERIC_TIMER1, APP_BC_VDP_DM_SRC_ON_PERIOD, bc_tmr_cbk);
    }
    else if(evt == BC_FSM_EVT_TIMEOUT1)
    {
        /* Now measure D+ and see if D- is pulled up to VDM_SRC. */
        if (chgb_set_comp (c_port, BC_CMP_0_IDX, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
            CHGB_VREF_0_425V, CHGB_COMP_NO_INTR) == true)
        {
            /* DCP connected. */
            bc_stat->cur_mode = BC_CHARGE_DCP;
            bc_stat->bc_fsm_state = BC_FSM_SINK_DCP_CONNECTED;
            bc_set_bc_evt (c_port, BC_EVT_ENTRY);
        }
        else
        {
            /* CDP connected. */
            bc_stat->bc_fsm_state = BC_FSM_SINK_CDP_CONNECTED;
            bc_set_bc_evt (c_port, BC_EVT_ENTRY);
        }
        /* Remove applied terminations */
        chgb_remove_term(c_port);
    }
}

static void bc_fsm_sink_dcp_connected(uint8_t c_port, bc_fsm_evt_t evt)
{
    if (evt == BC_FSM_EVT_ENTRY)
    {
        /* DCP is connected. */
        psnk_set_current (c_port, I_1P5A);
        chgb_disable(c_port);

#ifdef CCG6
        /* Enable DP/DM Mux if not done so far. */
        ccg_config_dp_dm_mux (c_port,
                dpm_get_info(c_port)->polarity ? DPDM_MUX_CONN_USB_BOT_UART : DPDM_MUX_CONN_USB_TOP_UART);
#endif /* CCG6 */
    }
}

static void bc_fsm_sink_sdp_connected(uint8_t c_port, bc_fsm_evt_t evt)
{
    if (evt == BC_FSM_EVT_ENTRY)
    {
        /* SDP is connected. */
        psnk_set_current (c_port, ISAFE_DEF);
        chgb_disable(c_port);

#ifdef CCG6
        /* Enable DP/DM Mux if not done so far. */
        ccg_config_dp_dm_mux (c_port,
                dpm_get_info(c_port)->polarity ? DPDM_MUX_CONN_USB_BOT_UART : DPDM_MUX_CONN_USB_TOP_UART);
#endif /* CCG6 */
    }
}

static void bc_fsm_sink_cdp_connected(uint8_t c_port, bc_fsm_evt_t evt)
{
    if (evt == BC_FSM_EVT_ENTRY)
    {
        /* CDP is connected. */
        /* Set the battery charging current to 1.5A. */
        psnk_set_current (c_port, I_1P5A);
        chgb_disable(c_port);

#ifdef CCG6
        /* Enable DP/DM Mux if not done so far. */
        ccg_config_dp_dm_mux (c_port,
                dpm_get_info(c_port)->polarity ? DPDM_MUX_CONN_USB_BOT_UART : DPDM_MUX_CONN_USB_TOP_UART);
#endif /* CCG6 */
    }
}
#endif /* (!(CCG_SOURCE_ONLY)) */

/* FSM functions end */

#if ((!LEGACY_DYN_CFG_ENABLE) || (QC_AFC_CHARGING_DISABLED))

void bc_afc_form_vi(uint8_t cport)
{

}

#endif /* ((!LEGACY_DYN_CFG_ENABLE) || (QC_AFC_CHARGING_DISABLED)) */

#if QC_AFC_CHARGING_DISABLED
void qc_set_cf_limit(uint8_t cport)
{

}

#endif /* QC_AFC_CHARGING_DISABLED */

#else /* !BATTERY_CHARGING_ENABLE */
/* Dummy definitions to suppress the undefined reference for libraries */
const chg_cfg_params_t *bc_get_config(uint8_t cport)
{
    return NULL;
}

ccg_status_t bc_stop(uint8_t cport)
{
    return CCG_STAT_SUCCESS;
}

bool bc_is_active(uint8_t cport)
{
    return false;
}

ccg_status_t bc_set_config(uint8_t cport, chg_cfg_params_t *chg_cfg)
{
    return CCG_STAT_FAILURE;
}

uint8_t ccg_get_system_max_pdp(uint8_t cport)
{
    /* TODO: The CCG_MAX_PDP_VALUE constant needs to be defined with appropriate value. */
    return 45; //CCG_MAX_PDP_VALUE;
}

void qc_set_cf_limit(uint8_t cport)
{

}

void bc_afc_form_vi(uint8_t cport)
{

}

const bc_status_t* bc_get_status(uint8_t cport)
{
    return NULL;
}


#endif /* BATTERY_CHARGING_ENABLE */

/* End of File */
