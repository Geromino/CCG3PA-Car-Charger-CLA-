/**
 * @file battery_charging.h
 *
 * @brief @{Battery Charging header file.@}
 *
 *******************************************************************************
 * @copyright
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
#ifndef _BATTERY_CHARGING_H_
#define _BATTERY_CHARGING_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <config.h>
#include <pd.h>
#include <status.h>
#include <chgb_hal.h>

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/** @cond DOXYGEN_HIDE */

/* Combined mask for all legacy battery charging protocols in source mode. */
#define BATT_CHARGING_SRC_MASK  (BC_SRC_APPLE_MODE_ENABLE_MASK | \
        BC_SRC_1_2_MODE_ENABLE_MASK | BC_SRC_QC_MODE_ENABLE_MASK | \
        BC_SRC_AFC_MODE_ENABLE_MASK)

/* Combined mask for all legacy battery charging protocols in sink mode. */
#define BATT_CHARGING_SINK_MASK (BC_SINK_1_2_MODE_ENABLE_MASK | \
        BC_SINK_APPLE_MODE_ENABLE_MASK)

/* QC Dp/Dm macros*/
#define QC_MODE_12V                                 (((BC_D_0_6V) << 8)|BC_D_0_6V)
#define QC_MODE_9V                                  (((BC_D_0_6V) << 8)|BC_D_3_3V)
#define QC_MODE_CONT                                (((BC_D_3_3V) << 8)|BC_D_0_6V)
#define QC_MODE_20V                                 (((BC_D_3_3V) << 8)|BC_D_3_3V)
#define QC_MODE_5V                                  (((BC_D_GND) << 8)|BC_D_0_6V)
#define QC_MODE_RSVD                                (((BC_D_ERR) << 8)|BC_D_ERR)

/* BC events macros. */
#define BC_EVT_NONE                                 (0x00000000u)
#define BC_EVT_ENTRY                                (0x00000001u)
#define BC_EVT_CMP1_FIRE                            (0x00000002u)
#define BC_EVT_CMP2_FIRE                            (0x00000004u)
#define BC_EVT_QC_CHANGE                            (0x00000008u)
#define BC_EVT_QC_CONT                              (0x00000010u)
#define BC_EVT_AFC_RESET_RCVD                       (0x00000020u)
#define BC_EVT_AFC_MSG_RCVD                         (0x00000040u)
#define BC_EVT_AFC_MSG_SENT                         (0x00000080u)
#define BC_EVT_AFC_MSG_SEND_FAIL                    (0x00000100u)
#define BC_EVT_TIMEOUT1                             (0x00000200u)
#define BC_EVT_TIMEOUT2                             (0x00000400u)
#define BC_EVT_DISCONNECT                           (0x00000800u)
#define BC_EVT_ALL_MASK                             (0xFFFFFFFFu)

/** @endcond */

#define BC_CMP_0_IDX                    (0u)    /**< Battery charger comparator #1. */
#define BC_CMP_1_IDX                    (1u)    /**< Battery charger comparator #2. */

#define BC_AMP_LIMIT                    (300)   /**< Maximum current across various BC modes: 3.0 A. */
#define APPLE_AMP_1A                    (100)   /**< Current limit for Apple 1.0A brick. */
#define APPLE_AMP_2_1A                  (210)   /**< Current limit for Apple 2.1A brick. */
#define APPLE_AMP_2_4A                  (240)   /**< Current limit for Apple 2.4A brick. */
#define QC_AMP_5V                       (300)   /**< Current limit for Quick Charge at 5 V. */
#define QC_AMP_9V                       (300)   /**< Current limit for Quick Charge at 9 V. */
#define QC_AMP_12V                      (300)   /**< Current limit for Quick Charge at 12 V. */
#define QC_AMP_20V                      (300)   /**< Current limit for Quick Charge at 20 V. */
#define QC_AMP_CONT                     (300)   /**< Current limit for Quick Charge continuous mode. */
#define QC_CONT_VOLT_CHANGE_PER_PULSE   (200u)  /**< Quick Charge continuous mode voltage change per pulse received. */
#define QC3_MIN_VOLT                    (3400u) /**< Minimum supply voltage used in QC charging. */

#define CCG_POWER_PRECISION_MULT        (100000u) /**< Multiplier used in PDP value simplification */
    
/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/
/**
 * @typedef bc_port_type_t
 * @brief Battery charging port types.
 */
typedef enum{
    BC_PORT_TYPE_A = 0,         /**< Type-A port. Only supported in power adapter and power bank designs. */
    BC_PORT_TYPE_C              /**< Type-C port. */
}bc_port_type_t;

/**
 * @typedef bc_port_role_t
 * @brief Battery charging port role.
 */
typedef enum{
    BC_PORT_SINK = 0,           /**< Power sink, device being charged. */
    BC_PORT_SOURCE              /**< Power source. */
}bc_port_role_t;

/**
 * @typedef bc_qc_class_t
 * @brief Qualcomm Quick Charge charger class.
 */
typedef enum{
    BC_QC_CLASS_A = 0,          /**< Class A HVDCP: Supports 5V, 9V and 12V supplies. */
    BC_QC_CLASS_B               /**< Class B HVDCP: Supports 5V, 9B, 12V and 20V supplies. */
}bc_qc_class_t;

/**
 * @typedef bc_qc_ver_t
 * @brief Qualcomm charger version.
 */
typedef enum{
    BC_QC_VER_2 = 0,            /**< QC 2.0 charger. */
    BC_QC_VER_3                 /**< QC 3.0 charger. */
}bc_qc_ver_t;

/**
 * @typedef bc_apple_id_t
 * @brief Apple charger brick id.
 */
typedef enum{
    BC_APPLE_ID_1A = 0,         /**< Apple 1.0 A charging brick. */
    BC_APPLE_ID_2_1A,           /**< Apple 2.1 A charging brick. */
    BC_APPLE_ID_2_4A            /**< Apple 2.4 A charging brick. */
}bc_apple_id_t;

/**
 * @typedef bc_charge_mode_t
 * @brief List of legacy battery charging schemes supported over a Type-C or Type-A port.
 */
typedef enum{
    BC_CHARGE_NONE = 0,         /**< No active battery charging modes. */
    BC_CHARGE_DCP,              /**< Dedicated Charging Port as defined by BC 1.2 spec. */
    BC_CHARGE_QC2,              /**< QC2.0 charger. */
    BC_CHARGE_QC3,              /**< QC3.0 charger. */
    BC_CHARGE_AFC,              /**< Adaptive Fast Charging mode. */
    BC_CHARGE_APPLE,            /**< Apple power brick. */
    BC_CHARGE_CDP               /**< Charging Downstream Port as defined by BC 1.2 spec. */
}bc_charge_mode_t;

/**
 * @typedef bc_d_status_t
 * @brief Enumeration of the various Dp/Dm states
 */
typedef enum
{
    BC_D_GND = 0,                       /**< DP/DM voltage < 0.325 */
    BC_D_0_6V,                          /**< 0.325 < DP/DPM voltage < 2V  */
    BC_D_3_3V,                          /**< DP/DM voltage > 2V */
    BC_D_ERR                            /**< Error state. */
} bc_d_status_t;

/**
 * @typedef bc_apple_term
 * @brief Enumeration of Apple terminations codes.
 */
typedef enum
{
    APPLE_TERM1 = 1,                    /**< Termination 1 code: 1 - 2.22 V */
    APPLE_TERM2 = 2,                    /**< Termination 2 code: 2.22 - 2.89 V */
    APPLE_TERM3 = 3                     /**< Termination 3 code: 2.89+ V */
} bc_apple_term;

/**
 * @typedef bc_apple_brick_id
 * @brief List of possible Apple Brick IDs based on the terminations on DP and DM pins.
 */
typedef enum
{
    APPLE_BRICK_ID_0 = 0x11,            /**< APPLE_TERM1 on DM, APPLE_TERM1 on DP. */
    APPLE_BRICK_ID_1 = 0x12,            /**< APPLE_TERM1 on DM, APPLE_TERM2 on DP. */
    APPLE_BRICK_ID_2 = 0x13,            /**< APPLE_TERM1 on DM, APPLE_TERM3 on DP. */
    APPLE_BRICK_ID_3 = 0x21,            /**< APPLE_TERM2 on DM, APPLE_TERM1 on DP. */
    APPLE_BRICK_ID_4 = 0x22,            /**< APPLE_TERM2 on DM, APPLE_TERM2 on DP. */
    APPLE_BRICK_ID_5 = 0x23,            /**< APPLE_TERM2 on DM, APPLE_TERM3 on DP. */
    APPLE_BRICK_ID_6 = 0x31,            /**< APPLE_TERM3 on DM, APPLE_TERM1 on DP. */
    APPLE_BRICK_ID_7 = 0x32,            /**< APPLE_TERM3 on DM, APPLE_TERM2 on DP. */
    APPLE_BRICK_ID_8 = 0x33,            /**< APPLE_TERM3 on DM, APPLE_TERM3 on DP. */
} bc_apple_brick_id;

/**
 * @brief Union to hold Dp/Dm status.
 */
typedef union
{
    uint16_t state;                     /**< Combined status of Dp and Dm. */
    uint8_t  d[2];                      /**< Individual status of Dp(d[0]) and Dm(d[1]). */
} bc_dp_dm_state_t;

/**
 * @typedef bc_state_t
 * @brief List of states in the legacy battery charging state machine.
 */
typedef enum{
    BC_FSM_OFF = 0,                                     /**< BC state machine inactive. */
    BC_FSM_SRC_LOOK_FOR_CONNECT,                        /**< Look for connection as a DCP. */
    BC_FSM_SRC_INITIAL_CONNECT,                         /**< Initial BC1.2 sink connection detected. */
    BC_FSM_SRC_APPLE_CONNECTED,                         /**< Connected to sink as an Apple power brick. */
    BC_FSM_SRC_CDP_CONNECTED,                           /**< Port configured as a Charging Downstream Port (CDP). */
    BC_FSM_SRC_OTHERS_CONNECTED,                        /**< DCP failed Apple sink detection. */
    BC_FSM_SRC_QC_OR_AFC,                               /**< Port connected to a QC or AFC sink. */
    BC_FSM_SRC_QC_CONNECTED,                            /**< Connected to sink as a QC HVDCP. */
    BC_FSM_SRC_AFC_CONNECTED,                           /**< Connected to sink as an AFC charger. */
#if (!(CCG_SOURCE_ONLY))
    BC_FSM_SINK_START,                                  /**< BC sink state machine start state. */
    BC_FSM_SINK_APPLE_CHARGER_DETECT,                   /**< Sink looking for an Apple charger. */
    BC_FSM_SINK_APPLE_BRICK_ID_DETECT,                  /**< Sink identified Apple charger, identifying brick ID. */
    BC_FSM_SINK_PRIMARY_CHARGER_DETECT,                 /**< BC 1.2 primary charger detect state. */
    BC_FSM_SINK_TYPE_C_ONLY_SOURCE_CONNECTED,           /**< BC 1.2 src detection failed, connected as Type-C sink. */
    BC_FSM_SINK_SECONDARY_CHARGER_DETECT,               /**< BC 1.2 secondary charger detect state. */
    BC_FSM_SINK_DCP_CONNECTED,                          /**< Sink connected to a BC 1.2 DCP. */
    BC_FSM_SINK_SDP_CONNECTED,                          /**< Sink connected to a Standard Downstream Port (SDP). */
    BC_FSM_SINK_CDP_CONNECTED,                          /**< Sink connected to a BC 1.2 CDP. */
#endif /* (!(CCG_SOURCE_ONLY)) */
    BC_FSM_MAX_STATES,                                  /**< Invalid state ID. */
}bc_state_t;

/**
 * @typedef bc_sink_timer_t
 * @brief List of possible timers in sink mode.
 */
typedef enum
{
    BC_SINK_TIMER_NONE = 0,                             /**< No timers running. */
    BC_SINK_DCD_DEBOUNCE_TIMER = 1,                     /**< DCD Debounce timer. */
} bc_sink_timer_t;

/*******************************************************************************
 * Data Struct Definition
 ******************************************************************************/

/**
 * @brief Struct to define battery charger status.
 */
typedef struct {
    bc_charge_mode_t cur_mode;                  /**< Active charging scheme. */
    bc_state_t bc_fsm_state;                    /**< Current state of the BC state machine. */
    uint32_t volatile bc_evt;                   /**< Bitmap representing event notifications to the state machine. */
    uint16_t cur_volt;                          /**< Active VBus voltage in mV units. */
    uint16_t cur_amp;                           /**< Active supply current in 10 mA units. */
    bool volatile connected;                    /**< Whether BC connection is detected. */

    bool volatile comp_rising;                  /**< Whether comparator is looking for a rising edge or falling edge. */
    bc_dp_dm_state_t volatile dp_dm_status;     /**< Debounced status of the DP/DM pins. */
    bc_dp_dm_state_t volatile old_dp_dm_status; /**< Previous status of the DP/DM pins. */
    bc_sink_timer_t cur_timer;                  /**< Identifies the BC timer that is running. */

    uint8_t afc_src_msg_count;                  /**< Number of successful AFC message transfers. 3 transfers have
                                                     to be successful before we can make any VI changes. */
    bool volatile afc_src_is_matched;           /**< Status of VI match from the last received AFC byte. */
    uint8_t afc_src_cur_match_byte;             /**< The currently matched AFC VI value. */
    uint8_t afc_src_last_match_byte;            /**< Previously matched AFC VI value. */
    uint8_t afc_src_matched_byte;               /**< Holds the VI byte that is matched in two out of the last three
                                                     messages. */
    uint8_t afc_src_match_count;                /**< Number of AFC VI byte matches detected. */
    uint8_t afc_tx_active;                      /**< Whether AFC message transmission is active. */

    bool attach;                                /**< Whether BC attach has been detected. */
}bc_status_t;

/*******************************************************************************
 * Global Function Declaration
 ******************************************************************************/

/**
 * @brief This function initializes the Battery Charging block. This should be
 * called one time only at system startup for each port on which the BC state
 * machine needs to run.
 * @param cport Battery Charging port index.
 * @return ccg_status_t
 */
ccg_status_t bc_init(uint8_t cport);

/**
 * @brief This function retrieves the current Battery Charging configuration.
 * If the LEGACY_DYN_CFG_ENABLE macro is enabled, then this returns the
 * latest configuration. If not, returns the configuration structure from
 * the configuration table.
 * @param cport Battery Charging port index.
 * @return Pointer to current configuration data. This should not be modified
 * by the caller.
 */
const chg_cfg_params_t *bc_get_config(uint8_t cport);

#if (LEGACY_DYN_CFG_ENABLE)

/**
 * @brief This function updates the battery charging configuration.
 * This is valid only if LEGACY_DYN_CFG_ENABLE macro is enabled. The configuration
 * can only be updated when the BC state machine is stopped. Also note that
 * bc_init() shall reload the configuration from the table.
 *
 * @param cport Battery Charging port index.
 * @param chg_cfg Pointer to structure containing new configuration.
 *
 * @return ccg_status_t
 */
ccg_status_t bc_set_config(uint8_t cport, chg_cfg_params_t *chg_cfg);
#endif /* (LEGACY_DYN_CFG_ENABLE) */

/**
 * @brief This function starts the Battery Charging block with desired configuration.
 * @param cport Battery Charging port index.
 * @param port_role Battery Charging port power role.
 * @return ccg_status_t
 */
ccg_status_t bc_start(uint8_t cport, bc_port_role_t port_role);

/**
 * @brief This function stops the Battery Charging block.
 * @param cport Battery Charging port index.
 * @return ccg_status_t
 */
ccg_status_t bc_stop(uint8_t cport);

/**
 * @brief This function returns whether the BC module is active or not.
 * @param cport Battery Charging port index.
 * @return true if the BC module is running, false otherwise.
 */
bool bc_is_active(uint8_t cport);

/**
 * @brief This function handles the Battery Charging block state machine.
 * @param cport Battery Charging port index.
 * @return ccg_status_t
 */
ccg_status_t bc_fsm(uint8_t cport);

/**
 * @brief Check whether the port is currently functioning as a CDP (Charging Downstream Port).
 * @param cport Battery Charging port index.
 * @return true if port is CDP, false otherwise.
 */
bool bc_port_is_cdp(uint8_t cport);

/**
 * @brief This function puts the Battery Charging block to sleep.
 * @return Returns true if the sleep is successful, false otherwise.
 */
bool bc_sleep(void);

/**
 * @brief This function wakes up the Battery Charging block.
 * @return Returns true if wakeup successful, false otherwise
 */
bool bc_wakeup(void);

/**
 * @brief This function retrieves the current status of the BC state machine.
 * @param cport Battery Charging port index to be queried.
 * @return Pointer to BC status. The structure must not be modified by caller.
 */
const bc_status_t* bc_get_status(uint8_t cport);

/**
 * @brief This function handles events from USB-PD Device Policy Manager.
 * This event handler is used call the bc_start and bc_stop functions to enable/disable
 * the BC state machine at appropriate times.
 * @param port PD port index.
 * @param evt PD event.
 * @return None
 */
void bc_pd_event_handler(uint8_t port, app_evt_t evt);

/**
 * @brief This function sets an event status for the BC state machine to process.
 * @param cport Charging port index
 * @param evt_mask Event Mask
 * @return None
 */
void bc_set_bc_evt(uint8_t cport, uint32_t evt_mask);

/**
 * @brief This function clears one or more BC events after the state machine has dealt
 * with them.
 * @param cport Port index.
 * @param evt_mask Event Mask to be cleared.
 * @return None
 */
void bc_clear_bc_evt(uint8_t cport, uint32_t evt_mask);

#if (LEGACY_APPLE_SRC_SLN_TERM_ENABLE)
/**
 * @brief This function enables solution specific Apple terminations on DP, DM.
 *
 * The legacy Apple + BC 1.2 source operation requires external resistive
 * terminations. This function chooses to apply internal / external terminations
 * based on solution hardware and requirements. This can be used to selectively
 * apply parallel operation on port basis. The user is expected to implement
 * this function. The function should provide Apple termination for 2.4A when
 * requiring parallel operation. When using Apple only mode, internal termination
 * can be used.
 *
 * To limit the external components in the system, the following configuration
 * is recommended.
 *
 * Since the detection algorithm is applied only on DP line, the resistor divider
 * logic is only required on the DP line. The internal termination can be applied
 * for DM line.
 *
 * For DP line, connect Rext from a dedicated GPIO to the DP line. The value of
 * this resistor should be such that the DP line voltage should be in range of
 * 2.5V - 2.7V. When the GPIO is strong driven high, it shall provide VDDIO voltage.
 * We can generate this voltage via resistor divider network. To avoid a second
 * resistance, internal Rdp_dwn can be used for this purpose. The resistance
 * range for this is 18.4KOhm - 21.91KOhm range (based on char data). Depending on
 * the VDDIO of the system, select Rext suitably to generate the optimum voltage.
 *
 * Over and above this, it is seen that some BC devices requires the DP-DM short
 * to exist before it applies the VDP_SRC correctly. For this purpose, it is
 * recommended to use internal short.
 *
 * The below equations can be used to determine the optimal Rext.
 *
 * Rext_min = (((VDDIO - VDPmax) * Rint_max) / VDPmax)
 * Rext_max = (((VDDIO - VDPmin) * Rint_min) / VDPmin)
 *
 * Recommended Rext for 5V VDDIO is 18Kohm.
 * Recommended Rext for 3.3V VDDIO is 5.1KOhm.
 *
 * @param cport Port index.
 * @param charger_term Termination ID to be applied.
 * @return None
 */
void sln_apply_apple_src_term(uint8_t cport, chgb_src_term_t charger_term);

/**
 * @brief This function disables solution specific Apple terminations on DP, DM.
 *
 * The legacy Apple + BC 1.2 source operation requires external resistive
 * terminiations. This function removes previously applied teriminations.
 *
 * @param cport Port index.
 * @return None
 */
void sln_remove_apple_src_term(uint8_t cport);

#endif /* (LEGACY_APPLE_SRC_SLN_TERM_ENABLE) */
    
/**
 * @brief This function returns the port PDP value.
 *
 * If PD3.0 is supported, then the value returned will the PDP value from the extended source cap
 * If PD3.0 is not supported, then the preconfigured PDP value will be returned
 *
 * @param cport Port index.
 * @return System PDP
 */
uint8_t ccg_get_system_max_pdp(uint8_t cport);

/**
 * @brief This function enables Current Foldback on the port when in QC mode.
 *
 * @param cport Port index.
 * @return None
 */
void qc_set_cf_limit(uint8_t cport);

/**
 * @brief This function forms the new AFC source capablities based on the power passed.
 *
 * The number of source caps for AFC will be same as that configured in the table. 
 * This function does not modify the voltage value of the source cap in the config table.
 * The current will be updated based on the power value and the AFC voltage.
 *
 * @param cport Port index.
 * @return None
 */
void bc_afc_form_vi(uint8_t cport);

#endif /* __BATTERY_CHARGING_H__ */

/* End of File */
