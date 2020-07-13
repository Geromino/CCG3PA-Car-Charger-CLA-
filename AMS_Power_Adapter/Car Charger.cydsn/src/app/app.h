/**
 * @file app.h
 *
 * @brief @{PD application handler header file.@}
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

#ifndef _APP_H_
#define _APP_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <stdint.h>
#include <pd.h>
#include <pdss_hal.h>
#include <alt_mode_hw.h>
#include <timer_id.h>

extern uint8_t connect_cable_debug;
/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

#define ADC_VBUS_MIN_OVP_LEVEL                          (6500u)
/**< Minimum OVP detection voltage when ADC is used to implement OVP (applies to CCG4). */

#define APP_MAX_SWAP_ATTEMPT_COUNT                      (10u)
/**< Number of swap attempts to be made when port partner is sending a WAIT response. */

/*************** Timer duration settings. Changes not recommended. ********************/

#define APP_PSOURCE_CF_TIMER_PERIOD                     (100u)
/**< Duration of Power Source Current Foldback timer (in ms). */

#define APP_PSOURCE_DIS_EXT_DIS_TIMER_PERIOD            (10u)
/**< Duration of extra VBus discharge after voltage drops below desired level (in ms). */

#define APP_PSINK_DIS_TIMER_PERIOD                      (250u)
/**< Maximum time allowed for power sink disable operation (in ms). */

#define APP_PSINK_DIS_MONITOR_TIMER_PERIOD              (1u)
/**< Period of VBus voltage checks performed during power sink disable operation. */

#define APP_PSINK_DIS_VBUS_IN_DIS_PERIOD                (20u)
/**< Duration of discharge sequence on the VBUS_IN supply in CCG3PA/CCG3PA2 designs. */

#define APP_FAULT_RECOVERY_TIMER_PERIOD                 (100u)
/**< Period of VBus presence checks after a fault (Over-Voltage) detection while in a sink contract. */

#define APP_FAULT_RECOVERY_MAX_WAIT                     (500u)
/**< Time for which VBus will be monitored to ensure removal of VBus by a faulty power source. */

#define APP_SBU_DELAYED_CONNECT_PERIOD                  (25u)
/**< Delay (in ms) between Thunderbolt mode entry and SBU path configuration. */

#define APP_CBL_DISC_TIMER_PERIOD                       (100u)
/**< Delay to be used between cable discovery init commands. */

#define APP_UFP_RECOV_VCONN_SWAP_TIMER_PERIOD           (1000u)
/**< Timer period used to run Vconn swap after V5V was lost and recovered while UFP. */

#define APP_VDM_BUSY_TIMER_PERIOD                       (50u)
/**< VDM busy timer period (in ms). */

#define APP_VDM_FAIL_RETRY_PERIOD                       (100u)
/**< VDM retry (on failure) timer period in ms. */

#define APP_CABLE_POWER_UP_DELAY                        (55u)
/**< Time allowed for cable power up to be complete. */

#define APP_CABLE_VDM_START_DELAY                       (5u)
/**< Cable query delay period in ms. */

#define APP_AME_TIMEOUT_TIMER_PERIOD                    (800u)
/**< tAME timer period (in ms). */

#define APP_DB_SNK_FET_DIS_DELAY_TIMER_PERIOD           (50u)
/**< Dead battery Sink Fet disable delay timer period. */

#define APP_BB_ON_TIMER_PERIOD                          (250u)
/**< Billboard ON delay timer period. This should be long enough for a host to properly
     recognize the disconnection and reconnection of the USB Billboard device. */

#define APP_INITIATE_DR_SWAP_TIMER_PERIOD               (5u)
/**< Time delay between successive DR_SWAP attempts (in ms). */

#define APP_INITIATE_PR_SWAP_TIMER_PERIOD               (500u)
/**< Time delay between successive PR_SWAP attempts (in ms). PR_SWAP attempts are kept slow to allow
     other sequences such as alternate mode negotiation to complete. */

#define APP_INITIATE_SEND_IRQ_CLEAR_ACK_PERIOD          (1u)
/**< Timer period for initiating Virtual HPD IRQ CLEAR ACK. */

#define RIDGE_INIT_HPD_DEQUEUE_TIMER_PERIOD             (1u)
/**< Timer period in ms for initiating virtual HPD dequeue. */

#define UCSI_CONNECT_EVENT_PERIOD                       (500u)
/**< Delay (in ms) between Type-C connection and sending UCSI event notification. */

#define APP_BC_CDP_SM_TIMER_PERIOD                      (30000u)
/**< CDP state machine timeout period. */

#define APP_BC_VDP_DM_SRC_ON_PERIOD                     (40u)
/**< VDP_SRC or VDM_SRC minimum turn on time. */

#define APP_BC_VDMSRC_EN_DIS_PERIOD                     (20u)
/**< VDM_SRC enable/disable maximum period. */

#define CCG_ACTIVITY_TIMER_PERIOD                       (500)
/**< CCG activity timer period in ms. */

#define OTP_DEBOUNCE_PERIOD                             (1)
/**< OTP debounce timer period in ms. */

#define TYPE_A_CUR_SENSE_TIMER_PERIOD                   (30000)
/**< TYPE-A current sense timer period. */

#define TYPE_A_REG_SWITCH_TIMER_PERIOD                  (10)
/**< TYPE-A regulator switch timer period in ms. */

#define TYPE_A_PWM_STEP_TIMER_PERIOD                    (1)
/**< TYPE-A port PWM step change time in ms. */

#define PB_DEBOUNCE_PERIOD                              (1)
/**< PB debounce timer period in ms. */

#define APP_PB_VBATT_DEBOUNCE_IN_MS                     (10)
/**< PB debounce period in ms. */

#define THROTTLE_DEBOUNCE_PERIOD                        (10)
/**< THROTTLE time period in ms. */

#define THROTTLE_WAIT_FOR_PD_PERIOD                     (500)
/**< THROTTLE time period in ms. */

#define APP_BAD_SINK_TIMEOUT_TIMER_PERIOD               (1000u)
/**< PD bad sink timeout timer period in ms. */

#define APP_PSOURCE_VBUS_SET_TIMER_PERIOD               (1)
/**< Power source VBUS set timer period in ms. */

#define APP_PSOURCE_SAFE_FET_ON_MONITOR_TIMER_PERIOD    (10)
/**< Psource safe FET ON monitor timer period in ms. */

#define APP_BC_VBUS_CYCLE_TIMER_PERIOD                  (200u)
/**< VBUS OFF time to do a VBUS power cycle. */

#define APP_BC_SINK_CONTACT_STABLE_TIMER_PERIOD         (50u)
/**< Sink DCD stable time period. */

#define APP_BC_DCP_DETECT_TIMER_PERIOD                  (1100u)
/**< Tglitch_done time waiting for the portable device to complete detection.
     This is used by QC/AFC devices to proceed with subsequent detection. */

#define APP_BC_APPLE_DETECT_TIMER_PERIOD                (5u)
/**< Debounce time to verify if the attached device is Apple or not. Apple devices
     create a glitch on DP line whereas BC devices continue to drive DP lower.
     This period is used when Apple and BC 1.2 source protocols are supported
     together. */

#define APP_BC_DP_DM_DEBOUNCE_TIMER_PERIOD              (40u)
/**< Debounce time for identifying state change for DP and DM lines. */

#define APP_BC_AFC_DETECT_TIMER_PERIOD                  (100u)
/**< AFC detection time. */

#define APP_BC_GLITCH_BC_DONE_TIMER_PERIOD              (1500)
/**< TGLITCH_BC_DONE timer period. This timer is used in sink mode to detect QC charger. */

#define APP_BC_GLITCH_DM_HIGH_TIMER_PERIOD              (40)
/**< T_GLITCH_DM_HIGH timer period. After DCP opens D+/- short, sink shall wait for this time before requesting VBUS. */

#define APP_BC_V_NEW_REQUEST_TIMER_PERIOD               (200)
/**< T_V_NEW_REQUEST timer period. After entering QC mode, sink must wait this much time before requesting
     next voltage. */

#define APP_OT_VBE_LOW_TEMP_ADDR                        (0x0FFFF427)
/**< Low tempetarure VBJT code sflash address. */

#define APP_OT_VBE_HIGH_TEMP_ADDR                       (0x0FFFF426)
/**< High tempetarure VBJT code sflash address. */

#define APP_OT_VBE_25_C_TEMP_ADDR                       (0x0FFFF428)
/**< 25C tempetarure VBJT code sflash address. */

#define APP_OT_LOW_TEMP                                 (-40)
/**< Low temperature which corresponds to low tempetarure VBJT code stored in sflash (in Celsius degrees) */

#define APP_OT_HIGH_TEMP                                (120u)
/**< High temperature which corresponds to high tempetarure VBJT code stored in sflash (in Celsius degrees) */

#define APP_OT_ROOM_TEMP                                (25)
/**< Room temperature used for VBJT code stored in sflash (in Celsius degrees) */

#define APP_OT_DETECTION_TIMER_PERIOD                   (500u)
/**< OT detection scanning period */

#define APP_VCONN_SWAP_PENDING                          (1u)
/**< VConn swap pending bit in the app. level pending swaps flag. */

#define APP_DR_SWAP_PENDING                             (2u)
/**< Data Role swap pending bit in the app. level pending swaps flag. */

#define APP_PR_SWAP_PENDING                             (4u)
/**< Power Role swap pending bit in the app. level pending swaps flag. */
    
#if BB_RETIMER_ENABLE 
    
    #define APP_RETIMER_DISABLE_DELAY                   (200u)
    /**< Time to wait after disconnect to power the retimer off */
    
#endif /* BB_RETIMER_ENABLE */     

#if ICL_RVP_HW
    
    #define APP_AUTO_DR_SWAP_TRY_PERIOD                 (10u)
    /**< Wait time between DR_Swap tries */
    
    #define ICL_VSYS_STABLE_WAIT_TIME                   (10u)
    /**< How long to wait for VSYS to stablilze */
    
    #define TBT_MODE_EXIT_CHECK_PERIOD                  (5u)
    /**< Periodicity of checking if TBT mode should be exited */
    
#endif /* ICL_RVP_HW */

#if ICL_ENABLE
    
/* System states: Values of the SYS_PWR_STATE register */
#define SYSTEM_STATE_S0                                     (0)
#define SYSTEM_STATE_S3                                     (1)
#define SYSTEM_STATE_S4                                     (2)
#define SYSTEM_STATE_S5                                     (3)

/***** DPM swap masks *****/
#define DPM_DR_SWAP_RESP_MASK               (0x03)
#define DPM_DR_SWAP_RESP_POS                (0x00)
#define DPM_PR_SWAP_RESP_MASK               (0x0C)
#define DPM_PR_SWAP_RESP_POS                (0x02)
#define DPM_VCONN_SWAP_RESP_MASK            (0x30)
#define DPM_VCONN_SWAP_RESP_POS             (0x04)
    
#endif /* ICL_ENABLE */    

/*****************************************************************************
 * Data Struct Definition
 ****************************************************************************/

/**
 * @brief This type of the function is used by app layer to call MUX polling
 * function.
 *
 * @param port Port index the function is performed for.
 * @return MUX polling status
 */
typedef mux_poll_status_t
(*mux_poll_fnc_cbk_t) (
        uint8_t         port);

/**
 * @typedef app_port_fault_status_mask_t
 * @brief Fault detection and handling related status bits tracked in the fault_status field.
 */
typedef enum {
    APP_PORT_FAULT_NONE                 = 0x00, /**< System functioning without any fault. */
    APP_PORT_VCONN_FAULT_ACTIVE         = 0x01, /**< Status bit that indicates VConn fault is active. */
    APP_PORT_SINK_FAULT_ACTIVE          = 0x02, /**< Status bit that indicates sink fault handling is pending. */
    APP_PORT_SRC_FAULT_ACTIVE           = 0x04, /**< Status bit that indicates source fault handling is pending. */
    APP_PORT_VBUS_DROP_WAIT_ACTIVE      = 0x08, /**< Status bit that indicates wait for VBus drop is pending. */
    APP_PORT_V5V_SUPPLY_LOST            = 0x10, /**< Status bit that indicates that V5V supply (for VConn) has been lost. */
    APP_PORT_DISABLE_IN_PROGRESS        = 0x80  /**< Port disable operation is in progress. */
} app_port_fault_status_mask_t;

/**
 * @typedef app_nb_sys_pwr_state_t
 * @brief List of system power states for Notebook/Desktop designs. These states will be reported by
 * the EC to CCG device through HPI registers.
 */
typedef enum
{
    NB_SYS_PWR_STATE_S0 = 0,            /**< Notebook/Desktop is in active (S0) state. */
    NB_SYS_PWR_STATE_S3,                /**< Notebook/Desktop is in Sleep (S3) state. */
    NB_SYS_PWR_STATE_S4,                /**< Notebook/Desktop is in Hibernate (S4) state. */
    NB_SYS_PWR_STATE_S5                 /**< Notebook/Desktop is in Off (S5) state. */
} app_nb_sys_pwr_state_t;

/**
   @brief This structure hold all variables related to application layer functionality.
 */
typedef struct
{
    pwr_ready_cbk_t pwr_ready_cbk;        /**< Registered Power source callback. */
    sink_discharge_off_cbk_t snk_dis_cbk; /**< Registered Power sink callback. */
    app_resp_t app_resp;                  /**< Buffer for APP responses. */
    vdm_resp_t vdm_resp;                  /**< Buffer for VDM responses. */
    uint16_t psrc_volt;                   /**< Current Psource voltage in mV */
    uint16_t psrc_volt_old;               /**< Old Psource voltage in mV */
    uint16_t psnk_volt;                   /**< Current PSink voltage in mV units. */
    uint16_t psnk_cur;                    /**< Current PSink current in 10mA units. */
    uint8_t vdm_task_en;                  /**< Flag to indicate is vdm task manager enabled. */
    uint8_t disc_cbl_pending;             /**< Flag to indicate is cable discovery is pending. */
    uint8_t cbl_disc_id_finished;         /**< Flag to indicate that cable disc id finished. */
    uint8_t vdm_version;                  /**< Live VDM version. */
    uint8_t alt_mode_trig_mask;           /**< Mask to indicate which alt mode should be enabled by EC. */
    uint8_t dfp_alt_mode_mask;            /**< Mask to enable DFP alternate modes. */
    uint8_t ufp_alt_mode_mask;            /**< Mask to enable UFP alternate modes. */
    uint16_t custom_hpi_svid;             /**< Holds custom alternate mode SVID received from HPI. */
    volatile uint8_t fault_status;        /**< Fault status bits for this port. */
#if ICL_RVP_HW
    bool debug_acc_attached;              /**< Debug accessory attach status */
#endif    
    bool alt_mode_entered;                /**< Flag to indicate is alternate modes currently entered. */
    bool vdm_prcs_failed;                 /**< Flag to indicate is vdm process failed. */
    bool is_vbus_on;                      /**< Is supplying VBUS flag. */
    bool is_vconn_on;                     /**< Is supplying VCONN flag. */
    bool vdm_retry_pending;               /**< Whether VDM retry on timeout is pending. */
    bool psrc_rising;                     /**< Voltage ramp up/down. */

#if (!defined(CCG4PD3))
    bool cur_fb_enabled;                  /**< Indicates that current foldback is enabled */
    bool ld_sw_ctrl;                      /**< Indicates whether the VBUS load switch control is active or not. */
    bool bc_12_src_disabled;              /**< BC 1.2 source disabled flag. */

    bool is_mux_busy;                     /**< Flag to indicate that mux is switching. */
    vdm_resp_cbk_t vdm_resp_cbk;          /**< VDM response handler callback. */
    bool is_vdm_pending;                  /**< VDM handling flag for MUX callback. */

    mux_poll_fnc_cbk_t mux_poll_cbk;      /**< Holds pointer to MUX polling function. */
#endif /* (!defined(CCG4PD3)) */

#if (DEFER_VCONN_SRC_ROLE_SWAP)
    bool keep_vconn_src;                  /**< Flag indicating that we should keep VConn source role. */
#endif /* (DEFER_VCONN_SRC_ROLE_SWAP) */

#if (ROLE_PREFERENCE_ENABLE)
    uint8_t       app_pending_swaps;      /**< Variable denoting the types of swap operation that are pending. */
    uint8_t       actv_swap_type;         /**< Denotes the active Swap operation. */
    uint8_t       actv_swap_count;        /**< Denotes number of active swap attempts completed. */
    uint16_t      actv_swap_delay;        /**< Delay to be applied between repeated swap attempts. */
#endif /* (ROLE_PREFERENCE_ENABLE) */

#if OTP_ENABLE
    uint8_t turn_on_temp_limit;           /**< Temperature threshold to turn on internal FET after turn off condition */
    uint8_t turn_off_temp_limit;          /**< Temperature threshold to turn off internal FET */
    bool is_hot_shutdown;                 /**< Indicates that hot shutdown detected */
#endif /* OTP_ENABLE */

} app_status_t;

/**
 * @typedef sys_hw_error_type_t
 * @brief List of possible hardware errors defined for the system.
 */
typedef enum {
    SYS_HW_ERROR_NONE        = 0x00,            /**< No error. */
    SYS_HW_ERROR_MUX_ACCESS  = 0x01,            /**< Error while accessing data MUX. */
    SYS_HW_ERROR_REG_ACCESS  = 0x02,            /**< Error while accessing regulator. */
    SYS_HW_ERROR_BAD_VOLTAGE = 0x04             /**< Unexpected voltage generated by source regulator. */
} sys_hw_error_type_t;

/**
 * @typedef app_thermistor_type_t
 * @brief List of possible Thermistor types that can be configured.
 */
typedef enum {
    APP_THERMISTOR_TYPE_NTC       = 0x00,            /**< NTC Thermistor type configured. */
    APP_THERMISTOR_TYPE_PTC       = 0x01,            /**< PTC Thermistor type configured. */
    APP_THERMISTOR_TYPE_INTRNL    = 0x02,            /**< CCGx internal VBJT based temperature sensing. */
    APP_THERMISTOR_TYPE_ERROR     = 0x03             /**< Invalid Thermistor type configured. */
} app_thermistor_type_t;

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief Application level init function.
 *
 * This function performs any Application level initialization required
 * for the CCG solution. This should be called before calling the
 * dpm_init function.
 *
 * @return None.
 *
 */
void app_init(void);

/**
 * @brief Check whether the specified PD port is enabled in the system configuration.
 * @param port PD port index.
 * @return true if the port is enabled, false otherwise.
 */
bool app_is_port_enabled(uint8_t port);

/**
 * @brief Handler for application level asynchronous tasks.
 * @param port USB-PD port for which tasks are to be handled.
 * @return 1 in case of success, 0 in case of task handling error.
 */
uint8_t app_task(uint8_t port);

/**
 * @brief This function return the App callback structure pointer
 * @param port port index
 * @return  Application callback structure pointer
 */
app_cbk_t* app_get_callback_ptr(uint8_t port);

/**
 * @brief Handler for event notifications from the PD stack.
 * @param port Port on which events are to be handled.
 * @param evt Type of event to be handled.
 * @param dat Data associated with the event.
 * @return None
 */
void app_event_handler (uint8_t port, app_evt_t evt, const void* dat);

/**
 * @brief Get a handle to the application provide PD command response buffer.
 * @param port PD port corresponding to the command and response.
 * @return Pointer to the response buffer.
 */
app_resp_t* app_get_resp_buf(uint8_t port);

/**
 * @brief Get handle to structure containing information about the system status for a PD port.
 * @param port PD port to be queried.
 * @return Pointer to the system information structure.
 */
app_status_t* app_get_status(uint8_t port);

/**
 * @brief Check whether the APP handlers are ready to allow device deep sleep.
 * @return true if APP handler is idle, false otherwise.
 */
bool app_sleep (void);

/**
 * @brief Restore the APP handler state after CCG device wakes from deep-sleep.
 * @return None
 */
void app_wakeup (void);

/**
 * @brief Function to place CCG device in power saving mode if possible.
 *
 * This function places the CCG device in power saving deep sleep mode
 * if possible. The function checks for each interface (PD, HPI etc.)
 * being idle and then enters sleep mode with the appropriate wake-up
 * triggers. If the device enters sleep mode, the function will only
 * return after the device has woken up.
 *
 * @return true if the device went into sleep, false otherwise.
 */
bool system_sleep(void);

/*****************************************************************************
  Functions related to power
 *****************************************************************************/

/**
 * @brief This function enables VCONN power
 *
 * @param port Port index the function is performed for.
 * @param channel Selected CC line.
 *
 * @return True if VConn was turned ON; false if NOT.
 */
bool vconn_enable(uint8_t port, uint8_t channel);

/**
 * @brief This function disables VCONN power
 *
 * @param port Port index the function is performed for.
 * @param channel Selected CC line.
 *
 * @return None
 */
void vconn_disable(uint8_t port, uint8_t channel);

/**
 * @brief This function checks if power is present on VConn
 *
 * @param port Port index the function is performed for.
 *
 * @return true if power is present on VConn, else returns false
 */
bool vconn_is_present(uint8_t port);

/**
 * @brief This function checks if power is present on VBus
 *
 * @param port Port index the function is performed for.
 * @param volt Voltage in mV units.
 * @param per  Threshold margin.
 *
 * @return true if power is present on VBus, else returns false
 */
bool vbus_is_present(uint8_t port, uint16_t volt, int8_t per);

/**
 * @brief This function return current VBUS voltage in mV
 *
 * @param port Port index the function is performed for.
 *
 * @return VBUS voltage in mV
 */
uint16_t vbus_get_value(uint8_t port);

/**
 * @brief This function turns on dischange FET on selected port
 *
 * @param port Port index the function is performed for.
 *
 * @return None
 */
void vbus_discharge_on(uint8_t port);

/**
 * @brief This function turns off dischange FET on selected port
 *
 * @param port Port index the function is performed for.
 *
 * @return None
 */
void vbus_discharge_off(uint8_t port);

/**
 * @brief This function enable vconn ocp
 * @param port Port index
 * @param cbk OCP callback
 * @return Returns true on success, false if parameters are invalid.
 */
bool system_vconn_ocp_en(uint8_t port, PD_ADC_CB_T cbk);

/**
 * @brief This function disable vconn ocp
 * @param port Port index
 * @return None
 */
void system_vconn_ocp_dis(uint8_t port);

/**
 * @brief Enable and configure the Over-Voltage protection circuitry.
 * @param port PD port to be configured.
 * @param volt_mV Expected VBus voltage.
 * @param pfet Whether PFET is used for the power supply control.
 * @param ovp_cb Callback function to be triggered when there is an OV event.
 * @return None
 */
void app_ovp_enable(uint8_t port, uint16_t volt_mV, bool pfet, PD_ADC_CB_T ovp_cb);

/**
 * @brief Disable the Over-Voltage protection circuitry.
 * @param port PD port to be configured.
 * @param pfet Whether PFET is used for the power supply control.
 * @return None
 */
void app_ovp_disable(uint8_t port, bool pfet);

/**
 * @brief Enable and configure the Under-Voltage protection circuitry.
 * @param port PD port to be configured.
 * @param volt_mV Expected VBus voltage.
 * @param pfet Whether PFET is used for the power supply control.
 * @param uvp_cb Callback function to be triggered when there is an UV event.
 * @return None
 */
void app_uvp_enable(uint8_t port, uint16_t volt_mV, bool pfet, PD_ADC_CB_T uvp_cb);

/**
 * @brief Disable the Under-Voltage protection circuitry.
 * @param port PD port to be configured.
 * @param pfet Whether PFET is used for the power supply control.
 * @return None
 */
void app_uvp_disable(uint8_t port, bool pfet);

/**
 * @brief Function to update the BC 1.2 source support.
 * @param port PD port to be configured.
 * @param enable Whether BC 1.2 support should be enabled or disabled.
 * @return None
 */
void app_update_bc_src_support(uint8_t port, uint8_t enable);

/**
 * @brief Function to update the system power state.
 * @param state Current system power state: 0=S0, Non-zero=other states.
 */
void app_update_sys_pwr_state(uint8_t state);

/**
 * @brief Wrapper function for PD port disable. This function is used to ensure that
 * any application level state associated with a faulty connection are cleared when the
 * user wants to disable a PD port.
 * @param port Index of port to be disabled.
 * @param cbk Callback to be called after operation is complete.
 * @return CCG_STAT_SUCCESS on success, appropriate error code otherwise.
 */
ccg_status_t app_disable_pd_port(uint8_t port, dpm_typec_cmd_cbk_t cbk);

/**
 * @brief Validate the configuration table specified.
 *
 * Each copy of CCGx firmware on the device flash contains an embedded
 * configuration table that defines the runtime behaviour of the CCGx device. This
 * function checks whether the configuration table located at the specified location
 * is valid (has valid offsets).
 *
 * @return true if the table is valid, false otherwise.
 */
bool app_validate_configtable_offsets(void);

/**
 * @brief Initialize CCGx periodic application level tasks.
 * @return None
 */
void ccg_app_task_init(void);

/**
 * @brief Start the simplified BC 1.2 (DCP/CDP) state machine for CCG5 device
 * on detection of a Type-C sink connection.
 *
 * @param port Type-C port index.
 * @return None
 */
void app_bc_12_sm_start(uint8_t port);

/*****************************************************************************
  Functions to be provided at the solution level.
 *****************************************************************************/

/**
 * @brief Solution handler for PD events reported from the stack.
 *
 * The function provides all PD events to the solution. For a solution
 * supporting HPI, the solution function should re-direct the calls to
 * hpi_pd_event_handler.
 *
 * @param port PD port corresponding to the event.
 * @param evt Event that is being notified.
 * @param data Data associated with the event. This is an opaque pointer that
 * needs to be de-referenced based on event type.
 *
 * @return None
 */
void sln_pd_event_handler(uint8_t port, app_evt_t evt, const void *data);

/**
 * @brief Initialize the Type-C Data Mux for a specific PD port.
 *
 * @param port USB-PD port for which the MUX is to be initialized.
 * @return Returns true if the MUX is initialized successfully, false otherwise.
 */
bool mux_ctrl_init(uint8_t port);

/**
 * @brief Set the Type-C MUX to the desired configuration.
 * @param port PD port on which MUX is to be configured.
 * @param cfg Desired MUX configuration.
 * @param polarity Polarity of the Type-C connection.
 * @return Returns true if the operation is successful, false otherwise.
 */
bool mux_ctrl_set_cfg(uint8_t port, mux_select_t cfg, uint8_t polarity);

/**
 * @brief Enable BB device enumeration on the board.
 * @param port PD port index.
 * @param polarity PD connection (plug) orientation.
 */
void mux_ctrl_bb_enable (uint8_t port, uint8_t polarity);

/**
 * @brief Initialize fault-handling related variables from the configuration table.
 * @param port PD port index.
 * @return false if the configuration table is invalid, true otherwise.
 */
bool fault_handler_init_vars (uint8_t port);

/**
 * @brief Clear the fault occurrence counts after a Type-C detach is detected.
 * @param port PD port index.
 * @return None.
 */
void fault_handler_clear_counts (uint8_t port);

/**
 * @brief Perform any fault handler related tasks.
 * @param port PD port on which the tasks are to be performed.
 * @return None
 */
void fault_handler_task (uint8_t port);

/**
 * @brief Handle any application events associated with fault handling logic.
 * @param port PD port index on which the event is received.
 * @param evt Event code.
 * @param dat Data associated with the event code.
 * @return true if the event does not need to be passed up to the solution layer.
 */
bool fault_event_handler(uint8_t port, app_evt_t evt, const void *dat);

/**
 * @brief Enable the Over-Temperature Protection logic.
 * @param port PD port index.
 * @return None.
 */
void app_otp_enable(uint8_t port);

/**
 * @brief Perform OTP check. This function is called from app task
 * @param port PD port index.
 * @return None.
 */
void app_otp_check_temp(uint8_t port);

/**
 * @brief Prepare the CCG to wait for physical detach of faulty port partner.
 * @param port PD port index.
 * @return None
 */
void app_conf_for_faulty_dev_removal(uint8_t port);

/**
 * @brief Restarts alternate mode layer.
 * @param port PD port index.
 * @return True if the alternate mode resetted, false otherwise.
 */
bool app_vdm_layer_reset(uint8_t port);

/**
 * @brief Check whether any fault count has exceeded limit for the specified PD port. 
 * @param port PD port index.
 * @return true if fault count has exceeded limit, false otherwise.
 */    
bool app_port_fault_count_exceeded(uint8_t port);

#endif /* _APP_H_ */

/* End of File */
