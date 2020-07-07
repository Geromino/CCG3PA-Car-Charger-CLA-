/**
 * @file config.h
 *
 * @brief @{Header file that enables/disables various CCG firmware features.
 *
 * This file also provides mapping to the implementation for hardware dependent
 * functions like FET control, voltage selection etc.
 *
 * This current implementation matches the CY4531 EVK from Cypress. This can be
 * updated by users to match their hardware implementation.@}
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
#ifndef _CONFIG_H_
#define _CONFIG_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <project.h>
#include <stack_params.h>
#include <vbus_ctrl.h>

/*******************************************************************************
 * Enable PD spec Rev 3 support
 ******************************************************************************/
#define CCG_PD_REV3_ENABLE                              (1u)

#if CCG_PD_REV3_ENABLE
    #define CCG_FRS_RX_ENABLE                           (0u)
    #define CCG_FRS_TX_ENABLE                           (0u)
    #define CCG_PPS_SRC_ENABLE                          (1u)
#endif /* CCG_PD_REV3_ENABLE */

#define CCG_PROG_SOURCE_ENABLE                          (1u)

#if CCG_PPS_SRC_ENABLE
    /*
     * Enable / disable CF mode operation. CF mode operation must be enabled
     * for supporting PPS.
     */    
    #define VBUS_CF_EN                                  (1u)
    /*
     * Enable/disable workaround for supporting bad sink devices
     * which misbehaves on seeing a PPS PDO.
     */    
    #define CCG_REV3_HANDLE_BAD_SINK                    (1u)
#endif /* CCG_PPS_SRC_ENABLE */

/*
 * Macro defines additional delay in milliseconds before the PD stack starts sending
 * SRC_CAP message. This may be required to work with some non-compliant sink devices
 * which require more start up time for PD.
 */
#define DELAY_SRC_CAP_START_MS                      (100u)

/*******************************************************************************
 * PSOURCE controls for PD port 1.
 ******************************************************************************/
#define REGULATOR_REQUIRE_STABLE_ON_TIME           (1)
#define REGULATOR_ENABLE()                          BUCK_BOOST_EN_C_Write(0)
#define REGULATOR_DISABLE()                         BUCK_BOOST_EN_C_Write(1)

#define REGULATOR_STATUS()                          (!BUCK_BOOST_EN_C_Read())

#define REGULATOR_TURN_ON_DELAY                     (50)

#define APP_VBUS_SRC_FET_ON_P1()                    \
{                                                   \
    pd_internal_pfet_on(0, false);                  \
    BUCK_BOOST_EN_C_Write(0);                       \
}

#define APP_VBUS_SET_VOLT_P1(volt_mV)             vbus_ctrl_fb_set_volt(TYPEC_PORT_0_IDX, volt_mV)

#define APP_VBUS_SRC_FET_OFF_P1()                   \
{                                                   \
    BUCK_BOOST_EN_C_Write(1);                       \
    pd_internal_pfet_off(0, false);                 \
}

/* Function/Macro to turn VBUS_DISCHARGE FET for P1 ON. */
#define APP_DISCHARGE_FET_ON_P1()                   pd_internal_vbus_discharge_on(0)

/* Function/Macro to turn VBUS_DISCHARGE FET for P1 OFF. */
#define APP_DISCHARGE_FET_OFF_P1()                  pd_internal_vbus_discharge_off(0)

/*******************************************************************************
 * Power Sink (PSINK) controls for PD port 1.
 ******************************************************************************/

/* Function/Macro to turn consumer FET for P1 ON. */
#define APP_VBUS_SNK_FET_ON_P1()                   \
{                                                  \
}

/* Function/Macro to turn consumer FET for P1 OFF. */
#define APP_VBUS_SNK_FET_OFF_P1()                   \
{                                                   \
}

/*******************************************************************************
 * PSOURCE controls for PD port 2.
 ******************************************************************************/

#define APP_VBUS_SRC_FET_ON_P2()                    vbus_ctrl_pwm_turn_on(TYPEC_PORT_1_IDX)

#define APP_VBUS_SET_VOLT_P2(volt_mV)             vbus_ctrl_pwm_set_volt(TYPEC_PORT_1_IDX, volt_mV)

#define APP_VBUS_SRC_FET_OFF_P2()                   vbus_ctrl_pwm_turn_off(TYPEC_PORT_1_IDX)

/* Function/Macro to turn VBUS_DISCHARGE FET for P1 ON. */
#define APP_DISCHARGE_FET_ON_P2()                   ((void)0)

/* Function/Macro to turn VBUS_DISCHARGE FET for P2 OFF. */
#define APP_DISCHARGE_FET_OFF_P2()                  ((void)0)

/*******************************************************************************
 * Power Sink (PSINK) controls for PD port 2.
 ******************************************************************************/

/* Function/Macro to turn consumer FET for P1 ON. */
#define APP_VBUS_SNK_FET_ON_P2()                   \
{                                                  \
}

/* Function/Macro to turn consumer FET for P1 OFF. */
#define APP_VBUS_SNK_FET_OFF_P2()                   \
{                                                   \
}

/*******************************************************************************
 * Power Source (PSOURCE) Configuration.
 ******************************************************************************/

/* 
 * VBUS_IN voltage is monitored during FET ON.
 * If VBUS_IN voltage is more than safe range (VSAFE_5V +  OVP threshold),
 * then a discharge path is enabled instead of FET ON.
 */
#define APP_PSOURCE_SAFE_FET_ON_ENABLE              (1u)

/* Time (in ms) allowed to wait for VBUS_IN discharge for safe FET ON */
#define APP_PSOURCE_SAFE_FET_ON_TIMER_PERIOD        (50u)

/* Time (in ms) allowed for source voltage to become valid. */
#define APP_PSOURCE_EN_TIMER_PERIOD                 (250u)

/* 
 * Period (in ms) between every step change of vbus. This is not used for 
 * this device family.
 */
#define APP_PSOURCE_EN_MONITOR_TIMER_PERIOD         (1u)

/* Time (in ms) between VBus valid and triggering of PS_RDY. */
#define APP_PSOURCE_EN_HYS_TIMER_PERIOD             (5u)

/*
 * Time (in ms) for which the VBus_Discharge path will be enabled when turning
 * power source OFF.
 */
#define APP_PSOURCE_DIS_TIMER_PERIOD                (600u)

/* Period (in ms) of VBus drop to VSAFE0 checks after power source is turned OFF. */
#define APP_PSOURCE_DIS_MONITOR_TIMER_PERIOD        (1u)

/*
 * Maximum voltage step in mV transition that can happen in one change when
 * voltage is rising above 5V.
 */
#define VBUS_CTRL_ABOVE_5V_MAX_STEP                 (20000u)

/*
 * Maximum voltage step in mV transition that can happen in one change when
 * voltage is falling below 5V.
 */
#define VBUS_CTRL_BELOW_5V_MAX_STEP                 (5000u)

/*
 * Enable this macro if the interval between steps need to be done in micro-
 * seconds. This may be required if the regulator is very sensitive to feedback
 * node change. This will allow for smaller feedback voltage step size. When
 * enabling this feature, a TCPWM timer should be added with TC interrupt 
 * configured for one-shot timer interrupting with the required interval as
 * period. The minimum interval allowed is 200us. Use this only if you need step
 * interval between 200us and 1ms. If this is disabled, the step size is assumed
 * to be 1ms.
 *
 * Most direct feedback based regulators are fast and does not require this. So
 * this is left disabled by default.
 */
#define VBUS_CTRL_STEP_US_ENABLE                    (0u)

/* 
 * Period (in ms) to debounce the VBUS stable detection. VBUS change is detected
 * using ADC over the specified duration.
 */
#define VBUS_CTRL_SLOPE_DEBOUNCE_PERIOD             (5u)

/* 
 * Minimum period (in ms) for vbus to settle after reaching stable slope. This
 * is imposed to ensure that we provide a minimum time before indicating ready.
 */
#define VBUS_CTRL_SETTLE_TIME_PERIOD                (15u)

/*
 * Enable this macro to allow the VBUS voltage to be corrected based on ADC
 * readings. This allows for better accuracy in direct feedback systems. This
 * should not be enabled for opto-isolator designs.
 */
#define VBUS_CTRL_ADC_ADJUST_ENABLE                 (1u)

/*
 * Enable / disable VBUS Slow Discharge Feature. When this feature is enabled,
 * the discharge drive strength shall be increased by steps every ms until the
 * selected top drive strength is achieved. Similarly, the drive strength is
 * decreased in steps while stopping the discharge.
 */
#define VBUS_SLOW_DISCHARGE_EN                      (1u)

/* VBUS in discharge enable */
#define VBUS_IN_DISCHARGE_EN                        (1u)

/* VBUS discharge drive strength settings. */
#if VBUS_IN_DISCHARGE_EN
#define VBUS_C_DISCHG_DS                            (4u)
#else /* !VBUS_IN_DISCHARGE_EN */
#define VBUS_C_DISCHG_DS                            (8u)
#endif
#define VBUS_IN_DISCHG_DS                           (4u)

/*******************************************************************************
 * VBus monitor configuration.
 ******************************************************************************/

/* Allowed VBus valid margin as percentage of expected voltage. */
#define VBUS_TURN_ON_MARGIN                         (-20)

/* Allowed VBus valid margin (as percentage of expected voltage) before detach detection is triggered. */
#define VBUS_TURN_OFF_MARGIN                        (-20)

/* Allowed margin over expected voltage (as percentage) for negative VBus voltage transitions. */
#define VBUS_DISCHARGE_MARGIN                       (20)

/* Allowed margin over 5V before the provider FET is turned OFF when discharging to VSAFE0. */
#define VBUS_DISCHARGE_TO_5V_MARGIN                 (10)

/*******************************************************************************
 * VBus Monitor connection configuration for Port 1.
 ******************************************************************************/

/* CCG IO port to which the VBUS_MON_P1 pin corresponds. This is taken from the PSoC Creator Schematic. */
#define APP_VBUS_MON_PORT_NO_P1                     (VBUS_MON_P1__PORT)

/* CCG IO pin to which the VBUS_MON_P1 pin corresponds. This is taken from the PSoC Creator Schematic. */
#define APP_VBUS_MON_PIN_NO_P1                      (VBUS_MON_P1__SHIFT)

/* Combined Port+Pin representation for the VBUS_MON_P1 pin. */
#define APP_VBUS_MON_PORT_PIN_P1                    ((VBUS_MON_P1__PORT << 4) | VBUS_MON_P1__SHIFT)

/*
 * IO setting to connect VBUS_MON_P1 to an internal comparator. This should be selected from:
 * a) HSIOM_MODE_AMUXA = 6, AMUXBUS A connection
 * b) HSIOM_MODE_AMUXB = 7, AMUXBUS B connection
 */
#define APP_VBUS_MON_AMUX_INPUT_P1                  (6)

/*******************************************************************************
 * VBus Monitoring Controls for detach detection.
 ******************************************************************************/

/* Division factor applied between VBus and the voltage on VBUS_MON input. */
#define VBUS_MON_DIVIDER                            (11u)

/*******************************************************************************
 * VBus OCP fault GPIO connection configuration.
 ******************************************************************************/

/* CCG port to which the OCP_FAULT_P1 pin corresponds. This is derived from the PSoC Creator Schematic. */
#define APP_VBUS_OCP_FAULT_PORT_NO_P1               (OCP_FAULT_P1__PORT)

/* CCG pin to which the OCP_FAULT_P1 pin corresponds. This is derived from the PSoC Creator Schematic. */
#define APP_VBUS_OCP_FAULT_PIN_NO_P1                (OCP_FAULT_P1__SHIFT)

/* Combined Port+Pin representation for the OCP_FAULT_P1 pin. */
#define APP_VBUS_OCP_PORT_PIN_P1                    ((OCP_FAULT_P1__PORT << 4) | OCP_FAULT_P1__SHIFT)

/*******************************************************************************
 * VBUS_OVP_TRIP GPIO connection configuration.
 ******************************************************************************/

/* CCG port to which the VBUS_OVP_TRIP_P1 pin corresponds. */
#define APP_VBUS_OVP_TRIP_PORT_NO_P1                (VBUS_OVP_TRIP_P1__PORT)

/* CCG pin to which the VBUS_OVP_TRIP_P1 pin corresponds. */
#define APP_VBUS_OVP_TRIP_PIN_NO_P1                 (VBUS_OVP_TRIP_P1__SHIFT)

/* Combined Port+Pin representation of the VBUS_OVP_TRIP_P1 pin. */
#define APP_VBUS_OVP_TRIP_PORT_PIN_P1               ((VBUS_OVP_TRIP_P1__PORT << 4) | VBUS_OVP_TRIP_P1__SHIFT)

/* CCG IO mode corresponding to the VBUS_OVP_TRIP functionality. This should be set to 12. */
#define APP_VBUS_OVP_TRIP_HSIOM_P1                  (12)

/*******************************************************************************
 * VConn Monitor connection configuration for Port 1.
 * This section is optional as VConn monitoring is not enabled in the stack.
 ******************************************************************************/

/* CCG IO port to which the VCONN_MON_P1 pin corresponds. This is taken from the PSoC Creator Schematic. */
#define APP_VCONN_MON_PORT_NO_P1                    (VCONN_MON_P1__PORT)

/* CCG IO pin to which the VCONN_MON_P1 pin corresponds. This is taken from the PSoC Creator Schematic. */
#define APP_VCONN_MON_PIN_NO_P1                     (VCONN_MON_P1__SHIFT)

/* Combined Port+Pin representation for the VCONN_MON_P1 pin. */
#define APP_VCONN_MON_PORT_PIN_P1                   ((VCONN_MON_P1__PORT << 4) | VCONN_MON_P1__SHIFT)

/*
 * IO setting to connect VCONN_MON_P1 to an internal comparator. This should be selected from:
 * a) HSIOM_MODE_AMUXA = 6, AMUXBUS A connection
 * b) HSIOM_MODE_AMUXB = 7, AMUXBUS B connection
 */
#define APP_VCONN_MON_AMUX_INPUT_P1                 (7)

/*******************************************************************************
 * VBUS offset voltage configuration.
 *
 * Offset voltage value is a configuration table parameter.
 ******************************************************************************/
 /* VBUS offset voltage enable setting. */
#define VBUS_OFFSET_VOLTAGE_ENABLE                  (1u)

/*******************************************************************************
 * Cable compensation feature configuration.
 ******************************************************************************/

/*
 * The following macro enables cable compensation logic. The feature is 
 * applicable only for Type-C port of CCG3PA/CCG3PA2 designs which uses the
 * internal feedback based voltage control.
 */
#define CCG_CABLE_COMP_ENABLE                       (1u)

/* 
 * The following macro disables the cable compensation logic for
 * PPS contracts.
 */
#define CCG_CABLE_COMP_IN_PPS_DISABLE               (1u)

/*******************************************************************************
 * Battery Charging Support Configuration
 ******************************************************************************/

#define  BATTERY_CHARGING_ENABLE                   (1u)
#if CCG_TYPE_A_PORT_ENABLE
#define  NO_OF_BC_PORTS                            (2u)
#else /* !CCG_TYPE_A_PORT_ENABLE */
#define  NO_OF_BC_PORTS                            (1u)
#endif /* CCG_TYPE_A_PORT_ENABLE */
#define  BC_PORT_0_IDX                             (0u)
#define  BC_PORT_1_IDX                             (1u)

#if VBUS_CF_EN
/* Enable/Disable Current Foldback in QC mode */
#define QC_CF_EN                                   (1u)    
#endif /* VBUS_CF_EN */

#if BATTERY_CHARGING_ENABLE
/* Enable this option if PD and legacy state machines shall run in parallel. */
#define LEGACY_PD_PARALLEL_OPER                    (1u)

/*
 * Enable / disable external Apple source termination via external resistor 
 * combination.
 *
 * This is useful only when legacy BC 1.2 and Apple source protocols need to 
 * co-exist. This compile time option also requires the solution to implement 
 * external Apple 2.4A terminations. The sln_apply_apple_src_term() and 
 * sln_remove_apple_src_term() functions shall be implemented to match the
 * solution hardware and requirement.
 *
 * This macro is port independant. If only one port needs support for external 
 * termination, even then the solution specific functions need to be implemented.
 *
 * To support parallel detection logic, an external resistance(s) and a dedicated
 * GPIO per port is required. Each port can be configured differently. If the 
 * specific port does not have external resistance connected, then it should not
 * be configured for parallel operation. In this case, the solution handler can
 * invoke the HAL function directly to use the internal terminations. Refer to
 * description of sln_apply_apple_src_term() function for details of hardware 
 * connection and usage.
 */
#define LEGACY_APPLE_SRC_SLN_TERM_ENABLE            (0u)
#endif /* BATTERY_CHARGING_ENABLE */

#if (QC_CF_EN || LEGACY_DYN_CFG_ENABLE)
/* Defines the max PDP value when PD3.0 is not supported */    
#define CCG_MAX_PDP_VALUE                          (36u)
/* Defines the max cable rating when connected to a legacy device. Units of 10mA */    
#define LEGACY_MAX_CABLE_RATING                    (300u)
#endif /* (QC_CF_EN || LEGACY_DYN_CFG_ENABLE) */

/*******************************************************************************
 * VBus Over-Voltage Protection Configuration.
 *
 * The VBus OVP feature uses an internal ADC in the CCG to measure the voltage
 * on the VBUS_MON input and uses the ADC output to detect over-voltage
 * conditions.
 *
 * The default implementation of OVP uses firmware ISRs to turn off the FETs
 * when OVP is detected. If quicker response is desired, there is the option of
 * using a direct OVP_TRIP output derived from a hardware comparator associated
 * with the ADC.
 ******************************************************************************/

/* VBus OVP enable setting. */
#define VBUS_OVP_ENABLE                             (1u)

/*
 * OVP mode selection
 * 0 - OVP using ADC comparator.
 * 1 - OVP using dedicated comparator. Firmware detects trip interrupt and turns off the FETs.
 * 2 - OVP using dedicated comparator. Hardware detects trip interrupt and turns off the FETs.
 */
#define VBUS_OVP_MODE                               (2u)

/*******************************************************************************
 * VBus Under-Voltage Protection Configuration.
 ******************************************************************************/

/* VBus UVP enable setting. */
#define VBUS_UVP_ENABLE                             (1u)

/*
 * UVP mode selection
 * 0 - UVP using ADC comparator.
 * 1 - UVP using dedicated comparator. Firmware detects trip interrupt and turns off the FETs.
 * 2 - UVP using dedicated comparator. Hardware detects trip interrupt and turns off the FETs.
 */
#define VBUS_UVP_MODE                               (2u)

/*******************************************************************************
 * VBus Over-Current Protection Configuration.
 *
 * The VBus OCP feature is implemented based on an external load switch and
 * only uses a fault indicator GPIO coming into the CCG device.
 ******************************************************************************/

/*
 * VBus OCP feature enable. This can be enabled on CY4531 and on other boards
 * that have the load switch.
 */
#define VBUS_OCP_ENABLE                             (1u)

/*
 * Software OCP mode.
 * 0 - External OCP hardware.
 * 1 - Internal OCP with neither software debounce nor automatic FET control.
 * 2 - Internal OCP with automatic FET control by hardware when an OCP event is
 *     detected.
 * 3 - Internal OCP with software debounce using delay in milliseconds from the
 *     config table.
 */
#define VBUS_OCP_MODE                               (3u)

/*******************************************************************************
 * VBus Short Circuit Protection Configuration.
 ******************************************************************************/

/*
 * VBus SCP feature enable.
 */
#define VBUS_SCP_ENABLE                             (1u)

/* SCP mode: AUTO Control. */
#define VBUS_SCP_MODE                               (2u)

/*******************************************************************************
 * VConn Over-Current Protection Configuration.
 *
 * The VConn OCP feature is implemented based on an external load switch and
 * only uses a fault indicator GPIO coming into the CCG device.
 ******************************************************************************/
/* VConn OCP enable setting. */
#define VCONN_OCP_ENABLE                            (0u)

/*******************************************************************************
 * System fault configuration features.
 ******************************************************************************/

/* Enable watchdog hardware reset for CPU lock-up recovery */
#define WATCHDOG_HARDWARE_RESET_ENABLE              (1u)

/* Disable CCG device reset on error (watchdog expiry or hard fault). */
#define RESET_ON_ERROR_ENABLE                       (1u)

/* Enable reset reporting through HPI. */
#define HPI_WATCHDOG_RESET_ENABLE                   (0u)

/* Watchdog reset timer id. */
#define WATCHDOG_TIMER_ID                           (0xC2u)

/*
 * Watchdog reset period in ms. This should be set to a value greater than
 * 500 ms to avoid significant increase in power consumption.
 */
#define WATCHDOG_RESET_PERIOD_MS                    (750u)

/* Enable tracking of maximum stack usage. */
#define STACK_USAGE_CHECK_ENABLE                    (0u)

/*******************************************************************************
 * Firmware feature configuration.
 ******************************************************************************/

/* Set to 1 if building a debug enabled binary with no boot-loader dependency. */
#define CCG_FIRMWARE_APP_ONLY                       (0u)

/* Enable CCG deep sleep to save power. */
#define SYS_DEEPSLEEP_ENABLE                        (1u)

/* Enable Alternate Mode support when CCG is DFP. */
#define DFP_ALT_MODE_SUPP                           (0u)

/* Enable DisplayPort Source support as DFP. */
#define DP_DFP_SUPP                                 (0u)

/* Enable Alt mode as UFP */
#define UFP_ALT_MODE_SUPP                           (0u)

/* Enable saving only SVIDs which are supported by CCG. */
#define SAVE_SUPP_SVID_ONLY                         (1u)

/* Enable HPI support. */
#define CCG_HPI_ENABLE                              (0u)

/* Enable PD policy registers in HPI. */
#define CCG_HPI_PD_ENABLE                           (0u)

/*
 * Index of SCB used for HPI interface. This should be set based on
 * the pin selection in the project schematic.
 */
#define HPI_SCB_INDEX                               (2u)

/* Enable image selection based on APP Priority Field. */
#define APP_PRIORITY_FEATURE_ENABLE                 (0u)

/*
 * Enable/Disable firmware active LED operation.
 *
 * The blinking LED is enabled by default but it is recommended to disable it
 * for production designs to save power.
 */
#define APP_FW_LED_ENABLE                           (1u)

/*
 * Select CCG3 GPIO to be used as Activity Indication. This should be set to a
 * valid value if APP_FW_LED_ENABLE is non-zero.
 */
#define FW_LED_GPIO_PORT_PIN                            (GPIO_PORT_2_PIN_3)
#define APP_DEBUG_DPM_DEVICE_TOBECHARGED                (0u)
#define APP_DEBUG_RTDP_DEVICE_TOBECHARGED               (0u)
#define APP_DEBUG_SDK_INTERFACE_DPM_VOLTAGE_CURRENT     (1u)
#define APP_DEBUG_SDK_INTERFACE_DETECTION_CONNECT       (0u)

/*
 * Timer ID allocation for various solution soft timers.
 */

/*
 * Activity indicator LED timer. The timer is used to indicate that the firmware
 * is functional. The feature is controlled by APP_FW_LED_ENABLE.
 */
#define LED_TIMER_ID                                (0xC0)
/*
 * The LED toggle period.
 */
#define LED_TIMER_PERIOD                            (1000)

/* Timer used to ensure I2C transfers to the MUX complete on time. */
#define MUX_I2C_TIMER                               (0xC1)
/* The MUX transfer timeout is set to 10 ms timeout period. */
#define MUX_I2C_TIMER_PERIOD                        (10u)

/* QC 4.0 support. */
#define QC_4_0_ENABLE                               (1)

#if QC_4_0_ENABLE
#define QC_4_0_SVID                                 (0x05C6)
#endif /* QC_4_0_ENABLE */

/***********************************************************************************/

#endif /* _CONFIG_H_ */

/* End of file */
