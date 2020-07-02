/**
 * @file stack_params.h
 *
 * @brief @{Header file that defines parameters to configure the CCGx Firmware
 * Stack. The current definitions for these values are optimized for the CCG3PA
 * Port Controller implementation and should not be changed.
 *
 * Please contact Cypress for details of possible customizations in these
 * settings.@}
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
#ifndef _STACK_PARAMS_H_
#define _STACK_PARAMS_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <project.h>

/*******************************************************************************
 * CCG Device Selection.
 ******************************************************************************/

/*
 * Select target device family. This definition is used to implement the PD
 * block driver.
 */
#define CCG3PA

/* Select target silicon ID for CYPD3171-24LQXQ. */
#define CCG_DEV_SILICON_ID                          (0x2003)
#define CCG_DEV_FAMILY_ID                           (0x11B0)

/* No. of USB-PD ports supported. CYPD3171-24LQXQ supports one port. */
#define NO_OF_TYPEC_PORTS                           (1u)

#define TYPEC_PORT_0_IDX                            (0u)
#define TYPEC_PORT_1_IDX                            (1u)

/*
 * Whether the system supports VCONN supply. It should be noted that when VCONN
 * support is enabled, Type-A support has to be disabled as the same lines are
 * used for control. Also, it should be noted that when VCONN supply is enabled,
 * no device should be attached to the type-A port, as it may damage the device
 * due to un-controlled Type-A VBUS voltage. In default Kit board layout, this
 * should be left as zero. The change also requires the corresponding updates to
 * the PSoC Creator schematics file.
 */
#define VCONN_SUPPORT_ENABLE                        (0u)

/* Enable TYPE-A support. */
#if (VCONN_SUPPORT_ENABLE)
#define CCG_TYPE_A_PORT_ENABLE                      (0u)
#else
#define CCG_TYPE_A_PORT_ENABLE                      (1u)
#endif

#if CCG_TYPE_A_PORT_ENABLE

/* TYPE-A port ID. */
#define TYPE_A_PORT_ID                              (1u)

/* Dual regulator support for TYPE-A VBUS. */
#define TYPE_A_DUAL_REG_ENABLE                      (1u)

#endif /* CCG_TYPE_A_PORT_ENABLE */

#if (NO_OF_TYPEC_PORTS >= 2) || (CCG_TYPE_A_PORT_ENABLE == 1)
    /* Set this flag to enable the second PD port. */
    #define CCG_PD_DUALPORT_ENABLE                  (1u)
#else
    #define CCG_PD_DUALPORT_ENABLE                  (0u)
#endif

/*******************************************************************************
 * Solution workarounds.
 ******************************************************************************/
/*
 * FET control lines for producer and consumer may be reversed or used differently
 * for certain packages or applications.
 */
#define CCG_FLIPPED_FET_CTRL                        (0u)
#define CCG_SRC_FET                                 (!CCG_FLIPPED_FET_CTRL)
#define CCG_SNK_FET                                 (CCG_FLIPPED_FET_CTRL)

/*
 * Disable Pseudo-metadata handling in flashing sequence.
 * This definition should be left enabled for CCG4 solutions.
 */
#define CCG_PSEUDO_METADATA_DISABLE                 (1u)

/*******************************************************************************
 * VBus Control Type
 ******************************************************************************/

#define VBUS_CTRL_NONE                              (0)

/* VBUS Control using PWM. */
#define VBUS_CTRL_PWM                               (1)

/* VBUS Control using Direct Feedback. */
#define VBUS_CTRL_DIR_FB                            (2)

/* VBUS Control using Opto Feedback. */
#define VBUS_CTRL_OPTO_FB                           (3)

/* This is the VBUS Control type used by the application for P1. */
#define VBUS_CTRL_TYPE_P1                           (VBUS_CTRL_DIR_FB)

/* This is the VBUS Control type used by the application for P2 (TYPE-A port). */
#define VBUS_CTRL_TYPE_P2                           (VBUS_CTRL_PWM)

/* Enable Internal UVP Comparator to be used as VBUS divider. */
#define VBUS_MON_INTERNAL                           (1u)

/*******************************************************************************
 * High level firmware feature selection.
 ******************************************************************************/

/*
 * Enables solution space usage of source only PD functionality.
 * Should be set to true only if source only PD library is included.
 */
#define CCG_SOURCE_ONLY                             (1u)

/* Enable hardware based DRP toggle for additional power saving. */
#define CCG_HW_DRP_TOGGLE_ENABLE                    (1u)

/* Enabling flashing of the device via PD interface. */
#define FLASHING_MODE_PD_ENABLE                     (1u)

/* Single firmware Image */
#define CCG_DUALAPP_DISABLE                         (1u)

/*******************************************************************************
 * Timer Module Configuration
 ******************************************************************************/

/*
 * The timer module provides software timer support. It provides multiple
 * timers running off a single hardware timer and has a general accuracy of 5%.
 * This module can generate callbacks at a granularity of 1ms. It provides
 * various backend implementations selectable via the following compile time
 * options. The various options can be selected by selecting the required value
 * for implemenation macro TIMER_TYPE.
 */

/*
 * SYS_TICK based timer backend, interrupting the system every 1ms.
 * This implementation requires the SYS_TICK timer to be reserved for the use
 * of this module. This implementation shall not function in DEEP_SLEEP mode
 * and the user should ensure that the timer is shut-off before entering
 * DEEP_SLEEP. To shut off the timer, just ensure that all soft-timers are
 * stopped or have expired.
 */
#define TIMER_TYPE_SYSTICK                          (1)

/*
 * WDT based timer backend.
 * This requires user to reserve both WDT and SYS_TICK timers for the use of this.
 * The WDT timer runs off ILO which is highly inaccurate. The SYS_TICK timer is
 * used to calibrate the WDT to match IMO accuracy. The WDT based
 * implementation works across DEEP_SLEEP.
 */
#define TIMER_TYPE_WDT                              (2)

/*
 * Timer implementation selection.
 * TIMER_TYPE_WDT should be used if deep sleep entry for power saving is
 * being used.
 */
#define TIMER_TYPE                                  (TIMER_TYPE_WDT)

/*
 * In addition to the hardware timer options, the module also provides a
 * TICKLESS timer implementation. The TICKLESS implementation is currently
 * available only for WDT based timer backend. The TICKLESS timer interrupts
 * the system only at the timer expiry (instead of every 1ms). Since this
 * involves a more complex algorithm, it requires more code space (about 200
 * bytes more). This implementation allows the same timer to be used in ACTIVE
 * as well as DEEP_SLEEP modes, due to the WDT backend. It also gives maximum
 * power savings as well as faster execution due to less number of interrupts.
 */
#define TIMER_TICKLESS_ENABLE                       (1)

/*
 * Timer module supports multiple software instances based on a single hardware
 * timer. The number of instances is defined based on the PD port count.
 */
#if CCG_TYPE_A_PORT_ENABLE
#define TIMER_NUM_INSTANCES                         (2)
#else
#define TIMER_NUM_INSTANCES                         (NO_OF_TYPEC_PORTS)
#endif /* CCG_TYPE_A_PORT_ENABLE */

/*******************************************************************************
 * ADC selection for various functions.
 ******************************************************************************/
#define APP_VBUS_POLL_ADC_ID                        (PD_ADC_ID_0)
#define APP_VBUS_POLL_ADC_INPUT                     (PD_ADC_INPUT_AMUX_B)

/*******************************************************************************
 * Types of Data path switches supported.
 ******************************************************************************/

/* CCG controlled switch for DisplayPort and USB lines. */
#define DP_MUX                                      (0u)

/* Data path switching handled by Alpine Ridge. */
#define RIDGE_MUX                                   (1u)

/* This firmware supports only CCG controlled DP/USB switch operation. */
#define MUX_TYPE                                    (3u)

/******************************************************************************/

#endif /* _STACK_PARAMS_H_ */

/* End of file */
