/**
 * @file hal_ccgx.h
 *
 * @brief @{PD and Type-C HAL layer for CCGx device family.@}
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

#ifndef _HAL_CCGX_H_
#define _HAL_CCGX_H_

#include "config.h"
#include "pd.h"
#include "stdint.h"
#include "stdbool.h"

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/** @cond DOXYGEN_HIDE */

/*
 * The following definitions are clock source IDs for various sub-blocks. These
 * are defined in hardware and should not be modified.
 */

#if (defined(CCG3) || defined(CCG4))

#define PDSS_PORT0_PCLK_RX_IDX          (8u)
#define PDSS_PORT0_PCLK_TX_IDX          (9u)
#define PDSS_PORT0_PCLK_SAR_IDX         (10u)

#ifdef CCG3
#define PDSS_PORT0_PCLK_SWAP_IDX        (11u)
#else
#define PDSS_PORT0_PCLK_SWAP_IDX        (6u)
#endif /* CCG3 */

#define PDSS_PORT1_PCLK_RX_IDX          (11u)
#define PDSS_PORT1_PCLK_TX_IDX          (12u)
#define PDSS_PORT1_PCLK_SAR_IDX         (13u)
#define PDSS_PORT1_PCLK_SWAP_IDX        (7u)

#endif /* (defined(CCG3) || defined(CCG4)) */

#if (defined(CCG3PA) || defined(CCG3PA2))
#define PDSS_PORT0_PCLK_RX_IDX          (6u)
#define PDSS_PORT0_PCLK_TX_IDX          (7u)
#define PDSS_PORT0_PCLK_SAR_IDX         (8u)
#define PDSS_PORT0_PCLK_REFGEN_IDX      (10u)
#define PDSS_PORT0_PCLK_BCH_IDX         (11u)
#define PDSS_PORT0_PCLK_FILT1_IDX       (12u)
#define PDSS_PORT0_PCLK_FILT2_IDX       (13u)
#define PDSS_PORT0_PCLK_ISINK_IDX       (14u)
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (defined(PAG1S))
#define PDSS_PORT0_PCLK_RX_IDX          (1u)
#define PDSS_PORT0_PCLK_TX_IDX          (2u)
#define PDSS_PORT0_PCLK_SAR_IDX         (3u)
#define PDSS_PORT0_PCLK_REFGEN_IDX      (6u)
#define PDSS_PORT0_PCLK_BCH_IDX         (7u)
#define PDSS_PORT0_PCLK_FILT1_IDX       (4u)
#define PDSS_PORT0_PCLK_FILT2_IDX       (5u)
#define PDSS_PORT0_PCLK_PASC_IDX        (8u)
#define PDSS_PORT0_PCLK_GDRV_IDX        (9u)
#define PDSS_PORT0_PCLK_VBTR_IDX        (10u)
#endif /* (defined(PAG1S)) */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))

/* Common clock divider ids for PD blocks on the CCG5, CCG5C and CCG6 devices. */
#define PDSS_PORT0_PCLK_RX_IDX          (6u)
#define PDSS_PORT0_PCLK_TX_IDX          (7u)
#define PDSS_PORT0_PCLK_SAR_IDX         (8u)
#define PDSS_PORT0_PCLK_SWAP_IDX        (9u)
#define PDSS_PORT0_PCLK_FILT1_IDX       (10u)
#define PDSS_PORT0_PCLK_FILT2_IDX       (11u)
#define PDSS_PORT0_PCLK_REFGEN_IDX      (12u)

#endif /* (defined (CCG5) || defined(CCG6) || defined(CCG5C)) */

#if (defined(CCG5))

/* Clock divider ids for the second PD port on dual-port CCG5 devices. */

#define PDSS_PORT1_PCLK_RX_IDX          (13u)
#define PDSS_PORT1_PCLK_TX_IDX          (14u)
#define PDSS_PORT1_PCLK_SAR_IDX         (15u)
#define PDSS_PORT1_PCLK_SWAP_IDX        (16u)
#define PDSS_PORT1_PCLK_FILT1_IDX       (17u)
#define PDSS_PORT1_PCLK_FILT2_IDX       (18u)

#endif /* (defined(CCG5)) */

#if (defined (CCG5C) || defined (CCG6))

/* Common clock divider ids for PD blocks on the CCG5C and CCG6 devices. */
#define PDSS_PORT0_PCLK_BCH_IDX         (13u)
#define PDSS_PORT0_PCLK_ISINK_IDX       (14u)

#endif /* (defined (CCG5C) || defined (CCG6)) */

/* Macros for OCP table. */
#if (defined(CCG3) || defined (CCG5))
#define MA_PER_UNIT                     (10u)
#define VSENSE_MIN                      (10)
#define VSENSE_MAX                      (65)
#endif /* (defined(CCG3) || defined (CCG5)) */

#define CCG_SILICON_REV00_VALUE         (0x11)
/**< Revision id corresponding to Rev ** version of any CCG silicon. */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))

/**< T_VCONN value corresponding to CC1/CC2 OVP detection enable. */
#define CCG5_CC_OVP_DET_ENABLE_VAL      (0x04)

#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

/** @endcond */

/*****************************************************************************
 * Data Struct Definition
 ****************************************************************************/

/**
 * @typedef vbus_ocp_cbk_t
 * @brief VBus over current protection callback function.
 */
typedef void (*vbus_ocp_cbk_t)(
        uint8_t port     /**< USB-PD port on which over-current event occurred. */
        );

/*****************************************************************************
 **************************** Function prototypes ****************************
 *****************************************************************************/

/** @cond DOXYGEN_HIDE */
/**
 * @brief Function to check CCG silicon revision.
 * @return Returns the silicon revision of the CCG part. 0 for Rev ** and non-zero for others.
 */
uint8_t ccg_get_si_revision(void);
/** @endcond */

/**
 * @brief USB-PD system initialization.
 *
 * This function initializes the PD block clocks by setting the divider values
 * for PERI registers and enabling the corresponding control registers.
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6.
 */
void system_init(void);

/**
 * @brief Enables VBus OCP checks on the specified port.
 *
 * @param port USB-PD port on which to enable VBus OCP checks.
 * @param cur Maximum current (load) expected in 10 mA units.
 * @param cbk Function to be called when OCP event is detected.
 *
 * @return Returns 1 if params are ok else return 0
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6.
 */
uint8_t system_vbus_ocp_en(uint8_t port, uint32_t cur, vbus_ocp_cbk_t cbk);

/**
 * @brief Disables VBus OCP checks on the specified port.
 *
 * @param port USB-PD port on which to disable VBus OCP checks.
 *
 * @return Returns 1 if params are ok else return 0
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6.
 */
uint8_t system_vbus_ocp_dis(uint8_t port);

/**
 * @brief Vbus OCP interrupt handler.
 *
 * @param port USB-PD port on which the OCP interrupt was triggered.
 *
 * @return None
 *
 * <B>Applicable devices:</B> CCG3, CCG4, CCG5, CCG3PA, CCG3PA2, CCG5C, CCG6.
 */
void vbus_ocp_handler(uint8_t port);

/**
 * @brief This function disconnects the OVP comparator output from OVP Trip GPIO
 * Before disconnecting, this function also make sure OVP Trip GPIO is strongly
 * driving the last output of comparator.
 *
 * @param port USB-PD port
 * @return None
 *
 * <B>Applicable devices:</B> CCG4
 */
void system_disconnect_ovp_trip(uint8_t port);

/**
 * @brief This function connects the OVP comparator output to OVP Trip GPIO.
 * @param port USB-PD port
 * @return None
 *
 * <B>Applicable devices:</B> CCG4
 */
void system_connect_ovp_trip(uint8_t port);

/**
 * @brief Enables VBus SCP checks on the specified port.
 *
 * @param port USB-PD port on which to enable VBus SCP checks.
 * @param cur Current limit for Short-Circuit detection in 10mA units.
 * @param cbk Function to be called when SCP event is detected.
 *
 * @return Returns 1 if params are ok else return 0
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, CCG6.
 */
uint8_t system_vbus_scp_en(uint8_t port, uint32_t cur, vbus_ocp_cbk_t cbk);

/**
 * @brief Disables VBus SCP checks on the specified port.
 *
 * @param port USB-PD port on which to disable VBus SCP checks.
 * @return Returns 1 if params are ok else return 0
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, CCG6.
 */
uint8_t system_vbus_scp_dis(uint8_t port);

/**
 * @brief Vbus SCP interrupt handler.
 * @param port USB-PD port on which the SCP interrupt was triggered.
 * @return None
 *
 * <B>Applicable devices:</B> CCG3PA, CCG3PA2, CCG6.
 */
void vbus_scp_handler(uint8_t port);


/**
 * @brief Enables VBus RCP checks on the specified port.
 *
 * @param port USB-PD port on which to enable VBus RCP checks.
 * @param cbk Function to be called when RCP event is detected.
 * @return Returns 1 if params are ok else return 0
 *
 * <B>Applicable devices:</B> CCG6.
 */
uint8_t system_vbus_rcp_en(uint8_t port, vbus_ocp_cbk_t cbk);

/**
 * @brief Disables VBus RCP checks on the specified port.
 * @param port USB-PD port on which to disable VBus RCP checks.
 * @return Returns 1 if params are ok else return 0
 *
 * <B>Applicable devices:</B> CCG6.
 */
uint8_t system_vbus_rcp_dis(uint8_t port);

/**
 * @brief Vbus RCP interrupt handler.
 * @param port USB-PD port on which the RCP interrupt was triggered.
 * @return None
 *
 * <B>Applicable devices:</B> CCG6.
 */
void vbus_rcp_handler(uint8_t port);

#endif /* _HAL_CCGX_H_ */

/* End of file */

