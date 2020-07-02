/**
 * @file psource.h
 *
 * @brief @{Power source (Provider) manager header file.@}
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

#ifndef _PSOURCE_H_
#define _PSOURCE_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <stdint.h>
#include <pd.h>
#include <pdss_hal.h>

#define PPS_CF_VBUS_DECREMENT_STEP     (200u)   /**< VBUS decrement value on each Current foldback event */

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief This function sets the VBus source voltage to the desired value. It
 * also updates the voltage thresholds associated with protection schemes such
 * as OVP and UVP.
 *
 * @param port Port index the function is performed for.
 * @param volt_mV Voltage in mV units.
 *
 * @return None
 */
void psrc_set_voltage (uint8_t port, uint16_t volt_mV);

/**
 * @brief This function gets the VBus source voltage that is currently
 * configured. This is different from the vbus_get_value() function which
 * measures the actual VBus voltage.
 *
 * @param port Port index the function is performed for.
 * @return Voltage in mV units.
 */
uint32_t psrc_get_voltage (uint8_t port);

/**
 * @brief This function sets the VBus source current limit. The current limits
 * are used to configure the current sensing circuits to trigger fault indication
 * in case of overload.
 *
 * @param port Port index the function is performed for.
 * @param cur_10mA Current in 10mA units.
 * @return None
 */
void psrc_set_current (uint8_t port, uint16_t cur_10mA);

/**
 * @brief This function enables the VBus power supply. The voltage and current
 * to be supplied would have been specified through the psrc_set_voltage() and
 * psrc_set_current() calls before the supply is enabled. The function returns
 * as soon as the supply enable operation has been started. The pwr_ready_handler
 * is expected to be called once VBus voltage has stabilized at the desired level.
 *
 * @param port Port index the function is performed for.
 * @param pwr_ready_handler Application handler callback function.
 * @return None
 */
void psrc_enable (uint8_t port, pwr_ready_cbk_t pwr_ready_handler);

/**
 * @brief This function disables the VBus power supply. If a non-NULL
 * pwr_ready_handler callback is specified, the function can return after
 * starting the VBus disable operation. The callback will be called once
 * the VBus voltage has been safely brought to vSafe0V. If the callback
 * is NULL, the function is expected to return after shutting down the supply
 * without initiating any VBus discharge sequence.
 *
 * @param port Port index the function is performed for.
 * @param pwr_ready_handler Application handler callback function.
 *
 * @return None
 */
void psrc_disable (uint8_t port, pwr_ready_cbk_t pwr_ready_handler);

#endif /* _PSOURCE_H_ */

/* End of File */

