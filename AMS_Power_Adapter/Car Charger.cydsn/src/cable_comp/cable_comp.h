/**
 * @file cable_comp.h
 *
 * @brief @{Cable compensation header file.@}
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

#ifndef _CABLE_COMP_H_
#define _CABLE_COMP_H_

#if CCG_CABLE_COMP_ENABLE

/**************************************************************************************************
 ****************************************** DATA TYPES ********************************************
 *************************************************************************************************/
 /**
 * @brief Vbus current change callback prototype
 * 
 * This is the function prototype for the Vbus current change callback.
 * Callback can be registered with the cable compensation through 
 * 'cable_comp_enable' function and the callback handler will be called on every
 * comparator interrupt for the changing Vbus current level.
 *
 * @param port Port index.
 * @param vbus_cur Latest Vbus current in 10mA units
 *
 * @return None
 */
typedef void (*cable_comp_vbus_cur_cbk)(uint8_t port, uint16_t vbus_cur);

/**************************************************************************************************
 *********************************FUNCTION PROTOTYPES**********************************************
 *************************************************************************************************/

/**
 * @brief Enable the cable compensation module.
 * 
 * This function enables the cable compensation module as required. Cable
 * compensation across controllers will not be supported, if not enabled.
 *
 * @param port Port index.
 * @param vbus_cur_cbk Callback handler for Vbus current change,
 * refer to 'cable_comp_vbus_cur_cbk'.
 *
 * @return None
 */
void cable_comp_enable(uint8_t port, cable_comp_vbus_cur_cbk vbus_cur_cbk);

/**
 * @brief Disable the cable compensation module.
 * 
 * This function disables the cable compensation module.
 *
 * @param port Port index.
 *
 * @return None
 */
void cable_comp_disable(uint8_t port);

/**
 * @brief Return enable status of the cable compensation module.
 * 
 * This function returns the enable or disable status of the cable compensation
 * module.
 *
 * @param port Port index.
 *
 * @return True if cable compensation is enabled. False otherwise.
 */
bool cable_comp_is_enabled(uint8_t port);

/**
 * @brief Pause the cable compensation module.
 * 
 * This function pauses the cable compensation module by turning off both SR
 * and PFC comparator interrupts.
 *
 * @param port Port index.
 *
 * @return None
 */
void cable_comp_pause(uint8_t port);

/**
 * @brief Resume the cable compensation module.
 * 
 * This function resumes the cable compensation module by turning on both SR
 * and PFC comparator interrupts for the calculated hysteresis current levels.
 *
 * @param port Port index.
 *
 * @return None
 */
void cable_comp_resume(uint8_t port);

/**
 * @brief Configure the cable compensation.
 * 
 * This function enables cable compensation if required, pauses any running
 * compensation and then calculates the required voltage compensation by
 * measuring the current. The function does not resume the compensation as
 * the voltage is not yet setup.
 * NOTE: Comparator configuration to ISR exit must be less than the
 * comprator interrupt filtering time to avoid spurious interrupts.
 * The filtering time = Filter clock perod * 32. Considering the
 * fastest clock period of 1us, this has to be less than 32us. To ensure
 * this, all calculations and configurations must be done before this
 * and comparator configuration should be done last.
 *
 * @param port Port index.
 * @param volt_mV Current Vbus voltage in mVolts.
 *
 * @return None
 */
void cable_comp_cfg(uint8_t port, uint16_t volt_mV);

/**
 * @brief Get the cable compensation voltage.
 * 
 * This function returns the cable compensation voltage to compensate the
 * voltage drop across the cable for the latest current load on the Vbus. 
 *
 * @param port Port index.
 *
 * @return Cable compensation voltage in mVolts.
 */
uint32_t cable_comp_get_voltage(uint8_t port);

/**
 * @brief Function runs the cable compensation module task routine for the
 * specified port.
 *
 * @param port Port index.
 *
 * @return None.
 */
void ccg_cable_comp_task(uint8_t port);

/**
 * @brief Facilitates to know if cable compensation module is idle. 
 * System sleep is allowed in idle state only. In idle state,
 * ccg_cable_comp_task() can run in lower frequency.
 *
 * @param port Port index.
 *
 * @return True if cable compensation module is idle. False otherwise.
 */
uint8_t ccg_cable_comp_is_idle(uint8_t port);

#endif /* CCG_CABLE_COMP_ENABLE */

#endif /* _CABLE_COMP_H_ */

/* [] END OF FILE */
