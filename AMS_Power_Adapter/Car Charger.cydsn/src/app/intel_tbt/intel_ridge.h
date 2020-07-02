/**
 * @file intel_ridge.h
 *
 * @brief @{Intel Alpine/Titan Ridge control interface header file.@}
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

#ifndef _INTEL_RIDGE_H_
#define _INTEL_RIDGE_H_

/*****************************************************************************
 * Header files including
 *****************************************************************************/

#include <alt_mode_hw.h>
#include <status.h>
#include <hpd.h>

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief This function set AR/TR registers in accordance to input parameters
 *
 * @param port Port index the AR/TR settings are performed for.
 * @param mux_cfg MUX configuration.
 * @param polarity Attached target Type-C Polarity.
 * @param cfg Contains AR/TR register settings in case of TBT alt mode is active.
 *
 * @return true if AR/TR was set successful, in the other case - false
 */
bool ridge_set_mux(uint8_t port, mux_select_t mux_cfg, uint8_t polarity, uint32_t cfg);

/**
 * @brief Enables Titan Ridge the HPD functionality for the specified PD port.
 *
 * @param port PD port index. Caller should ensure to provide only valid values.
 * @param cbk callback to be used for command completion event.
 * @return Returns CCG_STAT_SUCCESS in case of success, error code otherwise.
 */
ccg_status_t tr_hpd_init(uint8_t port, hpd_event_cbk_t cbk);

/**
 * @brief Disables Titan Ridge the HPD functionality for the specified PD port.
 *
 * @param port PD port index. Caller should ensure to provide only valid values.
 * @return None.
 */
void tr_hpd_deinit(uint8_t port);

/**
 * @brief Send the desired HPD event out through the Titan Ridge HPD GPIO. Only
 * the HPD_EVENT_UNPLUG, HPD_EVENT_UNPLUG and HPD_EVENT_IRQ events
 * should be requested.
 *
 * @param port Port on which HPD event is to be sent.
 * @param evtype Type of HPD event to be sent.
 * @return Returns CCG_STAT_SUCCESS in case of success, error code otherwise.
 */
ccg_status_t tr_hpd_sendevt(uint8_t port, hpd_event_type_t evtype);

/**
 * @brief Analyses received ridge command register content.
 *
 * @param port USB-PD port index corresponding to the status update.
 * @param stat Command register value.
 * @param stat_mask Bit mask of TR command register which were changed from the previous time
 *
 * @return true if the interface is idle, false otherwise.
 */
void ridge_eval_cmd(uint8_t port, uint32_t stat, uint32_t stat_mask);

/**
 * @brief Indicates is HPD level changed from the previous state
 *
 * @param port USB-PD port index corresponding to the status update.
 *
 * @return None.
 */
bool tr_is_hpd_change(uint8_t port);

#if ICL_ENABLE
/**
 * @brief Force TBT status update
 *
 * @param port PD port index.
 * @param force_update Force next update on this port
 * @return None
 */
void ridge_force_status_update(uint8_t port, uint8_t force_update);

#endif /* ICL_ENABLE */

#endif /*_INTEL_RIDGE_H_ */

/* [] END OF FILE */
