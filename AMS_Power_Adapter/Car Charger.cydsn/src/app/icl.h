/**
 * @file icl.h
 *
 * @brief @{Ice Lake specific logic.@}
 */

/*
 * Copyright (2014-2018), Cypress Semiconductor Corporation or a subsidiary of
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

#ifndef _ICL_H_
#define _ICL_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <stdint.h>
#include <pd.h>
#include <hal_ccgx.h>

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/
    
typedef enum {
    ICL_CTRL_REG = 0x0040,
    ICL_STS_REG  = 0x0042,
    ICL_DBG_REG  = 0x0044,
    ICL_RT_PWR   = 0x0045,
} icl_reg_t;
    
typedef enum {
    ICL_EVT_DEQUEUE_HPD,
    ICL_EVT_CHANGE_MUX_STATE,
    ICL_MAX_EVTS
} icl_evt_t;

#define ICL_CTRL_CMD_FORCE_TBT_MODE             (0x01u)

#define ICL_STS_REG_FORCE_TBT_MODE              (1 << 0)
    
#define ICL_MUX_STATE_CHANGE_DURATION      (20u) /* 25ms */

#define ICL_ADP_DETACH_DEBOUNCE_TIME       (1500)
#define ICL_ADP_ATTACH_DEBOUNCE_TIME       (5)
#define ICL_ADP_INIT_DEBOUNCE_TIME         (5)

/* Init state is neither 0 nor 1 so that the debounce logic gets triggered properly */
#define ADP_STATE_INIT                     (2)

#define ADP_EDGE_NONE                      (0)
#define ADP_POSEDGE                        (1)
#define ADP_NEGEDGE                        (-1)

/*******************************************************************************
 * Function Declaration
 ******************************************************************************/

/**
 * @brief Initialize logic needed for Ice Lake designs
 *
 * @return None
 */
void icl_init(void);

/**
 * @brief Run ICL RVP-specific tasks
 * @param port Port index
 * @return None
 */
void icl_task(uint8_t port);

#if ICL_OCP_ENABLE
/**
 * @brief Register a function to be called when OCP happens
 * @param port Port index
 * @param enable Enable or disable OCP
 * @param cbk Callback to be called
 * @return None
 */
void icl_ocp_cfg(uint8_t port, bool enable, vbus_ocp_cbk_t cbk);
#endif /* ICL_OCP_ENABLE */

/**
 * @brief Set an event on the ICL module
 * @param port Port index
 * @param evt ICL event
 * @return None
 */
void icl_set_evt(uint8_t port, icl_evt_t evt);

/**
 * @brief Helper function to get VSYS state
 * @return bool True if VSYS is available else False
 */
bool icl_vsys_is_present(void);

/**
 * @brief Does ICL module allow CCG to enter deep-sleep
 * @return bool
 */
bool icl_sleep_allowed(void);

#endif /* _ICL_H_ */

/* End of File */
