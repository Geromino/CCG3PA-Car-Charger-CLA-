/**
 * @file custom_hpi_vid.h
 *
 * @brief @{Custom HPI alternate mode handler header file.@}
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

#ifndef _CUSTOM_HPI_VID_H_
#define _CUSTOM_HPI_VID_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <pd.h>
#include <alt_modes_mngr.h>

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/
#define MAX_HPI_AM_VDO_NUMB   (1u)
#define HPI_AM_SVID                         (0x0000u)
/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/

/**
  @typedef tbt_state_t
  @brief This enumeration holds all possible custom HPI alt mode states.
 */
typedef enum
{
    HPI_AM_STATE_IDLE = 0,                 /**< Idle state. */
    HPI_AM_STATE_ENTER = 4,                /**< Enter mode state. */
    HPI_AM_STATE_EXIT = 5,                 /**< Exit mode state. */
    HPI_AM_STATE_ATT = 6,                  /**< Attention state. */
} hpi_am_state_t;

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief This function analyses Discovery information to find out
 * if further custom HPI alternative mode processing is allowed.
 *
 * @param port Port index the function is performed for.
 * @param reg_info Pointer to structure which holds alt mode register info.
 *
 * @return Pointer to custom HPI alternative mode command structure if analysis passed
 * successful. In case of failure, function returns NULL pointer.
 */
alt_mode_info_t* reg_hpi_modes(uint8_t port, alt_mode_reg_info_t* reg_info);

#endif /* _CUSTOM_HPI_VID_H_ */

/* [] END OF FILE */
