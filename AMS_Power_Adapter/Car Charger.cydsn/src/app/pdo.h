/**
 * @file pdo.h
 *
 * @brief @{PDO evaluation and handler definitions.@}
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

#ifndef _PDO_H_
#define _PDO_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <pd.h>    

/*****************************************************************************
 * Macro Definitions
 *****************************************************************************/

#define APP_PPS_SNK_CONTRACT_PERIOD             (9000u)
/**< Period after which a PPS Sink repeats PD contract attempts. This should be faster than once in 10 seconds. */

#define APP_PPS_SNK_CONTRACT_RETRY_PERIOD       (5u)
/**< Period after which a failed PPS sink re-contract attempt will be retried. */

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief This function is called by the PD stack to allow the application
 * logic to evaluate the Source Capabilities received from the port partner
 * and generate the desired request. The request object is expected to be
 * passed back to the stack through the app_resp_handler() callback.
 *
 * The default implementation of this function matches each of the received
 * source PDOs against the active sink capabilities; and then selects the
 * source PDO that can deliver the maximum power to the system as a sink.
 *
 * @warning On the CCG6 device, the eval_src_cap function is executed from
 * ROM and hence changes in this function will not affect the policy engine
 * behavior. The app_update_rdo() function should be used to update the request
 * object in this case.
 *
 * @param port Port index the function is performed for.
 * @param src_cap Pointer to PD packet which contains source capabilities.
 * @param app_resp_handler Application handler callback function.
 *
 * @return None
 */
void eval_src_cap(uint8_t port, const pd_packet_t* src_cap, app_resp_cbk_t app_resp_handler) ;

/**
 * @brief This function is called by the PD stack to allow the application
 * to evaluate a power request data object received from the port partner and
 * decide whether it should be satisfied. The response to the request should
 * be passed back to the stack through the app_resp_handler() callback.
 *
 * @warning On the CCG6 device, the eval_rdo function is executed from ROM
 * and hence changes in this function will not affect the policy engine
 * behavior. The application is expected to update the source capabilities
 * registered with the PD stack based on the current system status.
 *
 * @param port Port index the function is performed for.
 * @param rdo The request data object received.
 * @param app_resp_handler Application handler callback function.
 *
 * @return None
 */
void eval_rdo(uint8_t port, pd_do_t rdo, app_resp_cbk_t app_resp_handler) ;

/**
 * @brief This function is only applicable in the case of the CCG6 device,
 * and can be used by the application logic to modify the RDO generated by
 * the ROM-ed version of the eval_src_cap function.
 *
 * @param port Port index the function is performed for.
 * @param src_cap Pointer to PD packet which contains source capabilities.
 * @param app_resp Response containing the updated RDO.
 *
 * @return None
 */
void app_update_rdo (uint8_t port, const pd_packet_t* src_cap, app_resp_t *app_resp);

/** @cond DOXYGEN_HIDE */

/* Function pointer types used to access ROM-ed versions of PDO evaluation APIs.
 * These correspond to the various functions defined above.
 */

typedef void (*eval_src_cap_fptr)(uint8_t port, const pd_packet_t* src_cap, app_resp_cbk_t app_resp_handler) ;
typedef void (*eval_rdo_fptr)(uint8_t port, pd_do_t rdo, app_resp_cbk_t app_resp_handler) ;

/** @endcond */

#endif /* _PDO_H_ */

/* End of File */

