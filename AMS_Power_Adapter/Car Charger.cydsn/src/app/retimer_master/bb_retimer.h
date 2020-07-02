/**
 * @file bb_retimer.h
 *
 * @brief @{Burnside/Delta bridge retimer interface definitions.@}
 */

/*
 * Copyright (2014-2016), Cypress Semiconductor Corporation or a subsidiary of
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

#ifndef _ICL_RETIMER_H_
#define _ICL_RETIMER_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <pd.h>
    
/*******************************************************************************
 * Enumerated Types
 ******************************************************************************/
    
typedef enum {
    RT_EVT_VSYS_REMOVED,
    RT_EVT_VSYS_ADDED,
    RT_EVT_PWR_NOT_AVAILABLE,
    RT_EVT_RETRY_STATUS,
    RT_EVT_UPDATE_STATUS,
    RT_EVT_FORCE_PWR_CHNG,
    RT_EVT_WRITE_DEBUG_MODE,
    RT_EVT_READ_DEBUG_MODE,
    RT_MAX_EVTS
} rt_evt_t;

/*******************************************************************************
 * Helper Macros
 ******************************************************************************/

/* Macro to enable/disable manual VSYS sampling */
#define RETIMER_MANUAL_VSYS_SAMPLE      (1)

/* Size of the BB Debug Mode register */
#define BB_DEBUG_MODE_SIZE              (4u)

/* Number of times to poll the retimer before giving up */
#define BB_DEBUGMODE_POLL_COUNT         (10u)

/* Max number of retimers available on any platform */
#define MAX_NO_OF_RETIMERS              (1u)

/* Retimer Configuration bit: is retimer available? */
#define RETIMER_CFG_AVAILABLE           (0x80)
/* Retimer Configuration bit: 7-bit slave address */
#define RETIMER_CFG_SLAVE_ADDR          (0x7F)

/* BB Register: Connection State */
#define BB_CONN_STATE_REG               (0x04)
/* BB Register: TBT Status */
#define BB_STATUS_REG                   (0x05)
/* BB Register: Debug Mode */
#define BB_DEBUG_MODE_REG               (0x07)

/* TBT Status bits that are used in BB */
//#define BB_STATUS_MASK                  (0xBEBFDFE7u)
#define BB_STATUS_MASK                   (0xBEBFDF27u)

/* Bit 30 (previously reserved) indicates S0=0 vs Sx=1 */
#define BB_STATUS_Sx_ACTIVE             (1 << 24)

/* 'Rx Locked' bit in the status register */
#define BB_DEBUG_MODE_RX_LOCKED         (0x8000u)

/* Bits to be set in DEBUG_MODE for USB Compliance mode entry
   Bit 31 (Global Enable) = 1
   Bit 30 (Compliance Enabled) = 1
   Bit 29 (USB Compliance Mode) = 1
   Bit 28 (HTI Mode) = 0 */
#define BB_DEBUG_MODE_USB_COMPLIANCE    (0xD0000000u)
#define BB_DEBUG_MODE_CLEAR             (0u)

/* Default mask used to "AND" all debug mode data from retimers */
#define BB_DEBUG_MODE_DEFAULT           (0xFFFFFFFFu)
    
/* Add a 100ms timer to retry reading the DEBUG register from the retimer */
#define BB_DBR_DEBUG_POLL_DELAY         (100u)

/* Retimer was in low power and NAKed the request. The initial I2C request will wake the retimer up after which
     * it starts re-loading firmware. Give it ~50ms to do this and start accepting state change requests.
     * (Intel retimers unload their firmare upon entering low power mode)
     * Retry status update after 50ms */    
#define BB_DBR_WAKEUP_DELAY             (50u)
    
#define RETIMER_SLADDR_DEV0             (0x40)
#define RETIMER_SLADDR_DEV1             (0x42)
#define RETIMER_SLADDR_DEV2             (0x44)

/* Timeout for the fucntion completion is 100 ms */
#define RETIMER_I2C_TIMEOUT             (0x64)

/*******************************************************************************
 * Extern variable definitions
 ******************************************************************************/

extern uint8_t gl_system_state;

/*******************************************************************************
 * Function Definitions
 ******************************************************************************/

/**
 * @brief This initializes the retimer I2C and GPIO interfaces
 *
 * @param None
 *
 * @return None
 */
void retimer_init(void);

/**
 * @brief Enable retimer on the specific port. This releases its reset
 *        and turns the power gate on
 *
 * @param port Port index the function is performed for.
 *
 * @return None
 */
void retimer_enable(uint8_t port);

/**
 * @brief Disable retimer on the specific port. This holds its reset
 *        and turns the power gate off
 *
 * @param port Port index the function is performed for.
 *
 * @return None
 */
void retimer_disable(uint8_t port);

/**
 * @brief Set an event on the retimer module
 *
 * @param port Port index the function is performed for.
 * @param evt Event to signal on the module
 *
 * @return None
 */
void retimer_set_evt(uint8_t port, rt_evt_t evt);

/**
 * @brief Clear an event on the retimer module
 *
 * @param port Port index the function is performed for.
 * @param evt Event to clear on the module
 *
 * @return None
 */
void retimer_clr_evt(uint8_t port, rt_evt_t evt);

/**
 * @brief Start polling the debug mode register on the retimer
 *
 * @param  port Port index the function is performed for.
 * @param  debug_mode_data Buffer that holds debug mode data
 *
 * @return None
 */
void retimer_start_debug_poll(uint8_t port, uint8_t* debug_mode_data);

/**
 * @brief Update the retimer status
 *
 * @param  port Port index the function is performed for.
 * @param  status Retimer status
 * @param  force_update Force update of retimer status
 *
 * @return bool Result of the update
 */
bool retimer_status_update(uint8_t port, uint32_t status, bool force_update);

/**
 * @brief Check if retimer update is pending
 *
 * @param  port Port index the function is performed for.
 * @return bool Update pending status
 */
bool retimer_update_is_pending(uint8_t port);

/**
 * @brief Run the retimer task for that port
 *
 * @param port Port index the retimer functions are handled for
 *
 * @return None
 */
void retimer_task(uint8_t port);

#if RETIMER_CFG_TBL_SUPP == 0 
/**
 * @brief Set the retimer slave address
 *
 * @param base_addr Base address of the retimers on this port
 *
 * @return None
 */
void retimer_set_slave_address(uint8_t base_addr);
#endif

/**
 * @brief Check if retimer interface can go to sleep
 *
 * @param None
 *
 * @return bool Can enter sleep or not
 */
uint8_t retimer_sleep_allowed(void);

/**
 * @brief Indicator of whether host is in compliance mode
 *
 * @param compl_mode Mode status
 *
 * @return None
 */
void retimer_set_compl_mode(uint8_t compl_mode);

#endif /* _ICL_RETIMER_H_ */

/* End of File */

