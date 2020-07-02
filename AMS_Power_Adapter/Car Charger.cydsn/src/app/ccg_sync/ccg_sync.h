/**
 * @file ccg_sync.h
 *
 * @brief @{Header file containing CCG5-CCG5 synchronization interface definitions. @}
 */

/*
 * Copyright (2014-2017), Cypress Semiconductor Corporation or a subsidiary of
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

#ifndef _CCG_SYNC_H_
#define _CCG_SYNC_H_

#include <pd.h>
#include <config.h>
    
/********************************************************************************
 ***********                       MACROS                              **********
 *******************************************************************************/
#define CCG_SYNC_D0_SLAVE_ADDR          (0x12)
#define CCG_SYNC_D1_SLAVE_ADDR          (0x21)
    
#define CCG_SYNC_DPM_RETRY_TIMER        (0x95u)
#define CCG_SYNC_DPM_RETRY_WAIT_PERIOD  (20)   /* 20ms */

#define CCG_SYNC_BUSY_TIMER             (0x96u)
#define CCG_SYNC_BUSY_TIMER_PERIOD      (20)   /* 20ms */

#define V1                              (1)
#define V2                              (2)
#define CCG_SYNC_ALGORITHM              (V2)
    
/********************************************************************************
 ***********                  ENUMERATED TYPES                         **********
 *******************************************************************************/
#if CCG_SYNC_ENABLE
/**
 * @typedef ccg_sync_cmd_t
 * @brief Enumeration of possible synchronization commands
 */
typedef enum {
    RESERVED,               /* Byte: 0x00 */
    UPDATE_STATUS,          /* Byte: 0x01 */
    UCSI_RESET_PPM,         /* Byte: 0x02 */
    UCSI_SET_NTFN_ENABLE,   /* Byte: 0x03 */
    UCSI_SET_PWR_LEVEL,     /* Byte: 0x04 */
    PING,                   /* Byte: 0x05 */
} ccg_sync_cmd_t;

/**
 * @typedef ccg_sync_evt_t
 * @brief Enumeration of possible events that can be generated internal to the module
 */
typedef enum {
    CCG_SYNC_EVT_VSYS_ADDED,
    CCG_SYNC_EVT_VSYS_REMOVED,
    CCG_SYNC_EVT_PKT_RCVD,
    CCG_SYNC_EVT_CHNG_PORT_CUR_P0,
    CCG_SYNC_EVT_CHNG_PORT_CUR_P1,
    CCG_SYNC_EVT_UPDT_SRC_CAP_P0,
    CCG_SYNC_EVT_UPDT_SRC_CAP_P1,
    CCG_SYNC_EVT_SEND_PKT,
    CCG_SYNC_EVT_UPDATE_REMOTE_PORT,
    CCG_SYNC_MAX_EVTS
} ccg_sync_evt_t;

/********************************************************************************
 ***********                  DATA STRUCTURES                          **********
 *******************************************************************************/
/**
 * @brief The sync packet
 */
typedef struct {
    ccg_sync_cmd_t  cmd;
    union {
        uint16_t val;
        struct {
            uint16_t port0_enabled      : 1;
            uint16_t port1_enabled      : 1;
            uint16_t port0_connected    : 1;
            uint16_t port1_connected    : 1;
            uint16_t rsvd1              : 4;
            uint16_t port0_is_high_pwr  : 1;
            uint16_t port1_is_high_pwr  : 1;
            uint16_t rsvd2              : 6;
        } port_sts;
    } data;
} ccg_sync_pkt_t;

/********************************************************************************
 ***********                  FUNCTION DECLARATIONS                    **********
 *******************************************************************************/
/**
 * @brief Initialize the CCG-Sync interface
 * @return None
 */
void ccg_sync_intf_init(void);

/**
 * @brief Start the sync interface
 * @return None
 */
void ccg_sync_intf_start(void);

/**
 * @brief Stop the sync interface
 * @return None
 */
void ccg_sync_intf_stop(void);

/**
 * @brief Set an event on the sync module
 * @param evt Event to signal
 * @return None
 */
void ccg_sync_set_evt(ccg_sync_evt_t evt);

/**
 * @brief Set the slave address for the local and remote CCG5 I2C ports
 * @return None
 */
void ccg_sync_set_slave_addr(uint8_t local_addr, uint8_t remote_addr);

/**
 * @brief Sync task
 * @return None
 */
void ccg_sync_task(void);

/**
 * @brief Get the enable/disable status of the other CCG5's ports
 * @return Bitmask where bit 'x' if set indicates that Port 'x' is enabled
 */
uint8_t ccg_sync_get_remote_port_en(void);

/**
 * @brief Update the other CCG5 with the status of local device
 * @return None
 */
void ccg_sync_update_remote_port(void);

/**
 * @brief Send a packet with updated state to other CCG5
 * @param cmd The cmd to send to the other CCG
 * @param data The 16-bit cmd-specific data
 * @return None
 */
void ccg_sync_send_remote_pkt(ccg_sync_cmd_t cmd, uint16_t data);

/**
 * @brief Schedule alt mode to be re-entered once port advertises 3A
 * @param svid Alt mode's SVID
 * @param alt_mode_id Alt mode's ID
 * @return None
 */
void ccg_sync_schedule_mode_reentry(uint8_t port, uint16_t svid, uint8_t alt_mode_id);

/**
 * @brief PD event sink
 * @return None
 */
void ccg_sync_pd_event_handler(uint8_t port, app_evt_t evt, const void *data);

/**
 * @brief Enable the local power policy manager
 * @return enable A boolean indicating if policy should be enabled or not
 */
void ccg_sync_en_pdo_ctrl(uint8_t enable);

/**
 * @brief Check if sync module allows CCG to be put to sleep
 * @return Can CCG be put to sleep
 */
bool ccg_sync_sleep_allowed(void);

/**
 * @brief Set the preferred status of the port
 * @param preffered If set, this CCG is preferred when allocating higher power during arbitration
 * @return None
 */
void ccg_sync_set_preferred(uint8_t preferred);

#endif /* CCG_SYNC_ENABLE */

#endif /* _CCG_SYNC_H_ */
