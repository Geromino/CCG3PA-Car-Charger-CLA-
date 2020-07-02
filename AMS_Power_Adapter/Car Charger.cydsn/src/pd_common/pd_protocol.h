/**
 * @file pd_protocol.h
 *
 * @brief @{USB-PD protocol layer header file.@}
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

#ifndef _PD_PROTOCOL_H_
#define _PD_PROTOCOL_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <pd.h>
#include <status.h>
#include <pdss_hal.h>
#include <timer.h>


/**
 * @brief Struct to hold PD protocol message IDs and flags. Separate structures need to be
 * maintained for each packet type (SOP, SOP' and SOP'').
 */
typedef struct prl_cntrs
{
    uint8_t  tr_msg_id;                 /**< The message ID to be used on the next transmitted message. */
    uint8_t  rec_msg_id;                /**< The message ID of last received message. */
    uint8_t  volatile first_msg_rcvd;   /**< Flag that indicates whether any message has been received so far. */
} prl_cntrs_t;

/**
 * @brief Structure to hold protocol layer status.
 *
 * Warning: Changes to this structure will break compatibility with the ROM-ed version of
 * the Protocol layer on the CCG6 device, and can cause application malfunction. This restriction
 * only applies to projects that use the ROM-ed protocol layer code on the CCG6 device family.
 */
typedef struct pd_status
{
    pd_cbk_t cbk;                       /**< Callback used to send notifications to Policy Engine. */
    prl_cntrs_t  ctrs[MAX_SOP_TYPES];   /**< Protocol counters for various packet types. */
    void* tx_dobj;                      /**< Pointer to data object being transmitted. */
    uint32_t tx_buf[MAX_PD_PKT_WORDS];  /**< Buffer used to hold message to be (or being) transmitted. */
    uint32_t tx_header;                 /**< PD header corresponding to the message being transmitted. */
    pd_packet_extd_t rx_packet;         /**< Buffer used to hold received PD message. Can be extended or not. */
    uint8_t  cur_rec_msg_id;            /**< Stores message ID (excluding GoodCRC) of the received message. */
    sop_t last_rcvd_sop;                /**< Stores last received SOP type. */
    uint8_t volatile avoid_retry;       /**< Flag to skip retry on CRCReceiveTimer expiry. */
    uint8_t volatile tx_busy;           /**< Flag indicating TX state machine is busy. */
    sop_t tx_sop;                       /**< Stores the SOP type of message being transmitted. */
    pd_extd_hdr_t tx_extd_hdr;          /**< Holds the extended header for the message being transmitted. */
    bool tx_extd;                       /**< Flag that indicates that the current message is extended. */
    sop_t last_tx_sop;                  /**< Stores the SOP type of last transmitted message. */
    uint8_t tx_msg_type;                /**< Stores the message type of message being transmitted. */
    uint8_t tx_count;                   /**< Holds the data object count of the message being transmitted. */
    uint8_t bist_test_en;               /**< Flag indicating BIST test data is enabled. */
    uint32_t volatile rx_evt;           /**< Type of receive state machine event received from the HAL. */
    bool volatile rx_busy;              /**< Flag indicating RX state machine is busy. */
    volatile uint8_t rev3_enable;       /**< Flag indicating the PD Revision 3.0 support is enabled. */
    volatile uint8_t frs_tx_enable;     /**< Flag indicating that FRS transmit support is enabled. */
    volatile uint8_t frs_rx_enable;     /**< Flag indicating that FRS receive support is enabled. */
} pd_status_t;

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief This function sets the clock and necessary registers for PD hw ip to
 * function and initializes the PD protocol layer.
 * @param port port index
 * @param cbk pd event handler callback
 * @return ccg_status_t
 */
ccg_status_t pd_prot_init(uint8_t port, pd_cbk_t cbk);

/**
 * @brief This function starts the protocol layer and configures pd phy as per
 * current port role /data role/contract status of port.
 * This API does not enable the receiver
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_start(uint8_t port);

/**
 * @brief This function configures pd phy as per current port role /data role/contract status of port.
 * This API does not enable the receiver
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_refresh_roles(uint8_t port);

/**
 * @brief This function completely stops the pd hw and put it in lowest power state.
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_stop(uint8_t port);

/**
 * @brief This function enables the bmc receiver.
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_rx_en(uint8_t port);

/**
 * @brief This function disables the bmc receiver.
 * @param port port index
 * @param hard_reset_en When 0 means rx is completely disabled, When 1 means only
 * hard reset can be received.
 * @return ccg_status_t
 */
ccg_status_t pd_prot_rx_dis(uint8_t port, uint8_t hard_reset_en);

/**
 * @brief This function checks if protocol layer is busy
 * @param port port index
 * @return Return true when busy else return false
 */
bool pd_prot_is_busy(uint8_t port);

/**
 * @brief This function resets the protocol layer(TX and RX) counters for each sop type.
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_reset_all(uint8_t port);

/**
 * @brief This function resets protocol layer(TX and RX) counter for a specific sop type.
 * @param port port index
 * @param sop sop type
 * @return ccg_status_t
 */
ccg_status_t pd_prot_reset(uint8_t port, sop_t sop);

/**
 * @brief This function resets protocol layer RX only counter for a specific sop type.
 * @param port port index
 * @param sop sop type
 * @return ccg_status_t
 */
ccg_status_t pd_prot_reset_rx(uint8_t port, sop_t sop);

/**
 * @brief This function sends a control message. Results will be known to caller via callback
 * function registered in pd_prot_int() if this function returns success.
 * This function returns after registering the request.
 * @param port port index
 * @param sop sop type
 * @param msg_type control message type.
 * @return ccg_status_t
 */
ccg_status_t pd_prot_send_ctrl_msg(uint8_t port, sop_t sop, ctrl_msg_t msg_type);

/**
 * @brief This function sends a data message. Results will be known to caller via callback
 * function registered in pd_phy_init() this this function returns success.
 * This function returns after registering the request.
 * @param port port index
 * @param sop sop type
 * @param msg_type data message type
 * @param count data objects count
 * @param dobj pointer to data objects
 * @return ccg_status_t
 */
ccg_status_t pd_prot_send_data_msg(
        uint8_t port,
        sop_t sop,
        data_msg_t msg_type,
        uint8_t count,
        pd_do_t *dobj);

/**
 * @brief This function sends an extended message. Results will be known to caller via callback
 * function registered in pd_phy_init() if this function returns sucsess.
 * This function returns after registering the request.
 * @param port port index
 * @param sop sop type
 * @param msg_type data message type
 * @param ext_hdr 16 bit extended header
 * @param dobj pointer to data
 * @return ccg_status_t
 */
ccg_status_t pd_prot_send_extd__msg(uint8_t port,
                       sop_t sop,
                       extd_msg_t msg_type,
                       pd_extd_hdr_t ext_hdr,
                       uint8_t* dobj);

/**
 * @brief This function sends a hard reset. Results will be known to caller via callback
 * function registered in pd_phy_init() if this function returns success.
 * This function returns after registering the request.
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_send_hard_reset(uint8_t port);

/**
 * @brief This function sends a cable reset. Results will be known to caller via callback
 * function registered in pd_phy_init() if this fucntion returns sucsess.
 * This function returns after registering the request.
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_send_cable_reset(uint8_t port);

/**
 * @brief This function enables sending bist carrier mode 2. There is no call back for this function
 * This function returns after registering the request.
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_en_bist_cm2(uint8_t port);

/**
 * @brief This function disable sending bist carrier mode 2. There is no call back for this function
 * This function returns after registering the request.
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_dis_bist_cm2(uint8_t port);

/**
 * @brief This function puts the receiver in bist test data mode.
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_en_bist_test_data(uint8_t port);

/**
 * @brief This function disables the bist test data mode
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_dis_bist_test_data(uint8_t port);

/**
 * @brief This function can be used by higher layers to avoid retry on CRC expire
 * for a particular message. When this flag is set before calling
 * pd_prot_send_data_msg() or pd_send_crtl_msg() APIs, retry is avoided on CRC failure.
 * This is one time only. Flag is automatically cleared after
 * transmission(success or fail).
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_set_avoid_retry(uint8_t port);

/**
 * @brief This function returns pointer to received pd packet
 * @param port port index
 * @return Returns pointer to received data if port param is not correct return null
 */
pd_packet_extd_t* pd_prot_get_rx_packet(uint8_t port);

/**
 * @brief This function enables the fast role swap receive functionality.
 *
 * @param port: Port index.
 * @return ccg_status_t
 */
ccg_status_t pd_prot_frs_rx_enable(uint8_t port);

/**
 * @brief This function disables the fast role swap receive functionality.
 *
 * @param port: Port index.
 * @return ccg_status_t
 */
ccg_status_t pd_prot_frs_rx_disable(uint8_t port);

/**
 * @brief This function enables the fast role swap transmit functionality.
 *
 * @param port: Port index.
 * @return ccg_status_t
 */
ccg_status_t pd_prot_frs_tx_enable(uint8_t port);

/**
 * @brief This function disables the fast role swap transmit functionality.
 *
 * @param port: Port index.
 * @return ccg_status_t
 */
ccg_status_t pd_prot_frs_tx_disable(uint8_t port);

#if (CCG_SROM_CODE_ENABLE)
void phy_cbk(uint8_t port, pd_phy_evt_t event);
typedef void (*phy_cbk_fptr)(uint8_t port, pd_phy_evt_t event);

void prl_tmr_cbk(uint8_t port, timer_id_t id);
typedef void (*prl_tmr_cbk_fptr)(uint8_t port, timer_id_t id);
#endif /* (CCG_SROM_CODE_ENABLE) */

/** @cond DOXYGEN_HIDE */

/* Function pointer types used to access ROM-ed versions of PD Protocol module APIs.
 * These correspond to the various functions defined above.
 */
typedef ccg_status_t (*pd_prot_init_fptr)(uint8_t port, pd_cbk_t cbk);
typedef ccg_status_t (*pd_prot_start_fptr)(uint8_t port);
typedef ccg_status_t (*pd_prot_refresh_roles_fptr)(uint8_t port);
typedef ccg_status_t (*pd_prot_stop_fptr)(uint8_t port);
typedef ccg_status_t (*pd_prot_rx_en_fptr)(uint8_t port);
typedef ccg_status_t (*pd_prot_rx_dis_fptr)(uint8_t port, uint8_t hard_reset_en);
typedef bool (*pd_prot_is_busy_fptr)(uint8_t port);
typedef ccg_status_t (*pd_prot_reset_all_fptr)(uint8_t port);
typedef ccg_status_t (*pd_prot_reset_fptr)(uint8_t port, sop_t sop);
typedef ccg_status_t (*pd_prot_reset_rx_fptr)(uint8_t port, sop_t sop);
typedef ccg_status_t (*pd_prot_send_ctrl_msg_fptr)(uint8_t port, sop_t sop, ctrl_msg_t msg_type);
typedef ccg_status_t (*pd_prot_send_data_msg_fptr)(uint8_t port, sop_t sop, data_msg_t msg_type,
        uint8_t count, pd_do_t *dobj);
typedef ccg_status_t (*pd_prot_send_extd__msg_fptr)(uint8_t port, sop_t sop, extd_msg_t msg_type,
        pd_extd_hdr_t ext_hdr, uint8_t* dobj);
typedef ccg_status_t (*pd_prot_send_hard_reset_fptr)(uint8_t port);
typedef ccg_status_t (*pd_prot_send_cable_reset_fptr)(uint8_t port);
typedef ccg_status_t (*pd_prot_en_bist_cm2_fptr)(uint8_t port);
typedef ccg_status_t (*pd_prot_dis_bist_cm2_fptr)(uint8_t port);
typedef ccg_status_t (*pd_prot_en_bist_test_data_fptr)(uint8_t port);
typedef ccg_status_t (*pd_prot_dis_bist_test_data_fptr)(uint8_t port);
typedef ccg_status_t (*pd_prot_set_avoid_retry_fptr)(uint8_t port);
typedef pd_packet_extd_t* (*pd_prot_get_rx_packet_fptr)(uint8_t port);
typedef ccg_status_t (*pd_prot_frs_rx_enable_fptr)(uint8_t port);
typedef ccg_status_t (*pd_prot_frs_rx_disable_fptr)(uint8_t port);
typedef ccg_status_t (*pd_prot_frs_tx_enable_fptr)(uint8_t port);
typedef ccg_status_t (*pd_prot_frs_tx_disable_fptr)(uint8_t port);

/** @endcond */

#endif /* _PD_PROTOCOL_H_ */

/* End of File */

