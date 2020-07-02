/**
 * @file dp_sid.h
 *
 * @brief @{DisplayPort alternate mode handler header file.@}
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

#ifndef _DP_SID_H_
#define _DP_SID_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <pd.h>
#include <alt_modes_mngr.h>

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

#define DP_SVID                         (0xFF01u)
/**< DisplayPort SVID. */

#define DP_ALT_MODE_ID                     (0u)
/**< Unique ID assigned to DP alternate mode in the CCGx SDK. */

#define MAX_DP_VDO_NUMB                    (1u)
/**< Maximum number of VDOs DP uses for the alt mode flow. */

#define DP_VDO_IDX                        (0u)
/**< Index of VDO buffer element which is used to handle DP VDO. */

#define STATUS_UPDATE_VDO                (0x01u)
/**< DP Status Update command VDO used by DFP_U. */

#define DFP_D_CONN                      (0x01u)
/**< DP Status Update command VDO when DP is DFP (DFP_D DP data role). */

#define UFP_D_CONN                      (0x02u)
/**< UFP_D DP data role configuration. */

#define DP_1_3_SIGNALING                (0x0001u)
/**< Standard DP signaling type value in the signaling field for Config VDO. */

#define DP_CONFIG_SELECT                (2u)
/**< Value when DFP set configuration for UFP_U as UFP_D in the configuration
     select field for Config VDO. */

#define DP_CONFIG_SELECT_DPSRC          (1u)
/**< Value when DFP set configuration for UFP_U as DFP_D in the configuration
     select field of Config VDO. */

#define USB_CONFIG_SELECT               (0u)
/**< Value when DFP set configuration as USB in the configuration select field
     for Config VDO. */

#define DP_USB_SS_CONFIG                (0u)
/**< Pin assigment mask for USB pin assigment in the Configure Pin Assigment
     field for Config VDO. */

#define DP_DFP_D_CONFIG_C               (4u)
/**< Pin assigment mask for C pin assigment in the Configure Pin Assigment field for Config VDO. */

#define DP_DFP_D_CONFIG_D               (8u)
/**< Pin assigment mask for D pin assigment in the Configure Pin Assigment field for Config VDO. */

#define DP_DFP_D_CONFIG_E               (0x10u)
/**< Pin assigment mask for E pin assigment in the Configure Pin Assigment field for Config VDO. */

#define DP_DFP_D_CONFIG_F               (0x20u)
/**< Pin assigment mask for F pin assigment in the Configure Pin Assigment field for Config VDO. */

#define DP_INVALID_CFG                  (0xFFu)
/**< Internal DP denotation for invalid pin assigment. */

#define HPD_LOW_IRQ_LOW                 (0x0u)
/**< HPD low and IRQ low value obtained from UFP Attention/Status Update VDO. */

#define HPD_HIGH_IRQ_LOW                (0x1u)
/**< HPD high and IRQ low value obtained from UFP Attention/Status Update VDO. */

#define HPD_LOW_IRQ_HIGH                (0x2u)
/**< HPD low and IRQ high value obtained from UFP Attention/Status Update VDO. */

#define HPD_HIGH_IRQ_HIGH               (0x3u)
/**< HPD high and IRQ high value obtained from UFP Attention/Status Update VDO. */

#define DP_QUEUE_STATE_SIZE             (2u)
/**< Size of one element (in bits) in the HPD queue. */

#define DP_HPD_STATE_MASK               (0x0003u)
/**< Mask to extract the first element from the HPD queue. */

#define DP_QUEUE_EMPTY_INDEX            (0x0u)
/**< Flag to indicate an empty HPD queue. */

#define DP_QUEUE_FULL_INDEX             (7u)
/**< Flag to indicate a full HPD queue. */

#define DP_UFP_MAX_QUEUE_SIZE           (4u)
/**< Maximum number of entries in the HPD queue. */

#define GET_HPD_IRQ_STAT(status)        (((status) >> 7u)& 0x3 )
/**< Macros to get HPD state from Attention/Status Update VDO. */

#define    DP_SINK_CTRL_CMD           (3u)
/**< DP Sink Control APP command value. */

#define    DP_MUX_CTRL_CMD            (1u)
/**< DP MUX Control APP command value. */

#define    DP_APP_CFG_CMD             (2u)
/**< DP MUX Configure APP command value. */

#define    DP_APP_VCONN_SWAP_CFG_CMD  (4u)
/**< DP UFP Vconn Swap command value. */

#define DP_APP_CFG_USB_IDX            (6u)
/**< Bit index of USB configuration in APP DP MUX Configure command data. */

#define DP_APP_CFG_CMD_MAX_NUMB       (6u)
/**< Maximum vualue of USB configuration in APP DP MUX Configure command data. */

#define DP_ALLOWED_MUX_CONFIG_EVT     (1u)
/**< DP allowed MUX Configuration APP event value. */

#define DP_STATUS_UPDATE_EVT          (2u)
/**< DP Status Update APP event value. */

#define DP_CFG_CMD_ACK_MASK           (0x200)
/**< Acked field mask for EC DP MUX Configure command data. */

/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/

/**
  @typedef dp_state_t
  @brief This enumeration holds all possible DP states.
 */
typedef enum
{
    DP_STATE_IDLE = 0,                  /**< Idle state. */
    DP_STATE_ENTER = 4,                 /**< Enter mode state. */
    DP_STATE_EXIT = 5,                  /**< Exit mode state. */
    DP_STATE_ATT = 6,                   /**< DP Attention state. */
    DP_STATE_STATUS_UPDATE = 16,        /**< DP Status Update state. */
    DP_STATE_CONFIG = 17                /**< DP Configure state. */
}dp_state_t;

/**
  @typedef dp_port_cap_t
  @brief This enumeration holds possible DP capabilities.
 */
typedef enum
{
    DP_PORT_CAP_RSVD = 0,               /**< Reserved capability. */
    DP_PORT_CAP_UFP_D,                  /**< UFP is UFP_D-capable. */
    DP_PORT_CAP_DFP_D,                  /**< UFP is DFP_D-capable. */
    DP_PORT_CAP_BOTH                    /**< UFP is DFP_D and UFP-D capable. */
}dp_port_cap_t;

/**
  @typedef dp_conn_t
  @brief This enumeration holds possible DFP_D/UFP_D Connected status (Status Update message).
 */
typedef enum
{
    DP_CONN_NONE = 0,                   /**< Neither DFP_D nor UFP_D is connected. */
    DP_CONN_DFP_D,                      /**< DFP_D is connected. */
    DP_CONN_UFP_D,                      /**< UFP_D is connected. */
    DP_CONN_BOTH                        /**< Both DFP_D and UFP_D are connected. */
}dp_conn_t;

/**
  @typedef dp_stat_bm_t
  @brief This enumeration holds corresponding bit positions of Status Update VDM fields.
 */
typedef enum
{
    DP_STAT_DFP_CONN = 0,               /**< DFP_D is connected field bit position. */
    DP_STAT_UFP_CONN,                   /**< UFP_D is connected field bit position. */
    DP_STAT_PWR_LOW,                    /**< Power Low field bit position. */
    DP_STAT_EN,                         /**< Enabled field bit position. */
    DP_STAT_MF,                         /**< Multi-function Prefered field bit position. */
    DP_STAT_USB_CNFG,                   /**< USB Configuration Request field bit position. */
    DP_STAT_EXIT,                       /**< Exit DP Request field bit position. */
    DP_STAT_HPD,                        /**< HPD state field bit position. */
    DP_STAT_IRQ                         /**< HPD IRQ field bit position. */
}dp_stat_bm_t;

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief This function analyses Discovery information to find out
 * if further DP alternative mode processing is allowed.
 *
 * @param port Port index the function is performed for.
 * @param reg_info Pointer to structure which holds alt mode register info.
 *
 * @return Pointer to DP alternative mode command structure if analysis passed
 * successful. In case of failure, function returns NULL pointer.
 */
alt_mode_info_t* reg_dp_modes(uint8_t port, alt_mode_reg_info_t* reg_info);

/** @cond DOXYGEN_HIDE */

#if DP_GPIO_CONFIG_SELECT

/**
 * @brief Store the DP Pin configuration based on GPIO status.
 * @param dp_config DP Pin configuration
 * @return None
 */
void dp_sink_set_pin_config(uint8_t dp_config);

/**
 * @brief Return the DP Pin configuration based on GPIO status.
 * @return DP Pin configuration
 */
uint8_t dp_sink_get_pin_config(void);

#endif /* DP_GPIO_CONFIG_SELECT */

#if CCG4_DOCK

/* Workaround for CCG4 Dock. This info is needed to enable config E */
uint8_t get_dp_config(uint8_t port);

typedef enum
{
    PREF_NOT_SELECTED = 0,
    PREF_2LANE,
    PREF_4LANE
} dp_pref_lane;

/**
 * @brief This function returns preffered 2/4L according to mode selected by Button in Dock design
 * @return Returns preffered 2/4L mode
 */
dp_pref_lane get_pref_lane(void);

/**
 * @brief This function updated preffered 2/4L according to mode selected by Button in Dock design
 * @param lane Updates current preffered 2/4L mode
 */
void updt_pref_lane(dp_pref_lane lane);

#endif /* CCG4_DOCK */

/** @endcond */

#endif /* _DP_SID_H_ */

/* [] END OF FILE */
