/**
 * @file intel_vid.h
 *
 * @brief @{Thunderbolt (Intel VID) alternate mode handler header file.@}
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

#ifndef _INTEL_VID_H_
#define _INTEL_VID_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <pd.h>
#include <alt_modes_mngr.h>

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

#define INTEL_VID                       (0x8087u)
/**< Intel (Thunderbolt 3) SVID. */

#define TBT_ALT_MODE_ID                 (1u)
/**< Unique ID assigned to Thunderbolt mode in the CCGx SDK. */

#define MAX_TBT_VDO_NUMB                (1u)
/**< Maximum number of VDOs Thunderbolt-3 uses for the alt mode flow. */

#define TBT_VDO_IDX                     (0u)
/**< Index of VDO used for Thunderbolt. */

/** @cond DOXYGEN_HIDE */

#define TBT_MODE_VPRO_AVAIL             (0x00040000u)
/**< VPro available bit in TBT Mode VDO. */

#define TBT_MODE_VPRO_DOCK_HOST         (0x04000000u)
/**< Vpro_Dock_and_Host bit in TBT Enter Mode VDO. */

#define GET_VPRO_AVAILABLE_STAT(status) ((status & TBT_MODE_VPRO_AVAIL) != 0)
/**< Macro to get VPro Available bit from Discover Mode response VDO. */

/** @endcond */

#define GET_LEGACY_TBT_ADAPTER(status)  ((status >> 16) & 0x1)
/**< Macro to get legacy adapter status from Discovery Mode response  VDO. */

#define TBT_EXIT(status)                ((status >> 4) & 0x1)
/**< Macro to get Exit status from Attention VDO. */

#define BB_STATUS(status)               ((status >> 3) & 0x1)
/**< Macro to get BB status from Attention VDO. */

#define USB2_ENABLE(status)             ((status >> 2) & 0x1)
/**< Macro to get USB enable status from Attention VDO. */
    
#if ICL_ENABLE    
/**
  @brief Macros to get USB enable and BB status from Attention VDO.
 */
#define BB_STS_USB2_ENABLE(status)      ((status >> 2) & 0x3)
/**
  @brief Billboard is present and mode has to be exited
 */
#define BB_PRESENT_EXIT_MODE            (0x03u)   
    
#endif /* ICL_ENABLE */    

/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/

/**
  @typedef tbt_state_t
  @brief This enumeration holds all possible TBT states.
 */
typedef enum
{
    TBT_STATE_IDLE = 0,                 /**< Idle state. */
    TBT_STATE_ENTER = 4,                /**< Enter mode state. */
    TBT_STATE_EXIT = 5,                 /**< Exit mode state. */
    TBT_STATE_ATT = 6,                  /**< Attention state. */
} tbt_state_t;

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief This function analyses Discovery information to find out
 * if further TBT alternative mode processing is allowed.
 *
 * @param port Port index the function is performed for.
 * @param reg_info Pointer to structure which holds alt mode register info.
 *
 * @return Pointer to TBT alternative mode command structure if analysis passed
 * successful. In case of failure, function returns NULL pointer.
 */
alt_mode_info_t* reg_intel_modes(uint8_t port, alt_mode_reg_info_t* reg_info);

#endif /* _INTEL_VID_H_ */

/* [] END OF FILE */
