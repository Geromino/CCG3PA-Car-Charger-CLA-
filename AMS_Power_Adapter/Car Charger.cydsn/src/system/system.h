/**
 * @file system.h
 *
 * @brief @{Support functions and definitions for bootloader and flash updates.@}
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

#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include "stdint.h"

/*****************************************************************************
* MACRO Definition
*****************************************************************************/

#define SYS_BOOT_VERSION_ADDRESS        (0x000000E0)    /**< Boot loader version address in FLASH. */
#define SYS_FW_VERSION_OFFSET           (0x000000E0)    /**< Offset of FW version from start of FW image in flash. */
#define SYS_APP_VERSION_OFFSET          (0x00000004)    /**< Offset of App version from start of firmware version. */
#define SYS_SILICON_ID_OFFSET           (0x000000EA)    /**< Offset of Silicon ID stored in FW image in flash. */
#define SYS_BOOT_TYPE_FIELD_OFFSET      (0x000000EC)    /**< Offset of Bootloader type in Bootloader flash region. */
#define SYS_FW_CUSTOM_INFO_OFFSET       (0x000000C0)    /**< Offset of Customer Specific Info from FW start. */

#define SYS_METADATA_VALID_SIG          (0x4359)        /**< Metadata table valid signature: "CY" */
#define SYS_PSEUDO_METADATA_VALID_SIG   (0x4350)        /**< Pseudo-Metadata valid signature: "CP" */
#define SYS_BOOT_MODE_RQT_SIG           (0x424C)        /**< Boot Mode Request Signature: "BL" */
#define SYS_CONFIG_TABLE_SIGN           (0x4359u)       /**< Configuration table valid signature: "CY" */
    
#define SYS_INVALID_FW_START_ADDR       (0x00000000)    /**< Invalid FW Start Address. */

#define SYS_BOOT_TYPE_SECURE_BOOT_MASK          (0x01)  /**< Mask to get Secure Boot feature bit in BL type. */
#define SYS_BOOT_TYPE_FW_UPDATE_INTERFACE_POS   (0x01)  /**< Position of FW update interface bit in BL type. */
#define SYS_BOOT_TYPE_FW_UPDATE_INTERFACE_MASK  (0x02)  /**< Mask to get FW update interface bit in BL type. */
#define SYS_BOOT_TYPE_APP_PRIORITY_POS          (0x02)  /**< Position of app priority bit in Bootloader type. */

#define SYS_SILICON_ID_MASK                     (0xFF00)        /**< Mask to extract the device ID from Silicon ID. */
    
/*****************************************************************************
* Enumerated Data Definition
*****************************************************************************/

/**
 * @typedef sys_fw_mode_t
 * @brief List of CCG firmware modes.
 */
typedef enum
{
    SYS_FW_MODE_BOOTLOADER = 0,     /**< Bootloader mode. */
    SYS_FW_MODE_FWIMAGE_1,          /**< Firmware Image #1 */
    SYS_FW_MODE_FWIMAGE_2,          /**< Firmware Image #2 */
    SYS_FW_MODE_INVALID             /**< Invalid value. */
} sys_fw_mode_t;

/*****************************************************************************
* Global Variable Declaration
*****************************************************************************/

/**
 * @brief Variable representing the current firmware mode.
 */
extern sys_fw_mode_t gl_active_fw;

/**
 * @brief Invalid firmware version.
 */
extern uint8_t gl_invalid_version[8];

/*****************************************************************************
* Global Function Declaration
*****************************************************************************/

/**
 * @brief Set the current firmware mode.
 *
 * This function is used by the start-up logic to store the current firmware
 * mode for the CCGx device.
 *
 * This should not be used outside of the default start-up logic for the CCGx
 * bootloader and firmware applications.
 *
 * @param fw_mode The active firmware mode to be set.
 *
 * @return None
 */
void sys_set_device_mode(sys_fw_mode_t fw_mode);

/**
 * @brief Get the current firmware mode.
 *
 * This function retrieves the current firmware mode of the CCG device.
 *
 * @return The current firmware mode.
 */
sys_fw_mode_t sys_get_device_mode(void);

/**
 * @brief Get bootloader version.
 *
 * The bootloader version is stored at absolute address SYS_CCG_BOOT_VERSION_ADDRESS
 * in device FLASH. This function returns a pointer to this version information.
 * 
 * @return Pointer to the bootloader version information.
 */
uint8_t* sys_get_boot_version(void);

/**
 * @brief Get version for firmware image-1.
 *
 * This function returns a pointer to the version information for firmware image-1 (FW1).
 * The version is located at a fixed offset of CY_PD_FW_VERSION_OFFSET bytes
 * from the start of the firmware binary.
 *
 * @return Pointer to the firmware image-1 version information.
 */
uint8_t* sys_get_img1_fw_version(void);

/**
 * @brief Get version for firmware image-2.
 *
 * This function returns a pointer to the version information for firmware image-2 (FW2).
 * The version is located at a fixed offset of CY_PD_FW_VERSION_OFFSET bytes
 * from the start of the firmware binary.
 *
 * @return Pointer to the firmware image-2 version information.
 */
uint8_t* sys_get_img2_fw_version(void);

/**
 * @brief Get the flash start address of firmware image-1.
 *
 * This function returns the flash address from where firmware image-1 (FW1) has
 * been stored.
 *
 * @return Start address of firmware image-1.
 */
uint32_t sys_get_fw_img1_start_addr(void);

/**
 * @brief Get the flash start address of firmware image-2.
 *
 * This function returns the flash address from where firmware image-2 (FW2) has
 * been stored.
 *
 * @return Start address of firmware image-2.
 */
uint32_t sys_get_fw_img2_start_addr(void);

/**
 * @brief Determines the more recently update firmware image.
 *
 * The CCG Bootloader uses this function to determine the more recently updated
 * firmware image (from among FW1 and FW2) by comparing the sequence numbers of
 * images which are stored in the firmware metadata table. The bootloader loads
 * the most recently updated binary by default (even if its version is older 
 * than that of the other firmware binary).
 *
 * @return Firmware id: 1 for Image-1 and 2 for Image-2.
 */
uint8_t sys_get_recent_fw_image(void);

/**
 * @brief Get Silicon ID of device.
 *
 * This function retrieves the Silicon ID of the CCG device.
 * 
 * @param silicon_id Pointer to buffer to hold the Silicon ID.
 *
 * @return None
 */
void sys_get_silicon_id(uint32_t *silicon_id);

/**
 * @brief Returns Silicon revision.
 *
 * @return Silicon revision
 *      B[7:4] - Major rev
 *      B[3:0] - Minor rev
 */
uint8_t get_silicon_revision(void);

/**
 * @brief Get start address of Customer info section.
 *
 * This function returns the start address of Customer info section.
 * 
 * @return Address of Customer info section.
 */
uint32_t sys_get_custom_info_addr(void);

/**
 * @brief Get bcdDevice version of device.
 *
 * This function returns bcdDevice version for the device which can be used
 * as part of D_ID response, secure boot checks etc. Format of bcdDevice version
 * is documented in the fucntion body.
 * 
 * @param ver_addr Offset of version in Flash memory.
 *
 * @return 16 bits bcdDevice version.
 */
uint16_t sys_get_bcdDevice_version(uint32_t ver_addr);

/** @cond DOXYGEN_HIDE */

/* Function pointer types used to access ROM-ed versions of SYSTEM module APIs.
 * These correspond to the various functions defined above.
 */
typedef void (*sys_set_device_mode_fptr)(sys_fw_mode_t fw_mode);
typedef sys_fw_mode_t (*sys_get_device_mode_fptr)(void);
typedef uint8_t* (*sys_get_boot_version_fptr)(void);
typedef uint8_t* (*sys_get_img1_fw_version_fptr)(void);
typedef uint8_t* (*sys_get_img2_fw_version_fptr)(void);
typedef uint32_t (*sys_get_fw_img1_start_addr_fptr)(void);
typedef uint32_t (*sys_get_fw_img2_start_addr_fptr)(void);
typedef uint8_t (*sys_get_recent_fw_image_fptr)(void);
typedef void (*sys_get_silicon_id_fptr)(uint32_t *silicon_id);
typedef uint8_t (*get_silicon_revision_fptr)(void);
typedef uint32_t (*sys_get_custom_info_addr_fptr)(void);
typedef uint16_t (*sys_get_bcdDevice_version_fptr)(uint32_t ver_addr);

/** @endcond */

#endif /* _SYSTEM_H_ */
