/**
 * @file system.c
 *
 * @brief @{Support functions for boot-loader and flash updates.@}
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

#include "stdbool.h"
#include "stdint.h"
#include "config.h"
#include "flash_config.h"
#include "status.h"
#include "boot.h"
#include "ccgx_regs.h"
#include "system.h"
#include "srom_vars.h"

#if (!CCG_SROM_CODE_ENABLE)

/* Invalid FW Version. */
uint8_t gl_invalid_version[8] = {0};

/* Variable representing the current firmware mode. */
sys_fw_mode_t gl_active_fw = SYS_FW_MODE_INVALID;

#endif /* (!CCG_SROM_CODE_ENABLE) */

ATTRIBUTES void sys_set_device_mode(sys_fw_mode_t fw_mode)
{
    GET_IN_VARIABLE(gl_active_fw) = fw_mode;
}    

ATTRIBUTES sys_fw_mode_t sys_get_device_mode(void)
{
#if (CCG_BOOT == 1)
    return SYS_FW_MODE_BOOTLOADER;
#else /* Firmware */
    return (GET_IN_VARIABLE(gl_active_fw));
#endif /* CCG_BOOT */
}

#if !CCG_FIRMWARE_APP_ONLY
ATTRIBUTES static uint8_t* sys_get_fw_version(sys_fw_metadata_t *fw_metadata)
{
    /*
     * FW stores its version at an offset of CY_PD_FW_VERSION_OFFSET
     * from its start address. Read the version from that location.
     * The start address of FW is determined from the boot_last_row
     * field of the FW metadata.
     */
    uint32_t addr = ((fw_metadata->boot_last_row << CCG_FLASH_ROW_SHIFT_NUM)
            + CCG_FLASH_ROW_SIZE + SYS_FW_VERSION_OFFSET);
    return (uint8_t *)addr;
}
#endif /* CCG_FIRMWARE_APP_ONLY */

ATTRIBUTES uint8_t* sys_get_boot_version(void)
{
    return (uint8_t *)SYS_BOOT_VERSION_ADDRESS;
}

ATTRIBUTES uint8_t* sys_get_img1_fw_version(void)
{
#if !CCG_FIRMWARE_APP_ONLY
    /* Check if Image 1 is valid. */
    if ((CALL_OUT_FUNCTION(get_boot_mode_reason)()).status.fw1_invalid == 0)
    {
        return sys_get_fw_version (GET_IN_VARIABLE(gl_img1_fw_metadata));
    }
    else
    {
        return (GET_IN_VARIABLE(gl_invalid_version));
    }
#else
    return (GET_IN_VARIABLE(gl_invalid_version));
#endif /* CCG_FIRMWARE_APP_ONLY */
}

#if (!CCG_DUALAPP_DISABLE)    
ATTRIBUTES uint8_t* sys_get_img2_fw_version(void)
{
#if !CCG_FIRMWARE_APP_ONLY
    /* Check if Image 2 is valid. */
    if ((CALL_OUT_FUNCTION(get_boot_mode_reason)()).status.fw2_invalid == 0)
    {
        return sys_get_fw_version (GET_IN_VARIABLE(gl_img2_fw_metadata));
    }
    else
    {
    return (GET_IN_VARIABLE(gl_invalid_version));
    }
#else
    return (GET_IN_VARIABLE(gl_invalid_version));
#endif /* CCG_FIRMWARE_APP_ONLY */
}
#endif /* (!CCG_DUALAPP_DISABLE) */

ATTRIBUTES uint32_t sys_get_fw_img1_start_addr(void)
{
#if !CCG_FIRMWARE_APP_ONLY
    uint32_t bl_last;

    /* Check if Image 1 is valid. */
    if (((CALL_OUT_FUNCTION(get_boot_mode_reason)()).status.fw1_invalid) != 1)
    {        
        bl_last = GET_IN_VARIABLE(gl_img1_fw_metadata)->boot_last_row;
        return ((bl_last << CCG_FLASH_ROW_SHIFT_NUM) + CCG_FLASH_ROW_SIZE);
    }
    else
    {
        /* Image 1 is invalid. Can't use MD Row info. Use Boot last row info
         * from flash configuration and response. */
        return ((CCG_BOOT_LOADER_LAST_ROW << CCG_FLASH_ROW_SHIFT_NUM)
                + CCG_FLASH_ROW_SIZE);
    }
#else
    return SYS_INVALID_FW_START_ADDR;
#endif /* CCG_FIRMWARE_APP_ONLY */
}

#if (!CCG_DUALAPP_DISABLE) 
ATTRIBUTES uint32_t sys_get_fw_img2_start_addr(void)
{
#if !CCG_FIRMWARE_APP_ONLY
    uint32_t bl_last;

    /* Check if Image 2 is valid. */
    if (((CALL_OUT_FUNCTION(get_boot_mode_reason)()).status.fw2_invalid) != 1)
    {
        bl_last = GET_IN_VARIABLE(gl_img2_fw_metadata)->boot_last_row;
        return ((bl_last << CCG_FLASH_ROW_SHIFT_NUM) + CCG_FLASH_ROW_SIZE);
    }
    else
    {
        /* Image 2 is invalid. Can't use MD Row info. Use last row FW1 info info
         * from flash configuration and response. */
        return ((CCG_IMG1_LAST_FLASH_ROW_NUM << CCG_FLASH_ROW_SHIFT_NUM)
                + CCG_FLASH_ROW_SIZE);
    }
#else
    return SYS_INVALID_FW_START_ADDR;
#endif /* CCG_FIRMWARE_APP_ONLY */
}

ATTRIBUTES uint8_t sys_get_recent_fw_image(void)
{
#if (APP_PRIORITY_FEATURE_ENABLE == 1)
    /* Read APP Priority field to determine which image gets the priority. */
    uint8_t app_priority;

    app_priority = *((uint8_t *)(CCG_APP_PRIORITY_ROW_NUM << CCG_FLASH_ROW_SHIFT_NUM));
    if (app_priority != 0)
    {
        if (app_priority == 0x01)
            return SYS_FW_MODE_FWIMAGE_1;
        else
            return SYS_FW_MODE_FWIMAGE_2;
    }
    else
#endif /* !APP_PRIORITY_FEATURE_ENABLE */
    {
        /* Prioritize FW2 over FW1 if no priority is defined. */
        if ((GET_IN_VARIABLE(gl_img2_fw_metadata))->boot_seq >= (GET_IN_VARIABLE(gl_img1_fw_metadata))->boot_seq)
            return SYS_FW_MODE_FWIMAGE_2;
        else
            return SYS_FW_MODE_FWIMAGE_1;
    }
}
#endif /* (!CCG_DUALAPP_DISABLE) */

ATTRIBUTES void sys_get_silicon_id(uint32_t *silicon_id)
{
    /* Read Silicon ID. */
    *silicon_id = SFLASH->silicon_id;
}

ATTRIBUTES uint8_t get_silicon_revision(void)
{

    uint8_t srev;

    /* Silicon revision is stored in core-sight tables.
     * Major revision = ROMTABLE_PID2(7:4)
     * Minor revision = ROMTABLE_PID3(7:4) */
    srev = CY_GET_REG32 (CYREG_ROMTABLE_PID2) & 0xF0;
    srev |= ((CY_GET_REG32 (CYREG_ROMTABLE_PID3)) & 0xF0) >> 4;
    return srev;
}

ATTRIBUTES uint32_t sys_get_custom_info_addr(void)
{
    uint32_t addr;

    /* Get FW start address based on FW image. */
#if (CCG_DUALAPP_DISABLE)
    addr = CALL_IN_FUNCTION(sys_get_fw_img1_start_addr)();
#else /* (!CCG_DUALAPP_DISABLE) */
    if (CALL_IN_FUNCTION(sys_get_device_mode)() == SYS_FW_MODE_FWIMAGE_1)
    {
        addr = CALL_IN_FUNCTION(sys_get_fw_img1_start_addr)();
    }
    else
    {
        addr = CALL_IN_FUNCTION(sys_get_fw_img2_start_addr)();
    }
#endif /* (!CCG_DUALAPP_DISABLE) */

    return (addr + SYS_FW_CUSTOM_INFO_OFFSET);
}

ATTRIBUTES uint16_t sys_get_bcdDevice_version(uint32_t ver_addr)
{
    /*     
     * bcdDevice version is defined and derived from FW version as follows:
     * Bit 0:3   = FW_MINOR_VERSION
     * Bit 4:6   = FW_MAJOR_VERSION (lower 3 bits)
     * Bit 7:13  = APP_EXT_CIR_NUM (lower 7 bits)
     * Bit 14:15 = APP_MAJOR_VERSION (lower 2 bits)
     * FW version is stored at a fixed offset in FLASH in this format:
     * 4 Byte BASE_VERSION, 4 Byte APP_VERSION
     */
    
    uint8_t base_major_minor = *((uint8_t *)(ver_addr + 3));
    uint8_t app_cir_num = *((uint8_t *)(ver_addr + 6));
    uint8_t app_major_minor = *((uint8_t *)(ver_addr + 7));
    
    return ((base_major_minor & 0x7F) | ((app_cir_num & 0x7F) << 7)
        | ((app_major_minor & 0x30) << 10));
}

/* [] END OF FILE */
