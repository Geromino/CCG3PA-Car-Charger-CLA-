/**
 * @file cy_alt_mode.c
 *
 * @brief @{CY Alternate Mode Module Source File.@}
 *
 *******************************************************************************
 *
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

#include <project.h>
#include <stdbool.h>
#include <stdint.h>
#include <config.h>
#include <pd.h>
#include <alt_modes_mngr.h>
#include <flash.h>
#include <uvdm.h>
#include <cy_alt_mode.h>

alt_mode_info_t gl_cy_alt_mode_handle[NO_OF_TYPEC_PORTS];

static void cy_alt_mode_cbk(uint8_t port)
{
    /* Handle ENTER and EXIT Mode command. */
    switch (gl_cy_alt_mode_handle[port].vdm_header.std_vdm_hdr.cmd)
    {
        case VDM_CMD_ENTER_MODE:
            gl_cy_alt_mode_handle[port].is_active = true;
            gl_cy_alt_mode_handle[port].mode_state = ALT_MODE_STATE_IDLE;
            /* Notify CY UVDM Module. */
            uvdm_enter_cy_alt_mode ();
            break;
        case VDM_CMD_EXIT_MODE:
        case EXIT_ALL_MODES:
            if (gl_cy_alt_mode_handle[port].is_active == true)
            {
                gl_cy_alt_mode_handle[port].is_active = false;
                gl_cy_alt_mode_handle[port].mode_state = ALT_MODE_STATE_EXIT;
                /* Notify CY UVDM Module. */
                uvdm_exit_cy_alt_mode ();
            }
            break;
        default:
            gl_cy_alt_mode_handle[port].mode_state = ALT_MODE_STATE_FAIL;
            break;
    }
}

alt_mode_info_t* cy_alt_mode_handler(uint8_t port, alt_mode_reg_info_t* disc_mode_info)
{
    /* Check the value of Mode VDO. It has to be Value of CY Alternate Flashing mode. */
    if (disc_mode_info->svid_vdo.val == CY_ALT_FLASHING_MODE_VAL)
    {
        /* Setup the handler. */
        gl_cy_alt_mode_handle[port].cbk = cy_alt_mode_cbk;
        gl_cy_alt_mode_handle[port].vdm_header.std_vdm_hdr.svid = CY_VID;
        disc_mode_info->alt_mode_id = CY_ALT_FLASHING_MODE_VAL;
        return &gl_cy_alt_mode_handle[port];
    }
    else
    {
        disc_mode_info->alt_mode_id = MODE_NOT_SUPPORTED;
        return NULL;
    }
}

/* End of File */
