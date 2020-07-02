/**
 * @file custom_hpi.c
 *
 * @brief @{Custom HPI alternate mode handler source file.@}
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

#include <alt_modes_mngr.h>
#include <custom_hpi_vid.h>
#include <dpm.h>
#include <app.h>
#include <config.h>

#if (HPI_AM_SUPP) && (DFP_ALT_MODE_SUPP)

typedef struct
{
    alt_mode_info_t info;
    hpi_am_state_t state;
    pd_do_t vdo[MAX_HPI_AM_VDO_NUMB];
    uint8_t max_sop_supp;
    uint16_t custom_hpi_svid;
    const atch_tgt_info_t* tgt_info_ptr;
}hpi_am_status;

hpi_am_status hpi_am[NO_OF_TYPEC_PORTS];

/* Composes VDM for sending by alt mode manager */
static void send_cmd(uint8_t port);
/* Inits HPI alt mode */
static void init_hpi_am(uint8_t port);
/* Exits HPI alt mode */
static void hpi_am_exit(uint8_t port);
/* Main HPI alt mode source handling functions */
static void hpi_am_dfp_run(uint8_t port);

/* Evaluates command received from application */
static bool hpi_am_eval_app_cmd(uint8_t port, alt_mode_evt_t cmd_data);
/* Returns pointer to alt mode info structure */
static alt_mode_info_t* hpi_am_info(uint8_t port);

/************************** Function definitions *****************************/

alt_mode_info_t* reg_hpi_modes(uint8_t port, alt_mode_reg_info_t* reg_info)
{
    /* Save custom HPI SVID */
    hpi_am[port].custom_hpi_svid = app_get_status(port)->custom_hpi_svid;
    if (
            (hpi_am[port].custom_hpi_svid != NO_DATA) &&
            (reg_info->data_role == PRT_TYPE_DFP)
        )
    {
        /* Reset alt mode info struct */
        reset_alt_mode_info(&(hpi_am[port].info));
        /* Copy cable, device/AMA info pointer */
        hpi_am[port].tgt_info_ptr = reg_info->atch_tgt_info;
        hpi_am[port].max_sop_supp = SOP;
        /* Init HPI alt mode */
        hpi_am_info(port)->mode_state   = ALT_MODE_STATE_INIT;
        hpi_am_info(port)->vdo_max_numb = MAX_HPI_AM_VDO_NUMB;
        hpi_am_dfp_run(port);

        return &hpi_am[port].info;
    }
    else
    {
        return NULL;
    }
}


static void hpi_am_dfp_run(uint8_t port)
{
    switch (hpi_am_info(port)->mode_state)
    {
        case ALT_MODE_STATE_INIT:
            init_hpi_am(port);
            send_cmd(port);
            break;

        case ALT_MODE_STATE_WAIT_FOR_RESP:
            hpi_am[port].state = (hpi_am_state_t)hpi_am_info(port)->VDM_HDR.cmd;
            hpi_am_info(port)->mode_state = ALT_MODE_STATE_IDLE;
            switch (hpi_am[port].state)
            {
                case HPI_AM_STATE_ENTER:
                    hpi_am[port].state = HPI_AM_STATE_IDLE;
                    break;

                case HPI_AM_STATE_EXIT:
                    hpi_am_exit(port);
                    break;

                default:
                    break;
            }
            return;

        case ALT_MODE_STATE_FAIL:
            switch (hpi_am[port].state)
            {
                case HPI_AM_STATE_ENTER:
                case HPI_AM_STATE_EXIT:
                    hpi_am_exit(port);
                    break;

                default:
                    break;
            }
            break;

        case ALT_MODE_STATE_EXIT:
            hpi_am[port].state = HPI_AM_STATE_EXIT;
            send_cmd(port);
            break;

        default:
            break;
    }
}


static void init_hpi_am(uint8_t port)
{
    hpi_am_info(port)->sop_state[SOP]  = ALT_MODE_STATE_SEND;
    hpi_am_info(port)->vdo[SOP]        = hpi_am[port].vdo;
    hpi_am_info(port)->cbk             = hpi_am_dfp_run;
    hpi_am_info(port)->eval_app_cmd    = hpi_am_eval_app_cmd;
    hpi_am_info(port)->set_mux_isolate = false;
    /* Set HPI alt mode state as enter */
    hpi_am[port].state = HPI_AM_STATE_ENTER;
}

static void hpi_am_exit(uint8_t port)
{
    hpi_am_info(port)->mode_state = ALT_MODE_STATE_EXIT;
}


static alt_mode_info_t* hpi_am_info(uint8_t port)
{
    return &(hpi_am[port].info);
}

static void send_cmd(uint8_t port)
{
    if (hpi_am[port].state != HPI_AM_STATE_IDLE)
    {
        hpi_am_info(port)->vdm_header.val = NO_DATA;
        hpi_am_info(port)->VDM_HDR.cmd    = (uint32_t)hpi_am[port].state;
        hpi_am_info(port)->VDM_HDR.svid   = hpi_am[port].custom_hpi_svid;
    
        hpi_am_info(port)->sop_state[SOP] = ALT_MODE_STATE_SEND;
        hpi_am_info(port)->mode_state = ALT_MODE_STATE_SEND;
    }
}


/******************* HPI alt mode application Related Functions ************************/

/* Activates only when application HPI alt mode command is received */
static bool hpi_am_eval_app_cmd(uint8_t port, alt_mode_evt_t cmd_data)
{
    (void)port;
    (void)cmd_data;

    return true;
}

#endif /* (HPI_AM_SUPP) && (DFP_ALT_MODE_SUPP) */


/* [] END OF FILE */
