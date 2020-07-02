/**
 * @file swap.c
 *
 * @brief @{Swap request (PR_SWAP, DR_SWAP, VCONN_SWAP) handlers.@}
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

#include <config.h>
#include <pd.h>
#include <dpm.h>
#include <swap.h>
#include <app.h>

#if ICL_RVP_HW
#include <icl.h>    
extern bool gl_block_swaps;   
#endif

static app_req_status_t get_response(uint8_t port, uint8_t raw_resp)
{
#if CCG_PD_REV3_ENABLE
    const dpm_status_t* dpm = dpm_get_info(port);
#endif /* CCG_PD_REV3_ENABLE */

    app_req_status_t retVal;
    switch(raw_resp)
    {
    case APP_RESP_ACCEPT:
        retVal = REQ_ACCEPT;
        break;
    case APP_RESP_WAIT:
        retVal =  REQ_WAIT;
        break;
#if CCG_PD_REV3_ENABLE
    case APP_RESP_NOT_SUPPORTED:
        retVal = REQ_NOT_SUPPORTED;
        break;
#endif /* CCG_PD_REV3_ENABLE */
    default:
        retVal = REQ_REJECT;
        break;
    }

#if CCG_PD_REV3_ENABLE
    if(dpm->spec_rev_sop_live <= PD_REV2)
    {
        if(retVal == REQ_NOT_SUPPORTED)
        {
            retVal = REQ_REJECT;
        }
    }
#endif /* CCG_PD_REV3_ENABLE */
    return retVal;
}

#if (ROLE_PREFERENCE_ENABLE)
/* Variable storing current preference for power role. */
extern volatile uint8_t app_pref_power_role[NO_OF_TYPEC_PORTS];
/* Variable storing current preference for data role. */
extern volatile uint8_t app_pref_data_role[NO_OF_TYPEC_PORTS];
#endif /* (ROLE_PREFERENCE_ENABLE) */

void eval_dr_swap (uint8_t port, app_resp_cbk_t app_resp_handler)
{
    const dpm_status_t* dpm = dpm_get_info(port);
    app_req_status_t result = REQ_REJECT;

    if (app_get_status(port)->alt_mode_entered == true)
    {
        result = REQ_SEND_HARD_RESET;
    }
    else
    {
#if (ROLE_PREFERENCE_ENABLE)
        /* Don't accept DR_SWAP away from preferred role. */
        if (dpm_get_info(port)->cur_port_type != app_pref_data_role[port])
#endif /* (ROLE_PREFERENCE_ENABLE) */
        {
            result = get_response(port, GET_DR_SWAP_RESP(dpm->swap_response));
#if ICL_RVP_HW
            /* If we're blocking swaps (until cable discovery is finished), reject DR_Swap */
            if (gl_block_swaps == true)
                result = REQ_REJECT;
#endif /* ICL_RVP_HW */            
        }
    }

#if ICL_SLAVE_ENABLE
    if(result == REQ_ACCEPT)
        set_mux (port, MUX_CONFIG_SAFE, 0);
#endif /* ICL_SLAVE_ENABLE */

    app_get_resp_buf(port)->req_status = result;
    app_resp_handler(port, app_get_resp_buf(port));
}

void eval_pr_swap (uint8_t port, app_resp_cbk_t app_resp_handler)
{
#if (CCG_PD_REV3_ENABLE || ((!(CCG_SOURCE_ONLY)) && (!CCG_SINK_ONLY)))
    const dpm_status_t* dpm = dpm_get_info(port);
#endif /* (CCG_PD_REV3_ENABLE || ((!(CCG_SOURCE_ONLY)) && (!CCG_SINK_ONLY))) */
    app_req_status_t result = REQ_REJECT;

#if ((!CCG_SOURCE_ONLY) && (!ICL_RVP_HW) && (!CCG_SINK_ONLY))
    uint8_t pdo_mask;

    if(dpm->cur_port_role == PRT_ROLE_SOURCE)
    {
        pdo_mask = dpm->src_pdo_mask;
    }
    else
    {
        pdo_mask = dpm->snk_pdo_mask;
    }
#endif /* ((!CCG_SOURCE_ONLY) && (!ICL_RVP_HW) && (!CCG_SINK_ONLY)) */

    /*
     * Default response shall be NOT_SUPPORTED instead of REJECT if current mode
     * is PD REV3 and port role is source/sink only.
     */
#if CCG_PD_REV3_ENABLE
    if ((dpm_get_info(port)->spec_rev_sop_live >= PD_REV3) &&
        (dpm->port_role != PRT_DUAL))
    {
        result = REQ_NOT_SUPPORTED;
    }
#endif /* CCG_PD_REV3_ENABLE */

#if ((!(CCG_SOURCE_ONLY)) && (!CCG_SINK_ONLY))
#if (ROLE_PREFERENCE_ENABLE)
    /* Do not allow PR_SWAP to a non-preferred role. */
    if (app_pref_power_role[port] != dpm->cur_port_role)
#endif /* (ROLE_PREFERENCE_ENABLE) */
    {
        if ((dpm->dead_bat == false) && (dpm->port_role == PRT_DUAL) 
        #if !ICL_RVP_HW /* Don't do externally powered bit check, to pass UCSI tests */        
            && (((pdo_mask & (0x1 << PD_EXTERNALLY_POWERED_BIT_POS)) == 0) || (dpm->cur_port_role == PRT_ROLE_SINK)
            || (((pdo_mask & (0x1 << PD_EXTERNALLY_POWERED_BIT_POS)) != 0) && (pd_get_ptr_host_cfg_tbl(port)->ext_powered_prs != 0)))
        #endif /*!ICL_RVP_HW*/
            )
        {
            result = get_response(port, GET_PR_SWAP_RESP(dpm->swap_response));
        }
    }
#if (ROLE_PREFERENCE_ENABLE)
    else
    {
        result = REQ_REJECT;
    }
#endif /* (ROLE_PREFERENCE_ENABLE) */
#endif /* ((!(CCG_SOURCE_ONLY)) && (!CCG_SINK_ONLY)) */

#if PROCHOT_SUPP
    if((result == REQ_ACCEPT) && !ADP_DETECT_Read())
        PROCHOT_Write(0);
#endif /* PROCHOT_SUPP */

    app_get_resp_buf(port)->req_status = result;
    app_resp_handler(port, app_get_resp_buf(port));
}

void eval_vconn_swap (uint8_t port, app_resp_cbk_t app_resp_handler)
{
    const dpm_status_t* dpm = dpm_get_info(port);
    app_req_status_t result = REQ_REJECT;

#if ICL_ENABLE && ICL_RVP_HW
    if((dpm->vconn_logical == true) && !gl_block_swaps)
#else    
    if (dpm->vconn_logical == true)
#endif /* ICL_ENABLE */    
    {
        /* Allow VConn Swap to Sink if app. level keep vconn src setting is not ON. */
#if (DEFER_VCONN_SRC_ROLE_SWAP)
        if (app_get_status(port)->keep_vconn_src)
        {
            result = REQ_WAIT;
        }
        else
#endif /* (DEFER_VCONN_SRC_ROLE_SWAP) */
        {
            result = REQ_ACCEPT;
        }
    }
    else
    {
        result = get_response(port, GET_VCONN_SWAP_RESP(dpm->swap_response));

        if (result == REQ_ACCEPT)
        {
#if ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6)))
            if (!pd_is_v5v_supply_on (port))
            {
                result = REQ_WAIT;
            }
#endif /* ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6))) */

#if VCONN_OCP_ENABLE
            /* Do not allow VCONN_SWAP to become VConn source if fault is active. */
            if ((app_get_status(port)->fault_status & APP_PORT_VCONN_FAULT_ACTIVE) != 0)
            {
                result = REQ_REJECT;
            }
#endif /* VCONN_OCP_ENABLE */
        }
    }

#if ICL_RVP_HW
    if(icl_vsys_is_present() == false)
    {
        result = REQ_REJECT;
    }
#endif /* ICL_RVP_HW */    
    
    app_get_resp_buf(port)->req_status = result;
    app_resp_handler(port, app_get_resp_buf(port));
}

#if ((!(CCG_SOURCE_ONLY)) && (!CCG_SINK_ONLY))
#if CCG_PD_REV3_ENABLE

void eval_fr_swap (uint8_t port, app_resp_cbk_t app_resp_handler)
{
    /* Always accept, FRS support is enabled/disabled by separate bit in config table */
    app_req_status_t result = REQ_ACCEPT;

    app_get_resp_buf(port)->req_status = result;
    app_resp_handler(port, app_get_resp_buf(port));

}

#endif /* CCG_PD_REV3_ENABLE */
#endif /* ((!(CCG_SOURCE_ONLY)) && (!CCG_SINK_ONLY)) */

 /* End of File */

