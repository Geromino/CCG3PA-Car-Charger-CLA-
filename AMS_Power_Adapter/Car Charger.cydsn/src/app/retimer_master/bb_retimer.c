/**
 * @file bb_retimer.c
 *
 * @brief @{Burnside/Delta bridge retimer handling functions.@}
 *
 *******************************************************************************
 *
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

#include <pd.h>
#include <dpm.h>
#include <app.h>
#include <timer.h>

#if BB_RETIMER_ENABLE
#include <bb_retimer.h>
#include <ridge_slave.h>
    
#define RETIMER_MAX_RETRIES                   (1u)
        
#if BB_RETIMER_FORCE_PWR_INP_SUPP
static void retimer_force_pwr_change_handler(void);
#endif /* BB_RETIMER_FORCE_PWR_INP_SUPP */

static struct {
    uint8_t  debug_poll_count;   /**< Debug mode read retry count */
    uint8_t  retry_count;        /**< Status update retry count */
    uint32_t debug_mode;         /**< Debug Mode data from SoC */
    uint32_t last_status;        /**< Latest status register data */
    uint32_t last_value;         /**< Latest register data */
    uint8_t  last_register;      /**< Latest register address */
#if RETIMER_CFG_TBL_SUPP == 0    
    uint8_t  address[MAX_NO_OF_RETIMERS]; /**< If config table address not used, assign addresses locally */
#endif
} retimer[NO_OF_TYPEC_PORTS];

static uint8_t gl_retimer_active = false;
static uint8_t gl_host_compliance_mode = false;
static volatile uint32_t gl_retimer_evts[NO_OF_TYPEC_PORTS] = {0
#if CCG_PD_DUALPORT_ENABLE
    , 
    0
#endif /* CCG_PD_DUALPORT_ENABLE */    
};

/* Start the retimer interface */
void retimer_start()
{
    RTM_Start();
#if BB_RETIMER_FORCE_PWR_INP_SUPP
    Retimer_ForcePwr_StartEx(retimer_force_pwr_change_handler);
#endif /* BB_RETIMER_FORCE_PWR_INP_SUPP */
}

/* Stop retimer interfaces */
void retimer_stop()
{
    RTM_Stop();
#if BB_RETIMER_FORCE_PWR_INP_SUPP
    Retimer_ForcePwr_Stop();
#endif /* BB_RETIMER_FORCE_PWR_INP_SUPP */
}

/* Try to initialize the retimer interface */
void retimer_init()
{
    /* Retimer cannot be started until VSYS (i.e its power) is available.
     * retimer_task() looks for VSYS addition and removal and handles start/stop */
}

/* Signal an event within */
void retimer_set_evt(uint8_t port, rt_evt_t evt)
{
    uint8_t intr = CyEnterCriticalSection();
    gl_retimer_evts[port] |= (1 << evt);
    CyExitCriticalSection(intr);
}

/* Clear an event within */
void retimer_clr_evt(uint8_t port, rt_evt_t evt)
{
    uint8_t intr = CyEnterCriticalSection();
    gl_retimer_evts[port] &= ~(1 << evt);
    CyExitCriticalSection(intr);
}

#if RETIMER_CFG_TBL_SUPP == 0
/* If retimer slave address is not fetched by config table, set addresses based on I2C_CFG pin */
void retimer_set_slave_address(uint8_t base_addr)
{
    uint8_t port;
    for(port = 0; port < NO_OF_TYPEC_PORTS; port++)
        retimer[port].address[0] = base_addr + port;
}
#endif

/* Enable power and release reset on retimer */
void retimer_enable(uint8_t port)
{
    /* Enable retimer when retimer is active */
    if(gl_retimer_active)
    {
        if(port == TYPEC_PORT_0_IDX)
        {
            RETIMER_PWR_EN_P1_Write(1);
    #if RETIMER_CFG_TBL_SUPP
            CyDelayUs(get_pd_port_config(port)->retimer_on_delay * 100);
    #else        
            CyDelayUs(1050);
    #endif
            RETIMER_RESET_N_P1_Write(1);
        }
        else
        {
            RETIMER_PWR_EN_P2_Write(1);
    #if RETIMER_CFG_TBL_SUPP
            CyDelayUs(get_pd_port_config(port)->retimer_on_delay * 100);
    #else
            CyDelayUs(1050);
    #endif
            RETIMER_RESET_N_P2_Write(1);
        }
    }
}

/* Disable retimer power and assert reset */
void retimer_disable(uint8_t port)
{
#if BB_RETIMER_FORCE_PWR_INP_SUPP
    if (RETIMER_FORCE_PWR_EN_Read())
        return;
#endif /* BB_RETIMER_FORCE_PWR_INP_SUPP */

    if(port == TYPEC_PORT_0_IDX)
    {
        RETIMER_RESET_N_P1_Write(0);
        CyDelayUs(1);
        RETIMER_PWR_EN_P1_Write(0);
    }
    else
    {
        RETIMER_RESET_N_P2_Write(0);
        CyDelayUs(1);
        RETIMER_PWR_EN_P2_Write(0);
    }
}

#if BB_RETIMER_FORCE_PWR_INP_SUPP
static void retimer_force_pwr_change_handler()
{
    retimer_set_evt (0, RT_EVT_FORCE_PWR_CHNG);
    retimer_set_evt (1, RT_EVT_FORCE_PWR_CHNG);

    Retimer_ForcePwr_ClearPending();
    RETIMER_FORCE_PWR_EN_ClearInterrupt();
}

#endif /* BB_RETIMER_FORCE_PWR_INP_SUPP */

void retimer_set_compl_mode(uint8_t compl_mode)
{
    gl_host_compliance_mode = compl_mode;
}

#if RETIMER_MANUAL_VSYS_SAMPLE
static uint8_t vsys_is_available()
{
    /* Only call this function when I2C is idle */
    return HPI_IF_scl_Read() && HPI_IF_sda_Read();
}
#endif /* RETIMER_MANUAL_VSYS_SAMPLE */

/* Function called when SoC writes to retimer */
void retimer_start_debug_poll(uint8_t port, uint8_t* debug_mode_data)
{
    memcpy((uint8_t*)&retimer[port].debug_mode, debug_mode_data, 4);
    retimer[port].debug_poll_count = BB_DEBUGMODE_POLL_COUNT;
    retimer_set_evt(port, RT_EVT_WRITE_DEBUG_MODE);
}

/* I2C writes to retimer */
static bool retimer_write(uint8_t slave_address, uint8_t offset, uint8_t* data, uint8_t num_bytes)
{
    if(RTM_I2CMasterSendStart(slave_address, RTM_I2C_WRITE_XFER_MODE, RETIMER_I2C_TIMEOUT) != RTM_I2C_MSTR_NO_ERROR)
        goto end_transfer;
    
    if(RTM_I2CMasterWriteByte(offset, RETIMER_I2C_TIMEOUT) != RTM_I2C_MSTR_NO_ERROR)
        goto end_transfer;

    if(RTM_I2CMasterWriteByte(num_bytes, RETIMER_I2C_TIMEOUT) != RTM_I2C_MSTR_NO_ERROR)
        goto end_transfer;

    while(num_bytes--)
        if(RTM_I2CMasterWriteByte(*data++, RETIMER_I2C_TIMEOUT) != RTM_I2C_MSTR_NO_ERROR)
            goto end_transfer;

    if(RTM_I2CMasterSendStop(RETIMER_I2C_TIMEOUT) != RTM_I2C_MSTR_NO_ERROR)
        return false;
    
    return true;
    
end_transfer:
    RTM_I2CMasterSendStop(RETIMER_I2C_TIMEOUT);
    return false;
}

#if BB_RETIMER_DEBUG_MODE_SUPP
/* I2C read from retimer */
static bool retimer_read(uint8_t slave_address, uint8_t offset, uint8_t* data_buffer, uint8_t num_bytes)
{
    uint8_t rd_byte[num_bytes];
    uint32_t status;
    
    if(RTM_I2CMasterSendStart(slave_address, RTM_I2C_WRITE_XFER_MODE, RETIMER_I2C_TIMEOUT) != RTM_I2C_MSTR_NO_ERROR)
        goto end_transfer;
    
    if(RTM_I2CMasterWriteByte(offset, RETIMER_I2C_TIMEOUT) != RTM_I2C_MSTR_NO_ERROR)
        goto end_transfer;
    
    if(RTM_I2CMasterSendRestart(slave_address, RTM_I2C_READ_XFER_MODE, RETIMER_I2C_TIMEOUT) != RTM_I2C_MSTR_NO_ERROR)
        goto end_transfer;

    status = RTM_I2CMasterReadByte(RTM_I2C_ACK_DATA, rd_byte, RETIMER_I2C_TIMEOUT);
    if(status != RTM_I2C_MSTR_NO_ERROR)
        goto end_transfer;

    while(num_bytes--)
    {
        status = RTM_I2CMasterReadByte(num_bytes ? RTM_I2C_ACK_DATA : RTM_I2C_NAK_DATA, &rd_byte[num_bytes], RETIMER_I2C_TIMEOUT );
        if(status != RTM_I2C_MSTR_NO_ERROR)
            *data_buffer++ = rd_byte[num_bytes];
        else
            goto end_transfer;
    }

    if(RTM_I2CMasterSendStop(RETIMER_I2C_TIMEOUT) != RTM_I2C_MSTR_NO_ERROR)
        return false;
    
    return true;
        
end_transfer:
    RTM_I2CMasterSendStop(RETIMER_I2C_TIMEOUT);
    return false;
}
#endif /* BB_RETIMER_DEBUG_MODE_SUPP */

bool retimer_update_is_pending(uint8_t port)
{
    return timer_is_running(port, BB_DBR_WAIT_TIMER);
}

uint8_t retimer_sleep_allowed()
{
    return (gl_retimer_evts[0] || gl_retimer_evts[1]
            || retimer_update_is_pending(0) || retimer_update_is_pending(1)) == 0;
}

void retimer_timer_cbk(uint8_t port, timer_id_t id)
{
    switch(id)
    {
        case BB_DBR_WAIT_TIMER:
            retimer_set_evt(port, RT_EVT_UPDATE_STATUS);
            break;

        case BB_DBR_DEBUG_POLL_TIMER:
            retimer_set_evt(port, RT_EVT_READ_DEBUG_MODE);
            break;
    }
}

bool retimer_reg_write(uint8_t port, uint8_t reg, uint32_t value)
{    
    bool write_status = false;
    
    if ((retimer[port].last_register != reg) || (retimer[port].last_value != value))
        retimer[port].retry_count = 0;

    /* Store for retries */
    retimer[port].last_value = value;
    retimer[port].last_register = reg;
    
    /* If retimer is booting/waking up, wait until the prescribed time and retry */
    if (timer_is_running(port, BB_DBR_WAIT_TIMER))
    {
        retimer_set_evt(port, RT_EVT_RETRY_STATUS);
        return false;
    }
    
    /* If retimer is not powered, don't proceed */
    if(!gl_retimer_active)
    {
        retimer_set_evt(port, RT_EVT_PWR_NOT_AVAILABLE);
        return false;
    }
    
    if (reg == BB_CONN_STATE_REG)
    {
        /* Clear reserved bits since BB register uses only a subset */
        value &= BB_STATUS_MASK;
        
        /* Set system state */
        if(gl_system_state != SYSTEM_STATE_S0)
            value |= BB_STATUS_Sx_ACTIVE;
    }
    
    write_status = retimer_write(retimer[port].address[0], reg, (uint8_t*)&value, sizeof(value));

    if(write_status) /* Write succeeded */
    {
        retimer[port].retry_count = 0;
        retimer_clr_evt(port, RT_EVT_RETRY_STATUS);
    }
    else if(retimer[port].retry_count != RETIMER_MAX_RETRIES) /* Write failed. Retry if possible */
    {
        retimer[port].retry_count++;
        retimer_set_evt(port, RT_EVT_RETRY_STATUS);
    }

    return write_status;
}

/* Update retimer TBT status register */
bool retimer_status_update(uint8_t port, uint32_t status, bool force_update)
{
    /* Only proceed if status has changed, or previous update is being retried */
    if(!force_update && (retimer[port].last_status == status))
        return true;
    
    retimer[port].last_status = status;
    
    return retimer_reg_write (port, BB_CONN_STATE_REG, status);
}

/* Handle pending retimer tasks */
void retimer_task(uint8_t port)
{
    rt_evt_t evt;
    
    evt = event_group_get_event(&gl_retimer_evts[port], true);
    
    switch(evt)
    {
        case RT_EVT_VSYS_ADDED:
            if(!gl_retimer_active)
                retimer_start();
                
            /* Make sure that it is enabled before retimer_enable(port)*/
            gl_retimer_active = true;

            /* If device is already attached when VSYS is ready, retry status update
    		 * after some time (give time for retimer to load its firmware) */
    		if (dpm_get_info(port)->attach)
            {
                retimer_enable(port);
                retimer_set_evt(port, RT_EVT_RETRY_STATUS);
            }
            
            retimer[port].retry_count = 0;
            retimer_clr_evt(port, RT_EVT_PWR_NOT_AVAILABLE);
            
            break;

        case RT_EVT_VSYS_REMOVED:
            retimer[port].retry_count = 0;
            retimer_disable(port);
            if(gl_retimer_active)
                retimer_stop();
            gl_retimer_active = false;
            retimer_set_evt(port, RT_EVT_PWR_NOT_AVAILABLE);
            break;

        case RT_EVT_PWR_NOT_AVAILABLE:
#if RETIMER_MANUAL_VSYS_SAMPLE
            /* Manual override to auto VSYS sampling check */
            if (vsys_is_available())
                retimer_set_evt(port, RT_EVT_VSYS_ADDED);
            else
                retimer_clr_evt(port, RT_EVT_VSYS_ADDED);
#endif /* RETIMER_MANUAL_VSYS_SAMPLE */
            /* Do not process further events if VSYS is not available */
            retimer_set_evt(port, RT_EVT_PWR_NOT_AVAILABLE);
            break;

        case RT_EVT_RETRY_STATUS:
            if(timer_is_running(port, BB_DBR_WAIT_TIMER) == false)
                timer_start(port, BB_DBR_WAIT_TIMER, BB_DBR_WAKEUP_DELAY, retimer_timer_cbk);
            break;

        case RT_EVT_UPDATE_STATUS:
            retimer_reg_write (port, retimer[port].last_register, retimer[port].last_value);
            break;
            
#if BB_RETIMER_FORCE_PWR_INP_SUPP
        case RT_EVT_FORCE_PWR_CHNG:
            /* If Force_Pwr is high, we're in 'host compliance' mode */
            gl_host_compliance_mode = RETIMER_FORCE_PWR_EN_Read();

            if(gl_host_compliance_mode)
                retimer_enable(port);

            /* Set compliance mode */
            retimer[port].debug_poll_count = 0;
            retimer[port].debug_mode = gl_host_compliance_mode ? BB_DEBUG_MODE_USB_COMPLIANCE : BB_DEBUG_MODE_CLEAR;
            retimer_set_evt(port, RT_EVT_WRITE_DEBUG_MODE);
            break;
#endif /* BB_RETIMER_FORCE_PWR_INP_SUPP */
            
#if BB_RETIMER_DEBUG_MODE_SUPP
        case RT_EVT_WRITE_DEBUG_MODE:
            retimer_reg_write (port, BB_DEBUG_MODE_REG, retimer[port].debug_mode);
            
            if (retimer[port].debug_poll_count != 0)
            {
                retimer_set_evt(port, RT_EVT_READ_DEBUG_MODE);
            }
            else 
            {
                /* If Force_Pwr = 0 and we're not attached, we can cut power to the retimer */
                if((gl_host_compliance_mode == 0) && (dpm_get_info(port)->attach == false))
                    retimer_disable(port);
            }
            break;

        case RT_EVT_READ_DEBUG_MODE:
            retimer[port].debug_mode = BB_DEBUG_MODE_DEFAULT;
        
            retimer_read(retimer[port].address[0],
                          BB_DEBUG_MODE_REG, (uint8_t*)&retimer[port].debug_mode, BB_DEBUG_MODE_SIZE);

            if(retimer[port].debug_mode & BB_DEBUG_MODE_RX_LOCKED)
            {
                ridge_slave_set_debug(port, retimer[port].debug_mode);
                retimer[port].debug_poll_count = 0;
            }
            else if(retimer[port].debug_poll_count > 0)
            {
                /* Retry after a nominal 100ms + retimer_wakeup_delay. Wakeup delay is added since the write
                 * to debug_mode may not have gone through and we're retrying it once */
                timer_start(port, BB_DBR_DEBUG_POLL_TIMER, BB_DBR_DEBUG_POLL_DELAY + BB_DBR_WAKEUP_DELAY, retimer_timer_cbk);
                retimer[port].debug_poll_count--;
            }
            break;
#endif /* BB_RETIMER_DEBUG_MODE_SUPP */
        default:
            break;
    }
}
#endif /* BB_RETIMER_ENABLE */

 /* End of File */

