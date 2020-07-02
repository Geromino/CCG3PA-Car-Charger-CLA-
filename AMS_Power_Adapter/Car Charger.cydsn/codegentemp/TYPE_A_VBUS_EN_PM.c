/*******************************************************************************
* File Name: TYPE_A_VBUS_EN.c  
* Version 2.20
*
* Description:
*  This file contains APIs to set up the Pins component for low power modes.
*
* Note:
*
********************************************************************************
* Copyright 2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "TYPE_A_VBUS_EN.h"

static TYPE_A_VBUS_EN_BACKUP_STRUCT  TYPE_A_VBUS_EN_backup = {0u, 0u, 0u};


/*******************************************************************************
* Function Name: TYPE_A_VBUS_EN_Sleep
****************************************************************************//**
*
* \brief Stores the pin configuration and prepares the pin for entering chip 
*  deep-sleep/hibernate modes. This function applies only to SIO and USBIO pins.
*  It should not be called for GPIO or GPIO_OVT pins.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None 
*  
* \sideeffect
*  For SIO pins, this function configures the pin input threshold to CMOS and
*  drive level to Vddio. This is needed for SIO pins when in device 
*  deep-sleep/hibernate modes.
*
* \funcusage
*  \snippet TYPE_A_VBUS_EN_SUT.c usage_TYPE_A_VBUS_EN_Sleep_Wakeup
*******************************************************************************/
void TYPE_A_VBUS_EN_Sleep(void)
{
    #if defined(TYPE_A_VBUS_EN__PC)
        TYPE_A_VBUS_EN_backup.pcState = TYPE_A_VBUS_EN_PC;
    #else
        #if (CY_PSOC4_4200L)
            /* Save the regulator state and put the PHY into suspend mode */
            TYPE_A_VBUS_EN_backup.usbState = TYPE_A_VBUS_EN_CR1_REG;
            TYPE_A_VBUS_EN_USB_POWER_REG |= TYPE_A_VBUS_EN_USBIO_ENTER_SLEEP;
            TYPE_A_VBUS_EN_CR1_REG &= TYPE_A_VBUS_EN_USBIO_CR1_OFF;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(TYPE_A_VBUS_EN__SIO)
        TYPE_A_VBUS_EN_backup.sioState = TYPE_A_VBUS_EN_SIO_REG;
        /* SIO requires unregulated output buffer and single ended input buffer */
        TYPE_A_VBUS_EN_SIO_REG &= (uint32)(~TYPE_A_VBUS_EN_SIO_LPM_MASK);
    #endif  
}


/*******************************************************************************
* Function Name: TYPE_A_VBUS_EN_Wakeup
****************************************************************************//**
*
* \brief Restores the pin configuration that was saved during Pin_Sleep(). This 
* function applies only to SIO and USBIO pins. It should not be called for
* GPIO or GPIO_OVT pins.
*
* For USBIO pins, the wakeup is only triggered for falling edge interrupts.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None
*  
* \funcusage
*  Refer to TYPE_A_VBUS_EN_Sleep() for an example usage.
*******************************************************************************/
void TYPE_A_VBUS_EN_Wakeup(void)
{
    #if defined(TYPE_A_VBUS_EN__PC)
        TYPE_A_VBUS_EN_PC = TYPE_A_VBUS_EN_backup.pcState;
    #else
        #if (CY_PSOC4_4200L)
            /* Restore the regulator state and come out of suspend mode */
            TYPE_A_VBUS_EN_USB_POWER_REG &= TYPE_A_VBUS_EN_USBIO_EXIT_SLEEP_PH1;
            TYPE_A_VBUS_EN_CR1_REG = TYPE_A_VBUS_EN_backup.usbState;
            TYPE_A_VBUS_EN_USB_POWER_REG &= TYPE_A_VBUS_EN_USBIO_EXIT_SLEEP_PH2;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(TYPE_A_VBUS_EN__SIO)
        TYPE_A_VBUS_EN_SIO_REG = TYPE_A_VBUS_EN_backup.sioState;
    #endif
}


/* [] END OF FILE */
