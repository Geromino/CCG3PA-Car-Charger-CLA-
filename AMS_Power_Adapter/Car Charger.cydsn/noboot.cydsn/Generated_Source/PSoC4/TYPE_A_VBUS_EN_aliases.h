/*******************************************************************************
* File Name: TYPE_A_VBUS_EN.h  
* Version 2.20
*
* Description:
*  This file contains the Alias definitions for Per-Pin APIs in cypins.h. 
*  Information on using these APIs can be found in the System Reference Guide.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_TYPE_A_VBUS_EN_ALIASES_H) /* Pins TYPE_A_VBUS_EN_ALIASES_H */
#define CY_PINS_TYPE_A_VBUS_EN_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define TYPE_A_VBUS_EN_0			(TYPE_A_VBUS_EN__0__PC)
#define TYPE_A_VBUS_EN_0_PS		(TYPE_A_VBUS_EN__0__PS)
#define TYPE_A_VBUS_EN_0_PC		(TYPE_A_VBUS_EN__0__PC)
#define TYPE_A_VBUS_EN_0_DR		(TYPE_A_VBUS_EN__0__DR)
#define TYPE_A_VBUS_EN_0_SHIFT	(TYPE_A_VBUS_EN__0__SHIFT)
#define TYPE_A_VBUS_EN_0_INTR	((uint16)((uint16)0x0003u << (TYPE_A_VBUS_EN__0__SHIFT*2u)))

#define TYPE_A_VBUS_EN_INTR_ALL	 ((uint16)(TYPE_A_VBUS_EN_0_INTR))


#endif /* End Pins TYPE_A_VBUS_EN_ALIASES_H */


/* [] END OF FILE */
