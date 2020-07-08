/*******************************************************************************
* File Name: SW_Tx_UART.h
* Version 1.50
*
* Description:
*  This file provides constants and parameter values for the Software Transmit
*  UART Component.
*
********************************************************************************
* Copyright 2013-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef CY_SW_TX_UART_SW_Tx_UART_H
#define CY_SW_TX_UART_SW_Tx_UART_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"

#define SW_Tx_UART_BAUD_RATE                      (115200u)
#define SW_Tx_UART_PIN_STATIC_MODE                (1u)


/***************************************
*        Function Prototypes
***************************************/
#if(SW_Tx_UART_PIN_STATIC_MODE == 1u)
    void SW_Tx_UART_Start(void) ;
#else
    void SW_Tx_UART_StartEx(uint8 port, uint8 pin) ;
#endif /* (SW_Tx_UART_PIN_STATIC_MODE == 1u) */

void SW_Tx_UART_Stop(void) ;
void SW_Tx_UART_PutChar(uint8 txDataByte) ;
void SW_Tx_UART_PutString(const char8 string[]) ;
void SW_Tx_UART_PutArray(const uint8 array[], uint32 byteCount) ;
void SW_Tx_UART_PutHexByte(uint8 txHexByte) ;
void SW_Tx_UART_PutHexInt(uint16 txHexInt) ;
void SW_Tx_UART_PutCRLF(void) ;

#endif /* CY_SW_TX_UART_SW_Tx_UART_H */


/* [] END OF FILE */
