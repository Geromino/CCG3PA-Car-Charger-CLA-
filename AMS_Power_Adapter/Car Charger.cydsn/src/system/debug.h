/**
 * @file debug.h
 *
 * @brief @{UART debug logging header file.
 *
 * These functions are available only if a SCB UART or SW UART Creator
 * component with name UART is part of the project. @}
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

#ifndef _DEBUG_H_
#define _DEBUG_H_

#include "stdint.h"

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

#if (defined(CY_SCB_UART_H) || defined(CY_SW_TX_UART_UART_H))

/**
 *  @brief Initialize the debug interface.
 *  @return None
 */
void debug_init(void);

/**
 * @brief Print out the NULL terminated string provided as input.
 * @param str Null terminated string to be printed.
 * @return None
 */
void debug_print(const char *str);

/**
 * @brief Format the received byte as HEX into the provided byte array. The string is not
 * NULL terminated by the function.
 * @param ptr Pointer to buffer to be filled by the HEX string.
 * @param byte Byte value to be converted for printing.
 * @return None
 */
void debug_format8(char *ptr, uint8_t byte);

/**
 * @brief Format the received word (16-bit) as HEX into the provided byte array. The string is not
 * NULL terminated by the function. The string format is MSB first.
 * @param ptr Pointer to buffer to be filled by the HEX string.
 * @param word Word value to be converted for printing.
 * @return None
 */
void debug_format16(char *ptr, uint16_t word);

/**
 * @brief Format the received DWORD (32-bit) as HEX into the provided byte array. The string is not
 * NULL terminated by the function. The string format is MSB first.
 * @param ptr Pointer to buffer to be filled by the HEX string.
 * @param dword DWORD value to be converted for printing.
 * @return None
 */
void debug_format32(char *ptr, uint32_t dword);

/**
 * @brief Format and print the received BYTE in hexadecimal format.
 * @param num Byte value to be printed.
 * @return None
 */
void debug_print_byte(uint8_t num);

/**
 * @brief Format and print the received WORD in hexadecimal format with MSB first.
 * @param num Word value to be printed.
 * @return None
 */
void debug_print_word(uint16_t num);

/**
 * @brief Format and print the received DWORD in hexadecimal format with MSB first.
 * @param num DWORD value to be printed.
 * @return None
 */
void debug_print_dword(uint32_t num);

/**
 * @brief Format and print the received unsigned integer in decimal format.
 * @param num Unsigned integer to be printed.
 * @return None
 */
void debug_print_num(uint32_t num);

/**
 * @brief Print out end of line characters (carriage return and line feed).
 * @return None
 */
void debug_print_eol(void);

#else /* !(defined(CY_SCB_UART_H) || defined(CY_SW_TX_UART_UART_H)) */

/** @cond DOXYGEN_HIDE */

/* 
 * The below definitions are available to allow compilation to go through
 * in case the UART component is not added.
 */

#define debug_init()
#define debug_print(str)
#define debug_format8(ptr, byte)
#define debug_format16(ptr, word)
#define debug_format32(ptr, dword)
#define debug_print_byte(num)
#define debug_print_word(num)
#define debug_print_dword(num)
#define debug_print_num(num)
#define debug_print_eol()

/** @endcond */

#endif /* (defined(CY_SCB_UART_H) || defined(CY_SW_TX_UART_UART_H)) */

#endif /* _DEBUG_H_ */

/* End of File */
