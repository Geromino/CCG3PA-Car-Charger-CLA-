/**
 * @file debug.c
 *
 * @brief @{UART debug source file.@}
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
#include "stdint.h"
#include "utils.h"

#if (defined(CY_SCB_UART_H) || defined(CY_SW_TX_UART_UART_H))

void debug_init(void)
{
    UART_Start();
}

void debug_print(const char *str)
{
#ifdef CY_SCB_UART_H
    UART_UartPutString(str);
#elif (defined(CY_SW_TX_UART_UART_H))
        UART_PutString(str);
#endif /* CY_SCB_UART_H */
}

static uint8_t debug_nibble_to_char(uint8_t nibble)
{
    if (nibble < 10)
    {
        return (nibble + 0x30);
    }

    return (nibble + 0x37);
}

void debug_format8(char *ptr, uint8_t byte)
{
    ptr[0] = debug_nibble_to_char((byte >> 4) & 0x0F);
    ptr[1] = debug_nibble_to_char((byte >> 0) & 0x0F);
}

void debug_format16(char *ptr, uint16_t word)
{
    debug_format8(&ptr[0], WORD_GET_MSB(word));
    debug_format8(&ptr[2], WORD_GET_LSB(word));
}

void debug_format32(char *ptr, uint32_t dword)
{
    debug_format8(&ptr[0], DWORD_GET_BYTE3(dword));
    debug_format8(&ptr[2], DWORD_GET_BYTE2(dword));
    debug_format8(&ptr[4], DWORD_GET_BYTE1(dword));
    debug_format8(&ptr[6], DWORD_GET_BYTE0(dword));
}

void debug_print_byte(uint8_t num)
{
    char str[3];
    debug_format8(str, num);
    str[2] = 0;
    debug_print(str);
}

void debug_print_word(uint16_t num)
{
    char str[5];
    debug_format16(str, num);
    str[4] = 0;
    debug_print(str);
}

void debug_print_dword(uint32_t num)
{
    char str[9];
    debug_format32(str, num);
    str[8] = 0;
    debug_print(str);
}

void debug_print_num(uint32_t num)
{
    uint32_t val;
    char str[32];
    uint8_t index;

    val = num;
    str[sizeof(str) - 1] = 0;
    index = sizeof(str) - 2;
    do
    {
        str[index] = (val % 10) + 0x30;
        val /= 10;
        index--;

    } while (val != 0);

    debug_print(&str[index + 1]);
}

void debug_print_eol(void)
{
    debug_print("\r\n");
}

#endif/* (defined(CY_SCB_UART_H) || defined(CY_SW_TX_UART_UART_H)) */

/* End of File */
