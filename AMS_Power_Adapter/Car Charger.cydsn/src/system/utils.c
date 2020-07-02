/**
 * @file utils.c
 *
 * @brief @{General utility functions for the CCGx firmware stack.@}
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

#include "stdint.h"
#include "stddef.h"
#include "stdbool.h"
#include "utils.h"
#include "config.h"

/* The constant used for CRC-16 calculation. */
#define CRC16_XOR_VALUE       (0xA001u)

uint8_t mem_calculate_byte_checksum(uint8_t *ptr, uint32_t size)
{
    uint8_t checksum = 0;
    uint32_t index;

    if ((ptr == NULL) || (size == 0))
    {
        return checksum;
    }

    /* Calculate the binary sum of all the data. */
    for (index = 0; index < size; index++)
    {
        checksum += ptr[index];
    }

    /* Return the 2's complement of the binary sum. */
    return ((uint8_t)(1u) + (uint8_t)(~checksum));
}

uint16_t mem_calculate_word_checksum(uint16_t *ptr, uint32_t size)
{
    uint16_t checksum = 0;
    uint32_t index;

    if ((ptr == NULL) || (size == 0))
    {
        return checksum;
    }

    /* Calculate the binary sum of all the data. */
    for (index = 0; index < size; index++)
    {
        checksum += ptr[index];
    }

    /* Return the 2's complement of the binary sum. */
    return ((uint16_t)(1u) + (uint16_t)(~checksum));
}

uint32_t mem_calculate_dword_checksum(uint32_t *ptr, uint32_t size)
{
    uint32_t index;
    uint32_t checksum = 0;

    if ((ptr == NULL) || (size == 0))
    {
        return checksum;
    }

    /* Calculate the binary sum of all the data. */
    for (index = 0; index < size; index++)
    {
        checksum += ptr[index];
    }

    /* Return the 2's complement of the binary sum. */
    return ((uint32_t)(1u) + (uint32_t)(~checksum));
}

uint16_t crc16(uint16_t crc, uint8_t data)
{
    int i = 8;

    crc ^= data;
    do
    {
        if (crc & 1) 
        {
            crc = (crc >> 1) ^ CRC16_XOR_VALUE;
        } 
        else 
        {
            crc = (crc >> 1);
        }
    } while(--i);
    
    return crc;
}

void mem_copy_word(uint32_t* dest, const uint32_t* source, uint32_t size)
{
    uint32_t i;
    for(i = 0 ; i < size; i++)
    {
        dest[i] = source[i];
    }
}

void mem_copy (uint8_t* dest, const uint8_t* source, uint32_t size)
{
    uint32_t i;
    for(i = 0 ; i < size; i++)
    {
        dest[i] = source[i];
    }
}

void mem_set (uint8_t* dest, uint8_t c, uint32_t size)
{
    uint32_t i;
    for(i = 0 ; i < size; i++)
    {
        dest[i] = c;
    }
}

uint32_t div_round_up(uint32_t x, uint32_t y)
{
    volatile uint32_t z = 0;
    volatile uint32_t count = 0;

    /*
     * Using repeated subtraction so that the compiler does not insert any C or math
     * library code to do the division.
     */
    if (y != 0)
    {
        z = (x) + (y) - 1;

        while (z >= (y))
        {
            z = z - (y);
            count ++;
        }
    }

    return count;
}

int32_t apply_threshold(int32_t val, int8_t percentage)
{
    return val + ((val * percentage) / 100);
}

#define MAX_EVENT_INDEX                 (31)

void event_group_set_event(uint32_t *event_map, uint8_t event_sel)
{
    uint8_t int_status = CyEnterCriticalSection ();

    if ((event_map != 0) && (event_sel <= MAX_EVENT_INDEX))
    {
        *event_map |= (1 << event_sel);
    }

    CyExitCriticalSection (int_status);
}

void event_group_set_events_by_val(uint32_t *event_map, uint32_t event_sel)
{
    uint8_t int_status = CyEnterCriticalSection ();

    if (event_map != 0)
    {
        *event_map |= event_sel;
    }

    CyExitCriticalSection (int_status);
}

void event_group_clear_event(uint32_t *event_map, uint8_t event_sel)
{
    uint8_t int_status = CyEnterCriticalSection ();

    if ((event_map != 0) && (event_sel <= MAX_EVENT_INDEX))
    {
        *event_map &= ~(1 << event_sel);
    }

    CyExitCriticalSection (int_status);
}

void event_group_clear_events_by_val(uint32_t *event_map, uint32_t event_sel)
{
    uint8_t int_status = CyEnterCriticalSection ();

    if (event_map != 0)
    {
        *event_map &= ~event_sel;
    }

    CyExitCriticalSection (int_status);
}

uint8_t event_group_get_event(volatile uint32_t *event_map, bool clr_stat)
{
    uint32_t evt_stat;
    uint8_t  int_status = CyEnterCriticalSection ();
    uint8_t  ret = MAX_EVENT_INDEX + 1;

    if (event_map != 0)
    {
        if (*event_map != 0)
        {
            evt_stat = *event_map;
            for (ret = 0; ret <= MAX_EVENT_INDEX; ret++)
            {
                if ((evt_stat & (1 << ret)) != 0)
                {
                    if (clr_stat)
                    {
                        *event_map &= ~(1 << ret);
                    }

                    break;
                }
            }
        }
    }

    CyExitCriticalSection (int_status);
    return (ret);
}

/* [] END OF FILE */
