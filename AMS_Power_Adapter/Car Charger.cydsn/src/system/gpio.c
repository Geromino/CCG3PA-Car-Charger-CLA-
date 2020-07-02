/**
 * @file gpio.c
 *
 * @brief @{GPIO and IO mapping control functions.@}
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
#include "stdbool.h"
#include "ccgx_regs.h"
#include "gpio.h"
#include "srom_vars.h"

#if (!(CCG_SROM_CODE_ENABLE))

#ifdef PAG1S
/*
 * PAG1S has only one GPIO port. Since the code base assumes
 * an array, it is better to use it as an array here. Undefine
 * the macro so that the variable can be created locally.
 */
#undef GPIO
#define GPIO0   (PGPIO_REGS_T)GPIO_BASE_ADDR
#undef HSIOM
#define HSIOM0  (PHSIOM_REGS_T)HSIOM_BASE_ADDR
#endif /* PAG1S */
    
/* GPIO block address array */
const PGPIO_REGS_T GPIO[] =
{
    GPIO0,
#if (!defined(PAG1S))
    GPIO1,
    GPIO2,
    GPIO3,
#if (!defined(CCG3PA) && !defined(CCG3PA2))
    GPIO4,
#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    GPIO5
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */
#endif /* (!defined(CCG3PA) && !defined(CCG3PA2)) */
#endif /* (!defined(PAG1S)) */
};

/* HSIOM block address array */
const PHSIOM_REGS_T HSIOM[] =
{
    HSIOM0,
#if (!defined(PAG1S))
    HSIOM1,
    HSIOM2,
    HSIOM3,
#if (!defined(CCG3PA) && !defined(CCG3PA2))
    HSIOM4,
#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    HSIOM5
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */
#endif /* (!defined(CCG3PA) && !defined(CCG3PA2)) */
#endif /* (!defined(PAG1S)) */
};
#endif /* (!(CCG_SROM_CODE_ENABLE)) */

ATTRIBUTES void gpio_set_value(gpio_port_pin_t port_pin, bool value)
{
    uint8_t port = port_pin >> 4;
    uint8_t pin = port_pin & 0x0F;

    if (value)
    {
        GET_IN_VARIABLE(GPIO)[port]->dr |= (1 << pin);
    }
    else
    {
        GET_IN_VARIABLE(GPIO)[port]->dr &= ~(1 << pin);
    }
}

ATTRIBUTES bool gpio_read_value(gpio_port_pin_t port_pin)
{
    uint8_t port = port_pin >> 4;
    uint8_t pin = port_pin & 0x0F;

    return (((GET_IN_VARIABLE(GPIO))[port]->ps & (1 << pin)) ? true : false);
}

ATTRIBUTES void gpio_set_drv_mode(gpio_port_pin_t port_pin, gpio_dm_t drv_mode)
{
    uint8_t pos;
    uint8_t port = port_pin >> 4;
    uint8_t pin = port_pin & 0x0F;
    uint32_t regVal;

    /* Multiply by three as pin uses 3 bits in the register. */
    pos = pin * GPIO_DM_FIELD_SIZE;

    /* Need to mask all other bits. Just update
     * the three bits of the current pin.
     */
    regVal = (GET_IN_VARIABLE(GPIO)[port]->pc & ~(GPIO_DM_FIELD_MASK << pos));
    GET_IN_VARIABLE(GPIO)[port]->pc = (regVal | (drv_mode << pos));
}

ATTRIBUTES void gpio_int_set_config(gpio_port_pin_t port_pin, uint8_t int_mode)
{
    uint8_t pos;
    uint8_t port = port_pin >> 4;
    uint8_t pin = port_pin & 0x0F;
    uint32_t regVal;

    pos = pin << 1;

    /* Make sure that the interrupt is cleared. */
    GET_IN_VARIABLE(GPIO)[port]->intr = (1 << pin);

    /* Set the configuration. */
    regVal = ((GET_IN_VARIABLE(GPIO))[port]->intr_cfg & ~(GPIO_INT_FIELD_MASK << pos));
    GET_IN_VARIABLE(GPIO)[port]->intr_cfg = (regVal | (int_mode << pos));
}

ATTRIBUTES bool gpio_get_intr(gpio_port_pin_t port_pin)
{
    uint8_t port = port_pin >> 4;
    uint8_t pin = port_pin & 0x0F;

    /* Check if intr set */
    if(GET_IN_VARIABLE(GPIO)[port]->intr & (1 << pin))
    {
        return true;
    }

    return false;
}

ATTRIBUTES void gpio_clear_intr(gpio_port_pin_t port_pin)
{
    uint8_t port = port_pin >> 4;
    uint8_t pin = port_pin & 0x0F;

    /* Clear interrupt */
    (GET_IN_VARIABLE(GPIO))[port]->intr = (1 << pin);
}

ATTRIBUTES void hsiom_set_config(gpio_port_pin_t port_pin, hsiom_mode_t hsiom_mode)
{
    uint8_t port = port_pin >> 4;
    uint8_t pin = port_pin & 0x0F;
    uint32_t regVal;

    regVal = GET_IN_VARIABLE(HSIOM)[port]->port_sel;

    regVal &= ~(HSIOM_PORT_SEL_IO0_SEL_MASK << (pin << HSIOM_FIELD_SHIFT));
    regVal |= (hsiom_mode << (pin << HSIOM_FIELD_SHIFT));

    GET_IN_VARIABLE(HSIOM)[port]->port_sel = regVal;
}

ATTRIBUTES void gpio_hsiom_set_config(gpio_port_pin_t port_pin, hsiom_mode_t hsiom_mode,
    gpio_dm_t drv_mode, bool value)
{
    /* Drive GPIO. */
    CALL_IN_FUNCTION(gpio_set_value)(port_pin, value);
    /* Set Drive Mode of GPIO. */
    CALL_IN_FUNCTION(gpio_set_drv_mode)(port_pin, drv_mode);
    /* Set HSIOM Configuration. */
    CALL_IN_FUNCTION(hsiom_set_config)(port_pin, hsiom_mode);
}

ATTRIBUTES void gpio_set_lvttl_mode(uint8_t port)
{
    GET_IN_VARIABLE(GPIO)[port]->pc |= GPIO_PRT_PC_PORT_VTRIP_SEL;
}

/* [] END OF FILE */
