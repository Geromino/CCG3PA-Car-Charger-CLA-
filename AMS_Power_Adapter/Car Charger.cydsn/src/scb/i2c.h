/**
 * @file i2c.h
 *
 * @brief @{I2C slave driver header file.@}
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

#ifndef _I2C_H_
#define _I2C_H_

#include <stdint.h>
#include <stdbool.h>
#include <config.h>
#include "timer.h"
#include "timer_id.h"

/**
   @brief Number of I2C blocks supported by the device.

   The CCG3 and CCG4 device families support 4 SCB blocks which can be used for I2C
   functionality. CCG3PA, CCG3PA2 supports two SCB blocks. If the target device using this stack
   supports a different number of SCBs, this macro value will need to be updated.
 */
#if (defined(CCG3PA) || defined(CCG3PA2))
#define I2C_BLOCK_COUNT                         (2)
#else /* CCG3, CCG4, CCG5, CCG6 */
#define I2C_BLOCK_COUNT                         (4)
#endif /* CCGx */

#define I2C_CLEAR_INTR_MASK                     (0x00000000)    /**< Value to clear Interrupt mask bits. */
#define I2C_CLEAR_INTR_REQUEST_REG              (0xFFFFFFFFu)   /**< Value to clear Interrupt request bits. */

#ifdef CCG3
#define I2C_SCB_FIFO_SIZE                       (0x40)  /**< Size of FIFO in the SCB block for I2C transfers. */
#else /* CCG4, CCG3PA, CCG3PA2, CCG5 */
#define I2C_SCB_FIFO_SIZE                       (8u)    /**< Size of FIFO in the SCB block for I2C transfers. */
#endif /* CCGx */

#define I2C_SCB_RX_FIFO_SIZE                    (I2C_SCB_FIFO_SIZE)
/**< Size of FIFO provided by the SCB block for I2C read (incoming data) transfers. */

#define I2C_SCB_TX_FIFO_SIZE                    (I2C_SCB_FIFO_SIZE)
/**< Size of FIFO provided by the SCB block for I2C write (outgoing data) transfers. */

/** @cond DOXYGEN_HIDE */

/* Interrupt vector numbers for various SCB blocks. For internal use only. */
#ifdef CCG4
#define SCB0_INT_VECTOR_LOCATION                (9)
#define SCB1_INT_VECTOR_LOCATION                (10)
#define SCB2_INT_VECTOR_LOCATION                (11)
#define SCB3_INT_VECTOR_LOCATION                (12)
#elif defined (CCG3) || defined (DMC)
#define SCB0_INT_VECTOR_LOCATION                (8)
#define SCB1_INT_VECTOR_LOCATION                (9)
#define SCB2_INT_VECTOR_LOCATION                (10)
#define SCB3_INT_VECTOR_LOCATION                (11)
#elif (defined(CCG3PA) || defined(CCG3PA2))
#define SCB0_INT_VECTOR_LOCATION                (7)
#define SCB1_INT_VECTOR_LOCATION                (8)
#define SCB2_INT_VECTOR_LOCATION                (255)
#define SCB3_INT_VECTOR_LOCATION                (255)    
#elif (defined(CCG5C) || defined (CCG6))
#define SCB0_INT_VECTOR_LOCATION                (9)
#define SCB1_INT_VECTOR_LOCATION                (10)
#define SCB2_INT_VECTOR_LOCATION                (11)
#define SCB3_INT_VECTOR_LOCATION                (12)    
#elif defined (CCG5)
#define SCB0_INT_VECTOR_LOCATION                (10)
#define SCB1_INT_VECTOR_LOCATION                (11)
#define SCB2_INT_VECTOR_LOCATION                (12)
#define SCB3_INT_VECTOR_LOCATION                (13)    
#else
#error "Unsupported device selection."
#endif /* CCGx */

/** @endcond */

#define I2C_SLAVE_ADDR_MASK_DEFAULT             (0xFE)
/**< I2C Slave address mask to be applied on received preamble. */

#define I2C_SLAVE_TIMER_BASE                    (I2C_SLAVE_SCB0_TIMER)
/**< Base ID of timers reserved for I2C transfer timeout implementation. */

#define I2C_SLAVE_TIMER_PERIOD                  (500)
/**< Timeout period for I2C transfers in milliseconds. The I2C block will be reset
     if any transaction does not complete within this time period. */

/**
 * @typedef i2c_scb_state_t
 * @brief List of possible I2C block states.
 *
 * The I2C driver implements a state machine which ensures that read/write transfers
 * are responded to correctly without any corruption. This data type lists the possible
 * states in this slave mode state machine.
 */
typedef enum
{
    I2C_SCB_STATE_DISABLED = 0,                 /**< I2C interface is disabled. */
    I2C_SCB_STATE_INIT,                         /**< Interface initialized and waiting to be enabled. */
    I2C_SCB_STATE_IDLE,                         /**< Interface ready and waiting for preamble from the master. */
    I2C_SCB_STATE_PREAMBLE,                     /**< Preamble phase is in progress. */
    I2C_SCB_STATE_READ,                         /**< I2C read operation is in progress. */
    I2C_SCB_STATE_WRITE,                        /**< I2C write operation is in progress. */
    I2C_SCB_STATE_CLK_STRETCH,                  /**< Drive is stretching I2C clock to delay operation. */
    I2C_SCB_STATE_ERROR,                        /**< Error state: transaction error detected. */
    I2C_SCB_NUM_STATES                          /**< Last state ID: not used. */
} i2c_scb_state_t;

/**
 * @typedef i2c_scb_mode_t
 * @brief List of possible I2C block operating modes.
 *
 * This type lists the possible I2C block operating modes. Multiple slave modes are defined
 * in case there is any mode specific handling required for any of them.
 *
 * Note: Master mode is currently not supported by the driver.
 * Note: The HPI and ALP_RIDGE mode handling is equivalent.
 */
typedef enum
{
    I2C_SCB_MODE_MASTER = 0,            /**< Master mode: Can be used for mux control. Not supported as of now. */
    I2C_SCB_MODE_HPI,                   /**< Slave mode: Used for HPI interface. */
    I2C_SCB_MODE_ALP_RIDGE              /**< Slave mode: Used for Alpine Ridge interface. */
} i2c_scb_mode_t;

/**
 * @typedef i2c_scb_clock_freq_t
 * @brief List of possible I2C bus bit rates.
 */
typedef enum
{
    I2C_SCB_CLOCK_FREQ_100_KHZ = 0,     /**< 100 KHz operation. */
    I2C_SCB_CLOCK_FREQ_400_KHZ,         /**< 400 KHz operation. */
    I2C_SCB_CLOCK_FREQ_1_MHZ            /**< 1 MHz operation. */
} i2c_scb_clock_freq_t;

/**
 * @typedef i2c_cb_cmd_t
 * @brief Type of I2C operation being notified through a callback function.
 */
typedef enum
{
    I2C_CB_CMD_READ = 0,                /**< Read command from master. */
    I2C_CB_CMD_WRITE,                   /**< Write command from master. */
    I2C_CB_CMD_XFER_END,                /**< End of read transfer: STOP condition signalled by master. */
    I2C_CB_CMD_TIMEOUT,                 /**< Timeout on I2C operation. */
    I2C_CB_SLAVE_ADDR_MATCH             /**< I2C slave address match detected. */
} i2c_cb_cmd_t;

/**
 * @typedef i2c_cb_fun_t
 * @brief I2C callback function for interrupt notifications.
 */
typedef bool (*i2c_cb_fun_t)(
    i2c_cb_cmd_t cmd,                   /**< Type of I2C event being notified. */
    i2c_scb_state_t i2c_state,          /**< Current I2C block state. */
    uint16_t count                      /**< Transaction size. */
    );

/**
   @brief This structure holds all configuration associated with each I2C block.

   This structure is used internally by the driver, and is not expected to be manipulated directly
   by the caller.
 */
typedef struct i2c_scb_config
{
    i2c_scb_mode_t              mode;                   /**< I2C operating mode. */
    uint8_t                     slave_address;          /**< Slave address. Only valid in slave modes. */
    uint8_t                     slave_mask;             /**< Slave address mask. Only valid in slave modes. */
    i2c_scb_clock_freq_t        clock_freq;             /**< Clock frequency. Only valid in master mode. */
    i2c_scb_state_t             i2c_state;              /**< Current state of the I2C block. */
    uint8_t                    *buffer;                 /**< Buffer to be used to receive data. */
    uint16_t                    buf_size;               /**< Size of receive buffer. */
    i2c_cb_fun_t                cb_fun_ptr;             /**< Callback function pointer. */
    volatile uint16_t           i2c_write_count;        /**< Current index into receive buffer. */
} i2c_scb_config_t;

/**
 * @brief Configure one of the I2C blocks as required.
 *
 * This API is used to enable and configure one of the I2C blocks for driver operation.
 * Only I2C slave operation is currently supported by the driver as of now.
 
 * The I2C driver is agnostic of the actual data transfer protocol. It reads all data
 * written by the master into a receive buffer provided by the protocol layer. A callback
 * function is used to notify the protocol layer when the write is complete. The receive
 * buffer provided should be big enough to hold the maximum amount of data that the master
 * may provide in a write operation. If the write contains more data than the buffer can
 * hold, the I2C driver will NAK the transaction.
 *
 * Read requests from the I2C master are automatically delayed by clock stretching. A
 * callback function is used to notify the protocol layer that the master is waiting
 * for data. The i2c_scb_write function can be used by the protocol layer to write data
 * into the transmit FIFO in response to the read request.
 *
 * All I2C driver events are generated from interrupt context, and are expected to be
 * handled with care. The protocol layer should defer any long operations to a non-interrupt
 * context.
 *
 * @param scb_index SCB index being configured for I2C operation.
 * @param mode Desired mode of operation.
 * @param clock_freq Desired I2C clock frequency.
 * @param slave_addr Device address to be used in case of slave operation.
 * @param slave_mask Mask to be applied on for slave address matching.
 * @param cb_fun_ptr Callback function to be called for event notification.
 * @param scratch_buffer Receive buffer used to hold written by master.
 * @param scratch_buffer_size Size of the receive buffer in bytes.
 *
 * @return None
 */
void i2c_scb_init(uint8_t scb_index, i2c_scb_mode_t mode,
        i2c_scb_clock_freq_t clock_freq, uint8_t slave_addr,
        uint8_t slave_mask, i2c_cb_fun_t cb_fun_ptr,
        uint8_t *scratch_buffer, uint16_t scratch_buffer_size);

/**
 * @brief De-initialize a previously initialized SCB block.
 *
 * @param scb_index SCB index to be disabled.
 *
 * @return None
 */
void i2c_scb_deinit(uint8_t scb_index);

/**
 * @brief Write data into the transmit FIFO associated with the I2C block.
 *
 * This function is used by the protocol layer to write data into the I2C transmit FIFO
 * in response to a master read operation.
 *
 * @param scb_index SCB ID to do the I2C transfer through.
 * @param source_ptr Pointer to buffer containing the data to be written.
 * @param size Size of the data buffer. Maximum amount of data that may be written.
 * @param count Return parameter through which the actual write size is returned.
 *
 * @return None
 */
void i2c_scb_write(uint8_t scb_index, uint8_t *source_ptr, uint8_t  size,
        uint8_t *count);

/**
 * @brief Reset the I2C block specified.
 *
 * This function resets the I2C block in response to an error or explicit request from protocol layer.
 *
 * @param scb_index SCB ID to be reset.
 *
 * @return None
 */
void i2c_reset(uint8_t scb_index);

/**
 * @brief Enable/Disable the I2C slave acknowledgement.
 *
 * This function enables/disables the slave address ACK from the I2C block. The protocol
 * layer can disable the address ACK to hold off data transfers when it is not ready to
 * respond to the master.
 *
 * @param scb_index SCB ID to configure.
 * @param enable Whether to enable or disable the auto slave address acknowledgement.
 *
 * @return None
 */
void i2c_slave_ack_ctrl(uint8_t scb_index, bool enable);

/**
 * @brief Check whether the I2C block is idle.
 *
 * This function checks whether the specified I2C block is idle. This check
 * should be performed before the device enters deep sleep. Deep sleep entry
 * should be avoided if this function returns false.
 *
 * @param scb_index SCB ID to be checked.
 *
 * @return true if the I2C block is idle, false otherwise.
 */
bool i2c_scb_is_idle(uint8_t scb_index);

/**
 * @brief Enable deep-sleep wakeup due to address match on the specified SCB block.
 *
 * @param scb_index SCB ID to be configured as deep-sleep wakeup trigger.
 *
 * @return None
 */
void i2c_scb_enable_wakeup(uint8_t scb_index);

#if (CCG_BOOT == 0)

/**
 * @brief Timer callback that indicates I2C transaction that has timed out.
 * @param instance The Timer instance associated with this timer. Will be 0.
 * @param id The timer id used to time I2C transactions.
 * @return None
 */
extern void i2c_timer_cb(uint8_t instance, timer_id_t id);

#endif /* (CCG_BOOT == 0) */

/** @cond DOXYGEN_HIDE */

extern void i2c_scb_0_intr_handler(void);
typedef    void(*i2c_scb_0_intr_handler_fptr)(void);

#if (I2C_BLOCK_COUNT > 1)
void i2c_scb_1_intr_handler(void);
typedef    void(*i2c_scb_1_intr_handler_fptr)(void);
#endif /* (I2C_BLOCK_COUNT > 1) */

#if (I2C_BLOCK_COUNT > 2)
void i2c_scb_2_intr_handler(void);
typedef    void(*i2c_scb_2_intr_handler_fptr)(void);
#endif /* (I2C_BLOCK_COUNT > 2) */

#if (I2C_BLOCK_COUNT > 3)
void i2c_scb_3_intr_handler(void);
typedef    void(*i2c_scb_3_intr_handler_fptr)(void);
#endif /* (I2C_BLOCK_COUNT > 3) */

/* Function pointer types used to access ROM-ed versions of SCB/I2C module APIs.
 * These correspond to the various functions defined above.
 */
typedef void(*i2c_scb_init_fptr)(uint8_t scb_index,
    i2c_scb_mode_t mode, i2c_scb_clock_freq_t clock_freq,
    uint8_t slave_addr, uint8_t slave_mask, i2c_cb_fun_t cb_fun_ptr,
    uint8_t *scratch_buffer, uint16_t scratch_buffer_size);

typedef void(*i2c_scb_deinit_fptr)(uint8_t scb_index);

typedef void(*i2c_scb_write_fptr)(uint8_t scb_index,
    uint8_t *source_ptr, uint8_t size, uint8_t *count);

typedef void(*i2c_reset_fptr)(uint8_t scb_index);

typedef void(*i2c_slave_ack_ctrl_fptr)(uint8_t scb_index, bool enable);

typedef bool(*i2c_scb_is_idle_fptr)(uint8_t scb_index);

typedef void(*i2c_scb_enable_wakeup_fptr)(uint8_t scb_index);

typedef void(*i2c_timer_cb_fptr)(uint8_t instance, timer_id_t id);

/** @endcond */

#endif /* _I2C_H_ */

/* [] END OF FILE */

