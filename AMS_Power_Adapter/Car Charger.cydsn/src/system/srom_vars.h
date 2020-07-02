/**
 * @file srom_vars.h
 *
 * @brief @{SROM Code header file.@}
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

#ifndef _SROM_VARS_H_
#define _SROM_VARS_H_

#include <config.h>
#include <stdio.h>
#include <string.h>

#if (CCG_SROM_CODE_ENABLE)

#include "i2c.h"
#include "gpio.h"
#include "hpi.h"
#include "utils.h"
#include "boot.h"
#include "ccgx_version.h"
#include "system.h"
#include "timer.h"
#include "ccgx_regs.h"
#include "flash.h"
#include "pd_protocol.h"

#define ATTRIBUTES __attribute__ ((section (".sromCode"))) __attribute__((used))
#define CALL_OUT_FUNCTION(func_name) (glp_srom_out_cbk->func_name)
#define CALL_IN_FUNCTION(func_name) (glp_srom_in_cbk->func_name)
#define GET_IN_VARIABLE(var_name) (glp_srom_vars->var_name)

#if ((CCG_DUALAPP_DISABLE != 0) || (CCG_BOOT != 0) || (I2C_BLOCK_COUNT != 4))
#error "Incorrect configuration for SROM code usage."
#endif /* ((CCG_DUALAPP_DISABLE != 0) || (CCG_BOOT != 0) || (I2C_BLOCK_COUNT != 4)) */

typedef struct
{
    /* SCB */
    PSCB_PRT_REGS_T *SCB_PRT;
    i2c_scb_config_t gl_i2c_scb_config[I2C_BLOCK_COUNT];
    volatile bool i2c_ack_disable[I2C_BLOCK_COUNT];
    volatile bool i2c_ack_pending[I2C_BLOCK_COUNT];

    /* GPIO */
    PGPIO_REGS_T *GPIO;
    PHSIOM_REGS_T *HSIOM;

    /* System */
    /* Invalid FW Version. */
    uint8_t gl_invalid_version[8];

    /* Variable representing the current firmware mode. */
    sys_fw_mode_t gl_active_fw;

    /* Pointer to Image-1 FW metadata table. */
    sys_fw_metadata_t *gl_img1_fw_metadata;
#if (!CCG_DUALAPP_DISABLE)
    /* Pointer to Image-2 FW metadata table. */
    sys_fw_metadata_t *gl_img2_fw_metadata;
#endif /* (!CCG_DUALAPP_DISABLE) */

    /* Boot variables. */


    /* PDO Variables. */
    uint32_t gl_max_min_cur_pwr[NO_OF_TYPEC_PORTS];
    uint32_t gl_contract_power[NO_OF_TYPEC_PORTS];
    uint32_t gl_contract_voltage[NO_OF_TYPEC_PORTS];
    uint32_t gl_op_cur_power[NO_OF_TYPEC_PORTS];

    /*PD status*/
    pd_status_t *gl_pd_status[NO_OF_TYPEC_PORTS];

} srom_vars_t;

typedef struct
{
    /* System Functions. */
    void(*CyDelayUs)(uint16_t microseconds);
    cyisraddress(*CyIntSetVector)(uint8_t number, cyisraddress address);
    void(*CyIntEnable)(uint8_t number);
    void(*CyIntDisable)(uint8_t number);
    uint8_t (*CyEnterCriticalSection)(void);
    void  (*CyExitCriticalSection)(uint8_t savedIntrStatus);

    /* Timer Functions */
    bool(*timer_start)(uint8_t instance, timer_id_t id, uint16_t period, timer_cb_t cb);
    void(*timer_stop) (uint8_t instance, timer_id_t id);
    void (*timer_stop_range)(uint8_t instance, timer_id_t start, timer_id_t stop);
    bool (*timer_is_running) (uint8_t instance, timer_id_t id);

    /* HPI Function */
    bool (*hpi_i2c_cmd_callback)(i2c_cb_cmd_t cmd, i2c_scb_state_t i2c_state, uint16_t count);

    /* Utils functions. */
    uint8_t (*mem_calculate_byte_checksum)(uint8_t *ptr, uint32_t  size);
    void (*mem_copy_word)(uint32_t* dest, const uint32_t* source, uint32_t size);
    void (*mem_set)(uint8_t* dest, uint8_t c, uint32_t size);
    void (*mem_copy)(uint8_t* dest, const uint8_t* source, uint32_t size);
    uint32_t (*div_round_up)(uint32_t x, uint32_t y);

    /* DPM Functions. */
    const dpm_status_t* (*dpm_get_info)(uint8_t port);
    ccg_status_t (*dpm_is_rdo_valid)(uint8_t port, pd_do_t rdo);
    dpm_status_t* (*dpm_get_status)(uint8_t port);

    /* App Functions. */
    app_resp_t* (*app_get_resp_buf)(uint8_t port);

    /* PD PHY Functions. */
    bool (*pd_frs_rx_enable)(uint8_t port);
    bool (*pd_frs_rx_disable)(uint8_t port);
    bool (*pd_frs_tx_enable)(uint8_t port);
    bool (*pd_frs_tx_disable)(uint8_t port);

    ccg_status_t (*pd_phy_send_bist_cm2)(uint8_t port);
    ccg_status_t (*pd_phy_abort_bist_cm2)(uint8_t port);
    ccg_status_t (*pd_phy_abort_tx_msg)(uint8_t port);
    ccg_status_t (*pd_phy_send_reset)(uint8_t port, sop_t sop);

    bool (*pd_phy_is_busy)(uint8_t port);
    ccg_status_t (*pd_phy_init)(uint8_t port, pd_phy_cbk_t cbk);
    void (*pd_phy_refresh_roles)(uint8_t port);
    void (*pd_phy_dis_unchunked_tx)(uint8_t port);
    bool (*pd_phy_send_msg)(uint8_t port);
    pd_packet_extd_t* (*pd_phy_get_rx_packet)(uint8_t port);

    bool (*pd_phy_load_msg)(uint8_t port, sop_t sop, uint8_t retries,
            uint8_t dobj_count, uint32_t header, bool unchunked, uint32_t* buf);

    bool (*pd_is_msg)(const pd_packet_t* pkt, sop_t sop_type, 
            pd_msg_class_t msg_class, uint32_t msg_mask, uint16_t length);


    /* Boot functions. */
    fw_img_status_t (*get_boot_mode_reason)(void);
} srom_out_cbk_t;

typedef struct
{
    void(*i2c_slave_ack_ctrl)(uint8_t scb_index, bool enable);
    void(*i2c_reset)(uint8_t scb_index);
    bool(*i2c_scb_is_idle)(uint8_t scb_index);
    void(*i2c_scb_enable_wakeup)(uint8_t scb_index);

#if (CCG_BOOT == 0)
    void(*i2c_timer_cb)(uint8_t instance, timer_id_t id);
#endif /* (CCG_BOOT == 0) */
    void(*i2c_scb_0_intr_handler)(void);
#if (I2C_BLOCK_COUNT > 1)
    void(*i2c_scb_1_intr_handler)(void);
#endif /* (I2C_BLOCK_COUNT > 1) */
#if (I2C_BLOCK_COUNT > 2)
    void(*i2c_scb_2_intr_handler)(void);
#endif /* (I2C_BLOCK_COUNT > 2) */
#if (I2C_BLOCK_COUNT > 3)
    void(*i2c_scb_3_intr_handler)(void);
#endif /* (I2C_BLOCK_COUNT > 3) */
    void(*i2c_scb_init)(uint8_t scb_index, i2c_scb_mode_t mode,
            i2c_scb_clock_freq_t clock_freq, uint8_t slave_addr,
            uint8_t slave_mask, i2c_cb_fun_t cb_fun_ptr,
            uint8_t *scratch_buffer, uint16_t scratch_buffer_size
            );
    void(*i2c_scb_deinit)(uint8_t scb_index);
    void(*i2c_scb_write)(uint8_t scb_index,
            uint8_t *source_ptr, uint8_t size,
            uint8_t *count );

    /* GPIO Functions. */
    void (*gpio_set_value)(gpio_port_pin_t port_pin, bool value);
    bool (*gpio_read_value)(gpio_port_pin_t port_pin);
    void (*gpio_set_drv_mode)(gpio_port_pin_t port_pin, gpio_dm_t drv_mode);
    void (*gpio_int_set_config)(gpio_port_pin_t port_pin, uint8_t int_mode);
    bool (*gpio_get_intr)(gpio_port_pin_t port_pin);
    void (*gpio_clear_intr)(gpio_port_pin_t port_pin);
    void (*hsiom_set_config)(gpio_port_pin_t port_pin, hsiom_mode_t hsiom_mode);
    void (*gpio_hsiom_set_config)(gpio_port_pin_t port_pin, hsiom_mode_t hsiom_mode,
            gpio_dm_t drv_mode, bool value);
    void (*gpio_set_lvttl_mode)(uint8_t port);

    /* System functions. */
    void (*sys_set_device_mode)(sys_fw_mode_t fw_mode);
    sys_fw_mode_t (*sys_get_device_mode)(void);
    uint8_t* (*sys_get_boot_version)(void);
    uint8_t* (*sys_get_img1_fw_version)(void);
    uint8_t* (*sys_get_img2_fw_version)(void);
    uint32_t (*sys_get_fw_img1_start_addr)(void);
    uint32_t (*sys_get_fw_img2_start_addr)(void);
    uint8_t (*sys_get_recent_fw_image)(void);
    void (*sys_get_silicon_id)(uint32_t *silicon_id);
    uint8_t (*get_silicon_revision)(void);
    uint32_t (*sys_get_custom_info_addr)(void);
    uint16_t (*sys_get_bcdDevice_version)(uint32_t ver_addr);

    /* PDO Functions. */
    void (*eval_src_cap)(uint8_t port, const pd_packet_t* src_cap, app_resp_cbk_t app_resp_handler) ;
    void (*eval_rdo)(uint8_t port, pd_do_t rdo, app_resp_cbk_t app_resp_handler) ;

    /* PD Protocol Functions. */
    void (*phy_cbk)(uint8_t port, pd_phy_evt_t event);
    void (*prl_tmr_cbk)(uint8_t port, timer_id_t id);

    ccg_status_t (*pd_prot_init)(uint8_t port, pd_cbk_t cbk);
    ccg_status_t (*pd_prot_start)(uint8_t port);
    ccg_status_t (*pd_prot_refresh_roles)(uint8_t port);

    ccg_status_t (*pd_prot_reset_all)(uint8_t port);
    ccg_status_t (*pd_prot_reset)(uint8_t port, sop_t sop);
    ccg_status_t (*pd_prot_reset_rx)(uint8_t port, sop_t sop);

    bool (*pd_prot_is_busy)(uint8_t port);
    ccg_status_t (*pd_prot_send_ctrl_msg)(
            uint8_t port,
            sop_t sop,
            ctrl_msg_t msg_type);
    ccg_status_t (*pd_prot_send_data_msg)(
            uint8_t port,
            sop_t sop,
            data_msg_t msg_type,
            uint8_t count,
            pd_do_t *dobj);

    ccg_status_t (*pd_prot_send_extd__msg)(uint8_t port,
            sop_t sop,
            extd_msg_t msg_type,
            pd_extd_hdr_t ext_hdr,
            uint8_t* dobj);

    ccg_status_t (*pd_prot_send_hard_reset)(uint8_t port);
    ccg_status_t (*pd_prot_send_cable_reset)(uint8_t port);
    ccg_status_t (*pd_prot_en_bist_cm2)(uint8_t port);
    ccg_status_t (*pd_prot_dis_bist_cm2)(uint8_t port);
    ccg_status_t (*pd_prot_en_bist_test_data)(uint8_t port);
    ccg_status_t (*pd_prot_dis_bist_test_data)(uint8_t port);
    ccg_status_t (*pd_prot_set_avoid_retry)(uint8_t port);
    pd_packet_extd_t* (*pd_prot_get_rx_packet)(uint8_t port);

    ccg_status_t (*pd_prot_frs_rx_enable)(uint8_t port);
    ccg_status_t (*pd_prot_frs_rx_disable)(uint8_t port);

    ccg_status_t (*pd_prot_frs_tx_enable)(uint8_t port);
    ccg_status_t (*pd_prot_frs_tx_disable)(uint8_t port);

} srom_in_cbk_t;

extern srom_out_cbk_t *glp_srom_out_cbk;
extern srom_in_cbk_t *glp_srom_in_cbk;
extern srom_vars_t *glp_srom_vars;

void srom_vars_init (uint32_t img1_fw_md_addr, uint32_t img2_fw_md_addr);

#else

#define ATTRIBUTES
#define CALL_OUT_FUNCTION(func_name) func_name
#define CALL_IN_FUNCTION(func_name) func_name
#define GET_IN_VARIABLE(var_name) var_name

#endif /* (CCG_SROM_CODE_ENABLE) */

#endif /* _SROM_VARS_H_ */

