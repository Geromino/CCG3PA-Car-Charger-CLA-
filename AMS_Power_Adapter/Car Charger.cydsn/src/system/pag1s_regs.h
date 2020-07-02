/*******************************************************************************
 * File Name: pag1s_regs.h
 *
 * Version: M0S8-Product-PAG1S.xlsx: 29-Jan-19
 *
 * Description:
 * Cypress product header file.
 *
 * This file is auto generated from the register map spreadsheet.
 * DO NOT MODIFY THIS FILE.
 *
 *******************************************************************************
 * © (2017-2018), Cypress Semiconductor Corporation. All rights reserved.
 *
 * This software, including source code, documentation and related materials
 * ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
 * protected by and subject to worldwide patent protection (United States and
 * foreign), United States copyright laws and international treaty provisions.
 * Cypress hereby grants to licensee a personal, non-exclusive,
 * non-transferable license to copy, use, modify, create derivative works of,
 * and compile the Cypress source code and derivative works for the sole
 * purpose of creating custom software in support of licensee product, such
 * licensee product to be used only in conjunction with Cypress's integrated
 * circuit as specified in the applicable agreement. Any reproduction,
 * modification, translation, compilation, or representation of this Software
 * except as specified above is prohibited without the express written
 * permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * Cypress reserves the right to make changes to the Software without notice.
 * Cypress does not assume any liability arising out of the application or use
 * of Software or any product or circuit described in the Software.
 * Cypress does not authorize its products for use as critical components in
 * any products where a malfunction or failure may reasonably be expected to
 * result in significant injury or death ("High Risk Product"). By including
 * Cypress's product in a High Risk Product, the manufacturer of such system or
 * application assumes all risk of such use and in doing so indemnifies Cypress
 * against all liability.
 *
 * Use of this Software may be limited by and subject to the applicable Cypress
 * software license agreement.
 ******************************************************************************/
#ifndef _PAG1S_REGS_H_
#define _PAG1S_REGS_H_

#include <stdint.h>
#include <stdbool.h>


#define SFLASH_BASE_ADDR                                 (0x0ffff000)

typedef struct __attribute__((__packed__)) {
    volatile uint8_t  prot_row[64];                       /* 0x0ffff000 */
    volatile uint8_t  rsrvd0[63];
    volatile uint8_t  prot_protection;                    /* 0x0ffff07f */
    volatile uint8_t  av_pairs_8b[128];                   /* 0x0ffff080 */
    volatile uint32_t av_pairs_32b[16];                   /* 0x0ffff100 */
    volatile uint32_t cpuss_wounding;                     /* 0x0ffff140 */
    volatile uint32_t silicon_id;                         /* 0x0ffff144 */
    volatile uint16_t cpuss_priv_ram;                     /* 0x0ffff148 */
    volatile uint16_t cpuss_priv_rom_brom;                /* 0x0ffff14a */
    volatile uint16_t cpuss_priv_flash;                   /* 0x0ffff14c */
    volatile uint16_t cpuss_priv_rom_srom;                /* 0x0ffff14e */
    volatile uint16_t hib_key_delay;                      /* 0x0ffff150 */
    volatile uint16_t dpslp_key_delay;                    /* 0x0ffff152 */
    volatile uint8_t  swd_config;                         /* 0x0ffff154 */
    volatile uint8_t  rsrvd1[3];
    volatile uint32_t swd_listen;                         /* 0x0ffff158 */
    volatile uint32_t flash_start;                        /* 0x0ffff15c */
    volatile uint8_t  rsrvd2[9];
    volatile uint8_t  skip_checksum;                      /* 0x0ffff169 */
    volatile uint8_t  initial_pwr_bg_trim1;               /* 0x0ffff16a */
    volatile uint8_t  initial_pwr_bg_trim1_inv;           /* 0x0ffff16b */
    volatile uint8_t  initial_pwr_bg_trim2;               /* 0x0ffff16c */
    volatile uint8_t  initial_pwr_bg_trim2_inv;           /* 0x0ffff16d */
    volatile uint8_t  initial_spcif_trim_m0_dac0;         /* 0x0ffff16e */
    volatile uint8_t  initial_spcif_trim_m0_dac0_inv;     /* 0x0ffff16f */
    volatile uint8_t  prot_virginkey[8];                  /* 0x0ffff170 */
    volatile uint8_t  die_lot[3];                         /* 0x0ffff178 */
    volatile uint8_t  die_wafer;                          /* 0x0ffff17b */
    volatile uint8_t  die_x;                              /* 0x0ffff17c */
    volatile uint8_t  die_y;                              /* 0x0ffff17d */
    volatile uint8_t  die_sort;                           /* 0x0ffff17e */
    volatile uint8_t  die_minor;                          /* 0x0ffff17f */
    volatile uint8_t  pe_te_data[32];                     /* 0x0ffff180 */
    volatile uint32_t pp;                                 /* 0x0ffff1a0 */
    volatile uint32_t e;                                  /* 0x0ffff1a4 */
    volatile uint32_t p;                                  /* 0x0ffff1a8 */
    volatile uint32_t ea_e;                               /* 0x0ffff1ac */
    volatile uint32_t ea_p;                               /* 0x0ffff1b0 */
    volatile uint32_t es_e;                               /* 0x0ffff1b4 */
    volatile uint32_t es_p_eo;                            /* 0x0ffff1b8 */
    volatile uint8_t  rsrvd3[2];
    volatile uint8_t  imo_trim_usbmode_24;                /* 0x0ffff1be */
    volatile uint8_t  imo_trim_usbmode_48;                /* 0x0ffff1bf */
    volatile uint32_t rsrvd4[3];
    volatile uint8_t  imo_tctrim_lt[25];                  /* 0x0ffff1cc */
    volatile uint8_t  imo_trim_lt[25];                    /* 0x0ffff1e5 */
    volatile uint16_t checksum;                           /* 0x0ffff1fe */
} SFLASH_REGS_T, *PSFLASH_REGS_T;

#define SFLASH        ((PSFLASH_REGS_T) SFLASH_BASE_ADDR)


#define PERI_BASE_ADDR                                   (0x40010000)

typedef struct {
    volatile uint32_t div_cmd;                            /* 0x40010000 */
    volatile uint32_t rsrvd0[63];
    volatile uint32_t pclk_ctl[11];                       /* 0x40010100 */
    volatile uint32_t rsrvd1[53];
    volatile uint32_t div_8_ctl[6];                       /* 0x40010200 */
    volatile uint32_t rsrvd2[58];
    volatile uint32_t div_16_ctl[5];                      /* 0x40010300 */
} PERI_REGS_T, *PPERI_REGS_T;

#define PERI        ((PPERI_REGS_T) PERI_BASE_ADDR)


#define HSIOM_BASE_ADDR                                  (0x40020000)

typedef struct {
    volatile uint32_t port_sel;                           /* 0x40020000 */
} HSIOM_REGS_T, *PHSIOM_REGS_T;

#define HSIOM        ((PHSIOM_REGS_T) HSIOM_BASE_ADDR)


#define SRSSULT_BASE_ADDR                                (0x40030000)

typedef struct {
    volatile uint32_t pwr_control;                        /* 0x40030000 */
    volatile uint32_t pwr_key_delay;                      /* 0x40030004 */
    volatile uint32_t pwr_adft_select;                    /* 0x40030008 */
    volatile uint32_t pwr_ddft_select;                    /* 0x4003000c */
    volatile uint32_t pwr_ddft_xres;                      /* 0x40030010 */
    volatile uint32_t tst_mode;                           /* 0x40030014 */
    volatile uint32_t tst_ddft_ctrl;                      /* 0x40030018 */
    volatile uint32_t tst_trim_cntr1;                     /* 0x4003001c */
    volatile uint32_t tst_trim_cntr2;                     /* 0x40030020 */
    volatile uint32_t tst_adft_ctrl;                      /* 0x40030024 */
    volatile uint32_t clk_select;                         /* 0x40030028 */
    volatile uint32_t clk_ilo_config;                     /* 0x4003002c */
    volatile uint32_t clk_imo_config;                     /* 0x40030030 */
    volatile uint32_t clk_dft_select;                     /* 0x40030034 */
    volatile uint32_t wdt_disable_key;                    /* 0x40030038 */
    volatile uint32_t wdt_counter;                        /* 0x4003003c */
    volatile uint32_t wdt_match;                          /* 0x40030040 */
    volatile uint32_t srss_intr;                          /* 0x40030044 */
    volatile uint32_t spare_2;                            /* 0x40030048 */
    volatile uint32_t srss_intr_mask;                     /* 0x4003004c */
    volatile uint32_t srss_adft_control;                  /* 0x40030050 */
    volatile uint32_t res_cause;                          /* 0x40030054 */
    volatile uint32_t res_dft;                            /* 0x40030058 */
    volatile uint32_t rsrvd0[937];
    volatile uint32_t pwr_bg_trim1;                       /* 0x40030f00 */
    volatile uint32_t pwr_bg_trim2;                       /* 0x40030f04 */
    volatile uint32_t clk_imo_select;                     /* 0x40030f08 */
    volatile uint32_t clk_imo_trim1;                      /* 0x40030f0c */
    volatile uint32_t clk_imo_trim2;                      /* 0x40030f10 */
    volatile uint32_t rsrvd1;
    volatile uint32_t clk_imo_trim3;                      /* 0x40030f18 */
    volatile uint32_t regulator;                          /* 0x40030f1c */
} SRSSULT_REGS_T, *PSRSSULT_REGS_T;

#define SRSSULT        ((PSRSSULT_REGS_T) SRSSULT_BASE_ADDR)


#define GPIO_BASE_ADDR                                   (0x40040000)

typedef struct {
    volatile uint32_t dr;                                 /* 0x40040000 */
    volatile uint32_t ps;                                 /* 0x40040004 */
    volatile uint32_t pc;                                 /* 0x40040008 */
    volatile uint32_t intr_cfg;                           /* 0x4004000c */
    volatile uint32_t intr;                               /* 0x40040010 */
    volatile uint32_t rsrvd0;
    volatile uint32_t pc2;                                /* 0x40040018 */
    volatile uint32_t rsrvd1[9];
    volatile uint32_t dr_set;                             /* 0x40040040 */
    volatile uint32_t dr_clr;                             /* 0x40040044 */
    volatile uint32_t dr_inv;                             /* 0x40040048 */
    volatile uint32_t rsrvd2[1005];
    volatile uint32_t intr_cause;                         /* 0x40041000 */
    volatile uint32_t rsrvd3[3];
    volatile uint32_t dft_io_test;                        /* 0x40041010 */
} GPIO_REGS_T, *PGPIO_REGS_T;

#define GPIO        ((PGPIO_REGS_T) GPIO_BASE_ADDR)


#define TCPWM_BASE_ADDR                                  (0x40090000)

typedef struct {
    volatile uint32_t ctrl;                               /* 0x40090000 */
    volatile uint32_t rsrvd0;
    volatile uint32_t cmd;                                /* 0x40090008 */
    volatile uint32_t intr_cause;                         /* 0x4009000c */
} TCPWM_REGS_T, *PTCPWM_REGS_T;

#define TCPWM        ((PTCPWM_REGS_T) TCPWM_BASE_ADDR)


#define CNT_BASE_ADDR                                    (0x40090100)

typedef struct {
    volatile uint32_t ctrl;                               /* 0x40090100 */
    volatile uint32_t status;                             /* 0x40090104 */
    volatile uint32_t counter;                            /* 0x40090108 */
    volatile uint32_t cc;                                 /* 0x4009010c */
    volatile uint32_t cc_buff;                            /* 0x40090110 */
    volatile uint32_t period;                             /* 0x40090114 */
    volatile uint32_t period_buff;                        /* 0x40090118 */
    volatile uint32_t rsrvd0;
    volatile uint32_t tr_ctrl0;                           /* 0x40090120 */
    volatile uint32_t tr_ctrl1;                           /* 0x40090124 */
    volatile uint32_t tr_ctrl2;                           /* 0x40090128 */
    volatile uint32_t rsrvd1;
    volatile uint32_t intr;                               /* 0x40090130 */
    volatile uint32_t intr_set;                           /* 0x40090134 */
    volatile uint32_t intr_mask;                          /* 0x40090138 */
    volatile uint32_t intr_masked;                        /* 0x4009013c */
} CNT_REGS_T, *PCNT_REGS_T;

#define CNT        ((PCNT_REGS_T) CNT_BASE_ADDR)


#define PDSS_BASE_ADDR                                   (0x400a0000)

typedef struct {
    volatile uint32_t ctrl;                               /* 0x400a0000 */
    volatile uint32_t header_info;                        /* 0x400a0004 */
    volatile uint32_t rsrvd0;
    volatile uint32_t tx_header;                          /* 0x400a000c */
    volatile uint32_t tx_mem_data[16];                    /* 0x400a0010 */
    volatile uint32_t rsrvd1[3];
    volatile uint32_t rx_header;                          /* 0x400a005c */
    volatile uint32_t rx_mem_data[16];                    /* 0x400a0060 */
    volatile uint32_t sram_ptr;                           /* 0x400a00a0 */
    volatile uint32_t status;                             /* 0x400a00a4 */
    volatile uint32_t rx_sop_good_crc_en_ctrl;            /* 0x400a00a8 */
    volatile uint32_t rx_expect_goodcrc_msg;              /* 0x400a00ac */
    volatile uint32_t rx_goodcrc_msg;                     /* 0x400a00b0 */
    volatile uint32_t rx_cc_0_cfg;                        /* 0x400a00b4 */
    volatile uint32_t rx_cc_1_cfg;                        /* 0x400a00b8 */
    volatile uint32_t rx_order_set_ctrl;                  /* 0x400a00bc */
    volatile uint32_t tx_goodcrc_msg_order_set;           /* 0x400a00c0 */
    volatile uint32_t tx_ctrl;                            /* 0x400a00c4 */
    volatile uint32_t tx_sop_order_set;                   /* 0x400a00c8 */
    volatile uint32_t tx_hard_cable_order_set;            /* 0x400a00cc */
    volatile uint32_t crc_counter;                        /* 0x400a00d0 */
    volatile uint32_t inter_packet_counter;               /* 0x400a00d4 */
    volatile uint32_t timer_trigger;                      /* 0x400a00d8 */
    volatile uint32_t debug_ctrl;                         /* 0x400a00dc */
    volatile uint32_t debug_cc_0;                         /* 0x400a00e0 */
    volatile uint32_t debug_cc_1;                         /* 0x400a00e4 */
    volatile uint32_t debug_cc_2;                         /* 0x400a00e8 */
    volatile uint32_t rsrvd2[15];
    volatile uint32_t bit_en_cntr_ctrl;                   /* 0x400a0128 */
    volatile uint32_t rsrvd3[2];
    volatile uint32_t lf_cntr;                            /* 0x400a0134 */
    volatile uint32_t lf_cntr_match;                      /* 0x400a0138 */
    volatile uint32_t rsrvd4;
    volatile uint32_t afc_1_ctrl[1];                         /* 0x400a0140 */
    volatile uint32_t rsrvd5[3];
    volatile uint32_t afc_2_ctrl[1];                         /* 0x400a0150 */
    volatile uint32_t rsrvd6[3];
    volatile uint32_t afc_opcode_ctrl[1];                    /* 0x400a0160 */
    volatile uint32_t rsrvd7[3];
    volatile uint32_t afc_ping_pong[1];                      /* 0x400a0170 */
    volatile uint32_t rsrvd8[3];
    volatile uint32_t qc3_chrger_ctrl[1];                    /* 0x400a0180 */
    volatile uint32_t rsrvd9[3];
    volatile uint32_t qc3_device_ctrl[1];                    /* 0x400a0190 */
    volatile uint32_t rsrvd10[3];
    volatile uint32_t afc_sm_status[1];                      /* 0x400a01a0 */
    volatile uint32_t rsrvd11[3];
    volatile uint32_t afc_hs_filter_cfg[1];                  /* 0x400a01b0 */
    volatile uint32_t rsrvd12[3];
    volatile uint32_t adc_sar_ctrl;                       /* 0x400a01c0 */
    volatile uint32_t rsrvd13[7];
    volatile uint32_t refgen_sel6_sel8_ctrl;              /* 0x400a01e0 */
    volatile uint32_t rsrvd14[47];
    volatile uint32_t intr1_cfg;                          /* 0x400a02a0 */
    volatile uint32_t intr1_cfg_cc1_cc2_ls;               /* 0x400a02a4 */
    volatile uint32_t intr1_cfg_vcmp_up_down_ls;          /* 0x400a02a8 */
    volatile uint32_t rsrvd15[2];
    volatile uint32_t intr1_status;                       /* 0x400a02b4 */
    volatile uint32_t intr1;                              /* 0x400a02b8 */
    volatile uint32_t intr1_set;                          /* 0x400a02bc */
    volatile uint32_t intr1_mask;                         /* 0x400a02c0 */
    volatile uint32_t intr1_masked;                       /* 0x400a02c4 */
    volatile uint32_t rsrvd16[15];
    volatile uint32_t intr3_cfg_adc_hs;                   /* 0x400a0304 */
    volatile uint32_t rsrvd17[14];
    volatile uint32_t intr3_status_0;                     /* 0x400a0340 */
    volatile uint32_t rsrvd18;
    volatile uint32_t intr3;                              /* 0x400a0348 */
    volatile uint32_t intr3_set;                          /* 0x400a034c */
    volatile uint32_t intr3_mask;                         /* 0x400a0350 */
    volatile uint32_t intr3_masked;                       /* 0x400a0354 */
    volatile uint32_t rsrvd19[10];
    volatile uint32_t intr5_filter_cfg[6];                /* 0x400a0380 */
    volatile uint32_t rsrvd20[18];
    volatile uint32_t intr5_status_0;                     /* 0x400a03e0 */
    volatile uint32_t rsrvd21;
    volatile uint32_t intr5;                              /* 0x400a03e8 */
    volatile uint32_t intr5_set;                          /* 0x400a03ec */
    volatile uint32_t intr5_mask;                         /* 0x400a03f0 */
    volatile uint32_t intr5_masked;                       /* 0x400a03f4 */
    volatile uint32_t rsrvd22[10];
    volatile uint32_t intr7_filter_cfg[3];                /* 0x400a0420 */
    volatile uint32_t rsrvd23[5];
    volatile uint32_t intr7_status;                       /* 0x400a0440 */
    volatile uint32_t intr7;                              /* 0x400a0444 */
    volatile uint32_t intr7_set;                          /* 0x400a0448 */
    volatile uint32_t intr7_mask;                         /* 0x400a044c */
    volatile uint32_t intr7_masked;                       /* 0x400a0450 */
    volatile uint32_t rsrvd24[7];
    volatile uint32_t intr9_cfg_bch_det[1];               /* 0x400a0470 */
    volatile uint32_t rsrvd25[6];
    volatile uint32_t intr9_status_0;                     /* 0x400a048c */
    volatile uint32_t intr9_status_1;                     /* 0x400a0490 */
    volatile uint32_t intr9;                              /* 0x400a0494 */
    volatile uint32_t intr9_set;                          /* 0x400a0498 */
    volatile uint32_t intr9_mask;                         /* 0x400a049c */
    volatile uint32_t intr9_masked;                       /* 0x400a04a0 */
    volatile uint32_t rsrvd26[7];
    volatile uint32_t intr11_filter_cfg;                  /* 0x400a04c0 */
    volatile uint32_t intr11_status_0;                    /* 0x400a04c4 */
    volatile uint32_t intr11;                             /* 0x400a04c8 */
    volatile uint32_t intr11_set;                         /* 0x400a04cc */
    volatile uint32_t intr11_mask;                        /* 0x400a04d0 */
    volatile uint32_t intr11_masked;                      /* 0x400a04d4 */
    volatile uint32_t rsrvd27[10];
    volatile uint32_t intr0;                              /* 0x400a0500 */
    volatile uint32_t intr0_set;                          /* 0x400a0504 */
    volatile uint32_t intr0_mask;                         /* 0x400a0508 */
    volatile uint32_t intr0_masked;                       /* 0x400a050c */
    volatile uint32_t intr2;                              /* 0x400a0510 */
    volatile uint32_t intr2_set;                          /* 0x400a0514 */
    volatile uint32_t intr2_mask;                         /* 0x400a0518 */
    volatile uint32_t intr2_masked;                       /* 0x400a051c */
    volatile uint32_t intr4;                              /* 0x400a0520 */
    volatile uint32_t intr4_set;                          /* 0x400a0524 */
    volatile uint32_t intr4_mask;                         /* 0x400a0528 */
    volatile uint32_t intr4_masked;                       /* 0x400a052c */
    volatile uint32_t intr6;                              /* 0x400a0530 */
    volatile uint32_t intr6_set;                          /* 0x400a0534 */
    volatile uint32_t intr6_mask;                         /* 0x400a0538 */
    volatile uint32_t intr6_masked;                       /* 0x400a053c */
    volatile uint32_t intr8;                              /* 0x400a0540 */
    volatile uint32_t intr8_set;                          /* 0x400a0544 */
    volatile uint32_t intr8_mask;                         /* 0x400a0548 */
    volatile uint32_t intr8_masked;                       /* 0x400a054c */
    volatile uint32_t rsrvd28[12];
    volatile uint32_t ddft_mux;                           /* 0x400a0580 */
    volatile uint32_t intr_ddft_mux;                      /* 0x400a0584 */
    volatile uint32_t ncell_ddft_mux;                     /* 0x400a0588 */
    volatile uint32_t gpio_ddft_mux;                      /* 0x400a058c */
    volatile uint32_t gpio_intr_ddft_mux;                 /* 0x400a0590 */
    volatile uint32_t fault_gpio_ctrl;                    /* 0x400a0594 */
    volatile uint32_t rsrvd29[26];
    volatile uint32_t cc_ctrl_0;                          /* 0x400a0600 */
    volatile uint32_t cc_ctrl_1;                          /* 0x400a0604 */
    volatile uint32_t dpslp_ref_ctrl;                     /* 0x400a0608 */
    volatile uint32_t rsrvd30;
    volatile uint32_t pump_ctrl;                          /* 0x400a0610 */
    volatile uint32_t rsrvd31[2];
    volatile uint32_t vreg_vsys_ctrl;                     /* 0x400a061c */
    volatile uint32_t rsrvd32[2];
    volatile uint32_t amux_ctrl;                          /* 0x400a0628 */
    volatile uint32_t rsrvd33;
    volatile uint32_t amux_denfet_ctrl;                   /* 0x400a0630 */
    volatile uint32_t rsrvd34[19];
    volatile uint32_t adc_ctrl;                           /* 0x400a0680 */
    volatile uint32_t rsrvd35[3];
    volatile uint32_t refgen_0_ctrl;                      /* 0x400a0690 */
    volatile uint32_t rsrvd36[3];
    volatile uint32_t refgen_1_ctrl;                      /* 0x400a06a0 */
    volatile uint32_t rsrvd37[3];
    volatile uint32_t refgen_2_ctrl;                      /* 0x400a06b0 */
    volatile uint32_t rsrvd38[3];
    volatile uint32_t refgen_3_ctrl;                      /* 0x400a06c0 */
    volatile uint32_t rsrvd39[3];
    volatile uint32_t refgen_4_ctrl;                      /* 0x400a06d0 */
    volatile uint32_t rsrvd40[11];
    volatile uint32_t bch_det_0_ctrl[1];                  /* 0x400a0700 */
    volatile uint32_t rsrvd41[3];
    volatile uint32_t bch_det_1_ctrl[1];                  /* 0x400a0710 */
    volatile uint32_t rsrvd42[11];
    volatile uint32_t lscsa_0_ctrl;                       /* 0x400a0740 */
    volatile uint32_t rsrvd43[15];
    volatile uint32_t lscsa_1_ctrl;                       /* 0x400a0780 */
    volatile uint32_t rsrvd44[3];
    volatile uint32_t dischg_shv_ctrl[2];                 /* 0x400a0790 */
    volatile uint32_t rsrvd45[6];
    volatile uint32_t comp_tr_ctrl;                       /* 0x400a07b0 */
    volatile uint32_t rsrvd46[7];
    volatile uint32_t comp_ctrl[8];                       /* 0x400a07d0 */
    volatile uint32_t rsrvd47[43];
    volatile uint32_t dischg_shv_1_ctrl[2];               /* 0x400a089c */
    volatile uint32_t rsrvd48[10];
    volatile uint32_t amux_nhvn_ctrl;                     /* 0x400a08cc */
    volatile uint32_t pds_scp_ctrl;                       /* 0x400a08d0 */
    volatile uint32_t pwm_0_ctrl;                         /* 0x400a08d4 */
    volatile uint32_t pwm_1_ctrl;                         /* 0x400a08d8 */
    volatile uint32_t srsns_0_ctrl;                       /* 0x400a08dc */
    volatile uint32_t srsns_1_ctrl;                       /* 0x400a08e0 */
    volatile uint32_t srsns_2_ctrl;                       /* 0x400a08e4 */
    volatile uint32_t srsns_3_ctrl;                       /* 0x400a08e8 */
    volatile uint32_t gdrv_0_ctrl;                        /* 0x400a08ec */
    volatile uint32_t gdrv_1_ctrl;                        /* 0x400a08f0 */
    volatile uint32_t ea_ctrl;                            /* 0x400a08f4 */
    volatile uint32_t pds_ea_1_ctrl;                      /* 0x400a08f8 */
    volatile uint32_t pds_ea_2_ctrl;                      /* 0x400a08fc */
    volatile uint32_t ngdo_ctrl;                          /* 0x400a0900 */
    volatile uint32_t pds_vreg_ctrl;                      /* 0x400a0904 */
    volatile uint32_t rsrvd49[1470];
    volatile uint32_t ngdo_1_cfg;                         /* 0x400a2000 */
    volatile uint32_t ngdo_2_cfg;                         /* 0x400a2004 */
    volatile uint32_t srsns_1_cfg;                        /* 0x400a2008 */
    volatile uint32_t srsns_2_cfg;                        /* 0x400a200c */
    volatile uint32_t intr15_cfg_pds_scp;                 /* 0x400a2010 */
    volatile uint32_t intr15_cfg_0_srsense;               /* 0x400a2014 */
    volatile uint32_t intr15_cfg_1_srsense;               /* 0x400a2018 */
    volatile uint32_t intr15_cfg_2_srsense;               /* 0x400a201c */
    volatile uint32_t intr15_cfg_3_srsense;               /* 0x400a2020 */
    volatile uint32_t intr15_cfg_4_srsense;               /* 0x400a2024 */
    volatile uint32_t intr15_cfg_0_pwm;                   /* 0x400a2028 */
    volatile uint32_t intr15_cfg_1_pwm;                   /* 0x400a202c */
    volatile uint32_t intr15_cfg_cc_flag;                 /* 0x400a2030 */
    volatile uint32_t intr15_cfg_vreg20_vbus;             /* 0x400a2034 */
    volatile uint32_t intr15_status;                      /* 0x400a2038 */
    volatile uint32_t intr15;                             /* 0x400a203c */
    volatile uint32_t intr15_set;                         /* 0x400a2040 */
    volatile uint32_t intr15_mask;                        /* 0x400a2044 */
    volatile uint32_t intr15_masked;                      /* 0x400a2048 */
    volatile uint32_t vbtr_cfg;                           /* 0x400a204c */
    volatile uint32_t vbtr_ctrl;                          /* 0x400a2050 */
    volatile uint32_t vbtr_src_snk_opr_value;             /* 0x400a2054 */
    volatile uint32_t vbtr_src_init_fin_value;            /* 0x400a2058 */
    volatile uint32_t vbtr_snk_init_fin_value;            /* 0x400a205c */
    volatile uint32_t vbtr_status;                        /* 0x400a2060 */
    volatile uint32_t pasc_ctrl;                          /* 0x400a2064 */
    volatile uint32_t srgdrv_0_ctrl;                      /* 0x400a2068 */
    volatile uint32_t srgdrv_1_ctrl;                      /* 0x400a206c */
    volatile uint32_t pasc_pwm_0_ctrl;                    /* 0x400a2070 */
    volatile uint32_t pasc_pwm_1_ctrl;                    /* 0x400a2074 */
    volatile uint32_t pasc_pwm_2_ctrl;                    /* 0x400a2078 */
    volatile uint32_t mode_0_ctrl;                        /* 0x400a207c */
    volatile uint32_t mode_1_ctrl;                        /* 0x400a2080 */
    volatile uint32_t mode_2_ctrl;                        /* 0x400a2084 */
    volatile uint32_t mode_3_ctrl;                        /* 0x400a2088 */
    volatile uint32_t mode_4_ctrl;                        /* 0x400a208c */
    volatile uint32_t peakgen_0_ctrl;                     /* 0x400a2090 */
    volatile uint32_t peakgen_1_ctrl;                     /* 0x400a2094 */
    volatile uint32_t peakgen_2_ctrl;                     /* 0x400a2098 */
    volatile uint32_t feedfwd_ctrl;                       /* 0x400a209c */
    volatile uint32_t hip_seq_gen_0_ctrl;                 /* 0x400a20a0 */
    volatile uint32_t hip_seq_gen_1_ctrl;                 /* 0x400a20a4 */
    volatile uint32_t hip_seq_gen_2_ctrl;                 /* 0x400a20a8 */
    volatile uint32_t pasc_status_0;                      /* 0x400a20ac */
    volatile uint32_t pasc_status_1;                      /* 0x400a20b0 */
    volatile uint32_t pasc_status_2;                      /* 0x400a20b4 */
    volatile uint32_t pasc_status_3;                      /* 0x400a20b8 */
    volatile uint32_t pasc_status_4;                      /* 0x400a20bc */
    volatile uint32_t pasc_ddft_mux;                      /* 0x400a20c0 */
    volatile uint32_t pasc_gpio_ddft_mux;                 /* 0x400a20c4 */
    volatile uint32_t rsrvd50[978];
    volatile uint32_t peak_mem_data[8];                   /* 0x400a3010 */
} PDSS_REGS_T, *PPDSS_REGS_T;

#define PDSS        ((PPDSS_REGS_T) PDSS_BASE_ADDR)


#define PDSS_TRIMS_BASE_ADDR                             (0x400aff00)

typedef struct {
    volatile uint32_t trim_cc_0;                          /* 0x400aff00 */
    volatile uint32_t trim_cc_1;                          /* 0x400aff04 */
    volatile uint32_t trim_cc_2;                          /* 0x400aff08 */
    volatile uint32_t trim_cc_3;                          /* 0x400aff0c */
    volatile uint32_t trim_cc_4;                          /* 0x400aff10 */
    volatile uint32_t trim_cc_5;                          /* 0x400aff14 */
    volatile uint32_t trim_cc_6;                          /* 0x400aff18 */
    volatile uint32_t trim_cc_7;                          /* 0x400aff1c */
    volatile uint32_t trim_pds_vreg20;                    /* 0x400aff20 */
    volatile uint32_t trim_bch_det1_0;                    /* 0x400aff24 */
    volatile uint32_t trim_bch_det1_1;                    /* 0x400aff28 */
    volatile uint32_t trim_bch_det1_2;                    /* 0x400aff2c */
    volatile uint32_t trim_bch_det1_3;                    /* 0x400aff30 */
    volatile uint32_t trim_lscsa1_0;                      /* 0x400aff34 */
    volatile uint32_t trim_lscsa1_1;                      /* 0x400aff38 */
    volatile uint32_t trim_lscsa1_2;                      /* 0x400aff3c */
    volatile uint32_t trim_refgen1_0;                     /* 0x400aff40 */
    volatile uint32_t trim_refgen1_1;                     /* 0x400aff44 */
    volatile uint32_t trim_pds_ea;                        /* 0x400aff48 */
    volatile uint32_t trim_spare1;                        /* 0x400aff4c */
    volatile uint32_t trim_comp1_0;                       /* 0x400aff50 */
    volatile uint32_t trim_dpslp_0;                       /* 0x400aff54 */
    volatile uint32_t trim_dpslp_1;                       /* 0x400aff58 */
    volatile uint32_t trim_sr_sense_0;                    /* 0x400aff5c */
    volatile uint32_t trim_sr_sense_1;                    /* 0x400aff60 */
    volatile uint32_t trim_sr_sense_2;                    /* 0x400aff64 */
    volatile uint32_t trim_sr_sense_3;                    /* 0x400aff68 */
    volatile uint32_t trim_sr_sense_4;                    /* 0x400aff6c */
    volatile uint32_t trim_sr_sense_5;                    /* 0x400aff70 */
    volatile uint32_t trim_sr_sense_6;                    /* 0x400aff74 */
    volatile uint32_t trim_sr_sense_7;                    /* 0x400aff78 */
    volatile uint32_t trim_sr_sense_8;                    /* 0x400aff7c */
    volatile uint32_t trim_sr_sense_9;                    /* 0x400aff80 */
    volatile uint32_t trim_pwm_0;                         /* 0x400aff84 */
    volatile uint32_t trim_pwm_1;                         /* 0x400aff88 */
    volatile uint32_t trim_pwm_2;                         /* 0x400aff8c */
    volatile uint32_t trim_scp;                           /* 0x400aff90 */
} PDSS_TRIMS_REGS_T, *PPDSS_TRIMS_REGS_T;

#define PDSS_TRIMS        ((PPDSS_TRIMS_REGS_T) PDSS_TRIMS_BASE_ADDR)


#define CPUSS_BASE_ADDR                                  (0x40100000)

typedef struct {
    volatile uint32_t config;                             /* 0x40100000 */
    volatile uint32_t sysreq;                             /* 0x40100004 */
    volatile uint32_t sysarg;                             /* 0x40100008 */
    volatile uint32_t protection;                         /* 0x4010000c */
    volatile uint32_t priv_rom;                           /* 0x40100010 */
    volatile uint32_t priv_ram;                           /* 0x40100014 */
    volatile uint32_t priv_flash;                         /* 0x40100018 */
    volatile uint32_t wounding;                           /* 0x4010001c */
    volatile uint32_t rsrvd0[4];
    volatile uint32_t flash_ctl;                          /* 0x40100030 */
    volatile uint32_t rom_ctl;                            /* 0x40100034 */
    volatile uint32_t rsrvd1[2];
    volatile uint32_t bist_cmd;                           /* 0x40100040 */
    volatile uint32_t bist_data;                          /* 0x40100044 */
    volatile uint32_t rsrvd2;
    volatile uint32_t bist_ctl;                           /* 0x4010004c */
    volatile uint32_t bist_step0_ctl;                     /* 0x40100050 */
    volatile uint32_t rsrvd3[11];
    volatile uint32_t bist_status;                        /* 0x40100080 */
    volatile uint32_t bist_data_act;                      /* 0x40100084 */
    volatile uint32_t bist_data_exp;                      /* 0x40100088 */
    volatile uint32_t bist_addr;                          /* 0x4010008c */
    volatile uint32_t bist_misr;                          /* 0x40100090 */
    volatile uint32_t rsrvd4[11];
    volatile uint32_t ptm_ctl;                            /* 0x401000c0 */
} CPUSS_REGS_T, *PCPUSS_REGS_T;

#define CPUSS        ((PCPUSS_REGS_T) CPUSS_BASE_ADDR)


#define SPCIF_BASE_ADDR                                  (0x40110000)

typedef struct {
    volatile uint32_t geometry;                           /* 0x40110000 */
    volatile uint32_t address;                            /* 0x40110004 */
    volatile uint32_t timer;                              /* 0x40110008 */
    volatile uint32_t flash_control;                      /* 0x4011000c */
    volatile uint32_t flash_wr_data;                      /* 0x40110010 */
    volatile uint32_t rsrvd0[5];
    volatile uint32_t bookmark;                           /* 0x40110028 */
    volatile uint32_t rsrvd1;
    volatile uint32_t fmlt_dft;                           /* 0x40110030 */
    volatile uint32_t rsrvd2[495];
    volatile uint32_t intr;                               /* 0x401107f0 */
    volatile uint32_t intr_set;                           /* 0x401107f4 */
    volatile uint32_t intr_mask;                          /* 0x401107f8 */
    volatile uint32_t intr_masked;                        /* 0x401107fc */
    volatile uint32_t rsrvd3[15808];
    volatile uint32_t trim_m0_dac0;                       /* 0x4011ff00 */
    volatile uint32_t trim_m0_dac1;                       /* 0x4011ff04 */
    volatile uint32_t trim_m0_dac2;                       /* 0x4011ff08 */
    volatile uint32_t trim_m0_dac3;                       /* 0x4011ff0c */
} SPCIF_REGS_T, *PSPCIF_REGS_T;

#define SPCIF        ((PSPCIF_REGS_T) SPCIF_BASE_ADDR)


#define CM0_BASE_ADDR                                    (0xe0001000)

typedef struct {
    volatile uint32_t dwt_ctrl;                           /* 0xe0001000 */
    volatile uint32_t rsrvd0[6];
    volatile uint32_t dwt_pcsr;                           /* 0xe000101c */
    volatile uint32_t dwt_comp0;                          /* 0xe0001020 */
    volatile uint32_t dwt_mask0;                          /* 0xe0001024 */
    volatile uint32_t dwt_function0;                      /* 0xe0001028 */
    volatile uint32_t rsrvd1;
    volatile uint32_t dwt_comp1;                          /* 0xe0001030 */
    volatile uint32_t dwt_mask1;                          /* 0xe0001034 */
    volatile uint32_t dwt_function1;                      /* 0xe0001038 */
    volatile uint32_t rsrvd2[997];
    volatile uint32_t dwt_pid4;                           /* 0xe0001fd0 */
    volatile uint32_t rsrvd3[3];
    volatile uint32_t dwt_pid0;                           /* 0xe0001fe0 */
    volatile uint32_t dwt_pid1;                           /* 0xe0001fe4 */
    volatile uint32_t dwt_pid2;                           /* 0xe0001fe8 */
    volatile uint32_t dwt_pid3;                           /* 0xe0001fec */
    volatile uint32_t dwt_cid0;                           /* 0xe0001ff0 */
    volatile uint32_t dwt_cid1;                           /* 0xe0001ff4 */
    volatile uint32_t dwt_cid2;                           /* 0xe0001ff8 */
    volatile uint32_t dwt_cid3;                           /* 0xe0001ffc */
    volatile uint32_t bp_ctrl;                            /* 0xe0002000 */
    volatile uint32_t rsrvd4;
    volatile uint32_t bp_comp0;                           /* 0xe0002008 */
    volatile uint32_t bp_comp1;                           /* 0xe000200c */
    volatile uint32_t bp_comp2;                           /* 0xe0002010 */
    volatile uint32_t bp_comp3;                           /* 0xe0002014 */
    volatile uint32_t rsrvd5[1006];
    volatile uint32_t bp_pid4;                            /* 0xe0002fd0 */
    volatile uint32_t rsrvd6[3];
    volatile uint32_t bp_pid0;                            /* 0xe0002fe0 */
    volatile uint32_t bp_pid1;                            /* 0xe0002fe4 */
    volatile uint32_t bp_pid2;                            /* 0xe0002fe8 */
    volatile uint32_t bp_pid3;                            /* 0xe0002fec */
    volatile uint32_t bp_cid0;                            /* 0xe0002ff0 */
    volatile uint32_t bp_cid1;                            /* 0xe0002ff4 */
    volatile uint32_t bp_cid2;                            /* 0xe0002ff8 */
    volatile uint32_t bp_cid3;                            /* 0xe0002ffc */
    volatile uint32_t rsrvd7[11266];
    volatile uint32_t actlr;                              /* 0xe000e008 */
    volatile uint32_t rsrvd8;
    volatile uint32_t syst_csr;                           /* 0xe000e010 */
    volatile uint32_t syst_rvr;                           /* 0xe000e014 */
    volatile uint32_t syst_cvr;                           /* 0xe000e018 */
    volatile uint32_t syst_calib;                         /* 0xe000e01c */
    volatile uint32_t rsrvd9[56];
    volatile uint32_t iser;                               /* 0xe000e100 */
    volatile uint32_t rsrvd10[31];
    volatile uint32_t icer;                               /* 0xe000e180 */
    volatile uint32_t rsrvd11[31];
    volatile uint32_t ispr;                               /* 0xe000e200 */
    volatile uint32_t rsrvd12[31];
    volatile uint32_t icpr;                               /* 0xe000e280 */
    volatile uint32_t rsrvd13[95];
    volatile uint32_t ipr[8];                             /* 0xe000e400 */
    volatile uint32_t rsrvd14[568];
    volatile uint32_t cpuid;                              /* 0xe000ed00 */
    volatile uint32_t icsr;                               /* 0xe000ed04 */
    volatile uint32_t rsrvd15;
    volatile uint32_t aircr;                              /* 0xe000ed0c */
    volatile uint32_t scr;                                /* 0xe000ed10 */
    volatile uint32_t ccr;                                /* 0xe000ed14 */
    volatile uint32_t rsrvd16;
    volatile uint32_t shpr2;                              /* 0xe000ed1c */
    volatile uint32_t shpr3;                              /* 0xe000ed20 */
    volatile uint32_t shcsr;                              /* 0xe000ed24 */
    volatile uint32_t rsrvd17[2];
    volatile uint32_t dfsr;                               /* 0xe000ed30 */
    volatile uint32_t rsrvd18[47];
    volatile uint32_t dhcsr;                              /* 0xe000edf0 */
    volatile uint32_t dcrsr;                              /* 0xe000edf4 */
    volatile uint32_t dcrdr;                              /* 0xe000edf8 */
    volatile uint32_t demcr;                              /* 0xe000edfc */
    volatile uint32_t rsrvd19[116];
    volatile uint32_t scs_pid4;                           /* 0xe000efd0 */
    volatile uint32_t rsrvd20[3];
    volatile uint32_t scs_pid0;                           /* 0xe000efe0 */
    volatile uint32_t scs_pid1;                           /* 0xe000efe4 */
    volatile uint32_t scs_pid2;                           /* 0xe000efe8 */
    volatile uint32_t scs_pid3;                           /* 0xe000efec */
    volatile uint32_t scs_cid0;                           /* 0xe000eff0 */
    volatile uint32_t scs_cid1;                           /* 0xe000eff4 */
    volatile uint32_t scs_cid2;                           /* 0xe000eff8 */
    volatile uint32_t scs_cid3;                           /* 0xe000effc */
    volatile uint32_t rsrvd21[245760];
    volatile uint32_t rom_scs;                            /* 0xe00ff000 */
    volatile uint32_t rom_dwt;                            /* 0xe00ff004 */
    volatile uint32_t rom_bpu;                            /* 0xe00ff008 */
    volatile uint32_t rom_end;                            /* 0xe00ff00c */
    volatile uint32_t rsrvd22[1007];
    volatile uint32_t rom_csmt;                           /* 0xe00fffcc */
    volatile uint32_t rom_pid4;                           /* 0xe00fffd0 */
    volatile uint32_t rsrvd23[3];
    volatile uint32_t rom_pid0;                           /* 0xe00fffe0 */
    volatile uint32_t rom_pid1;                           /* 0xe00fffe4 */
    volatile uint32_t rom_pid2;                           /* 0xe00fffe8 */
    volatile uint32_t rom_pid3;                           /* 0xe00fffec */
    volatile uint32_t rom_cid0;                           /* 0xe00ffff0 */
    volatile uint32_t rom_cid1;                           /* 0xe00ffff4 */
    volatile uint32_t rom_cid2;                           /* 0xe00ffff8 */
    volatile uint32_t rom_cid3;                           /* 0xe00ffffc */
} CM0_REGS_T, *PCM0_REGS_T;

#define CM0        ((PCM0_REGS_T) CM0_BASE_ADDR)


#define ROMTABLE_BASE_ADDR                               (0xf0000000)

typedef struct {
    volatile uint32_t addr;                               /* 0xf0000000 */
    volatile uint32_t rsrvd0[1010];
    volatile uint32_t did;                                /* 0xf0000fcc */
    volatile uint32_t pid4;                               /* 0xf0000fd0 */
    volatile uint32_t pid5;                               /* 0xf0000fd4 */
    volatile uint32_t pid6;                               /* 0xf0000fd8 */
    volatile uint32_t pid7;                               /* 0xf0000fdc */
    volatile uint32_t pid0;                               /* 0xf0000fe0 */
    volatile uint32_t pid1;                               /* 0xf0000fe4 */
    volatile uint32_t pid2;                               /* 0xf0000fe8 */
    volatile uint32_t pid3;                               /* 0xf0000fec */
    volatile uint32_t cid0;                               /* 0xf0000ff0 */
    volatile uint32_t cid1;                               /* 0xf0000ff4 */
    volatile uint32_t cid2;                               /* 0xf0000ff8 */
    volatile uint32_t cid3;                               /* 0xf0000ffc */
} ROMTABLE_REGS_T, *PROMTABLE_REGS_T;

#define ROMTABLE        ((PROMTABLE_REGS_T) ROMTABLE_BASE_ADDR)


/*******************************************************************************
 ************************** REGISTER FIELD DEFINITIONS *************************
 ******************************************************************************/

/*
 * Per Page Write Protection
 * Flash row protection data for Macro #0 (1 bit for each page in main data
 * area).  Byte N, bit X applies to (Page 8*N+X).  When set indicates row
 * cannot be erased or programmed except during total erase and back to PROTECTION=OPEN.
 */
#define SFLASH_PROT_ROW_ADDRESS(n)                          (0x0ffff000 + ((n) * (0x0001)))
#define SFLASH_PROT_ROW(n)                                  (*(volatile uint8_t *)(0x0ffff000 + ((n) * 0x0001)))
#define SFLASH_PROT_ROW_DEFAULT                             (0x00000000)

/*
 * Protection Data (1b per page)
 */
#define SFLASH_PROT_ROW_DATA8_MASK                          (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_PROT_ROW_DATA8_POS                           (0)


/*
 * Protection Level
 * Copied during Boot to CPUSS.PROTECTION
 */
#define SFLASH_PROT_PROTECTION_ADDRESS                      (0x0ffff07f)
#define SFLASH_PROT_PROTECTION                              (*(volatile uint8_t *)(0x0ffff07f))
#define SFLASH_PROT_PROTECTION_DEFAULT                      (0x00000000)

/*
 * Current Protection Mode - note that encoding is different from CPUSS_PROTECTION
 * !!
 */
#define SFLASH_PROT_PROTECTION_PROT_LEVEL_MASK              (0x00000003) /* <0:1> :RW:X: */
#define SFLASH_PROT_PROTECTION_PROT_LEVEL_POS               (0)


/*
 * 8b Addr/Value pair Section
 * Up to 32 Strings of (8b Compressed Address (CA), N*8b Data).  For each
 * string the data value is copied into the trim region for the indicated
 * peripheral on boot.  This section is used only for trim registers.  Compressed
 * address is encoded as follows:
 * BridgeNo= CA[3]
 * PeripheralNo= CA[7:4]
 * NumberOfBytes= CA[2:0]+1
 * Address= 0x4000FF00 + BridgeNo*0x100000 + PerpheralNo*0x10000 + RegisterIndex*4
 */
#define SFLASH_AV_PAIRS_8B_ADDRESS(n)                       (0x0ffff080 + ((n) * (0x0001)))
#define SFLASH_AV_PAIRS_8B(n)                               (*(volatile uint8_t *)(0x0ffff080 + ((n) * 0x0001)))
#define SFLASH_AV_PAIRS_8B_DEFAULT                          (0x00000000)

/*
 * Address or Value Byte
 */
#define SFLASH_AV_PAIRS_8B_DATA8_MASK                       (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_AV_PAIRS_8B_DATA8_POS                        (0)


/*
 * 32b Addr/Value pair Section
 * 8 Pairs of full 32-bit Address and Value. For each pair the value is copied
 * into the indicated address on boot.
 */
#define SFLASH_AV_PAIRS_32B_ADDRESS(n)                      (0x0ffff100 + ((n) * (0x0004)))
#define SFLASH_AV_PAIRS_32B(n)                              (*(volatile uint32_t *)(0x0ffff100 + ((n) * 0x0004)))
#define SFLASH_AV_PAIRS_32B_DEFAULT                         (0x00000000)

/*
 * Address or Value Word
 */
#define SFLASH_AV_PAIRS_32B_DATA32_MASK                     (0xffffffff) /* <0:31> :RW:X: */
#define SFLASH_AV_PAIRS_32B_DATA32_POS                      (0)


/*
 * CPUSS Wounding Register
 * Copied during Boot to CPUSS.WOUNDING
 */
#define SFLASH_CPUSS_WOUNDING_ADDRESS                       (0x0ffff140)
#define SFLASH_CPUSS_WOUNDING                               (*(volatile uint32_t *)(0x0ffff140))
#define SFLASH_CPUSS_WOUNDING_DEFAULT                       (0x00000000)

/*
 * Data to use for register
 */
#define SFLASH_CPUSS_WOUNDING_DATA32_MASK                   (0xffffffff) /* <0:31> :RW:X: */
#define SFLASH_CPUSS_WOUNDING_DATA32_POS                    (0)


/*
 * Silicon ID
 * Copied during Boot to CPUSS.SILICON_ID
 */
#define SFLASH_SILICON_ID_ADDRESS                           (0x0ffff144)
#define SFLASH_SILICON_ID                                   (*(volatile uint32_t *)(0x0ffff144))
#define SFLASH_SILICON_ID_DEFAULT                           (0x00000000)

/*
 * Silicon ID
 */
#define SFLASH_SILICON_ID_ID_MASK                           (0x0000ffff) /* <0:15> :RW:X: */
#define SFLASH_SILICON_ID_ID_POS                            (0)


/*
 * RAM Privileged Limit
 * Copied during Boot to CPUSS.PRIV_RAM
 */
#define SFLASH_CPUSS_PRIV_RAM_ADDRESS                       (0x0ffff148)
#define SFLASH_CPUSS_PRIV_RAM                               (*(volatile uint16_t *)(0x0ffff148))
#define SFLASH_CPUSS_PRIV_RAM_DEFAULT                       (0x00000000)

/*
 * Indicates the limit where the privileged area of SRAM starts in increments
 * of 256 Bytes.
 * "0":  Entire SRAM is Privileged.
 * "1":  First 256 Bytes are User accessable.
 *
 * Any number larger than the size of the SRAM indicates that the entire
 * SRAM is user mode accessible.
 */
#define SFLASH_CPUSS_PRIV_RAM_RAM_PROT_LIMIT_MASK           (0x000001ff) /* <0:8> R:RW:0: */
#define SFLASH_CPUSS_PRIV_RAM_RAM_PROT_LIMIT_POS            (0)


/*
 * Boot ROM Privileged Limit
 * Copied during Boot to CPUSS.PRIV_ROM.BROM_PROT_LIMIT
 */
#define SFLASH_CPUSS_PRIV_ROM_BROM_ADDRESS                  (0x0ffff14a)
#define SFLASH_CPUSS_PRIV_ROM_BROM                          (*(volatile uint16_t *)(0x0ffff14a))
#define SFLASH_CPUSS_PRIV_ROM_BROM_DEFAULT                  (0x00000000)

/*
 * Indicates the limit where the privileged area of the Boot ROM partition
 * starts in increments of 256 Bytes.
 * "0":  Entire Boot ROM is Privileged.
 * "1":  First 256 Bytes are User accessable.
 * ...
 * BROM_PROT_LIMIT >= "Boot ROM partition capacity": Entire Boot ROM partition
 * is user mode accessible.
 */
#define SFLASH_CPUSS_PRIV_ROM_BROM_BROM_PROT_LIMIT_MASK     (0x000000ff) /* <0:7> R:RW:0: */
#define SFLASH_CPUSS_PRIV_ROM_BROM_BROM_PROT_LIMIT_POS      (0)


/*
 * Flash Privileged Limit
 * Copied during Boot CPUSS.PRIV_FLASH
 */
#define SFLASH_CPUSS_PRIV_FLASH_ADDRESS                     (0x0ffff14c)
#define SFLASH_CPUSS_PRIV_FLASH                             (*(volatile uint16_t *)(0x0ffff14c))
#define SFLASH_CPUSS_PRIV_FLASH_DEFAULT                     (0x00000000)

/*
 * Indicates the limit where the privileged area of flash starts in increments
 * of 256 Bytes.
 * "0":  Entire flash is Privileged.
 * "1":  First 256 Bytes are User accessable.
 *
 * Any number larger than the size of the flash indicates that the entire
 * flash is user mode accessible. Note that SuperVisory rows are always User
 * accessable.
 *
 * If FLASH_PROT_LIMIT defines a non-empty privileged area, the boot ROM
 * will assume that a system call table exists at the beginning of the Flash
 * privileged area and use it for all SystemCalls made using SYSREQ.
 */
#define SFLASH_CPUSS_PRIV_FLASH_FLASH_PROT_LIMIT_MASK       (0x000007ff) /* <0:10> R:RW:0: */
#define SFLASH_CPUSS_PRIV_FLASH_FLASH_PROT_LIMIT_POS        (0)


/*
 * System ROM Privileged Limit
 * Copied during Boot to CPUSS.PRIV_ROM.SROM_PROT_LIMIT
 */
#define SFLASH_CPUSS_PRIV_ROM_SROM_ADDRESS                  (0x0ffff14e)
#define SFLASH_CPUSS_PRIV_ROM_SROM                          (*(volatile uint16_t *)(0x0ffff14e))
#define SFLASH_CPUSS_PRIV_ROM_SROM_DEFAULT                  (0x00000000)

/*
 * Indicates the limit where the privileged area of System ROM partition
 * starts in increments of 256 Bytes. The limit is wrt. the start of the
 * ROM memory (start of the Boot ROM partition).
 * SROM_PROT_LIMIT * 256 Byte <= "Boot ROM partition capacity":  Entire System
 * ROM is Privileged.
 * SROM_PROT_LIMIT * 256 Byte > "Boot ROM partition capacity":  First SROM_PROT_LIMIT
 * * 256 - "Boot ROM partition capacity" Bytes are User accessable.
 * ...
 * SROM_PROT_LIMIT >= "ROM capacity": Entire System ROM is user mode accessible.
 */
#define SFLASH_CPUSS_PRIV_ROM_SROM_SROM_PROT_LIMIT_MASK     (0x000003ff) /* <0:9> R:RW:0: */
#define SFLASH_CPUSS_PRIV_ROM_SROM_SROM_PROT_LIMIT_POS      (0)


/*
 * Hibernate wakeup value for PWR_KEY_DELAY
 * Used by firmware during entry into hibernate
 */
#define SFLASH_HIB_KEY_DELAY_ADDRESS                        (0x0ffff150)
#define SFLASH_HIB_KEY_DELAY                                (*(volatile uint16_t *)(0x0ffff150))
#define SFLASH_HIB_KEY_DELAY_DEFAULT                        (0x00000000)

/*
 * Delay (in 12MHz IMO clock cycles) to wait for references to settle on
 * wakeup from hibernate/deepsleep.  PBOD is ignored and system does not
 * resume until this delay expires. Note that the same delay on POR is hard-coded.
 */
#define SFLASH_HIB_KEY_DELAY_WAKEUP_HOLDOFF_MASK            (0x000003ff) /* <0:9> R:RW:X: */
#define SFLASH_HIB_KEY_DELAY_WAKEUP_HOLDOFF_POS             (0)


/*
 * DeepSleep wakeup value for PWR_KEY_DELAY
 * Used by firmware during entry into hibernate
 */
#define SFLASH_DPSLP_KEY_DELAY_ADDRESS                      (0x0ffff152)
#define SFLASH_DPSLP_KEY_DELAY                              (*(volatile uint16_t *)(0x0ffff152))
#define SFLASH_DPSLP_KEY_DELAY_DEFAULT                      (0x00000000)

/*
 * Delay (in 12MHz IMO clock cycles) to wait for references to settle on
 * wakeup from hibernate/deepsleep.  PBOD is ignored and system does not
 * resume until this delay expires. Note that the same delay on POR is hard-coded.
 */
#define SFLASH_DPSLP_KEY_DELAY_WAKEUP_HOLDOFF_MASK          (0x000003ff) /* <0:9> R:RW:X: */
#define SFLASH_DPSLP_KEY_DELAY_WAKEUP_HOLDOFF_POS           (0)


/*
 * SWD pinout selector (not present in TSG4/TSG5-M)
 * Used in BootROM to determine primary or alternate SWD location.
 */
#define SFLASH_SWD_CONFIG_ADDRESS                           (0x0ffff154)
#define SFLASH_SWD_CONFIG                                   (*(volatile uint8_t *)(0x0ffff154))
#define SFLASH_SWD_CONFIG_DEFAULT                           (0x00000000)

/*
 * 0: Use Primary SWD location
 * 1: Use Alternate SWD location
 */
#define SFLASH_SWD_CONFIG_SWD_SELECT                        (1u << 0) /* <0:0> :RW:X: */


/*
 * Listen Window Length
 * Number of clock cycles to wait for SWD acquire to occur during boot.
 */
#define SFLASH_SWD_LISTEN_ADDRESS                           (0x0ffff158)
#define SFLASH_SWD_LISTEN                                   (*(volatile uint32_t *)(0x0ffff158))
#define SFLASH_SWD_LISTEN_DEFAULT                           (0x00000000)

/*
 * Number of clock cycles
 */
#define SFLASH_SWD_LISTEN_CYCLES_MASK                       (0xffffffff) /* <0:31> :RW:X: */
#define SFLASH_SWD_LISTEN_CYCLES_POS                        (0)


/*
 * Flash Image Start Address
 * Flash Image start address.  If this field is FFFF_FFFF (recommended) the
 * firmware start address is loaded from vector table locations 0,1 (0000_0000
 * - 0000_0007).  If this field is not FFFF_FFFF it is assumed to be a valid
 * firmware start address.
 */
#define SFLASH_FLASH_START_ADDRESS                          (0x0ffff15c)
#define SFLASH_FLASH_START                                  (*(volatile uint32_t *)(0x0ffff15c))
#define SFLASH_FLASH_START_DEFAULT                          (0x00000000)

/*
 * Start Address
 */
#define SFLASH_FLASH_START_ADDRESS_MASK                     (0xffffffff) /* <0:31> :RW:X: */
#define SFLASH_FLASH_START_ADDRESS_POS                      (0)


/*
 * Checksum Skip Option Register
 * Directs BootROM to perform CHECKSUM check on SR layout during boot or
 * not.
 */
#define SFLASH_SKIP_CHECKSUM_ADDRESS                        (0x0ffff169)
#define SFLASH_SKIP_CHECKSUM                                (*(volatile uint8_t *)(0x0ffff169))
#define SFLASH_SKIP_CHECKSUM_DEFAULT                        (0x00000000)

/*
 * 0: Perform checksum check (see CHECKSUM fueld below)
 * 1: Skip checksum check
 * >1: Undefined - do not use
 */
#define SFLASH_SKIP_CHECKSUM_SKIP_MASK                      (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_SKIP_CHECKSUM_SKIP_POS                       (0)


/*
 * SRSSLT BG Vref trim used during boot
 * This field contains SRSS-Lite Vref trim that is required for flash reads
 * to work.  It is retrieved using trial and error as documented in SAS Part-V,
 * Sec 3.4.  Note: this field exists only in products that use SRSS-Lite
 * (s8srsslt).
 */
#define SFLASH_INITIAL_PWR_BG_TRIM1_ADDRESS                 (0x0ffff16a)
#define SFLASH_INITIAL_PWR_BG_TRIM1                         (*(volatile uint8_t *)(0x0ffff16a))
#define SFLASH_INITIAL_PWR_BG_TRIM1_DEFAULT                 (0x00000000)

/*
 * See PWR_BG_TRIM2 in SRSSLT
 */
#define SFLASH_INITIAL_PWR_BG_TRIM1_REF_ITRIM_MASK          (0x0000003f) /* <0:5> R:RW:0: */
#define SFLASH_INITIAL_PWR_BG_TRIM1_REF_ITRIM_POS           (0)


/*
 * SRSSLT BG Vref trim used during boot
 * This field contains the complement of the PWR_BG_TRIM1 value above.  It
 * is used to verify correct readout in the trial and error process as documented
 * in SAS Part-V, Sec 3.4.  Note: this field exists only in products that
 * use SRSS-Lite (s8srsslt).
 */
#define SFLASH_INITIAL_PWR_BG_TRIM1_INV_ADDRESS             (0x0ffff16b)
#define SFLASH_INITIAL_PWR_BG_TRIM1_INV                     (*(volatile uint8_t *)(0x0ffff16b))
#define SFLASH_INITIAL_PWR_BG_TRIM1_INV_DEFAULT             (0x00000000)

/*
 * See PWR_BG_TRIM2 in SRSSLT
 */
#define SFLASH_INITIAL_PWR_BG_TRIM1_INV_REF_ITRIM_MASK      (0x0000003f) /* <0:5> R:RW:0: */
#define SFLASH_INITIAL_PWR_BG_TRIM1_INV_REF_ITRIM_POS       (0)


/*
 * SRSSLT BG Iref trim used during boot
 * This field contains SRSS-Lite Iref trim that is required for flash reads
 * to work.  It is retrieved using trial and error as documented in SAS Part-V,
 * Sec 3.4.  Note: this field exists only in products that use SRSS-Lite
 * (s8srsslt).
 */
#define SFLASH_INITIAL_PWR_BG_TRIM2_ADDRESS                 (0x0ffff16c)
#define SFLASH_INITIAL_PWR_BG_TRIM2                         (*(volatile uint8_t *)(0x0ffff16c))
#define SFLASH_INITIAL_PWR_BG_TRIM2_DEFAULT                 (0x00000000)

/*
 * See PWR_BG_TRIM2 in SRSSLT
 */
#define SFLASH_INITIAL_PWR_BG_TRIM2_REF_ITRIM_MASK          (0x0000003f) /* <0:5> R:RW:0: */
#define SFLASH_INITIAL_PWR_BG_TRIM2_REF_ITRIM_POS           (0)


/*
 * SRSSLT BG Iref trim used during boot
 * This field contains the complement of the PWR_BG_TRIM2 value above.  It
 * is used to verify correct readout in the trial and error process as documented
 * in SAS Part-V, Sec 3.4.  Note: this field exists only in products that
 * use SRSS-Lite (s8srsslt).
 */
#define SFLASH_INITIAL_PWR_BG_TRIM2_INV_ADDRESS             (0x0ffff16d)
#define SFLASH_INITIAL_PWR_BG_TRIM2_INV                     (*(volatile uint8_t *)(0x0ffff16d))
#define SFLASH_INITIAL_PWR_BG_TRIM2_INV_DEFAULT             (0x00000000)

/*
 * See PWR_BG_TRIM2 in SRSSLT
 */
#define SFLASH_INITIAL_PWR_BG_TRIM2_INV_REF_ITRIM_MASK      (0x0000003f) /* <0:5> R:RW:0: */
#define SFLASH_INITIAL_PWR_BG_TRIM2_INV_REF_ITRIM_POS       (0)


/*
 * FLASH IDAC trim used during boot
 * This field contains FLASH IDAC trim that is required for flash reads to
 * work.  It is retrieved using trial and error as documented in SAS Part-V,
 * Sec 3.4.  Note: this field exists only in products that use SRSS-Lite
 * (s8srsslt).
 */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_ADDRESS           (0x0ffff16e)
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0                   (*(volatile uint8_t *)(0x0ffff16e))
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_DEFAULT           (0x00000000)

/*
 * See SPCIF_TRIM1
 */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_IDAC_MASK         (0x0000001f) /* <0:4> R:RW:0: */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_IDAC_POS          (0)


/*
 * See SPCIF_TRIM1
 */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_SLOPE_MASK        (0x000000e0) /* <5:7> R:RW:0: */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_SLOPE_POS         (5)


/*
 * FLASH IDAC trim used during boot
 * This field contains the complement of the SPCIF_TRIM_M0_DAC0 value above.
 *  It is used to verify correct readout in the trial and error process as
 * documented in SAS Part-V, Sec 3.4.  Note: this field exists only in products
 * that use SRSS-Lite (s8srsslt).
 */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_INV_ADDRESS       (0x0ffff16f)
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_INV               (*(volatile uint8_t *)(0x0ffff16f))
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_INV_DEFAULT       (0x00000000)

/*
 * See SPCIF_TRIM1
 */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_INV_IDAC_MASK     (0x0000001f) /* <0:4> R:RW:0: */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_INV_IDAC_POS      (0)


/*
 * See SPCIF_TRIM1
 */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_INV_SLOPE_MASK    (0x000000e0) /* <5:7> R:RW:0: */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_INV_SLOPE_POS     (5)


/*
 * Virgin Protection Mode Key
 * Magic 64b Key that indicates part is no longer in Virgin State.  Key is
 * programmed to fixed value of {0xDE, 0xAD, 0xBE, 0xEF, 0x19, 0x67, 0xFA,
 * 0xDE} during manufacturing to indicate PROT_PROTECTION is valid.
 */
#define SFLASH_PROT_VIRGINKEY_ADDRESS(n)                    (0x0ffff170 + ((n) * (0x0001)))
#define SFLASH_PROT_VIRGINKEY(n)                            (*(volatile uint8_t *)(0x0ffff170 + ((n) * 0x0001)))
#define SFLASH_PROT_VIRGINKEY_DEFAULT                       (0x00000000)

/*
 * Key Byte
 */
#define SFLASH_PROT_VIRGINKEY_KEY8_MASK                     (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_PROT_VIRGINKEY_KEY8_POS                      (0)


/*
 * Lot Number (3 bytes)
 * Manufacturing Data, can be used to form Unique DieID.
 */
#define SFLASH_DIE_LOT_ADDRESS(n)                           (0x0ffff178 + ((n) * (0x0001)))
#define SFLASH_DIE_LOT(n)                                   (*(volatile uint8_t *)(0x0ffff178 + ((n) * 0x0001)))
#define SFLASH_DIE_LOT_DEFAULT                              (0x00000000)

/*
 * Lot Number Byte
 */
#define SFLASH_DIE_LOT_LOT_MASK                             (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_DIE_LOT_LOT_POS                              (0)


/*
 * Wafer Number
 * Manufacturing Data, can be used to form Unique DieID
 */
#define SFLASH_DIE_WAFER_ADDRESS                            (0x0ffff17b)
#define SFLASH_DIE_WAFER                                    (*(volatile uint8_t *)(0x0ffff17b))
#define SFLASH_DIE_WAFER_DEFAULT                            (0x00000000)

/*
 * Wafer Number
 */
#define SFLASH_DIE_WAFER_WAFER_MASK                         (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_DIE_WAFER_WAFER_POS                          (0)


/*
 * X Position on Wafer, CRI Pass/Fail Bin
 * Manufacturing Data, can be used to form Unique DieID
 */
#define SFLASH_DIE_X_ADDRESS                                (0x0ffff17c)
#define SFLASH_DIE_X                                        (*(volatile uint8_t *)(0x0ffff17c))
#define SFLASH_DIE_X_DEFAULT                                (0x00000000)

/*
 * X Position
 */
#define SFLASH_DIE_X_X_MASK                                 (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_DIE_X_X_POS                                  (0)


/*
 * Y Position on Wafer, CHI Pass/Fail Bin
 * Manufacturing Data, can be used to form Unique DieID
 */
#define SFLASH_DIE_Y_ADDRESS                                (0x0ffff17d)
#define SFLASH_DIE_Y                                        (*(volatile uint8_t *)(0x0ffff17d))
#define SFLASH_DIE_Y_DEFAULT                                (0x00000000)

/*
 * Y Position
 */
#define SFLASH_DIE_Y_Y_MASK                                 (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_DIE_Y_Y_POS                                  (0)


/*
 * Sort1/2/3 Pass/Fail Bin
 * Manufacturing Data, can be used to form Unique DieID
 */
#define SFLASH_DIE_SORT_ADDRESS                             (0x0ffff17e)
#define SFLASH_DIE_SORT                                     (*(volatile uint8_t *)(0x0ffff17e))
#define SFLASH_DIE_SORT_DEFAULT                             (0x00000000)

/*
 * SORT1 Pass Bin (1) or 0 (Fail Bin)
 */
#define SFLASH_DIE_SORT_S1_PASS                             (1u << 0) /* <0:0> :RW:X: */


/*
 * SORT2 Pass Bin (1) or 0 (Fail Bin)
 */
#define SFLASH_DIE_SORT_S2_PASS                             (1u << 1) /* <1:1> :RW:X: */


/*
 * SORT3 Pass Bin (1) or 0 (Fail Bin)
 */
#define SFLASH_DIE_SORT_S3_PASS                             (1u << 2) /* <2:2> :RW:X: */


/*
 * CRI Pass Bin (1) or 0 (Fail Bin)
 */
#define SFLASH_DIE_SORT_CRI_PASS                            (1u << 3) /* <3:3> :RW:X: */


/*
 * CHI Pass Bin (1) or 0 (Fail Bin)
 */
#define SFLASH_DIE_SORT_CHI_PASS                            (1u << 4) /* <4:4> :RW:X: */


/*
 * ENG Pass Bin
 */
#define SFLASH_DIE_SORT_ENG_PASS                            (1u << 5) /* <5:5> :RW:X: */


/*
 * Minor Revision Number
 * Manufacturing Data, can be used to form Unique DieID
 */
#define SFLASH_DIE_MINOR_ADDRESS                            (0x0ffff17f)
#define SFLASH_DIE_MINOR                                    (*(volatile uint8_t *)(0x0ffff17f))
#define SFLASH_DIE_MINOR_DEFAULT                            (0x00000000)

/*
 * Minor revision number
 */
#define SFLASH_DIE_MINOR_MINOR_MASK                         (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_DIE_MINOR_MINOR_POS                          (0)


/*
 * PE/TE Data
 * PE/TE Specific Data
 */
#define SFLASH_PE_TE_DATA_ADDRESS(n)                        (0x0ffff180 + ((n) * (0x0001)))
#define SFLASH_PE_TE_DATA(n)                                (*(volatile uint8_t *)(0x0ffff180 + ((n) * 0x0001)))
#define SFLASH_PE_TE_DATA_DEFAULT                           (0x00000000)

/*
 * PE/TE Data
 */
#define SFLASH_PE_TE_DATA_DATA8_MASK                        (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_PE_TE_DATA_DATA8_POS                         (0)


/*
 * Preprogram Settings
 * Contains pulse width (SPCIF_TIMER) and DAC (SPCIF_PNDAC) settings.
 */
#define SFLASH_PP_ADDRESS                                   (0x0ffff1a0)
#define SFLASH_PP                                           (*(volatile uint32_t *)(0x0ffff1a0))
#define SFLASH_PP_DEFAULT                                   (0x00000000)

/*
 * Period of timer in clk_spcif_timer ticks.  For regular FLASH, clock is
 * 36MHz from dedicated oscillator. For FLASH-Lite, clock is same as clk_hf,
 * which must be set to 48MHz for 48MHz devices and 12MHz for max 16MHz devices
 */
#define SFLASH_PP_PERIOD_MASK                               (0x00ffffff) /* <0:23> :RW:X: */
#define SFLASH_PP_PERIOD_POS                                (0)


/*
 * PDAC input. Each increment in PDAC causes an increase of ~0.10V in VPOS
 */
#define SFLASH_PP_PDAC_MASK                                 (0x0f000000) /* <24:27> :RW:X: */
#define SFLASH_PP_PDAC_POS                                  (24)


/*
 * NDAC input. Each increment in NDAC causes an increase of ~0.10V in VNEG
 */
#define SFLASH_PP_NDAC_MASK                                 (0xf0000000) /* <28:31> :RW:X: */
#define SFLASH_PP_NDAC_POS                                  (28)


/*
 * Erase Settings
 * Contains pulse width (SPCIF_TIMER) and DAC (SPCIF_PNDAC) settings.
 */
#define SFLASH_E_ADDRESS                                    (0x0ffff1a4)
#define SFLASH_E                                            (*(volatile uint32_t *)(0x0ffff1a4))
#define SFLASH_E_DEFAULT                                    (0x00000000)

/*
 * Period of timer in clk_spcif_timer ticks.  For regular FLASH, clock is
 * 36MHz from dedicated oscillator. For FLASH-Lite, clock is same as clk_hf,
 * which must be set to 48MHz for 48MHz devices and 12MHz for max 16MHz devices
 */
#define SFLASH_E_PERIOD_MASK                                (0x00ffffff) /* <0:23> :RW:X: */
#define SFLASH_E_PERIOD_POS                                 (0)


/*
 * PDAC input. Each increment in PDAC causes an increase of ~0.10V in VPOS
 */
#define SFLASH_E_PDAC_MASK                                  (0x0f000000) /* <24:27> :RW:X: */
#define SFLASH_E_PDAC_POS                                   (24)


/*
 * NDAC input. Each increment in NDAC causes an increase of ~0.10V in VNEG
 */
#define SFLASH_E_NDAC_MASK                                  (0xf0000000) /* <28:31> :RW:X: */
#define SFLASH_E_NDAC_POS                                   (28)


/*
 * Program Settings
 * Contains pulse width (SPCIF_TIMER) and DAC (SPCIF_PNDAC) settings.
 */
#define SFLASH_P_ADDRESS                                    (0x0ffff1a8)
#define SFLASH_P                                            (*(volatile uint32_t *)(0x0ffff1a8))
#define SFLASH_P_DEFAULT                                    (0x00000000)

/*
 * Period of timer in clk_spcif_timer ticks.  For regular FLASH, clock is
 * 36MHz from dedicated oscillator. For FLASH-Lite, clock is same as clk_hf,
 * which must be set to 48MHz for 48MHz devices and 12MHz for max 16MHz devices
 */
#define SFLASH_P_PERIOD_MASK                                (0x00ffffff) /* <0:23> :RW:X: */
#define SFLASH_P_PERIOD_POS                                 (0)


/*
 * PDAC input. Each increment in PDAC causes an increase of ~0.10V in VPOS
 */
#define SFLASH_P_PDAC_MASK                                  (0x0f000000) /* <24:27> :RW:X: */
#define SFLASH_P_PDAC_POS                                   (24)


/*
 * NDAC input. Each increment in NDAC causes an increase of ~0.10V in VNEG
 */
#define SFLASH_P_NDAC_MASK                                  (0xf0000000) /* <28:31> :RW:X: */
#define SFLASH_P_NDAC_POS                                   (28)


/*
 * Erase All - Erase Settings
 * Contains pulse width (SPCIF_TIMER) and DAC (SPCIF_PNDAC) settings.
 */
#define SFLASH_EA_E_ADDRESS                                 (0x0ffff1ac)
#define SFLASH_EA_E                                         (*(volatile uint32_t *)(0x0ffff1ac))
#define SFLASH_EA_E_DEFAULT                                 (0x00000000)

/*
 * Period of timer in clk_spcif_timer ticks.  For regular FLASH, clock is
 * 36MHz from dedicated oscillator. For FLASH-Lite, clock is same as clk_hf,
 * which must be set to 48MHz for 48MHz devices and 12MHz for max 16MHz devices
 */
#define SFLASH_EA_E_PERIOD_MASK                             (0x00ffffff) /* <0:23> :RW:X: */
#define SFLASH_EA_E_PERIOD_POS                              (0)


/*
 * PDAC input. Each increment in PDAC causes an increase of ~0.10V in VPOS
 */
#define SFLASH_EA_E_PDAC_MASK                               (0x0f000000) /* <24:27> :RW:X: */
#define SFLASH_EA_E_PDAC_POS                                (24)


/*
 * NDAC input. Each increment in NDAC causes an increase of ~0.10V in VNEG
 */
#define SFLASH_EA_E_NDAC_MASK                               (0xf0000000) /* <28:31> :RW:X: */
#define SFLASH_EA_E_NDAC_POS                                (28)


/*
 * Erase All - Program Settings
 * Contains pulse width (SPCIF_TIMER) and DAC (SPCIF_PNDAC) settings.
 */
#define SFLASH_EA_P_ADDRESS                                 (0x0ffff1b0)
#define SFLASH_EA_P                                         (*(volatile uint32_t *)(0x0ffff1b0))
#define SFLASH_EA_P_DEFAULT                                 (0x00000000)

/*
 * Period of timer in clk_spcif_timer ticks.  For regular FLASH, clock is
 * 36MHz from dedicated oscillator. For FLASH-Lite, clock is same as clk_hf,
 * which must be set to 48MHz for 48MHz devices and 12MHz for max 16MHz devices
 */
#define SFLASH_EA_P_PERIOD_MASK                             (0x00ffffff) /* <0:23> :RW:X: */
#define SFLASH_EA_P_PERIOD_POS                              (0)


/*
 * PDAC input. Each increment in PDAC causes an increase of ~0.10V in VPOS
 */
#define SFLASH_EA_P_PDAC_MASK                               (0x0f000000) /* <24:27> :RW:X: */
#define SFLASH_EA_P_PDAC_POS                                (24)


/*
 * NDAC input. Each increment in NDAC causes an increase of ~0.10V in VNEG
 */
#define SFLASH_EA_P_NDAC_MASK                               (0xf0000000) /* <28:31> :RW:X: */
#define SFLASH_EA_P_NDAC_POS                                (28)


/*
 * Erase Sector - Erase Settings
 * Contains pulse width (SPCIF_TIMER) and DAC (SPCIF_PNDAC) settings.
 */
#define SFLASH_ES_E_ADDRESS                                 (0x0ffff1b4)
#define SFLASH_ES_E                                         (*(volatile uint32_t *)(0x0ffff1b4))
#define SFLASH_ES_E_DEFAULT                                 (0x00000000)

/*
 * Period of timer in clk_spcif_timer ticks.  For regular FLASH, clock is
 * 36MHz from dedicated oscillator. For FLASH-Lite, clock is same as clk_hf,
 * which must be set to 48MHz for 48MHz devices and 12MHz for max 16MHz devices
 */
#define SFLASH_ES_E_PERIOD_MASK                             (0x00ffffff) /* <0:23> :RW:X: */
#define SFLASH_ES_E_PERIOD_POS                              (0)


/*
 * PDAC input. Each increment in PDAC causes an increase of ~0.10V in VPOS
 */
#define SFLASH_ES_E_PDAC_MASK                               (0x0f000000) /* <24:27> :RW:X: */
#define SFLASH_ES_E_PDAC_POS                                (24)


/*
 * NDAC input. Each increment in NDAC causes an increase of ~0.10V in VNEG
 */
#define SFLASH_ES_E_NDAC_MASK                               (0xf0000000) /* <28:31> :RW:X: */
#define SFLASH_ES_E_NDAC_POS                                (28)


/*
 * Erase Sector - Program EO Settings
 * Contains pulse width (SPCIF_TIMER) and DAC (SPCIF_PNDAC) settings.
 */
#define SFLASH_ES_P_EO_ADDRESS                              (0x0ffff1b8)
#define SFLASH_ES_P_EO                                      (*(volatile uint32_t *)(0x0ffff1b8))
#define SFLASH_ES_P_EO_DEFAULT                              (0x00000000)

/*
 * Period of timer in clk_spcif_timer ticks.  For regular FLASH, clock is
 * 36MHz from dedicated oscillator. For FLASH-Lite, clock is same as clk_hf,
 * which must be set to 48MHz for 48MHz devices and 12MHz for max 16MHz devices
 */
#define SFLASH_ES_P_EO_PERIOD_MASK                          (0x00ffffff) /* <0:23> :RW:X: */
#define SFLASH_ES_P_EO_PERIOD_POS                           (0)


/*
 * PDAC input. Each increment in PDAC causes an increase of ~0.10V in VPOS
 */
#define SFLASH_ES_P_EO_PDAC_MASK                            (0x0f000000) /* <24:27> :RW:X: */
#define SFLASH_ES_P_EO_PDAC_POS                             (24)


/*
 * NDAC input. Each increment in NDAC causes an increase of ~0.10V in VNEG
 */
#define SFLASH_ES_P_EO_NDAC_MASK                            (0xf0000000) /* <28:31> :RW:X: */
#define SFLASH_ES_P_EO_NDAC_POS                             (28)


/*
 * USB IMO TRIM 24MHz
 * VCTAT Slope for Programming Operations
 */
#define SFLASH_IMO_TRIM_USBMODE_24_ADDRESS                  (0x0ffff1be)
#define SFLASH_IMO_TRIM_USBMODE_24                          (*(volatile uint8_t *)(0x0ffff1be))
#define SFLASH_IMO_TRIM_USBMODE_24_DEFAULT                  (0x00000000)

/*
 * TRIM value for IMO with USB at 24MHz
 */
#define SFLASH_IMO_TRIM_USBMODE_24_TRIM_24_MASK             (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_IMO_TRIM_USBMODE_24_TRIM_24_POS              (0)


/*
 * USB IMO TRIM 48MHz
 * VCTAT Slope for Programming Operations
 */
#define SFLASH_IMO_TRIM_USBMODE_48_ADDRESS                  (0x0ffff1bf)
#define SFLASH_IMO_TRIM_USBMODE_48                          (*(volatile uint8_t *)(0x0ffff1bf))
#define SFLASH_IMO_TRIM_USBMODE_48_DEFAULT                  (0x00000000)

/*
 * TRIM value for IMO with USB at 24MHz
 */
#define SFLASH_IMO_TRIM_USBMODE_48_TRIM_24_MASK             (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_IMO_TRIM_USBMODE_48_TRIM_24_POS              (0)


/*
 * IMO TempCo Trim Register (SRSS-Lite)
 * CLK_IMO_TRIM3 for each integer frequency 24..48MHz
 * IMO_TCTRIM_LT[0]: 24 MHz
 * IMO_TCTRIM_LT[1]: 25 MHz
 * etc.
 */
#define SFLASH_IMO_TCTRIM_LT_ADDRESS(n)                     (0x0ffff1cc + ((n) * (0x0001)))
#define SFLASH_IMO_TCTRIM_LT(n)                             (*(volatile uint8_t *)(0x0ffff1cc + ((n) * 0x0001)))
#define SFLASH_IMO_TCTRIM_LT_DEFAULT                        (0x00000050)

/*
 * IMO trim stepsize bits.  These bits are determined at manufacturing time
 * to adjust for process variation.  They are used to tune the stepsize of
 * the FSOFFSET and OFFSET trims.
 */
#define SFLASH_IMO_TCTRIM_LT_STEPSIZE_MASK                  (0x0000001f) /* <0:4> R:RW:16: */
#define SFLASH_IMO_TCTRIM_LT_STEPSIZE_POS                   (0)


/*
 * IMO temperature compesation trim.  These bits are determined at manufacturing
 * time to adjust for temperature dependence. This bits are dependent on
 * frequency and need to be changed using the Cypress provided frequency
 * change algorithm.
 */
#define SFLASH_IMO_TCTRIM_LT_TCTRIM_MASK                    (0x00000060) /* <5:6> R:RW:2: */
#define SFLASH_IMO_TCTRIM_LT_TCTRIM_POS                     (5)


/*
 * IMO Frequency Trim Register (SRSS-Lite)
 * CLK_IMO_TRIM1 for each integer frequency 24..48MHz
 * IMO_TRIM_LT[0]: 24 MHz
 * IMO_TRIM_LT[1]: 25 MHz
 * etc.
 */
#define SFLASH_IMO_TRIM_LT_ADDRESS(n)                       (0x0ffff1e5 + ((n) * (0x0001)))
#define SFLASH_IMO_TRIM_LT(n)                               (*(volatile uint8_t *)(0x0ffff1e5 + ((n) * 0x0001)))
#define SFLASH_IMO_TRIM_LT_DEFAULT                          (0x00000000)

/*
 * Frequency trim bits.  These bits are determined at manufacturing time
 * for each FREQ setting (IMO_TRIM2) and stored in SFLASH.
 */
#define SFLASH_IMO_TRIM_LT_OFFSET_MASK                      (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_IMO_TRIM_LT_OFFSET_POS                       (0)


/*
 * Boot Checksum
 */
#define SFLASH_CHECKSUM_ADDRESS                             (0x0ffff1fe)
#define SFLASH_CHECKSUM                                     (*(volatile uint16_t *)(0x0ffff1fe))
#define SFLASH_CHECKSUM_DEFAULT                             (0x00000000)

/*
 * Checksum of fixed data checked during boot.  This checksum covers all
 * of rows 1,2,3 of macro 0 + row 3 of macro 1 (except this checksum, and
 * row 3 of macro 1 only if it exists).
 */
#define SFLASH_CHECKSUM_CHECKSUM_MASK                       (0x0000ffff) /* <0:15> :RW:X: */
#define SFLASH_CHECKSUM_CHECKSUM_POS                        (0)


/*
 * Divider command register
 * The (PA_SEL_TYPE, PA_SEL_DIV) field pair allows a divider to be phase
 * aligned with another divider. E.g., consider a 48 MHz "clk_hf", and a
 * need for a 12 MHz divided clock A and a 8 MHz divided clock B. Clock 
 * A uses 8.0 integer divider 0 and is created by aligning it to "clk_hf"
 * ((PA_SEL_TYPE, PA_SEL_DIV) is (3, 63)) and DIV_8_CTL0.INT8_DIV is "4-1".
 * Clock  B uses 8.0 integer divider 1 and is created by aligning it to clock
 * A ((PA_SEL_TYPE, PA_SEL_DIV) is (0, 0)) and DIV_8_CTL1.INT8_DIV is "6-1".
 * This guarantees that clock B is phase aligned with clock B: as the smallest
 * common multiple of the two clock periods is 12 "clk_hf" cycles, the clocks
 * A and B will be aligned every 12 "clk_hf" cycles. Note: clock B is phase
 * aligned to clock A, but still uses "clk_hf" as a reference clock for its
 * divider value.
 */
#define PERI_DIV_CMD_ADDRESS                                (0x40010000)
#define PERI_DIV_CMD                                        (*(volatile uint32_t *)(0x40010000))
#define PERI_DIV_CMD_DEFAULT                                (0x0000ffff)

/*
 * (SEL_TYPE, SEL_DIV) specifies the divider on which the command (DISABLE/ENABLE)
 * is performed.
 *
 * If SEL_DIV is "63" and "SEL_TYPE" is "3" (default/reset value), no divider
 * is specified and no clock signal(s) are generated.
 */
#define PERI_DIV_CMD_SEL_DIV_MASK                           (0x0000003f) /* <0:5> R:RW:63: */
#define PERI_DIV_CMD_SEL_DIV_POS                            (0)


/*
 * Specifies the divider type of the divider on which the command is performed:
 * 0: 8.0 (integer) clock dividers.
 * 1: 16.0 (integer) clock dividers.
 * 2: 16.5 (fractional) clock dividers.
 * 3: 24.5 (fractional) clock dividers.
 */
#define PERI_DIV_CMD_SEL_TYPE_MASK                          (0x000000c0) /* <6:7> R:RW:3: */
#define PERI_DIV_CMD_SEL_TYPE_POS                           (6)


/*
 * (PA_SEL_TYPE, PA_SEL_DIV) pecifies the divider to which phase alignment
 * is performed for the clock enable command. Any enabled divider can be
 * used as reference. This allows all dividers to be aligned with each other,
 * even when they are enabled at different times.
 *
 * If PA_SEL_DIV is "63" and "PA_SEL_TYPE" is "3", "clk_hf" is used as reference.
 */
#define PERI_DIV_CMD_PA_SEL_DIV_MASK                        (0x00003f00) /* <8:13> R:RW:63: */
#define PERI_DIV_CMD_PA_SEL_DIV_POS                         (8)


/*
 * Specifies the divider type of the divider to which phase alignment is
 * performed for the clock enable command:
 * 0: 8.0 (integer) clock dividers.
 * 1: 16.0 (integer) clock dividers.
 * 2: 16.5 (fractional) clock dividers.
 * 3: 24.5 (fractional) clock dividers.
 */
#define PERI_DIV_CMD_PA_SEL_TYPE_MASK                       (0x0000c000) /* <14:15> R:RW:3: */
#define PERI_DIV_CMD_PA_SEL_TYPE_POS                        (14)


/*
 * Clock divider disable command (mutually exlusive with ENABLE). SW sets
 * this field to '1' and HW sets this field to '0'.
 *
 * The SEL_DIV and SEL_TYPE fields specify which divider is to be disabled.
 *
 * The HW sets the DISABLE field to '0' immediately and the HW sets the DIV_XXX_CTL.EN
 * field of the divider to '0' immediately.
 */
#define PERI_DIV_CMD_DISABLE                                (1u << 30) /* <30:30> RW1C:RW:0: */


/*
 * Clock divider enable command (mutually exclusive with DISABLE). Typically,
 * SW sets this field to '1' to enable a divider and HW sets this field to
 * '0' to indicate that divider enabling has completed. When a divider is
 * enabled, its integer and fractional (if present) counters are initialized
 * to "0". If a divider is to be re-enabled using different integer and fractional
 * divider values, the SW should follow these steps:
 * 0: Disable the divider using the DIV_CMD.DISABLE field.
 * 1: Configure the divider's DIV_XXX_CTL register.
 * 2: Enable the divider using the DIV_CMD_ENABLE field.
 *
 * The SEL_DIV and SEL_TYPE fields specify which divider is to be enabled.
 * The enabled divider may be phase aligned to either "clk_hf" (typical usage)
 * or to ANY enabled divider.
 *
 * The PA_SEL_DIV and P_SEL_TYPE fields specify the reference divider.
 *
 * The HW sets the ENABLE field to '0' when the enabling is performed and
 * the HW set the DIV_XXX_CTL.EN field of the divider to '1' when the enabling
 * is performed. Note that enabling with phase alignment to a low frequency
 * divider takes time. E.g. To align to a divider that generates a clock
 * of "clk_hf"/n (with n being the integer divider value INT_DIV+1), up to
 * n cycles may be required to perform alignment. Phase alignment to "clk_hf"
 * takes affect immediately. SW can set this field to '0' during phase alignment
 * to abort the enabling process.
 */
#define PERI_DIV_CMD_ENABLE                                 (1u << 31) /* <31:31> RW1C:RW:0: */


/*
 * Programmable clock control register
 */
#define PERI_PCLK_CTL_ADDRESS(n)                            (0x40010100 + ((n) * (0x0004)))
#define PERI_PCLK_CTL(n)                                    (*(volatile uint32_t *)(0x40010100 + ((n) * 0x0004)))
#define PERI_PCLK_CTL_DEFAULT                               (0x000000c7)

/*
 * Specifies one of the dividers of the divider type specified by SEL_TYPE.
 *
 * If SEL_DIV is "63" and "SEL_TYPE" is "3" (default/reset value), no divider
 * is specified and no clock control signal(s) are generated.
 *
 * When transitioning a clock between two out of phase dividers, spurious
 * clock control signals may be generated for one “clk_hf” cycle during this
 * transition. These clock control signals may cause a single clock period
 * that is smaller than any of the two divider periods. To prevent these
 * spurious clock signals, the clock multiplexer can be disconnected (SEL_DIV
 * is "63" and "SEL_TYPE" is "3") for a transition time that is larger than
 * the smaller of the two divider periods.
 */
#define PERI_PCLK_CTL_SEL_DIV_MASK                          (0x00000007) /* <0:2> R:RW:7:PCLK_DIV_ADDR_WIDTH */
#define PERI_PCLK_CTL_SEL_DIV_POS                           (0)


/*
 * Specifies divider type:
 * 0: 8.0 (integer) clock dividers.
 * 1: 16.0 (integer) clock dividers.
 * 2: 16.5 (fractional) clock dividers.
 * 3: 24.5 (fractional) clock dividers.
 */
#define PERI_PCLK_CTL_SEL_TYPE_MASK                         (0x000000c0) /* <6:7> R:RW:3: */
#define PERI_PCLK_CTL_SEL_TYPE_POS                          (6)


/*
 * Divider control register (for 8.0 divider)
 * Smallest of the divider types.
 */
#define PERI_DIV_8_CTL_ADDRESS(n)                           (0x40010200 + ((n) * (0x0004)))
#define PERI_DIV_8_CTL(n)                                   (*(volatile uint32_t *)(0x40010200 + ((n) * 0x0004)))
#define PERI_DIV_8_CTL_DEFAULT                              (0x00000000)

/*
 * Divider enabled. HW sets this field to '1' as a result of an ENABLE command.
 * HW sets this field to '0' as a result on a DISABLE command.
 *
 * Note that this field is retained. As a result, the divider does NOT have
 * to be re-enabled after transitioning from DeepSleep to Active power mode.
 */
#define PERI_DIV_8_CTL_EN                                   (1u << 0) /* <0:0> RW:R:0: */


/*
 * Integer division by (1+INT8_DIV). Allows for integer divisions in the
 * range [1, 256]. Note: this type of divider does NOT allow for a fractional
 * division.
 *
 * For the generation of a divided clock, the integer division range is restricted
 * to [2, 256].
 *
 * For the generation of a 50/50% duty cycle digital divided clock, the integer
 * division range is resticited to even numbers in the range [2, 256]. The
 * generation of a 50/50 % duty cycle analog divided clock has no restrictions.
 *
 * Note that this field is retained. However, the counter that is used to
 * implement the division is not and will be initialized by HW to "0" when
 * transitioning from DeepSleep to Active power mode.
 */
#define PERI_DIV_8_CTL_INT8_DIV_MASK                        (0x0000ff00) /* <8:15> R:RW:0: */
#define PERI_DIV_8_CTL_INT8_DIV_POS                         (8)


/*
 * Divider control register (for 16.0 divider)
 */
#define PERI_DIV_16_CTL_ADDRESS(n)                          (0x40010300 + ((n) * (0x0004)))
#define PERI_DIV_16_CTL(n)                                  (*(volatile uint32_t *)(0x40010300 + ((n) * 0x0004)))
#define PERI_DIV_16_CTL_DEFAULT                             (0x00000000)

/*
 * Divider enabled. HW sets this field to '1' as a result of an ENABLE command.
 * HW sets this field to '0' as a result on a DISABLE command.
 *
 * Note that this field is retained. As a result, the divider does NOT have
 * to be re-enabled after transitioning from DeepSleep to Active power mode.
 */
#define PERI_DIV_16_CTL_EN                                  (1u << 0) /* <0:0> RW:R:0: */


/*
 * Integer division by (1+INT16_DIV). Allows for integer divisions in the
 * range [1, 65,536]. Note: this type of divider does NOT allow for a fractional
 * division.
 *
 * For the generation of a divided clock, the integer division range is restricted
 * to [2, 65,536].
 *
 * For the generation of a 50/50% duty cycle digital divided clock, the integer
 * division range is restricted to even numbers in the range [2, 65,536].
 * The generation of a 50/50 % duty cycle analog divided clock has no restrictions.
 *
 * Note that this field is retained. However, the counter that is used to
 * implement the division is not and will be initialized by HW to "0" when
 * transitioning from DeepSleep to Active power mode.
 */
#define PERI_DIV_16_CTL_INT16_DIV_MASK                      (0x00ffff00) /* <8:23> R:RW:0: */
#define PERI_DIV_16_CTL_INT16_DIV_POS                       (8)


/*
 * Port selection register
 * Note: these fields are HW:W purely for DfT observe functionality, not
 * for functional reasons.
 */
#define HSIOM_PORT_SEL_ADDRESS                              (0x40020000)
#define HSIOM_PORT_SEL                                      (*(volatile uint32_t *)(0x40020000))
#define HSIOM_PORT_SEL_DEFAULT                              (0x00000000)

/*
 * Selects connection for IO pad 0 route.
 */
#define HSIOM_PORT_SEL_IO0_SEL_MASK                         (0x0000000f) /* <0:3> RW:RW:0:IO0 */
#define HSIOM_PORT_SEL_IO0_SEL_POS                          (0)


/*
 * Selects connection for IO pad 1 route.
 */
#define HSIOM_PORT_SEL_IO1_SEL_MASK                         (0x000000f0) /* <4:7> RW:RW:0:IO1 */
#define HSIOM_PORT_SEL_IO1_SEL_POS                          (4)


/*
 * Selects connection for IO pad 2 route.
 */
#define HSIOM_PORT_SEL_IO2_SEL_MASK                         (0x00000f00) /* <8:11> RW:RW:0:IO2 */
#define HSIOM_PORT_SEL_IO2_SEL_POS                          (8)


/*
 * Selects connection for IO pad 3 route.
 */
#define HSIOM_PORT_SEL_IO3_SEL_MASK                         (0x0000f000) /* <12:15> RW:RW:0:IO3 */
#define HSIOM_PORT_SEL_IO3_SEL_POS                          (12)


/*
 * Selects connection for IO pad 4 route.
 */
#define HSIOM_PORT_SEL_IO4_SEL_MASK                         (0x000f0000) /* <16:19> RW:RW:0:IO4 */
#define HSIOM_PORT_SEL_IO4_SEL_POS                          (16)


/*
 * Selects connection for IO pad 5 route.
 */
#define HSIOM_PORT_SEL_IO5_SEL_MASK                         (0x00f00000) /* <20:23> RW:RW:0:IO5 */
#define HSIOM_PORT_SEL_IO5_SEL_POS                          (20)


/*
 * Selects connection for IO pad 6 route.
 */
#define HSIOM_PORT_SEL_IO6_SEL_MASK                         (0x0f000000) /* <24:27> RW:RW:0:IO6 */
#define HSIOM_PORT_SEL_IO6_SEL_POS                          (24)


/*
 * Selects connection for IO pad 7 route.
 */
#define HSIOM_PORT_SEL_IO7_SEL_MASK                         (0xf0000000) /* <28:31> RW:RW:0:IO7 */
#define HSIOM_PORT_SEL_IO7_SEL_POS                          (28)


/*
 * Power Mode Control
 * Controls the device power mode options and allows observation of current
 * state.
 */
#define PWR_CONTROL_ADDRESS                                 (0x40030000)
#define PWR_CONTROL                                         (*(volatile uint32_t *)(0x40030000))
#define PWR_CONTROL_DEFAULT                                 (0x00000000)

/*
 * Current power mode of the device.  Note that this field cannot be read
 * in all power modes on actual silicon.
 */
#define PWR_CONTROL_POWER_MODE_MASK                         (0x0000000f) /* <0:3> RW:R:0: */
#define PWR_CONTROL_POWER_MODE_POS                          (0)


/*
 * Indicates whether a debug session is active (CDBGPWRUPREQ signal is 1)
 */
#define PWR_CONTROL_DEBUG_SESSION                           (1u << 4) /* <4:4> RW:R:0: */


/*
 * Indicates whether the low power mode regulator is ready to enter DEEPSLEEP
 * mode.
 * 0: If DEEPSLEEP mode is requested, device will enter SLEEP mode.  When
 * low power regulators are ready, device will automatically enter the originally
 * requested mode.
 * 1: Normal operation.  DEEPSLEEP works as described.
 */
#define PWR_CONTROL_LPM_READY                               (1u << 5) /* <5:5> RW:R:0: */


/*
 * Spare AHB readback bits that are hooked to PWR_PWRSYS_TRIM1.SPARE_TRIM[1:0]
 * through spare logic equivalent to bitwise inversion.  Engineering only.
 */
#define PWR_CONTROL_SPARE_MASK                              (0x000c0000) /* <18:19> RW:R:0: */
#define PWR_CONTROL_SPARE_POS                               (18)


/*
 * Always write 0 except as noted below.
 *
 * PSoC4-S0 and Streetfighter CapSense products may set this bit if Vccd
 * is provided externally (on Vccd pin).  Setting this bit turns off the
 * active regulator and will lead to system reset (BOD) unless both Vddd
 * and Vccd pins are supplied externally.  This register bit only resets
 * for XRES, POR, or a detected BOD.
 */
#define PWR_CONTROL_EXT_VCCD                                (1u << 23) /* <23:23> A:RW:0: */


/*
 * Power System Key&Delay Register
 */
#define PWR_KEY_DELAY_ADDRESS                               (0x40030004)
#define PWR_KEY_DELAY                                       (*(volatile uint32_t *)(0x40030004))
#define PWR_KEY_DELAY_DEFAULT                               (0x000000f8)

/*
 * Delay to wait for references to settle on wakeup from deepsleep.  BOD
 * is ignored and system does not resume until this delay expires. Note that
 * the same delay on POR is hard-coded.  The default assumes the output of
 * the predivider is 48MHz + 3%.  Firmware may scale this setting according
 * to the fastest actual clock frequency that can occur when waking from
 * DEEPSLEEP.
 */
#define PWR_KEY_DELAY_WAKEUP_HOLDOFF_MASK                   (0x000003ff) /* <0:9> R:RW:248: */
#define PWR_KEY_DELAY_WAKEUP_HOLDOFF_POS                    (0)


/*
 * Power ADFT Mode Selection Register
 * Controls System Resources ADFT mode settings and observability.   Writes
 * to this register are ignored and settings in this register have no effect
 * unless the part is in a XRES key selected DfT mode.  Entire register is
 * engineering only.  Note that PWR_DDFT_XRES can be used to enter an XRES
 * key if XRES key sequence is not desired.
 */
#define PWR_ADFT_SELECT_ADDRESS                             (0x40030008)
#define PWR_ADFT_SELECT                                     (*(volatile uint32_t *)(0x40030008))
#define PWR_ADFT_SELECT_DEFAULT                             (0x00000000)

/*
 * Selects the current or voltage source from SRSS-Lite that needs to be
 * observed.  Currents are only available through amuxbusa.  Voltages are
 * available though both amuxbusa and amuxbusb (Kelvin connections).
 */
#define PWR_ADFT_SELECT_SRSS_SEL_MASK                       (0x0000001f) /* <0:4> R:RW:0: */
#define PWR_ADFT_SELECT_SRSS_SEL_POS                        (0)


/*
 * Connect amuxbusa to selected ADFT signal.
 * 0: do not connect
 * 1: connect
 */
#define PWR_ADFT_SELECT_SRSS_AMUXA                          (1u << 8) /* <8:8> R:RW:0: */


/*
 * Connect amuxbusb to selected ADFT signal.
 * 0: do not connect
 * 1: connect
 */
#define PWR_ADFT_SELECT_SRSS_AMUXB                          (1u << 9) /* <9:9> R:RW:0: */


/*
 * When set enables bleeder cells on various switched power nets to accelerate
 * discharge during low power mode testing.
 */
#define PWR_ADFT_SELECT_BLEED_EN                            (1u << 29) /* <29:29> R:RW:0: */


/*
 * Setting this bit will trigger a rst_por_hv_n reset from the s8SRSSULTa.
 *  The system will go through POR reset.
 */
#define PWR_ADFT_SELECT_POR_TRIGGER                         (1u << 30) /* <30:30> R:RW:0: */


/*
 * Enables/disables the system resources ADFT observability.  Disable this
 * bit before changing the ADFT selection.  This prevents glitches and transients
 * from affecting the references.  There is no internal hardware break-before-make.
 */
#define PWR_ADFT_SELECT_SRSS_EN                             (1u << 31) /* <31:31> R:RW:0: */


/*
 * Power DDFT Mode Selection Register
 * Selects the signal sources output to the DDFT outputs of the power subsystem.
 * Entire register is engineering only.
 */
#define PWR_DDFT_SELECT_ADDRESS                             (0x4003000c)
#define PWR_DDFT_SELECT                                     (*(volatile uint32_t *)(0x4003000c))
#define PWR_DDFT_SELECT_DEFAULT                             (0x00000000)

/*
 * Select signal for power DDFT output #0
 */
#define PWR_DDFT_SELECT_DDFT0_SEL_MASK                      (0x0000000f) /* <0:3> R:RW:0: */
#define PWR_DDFT_SELECT_DDFT0_SEL_POS                       (0)


/*
 * Select signal for power DDFT output #1
 */
#define PWR_DDFT_SELECT_DDFT1_SEL_MASK                      (0x000000f0) /* <4:7> R:RW:0: */
#define PWR_DDFT_SELECT_DDFT1_SEL_POS                       (4)


/*
 * XRES DfT Key observer logic test register
 * This register is used to test the XRES TestMode key logic.  It allows
 * a test routine (firmware or ATE driven) to stimulate the key listener
 * and observe its functionality.  Extreme case must be taken in these tests,
 * since they will result in actual test mode entry.  For example, shifting
 * in a scan mode key, will transition the system into scan mode immediately.
 *  Note that test_scan_mode is not observable in this register for that
 * reason.
 */
#define PWR_DDFT_XRES_ADDRESS                               (0x40030010)
#define PWR_DDFT_XRES                                       (*(volatile uint32_t *)(0x40030010))
#define PWR_DDFT_XRES_DEFAULT                               (0x00000000)

/*
 * Tied to the XRES DfT key observation shift register input.
 */
#define PWR_DDFT_XRES_KEY_IN                                (1u << 0) /* <0:0> R:RW:0: */


/*
 * Tied to the XRES DfT key observation shift register clock.
 */
#define PWR_DDFT_XRES_KEY_CLK                               (1u << 1) /* <1:1> R:RW:0: */


/*
 * Output of the 32-bit hard key shift register
 */
#define PWR_DDFT_XRES_HARD_KEY_OUT                          (1u << 2) /* <2:2> RW:R:0: */


/*
 * Set this to 1 to block all test_key_* signals (blocks both logic side
 * effects and  bits in this register)
 */
#define PWR_DDFT_XRES_BLOCK                                 (1u << 3) /* <3:3> R:RW:0: */


/*
 * Hooked up to test_key_dft_en
 */
#define PWR_DDFT_XRES_KEY_DFT_EN                            (1u << 8) /* <8:8> RW:R:0: */


/*
 * Hooked up to test_key_reg_disable
 */
#define PWR_DDFT_XRES_KEY_REG_DISABLE                       (1u << 9) /* <9:9> RW:R:0: */


/*
 * Hooked up to test_key_safe_mode
 */
#define PWR_DDFT_XRES_KEY_SAFE_MODE                         (1u << 10) /* <10:10> RW:R:0: */


/*
 * Hooked up to test_key_por_circuit
 */
#define PWR_DDFT_XRES_KEY_POR_CIRCUIT                       (1u << 11) /* <11:11> RW:R:0: */


/*
 * Hooked up to test_key_clk_ext
 */
#define PWR_DDFT_XRES_KEY_CLK_EXT                           (1u << 12) /* <12:12> RW:R:0: */


/*
 * Indicates that the 32-bit shift register has observed the correct key
 */
#define PWR_DDFT_XRES_HARD_KEY_OK                           (1u << 30) /* <30:30> RW:R:0: */


/*
 * Indicates that the 125-bit key observer has observed the correct key
 */
#define PWR_DDFT_XRES_SOFT_KEY_OK                           (1u << 31) /* <31:31> RW:R:0: */


/*
 * Test Mode Control Register
 * Controls primary test mode.  This is a single bit that can be written
 * directly from the ATE/Programmer in any protection mode.  It's main function
 * is to signal to the Boot ROM that normal firmware execution is not to
 * commence after boot is complete.  Instead the Boot ROM will enter a wait
 * loop for system commands.
 */
#define TST_MODE_ADDRESS                                    (0x40030014)
#define TST_MODE                                            (*(volatile uint32_t *)(0x40030014))
#define TST_MODE_DEFAULT                                    (0x00000000)

/*
 * 0: SWD not active
 * 1: SWD activated (Line Reset & Connect sequence passed)
 * (Note: this bit replaces TST_CTRL.SWD_CONNECTED and is present in all
 * M0S8 products except TSG4)
 */
#define TST_MODE_SWD_CONNECTED                              (1u << 2) /* <2:2> RW:R:0: */


/*
 * Relevant only for parts that have the alternate XRES mechanism of overloading
 * a GPIO pin temporarily as alternate XRES during test.  When set, this
 * bit blocks the alternate XRES function, such that the pin can be used
 * for normal I/O or for ddft/adft observation.  See SAS Part-V and Part-IX
 * for details. This register bit only resets for XRES, POR, or a detected
 * BOD.
 */
#define TST_MODE_BLOCK_ALT_XRES                             (1u << 28) /* <28:28> A:RW:0: */


/*
 * This bit is set when a XRES test mode key is shifted in.  It is the value
 * of the test_key_dft_en signal.  When this bit is set, the BootROM will
 * not yield execution to the FLASH image (same function as setting TEST_MODE
 * bit below).
 */
#define TST_MODE_TEST_KEY_DFT_EN                            (1u << 30) /* <30:30> RW:R:0: */


/*
 * 0: Normal operation mode
 * 1: Test mode (any test mode)
 * Setting this bit will prevent BootROM from yielding execution to Flash
 * image.
 */
#define TST_MODE_TEST_MODE                                  (1u << 31) /* <31:31> R:RW:0: */


/*
 * Digital DFT Control Register
 * Controls system level DDFT observability muxes and comparators.
 */
#define TST_DDFT_CTRL_ADDRESS                               (0x40030018)
#define TST_DDFT_CTRL                                       (*(volatile uint32_t *)(0x40030018))
#define TST_DDFT_CTRL_DEFAULT                               (0x00000f0f)

/*
 * Select signal for DDFT output #0
 */
#define TST_DDFT_CTRL_DFT_SEL0_MASK                         (0x0000000f) /* <0:3> R:RW:15: */
#define TST_DDFT_CTRL_DFT_SEL0_POS                          (0)


/*
 * Select signal for DDFT output #1
 */
#define TST_DDFT_CTRL_DFT_SEL1_MASK                         (0x00000f00) /* <8:11> R:RW:15: */
#define TST_DDFT_CTRL_DFT_SEL1_POS                          (8)


/*
 * 1: Enables DDFT functionality.  Connects output of DDFT mux to designated
 * DDFT pin.
 */
#define TST_DDFT_CTRL_ENABLE                                (1u << 31) /* <31:31> R:RW:0: */


/*
 * IMO trim down-counter and status (clk_sys)
 */
#define TST_TRIM_CNTR1_ADDRESS                              (0x4003001c)
#define TST_TRIM_CNTR1                                      (*(volatile uint32_t *)(0x4003001c))
#define TST_TRIM_CNTR1_DEFAULT                              (0x80000000)

/*
 * Down-counter clocked on clk_sys. By writing non-zero value to this counter
 * TRIM_CNTR2.COUNTER clears and counts up. TRIM_CNTR1.COUNTER counts down
 * until TRIM_CNTR1.COUNTER==0
 */
#define TST_TRIM_CNTR1_COUNTER_MASK                         (0x0000ffff) /* <0:15> RW:RW:0: */
#define TST_TRIM_CNTR1_COUNTER_POS                          (0)


/*
 * Status bit indicating that TRIM_CNTR1.COUNTER==0 and TRIM_CNT2.COUNTER
 * stopped counting up
 */
#define TST_TRIM_CNTR1_COUNTER_DONE                         (1u << 31) /* <31:31> W:R:1: */


/*
 * IMO trim up-counter  (ddft)
 */
#define TST_TRIM_CNTR2_ADDRESS                              (0x40030020)
#define TST_TRIM_CNTR2                                      (*(volatile uint32_t *)(0x40030020))
#define TST_TRIM_CNTR2_DEFAULT                              (0x00000000)

/*
 * Up-counter clocked on Clock DDFT output #1. When TRIM_CNTR1.COUNT_DONE==1
 * counter stopped and can be read by SW.  The expected final value is related
 * to the ratio of clock frequencies used for the two counters and the value
 * loaded into counter 1: TST_TRIM_CNTR2.COUNTER=(F_cntr2/F_cntr1)*(TST_TRIM_CNTR1.COUNTER-1)
 */
#define TST_TRIM_CNTR2_COUNTER_MASK                         (0x0000ffff) /* <0:15> RW:R:0: */
#define TST_TRIM_CNTR2_COUNTER_POS                          (0)


/*
 * ADFT buffer/comparator control register
 * Controls System Resources ADFT mode settings and observability.  Writes
 * to this register are ignored and settings in this register have no effect
 * unless the part is in a XRES key selected DfT mode.  Entire register is
 * engineering only. Note that PWR_DDFT_XRES can be used to enter an XRES
 * key if XRES key sequence is not desired.
 */
#define TST_ADFT_CTRL_ADDRESS                               (0x40030024)
#define TST_ADFT_CTRL                                       (*(volatile uint32_t *)(0x40030024))
#define TST_ADFT_CTRL_DEFAULT                               (0x00000000)

/*
 * Not used. Spare bit
 */
#define TST_ADFT_CTRL_SPARE_4                               (1u << 31) /* <31:31> R:RW:0: */


/*
 * Clock Select Register
 * Configures direction of all clock multiplexers and selectors.  See Section
 * 20.3 in SAS for details on clock network topology.  See PAS for DSI signal
 * connectivity list.
 */
#define CLK_SELECT_ADDRESS                                  (0x40030028)
#define CLK_SELECT                                          (*(volatile uint32_t *)(0x40030028))
#define CLK_SELECT_DEFAULT                                  (0x00000008)

/*
 * Selects a source for clk_hf and dsi_in[0].  Note that not all products
 * support all clock sources.  Selecting a clock source that is not supported
 * will result in undefined behavior.
 */
#define CLK_SELECT_HFCLK_SEL_MASK                           (0x00000003) /* <0:1> R:RW:0: */
#define CLK_SELECT_HFCLK_SEL_POS                            (0)


/*
 * Selects clk_hf predivider value.
 */
#define CLK_SELECT_HFCLK_DIV_MASK                           (0x0000000c) /* <2:3> R:RW:2: */
#define CLK_SELECT_HFCLK_DIV_POS                            (2)


/*
 * Selects clock source for charge pump clock.  This clock is not guaranteed
 * to be glitch free when changing any of its sources or settings.
 */
#define CLK_SELECT_PUMP_SEL_MASK                            (0x00000030) /* <4:5> R:RW:0: */
#define CLK_SELECT_PUMP_SEL_POS                             (4)


/*
 * Select clk_sys prescaler value.
 */
#define CLK_SELECT_SYSCLK_DIV_MASK                          (0x000000c0) /* <6:7> R:RW:0: */
#define CLK_SELECT_SYSCLK_DIV_POS                           (6)


/*
 * ILO Configuration
 * Internal slow speed R/C oscillator (32kHz) configuration register. Note:
 * writes to this register are ignored when WDT_DISABLE_KEY is not set to
 * the magic value defined for it.
 */
#define CLK_ILO_CONFIG_ADDRESS                              (0x4003002c)
#define CLK_ILO_CONFIG                                      (*(volatile uint32_t *)(0x4003002c))
#define CLK_ILO_CONFIG_DEFAULT                              (0x80000000)

/*
 * Master enable for ILO oscillator.  This bit is hardware set whenever the
 * WD_DISABLE_KEY is not set to the magic value.
 */
#define CLK_ILO_CONFIG_ENABLE                               (1u << 31) /* <31:31> RW:RW:1: */


/*
 * IMO Configuration
 * Internal high speed R/C oscillator configuration register. Note that this
 * oscillator comes up active on power up.  The oscillator provides the primary
 * system clock (HFCLK) on power up until firmware configures differently.
 *  This oscillator is also used before system start to count out power up
 * delays.  This is done in fast IMO (FIMO) mode that does not require any
 * external references and runs at a fixed 12MHz.
 */
#define CLK_IMO_CONFIG_ADDRESS                              (0x40030030)
#define CLK_IMO_CONFIG                                      (*(volatile uint32_t *)(0x40030030))
#define CLK_IMO_CONFIG_DEFAULT                              (0x80000000)

/*
 * Master enable for IMO oscillator.  Clearing this bit will disable the
 * IMO.  Don't do this if the system is running off it.
 */
#define CLK_IMO_CONFIG_ENABLE                               (1u << 31) /* <31:31> R:RW:1: */


/*
 * Clock DFT Mode Selection Register
 * Selects which clock signals to bring out to to DFT pins.  Two signals
 * can be selected to enable comparison of clocks.  Clocks can be divided
 * down to deal with slower equipment and I/Os.  See TST_DFT_SELECT for details
 * on bringing these pins out.  Entire register is engineering only.
 */
#define CLK_DFT_SELECT_ADDRESS                              (0x40030034)
#define CLK_DFT_SELECT                                      (*(volatile uint32_t *)(0x40030034))
#define CLK_DFT_SELECT_DEFAULT                              (0x00000000)

/*
 * Select signal for DFT output #0
 */
#define CLK_DFT_SELECT_DFT_SEL0_MASK                        (0x0000000f) /* <0:3> R:RW:0: */
#define CLK_DFT_SELECT_DFT_SEL0_POS                         (0)


/*
 * DFT Output Divide Down.
 */
#define CLK_DFT_SELECT_DFT_DIV0_MASK                        (0x00000030) /* <4:5> R:RW:0: */
#define CLK_DFT_SELECT_DFT_DIV0_POS                         (4)


/*
 * Edge sensitivity for in-line divider on output #0 (only relevant when
 * DIV0>0).
 */
#define CLK_DFT_SELECT_DFT_EDGE0                            (1u << 6) /* <6:6> R:RW:0: */


/*
 * Select signal for DFT output #1
 */
#define CLK_DFT_SELECT_DFT_SEL1_MASK                        (0x00000f00) /* <8:11> R:RW:0: */
#define CLK_DFT_SELECT_DFT_SEL1_POS                         (8)


/*
 * DFT Output Divide Down.
 */
#define CLK_DFT_SELECT_DFT_DIV1_MASK                        (0x00003000) /* <12:13> R:RW:0: */
#define CLK_DFT_SELECT_DFT_DIV1_POS                         (12)


/*
 * Edge sensitivity for in-line divider on output #1 (only relevant when
 * DIV1>0).
 */
#define CLK_DFT_SELECT_DFT_EDGE1                            (1u << 14) /* <14:14> R:RW:0: */


/*
 * Watchdog Disable Key Register
 * This key can be used to disable the watchdog timer reset generation in
 * applications that do not require absolute brown-out safety and do not
 * want to deal with the hassle of feeding the watchdog regularly.  Setting
 * the key will also enable the CLK_ILO_CONFIG.ENABLE bit to be effective.
 *  It will not have any other effect, i.e. the WDT timer/interrupt functionality
 * can still be used.
 */
#define WDT_DISABLE_KEY_ADDRESS                             (0x40030038)
#define WDT_DISABLE_KEY                                     (*(volatile uint32_t *)(0x40030038))
#define WDT_DISABLE_KEY_DEFAULT                             (0x00000000)

/*
 * Disables WDT reset when equal to 0xACED8865.  The WDT reset functions
 * normally for any other setting.
 */
#define WDT_DISABLE_KEY_KEY_MASK                            (0xffffffff) /* <0:31> R:RW:0: */
#define WDT_DISABLE_KEY_KEY_POS                             (0)


/*
 * Watchdog Counter Register
 * Provides actual counter value for watchdog counter.  Watchdog counter
 * always counts up, is free-running and is clocked using clk_lf.
 */
#define WDT_COUNTER_ADDRESS                                 (0x4003003c)
#define WDT_COUNTER                                         (*(volatile uint32_t *)(0x4003003c))
#define WDT_COUNTER_DEFAULT                                 (0x00000000)

/*
 * Current value of WDT Counter
 */
#define WDT_COUNTER_COUNTER_MASK                            (0x0000ffff) /* <0:15> RW:R:0: */
#define WDT_COUNTER_COUNTER_POS                             (0)


/*
 * Watchdog Match Register
 * Firmware provided match value that is compared against WDT_COUNTER.  The
 * expectation is that firmware modifies this register after each match as
 * part of the WDT interrupt service routine.
 */
#define WDT_MATCH_ADDRESS                                   (0x40030040)
#define WDT_MATCH                                           (*(volatile uint32_t *)(0x40030040))
#define WDT_MATCH_DEFAULT                                   (0x00001000)

/*
 * Match value for Watchdog counter.  Every time WDT_COUNTER reaches MATCH
 * an interrupt is generated.  Two unserviced interrupts will lead to a system
 * reset (i.e. at the third match).
 */
#define WDT_MATCH_MATCH_MASK                                (0x0000ffff) /* <0:15> R:RW:4096: */
#define WDT_MATCH_MATCH_POS                                 (0)


/*
 * The number of MSB bits of the watchdog timer that are NOT checked against
 * MATCH.  This value provides control over the time-to-reset of the watchdog
 * (which happens after 3 successive matches).  Note that certain products
 * may enforce a minimum value for this register through design time configuration.
 */
#define WDT_MATCH_IGNORE_BITS_MASK                          (0x000f0000) /* <16:19> R:RW:0: */
#define WDT_MATCH_IGNORE_BITS_POS                           (16)


/*
 * SRSS Interrupt Register
 * The intent is that this register is cleared for every WDT interrupt under
 * all circumstances, including when the system is in DeepSleep.
 */
#define SRSS_INTR_ADDRESS                                   (0x40030044)
#define SRSS_INTR                                           (*(volatile uint32_t *)(0x40030044))
#define SRSS_INTR_DEFAULT                                   (0x00000000)

/*
 * WDT Interrupt Request.  This bit is set each time WDT_COUNTR==WDT_MATCH.
 *  Clearing this bit also feeds the watch dog.  Missing 2 interrupts in
 * a row will generate brown-out reset.  Due to internal synchronization,
 * it takes 2 SYSCLK cycles to update after a W1C.
 */
#define SRSS_INTR_WDT_MATCH                                 (1u << 0) /* <0:0> A:RW1C:0: */


/*
 * Spare
 * SPARE_2
 */
#define SPARE_2_ADDRESS                                     (0x40030048)
#define SPARE_2                                             (*(volatile uint32_t *)(0x40030048))
#define SPARE_2_DEFAULT                                     (0x00000000)

/*
 * Spare
 */
#define SPARE_2_SPARE_2                                     (1u << 1) /* <1:1> R:RW:0: */


/*
 * SRSS Interrupt Mask Register
 * Controls whether interrupt is forwarded to CPU.
 */
#define SRSS_INTR_MASK_ADDRESS                              (0x4003004c)
#define SRSS_INTR_MASK                                      (*(volatile uint32_t *)(0x4003004c))
#define SRSS_INTR_MASK_DEFAULT                              (0x00000000)

/*
 * Clearing this bit will not forward the interrupt to the CPU.  It will
 * not, however, disable the WDT reset generation on 2 missed interrupts.
 */
#define SRSS_INTR_MASK_WDT_MATCH                            (1u << 0) /* <0:0> R:RW:0: */


/*
 * SPARE
 */
#define SRSS_INTR_MASK_SPARE_5                              (1u << 1) /* <1:1> R:RW:0: */


/*
 * SRSS ADFT control register
 * This register can be used only when in a test mode entered through an
 * XRES:DFT:* key.  It provides direct control over and visibility of the
 * SRSS power and reference circuits. Note that act_reg_en is controlled
 * through PWR_CONTROL.EXT_VCCD.  It is possible to cause behavior that is
 * normally considered illegal, such as disabling a circuit without regard
 * for dependencies.  Engineering only. Note that PWR_DDFT_XRES can be used
 * to enter an XRES key if XRES key sequence is not desired.
 */
#define SRSS_ADFT_CONTROL_ADDRESS                           (0x40030050)
#define SRSS_ADFT_CONTROL                                   (*(volatile uint32_t *)(0x40030050))
#define SRSS_ADFT_CONTROL_DEFAULT                           (0x0000003f)

/*
 * Enables/disables the Active reference.  Set CLK_SELECT.HFCLK_SEL=EXTCLK
 * before disabling the reference.
 */
#define SRSS_ADFT_CONTROL_ACT_REF_EN                        (1u << 0) /* <0:0> R:RW:1: */


/*
 * Enables/disables the Active power comparator.  Set CLK_SELECT.HFCLK_SEL=EXTCLK
 * before disabling the comparator.
 */
#define SRSS_ADFT_CONTROL_ACT_COMP_EN                       (1u << 1) /* <1:1> R:RW:1: */


/*
 * Enables/disables the DeepSleep reference
 */
#define SRSS_ADFT_CONTROL_DPSLP_REF_EN                      (1u << 2) /* <2:2> R:RW:1: */


/*
 * Enables/disables the DeepSleep regulator
 */
#define SRSS_ADFT_CONTROL_DPSLP_REG_EN                      (1u << 3) /* <3:3> R:RW:1: */


/*
 * Enables/disables the DeepSleep power comparator
 */
#define SRSS_ADFT_CONTROL_DPSLP_COMP_EN                     (1u << 4) /* <4:4> R:RW:1: */


/*
 * Mode override for the DeepSleep reference.  0=DEEPSLEEP, 1=ACTIVE.  Clearing
 * this bit allows characterization of the DeepSleep reference during ACTIVE
 * mode.
 */
#define SRSS_ADFT_CONTROL_DPSLP_REF_MODE                    (1u << 5) /* <5:5> R:RW:1: */


/*
 * Indicates Active reference is valid
 */
#define SRSS_ADFT_CONTROL_ACT_REF_VALID                     (1u << 8) /* <8:8> RW:R:0: */


/*
 * Indicates Active regulator is regulating
 */
#define SRSS_ADFT_CONTROL_ACT_REG_VALID                     (1u << 9) /* <9:9> RW:R:0: */


/*
 * Indicates Active comparator is valid
 */
#define SRSS_ADFT_CONTROL_ACT_COMP_OUT                      (1u << 10) /* <10:10> RW:R:0: */


/*
 * Indicates DeepSleep comparator is valid
 */
#define SRSS_ADFT_CONTROL_DPSLP_COMP_OUT                    (1u << 11) /* <11:11> RW:R:0: */


/*
 * Reset Cause Observation Register
 * Indicates the cause for the latest reset(s) that occurred in the system.
 *  Note that resets due to power up and brown-outs below state retention
 * voltages in regulated and unregulated domains cannot be distinguished
 * from eachother.  All bits in this register assert when the corresponding
 * reset cause occurs and must be cleared by firmware.  These bits are cleared
 * by hardware only during XRES, POR or after a detected brown-out.
 */
#define RES_CAUSE_ADDRESS                                   (0x40030054)
#define RES_CAUSE                                           (*(volatile uint32_t *)(0x40030054))
#define RES_CAUSE_DEFAULT                                   (0x00000000)

/*
 * A WatchDog Timer reset has occurred since last power cycle.
 */
#define RES_CAUSE_RESET_WDT                                 (1u << 0) /* <0:0> A:RW1C:0: */


/*
 * A protection violation occurred that requires a RESET.  This includes,
 * but is not limited to, hitting a debug breakpoint while in Privileged
 * Mode.
 */
#define RES_CAUSE_RESET_PROT_FAULT                          (1u << 3) /* <3:3> A:RW1C:0: */


/*
 * Cortex-M0 requested a system reset through it's SYSRESETREQ.  This can
 * be done via a debugger probe or in firmware.
 */
#define RES_CAUSE_RESET_SOFT                                (1u << 4) /* <4:4> A:RW1C:0: */


/*
 * Reset DFT Register
 * Controls the DFT options for the reset system.   Writes to this register
 * are ignored and settings in this register have no effect unless DFT is
 * enabled through a XRES:DFT:* key (see SAS for details).  Note that PWR_DDFT_XRES
 * can be used to enter an XRES key if XRES key sequence is not desired.
 */
#define RES_DFT_ADDRESS                                     (0x40030058)
#define RES_DFT                                             (*(volatile uint32_t *)(0x40030058))
#define RES_DFT_DEFAULT                                     (0x00000000)

/*
 * Setting this bit will disconnect the output of the reset delay line from
 * the reset system blocking soft resets from reaching the logic.  Note:
 * XRES/POR/BOD resets still have other effects, but this bit MUST be cleared
 * before XRES or any other reset can be applied to the system after doing
 * the delay line testing.
 * When DELAY_BLOCK=1, reset DDFT output #0 is connected the input of the
 * delay line and output #1 is connected to the output of the delay line.
 */
#define RES_DFT_DELAY_BLOCK                                 (1u << 0) /* <0:0> R:RW:0: */


/*
 * When DELAY_BLOCK=1, this bit is connected to the input of the reset delay
 * line.  Both the input and output can be observed through the DDFT network
 * to measure their timing relationship.
 */
#define RES_DFT_DELAY_IN                                    (1u << 1) /* <1:1> R:RW:0: */


/*
 * Bandgap Trim Register
 * Trim bits for Reference System. Entire register is engineering only.
 */
#define PWR_BG_TRIM1_ADDRESS                                (0x40030f00)
#define PWR_BG_TRIM1                                        (*(volatile uint32_t *)(0x40030f00))
#define PWR_BG_TRIM1_DEFAULT                                (0x00000010)

/*
 * Trims the bandgap reference voltage output.  Used to trim the VBG to the
 * voltage where its temperature curvature is minimal.  Bit [5] is unused
 * within the bandgap block.
 */
#define PWR_BG_TRIM1_REF_VTRIM_MASK                         (0x0000003f) /* <0:5> R:RW:16: */
#define PWR_BG_TRIM1_REF_VTRIM_POS                          (0)


/*
 * Active-Reference temperature compensation trim
 * trim the Active-Reference VREF temperature coefficient (TC).
 *   00: TC = 0 (unchanged)
 *   01: TC = +80ppm/C
 *   10: TC = -80ppm/C
 *   11: TC = -150ppm/C
 */
#define PWR_BG_TRIM1_VREF_TC_TRIM_MASK                      (0x000000c0) /* <6:7> R:RW:0: */
#define PWR_BG_TRIM1_VREF_TC_TRIM_POS                       (6)


/*
 * Bandgap Trim Register
 * Trim bits for Reference System. Entire register is engineering only.
 */
#define PWR_BG_TRIM2_ADDRESS                                (0x40030f04)
#define PWR_BG_TRIM2                                        (*(volatile uint32_t *)(0x40030f04))
#define PWR_BG_TRIM2_DEFAULT                                (0x0000001c)

/*
 * Trims the bandgap reference current output.  Used to trim the IBG to the
 * voltage where its temperature curvature is minimal.
 */
#define PWR_BG_TRIM2_REF_ITRIM_MASK                         (0x0000003f) /* <0:5> R:RW:28: */
#define PWR_BG_TRIM2_REF_ITRIM_POS                          (0)


/*
 * Active-Reference temperature compensation trim
 * trim the Active-Reference IREF temperature coefficient (TC).
 *   00: TC = 0 (unchanged)
 *   01: TC = +80ppm/C
 *   10: TC = -80ppm/C
 *   11: TC = -150ppm/C
 */
#define PWR_BG_TRIM2_IREF_TC_TRIM_MASK                      (0x000000c0) /* <6:7> R:RW:0: */
#define PWR_BG_TRIM2_IREF_TC_TRIM_POS                       (6)


/*
 * IMO Frequency Select Register
 * Selects the operating frequency of the IMO
 */
#define CLK_IMO_SELECT_ADDRESS                              (0x40030f08)
#define CLK_IMO_SELECT                                      (*(volatile uint32_t *)(0x40030f08))
#define CLK_IMO_SELECT_DEFAULT                              (0x00000000)

/*
 * Select operating frequency
 */
#define CLK_IMO_SELECT_FREQ_MASK                            (0x00000007) /* <0:2> R:RW:0: */
#define CLK_IMO_SELECT_FREQ_POS                             (0)


/*
 * IMO Trim Register
 * Trims IMO frequency to within datasheet accuracy.  Must be applied
 */
#define CLK_IMO_TRIM1_ADDRESS                               (0x40030f0c)
#define CLK_IMO_TRIM1                                       (*(volatile uint32_t *)(0x40030f0c))
#define CLK_IMO_TRIM1_DEFAULT                               (0x00000080)

/*
 * Frequency trim bits.  These bits are determined at manufacturing time
 * for each FREQ setting (IMO_TRIM2) and stored in SFLASH.
 */
#define CLK_IMO_TRIM1_OFFSET_MASK                           (0x000000ff) /* <0:7> R:RW:128: */
#define CLK_IMO_TRIM1_OFFSET_POS                            (0)


/*
 * Dummy
 * Dummy
 */
#define CLK_IMO_TRIM2_ADDRESS                               (0x40030f10)
#define CLK_IMO_TRIM2                                       (*(volatile uint32_t *)(0x40030f10))
#define CLK_IMO_TRIM2_DEFAULT                               (0x00000000)

/*
 * SPARE1
 */
#define CLK_IMO_TRIM2_DUMMY1                                (1u << 0) /* <0:0> R:RW:0: */


/*
 * IMO Trim Register
 * IMO Trim Bits.   Entire register is engineering only.
 */
#define CLK_IMO_TRIM3_ADDRESS                               (0x40030f18)
#define CLK_IMO_TRIM3                                       (*(volatile uint32_t *)(0x40030f18))
#define CLK_IMO_TRIM3_DEFAULT                               (0x00000050)

/*
 * IMO trim stepsize bits.  These bits are determined at manufacturing time
 * to adjust for process variation.  They are used to tune the stepsize of
 * the FSOFFSET and OFFSET trims.
 */
#define CLK_IMO_TRIM3_STEPSIZE_MASK                         (0x0000001f) /* <0:4> R:RW:16: */
#define CLK_IMO_TRIM3_STEPSIZE_POS                          (0)


/*
 * IMO temperature compesation trim.  These bits are determined at manufacturing
 * time to adjust for temperature dependence. This bits are dependent on
 * frequency and need to be changed using the Cypress provided frequency
 * change algorithm.
 */
#define CLK_IMO_TRIM3_TCTRIM_MASK                           (0x00000060) /* <5:6> R:RW:2: */
#define CLK_IMO_TRIM3_TCTRIM_POS                            (5)


/*
 * Regulator Trim
 * IMO Trim Bits.   Entire register is engineering only.
 */
#define REGULATOR_ADDRESS                                   (0x40030f1c)
#define REGULATOR                                           (*(volatile uint32_t *)(0x40030f1c))
#define REGULATOR_DEFAULT                                   (0x00000000)

/*
 * Trim for the regulator
 */
#define REGULATOR_TRIM_MASK                                 (0x00000007) /* <0:2> R:RW:0:NO_USE_SWITCH */
#define REGULATOR_TRIM_POS                                  (0)


/*
 * Port output data register
 * Used to read and write the output data for the IO pads in the port. A
 * DR register write changes the output data to the written value. A DR register
 * read reflects the output data (and not the current state of the input
 * data for the IO pads). Using this DR register, Read-Modify-Write sequences
 * are safely performed on a port with some IO pads configured as inputs.
 */
#define GPIO_PRT_DR_ADDRESS                                 (0x40040000)
#define GPIO_PRT_DR                                         (*(volatile uint32_t *)(0x40040000))
#define GPIO_PRT_DR_DEFAULT                                 (0x00000000)

/*
 * IO pad 0 output data.
 */
#define GPIO_PRT_DR_DATA0                                   (1u << 0) /* <0:0> RW:RW:0:IO0 */


/*
 * IO pad 1 output data.
 */
#define GPIO_PRT_DR_DATA1                                   (1u << 1) /* <1:1> RW:RW:0:IO1 */


/*
 * IO pad 2 output data.
 */
#define GPIO_PRT_DR_DATA2                                   (1u << 2) /* <2:2> RW:RW:0:IO2 */


/*
 * IO pad 3 output data.
 */
#define GPIO_PRT_DR_DATA3                                   (1u << 3) /* <3:3> RW:RW:0:IO3 */


/*
 * IO pad 4 output data.
 */
#define GPIO_PRT_DR_DATA4                                   (1u << 4) /* <4:4> RW:RW:0:IO4 */


/*
 * IO pad 5 output data.
 */
#define GPIO_PRT_DR_DATA5                                   (1u << 5) /* <5:5> RW:RW:0:IO5 */


/*
 * IO pad 6 output data.
 */
#define GPIO_PRT_DR_DATA6                                   (1u << 6) /* <6:6> RW:RW:0:IO6 */


/*
 * IO pad 7 output data.
 */
#define GPIO_PRT_DR_DATA7                                   (1u << 7) /* <7:7> RW:RW:0:IO7 */


/*
 * Port IO pad state register
 * Used to read. Writes to this register have no effect. If the drive mode
 * for the pin is set to high Z Analog, the state will read 0 independent
 * of the voltage on the pin.
 */
#define GPIO_PRT_PS_ADDRESS                                 (0x40040004)
#define GPIO_PRT_PS                                         (*(volatile uint32_t *)(0x40040004))
#define GPIO_PRT_PS_DEFAULT                                 (0x00000000)

/*
 * IO pad 0 state:
 * 1: Logic high, if the pin voltage is above the input buffer threshold,
 * logic high.
 * 0: Logic low, if the pin voltage is below that threshold, logic low.
 * If the drive mode for the pin is set to high Z Analog, the pin state will
 * read 0 independent of the voltage on the pin.
 */
#define GPIO_PRT_PS_DATA0                                   (1u << 0) /* <0:0> W:R:0:IO0 */


/*
 * IO pad 1 state.
 */
#define GPIO_PRT_PS_DATA1                                   (1u << 1) /* <1:1> W:R:0:IO1 */


/*
 * IO pad 2 state.
 */
#define GPIO_PRT_PS_DATA2                                   (1u << 2) /* <2:2> W:R:0:IO2 */


/*
 * IO pad 3 state.
 */
#define GPIO_PRT_PS_DATA3                                   (1u << 3) /* <3:3> W:R:0:IO3 */


/*
 * IO pad 4 state.
 */
#define GPIO_PRT_PS_DATA4                                   (1u << 4) /* <4:4> W:R:0:IO4 */


/*
 * IO pad 5 state.
 */
#define GPIO_PRT_PS_DATA5                                   (1u << 5) /* <5:5> W:R:0:IO5 */


/*
 * IO pad 6 state.
 */
#define GPIO_PRT_PS_DATA6                                   (1u << 6) /* <6:6> W:R:0:IO6 */


/*
 * IO pad 7 state.
 */
#define GPIO_PRT_PS_DATA7                                   (1u << 7) /* <7:7> W:R:0:IO7 */


/*
 * Reads of this register return the logical state of the filtered pin.
 */
#define GPIO_PRT_PS_FLT_DATA                                (1u << 8) /* <8:8> W:R:0: */


/*
 * Port configuration register
 * Configures the output drive and input buffer state for each pin, and the
 * slew rate and input threshold selection for the whole port. One register
 * is provided per port.
 */
#define GPIO_PRT_PC_ADDRESS                                 (0x40040008)
#define GPIO_PRT_PC                                         (*(volatile uint32_t *)(0x40040008))
#define GPIO_PRT_PC_DEFAULT                                 (0x00000000)

/*
 * The GPIO drive mode for IO pad 0.
 * Note: when initializing IO's that are connected to a live bus (such as
 * I2C), make sure the HSIOM is properly configured (HSIOM_PRT_SELx) before
 * turning the IO on here to avoid producing glitches on the bus.
 */
#define GPIO_PRT_PC_DM0_MASK                                (0x00000007) /* <0:2> R:RW:0:IO0 */
#define GPIO_PRT_PC_DM0_POS                                 (0)

/*
 * Mode 0 (analog mode): Output buffer off (high Z). Input buffer off.
 */
#define GPIO_PRT_PC_DM0_DM0_OFF                             (0)
/*
 * Mode 1: Output buffer off (high Z). Input buffer on.
 */
#define GPIO_PRT_PC_DM0_DM0_INPUT                           (1)
/*
 * Mode 2: Strong pull down ('0'), weak/resistive pull up (PU). Input buffer
 * on.
 * For GPIOV1P2_I2C, Strong pull down only.
 */
#define GPIO_PRT_PC_DM0_DM0_0_PU                            (2)
/*
 * Mode 3: Weak/resistive pull down (PD), strong pull up ('1'). Input buffer
 * on.
 * For GPIOV1P2_I2C: Weak pull down only.
 */
#define GPIO_PRT_PC_DM0_DM0_PD_1                            (3)
/*
 * Mode 4: Strong pull down ('0'), open drain (pull up off). Input buffer
 * on.
 * For GPIOV1P2_I2C, Strong pull down only.
 */
#define GPIO_PRT_PC_DM0_DM0_0_Z                             (4)
/*
 * Mode 5: Open drain (pull down off), strong pull up ('1'). Input buffer
 * on.
 * Illegal for GPIOV1P2_I2C
 */
#define GPIO_PRT_PC_DM0_DM0_Z_1                             (5)
/*
 * Mode 6: Strong pull down ('0'), strong pull up ('1'). Input buffer on.
 * For GPIOV1P2_I2C, Strong pull down only.
 */
#define GPIO_PRT_PC_DM0_DM0_0_1                             (6)
/*
 * Mode 7: Weak/resistive pull down (PD), weak/resistive pull up (PU). Input
 * buffer on.
 * For GPIOV1P2_I2C: Weak pull down only.
 */
#define GPIO_PRT_PC_DM0_DM0_PD_PU                           (7)

/*
 * The GPIO drive mode for IO pad 1.
 */
#define GPIO_PRT_PC_DM1_MASK                                (0x00000038) /* <3:5> R:RW:0:IO1 */
#define GPIO_PRT_PC_DM1_POS                                 (3)


/*
 * The GPIO drive mode for IO pad 2.
 */
#define GPIO_PRT_PC_DM2_MASK                                (0x000001c0) /* <6:8> R:RW:0:IO2 */
#define GPIO_PRT_PC_DM2_POS                                 (6)


/*
 * The GPIO drive mode for IO pad 3.
 */
#define GPIO_PRT_PC_DM3_MASK                                (0x00000e00) /* <9:11> R:RW:0:IO3 */
#define GPIO_PRT_PC_DM3_POS                                 (9)


/*
 * The GPIO drive mode for IO pad 4.
 */
#define GPIO_PRT_PC_DM4_MASK                                (0x00007000) /* <12:14> R:RW:0:IO4 */
#define GPIO_PRT_PC_DM4_POS                                 (12)


/*
 * The GPIO drive mode for IO pad 5.
 */
#define GPIO_PRT_PC_DM5_MASK                                (0x00038000) /* <15:17> R:RW:0:IO5 */
#define GPIO_PRT_PC_DM5_POS                                 (15)


/*
 * The GPIO drive mode for IO pad 6.
 */
#define GPIO_PRT_PC_DM6_MASK                                (0x001c0000) /* <18:20> R:RW:0:IO6 */
#define GPIO_PRT_PC_DM6_POS                                 (18)


/*
 * The GPIO drive mode for IO pad 7.
 */
#define GPIO_PRT_PC_DM7_MASK                                (0x00e00000) /* <21:23> R:RW:0:IO7 */
#define GPIO_PRT_PC_DM7_POS                                 (21)


/*
 * The GPIO cells include a VTRIP_SEL signal to alter the input buffer voltage.
 * Note: this bit is ignored for SIO ports, the VTRIP_SEL settings in the
 * SIO register are used instead (a separate VTRIP_SEL is provided for each
 * pin pair).
 * 0: input buffer functions as a CMOS input buffer.
 * 1: input buffer functions as a LVTTL input buffer.
 * For GPIOV1P2 and GPIOV1P_I2C cells, PC.PORT_VTRIP_SEL is unused. Refer
 * to DS register
 */
#define GPIO_PRT_PC_PORT_VTRIP_SEL                          (1u << 24) /* <24:24> R:RW:0: */


/*
 * This field controls the output edge rate of all pins on the port:
 * '0': fast.
 * '1': slow.
 */
#define GPIO_PRT_PC_PORT_SLOW                               (1u << 25) /* <25:25> R:RW:0: */


/*
 * This field selects the input buffer reference. The size (1 or 2 bits)
 * and functionality is dependent on the IO cell.
 * For GPIOv2 IO cells, bit PORT_IB_MODE_SEL[1] is not used (GPIOv2 IO cell
 * replaces GPIO IO cell):
 * "0"/"2": CMOS input buffer (PORT_VTRIP_SEL is '0'), LVTTL input buffer
 * (PORT_VTRIP_SEL is '1')
 * "1"/"3": vcchib.
 * For GPIO_OVTv2 and SIOv2 IO cells:
 * "0": CMOS input buffer (PORT_VTRIP_SEL is '0'), LVTTL input buffer (PORT_VTRIP_SEL
 * is '1')
 * "1": vcchib.
 * "2": OVT.
 * "3": Reference (possibly from reference generator cell).
 * For SIO IO cell, this field is present but not used as the SIO IO cell
 * does not provide input buffer mode select functionality (SIOv2 IO cell
 * will replace SIO IO cell, as soon as it is available).
 * For GPIOV1P2 and GPIOV1P2_I2C cells, PC.PORT_IB_MODE_SEL unused. Refer
 * to DS register.
 */
#define GPIO_PRT_PC_PORT_IB_MODE_SEL_MASK                   (0xc0000000) /* <30:31> R:RW:0: */
#define GPIO_PRT_PC_PORT_IB_MODE_SEL_POS                    (30)


/*
 * Port interrupt configuration register
 * This register configures the IRQ configuration for all pins in a port,
 * with the IRQ type being individually pin-configurable.
 */
#define GPIO_PRT_INTR_CFG_ADDRESS                           (0x4004000c)
#define GPIO_PRT_INTR_CFG                                   (*(volatile uint32_t *)(0x4004000c))
#define GPIO_PRT_INTR_CFG_DEFAULT                           (0x00000000)

/*
 * Sets which edge will trigger an IRQ for IO pad 0.
 */
#define GPIO_PRT_INTR_CFG_EDGE0_SEL_MASK                    (0x00000003) /* <0:1> R:RW:0:IO0 */
#define GPIO_PRT_INTR_CFG_EDGE0_SEL_POS                     (0)


/*
 * Sets which edge will trigger an IRQ for IO pad 1.
 */
#define GPIO_PRT_INTR_CFG_EDGE1_SEL_MASK                    (0x0000000c) /* <2:3> R:RW:0:IO1 */
#define GPIO_PRT_INTR_CFG_EDGE1_SEL_POS                     (2)


/*
 * Sets which edge will trigger an IRQ for IO pad 2.
 */
#define GPIO_PRT_INTR_CFG_EDGE2_SEL_MASK                    (0x00000030) /* <4:5> R:RW:0:IO2 */
#define GPIO_PRT_INTR_CFG_EDGE2_SEL_POS                     (4)


/*
 * Sets which edge will trigger an IRQ for IO pad 3.
 */
#define GPIO_PRT_INTR_CFG_EDGE3_SEL_MASK                    (0x000000c0) /* <6:7> R:RW:0:IO3 */
#define GPIO_PRT_INTR_CFG_EDGE3_SEL_POS                     (6)


/*
 * Sets which edge will trigger an IRQ for IO pad 4.
 */
#define GPIO_PRT_INTR_CFG_EDGE4_SEL_MASK                    (0x00000300) /* <8:9> R:RW:0:IO4 */
#define GPIO_PRT_INTR_CFG_EDGE4_SEL_POS                     (8)


/*
 * Sets which edge will trigger an IRQ for IO pad 5.
 */
#define GPIO_PRT_INTR_CFG_EDGE5_SEL_MASK                    (0x00000c00) /* <10:11> R:RW:0:IO5 */
#define GPIO_PRT_INTR_CFG_EDGE5_SEL_POS                     (10)


/*
 * Sets which edge will trigger an IRQ for IO pad 6.
 */
#define GPIO_PRT_INTR_CFG_EDGE6_SEL_MASK                    (0x00003000) /* <12:13> R:RW:0:IO6 */
#define GPIO_PRT_INTR_CFG_EDGE6_SEL_POS                     (12)


/*
 * Sets which edge will trigger an IRQ for IO pad 7.
 */
#define GPIO_PRT_INTR_CFG_EDGE7_SEL_MASK                    (0x0000c000) /* <14:15> R:RW:0:IO7 */
#define GPIO_PRT_INTR_CFG_EDGE7_SEL_POS                     (14)


/*
 * Same for the glitch filtered pin (selected by FLT_SELECT).
 */
#define GPIO_PRT_INTR_CFG_FLT_EDGE_SEL_MASK                 (0x00030000) /* <16:17> R:RW:0: */
#define GPIO_PRT_INTR_CFG_FLT_EDGE_SEL_POS                  (16)


/*
 * Selects which pin is routed through the 50ns glitch filter to provide
 * a glitch-safe interrupt.
 */
#define GPIO_PRT_INTR_CFG_FLT_SEL_MASK                      (0x001c0000) /* <18:20> R:RW:0: */
#define GPIO_PRT_INTR_CFG_FLT_SEL_POS                       (18)


/*
 * Port interrupt status register
 * An interrupt cause is cleared (set to '0') by writing a '1' to the corresponding
 * bit field. It is not recommended to write 0xff to clear all interrupt
 * causes, as a new interrupt cause may have occurred between reading the
 * register and clearing. Note that the interrupt cause fields and the associated
 * interrupt provide Hibernate functionality (interrupt causes can be set
 * to '1' and the interrupt can be activated in Hibernate power mode). The
 * PS_DATA fields reflect the logical IO pad states of the port (also found
 * in the PS register).
 */
#define GPIO_PRT_INTR_ADDRESS                               (0x40040010)
#define GPIO_PRT_INTR                                       (*(volatile uint32_t *)(0x40040010))
#define GPIO_PRT_INTR_DEFAULT                               (0x00000000)

/*
 * Interrupt pending on IO pad 0. Firmware writes 1 to clear the interrupt.
 */
#define GPIO_PRT_INTR_DATA0                                 (1u << 0) /* <0:0> A:RW1C:0:IO0 */


/*
 * Interrupt pending on IO pad 1. Firmware writes 1 to clear the interrupt.
 */
#define GPIO_PRT_INTR_DATA1                                 (1u << 1) /* <1:1> A:RW1C:0:IO1 */


/*
 * Interrupt pending on IO pad 2. Firmware writes 1 to clear the interrupt.
 */
#define GPIO_PRT_INTR_DATA2                                 (1u << 2) /* <2:2> A:RW1C:0:IO2 */


/*
 * Interrupt pending on IO pad 3. Firmware writes 1 to clear the interrupt.
 */
#define GPIO_PRT_INTR_DATA3                                 (1u << 3) /* <3:3> A:RW1C:0:IO3 */


/*
 * Interrupt pending on IO pad 4. Firmware writes 1 to clear the interrupt.
 */
#define GPIO_PRT_INTR_DATA4                                 (1u << 4) /* <4:4> A:RW1C:0:IO4 */


/*
 * Interrupt pending on IO pad 5. Firmware writes 1 to clear the interrupt.
 */
#define GPIO_PRT_INTR_DATA5                                 (1u << 5) /* <5:5> A:RW1C:0:IO5 */


/*
 * Interrupt pending on IO pad 6. Firmware writes 1 to clear the interrupt.
 */
#define GPIO_PRT_INTR_DATA6                                 (1u << 6) /* <6:6> A:RW1C:0:IO6 */


/*
 * Interrupt pending on IO pad 7. Firmware writes 1 to clear the interrupt.
 */
#define GPIO_PRT_INTR_DATA7                                 (1u << 7) /* <7:7> A:RW1C:0:IO7 */


/*
 * Deglitched interrupt pending (selected by FLT_SELECT).
 */
#define GPIO_PRT_INTR_FLT_DATA                              (1u << 8) /* <8:8> A:RW1C:0: */


/*
 * `
 */
#define GPIO_PRT_INTR_PS_DATA0                              (1u << 16) /* <16:16> W:R:0:IO0 */


#define GPIO_PRT_INTR_PS_DATA1                              (1u << 17) /* <17:17> W:R:0:IO1 */


#define GPIO_PRT_INTR_PS_DATA2                              (1u << 18) /* <18:18> W:R:0:IO2 */


#define GPIO_PRT_INTR_PS_DATA3                              (1u << 19) /* <19:19> W:R:0:IO3 */


#define GPIO_PRT_INTR_PS_DATA4                              (1u << 20) /* <20:20> W:R:0:IO4 */


#define GPIO_PRT_INTR_PS_DATA5                              (1u << 21) /* <21:21> W:R:0:IO5 */


#define GPIO_PRT_INTR_PS_DATA6                              (1u << 22) /* <22:22> W:R:0:IO6 */


#define GPIO_PRT_INTR_PS_DATA7                              (1u << 23) /* <23:23> W:R:0:IO7 */


/*
 * This is a duplicate of the contents of the PS register, provided here
 * to allow reading of both pin state and interrupt state of the port in
 * a single read operation.
 */
#define GPIO_PRT_INTR_PS_FLT_DATA                           (1u << 24) /* <24:24> W:R:0: */


/*
 * Port configuration register 2
 * Configures the input disable for each pin.
 */
#define GPIO_PRT_PC2_ADDRESS                                (0x40040018)
#define GPIO_PRT_PC2                                        (*(volatile uint32_t *)(0x40040018))
#define GPIO_PRT_PC2_DEFAULT                                (0x00000000)

/*
 * Disables the input buffer for IO pad 0 independent of the port control
 * drive mode (PC.DM). This bit should be set when analog signals are present
 * on the pin and PC.DM != 0 is required to use the output driver.
 */
#define GPIO_PRT_PC2_INP_DIS0                               (1u << 0) /* <0:0> R:RW:0:IO0 */


/*
 * Disables the input buffer for IO pad 1.
 */
#define GPIO_PRT_PC2_INP_DIS1                               (1u << 1) /* <1:1> R:RW:0:IO1 */


/*
 * Disables the input buffer for IO pad 2.
 */
#define GPIO_PRT_PC2_INP_DIS2                               (1u << 2) /* <2:2> R:RW:0:IO2 */


/*
 * Disables the input buffer for IO pad 3.
 */
#define GPIO_PRT_PC2_INP_DIS3                               (1u << 3) /* <3:3> R:RW:0:IO3 */


/*
 * Disables the input buffer for IO pad 4.
 */
#define GPIO_PRT_PC2_INP_DIS4                               (1u << 4) /* <4:4> R:RW:0:IO4 */


/*
 * Disables the input buffer for IO pad 5.
 */
#define GPIO_PRT_PC2_INP_DIS5                               (1u << 5) /* <5:5> R:RW:0:IO5 */


/*
 * Disables the input buffer for IO pad 6.
 */
#define GPIO_PRT_PC2_INP_DIS6                               (1u << 6) /* <6:6> R:RW:0:IO6 */


/*
 * Disables the input buffer for IO pad 7.
 */
#define GPIO_PRT_PC2_INP_DIS7                               (1u << 7) /* <7:7> R:RW:0:IO7 */


/*
 * Port output data set register
 * Used to set output data of specific IO pads in the corresponding port
 * to '1', without affecting the output data of the other IO pads in the
 * port. A DR_SET register read returns the same value as a DR register read.
 */
#define GPIO_PRT_DR_SET_ADDRESS                             (0x40040040)
#define GPIO_PRT_DR_SET                                     (*(volatile uint32_t *)(0x40040040))
#define GPIO_PRT_DR_SET_DEFAULT                             (0x00000000)

/*
 * IO pad i:
 * '0': Output state DR.DATA[i] not affected.
 * '1': Output state DR.DATA[i] set to '1'.
 */
#define GPIO_PRT_DR_SET_DATA_MASK                           (0x000000ff) /* <0:7> A:RW:0: */
#define GPIO_PRT_DR_SET_DATA_POS                            (0)


/*
 * Port output data clear register
 * Used to clear output data of specific IO pads in the corresponding port
 * to '0', without affecting the output data of the other IO pads in the
 * port. A DR_CLR register read returns the same value as a DR register read.
 */
#define GPIO_PRT_DR_CLR_ADDRESS                             (0x40040044)
#define GPIO_PRT_DR_CLR                                     (*(volatile uint32_t *)(0x40040044))
#define GPIO_PRT_DR_CLR_DEFAULT                             (0x00000000)

/*
 * IO pad i:
 * '0': Output state DR.DATA[i] not affected.
 * '1': Output state DR.DATA[i] set to '0'.
 */
#define GPIO_PRT_DR_CLR_DATA_MASK                           (0x000000ff) /* <0:7> A:RW:0: */
#define GPIO_PRT_DR_CLR_DATA_POS                            (0)


/*
 * Port output data invert register
 * Used to invert output data of specific IO pads in the corresponding port,
 * without affecting the output data of the other IO pads in the port. A
 * DR_INV register read returns the same value as a DR register read.
 */
#define GPIO_PRT_DR_INV_ADDRESS                             (0x40040048)
#define GPIO_PRT_DR_INV                                     (*(volatile uint32_t *)(0x40040048))
#define GPIO_PRT_DR_INV_DEFAULT                             (0x00000000)

/*
 * IO pad i:
 * '0': Output state DR.DATA[i] not affected.
 * '1': Output state DR.DATA[i] inverted ('0' => '1', '1' => '0').
 */
#define GPIO_PRT_DR_INV_DATA_MASK                           (0x000000ff) /* <0:7> A:RW:0: */
#define GPIO_PRT_DR_INV_DATA_POS                            (0)


/*
 * Interrupt port cause register
 */
#define GPIO_INTR_CAUSE_ADDRESS                             (0x40041000)
#define GPIO_INTR_CAUSE                                     (*(volatile uint32_t *)(0x40041000))
#define GPIO_INTR_CAUSE_DEFAULT                             (0x00000000)

/*
 * Each IO port has an associated bit field in this register. The bit field
 * reflects the IO port's interrupt line (bit field i reflects "gpio_interrupts[i]"
 * for IO port i). The register is used when the system uses a shared/combined
 * interrupt line "gpio_interrupt". The SW ISR reads the register to deternine
 * which IO port(s) is responsible for the shared/combined interrupt line
 * "gpio_interrupt". Once, the IO port(s) is determined, the IO port's INTR
 * register is read to determine the IO pad(s) in the IO port that caused
 * the interrupt.
 */
#define GPIO_INTR_CAUSE_PORT_INT                            (1u << 0) /* <0:0> W:R:0:GPIO_PORT_NR */


/*
 * IO SELF TEST control register for DfT purposes only
 * This register is used to significantly reduce the test time for IO cells.
 * It also avoids the need to develop a large amount of chip specific functional
 * test vectors.
 * With ATPG it is not possible to get full stuck-at fault coverage for some
 * IO cell inputs (“hld_ovr, oe_n, analog_en, analog_sel, analog_pol”). This
 * register gives direct controlabilty on those  inputs of all IO cells by
 * bypassing the functional paths. That allows generic (not chip specific)
 * ROOS code (also SWD IO cells are included) to get DfT fault-coverage for
 * these signals. This register is used in conjunction with the GPIO.PC.dm[2:0],
 * GPIO.DR.out and SRSS.CORE.PWR_STOP.FREEZE for control and results are
 * observed through GPIO.PS.data.
 * Only one IO cell on the whole chip gets IO_TEST_0  and only one gets IO_TEST_1
 * and default values of IO_TEST_0 is '0' and IO_TEST_1 is '1'. Also only
 * one IO cell on the whole chip gets asssigned ADFT-0 and only one gets
 * asssigned ADFT-1. All four IO_TEST_0/1 and ADFT-0/1 pins are assigned
 * in the product pin spreadsheet.
 */
#define GPIO_DFT_IO_TEST_ADDRESS                            (0x40041010)
#define GPIO_DFT_IO_TEST                                    (*(volatile uint32_t *)(0x40041010))
#define GPIO_DFT_IO_TEST_DEFAULT                            (0x02020200)

/*
 * DfT IO SELF TEST mode:
 */
#define GPIO_DFT_IO_TEST_DFT_IO_TEST_MODE_MASK              (0x00000003) /* <0:1> R:RW:0: */
#define GPIO_DFT_IO_TEST_DFT_IO_TEST_MODE_POS               (0)


/*
 * “hld_ovr” DfT control for IO cells depending on DFT_IO_TEST_MODE as given
 * below.
 * TEST_ADFT: Connects to “hld_ovr” of the ADFT-0  assigned IO cell.
 * TEST_ANA: Connects to “hld_ovr” of the IO_TEST_0  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_HLD_OVR_0                      (1u << 8) /* <8:8> R:RW:0: */


/*
 * “oe_n” DfT control for IO cells depending on DFT_IO_TEST_MODE as given
 * below.
 * TEST_ADFT: Connects to “oe_n” of the ADFT-0  assigned IO cell.
 * TEST_ANA: Connects to “oe_n” of the IO_TEST_0  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_OE_N_0                         (1u << 9) /* <9:9> R:RW:1: */


/*
 * “analog_en” DfT control for IO cells depending on DFT_IO_TEST_MODE as
 * given below.
 * TEST_ADFT: Connects to “analog_en” of the ADFT-0  assigned IO cell.
 * TEST_ANA: Connects to “analog_en” of the IO_TEST_0  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_ANALOG_EN_0                    (1u << 10) /* <10:10> R:RW:0: */


/*
 * “analog_sel” DfT control for IO cells depending on DFT_IO_TEST_MODE as
 * given below.
 * TEST_ADFT: Connects to “analog_sel” of the ADFT-0  assigned IO cell.
 * TEST_ANA: Connects to “analog_sel” of the IO_TEST_0  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_ANA_SEL_0                      (1u << 11) /* <11:11> R:RW:0: */


/*
 * “analog_pol” DfT control for IO cells depending on DFT_IO_TEST_MODE as
 * given below.
 * TEST_ADFT: Connects to “analog_pol” of the ADFT-0  assigned IO cell.
 * TEST_ANA: Connects to “analog_pol” of the IO_TEST_0  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_ANA_POL_0                      (1u << 12) /* <12:12> R:RW:0: */


/*
 * “hld_ovr” DfT control for IO cells depending on DFT_IO_TEST_MODE as given
 * below.
 * TEST_ADFT: Connects to “hld_ovr” of the ADFT-1  assigned IO cell.
 * TEST_ANA: Connects to “hld_ovr” of the IO_TEST_1  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_HLD_OVR_1                      (1u << 16) /* <16:16> R:RW:0: */


/*
 * “oe_n” DfT control for IO cells depending on DFT_IO_TEST_MODE as given
 * below.
 * TEST_ADFT: Connects to “oe_n” of the ADFT-1  assigned IO cell.
 * TEST_ANA: Connects to “oe_n” of the IO_TEST_1  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_OE_N_1                         (1u << 17) /* <17:17> R:RW:1: */


/*
 * “analog_en” DfT control for IO cells depending on DFT_IO_TEST_MODE as
 * given below.
 * TEST_ADFT: Connects to “analog_en” of the ADFT-1  assigned IO cell.
 * TEST_ANA: Connects to “analog_en” of the IO_TEST_1  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_ANALOG_EN_1                    (1u << 18) /* <18:18> R:RW:0: */


/*
 * “analog_sel” DfT control for IO cells depending on DFT_IO_TEST_MODE as
 * given below.
 * TEST_ADFT: Connects to “analog_sel” of the ADFT-1  assigned IO cell.
 * TEST_ANA: Connects to “analog_sel” of the IO_TEST_1  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_ANA_SEL_1                      (1u << 19) /* <19:19> R:RW:0: */


/*
 * “analog_pol” DfT control for IO cells depending on DFT_IO_TEST_MODE as
 * given below.
 * TEST_ADFT: Connects to “analog_pol” of the ADFT-1  assigned IO cell.
 * TEST_ANA: Connects to “analog_pol” of the IO_TEST_1  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_ANA_POL_1                      (1u << 20) /* <20:20> R:RW:0: */


/*
 * “hld_ovr” DfT control for IO cells depending on DFT_IO_TEST_MODE as given
 * below.
 * TEST_ADFT: Connects to “hld_ovr” of all IO cells other than ADFT-0/1 .
 * TEST_ANA: Connects to “hld_ovr” of all IO cells other than IO_TEST_0/1.
 * TEST_GEN: Connects to “hld_ovr” of all IO cells.
 */
#define GPIO_DFT_IO_TEST_DFT_HLD_OVR_2                      (1u << 24) /* <24:24> R:RW:0: */


/*
 * “oe_n” DfT control for IO cells depending on DFT_IO_TEST_MODE as given
 * below.
 * TEST_ADFT: Connects to “oe_n” of all IO cells other than ADFT-0/1.
 * TEST_ANA: Connects to “oe_n” of all IO cells other than IO_TEST_0/1.
 * TEST_GEN: Connects to “oe_n” of all IO cells
 */
#define GPIO_DFT_IO_TEST_DFT_OE_N_2                         (1u << 25) /* <25:25> R:RW:1: */


/*
 * “analog_en” DfT control for IO cells depending on DFT_IO_TEST_MODE as
 * given below.
 * TEST_ADFT: Connects to “analog_en” of all IO cells other than ADFT-0/1.
 * TEST_ANA: DFT_ANALOG_EN_2 && DM[0] connects to “analog_en” of all IO cells
 * other than IO_TEST_0/1.
 * TEST_GEN: Connects to “analog_en” of all IO cells
 */
#define GPIO_DFT_IO_TEST_DFT_ANALOG_EN_2                    (1u << 26) /* <26:26> R:RW:0: */


/*
 * “analog_sel” DfT control for IO cells depending on DFT_IO_TEST_MODE as
 * given below.
 * TEST_ADFT: Connects to “analog_sel” of all IO cells other than ADFT-0/1.
 * TEST_ANA: Connects to “analog_sel” of all IO cells other than IO_TEST_0/1.
 * TEST_GEN: Connects to “analog_sel” of all IO cells.
 */
#define GPIO_DFT_IO_TEST_DFT_ANA_SEL_2                      (1u << 27) /* <27:27> R:RW:0: */


/*
 * “analog_pol” DfT control for IO cells depending on DFT_IO_TEST_MODE as
 * given below.
 * TEST_ADFT: Connects to “analog_pol” of all IO cells other than ADFT-0/1.
 * TEST_ANA: Connects to “analog_pol” of all IO cells other than IO_TEST_0/1.
 * TEST_GEN: Connects to “analog_pol” of all IO cells.
 */
#define GPIO_DFT_IO_TEST_DFT_ANA_POL_2                      (1u << 28) /* <28:28> R:RW:0: */


/*
 * TCPWM control register 0.
 * Used to disbale/enable the counters.
 */
#define TCPWM_CTRL_ADDRESS                                  (0x40090000)
#define TCPWM_CTRL                                          (*(volatile uint32_t *)(0x40090000))
#define TCPWM_CTRL_DEFAULT                                  (0x00000000)

/*
 * Counter enables for counters 0 up to CNT_NR-1.
 * '0': counter disabled.
 * '1': counter enabled.
 * Counter static configuration information (e.g. CTRL.MODE, all TR_CTRL0,
 * TR_CTRL1, and TR_CTRL2 register fields) should only be modified when the
 * counter is disabled. When a counter is disabled, command and status information
 * associated to the counter is cleared by HW, this includes:
 * - the associated counter triggers in the CMD register are set to '0'.
 * - the counter's interrupt cause fields in counter's INTR register.
 * - the counter's status fields in counter's STATUS register..
 * - the counter's trigger outputs ("tr_overflow", "tr_underflow" and "tr_compare_match").
 * - the counter's line outputs ("line_out" and "line_compl_out").
 */
#define TCPWM_CTRL_COUNTER_ENABLED                          (1u << 0) /* <0:0> R:RW:0:CNT_NR */


/*
 * TCPWM command register.
 * Enables software controlled counter operation.
 */
#define TCPWM_CMD_ADDRESS                                   (0x40090008)
#define TCPWM_CMD                                           (*(volatile uint32_t *)(0x40090008))
#define TCPWM_CMD_DEFAULT                                   (0x00000000)

/*
 * Counters SW capture trigger. When written with '1', a capture trigger
 * is generated and the HW sets the field to '0' when the SW trigger has
 * taken effect. It should be noted that the HW operates on the counter frequency.
 * If the counter is disabled through CTRL.COUNTER_ENABLED, the field is
 * immediately set to '0'.
 */
#define TCPWM_CMD_COUNTER_CAPTURE                           (1u << 0) /* <0:0> RW1C:RW1S:0:CNT_NR */


/*
 * Counters SW reload trigger. For HW behavior, see COUNTER_CAPTURE field.
 */
#define TCPWM_CMD_COUNTER_RELOAD                            (1u << 8) /* <8:8> RW1C:RW1S:0:CNT_NR */


/*
 * Counters SW stop trigger. For HW behavior, see COUNTER_CAPTURE field.
 */
#define TCPWM_CMD_COUNTER_STOP                              (1u << 16) /* <16:16> RW1C:RW1S:0:CNT_NR */


/*
 * Counters SW start trigger. For HW behavior, see COUNTER_CAPTURE field.
 */
#define TCPWM_CMD_COUNTER_START                             (1u << 24) /* <24:24> RW1C:RW1S:0:CNT_NR */


/*
 * TCPWM Counter interrupt cause register.
 * Enables software to determine the source of the combined interrupt output
 * signal "interrupt". The register fields are not retained. This is to ensure
 * that they come up as '0' after coming out of DeepSleep system power mode.
 */
#define TCPWM_INTR_CAUSE_ADDRESS                            (0x4009000c)
#define TCPWM_INTR_CAUSE                                    (*(volatile uint32_t *)(0x4009000c))
#define TCPWM_INTR_CAUSE_DEFAULT                            (0x00000000)

/*
 * Counters interrupt signal active. If the counter is disabled through CTRL.COUNTER_ENABLED,
 * the associated interrupt field is immediately set to '0'.
 */
#define TCPWM_INTR_CAUSE_COUNTER_INT                        (1u << 0) /* <0:0> W:R:0:CNT_NR */


/*
 * Counter control register
 */
#define TCPWM_CNT_CTRL_ADDRESS                              (0x40090100)
#define TCPWM_CNT_CTRL                                      (*(volatile uint32_t *)(0x40090100))
#define TCPWM_CNT_CTRL_DEFAULT                              (0x00000000)

/*
 * Specifies switching of the CC and buffered CC values. This field has a
 * function in TIMER, PWM, PWM_DT and PWM_PR modes.
 * Timer mode:
 * '0': never switch.
 * '1': switch on a compare match event.
 * PWM, PWM_DT, PWM_PR modes:
 * '0: never switch.
 * '1': switch on a terminal count event with an actively pending switch
 * event.
 */
#define TCPWM_CNT_CTRL_AUTO_RELOAD_CC                       (1u << 0) /* <0:0> R:RW:0: */


/*
 * Specifies switching of the PERIOD and buffered PERIOD values. This field
 * has a function in PWM, PWM_DT and PWM_PR modes.
 * '0': never switch.
 * '1': switch on a terminal count event with and actively pending siwtch
 * event.
 */
#define TCPWM_CNT_CTRL_AUTO_RELOAD_PERIOD                   (1u << 1) /* <1:1> R:RW:0: */


/*
 * Specifies asynchronous/synchronous kill behavior:
 * '1': synchronous kill mode: the kill event disables the "dt_line_out"
 * and "dt_line_compl_out" signals till the next terminal count event (synchronous
 * kill). In synchronous kill mode, STOP_EDGE should  be RISING_EDGE.
 * '0': asynchronous kill mode: the kill event only disables the "dt_line_out"
 * and "dt_line_compl_out" signals when present. In asynchronous kill mode,
 * STOP_EDGE should be NO_EDGE_DET.
 *
 * This field has a function in PWM and PWM_DT modes only. This field is
 * only used when PWM_STOP_ON_KILL is '0'.
 */
#define TCPWM_CNT_CTRL_PWM_SYNC_KILL                        (1u << 2) /* <2:2> R:RW:0: */


/*
 * Specifies whether the counter stops on a kill events:
 * '0': kill event does NOT stop counter.
 * '1': kill event stops counter.
 *
 * This field has a function in PWM, PWM_DT and PWM_PR modes only.
 */
#define TCPWM_CNT_CTRL_PWM_STOP_ON_KILL                     (1u << 3) /* <3:3> R:RW:0: */


/*
 * Generic 8-bit control field. In PWM_DT mode, this field is used to determine
 * the dead time: amount of dead time cycles in the counter clock domain.
 * In all other modes, the lower 3 bits of this field determine pre-scaling
 * of the selected counter clock.
 */
#define TCPWM_CNT_CTRL_GENERIC_MASK                         (0x0000ff00) /* <8:15> R:RW:0: */
#define TCPWM_CNT_CTRL_GENERIC_POS                          (8)


/*
 * Determines counter direction.
 */
#define TCPWM_CNT_CTRL_UP_DOWN_MODE_MASK                    (0x00030000) /* <16:17> R:RW:0: */
#define TCPWM_CNT_CTRL_UP_DOWN_MODE_POS                     (16)


/*
 * When '0', counter runs continuous. When '1', counter is turned off by
 * hardware when a terminal count event is generated.
 */
#define TCPWM_CNT_CTRL_ONE_SHOT                             (1u << 18) /* <18:18> R:RW:0: */


/*
 * In QUAD mode selects quadrature encoding mode (X1/X2/X4).
 * In PWM, PWM_DT and PWM_PR modes, these two bits can be used to invert
 * "dt_line_out" and "dt_line_compl_out".  Inversion is the last step in
 * generation of "dt_line_out" and "dt_line_compl_out"; i.e. a disabled output
 * line "dt_line_out" has the value QUADRATURE_MODE[0] and a disabled output
 * line "dt_line_compl_out" has the value QUADRATURE_MODE[1].
 */
#define TCPWM_CNT_CTRL_QUADRATURE_MODE_MASK                 (0x00300000) /* <20:21> R:RW:0: */
#define TCPWM_CNT_CTRL_QUADRATURE_MODE_POS                  (20)


/*
 * Counter mode.
 */
#define TCPWM_CNT_CTRL_MODE_MASK                            (0x07000000) /* <24:26> R:RW:0: */
#define TCPWM_CNT_CTRL_MODE_POS                             (24)


/*
 * Counter status register
 */
#define TCPWM_CNT_STATUS_ADDRESS                            (0x40090104)
#define TCPWM_CNT_STATUS                                    (*(volatile uint32_t *)(0x40090104))
#define TCPWM_CNT_STATUS_DEFAULT                            (0x00000000)

/*
 * When '0', counter is counting up. When '1', counter is counting down.
 * In QUAD mode, this field indicates the direction of the latest counter
 * change: '0' when last incremented and '1' when last decremented.
 */
#define TCPWM_CNT_STATUS_DOWN                               (1u << 0) /* <0:0> RW:R:0: */


/*
 * Generic 8-bit counter field. In PWM_DT mode, this counter is used for
 * dead time insertion. In all other modes, this counter is used for pre-scaling
 * the selected counter clock. PWM_DT mode can NOT use prescaled clock functionality.
 */
#define TCPWM_CNT_STATUS_GENERIC_MASK                       (0x0000ff00) /* <8:15> RW:R:0: */
#define TCPWM_CNT_STATUS_GENERIC_POS                        (8)


/*
 * When '0', the counter is NOT running. When '1', the counter is running.
 */
#define TCPWM_CNT_STATUS_RUNNING                            (1u << 31) /* <31:31> RW:R:0: */


/*
 * Counter count register
 */
#define TCPWM_CNT_COUNTER_ADDRESS                           (0x40090108)
#define TCPWM_CNT_COUNTER                                   (*(volatile uint32_t *)(0x40090108))
#define TCPWM_CNT_COUNTER_DEFAULT                           (0x00000000)

/*
 * 16-bit counter value. It is advised to not write to this field when the
 * counter is running.
 */
#define TCPWM_CNT_COUNTER_COUNTER_MASK                      (0x0000ffff) /* <0:15> RW:RW:0: */
#define TCPWM_CNT_COUNTER_COUNTER_POS                       (0)


/*
 * Counter compare/capture register
 */
#define TCPWM_CNT_CC_ADDRESS                                (0x4009010c)
#define TCPWM_CNT_CC                                        (*(volatile uint32_t *)(0x4009010c))
#define TCPWM_CNT_CC_DEFAULT                                (0x0000ffff)

/*
 * In CAPTURE mode, captures the counter value. In other modes, compared
 * to counter value.
 */
#define TCPWM_CNT_CC_CC_MASK                                (0x0000ffff) /* <0:15> RW:RW:65535: */
#define TCPWM_CNT_CC_CC_POS                                 (0)


/*
 * Counter buffered compare/capture register
 */
#define TCPWM_CNT_CC_BUFF_ADDRESS                           (0x40090110)
#define TCPWM_CNT_CC_BUFF                                   (*(volatile uint32_t *)(0x40090110))
#define TCPWM_CNT_CC_BUFF_DEFAULT                           (0x0000ffff)

/*
 * Additional buffer for counter CC register.
 */
#define TCPWM_CNT_CC_BUFF_CC_MASK                           (0x0000ffff) /* <0:15> RW:RW:65535: */
#define TCPWM_CNT_CC_BUFF_CC_POS                            (0)


/*
 * Counter period register
 */
#define TCPWM_CNT_PERIOD_ADDRESS                            (0x40090114)
#define TCPWM_CNT_PERIOD                                    (*(volatile uint32_t *)(0x40090114))
#define TCPWM_CNT_PERIOD_DEFAULT                            (0x0000ffff)

/*
 * Period value: upper value of the counter. When the counter should count
 * for n cycles, this field should be set to n-1.
 */
#define TCPWM_CNT_PERIOD_PERIOD_MASK                        (0x0000ffff) /* <0:15> RW:RW:65535: */
#define TCPWM_CNT_PERIOD_PERIOD_POS                         (0)


/*
 * Counter buffered period register
 */
#define TCPWM_CNT_PERIOD_BUFF_ADDRESS                       (0x40090118)
#define TCPWM_CNT_PERIOD_BUFF                               (*(volatile uint32_t *)(0x40090118))
#define TCPWM_CNT_PERIOD_BUFF_DEFAULT                       (0x0000ffff)

/*
 * Additional buffer for counter PERIOD register.
 */
#define TCPWM_CNT_PERIOD_BUFF_PERIOD_MASK                   (0x0000ffff) /* <0:15> RW:RW:65535: */
#define TCPWM_CNT_PERIOD_BUFF_PERIOD_POS                    (0)


/*
 * Counter trigger control register 0
 * Used to select triggers for specific counter events.
 */
#define TCPWM_CNT_TR_CTRL0_ADDRESS                          (0x40090120)
#define TCPWM_CNT_TR_CTRL0                                  (*(volatile uint32_t *)(0x40090120))
#define TCPWM_CNT_TR_CTRL0_DEFAULT                          (0x00000010)

/*
 * Selects one of the 16 input triggers as a capture trigger. Input trigger
 * 0 is always '0' and input trigger is always '1'. In the PWM, PWM_DT and
 * PWM_PR modes this trigger is used to switch the values if the compare
 * and period registers with their buffer counterparts.
 */
#define TCPWM_CNT_TR_CTRL0_CAPTURE_SEL_MASK                 (0x0000000f) /* <0:3> R:RW:0: */
#define TCPWM_CNT_TR_CTRL0_CAPTURE_SEL_POS                  (0)


/*
 * Selects one of the 16 input triggers as a count trigger. In QUAD mode,
 * this is the first phase (phi A). Default setting selects input trigger
 * 1, which is always '1'.
 */
#define TCPWM_CNT_TR_CTRL0_COUNT_SEL_MASK                   (0x000000f0) /* <4:7> R:RW:1: */
#define TCPWM_CNT_TR_CTRL0_COUNT_SEL_POS                    (4)


/*
 * Selects one of the 16 input triggers as a reload trigger. In QUAD mode,
 * this is the index or revolution pulse. In this mode, it will update the
 * counter with the value in the TCPWM_CNTn_PERIOD register.
 */
#define TCPWM_CNT_TR_CTRL0_RELOAD_SEL_MASK                  (0x00000f00) /* <8:11> R:RW:0: */
#define TCPWM_CNT_TR_CTRL0_RELOAD_SEL_POS                   (8)


/*
 * Selects one of the 16 input triggers as a stop trigger. In PWM, PWM_DT
 * and PWM_PR modes, this is the kill trigger. In these modes, the kill trigger
 * is used to either temporarily block the PWM outputs (PWM_STOP_ON_KILL
 * is '0') or stop the functionality (PWM_STOP_ON_KILL is '1'). For the PWM
 * and PWM_DT modes, the blocking of the output signals can be  asynchronous
 * (STOP_EDGE should be NO_EDGE_DET) in which case the blocking is as long
 * as the trigger is '1' or synchronous (STOP_EDGE should be RISING_EDGE)
 * in which case it extends till the next terminal count event.
 */
#define TCPWM_CNT_TR_CTRL0_STOP_SEL_MASK                    (0x0000f000) /* <12:15> R:RW:0: */
#define TCPWM_CNT_TR_CTRL0_STOP_SEL_POS                     (12)


/*
 * Selects one of the 16 input triggers as a start trigger. In QUAD mode,
 * this is the second phase (phi B).
 */
#define TCPWM_CNT_TR_CTRL0_START_SEL_MASK                   (0x000f0000) /* <16:19> R:RW:0: */
#define TCPWM_CNT_TR_CTRL0_START_SEL_POS                    (16)


/*
 * Counter trigger control register 1
 * Used to determine edge detection for specific counter triggers. Events
 * will only take effect on enabled counters.
 */
#define TCPWM_CNT_TR_CTRL1_ADDRESS                          (0x40090124)
#define TCPWM_CNT_TR_CTRL1                                  (*(volatile uint32_t *)(0x40090124))
#define TCPWM_CNT_TR_CTRL1_DEFAULT                          (0x000003ff)

/*
 * A capture event will copy the counter value into the CC register.
 */
#define TCPWM_CNT_TR_CTRL1_CAPTURE_EDGE_MASK                (0x00000003) /* <0:1> R:RW:3: */
#define TCPWM_CNT_TR_CTRL1_CAPTURE_EDGE_POS                 (0)


/*
 * A counter event will increase or decrease the counter by '1'.
 */
#define TCPWM_CNT_TR_CTRL1_COUNT_EDGE_MASK                  (0x0000000c) /* <2:3> R:RW:3: */
#define TCPWM_CNT_TR_CTRL1_COUNT_EDGE_POS                   (2)


/*
 * A reload event will initialize the counter. When counting up, the counter
 * is initialized to "0". When counting down, the counter is initialized
 * with PERIOD.
 */
#define TCPWM_CNT_TR_CTRL1_RELOAD_EDGE_MASK                 (0x00000030) /* <4:5> R:RW:3: */
#define TCPWM_CNT_TR_CTRL1_RELOAD_EDGE_POS                  (4)


/*
 * A stop event, will stop the counter; i.e. it will no longer be running.
 * Stopping will NOT disable the counter.
 */
#define TCPWM_CNT_TR_CTRL1_STOP_EDGE_MASK                   (0x000000c0) /* <6:7> R:RW:3: */
#define TCPWM_CNT_TR_CTRL1_STOP_EDGE_POS                    (6)


/*
 * A start event will start the counter; i.e. the counter will become running.
 * Starting does NOT enable the counter. A start event will not initialize
 * the counter whereas the reload event does.
 */
#define TCPWM_CNT_TR_CTRL1_START_EDGE_MASK                  (0x00000300) /* <8:9> R:RW:3: */
#define TCPWM_CNT_TR_CTRL1_START_EDGE_POS                   (8)


/*
 * Counter trigger control register 2
 * Used to control counter "line_out", "dt_line_out" and "dt_line_compl_out"
 * output signals.
 */
#define TCPWM_CNT_TR_CTRL2_ADDRESS                          (0x40090128)
#define TCPWM_CNT_TR_CTRL2                                  (*(volatile uint32_t *)(0x40090128))
#define TCPWM_CNT_TR_CTRL2_DEFAULT                          (0x0000003f)

/*
 * Determines the effect of a compare match event (COUNTER equals CC register)
 * on the "line_out" output signals.  Note that INVERT is especially useful
 * for center aligned pulse width modulation.
 * To generate a duty cycle of 0%, the counter CC register should be set
 * to "0". For a 100% duty cycle, the counter CC register should be set to
 * larger than the counter PERIOD register.
 */
#define TCPWM_CNT_TR_CTRL2_CC_MATCH_MODE_MASK               (0x00000003) /* <0:1> R:RW:3: */
#define TCPWM_CNT_TR_CTRL2_CC_MATCH_MODE_POS                (0)


/*
 * Determines the effect of a counter overflow event (COUNTER reaches PERIOD)
 * on the "line_out" output signals.
 */
#define TCPWM_CNT_TR_CTRL2_OVERFLOW_MODE_MASK               (0x0000000c) /* <2:3> R:RW:3: */
#define TCPWM_CNT_TR_CTRL2_OVERFLOW_MODE_POS                (2)


/*
 * Determines the effect of a counter underflow event (COUNTER reaches "0")
 * on the "line_out" output signals.
 */
#define TCPWM_CNT_TR_CTRL2_UNDERFLOW_MODE_MASK              (0x00000030) /* <4:5> R:RW:3: */
#define TCPWM_CNT_TR_CTRL2_UNDERFLOW_MODE_POS               (4)


/*
 * Interrupt request register.
 * The register fields are not retained. This is to ensure that they come
 * up as '0' after coming out of DeepSleep system power mode. HW clears the
 * interrupt causes to '0', when the counter is disabled.
 */
#define TCPWM_CNT_INTR_ADDRESS                              (0x40090130)
#define TCPWM_CNT_INTR                                      (*(volatile uint32_t *)(0x40090130))
#define TCPWM_CNT_INTR_DEFAULT                              (0x00000000)

/*
 * Terminal count event. Set to '1', when event is detected. Write with '1'
 * to clear bit.
 */
#define TCPWM_CNT_INTR_TC                                   (1u << 0) /* <0:0> RW1S:RW1C:0: */


/*
 * Counter matches CC register event. Set to '1', when event is detected.
 * Write with '1' to clear bit.
 */
#define TCPWM_CNT_INTR_CC_MATCH                             (1u << 1) /* <1:1> RW1S:RW1C:0: */


/*
 * Interrupt set request register.
 * When read, this register reflects the interrupt request register.
 */
#define TCPWM_CNT_INTR_SET_ADDRESS                          (0x40090134)
#define TCPWM_CNT_INTR_SET                                  (*(volatile uint32_t *)(0x40090134))
#define TCPWM_CNT_INTR_SET_DEFAULT                          (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define TCPWM_CNT_INTR_SET_TC                               (1u << 0) /* <0:0> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define TCPWM_CNT_INTR_SET_CC_MATCH                         (1u << 1) /* <1:1> A:RW1S:0: */


/*
 * Interrupt mask register.
 */
#define TCPWM_CNT_INTR_MASK_ADDRESS                         (0x40090138)
#define TCPWM_CNT_INTR_MASK                                 (*(volatile uint32_t *)(0x40090138))
#define TCPWM_CNT_INTR_MASK_DEFAULT                         (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define TCPWM_CNT_INTR_MASK_TC                              (1u << 0) /* <0:0> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define TCPWM_CNT_INTR_MASK_CC_MATCH                        (1u << 1) /* <1:1> R:RW:0: */


/*
 * Interrupt masked request register
 * When read, this register reflects a bitwise AND between the interrupt
 * request and mask registers.
 */
#define TCPWM_CNT_INTR_MASKED_ADDRESS                       (0x4009013c)
#define TCPWM_CNT_INTR_MASKED                               (*(volatile uint32_t *)(0x4009013c))
#define TCPWM_CNT_INTR_MASKED_DEFAULT                       (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define TCPWM_CNT_INTR_MASKED_TC                            (1u << 0) /* <0:0> W:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define TCPWM_CNT_INTR_MASKED_CC_MATCH                      (1u << 1) /* <1:1> W:R:0: */


/*
 * Generic control register.
 */
#define PDSS_CTRL_ADDRESS                                   (0x400a0000)
#define PDSS_CTRL                                           (*(volatile uint32_t *)(0x400a0000))
#define PDSS_CTRL_DEFAULT                                   (0x00000000)

/*
 * Setting this register will bypass 5b/4b, CRC.
 */
#define PDSS_CTRL_TX_BYPASS_EN                              (1u << 0) /* <0:0> R:RW:0:BYPASS_MODE_EN */


/*
 * Setting this register will bypass 5b/4b, CRC.
 */
#define PDSS_CTRL_RX_BYPASS_EN_MASK                         (0x00000006) /* <1:2> R:RW:0:BYPASS_MODE_EN */
#define PDSS_CTRL_RX_BYPASS_EN_POS                          (1)


/*
 * This bit is used to selects which of the clk_filter or clk_lf drivers
 * the High-Speed filters.
 * 0: clk_lf drives the High Speed filters
 * 1: clk_filter drives the High Speed filters
 */
#define PDSS_CTRL_SEL_CLK_FILTER                            (1u << 4) /* <4:4> R:RW:0:HS_CLK_FILT_EN */


/*
 * 0: Clocks is turn off for AFC block
 * 1: Clock is runing in the AFC block
 * This bit must be set after BC1.2 detection for DCP is completed and AFC
 * functionality is required.
 */
#define PDSS_CTRL_AFC_ENABLED                               (1u << 25) /* <25:25> R:RW:0:BCH_DET_NUM */
#define PDSS_CTRL_AFC_ENABLED_POS                           (25u)

/*
 * IP enabled ('1') or not ('0').
 * "0" Resets the IP. The reset is an async reset.
 * Note that when the IP is disabled, all the interrupt sources are also
 * disabled.
 * All the clocks that their source is clk_hf will be turned off when IP
 * is disabled.
 */
#define PDSS_CTRL_IP_ENABLED                                (1u << 31) /* <31:31> R:RW:0: */


/*
 * Header INFO
 */
#define PDSS_HEADER_INFO_ADDRESS                            (0x400a0004)
#define PDSS_HEADER_INFO                                    (*(volatile uint32_t *)(0x400a0004))
#define PDSS_HEADER_INFO_DEFAULT                            (0x03f10f00)

/*
 * This bit will enable/disable extended data messaging.
 * 0: Disable RX extended data messaging
 * 1: Enable  RX extended data messaging
 */
#define PDSS_HEADER_INFO_EN_RX_EXTENDED_DATA                (1u << 0) /* <0:0> R:RW:0: */


/*
 * This bit will enable/disable extended data messaging.
 * 0: Disable TX extended data messaging
 * 1: Enable  TX extended data messaging
 */
#define PDSS_HEADER_INFO_EN_TX_EXTENDED_DATA                (1u << 1) /* <1:1> R:RW:0:EXT_DATA_MSG_EN */


/*
 * The location of the extended data field in the Header[15:0].
 * 0: First Bit of the header
 * 1: Second Bit of the header
 * …
 * 15: 15th bit of theheader
 */
#define PDSS_HEADER_INFO_EXTENDED_DATA_BIT_LOCATION_MASK    (0x00000f00) /* <8:11> R:RW:15:EXT_DATA_MSG_EN */
#define PDSS_HEADER_INFO_EXTENDED_DATA_BIT_LOCATION_POS     (8)


/*
 * The first bit location of the extended data size field in the header.
 * 0: First Bit of the header
 * 1: Second Bit of the header
 * …
 * 31: Thirt first bit of the header
 */
#define PDSS_HEADER_INFO_EXTENDED_DATA_BYTE_FIRST_BIT_LOCATION_MASK    (0x0001f000) /* <12:16> R:RW:16:EXT_DATA_MSG_EN */
#define PDSS_HEADER_INFO_EXTENDED_DATA_BYTE_FIRST_BIT_LOCATION_POS    (12)


/*
 * The first bit location of the extended data size field in the header.
 * 0: First Bit of the header
 * 1: Second Bit of the header
 * …
 * 31: Thirt first bit of the header
 */
#define PDSS_HEADER_INFO_EXTENDED_DATA_BYTE_LAST_BIT_LOCATION_MASK    (0x003e0000) /* <17:21> R:RW:24:EXT_DATA_MSG_EN */
#define PDSS_HEADER_INFO_EXTENDED_DATA_BYTE_LAST_BIT_LOCATION_POS    (17)


/*
 * The location of the chunk field in the Header[31:16].
 * 16: 16th Bit of the header
 * 17: 17th Bit of the header
 * …
 * 31: 31th bit of theheader
 */
#define PDSS_HEADER_INFO_CHUNK_BIT_LOCATION_MASK            (0x03c00000) /* <22:25> R:RW:15:KEEP_REG_BIT */
#define PDSS_HEADER_INFO_CHUNK_BIT_LOCATION_POS             (22)


/*
 * Transmit Header
 */
#define PDSS_TX_HEADER_ADDRESS                              (0x400a000c)
#define PDSS_TX_HEADER                                      (*(volatile uint32_t *)(0x400a000c))
#define PDSS_TX_HEADER_DEFAULT                              (0x00000000)

/*
 * The transmit Header. This register contains the 16-bit(Regular Packet)
 * or 32-bit(Extended) header. Hardware uses this register along with HEADER_INFO
 * register to send either 16-bit (Regular Packet) or 32-bit (Extended Packet)
 * header.
 */
#define PDSS_TX_HEADER_TX_HEADER_MASK                       (0xffffffff) /* <0:31> R:RW:0: */
#define PDSS_TX_HEADER_TX_HEADER_POS                        (0)


/*
 * TX SRAM Data
 * CMG1:  Only one storage element of 32 bytes is available for both transmit
 * and receive.
 *              Only access to address space 0x0010-0x002C mapps to one storage
 * element which is also used in RX direction.
 * Others: The memory for the TX USB power controller is a 64 byte SRAM.
 * This SRAM containts only Data part of a packet in non-bypass mode.
 *              Any access to address space 0x0010 - 0x004C will map to SRAM
 * address x0-x31
 */
#define PDSS_TX_MEM_DATA_ADDRESS(n)                         (0x400a0010 + ((n) * (0x0004)))
#define PDSS_TX_MEM_DATA(n)                                 (*(volatile uint32_t *)(0x400a0010 + ((n) * 0x0004)))
#define PDSS_TX_MEM_DATA_DEFAULT                            (0x00000000)

/*
 * Data information in the transmitter SRAM. SOP/CRC/EOP will be appened
 * by HW.
 * The TX Header needs to be programmed in the TX_HEADER register.
 */
#define PDSS_TX_MEM_DATA_DATA_MASK                          (0xffffffff) /* <0:31> R:RW:0: */
#define PDSS_TX_MEM_DATA_DATA_POS                           (0)


/*
 * Receive Header
 */
#define PDSS_RX_HEADER_ADDRESS                              (0x400a005c)
#define PDSS_RX_HEADER                                      (*(volatile uint32_t *)(0x400a005c))
#define PDSS_RX_HEADER_DEFAULT                              (0x00000000)

/*
 * The receive Header. This register contains the 16-bit(Regular Packet)
 * or 32-bit(Extended) header. The INTR2.EXTENDED_MSG_DET and INTR2.CHUNK_DET
 * interrupts indicates of the Packet type.  Hardware uses the HEADER_INFO
 * register for decoding the incoming packets.
 */
#define PDSS_RX_HEADER_RX_HEADER_MASK                       (0xffffffff) /* <0:31> RW:R:0: */
#define PDSS_RX_HEADER_RX_HEADER_POS                        (0)


/*
 * RX SRAM Data
 * CMG1:  Only one storage element of 32 bytes is available for both transmit
 * and receive.
 *               Only access to address space 0x0060-0x006C mapps to one
 * storage element which is also used in RX direction.
 * Others: The memory for the RX USB power controller is a 64 byte SRAM.
 * This SRAM containts only the Data part of a packet in non-bypass mode.
 *              Any access to address space 0x0060 - 0x009C will map to SRAM
 * address x0-x31
 */
#define PDSS_RX_MEM_DATA_ADDRESS(n)                         (0x400a0060 + ((n) * (0x0004)))
#define PDSS_RX_MEM_DATA(n)                                 (*(volatile uint32_t *)(0x400a0060 + ((n) * 0x0004)))
#define PDSS_RX_MEM_DATA_DEFAULT                            (0x00000000)

/*
 * The Data information in the receive SRAM. SOP type is stored in STATUS.SOP_TYPE_DETECTED
 * register.
 * STATUS.SOP_TYPE_DETECTED contains the SOP type for the packet in the RX
 * SRAM.
 * At the start of every packet, INTR.RCV_PACKET_COMPLETE and INTR.RCV_RST
 * status is evaluated, if its reset, then only a new packet will be written
 * else new packet will be dropped.
 */
#define PDSS_RX_MEM_DATA_DATA_MASK                          (0xffffffff) /* <0:31> RW:R:0: */
#define PDSS_RX_MEM_DATA_DATA_POS                           (0)


/*
 * TX/RX SRAM Read/Write pointer
 * FW can use these pointers to Read/Write more data after the RX_SRAM_HALF_END/TX_SRAM_HALF_END
 * interrupts.
 */
#define PDSS_SRAM_PTR_ADDRESS                               (0x400a00a0)
#define PDSS_SRAM_PTR                                       (*(volatile uint32_t *)(0x400a00a0))
#define PDSS_SRAM_PTR_DEFAULT                               (0x00000000)

/*
 * The transmit SRAM read pointer.
 */
#define PDSS_SRAM_PTR_TX_FUNC_RD_PTR_MASK                   (0x0000001f) /* <0:4> RW:R:0: */
#define PDSS_SRAM_PTR_TX_FUNC_RD_PTR_POS                    (0)


/*
 * The recevie SRAM write pointer.
 */
#define PDSS_SRAM_PTR_RX_FUNC_WR_PTR_MASK                   (0x00001f00) /* <8:12> RW:R:0: */
#define PDSS_SRAM_PTR_RX_FUNC_WR_PTR_POS                    (8)


/*
 * The packet size is odd. Only the first byte of the last two byte is valid.
 * (RX_FUNC_WR_PTR - 1)
 */
#define PDSS_SRAM_PTR_RX_ODD_LENGTH                         (1u << 13) /* <13:13> RW:R:0: */


/*
 * Generic status register.
 */
#define PDSS_STATUS_ADDRESS                                 (0x400a00a4)
#define PDSS_STATUS                                         (*(volatile uint32_t *)(0x400a00a4))
#define PDSS_STATUS_DEFAULT                                 (0x00000000)

/*
 * Receiver is currently receiving a packet.
 * This signal gives information that Controlller has locked on to SOP and
 * is in process of receiving a packet. This will assert only after locking
 * on to Sop* symbols
 */
#define PDSS_STATUS_RX_BUSY                                 (1u << 0) /* <0:0> RW:R:0: */


/*
 * Transmitter is currently transmitting a packet or the crc timmer is running
 */
#define PDSS_STATUS_TX_BUSY                                 (1u << 1) /* <1:1> RW:R:0: */


/*
 * This status bit shows the CC_RX_VALID signal.
 * 0: No Valid data on the CC line'
 * 1: Valid Data detectd on the CC line
 */
#define PDSS_STATUS_CC_DATA_VALID                           (1u << 2) /* <2:2> RW:R:0: */


/*
 * Type of SOP detected for the packet stored in the RX SRAM:
 * At the start of every packet, INTR.RCV_PACKET_COMPLETE and INTR.RCV_RST
 * status is evaluated.
 * If both are "0", then this register will be updated with the new packet
 * SOP value.
 * There is no clearing option.
 */
#define PDSS_STATUS_SOP_TYPE_DETECTED_MASK                  (0x00000038) /* <3:5> RW:R:0: */
#define PDSS_STATUS_SOP_TYPE_DETECTED_POS                   (3)


/*
 * GoodCrc Message SOP type detected:
 * At the start of every packet, INTR.RCV_GOODCRC_MSG_COMPLETE status is
 * evaluated, if its reset, then this register will be updated with the new
 * packet SOP value.
 */
#define PDSS_STATUS_GOODCRC_MSG_SOP_TYPE_DETECTED_MASK      (0x000001c0) /* <6:8> RW:R:0: */
#define PDSS_STATUS_GOODCRC_MSG_SOP_TYPE_DETECTED_POS       (6)


/*
 * RST Type detected:
 */
#define PDSS_STATUS_RST_TYPE_DET_MASK                       (0x00000e00) /* <9:11> RW:R:0: */
#define PDSS_STATUS_RST_TYPE_DET_POS                        (9)


/*
 * This status bit shows that the transmit is in the process of sending googcrc
 * msg
 */
#define PDSS_STATUS_SENDING_GOODCRC_MSG                     (1u << 13) /* <13:13> RW:R:0: */


/*
 * RX SOP Control for sending GoodCRC Message
 * Hardware will wait for programmable IDLE_COUNTER and then send Good Crc
 * Message.
 */
#define PDSS_RX_SOP_GOOD_CRC_EN_CTRL_ADDRESS                (0x400a00a8)
#define PDSS_RX_SOP_GOOD_CRC_EN_CTRL                        (*(volatile uint32_t *)(0x400a00a8))
#define PDSS_RX_SOP_GOOD_CRC_EN_CTRL_DEFAULT                (0x00000003)

/*
 * Setting this bit will enable sending GoodCrcMsg for packet with Bad EOP.
 * This should be left to default for normal operation.
 */
#define PDSS_RX_SOP_GOOD_CRC_EN_CTRL_SEND_GOOD_CRC_BAD_EOP    (1u << 0) /* <0:0> R:RW:1: */


/*
 * Setting this bit will enable sending GoodCrcMsg for packet with KCHAR
 * Error. This should be left to default for normal operation.
 */
#define PDSS_RX_SOP_GOOD_CRC_EN_CTRL_SEND_GOOD_CRC_BAD_KCHAR    (1u << 1) /* <1:1> R:RW:1: */


/*
 * Setting this bit will enable sending GoodCrcMsg for packet even when there
 * is RX Sram Over Flow is detected. This should be left to default for normal
 * operation.
 */
#define PDSS_RX_SOP_GOOD_CRC_EN_CTRL_SEND_GOOD_CRC_SRAM_OVERFLOW    (1u << 2) /* <2:2> R:RW:0:KEEP_REG_BIT */


/*
 * RX Excepted good CRC message to stop the CRC timers
 */
#define PDSS_RX_EXPECT_GOODCRC_MSG_ADDRESS                  (0x400a00ac)
#define PDSS_RX_EXPECT_GOODCRC_MSG                          (*(volatile uint32_t *)(0x400a00ac))
#define PDSS_RX_EXPECT_GOODCRC_MSG_DEFAULT                  (0x00000001)

/*
 * The expected GoodCRC Messgae Header on the RX side. The expected message
 * ID is handled by Firmware. The DEBUG_CC_2.EXPECTED_HEADER_MASK can be
 * used to mask comparing individual bits.
 * The CRC timer will stop on:
 * 1: On the reception of GoodCRC Messegae with good CRC32 where its header
 * matches with this register AND
 * 2: The SOP of the GoodCRC Messgae matches with the EXPECTED_SOP.
 */
#define PDSS_RX_EXPECT_GOODCRC_MSG_EXPECTED_HEADER_MASK     (0x0000ffff) /* <0:15> R:RW:1:NOT_USE_TX_HEADER */
#define PDSS_RX_EXPECT_GOODCRC_MSG_EXPECTED_HEADER_POS      (0)


/*
 * The expected SOP of GoodCRC Messgae on the RX side.
 */
#define PDSS_RX_EXPECT_GOODCRC_MSG_EXPECTED_SOP_MASK        (0x00070000) /* <16:18> R:RW:0:NOT_USE_RX_ORDER_SET */
#define PDSS_RX_EXPECT_GOODCRC_MSG_EXPECTED_SOP_POS         (16)


/*
 * FW should toggle this bit before commiting a new packet to be transferred
 * from TX_MEM.
 * 0: Don’t disable the RX CRC count down
 * 1: Disable the RX CRC count down.
 *    FW can disable the RX CRC timer whenever it detects the required condition.
 */
#define PDSS_RX_EXPECT_GOODCRC_MSG_DISABLE_RX_CRC_TIMER     (1u << 19) /* <19:19> R:RW:0: */


/*
 * The 2-Byte Header of the received GoodCRC Message
 */
#define PDSS_RX_GOODCRC_MSG_ADDRESS                         (0x400a00b0)
#define PDSS_RX_GOODCRC_MSG                                 (*(volatile uint32_t *)(0x400a00b0))
#define PDSS_RX_GOODCRC_MSG_DEFAULT                         (0x00000000)

/*
 * The INTR.RCV_GOODCRC_MSG_COMPLETE interrupt indicates the 2-Byte header
 * for GoodCRC message is received and stored in this registers.
 * GOODCRC_MSG_SOP_TYPE_DETECTED contains the SOP type for the GoodCRC MSG.
 * At the start of every packet, INTR.RCV_GOODCRC_MSG_COMPLETE status is
 * evaluated, if its reset, then only a new packet will be written else new
 * packet will be dropped.
 */
#define PDSS_RX_GOODCRC_MSG_HEADER_MASK                     (0x0000ffff) /* <0:15> RW:R:0: */
#define PDSS_RX_GOODCRC_MSG_HEADER_POS                      (0)


/*
 * The Receive C-Connect registers 0
 */
#define PDSS_RX_CC_0_CFG_ADDRESS                            (0x400a00b4)
#define PDSS_RX_CC_0_CFG                                    (*(volatile uint32_t *)(0x400a00b4))
#define PDSS_RX_CC_0_CFG_DEFAULT                            (0x00000580)

/*
 * This value is internally multiplied by 16.
 * The 16X value when multiplied by the period of CLK_RX defines the maximum
 * clock period.
 * This value is used to cause the RX state machine to return to idle state
 * if no transitions are detected.
 * For 12 Mhz, the count should be 20 Decimal ( becomes approx. 26usec)
 * For 24 Mhz, the count should be 40 Decimal
 */
#define PDSS_RX_CC_0_CFG_RX_CNT_MAX_MASK                    (0x000000ff) /* <0:7> R:RW:128: */
#define PDSS_RX_CC_0_CFG_RX_CNT_MAX_POS                     (0)


/*
 * Hardware calculates the UI by averaging the pre-programmed number of Preamble
 * Bits in the DEBUG_CC_1 register.
 * Once UI is calculated, hardware uses a sampling point percentage to know
 * whether incoming BMC pattern is a 0 or 1.
 * The Hardware automatically calculates the 75% location with respect to
 * UI.
 * The value of this register determines the sampling point by subtracting
 * number of RX_CLK from 75% UI. The value of 0 will mean that hardware samples
 * at 75% of UI and 1 will mean hardware will sample at around 72.5% of UI
 * for 12Mhz clock and 73.75 for 24Mhz RX_CLK
 * (RX_CLK period/UI will determine the step size percentage)
 * For 12 Mhz CLK_RX operations, set the value to 0x2 which means that hardware
 * will sample incoming BMC pattern at around 70%.
 * For 24Mhz CLK_RX operations, set the value to 0x4 which means that hardware
 * will sample incoming BMC pattern at around 70%
 */
#define PDSS_RX_CC_0_CFG_RX_UI_BOUNDARY_DELTA_MASK          (0x00003f00) /* <8:13> R:RW:5: */
#define PDSS_RX_CC_0_CFG_RX_UI_BOUNDARY_DELTA_POS           (8)


/*
 * The Receive C-Connect registers 1
 */
#define PDSS_RX_CC_1_CFG_ADDRESS                            (0x400a00b8)
#define PDSS_RX_CC_1_CFG                                    (*(volatile uint32_t *)(0x400a00b8))
#define PDSS_RX_CC_1_CFG_DEFAULT                            (0x00040000)

/*
 * Number of RX_CC transitions before RX_VALID output is raised.
 */
#define PDSS_RX_CC_1_CFG_DELAY_VALID_COUNT_MASK             (0x000f0000) /* <16:19> R:RW:4: */
#define PDSS_RX_CC_1_CFG_DELAY_VALID_COUNT_POS              (16)


/*
 * Receive SOPs and RSTs order set control
 */
#define PDSS_RX_ORDER_SET_CTRL_ADDRESS                      (0x400a00bc)
#define PDSS_RX_ORDER_SET_CTRL                              (*(volatile uint32_t *)(0x400a00bc))
#define PDSS_RX_ORDER_SET_CTRL_DEFAULT                      (0x00006103)

/*
 * This register is used for SOP, SOP',SOP'", DEBUG SOP', DEBUG SOP" and
 * RX_RESERVED1/2_ORDER_SET(if configured for SOP) oder set detection. It
 * is recommended that CPU program this register to 1 ( 4 out of 4 option).
 * 0: Compare 3 out of 4 order sets
 * 1: Compare 4 out of 4 order sets
 */
#define PDSS_RX_ORDER_SET_CTRL_SOP_CMP_OPT                  (1u << 0) /* <0:0> R:RW:1:KEEP_REG_BIT */


/*
 * This register is used for Cable RST, Hard RST and RX_RESERVED1/2_ORDER_SET(if
 * configure for RST) order set detection.
 * It is recommended that CPU program this register to 1 ( 4 out of 4 option).
 * 0: Compare 3 out of 4 order sets
 * 1: Compare 4 out of 4 order sets
 */
#define PDSS_RX_ORDER_SET_CTRL_RST_CMP_OPT                  (1u << 1) /* <1:1> R:RW:1:KEEP_REG_BIT */


/*
 * This register is used to enable/disdable 4-bit preamble detection for
 * SOP detection.
 * 0: SOP Detection:                              SOP logic detection
 * 1: SOP detection: Preamble(4-bit)+ SOP logic detection
 */
#define PDSS_RX_ORDER_SET_CTRL_PREAMBLE_SOP_EN              (1u << 2) /* <2:2> R:RW:0:KEEP_REG_BIT */


/*
 * This register is used to enable/disdable 4-bit preamble detection for
 * RST detection.
 * 0: RST Detection:                              RST logic detection
 * 1: RST detection: Preamble(4-bit)+ RST logic detection
 */
#define PDSS_RX_ORDER_SET_CTRL_PREAMBLE_RST_EN              (1u << 3) /* <3:3> R:RW:0:KEEP_REG_BIT */


/*
 * Host Mode: F/W can enable SOP, SOP’, SOP” and Hard Reset Detection.
 * Device Mode: F/W should enable only SOP and Hard Reset Detection.
 * Cable Mode: Either SOP’ or SOP” based on VCONN, Hard Reset and Cable Reset
 * should be enabled.
 */
#define PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_MASK              (0x00007f00) /* <8:14> R:RW:97: */
#define PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_POS               (8)


/*
 * Transmit GoodCrc Message order set
 */
#define PDSS_TX_GOODCRC_MSG_ORDER_SET_ADDRESS               (0x400a00c0)
#define PDSS_TX_GOODCRC_MSG_ORDER_SET                       (*(volatile uint32_t *)(0x400a00c0))
#define PDSS_TX_GOODCRC_MSG_ORDER_SET_DEFAULT               (0x00000000)

/*
 * [15:0]   Can also be used for Transmiting GoodCRC Message to SOP'
 * [31:16] Can also be used for Transmiting GoodCRC Message to SOP"
 * This register constains the Transmit GoodCRC Message Header except the
 * MessageID which Is handled by Hardware.
 * [11:9] Message ID (This is handled by HardWare) for SOP'
 * [27:25] Message ID (This is handled by HardWare) for SOP"
 */
#define PDSS_TX_GOODCRC_MSG_ORDER_SET_TX_GOODCRC_OS_MASK    (0xffffffff) /* <0:31> R:RW:0: */
#define PDSS_TX_GOODCRC_MSG_ORDER_SET_TX_GOODCRC_OS_POS     (0)


/*
 * TX Control
 */
#define PDSS_TX_CTRL_ADDRESS                                (0x400a00c4)
#define PDSS_TX_CTRL                                        (*(volatile uint32_t *)(0x400a00c4))
#define PDSS_TX_CTRL_DEFAULT                                (0x8f200041)

/*
 * For SOP Only.
 * This register constains the Transmit GoodCRC Message Header except the
 * MessageID which Is handled by Hardware.
 * [11:9] Message ID (This is handled by HardWare)
 */
#define PDSS_TX_CTRL_GOODCRC_MSG_BITS_MASK                  (0x0000ffff) /* <0:15> R:RW:65:KEEP_REG_BIT */
#define PDSS_TX_CTRL_GOODCRC_MSG_BITS_POS                   (0)


/*
 * Setting the EN_TX_BIST_CM2 to "1" will start the transmision of Bist Carrier
 * Mode 2 pattern.
 * FW must manually set TX_CTRL.TX_REG_EN to "1" before setting this register(EN_TX_BIST_CM2)
 * The TX_GO register is not required to be set for this mode.
 * FW should wait for TX_REG_TIMMER (50usec) before setting this bit.
 */
#define PDSS_TX_CTRL_EN_TX_BIST_CM2                         (1u << 16) /* <16:16> R:RW:0: */


/*
 * TX_GO causes a packet to be sent. FW can send GoodCrcMsg by storing it
 * in the TX SRAM and use TX_GO to send it.
 * Writing a 1 to this bit to cause the message stored in the SRAM Memory
 * to be sent.  Hardware clears this bit once the command is accepted and
 * processing has begun.
 * If TX_GO is set and there is a ongoing receive packet, the TX packet wont
 * be sent and the COLLISION_TYPE1
 * interrupt will be set. In this case, hardware clears the TX_GO.
 * Before setting this FW should check:
 * INTR0->RX_GOOD_PKT && STATUS->DATA_VALID == 0
 */
#define PDSS_TX_CTRL_TX_GO                                  (1u << 17) /* <17:17> RW1C:RW:0: */


/*
 * Send a Reset over the link. Write a 1 to this bit to cause the transmitter
 * to send a Hard Reset or Cable Reset(TX_HARD_CABLE_ORDER_SET register)
 * over the link. Hardware clears this bit once the command is accepted and
 * processing has begun.
 * If TX_SEND_RST is set and there is a ongoing receive packet, the Reset
 * Sequqnce wont be sent and the COLLISION_TYPE4 interrupt will be set. In
 * this case, hardware clears the TX_SEND_RST.
 * Before settting this FW should check:
 * INTR0->RX_GOOD_PKT && STATUS->DATA_VALID == 0
 */
#define PDSS_TX_CTRL_TX_SEND_RST                            (1u << 18) /* <18:18> RW1C:RW:0: */


/*
 * Enable transmit retry. Hardware clears this bit once the command is accepted
 * and processing has begun.
 * CPU should increment the retry counter in firmware once TX_PACKET_DONE
 * interrupt is detected by CPU.
 * The following operation is recommneded to FW:
 *    • FW maintains the retry counter
 *    • FW writes a packet in TX_Memory.
 *    • FW checks its retry counter and if its >0 , sets the retry enable
 * bit.
 *    • FW sets the TX_GO.
 *    • HW sends the packet and starts CRC_Timer if enabled.
 *    • On Expiry of CRC_timer, HW retries the packet if retry enable bit
 * was set and then HW auto clears that bit and generates the
 *      TX_RETRY_ENABLE_CLRD interrupt
 *    • HW will start the CRC_timer again.
 *    • FW in parallel would have received the CRC_TIMER expiry interrupt…
 * FW will decrement its retry counter and if retry
 *      counter is still >0, then it will set the retry enable again
 *
 * The usage model is as follows for setting the retry enable bit:
 * 1. Firmware gets the CRC receive timeout interrupt. After getting this
 * interrupt, firmware can either do step 2 or step 3.
 * 2. Firmware should wait for 75 usec after the timeout interrupt and sample
 * retry bit. Retry bit should be zero at this point indicating retry packet
 * has started transmitting or if retry bit is still one, then it could be
 * that collission has been detected on the bus. If Retry bit is 0, firmware
 * can set the retry bit firmware will have approximately Packet size  time
 * + crc timer (Minimum of 1.5 msec)
 * 3. Firmware doesnt want to put delay of 75 usec, it can instead use TX_RETRY_ENABLE_CLRD
 * interrupt.
 */
#define PDSS_TX_CTRL_TX_RETRY_ENABLE                        (1u << 19) /* <19:19> RW1C:RW:0:KEEP_REG_BIT */


/*
 * Enable the transmitter regulator.
 * This should be set to "1" for BIST mode other wise hardware automatically
 * takes enabling the regulator when TX_REG_CFG=1
 */
#define PDSS_TX_CTRL_TX_REG_EN                              (1u << 20) /* <20:20> R:RW:0: */


/*
 * 0: Hardware controlling of TX regulator Enable is disabled. CPU can fully
 * control the TX regulator enable by using TX_REG_EN.
 * 1: Hardware controlling the TX regulator Enable is enabled. In this case,
 * CPU can only set the regulator enable to one by setting TX_REG_EN
 */
#define PDSS_TX_CTRL_TX_REG_CFG                             (1u << 21) /* <21:21> R:RW:1: */


/*
 * The time needed to enable the TX regulator before transmission.
 * The counter runs on CLK_TX_HALF(600Khz)
 */
#define PDSS_TX_CTRL_TX_REG_TIMER_MASK                      (0x3f000000) /* <24:29> R:RW:15:KEEP_REG_BIT */
#define PDSS_TX_CTRL_TX_REG_TIMER_POS                       (24)


/*
 * Setting this bit will enable corrupting the TX CRC when there is TX Sram
 * Under Flow is detected. This should be left to default for normal operation.
 */
#define PDSS_TX_CTRL_TX_CORRUPT_CRC_ON_UNDER_FLOW           (1u << 31) /* <31:31> R:RW:1: */


/*
 * Transmit SOP order set
 */
#define PDSS_TX_SOP_ORDER_SET_ADDRESS                       (0x400a00c8)
#define PDSS_TX_SOP_ORDER_SET                               (*(volatile uint32_t *)(0x400a00c8))
#define PDSS_TX_SOP_ORDER_SET_DEFAULT                       (0x0008e318)

/*
 * Transmit SOP order Set use in transmit except GoodCrcMsg
 */
#define PDSS_TX_SOP_ORDER_SET_TX_SOP_OS_MASK                (0x000fffff) /* <0:19> R:RW:582424: */
#define PDSS_TX_SOP_ORDER_SET_TX_SOP_OS_POS                 (0)


/*
 * Transmit Hard/Cable reset order set
 */
#define PDSS_TX_HARD_CABLE_ORDER_SET_ADDRESS                (0x400a00cc)
#define PDSS_TX_HARD_CABLE_ORDER_SET                        (*(volatile uint32_t *)(0x400a00cc))
#define PDSS_TX_HARD_CABLE_ORDER_SET_DEFAULT                (0x000e7393)

/*
 * Transmit Hard/Cable Reset order Set.
 * Default: Hard Reset Value 0xE7393
 * Cable Reset Value: 0xE0F8C
 */
#define PDSS_TX_HARD_CABLE_ORDER_SET_TX_RESET_OS_MASK       (0x000fffff) /* <0:19> R:RW:947091: */
#define PDSS_TX_HARD_CABLE_ORDER_SET_TX_RESET_OS_POS        (0)


/*
 * The CRC timer counters configuration
 * Counters used for the timmers needed by this IP
 */
#define PDSS_CRC_COUNTER_ADDRESS                            (0x400a00d0)
#define PDSS_CRC_COUNTER                                    (*(volatile uint32_t *)(0x400a00d0))
#define PDSS_CRC_COUNTER_DEFAULT                            (0x00000000)

/*
 * This counter will run on TX_CLK/2 (PD bit period) clock.
 * This counter is used for RCReceiveTimer(tReceive)/BISTReceiveErrorTimertBistReceive
 * 0: Counter is disabled. Hardware will NOT wait for the CRC_COUNTER to
 * expire.
 * Other: Hardware will wait for the CRC_COUNTER to expire.
 * Once the CRC_COUNTER reaches zero and no Valid CoodCRC message is not
 * received, The CRC_RX_TIMER_EXP interrupt gets set.
 *
 * If TX_BYPASS_EN = 1:
 *         [15:0]: Total number of bits
 */
#define PDSS_CRC_COUNTER_CRC_COUNTER_MASK                   (0x0000ffff) /* <0:15> R:RW:0: */
#define PDSS_CRC_COUNTER_CRC_COUNTER_POS                    (0)


/*
 * The Inter Packet counters
 * Counters used for IDLE/IFG and  by this IP
 * All the timers/counters have a resolution of 1 UI (Unit Interval) of a
 * Bit. If transmit rate is 300Khz, then each count will tick for 3.33 usec.
 */
#define PDSS_INTER_PACKET_COUNTER_ADDRESS                   (0x400a00d4)
#define PDSS_INTER_PACKET_COUNTER                           (*(volatile uint32_t *)(0x400a00d4))
#define PDSS_INTER_PACKET_COUNTER_DEFAULT                   (0x01004008)

/*
 * USED FOR TX->TX
 * This register is used by DUT to create gap between two back to back transmit
 * packets. For example: For a DFP application if DFP wants to send Hard
 * Reset after a valid packet, then this register could be used for complying
 * with Inter-Packet Gap of 25 usec specified by spec. In cable application,
 * after sending Good CRC handshake for request from DUT, this register could
 * be used to comply with 750usec requirement of cable response after sending
 * Good CRC pkt.
 */
#define PDSS_INTER_PACKET_COUNTER_BUS_IDLE_CNT_MASK         (0x000007ff) /* <0:10> R:RW:8: */
#define PDSS_INTER_PACKET_COUNTER_BUS_IDLE_CNT_POS          (0)


/*
 * RX -> AUTO_GOODCRC_RESPONSE
 * This counter specifies how long the HW should wait after the end of RX
 * packet to send GoodCRC message. This can be used to comply with interpacket
 * gap of 25usec.
 * 0: Counter is disabled. Hardware will issue goodcrc message if needed
 * after end of the RX Packet
 *
 * For CCG2 ** Silicon, this can stay at spec value of 25usec,
 * For CCG2 *A Silicon, this need to be programmed at minimum of 150usec
 * (Value 50). The new noise circuit gets settled at 75usec and then RX_VALID
 * logic will need another 75usec to declare IDLE on the bus. This value
 * is worst case number and can be used for CCG2 ** Silicon as well.
 * For CCG3/4: The value can be as low as 120usec.
 */
#define PDSS_INTER_PACKET_COUNTER_IDLE_COUNTER_MASK         (0x001ff800) /* <11:20> R:RW:8: */
#define PDSS_INTER_PACKET_COUNTER_IDLE_COUNTER_POS          (11)


/*
 * END OF ANY RX ON CC-LINE
 * On any RX Packet, this counter will start at end of RX Packet and will
 * reset on Start of RX Packet. This register can be again used to comply
 * with Interpacket Gap (25 usec + end of IDLE detection(12 usec)).
 * CPU after seeing no activity on the bus can immedeately set the TX_GO/TX_SEND_RST
 * bit and this register will make sure that we don't violate any interpacket
 * gap requirement.
 * 0: Counter is disabled.
 */
#define PDSS_INTER_PACKET_COUNTER_IFG_COUNTER_MASK          (0xffe00000) /* <21:31> R:RW:8: */
#define PDSS_INTER_PACKET_COUNTER_IFG_COUNTER_POS           (21)


/*
 * The trigger enable registers
 * The tr_out[4:0] pins of this IP will be connected to the tr_in pin of
 * tcpwm_ver2 IP at the full chip. The mapping of the these signals is SoC
 * depended and it is defined
 * in SAS.
 */
#define PDSS_TIMER_TRIGGER_ADDRESS                          (0x400a00d8)
#define PDSS_TIMER_TRIGGER                                  (*(volatile uint32_t *)(0x400a00d8))
#define PDSS_TIMER_TRIGGER_DEFAULT                          (0x00000000)

/*
 * 1: The tr_out[0] pin of the IP will toggle on the transmission of the
 * last Bit Of EOP.
 * 0: The toggling of the tr_out[0] pin of the IP is disabled.
 */
#define PDSS_TIMER_TRIGGER_EN_TRIGGER0                      (1u << 0) /* <0:0> R:RW:0: */


/*
 * 1: The tr_out[1] pin of the IP will toggle on the reception of EOP for
 * any message with Valid CRC (Includes Good Crc Message)
 * 0: The toggling of the tr_out[1] pin of the IP is disabled.
 */
#define PDSS_TIMER_TRIGGER_EN_TRIGGER1                      (1u << 1) /* <1:1> R:RW:0: */


/*
 * 1: The tr_out[2] pin of the IP will toggle on the reception of EOP for
 * any message with Valid CRC (Excludes the message types specified in the
 * RX_ORDER_SET_CTRL register)
 * 0: The toggling of the tr_out[2] pin of the IP is disabled.
 */
#define PDSS_TIMER_TRIGGER_EN_TRIGGER2                      (1u << 2) /* <2:2> R:RW:0: */


/*
 * 1: The tr_out[3] pin of the IP will toggle on the reception of EOP for
 * any message (Example: RX Hard Reset/ BIST)
 * 0: The toggling of the tr_out[3] pin of the IP is disabled.
 */
#define PDSS_TIMER_TRIGGER_EN_TRIGGER3                      (1u << 3) /* <3:3> R:RW:0: */


/*
 * 1: The tr_out[4] pin of the IP will toggle on the transmission of the
 * first Bit of SOP (Additional trigger for starting some different timer).
 * 0: The toggling of the tr_out[4] pin of the IP is disabled.
 */
#define PDSS_TIMER_TRIGGER_EN_TRIGGER4                      (1u << 4) /* <4:4> R:RW:0: */


/*
 * 1: The tr_out[5] pin of the IP will toggle on rx_valid
 * 0: The toggling of the tr_out[5] pin of the IP is disabled.
 */
#define PDSS_TIMER_TRIGGER_EN_TRIGGER5                      (1u << 5) /* <5:5> R:RW:0: */


/*
 * 1: The tr_out[6] pin of the IP will toggle on the reception of valid SOP.
 * 0: The toggling of the tr_out[6] pin of the IP is disabled.
 */
#define PDSS_TIMER_TRIGGER_EN_TRIGGER6                      (1u << 6) /* <6:6> R:RW:0: */


/*
 * Debug Control Register
 */
#define PDSS_DEBUG_CTRL_ADDRESS                             (0x400a00dc)
#define PDSS_DEBUG_CTRL                                     (*(volatile uint32_t *)(0x400a00dc))
#define PDSS_DEBUG_CTRL_DEFAULT                             (0x037f0000)

/*
 * This register are for debugging purposes.
 * 0: Receive path is not at reset.
 * 1: Reset the logic on the receive path except the Hard-IP.
 *     FW should check STATUS.RX_BUSY to make sure it is zero before setting
 * this bit.
 */
#define PDSS_DEBUG_CTRL_RESET_RX                            (1u << 0) /* <0:0> R:RW:0:KEEP_REG_BIT */


/*
 * Message Cal State
 */
#define PDSS_DEBUG_CTRL_RX_MSG_CAL_STATE_MASK               (0x0000000e) /* <1:3> RW:R:0:KEEP_REG_BIT */
#define PDSS_DEBUG_CTRL_RX_MSG_CAL_STATE_POS                (1)


/*
 * This register are for debugging purposes
 * 0: Transmit path is not at reset.
 * 1: Reset the logic on the transmit path except the Hard-IP.
 *     FW should check STATUS.TX_BUSY to make sure it is zero before setting
 * this bit.
 */
#define PDSS_DEBUG_CTRL_RESET_TX                            (1u << 8) /* <8:8> R:RW:0:KEEP_REG_BIT */


/*
 * Transmit state machine
 */
#define PDSS_DEBUG_CTRL_TX_MSG_STATE_MASK                   (0x00000e00) /* <9:11> RW:R:0:KEEP_REG_BIT */
#define PDSS_DEBUG_CTRL_TX_MSG_STATE_POS                    (9)


/*
 * TX SRC Select state machine
 */
#define PDSS_DEBUG_CTRL_TX_SRC_SEL_STATE_MASK               (0x0000e000) /* <13:15> RW:R:0:KEEP_REG_BIT */
#define PDSS_DEBUG_CTRL_TX_SRC_SEL_STATE_POS                (13)


/*
 * Number of TX preambles+1 (bit transitions)
 */
#define PDSS_DEBUG_CTRL_TX_PREAMBLE_CNT_MASK                (0x003f0000) /* <16:21> R:RW:63:KEEP_REG_BIT */
#define PDSS_DEBUG_CTRL_TX_PREAMBLE_CNT_POS                 (16)


/*
 * EOP value for Both RX and TX
 */
#define PDSS_DEBUG_CTRL_EOP_VALUE_MASK                      (0x07c00000) /* <22:26> R:RW:13:KEEP_REG_BIT */
#define PDSS_DEBUG_CTRL_EOP_VALUE_POS                       (22)


/*
 * This bit enables the Receive state machine to exit from RX DATA and CRC
 * states when an EOP is detected.
 * 0: Ignore EOP during the Data and CRC state
 * 1: Exit from Data and CRC state when an EOP is detected and move to STATUS
 * state.
 */
#define PDSS_DEBUG_CTRL_ENABLE_EXIT_ON_EOP                  (1u << 27) /* <27:27> R:RW:0: */


/*
 * This register are for debugging purposes.
 * 0: VBTR block  is not at reset.
 * 1: Reset the logic on the VBTR except the Hard-IP.
 */
#define PDSS_DEBUG_CTRL_RESET_VBTR                          (1u << 30) /* <30:30> R:RW:0:VBTR_EN */


/*
 * This register are for debugging purposes.
 * 0: PASC block  is not at reset.
 * 1: Reset the logic on the PASC except the Hard-IP.
 */
#define PDSS_DEBUG_CTRL_RESET_PASC                          (1u << 31) /* <31:31> R:RW:0:PASC_EN */


/*
 * C-Connector Debug control register 0
 */
#define PDSS_DEBUG_CC_0_ADDRESS                             (0x400a00e0)
#define PDSS_DEBUG_CC_0                                     (*(volatile uint32_t *)(0x400a00e0))
#define PDSS_DEBUG_CC_0_DEFAULT                             (0x00000000)

/*
 * FW can only use this bit when the DEBUG_CC_0.TX_CC_DRIVE_SRC is set to
 * "1".
 * 0: Disables the Transceiver to transmit data
 * 1: Enables the Transceiver to transmit data
 */
#define PDSS_DEBUG_CC_0_TX_FIRST_BIT_LEVEL                  (1u << 0) /* <0:0> R:RW:0: */


/*
 * When set enables TX to RX loopback before CC encoding/Decoding data.
 */
#define PDSS_DEBUG_CC_0_LOOP_BACK_NO_BMC                    (1u << 1) /* <1:1> R:RW:0: */


/*
 * Loobback after data encdoing. When set, the BMC encoded tx output will
 * loop back into cc_rx module.
 */
#define PDSS_DEBUG_CC_0_LOOP_BACK_WITH_BMC                  (1u << 2) /* <2:2> R:RW:0: */


/*
 * When set, enables rx module to decode cc_rx line all the time. (Including
 * during transmission).
 */
#define PDSS_DEBUG_CC_0_EXT_LOOP_BACK                       (1u << 3) /* <3:3> R:RW:0: */


/*
 * When set to one, clears the BMC decoder RX state machines and counters.
 * It has to be set back to zero for normal operations.
 * RX_RESET is not required to be set
 */
#define PDSS_DEBUG_CC_0_RX_CLEAR                            (1u << 4) /* <4:4> R:RW:0: */


/*
 * When set to one, clears the TX state machines and counters. It has to
 * be set back to zero for normal operations.
 */
#define PDSS_DEBUG_CC_0_TX_CLEAR                            (1u << 5) /* <5:5> R:RW:0: */


/*
 * This will selects either the mxusbpd_cc_tx or FW to control the TX_EN/TX_DATA
 * ports of the s8usbpd_cc_top Hard IP.
 * 0: Hardware (mxusbpd_cc_tx) controls the TX_EN/TX_DATA ports of the s8usbpd_cc_top
 * Hard IP.
 * 1: This option is for Testing/Char. FW controls the TX_EN/TX_DATA ports
 * of the s8usbpd_cc_top Hard IP.
 */
#define PDSS_DEBUG_CC_0_TX_CC_DRIVE_SRC                     (1u << 6) /* <6:6> R:RW:0: */


/*
 * FW can use this bit to dirve the CC data line when the TX_CC_DRIVE_SRC=1
 * and CC_DPSLP_REF_CTRL.TX_EN=1
 * When TX_CC_DRIVE_SRC is set to one:
 * - TX_EN port of the s8usbpd_cc_top is controlled by CC_DPSLP_REF_CTRL.TX_EN
 * - TX_DATA port of s8usbpd_cc_top Hard IP is controlled by TX_CC_DATA
 */
#define PDSS_DEBUG_CC_0_TX_CC_DATA                          (1u << 7) /* <7:7> R:RW:0: */


/*
 * Selects the inputs to CC_DEBUG_OUT. Used for debug.
 * 0. RX clk_cnt_q[7:0]
 * 1. RX {clk_cnt_q[9:8], cdr_accum_q[11:8], cq_2, cq_3}
 * 2. {count_q, 2'h0}
 * 3. cdr_accum_q[7:0]
 * 4. {1'h0, rx_state_q, cc_rx_data_del_q}
 * 5. limit_q
 * 6. RX {one_detect_q, cc_rx_bit_early_q,  cc_rx_data, cc_rx_bit, cc_rx_valid,
 * cc_rx_data_2_sync, cc_tx_data_pin, cc_data_pin_oe}
 * 7. cdr_average_q_latched_q
 * 8. {diff_q[3:0], low_count_q[3:0]}
 * 9. cdr_average_q[7:0]
 * 16. Not defined yet.
 * 17 TX {2'h0, tx_state_q, new_data_q}
 * 18. TX {2'h0, cq_0, cq_3, cc_tx_data_lat_q, cc_tx_eof_lat_q, cc_tx_data_valid_lat_q,
 * cc_new_data_lat_q}
 * 19.  TX {3'h0, cc_tx_data_pin, cc_data_pin_oe, cc_tx_data, cc_tx_eof,
 * cc_tx_data_valid}
 * 20-15 not defined yet.
 */
#define PDSS_DEBUG_CC_0_DEBUG_SEL_MASK                      (0x00001f00) /* <8:12> R:RW:0:KEEP_REG_BIT */
#define PDSS_DEBUG_CC_0_DEBUG_SEL_POS                       (8)


/*
 * 0: RX_CC_DATA_VALID signal is not disabled.
 * 1: RX_CC_DATA_VALID signal is disabled.
 */
#define PDSS_DEBUG_CC_0_RX_CC_DATA_VALID_DIS                (1u << 13) /* <13:13> R:RW:0: */


/*
 * Debug output register. Its inputs are selected by CC_DEBUG_SEL
 */
#define PDSS_DEBUG_CC_0_DEBUG_OUT_MASK                      (0x0ff00000) /* <20:27> RW:R:0:KEEP_REG_BIT */
#define PDSS_DEBUG_CC_0_DEBUG_OUT_POS                       (20)


/*
 * Selection bit for deepsleep vs. active current reference for Rp pull-up
 * termination
 * 0 - Select deepsleep current reference
 * 1 - Select active current reference
 */
#define PDSS_DEBUG_CC_0_IREF_SEL                            (1u << 28) /* <28:28> R:RW:0:KEEP_REG_BIT */


/*
 * 0: Create EOP in the case of TX BIST
 * 1: Don’t create EOP in the case of TX BIST
 */
#define PDSS_DEBUG_CC_0_DISABLE_BIST_EOP                    (1u << 29) /* <29:29> R:RW:0: */


/*
 * C-Connector Debug control register 1
 */
#define PDSS_DEBUG_CC_1_ADDRESS                             (0x400a00e4)
#define PDSS_DEBUG_CC_1                                     (*(volatile uint32_t *)(0x400a00e4))
#define PDSS_DEBUG_CC_1_DEFAULT                             (0x000140cc)

/*
 * Number of preamble bits to be used in the RX for averaging CDR frequency.
 * Any time the value of these bits are changed, the values of NUM_TRANS_AVG
 * will need to be updated.
 */
#define PDSS_DEBUG_CC_1_NUM_PREAMBLE_AVG_MASK               (0x00000007) /* <0:2> R:RW:4: */
#define PDSS_DEBUG_CC_1_NUM_PREAMBLE_AVG_POS                (0)


/*
 * Number of transitions required to complete averaging in the receiver.
 * This register will need to be updated any time values of NUM_PREAMBLE_AVG
 * is changed.
 * Total of this register and RX_IGNR_TRANS_NUM should not be more than 63
 * decimal.
 * The values programmed into this register comes from the following table:
 * NUM_PREAMBLE_AVG = 000 : Use 0x19
 * NUM_PREAMBLE_AVG = 001 : Use 0x19
 * NUM_PREAMBLE_AVG = 010 : Use 0x7
 * NUM_PREAMBLE_AVG = 011 : Use 0xd
 * NUM_PREAMBLE_AVG = 100 : Use 0x19
 * NUM_PREAMBLE_AVG = 101 : Use 0x31
 */
#define PDSS_DEBUG_CC_1_NUM_TRANS_AVG_MASK                  (0x000001f8) /* <3:8> R:RW:25: */
#define PDSS_DEBUG_CC_1_NUM_TRANS_AVG_POS                   (3)


/*
 * 0: Automatic bit rate calculation by HW
 * 1: Disables the RX-CC automatic bit rate detection and the RX_UI_PRERIOD
 * register is used for RX UI period.
 */
#define PDSS_DEBUG_CC_1_RX_DISABLE_AUTO_ADJ                 (1u << 9) /* <9:9> R:RW:0: */


/*
 * When RX_DISABLE_AUTO_ADJ is set, this register value will define RX UI
 * period.
 */
#define PDSS_DEBUG_CC_1_RX_UI_PERIOD_MASK                   (0x0003fc00) /* <10:17> R:RW:80: */
#define PDSS_DEBUG_CC_1_RX_UI_PERIOD_POS                    (10)


/*
 * 0: The TX statemachine does not reset to Idle on the assertion of "send_good_crc"
 * 1: The TX statemachine does       reset to Idle on the assertion of "send_good_crc"
 */
#define PDSS_DEBUG_CC_1_TX_STATE_RST                        (1u << 18) /* <18:18> R:RW:0: */


/*
 * 300ma switch CC1 Pull down value
 */
#define PDSS_DEBUG_CC_1_PFET300_PULLDN_EN_CC1               (1u << 19) /* <19:19> R:RW:0:KEEP_REG_BIT */


/*
 * 300ma switch CC2 Pull down value
 */
#define PDSS_DEBUG_CC_1_PFET300_PULLDN_EN_CC2               (1u << 20) /* <20:20> R:RW:0:KEEP_REG_BIT */


/*
 * This bit is used only for Receive Extended Messages with the Chunk bit
 * set.
 * 0: Include the 2-byte extended data message header count
 * 1: Don't include the 2-byte extended data message header count
 */
#define PDSS_DEBUG_CC_1_INC_EXT_CHUNK_HDR_COUNT             (1u << 25) /* <25:25> R:RW:0: */


/*
 * If CTRL.RX_BYPASS_EN OR DEBUG_CC_0.LOOP_BACK_NO_BMC are set then the RX
 * processes will be based on Header Length.
 * 1: The RX processes the packets based on the EOP
 * 0: The RX processes the packets based on the Header length
 */
#define PDSS_DEBUG_CC_1_RX_EOP_BASED_EN                     (1u << 27) /* <27:27> R:RW:0: */


/*
 * This bit will disable the RX CC to generate the last rx_cc_bit_en and
 * RX State machine will create rx_cc_bit_en when the internal counter (BIT_EN_CNTR_CTRL.BIT_EN_CNTR)
 * is equal to BIT_EN_CNTR_CTRL.GEN_BIT_EN_CNTR
 */
#define PDSS_DEBUG_CC_1_DIS_CC_LAST_BIT_EN                  (1u << 28) /* <28:28> R:RW:0: */


/*
 * 0: Regular operation
 * 1: If collision is seen on CRC Response, it would clrea the pending TX_GO
 */
#define PDSS_DEBUG_CC_1_EN_CRC_COLL_GO_CLR                  (1u << 29) /* <29:29> R:RW:0: */


/*
 * C-Connector Debug control register 2
 */
#define PDSS_DEBUG_CC_2_ADDRESS                             (0x400a00e8)
#define PDSS_DEBUG_CC_2                                     (*(volatile uint32_t *)(0x400a00e8))
#define PDSS_DEBUG_CC_2_DEFAULT                             (0x00ff0100)

/*
 * 0: delta value is subtracted from calculated UI period.
 * 1: delta value is added to calculated UI.
 */
#define PDSS_DEBUG_CC_2_RX_DELTA_POLARITY                   (1u << 0) /* <0:0> R:RW:0: */


/*
 * 1: Disable CC line monitoring while waiting for Auto GoodCRC response
 * for detection collision.
 * When set, Collision will be only checked once IDLE_COUNTER reaches it
 * programmed value.
 * 0: Enable CC line monitoring even when IDLE_COUNTER
 * is running. Any activity on CC line will trigger a collision.
 */
#define PDSS_DEBUG_CC_2_DIS_CC_MON_AUTO_CRC                 (1u << 1) /* <1:1> R:RW:0: */


/*
 * 1: If any EOP pattern is received during packet, Packet will be aborted.
 * 0: Idle condition/Normal packe-end on CC line will cause abort.
 */
#define PDSS_DEBUG_CC_2_RESET_RX_ON_EOP                     (1u << 2) /* <2:2> R:RW:0: */


/*
 * Number of initial transitions to be ignored before Preamble averaging
 * starts and cc_rx_valid is raised.
 * Total of this register and NUM_TRANS_AVG should not be more than 63 decimal.
 */
#define PDSS_DEBUG_CC_2_RX_IGNR_TRANS_NUM_MASK              (0x00000fc0) /* <6:11> R:RW:4: */
#define PDSS_DEBUG_CC_2_RX_IGNR_TRANS_NUM_POS               (6)


/*
 * The bit wise mask bit for RX_EXPEC_GOODCRC_MSG.
 * This is a bit wise mask and if set, that particular bit is enabled for
 * comparison.
 */
#define PDSS_DEBUG_CC_2_EXPECTED_HEADER_MASK_MASK           (0xffff0000) /* <16:31> R:RW:255: */
#define PDSS_DEBUG_CC_2_EXPECTED_HEADER_MASK_POS            (16)


/*
 * Counter for IDLE detection
 */
#define PDSS_BIT_EN_CNTR_CTRL_ADDRESS                       (0x400a0128)
#define PDSS_BIT_EN_CNTR_CTRL                               (*(volatile uint32_t *)(0x400a0128))
#define PDSS_BIT_EN_CNTR_CTRL_DEFAULT                       (0x00000000)

/*
 * The number of rx clock that Receiver does not see any rx_bit_en to detect
 * a idle condition
 * 0: Disabled
 */
#define PDSS_BIT_EN_CNTR_CTRL_BIT_EN_CNTR_MASK              (0x000003ff) /* <0:9> R:RW:0: */
#define PDSS_BIT_EN_CNTR_CTRL_BIT_EN_CNTR_POS               (0)


/*
 * An internal rx_bit_en is created when this counter is equal to BIT_EN_CNTR.
 */
#define PDSS_BIT_EN_CNTR_CTRL_GEN_BIT_EN_CNTR_MASK          (0x001f0000) /* <16:20> R:RW:0: */
#define PDSS_BIT_EN_CNTR_CTRL_GEN_BIT_EN_CNTR_POS           (16)


/*
 * LF Counter Register
 * Provides actual counter value for LF counter.  LF counter always counts
 * up, is free-running and is clocked using clk_lf.
 */
#define PDSS_LF_CNTR_ADDRESS                                (0x400a0134)
#define PDSS_LF_CNTR                                        (*(volatile uint32_t *)(0x400a0134))
#define PDSS_LF_CNTR_DEFAULT                                (0x00000000)

/*
 * Current value of WDT Counter
 */
#define PDSS_LF_CNTR_COUNTER_MASK                           (0x0000ffff) /* <0:15> RW:R:0: */
#define PDSS_LF_CNTR_COUNTER_POS                            (0)


/*
 * LF Counter Match Register
 * Firmware provided match value that is compared against LF_CNTR.  The expectation
 * is that firmware modifies this register after each match as part of the
 * LF interrupt service routine.
 */
#define PDSS_LF_CNTR_MATCH_ADDRESS                          (0x400a0138)
#define PDSS_LF_CNTR_MATCH                                  (*(volatile uint32_t *)(0x400a0138))
#define PDSS_LF_CNTR_MATCH_DEFAULT                          (0x00001000)

/*
 * Match value for LF counter.  Every time LF_CNTR reaches MATCH an interrupt
 * is generated.
 */
#define PDSS_LF_CNTR_MATCH_MATCH_MASK                       (0x0000ffff) /* <0:15> R:RW:4096: */
#define PDSS_LF_CNTR_MATCH_MATCH_POS                        (0)


/*
 * The number of MSB bits of the LF timer that are NOT checked against MATCH.
 *  This value provides control over the time-to-reset of the watchdog (which
 * happens after 3 successive matches).  Note that certain products may enforce
 * a minimum value for this register through design time configuration.
 */
#define PDSS_LF_CNTR_MATCH_IGNORE_BITS_MASK                 (0x000f0000) /* <16:19> R:RW:0: */
#define PDSS_LF_CNTR_MATCH_IGNORE_BITS_POS                  (16)


/*
 * AFC Charger Detect contrl 1 for logic
 */
#define PDSS_AFC_1_CTRL_ADDRESS                             (0x400a0140)
#define PDSS_AFC_1_CTRL                                     (*(volatile uint32_t *)(0x400a0140))
#define PDSS_AFC_1_CTRL_DEFAULT                             (0x00002648)

/*
 * F/W sets this bit and hardware will transmit 1 for number of UIs programmed
 * in  RESET_UI_COUNT
 */
#define PDSS_AFC_1_CTRL_TX_RESET                            (1u << 0) /* <0:0> RW1C:RW:0: */


/*
 * No of UIs Master will wait for slave ping response before timeout is declared
 * and master state machine will go back to IDLE
 */
#define PDSS_AFC_1_CTRL_UI_WAIT_COUNT_PING_RESPONSE_MASK    (0x0000000e) /* <1:3> R:RW:4: */
#define PDSS_AFC_1_CTRL_UI_WAIT_COUNT_PING_RESPONSE_POS     (1)


/*
 * Duration of number of UIs of 1 on Dminus used for sending/detection of
 * reset
 */
#define PDSS_AFC_1_CTRL_RESET_UI_COUNT_MASK                 (0x00000ff0) /* <4:11> R:RW:100: */
#define PDSS_AFC_1_CTRL_RESET_UI_COUNT_POS                  (4)


/*
 * Master will update its transmission rate based on the rate at which ping
 * is received (total Ping duration /16 = 1UI)
 */
#define PDSS_AFC_1_CTRL_UPDATE_TXCLK                        (1u << 12) /* <12:12> R:RW:0: */


/*
 * No of Bytes to be transmitted as response to V_I_Byte request from Master
 */
#define PDSS_AFC_1_CTRL_NO_OF_BYTES_TX_MASK                 (0x0003e000) /* <13:17> R:RW:1: */
#define PDSS_AFC_1_CTRL_NO_OF_BYTES_TX_POS                  (13)


/*
 * This will selects either the AFC Hardware or FW to control the afc_tx_data/afc_tx_en
 * ports of the s8usbpd_chgdet_afc_top Hard IP.
 * 0: Hardware controls
 * 1: This option is for Testing/Char. FW controls.
 *     afc_tx_en    port of the s8usbpd_chgdet_afc_top is driven by AFC_TX_DATA_EN
 *     afc_tx_data port of the s8usbpd_chgdet_afc_top is driven by AFC_TX_DATA
 */
#define PDSS_AFC_1_CTRL_AFC_TX_DATA_SEL                     (1u << 18) /* <18:18> R:RW:0: */


/*
 * Refer to AFC_TX_DATA_SEL register
 */
#define PDSS_AFC_1_CTRL_AFC_TX_DATA_EN                      (1u << 19) /* <19:19> R:RW:0: */


/*
 * Refer to AFC_TX_DATA_SEL register
 */
#define PDSS_AFC_1_CTRL_AFC_TX_DATA                         (1u << 20) /* <20:20> R:RW:0: */


/*
 * AFC Charger Detect contrl 2 for logic
 */
#define PDSS_AFC_2_CTRL_ADDRESS                             (0x400a0150)
#define PDSS_AFC_2_CTRL                                     (*(volatile uint32_t *)(0x400a0150))
#define PDSS_AFC_2_CTRL_DEFAULT                             (0x00000051)

/*
 * Setting this bit will disable the update of the UI count on SOP/EOP pulses
 */
#define PDSS_AFC_2_CTRL_DISABLE_SYNC_ON_RX_BY_4_UI          (1u << 0) /* <0:0> R:RW:1: */


/*
 * Unit Interval duration in Clk_bch cycles
 */
#define PDSS_AFC_2_CTRL_UI_COUNT_MASK                       (0x000001fe) /* <1:8> R:RW:40: */
#define PDSS_AFC_2_CTRL_UI_COUNT_POS                        (1)


/*
 * Received Unit Interval
 */
#define PDSS_AFC_2_CTRL_RX_UI_COUNT_MASK                    (0x0001fe00) /* <9:16> RW:R:0: */
#define PDSS_AFC_2_CTRL_RX_UI_COUNT_POS                     (9)


/*
 * 1: During Receive use programmed UI_COUNT instead of RX_UI_COUNT
 * 0: Use calculated RX_UI_COUNT
 */
#define PDSS_AFC_2_CTRL_OVERRIDE_UI_FOR_RX                  (1u << 17) /* <17:17> R:RW:0: */


/*
 * 0: TX PING low is only driven for 1/2-UI
 * 1: TX PING low is only driven for 1-UI
 */
#define PDSS_AFC_2_CTRL_DRIVE_PING_LOW_1UI                  (1u << 18) /* <18:18> R:RW:0: */


/*
 * AFC Charger Detect Opcode contrl for logic
 */
#define PDSS_AFC_OPCODE_CTRL_ADDRESS                        (0x400a0160)
#define PDSS_AFC_OPCODE_CTRL                                (*(volatile uint32_t *)(0x400a0160))
#define PDSS_AFC_OPCODE_CTRL_DEFAULT                        (0x00000000)

/*
 * This register encodes the sequence of events to be sent/monitored on the
 * AFC DMINUS line as opcode.  An operation – sequence of opcodes- can be
 * 10 events long.
 * Each event is specified by a 3-bit opcode. Op_code0 is 2:0, Op_code1 is
 * 5:3 and so on to MSB.
 *                                          3’b000 – IDLE. No operation
 *                                          3’b001- TX_PING. Send a PING
 *                                          3’b010 – RX_PING. Wait for a
 * PING
 *                                          3’h011 – TX_DATA_M. Send Data
 * in Master Role.
 *                                          3’h100 – TX_DATA_S. Send Data
 * in Slave Role
 *                                          3’h101 – RX_DATA. Receive Data
 *
 *                                         The DATA phase includes – SOP,TX/RX
 * and EOP. Since EOP has a PING, there should be a RX_PING after DATA phase
 * if required.
 *                                         The last opcode should always
 * be IDLE.
 * The last op_code 29:27 is not available. Instead bit 29 is used to provide
 * an additional functionality. If this bit is set, on completion of the
 * sequence, HW restarts the same sequence. This can be used if the next
 * AFC transaction can happen before the FW can respond
 */
#define PDSS_AFC_OPCODE_CTRL_OP_CODE_MASK                   (0x3fffffff) /* <0:29> R:RW:0: */
#define PDSS_AFC_OPCODE_CTRL_OP_CODE_POS                    (0)


/*
 * After programming the sequence, FW should set this bit to trigger the
 * execution. On completion of execution or any error event, this bit will
 * be cleared by HW
 */
#define PDSS_AFC_OPCODE_CTRL_OP_CODE_START                  (1u << 31) /* <31:31> RW1C:RW1S:0: */


/*
 * AFC Charger Detect Ping Pong register
 */
#define PDSS_AFC_PING_PONG_ADDRESS                          (0x400a0170)
#define PDSS_AFC_PING_PONG                                  (*(volatile uint32_t *)(0x400a0170))
#define PDSS_AFC_PING_PONG_DEFAULT                          (0x00000000)

/*
 * AFC TX/RX Data
 */
#define PDSS_AFC_PING_PONG_AFC_DATA_MASK                    (0x0000ffff) /* <0:15> RW:RW:0: */
#define PDSS_AFC_PING_PONG_AFC_DATA_POS                     (0)


/*
 * Slave ( Dedicated Charger) Control Registers for QC3.0
 */
#define PDSS_QC3_CHRGER_CTRL_ADDRESS                        (0x400a0180)
#define PDSS_QC3_CHRGER_CTRL                                (*(volatile uint32_t *)(0x400a0180))
#define PDSS_QC3_CHRGER_CTRL_DEFAULT                        (0x00000000)

/*
 * Setting this bit to 1 will enable the valid value in DP_PULSE_COUNT.
 * Hardware will clear this bit once it deposits the correct value in
 * DP_PULSE_COUNT/ DM_PULSE_COUNT register.
 */
#define PDSS_QC3_CHRGER_CTRL_READ_DPDM_COUNT                (1u << 0) /* <0:0> RW1C:RW:0: */


/*
 * Master Enable for hardware to start sensing the pulses on Dplus and Dminus.
 */
#define PDSS_QC3_CHRGER_CTRL_QC3_0_CHG_EN                   (1u << 8) /* <8:8> R:RW:0: */


/*
 * Pulse count received on DPlus. Only valid when the READ_DPDM_COUNT bit
 * is 0
 */
#define PDSS_QC3_CHRGER_CTRL_DP_PULSE_COUNT_MASK            (0x00003e00) /* <9:13> RW:R:0: */
#define PDSS_QC3_CHRGER_CTRL_DP_PULSE_COUNT_POS             (9)


/*
 * Pulse count received on DPlus. Only valid when the READ_DPDM_COUNT bit
 * is 0
 */
#define PDSS_QC3_CHRGER_CTRL_DM_PULSE_COUNT_MASK            (0x0007c000) /* <14:18> RW:R:0: */
#define PDSS_QC3_CHRGER_CTRL_DM_PULSE_COUNT_POS             (14)


/*
 * Register used to control requests made by QC3.0 Portable Device
 */
#define PDSS_QC3_DEVICE_CTRL_ADDRESS                        (0x400a0190)
#define PDSS_QC3_DEVICE_CTRL                                (*(volatile uint32_t *)(0x400a0190))
#define PDSS_QC3_DEVICE_CTRL_DEFAULT                        (0x00000000)

/*
 * Enable to send the request. This will be cleared by hardware once it starts
 * the state-machine
 */
#define PDSS_QC3_DEVICE_CTRL_QC3_SEND_REQUEST               (1u << 0) /* <0:0> RW1C:RW:0: */


/*
 * Pulse width of the Dplus or Dminus Signal for making requests. The charger_det_clock’s
 * period will be used to program this value. Spec requires 200usec-15 msec
 * (any value can be chosen).
 */
#define PDSS_QC3_DEVICE_CTRL_TACTIVE_TIME_MASK              (0x000001fe) /* <1:8> R:RW:0: */
#define PDSS_QC3_DEVICE_CTRL_TACTIVE_TIME_POS               (1)


/*
 * Inactive Period Before a new pulse is sent. Spec requires 100-200 usec
 * timing between two active pulses. Again the count will be function of
 * the  charger_det_clock’s period
 */
#define PDSS_QC3_DEVICE_CTRL_TINACTIVE_TIME_MASK            (0x0001fe00) /* <9:16> R:RW:0: */
#define PDSS_QC3_DEVICE_CTRL_TINACTIVE_TIME_POS             (9)


/*
 * No of increment/decrement requests firmware wants to send on Dplus or
 * Dminus ( controlled by selection DPNDM)
 */
#define PDSS_QC3_DEVICE_CTRL_PULSE_COUNT_MASK               (0x01fe0000) /* <17:24> R:RW:0: */
#define PDSS_QC3_DEVICE_CTRL_PULSE_COUNT_POS                (17)


/*
 * 1: Increment request pulses are sent on Dplus
 * 0: Decrement request pulses are sent on Dminus
 */
#define PDSS_QC3_DEVICE_CTRL_DPNDM                          (1u << 25) /* <25:25> R:RW:0: */


/*
 * AFC LOGIC Status
 */
#define PDSS_AFC_SM_STATUS_ADDRESS                          (0x400a01a0)
#define PDSS_AFC_SM_STATUS                                  (*(volatile uint32_t *)(0x400a01a0))
#define PDSS_AFC_SM_STATUS_DEFAULT                          (0x00000000)

/*
 * This is updated whenever the state machine enters the IDLE state. The
 * last state is captured from where the state machine exited.
 */
#define PDSS_AFC_SM_STATUS_LAST_AFC_SM_STATE_MASK           (0x00000007) /* <0:2> RW:R:0: */
#define PDSS_AFC_SM_STATUS_LAST_AFC_SM_STATE_POS            (0)


/*
 * This provides bit wise status of parity errors received on incoming bits.
 * The status is cleared by Hardware whenever it exits the IDLE state and
 * is updated on every end of byte parity reception
 */
#define PDSS_AFC_SM_STATUS_MASTER_PARITY_ERROR_STATUS_MASK    (0x0007fff8) /* <3:18> RW:R:0: */
#define PDSS_AFC_SM_STATUS_MASTER_PARITY_ERROR_STATUS_POS    (3)


/*
 * This indicates the number of bytes received from slave as a result of
 * request sent by master. This is cleared whenever master exits the idle
 * state.
 */
#define PDSS_AFC_SM_STATUS_MASTER_NUM_OF_SLAVE_BYTES_RCVD_MASK    (0x00f80000) /* <19:23> RW:R:0: */
#define PDSS_AFC_SM_STATUS_MASTER_NUM_OF_SLAVE_BYTES_RCVD_POS    (19)


/*
 * This field shows the current opcode in execution
 */
#define PDSS_AFC_SM_STATUS_CURR_OP_CODE_MASK                (0x0f000000) /* <24:27> RW:R:0: */
#define PDSS_AFC_SM_STATUS_CURR_OP_CODE_POS                 (24)


/*
 * BCH_DET logic HS filter config
 */
#define PDSS_AFC_HS_FILTER_CFG_ADDRESS                      (0x400a01b0)
#define PDSS_AFC_HS_FILTER_CFG                              (*(volatile uint32_t *)(0x400a01b0))
#define PDSS_AFC_HS_FILTER_CFG_DEFAULT                      (0x00000000)

/*
 * 1: Filter is enabled
 */
#define PDSS_AFC_HS_FILTER_CFG_DP_FILT_EN                   (1u << 0) /* <0:0> R:RW:0: */


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_AFC_HS_FILTER_CFG_DP_FILT_RESET                (1u << 1) /* <1:1> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_AFC_HS_FILTER_CFG_DP_FILT_BYPASS               (1u << 2) /* <2:2> R:RW:0: */


/*
 * #of clock CLK_FILTER filtering. Should be programmed before FILTER is
 * enabled.
 */
#define PDSS_AFC_HS_FILTER_CFG_DP_FILT_SEL_MASK             (0x00000ff8) /* <3:11> R:RW:0: */
#define PDSS_AFC_HS_FILTER_CFG_DP_FILT_SEL_POS              (3)


/*
 * 1: Filter is enabled
 */
#define PDSS_AFC_HS_FILTER_CFG_DM_FILT_EN                   (1u << 16) /* <16:16> R:RW:0: */


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_AFC_HS_FILTER_CFG_DM_FILT_RESET                (1u << 17) /* <17:17> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_AFC_HS_FILTER_CFG_DM_FILT_BYPASS               (1u << 18) /* <18:18> R:RW:0: */


/*
 * #of clock CLK_FILTER filtering. Should be programmed before FILTER is
 * enabled.
 */
#define PDSS_AFC_HS_FILTER_CFG_DM_FILT_SEL_MASK             (0x0ff80000) /* <19:27> R:RW:0: */
#define PDSS_AFC_HS_FILTER_CFG_DM_FILT_SEL_POS              (19)


/*
 * ADC1-4 SAR Control Register
 * General Purpose voltgae measurement, Temperature Sceining
 */
#define PDSS_ADC_SAR_CTRL_ADDRESS                           (0x400a01c0)
#define PDSS_ADC_SAR_CTRL                                   (*(volatile uint32_t *)(0x400a01c0))
#define PDSS_ADC_SAR_CTRL_DEFAULT                           (0x00008000)

/*
 * Setting this bit will enable the HW SAR logic.
 * Once the SAR_EN is one, Hardware will update the  SAR_OUT register after
 * 8 cycles of clk_sar and clear this register.
 */
#define PDSS_ADC_SAR_CTRL_SAR_EN                            (1u << 0) /* <0:0> RW1C:RW1S:0: */


/*
 * ADC starting mid value
 */
#define PDSS_ADC_SAR_CTRL_MID_VAL_MASK                      (0x0000ff00) /* <8:15> R:RW:128: */
#define PDSS_ADC_SAR_CTRL_MID_VAL_POS                       (8)


/*
 * ADC output resistance value
 * Stored 8-bit ADC value after the ID Pin voltage is sampled.
 */
#define PDSS_ADC_SAR_CTRL_SAR_OUT_MASK                      (0x00ff0000) /* <16:23> RW:R:0: */
#define PDSS_ADC_SAR_CTRL_SAR_OUT_POS                       (16)


/*
 * RefGen Sel8, Sel6 control
 */
#define PDSS_REFGEN_SEL6_SEL8_CTRL_ADDRESS                  (0x400a01e0)
#define PDSS_REFGEN_SEL6_SEL8_CTRL                          (*(volatile uint32_t *)(0x400a01e0))
#define PDSS_REFGEN_SEL6_SEL8_CTRL_DEFAULT                  (0x00000000)

/*
 * 0: HW controls the RefGen HardIP SEL6 port based on the PFC comparator
 * output.
 *     PFC comparator output 1:   RefGen HardIP SEL6 == REFGEN_2_CTRL.SEL7
 *     PFC comparator output 0:   RefGen HardIP SEL6 == REFGEN_2_CTRL.SEL6
 * 1: FW controls the RefGen HardIP SEL6 port using REFGEN_2_CTRL.SEL6 register
 */
#define PDSS_REFGEN_SEL6_SEL8_CTRL_PFC_SEL6                 (1u << 0) /* <0:0> R:RW:0: */


/*
 * 0: HW controls the RefGen HardIP SEL8 port based on the SR  comparator
 * output.
 *     SR  comparator output 1:   RefGen HardIP SEL8 == REFGEN_3_CTRL.SEL9
 *     SR  comparator output 0:   RefGen HardIP SEL8 == REFGEN_3_CTRL.SEL8
 * 1: FW controls the RefGen HardIP SEL8 port using REFGEN_3_CTRL.SEL8 register
 */
#define PDSS_REFGEN_SEL6_SEL8_CTRL_SR_SEL8                  (1u << 1) /* <1:1> R:RW:0: */


/*
 * Wakeup Interrupts edge and filter configuration
 */
#define PDSS_INTR1_CFG_ADDRESS                              (0x400a02a0)
#define PDSS_INTR1_CFG                                      (*(volatile uint32_t *)(0x400a02a0))
#define PDSS_INTR1_CFG_DEFAULT                              (0x00000000)

/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR1_CFG_VCMP_LA_CFG_MASK                     (0x00300000) /* <20:21> R:RW:0:VCMP_LA_INTR_EN */
#define PDSS_INTR1_CFG_VCMP_LA_CFG_POS                      (20)


/*
 * CC1/CC2 Wakeup Interrupts edge and filter configuration
 */
#define PDSS_INTR1_CFG_CC1_CC2_LS_ADDRESS                   (0x400a02a4)
#define PDSS_INTR1_CFG_CC1_CC2_LS                           (*(volatile uint32_t *)(0x400a02a4))
#define PDSS_INTR1_CFG_CC1_CC2_LS_DEFAULT                   (0x00000000)

/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR1_CFG_CC1_CC2_LS_CC1_FILT_EN               (1u << 0) /* <0:0> R:RW:0:CC1_INTR_EN */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR1_CFG_CC1_CC2_LS_CC1_CFG_MASK              (0x00000006) /* <1:2> R:RW:0:CC1_INTR_EN */
#define PDSS_INTR1_CFG_CC1_CC2_LS_CC1_CFG_POS               (1)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR1_CFG_CC1_CC2_LS_CC1_FILT_RESET            (1u << 3) /* <3:3> R:RW:0:CC1_INTR_EN */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR1_CFG_CC1_CC2_LS_CC1_FILT_BYPASS           (1u << 4) /* <4:4> R:RW:0:CC1_INTR_EN */


/*
 * #of clock CLK_LF filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR1_CFG_CC1_CC2_LS_CC1_FILT_SEL_MASK         (0x000003e0) /* <5:9> R:RW:0:CC1_INTR_EN */
#define PDSS_INTR1_CFG_CC1_CC2_LS_CC1_FILT_SEL_POS          (5)


/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR1_CFG_CC1_CC2_LS_CC2_FILT_EN               (1u << 10) /* <10:10> R:RW:0:CC2_INTR_EN */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR1_CFG_CC1_CC2_LS_CC2_CFG_MASK              (0x00001800) /* <11:12> R:RW:0:CC2_INTR_EN */
#define PDSS_INTR1_CFG_CC1_CC2_LS_CC2_CFG_POS               (11)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR1_CFG_CC1_CC2_LS_CC2_FILT_RESET            (1u << 13) /* <13:13> R:RW:0:CC2_INTR_EN */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR1_CFG_CC1_CC2_LS_CC2_FILT_BYPASS           (1u << 14) /* <14:14> R:RW:0:CC2_INTR_EN */


/*
 * #of clock CLK_LF filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR1_CFG_CC1_CC2_LS_CC2_FILT_SEL_MASK         (0x000f8000) /* <15:19> R:RW:0:CC2_INTR_EN */
#define PDSS_INTR1_CFG_CC1_CC2_LS_CC2_FILT_SEL_POS          (15)


/*
 * VCMP_UP/DOWN Wakeup Interrupts edge and filter configuration
 */
#define PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_ADDRESS              (0x400a02a8)
#define PDSS_INTR1_CFG_VCMP_UP_DOWN_LS                      (*(volatile uint32_t *)(0x400a02a8))
#define PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_DEFAULT              (0x00000000)

/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_EN      (1u << 0) /* <0:0> R:RW:0:VCMP_UP_INTR_EN */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_CFG_MASK     (0x00000006) /* <1:2> R:RW:0:VCMP_UP_INTR_EN */
#define PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_CFG_POS      (1)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_RESET    (1u << 3) /* <3:3> R:RW:0:VCMP_UP_INTR_EN */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_BYPASS    (1u << 4) /* <4:4> R:RW:0:VCMP_UP_INTR_EN */


/*
 * #of clock CLK_LF filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_SEL_MASK    (0x000003e0) /* <5:9> R:RW:0:VCMP_UP_INTR_EN */
#define PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_SEL_POS    (5)


/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_EN      (1u << 10) /* <10:10> R:RW:0:VCMP_DN_INTR_EN */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_CFG_MASK     (0x00001800) /* <11:12> R:RW:0:VCMP_DN_INTR_EN */
#define PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_CFG_POS      (11)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_RESET    (1u << 13) /* <13:13> R:RW:0:VCMP_DN_INTR_EN */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_BYPASS    (1u << 14) /* <14:14> R:RW:0:VCMP_DN_INTR_EN */


/*
 * #of clock CLK_LF filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_SEL_MASK    (0x000f8000) /* <15:19> R:RW:0:VCMP_DN_INTR_EN */
#define PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_SEL_POS    (15)


/*
 * INTR1 Status
 */
#define PDSS_INTR1_STATUS_ADDRESS                           (0x400a02b4)
#define PDSS_INTR1_STATUS                                   (*(volatile uint32_t *)(0x400a02b4))
#define PDSS_INTR1_STATUS_DEFAULT                           (0x00000000)

/*
 * CC1 status (wakeup interrupt from deepsleep)
 * 1: CC1 attached
 * 0: CC1 detached
 */
#define PDSS_INTR1_STATUS_CC1_STATUS                        (1u << 4) /* <4:4> RW:R:0:CC1_INTR_EN */


/*
 * CC1 Filtered output
 */
#define PDSS_INTR1_STATUS_CC1_FILT                          (1u << 5) /* <5:5> RW:R:0:CC1_INTR_EN */


/*
 * CC2 status (wakeup interrupt from deepsleep)
 * 1: CC2 attached
 * 0: CC2 detached
 */
#define PDSS_INTR1_STATUS_CC2_STATUS                        (1u << 6) /* <6:6> RW:R:0:CC2_INTR_EN */


/*
 * CC2 Filtered output
 */
#define PDSS_INTR1_STATUS_CC2_FILT                          (1u << 7) /* <7:7> RW:R:0:CC2_INTR_EN */


/*
 * This register provides the status CC_LINE_ACTIVITY (wakeup interrupt from
 * deepsleep).
 */
#define PDSS_INTR1_STATUS_VCMP_LA_STATUS                    (1u << 8) /* <8:8> RW:R:0:VCMP_LA_INTR_EN */


/*
 * VCMP_LA Filtered output
 */
#define PDSS_INTR1_STATUS_VCMP_LA_FILT                      (1u << 9) /* <9:9> RW:R:0:VCMP_LA_INTR_EN */


/*
 * This register provides the status of VCMP_UP (wakeup interrupt from deepsleep).
 * Not enabled unless CMP_EN bit is set.
 * Edge: Ra/Rd value changed
 * {VCMP_UP, VCMP_DN}:
 * 00: Ra connected
 * 01: Rd connected
 * 11: Nothing connected (float)
 *                OR
 * Edge: Rp value changed
 * {VCMP_UP, VCMP_DN}:
 * 00: Default Rp broadcast
 * 01: 1.5A Rp broadcast
 * 11: 3.0A Rp broadcast
 */
#define PDSS_INTR1_STATUS_VCMP_UP_STATUS                    (1u << 10) /* <10:10> RW:R:0:VCMP_UP_INTR_EN */


/*
 * VCMP_UP Filtered output
 */
#define PDSS_INTR1_STATUS_VCMP_UP_FILT                      (1u << 11) /* <11:11> RW:R:0:VCMP_UP_INTR_EN */


/*
 * This register provides the status of VCMP_DN (wakeup interrupt from deepsleep).
 * Not enabled unless CMP_EN bit is set.
 * Edge: Ra/Rd value changed
 *                OR
 * Edge: Rp value changed
 */
#define PDSS_INTR1_STATUS_VCMP_DN_STATUS                    (1u << 12) /* <12:12> RW:R:0:VCMP_DN_INTR_EN */


/*
 * VCMP_DN Filtered output
 */
#define PDSS_INTR1_STATUS_VCMP_DN_FILT                      (1u << 13) /* <13:13> RW:R:0:VCMP_DN_INTR_EN */


/*
 * INTR1 Cause.  These are the wakeup interrupts get reflected on interrupt_wakeup
 * pin.
 * The configurations for using the comparators:
 *
 * DFP waiting for attach:
 * vcmp_up connected to CC1: HI = no connect, LO = attach
 * vcmp_dn connected to CC2: HI = no connect, LO = attach
 *
 * DFP after attach:
 * vcmup_up at detach threshold: HI = detach, LO = attach
 * vcmp_dn at Rd/Ra threshold: HI = Rd connected, LO = Ra connected
 * vcmp_la at CC line activity threshold: HI = no activity, LO/Toggling =
 * activity
 *
 * UFP (with VBUS present):
 * vcmup_up at Default/1.5A threshold: HI = Default, LO = 1.5A
 * vcmp_dn at 1.5A/3.0A threshold: HI = 1.5A, LO = 3.0A
 * vcmp_la at CC line activity threshold: HI = no activity, LO/Toggling =
 * activity
 *
 * For detecting the difference between Rd/Ra, firmware will have to check
 * the “DFP after attach” state above to determine it.
 */
#define PDSS_INTR1_ADDRESS                                  (0x400a02b8)
#define PDSS_INTR1                                          (*(volatile uint32_t *)(0x400a02b8))
#define PDSS_INTR1_DEFAULT                                  (0x00000000)

/*
 * CC1 changed (wakeup interrupt from deepsleep)
 * Check the STATUS.CC1_STATUS value
 */
#define PDSS_INTR1_CC1_CHANGED                              (1u << 2) /* <2:2> RW1S:RW1C:0:CC1_INTR_EN */


/*
 * CC2 changed (wakeup interrupt from deepsleep)
 * Check the STATUS.CC2_STATUS value
 */
#define PDSS_INTR1_CC2_CHANGED                              (1u << 3) /* <3:3> RW1S:RW1C:0:CC2_INTR_EN */


/*
 * VCMP_LA changed (wakeup interrupt from deepsleep)
 * Check the STATUS.VCMP_LA_STATUS value
 */
#define PDSS_INTR1_VCMP_LA_CHANGED                          (1u << 4) /* <4:4> RW1S:RW1C:0:VCMP_LA_INTR_EN */


/*
 * VCMP_UP changed (wakeup interrupt from deepsleep)
 * Check the STATUS.VCMP_UP_STATUS value
 */
#define PDSS_INTR1_VCMP_UP_CHANGED                          (1u << 5) /* <5:5> RW1S:RW1C:0:VCMP_UP_INTR_EN */


/*
 * VCMP_DN changed (wakeup interrupt from deepsleep)
 * Check the STATUS.VCMP_DN_STATUS value
 */
#define PDSS_INTR1_VCMP_DN_CHANGED                          (1u << 6) /* <6:6> RW1S:RW1C:0:VCMP_DN_INTR_EN */


/*
 * LF Interrupt Request.  This bit is set each time LF_CNTR==LF_MATCH.
 * Due to internal synchronization, it takes 2 SYSCLK cycles to update after
 * a W1C.
 */
#define PDSS_INTR1_LF_CNTR_MATCH                            (1u << 19) /* <19:19> RW1S:RW1C:0:LF_CNTR_EN */


/*
 * INTR1 Set
 */
#define PDSS_INTR1_SET_ADDRESS                              (0x400a02bc)
#define PDSS_INTR1_SET                                      (*(volatile uint32_t *)(0x400a02bc))
#define PDSS_INTR1_SET_DEFAULT                              (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_SET_CC1_CHANGED                          (1u << 2) /* <2:2> A:RW1S:0:CC1_INTR_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_SET_CC2_CHANGED                          (1u << 3) /* <3:3> A:RW1S:0:CC2_INTR_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_SET_VCMP_LA_CHANGED                      (1u << 4) /* <4:4> A:RW1S:0:VCMP_LA_INTR_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_SET_VCMP_UP_CHANGED                      (1u << 5) /* <5:5> A:RW1S:0:VCMP_UP_INTR_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_SET_VCMP_DN_CHANGED                      (1u << 6) /* <6:6> A:RW1S:0:VCMP_DN_INTR_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_SET_LF_CNTR_MATCH                        (1u << 19) /* <19:19> A:RW1S:0:LF_CNTR_EN */


/*
 * INTR1 Mask
 */
#define PDSS_INTR1_MASK_ADDRESS                             (0x400a02c0)
#define PDSS_INTR1_MASK                                     (*(volatile uint32_t *)(0x400a02c0))
#define PDSS_INTR1_MASK_DEFAULT                             (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_MASK_CC1_CHANGED_MASK                    (1u << 2) /* <2:2> R:RW:0:CC1_INTR_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_MASK_CC2_CHANGED_MASK                    (1u << 3) /* <3:3> R:RW:0:CC2_INTR_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_MASK_VCMP_LA_CHANGED_MASK                (1u << 4) /* <4:4> R:RW:0:VCMP_LA_INTR_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_MASK_VCMP_UP_CHANGED_MASK                (1u << 5) /* <5:5> R:RW:0:VCMP_UP_INTR_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_MASK_VCMP_DN_CHANGED_MASK                (1u << 6) /* <6:6> R:RW:0:VCMP_DN_INTR_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_MASK_LF_CNTR_MASK                        (1u << 19) /* <19:19> R:RW:0:LF_CNTR_EN */


/*
 * INTR1 Masked
 */
#define PDSS_INTR1_MASKED_ADDRESS                           (0x400a02c4)
#define PDSS_INTR1_MASKED                                   (*(volatile uint32_t *)(0x400a02c4))
#define PDSS_INTR1_MASKED_DEFAULT                           (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR1_MASKED_CC1_CHANGED_MASKED                (1u << 2) /* <2:2> RW:R:0:CC1_INTR_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR1_MASKED_CC2_CHANGED_MASKED                (1u << 3) /* <3:3> RW:R:0:CC2_INTR_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR1_MASKED_VCMP_LA_CHANGED_MASKED            (1u << 4) /* <4:4> RW:R:0:VCMP_LA_INTR_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR1_MASKED_VCMP_UP_CHANGED_MASKED            (1u << 5) /* <5:5> RW:R:0:VCMP_UP_INTR_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR1_MASKED_VCMP_DN_CHANGED_MASKED            (1u << 6) /* <6:6> RW:R:0:VCMP_DN_INTR_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR1_MASKED_LF_CNTR_MASKED                    (1u << 19) /* <19:19> RW:R:0:LF_CNTR_EN */


/*
 * ADC1-4 High/Low Speed Filter and Edge detector configuration
 */
#define PDSS_INTR3_CFG_ADC_HS_ADDRESS                       (0x400a0304)
#define PDSS_INTR3_CFG_ADC_HS                               (*(volatile uint32_t *)(0x400a0304))
#define PDSS_INTR3_CFG_ADC_HS_DEFAULT                       (0x00000400)

/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR3_CFG_ADC_HS_FILT_EN                       (1u << 0) /* <0:0> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR3_CFG_ADC_HS_FILT_CFG_MASK                 (0x00000006) /* <1:2> R:RW:0: */
#define PDSS_INTR3_CFG_ADC_HS_FILT_CFG_POS                  (1)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR3_CFG_ADC_HS_FILT_RESET                    (1u << 3) /* <3:3> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR3_CFG_ADC_HS_FILT_BYPASS                   (1u << 4) /* <4:4> R:RW:0: */


/*
 * #of clock CLK_FILTER filtering. Should be programmed before FILTER is
 * enabled.
 */
#define PDSS_INTR3_CFG_ADC_HS_FILT_SEL_MASK                 (0x000003e0) /* <5:9> R:RW:0: */
#define PDSS_INTR3_CFG_ADC_HS_FILT_SEL_POS                  (5)


/*
 * This bit enables CPU to bypass the Filter when the part is in deep-sleep
 * state.  This over-rides the FILTER_EN settings during DEEP SLEEP state
 * ONLY.
 * The bit should be set by CPU only when its using the filter with high
 * frequency active clock and wants to wakeup from deep sleep on the transition
 * of the incoming signal.
 * This bit if set also disables the AUTO shutoff logic in DEEP SLEEP state.
 */
#define PDSS_INTR3_CFG_ADC_HS_DPSLP_MODE                    (1u << 10) /* <10:10> R:RW:1: */


/*
 * INTR3 Status 0
 */
#define PDSS_INTR3_STATUS_0_ADDRESS                         (0x400a0340)
#define PDSS_INTR3_STATUS_0                                 (*(volatile uint32_t *)(0x400a0340))
#define PDSS_INTR3_STATUS_0_DEFAULT                         (0x00000000)

/*
 * The status of cmp_out from the ADC#1-4
 */
#define PDSS_INTR3_STATUS_0_CMP_OUT_STATUS                  (1u << 2) /* <2:2> RW:R:0:ADC_NUM */


/*
 * ADC1-4 COMP_OUT Filtered output
 */
#define PDSS_INTR3_STATUS_0_CMP_OUT_FILT                    (1u << 6) /* <6:6> RW:R:0:ADC_NUM */


/*
 * INTR3 interrupt Cause.  These are the wakeup interrupts get reflected
 * on interrupt_wakeup pin.
 */
#define PDSS_INTR3_ADDRESS                                  (0x400a0348)
#define PDSS_INTR3                                          (*(volatile uint32_t *)(0x400a0348))
#define PDSS_INTR3_DEFAULT                                  (0x00000000)

/*
 * CMP_OUT1-4 changed (wakeup interrupt from deepsleep)
 * Check the INTR3_STATUS.ADC_CMP_OUT_STATUS value
 */
#define PDSS_INTR3_CMP_OUT_CHANGED                          (1u << 1) /* <1:1> RW1S:RW1C:0:ADC_NUM */


/*
 * INTR3 Interrupt Set
 */
#define PDSS_INTR3_SET_ADDRESS                              (0x400a034c)
#define PDSS_INTR3_SET                                      (*(volatile uint32_t *)(0x400a034c))
#define PDSS_INTR3_SET_DEFAULT                              (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR3_SET_CMP_OUT_CHANGED                      (1u << 1) /* <1:1> A:RW1S:0:ADC_NUM */


/*
 * INTR3 interrupt Mask
 */
#define PDSS_INTR3_MASK_ADDRESS                             (0x400a0350)
#define PDSS_INTR3_MASK                                     (*(volatile uint32_t *)(0x400a0350))
#define PDSS_INTR3_MASK_DEFAULT                             (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR3_MASK_CMP_OUT_CHANGED_MASK                (1u << 1) /* <1:1> R:RW:0:ADC_NUM */


/*
 * INTR3 interrupt Masked
 */
#define PDSS_INTR3_MASKED_ADDRESS                           (0x400a0354)
#define PDSS_INTR3_MASKED                                   (*(volatile uint32_t *)(0x400a0354))
#define PDSS_INTR3_MASKED_DEFAULT                           (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR3_MASKED_CMP_OUT_CHANGED_MASKED            (1u << 1) /* <1:1> RW:R:0:ADC_NUM */


/*
 * CLK_FILTER (Refer to CTRL.SEL_CLK_FILTER) Filter configuration
 * Refer to COMP_CTRL, COMP_TR_CTRL comments for the mapping
 */
#define PDSS_INTR5_FILTER_CFG_ADDRESS(n)                    (0x400a0380 + ((n) * (0x0004)))
#define PDSS_INTR5_FILTER_CFG(n)                            (*(volatile uint32_t *)(0x400a0380 + ((n) * 0x0004)))
#define PDSS_INTR5_FILTER_CFG_DEFAULT                       (0x00000400)

/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR5_FILTER_CFG_FILT_EN                       (1u << 0) /* <0:0> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR5_FILTER_CFG_FILT_CFG_MASK                 (0x00000006) /* <1:2> R:RW:0: */
#define PDSS_INTR5_FILTER_CFG_FILT_CFG_POS                  (1)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR5_FILTER_CFG_FILT_RESET                    (1u << 3) /* <3:3> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR5_FILTER_CFG_FILT_BYPASS                   (1u << 4) /* <4:4> R:RW:0: */


/*
 * #of clock CLK_FILTER filtering. Should be programmed before FILTER is
 * enabled.
 */
#define PDSS_INTR5_FILTER_CFG_FILT_SEL_MASK                 (0x000003e0) /* <5:9> R:RW:0: */
#define PDSS_INTR5_FILTER_CFG_FILT_SEL_POS                  (5)


/*
 * This bit enables CPU to bypass the Filter when the part is in deep-sleep
 * state.  This over-rides the FILTER_EN settings during DEEP SLEEP state
 * ONLY.
 * The bit should be set by CPU only when its using the filter with high
 * frequency active clock and wants to wakeup from deep sleep on the transition
 * of the incoming signal.
 * This bit if set also disables the AUTO shutoff logic in DEEP SLEEP state.
 */
#define PDSS_INTR5_FILTER_CFG_DPSLP_MODE                    (1u << 10) /* <10:10> R:RW:1: */


/*
 * INTR5 Status 0
 */
#define PDSS_INTR5_STATUS_0_ADDRESS                         (0x400a03e0)
#define PDSS_INTR5_STATUS_0                                 (*(volatile uint32_t *)(0x400a03e0))
#define PDSS_INTR5_STATUS_0_DEFAULT                         (0x00000000)

/*
 * The status of HS/LS filter edge detectors 1-12
 */
#define PDSS_INTR5_STATUS_0_STATUS_12_MASK                  (0x0000003f) /* <0:5> RW:R:0:CLK_FILTER_FILT_NUM_LOG1_12 */
#define PDSS_INTR5_STATUS_0_STATUS_12_POS                   (0)


/*
 * 1-12 HS/LS Filtered output
 */
#define PDSS_INTR5_STATUS_0_FILT_12_MASK                    (0x0003f000) /* <12:17> RW:R:0:CLK_FILTER_FILT_NUM_LOG1_12 */
#define PDSS_INTR5_STATUS_0_FILT_12_POS                     (12)


/*
 * INTR5 interrupt Cause.  These are the wakeup interrupts get reflected
 * on interrupt_wakeup pin from HS filters
 */
#define PDSS_INTR5_ADDRESS                                  (0x400a03e8)
#define PDSS_INTR5                                          (*(volatile uint32_t *)(0x400a03e8))
#define PDSS_INTR5_DEFAULT                                  (0x00000000)

/*
 * Change in edge of HS filter edge detectors (wakeup interrupt from deepsleep)
 * Check the  INTR5_STATUS_0/1.STATUS_12/24 value
 */
#define PDSS_INTR5_EDGE_CHANGED_MASK                        (0x0000003f) /* <0:5> RW1S:RW1C:0:CLK_FILTER_LOG1 */
#define PDSS_INTR5_EDGE_CHANGED_POS                         (0)


/*
 * INTR5 Interrupt Set
 */
#define PDSS_INTR5_SET_ADDRESS                              (0x400a03ec)
#define PDSS_INTR5_SET                                      (*(volatile uint32_t *)(0x400a03ec))
#define PDSS_INTR5_SET_DEFAULT                              (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR5_SET_EDGE_CHANGED_MASK                    (0x0000003f) /* <0:5> A:RW1S:0:CLK_FILTER_LOG1 */
#define PDSS_INTR5_SET_EDGE_CHANGED_POS                     (0)


/*
 * INTR5 interrupt Mask
 */
#define PDSS_INTR5_MASK_ADDRESS                             (0x400a03f0)
#define PDSS_INTR5_MASK                                     (*(volatile uint32_t *)(0x400a03f0))
#define PDSS_INTR5_MASK_DEFAULT                             (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR5_MASK_EDGE_CHANGED_MASK_MASK              (0x0000003f) /* <0:5> R:RW:0:CLK_FILTER_LOG1 */
#define PDSS_INTR5_MASK_EDGE_CHANGED_MASK_POS               (0)


/*
 * INTR5 interrupt Masked
 */
#define PDSS_INTR5_MASKED_ADDRESS                           (0x400a03f4)
#define PDSS_INTR5_MASKED                                   (*(volatile uint32_t *)(0x400a03f4))
#define PDSS_INTR5_MASKED_DEFAULT                           (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR5_MASKED_EDGE_CHANGED_MASKED_MASK          (0x0000003f) /* <0:5> RW:R:0:CLK_FILTER_LOG1 */
#define PDSS_INTR5_MASKED_EDGE_CHANGED_MASKED_POS           (0)


/*
 * CLK_LF ONLY Filter configuration
 * Refer to COMP_CTRL, COMP_TR_CTRL comments for the mapping
 */
#define PDSS_INTR7_FILTER_CFG_ADDRESS(n)                    (0x400a0420 + ((n) * (0x0004)))
#define PDSS_INTR7_FILTER_CFG(n)                            (*(volatile uint32_t *)(0x400a0420 + ((n) * 0x0004)))
#define PDSS_INTR7_FILTER_CFG_DEFAULT                       (0x00000000)

/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR7_FILTER_CFG_FILT_EN                       (1u << 0) /* <0:0> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR7_FILTER_CFG_FILT_CFG_MASK                 (0x00000006) /* <1:2> R:RW:0: */
#define PDSS_INTR7_FILTER_CFG_FILT_CFG_POS                  (1)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR7_FILTER_CFG_CLK_LF_FILT_RESET             (1u << 3) /* <3:3> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR7_FILTER_CFG_CLK_LF_FILT_BYPASS            (1u << 4) /* <4:4> R:RW:0: */


/*
 * #of clock CLK_FILTER filtering. Should be programmed before FILTER is
 * enabled.
 */
#define PDSS_INTR7_FILTER_CFG_CLK_LF_FILT_SEL_MASK          (0x000000e0) /* <5:7> R:RW:0: */
#define PDSS_INTR7_FILTER_CFG_CLK_LF_FILT_SEL_POS           (5)


/*
 * INTR7 Status
 */
#define PDSS_INTR7_STATUS_ADDRESS                           (0x400a0440)
#define PDSS_INTR7_STATUS                                   (*(volatile uint32_t *)(0x400a0440))
#define PDSS_INTR7_STATUS_DEFAULT                           (0x00000000)

/*
 * The status of LS filter edge detectors
 */
#define PDSS_INTR7_STATUS_STATUS_8_MASK                     (0x00000007) /* <0:2> RW:R:0:CLK_LF_FILT_NUM */
#define PDSS_INTR7_STATUS_STATUS_8_POS                      (0)


/*
 * 1-8 LS Filtered output
 */
#define PDSS_INTR7_STATUS_FILT_8_MASK                       (0x00000700) /* <8:10> RW:R:0:CLK_LF_FILT_NUM */
#define PDSS_INTR7_STATUS_FILT_8_POS                        (8)


/*
 * INTR7 interrupt Cause.  These are the wakeup interrupts get reflected
 * on interrupt_wakeup pin from LS filters
 */
#define PDSS_INTR7_ADDRESS                                  (0x400a0444)
#define PDSS_INTR7                                          (*(volatile uint32_t *)(0x400a0444))
#define PDSS_INTR7_DEFAULT                                  (0x00000000)

/*
 * Change in edge of HS filter edge detectors (wakeup interrupt from deepsleep)
 * Check the  INTR7_STATUS.STATUS_8 value
 */
#define PDSS_INTR7_CLK_LF_EDGE_CHANGED_MASK                 (0x00000007) /* <0:2> RW1S:RW1C:0:CLK_LF_FILT_NUM */
#define PDSS_INTR7_CLK_LF_EDGE_CHANGED_POS                  (0)


/*
 * INTR7 Interrupt Set
 */
#define PDSS_INTR7_SET_ADDRESS                              (0x400a0448)
#define PDSS_INTR7_SET                                      (*(volatile uint32_t *)(0x400a0448))
#define PDSS_INTR7_SET_DEFAULT                              (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR7_SET_CLK_LF_EDGE_CHANGED_MASK             (0x00000007) /* <0:2> A:RW1S:0:CLK_LF_FILT_NUM */
#define PDSS_INTR7_SET_CLK_LF_EDGE_CHANGED_POS              (0)


/*
 * INTR7 interrupt Mask
 */
#define PDSS_INTR7_MASK_ADDRESS                             (0x400a044c)
#define PDSS_INTR7_MASK                                     (*(volatile uint32_t *)(0x400a044c))
#define PDSS_INTR7_MASK_DEFAULT                             (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR7_MASK_CLK_LF_EDGE_CHANGED_MASK_MASK       (0x00000007) /* <0:2> R:RW:0:CLK_LF_FILT_NUM */
#define PDSS_INTR7_MASK_CLK_LF_EDGE_CHANGED_MASK_POS        (0)


/*
 * INTR7 interrupt Masked
 */
#define PDSS_INTR7_MASKED_ADDRESS                           (0x400a0450)
#define PDSS_INTR7_MASKED                                   (*(volatile uint32_t *)(0x400a0450))
#define PDSS_INTR7_MASKED_DEFAULT                           (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR7_MASKED_CLK_LF_EDGE_CHANGED_MASKED_MASK    (0x00000007) /* <0:2> RW:R:0:CLK_LF_FILT_NUM */
#define PDSS_INTR7_MASKED_CLK_LF_EDGE_CHANGED_MASKED_POS    (0)


/*
 * Battery Charger 1-4 Wakeup Interrupts edge and filter configuration
 */
#define PDSS_INTR9_CFG_BCH_DET_ADDRESS                      (0x400a0470)
#define PDSS_INTR9_CFG_BCH_DET                              (*(volatile uint32_t *)(0x400a0470))
#define PDSS_INTR9_CFG_BCH_DET_DEFAULT                      (0x00000000)

/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_EN        (1u << 0) /* <0:0> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_CFG_MASK       (0x00000006) /* <1:2> R:RW:0: */
#define PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_CFG_POS        (1)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_RESET     (1u << 3) /* <3:3> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_BYPASS    (1u << 4) /* <4:4> R:RW:0: */


/*
 * #of clock CLK_LF filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_SEL_MASK    (0x000000e0) /* <5:7> R:RW:0: */
#define PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_SEL_POS    (5)


/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_EN        (1u << 8) /* <8:8> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_CFG_MASK       (0x00000600) /* <9:10> R:RW:0: */
#define PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_CFG_POS        (9)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_RESET     (1u << 11) /* <11:11> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_BYPASS    (1u << 12) /* <12:12> R:RW:0: */


/*
 * #of clock CLK_LF filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_SEL_MASK    (0x0000e000) /* <13:15> R:RW:0: */
#define PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_SEL_POS    (13)


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR9_CFG_BCH_DET_QCOM_RCVR_DM_CFG_MASK        (0x00030000) /* <16:17> R:RW:0: */
#define PDSS_INTR9_CFG_BCH_DET_QCOM_RCVR_DM_CFG_POS         (16)


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR9_CFG_BCH_DET_QCOM_RCVR_DP_CFG_MASK        (0x000c0000) /* <18:19> R:RW:0: */
#define PDSS_INTR9_CFG_BCH_DET_QCOM_RCVR_DP_CFG_POS         (18)


/*
 * INTR9 Status 0
 */
#define PDSS_INTR9_STATUS_0_ADDRESS                         (0x400a048c)
#define PDSS_INTR9_STATUS_0                                 (*(volatile uint32_t *)(0x400a048c))
#define PDSS_INTR9_STATUS_0_DEFAULT                         (0x00000000)

/*
 * The status of battery charger comparators1,0
 */
#define PDSS_INTR9_STATUS_0_BCH_DET_STATUS_MASK             (0x00000003) /* <0:1> RW:R:0:BCH_DET_NUM_LOG2 */
#define PDSS_INTR9_STATUS_0_BCH_DET_STATUS_POS              (0)


/*
 * BCH_DET comp0/1 output  Filtered output
 */
#define PDSS_INTR9_STATUS_0_BCH_DET_FILT_MASK               (0x00000300) /* <8:9> RW:R:0:BCH_DET_NUM_LOG2 */
#define PDSS_INTR9_STATUS_0_BCH_DET_FILT_POS                (8)


/*
 * INTR9 Status 1
 */
#define PDSS_INTR9_STATUS_1_ADDRESS                         (0x400a0490)
#define PDSS_INTR9_STATUS_1                                 (*(volatile uint32_t *)(0x400a0490))
#define PDSS_INTR9_STATUS_1_DEFAULT                         (0x00000000)

/*
 * The status of qcom receiver dm
 */
#define PDSS_INTR9_STATUS_1_QCOM_RCVR_DM_STATUS             (1u << 8) /* <8:8> RW:R:0:BCH_DET_NUM */


/*
 * QCOM RCVR DM  Filtered output
 */
#define PDSS_INTR9_STATUS_1_QCOM_RCVR_DM_FILT               (1u << 12) /* <12:12> RW:R:0:BCH_DET_NUM */


/*
 * The status of qcom receiver dp
 */
#define PDSS_INTR9_STATUS_1_QCOM_RCVR_DP_STATUS             (1u << 16) /* <16:16> RW:R:0:BCH_DET_NUM */


/*
 * QCOM RCVR DP  Filtered output
 */
#define PDSS_INTR9_STATUS_1_QCOM_RCVR_DP_FILT               (1u << 20) /* <20:20> RW:R:0:BCH_DET_NUM */


/*
 * DM  Filtered output
 */
#define PDSS_INTR9_STATUS_1_DM_FILT                         (1u << 24) /* <24:24> RW:R:0:BCH_DET_NUM */


/*
 * DP  Filtered output
 */
#define PDSS_INTR9_STATUS_1_DP_FILT                         (1u << 28) /* <28:28> RW:R:0:BCH_DET_NUM */


/*
 * INTR9 interrupt Cause.  These are the wakeup interrupts get reflected
 * on interrupt_wakeup pin.
 */
#define PDSS_INTR9_ADDRESS                                  (0x400a0494)
#define PDSS_INTR9                                          (*(volatile uint32_t *)(0x400a0494))
#define PDSS_INTR9_DEFAULT                                  (0x00000000)

/*
 * Edge Detected (wakeup interrupt from deepsleep), COMP0-1=Bit0, COMP0-2=Bit1,
 *  COMP1-1=Bit2, COMP1-2=Bit3,
 * Check the  INTR9_STATUS.BCH_DET_STATUS value
 */
#define PDSS_INTR9_BCH_DET_CHANGED_MASK                     (0x00000003) /* <0:1> RW1S:RW1C:0:BCH_DET_NUM_LOG2 */
#define PDSS_INTR9_BCH_DET_CHANGED_POS                      (0)


/*
 * Edge Detected (wakeup interrupt from deepsleep)
 * Check the  INTR9_STATUS.QCOM_RCVR_DM_STATUS value
 */
#define PDSS_INTR9_QCOM_RCVR_DM_CHANGED                     (1u << 20) /* <20:20> RW1S:RW1C:0:BCH_DET_NUM */
#define PDSS_INTR9_QCOM_RCVR_DM_CHANGED_POS                 (20u)
#define PDSS_INTR9_QCOM_RCVR_DM_CHANGED_MASK                (1u << 20)
/*
 * Edge Detected (wakeup interrupt from deepsleep)
 * Check the  INTR9_STATUS.QCOM_RCVR_DM_STATUS value
 */
#define PDSS_INTR9_QCOM_RCVR_DP_CHANGED                     (1u << 24) /* <24:24> RW1S:RW1C:0:BCH_DET_NUM */
#define PDSS_INTR9_QCOM_RCVR_DP_CHANGED_POS                 (24u)
#define PDSS_INTR9_QCOM_RCVR_DP_CHANGED_MASK                (1u << 24)
/*
 * INTR9 Interrupt Set
 */
#define PDSS_INTR9_SET_ADDRESS                              (0x400a0498)
#define PDSS_INTR9_SET                                      (*(volatile uint32_t *)(0x400a0498))
#define PDSS_INTR9_SET_DEFAULT                              (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR9_SET_BCH_DET_CHANGED_MASK                 (0x00000003) /* <0:1> A:RW1S:0:BCH_DET_NUM_LOG2 */
#define PDSS_INTR9_SET_BCH_DET_CHANGED_POS                  (0)


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR9_SET_QCOM_RCVR_DM_CHANGED                 (1u << 20) /* <20:20> A:RW1S:0:BCH_DET_NUM */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR9_SET_QCOM_RCVR_DP_CHANGED                 (1u << 24) /* <24:24> A:RW1S:0:BCH_DET_NUM */


/*
 * INTR9 interrupt Mask
 */
#define PDSS_INTR9_MASK_ADDRESS                             (0x400a049c)
#define PDSS_INTR9_MASK                                     (*(volatile uint32_t *)(0x400a049c))
#define PDSS_INTR9_MASK_DEFAULT                             (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR9_MASK_BCH_DET_CHANGED_MASK_MASK           (0x00000003) /* <0:1> R:RW:0:BCH_DET_NUM_LOG2 */
#define PDSS_INTR9_MASK_BCH_DET_CHANGED_MASK_POS            (0)


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR9_MASK_QCOM_RCVR_DM_CHANGED_MASK           (1u << 20) /* <20:20> R:RW:0:BCH_DET_NUM */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR9_MASK_QCOM_RCVR_DP_CHANGED_MASK           (1u << 24) /* <24:24> R:RW:0:BCH_DET_NUM */


/*
 * INTR9 interrupt Masked
 */
#define PDSS_INTR9_MASKED_ADDRESS                           (0x400a04a0)
#define PDSS_INTR9_MASKED                                   (*(volatile uint32_t *)(0x400a04a0))
#define PDSS_INTR9_MASKED_DEFAULT                           (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR9_MASKED_BCH_DET_CHANGED_MASKED_MASK       (0x00000003) /* <0:1> RW:R:0:BCH_DET_NUM_LOG2 */
#define PDSS_INTR9_MASKED_BCH_DET_CHANGED_MASKED_POS        (0)


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR9_MASKED_QCOM_RCVR_DM_CHANGED_MASKED       (1u << 20) /* <20:20> RW:R:0:BCH_DET_NUM */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR9_MASKED_QCOM_RCVR_DP_CHANGED_MASKED       (1u << 24) /* <24:24> RW:R:0:BCH_DET_NUM */


/*
 * CLK_FILTER2 (Divided version of CLK_HF) Filter configuration
 * Refer to COMP_CTRL, COMP_TR_CTRL comments for the mapping
 */
#define PDSS_INTR11_FILTER_CFG_ADDRESS                      (0x400a04c0)
#define PDSS_INTR11_FILTER_CFG                              (*(volatile uint32_t *)(0x400a04c0))
#define PDSS_INTR11_FILTER_CFG_DEFAULT                      (0x00000000)

/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR11_FILTER_CFG_FILT21_EN                    (1u << 0) /* <0:0> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR11_FILTER_CFG_FILT21_CFG_MASK              (0x00000006) /* <1:2> R:RW:0: */
#define PDSS_INTR11_FILTER_CFG_FILT21_CFG_POS               (1)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR11_FILTER_CFG_FILT21_RESET                 (1u << 3) /* <3:3> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR11_FILTER_CFG_FILT21_BYPASS                (1u << 4) /* <4:4> R:RW:0: */


/*
 * #of clock CLK_FILTER2 filtering. Should be programmed before FILTER is
 * enabled.
 */
#define PDSS_INTR11_FILTER_CFG_FILT21_SEL_MASK              (0x000003e0) /* <5:9> R:RW:0: */
#define PDSS_INTR11_FILTER_CFG_FILT21_SEL_POS               (5)


/*
 * Select from 12 comparator output
 */
#define PDSS_INTR11_FILTER_CFG_FILT21_INPUT_SEL_MASK        (0x00003c00) /* <10:13> R:RW:0: */
#define PDSS_INTR11_FILTER_CFG_FILT21_INPUT_SEL_POS         (10)


/*
 * This bit enables CPU to bypass the Filter when the part is in deep-sleep
 * state.  This over-rides the FILTER_EN settings during DEEP SLEEP state
 * ONLY.
 * The bit should be set by CPU only when its using the filter with high
 * frequency active clock and wants to wakeup from deep sleep on the transition
 * of the incoming signal.
 * This bit if set also disables the AUTO shutoff logic in DEEP SLEEP state.
 */
#define PDSS_INTR11_FILTER_CFG_FILT21_DPSLP_MODE            (1u << 14) /* <14:14> R:RW:0: */


/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR11_FILTER_CFG_FILT22_EN                    (1u << 15) /* <15:15> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR11_FILTER_CFG_FILT22_CFG_MASK              (0x00030000) /* <16:17> R:RW:0: */
#define PDSS_INTR11_FILTER_CFG_FILT22_CFG_POS               (16)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR11_FILTER_CFG_FILT22_RESET                 (1u << 18) /* <18:18> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR11_FILTER_CFG_FILT22_BYPASS                (1u << 19) /* <19:19> R:RW:0: */


/*
 * #of clock CLK_FILTER2 filtering. Should be programmed before FILTER is
 * enabled.
 */
#define PDSS_INTR11_FILTER_CFG_FILT22_SEL_MASK              (0x01f00000) /* <20:24> R:RW:0: */
#define PDSS_INTR11_FILTER_CFG_FILT22_SEL_POS               (20)


/*
 * Select from 12 comparator output
 */
#define PDSS_INTR11_FILTER_CFG_FILT22_INPUT_SEL_MASK        (0x1e000000) /* <25:28> R:RW:0: */
#define PDSS_INTR11_FILTER_CFG_FILT22_INPUT_SEL_POS         (25)


/*
 * This bit enables CPU to bypass the Filter when the part is in deep-sleep
 * state.  This over-rides the FILTER_EN settings during DEEP SLEEP state
 * ONLY.
 * The bit should be set by CPU only when its using the filter with high
 * frequency active clock and wants to wakeup from deep sleep on the transition
 * of the incoming signal.
 * This bit if set also disables the AUTO shutoff logic in DEEP SLEEP state.
 */
#define PDSS_INTR11_FILTER_CFG_FILT22_DPSLP_MODE            (1u << 29) /* <29:29> R:RW:0: */


/*
 * INTR11 Status 0
 */
#define PDSS_INTR11_STATUS_0_ADDRESS                        (0x400a04c4)
#define PDSS_INTR11_STATUS_0                                (*(volatile uint32_t *)(0x400a04c4))
#define PDSS_INTR11_STATUS_0_DEFAULT                        (0x00000000)

/*
 * The status of CLK_FILTER2 HS filter edge detectors 1-2
 */
#define PDSS_INTR11_STATUS_0_STATUS2_MASK                   (0x00000003) /* <0:1> RW:R:0: */
#define PDSS_INTR11_STATUS_0_STATUS2_POS                    (0)


/*
 * 1-2 HS Filtered output
 */
#define PDSS_INTR11_STATUS_0_FILT2_MASK                     (0x0000000c) /* <2:3> RW:R:0: */
#define PDSS_INTR11_STATUS_0_FILT2_POS                      (2)


/*
 * INTR11 interrupt Cause.  These are the wakeup interrupts get reflected
 * on interrupt_wakeup pin from HS filters
 */
#define PDSS_INTR11_ADDRESS                                 (0x400a04c8)
#define PDSS_INTR11                                         (*(volatile uint32_t *)(0x400a04c8))
#define PDSS_INTR11_DEFAULT                                 (0x00000000)

/*
 * Change in edge of HS filter edge detectors (wakeup interrupt from deepsleep)
 * Check the  INTR11_STATUS_0.STATUS2 value
 */
#define PDSS_INTR11_FILT2_EDGE_CHANGED_MASK                 (0x00000003) /* <0:1> RW1S:RW1C:0: */
#define PDSS_INTR11_FILT2_EDGE_CHANGED_POS                  (0)


/*
 * INTR11 Interrupt Set
 */
#define PDSS_INTR11_SET_ADDRESS                             (0x400a04cc)
#define PDSS_INTR11_SET                                     (*(volatile uint32_t *)(0x400a04cc))
#define PDSS_INTR11_SET_DEFAULT                             (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR11_SET_FILT2_EDGE_CHANGED_MASK             (0x00000003) /* <0:1> A:RW1S:0: */
#define PDSS_INTR11_SET_FILT2_EDGE_CHANGED_POS              (0)


/*
 * INTR11 interrupt Mask
 */
#define PDSS_INTR11_MASK_ADDRESS                            (0x400a04d0)
#define PDSS_INTR11_MASK                                    (*(volatile uint32_t *)(0x400a04d0))
#define PDSS_INTR11_MASK_DEFAULT                            (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR11_MASK_FILT2_EDGE_CHANGED_MASK_MASK       (0x00000003) /* <0:1> R:RW:0: */
#define PDSS_INTR11_MASK_FILT2_EDGE_CHANGED_MASK_POS        (0)


/*
 * INTR11 interrupt Masked
 */
#define PDSS_INTR11_MASKED_ADDRESS                          (0x400a04d4)
#define PDSS_INTR11_MASKED                                  (*(volatile uint32_t *)(0x400a04d4))
#define PDSS_INTR11_MASKED_DEFAULT                          (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR11_MASKED_FILT2_EDGE_CHANGED_MASKED_MASK    (0x00000003) /* <0:1> RW:R:0: */
#define PDSS_INTR11_MASKED_FILT2_EDGE_CHANGED_MASKED_POS    (0)


/*
 * INTR0 Cause. These are the active interrupts get reflected on interrupt_usbpd
 * pin.
 */
#define PDSS_INTR0_ADDRESS                                  (0x400a0500)
#define PDSS_INTR0                                          (*(volatile uint32_t *)(0x400a0500))
#define PDSS_INTR0_DEFAULT                                  (0x00000000)

/*
 * Receive a Good non-GoodCRC-message Data Packet Complete. Indicates that
 * the Receive Packet has been received in its entirety.  The received packet
 * had no CRC and no KCHAR error.
 * If this interrupt is not cleared, then RX_OVER_RUN will be set on the
 * next new data and the new data won't be written into RX SRAM.
 */
#define PDSS_INTR0_RCV_GOOD_PACKET_COMPLETE                 (1u << 0) /* <0:0> RW1S:RW1C:0: */


/*
 * Receive a Bad non-GoodCRC-message Data Packet Complete. Indicates that
 * the Receive Packet has been received in its entirety.  The received packet
 * had CRC or KCHAR error.
 */
#define PDSS_INTR0_RCV_BAD_PACKET_COMPLETE                  (1u << 1) /* <1:1> RW1S:RW1C:0: */


/*
 * Receive a SOP. FW should read SOP_TYPE_DETECTED for the SOP type
 */
#define PDSS_INTR0_RX_SOP                                   (1u << 2) /* <2:2> RW1S:RW1C:0: */


/*
 * Receive GoodCRC-message Complete. Indicates that the GoodCRC-message is
 * stored in RX_GOODCRC_MSG register.
 * The received GoodCRC-message had no CRC error and no KCHAR error
 */
#define PDSS_INTR0_RCV_GOODCRC_MSG_COMPLETE                 (1u << 3) /* <3:3> RW1S:RW1C:0: */


/*
 * Receive the expted GoodCRC-message based on the RX_EXPECT_GOODCRC_MSG
 * register. Indicates that the expected GoodCRC-message is stored in RX_GOODCRC_MSG
 * register.
 * The received expected GoodCRC-message had no CRC error and no KCHAR error
 * This interrupt gets evaluated on end of every packet (except hard reset)
 * and needs to be cleared.
 * For Correct usage of the interrupt:
 * This interrupt should be cleared on every packet if set.
 * RCV_GOOGCRC_MSG_COMPLETE should be cleared before new good crc response
 * comes else RX_GOODCRC_MSG will not get updated.
 */
#define PDSS_INTR0_RCV_EXPT_GOODCRC_MSG_COMPLETE            (1u << 4) /* <4:4> RW1S:RW1C:0: */


/*
 * Received Symbol wasn't a valid EOP K-Code.  It should be evaludated after
 * RCV_PACKET_COMPLETE.
 */
#define PDSS_INTR0_EOP_ERROR                                (1u << 5) /* <5:5> RW1S:RW1C:0: */


/*
 * New data was received when the RCV_PACKET_COMPLETE is not cleared by CPU.
 */
#define PDSS_INTR0_RX_OVER_RUN                              (1u << 6) /* <6:6> RW1S:RW1C:0: */


/*
 * Transmitter done sending data packet.
 */
#define PDSS_INTR0_TX_PACKET_DONE                           (1u << 7) /* <7:7> RW1S:RW1C:0: */


/*
 * Transmitter done sending Hard Reset
 */
#define PDSS_INTR0_TX_HARD_RST_DONE                         (1u << 8) /* <8:8> RW1S:RW1C:0: */


/*
 * Received a REST. FW should read RST_TYPE for the type of RST.
 * Firmware should process this interrupt according to the USB-PD spec.
 * Hardware does not process the Reset packets other than providing this
 * interrupt.
 * Hardware will stop processing any pending transmit packet until this interrupt
 * is cleared.
 */
#define PDSS_INTR0_RCV_RST                                  (1u << 9) /* <9:9> RW1S:RW1C:0: */


/*
 * A GoodCRC message was transmitted.
 */
#define PDSS_INTR0_TX_GOODCRC_MSG_DONE                      (1u << 10) /* <10:10> RW1S:RW1C:0: */


/*
 * Valid Data detected on the CC line
 */
#define PDSS_INTR0_CC_VALID_DATA_DETECTED                   (1u << 11) /* <11:11> RW1S:RW1C:0: */


/*
 * Valid Data got de-asserted on the CC line
 */
#define PDSS_INTR0_CC_NO_VALID_DATA_DETECTED                (1u << 12) /* <12:12> RW1S:RW1C:0: */


/*
 * CRCReceiveTimer has expired
 */
#define PDSS_INTR0_CRC_RX_TIMER_EXP                         (1u << 13) /* <13:13> RW1S:RW1C:0: */


/*
 * Transmit Collision Type1:
 * Collsion is detected Due to TX_GO(TX Data)/RX has occurred
 */
#define PDSS_INTR0_COLLISION_TYPE1                          (1u << 14) /* <14:14> RW1S:RW1C:0: */


/*
 * Transmit Collision Type2:
 * Collsion is detected  due to TX-RETRY/RX has occurred
 */
#define PDSS_INTR0_COLLISION_TYPE2                          (1u << 15) /* <15:15> RW1S:RW1C:0: */


/*
 * Transmit Collision Type3:
 * Collsion is detected due to TX-GoodCrc_MSG/RX has occurred
 */
#define PDSS_INTR0_COLLISION_TYPE3                          (1u << 16) /* <16:16> RW1S:RW1C:0: */


/*
 * Transmit Collision Type4:
 * Collsion is detected due to TX_SEND_RST/RX has occurred
 */
#define PDSS_INTR0_COLLISION_TYPE4                          (1u << 17) /* <17:17> RW1S:RW1C:0: */


/*
 * Hardware has passed reading the data from Half or End of the TX SRAM Memory
 * Location
 */
#define PDSS_INTR0_TX_SRAM_HALF_END                         (1u << 18) /* <18:18> RW1S:RW1C:0:KEEP_REG_BIT */


/*
 * Hardware has passed writing the data to Half or End of the RX SRAM Memory
 * Location
 */
#define PDSS_INTR0_RX_SRAM_HALF_END                         (1u << 19) /* <19:19> RW1S:RW1C:0:KEEP_REG_BIT */


/*
 * TX_RETRY_ENABLE is cleared
 */
#define PDSS_INTR0_TX_RETRY_ENABLE_CLRD                     (1u << 20) /* <20:20> RW1S:RW1C:0:KEEP_REG_BIT */


/*
 * Received Symbol wasn't a valid K-Code.
 */
#define PDSS_INTR0_KCHAR_ERROR                              (1u << 21) /* <21:21> RW1S:RW1C:0: */


/*
 * TX Data Output Enable of TX-CC is asserted
 */
#define PDSS_INTR0_TX_CC_DATA_OEN_ASSERT                    (1u << 22) /* <22:22> RW1S:RW1C:0: */


/*
 * TX Data Output Enable of TX-CC is de-asserted
 */
#define PDSS_INTR0_TX_CC_DATA_OEN_DEASSERT                  (1u << 23) /* <23:23> RW1S:RW1C:0: */


/*
 * The TX CC regulator is enabled
 */
#define PDSS_INTR0_TX_REGULATOR_ENABLED                     (1u << 24) /* <24:24> RW1S:RW1C:0: */


/*
 * The TX State Machine entered Idle state
 */
#define PDSS_INTR0_TX_STATE_IDLE                            (1u << 25) /* <25:25> RW1S:RW1C:0: */


/*
 * The RX State Machine entered Idle state
 */
#define PDSS_INTR0_RX_STATE_IDLE                            (1u << 26) /* <26:26> RW1S:RW1C:0: */


/*
 * Marks Completion of Third ADC SAR1-4 conversion at the end of 8 cycles
 * of clk_sar when SAR_EN is "1"
 */
#define PDSS_INTR0_SAR_DONE                                 (1u << 27) /* <27:27> RW1S:RW1C:0:ADC_NUM */


/*
 * INTR0 Set
 */
#define PDSS_INTR0_SET_ADDRESS                              (0x400a0504)
#define PDSS_INTR0_SET                                      (*(volatile uint32_t *)(0x400a0504))
#define PDSS_INTR0_SET_DEFAULT                              (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_RCV_GOOD_PACKET_COMPLETE             (1u << 0) /* <0:0> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_RCV_BAD_PACKET_COMPLETE              (1u << 1) /* <1:1> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_RX_SOP                               (1u << 2) /* <2:2> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_RCV_GOODCRC_MSG_COMPLETE             (1u << 3) /* <3:3> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_RCV_EXPT_GOODCRC_MSG_COMPLETE        (1u << 4) /* <4:4> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_EOP_ERROR                            (1u << 5) /* <5:5> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_RX_OVER_RUN                          (1u << 6) /* <6:6> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_TX_PACKET_DONE                       (1u << 7) /* <7:7> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_TX_HARD_RST_DONE                     (1u << 8) /* <8:8> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_RCV_RST                              (1u << 9) /* <9:9> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_TX_GOODCRC_MSG_DONE                  (1u << 10) /* <10:10> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_CC_VALID_DATA_DETECTED               (1u << 11) /* <11:11> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_CC_NO_VALID_DATA_DETECTED            (1u << 12) /* <12:12> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_CRC_RX_TIMER_EXP                     (1u << 13) /* <13:13> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_COLLISION_TYPE1                      (1u << 14) /* <14:14> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_COLLISION_TYPE2                      (1u << 15) /* <15:15> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_COLLISION_TYPE3                      (1u << 16) /* <16:16> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_COLLISION_TYPE4                      (1u << 17) /* <17:17> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_TX_SRAM_HALF_END                     (1u << 18) /* <18:18> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_RX_SRAM_HALF_END                     (1u << 19) /* <19:19> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_TX_RETRY_ENABLE_CLRD                 (1u << 20) /* <20:20> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_KCHAR_ERROR                          (1u << 21) /* <21:21> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_TX_CC_DATA_OEN_ASSERT                (1u << 22) /* <22:22> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_TX_CC_DATA_OEN_DEASSERT              (1u << 23) /* <23:23> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_TX_REGULATOR_ENABLED                 (1u << 24) /* <24:24> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_TX_STATE_IDLE                        (1u << 25) /* <25:25> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_RX_STATE_IDLE                        (1u << 26) /* <26:26> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_SAR_DONE                             (1u << 27) /* <27:27> A:RW1S:0:ADC_NUM */


/*
 * INTR0 Mask
 */
#define PDSS_INTR0_MASK_ADDRESS                             (0x400a0508)
#define PDSS_INTR0_MASK                                     (*(volatile uint32_t *)(0x400a0508))
#define PDSS_INTR0_MASK_DEFAULT                             (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_RCV_GOOD_PACKET_COMPLETE_MASK       (1u << 0) /* <0:0> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_RCV_BAD_PACKET_COMPLETE_MASK        (1u << 1) /* <1:1> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_RX_SOP_MASK                         (1u << 2) /* <2:2> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_RCV_GOODCRC_MSG_COMPLETE_MASK       (1u << 3) /* <3:3> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_RCV_EXPT_GOODCRC_MSG_COMPLETE_MASK    (1u << 4) /* <4:4> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_EOP_ERROR_MASK                      (1u << 5) /* <5:5> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_RX_OVER_RUN_MASK                    (1u << 6) /* <6:6> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_TX_PACKET_DONE_MASK                 (1u << 7) /* <7:7> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_TX_HARD_RST_DONE_MASK               (1u << 8) /* <8:8> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_RCV_RST_MASK                        (1u << 9) /* <9:9> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_TX_GOODCRC_MSG_DONE_MASK            (1u << 10) /* <10:10> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_CC_VALID_DATA_DETECTED_MASK         (1u << 11) /* <11:11> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_CC_NO_VALID_DATA_DETECTED_MASK      (1u << 12) /* <12:12> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_CRC_RX_TIMER_EXP_MASK               (1u << 13) /* <13:13> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_COLLISION_TYPE1_MASK                (1u << 14) /* <14:14> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_COLLISION_TYPE2_MASK                (1u << 15) /* <15:15> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_COLLISION_TYPE3_MASK                (1u << 16) /* <16:16> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_COLLISION_TYPE4_MASK                (1u << 17) /* <17:17> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_TX_SRAM_HALF_END_MASK               (1u << 18) /* <18:18> R:RW:0:KEEP_REG_BIT */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_RX_SRAM_HALF_END_MASK               (1u << 19) /* <19:19> R:RW:0:KEEP_REG_BIT */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_TX_RETRY_ENABLE_CLRD_MASK           (1u << 20) /* <20:20> R:RW:0:KEEP_REG_BIT */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_KCHAR_ERROR_MASK                    (1u << 21) /* <21:21> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_TX_CC_DATA_OEN_ASSERT_MASK          (1u << 22) /* <22:22> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_TX_CC_DATA_OEN_DEASSERT_MASK        (1u << 23) /* <23:23> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_TX_REGULATOR_ENABLED_MASK           (1u << 24) /* <24:24> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_TX_STATE_IDLE_MASK                  (1u << 25) /* <25:25> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_RX_STATE_IDLE_MASK                  (1u << 26) /* <26:26> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_SAR_DONE_MASK                       (1u << 27) /* <27:27> R:RW:0:ADC_NUM */


/*
 * INTR0 Masked
 */
#define PDSS_INTR0_MASKED_ADDRESS                           (0x400a050c)
#define PDSS_INTR0_MASKED                                   (*(volatile uint32_t *)(0x400a050c))
#define PDSS_INTR0_MASKED_DEFAULT                           (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_RCV_GOOD_PACKET_COMPLETE_MASKED    (1u << 0) /* <0:0> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_RCV_BAD_PACKET_COMPLETE_MASKED    (1u << 1) /* <1:1> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_RX_SOP_MASKED                     (1u << 2) /* <2:2> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_RCV_GOODCRC_MSG_COMPLETE_MASKED    (1u << 3) /* <3:3> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_RCV_EXPT_GOODCRC_MSG_COMPLETE_MASKED    (1u << 4) /* <4:4> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_EOP_ERROR_MASKED                  (1u << 5) /* <5:5> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_RX_OVER_RUN_MASKED                (1u << 6) /* <6:6> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_TX_PACKET_DONE_MASKED             (1u << 7) /* <7:7> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_TX_HARD_RST_DONE_MASKED           (1u << 8) /* <8:8> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_RCV_RST_MASKED                    (1u << 9) /* <9:9> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_TX_GOODCRC_MSG_DONE_MASKED        (1u << 10) /* <10:10> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_CC_VALID_DATA_DETECTED_MASKED     (1u << 11) /* <11:11> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_CC_NO_VALID_DATA_DETECTED_MASKED    (1u << 12) /* <12:12> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_CRC_RX_TIMER_EXP_MASKED           (1u << 13) /* <13:13> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_COLLISION_TYPE1_MASKED            (1u << 14) /* <14:14> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_COLLISION_TYPE2_MASKED            (1u << 15) /* <15:15> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_COLLISION_TYPE3_MASKED            (1u << 16) /* <16:16> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_COLLISION_TYPE4_MASKED            (1u << 17) /* <17:17> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_TX_SRAM_HALF_END_MASKED           (1u << 18) /* <18:18> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_RX_SRAM_HALF_END_MASKED           (1u << 19) /* <19:19> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_TX_RETRY_ENABLE_CLRD_MASKED       (1u << 20) /* <20:20> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_KCHAR_ERROR_MASKED                (1u << 21) /* <21:21> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_TX_CC_DATA_OEN_ASSERT_MASKED      (1u << 22) /* <22:22> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_TX_CC_DATA_OEN_DEASSERT_MASKED    (1u << 23) /* <23:23> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_TX_REGULATOR_ENABLED_MASKED       (1u << 24) /* <24:24> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_TX_STATE_IDLE_MASKED              (1u << 25) /* <25:25> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_RX_STATE_IDLE_MASKED              (1u << 26) /* <26:26> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_SAR_DONE_MASKED                   (1u << 27) /* <27:27> RW:R:0:ADC_NUM */


/*
 * INTR2 Cause.  These are the active interrupts get reflected on interrupt_usbpd
 * pin.
 */
#define PDSS_INTR2_ADDRESS                                  (0x400a0510)
#define PDSS_INTR2                                          (*(volatile uint32_t *)(0x400a0510))
#define PDSS_INTR2_DEFAULT                                  (0x00000000)

/*
 * The interrupt is raised at the end of UI calculation during preamble.
 */
#define PDSS_INTR2_UI_CAL_DONE                              (1u << 0) /* <0:0> RW1S:RW1C:0: */


/*
 * The extended receive message detected
 */
#define PDSS_INTR2_EXTENDED_MSG_DET                         (1u << 7) /* <7:7> RW1S:RW1C:0: */


/*
 * The chunked-extended receive message detected
 */
#define PDSS_INTR2_CHUNK_DET                                (1u << 8) /* <8:8> RW1S:RW1C:0: */


/*
 * Hardware has passed writing the data to Half or End of the RX SRAM Memory
 * Location but CPU has not read the Data (RX_SRAM_HALF_EN not cleared bt
 * FW). This interrupt should be cleared only after the end of the RX packet.
 */
#define PDSS_INTR2_RX_SRAM_OVER_FLOW                        (1u << 9) /* <9:9> RW1S:RW1C:0:KEEP_REG_BIT */


/*
 * Hardware has passed reading the data to Half or End of the TX SRAM Memory
 * Location but CPU has not wrote the Data (TX_SRAM_HALF_EN not cleared bt
 * FW). This interrupt should be cleared only after the end of the TX packet.
 */
#define PDSS_INTR2_TX_SRAM_UNDER_FLOW                       (1u << 10) /* <10:10> RW1S:RW1C:0:KEEP_REG_BIT */


/*
 * VREG Switching is complete
 */
#define PDSS_INTR2_VREG20V_SWITCH_DONE                      (1u << 15) /* <15:15> RW1S:RW1C:0:KEEP_REG_BIT */


/*
 * The discharge protection has triggered
 */
#define PDSS_INTR2_DISCHG_FAILSAFE_DETECTED_MASK            (0x00c00000) /* <22:23> RW1S:RW1C:0:DISCHG_PROT_NUM */
#define PDSS_INTR2_DISCHG_FAILSAFE_DETECTED_POS             (22)


/*
 * INTR2 Set
 */
#define PDSS_INTR2_SET_ADDRESS                              (0x400a0514)
#define PDSS_INTR2_SET                                      (*(volatile uint32_t *)(0x400a0514))
#define PDSS_INTR2_SET_DEFAULT                              (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_UI_CAL_DONE                          (1u << 0) /* <0:0> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_EXTENDED_MSG_DET                     (1u << 7) /* <7:7> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_CHUNK_DET                            (1u << 8) /* <8:8> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_RX_SRAM_OVER_FLOW                    (1u << 9) /* <9:9> A:RW1S:0:KEEP_REG_BIT */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_TX_SRAM_UNDER_FLOW                   (1u << 10) /* <10:10> A:RW1S:0:KEEP_REG_BIT */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_VREG20V_SWITCH_DONE                  (1u << 15) /* <15:15> A:RW1S:0:KEEP_REG_BIT */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_DISCHG_FAILSAFE_DETECTED_MASK        (0x00c00000) /* <22:23> A:RW1S:0:DISCHG_PROT_NUM */
#define PDSS_INTR2_SET_DISCHG_FAILSAFE_DETECTED_POS         (22)


/*
 * INTR2 Mask
 */
#define PDSS_INTR2_MASK_ADDRESS                             (0x400a0518)
#define PDSS_INTR2_MASK                                     (*(volatile uint32_t *)(0x400a0518))
#define PDSS_INTR2_MASK_DEFAULT                             (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_UI_CAL_DONE_MASK                    (1u << 0) /* <0:0> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_EXTENDED_MSG_DET_MASK               (1u << 7) /* <7:7> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_CHUNK_DET_MASK                      (1u << 8) /* <8:8> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_RX_SRAM_OVER_FLOW_MASK              (1u << 9) /* <9:9> R:RW:0:KEEP_REG_BIT */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_TX_SRAM_UNDER_FLOW_MASK             (1u << 10) /* <10:10> R:RW:0:KEEP_REG_BIT */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_VREG20V_SWITCH_DONE_MASK            (1u << 15) /* <15:15> R:RW:0:KEEP_REG_BIT */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_DISCHG_FAILSAFE_DETECTED_MASK_MASK    (0x00c00000) /* <22:23> R:RW:0:DISCHG_PROT_NUM */
#define PDSS_INTR2_MASK_DISCHG_FAILSAFE_DETECTED_MASK_POS    (22)


/*
 * INTR2 Masked
 */
#define PDSS_INTR2_MASKED_ADDRESS                           (0x400a051c)
#define PDSS_INTR2_MASKED                                   (*(volatile uint32_t *)(0x400a051c))
#define PDSS_INTR2_MASKED_DEFAULT                           (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_UI_CAL_DONE_MASKED                (1u << 0) /* <0:0> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_EXTENDED_MSG_DET_MASKED           (1u << 7) /* <7:7> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_CHUNK_DET_MASKED                  (1u << 8) /* <8:8> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_RX_SRAM_OVER_FLOW_MASKED          (1u << 9) /* <9:9> RW:R:0:KEEP_REG_BIT */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_TX_SRAM_UNDER_FLOW_MASKED         (1u << 10) /* <10:10> RW:R:0:KEEP_REG_BIT */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_VREG20V_SWITCH_DONE_MASKED        (1u << 15) /* <15:15> RW:R:0:KEEP_REG_BIT */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_DISCHG_FAILSAFE_DETECTED_MASKED_MASK    (0x00c00000) /* <22:23> RW:R:0:DISCHG_PROT_NUM */
#define PDSS_INTR2_MASKED_DISCHG_FAILSAFE_DETECTED_MASKED_POS    (22)


/*
 * INTR4 Cause for AFC. These are the active interrupts get reflected on
 * interrupt_usbpd pin.
 */
#define PDSS_INTR4_ADDRESS                                  (0x400a0520)
#define PDSS_INTR4                                          (*(volatile uint32_t *)(0x400a0520))
#define PDSS_INTR4_DEFAULT                                  (0x00000000)

/*
 * Interrupt whenever Ping (16UIs) is received. This is common for both master
 * and slave mode
 */
#define PDSS_INTR4_AFC_PING_RECVD                           (1u << 0) /* <0:0> RW1S:RW1C:0:BCH_DET_NUM */


/*
 * This interrupt fires when state machine goes to idle:
 *
 * In master mode: this will happen when:
 *    • Just Ping mode: Ping is sent and either Ping is received or timeout
 * happens
 *    • Ping+ Data Mode: Ping is sent, then Ping is received, then data is
 * sent, slave data is received along with
 *       final ping, master final ping transmitted and slave final ping received
 * or any timeout happens.
 *
 * This interrupt can be used as master interrupt to sample all received
 * data from slave
 * In slave mode: this will happened when:
 *    • Ping transmitted in response to master Ping.
 *
 * All data bytes transmitted in response to master V_I_Byte request, then
 * master ping is received and final slave ping is sent.
 */
#define PDSS_INTR4_AFC_SM_IDLE                              (1u << 12) /* <12:12> RW1S:RW1C:0:BCH_DET_NUM */
#define PDSS_INTR4_AFC_SM_IDLE_POS                          (12u)

/*
 * This bit is set whenever state machine is expecting response from link
 * partner but it never gets response. This could be due to Ping response
 * timeout, data response timeout etc. The status register can be read which
 * states which was the last state before timeout was declared.
 */
#define PDSS_INTR4_AFC_TIMEOUT                              (1u << 16) /* <16:16> RW1S:RW1C:0:BCH_DET_NUM */
#define PDSS_INTR4_AFC_TIMEOUT_POS                          (16u)

/*
 * Indicated 1 was seen on DMinus for RESET_UI_COUNT number of Uis
 */
#define PDSS_INTR4_AFC_RX_RESET                             (1u << 20) /* <20:20> RW1S:RW1C:0:BCH_DET_NUM */
#define PDSS_INTR4_AFC_RX_RESET_POS                         (20u)

/*
 * On this interrupt FW should read/update the AFC_PING_RECVD register
 */
#define PDSS_INTR4_UPDATE_PING_PONG                         (1u << 24) /* <24:24> RW1S:RW1C:0:BCH_DET_NUM */
#define PDSS_INTR4_UPDATE_PING_PONG_POS                     (24u)

/*
 * This interrupt is set when a AFC master request is seen while Slave is
 * trying to send data.
 */
#define PDSS_INTR4_AFC_ERROR                                (1u << 28) /* <28:28> RW1S:RW1C:0:BCH_DET_NUM */
#define PDSS_INTR4_AFC_ERROR_POS                            (28u)

/*
 * INTR4 Set
 */
#define PDSS_INTR4_SET_ADDRESS                              (0x400a0524)
#define PDSS_INTR4_SET                                      (*(volatile uint32_t *)(0x400a0524))
#define PDSS_INTR4_SET_DEFAULT                              (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR4_SET_AFC_PING_RECVD                       (1u << 0) /* <0:0> A:RW1S:0:BCH_DET_NUM */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR4_SET_AFC_SM_IDLE                          (1u << 12) /* <12:12> A:RW1S:0:BCH_DET_NUM */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR4_SET_AFC_TIMEOUT                          (1u << 16) /* <16:16> A:RW1S:0:BCH_DET_NUM */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR4_SET_AFC_RX_RESET                         (1u << 20) /* <20:20> A:RW1S:0:BCH_DET_NUM */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR4_SET_UPDATE_PING_PONG                     (1u << 24) /* <24:24> A:RW1S:0:BCH_DET_NUM */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR4_SET_AFC_ERROR                            (1u << 28) /* <28:28> A:RW1S:0:BCH_DET_NUM */


/*
 * INTR4 Mask
 */
#define PDSS_INTR4_MASK_ADDRESS                             (0x400a0528)
#define PDSS_INTR4_MASK                                     (*(volatile uint32_t *)(0x400a0528))
#define PDSS_INTR4_MASK_DEFAULT                             (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR4_MASK_AFC_PING_RECVD_MASK                 (1u << 0) /* <0:0> R:RW:0:BCH_DET_NUM */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR4_MASK_AFC_SM_IDLE_MASK                    (1u << 12) /* <12:12> R:RW:0:BCH_DET_NUM */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR4_MASK_AFC_TIMEOUT_MASK                    (1u << 16) /* <16:16> R:RW:0:BCH_DET_NUM */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR4_MASK_AFC_RX_RESET_MASK                   (1u << 20) /* <20:20> R:RW:0:BCH_DET_NUM */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR4_MASK_UPDATE_PING_PONG_MASK               (1u << 24) /* <24:24> R:RW:0:BCH_DET_NUM */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR4_MASK_AFC_ERROR_MASK                      (1u << 28) /* <28:28> R:RW:0:BCH_DET_NUM */


/*
 * INTR4 Masked
 */
#define PDSS_INTR4_MASKED_ADDRESS                           (0x400a052c)
#define PDSS_INTR4_MASKED                                   (*(volatile uint32_t *)(0x400a052c))
#define PDSS_INTR4_MASKED_DEFAULT                           (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR4_MASKED_AFC_PING_RECVD_MASKED             (1u << 0) /* <0:0> RW:R:0:BCH_DET_NUM */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR4_MASKED_AFC_SM_IDLE_MASKED                (1u << 12) /* <12:12> RW:R:0:BCH_DET_NUM */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR4_MASKED_AFC_TIMEOUT_MASKED                (1u << 16) /* <16:16> RW:R:0:BCH_DET_NUM */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR4_MASKED_AFC_RX_RESET_MASKED               (1u << 20) /* <20:20> RW:R:0:BCH_DET_NUM */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR4_MASKED_UPDATE_PING_PONG_MASKED           (1u << 24) /* <24:24> RW:R:0:BCH_DET_NUM */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR4_MASKED_AFC_ERROR_MASKED                  (1u << 28) /* <28:28> RW:R:0:BCH_DET_NUM */


/*
 * INTR6 Cause. These are the active interrupts get reflected on interrupt_usbpd
 * pin.
 */
#define PDSS_INTR6_ADDRESS                                  (0x400a0530)
#define PDSS_INTR6                                          (*(volatile uint32_t *)(0x400a0530))
#define PDSS_INTR6_DEFAULT                                  (0x00000000)

/*
 * Hardware will maintain internal Dplus pulse count. Anytime the pulse count
 * is greater than 0, this interrupt will be set.
 */
#define PDSS_INTR6_QC_3_D_P_PULSE_RCVD                      (1u << 0) /* <0:0> RW1S:RW1C:0:BCH_DET_NUM */


/*
 * Hardware will maintain internal Dminus pulse count. Anytime the pulse
 * count is greater than 0, this interrupt will be set.
 */
#define PDSS_INTR6_QC_3_D_M_PULSE_RCVD                      (1u << 4) /* <4:4> RW1S:RW1C:0:BCH_DET_NUM */


/*
 * This is used when QC3.0 is implemented for portable device which sends
 * voltage increment/decrement requests.
 * It indicates that hardware has sent the required increment or decrement
 * requests (as programmed) on Dplus or Dminus
 */
#define PDSS_INTR6_QC_3_DEVICE_REQ_SENT                     (1u << 8) /* <8:8> RW1S:RW1C:0:BCH_DET_NUM */


/*
 * INTR6 Set
 */
#define PDSS_INTR6_SET_ADDRESS                              (0x400a0534)
#define PDSS_INTR6_SET                                      (*(volatile uint32_t *)(0x400a0534))
#define PDSS_INTR6_SET_DEFAULT                              (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR6_SET_QC_3_D_P_PULSE_RCVD                  (1u << 0) /* <0:0> A:RW1S:0:BCH_DET_NUM */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR6_SET_QC_3_D_M_PULSE_RCVD                  (1u << 4) /* <4:4> A:RW1S:0:BCH_DET_NUM */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR6_SET_QC_3_DEVICE_REQ_SENT                 (1u << 8) /* <8:8> A:RW1S:0:BCH_DET_NUM */


/*
 * INTR6 Mask
 */
#define PDSS_INTR6_MASK_ADDRESS                             (0x400a0538)
#define PDSS_INTR6_MASK                                     (*(volatile uint32_t *)(0x400a0538))
#define PDSS_INTR6_MASK_DEFAULT                             (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR6_MASK_QC_3_D_P_PULSE_RCVD_MASK            (1u << 0) /* <0:0> R:RW:0:BCH_DET_NUM */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR6_MASK_QC_3_D_M_PULSE_RCVD_MASK            (1u << 4) /* <4:4> R:RW:0:BCH_DET_NUM */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR6_MASK_QC_3_DEVICE_REQ_SENT_MASK           (1u << 8) /* <8:8> R:RW:0:BCH_DET_NUM */


/*
 * INTR6 Masked
 */
#define PDSS_INTR6_MASKED_ADDRESS                           (0x400a053c)
#define PDSS_INTR6_MASKED                                   (*(volatile uint32_t *)(0x400a053c))
#define PDSS_INTR6_MASKED_DEFAULT                           (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR6_MASKED_QC_3_D_P_PULSE_RCVD_MASKED        (1u << 0) /* <0:0> RW:R:0:BCH_DET_NUM */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR6_MASKED_QC_3_D_M_PULSE_RCVD_MASKED        (1u << 4) /* <4:4> RW:R:0:BCH_DET_NUM */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR6_MASKED_QC_3_DEVICE_REQ_SENT_MASKED       (1u << 8) /* <8:8> RW:R:0:BCH_DET_NUM */


/*
 * INTR8 Cause. These are the active interrupts get reflected on interrupt_usbpd
 * pin.
 */
#define PDSS_INTR8_ADDRESS                                  (0x400a0540)
#define PDSS_INTR8                                          (*(volatile uint32_t *)(0x400a0540))
#define PDSS_INTR8_DEFAULT                                  (0x00000000)

/*
 * Indicates the timeout condition for Minimum Variable Frequency for secondary
 * controller.
 */
#define PDSS_INTR8_PASC_VAR_TMAX_TIMEOUT                    (1u << 0) /* <0:0> RW1S:RW1C:0:PASC_EN */


/*
 * Indicates the secondary controller has encountered MAX frequency in QR-mode
 */
#define PDSS_INTR8_PASC_VAR_TMIN_TIMEOUT                    (1u << 1) /* <1:1> RW1S:RW1C:0:PASC_EN */


/*
 * Inidcates the timeout condition for Maximum Audio Frequency for secondary
 * controller.
 */
#define PDSS_INTR8_PASC_AUD_TMIN_TIMEOUT                    (1u << 2) /* <2:2> RW1S:RW1C:0:PASC_EN */


/*
 * Inidcates the timeout condition for Minimum Audio Frequency for secondary
 * controller.
 */
#define PDSS_INTR8_PASC_AUD_TMAX_TIMEOUT                    (1u << 3) /* <3:3> RW1S:RW1C:0:PASC_EN */


/*
 * Inidicates the maximum pulse width for Secodnary FET ("gdrv_in") reached
 * for secondary controller.
 */
#define PDSS_INTR8_PASC_GDRV_IN_MAX_WIDTH_TIMEOUT           (1u << 4) /* <4:4> RW1S:RW1C:0:PASC_EN */


/*
 * Inidicates the maximum pulse width for Primary FET ("ptdrv_in") reached
 * for secondary controller.
 */
#define PDSS_INTR8_PASC_PTDRV_IN_MAX_WIDTH_TIMEOUT          (1u << 5) /* <5:5> RW1S:RW1C:0:PASC_EN */


/*
 * Indicates the secondary controller has hit Fix frequency time
 */
#define PDSS_INTR8_PASC_FIX_FREQ_TIMEOUT                    (1u << 6) /* <6:6> RW1S:RW1C:0:PASC_EN */


/*
 * NSN timeout occurred
 */
#define PDSS_INTR8_PASC_NSN_IDLE_TIMEOUT                    (1u << 7) /* <7:7> RW1S:RW1C:0:PASC_EN */


/*
 * Calibration of PTDRV to ZCDF-in is complete
 */
#define PDSS_INTR8_PASC_LOOP_CAL_DONE                       (1u << 8) /* <8:8> RW1S:RW1C:0:PASC_EN */


/*
 * Secondary controller has entered burst mode
 */
#define PDSS_INTR8_PASC_BURST_ENTRY                         (1u << 9) /* <9:9> RW1S:RW1C:0:PASC_EN */


/*
 * Loop switched from CCM to DCM
 */
#define PDSS_INTR8_PASC_CCM_2_DCM_CHG                       (1u << 10) /* <10:10> RW1S:RW1C:0:PASC_EN */


/*
 * Loop switched from DCM to CCM
 */
#define PDSS_INTR8_PASC_DCM_2_CCM_CHG                       (1u << 11) /* <11:11> RW1S:RW1C:0:PASC_EN */


/*
 * Indicates gdrv width is smaller than turn-off width
 */
#define PDSS_INTR8_GDRV_LESS_THAN_TURN_OFF                  (1u << 12) /* <12:12> RW1S:RW1C:0:PASC_EN */


/*
 * Indicates gdrv width has exceeded turn-off width
 */
#define PDSS_INTR8_GDRV_GREATER_THAN_TURN_OFF               (1u << 13) /* <13:13> RW1S:RW1C:0:PASC_EN */


/*
 * NSN OUT asserted when PTDRV is high
 */
#define PDSS_INTR8_GDRV_CONTENTION                          (1u << 14) /* <14:14> RW1S:RW1C:0:PASC_EN */


/*
 * Secondary gate driver on time is less than min width
 */
#define PDSS_INTR8_GDRV_INPUT_WIDTH_LESS_THAN_MIN           (1u << 15) /* <15:15> RW1S:RW1C:0:PASC_EN */


/*
 * Calibration has failed due to NSN_OUT being seen during PTDRV high or
 * NSN has timed out
 */
#define PDSS_INTR8_CAL_FAIL                                 (1u << 16) /* <16:16> RW1S:RW1C:0:PASC_EN */


/*
 * The PASC entered Idle state
 */
#define PDSS_INTR8_PASC_IDLE                                (1u << 17) /* <17:17> RW1S:RW1C:0:PASC_EN */


/*
 * FF OV timeout occurred
 */
#define PDSS_INTR8_PASC_FF_OV_IDLE_TIMEOUT                  (1u << 18) /* <18:18> RW1S:RW1C:0:PASC_EN */


/*
 * Interrupt for VBUS transition operation done.
 * If exit is requested by FW and if any of the VBTR_STATUS.SRC_DONE OR VBTR_STATUS.SNK_DONE
 * is "1", then this interrupt will be set.
 */
#define PDSS_INTR8_VBTR_OPR_DONE                            (1u << 30) /* <30:30> RW1S:RW1C:0:VBTR_EN */


/*
 * Interrupt for VBUS transition exit done
 * This interrupt bit will be set within one VBTR clock cycle of the VBTR_CTRL.EXIT
 * bit being set.
 */
#define PDSS_INTR8_VBTR_EXIT_DONE                           (1u << 31) /* <31:31> RW1S:RW1C:0:VBTR_EN */


/*
 * INTR8 Set
 */
#define PDSS_INTR8_SET_ADDRESS                              (0x400a0544)
#define PDSS_INTR8_SET                                      (*(volatile uint32_t *)(0x400a0544))
#define PDSS_INTR8_SET_DEFAULT                              (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_PASC_VAR_TMAX_TIMEOUT                (1u << 0) /* <0:0> A:RW1S:0:PASC_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_PASC_VAR_TMIN_TIMEOUT                (1u << 1) /* <1:1> A:RW1S:0:PASC_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_PASC_AUD_TMIN_TIMEOUT                (1u << 2) /* <2:2> A:RW1S:0:PASC_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_PASC_AUD_TMAX_TIMEOUT                (1u << 3) /* <3:3> A:RW1S:0:PASC_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_PASC_GDRV_IN_MAX_WIDTH_TIMEOUT       (1u << 4) /* <4:4> A:RW1S:0:PASC_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_PASC_PTDRV_IN_MAX_WIDTH_TIMEOUT      (1u << 5) /* <5:5> A:RW1S:0:PASC_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_PASC_FIX_FREQ_TIMEOUT                (1u << 6) /* <6:6> A:RW1S:0:PASC_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_PASC_NSN_IDLE_TIMEOUT                (1u << 7) /* <7:7> A:RW1S:0:PASC_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_PASC_LOOP_CAL_DONE                   (1u << 8) /* <8:8> A:RW1S:0:PASC_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_PASC_BURST_ENTRY                     (1u << 9) /* <9:9> A:RW1S:0:PASC_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_PASC_CCM_2_DCM_CHG                   (1u << 10) /* <10:10> A:RW1S:0:PASC_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_PASC_DCM_2_CCM_CHG                   (1u << 11) /* <11:11> A:RW1S:0:PASC_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_GDRV_LESS_THAN_TURN_OFF              (1u << 12) /* <12:12> A:RW1S:0:PASC_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_GDRV_GREATER_THAN_TURN_OFF           (1u << 13) /* <13:13> A:RW1S:0:PASC_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_GDRV_CONTENTION                      (1u << 14) /* <14:14> A:RW1S:0:PASC_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_GDRV_INPUT_WIDTH_LESS_THAN_MIN       (1u << 15) /* <15:15> A:RW1S:0:PASC_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_CAL_FAIL                             (1u << 16) /* <16:16> A:RW1S:0:PASC_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_PASC_IDLE                            (1u << 17) /* <17:17> A:RW1S:0:PASC_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_PASC_FF_OV_IDLE_TIMEOUT              (1u << 18) /* <18:18> A:RW1S:0:PASC_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_VBTR_OPR_DONE                        (1u << 30) /* <30:30> A:RW1S:0:VBTR_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_SET_VBTR_EXIT_DONE                       (1u << 31) /* <31:31> A:RW1S:0:VBTR_EN */


/*
 * INTR8 Mask
 */
#define PDSS_INTR8_MASK_ADDRESS                             (0x400a0548)
#define PDSS_INTR8_MASK                                     (*(volatile uint32_t *)(0x400a0548))
#define PDSS_INTR8_MASK_DEFAULT                             (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_PASC_VAR_TMAX_TIMEOUT_MASK          (1u << 0) /* <0:0> R:RW:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_PASC_VAR_TMIN_TIMEOUT_MASK          (1u << 1) /* <1:1> R:RW:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_PASC_AUD_TMIN_TIMEOUT_MASK          (1u << 2) /* <2:2> R:RW:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_PASC_AUD_TMAX_TIMEOUT_MASK          (1u << 3) /* <3:3> R:RW:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_PASC_GDRV_IN_MAX_WIDTH_TIMEOUT_MASK    (1u << 4) /* <4:4> R:RW:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_PASC_PTDRV_IN_MAX_WIDTH_TIMEOUT_MASK    (1u << 5) /* <5:5> R:RW:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_PASC_FIX_FREQ_TIMEOUT_MASK          (1u << 6) /* <6:6> R:RW:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_PASC_NSN_IDLE_TIMEOUT_MASK          (1u << 7) /* <7:7> R:RW:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_PASC_LOOP_CAL_DONE_MASK             (1u << 8) /* <8:8> R:RW:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_PASC_BURST_ENTRY_MASK               (1u << 9) /* <9:9> R:RW:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_PASC_CCM_2_DCM_CHG_MASK             (1u << 10) /* <10:10> R:RW:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_PASC_DCM_2_CCM_CHG_MASK             (1u << 11) /* <11:11> R:RW:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_GDRV_LESS_THAN_TURN_OFF_MASK        (1u << 12) /* <12:12> R:RW:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_GDRV_GREATER_THAN_TURN_OFF_MASK     (1u << 13) /* <13:13> R:RW:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_GDRV_CONTENTION_MASK                (1u << 14) /* <14:14> R:RW:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_GDRV_INPUT_WIDTH_LESS_THAN_MIN_MASK    (1u << 15) /* <15:15> R:RW:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_CAL_FAIL_MASK                       (1u << 16) /* <16:16> R:RW:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_PASC_IDLE_MASK                      (1u << 17) /* <17:17> R:RW:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_PASC_FF_OV_IDLE_TIMEOUT_MASK        (1u << 18) /* <18:18> R:RW:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_VBTR_OPR_DONE_MASK                  (1u << 30) /* <30:30> R:RW:0:VBTR_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASK_VBTR_EXIT_DONE_MASK                 (1u << 31) /* <31:31> R:RW:0:VBTR_EN */


/*
 * INTR8 Masked
 */
#define PDSS_INTR8_MASKED_ADDRESS                           (0x400a054c)
#define PDSS_INTR8_MASKED                                   (*(volatile uint32_t *)(0x400a054c))
#define PDSS_INTR8_MASKED_DEFAULT                           (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR8_MASKED_PASC_VAR_TMAX_TIMEOUT_MASKED      (1u << 0) /* <0:0> RW:R:0:PASC_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR8_MASKED_PASC_VAR_TMIN_TIMEOUT_MASKED      (1u << 1) /* <1:1> RW:R:0:PASC_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR8_MASKED_PASC_AUD_TMIN_TIMEOUT_MASKED      (1u << 2) /* <2:2> RW:R:0:PASC_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR8_MASKED_PASC_AUD_TMAX_TIMEOUT_MASKED      (1u << 3) /* <3:3> RW:R:0:PASC_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR8_MASKED_PASC_GDRV_IN_MAX_WIDTH_TIMEOUT_MASKED    (1u << 4) /* <4:4> RW:R:0:PASC_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR8_MASKED_PASC_PTDRV_IN_MAX_WIDTH_TIMEOUT_MASKED    (1u << 5) /* <5:5> RW:R:0:PASC_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR8_MASKED_PASC_FIX_FREQ_TIMEOUT_MASKED      (1u << 6) /* <6:6> RW:R:0:PASC_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR8_MASKED_PASC_NSN_IDLE_TIMEOUT_MASKED      (1u << 7) /* <7:7> RW:R:0:PASC_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR8_MASKED_PASC_LOOP_CAL_DONE_MASKED         (1u << 8) /* <8:8> RW:R:0:PASC_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR8_MASKED_PASC_BURST_ENTRY_MASKED           (1u << 9) /* <9:9> RW:R:0:PASC_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR8_MASKED_PASC_CCM_2_DCM_CHG_MASKED         (1u << 10) /* <10:10> RW:R:0:PASC_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR8_MASKED_PASC_DCM_2_CCM_CHG_MASKED         (1u << 11) /* <11:11> RW:R:0:PASC_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR8_MASKED_GDRV_LESS_THAN_TURN_OFF_MASKED    (1u << 12) /* <12:12> RW:R:0:PASC_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR8_MASKED_GDRV_GREATER_THAN_TURN_OFF_MASKED    (1u << 13) /* <13:13> RW:R:0:PASC_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR8_MASKED_GDRV_CONTENTION_MASKED            (1u << 14) /* <14:14> RW:R:0:PASC_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR8_MASKED_GDRV_INPUT_WIDTH_LESS_THAN_MIN_MASKED    (1u << 15) /* <15:15> RW:R:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASKED_CAL_FAIL_MASKED                   (1u << 16) /* <16:16> RW:R:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASKED_PASC_IDLE_MASKED                  (1u << 17) /* <17:17> RW:R:0:PASC_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR8_MASKED_PASC_FF_OV_IDLE_TIMEOUT_MASKED    (1u << 18) /* <18:18> RW:R:0:PASC_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR8_MASKED_VBTR_OPR_DONE_MASKED              (1u << 30) /* <30:30> RW:R:0:VBTR_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR8_MASKED_VBTR_EXIT_DONE_MASKED             (1u << 31) /* <31:31> RW:R:0:VBTR_EN */


/*
 * IP DDFT0/1 and TR_OUT0/1 Selections
 */
#define PDSS_DDFT_MUX_ADDRESS                               (0x400a0580)
#define PDSS_DDFT_MUX                                       (*(volatile uint32_t *)(0x400a0580))
#define PDSS_DDFT_MUX_DEFAULT                               (0x00000000)

/*
 * This register selects the following inputs to be routed to DDFT0 and tr_out[0]
 * ports of the IP.
 * DDFT0 for [63:55]:
 *        63: pasc_ddft1
 *        62: pasc_ddft0
 *        61: vcmp_cc_vbus_fx_scan
 *            60:57 small_vconn_fx_scan
 *            56 clk_isnk
 *            55 clk_refgen
 * TR_OUT[0] for [63:55]:
 *            63: vcmp_cc_vbus_fx_scan
 *            62:55: If CCG3PA2: pwm_dischg_en_o
 *                      If CCG6:
 *                                62:58 5'd0
 *                                57: (comp_out_lv[4] | comp_out_lv[5])
 *                                56: comp_out_lv[5]
 *                                55: comp_out_lv[4]
 *                    if PAG1S:
 *                         56:  pasc_ddft1
 *                         55: pasc_ddft0
 * 54 pd2_reg_control_enable_vcr
 * 53 cc1_ovp_fx_scan
 * 52 cc2_ovp_fx_scan
 * 51 cc1_ocp_fx_scan
 * 50 cc2_ocp_fx_scan
 * 49 adc_cmp_out_fx_scan[2]
 * 48 adc_cmp_out_fx_scan[3]
 * 47 1'b0//intr_swap_queue_set
 * 46 1'b0//intr_swap_unstable_set
 * 45 intr_swap_disconnect_set
 * 44 intr_swap_rcvd_set
 * 43 intr_swap_pulse_set
 * 42 intr_swapt_command_done_set
 * 41 intr_ddft1
 * 40 intr_ddft0
 * 39 ncell_ddft1
 * 38 ncell_ddft0
 * 37 1'b0
 * 36 1'b0
 * 35 intr_hpd_queue_set
 * 34 intr_hpd_unstable_set
 * 33 intr_hpd_unpluged_set
 * 32 intr_hpd_pluged_set
 * 31 intr_hpd_irq_set
 * 30 intr_hpdt_command_done_set
 * 29 hpdin_fx_scan
 * 28 ddft_collision_src[4]
 * 27 ddft_collision_src[3]
 * 26 ddft_collision_src[2]
 * 25 ddft_collision_src[1]
 * 24 ddft_collision_src[0]
 * 23 adc_cmp_out_fx_scan[0]
 * 22 adc_cmp_out_fx_scan[1]
 * 21 v5v_fx_scan
 * 20 ddft_cc_core_rx_data
 * 19 swapr_in_fx_scan
 * 18 cc_ctrl_0_tx_en
 * 17 ddft_cc_tx_data_eop
 * 16 ddft_cc_tx_data_valid
 * 15 clk_tx  (This is 0 for tr_out[0])
 * 14 vconn2_fx_scan
 * 13 vconn1_fx_scan
 * 12 vcmp_dn_fx_scan
 * 11 vcmp_la_fx_scan
 * 10 vcmp_up_fx_scan
 * 9 ddft_sop_valid
 * 8 ddft_rx_eop
 * 7 ddft_raw_cc_rx_valid
 * 6 ddft_cc_rx_bit_en
 * 5 ddft_cc_core_tx_data
 * 4 sip_cc_tx_data
 * 3 refgen_clk_out  (This is 0 for tr_out[0])
 * 2 cc1_fx_scan
 * 1 cc2_fx_scan
 * 0 0:DDFT0 from previous mxusbpd instantiation if any
 */
#define PDSS_DDFT_MUX_DDFT0_SEL_MASK                        (0x0000003f) /* <0:5> R:RW:0: */
#define PDSS_DDFT_MUX_DDFT0_SEL_POS                         (0)


/*
 * This register selects the following inputs to be routed to DDFT0 and tr_out[0]
 * ports of the IP.
 * DDFT0 for [63:55]:
 *        63: pasc_ddft1
 *        62: pasc_ddft0
 *        61: vcmp_cc_vbus_fx_scan
 *            60:57 small_vconn_fx_scan
 *            56 clk_isnk
 *            55 clk_refgen
 * TR_OUT[0] for [63:55]:
 *            63: vcmp_cc_vbus_fx_scan
 *            62:55: If CCG3PA2: pwm_dischg_en_o
 *                      If CCG6:
 *                                62:58 5'd0
 *                                57: (comp_out_lv[4] | comp_out_lv[5])
 *                                56: comp_out_lv[5]
 *                                55: comp_out_lv[4]
 *                    if PAG1S:
 *                         56:  pasc_ddft1
 *                         55: pasc_ddft0
 * 54 pd2_reg_control_enable_vcr
 * 53 cc1_ovp_fx_scan
 * 52 cc2_ovp_fx_scan
 * 51 cc1_ocp_fx_scan
 * 50 cc2_ocp_fx_scan
 * 49 adc_cmp_out_fx_scan[2]
 * 48 adc_cmp_out_fx_scan[3]
 * 47 1'b0//intr_swap_queue_set
 * 46 1'b0//intr_swap_unstable_set
 * 45 intr_swap_disconnect_set
 * 44 intr_swap_rcvd_set
 * 43 intr_swap_pulse_set
 * 42 intr_swapt_command_done_set
 * 41 intr_ddft1
 * 40 intr_ddft0
 * 39 ncell_ddft1
 * 38 ncell_ddft0
 * 37 1'b0
 * 36 1'b0
 * 35 intr_hpd_queue_set
 * 34 intr_hpd_unstable_set
 * 33 intr_hpd_unpluged_set
 * 32 intr_hpd_pluged_set
 * 31 intr_hpd_irq_set
 * 30 intr_hpdt_command_done_set
 * 29 hpdin_fx_scan
 * 28 ddft_collision_src[4]
 * 27 ddft_collision_src[3]
 * 26 ddft_collision_src[2]
 * 25 ddft_collision_src[1]
 * 24 ddft_collision_src[0]
 * 23 adc_cmp_out_fx_scan[0]
 * 22 adc_cmp_out_fx_scan[1]
 * 21 v5v_fx_scan
 * 20 ddft_cc_core_rx_data
 * 19 swapr_in_fx_scan
 * 18 cc_ctrl_0_tx_en
 * 17 ddft_cc_tx_data_eop
 * 16 ddft_cc_tx_data_valid
 * 15 clk_tx  (This is 0 for tr_out[0])
 * 14 vconn2_fx_scan
 * 13 vconn1_fx_scan
 * 12 vcmp_dn_fx_scan
 * 11 vcmp_la_fx_scan
 * 10 vcmp_up_fx_scan
 * 9 ddft_sop_valid
 * 8 ddft_rx_eop
 * 7 ddft_raw_cc_rx_valid
 * 6 ddft_cc_rx_bit_en
 * 5 ddft_cc_core_tx_data
 * 4 sip_cc_tx_data
 * 3 refgen_clk_out  (This is 0 for tr_out[0])
 * 2 cc1_fx_scan
 * 1 cc2_fx_scan
 * 0 0:DDFT0 from previous mxusbpd instantiation if any
 */
#define PDSS_DDFT_MUX_DDFT1_SEL_MASK                        (0x00000fc0) /* <6:11> R:RW:0: */
#define PDSS_DDFT_MUX_DDFT1_SEL_POS                         (6)


/*
 * Interrupt DDFT Selections
 */
#define PDSS_INTR_DDFT_MUX_ADDRESS                          (0x400a0584)
#define PDSS_INTR_DDFT_MUX                                  (*(volatile uint32_t *)(0x400a0584))
#define PDSS_INTR_DDFT_MUX_DEFAULT                          (0x00000000)

/*
 * 247 MXUSBPD_REGS_INST.intr15_cause_pds_vreg_vbus_done
 * 246 MXUSBPD_REGS_INST.intr15_ea_cc_flag_changed_done
 * 245 MXUSBPD_REGS_INST.intr15_ff_ov_changed
 * 244 MXUSBPD_REGS_INST.intr15_ff_uv_changed
 * 243 MXUSBPD_REGS_INST.intr15_zcdf_out_changed
 * 242 MXUSBPD_REGS_INST.intr15_peakdet_out_changed
 * 241 MXUSBPD_REGS_INST.intr15_peakdet_rst_out_changed
 * 240 MXUSBPD_REGS_INST.intr15_peakdet_clcmp_raw_out_changed
 * 239 MXUSBPD_REGS_INST.intr15_sr_sen_ovp_out_changed
 * 238 MXUSBPD_REGS_INST.intr15_pwm_out_changed
 * 237 MXUSBPD_REGS_INST.intr15_skip_out_changed
 * 236 MXUSBPD_REGS_INST.intr15_burst_exit_out_changed
 * 235 MXUSBPD_REGS_INST.intr15_pds_scp_changed
 * 234 MXUSBPD_REGS_INST.intr15_nsn_out_changed
 * 233 MXUSBPD_REGS_INST.intr15_zcd_out_changed
 * 232 MXUSBPD_REGS_INST.intr8_pasc_idle
 * 231 MXUSBPD_REGS_INST.intr8_vbtr_exit_done
 * 230 MXUSBPD_REGS_INST.intr8_vbtr_opr_done
 * 229 MXUSBPD_REGS_INST.intr8_pasc_dcm_2_ccm_chg
 * 228 MXUSBPD_REGS_INST.intr8_pasc_ccm_2_dcm_chg
 * 227 MXUSBPD_REGS_INST.intr8_gdrv_less_than_turn_off
 * 226 MXUSBPD_REGS_INST.intr8_gdrv_input_width_less_than_min
 * 225 MXUSBPD_REGS_INST.intr8_cal_fail
 * 224 MXUSBPD_REGS_INST.intr8_gdrv_contentiion
 * 223 MXUSBPD_REGS_INST.intr8_gdrv_greater_than_turn_off
 * 222 MXUSBPD_REGS_INST.intr8_pasc_loop_cal_done
 * 221 MXUSBPD_REGS_INST.intr8_pasc_burst_entry
 * 220 MXUSBPD_REGS_INST.intr8_pasc_var_tmin_timeout
 * 219 MXUSBPD_REGS_INST.intr8_pasc_nsn_idle_timeout
 * 218 MXUSBPD_REGS_INST.intr8_pasc_ff_ov_idle_timeout
 * 217 MXUSBPD_REGS_INST.intr8_pasc_fix_freq_timeout
 * 216 MXUSBPD_REGS_INST.intr8_pasc_var_tmax_timeout
 * 215 MXUSBPD_REGS_INST.intr8_pasc_aud_tmin_timeout
 * 214 MXUSBPD_REGS_INST.intr8_pasc_aud_tmax_timeout
 * 213 MXUSBPD_REGS_INST.intr8_pasc_gdrv_in_max_width_timeout
 * 212 MXUSBPD_REGS_INST.intr8_pasc_max_width_timeout
 * 211 MXUSBPD_REGS_INST.intr1_lf_cntr_match
 * 210 MXUSBPD_REGS_INST.intr13_cause_cc_vbus_changed
 * 209 MXUSBPD_REGS_INST.intr13_set_csa_vbus_ovp_changed
 * 208 MXUSBPD_REGS_INST.intr13_set_csa_comp_out_changed
 * 207 MXUSBPD_REGS_INST.intr13_set_csa_out_changed
 * 206 MXUSBPD_REGS_INST.intr13_set_csa_scp_changed
 * 205 MXUSBPD_REGS_INST.intr13_set_csa_ocp_changed
 * 204:181 MXUSBPD_REGS_INST.intr5_set_edge_changed,
 * 180:173 MXUSBPD_REGS_INST.intr3_set_sbu1_sbu2_ovp_changed,
 * 172:165 MXUSBPD_REGS_INST.intr7_set_clk_lf_edge_changed,
 * 164:157 MXUSBPD_REGS_INST.intr9_set_det_shv_det_changed,
 * 156:149 MXUSBPD_REGS_INST.intr9_set_bch_det_changed,
 * 148:145 MXUSBPD_REGS_INST.intr3_set_csa_oc_changed,
 * 144:141 MXUSBPD_REGS_INST.intr3_set_chgdet_changed,
 * 140:137 MXUSBPD_REGS_INST.intr3_set_vreg20_vbus_changed,
 * 136:133 MXUSBPD_REGS_INST.intr3_set_csa_vbus_changed,
 * 132:129 MXUSBPD_REGS_INST.intr9_set_qcom_rcvr_dm_changed,
 * 128:125 MXUSBPD_REGS_INST.intr9_set_qcom_rcvr_dp_changed,
 * 124:121 MXUSBPD_REGS_INST.intr9_set_shvreg_det_changed,
 * 120:117 MXUSBPD_REGS_INST.intr4_set_afc_ping_recvd,
 * 116:113 MXUSBPD_REGS_INST.intr4_set_afc_sm_idle,
 * 112:109 MXUSBPD_REGS_INST.intr4_set_afc_timeout,
 * 108:105 MXUSBPD_REGS_INST.intr4_set_afc_rx_reset,
 * 104:101 MXUSBPD_REGS_INST.intr4_set_update_ping_pong,
 * 100:97  MXUSBPD_REGS_INST.intr4_set_afc_error,
 * 96:93   MXUSBPD_REGS_INST.intr6_set_qc_3_d_p_pulse_rcvd,
 * 92:89   MXUSBPD_REGS_INST.intr6_set_qc_3_d_m_pulse_rcvd,
 * 88:85   MXUSBPD_REGS_INST.intr6_set_qc_3_device_req_sent,
 * 84:83   MXUSBPD_REGS_INST.intr11_set_filt2_edge_changed,
 * 82:81   MXUSBPD_REGS_INST.intr1_set_ngdo_spacing_done[3:2],
 * 80      MXUSBPD_REGS_INST.intr3_set_vsys_changed,
 * 79      MXUSBPD_REGS_INST.intr1_set_cc2_ocp_changed,
 * 78      MXUSBPD_REGS_INST.intr1_set_cc1_ocp_changed,
 * 77      MXUSBPD_REGS_INST.intr1_set_cc2_ovp_changed,
 * 76      MXUSBPD_REGS_INST.intr1_set_cc1_ovp_changed,
 * 75      MXUSBPD_REGS_INST.intr1_set_drp_attached_detected,
 * 74      MXUSBPD_REGS_INST.intr3_set_cmp_out_changed[3],
 * 73      MXUSBPD_REGS_INST.intr3_set_cmp_out_changed[2],
 * 72      MXUSBPD_REGS_INST.intr0_set_sar_done[3],
 * 71      MXUSBPD_REGS_INST.intr0_set_sar_done[2],
 * 70      MXUSBPD_REGS_INST.intr1_set_vswap_vbus_less_5_done,
 * 69      MXUSBPD_REGS_INST.intr2_set_swap_command_done,
 * 68      1'b0
 * 67      1'b0
 * 66      MXUSBPD_REGS_INST.intr2_set_swap_disconnect,
 * 65      MXUSBPD_REGS_INST.intr2_set_swap_rcvd,
 * 64      MXUSBPD_REGS_INST.intr2_set_swap_pulse,
 * 63      MXUSBPD_REGS_INST.intr1_set_ngdo_spacing_done[0],
 * 62      MXUSBPD_REGS_INST.intr1_set_ngdo_spacing_done[1],
 * 61      MXUSBPD_REGS_INST.intr2_set_vreg20v_switch_done,
 * 60      MXUSBPD_REGS_INST.intr2_set_vddd_sw_switch_done,
 * 59      MXUSBPD_REGS_INST.intr2_set_chunk_det,
 * 58      MXUSBPD_REGS_INST.intr2_set_tx_sram_under_flow,
 * 57      MXUSBPD_REGS_INST.intr2_set_rx_sram_over_flow,
 * 56      1'b0
 * 55      1'b0
 * 54      1'b0
 * 53      MXUSBPD_REGS_INST.intr2_set_extended_msg_det,
 * 52      MXUSBPD_REGS_INST.intr2_set_hpdt_command_done,
 * 51      MXUSBPD_REGS_INST.intr2_set_hpd_queue,
 * 50      MXUSBPD_REGS_INST.intr2_set_hpd_unstable,
 * 49      MXUSBPD_REGS_INST.intr2_set_hpd_unpluged,
 * 48      MXUSBPD_REGS_INST.intr2_set_hpd_pluged,
 * 47      MXUSBPD_REGS_INST.intr2_set_hpd_irq,
 * 46      MXUSBPD_REGS_INST.intr2_set_ui_cal_done,
 * 45      1'b0
 * 44      1'b0
 * 43      1'b0
 * 42      1'b0
 * 41      MXUSBPD_REGS_INST.intr1_set_hpdin_changed,
 * 40      MXUSBPD_REGS_INST.intr3_set_cmp_out_changed[1],
 * 39      MXUSBPD_REGS_INST.intr3_set_cmp_out_changed[0],
 * 38      MXUSBPD_REGS_INST.intr1_set_v5v_changed,
 * 37      MXUSBPD_REGS_INST.intr1_set_vcmp_dn_changed,
 * 36      MXUSBPD_REGS_INST.intr1_set_vcmp_up_changed,
 * 35      MXUSBPD_REGS_INST.intr1_set_vcmp_la_changed,
 * 34      MXUSBPD_REGS_INST.intr1_set_cc2_changed,
 * 33      MXUSBPD_REGS_INST.intr1_set_cc1_changed,
 * 32      MXUSBPD_REGS_INST.intr1_set_vconn2_changed,
 * 31      MXUSBPD_REGS_INST.intr1_set_vconn1_changed,
 * 30      1'b0
 * 29      MXUSBPD_REGS_INST.intr0_set_sar_done[1],
 * 28      MXUSBPD_REGS_INST.intr0_set_rx_state_idle,
 * 27      MXUSBPD_REGS_INST.intr0_set_tx_state_idle,
 * 26      MXUSBPD_REGS_INST.intr0_set_tx_regulator_enabled,
 * 25      MXUSBPD_REGS_INST.intr0_set_tx_cc_data_oen_deassert,
 * 24      MXUSBPD_REGS_INST.intr0_set_tx_cc_data_oen_assert,
 * 23      MXUSBPD_REGS_INST.intr0_set_kchar_error,
 * 22      MXUSBPD_REGS_INST.intr0_set_tx_retry_enable_clrd,
 * 21      MXUSBPD_REGS_INST.intr0_set_rx_sram_half_end,
 * 20      MXUSBPD_REGS_INST.intr0_set_tx_sram_half_end,
 * 19      1'b0
 * 18      MXUSBPD_REGS_INST.MXUSBPD_REGS_INST.intr0_set_collision_type4,
 * 17      MXUSBPD_REGS_INST.MXUSBPD_REGS_INST.intr0_set_collision_type3,
 * 16      MXUSBPD_REGS_INST.MXUSBPD_REGS_INST.intr0_set_collision_type2,
 * 15      MXUSBPD_REGS_INST.MXUSBPD_REGS_INST.intr0_set_collision_type1
 * 14      MXUSBPD_REGS_INST.intr0_set_crc_rx_timer_exp,
 * 13      MXUSBPD_REGS_INST.intr0_set_cc_no_valid_data_detected,
 * 12      MXUSBPD_REGS_INST.intr0_set_cc_valid_data_detected,
 * 11      MXUSBPD_REGS_INST.intr0_set_tx_goodcrc_msg_done,
 * 10      MXUSBPD_REGS_INST.intr0_set_sar_done[0],
 * 9       MXUSBPD_REGS_INST.intr0_set_rcv_rst,
 * 8       MXUSBPD_REGS_INST.intr0_set_tx_hard_rst_done,
 * 7       MXUSBPD_REGS_INST.intr0_set_tx_packet_done,
 * 6       MXUSBPD_REGS_INST.intr0_set_rx_over_run,
 * 5       MXUSBPD_REGS_INST.intr0_set_eop_error,
 * 4       MXUSBPD_REGS_INST.intr0_set_rcv_expt_goodcrc_msg_complete,
 * 3       MXUSBPD_REGS_INST.intr0_set_rcv_goodcrc_msg_complete,
 * 2       MXUSBPD_REGS_INST.intr0_set_rx_sop,
 * 1       MXUSBPD_REGS_INST.intr0_set_rcv_bad_packet_complete,
 * 0       MXUSBPD_REGS_INST.intr0_set_rcv_good_packet_complete,
 */
#define PDSS_INTR_DDFT_MUX_INTR_DDFT0_SEL_MASK              (0x000000ff) /* <0:7> R:RW:0: */
#define PDSS_INTR_DDFT_MUX_INTR_DDFT0_SEL_POS               (0)


/*
 * 247 MXUSBPD_REGS_INST.intr15_cause_pds_vreg_vbus_done
 * 246 MXUSBPD_REGS_INST.intr15_ea_cc_flag_changed_done
 * 245 MXUSBPD_REGS_INST.intr15_ff_ov_changed
 * 244 MXUSBPD_REGS_INST.intr15_ff_uv_changed
 * 243 MXUSBPD_REGS_INST.intr15_zcdf_out_changed
 * 242 MXUSBPD_REGS_INST.intr15_peakdet_out_changed
 * 241 MXUSBPD_REGS_INST.intr15_peakdet_rst_out_changed
 * 240 MXUSBPD_REGS_INST.intr15_peakdet_clcmp_raw_out_changed
 * 239 MXUSBPD_REGS_INST.intr15_sr_sen_ovp_out_changed
 * 238 MXUSBPD_REGS_INST.intr15_pwm_out_changed
 * 237 MXUSBPD_REGS_INST.intr15_skip_out_changed
 * 236 MXUSBPD_REGS_INST.intr15_burst_exit_out_changed
 * 235 MXUSBPD_REGS_INST.intr15_pds_scp_changed
 * 234 MXUSBPD_REGS_INST.intr15_nsn_out_changed
 * 233 MXUSBPD_REGS_INST.intr15_zcd_out_changed
 * 232 MXUSBPD_REGS_INST.intr8_pasc_idle
 * 231 MXUSBPD_REGS_INST.intr8_vbtr_exit_done
 * 230 MXUSBPD_REGS_INST.intr8_vbtr_opr_done
 * 229 MXUSBPD_REGS_INST.intr8_pasc_dcm_2_ccm_chg
 * 228 MXUSBPD_REGS_INST.intr8_pasc_ccm_2_dcm_chg
 * 227 MXUSBPD_REGS_INST.intr8_gdrv_less_than_turn_off
 * 226 MXUSBPD_REGS_INST.intr8_gdrv_input_width_less_than_min
 * 225 MXUSBPD_REGS_INST.intr8_cal_fail
 * 224 MXUSBPD_REGS_INST.intr8_gdrv_contentiion
 * 223 MXUSBPD_REGS_INST.intr8_gdrv_greater_than_turn_off
 * 222 MXUSBPD_REGS_INST.intr8_pasc_loop_cal_done
 * 221 MXUSBPD_REGS_INST.intr8_pasc_burst_entry
 * 220 MXUSBPD_REGS_INST.intr8_pasc_var_tmin_timeout
 * 219 MXUSBPD_REGS_INST.intr8_pasc_nsn_idle_timeout
 * 218 MXUSBPD_REGS_INST.intr8_pasc_ff_ov_idle_timeout
 * 217 MXUSBPD_REGS_INST.intr8_pasc_fix_freq_timeout
 * 216 MXUSBPD_REGS_INST.intr8_pasc_var_tmax_timeout
 * 215 MXUSBPD_REGS_INST.intr8_pasc_aud_tmin_timeout
 * 214 MXUSBPD_REGS_INST.intr8_pasc_aud_tmax_timeout
 * 213 MXUSBPD_REGS_INST.intr8_pasc_gdrv_in_max_width_timeout
 * 212 MXUSBPD_REGS_INST.intr8_pasc_max_width_timeout
 * 211 MXUSBPD_REGS_INST.intr1_lf_cntr_match
 * 210 MXUSBPD_REGS_INST.intr13_cause_cc_vbus_changed
 * 209 MXUSBPD_REGS_INST.intr13_set_csa_vbus_ovp_changed
 * 208 MXUSBPD_REGS_INST.intr13_set_csa_comp_out_changed
 * 207 MXUSBPD_REGS_INST.intr13_set_csa_out_changed
 * 206 MXUSBPD_REGS_INST.intr13_set_csa_scp_changed
 * 205 MXUSBPD_REGS_INST.intr13_set_csa_ocp_changed
 * 204:181 MXUSBPD_REGS_INST.intr5_set_edge_changed,
 * 180:173 MXUSBPD_REGS_INST.intr3_set_sbu1_sbu2_ovp_changed,
 * 172:165 MXUSBPD_REGS_INST.intr7_set_clk_lf_edge_changed,
 * 164:157 MXUSBPD_REGS_INST.intr9_set_det_shv_det_changed,
 * 156:149 MXUSBPD_REGS_INST.intr9_set_bch_det_changed,
 * 148:145 MXUSBPD_REGS_INST.intr3_set_csa_oc_changed,
 * 144:141 MXUSBPD_REGS_INST.intr3_set_chgdet_changed,
 * 140:137 MXUSBPD_REGS_INST.intr3_set_vreg20_vbus_changed,
 * 136:133 MXUSBPD_REGS_INST.intr3_set_csa_vbus_changed,
 * 132:129 MXUSBPD_REGS_INST.intr9_set_qcom_rcvr_dm_changed,
 * 128:125 MXUSBPD_REGS_INST.intr9_set_qcom_rcvr_dp_changed,
 * 124:121 MXUSBPD_REGS_INST.intr9_set_shvreg_det_changed,
 * 120:117 MXUSBPD_REGS_INST.intr4_set_afc_ping_recvd,
 * 116:113 MXUSBPD_REGS_INST.intr4_set_afc_sm_idle,
 * 112:109 MXUSBPD_REGS_INST.intr4_set_afc_timeout,
 * 108:105 MXUSBPD_REGS_INST.intr4_set_afc_rx_reset,
 * 104:101 MXUSBPD_REGS_INST.intr4_set_update_ping_pong,
 * 100:97  MXUSBPD_REGS_INST.intr4_set_afc_error,
 * 96:93   MXUSBPD_REGS_INST.intr6_set_qc_3_d_p_pulse_rcvd,
 * 92:89   MXUSBPD_REGS_INST.intr6_set_qc_3_d_m_pulse_rcvd,
 * 88:85   MXUSBPD_REGS_INST.intr6_set_qc_3_device_req_sent,
 * 84:83   MXUSBPD_REGS_INST.intr11_set_filt2_edge_changed,
 * 82:81   MXUSBPD_REGS_INST.intr1_set_ngdo_spacing_done[3:2],
 * 80      MXUSBPD_REGS_INST.intr3_set_vsys_changed,
 * 79      MXUSBPD_REGS_INST.intr1_set_cc2_ocp_changed,
 * 78      MXUSBPD_REGS_INST.intr1_set_cc1_ocp_changed,
 * 77      MXUSBPD_REGS_INST.intr1_set_cc2_ovp_changed,
 * 76      MXUSBPD_REGS_INST.intr1_set_cc1_ovp_changed,
 * 75      MXUSBPD_REGS_INST.intr1_set_drp_attached_detected,
 * 74      MXUSBPD_REGS_INST.intr3_set_cmp_out_changed[3],
 * 73      MXUSBPD_REGS_INST.intr3_set_cmp_out_changed[2],
 * 72      MXUSBPD_REGS_INST.intr0_set_sar_done[3],
 * 71      MXUSBPD_REGS_INST.intr0_set_sar_done[2],
 * 70      MXUSBPD_REGS_INST.intr1_set_vswap_vbus_less_5_done,
 * 69      MXUSBPD_REGS_INST.intr2_set_swap_command_done,
 * 68      1'b0
 * 67      1'b0
 * 66      MXUSBPD_REGS_INST.intr2_set_swap_disconnect,
 * 65      MXUSBPD_REGS_INST.intr2_set_swap_rcvd,
 * 64      MXUSBPD_REGS_INST.intr2_set_swap_pulse,
 * 63      MXUSBPD_REGS_INST.intr1_set_ngdo_spacing_done[0],
 * 62      MXUSBPD_REGS_INST.intr1_set_ngdo_spacing_done[1],
 * 61      MXUSBPD_REGS_INST.intr2_set_vreg20v_switch_done,
 * 60      MXUSBPD_REGS_INST.intr2_set_vddd_sw_switch_done,
 * 59      MXUSBPD_REGS_INST.intr2_set_chunk_det,
 * 58      MXUSBPD_REGS_INST.intr2_set_tx_sram_under_flow,
 * 57      MXUSBPD_REGS_INST.intr2_set_rx_sram_over_flow,
 * 56      1'b0
 * 55      1'b0
 * 54      1'b0
 * 53      MXUSBPD_REGS_INST.intr2_set_extended_msg_det,
 * 52      MXUSBPD_REGS_INST.intr2_set_hpdt_command_done,
 * 51      MXUSBPD_REGS_INST.intr2_set_hpd_queue,
 * 50      MXUSBPD_REGS_INST.intr2_set_hpd_unstable,
 * 49      MXUSBPD_REGS_INST.intr2_set_hpd_unpluged,
 * 48      MXUSBPD_REGS_INST.intr2_set_hpd_pluged,
 * 47      MXUSBPD_REGS_INST.intr2_set_hpd_irq,
 * 46      MXUSBPD_REGS_INST.intr2_set_ui_cal_done,
 * 45      1'b0
 * 44      1'b0
 * 43      1'b0
 * 42      1'b0
 * 41      MXUSBPD_REGS_INST.intr1_set_hpdin_changed,
 * 40      MXUSBPD_REGS_INST.intr3_set_cmp_out_changed[1],
 * 39      MXUSBPD_REGS_INST.intr3_set_cmp_out_changed[0],
 * 38      MXUSBPD_REGS_INST.intr1_set_v5v_changed,
 * 37      MXUSBPD_REGS_INST.intr1_set_vcmp_dn_changed,
 * 36      MXUSBPD_REGS_INST.intr1_set_vcmp_up_changed,
 * 35      MXUSBPD_REGS_INST.intr1_set_vcmp_la_changed,
 * 34      MXUSBPD_REGS_INST.intr1_set_cc2_changed,
 * 33      MXUSBPD_REGS_INST.intr1_set_cc1_changed,
 * 32      MXUSBPD_REGS_INST.intr1_set_vconn2_changed,
 * 31      MXUSBPD_REGS_INST.intr1_set_vconn1_changed,
 * 30      1'b0
 * 29      MXUSBPD_REGS_INST.intr0_set_sar_done[1],
 * 28      MXUSBPD_REGS_INST.intr0_set_rx_state_idle,
 * 27      MXUSBPD_REGS_INST.intr0_set_tx_state_idle,
 * 26      MXUSBPD_REGS_INST.intr0_set_tx_regulator_enabled,
 * 25      MXUSBPD_REGS_INST.intr0_set_tx_cc_data_oen_deassert,
 * 24      MXUSBPD_REGS_INST.intr0_set_tx_cc_data_oen_assert,
 * 23      MXUSBPD_REGS_INST.intr0_set_kchar_error,
 * 22      MXUSBPD_REGS_INST.intr0_set_tx_retry_enable_clrd,
 * 21      MXUSBPD_REGS_INST.intr0_set_rx_sram_half_end,
 * 20      MXUSBPD_REGS_INST.intr0_set_tx_sram_half_end,
 * 19      1'b0
 * 18      MXUSBPD_REGS_INST.MXUSBPD_REGS_INST.intr0_set_collision_type4,
 * 17      MXUSBPD_REGS_INST.MXUSBPD_REGS_INST.intr0_set_collision_type3,
 * 16      MXUSBPD_REGS_INST.MXUSBPD_REGS_INST.intr0_set_collision_type2,
 * 15      MXUSBPD_REGS_INST.MXUSBPD_REGS_INST.intr0_set_collision_type1
 * 14      MXUSBPD_REGS_INST.intr0_set_crc_rx_timer_exp,
 * 13      MXUSBPD_REGS_INST.intr0_set_cc_no_valid_data_detected,
 * 12      MXUSBPD_REGS_INST.intr0_set_cc_valid_data_detected,
 * 11      MXUSBPD_REGS_INST.intr0_set_tx_goodcrc_msg_done,
 * 10      MXUSBPD_REGS_INST.intr0_set_sar_done[0],
 * 9       MXUSBPD_REGS_INST.intr0_set_rcv_rst,
 * 8       MXUSBPD_REGS_INST.intr0_set_tx_hard_rst_done,
 * 7       MXUSBPD_REGS_INST.intr0_set_tx_packet_done,
 * 6       MXUSBPD_REGS_INST.intr0_set_rx_over_run,
 * 5       MXUSBPD_REGS_INST.intr0_set_eop_error,
 * 4       MXUSBPD_REGS_INST.intr0_set_rcv_expt_goodcrc_msg_complete,
 * 3       MXUSBPD_REGS_INST.intr0_set_rcv_goodcrc_msg_complete,
 * 2       MXUSBPD_REGS_INST.intr0_set_rx_sop,
 * 1       MXUSBPD_REGS_INST.intr0_set_rcv_bad_packet_complete,
 * 0       MXUSBPD_REGS_INST.intr0_set_rcv_good_packet_complete,
 */
#define PDSS_INTR_DDFT_MUX_INTR_DDFT1_SEL_MASK              (0x0000ff00) /* <8:15> R:RW:0: */
#define PDSS_INTR_DDFT_MUX_INTR_DDFT1_SEL_POS               (8)


/*
 * NCELL DDFT Selections
 */
#define PDSS_NCELL_DDFT_MUX_ADDRESS                         (0x400a0588)
#define PDSS_NCELL_DDFT_MUX                                 (*(volatile uint32_t *)(0x400a0588))
#define PDSS_NCELL_DDFT_MUX_DEFAULT                         (0x00000000)

/*
 * 214 MXUSBPD_MMIO_INST.ngdo_ctrl_ngdo_en_lv
 * 213 MXUSBPD_MMIO_INST.ngdo_ctrl_cp_en
 * 212 MXUSBPD_MMIO_INST.pds_vreg_ctrl_vbg_en
 * 211 MXUSBPD_MMIO_INST.gdrv_ctrl_ptdrv_pulldn_en
 * 210 MXUSBPD_MMIO_INST.gdrv_ctrl_ptdrv_pullup_en
 * 209 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_ea_top.cc_flag
 * 208 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_ea_top.cc_comp_out
 * 207 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_ea_top.cc_det
 * 206 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_ea_top.cv_det
 * 205 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_vreg_top.vbus_det
 * 204 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_scp_top.scp_out
 * 203 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.ff_uv
 * 202 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.ff_ov
 * 201 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.peakdet_out
 * 200 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.peakdet_rst_out
 * 199 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.peakdet_clcmp_raw_out
 * 198 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.zcdf_out
 * 197 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.sr_sen_ovp_out
 * 196 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.nsn_out
 * 195 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.zcd_out
 * 194 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.peakdet_rst_raw_out
 * 193 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.peakdet_pkd_raw_out
 * 192 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_pwm_top.pwm_out
 * 191 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_pwm_top.skip_out
 * 190 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_pwm_top.burst_exit_out
 * 189 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_pwm_top.eacomp_out
 * 188 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_pwm_top.hclamp_out
 * 187 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_pwm_top.pwm_ramp_reset_out
 * 186 MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_gate_driver_srsns_ctrl.ddft_faults_masked
 * 185 MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_gate_driver_srsns_ctrl.ddft_async_fault_det
 * 184 MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_gate_driver_srsns_ctrl.en_lv[0]
 * 183 MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_gate_driver_ngdo_ctrl.ddft_faults_masked
 * 182 MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_gate_driver_ngdo_ctrl.ddft_async_fault_det
 * 181 MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_gate_driver_ngdo_ctrl.en_lv[0]
 * 180     MXUSBPD_HARD_TOP_INST.y_cc_ufp_nord.u_s8usbpd_cc_top.vcmp_vbus
 * 179     MXUSBPD_HARD_TOP_INST.y_csa_rcp.u_s8usbpd_csa_rcp_top.vbus_ovp_comp_out_lv
 * 178     MXUSBPD_HARD_TOP_INST.y_csa_rcp.u_s8usbpd_csa_rcp_top.rcp_comp_out_lv
 * 177     MXUSBPD_HARD_TOP_INST.y_csa_rcp.u_s8usbpd_csa_rcp_top.csa_out_d
 * 176     MXUSBPD_HARD_TOP_INST.y_csa_scp.u_s8usbpd_csa_scp_top.out_d_scp
 * 175     MXUSBPD_HARD_TOP_INST.y_csa_scp.u_s8usbpd_csa_scp_top.out_d_ocp
 * 174         MXUSBPD_HARD_TOP_INST.y_csa_rcp.clk_div2_lv
 * 173         MXUSBPD_HARD_TOP_INST.y_csa_rcp.clk_div4_lv
 * 172         MXUSBPD_HARD_TOP_INST.y_csa_rcp.clk_div8_lv
 * 171         MXUSBPD_HARD_TOP_INST.y_csa_rcp.clk_div16_lv
 * 170         MXUSBPD_HARD_TOP_INST.y_csa_rcp.rignosc_out_lv
 * 169         MXUSBPD_HARD_TOP_INST.y_csa_rcp.autozero_clk_lv
 * 168         MXUSBPD_MMIO_INST.y_vconn20_cc12_switch.u_mxusbpd_gate_driver_vconn20_pump_en_ctrl.ddft_faults_masked
 * 167         MXUSBPD_MMIO_INST.y_vconn20_cc12_switch.u_mxusbpd_gate_driver_vconn20_pump_en_ctrl.ddft_async_fault_det
 * 166         MXUSBPD_MMIO_INST.y_vconn20_cc12_switch.u_mxusbpd_gate_driver_vconn20_cc1_en_ctrl.ddft_faults_masked
 * 165         MXUSBPD_MMIO_INST.y_vconn20_cc12_switch.u_mxusbpd_gate_driver_vconn20_cc1_en_ctrl.ddft_async_fault_det
 * 164         MXUSBPD_MMIO_INST.y_vconn20_cc12_switch.u_mxusbpd_gate_driver_vconn20_cc1_en_ctrl.ddft_faults_masked
 * 163         MXUSBPD_MMIO_INST.y_vconn20_cc12_switch.u_mxusbpd_gate_driver_vconn20_cc1_en_ctrl.ddft_async_fault_det
 * 162:159 MXUSBPD_MMIO_INST.y_sbu20_sbu12_en_ctrl.u_mxusbpd_gate_driver_sbu20_sbu2_en_ctrl.ddft_faults_masked[3:0]
 * 158:155 MXUSBPD_MMIO_INST.y_sbu20_sbu12_en_ctrl.u_mxusbpd_gate_driver_sbu20_sbu2_en_ctrl.ddft_async_fault_det[3:0]
 * 154:151 MXUSBPD_MMIO_INST.y_sbu20_sbu12_en_ctrl.u_mxusbpd_gate_driver_sbu20_sbu1_en_ctrl.ddft_faults_masked[3:0]
 * 150:147 MXUSBPD_MMIO_INST.y_sbu20_sbu12_en_ctrl.u_mxusbpd_gate_driver_sbu20_sbu1_en_ctrl.ddft_async_fault_det[3:0]
 * 146:143 MXUSBPD_HARD_TOP_INST.y_ngdo.ngdo_ddft[3:0]
 * 142:139 MXUSBPD_MMIO_INST.ngdo_ddft_faults_masked[3:0]
 * 138:135 MXUSBPD_MMIO_INST.ngdo_ddft_async_fault_det[3:0]
 * 134:131 MXUSBPD_MMIO_INST.y_pgdo_pu_ctrl.u_mxusbpd_pgdo_pu_ctrl.pgdo_pu_ddft_faults_masked[3:0]
 * 130:127 MXUSBPD_MMIO_INST.y_pgdo_pu_ctrl.u_mxusbpd_pgdo_pu_ctrl.pgdo_pu_ddft_async_fault_det[3:0]
 * 126:123 MXUSBPD_MMIO_INST.y_pgdo_pu_ctrl.u_mxusbpd_pgdo_pu_ctrl.pgdo_pu_ctrl_pgdo_in_lv[3:0]
 * 122:119 MXUSBPD_MMIO_INST.y_pgdo_pu_ctrl.u_mxusbpd_pgdo_pu_ctrl.pgdo_pu_ctrl_pgdo_en_lv[3:0]
 * 118:115 MXUSBPD_MMIO_INST.y_pgdo_ctrl.u_mxusbpd_pgdo_ctrl.pgdo_ddft_faults_masked[3:0]
 * 114:111 MXUSBPD_MMIO_INST.y_pgdo_ctrl.u_mxusbpd_pgdo_ctrl.pgdo_ddft_async_fault_det[3:0]
 * 110:107 MXUSBPD_MMIO_INST.y_pgdo_ctrl.u_mxusbpd_pgdo_ctrl.pgdo_ctrl_pgdo_en_lv[3:0]
 * 106:99  MXUSBPD_MMIO_INST.y_ngdo_ctrl.u_mxusbpd_ngdo_ctrl.ngdo_ctrl_ngdo_pulldn_en_lv[7:0]
 * 98:91   MXUSBPD_MMIO_INST.y_ngdo_ctrl.u_mxusbpd_ngdo_ctrl.ngdo_ctrl_ngdo_en_lv[7:0]
 * 90:87   MXUSBPD_HARD_TOP_INST.y_cc_5vpump.u_s8usbpd_5vpump_top.pump5v_clkout_ddft[3:0]
 * 86      MXUSBPD_HARD_TOP_INST.y_vsys.u_s8usbpd_vddd_sw_top.vsys_fx_scan
 * 85:84   MXUSBPD_HARD_TOP_INST.hs_filt2_fx_scan[1:0]
 * 83:60   MXUSBPD_HARD_TOP_INST.hs_filt_fx_scan[23:0]
 * 59:52   MXUSBPD_HARD_TOP_INST.ls_filt_fx_scan[7:0]
 * 51:44   MXUSBPD_HARD_TOP_INST.y_det_shv.u_s8usbpd_det_shv_top.det_shv_fx_scan[7:0]
 * 43:36   MXUSBPD_HARD_TOP_INST.y_bch_det.u_s8usbpd_bch_chgdet_top.bch_det_fx_scan[7:0]
 * 35:32   MXUSBPD_HARD_TOP_INST.y_sbu20.u_s8usbpd_sbu20_sw_top.sbu1_ovp_fx_scan[3:0]
 * 31:28   MXUSBPD_HARD_TOP_INST.y_sbu20.u_s8usbpd_sbu20_sw_top.sbu2_ovp_fx_scan[3:0]
 * 27:24   MXUSBPD_HARD_TOP_INST.qcom_rcvr_dp_fx_scan[3:0]
 * 23:20   MXUSBPD_HARD_TOP_INST.qcom_rcvr_dm_fx_scan[3:0]
 * 19:16   MXUSBPD_HARD_TOP_INST.y_shvreg.u_s8usbpd_vreg_top.shvreg_vbus_fx_scan[3:0]
 * 15:12   MXUSBPD_HARD_TOP_INST.y_20vreg.u_s8usbpd_vreg_top.vreg_vbus_fx_scan[3:0]
 * 11:8    MXUSBPD_HARD_TOP_INST.y_csa.u_s8usbpd_csa_top.csa_vbus_fx_scan[3:0]
 * 7:4     MXUSBPD_HARD_TOP_INST.y_chgdet.u_s8usbpd_chgdet_top.chgdet_fx_scan[3:0]
 * 3:0     MXUSBPD_HARD_TOP_INST.y_csa.u_s8usbpd_csa_top.csa_oc_fx_scan[3:0]
 */
#define PDSS_NCELL_DDFT_MUX_NCELL_DDFT0_SEL_MASK            (0x000000ff) /* <0:7> R:RW:0: */
#define PDSS_NCELL_DDFT_MUX_NCELL_DDFT0_SEL_POS             (0)


/*
 * 214 MXUSBPD_MMIO_INST.ngdo_ctrl_ngdo_en_lv
 * 213 MXUSBPD_MMIO_INST.ngdo_ctrl_cp_en
 * 212 MXUSBPD_MMIO_INST.pds_vreg_ctrl_vbg_en
 * 211 MXUSBPD_MMIO_INST.gdrv_ctrl_ptdrv_pulldn_en
 * 210 MXUSBPD_MMIO_INST.gdrv_ctrl_ptdrv_pullup_en
 * 209 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_ea_top.cc_flag
 * 208 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_ea_top.cc_comp_out
 * 207 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_ea_top.cc_det
 * 206 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_ea_top.cv_det
 * 205 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_vreg_top.vbus_det
 * 204 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_scp_top.scp_out
 * 203 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.ff_uv
 * 202 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.ff_ov
 * 201 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.peakdet_out
 * 200 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.peakdet_rst_out
 * 199 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.peakdet_clcmp_raw_out
 * 198 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.zcdf_out
 * 197 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.sr_sen_ovp_out
 * 196 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.nsn_out
 * 195 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.zcd_out
 * 194 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.peakdet_rst_raw_out
 * 193 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_srsense_top.peakdet_pkd_raw_out
 * 192 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_pwm_top.pwm_out
 * 191 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_pwm_top.skip_out
 * 190 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_pwm_top.burst_exit_out
 * 189 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_pwm_top.eacomp_out
 * 188 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_pwm_top.hclamp_out
 * 187 MXUSBPD_HARD_TOP_INST.y_pag1s.u_s8usbpd_pwm_top.pwm_ramp_reset_out
 * 186 MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_gate_driver_srsns_ctrl.ddft_faults_masked
 * 185 MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_gate_driver_srsns_ctrl.ddft_async_fault_det
 * 184 MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_gate_driver_srsns_ctrl.en_lv[0]
 * 183 MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_gate_driver_ngdo_ctrl.ddft_faults_masked
 * 182 MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_gate_driver_ngdo_ctrl.ddft_async_fault_det
 * 181 MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_gate_driver_ngdo_ctrl.en_lv[0]
 * 180     MXUSBPD_HARD_TOP_INST.y_cc_ufp_nord.u_s8usbpd_cc_top.vcmp_vbus
 * 179     MXUSBPD_HARD_TOP_INST.y_csa_rcp.u_s8usbpd_csa_rcp_top.vbus_ovp_comp_out_lv
 * 178     MXUSBPD_HARD_TOP_INST.y_csa_rcp.u_s8usbpd_csa_rcp_top.rcp_comp_out_lv
 * 177     MXUSBPD_HARD_TOP_INST.y_csa_rcp.u_s8usbpd_csa_rcp_top.csa_out_d
 * 176     MXUSBPD_HARD_TOP_INST.y_csa_scp.u_s8usbpd_csa_scp_top.out_d_scp
 * 175     MXUSBPD_HARD_TOP_INST.y_csa_scp.u_s8usbpd_csa_scp_top.out_d_ocp
 * 174         MXUSBPD_HARD_TOP_INST.y_csa_rcp.clk_div2_lv
 * 173         MXUSBPD_HARD_TOP_INST.y_csa_rcp.clk_div4_lv
 * 172         MXUSBPD_HARD_TOP_INST.y_csa_rcp.clk_div8_lv
 * 171         MXUSBPD_HARD_TOP_INST.y_csa_rcp.clk_div16_lv
 * 170         MXUSBPD_HARD_TOP_INST.y_csa_rcp.rignosc_out_lv
 * 169         MXUSBPD_HARD_TOP_INST.y_csa_rcp.autozero_clk_lv
 * 168         MXUSBPD_MMIO_INST.y_vconn20_cc12_switch.u_mxusbpd_gate_driver_vconn20_pump_en_ctrl.ddft_faults_masked
 * 167         MXUSBPD_MMIO_INST.y_vconn20_cc12_switch.u_mxusbpd_gate_driver_vconn20_pump_en_ctrl.ddft_async_fault_det
 * 166         MXUSBPD_MMIO_INST.y_vconn20_cc12_switch.u_mxusbpd_gate_driver_vconn20_cc1_en_ctrl.ddft_faults_masked
 * 165         MXUSBPD_MMIO_INST.y_vconn20_cc12_switch.u_mxusbpd_gate_driver_vconn20_cc1_en_ctrl.ddft_async_fault_det
 * 164         MXUSBPD_MMIO_INST.y_vconn20_cc12_switch.u_mxusbpd_gate_driver_vconn20_cc1_en_ctrl.ddft_faults_masked
 * 163         MXUSBPD_MMIO_INST.y_vconn20_cc12_switch.u_mxusbpd_gate_driver_vconn20_cc1_en_ctrl.ddft_async_fault_det
 * 162:159 MXUSBPD_MMIO_INST.y_sbu20_sbu12_en_ctrl.u_mxusbpd_gate_driver_sbu20_sbu2_en_ctrl.ddft_faults_masked[3:0]
 * 158:155 MXUSBPD_MMIO_INST.y_sbu20_sbu12_en_ctrl.u_mxusbpd_gate_driver_sbu20_sbu2_en_ctrl.ddft_async_fault_det[3:0]
 * 154:151 MXUSBPD_MMIO_INST.y_sbu20_sbu12_en_ctrl.u_mxusbpd_gate_driver_sbu20_sbu1_en_ctrl.ddft_faults_masked[3:0]
 * 150:147 MXUSBPD_MMIO_INST.y_sbu20_sbu12_en_ctrl.u_mxusbpd_gate_driver_sbu20_sbu1_en_ctrl.ddft_async_fault_det[3:0]
 * 146:143 MXUSBPD_HARD_TOP_INST.y_ngdo.ngdo_ddft[3:0]
 * 142:139 MXUSBPD_MMIO_INST.ngdo_ddft_faults_masked[3:0]
 * 138:135 MXUSBPD_MMIO_INST.ngdo_ddft_async_fault_det[3:0]
 * 134:131 MXUSBPD_MMIO_INST.y_pgdo_pu_ctrl.u_mxusbpd_pgdo_pu_ctrl.pgdo_pu_ddft_faults_masked[3:0]
 * 130:127 MXUSBPD_MMIO_INST.y_pgdo_pu_ctrl.u_mxusbpd_pgdo_pu_ctrl.pgdo_pu_ddft_async_fault_det[3:0]
 * 126:123 MXUSBPD_MMIO_INST.y_pgdo_pu_ctrl.u_mxusbpd_pgdo_pu_ctrl.pgdo_pu_ctrl_pgdo_in_lv[3:0]
 * 122:119 MXUSBPD_MMIO_INST.y_pgdo_pu_ctrl.u_mxusbpd_pgdo_pu_ctrl.pgdo_pu_ctrl_pgdo_en_lv[3:0]
 * 118:115 MXUSBPD_MMIO_INST.y_pgdo_ctrl.u_mxusbpd_pgdo_ctrl.pgdo_ddft_faults_masked[3:0]
 * 114:111 MXUSBPD_MMIO_INST.y_pgdo_ctrl.u_mxusbpd_pgdo_ctrl.pgdo_ddft_async_fault_det[3:0]
 * 110:107 MXUSBPD_MMIO_INST.y_pgdo_ctrl.u_mxusbpd_pgdo_ctrl.pgdo_ctrl_pgdo_en_lv[3:0]
 * 106:99  MXUSBPD_MMIO_INST.y_ngdo_ctrl.u_mxusbpd_ngdo_ctrl.ngdo_ctrl_ngdo_pulldn_en_lv[7:0]
 * 98:91   MXUSBPD_MMIO_INST.y_ngdo_ctrl.u_mxusbpd_ngdo_ctrl.ngdo_ctrl_ngdo_en_lv[7:0]
 * 90:87   MXUSBPD_HARD_TOP_INST.y_cc_5vpump.u_s8usbpd_5vpump_top.pump5v_clkout_ddft[3:0]
 * 86      MXUSBPD_HARD_TOP_INST.y_vsys.u_s8usbpd_vddd_sw_top.vsys_fx_scan
 * 85:84   MXUSBPD_HARD_TOP_INST.hs_filt2_fx_scan[1:0]
 * 83:60   MXUSBPD_HARD_TOP_INST.hs_filt_fx_scan[23:0]
 * 59:52   MXUSBPD_HARD_TOP_INST.ls_filt_fx_scan[7:0]
 * 51:44   MXUSBPD_HARD_TOP_INST.y_det_shv.u_s8usbpd_det_shv_top.det_shv_fx_scan[7:0]
 * 43:36   MXUSBPD_HARD_TOP_INST.y_bch_det.u_s8usbpd_bch_chgdet_top.bch_det_fx_scan[7:0]
 * 35:32   MXUSBPD_HARD_TOP_INST.y_sbu20.u_s8usbpd_sbu20_sw_top.sbu1_ovp_fx_scan[3:0]
 * 31:28   MXUSBPD_HARD_TOP_INST.y_sbu20.u_s8usbpd_sbu20_sw_top.sbu2_ovp_fx_scan[3:0]
 * 27:24   MXUSBPD_HARD_TOP_INST.qcom_rcvr_dp_fx_scan[3:0]
 * 23:20   MXUSBPD_HARD_TOP_INST.qcom_rcvr_dm_fx_scan[3:0]
 * 19:16   MXUSBPD_HARD_TOP_INST.y_shvreg.u_s8usbpd_vreg_top.shvreg_vbus_fx_scan[3:0]
 * 15:12   MXUSBPD_HARD_TOP_INST.y_20vreg.u_s8usbpd_vreg_top.vreg_vbus_fx_scan[3:0]
 * 11:8    MXUSBPD_HARD_TOP_INST.y_csa.u_s8usbpd_csa_top.csa_vbus_fx_scan[3:0]
 * 7:4     MXUSBPD_HARD_TOP_INST.y_chgdet.u_s8usbpd_chgdet_top.chgdet_fx_scan[3:0]
 * 3:0     MXUSBPD_HARD_TOP_INST.y_csa.u_s8usbpd_csa_top.csa_oc_fx_scan[3:0]
 */
#define PDSS_NCELL_DDFT_MUX_NCELL_DDFT1_SEL_MASK            (0x0000ff00) /* <8:15> R:RW:0: */
#define PDSS_NCELL_DDFT_MUX_NCELL_DDFT1_SEL_POS             (8)


/*
 * IP GPIO DDFT Selections
 */
#define PDSS_GPIO_DDFT_MUX_ADDRESS                          (0x400a058c)
#define PDSS_GPIO_DDFT_MUX                                  (*(volatile uint32_t *)(0x400a058c))
#define PDSS_GPIO_DDFT_MUX_DEFAULT                          (0x00000000)

/*
 * 117 MXUSBPD_MMIO_INST.y_pasc_gpio_ddft.u_mxusbpd_pasc_gpio_ddft1_mux.ddft_out
 * 116 MXUSBPD_MMIO_INST.y_pasc_gpio_ddft.u_mxusbpd_pasc_gpio_ddft0_mux.ddft_out
 * 115 MXUSBPD_MMIO_INST.y_cc_vbus_det.y_cc_vbus_det.u_intr_clk_filter_filter.out
 * 114   MXUSBPD_MMIO_INST.y_csa_scp_edge.u_csa_ocp_change.u_intr_clk_filter_filter.out
 * 113   MXUSBPD_MMIO_INST.y_csa_scp_edge.u_csa_scp_change.u_intr_clk_filter_filter.out
 * 112   MXUSBPD_MMIO_INST.y_csa_rcp_edge.u_csa_out_change.u_intr_clk_filter_filter.out
 * 111   MXUSBPD_MMIO_INST.y_csa_rcp_edge.u_csa_comp_out_change.u_intr_clk_filter_filter.out
 * 110   MXUSBPD_MMIO_INST.y_csa_rcp_edge.u_csa_vbus_ovp_change.u_intr_clk_filter_filter.out
 * 109:106 MXUSBPD_MMIO_INST.y_small_vconn.u_small_vconn_change.filt_out_gpio
 * 105     MXUSBPD_MMIO_INST.u_hs_filt22_change.u_intr_clk_filter_filter.out
 * 104     MXUSBPD_MMIO_INST.u_hs_filt21_change.u_intr_clk_filter_filter.out
 * 103:80  MXUSBPD_MMIO_INST.y_hs_edge_det.u_hs_change.u_intr_clk_filter_filter.out
 * 79:72   MXUSBPD_MMIO_INST.y_ls_edge.u_ls_change.u_intr_clk_lf_filter.out
 * 71:64   MXUSBPD_MMIO_INST.y_sbu20_ovp_edge.u_sbu2_ovp_change.u_intr_clk_filter_filter.out,MXUSBPD_MMIO_INST.y_sbu20_ovp_edge.u_sbu1_ovp_change.u_intr_clk_filter_filter.out
 * 63:56   MXUSBPD_MMIO_INST.y_bch_det_edge.u_bch_det_comp1_change.u_intr_clk_lf_filter.out,MXUSBPD_MMIO_INST.y_bch_det_edge.u_bch_det_comp0_change.u_intr_clk_lf_filter.out
 * 55:48   MXUSBPD_MMIO_INST.y_det_shv_edge.u_det_shv_change.u_intr_clk_lf_filter.out
 * 47:44   MXUSBPD_MMIO_INST.y_csa_edge.u_csa_vbus_change.u_intr_clk_lf_filter.out
 * 43:40   MXUSBPD_MMIO_INST.y_csa_edge.u_csa_oc_change.u_intr_clk_filter_filter.out
 * 39:36   MXUSBPD_MMIO_INST.y_bch_det_edge.u_qcom_rcvr_dm_change.u_intr_clk_lf_filter.out
 * 35:32   MXUSBPD_MMIO_INST.y_bch_det_edge.u_qcom_rcvr_dp_change.u_intr_clk_lf_filter.out
 * 31:28   MXUSBPD_MMIO_INST.y_chgdet_edge.u_chgdet_change.u_intr_clk_lf_filter.out
 * 27:24   MXUSBPD_MMIO_INST.y_shvreg_vbus_edge.u_shvreg_vbus_change.u_intr_clk_lf_filter.out
 * 23:20   MXUSBPD_MMIO_INST.y_vreg_vbus_edge.u_vreg_vbus_change.u_intr_clk_lf_filter.out
 * 19:16   MXUSBPD_MMIO_INST.y_adc.u_cmp_out_change.u_intr_clk_filter_filter.out
 * 15      MXUSBPD_MMIO_INST.y_vsys_edge.u_vsys_change.u_intr_clk_lf_filter.out
 * 14      MXUSBPD_MMIO_INST.u_hpd_change.u_intr_clk_lf_filter.out
 * 13      MXUSBPD_MMIO_INST.y_v5v_edge.u_v5v_change.u_intr_clk_lf_filter.out
 * 12      MXUSBPD_MMIO_INST.y_vconn2_edge.u_vconn2_change.u_intr_clk_lf_filter.out
 * 11      MXUSBPD_MMIO_INST.y_vconn1_edge.u_vconn1_change.u_intr_clk_lf_filter.out
 * 10      MXUSBPD_MMIO_INST.u_vcmp_up.u_intr_clk_lf_filter.out
 * 9       MXUSBPD_MMIO_INST.u_vcmp_dn.u_intr_clk_lf_filter.out
 * 8       MXUSBPD_MMIO_INST.u_vcmp_la.u_intr_clk_lf_filter.out
 * 7       MXUSBPD_MMIO_INST.y_vconn20_oxp_cc12_det.u_cc2_ocp_change.u_intr_clk_filter_filter.out
 * 6       MXUSBPD_MMIO_INST.y_vconn20_oxp_cc12_det.u_cc1_ocp_change.u_intr_clk_filter_filter.out
 * 5       MXUSBPD_MMIO_INST.y_vconn20_oxp_cc12_det.u_cc2_ovp_change.u_intr_clk_filter_filter.out
 * 4       MXUSBPD_MMIO_INST.y_vconn20_oxp_cc12_det.u_cc1_ovp_change.u_intr_clk_filter_filter.out
 * 3       MXUSBPD_MMIO_INST.u_cc2_change.u_intr_clk_lf_filter.out
 * 2       MXUSBPD_MMIO_INST.u_cc1_change.u_intr_clk_lf_filter.out
 * 1       gpio_intr_ddft1
 * 0       gpio_intr_ddft0
 */
#define PDSS_GPIO_DDFT_MUX_GPIO_DDFT0_SEL_MASK              (0x0000007f) /* <0:6> R:RW:0: */
#define PDSS_GPIO_DDFT_MUX_GPIO_DDFT0_SEL_POS               (0)


/*
 * 0: The selected output is not inverted
 * 1: The selected out is inverted
 */
#define PDSS_GPIO_DDFT_MUX_GPIO_DDFT0_POLARITY              (1u << 7) /* <7:7> R:RW:0: */


/*
 * 117 MXUSBPD_MMIO_INST.y_pasc_gpio_ddft.u_mxusbpd_pasc_gpio_ddft1_mux.ddft_out
 * 116 MXUSBPD_MMIO_INST.y_pasc_gpio_ddft.u_mxusbpd_pasc_gpio_ddft0_mux.ddft_out
 * 115 MXUSBPD_MMIO_INST.y_cc_vbus_det.y_cc_vbus_det.u_intr_clk_filter_filter.out
 * 114   MXUSBPD_MMIO_INST.y_csa_scp_edge.u_csa_ocp_change.u_intr_clk_filter_filter.out
 * 113   MXUSBPD_MMIO_INST.y_csa_scp_edge.u_csa_scp_change.u_intr_clk_filter_filter.out
 * 112   MXUSBPD_MMIO_INST.y_csa_rcp_edge.u_csa_out_change.u_intr_clk_filter_filter.out
 * 111   MXUSBPD_MMIO_INST.y_csa_rcp_edge.u_csa_comp_out_change.u_intr_clk_filter_filter.out
 * 110   MXUSBPD_MMIO_INST.y_csa_rcp_edge.u_csa_vbus_ovp_change.u_intr_clk_filter_filter.out
 * 109:106 MXUSBPD_MMIO_INST.y_small_vconn.u_small_vconn_change.filt_out_gpio
 * 105     MXUSBPD_MMIO_INST.u_hs_filt22_change.u_intr_clk_filter_filter.out
 * 104     MXUSBPD_MMIO_INST.u_hs_filt21_change.u_intr_clk_filter_filter.out
 * 103:80  MXUSBPD_MMIO_INST.y_hs_edge_det.u_hs_change.u_intr_clk_filter_filter.out
 * 79:72   MXUSBPD_MMIO_INST.y_ls_edge.u_ls_change.u_intr_clk_lf_filter.out
 * 71:64   MXUSBPD_MMIO_INST.y_sbu20_ovp_edge.u_sbu2_ovp_change.u_intr_clk_filter_filter.out,MXUSBPD_MMIO_INST.y_sbu20_ovp_edge.u_sbu1_ovp_change.u_intr_clk_filter_filter.out
 * 63:56   MXUSBPD_MMIO_INST.y_bch_det_edge.u_bch_det_comp1_change.u_intr_clk_lf_filter.out,MXUSBPD_MMIO_INST.y_bch_det_edge.u_bch_det_comp0_change.u_intr_clk_lf_filter.out
 * 55:48   MXUSBPD_MMIO_INST.y_det_shv_edge.u_det_shv_change.u_intr_clk_lf_filter.out
 * 47:44   MXUSBPD_MMIO_INST.y_csa_edge.u_csa_vbus_change.u_intr_clk_lf_filter.out
 * 43:40   MXUSBPD_MMIO_INST.y_csa_edge.u_csa_oc_change.u_intr_clk_filter_filter.out
 * 39:36   MXUSBPD_MMIO_INST.y_bch_det_edge.u_qcom_rcvr_dm_change.u_intr_clk_lf_filter.out
 * 35:32   MXUSBPD_MMIO_INST.y_bch_det_edge.u_qcom_rcvr_dp_change.u_intr_clk_lf_filter.out
 * 31:28   MXUSBPD_MMIO_INST.y_chgdet_edge.u_chgdet_change.u_intr_clk_lf_filter.out
 * 27:24   MXUSBPD_MMIO_INST.y_shvreg_vbus_edge.u_shvreg_vbus_change.u_intr_clk_lf_filter.out
 * 23:20   MXUSBPD_MMIO_INST.y_vreg_vbus_edge.u_vreg_vbus_change.u_intr_clk_lf_filter.out
 * 19:16   MXUSBPD_MMIO_INST.y_adc.u_cmp_out_change.u_intr_clk_filter_filter.out
 * 15      MXUSBPD_MMIO_INST.y_vsys_edge.u_vsys_change.u_intr_clk_lf_filter.out
 * 14      MXUSBPD_MMIO_INST.u_hpd_change.u_intr_clk_lf_filter.out
 * 13      MXUSBPD_MMIO_INST.y_v5v_edge.u_v5v_change.u_intr_clk_lf_filter.out
 * 12      MXUSBPD_MMIO_INST.y_vconn2_edge.u_vconn2_change.u_intr_clk_lf_filter.out
 * 11      MXUSBPD_MMIO_INST.y_vconn1_edge.u_vconn1_change.u_intr_clk_lf_filter.out
 * 10      MXUSBPD_MMIO_INST.u_vcmp_up.u_intr_clk_lf_filter.out
 * 9       MXUSBPD_MMIO_INST.u_vcmp_dn.u_intr_clk_lf_filter.out
 * 8       MXUSBPD_MMIO_INST.u_vcmp_la.u_intr_clk_lf_filter.out
 * 7       MXUSBPD_MMIO_INST.y_vconn20_oxp_cc12_det.u_cc2_ocp_change.u_intr_clk_filter_filter.out
 * 6       MXUSBPD_MMIO_INST.y_vconn20_oxp_cc12_det.u_cc1_ocp_change.u_intr_clk_filter_filter.out
 * 5       MXUSBPD_MMIO_INST.y_vconn20_oxp_cc12_det.u_cc2_ovp_change.u_intr_clk_filter_filter.out
 * 4       MXUSBPD_MMIO_INST.y_vconn20_oxp_cc12_det.u_cc1_ovp_change.u_intr_clk_filter_filter.out
 * 3       MXUSBPD_MMIO_INST.u_cc2_change.u_intr_clk_lf_filter.out
 * 2       MXUSBPD_MMIO_INST.u_cc1_change.u_intr_clk_lf_filter.out
 * 1       gpio_intr_ddft1
 * 0       gpio_intr_ddft0
 */
#define PDSS_GPIO_DDFT_MUX_GPIO_DDFT1_SEL_MASK              (0x00007f00) /* <8:14> R:RW:0: */
#define PDSS_GPIO_DDFT_MUX_GPIO_DDFT1_SEL_POS               (8)


/*
 * 0: The selected output is not inverted
 * 1: The selected out is inverted
 */
#define PDSS_GPIO_DDFT_MUX_GPIO_DDFT1_POLARITY              (1u << 15) /* <15:15> R:RW:0: */


/*
 * Interrupt GPIO DDFT Selections
 */
#define PDSS_GPIO_INTR_DDFT_MUX_ADDRESS                     (0x400a0590)
#define PDSS_GPIO_INTR_DDFT_MUX                             (*(volatile uint32_t *)(0x400a0590))
#define PDSS_GPIO_INTR_DDFT_MUX_DEFAULT                     (0x00000000)

/*
 * 247 MXUSBPD_REGS_INST.intr15_cause_pds_vreg_vbus_done
 * 246 MXUSBPD_REGS_INST.intr15_ea_cc_flag_changed_done
 * 245 MXUSBPD_REGS_INST.intr15_ff_ov_changed
 * 244 MXUSBPD_REGS_INST.intr15_ff_uv_changed
 * 243 MXUSBPD_REGS_INST.intr15_zcdf_out_changed
 * 242 MXUSBPD_REGS_INST.intr15_peakdet_out_changed
 * 241 MXUSBPD_REGS_INST.intr15_peakdet_rst_out_changed
 * 240 MXUSBPD_REGS_INST.intr15_peakdet_clcmp_raw_out_changed
 * 239 MXUSBPD_REGS_INST.intr15_sr_sen_ovp_out_changed
 * 238 MXUSBPD_REGS_INST.intr15_pwm_out_changed
 * 237 MXUSBPD_REGS_INST.intr15_skip_out_changed
 * 236 MXUSBPD_REGS_INST.intr15_burst_exit_out_changed
 * 235 MXUSBPD_REGS_INST.intr15_pds_scp_changed
 * 234 MXUSBPD_REGS_INST.intr15_nsn_out_changed
 * 233 MXUSBPD_REGS_INST.intr15_zcd_out_changed
 * 232 MXUSBPD_REGS_INST.intr8_pasc_idle
 * 231 MXUSBPD_REGS_INST.intr8_vbtr_exit_done
 * 230 MXUSBPD_REGS_INST.intr8_vbtr_opr_done
 * 229 MXUSBPD_REGS_INST.intr8_pasc_dcm_2_ccm_chg
 * 228 MXUSBPD_REGS_INST.intr8_pasc_ccm_2_dcm_chg
 * 227 MXUSBPD_REGS_INST.intr8_gdrv_less_than_turn_off
 * 226 MXUSBPD_REGS_INST.intr8_gdrv_input_width_less_than_min
 * 225 MXUSBPD_REGS_INST.intr8_cal_fail
 * 224 MXUSBPD_REGS_INST.intr8_gdrv_contentiion
 * 223 MXUSBPD_REGS_INST.intr8_gdrv_greater_than_turn_off
 * 222 MXUSBPD_REGS_INST.intr8_pasc_loop_cal_done
 * 221 MXUSBPD_REGS_INST.intr8_pasc_burst_entry
 * 220 MXUSBPD_REGS_INST.intr8_pasc_var_tmin_timeout
 * 219 MXUSBPD_REGS_INST.intr8_pasc_nsn_idle_timeout
 * 218 MXUSBPD_REGS_INST.intr8_pasc_ff_ov_idle_timeout
 * 217 MXUSBPD_REGS_INST.intr8_pasc_fix_freq_timeout
 * 216 MXUSBPD_REGS_INST.intr8_pasc_var_tmax_timeout
 * 215 MXUSBPD_REGS_INST.intr8_pasc_aud_tmin_timeout
 * 214 MXUSBPD_REGS_INST.intr8_pasc_aud_tmax_timeout
 * 213 MXUSBPD_REGS_INST.intr8_pasc_gdrv_in_max_width_timeout
 * 212 MXUSBPD_REGS_INST.intr8_pasc_max_width_timeout
 * 211 MXUSBPD_REGS_INST.intr1_lf_cntr_match
 * 210 MXUSBPD_REGS_INST.intr13_cause_cc_vbus_changed
 * 209 MXUSBPD_REGS_INST.intr13_set_csa_vbus_ovp_changed
 * 208 MXUSBPD_REGS_INST.intr13_set_csa_comp_out_changed
 * 207 MXUSBPD_REGS_INST.intr13_set_csa_out_changed
 * 206 MXUSBPD_REGS_INST.intr13_set_csa_scp_changed
 * 205 MXUSBPD_REGS_INST.intr13_set_csa_ocp_changed
 * 204:181 MXUSBPD_REGS_INST.intr5_set_edge_changed,
 * 180:173 MXUSBPD_REGS_INST.intr3_set_sbu1_sbu2_ovp_changed,
 * 172:165 MXUSBPD_REGS_INST.intr7_set_clk_lf_edge_changed,
 * 164:157 MXUSBPD_REGS_INST.intr9_set_det_shv_det_changed,
 * 156:149 MXUSBPD_REGS_INST.intr9_set_bch_det_changed,
 * 148:145 MXUSBPD_REGS_INST.intr3_set_csa_oc_changed,
 * 144:141 MXUSBPD_REGS_INST.intr3_set_chgdet_changed,
 * 140:137 MXUSBPD_REGS_INST.intr3_set_vreg20_vbus_changed,
 * 136:133 MXUSBPD_REGS_INST.intr3_set_csa_vbus_changed,
 * 132:129 MXUSBPD_REGS_INST.intr9_set_qcom_rcvr_dm_changed,
 * 128:125 MXUSBPD_REGS_INST.intr9_set_qcom_rcvr_dp_changed,
 * 124:121 MXUSBPD_REGS_INST.intr9_set_shvreg_det_changed,
 * 120:117 MXUSBPD_REGS_INST.intr4_set_afc_ping_recvd,
 * 116:113 MXUSBPD_REGS_INST.intr4_set_afc_sm_idle,
 * 112:109 MXUSBPD_REGS_INST.intr4_set_afc_timeout,
 * 108:105 MXUSBPD_REGS_INST.intr4_set_afc_rx_reset,
 * 104:101 MXUSBPD_REGS_INST.intr4_set_update_ping_pong,
 * 100:97  MXUSBPD_REGS_INST.intr4_set_afc_error,
 * 96:93   MXUSBPD_REGS_INST.intr6_set_qc_3_d_p_pulse_rcvd,
 * 92:89   MXUSBPD_REGS_INST.intr6_set_qc_3_d_m_pulse_rcvd,
 * 88:85   MXUSBPD_REGS_INST.intr6_set_qc_3_device_req_sent,
 * 84:83   MXUSBPD_REGS_INST.intr11_set_filt2_edge_changed,
 * 82:81   MXUSBPD_REGS_INST.intr1_set_ngdo_spacing_done[3:2],
 * 80      MXUSBPD_REGS_INST.intr3_set_vsys_changed,
 * 79      MXUSBPD_REGS_INST.intr1_set_cc2_ocp_changed,
 * 78      MXUSBPD_REGS_INST.intr1_set_cc1_ocp_changed,
 * 77      MXUSBPD_REGS_INST.intr1_set_cc2_ovp_changed,
 * 76      MXUSBPD_REGS_INST.intr1_set_cc1_ovp_changed,
 * 75      MXUSBPD_REGS_INST.intr1_set_drp_attached_detected,
 * 74      MXUSBPD_REGS_INST.intr3_set_cmp_out_changed[3],
 * 73      MXUSBPD_REGS_INST.intr3_set_cmp_out_changed[2],
 * 72      MXUSBPD_REGS_INST.intr0_set_sar_done[3],
 * 71      MXUSBPD_REGS_INST.intr0_set_sar_done[2],
 * 70      MXUSBPD_REGS_INST.intr1_set_vswap_vbus_less_5_done,
 * 69      MXUSBPD_REGS_INST.intr2_set_swap_command_done,
 * 68      1'b0
 * 67      1'b0
 * 66      MXUSBPD_REGS_INST.intr2_set_swap_disconnect,
 * 65      MXUSBPD_REGS_INST.intr2_set_swap_rcvd,
 * 64      MXUSBPD_REGS_INST.intr2_set_swap_pulse,
 * 63      MXUSBPD_REGS_INST.intr1_set_ngdo_spacing_done[0],
 * 62      MXUSBPD_REGS_INST.intr1_set_ngdo_spacing_done[1],
 * 61      MXUSBPD_REGS_INST.intr2_set_vreg20v_switch_done,
 * 60      MXUSBPD_REGS_INST.intr2_set_vddd_sw_switch_done,
 * 59      MXUSBPD_REGS_INST.intr2_set_chunk_det,
 * 58      MXUSBPD_REGS_INST.intr2_set_tx_sram_under_flow,
 * 57      MXUSBPD_REGS_INST.intr2_set_rx_sram_over_flow,
 * 56      1'b0
 * 55      1'b0
 * 54      1'b0
 * 53      MXUSBPD_REGS_INST.intr2_set_extended_msg_det,
 * 52      MXUSBPD_REGS_INST.intr2_set_hpdt_command_done,
 * 51      MXUSBPD_REGS_INST.intr2_set_hpd_queue,
 * 50      MXUSBPD_REGS_INST.intr2_set_hpd_unstable,
 * 49      MXUSBPD_REGS_INST.intr2_set_hpd_unpluged,
 * 48      MXUSBPD_REGS_INST.intr2_set_hpd_pluged,
 * 47      MXUSBPD_REGS_INST.intr2_set_hpd_irq,
 * 46      MXUSBPD_REGS_INST.intr2_set_ui_cal_done,
 * 45      1'b0
 * 44      1'b0
 * 43      1'b0
 * 42      1'b0
 * 41      MXUSBPD_REGS_INST.intr1_set_hpdin_changed,
 * 40      MXUSBPD_REGS_INST.intr3_set_cmp_out_changed[1],
 * 39      MXUSBPD_REGS_INST.intr3_set_cmp_out_changed[0],
 * 38      MXUSBPD_REGS_INST.intr1_set_v5v_changed,
 * 37      MXUSBPD_REGS_INST.intr1_set_vcmp_dn_changed,
 * 36      MXUSBPD_REGS_INST.intr1_set_vcmp_up_changed,
 * 35      MXUSBPD_REGS_INST.intr1_set_vcmp_la_changed,
 * 34      MXUSBPD_REGS_INST.intr1_set_cc2_changed,
 * 33      MXUSBPD_REGS_INST.intr1_set_cc1_changed,
 * 32      MXUSBPD_REGS_INST.intr1_set_vconn2_changed,
 * 31      MXUSBPD_REGS_INST.intr1_set_vconn1_changed,
 * 30      1'b0
 * 29      MXUSBPD_REGS_INST.intr0_set_sar_done[1],
 * 28      MXUSBPD_REGS_INST.intr0_set_rx_state_idle,
 * 27      MXUSBPD_REGS_INST.intr0_set_tx_state_idle,
 * 26      MXUSBPD_REGS_INST.intr0_set_tx_regulator_enabled,
 * 25      MXUSBPD_REGS_INST.intr0_set_tx_cc_data_oen_deassert,
 * 24      MXUSBPD_REGS_INST.intr0_set_tx_cc_data_oen_assert,
 * 23      MXUSBPD_REGS_INST.intr0_set_kchar_error,
 * 22      MXUSBPD_REGS_INST.intr0_set_tx_retry_enable_clrd,
 * 21      MXUSBPD_REGS_INST.intr0_set_rx_sram_half_end,
 * 20      MXUSBPD_REGS_INST.intr0_set_tx_sram_half_end,
 * 19      1'b0
 * 18      MXUSBPD_REGS_INST.MXUSBPD_REGS_INST.intr0_set_collision_type4,
 * 17      MXUSBPD_REGS_INST.MXUSBPD_REGS_INST.intr0_set_collision_type3,
 * 16      MXUSBPD_REGS_INST.MXUSBPD_REGS_INST.intr0_set_collision_type2,
 * 15      MXUSBPD_REGS_INST.MXUSBPD_REGS_INST.intr0_set_collision_type1
 * 14      MXUSBPD_REGS_INST.intr0_set_crc_rx_timer_exp,
 * 13      MXUSBPD_REGS_INST.intr0_set_cc_no_valid_data_detected,
 * 12      MXUSBPD_REGS_INST.intr0_set_cc_valid_data_detected,
 * 11      MXUSBPD_REGS_INST.intr0_set_tx_goodcrc_msg_done,
 * 10      MXUSBPD_REGS_INST.intr0_set_sar_done[0],
 * 9       MXUSBPD_REGS_INST.intr0_set_rcv_rst,
 * 8       MXUSBPD_REGS_INST.intr0_set_tx_hard_rst_done,
 * 7       MXUSBPD_REGS_INST.intr0_set_tx_packet_done,
 * 6       MXUSBPD_REGS_INST.intr0_set_rx_over_run,
 * 5       MXUSBPD_REGS_INST.intr0_set_eop_error,
 * 4       MXUSBPD_REGS_INST.intr0_set_rcv_expt_goodcrc_msg_complete,
 * 3       MXUSBPD_REGS_INST.intr0_set_rcv_goodcrc_msg_complete,
 * 2       MXUSBPD_REGS_INST.intr0_set_rx_sop,
 * 1       MXUSBPD_REGS_INST.intr0_set_rcv_bad_packet_complete,
 * 0       MXUSBPD_REGS_INST.intr0_set_rcv_good_packet_complete,
 */
#define PDSS_GPIO_INTR_DDFT_MUX_GPIO_INTR_DDFT0_SEL_MASK    (0x000000ff) /* <0:7> R:RW:0: */
#define PDSS_GPIO_INTR_DDFT_MUX_GPIO_INTR_DDFT0_SEL_POS     (0)


/*
 * 247 MXUSBPD_REGS_INST.intr15_cause_pds_vreg_vbus_done
 * 246 MXUSBPD_REGS_INST.intr15_ea_cc_flag_changed_done
 * 245 MXUSBPD_REGS_INST.intr15_ff_ov_changed
 * 244 MXUSBPD_REGS_INST.intr15_ff_uv_changed
 * 243 MXUSBPD_REGS_INST.intr15_zcdf_out_changed
 * 242 MXUSBPD_REGS_INST.intr15_peakdet_out_changed
 * 241 MXUSBPD_REGS_INST.intr15_peakdet_rst_out_changed
 * 240 MXUSBPD_REGS_INST.intr15_peakdet_clcmp_raw_out_changed
 * 239 MXUSBPD_REGS_INST.intr15_sr_sen_ovp_out_changed
 * 238 MXUSBPD_REGS_INST.intr15_pwm_out_changed
 * 237 MXUSBPD_REGS_INST.intr15_skip_out_changed
 * 236 MXUSBPD_REGS_INST.intr15_burst_exit_out_changed
 * 235 MXUSBPD_REGS_INST.intr15_pds_scp_changed
 * 234 MXUSBPD_REGS_INST.intr15_nsn_out_changed
 * 233 MXUSBPD_REGS_INST.intr15_zcd_out_changed
 * 232 MXUSBPD_REGS_INST.intr8_pasc_idle
 * 231 MXUSBPD_REGS_INST.intr8_vbtr_exit_done
 * 230 MXUSBPD_REGS_INST.intr8_vbtr_opr_done
 * 229 MXUSBPD_REGS_INST.intr8_pasc_dcm_2_ccm_chg
 * 228 MXUSBPD_REGS_INST.intr8_pasc_ccm_2_dcm_chg
 * 227 MXUSBPD_REGS_INST.intr8_gdrv_less_than_turn_off
 * 226 MXUSBPD_REGS_INST.intr8_gdrv_input_width_less_than_min
 * 225 MXUSBPD_REGS_INST.intr8_cal_fail
 * 224 MXUSBPD_REGS_INST.intr8_gdrv_contentiion
 * 223 MXUSBPD_REGS_INST.intr8_gdrv_greater_than_turn_off
 * 222 MXUSBPD_REGS_INST.intr8_pasc_loop_cal_done
 * 221 MXUSBPD_REGS_INST.intr8_pasc_burst_entry
 * 220 MXUSBPD_REGS_INST.intr8_pasc_var_tmin_timeout
 * 219 MXUSBPD_REGS_INST.intr8_pasc_nsn_idle_timeout
 * 218 MXUSBPD_REGS_INST.intr8_pasc_ff_ov_idle_timeout
 * 217 MXUSBPD_REGS_INST.intr8_pasc_fix_freq_timeout
 * 216 MXUSBPD_REGS_INST.intr8_pasc_var_tmax_timeout
 * 215 MXUSBPD_REGS_INST.intr8_pasc_aud_tmin_timeout
 * 214 MXUSBPD_REGS_INST.intr8_pasc_aud_tmax_timeout
 * 213 MXUSBPD_REGS_INST.intr8_pasc_gdrv_in_max_width_timeout
 * 212 MXUSBPD_REGS_INST.intr8_pasc_max_width_timeout
 * 211 MXUSBPD_REGS_INST.intr1_lf_cntr_match
 * 210 MXUSBPD_REGS_INST.intr13_cause_cc_vbus_changed
 * 209 MXUSBPD_REGS_INST.intr13_set_csa_vbus_ovp_changed
 * 208 MXUSBPD_REGS_INST.intr13_set_csa_comp_out_changed
 * 207 MXUSBPD_REGS_INST.intr13_set_csa_out_changed
 * 206 MXUSBPD_REGS_INST.intr13_set_csa_scp_changed
 * 205 MXUSBPD_REGS_INST.intr13_set_csa_ocp_changed
 * 204:181 MXUSBPD_REGS_INST.intr5_set_edge_changed,
 * 180:173 MXUSBPD_REGS_INST.intr3_set_sbu1_sbu2_ovp_changed,
 * 172:165 MXUSBPD_REGS_INST.intr7_set_clk_lf_edge_changed,
 * 164:157 MXUSBPD_REGS_INST.intr9_set_det_shv_det_changed,
 * 156:149 MXUSBPD_REGS_INST.intr9_set_bch_det_changed,
 * 148:145 MXUSBPD_REGS_INST.intr3_set_csa_oc_changed,
 * 144:141 MXUSBPD_REGS_INST.intr3_set_chgdet_changed,
 * 140:137 MXUSBPD_REGS_INST.intr3_set_vreg20_vbus_changed,
 * 136:133 MXUSBPD_REGS_INST.intr3_set_csa_vbus_changed,
 * 132:129 MXUSBPD_REGS_INST.intr9_set_qcom_rcvr_dm_changed,
 * 128:125 MXUSBPD_REGS_INST.intr9_set_qcom_rcvr_dp_changed,
 * 124:121 MXUSBPD_REGS_INST.intr9_set_shvreg_det_changed,
 * 120:117 MXUSBPD_REGS_INST.intr4_set_afc_ping_recvd,
 * 116:113 MXUSBPD_REGS_INST.intr4_set_afc_sm_idle,
 * 112:109 MXUSBPD_REGS_INST.intr4_set_afc_timeout,
 * 108:105 MXUSBPD_REGS_INST.intr4_set_afc_rx_reset,
 * 104:101 MXUSBPD_REGS_INST.intr4_set_update_ping_pong,
 * 100:97  MXUSBPD_REGS_INST.intr4_set_afc_error,
 * 96:93   MXUSBPD_REGS_INST.intr6_set_qc_3_d_p_pulse_rcvd,
 * 92:89   MXUSBPD_REGS_INST.intr6_set_qc_3_d_m_pulse_rcvd,
 * 88:85   MXUSBPD_REGS_INST.intr6_set_qc_3_device_req_sent,
 * 84:83   MXUSBPD_REGS_INST.intr11_set_filt2_edge_changed,
 * 82:81   MXUSBPD_REGS_INST.intr1_set_ngdo_spacing_done[3:2],
 * 80      MXUSBPD_REGS_INST.intr3_set_vsys_changed,
 * 79      MXUSBPD_REGS_INST.intr1_set_cc2_ocp_changed,
 * 78      MXUSBPD_REGS_INST.intr1_set_cc1_ocp_changed,
 * 77      MXUSBPD_REGS_INST.intr1_set_cc2_ovp_changed,
 * 76      MXUSBPD_REGS_INST.intr1_set_cc1_ovp_changed,
 * 75      MXUSBPD_REGS_INST.intr1_set_drp_attached_detected,
 * 74      MXUSBPD_REGS_INST.intr3_set_cmp_out_changed[3],
 * 73      MXUSBPD_REGS_INST.intr3_set_cmp_out_changed[2],
 * 72      MXUSBPD_REGS_INST.intr0_set_sar_done[3],
 * 71      MXUSBPD_REGS_INST.intr0_set_sar_done[2],
 * 70      MXUSBPD_REGS_INST.intr1_set_vswap_vbus_less_5_done,
 * 69      MXUSBPD_REGS_INST.intr2_set_swap_command_done,
 * 68      1'b0
 * 67      1'b0
 * 66      MXUSBPD_REGS_INST.intr2_set_swap_disconnect,
 * 65      MXUSBPD_REGS_INST.intr2_set_swap_rcvd,
 * 64      MXUSBPD_REGS_INST.intr2_set_swap_pulse,
 * 63      MXUSBPD_REGS_INST.intr1_set_ngdo_spacing_done[0],
 * 62      MXUSBPD_REGS_INST.intr1_set_ngdo_spacing_done[1],
 * 61      MXUSBPD_REGS_INST.intr2_set_vreg20v_switch_done,
 * 60      MXUSBPD_REGS_INST.intr2_set_vddd_sw_switch_done,
 * 59      MXUSBPD_REGS_INST.intr2_set_chunk_det,
 * 58      MXUSBPD_REGS_INST.intr2_set_tx_sram_under_flow,
 * 57      MXUSBPD_REGS_INST.intr2_set_rx_sram_over_flow,
 * 56      1'b0
 * 55      1'b0
 * 54      1'b0
 * 53      MXUSBPD_REGS_INST.intr2_set_extended_msg_det,
 * 52      MXUSBPD_REGS_INST.intr2_set_hpdt_command_done,
 * 51      MXUSBPD_REGS_INST.intr2_set_hpd_queue,
 * 50      MXUSBPD_REGS_INST.intr2_set_hpd_unstable,
 * 49      MXUSBPD_REGS_INST.intr2_set_hpd_unpluged,
 * 48      MXUSBPD_REGS_INST.intr2_set_hpd_pluged,
 * 47      MXUSBPD_REGS_INST.intr2_set_hpd_irq,
 * 46      MXUSBPD_REGS_INST.intr2_set_ui_cal_done,
 * 45      1'b0
 * 44      1'b0
 * 43      1'b0
 * 42      1'b0
 * 41      MXUSBPD_REGS_INST.intr1_set_hpdin_changed,
 * 40      MXUSBPD_REGS_INST.intr3_set_cmp_out_changed[1],
 * 39      MXUSBPD_REGS_INST.intr3_set_cmp_out_changed[0],
 * 38      MXUSBPD_REGS_INST.intr1_set_v5v_changed,
 * 37      MXUSBPD_REGS_INST.intr1_set_vcmp_dn_changed,
 * 36      MXUSBPD_REGS_INST.intr1_set_vcmp_up_changed,
 * 35      MXUSBPD_REGS_INST.intr1_set_vcmp_la_changed,
 * 34      MXUSBPD_REGS_INST.intr1_set_cc2_changed,
 * 33      MXUSBPD_REGS_INST.intr1_set_cc1_changed,
 * 32      MXUSBPD_REGS_INST.intr1_set_vconn2_changed,
 * 31      MXUSBPD_REGS_INST.intr1_set_vconn1_changed,
 * 30      1'b0
 * 29      MXUSBPD_REGS_INST.intr0_set_sar_done[1],
 * 28      MXUSBPD_REGS_INST.intr0_set_rx_state_idle,
 * 27      MXUSBPD_REGS_INST.intr0_set_tx_state_idle,
 * 26      MXUSBPD_REGS_INST.intr0_set_tx_regulator_enabled,
 * 25      MXUSBPD_REGS_INST.intr0_set_tx_cc_data_oen_deassert,
 * 24      MXUSBPD_REGS_INST.intr0_set_tx_cc_data_oen_assert,
 * 23      MXUSBPD_REGS_INST.intr0_set_kchar_error,
 * 22      MXUSBPD_REGS_INST.intr0_set_tx_retry_enable_clrd,
 * 21      MXUSBPD_REGS_INST.intr0_set_rx_sram_half_end,
 * 20      MXUSBPD_REGS_INST.intr0_set_tx_sram_half_end,
 * 19      1'b0
 * 18      MXUSBPD_REGS_INST.MXUSBPD_REGS_INST.intr0_set_collision_type4,
 * 17      MXUSBPD_REGS_INST.MXUSBPD_REGS_INST.intr0_set_collision_type3,
 * 16      MXUSBPD_REGS_INST.MXUSBPD_REGS_INST.intr0_set_collision_type2,
 * 15      MXUSBPD_REGS_INST.MXUSBPD_REGS_INST.intr0_set_collision_type1
 * 14      MXUSBPD_REGS_INST.intr0_set_crc_rx_timer_exp,
 * 13      MXUSBPD_REGS_INST.intr0_set_cc_no_valid_data_detected,
 * 12      MXUSBPD_REGS_INST.intr0_set_cc_valid_data_detected,
 * 11      MXUSBPD_REGS_INST.intr0_set_tx_goodcrc_msg_done,
 * 10      MXUSBPD_REGS_INST.intr0_set_sar_done[0],
 * 9       MXUSBPD_REGS_INST.intr0_set_rcv_rst,
 * 8       MXUSBPD_REGS_INST.intr0_set_tx_hard_rst_done,
 * 7       MXUSBPD_REGS_INST.intr0_set_tx_packet_done,
 * 6       MXUSBPD_REGS_INST.intr0_set_rx_over_run,
 * 5       MXUSBPD_REGS_INST.intr0_set_eop_error,
 * 4       MXUSBPD_REGS_INST.intr0_set_rcv_expt_goodcrc_msg_complete,
 * 3       MXUSBPD_REGS_INST.intr0_set_rcv_goodcrc_msg_complete,
 * 2       MXUSBPD_REGS_INST.intr0_set_rx_sop,
 * 1       MXUSBPD_REGS_INST.intr0_set_rcv_bad_packet_complete,
 * 0       MXUSBPD_REGS_INST.intr0_set_rcv_good_packet_complete,
 */
#define PDSS_GPIO_INTR_DDFT_MUX_GPIO_INTR_DDFT1_SEL_MASK    (0x0000ff00) /* <8:15> R:RW:0: */
#define PDSS_GPIO_INTR_DDFT_MUX_GPIO_INTR_DDFT1_SEL_POS     (8)


/*
 * The Fault GPIO control
 */
#define PDSS_FAULT_GPIO_CTRL_ADDRESS                        (0x400a0594)
#define PDSS_FAULT_GPIO_CTRL                                (*(volatile uint32_t *)(0x400a0594))
#define PDSS_FAULT_GPIO_CTRL_DEFAULT                        (0x00000000)

/*
 * 1: Fault is registered on occcurence until cleared(FAULT_CLEAR) by CPU
 * 0: Fault is transparent to External GPIO
 */
#define PDSS_FAULT_GPIO_CTRL_FAULT_EN                       (1u << 0) /* <0:0> R:RW:0: */


/*
 * 1: Clear the registered fault
 * 0: Normal function
 * This functionality is available only the OR option , i.e FAULT_GPIO_*_SEL
 * is 0
 */
#define PDSS_FAULT_GPIO_CTRL_FAULT_CLEAR                    (1u << 1) /* <1:1> R:RW:0: */


/*
 * CCG3PA:
 * 0: OR of 1-4 masked with FAULT_MASK register
 * 1: UV
 * 2: OV
 * 3: OCP
 * 4: SCP
 *
 * CCG5-Port0/1:
 * 0: OR of 1-7 masked with FAULT_MASK register
 * 1: UV
 * 2: OV
 * 3: CSA-OCP
 * 4: CC1-OVP
 * 5: CC2-OVP
 * 6: CC1-OCP
 * 7: CC2-OCP
 *
 * PAG1S:
 * 0: OR of 1-6 masked with FAULT_MASK register
 * 1: UV
 * 2: OV
 * 3: OCP
 * 4: SCP
 * 5: FF_OV
 * 6: FF_UV
 */
#define PDSS_FAULT_GPIO_CTRL_FAULT_GPIO_0_SEL_MASK          (0x0000001c) /* <2:4> R:RW:0: */
#define PDSS_FAULT_GPIO_CTRL_FAULT_GPIO_0_SEL_POS           (2)


/*
 * 0: The selected output is not inverted
 * 1: The selected out is inverted
 */
#define PDSS_FAULT_GPIO_CTRL_FAULT_GPIO_0_POLARITY          (1u << 5) /* <5:5> R:RW:0: */


/*
 * CCG3PA:
 * 0: OR of 1-4 masked with FAULT_MASK register
 * 1: UV
 * 2: OV
 * 3: OCP
 * 4: SCP
 *
 * CCG5-Port0/1:
 * 0: OR of 1-7 masked with FAULT_MASK register
 * 1: UV
 * 2: OV
 * 3: CSA-OCP
 * 4: CC1-OVP
 * 5: CC2-OVP
 * 6: CC1-OCP
 * 7: CC2-OCP
 *
 * PAG1S:
 * 0: OR of 1-6 masked with FAULT_MASK register
 * 1: UV
 * 2: OV
 * 3: OCP
 * 4: SCP
 * 5: FF_OV
 * 6: FF_UV
 */
#define PDSS_FAULT_GPIO_CTRL_FAULT_GPIO_1_SEL_MASK          (0x000001c0) /* <6:8> R:RW:0: */
#define PDSS_FAULT_GPIO_CTRL_FAULT_GPIO_1_SEL_POS           (6)


/*
 * 0: The selected output is not inverted
 * 1: The selected out is inverted
 */
#define PDSS_FAULT_GPIO_CTRL_FAULT_GPIO_1_POLARITY          (1u << 9) /* <9:9> R:RW:0: */


/*
 * Individual mask bit for FAULT_GPIO_0_SEL, FAULT_GPIO_1_SEL
 */
#define PDSS_FAULT_GPIO_CTRL_FAULT_MASK_MASK                (0x0001fc00) /* <10:16> R:RW:0: */
#define PDSS_FAULT_GPIO_CTRL_FAULT_MASK_POS                 (10)


/*
 * USBPD Hard IP C-connector Control Register 0
 */
#define PDSS_CC_CTRL_0_ADDRESS                              (0x400a0600)
#define PDSS_CC_CTRL_0                                      (*(volatile uint32_t *)(0x400a0600))
#define PDSS_CC_CTRL_0_DEFAULT                              (0xb0000000)

/*
 * FW can only use this bit when the DEBUG_CC_0.TX_CC_DRIVE_SRC is set to
 * "1".
 * 0: Disables the Transceiver to transmit data
 * 1: Enables the Transceiver to transmit data
 * Notes: This bit is not needed for normal data transmit . This is reqiired
 * only for manual data tranmit .
 *
 */
#define PDSS_CC_CTRL_0_TX_EN                                (1u << 0) /* <0:0> R:RW:0: */


/*
 * Enables the Transceiver to receive data, Active High
 * This bit should be set after CC Line active interrupt (wakeup) in DeepSleep.
 * FW should set this bit at init and not change after across deep sleep
 * and wake.
 * Notes: DFP_unatached : 1'b0
 *            DFP attached    :  1'b1
 *            UFP , Cable       : 1'b1  (It can se set to 1 from beginning
 * )
 */
#define PDSS_CC_CTRL_0_RX_EN                                (1u << 1) /* <1:1> R:RW:0: */


/*
 * Firmware detects cable attach and specifies whether CC1 or CC2 is connected
 * to the CC-line of the cable.
 * 0 - CC1
 * 1 - CC2
 * Notes:
 * DFP: 0 or 1 based on the plug orientation .
 * UFP: 0 or 1 based on the plug orientation.
 * AMA: 0 or 1 based on board , which ever line is being used for communication.
 * Cable: 0  (Only CC1 line is used)
 * As a SOURCE, when VCONN is presented on a CC pin, all of the *CC1V2 registers
 * must be set to point to the CC pin connecting to the cable's CC line.
 *  This is required to prevent the VCONN voltage from affecting the comparator
 * thresholds on the active CC line.  This includes the FRS register: CC_CTRL_1.CMP_FS_CC1V2
 *
 */
#define PDSS_CC_CTRL_0_CC_1V2                               (1u << 2) /* <2:2> R:RW:0: */


/*
 * CC line voltage Comparator enable
 * Notes: Set this bit as need basis . No need to set for DFP unattachd .
 */
#define PDSS_CC_CTRL_0_CMP_EN                               (1u << 3) /* <3:3> R:RW:0: */


/*
 * Connects cmp_dn comparator to CC1/CC2
 * 0 - CC1
 * 1 - CC2
 * See Note from CC_1V2 register.
 */
#define PDSS_CC_CTRL_0_CMP_DN_CC1V2                         (1u << 4) /* <4:4> R:RW:0: */


/*
 * Selects the voltage threshold for cmp_dn comparator
 * Notes: Pair of DN and UP comprator should be used to determine remote
 * Rp value when acting as UFP. Firmware can set DN comparator to 0.655V
 * in UFP mode. In DFP mode firmware can use this comparator to detect detach
 * by selecting 2.6V reference .
 */
#define PDSS_CC_CTRL_0_CMP_DN_VSEL_MASK                     (0x000000e0) /* <5:7> R:RW:0: */
#define PDSS_CC_CTRL_0_CMP_DN_VSEL_POS                      (5)


/*
 * Connects cmp_up comparator to CC1/CC2
 * 0 - CC1
 * 1 - CC2
 * See Note from CC_1V2 register.
 */
#define PDSS_CC_CTRL_0_CMP_UP_CC1V2                         (1u << 8) /* <8:8> R:RW:0: */


/*
 * Selects the voltage threshold for cmp_up comparator.
 * Notes: Pair of DN and UP comprator should be used to determine remote
 * Rp value when acting as UFP. Firmware can set UP comparator to 1.235V
 * in UFP mode. In DFP mode firmware can use this comparator to detect detach
 * by selecting 2.6V reference .
 */
#define PDSS_CC_CTRL_0_CMP_UP_VSEL_MASK                     (0x00000e00) /* <9:11> R:RW:0: */
#define PDSS_CC_CTRL_0_CMP_UP_VSEL_POS                      (9)


/*
 * SPARE
 * Used in CCG3PA
 * Bit 0: Uses as FAILSAFE_EN for the discharge protection on VBUS_P
 * Bit 1: Used as FAILSAFE_EN for the discharge protection on VBUS_C
 * Bit 2: Used to invert the SWAP source selected (CDT#281193)
 */
#define PDSS_CC_CTRL_0_CMP_UP_OFFSET_MASK                   (0x00007000) /* <12:14> R:RW:0: */
#define PDSS_CC_CTRL_0_CMP_UP_OFFSET_POS                    (12)


/*
 * SPARE
 */
#define PDSS_CC_CTRL_0_CMP_UP_OFFSET_EN                     (1u << 15) /* <15:15> R:RW:0: */


/*
 * SPARE
 */
#define PDSS_CC_CTRL_0_CMP_LA_CC1V2                         (1u << 16) /* <16:16> R:RW:0: */


/*
 * SPARE
 * Used in CCG3PA
 * Bit 0: Used to enable PWM mode of discharge enable on VBUS_P
 * Bit 1: Used to enable PWM mode of discharge enable on VBUS_C
 */
#define PDSS_CC_CTRL_0_CMP_LA_VSEL_MASK                     (0x000e0000) /* <17:19> R:RW:0: */
#define PDSS_CC_CTRL_0_CMP_LA_VSEL_POS                      (17)


/*
 * Disable Dead Battery Rd termination on CC1
 * Notes:
 * DFP  (Unattached and attached) : 1'b1
 * UFP_dead : 1'b0
 * UFP_attached: 1'b1
 * Cable: Don't care
 */
#define PDSS_CC_CTRL_0_RD_CC1_DB_DIS                        (1u << 20) /* <20:20> R:RW:0: */


/*
 * Disable Dead Battery Rd termination on CC2
 * Notes:
 * DFP  (Unattached and attached) : 1'b1
 * UFP_dead : 1'b0
 * UFP_attached: 1'b1
 * Cable: Don't care
 */
#define PDSS_CC_CTRL_0_RD_CC2_DB_DIS                        (1u << 21) /* <21:21> R:RW:0: */


/*
 * If RP_RD_CFG.HW_RP_RD_AUTO_TOGGLE == 0, then FW can use this register
 * 0: Disable CC1 Trimmed Rd Termination
 * 1: Enable CC1 Trimmed Rd Termination
 * Notes:
 * DFP_UA , DFP_A : 0
 * UFP_DEAD: 0
 * UFP_A: 1
 * Cable: 0
 */
#define PDSS_CC_CTRL_0_RD_CC1_EN                            (1u << 22) /* <22:22> R:RW:0: */


/*
 * If RP_RD_CFG.HW_RP_RD_AUTO_TOGGLE == 0, then FW can use this register
 * 0: Disable CC2 Trimmed Rd Termination
 * 1: Enable CC2 Trimmed Rd Termination
 * Notes:
 * DFP_UA , DFP_A : 0
 * UFP_DEAD: 0
 * UFP_A: 1
 * Cable: 0
 * -------------
 * DFP_UA: DFP unattached
 * DFP_A: DFP attached
 * UFP_A: UFP attached
 */
#define PDSS_CC_CTRL_0_RD_CC2_EN                            (1u << 23) /* <23:23> R:RW:0: */


/*
 * If RP_RD_CFG.HW_RP_RD_AUTO_TOGGLE == 0, then FW can use this register
 * 0: Disable CC1 Pull-up Termination (Rp)
 * 1: Enable CC1 Pull-up Termination
 * Notes:
 * DFP_UA  = 0
 * DFP_A : 1  (This bit must be set to 1'b0 if CC1 is not used for communication)
 * UFP_DEAD: 0
 * UFP_A: 0
 * Cable: 0
 */
#define PDSS_CC_CTRL_0_RP_CC1_EN                            (1u << 24) /* <24:24> R:RW:0: */


/*
 * If RP_RD_CFG.HW_RP_RD_AUTO_TOGGLE == 0, then FW can use this register
 * 0: Disable CC2 Pull-up Termination (Rp)
 * 1: Enable CC2 Pull-up Termination
 * Notes:
 * DFP_UA  = 0
 * DFP_A : 1 ( (This bit must be set to 1'b0 if CC2 is not used for communication)
 * UFP_DEAD: 0
 * UFP_A: 0
 * Cable: 0
 */
#define PDSS_CC_CTRL_0_RP_CC2_EN                            (1u << 25) /* <25:25> R:RW:0: */


/*
 * Selects the Pull-up Termination current value
 * 0, 2 - 80uA (Default current broadcast)
 * 1 - 180uA (1.5A current broadcast)
 * 3 - 330uA (3.0A current broadcast)
 * Notes: This field matters only when RP_CC1/2_EN is set.
 */
#define PDSS_CC_CTRL_0_RP_MODE_MASK                         (0x0c000000) /* <26:27> R:RW:0: */
#define PDSS_CC_CTRL_0_RP_MODE_POS                          (26)


/*
 * SPARE
 */
#define PDSS_CC_CTRL_0_EN_HYST                              (1u << 28) /* <28:28> R:RW:1: */


/*
 * SPARE
 */
#define PDSS_CC_CTRL_0_HYST_MODE                            (1u << 29) /* <29:29> R:RW:1: */


/*
 * Controls the reference voltage generator for DFP vs. UFP/Cable operation
 * 0: UFP/Cable - voltage reference is 2.4V
 * 1: DFP - voltage reference is 2.6V
 * This register is a user configuration that must be set.
 * Notes: This bit should only be enablled for DFP
 */
#define PDSS_CC_CTRL_0_DFP_EN                               (1u << 30) /* <30:30> R:RW:0: */


/*
 * Disables all active circuitry and DC paths
 * DS_ATTACH_DET_EN is still active
 * Notes: Firmware can disable the PHY for DFP waiting for attach. This will
 * save power while waiting for attach. Friware sets DS_ATTACH_DET_EN bit
 * to enable resistor based pullup on both CC lines. This pullup can be enabled
 * independent of PHY state.
 * DFP_UA: 1
 * DFP_A: 0
 * UFP: 0
 * Cable: 0
 */
#define PDSS_CC_CTRL_0_PWR_DISABLE                          (1u << 31) /* <31:31> R:RW:1: */


/*
 * USBPD Hard IP C-connector Control Register 1
 */
#define PDSS_CC_CTRL_1_ADDRESS                              (0x400a0604)
#define PDSS_CC_CTRL_1                                      (*(volatile uint32_t *)(0x400a0604))
#define PDSS_CC_CTRL_1_DEFAULT                              (0x00005000)

/*
 * Enables ADFT Mode
 */
#define PDSS_CC_CTRL_1_CC_ADFT_EN                           (1u << 0) /* <0:0> R:RW:0: */


/*
 * Selects ADFT connection
 * See s8usbpd BROS for decoding details
 */
#define PDSS_CC_CTRL_1_CC_ADFT_CTRL_MASK                    (0x0000003e) /* <1:5> R:RW:0: */
#define PDSS_CC_CTRL_1_CC_ADFT_CTRL_POS                     (1)


/*
 * SPARE
 */
#define PDSS_CC_CTRL_1_RX_OFFSET_EN                         (1u << 6) /* <6:6> R:RW:0: */


/*
 * SPARE
 */
#define PDSS_CC_CTRL_1_RX_OFFSET_MASK                       (0x00000380) /* <7:9> R:RW:0: */
#define PDSS_CC_CTRL_1_RX_OFFSET_POS                        (7)


/*
 * Enables the deepsleep attach detect pull-up resistor
 * Set HI for a DFP waiting for attach
 */
#define PDSS_CC_CTRL_1_DS_ATTACH_DET_EN                     (1u << 10) /* <10:10> R:RW:0: */


/*
 * Transmit voltage select
 */
#define PDSS_CC_CTRL_1_VTX_SEL_MASK                         (0x00003800) /* <11:13> R:RW:2: */
#define PDSS_CC_CTRL_1_VTX_SEL_POS                          (11)


/*
 * 0: All outputs are isolated to a known value
 * 1: Normal operation
 */
#define PDSS_CC_CTRL_1_CC_ISO_N                             (1u << 14) /* <14:14> R:RW:1: */


/*
 * Select reference source for transmitter
 * 0- Internal deepsleep reference
 * 1-CC_SHVT reference from reference generator selected
 */
#define PDSS_CC_CTRL_1_CC_VREF_1P1_SEL                      (1u << 15) /* <15:15> R:RW:0: */


/*
 * Controls the CC Charge Pump voltage references
 * 0 - Charge pump voltage is lower
 * 1 - Charge pump voltage is higher
 */
#define PDSS_CC_CTRL_1_CP_REF_SEL                           (1u << 16) /* <16:16> R:RW:0: */


/*
 * Connects cmp_dn comparator to CC1/CC2
 * 0 - CC1
 * 1 - CC2
 * See Note from CC_CTRL_0.CC_1V2 register.
 */
#define PDSS_CC_CTRL_1_CMP_FS_CC1V2                         (1u << 17) /* <17:17> R:RW:0: */


/*
 * Selects the voltage threshold for cmp_fs comparator
 * Notes: 0.52V reference should be used for Fast Swap Detection
 */
#define PDSS_CC_CTRL_1_CMP_FS_VSEL_MASK                     (0x001c0000) /* <18:20> R:RW:0: */
#define PDSS_CC_CTRL_1_CMP_FS_VSEL_POS                      (18)


/*
 * USBPD Hard IP DeepSleep-Reference Control Register
 */
#define PDSS_DPSLP_REF_CTRL_ADDRESS                         (0x400a0608)
#define PDSS_DPSLP_REF_CTRL                                 (*(volatile uint32_t *)(0x400a0608))
#define PDSS_DPSLP_REF_CTRL_DEFAULT                         (0x00000031)

/*
 * Setting this bit will enable the deepsleep current reference outputs.
 */
#define PDSS_DPSLP_REF_CTRL_IGEN_EN                         (1u << 0) /* <0:0> R:RW:1: */


/*
 * Setting this bit will enable the deepsleep reference generator ADFT mode.
 */
#define PDSS_DPSLP_REF_CTRL_DPSLP_ADFT_EN                   (1u << 1) /* <1:1> R:RW:0: */


/*
 * Controls the Deep Sleep reference ADFT mode
 * 0: ganged 7 iref current sources
 * 1: vrefdpslp voltage reference
 *
 */
#define PDSS_DPSLP_REF_CTRL_ADFT_CTRL                       (1u << 2) /* <2:2> R:RW:0: */


/*
 * Block enable input
 * 1 - All analog and DC paths cut off, outputs forced to known value
 *      This completely disables the CC Transceiver/Detect block.
 * 0 - Normal functionality
 * Notes: Firmware can disable the PHY for DFP waiting for attach. This will
 * save power while waiting for attach. Friware sets DS_ATTACH_DET_EN bit
 * to enable resistor based pullup on both CC lines. This pullup can be enabled
 * independent of PHY state.
 * DFP_UA: 1
 * DFP_A: 0
 * UFP: 0
 * Cable: 0
 */
#define PDSS_DPSLP_REF_CTRL_PD_DPSLP                        (1u << 3) /* <3:3> R:RW:0: */


/*
 * Control to "zero-out" PTAT/CTAT currents
 * 1. 00 : PTAT-CTAT Zeroed-out (ibias=0)
 * 2. 01 : CTAT zeroed-out, IBIAS = IPTAT
 * 3. 10 : PTAT zeroed-out, IBIAS-ICTAT
 * 4. 11 : Both enabled
 */
#define PDSS_DPSLP_REF_CTRL_PCTAT_CTRL_MASK                 (0x00000030) /* <4:5> R:RW:3: */
#define PDSS_DPSLP_REF_CTRL_PCTAT_CTRL_POS                  (4)


/*
 * USBPD Hard IP PUMP control Register
 */
#define PDSS_PUMP_CTRL_ADDRESS                              (0x400a0610)
#define PDSS_PUMP_CTRL                                      (*(volatile uint32_t *)(0x400a0610))
#define PDSS_PUMP_CTRL_DEFAULT                              (0x00000014)

/*
 * ADFT control
 */
#define PDSS_PUMP_CTRL_ADFT_MASK                            (0x00000003) /* <0:1> R:RW:0: */
#define PDSS_PUMP_CTRL_ADFT_POS                             (0)


/*
 * Bypasses the pumped output (vpump).
 * 0: Charge pump outputs vpump
 * 1: Pump output (vpump) is shorted to VCCD
 */
#define PDSS_PUMP_CTRL_BYPASS_LV                            (1u << 2) /* <2:2> R:RW:1: */


/*
 * External clock select
 * 0: Internal oscillator used for charge pump
 * 1: External clock used for charge pump
 */
#define PDSS_PUMP_CTRL_CLK_SEL                              (1u << 3) /* <3:3> R:RW:0: */


/*
 * Pump powerdown signal
 * 0: Pump enabled
 * 1: Pump disabled, all current paths cutoff
 * The pump must be disabled when the CC block is disabled (see CC_CTRL_0.PWR_DISABLE
 * bit).
 * The pump must be disabled when the deepsleep attach detect pull-up resistor
 * is enabled (see CC_CTRL_1.DS_ATTACH_DET_EN bit).
 * The pump may be disabled when in UFP/sink mode with no external Rp present.
 *  This corresponds to the Unattached.SNK USB Type-C state.
 * The pump must be enabled for all other modes. This includes DFP/source
 * with any Rp enabled and attached UFP/sink modes.
 */
#define PDSS_PUMP_CTRL_PD_PUMP                              (1u << 4) /* <4:4> R:RW:1: */


/*
 * USBPD Hard IP Regulator #1-4 and VSYS Supply Switch 1 Register
 */
#define PDSS_VREG_VSYS_CTRL_ADDRESS                         (0x400a061c)
#define PDSS_VREG_VSYS_CTRL                                 (*(volatile uint32_t *)(0x400a061c))
#define PDSS_VREG_VSYS_CTRL_DEFAULT                         (0x00000000)

/*
 * When VREG20_EN is set/de-assert, the regulator will turn off/on after
 * VREG20_ONOFF_CNTR cycle of clk_tx.
 * There won’t be any delay is this register is set to zero.
 */
#define PDSS_VREG_VSYS_CTRL_VREG20_ONOFF_CNTR_MASK          (0x0000ff00) /* <8:15> R:RW:0:KEEP_REG_BIT */
#define PDSS_VREG_VSYS_CTRL_VREG20_ONOFF_CNTR_POS           (8)


/*
 * USBPD Hard IP AMUX #1-32 Register
 */
#define PDSS_AMUX_CTRL_ADDRESS                              (0x400a0628)
#define PDSS_AMUX_CTRL                                      (*(volatile uint32_t *)(0x400a0628))
#define PDSS_AMUX_CTRL_DEFAULT                              (0x00000000)

/*
 * AMUX select control
 */
#define PDSS_AMUX_CTRL_SEL_MASK                             (0x00003fff) /* <0:13> R:RW:0:USB_AMUX_NUM */
#define PDSS_AMUX_CTRL_SEL_POS                              (0)


/*
 * USBPD Hard IP AMUX_DENFET #1-32 Register
 */
#define PDSS_AMUX_DENFET_CTRL_ADDRESS                       (0x400a0630)
#define PDSS_AMUX_DENFET_CTRL                               (*(volatile uint32_t *)(0x400a0630))
#define PDSS_AMUX_DENFET_CTRL_DEFAULT                       (0x00000000)

/*
 * AMUX select control
 */
#define PDSS_AMUX_DENFET_CTRL_SEL_MASK                      (0x00000003) /* <0:1> R:RW:0:USB_AMUX_DENFET_NUM */
#define PDSS_AMUX_DENFET_CTRL_SEL_POS                       (0)


/*
 * USBPD Hard IP DAC #1-4 Register
 */
#define PDSS_ADC_CTRL_ADDRESS                               (0x400a0680)
#define PDSS_ADC_CTRL                                       (*(volatile uint32_t *)(0x400a0680))
#define PDSS_ADC_CTRL_DEFAULT                               (0x80000200)

/*
 * Control bits for 8-bit DAC.
 * DAC_CNTRL register is used only if CPU wants to implement the SAR algorithm
 * in FW.
 */
#define PDSS_ADC_CTRL_DAC_CNTRL_MASK                        (0x000000ff) /* <0:7> R:RW:0: */
#define PDSS_ADC_CTRL_DAC_CNTRL_POS                         (0)


/*
 * ADC DFT Control:
 * 0: Normal operation
 * 1: DAC output voltage
 * p
 */
#define PDSS_ADC_CTRL_DFT_MUXSEL                            (1u << 8) /* <8:8> R:RW:0: */


/*
 * This is for when high voltage supply for a port is not present. This bit
 * should be set when the high voltage is present,
 * in order to ensure that the outputs are set to know values.
 * 0: All outputs are isolated to a known value
 * 1: Normal operation
 */
#define PDSS_ADC_CTRL_ADC_ISO_N                             (1u << 9) /* <9:9> R:RW:1: */


/*
 * Comparator Output.  If voltage on ID pin is less than DAC voltage, then
 * cmp_out is HIGH.
 */
#define PDSS_ADC_CTRL_CMP_OUT                               (1u << 15) /* <15:15> RW:R:0: */


/*
 * Input Voltage select
 * p
 */
#define PDSS_ADC_CTRL_VSEL_MASK                             (0x00060000) /* <17:18> R:RW:0: */
#define PDSS_ADC_CTRL_VSEL_POS                              (17)


/*
 * Bit to select between VDDD reference and vref_dac
 * 0 : vref_dac
 * 1 : vddd
 */
#define PDSS_ADC_CTRL_VREF_DAC_SEL                          (1u << 19) /* <19:19> R:RW:0: */


/*
 * ADC Power down control, active high.
 * p
 */
#define PDSS_ADC_CTRL_PD_LV                                 (1u << 31) /* <31:31> R:RW:1: */


/*
 * USBPD Hard IP RefGen #1-4 Register 0
 */
#define PDSS_REFGEN_0_CTRL_ADDRESS                          (0x400a0690)
#define PDSS_REFGEN_0_CTRL                                  (*(volatile uint32_t *)(0x400a0690))
#define PDSS_REFGEN_0_CTRL_DEFAULT                          (0x80000000)

/*
 * DFT input clock enable.  Overrides all other clock inputs and modes.
 */
#define PDSS_REFGEN_0_CTRL_REFGEN_CLK_DFT_EN                (1u << 0) /* <0:0> R:RW:0: */


/*
 * Selection for external vs internal clock: For clk_sel=0 (default), internal
 * clock is used.
 */
#define PDSS_REFGEN_0_CTRL_REFGEN_CLK_SEL                   (1u << 1) /* <1:1> R:RW:0: */


/*
 * Enable for internal clock driver and gate for input clock
 */
#define PDSS_REFGEN_0_CTRL_REFGEN_CLK_EN                    (1u << 2) /* <2:2> R:RW:0: */


/*
 * Vref input select for the reference opamp for regulation blocks such as
 * lscsa, EA, CC block and vbus_mon. 0 for deepsleep reference of 0.74V and
 * 1 for bandgap reference of 1.2V
 */
#define PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_SEL                (1u << 3) /* <3:3> R:RW:0: */


/*
 * Vref input select for the reference opamp for monitor blocks UV-OV. 0
 * for deepsleep reference of 0.74V and 1 for bandgap reference of 1.2V
 */
#define PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_MON_SEL            (1u << 4) /* <4:4> R:RW:0: */


/*
 * adft control inputs used to connect various analog internal signals to
 * atstio. See BROS for connectivity
 */
#define PDSS_REFGEN_0_CTRL_REFGEN_ATSTCFG_MASK              (0x000003e0) /* <5:9> R:RW:0: */
#define PDSS_REFGEN_0_CTRL_REFGEN_ATSTCFG_POS               (5)


/*
 * Block power down input
 * 1 - All analog and DC paths cut off, outputs forced to known value
 * 0 - Normal functionality
 */
#define PDSS_REFGEN_0_CTRL_REFGEN_PD                        (1u << 31) /* <31:31> R:RW:1: */


/*
 * USBPD Hard IP RefGen #1-4 Register 1
 */
#define PDSS_REFGEN_1_CTRL_ADDRESS                          (0x400a06a0)
#define PDSS_REFGEN_1_CTRL                                  (*(volatile uint32_t *)(0x400a06a0))
#define PDSS_REFGEN_1_CTRL_DEFAULT                          (0x00000000)

/*
 * Selection for vsrc_new_p[0] comparator. This selection ranges from 200mV
 * to 2190mV for 0 to 199 respectively. With the extra bits meaning a floating
 * (unconnected) output
 */
#define PDSS_REFGEN_1_CTRL_SEL0_MASK                        (0x000000ff) /* <0:7> R:RW:0: */
#define PDSS_REFGEN_1_CTRL_SEL0_POS                         (0)


/*
 * Selection for vsrc_new_m[0]  comparator. This selection ranges from 200mV
 * to 2190mV for 0 to 199 respectively. With the extra bits meaning a floating
 * (unconnected) output
 */
#define PDSS_REFGEN_1_CTRL_SEL1_MASK                        (0x0000ff00) /* <8:15> R:RW:0: */
#define PDSS_REFGEN_1_CTRL_SEL1_POS                         (8)


/*
 * Selection for UV comparator. This selection ranges from 200mV to 2190mV
 * for 0 to 199 respectively. With the extra bits meaning a floating (unconnected)
 * output
 */
#define PDSS_REFGEN_1_CTRL_SEL2_MASK                        (0x00ff0000) /* <16:23> R:RW:0: */
#define PDSS_REFGEN_1_CTRL_SEL2_POS                         (16)


/*
 * Selection for OV comparator. This selection ranges from 200mV to 2190mV
 * for 0 to 199 respectively. With the extra bits meaning a floating (unconnected)
 * output
 */
#define PDSS_REFGEN_1_CTRL_SEL3_MASK                        (0xff000000) /* <24:31> R:RW:0: */
#define PDSS_REFGEN_1_CTRL_SEL3_POS                         (24)


/*
 * USBPD Hard IP RefGen #1-4 Register 2
 */
#define PDSS_REFGEN_2_CTRL_ADDRESS                          (0x400a06b0)
#define PDSS_REFGEN_2_CTRL                                  (*(volatile uint32_t *)(0x400a06b0))
#define PDSS_REFGEN_2_CTRL_DEFAULT                          (0x00000000)

/*
 * Selection for SCP  comparator. This selection ranges from 130mV to 2120mV
 * for 0 to 199 respectively. With the extra bits meaning a floating (unconnected)
 * output
 * PAG1S:
 * FeedForward OV
 */
#define PDSS_REFGEN_2_CTRL_SEL4_MASK                        (0x000000ff) /* <0:7> R:RW:0: */
#define PDSS_REFGEN_2_CTRL_SEL4_POS                         (0)


/*
 * Selection for OCP comparator. This selection ranges from 130mV to 2120mV
 * for 0 to 199 respectively. With the extra bits meaning a floating (unconnected)
 * output
 */
#define PDSS_REFGEN_2_CTRL_SEL5_MASK                        (0x0000ff00) /* <8:15> R:RW:0: */
#define PDSS_REFGEN_2_CTRL_SEL5_POS                         (8)


/*
 * Selection for PFC_disable comparator. This selection ranges from 130mV
 * to 2120mV for 0 to 199 respectively. With the extra bits meaning a floating
 * (unconnected) output
 */
#define PDSS_REFGEN_2_CTRL_SEL6_MASK                        (0x00ff0000) /* <16:23> R:RW:0: */
#define PDSS_REFGEN_2_CTRL_SEL6_POS                         (16)


/*
 * Selection for PFC_enable comparator. This selection ranges from 130mV
 * to 2120mV for 0 to 199 respectively. With the extra bits meaning a floating
 * (unconnected) output
 */
#define PDSS_REFGEN_2_CTRL_SEL7_MASK                        (0xff000000) /* <24:31> R:RW:0: */
#define PDSS_REFGEN_2_CTRL_SEL7_POS                         (24)


/*
 * USBPD Hard IP RefGen #1-4 Register 3
 */
#define PDSS_REFGEN_3_CTRL_ADDRESS                          (0x400a06c0)
#define PDSS_REFGEN_3_CTRL                                  (*(volatile uint32_t *)(0x400a06c0))
#define PDSS_REFGEN_3_CTRL_DEFAULT                          (0x00000000)

/*
 * Selection for SR_disable comparator. This selection ranges from 130mV
 * to 2120mV for 0 to 199 respectively. With the extra bits meaning a floating
 * (unconnected) output
 */
#define PDSS_REFGEN_3_CTRL_SEL8_MASK                        (0x000000ff) /* <0:7> R:RW:0: */
#define PDSS_REFGEN_3_CTRL_SEL8_POS                         (0)


/*
 * Selection for SR_enable comparator. This selection ranges from 130mV to
 * 2120mV for 0 to 199 respectively. With the extra bits meaning a floating
 * (unconnected) output
 * PAG1S:
 * FeedForward UV
 */
#define PDSS_REFGEN_3_CTRL_SEL9_MASK                        (0x0000ff00) /* <8:15> R:RW:0: */
#define PDSS_REFGEN_3_CTRL_SEL9_POS                         (8)


/*
 * Selection for EA. This selection ranges from 130mV to 2120mV for 0 to
 * 199 respectively. With the extra bits meaning a floating (unconnected)
 * output
 */
#define PDSS_REFGEN_3_CTRL_SEL10_MASK                       (0x00ff0000) /* <16:23> R:RW:0: */
#define PDSS_REFGEN_3_CTRL_SEL10_POS                        (16)


/*
 * USBPD Hard IP RefGen #1-4 Register 4
 */
#define PDSS_REFGEN_4_CTRL_ADDRESS                          (0x400a06d0)
#define PDSS_REFGEN_4_CTRL                                  (*(volatile uint32_t *)(0x400a06d0))
#define PDSS_REFGEN_4_CTRL_DEFAULT                          (0x0000091b)

/*
 * Selection for dischg_en. This selection ranges from 450mV to 800mV for
 * 0 to 7 respectively in steps of 50mV. With the extra bits meaning a floating
 * (unconnected) output
 */
#define PDSS_REFGEN_4_CTRL_SEL11_MASK                       (0x00000007) /* <0:2> R:RW:3: */
#define PDSS_REFGEN_4_CTRL_SEL11_POS                        (0)


/*
 * Selection for vbus_c_mon. This selection ranges from 650mV to 1000mV for
 * 0 to 7 respectively in steps of 50mV. With the extra bits meaning a floating
 * (unconnected) output
 */
#define PDSS_REFGEN_4_CTRL_SEL12_MASK                       (0x00000038) /* <3:5> R:RW:3: */
#define PDSS_REFGEN_4_CTRL_SEL12_POS                        (3)


/*
 * Selection for ADC_Ref. This selection ranges from 1960mV to 2030mV for
 * 0 to 7 respectively in steps of 10mV. With the extra bits meaning a floating
 * (unconnected) output
 */
#define PDSS_REFGEN_4_CTRL_SEL13_MASK                       (0x000001c0) /* <6:8> R:RW:4: */
#define PDSS_REFGEN_4_CTRL_SEL13_POS                        (6)


/*
 * Selection for CC_SHVT Ref. This selection ranges from 1090mV to 1160mV
 * for 0 to 7 respectively in steps of 10mV. With the extra bits meaning
 * a floating (unconnected) output
 */
#define PDSS_REFGEN_4_CTRL_SEL14_MASK                       (0x00000e00) /* <9:11> R:RW:4: */
#define PDSS_REFGEN_4_CTRL_SEL14_POS                        (9)


/*
 * USBPD Hard IP Battery Charger #1-4 Register0
 */
#define PDSS_BCH_DET_0_CTRL_ADDRESS                         (0x400a0700)
#define PDSS_BCH_DET_0_CTRL                                 (*(volatile uint32_t *)(0x400a0700))
#define PDSS_BCH_DET_0_CTRL_DEFAULT                         (0x80000000)

/*
 * Enables BC1.2 Circuitry, Active High
 */
#define PDSS_BCH_DET_0_CTRL_EN_CHGDET                       (1u << 0) /* <0:0> R:RW:0: */


/*
 * IDP_SINK enable, Active High
 */
#define PDSS_BCH_DET_0_CTRL_IDP_SNK_EN                      (1u << 1) /* <1:1> R:RW:0: */


/*
 * IDM_SINK enable, Active High
 */
#define PDSS_BCH_DET_0_CTRL_IDM_SNK_EN                      (1u << 2) /* <2:2> R:RW:0: */


/*
 * VDP_SRC enable, Active High
 */
#define PDSS_BCH_DET_0_CTRL_VDP_SRC_EN                      (1u << 3) /* <3:3> R:RW:0: */


/*
 * VDM_SRC enable, Active High
 */
#define PDSS_BCH_DET_0_CTRL_VDM_SRC_EN                      (1u << 4) /* <4:4> R:RW:0: */


/*
 * IDP_SRC enable, Active High
 */
#define PDSS_BCH_DET_0_CTRL_IDP_SRC_EN                      (1u << 5) /* <5:5> R:RW:0: */


/*
 * DCP Short enable, Active High
 * Shorts D+ to D- with low resistance path
 */
#define PDSS_BCH_DET_0_CTRL_DCP_EN                          (1u << 6) /* <6:6> R:RW:0: */


/*
 * RDM_DWN enable, Active High
 */
#define PDSS_BCH_DET_0_CTRL_RDM_PD_EN                       (1u << 7) /* <7:7> R:RW:0: */


/*
 * RDM_UP enable, Active High
 */
#define PDSS_BCH_DET_0_CTRL_RDM_PU_EN                       (1u << 8) /* <8:8> R:RW:0: */


/*
 * RDP_DWN enable, Active High
 */
#define PDSS_BCH_DET_0_CTRL_RDP_PD_EN                       (1u << 9) /* <9:9> R:RW:0: */


/*
 * RDP_UP enable, Active High
 */
#define PDSS_BCH_DET_0_CTRL_RDP_PU_EN                       (1u << 10) /* <10:10> R:RW:0: */


/*
 * RDAT_LKG_DP enable, Active High
 */
#define PDSS_BCH_DET_0_CTRL_RDAT_LKG_DP_EN                  (1u << 11) /* <11:11> R:RW:0: */


/*
 * RDAT_LKG_DM enable, Active High
 */
#define PDSS_BCH_DET_0_CTRL_RDAT_LKG_DM_EN                  (1u << 12) /* <12:12> R:RW:0: */


/*
 * Output Comparator Negative input select bits:
 * 00 - DM pin
 * 01 - Vref
 * 10 - DP pin
 * 11 - GND
 */
#define PDSS_BCH_DET_0_CTRL_CMP1_INN_SEL_MASK               (0x00006000) /* <13:14> R:RW:0: */
#define PDSS_BCH_DET_0_CTRL_CMP1_INN_SEL_POS                (13)


/*
 * Output Comparator Positive input select bits:
 * 00 - DM pin
 * 01 - Vref
 * 10 - DP pin
 * 11 - GND
 */
#define PDSS_BCH_DET_0_CTRL_CMP1_INP_SEL_MASK               (0x00018000) /* <15:16> R:RW:0: */
#define PDSS_BCH_DET_0_CTRL_CMP1_INP_SEL_POS                (15)


/*
 * Reference voltage select bits
 * 0 - 0.325V
 * 1 - 0.7V
 * 2 - 0.85V
 * 3 - 1.4V
 * 4 - 1.7V
 * 5 - 2.0V
 * 6 - 2.2V
 * 7 - 2.9V
 */
#define PDSS_BCH_DET_0_CTRL_CMP1_VREF_SEL_MASK              (0x000e0000) /* <17:19> R:RW:0: */
#define PDSS_BCH_DET_0_CTRL_CMP1_VREF_SEL_POS               (17)


/*
 * Enable Output Comparator, Active High
 */
#define PDSS_BCH_DET_0_CTRL_EN_COMP1_CHGDET                 (1u << 20) /* <20:20> R:RW:0: */


/*
 * Comparator Offset Select:
 * 0: -50mV
 * 1: -100mV
 * 2: -150mV
 * 3: -200mV
 * 4: +50mV
 * 5: +100mV
 * 6: +150mV
 * 7: +200mV
 */
#define PDSS_BCH_DET_0_CTRL_CMP1_OFFSET_SEL_MASK            (0x00e00000) /* <21:23> R:RW:0: */
#define PDSS_BCH_DET_0_CTRL_CMP1_OFFSET_SEL_POS             (21)


/*
 * Output Comparator Offset Enable, Active High
 */
#define PDSS_BCH_DET_0_CTRL_CMP1_OFFSET_EN                  (1u << 24) /* <24:24> R:RW:0: */


/*
 * Output Comparator Unity Gain Mode Enable, Active High
 */
#define PDSS_BCH_DET_0_CTRL_CMP1_UGM_EN                     (1u << 25) /* <25:25> R:RW:0: */


/*
 * Charger Detect Block Power Down
 */
#define PDSS_BCH_DET_0_CTRL_PD                              (1u << 31) /* <31:31> R:RW:1: */


/*
 * USBPD Hard IP Battery Charger #1-4 Register1
 */
#define PDSS_BCH_DET_1_CTRL_ADDRESS                         (0x400a0710)
#define PDSS_BCH_DET_1_CTRL                                 (*(volatile uint32_t *)(0x400a0710))
#define PDSS_BCH_DET_1_CTRL_DEFAULT                         (0x00001000)

/*
 * Enable Charger Detect ADFT Mode
 */
#define PDSS_BCH_DET_1_CTRL_CHGDET_ADFT_EN                  (1u << 0) /* <0:0> R:RW:0: */


/*
 * ADFT Select Control.  See the s8usbpd_ver3 BROS for more details
 */
#define PDSS_BCH_DET_1_CTRL_CHGDET_ADFT_CTRL_MASK           (0x0000001e) /* <1:4> R:RW:0: */
#define PDSS_BCH_DET_1_CTRL_CHGDET_ADFT_CTRL_POS            (1)


/*
 * Output isolation control.  Active Low
 * 0: All digital outputs are forced low
 */
#define PDSS_BCH_DET_1_CTRL_CHGDET_ISO_N                    (1u << 5) /* <5:5> R:RW:0: */


/*
 * Apple detection enable to detect the 2.9V termination voltage on
 */
#define PDSS_BCH_DET_1_CTRL_CHGDET_APP_DET                  (1u << 6) /* <6:6> R:RW:0: */


/*
 * Apple DM termination enable control
 * 00 - Termination disabled
 * 01 - 1.5V
 * 10 - 2.5V: This mode uses the BC1.2 Driver, so BCH_DET_1_CTRL.CHGDET_APP_DET,
 * BCH_DET_0_CTRL.EN_CHGDET and BCH_DET_0_CTRL.VDM_SRC_EN must be set high
 * 11 - DO NOT USE: For 3.3V termination, use the RPU pullup. Set BCH_DET_0_CTRL.RDM_PU_EN
 * high
 */
#define PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DM_MASK       (0x00000180) /* <7:8> R:RW:0: */
#define PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DM_POS        (7)


/*
 * Apple DP termination enable control
 * 00 - Termination disabled
 * 01 - 1.5V
 * 10 - 2.5V: This mode uses the BC1.2 Driver, so BCH_DET_1_CTRL.CHGDET_APP_DET,
 * BCH_DET_0_CTRL.EN_CHGDET and BCH_DET_0_CTRL.VDM_SRC_EN must be set high
 * 11 - DO NOT USE: For 3.3V termination, use the RPU pullup. Set BCH_DET_0_CTRL.RDM_PU_EN
 * high
 */
#define PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DP_MASK       (0x00000600) /* <9:10> R:RW:0: */
#define PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DP_POS        (9)


/*
 * IDP_SRC enable, Active High. Enables idm source current to detect Samsung
 * AFC termination.
 */
#define PDSS_BCH_DET_1_CTRL_CHGDET_IDM_SRC_EN               (1u << 11) /* <11:11> R:RW:0: */


/*
 * To select between bandgap or deepsleep reference volatge as input.
 * 1 -> Bandgap reference voltage.
 * 0 -> Deepsleep reference voltage.
 */
#define PDSS_BCH_DET_1_CTRL_REFGEN_REFSEL                   (1u << 12) /* <12:12> R:RW:1: */


/*
 * Output Comparator Negative input select bits:
 * 00 - DM pin
 * 01 - Vref
 * 10 - DP pin
 * 11 - GND
 */
#define PDSS_BCH_DET_1_CTRL_CMP2_INN_SEL_MASK               (0x00006000) /* <13:14> R:RW:0: */
#define PDSS_BCH_DET_1_CTRL_CMP2_INN_SEL_POS                (13)


/*
 * Output Comparator Positive input select bits:
 * 00 - DM pin
 * 01 - Vref
 * 10 - DP pin
 * 11 - GND
 */
#define PDSS_BCH_DET_1_CTRL_CMP2_INP_SEL_MASK               (0x00018000) /* <15:16> R:RW:0: */
#define PDSS_BCH_DET_1_CTRL_CMP2_INP_SEL_POS                (15)


/*
 * Reference voltage select bits
 * 0 - 0.325V
 * 1 - 0.7V
 * 2 - 0.85V
 * 3 - 1.4V
 * 4 - 1.7V
 * 5 - 2.0V
 * 6 - 2.2V
 * 7 - 2.9V
 */
#define PDSS_BCH_DET_1_CTRL_CMP2_VREF_SEL_MASK              (0x000e0000) /* <17:19> R:RW:0: */
#define PDSS_BCH_DET_1_CTRL_CMP2_VREF_SEL_POS               (17)


/*
 * Enable Output Comparator, Active High
 */
#define PDSS_BCH_DET_1_CTRL_EN_COMP2_CHGDET                 (1u << 20) /* <20:20> R:RW:0: */


/*
 * Comparator Offset Select:
 * 0: -50mV
 * 1: -100mV
 * 2: -150mV
 * 3: -200mV
 * 4: +50mV
 * 5: +100mV
 * 6: +150mV
 * 7: +200mV
 */
#define PDSS_BCH_DET_1_CTRL_CMP2_OFFSET_SEL_MASK            (0x00e00000) /* <21:23> R:RW:0: */
#define PDSS_BCH_DET_1_CTRL_CMP2_OFFSET_SEL_POS             (21)


/*
 * Output Comparator Offset Enable, Active High
 */
#define PDSS_BCH_DET_1_CTRL_CMP2_OFFSET_EN                  (1u << 24) /* <24:24> R:RW:0: */


/*
 * Output Comparator Unity Gain Mode Enable, Active High
 */
#define PDSS_BCH_DET_1_CTRL_CMP2_UGM_EN                     (1u << 25) /* <25:25> R:RW:0: */


/*
 * Apple 2.5V DP termination with 5K,10K,20K,100K resistance in series path
 * to enable simultaneous apple and BC detection
 */
#define PDSS_BCH_DET_1_CTRL_APPLE_TERM_MASK                 (0x3c000000) /* <26:29> R:RW:0:PASC_EN */
#define PDSS_BCH_DET_1_CTRL_APPLE_TERM_POS                  (26)


/*
 * USBPD Hard IP LS CSA #1-4 Register0
 */
#define PDSS_LSCSA_0_CTRL_ADDRESS                           (0x400a0740)
#define PDSS_LSCSA_0_CTRL                                   (*(volatile uint32_t *)(0x400a0740))
#define PDSS_LSCSA_0_CTRL_DEFAULT                           (0x00000000)

/*
 * Selects the nominal voltage gain for OCP. Selects between 1 of 4 tap points
 * of gain. 2 bits per stage => 4 bits for 2 stages. [1:0] stage-1; [3:2]
 * stage-2
 */
#define PDSS_LSCSA_0_CTRL_AV_OCP_MASK                       (0x0000000f) /* <0:3> R:RW:0: */
#define PDSS_LSCSA_0_CTRL_AV_OCP_POS                        (0)


/*
 * Selects the nominal voltage gain for OCP. Selects between output of muxes
 * from 2 stages.
 */
#define PDSS_LSCSA_0_CTRL_AV_SEL_OCP                        (1u << 4) /* <4:4> R:RW:0: */


/*
 * Selects the nominal voltage gain for EA. Selects between 1 of 4 tap points
 * of gain. 2 bits per stage => 4 bits for 2 stages.[1:0] stage-1; [3:2]
 * stage-2
 */
#define PDSS_LSCSA_0_CTRL_AV_EA_MASK                        (0x000001e0) /* <5:8> R:RW:0: */
#define PDSS_LSCSA_0_CTRL_AV_EA_POS                         (5)


/*
 * Selects the nominal voltage gain for EA. Selects between output of muxes
 * from 2 stages.
 */
#define PDSS_LSCSA_0_CTRL_AV_SEL_EA                         (1u << 9) /* <9:9> R:RW:0: */


/*
 * Selects the nominal voltage gain for PFC_off. Selects between 1 of 4 tap
 * points of gain. 2 bits per stage => 4 bits for 2 stages.[1:0] stage-1;
 * [3:2] stage-2
 */
#define PDSS_LSCSA_0_CTRL_AV_PFC_OFF_MASK                   (0x00003c00) /* <10:13> R:RW:0: */
#define PDSS_LSCSA_0_CTRL_AV_PFC_OFF_POS                    (10)


/*
 * Selects the nominal voltage gain for PFC_off. Selects between output of
 * muxes from 2 stages.
 */
#define PDSS_LSCSA_0_CTRL_AV_SEL_PFC_OFF                    (1u << 14) /* <14:14> R:RW:0: */


/*
 * Selects the nominal voltage gain for PFC_on. Selects between 1 of 4 tap
 * points of gain. 2 bits per stage => 4 bits for 2 stages. [1:0] stage-1;
 * [3:2] stage-2
 */
#define PDSS_LSCSA_0_CTRL_AV_PFC_ON_MASK                    (0x00078000) /* <15:18> R:RW:0: */
#define PDSS_LSCSA_0_CTRL_AV_PFC_ON_POS                     (15)


/*
 * Selects the nominal voltage gain for PFC_on. Selects between output of
 * muxes from 2 stages.
 */
#define PDSS_LSCSA_0_CTRL_AV_SEL_PFC_ON                     (1u << 19) /* <19:19> R:RW:0: */


/*
 * Selects the nominal voltage gain for SR_off. Selects between 1 of 4 tap
 * points of gain. 2 bits per stage => 4 bits for 2 stages. [1:0] stage-1;
 * [3:2] stage-2
 */
#define PDSS_LSCSA_0_CTRL_AV_SR_OFF_MASK                    (0x00f00000) /* <20:23> R:RW:0: */
#define PDSS_LSCSA_0_CTRL_AV_SR_OFF_POS                     (20)


/*
 * Selects the nominal voltage gain for SR_off. Selects between output of
 * muxes from 2 stages.
 */
#define PDSS_LSCSA_0_CTRL_AV_SEL_SR_OFF                     (1u << 24) /* <24:24> R:RW:0: */


/*
 * Selects the nominal voltage gain for SR_on. Selects between 1 of 4 tap
 * points of gain. 2 bits per stage => 4 bits for 2 stages. [1:0] stage-1;
 * [3:2] stage-2
 */
#define PDSS_LSCSA_0_CTRL_AV_SR_ON_MASK                     (0x1e000000) /* <25:28> R:RW:0: */
#define PDSS_LSCSA_0_CTRL_AV_SR_ON_POS                      (25)


/*
 * Selects the nominal voltage gain for SR_on. Selects between output of
 * muxes from 2 stages.
 */
#define PDSS_LSCSA_0_CTRL_AV_SEL_SR_ON                      (1u << 29) /* <29:29> R:RW:0: */


/*
 * Enable automatic offset cancellation for stage-1
 */
#define PDSS_LSCSA_0_CTRL_OS1_EN                            (1u << 30) /* <30:30> R:RW:0: */


/*
 * Enable automatic offset cancellation for stage-2
 */
#define PDSS_LSCSA_0_CTRL_OS2_EN                            (1u << 31) /* <31:31> R:RW:0: */


/*
 * USBPD Hard IP LS CSA #1-4 Register1
 */
#define PDSS_LSCSA_1_CTRL_ADDRESS                           (0x400a0780)
#define PDSS_LSCSA_1_CTRL                                   (*(volatile uint32_t *)(0x400a0780))
#define PDSS_LSCSA_1_CTRL_DEFAULT                           (0x80000000)

/*
 * Trim control for analog bandwidth
 */
#define PDSS_LSCSA_1_CTRL_LSCSA_BW_MASK                     (0x00000003) /* <0:1> R:RW:0: */
#define PDSS_LSCSA_1_CTRL_LSCSA_BW_POS                      (0)


/*
 * adft control inputs used to connect various analog internal signals to
 * atstio.
 * See BROS doc for more info.
 */
#define PDSS_LSCSA_1_CTRL_LSCSA_ATSTCFG_MASK                (0x00000078) /* <3:6> R:RW:0: */
#define PDSS_LSCSA_1_CTRL_LSCSA_ATSTCFG_POS                 (3)


/*
 * Block power down input
 * 1 - All analog and DC paths cut off, outputs forced to known value
 * 0 - Normal functionality
 */
#define PDSS_LSCSA_1_CTRL_LSCSA_PD                          (1u << 31) /* <31:31> R:RW:1: */


/*
 * USBPD Hard IP VBUS Discharge SHV #1-8 Register0
 */
#define PDSS_DISCHG_SHV_CTRL_ADDRESS(n)                     (0x400a0790 + ((n) * (0x0004)))
#define PDSS_DISCHG_SHV_CTRL(n)                             (*(volatile uint32_t *)(0x400a0790 + ((n) * 0x0004)))
#define PDSS_DISCHG_SHV_CTRL_DEFAULT                        (0x00000000)

/*
 * VBUS Discharge 1-8 control signal, Active High
 */
#define PDSS_DISCHG_SHV_CTRL_DISCHG_EN                      (1u << 0) /* <0:0> R:RW:0: */


/*
 * Used only for CCG3PA-VBUS_IN, Second instance of s8usbpd_dischg_shv_top
 * (Address 0x0794)
 *    0: The disch_en pin of s8usbpd_dischg_shv_top is driven by comparator
 * output;
 *        In this Mode Discharge Enable/Disable is controlled through Discharge
 * Comparator Control Register (COMP_CTRL[3].COMP_PD)
 *        Discharge Enabled: COMP_PD = 0; Disabled: COMP_PD = 1;
 *    1: The disch_en pin of s8usbpd_dischg_shv_top is driven by DISCHG_SHV_CTRL.DISCHG_EN
 * register
 */
#define PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG                  (1u << 1) /* <1:1> R:RW:0: */


/*
 * Following are for CCG5  specific vbus discaharge control seetings
 * 00000: HiZ
 * 00001: 2K Ohms
 * 00011: 1K Ohms
 * 00111: 0.66K Ohms
 * 01111: 0.5K Ohms
 * 11111: 0.4K Ohms
 *
 * CCG3PA*A and CCG3PA2 - pluese use following discharge control settings
 * Discharge Drive Strenght control
 * 00000:HIZ
 * 00001: 500 Ohms - 2000Ohms
 * 00010: 250 Ohms - 1000Ohms
 * 00100: 125 Ohms - 500Ohms
 * 01000: 62.5 Ohms - 250Ohms
 * 10000: 31.25 Ohms - 125Ohms
 */
#define PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK                 (0x0000007c) /* <2:6> R:RW:0: */
#define PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS                  (2)


/*
 * USBPD Hard IP Comparators #1-8 with trim Register
 * CCG3PA:
 *       Function                            Comparator#       CLK_FILTER#
 *       CLK_LF#     FILTER21  FILTER22
 * ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *           SR                                        #0               
 *              #6                                   #6              #6
 */
#define PDSS_COMP_TR_CTRL_ADDRESS                           (0x400a07b0)
#define PDSS_COMP_TR_CTRL                                   (*(volatile uint32_t *)(0x400a07b0))
#define PDSS_COMP_TR_CTRL_DEFAULT                           (0x80000001)

/*
 * Output isolation control.  Active Low
 * 0: All digital outputs are forced to known value
 */
#define PDSS_COMP_TR_CTRL_COMP_ISO_N                        (1u << 0) /* <0:0> R:RW:1: */


/*
 * ADFT control inputs used to connect various analog internal signals to
 * ADFT buses
 */
#define PDSS_COMP_TR_CTRL_COMP_ADFT_CTRL_MASK               (0x0000000e) /* <1:3> R:RW:0: */
#define PDSS_COMP_TR_CTRL_COMP_ADFT_CTRL_POS                (1)


/*
 * Rising edge hysteresis enable.  Does not affect falling edge accuracy.
 */
#define PDSS_COMP_TR_CTRL_COMP_HYST_R_EN                    (1u << 4) /* <4:4> R:RW:0: */


/*
 * Falling edge hysteresis enable.  Does not affect rising edge accuracy.
 */
#define PDSS_COMP_TR_CTRL_COMP_HYST_F_EN                    (1u << 5) /* <5:5> R:RW:0: */


/*
 * Block power down input
 * 1 - All analog and DC paths cut off, outputs forced to known value
 * 0 - Normal functionality
 *
 */
#define PDSS_COMP_TR_CTRL_COMP_PD                           (1u << 31) /* <31:31> R:RW:1: */


/*
 * USBPD Hard IP Comparators #1-24 Register
 * CCG3PA:
 *       Function                            Comparator#       CLK_FILTER#
 *       CLK_LF#    FILTER21  FILTER22
 * ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *           UV                                        #0               
 *              #0                                   #0            #0
 *           OV                                        #1               
 *              #1                                   #1            #1
 *           VBUS_MON                          #2                       
 *     N/A                 #0              #7            #7
 *           DISCHG                                #3                   
 *          #2                                   #2            #2
 *           SCP                                      #4                
 *             #3                                   #3            #3
 *           OCP                                      #5                
 *             #4                                   #4            #4
 *           PFC                                      #6                
 *             #5                                   #5            #5
 *           VSRC_NEW_P                      #7                         
 *     N/A                #1             #8            #8
 *           VSRC_NEW_P                      #8                         
 *     N/A                #2             #9            #9
 * CCG5-Port0:
 * ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *           UV                                        #0               
 *              #0                                   #0            #0
 *           OV                                        #1               
 *              #1                                   #1            #1
 *           VBUS_MON                          #2                       
 *     N/A                 #0              #2            #2
 *           VSYS_DET                           #3                      
 *       N/A                #1               #3           #3
 * CCG5-Port1:
 * ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *           UV                                        #0               
 *              #0                                   #0            #0
 *           OV                                        #1               
 *              #1                                   #1            #1
 *           VBUS_MON                          #2                       
 *     N/A                 #0              #2            #2
 */
#define PDSS_COMP_CTRL_ADDRESS(n)                           (0x400a07d0 + ((n) * (0x0004)))
#define PDSS_COMP_CTRL(n)                                   (*(volatile uint32_t *)(0x400a07d0 + ((n) * 0x0004)))
#define PDSS_COMP_CTRL_DEFAULT                              (0x80000001)

/*
 * Output isolation control.  Active Low
 * 0: All digital outputs are forced to known value
 */
#define PDSS_COMP_CTRL_COMP_ISO_N                           (1u << 0) /* <0:0> R:RW:1: */


/*
 * ADFT control inputs used to connect various analog internal signals to
 * ADFT buses
 */
#define PDSS_COMP_CTRL_COMP_ADFT_CTRL_MASK                  (0x0000000e) /* <1:3> R:RW:0: */
#define PDSS_COMP_CTRL_COMP_ADFT_CTRL_POS                   (1)


/*
 * Rising edge hysteresis enable.  Does not affect falling edge accuracy.
 */
#define PDSS_COMP_CTRL_COMP_HYST_R_EN                       (1u << 4) /* <4:4> R:RW:0: */


/*
 * Falling edge hysteresis enable.  Does not affect rising edge accuracy.
 */
#define PDSS_COMP_CTRL_COMP_HYST_F_EN                       (1u << 5) /* <5:5> R:RW:0: */


/*
 * Block power down input
 * 1 - All analog and DC paths cut off, outputs forced to known value
 * 0 - Normal functionality
 */
#define PDSS_COMP_CTRL_COMP_PD                              (1u << 31) /* <31:31> R:RW:1: */


/*
 * USBPD Hard IP VBUS Discharge SHV #1-8 Register1
 */
#define PDSS_DISCHG_SHV_1_CTRL_ADDRESS(n)                   (0x400a089c + ((n) * (0x0004)))
#define PDSS_DISCHG_SHV_1_CTRL(n)                           (*(volatile uint32_t *)(0x400a089c + ((n) * 0x0004)))
#define PDSS_DISCHG_SHV_1_CTRL_DEFAULT                      (0x00000000)

/*
 * This bit enables the discharge protection funcitonality
 */
#define PDSS_DISCHG_SHV_1_CTRL_FAILSAFE_EN                  (1u << 0) /* <0:0> R:RW:0: */


/*
 * this bit selects the driving of the s8usbpd_dischg_shv_top enable
 * 0 - Enabled when FW enables the DISCHG
 * 1 - Dutycycle configured by FW is applied on DISCHG
 */
#define PDSS_DISCHG_SHV_1_CTRL_MODE                         (1u << 1) /* <1:1> R:RW:0: */


/*
 * This bit selects the source for monitoring the voltage level of VBUS.
 * IF the selected source does not fall when the programmed timer expires,
 * the protection is triggered.
 * CCG3PA2:
 * 0 - HS Filter 0 output (UV)
 * 1 - HS Filter 1 output (OV)
 * 2 - HS Filter 2 output (DISCHG_EN)
 * 3 - HS Filter 3 output (SCP)
 * 4 - HS Filter 4 output (OCP)
 * 5 - HS Filter 5 output (PFC)
 * 6 - HS Filter 6 output (SR)
 * 7 - LS Filter 0 output (VBUS_MON)
 * 8 - LS Filter 1 output (Vsrc_new_p)
 * 9 - LS Filter 2 output (Vsrc_new_m)
 * 10 - ADC 0 Filter output
 * 11 - ADC 1 Filter output
 *
 * PAG1S:
 * 0 - HS Filter 0 output (UV)
 * 1 - HS Filter 1 output (OV)
 * 2 - HS Filter 2 output (DISCHG_EN)
 * 3 - HS Filter 3 output (SCP)
 * 4 - HS Filter 4 output (OCP)
 * 5 - HS Filter 5 output (PFC)
 * 6 - HS Filter 6 output (SR)
 * 7 - LS Filter 0 output (VBUS_MON)
 * 8 - LS Filter 1 output (Vsrc_new_p)
 * 9 - LS Filter 2 output (Vsrc_new_m)
 * 10 - ADC 0 Filter output
 * 11 - FILT21 output (Based on INTR11_FILTER_CFG.FILT21_INPUT_SEL)
 * 12 - FILT22 output (Based on INTR11_FILTER_CFG.FILT22_INPUT_SEL)
 * This field needs to be programmed only when FAILSAFE_EN is 0
 */
#define PDSS_DISCHG_SHV_1_CTRL_SOURCE_SEL_MASK              (0x000003fc) /* <2:9> R:RW:0: */
#define PDSS_DISCHG_SHV_1_CTRL_SOURCE_SEL_POS               (2)


/*
 * Time period in LF clock cycles after which the selected source has to
 * be monitored to trigger the protection circuit. This field needs to be
 * programmed only when FAILSAFE_EN is 0
 */
#define PDSS_DISCHG_SHV_1_CTRL_TIMER_VALUE_MASK             (0x000ffc00) /* <10:19> R:RW:0: */
#define PDSS_DISCHG_SHV_1_CTRL_TIMER_VALUE_POS              (10)


/*
 * Time period in LF clock cycles of the time period on DISCHG enable, ON
 * + OFF Time. This field needs to be programmed only when MODE is 0
 */
#define PDSS_DISCHG_SHV_1_CTRL_PWM_PERIOD_MASK              (0x01f00000) /* <20:24> R:RW:0: */
#define PDSS_DISCHG_SHV_1_CTRL_PWM_PERIOD_POS               (20)


/*
 * Time period in LF clock cycles of the ON time on DISCHG enable, ON + OFF
 * Time. This field needs to be programmed only when MODE is 0
 */
#define PDSS_DISCHG_SHV_1_CTRL_ON_TIME_MASK                 (0x3e000000) /* <25:29> R:RW:0: */
#define PDSS_DISCHG_SHV_1_CTRL_ON_TIME_POS                  (25)


/*
 * USBPD Hard IP AMUX_NHVN #1-32 Register
 */
#define PDSS_AMUX_NHVN_CTRL_ADDRESS                         (0x400a08cc)
#define PDSS_AMUX_NHVN_CTRL                                 (*(volatile uint32_t *)(0x400a08cc))
#define PDSS_AMUX_NHVN_CTRL_DEFAULT                         (0x00000000)

/*
 * AMUX select control
 */
#define PDSS_AMUX_NHVN_CTRL_SEL_MASK                        (0xffffffff) /* <0:31> R:RW:0:USB_AMUX_NHVN_NUM */
#define PDSS_AMUX_NHVN_CTRL_SEL_POS                         (0)


/*
 * S8PDS Hard IP SCP Register
 */
#define PDSS_PDS_SCP_CTRL_ADDRESS                           (0x400a08d0)
#define PDSS_PDS_SCP_CTRL                                   (*(volatile uint32_t *)(0x400a08d0))
#define PDSS_PDS_SCP_CTRL_DEFAULT                           (0x20000000)

/*
 * SCP enable signal
 */
#define PDSS_PDS_SCP_CTRL_SCP_EN                            (1u << 0) /* <0:0> R:RW:0: */


/*
 * Output isolation control.  Active Low
 * 0: All digital outputs are forced to known value
 */
#define PDSS_PDS_SCP_CTRL_SCP_ISO_N                         (1u << 7) /* <7:7> R:RW:0: */


/*
 * adft control inputs used to connect various analog internal signals to
 * atstio.
 * See BROS doc for more info.
 */
#define PDSS_PDS_SCP_CTRL_SCP_ADFT_EN                       (1u << 8) /* <8:8> R:RW:0: */


/*
 * adft control inputs used to connect various analog internal signals to
 * atstio.
 * See BROS doc for more info.
 */
#define PDSS_PDS_SCP_CTRL_SCP_ADFT_CTRL_MASK                (0x00000e00) /* <9:11> R:RW:0: */
#define PDSS_PDS_SCP_CTRL_SCP_ADFT_CTRL_POS                 (9)


/*
 * testmode for SCP block:
 * <6>    : Spare
 * <5>    : Slow mode, block current reduced by half (default 0:  1x current)
 * <4:3> : Mode control (6A, 10A)
 *             00 : 0A SCP - works as ideal comparator : Should not be used
 *             01 : 8A SCP (40mV with 5mOhm Rsense)
 *             10 : 6A SCP (30mV with 5mOhm Rsense)
 *             11 : 10A SCP (50mV with 5mOhm Rsense)
 * <2>    : Hysteresis disable (0 : Enable, 1 : disable)
 * <1>    : Fast mode, double the block current; Also used for 10mOhm Rsense
 * <0>    : Spare
 */
#define PDSS_PDS_SCP_CTRL_SCP_TESTMODE_MASK                 (0xfe000000) /* <25:31> R:RW:16: */
#define PDSS_PDS_SCP_CTRL_SCP_TESTMODE_POS                  (25)


/*
 * S8PDS Hard IP PWM Register 0
 */
#define PDSS_PWM_0_CTRL_ADDRESS                             (0x400a08d4)
#define PDSS_PWM_0_CTRL                                     (*(volatile uint32_t *)(0x400a08d4))
#define PDSS_PWM_0_CTRL_DEFAULT                             (0x0005e000)

/*
 * Enable Signal for the whole PWM Block
 */
#define PDSS_PWM_0_CTRL_ENABLE_PWM                          (1u << 0) /* <0:0> R:RW:0: */


/*
 * Output isolation control.  Active Low
 * 0: All digital outputs are forced to known value
 */
#define PDSS_PWM_0_CTRL_PWM_ISO_N                           (1u << 1) /* <1:1> R:RW:0: */


/*
 * Enable signal for fixed DAC
 */
#define PDSS_PWM_0_CTRL_ENABLE_PWDM_DAC                     (1u << 2) /* <2:2> R:RW:0: */


/*
 * Enable signal for feedforward/Vin DAC
 */
#define PDSS_PWM_0_CTRL_ENABLE_PWM_DAC_FF                   (1u << 3) /* <3:3> R:RW:0: */


/*
 * Enable signal for hclamp comparator
 */
#define PDSS_PWM_0_CTRL_ENABLE_PWM_HCLAMP                   (1u << 4) /* <4:4> R:RW:0: */


/*
 * Enable signal for lclamp reference generation block
 */
#define PDSS_PWM_0_CTRL_ENABLE_PWM_LCLAMP                   (1u << 5) /* <5:5> R:RW:0: */


/*
 * Enable signal for skip comparator
 */
#define PDSS_PWM_0_CTRL_ENABLE_PWM_SKIP                     (1u << 6) /* <6:6> R:RW:0: */


/*
 * Fixed DAC Input current prescalar (0 : 80nA; 1: 160nA)
 */
#define PDSS_PWM_0_CTRL_DRV_IDAC                            (1u << 7) /* <7:7> R:RW:0: */


/*
 * Frequecny shift from 150KHz to 450Khz (0:150KHz; 1: 450Khz)
 */
#define PDSS_PWM_0_CTRL_OFFSET_EN_IDAC                      (1u << 8) /* <8:8> R:RW:0: */


/*
 * Enable signal for PWM capacitor trimming (FF_enable, Fixed_enable)
 */
#define PDSS_PWM_0_CTRL_ENABLE_CAP_TRIM_MASK                (0x00000600) /* <9:10> R:RW:0: */
#define PDSS_PWM_0_CTRL_ENABLE_CAP_TRIM_POS                 (9)


/*
 * operate pwm comparator at half current
 */
#define PDSS_PWM_0_CTRL_ENABLE_PWM_COMP_HALF                (1u << 11) /* <11:11> R:RW:0: */


/*
 * enable burst exit comparator
 */
#define PDSS_PWM_0_CTRL_ENABLE_PWM_BURST_EXIT               (1u << 12) /* <12:12> R:RW:0: */


/*
 * pre-scaling of current in ff dac (1: 2.4uA; 0: 1.2uA)
 */
#define PDSS_PWM_0_CTRL_DRV_IDAC_FF                         (1u << 13) /* <13:13> R:RW:1: */


/*
 * selection bits for burst exit reference(threshold)
 */
#define PDSS_PWM_0_CTRL_PWM_BURST_EXIT_SEL_MASK             (0x0003c000) /* <14:17> R:RW:7: */
#define PDSS_PWM_0_CTRL_PWM_BURST_EXIT_SEL_POS              (14)


/*
 * test-mode pins for PWM block: <spare,spare,spare,enabling diode in skip_comparator>
 */
#define PDSS_PWM_0_CTRL_T_PWM_MASK                          (0x003c0000) /* <18:21> R:RW:1: */
#define PDSS_PWM_0_CTRL_T_PWM_POS                           (18)


/*
 * ADFT Control for PWM Block
 */
#define PDSS_PWM_0_CTRL_ADFT_PWM_MASK                       (0x1fc00000) /* <22:28> R:RW:0: */
#define PDSS_PWM_0_CTRL_ADFT_PWM_POS                        (22)


/*
 * S8PDS Hard IP PWM Register 1
 */
#define PDSS_PWM_1_CTRL_ADDRESS                             (0x400a08d8)
#define PDSS_PWM_1_CTRL                                     (*(volatile uint32_t *)(0x400a08d8))
#define PDSS_PWM_1_CTRL_DEFAULT                             (0x00610294)

/*
 * Fixed DAC input
 */
#define PDSS_PWM_1_CTRL_PWM_DAC_MASK                        (0x0000003f) /* <0:5> R:RW:20: */
#define PDSS_PWM_1_CTRL_PWM_DAC_POS                         (0)


/*
 * Feedforward/Vin DAC input
 */
#define PDSS_PWM_1_CTRL_PWM_DAC_FF_MASK                     (0x00000fc0) /* <6:11> R:RW:10: */
#define PDSS_PWM_1_CTRL_PWM_DAC_FF_POS                      (6)


/*
 * hclamp reference selection
 */
#define PDSS_PWM_1_CTRL_PWM_HCLAMP_SEL_MASK                 (0x00030000) /* <16:17> R:RW:1: */
#define PDSS_PWM_1_CTRL_PWM_HCLAMP_SEL_POS                  (16)


/*
 * lclamp reference selection
 */
#define PDSS_PWM_1_CTRL_PWM_LCLAMP_SEL_MASK                 (0x00f00000) /* <20:23> R:RW:6: */
#define PDSS_PWM_1_CTRL_PWM_LCLAMP_SEL_POS                  (20)


/*
 * S8PDS Hard IP SRSENSE Register 0
 */
#define PDSS_SRSNS_0_CTRL_ADDRESS                           (0x400a08dc)
#define PDSS_SRSNS_0_CTRL                                   (*(volatile uint32_t *)(0x400a08dc))
#define PDSS_SRSNS_0_CTRL_DEFAULT                           (0xfc000608)

/*
 * Output isolation control.  Active Low
 * 0: All digital outputs are forced to known value
 */
#define PDSS_SRSNS_0_CTRL_SRSNS_ISO_N                       (1u << 0) /* <0:0> R:RW:0: */


/*
 * Feed-forward voltage sense block enable
 * This should be enabled only for PWM option and should have Firmware option
 * to keep it disabled for SR-Only option.
 */
#define PDSS_SRSNS_0_CTRL_FEEDFWD_EN                        (1u << 1) /* <1:1> R:RW:0: */


/*
 * Feed-forward uv-ov comparator out enable signal
 */
#define PDSS_SRSNS_0_CTRL_FF_UVOV_EN                        (1u << 2) /* <2:2> R:RW:0: */


/*
 * Vbg by R  block enable signal
 */
#define PDSS_SRSNS_0_CTRL_VBGBYR_EN                         (1u << 3) /* <3:3> R:RW:1: */


/*
 * testmode for vbg-by-R block
 * <5> spare
 * <4> half the output current(1.2uA)
 * <3> double the ouput current(4.8uA)
 * <2:0> select vref_peakdet
 * 000-125mV
 * 001-165mV
 * 010-205mV
 * 011-245mV
 * 100-285mV
 * 101-325mV
 * 110-365mV
 * 111-405mV
 */
#define PDSS_SRSNS_0_CTRL_T_VBGBYR_MASK                     (0x00007e00) /* <9:14> R:RW:3: */
#define PDSS_SRSNS_0_CTRL_T_VBGBYR_POS                      (9)


/*
 * ADFT master enable  for srsense block
 */
#define PDSS_SRSNS_0_CTRL_ADFT_EN_SRSENSE                   (1u << 15) /* <15:15> R:RW:0: */


/*
 * ADFT control for drain sensing block
 */
#define PDSS_SRSNS_0_CTRL_ADFT_CTRL_DRAIN_MASK              (0x03ff0000) /* <16:25> R:RW:0: */
#define PDSS_SRSNS_0_CTRL_ADFT_CTRL_DRAIN_POS               (16)


/*
 * ZCDF comparator powerdown signal
 */
#define PDSS_SRSNS_0_CTRL_ZCDF_PD                           (1u << 26) /* <26:26> R:RW:1: */


/*
 * Power down signal for Feedfwd over voltage detection comparator
 */
#define PDSS_SRSNS_0_CTRL_FF_OV_PD                          (1u << 27) /* <27:27> R:RW:1: */


/*
 * Power down signal for Feedfwd under voltage detection comparator
 */
#define PDSS_SRSNS_0_CTRL_FF_UV_PD                          (1u << 28) /* <28:28> R:RW:1: */


/*
 * nsn power down Signal
 */
#define PDSS_SRSNS_0_CTRL_NSN_PD                            (1u << 29) /* <29:29> R:RW:1: */


/*
 * Zcd Power Down Signal
 */
#define PDSS_SRSNS_0_CTRL_ZCD_PD                            (1u << 30) /* <30:30> R:RW:1: */


/*
 * Peak detector power down signal
 * This bit is enabled only for PWM mode.
 * Firmware option to keep it disabled for SR-Only option.
 */
#define PDSS_SRSNS_0_CTRL_PEAKDET_PD                        (1u << 31) /* <31:31> R:RW:1: */


/*
 * S8PDS Hard IP SRSENSE Register 1
 */
#define PDSS_SRSNS_1_CTRL_ADDRESS                           (0x400a08e0)
#define PDSS_SRSNS_1_CTRL                                   (*(volatile uint32_t *)(0x400a08e0))
#define PDSS_SRSNS_1_CTRL_DEFAULT                           (0x020f0000)

/*
 * Testmodes for peak-detector block:
 * 12:introduct 20mV offset for reset comparator
 * 11:powerdown of peakdet amplifier
 * 10:powerdown of peakdet comparator
 * 9:Powerdown of clamp comparator
 * 8: Powerdown of reset comparator
 * 7: reset select between analog (0) /digital (1)
 * 6: Bypass the monoshot gating on clamp comparator default 0
 * 5: mux input to peak-det comparator
 * 4: reserved
 * 3. Disconnect 2M resistor
 * 2: Increase hys to 50mV for peak-det comparator
 * 1:Disable hys for peak-det comparator
 * 0: sr_sen vs GPIO path for peakdet
 */
#define PDSS_SRSNS_1_CTRL_PEAKDET_TEST_MASK                 (0x00001fff) /* <0:12> R:RW:0: */
#define PDSS_SRSNS_1_CTRL_PEAKDET_TEST_POS                  (0)


/*
 * Testmodes for feedfwd block:
 * <15> increase CTAT current trim range
 * <14> force external i_feedfwd through GPIO
 * <13> Enable low current mode iout_vbus current mirror
 * <12> disable VBUS CTAT compensation
 * <11> disable vbus compensation
 * <10:8> select vbus compensation for different external resistor
 * • 000- 10K,9K
 * • 001-8K
 * • 010-7K
 * • 011-6K
 * • 100-5K
 * • 110-4K"
 * <7:6> i_feedfwd multiplied (00 - 1x, 01 - 2x, 10 - 3x, 11- 4x)
 * <5:0>  select "i_feedfwd" out for different turn-ratio
 */
#define PDSS_SRSNS_1_CTRL_FFWD_TEST_MASK                    (0xffff0000) /* <16:31> R:RW:527: */
#define PDSS_SRSNS_1_CTRL_FFWD_TEST_POS                     (16)


/*
 * S8PDS Hard IP SRSENSE Register 2
 */
#define PDSS_SRSNS_2_CTRL_ADDRESS                           (0x400a08e4)
#define PDSS_SRSNS_2_CTRL                                   (*(volatile uint32_t *)(0x400a08e4))
#define PDSS_SRSNS_2_CTRL_DEFAULT                           (0x00000088)

/*
 * Testmodes for negative sensing block:
 * t_nsn<8> 25mV positive ref enable
 * t_nsn<7> 50mV positive ref enable
 * t_nsn<6> 100mV positive ref enable
 * t_nsn<5> spare
 * t_nsn<4> Schmitt trigger mode enable
 * t_nsn<3> Main hysteresis enable
 * t_nsn<2> 80mV hysteresis enable
 * t_nsn<1> 40mV hysteresis disable
 * t_nsn<0> 20mV hysteresis enable
 */
#define PDSS_SRSNS_2_CTRL_TEST_NSN_MASK                     (0x000001ff) /* <0:8> R:RW:136: */
#define PDSS_SRSNS_2_CTRL_TEST_NSN_POS                      (0)


/*
 * Testmodes for SRSENSE block:
 * 5: Reserved
 * 4: Reserved
 * 3: Reserved
 * 2: ovp en_half current mode
 * 1: Disable hysteresis for ZCDF comparator
 * 0: Select 20mV hysteresis for ZCDF comparator (10mV by default)
 */
#define PDSS_SRSNS_2_CTRL_T_SRSENSE_MASK                    (0x00007e00) /* <9:14> R:RW:0: */
#define PDSS_SRSNS_2_CTRL_T_SRSENSE_POS                     (9)


/*
 * S8PDS Hard IP SRSENSE Register 3
 */
#define PDSS_SRSNS_3_CTRL_ADDRESS                           (0x400a08e8)
#define PDSS_SRSNS_3_CTRL                                   (*(volatile uint32_t *)(0x400a08e8))
#define PDSS_SRSNS_3_CTRL_DEFAULT                           (0x00000000)

/*
 * Testmodes for drain sensing block:
 * 7: To enable OVP testing by reducing OVP vref to lower value
 * 6: disable rc network on pad_sr_vss
 * 5: enable RTL based sr_sen_pulldn during OVP
 * 4: enable ff_ov based sr_sen_pulldn during OVP
 * 3: disable comparator based sr_sen_pulldn during OVP
 * 2: disable OVP comparator on pad_sr_sen
 * 1: short diode on pad_sr_vss during ZCD comp trim for offset
 * 0: short diode during 2k trim
 */
#define PDSS_SRSNS_3_CTRL_TEST_DRAIN_MASK                   (0x000000ff) /* <0:7> R:RW:0: */
#define PDSS_SRSNS_3_CTRL_TEST_DRAIN_POS                    (0)


/*
 * Testmodes for ZCD block:
 * t_zcd<7> reserved
 * t_zcd<6> reserved
 * t_zcd<5> to split the bias generation current/half current option of the
 * circuit
 * t_zcd<4> to double the hysteresis
 * t_zcd<3> reserved
 * t_zcd<2> reserved
 * t_zcd<1> hysteresis enable
 * t_zcd<0> doubling the trim network current
 */
#define PDSS_SRSNS_3_CTRL_TEST_ZCD_MASK                     (0x00ff0000) /* <16:23> R:RW:0: */
#define PDSS_SRSNS_3_CTRL_TEST_ZCD_POS                      (16)


/*
 * S8PDS Hard IP GDRV Register 0
 */
#define PDSS_GDRV_0_CTRL_ADDRESS                            (0x400a08ec)
#define PDSS_GDRV_0_CTRL                                    (*(volatile uint32_t *)(0x400a08ec))
#define PDSS_GDRV_0_CTRL_DEFAULT                            (0x10fffba2)

/*
 * This bit is used to enable the Gate driver operation.
 */
#define PDSS_GDRV_0_CTRL_GDRV_EN                            (1u << 0) /* <0:0> R:RW:0: */


/*
 * charge pump enable signal
 */
#define PDSS_GDRV_0_CTRL_CP_EN                              (1u << 1) /* <1:1> R:RW:1: */


/*
 * Enable signal for gdrv, when disabled, gdrv input will be bypassed and
 * gdrv will depend on other control signals
 */
#define PDSS_GDRV_0_CTRL_GDRV_IN_BYPASS                     (1u << 2) /* <2:2> R:RW:0: */


/*
 * set signal for gdrv input in gdrv bypass mode. Gate drive output can be
 * set to always high using this bit  and gdrv_in_set_sel
 */
#define PDSS_GDRV_0_CTRL_GDRV_IN_SET                        (1u << 3) /* <3:3> R:RW:0: */


/*
 * selection signal for gate driver input in gate drive input bypass mode.
 * 0: gdrv_in_set will be used as gate driver input
 * 1: nsn  will be used as gate driver input
 */
#define PDSS_GDRV_0_CTRL_GDRV_IN_SET_SEL                    (1u << 4) /* <4:4> R:RW:0: */


/*
 * reset signal for gdrv input in gdrv bypass mode. Gate drive output can
 * be set to always low using this bit and gdrv_in_reset_sel
 */
#define PDSS_GDRV_0_CTRL_GDRV_IN_RESET                      (1u << 5) /* <5:5> R:RW:1: */


/*
 * selection signal for gate driver input in gate drive input bypass mode.
 * 0: gdrv_in_reset will be used as gate driver input
 * 1: zcd  will be used as gate driver input
 */
#define PDSS_GDRV_0_CTRL_GDRV_IN_RESET_SEL                  (1u << 6) /* <6:6> R:RW:0: */


/*
 * Enable signal for doubler, 1: doubler will be bypassed to vddd. Firmware
 * should make it 0 if doubler needs to be used.
 */
#define PDSS_GDRV_0_CTRL_DOUBLER_BYPASS                     (1u << 7) /* <7:7> R:RW:1: */


/*
 * Skew options for nfet of gate driver
 */
#define PDSS_GDRV_0_CTRL_GDRV_NCONF_MASK                    (0x00000f00) /* <8:11> R:RW:11: */
#define PDSS_GDRV_0_CTRL_GDRV_NCONF_POS                     (8)


/*
 * skew options for nfet of doubler
 */
#define PDSS_GDRV_0_CTRL_GDRV_NCONF_BOT_MASK                (0x0000f000) /* <12:15> R:RW:15: */
#define PDSS_GDRV_0_CTRL_GDRV_NCONF_BOT_POS                 (12)


/*
 * skew options for boot-strap pfet of doubler
 */
#define PDSS_GDRV_0_CTRL_GDRV_PCONF_BOT_MASK                (0x000f0000) /* <16:19> R:RW:15: */
#define PDSS_GDRV_0_CTRL_GDRV_PCONF_BOT_POS                 (16)


/*
 * skew options :
 * <31:30> : Spare
 * <29:28> : Resistive divider for Pump to change pump voltage (default 01)
 * <27:26> : Spare
 * <25:24> : Pump oscillator frequency change
 * <23:20>: Slew control for pfet vddd switch (Default : 1111)
 */
#define PDSS_GDRV_0_CTRL_TRIM_SRGDRV_MASK                   (0xfff00000) /* <20:31> R:RW:271: */
#define PDSS_GDRV_0_CTRL_TRIM_SRGDRV_POS                    (20)


/*
 * S8PDS Hard IP GDRV Register 1
 */
#define PDSS_GDRV_1_CTRL_ADDRESS                            (0x400a08f0)
#define PDSS_GDRV_1_CTRL                                    (*(volatile uint32_t *)(0x400a08f0))
#define PDSS_GDRV_1_CTRL_DEFAULT                            (0x00000124)

/*
 * Testmodes for Gate-driver:
 * <21:19>  : spare
 * <18>       : ext_lk_sel for charge pump
 * <17>       : make all pads tristate
 * <16>       : disable pwrup_nreset
 * <15>       : bypass vdd comparator to delay based
 * <14>       : minimal or zero delay of bypassed vdd comparator
 * <13:12>  : control the delay of bypassed vdd comparator
 * <11>       : bypass the delay between ngdrv_bot en to pd_en
 *                  1 : delay  between ngdrv_bot_en to pd_en reduces the
 * average current consumption
 *                  0 : delay will be bypassed and pull down will be faster
 * <10:9>    : controls the delay between ngdrv_bot en to pd_en
 * <8>         : bypass the slewing option for pgdrv_bot en
 *                 1 : slew will be present between pgdrv_bot_en legs
 *                 0 : no slew will be present.
 * <7:6>      : controls the  slew delay for pgdrv_bot en
 * <5>         : bypass the slewing option for ngdrv_bot en
 *                 1 : slew will be present between ngdrv_bot_en legs
 *                 0 : no slew will be present.
 * <4:3>      : controls the  slew delay for ngdrv_bot en
 * <2>         : bypass the slewing option for pd
 *                 1 : slew will be present between nfet pd legs
 *                 0 : no slew will be present.
 * <1:0>      : controls the  slew delay for nfet pd legs
 */
#define PDSS_GDRV_1_CTRL_T_SRGDRV_MASK                      (0x003fffff) /* <0:21> R:RW:292: */
#define PDSS_GDRV_1_CTRL_T_SRGDRV_POS                       (0)


/*
 * adft control signal for gate-driver block
 */
#define PDSS_GDRV_1_CTRL_ADFT_CTRL_SRGDRV_MASK              (0x03c00000) /* <22:25> R:RW:0: */
#define PDSS_GDRV_1_CTRL_ADFT_CTRL_SRGDRV_POS               (22)


/*
 * PTDRV_IN pulldown enable for start/stop sequence  (0:disable, 1: enable)
 */
#define PDSS_GDRV_1_CTRL_PTDRV_PULLDN_EN                    (1u << 26) /* <26:26> R:RW:0: */


/*
 * PTDRV_IN pullup enable for start/stop sequence (0:disable, 1: enable)
 */
#define PDSS_GDRV_1_CTRL_PTDRV_PULLUP_EN                    (1u << 27) /* <27:27> R:RW:0: */


/*
 * Master adft enable signal for gate-driver block
 */
#define PDSS_GDRV_1_CTRL_ADFT_EN_SRGDRV                     (1u << 31) /* <31:31> R:RW:0: */


/*
 * S8PDS Hard IP Error Amplifier Register 0
 */
#define PDSS_EA_CTRL_ADDRESS                                (0x400a08f4)
#define PDSS_EA_CTRL                                        (*(volatile uint32_t *)(0x400a08f4))
#define PDSS_EA_CTRL_DEFAULT                                (0x00400004)

/*
 * Enable signal for constant current path
 * 0 - Disable
 * 1 - Enable
 */
#define PDSS_EA_CTRL_EN_CC                                  (1u << 0) /* <0:0> R:RW:0: */


/*
 * Enable signal for IDAC
 * 0 - Disable
 * 1 - Enable
 */
#define PDSS_EA_CTRL_EN_CV                                  (1u << 1) /* <1:1> R:RW:0: */


/*
 * Enable signal for CV AMP
 * 0 - Disable
 * 1 - Enable
 */
#define PDSS_EA_CTRL_EN_SHNT                                (1u << 2) /* <2:2> R:RW:1: */


/*
 * Enable signal for current sink DAC
 * 0 - Disable
 * 1 - Enable
 */
#define PDSS_EA_CTRL_ISNK_EN                                (1u << 3) /* <3:3> R:RW:0: */


/*
 * Enable signal for current source DAC
 * 0 - Disable
 * 1 - Enable
 */
#define PDSS_EA_CTRL_ISRC_EN                                (1u << 4) /* <4:4> R:RW:0: */


/*
 * 10-bit DAC control for current sink
 */
#define PDSS_EA_CTRL_ISNK_DAC_CTRL_MASK                     (0x00007fe0) /* <5:14> R:RW:0: */
#define PDSS_EA_CTRL_ISNK_DAC_CTRL_POS                      (5)


/*
 * 7-bit DAC control for current source
 */
#define PDSS_EA_CTRL_ISRC_DAC_CTRL_MASK                     (0x003f8000) /* <15:21> R:RW:0: */
#define PDSS_EA_CTRL_ISRC_DAC_CTRL_POS                      (15)


/*
 * Reference current selection for Constant voltage DAC
 * 00 - Deep sleep Refgen current
 * 01 - Internal Vgb/R current
 * 10 - BG current
 * 11 - Not used
 */
#define PDSS_EA_CTRL_EA_IREF_SEL_MASK                       (0x00c00000) /* <22:23> R:RW:1: */
#define PDSS_EA_CTRL_EA_IREF_SEL_POS                        (22)


/*
 * VBUS Internal resistor divider bypass enable
 * 0 - Resistor divider is included
 * 1 - Resistor divider is bypassed
 */
#define PDSS_EA_CTRL_RES_DIV_BYPASS                         (1u << 27) /* <27:27> R:RW:0: */


/*
 * Enable signal for CV AMP to work in low VDDD range
 * 1 - Disable
 * 0 - Enable
 */
#define PDSS_EA_CTRL_SHNT_ST_OPAMP_ENB                      (1u << 28) /* <28:28> R:RW:0: */


/*
 * Powers down the block, active high
 */
#define PDSS_EA_CTRL_EA_PD                                  (1u << 31) /* <31:31> R:RW:0: */


/*
 * S8PDS Hard IP Error Amplifier Register 1
 */
#define PDSS_PDS_EA_1_CTRL_ADDRESS                          (0x400a08f8)
#define PDSS_PDS_EA_1_CTRL                                  (*(volatile uint32_t *)(0x400a08f8))
#define PDSS_PDS_EA_1_CTRL_DEFAULT                          (0x00002460)

/*
 * Select signal for shunt regulator to work in srss or dpslp current
 * 0 - CV AMP work with srss current
 * 1 - CV AMP work with dpslp current
 */
#define PDSS_PDS_EA_1_CTRL_EN_SHNT_DPSLP                    (1u << 0) /* <0:0> R:RW:0: */


/*
 * This pin is used to enable cc_comp_out_hv output which is same as ea_out_pwm
 * pin.
 */
#define PDSS_PDS_EA_1_CTRL_EN_CC_COMP                       (1u << 1) /* <1:1> R:RW:0: */


/*
 * To enable CC Flag circuit
 */
#define PDSS_PDS_EA_1_CTRL_EN_CC_CV_FLAG                    (1u << 2) /* <2:2> R:RW:0: */


/*
 * Enable signal for PWM mode
 * 1 - Error Amplifier supports PWM mode
 * 0 - Error Amplifier supports OPTO mode
 */
#define PDSS_PDS_EA_1_CTRL_EN_PWM_MODE                      (1u << 3) /* <3:3> R:RW:0: */


/*
 * This pin is used to tristate pad_esd_cc_bufout (look at T_EA<2> as well)
 * en_cc=0, en_tristate=x - buffer is disabled and output is pulled to 0V
 * en_cc=1, en_tristate=0 - buffer is enabled
 * en_cc=1, en_tristate=1 -  buffer is disabled and output is pulled to hiZ
 */
#define PDSS_PDS_EA_1_CTRL_EN_CCBUFOUT_TRISTATE             (1u << 4) /* <4:4> R:RW:0: */


/*
 * To select ccdet in cc_flag output:
 * 0: cc_flag output will not include cc_det
 * 1: cc_flag output will include cc_det
 */
#define PDSS_PDS_EA_1_CTRL_SEL_CCDET                        (1u << 5) /* <5:5> R:RW:1: */


/*
 * To select cvdet in cc_flag output:
 * 0: cc_flag output will not include cv_det_n
 * 1: cc_flag output will include cv_det_n
 */
#define PDSS_PDS_EA_1_CTRL_SEL_CVDET                        (1u << 6) /* <6:6> R:RW:1: */


/*
 * Output isolation control.  Active Low
 * 0: All digital outputs are forced to known value
 */
#define PDSS_PDS_EA_1_CTRL_EA_ISO_N                         (1u << 7) /* <7:7> R:RW:0: */


/*
 * trims to vary CC error amp input bias current.
 */
#define PDSS_PDS_EA_1_CTRL_TR_CC_ERR_AMP_IBIAS_MASK         (0x00000700) /* <8:10> R:RW:4: */
#define PDSS_PDS_EA_1_CTRL_TR_CC_ERR_AMP_IBIAS_POS          (8)


/*
 * trims to vary CV error amp input bias current.
 */
#define PDSS_PDS_EA_1_CTRL_TR_CV_ERR_AMP_IBIAS_MASK         (0x00003800) /* <11:13> R:RW:4: */
#define PDSS_PDS_EA_1_CTRL_TR_CV_ERR_AMP_IBIAS_POS          (11)


/*
 * test mode options:
 * t_ea<14:4>: not used.
 *
 * t_ea<3>: 0 -> pad_cc_compn is isolated from EA.
 *   1 -> enables path from the inp terminal of CC AMP to pad_cc_compn.
 *
 * t_ea<2>: 0 -> pad_esd_cc_bufout is isolated from EA.
 *    1 -> enables path from the output of csainp_ugbuf to pad_esd_cc_bufout.
 *
 * t_ea<1>: 0 -> resistance between output of csainp_ugbuf and adft_out1(amuxbus_a)
 * is 3K?.
 *                   0 -> resistance between output of csainp_ugbuf and pad_esd_cc_bufout
 * is 3K?.
 *   1 ->  Test mode to increase resistance between output of csainp_ugbuf
 * and adft_out1(amuxbus_a) to 6K from 3K.
 *                  1 -> Test mode to increase resistance between output
 * of csainp_ugbuf and pad_esd_cc_bufout to 6K from 3K.
 *
 * t_ea<0>: To enable 1pF cap parallel to 200K of vbus resistor divider.
 */
#define PDSS_PDS_EA_1_CTRL_T_EA_MASK                        (0x7fff0000) /* <16:30> R:RW:0: */
#define PDSS_PDS_EA_1_CTRL_T_EA_POS                         (16)


/*
 * S8PDS Hard IP Error Amplifier Register 2
 */
#define PDSS_PDS_EA_2_CTRL_ADDRESS                          (0x400a08fc)
#define PDSS_PDS_EA_2_CTRL                                  (*(volatile uint32_t *)(0x400a08fc))
#define PDSS_PDS_EA_2_CTRL_DEFAULT                          (0x00422402)

/*
 * trims to vary CC_DET current
 */
#define PDSS_PDS_EA_2_CTRL_TR_CC_CV_FLAG_CCDET_MASK         (0x0000001f) /* <0:4> R:RW:2: */
#define PDSS_PDS_EA_2_CTRL_TR_CC_CV_FLAG_CCDET_POS          (0)


/*
 * trims to vary Bandwidth of CC input filter
 */
#define PDSS_PDS_EA_2_CTRL_TR_CCIN_RCFILTER_BW_MASK         (0x00000f00) /* <8:11> R:RW:4: */
#define PDSS_PDS_EA_2_CTRL_TR_CCIN_RCFILTER_BW_POS          (8)


/*
 * trims to vary pwm load current.
 * 000 - 0uA,     001 - 50uA,   010 - 75uA,    011 - 125uA,
 * 100 - 150uA, 101 - 200uA, 110 – 225uA, 111 - 275uA.
 */
#define PDSS_PDS_EA_2_CTRL_TR_ERR_AMP_PWM_ILOAD_MASK        (0x00007000) /* <12:14> R:RW:2: */
#define PDSS_PDS_EA_2_CTRL_TR_ERR_AMP_PWM_ILOAD_POS         (12)


/*
 * trims to vary CV_DET current
 */
#define PDSS_PDS_EA_2_CTRL_TR_CC_CV_FLAG_CVDET_MASK         (0x001f0000) /* <16:20> R:RW:2: */
#define PDSS_PDS_EA_2_CTRL_TR_CC_CV_FLAG_CVDET_POS          (16)


/*
 * trims to vary CC Buffer final stage source current
 */
#define PDSS_PDS_EA_2_CTRL_TR_CC_CSAINP_UGBUF_ILOAD_MASK    (0x00600000) /* <21:22> R:RW:2: */
#define PDSS_PDS_EA_2_CTRL_TR_CC_CSAINP_UGBUF_ILOAD_POS     (21)


/*
 * Control bits for the analog test mux
 * 0000-Normal operation adft_out's are hiZ
 */
#define PDSS_PDS_EA_2_CTRL_PDS_EA_ADFT_MASK                 (0x07800000) /* <23:26> R:RW:0: */
#define PDSS_PDS_EA_2_CTRL_PDS_EA_ADFT_POS                  (23)


/*
 * S8PDS Hard IP NGDO Register
 */
#define PDSS_NGDO_CTRL_ADDRESS                              (0x400a0900)
#define PDSS_NGDO_CTRL                                      (*(volatile uint32_t *)(0x400a0900))
#define PDSS_NGDO_CTRL_DEFAULT                              (0x00010300)

/*
 * Output isolation control.  Active Low
 * 0: All digital outputs are forced to known value
 */
#define PDSS_NGDO_CTRL_NGDO_ISO_N                           (1u << 0) /* <0:0> R:RW:0: */


/*
 * Enable signal for the bias circuitry, comparator and slew rate block
 * ngdo_en = 0 ; disabled
 * ngdo_en = 1 ; enabled
 */
#define PDSS_NGDO_CTRL_NGDO_EN_LV                           (1u << 1) /* <1:1> R:RW:0: */


/*
 * Optioned out bit in case required in future for clamp disable (not used
 * internally)
 */
#define PDSS_NGDO_CTRL_NGDO_CLAMP_EN                        (1u << 2) /* <2:2> R:RW:0: */


/*
 * Charge-pump ouptut control;
 * cp_en = 0 ; cp output = vbus
 * cp_en = 1 ; cp output = driven by cp action
 */
#define PDSS_NGDO_CTRL_NGDO_CP_EN                           (1u << 3) /* <3:3> R:RW:0: */


/*
 * Enable signal for adft.
 * adft_en = 0 ; ADFT disabled, normal functionality
 * adft_en = 1 ; ADFT network enabled
 */
#define PDSS_NGDO_CTRL_NGDO_ADFT_EN                         (1u << 4) /* <4:4> R:RW:0: */


/*
 * Current programmibility for shv vclamp voltage (default 00 = 9V)
 */
#define PDSS_NGDO_CTRL_NGDO_CLAMP_CTRL_MASK                 (0x00000060) /* <5:6> R:RW:0: */
#define PDSS_NGDO_CTRL_NGDO_CLAMP_CTRL_POS                  (5)


/*
 * Current programmibility to adjust comparator offset (default 10=8V)
 */
#define PDSS_NGDO_CTRL_NGDO_CP_CTRL_MASK                    (0x00000180) /* <7:8> R:RW:2: */
#define PDSS_NGDO_CTRL_NGDO_CP_CTRL_POS                     (7)


/*
 * Test mode bits for ngdo : <spare,spare,spare, extra charging current for
 * 0.3uA>
 */
#define PDSS_NGDO_CTRL_NGDO_TM_NGDO_MASK                    (0x00001e00) /* <9:12> R:RW:1: */
#define PDSS_NGDO_CTRL_NGDO_TM_NGDO_POS                     (9)


/*
 * ADFT selection pins.
 */
#define PDSS_NGDO_CTRL_NGDO_DFT_MASK                        (0x0000e000) /* <13:15> R:RW:0: */
#define PDSS_NGDO_CTRL_NGDO_DFT_POS                         (13)


/*
 * Programmibility for external nfet gate charging current (default 000001=
 * 0.15u)
 */
#define PDSS_NGDO_CTRL_NGDO_SLEW_CTRL_MASK                  (0x003f0000) /* <16:21> R:RW:1: */
#define PDSS_NGDO_CTRL_NGDO_SLEW_CTRL_POS                   (16)


/*
 * Test mode bits for bias control : <spare,spare,spare>
 */
#define PDSS_NGDO_CTRL_NGDO_BIAS_CTRL_MASK                  (0x01c00000) /* <22:24> R:RW:0: */
#define PDSS_NGDO_CTRL_NGDO_BIAS_CTRL_POS                   (22)


/*
 * Current programmability for Vbg/R block (default 00 = 2.4uA)
 */
#define PDSS_NGDO_CTRL_VBGBYR_CTRL_MASK                     (0x06000000) /* <25:26> R:RW:0: */
#define PDSS_NGDO_CTRL_VBGBYR_CTRL_POS                      (25)


/*
 * Testmode to enable discharge on pmos gate in slew control block with gdrv_en
 */
#define PDSS_NGDO_CTRL_SLEW_P_GATE_DSCHG                    (1u << 27) /* <27:27> R:RW:0: */


/*
 * S8PDS 20V Regulator
 */
#define PDSS_PDS_VREG_CTRL_ADDRESS                          (0x400a0904)
#define PDSS_PDS_VREG_CTRL                                  (*(volatile uint32_t *)(0x400a0904))
#define PDSS_PDS_VREG_CTRL_DEFAULT                          (0x00000400)

/*
 * adft control inputs used to select different analog signal to be brought
 * out on adft1 and adft2
 * 00 - Normal Operation
 * 01 – vreg_en_vddd on adft1, vbus_good_vcrude on adft2
 * 10 - vfb on adft1 and vreg_cr on adft2
 * 11 – vref_0p74 on adft1 and vreg_en_vcr on adft2
 */
#define PDSS_PDS_VREG_CTRL_PDS_ADFT_CTRL_MASK               (0x00000003) /* <0:1> R:RW:0: */
#define PDSS_PDS_VREG_CTRL_PDS_ADFT_CTRL_POS                (0)


/*
 * Output isolation control.  Active Low
 * 0: All digital outputs are forced to known value
 */
#define PDSS_PDS_VREG_CTRL_VREG_ISO_N                       (1u << 2) /* <2:2> R:RW:0: */


/*
 * Reference voltage select signal
 * 0 - Use 0.74V dpslp reference
 * 1 - Use 0.74V dervived from bandgap reference
 */
#define PDSS_PDS_VREG_CTRL_VBG_EN                           (1u << 8) /* <8:8> R:RW:0: */


/*
 * Test-mopde to change the regulator output to 3.3V (default 0 : 5V)
 */
#define PDSS_PDS_VREG_CTRL_T_VREG_3P3                       (1u << 9) /* <9:9> R:RW:0: */


/*
 * regulator enable signal
 */
#define PDSS_PDS_VREG_CTRL_PDS_VREG_EN                      (1u << 10) /* <10:10> R:RW:1: */


/*
 * NGate driver config1
 */
#define PDSS_NGDO_1_CFG_ADDRESS                             (0x400a2000)
#define PDSS_NGDO_1_CFG                                     (*(volatile uint32_t *)(0x400a2000))
#define PDSS_NGDO_1_CFG_DEFAULT                             (0x00000000)

/*
 * The gate driver control option.
 * 0: FW controlls the gdrv_en  pin
 * 1: HW controlls the  gdrv_en  pin
 */
#define PDSS_NGDO_1_CFG_AUTO_MODE                           (1u << 0) /* <0:0> R:RW:0: */


/*
 * The gate driver control option.
 * Any write-one to this register will reset the edge detector in the controller.
 * FW should cleared this register after the fault conditions are removed
 * by writing a “1” to this register.
 */
#define PDSS_NGDO_1_CFG_RST_EDGE_DET                        (1u << 1) /* <1:1> R:RW:0: */


/*
 * CPU can used this register to inform the hardware to drive the ON value
 * or OFF value when a fault is detected.
 * 0: Select OFF
 * 1: Select ON
 */
#define PDSS_NGDO_1_CFG_SEL_ON_OFF                          (1u << 5) /* <5:5> R:RW:0: */


/*
 * Bit[18]:
 * 0: FILT20 detection is not selected for turning off the gdrv_en
 * 1: FILT20 detection is       selected for turning off the gdrv_en
 * Bit[19]:
 * 0: FILT21 detection is not selected for turning off the gdrv_en
 * 1: FILT21 detection is       selected for turning off the gdrv_en
 */
#define PDSS_NGDO_1_CFG_SEL_FILT2_MASK                      (0x000000c0) /* <6:7> R:RW:0: */
#define PDSS_NGDO_1_CFG_SEL_FILT2_POS                       (6)


/*
 * The Off value used by Hardware in Automode to turn off the gdrv_en bit
 */
#define PDSS_NGDO_1_CFG_GDRV_EN_OFF_VALUE                   (1u << 17) /* <17:17> R:RW:0: */


/*
 * The ON value used by Hardware to turn on the gdrv_en bit
 */
#define PDSS_NGDO_1_CFG_GDRV_EN_ON_VALUE                    (1u << 18) /* <18:18> R:RW:0: */


/*
 * 0: zcd_out detection is not selected for turning off the gdrv_en
 * 1: zcd_out detection is       selected for turning off the gdrv_en
 */
#define PDSS_NGDO_1_CFG_SEL_ZCD_OUT                         (1u << 19) /* <19:19> R:RW:0: */


/*
 * 0: nsn_out detection is not selected for turning off the gdrv_en
 * 1: nsn_out detection is       selected for turning off the gdrv_en
 */
#define PDSS_NGDO_1_CFG_SEL_NSN_OUT                         (1u << 20) /* <20:20> R:RW:0: */


/*
 * 0: pwm_out detection is not selected for turning off the gdrv_en
 * 1: pwm_out detection is       selected for turning off the gdrv_en
 */
#define PDSS_NGDO_1_CFG_SEL_PWM_OUT                         (1u << 21) /* <21:21> R:RW:0: */


/*
 * 0: burst_exit_out detection is not selected for turning off the gdrv_en
 * 1: burst_exit_out detection is       selected for turning off the gdrv_en
 */
#define PDSS_NGDO_1_CFG_SEL_BURST_EXIT_OUT                  (1u << 22) /* <22:22> R:RW:0: */


/*
 * 0: sr_sen_ovp_out detection is not selected for turning off the gdrv_en
 * 1: sr_sen_ovp_out detection is       selected for turning off the gdrv_en
 */
#define PDSS_NGDO_1_CFG_SEL_SR_SEN_OVP_OUT                  (1u << 23) /* <23:23> R:RW:0: */


/*
 * 0: skip_out detection is not selected for turning off the gdrv_en
 * 1: skip_out detection is       selected for turning off the gdrv_en
 */
#define PDSS_NGDO_1_CFG_SEL_SKIP_OUT                        (1u << 24) /* <24:24> R:RW:0: */


/*
 * 0: peakdet_out detection is not selected for turning off the gdrv_en
 * 1: peakdet_out detection is       selected for turning off the gdrv_en
 */
#define PDSS_NGDO_1_CFG_SEL_PEAKDET_OUT                     (1u << 25) /* <25:25> R:RW:0: */


/*
 * 0: peakdet_rst_out detection is not selected for turning off the gdrv_en
 * 1: peakdet_rst_out detection is       selected for turning off the gdrv_en
 */
#define PDSS_NGDO_1_CFG_SEL_PEAKDET_RST_OUT                 (1u << 26) /* <26:26> R:RW:0: */


/*
 * 0: peakdet_clcmp_raw_out detection is not selected for turning off the
 * gdrv_en
 * 1: peakdet_clcmp_raw_out detection is       selected for turning off the
 * gdrv_en
 */
#define PDSS_NGDO_1_CFG_SEL_PEAKDET_CLCMP_RAW_OUT           (1u << 27) /* <27:27> R:RW:0: */


/*
 * 0: zcdf_out detection is not selected for turning off the gdrv_en
 * 1: zcdf_out detection is       selected for turning off the gdrv_en
 */
#define PDSS_NGDO_1_CFG_SEL_ZCDF_OUT                        (1u << 28) /* <28:28> R:RW:0: */


/*
 * 0: SCP detection is not selected for turning off the gdrv_en
 * 1: SCP detection is       selected for turning off the gdrv_en
 */
#define PDSS_NGDO_1_CFG_SEL_PDS_SCP                         (1u << 29) /* <29:29> R:RW:0: */


/*
 * 0: FF_UV detection is not selected for turning off the gdrv_en
 * 1:FF_UV detection is       selected for turning off the gdrv_en
 */
#define PDSS_NGDO_1_CFG_SEL_FF_UV                           (1u << 30) /* <30:30> R:RW:0: */


/*
 * 0: FF_OV detection is not selected for turning off the gdrv_en
 * 1: FF_OV detection is       selected for turning off the gdrv_en
 */
#define PDSS_NGDO_1_CFG_SEL_FF_OV                           (1u << 31) /* <31:31> R:RW:0: */


/*
 * NGate driver config2
 */
#define PDSS_NGDO_2_CFG_ADDRESS                             (0x400a2004)
#define PDSS_NGDO_2_CFG                                     (*(volatile uint32_t *)(0x400a2004))
#define PDSS_NGDO_2_CFG_DEFAULT                             (0x00000000)

/*
 * There can be a maximum of 24 HS filter.
 * This register can be use to select any of the comparators output for turning
 * on/off the function
 */
#define PDSS_NGDO_2_CFG_HS_SOURCE_SEL_MASK                  (0x0000003f) /* <0:5> R:RW:0:CLK_FILTER_FILT_NUM */
#define PDSS_NGDO_2_CFG_HS_SOURCE_SEL_POS                   (0)


/*
 * There can be a maximum of 8 LS filter.
 * This register can be use to select any of the comparators output for turning
 * on/off the function.
 */
#define PDSS_NGDO_2_CFG_LS_SOURCE_SEL_MASK                  (0x07000000) /* <24:26> R:RW:0:CLK_LF_FILT_NUM */
#define PDSS_NGDO_2_CFG_LS_SOURCE_SEL_POS                   (24)


/*
 * SRSENSE gate driver config1
 */
#define PDSS_SRSNS_1_CFG_ADDRESS                            (0x400a2008)
#define PDSS_SRSNS_1_CFG                                    (*(volatile uint32_t *)(0x400a2008))
#define PDSS_SRSNS_1_CFG_DEFAULT                            (0x00000000)

/*
 * The gate driver control option.
 * 0: FW controlls the sr_sen_pulldn_en  pin
 * 1: HW controlls the  sr_sen_pulldn_en  pin
 */
#define PDSS_SRSNS_1_CFG_AUTO_MODE                          (1u << 0) /* <0:0> R:RW:0: */


/*
 * The gate driver control option.
 * Any write-one to this register will reset the edge detector in the controller.
 * FW should cleared this register after the fault conditions are removed
 * by writing a “1” to this register.
 */
#define PDSS_SRSNS_1_CFG_RST_EDGE_DET                       (1u << 1) /* <1:1> R:RW:0: */


/*
 * CPU can used this register to inform the hardware to drive the ON value
 * or OFF value when a fault is detected.
 * 0: Select OFF
 * 1: Select ON
 */
#define PDSS_SRSNS_1_CFG_SEL_ON_OFF                         (1u << 5) /* <5:5> R:RW:0: */


/*
 * Bit[18]:
 * 0: FILT20 detection is not selected for turning off the sr_sen_pulldn_en
 * 1: FILT20 detection is       selected for turning off the sr_sen_pulldn_en
 * Bit[19]:
 * 0: FILT21 detection is not selected for turning off the sr_sen_pulldn_en
 * 1: FILT21 detection is       selected for turning off the sr_sen_pulldn_en
 */
#define PDSS_SRSNS_1_CFG_SEL_FILT2_MASK                     (0x000000c0) /* <6:7> R:RW:0: */
#define PDSS_SRSNS_1_CFG_SEL_FILT2_POS                      (6)


/*
 * The Off value used by Hardware in Automode to turn off the sr_sen_pulldn_en
 * bit
 */
#define PDSS_SRSNS_1_CFG_PULLDN_EN_OFF_VALUE                (1u << 17) /* <17:17> R:RW:0: */


/*
 * The ON value used by Hardware to turn on the sr_sen_pulldn_en bit
 */
#define PDSS_SRSNS_1_CFG_PULLDN_EN_ON_VALUE                 (1u << 18) /* <18:18> R:RW:0: */


/*
 * 0: zcd_out detection is not selected for turning off the sr_sen_pulldn_en
 * 1: zcd_out detection is       selected for turning off the sr_sen_pulldn_en
 */
#define PDSS_SRSNS_1_CFG_SEL_ZCD_OUT                        (1u << 19) /* <19:19> R:RW:0: */


/*
 * 0: nsn_out detection is not selected for turning off the sr_sen_pulldn_en
 * 1: nsn_out detection is       selected for turning off the sr_sen_pulldn_en
 */
#define PDSS_SRSNS_1_CFG_SEL_NSN_OUT                        (1u << 20) /* <20:20> R:RW:0: */


/*
 * 0: pwm_out detection is not selected for turning off the sr_sen_pulldn_en
 * 1: pwm_out detection is       selected for turning off the sr_sen_pulldn_en
 */
#define PDSS_SRSNS_1_CFG_SEL_PWM_OUT                        (1u << 21) /* <21:21> R:RW:0: */


/*
 * 0: burst_exit_out detection is not selected for turning off the sr_sen_pulldn_en
 * 1: burst_exit_out detection is       selected for turning off the sr_sen_pulldn_en
 */
#define PDSS_SRSNS_1_CFG_SEL_BURST_EXIT_OUT                 (1u << 22) /* <22:22> R:RW:0: */


/*
 * 0: sr_sen_ovp_out detection is not selected for turning off the sr_sen_pulldn_en
 * 1: sr_sen_ovp_out detection is       selected for turning off the sr_sen_pulldn_en
 */
#define PDSS_SRSNS_1_CFG_SEL_SR_SEN_OVP_OUT                 (1u << 23) /* <23:23> R:RW:0: */


/*
 * 0: skip_out detection is not selected for turning off the sr_sen_pulldn_en
 * 1: skip_out detection is       selected for turning off the sr_sen_pulldn_en
 */
#define PDSS_SRSNS_1_CFG_SEL_SKIP_OUT                       (1u << 24) /* <24:24> R:RW:0: */


/*
 * 0: peakdet_out detection is not selected for turning off the sr_sen_pulldn_en
 * 1: peakdet_out detection is       selected for turning off the sr_sen_pulldn_en
 */
#define PDSS_SRSNS_1_CFG_SEL_PEAKDET_OUT                    (1u << 25) /* <25:25> R:RW:0: */


/*
 * 0: peakdet_rst_out detection is not selected for turning off the sr_sen_pulldn_en
 * 1: peakdet_rst_out detection is       selected for turning off the sr_sen_pulldn_en
 */
#define PDSS_SRSNS_1_CFG_SEL_PEAKDET_RST_OUT                (1u << 26) /* <26:26> R:RW:0: */


/*
 * 0: peakdet_clcmp_raw_out detection is not selected for turning off the
 * sr_sen_pulldn_en
 * 1: peakdet_clcmp_raw_out detection is       selected for turning off the
 * sr_sen_pulldn_en
 */
#define PDSS_SRSNS_1_CFG_SEL_PEAKDET_CLCMP_RAW_OUT          (1u << 27) /* <27:27> R:RW:0: */


/*
 * 0: zcdf_out detection is not selected for turning off the sr_sen_pulldn_en
 * 1: zcdf_out detection is       selected for turning off the sr_sen_pulldn_en
 */
#define PDSS_SRSNS_1_CFG_SEL_ZCDF_OUT                       (1u << 28) /* <28:28> R:RW:0: */


/*
 * 0: SCP detection is not selected for turning off the sr_sen_pulldn_en
 * 1: SCP detection is       selected for turning off the sr_sen_pulldn_en
 */
#define PDSS_SRSNS_1_CFG_SEL_PDS_SCP                        (1u << 29) /* <29:29> R:RW:0: */


/*
 * 0: FF_UV detection is not selected for turning off the sr_sen_pulldn_en
 * 1:FF_UV detection is       selected for turning off the sr_sen_pulldn_en
 */
#define PDSS_SRSNS_1_CFG_SEL_FF_UV                          (1u << 30) /* <30:30> R:RW:0: */


/*
 * 0: FF_OV detection is not selected for turning off the sr_sen_pulldn_en
 * 1: FF_OV detection is       selected for turning off the sr_sen_pulldn_en
 */
#define PDSS_SRSNS_1_CFG_SEL_FF_OV                          (1u << 31) /* <31:31> R:RW:0: */


/*
 * SRSENSE gate driver config2
 */
#define PDSS_SRSNS_2_CFG_ADDRESS                            (0x400a200c)
#define PDSS_SRSNS_2_CFG                                    (*(volatile uint32_t *)(0x400a200c))
#define PDSS_SRSNS_2_CFG_DEFAULT                            (0x00000000)

/*
 * There can be a maximum of 24 HS filter.
 * This register can be use to select any of the comparators output for turning
 * on/off the function
 */
#define PDSS_SRSNS_2_CFG_HS_SOURCE_SEL_MASK                 (0x0000003f) /* <0:5> R:RW:0:CLK_FILTER_FILT_NUM */
#define PDSS_SRSNS_2_CFG_HS_SOURCE_SEL_POS                  (0)


/*
 * There can be a maximum of 8 LS filter.
 * This register can be use to select any of the comparators output for turning
 * on/off the function.
 */
#define PDSS_SRSNS_2_CFG_LS_SOURCE_SEL_MASK                 (0x07000000) /* <24:26> R:RW:0:CLK_LF_FILT_NUM */
#define PDSS_SRSNS_2_CFG_LS_SOURCE_SEL_POS                  (24)


/*
 * SRSENSE HardIP High/Low Speed Filter and Edge detector configuration for
 * SCP_OUT detection
 */
#define PDSS_INTR15_CFG_PDS_SCP_ADDRESS                     (0x400a2010)
#define PDSS_INTR15_CFG_PDS_SCP                             (*(volatile uint32_t *)(0x400a2010))
#define PDSS_INTR15_CFG_PDS_SCP_DEFAULT                     (0x00000400)

/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR15_CFG_PDS_SCP_SCP_OUT_FILT_EN             (1u << 0) /* <0:0> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR15_CFG_PDS_SCP_SCP_OUT_FILT_CFG_MASK       (0x00000006) /* <1:2> R:RW:0: */
#define PDSS_INTR15_CFG_PDS_SCP_SCP_OUT_FILT_CFG_POS        (1)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR15_CFG_PDS_SCP_SCP_OUT_FILT_RESET          (1u << 3) /* <3:3> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR15_CFG_PDS_SCP_SCP_OUT_FILT_BYPASS         (1u << 4) /* <4:4> R:RW:0: */


/*
 * #of clock CLK_FILT[0] filtering. Should be programmed before FILTER is
 * enabled.
 */
#define PDSS_INTR15_CFG_PDS_SCP_SCP_OUT_FILT_SEL_MASK       (0x000003e0) /* <5:9> R:RW:0: */
#define PDSS_INTR15_CFG_PDS_SCP_SCP_OUT_FILT_SEL_POS        (5)


/*
 * This bit enables CPU to bypass the Filter when the part is in deep-sleep
 * state.  This over-rides the FILTER_EN settings during DEEP SLEEP state
 * ONLY.
 * The bit should be set by CPU only when its using the filter with high
 * frequency active clock and wants to wakeup from deep sleep on the transition
 * of the incoming signal.
 * This bit if set also disables the AUTO shutoff logic in DEEP SLEEP state.
 */
#define PDSS_INTR15_CFG_PDS_SCP_SCP_OUT_DPSLP_MODE          (1u << 10) /* <10:10> R:RW:1: */


/*
 * SRSENSE HardIP CLK_LF Filter and Edge detector configuration for FF_UV
 * and FF_OV detection
 */
#define PDSS_INTR15_CFG_0_SRSENSE_ADDRESS                   (0x400a2014)
#define PDSS_INTR15_CFG_0_SRSENSE                           (*(volatile uint32_t *)(0x400a2014))
#define PDSS_INTR15_CFG_0_SRSENSE_DEFAULT                   (0x00000000)

/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_UV_FILT_EN             (1u << 0) /* <0:0> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_UV_FILT_CFG_MASK       (0x00000006) /* <1:2> R:RW:0: */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_UV_FILT_CFG_POS        (1)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_UV_FILT_RESET          (1u << 3) /* <3:3> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_UV_FILT_BYPASS         (1u << 4) /* <4:4> R:RW:0: */


/*
 * #of clock CLK_LF filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_UV_FILT_SEL_MASK       (0x000003e0) /* <5:9> R:RW:0: */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_UV_FILT_SEL_POS        (5)


/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_OV_FILT_EN             (1u << 11) /* <11:11> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_OV_FILT_CFG_MASK       (0x00003000) /* <12:13> R:RW:0: */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_OV_FILT_CFG_POS        (12)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_OV_FILT_RESET          (1u << 14) /* <14:14> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_OV_FILT_BYPASS         (1u << 15) /* <15:15> R:RW:0: */


/*
 * #of clock CLK_LF filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_OV_FILT_SEL_MASK       (0x001f0000) /* <16:20> R:RW:0: */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_OV_FILT_SEL_POS        (16)


/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_OV_FILT_EN_CLK_PASC    (1u << 21) /* <21:21> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_OV_FILT_CFG_CLK_PASC_MASK    (0x00c00000) /* <22:23> R:RW:0: */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_OV_FILT_CFG_CLK_PASC_POS    (22)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_OV_FILT_RESET_CLK_PASC    (1u << 24) /* <24:24> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_OV_FILT_BYPASS_CLK_PASC    (1u << 25) /* <25:25> R:RW:0: */


/*
 * #of clock CLK_PASC filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_OV_FILT_SEL_CLK_PASC_MASK    (0x3c000000) /* <26:29> R:RW:0: */
#define PDSS_INTR15_CFG_0_SRSENSE_FF_OV_FILT_SEL_CLK_PASC_POS    (26)


/*
 * SRSENSE HardIP CLK_PASC Filter and Edge detector configuration for PEAKDET_OUT
 * and ZCDF_OUT detection
 */
#define PDSS_INTR15_CFG_1_SRSENSE_ADDRESS                   (0x400a2018)
#define PDSS_INTR15_CFG_1_SRSENSE                           (*(volatile uint32_t *)(0x400a2018))
#define PDSS_INTR15_CFG_1_SRSENSE_DEFAULT                   (0x00208410)

/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR15_CFG_1_SRSENSE_PEAKDET_OUT_FILT_EN       (1u << 0) /* <0:0> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR15_CFG_1_SRSENSE_PEAKDET_OUT_FILT_CFG_MASK    (0x00000006) /* <1:2> R:RW:0: */
#define PDSS_INTR15_CFG_1_SRSENSE_PEAKDET_OUT_FILT_CFG_POS    (1)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR15_CFG_1_SRSENSE_PEAKDET_OUT_FILT_RESET    (1u << 3) /* <3:3> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR15_CFG_1_SRSENSE_PEAKDET_OUT_FILT_BYPASS    (1u << 4) /* <4:4> R:RW:1: */


/*
 * #of clock CLK_PASC filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR15_CFG_1_SRSENSE_PEAKDET_OUT_FILT_SEL_MASK    (0x000000e0) /* <5:7> R:RW:0: */
#define PDSS_INTR15_CFG_1_SRSENSE_PEAKDET_OUT_FILT_SEL_POS    (5)


/*
 * This bit enables CPU to bypass the Filter when the part is in deep-sleep
 * state.  This over-rides the FILTER_EN settings during DEEP SLEEP state
 * ONLY.
 * The bit should be set by CPU only when its using the filter with high
 * frequency active clock and wants to wakeup from deep sleep on the transition
 * of the incoming signal.
 * This bit if set also disables the AUTO shutoff logic in DEEP SLEEP state.
 */
#define PDSS_INTR15_CFG_1_SRSENSE_PEAKDET_OUT_DPSLP_MODE    (1u << 10) /* <10:10> R:RW:1: */


/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR15_CFG_1_SRSENSE_ZCDF_OUT_FILT_EN          (1u << 11) /* <11:11> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR15_CFG_1_SRSENSE_ZCDF_OUT_FILT_CFG_MASK    (0x00003000) /* <12:13> R:RW:0: */
#define PDSS_INTR15_CFG_1_SRSENSE_ZCDF_OUT_FILT_CFG_POS     (12)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR15_CFG_1_SRSENSE_ZCDF_OUT_FILT_RESET       (1u << 14) /* <14:14> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR15_CFG_1_SRSENSE_ZCDF_OUT_FILT_BYPASS      (1u << 15) /* <15:15> R:RW:1: */


/*
 * #of clock CLK_PASC filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR15_CFG_1_SRSENSE_ZCDF_OUT_FILT_SEL_MASK    (0x000f0000) /* <16:19> R:RW:0: */
#define PDSS_INTR15_CFG_1_SRSENSE_ZCDF_OUT_FILT_SEL_POS     (16)


/*
 * This bit enables CPU to bypass the Filter when the part is in deep-sleep
 * state.  This over-rides the FILTER_EN settings during DEEP SLEEP state
 * ONLY.
 * The bit should be set by CPU only when its using the filter with high
 * frequency active clock and wants to wakeup from deep sleep on the transition
 * of the incoming signal.
 * This bit if set also disables the AUTO shutoff logic in DEEP SLEEP state.
 */
#define PDSS_INTR15_CFG_1_SRSENSE_ZCDF_OUT_DPSLP_MODE       (1u << 21) /* <21:21> R:RW:1: */


/*
 * SRSENSE HardIP CLK_PASC Filter and Edge detector configuration for PEAKDET_RST_OUT
 * and PEAKDET_CLCMP_RAW_OUT detection
 */
#define PDSS_INTR15_CFG_2_SRSENSE_ADDRESS                   (0x400a201c)
#define PDSS_INTR15_CFG_2_SRSENSE                           (*(volatile uint32_t *)(0x400a201c))
#define PDSS_INTR15_CFG_2_SRSENSE_DEFAULT                   (0x00208410)

/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR15_CFG_2_SRSENSE_PEAKDET_RST_OUT_FILT_EN    (1u << 0) /* <0:0> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR15_CFG_2_SRSENSE_PEAKDET_RST_OUT_FILT_CFG_MASK    (0x00000006) /* <1:2> R:RW:0: */
#define PDSS_INTR15_CFG_2_SRSENSE_PEAKDET_RST_OUT_FILT_CFG_POS    (1)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR15_CFG_2_SRSENSE_PEAKDET_RST_OUT_FILT_RESET    (1u << 3) /* <3:3> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR15_CFG_2_SRSENSE_PEAKDET_RST_OUT_FILT_BYPASS    (1u << 4) /* <4:4> R:RW:1: */


/*
 * #of clock CLK_PASC filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR15_CFG_2_SRSENSE_PEAKDET_RST_OUT_FILT_SEL_MASK    (0x000003e0) /* <5:9> R:RW:0: */
#define PDSS_INTR15_CFG_2_SRSENSE_PEAKDET_RST_OUT_FILT_SEL_POS    (5)


/*
 * This bit enables CPU to bypass the Filter when the part is in deep-sleep
 * state.  This over-rides the FILTER_EN settings during DEEP SLEEP state
 * ONLY.
 * The bit should be set by CPU only when its using the filter with high
 * frequency active clock and wants to wakeup from deep sleep on the transition
 * of the incoming signal.
 * This bit if set also disables the AUTO shutoff logic in DEEP SLEEP state.
 */
#define PDSS_INTR15_CFG_2_SRSENSE_PEAKDET_RST_OUT_DPSLP_MODE    (1u << 10) /* <10:10> R:RW:1: */


/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR15_CFG_2_SRSENSE_PEAKDET_CLCMP_RAW_OUT_FILT_EN    (1u << 11) /* <11:11> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR15_CFG_2_SRSENSE_PEAKDET_CLCMP_RAW_OUT_FILT_CFG_MASK    (0x00003000) /* <12:13> R:RW:0: */
#define PDSS_INTR15_CFG_2_SRSENSE_PEAKDET_CLCMP_RAW_OUT_FILT_CFG_POS    (12)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR15_CFG_2_SRSENSE_PEAKDET_CLCMP_RAW_OUT_FILT_RESET    (1u << 14) /* <14:14> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR15_CFG_2_SRSENSE_PEAKDET_CLCMP_RAW_OUT_FILT_BYPASS    (1u << 15) /* <15:15> R:RW:1: */


/*
 * #of clock CLK_PASC filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR15_CFG_2_SRSENSE_PEAKDET_CLCMP_RAW_OUT_FILT_SEL_MASK    (0x00070000) /* <16:18> R:RW:0: */
#define PDSS_INTR15_CFG_2_SRSENSE_PEAKDET_CLCMP_RAW_OUT_FILT_SEL_POS    (16)


/*
 * This bit enables CPU to bypass the Filter when the part is in deep-sleep
 * state.  This over-rides the FILTER_EN settings during DEEP SLEEP state
 * ONLY.
 * The bit should be set by CPU only when its using the filter with high
 * frequency active clock and wants to wakeup from deep sleep on the transition
 * of the incoming signal.
 * This bit if set also disables the AUTO shutoff logic in DEEP SLEEP state.
 */
#define PDSS_INTR15_CFG_2_SRSENSE_PEAKDET_CLCMP_RAW_OUT_DPSLP_MODE    (1u << 21) /* <21:21> R:RW:1: */


/*
 * SRSENSE HardIP High/Low Speed Filter and Edge detector configuration for
 * SR_SEN_OVP_OUT detection
 */
#define PDSS_INTR15_CFG_3_SRSENSE_ADDRESS                   (0x400a2020)
#define PDSS_INTR15_CFG_3_SRSENSE                           (*(volatile uint32_t *)(0x400a2020))
#define PDSS_INTR15_CFG_3_SRSENSE_DEFAULT                   (0x00000400)

/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR15_CFG_3_SRSENSE_SR_SEN_OVP_OUT_FILT_EN    (1u << 0) /* <0:0> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR15_CFG_3_SRSENSE_SR_SEN_OVP_OUT_FILT_CFG_MASK    (0x00000006) /* <1:2> R:RW:0: */
#define PDSS_INTR15_CFG_3_SRSENSE_SR_SEN_OVP_OUT_FILT_CFG_POS    (1)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR15_CFG_3_SRSENSE_SR_SEN_OVP_OUT_FILT_RESET    (1u << 3) /* <3:3> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR15_CFG_3_SRSENSE_SR_SEN_OVP_OUT_FILT_BYPASS    (1u << 4) /* <4:4> R:RW:0: */


/*
 * #of clock CLK_PASC filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR15_CFG_3_SRSENSE_SR_SEN_OVP_OUT_FILT_SEL_MASK    (0x000003e0) /* <5:9> R:RW:0: */
#define PDSS_INTR15_CFG_3_SRSENSE_SR_SEN_OVP_OUT_FILT_SEL_POS    (5)


/*
 * This bit enables CPU to bypass the Filter when the part is in deep-sleep
 * state.  This over-rides the FILTER_EN settings during DEEP SLEEP state
 * ONLY.
 * The bit should be set by CPU only when its using the filter with high
 * frequency active clock and wants to wakeup from deep sleep on the transition
 * of the incoming signal.
 * This bit if set also disables the AUTO shutoff logic in DEEP SLEEP state.
 */
#define PDSS_INTR15_CFG_3_SRSENSE_SR_SEN_OVP_OUT_DPSLP_MODE    (1u << 10) /* <10:10> R:RW:1: */


/*
 * SRSENSE HardIP High/Low Speed Filter and Edge detector configuration for
 * NSN_OUT and ZCD_OUT detection
 */
#define PDSS_INTR15_CFG_4_SRSENSE_ADDRESS                   (0x400a2024)
#define PDSS_INTR15_CFG_4_SRSENSE                           (*(volatile uint32_t *)(0x400a2024))
#define PDSS_INTR15_CFG_4_SRSENSE_DEFAULT                   (0x00208410)

/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR15_CFG_4_SRSENSE_NSN_OUT_FILT_EN           (1u << 0) /* <0:0> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR15_CFG_4_SRSENSE_NSN_OUT_FILT_CFG_MASK     (0x00000006) /* <1:2> R:RW:0: */
#define PDSS_INTR15_CFG_4_SRSENSE_NSN_OUT_FILT_CFG_POS      (1)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR15_CFG_4_SRSENSE_NSN_OUT_FILT_RESET        (1u << 3) /* <3:3> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR15_CFG_4_SRSENSE_NSN_OUT_FILT_BYPASS       (1u << 4) /* <4:4> R:RW:1: */


/*
 * #of clock CLK_PASC filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR15_CFG_4_SRSENSE_NSN_OUT_FILT_SEL_MASK     (0x000001e0) /* <5:8> R:RW:0: */
#define PDSS_INTR15_CFG_4_SRSENSE_NSN_OUT_FILT_SEL_POS      (5)


/*
 * This bit enables CPU to bypass the Filter when the part is in deep-sleep
 * state.  This over-rides the FILTER_EN settings during DEEP SLEEP state
 * ONLY.
 * The bit should be set by CPU only when its using the filter with high
 * frequency active clock and wants to wakeup from deep sleep on the transition
 * of the incoming signal.
 * This bit if set also disables the AUTO shutoff logic in DEEP SLEEP state.
 */
#define PDSS_INTR15_CFG_4_SRSENSE_NSN_OUT_DPSLP_MODE        (1u << 10) /* <10:10> R:RW:1: */


/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR15_CFG_4_SRSENSE_ZCD_OUT_FILT_EN           (1u << 11) /* <11:11> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR15_CFG_4_SRSENSE_ZCD_OUT_FILT_CFG_MASK     (0x00003000) /* <12:13> R:RW:0: */
#define PDSS_INTR15_CFG_4_SRSENSE_ZCD_OUT_FILT_CFG_POS      (12)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR15_CFG_4_SRSENSE_ZCD_OUT_FILT_RESET        (1u << 14) /* <14:14> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR15_CFG_4_SRSENSE_ZCD_OUT_FILT_BYPASS       (1u << 15) /* <15:15> R:RW:1: */


/*
 * #of clock CLK_PASC filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR15_CFG_4_SRSENSE_ZCD_OUT_FILT_SEL_MASK     (0x000f0000) /* <16:19> R:RW:0: */
#define PDSS_INTR15_CFG_4_SRSENSE_ZCD_OUT_FILT_SEL_POS      (16)


/*
 * This bit enables CPU to bypass the Filter when the part is in deep-sleep
 * state.  This over-rides the FILTER_EN settings during DEEP SLEEP state
 * ONLY.
 * The bit should be set by CPU only when its using the filter with high
 * frequency active clock and wants to wakeup from deep sleep on the transition
 * of the incoming signal.
 * This bit if set also disables the AUTO shutoff logic in DEEP SLEEP state.
 */
#define PDSS_INTR15_CFG_4_SRSENSE_ZCD_OUT_DPSLP_MODE        (1u << 21) /* <21:21> R:RW:1: */


/*
 * PWM Hard IP CLK_PASC Filter and Edge detector configuration for PWM_OUT
 * and SKIP_OUT
 */
#define PDSS_INTR15_CFG_0_PWM_ADDRESS                       (0x400a2028)
#define PDSS_INTR15_CFG_0_PWM                               (*(volatile uint32_t *)(0x400a2028))
#define PDSS_INTR15_CFG_0_PWM_DEFAULT                       (0x00250c10)

/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR15_CFG_0_PWM_PWM_OUT_FILT_EN               (1u << 0) /* <0:0> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR15_CFG_0_PWM_PWM_OUT_FILT_CFG_MASK         (0x00000006) /* <1:2> R:RW:0: */
#define PDSS_INTR15_CFG_0_PWM_PWM_OUT_FILT_CFG_POS          (1)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR15_CFG_0_PWM_PWM_OUT_FILT_RESET            (1u << 3) /* <3:3> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR15_CFG_0_PWM_PWM_OUT_FILT_BYPASS           (1u << 4) /* <4:4> R:RW:1: */


/*
 * #of clock CLK_PASC filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR15_CFG_0_PWM_PWM_OUT_FILT_SEL_MASK         (0x000000e0) /* <5:7> R:RW:0: */
#define PDSS_INTR15_CFG_0_PWM_PWM_OUT_FILT_SEL_POS          (5)


/*
 * This bit enables CPU to bypass the Filter when the part is in deep-sleep
 * state.  This over-rides the FILTER_EN settings during DEEP SLEEP state
 * ONLY.
 * The bit should be set by CPU only when its using the filter with high
 * frequency active clock and wants to wakeup from deep sleep on the transition
 * of the incoming signal.
 * This bit if set also disables the AUTO shutoff logic in DEEP SLEEP state.
 */
#define PDSS_INTR15_CFG_0_PWM_PWM_OUT_DPSLP_MODE            (1u << 10) /* <10:10> R:RW:1: */


/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR15_CFG_0_PWM_SKIP_OUT_FILT_EN              (1u << 11) /* <11:11> R:RW:1: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR15_CFG_0_PWM_SKIP_OUT_FILT_CFG_MASK        (0x00003000) /* <12:13> R:RW:0: */
#define PDSS_INTR15_CFG_0_PWM_SKIP_OUT_FILT_CFG_POS         (12)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR15_CFG_0_PWM_SKIP_OUT_FILT_RESET           (1u << 14) /* <14:14> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR15_CFG_0_PWM_SKIP_OUT_FILT_BYPASS          (1u << 15) /* <15:15> R:RW:0: */


/*
 * #of clock CLK_PASC filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR15_CFG_0_PWM_SKIP_OUT_FILT_SEL_MASK        (0x001f0000) /* <16:20> R:RW:5: */
#define PDSS_INTR15_CFG_0_PWM_SKIP_OUT_FILT_SEL_POS         (16)


/*
 * This bit enables CPU to bypass the Filter when the part is in deep-sleep
 * state.  This over-rides the FILTER_EN settings during DEEP SLEEP state
 * ONLY.
 * The bit should be set by CPU only when its using the filter with high
 * frequency active clock and wants to wakeup from deep sleep on the transition
 * of the incoming signal.
 * This bit if set also disables the AUTO shutoff logic in DEEP SLEEP state.
 */
#define PDSS_INTR15_CFG_0_PWM_SKIP_OUT_DPSLP_MODE           (1u << 21) /* <21:21> R:RW:1: */


/*
 * PWM Hard IP CLK_PASC Filter and Edge detector configuration for BURST_EXIT_OUT
 */
#define PDSS_INTR15_CFG_1_PWM_ADDRESS                       (0x400a202c)
#define PDSS_INTR15_CFG_1_PWM                               (*(volatile uint32_t *)(0x400a202c))
#define PDSS_INTR15_CFG_1_PWM_DEFAULT                       (0x00000410)

/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR15_CFG_1_PWM_BURST_EXIT_OUT_FILT_EN        (1u << 0) /* <0:0> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR15_CFG_1_PWM_BURST_EXIT_OUT_FILT_CFG_MASK    (0x00000006) /* <1:2> R:RW:0: */
#define PDSS_INTR15_CFG_1_PWM_BURST_EXIT_OUT_FILT_CFG_POS    (1)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR15_CFG_1_PWM_BURST_EXIT_OUT_FILT_RESET     (1u << 3) /* <3:3> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR15_CFG_1_PWM_BURST_EXIT_OUT_FILT_BYPASS    (1u << 4) /* <4:4> R:RW:1: */


/*
 * #of clock CLK_PASC filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR15_CFG_1_PWM_BURST_EXIT_OUT_FILT_SEL_MASK    (0x000000e0) /* <5:7> R:RW:0: */
#define PDSS_INTR15_CFG_1_PWM_BURST_EXIT_OUT_FILT_SEL_POS    (5)


/*
 * This bit enables CPU to bypass the Filter when the part is in deep-sleep
 * state.  This over-rides the FILTER_EN settings during DEEP SLEEP state
 * ONLY.
 * The bit should be set by CPU only when its using the filter with high
 * frequency active clock and wants to wakeup from deep sleep on the transition
 * of the incoming signal.
 * This bit if set also disables the AUTO shutoff logic in DEEP SLEEP state.
 */
#define PDSS_INTR15_CFG_1_PWM_BURST_EXIT_OUT_DPSLP_MODE     (1u << 10) /* <10:10> R:RW:1: */


/*
 * S8PDS EA Hard IP CLK_LF Filter and Edge detector configuration for CC_FLAG
 */
#define PDSS_INTR15_CFG_CC_FLAG_ADDRESS                     (0x400a2030)
#define PDSS_INTR15_CFG_CC_FLAG                             (*(volatile uint32_t *)(0x400a2030))
#define PDSS_INTR15_CFG_CC_FLAG_DEFAULT                     (0x00000000)

/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR15_CFG_CC_FLAG_CC_FLAG_FILT_EN             (1u << 0) /* <0:0> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR15_CFG_CC_FLAG_CC_FLAG_FILT_CFG_MASK       (0x00000006) /* <1:2> R:RW:0: */
#define PDSS_INTR15_CFG_CC_FLAG_CC_FLAG_FILT_CFG_POS        (1)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR15_CFG_CC_FLAG_CC_FLAG_FILT_RESET          (1u << 3) /* <3:3> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR15_CFG_CC_FLAG_CC_FLAG_FILT_BYPASS         (1u << 4) /* <4:4> R:RW:0: */


/*
 * #of clock CLK_LF filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR15_CFG_CC_FLAG_CC_FLAG_FILT_SEL_MASK       (0x00003fe0) /* <5:13> R:RW:0: */
#define PDSS_INTR15_CFG_CC_FLAG_CC_FLAG_FILT_SEL_POS        (5)


/*
 * S8PDS 20V Regulator VBUS Wakeup Interrupts edge and filter configuration
 */
#define PDSS_INTR15_CFG_VREG20_VBUS_ADDRESS                 (0x400a2034)
#define PDSS_INTR15_CFG_VREG20_VBUS                         (*(volatile uint32_t *)(0x400a2034))
#define PDSS_INTR15_CFG_VREG20_VBUS_DEFAULT                 (0x00000000)

/*
 * 0: Filter is disabled
 * 1: Filter is enabled
 */
#define PDSS_INTR15_CFG_VREG20_VBUS_PDS_VREG_VBUS_FILT_EN    (1u << 0) /* <0:0> R:RW:0: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR15_CFG_VREG20_VBUS_PDS_VREG_VBUS_CFG_MASK    (0x00000006) /* <1:2> R:RW:0: */
#define PDSS_INTR15_CFG_VREG20_VBUS_PDS_VREG_VBUS_CFG_POS    (1)


/*
 * This field specifies the reset value of the Filter.
 * To reset the Filter, set this bit to the appropriate value and Toggle
 * FILTER_EN.
 *   Firmware can set the values (Reset Values, Config values etc when the
 * filter_en is 0 and next cycle it can take the filter out of reset by writing
 * 1 to filter_en).
 * FILTER_EN =0 acts as an asynchronous reset. Internally when filter is
 * enabled, the deassertion of reset is synchronized and it takes 3-4 cycles.
 * So firmware should at-least wait 5 cycles for dynamically changing filter
 * values and applying reset again.
 */
#define PDSS_INTR15_CFG_VREG20_VBUS_PDS_VREG_VBUS_FILT_RESET    (1u << 3) /* <3:3> R:RW:0: */


/*
 * Setting this bit bypasses the Filter. It is recommended to set FILTER_EN
 * to 1’b0 with this to save power.
 */
#define PDSS_INTR15_CFG_VREG20_VBUS_PDS_VREG_VBUS_FILT_BYPASS    (1u << 4) /* <4:4> R:RW:0: */


/*
 * #of clock CLK_LF filtering. Should be programmed before FILTER is enabled.
 */
#define PDSS_INTR15_CFG_VREG20_VBUS_PDS_VREG_VBUS_FILT_SEL_MASK    (0x000000e0) /* <5:7> R:RW:0: */
#define PDSS_INTR15_CFG_VREG20_VBUS_PDS_VREG_VBUS_FILT_SEL_POS    (5)


/*
 * INTR15 Status
 */
#define PDSS_INTR15_STATUS_ADDRESS                          (0x400a2038)
#define PDSS_INTR15_STATUS                                  (*(volatile uint32_t *)(0x400a2038))
#define PDSS_INTR15_STATUS_DEFAULT                          (0x00000000)

/*
 * The status of s8pds_ea_top.cc_flag
 */
#define PDSS_INTR15_STATUS_CC_FLAG_STATUS                   (1u << 0) /* <0:0> RW:R:0: */


/*
 * s8pds_ea_top.cc_flag Filtered output
 */
#define PDSS_INTR15_STATUS_CC_FLAG_FILT                     (1u << 1) /* <1:1> RW:R:0: */


/*
 * The status of vbus_det from the s8pds_20vreg_top
 */
#define PDSS_INTR15_STATUS_PDS_VREG20_VBUS_STATUS           (1u << 2) /* <2:2> RW:R:0: */


/*
 * s8pds_20vreg_top.vbus_det Filtered output
 */
#define PDSS_INTR15_STATUS_PDS_VREG20_VBUS_FILT             (1u << 3) /* <3:3> RW:R:0: */


/*
 * The status of s8pds_scp_top.scp_out
 */
#define PDSS_INTR15_STATUS_PDS_SCP_STATUS                   (1u << 4) /* <4:4> RW:R:0: */


/*
 * s8pds_scp_top.scp_out Filtered output
 */
#define PDSS_INTR15_STATUS_PDS_SCP_FILT                     (1u << 5) /* <5:5> RW:R:0: */


/*
 * The status of vs8pds_srsense_top.ff_uv
 */
#define PDSS_INTR15_STATUS_FF_UV_STATUS                     (1u << 6) /* <6:6> RW:R:0: */


/*
 * s8pds_srsense_top.ff_uv Filtered output
 */
#define PDSS_INTR15_STATUS_FF_UV_FILT                       (1u << 7) /* <7:7> RW:R:0: */


/*
 * The status of vs8pds_srsense_top.ff_ov
 */
#define PDSS_INTR15_STATUS_FF_OV_STATUS                     (1u << 8) /* <8:8> RW:R:0: */


/*
 * s8pds_srsense_top.ff_ov Filtered output
 */
#define PDSS_INTR15_STATUS_FF_OV_FILT                       (1u << 9) /* <9:9> RW:R:0: */


/*
 * The status of vs8pds_srsense_top.peakdet_out
 */
#define PDSS_INTR15_STATUS_PEAKDET_OUT_STATUS               (1u << 10) /* <10:10> RW:R:0: */


/*
 * s8pds_srsense_top.peakdet_out Filtered output
 */
#define PDSS_INTR15_STATUS_PEAKDET_OUT_FILT                 (1u << 11) /* <11:11> RW:R:0: */


/*
 * The status of vs8pds_srsense_top.peakdet_rst_out
 */
#define PDSS_INTR15_STATUS_PEAKDET_RST_OUT_STATUS           (1u << 12) /* <12:12> RW:R:0: */


/*
 * s8pds_srsense_top.peakdet_rst_out Filtered output
 */
#define PDSS_INTR15_STATUS_PEAKDET_RST_OUT_FILT             (1u << 13) /* <13:13> RW:R:0: */


/*
 * The status of vs8pds_srsense_top.peakdet_clcmp_raw_out
 */
#define PDSS_INTR15_STATUS_PEAKDET_CLCMP_RAW_OUT_STATUS     (1u << 14) /* <14:14> RW:R:0: */


/*
 * s8pds_srsense_top.peakdet_clcmp_raw_out Filtered output
 */
#define PDSS_INTR15_STATUS_PEAKDET_CLCMP_RAW_OUT_FILT       (1u << 15) /* <15:15> RW:R:0: */


/*
 * The status of vs8pds_srsense_top.zcdf_out
 */
#define PDSS_INTR15_STATUS_ZCDF_OUT_STATUS                  (1u << 16) /* <16:16> RW:R:0: */


/*
 * s8pds_srsense_top.zcdf_out Filtered output
 */
#define PDSS_INTR15_STATUS_ZCDF_OUT_FILT                    (1u << 17) /* <17:17> RW:R:0: */


/*
 * The status of vs8pds_srsense_top.sr_sen_ovp_out
 */
#define PDSS_INTR15_STATUS_SR_SEN_OVP_OUT_STATUS            (1u << 18) /* <18:18> RW:R:0: */


/*
 * s8pds_srsense_top.sr_sen_ovp_out Filtered output
 */
#define PDSS_INTR15_STATUS_SR_SEN_OVP_OUT_FILT              (1u << 19) /* <19:19> RW:R:0: */


/*
 * The status of vs8pds_srsense_top.nsn
 */
#define PDSS_INTR15_STATUS_NSN_OUT_STATUS                   (1u << 20) /* <20:20> RW:R:0: */


/*
 * s8pds_srsense_top.nsn Filtered output
 */
#define PDSS_INTR15_STATUS_NSN_OUT_FILT                     (1u << 21) /* <21:21> RW:R:0: */


/*
 * The status of vs8pds_srsense_top.zcd_out
 */
#define PDSS_INTR15_STATUS_ZCD_OUT_STATUS                   (1u << 22) /* <22:22> RW:R:0: */


/*
 * s8pds_srsense_top.zcd_out Filtered output
 */
#define PDSS_INTR15_STATUS_ZCD_OUT_FILT                     (1u << 23) /* <23:23> RW:R:0: */


/*
 * The status of s8pds_pwm_top.pwm_out
 */
#define PDSS_INTR15_STATUS_PWM_OUT_STATUS                   (1u << 24) /* <24:24> RW:R:0: */


/*
 * s8pds_pwm_top.pwm_out Filtered output
 */
#define PDSS_INTR15_STATUS_PWM_OUT_FILT                     (1u << 25) /* <25:25> RW:R:0: */


/*
 * The status of s8pds_pwm_top.skip_out
 */
#define PDSS_INTR15_STATUS_SKIP_OUT_STATUS                  (1u << 26) /* <26:26> RW:R:0: */


/*
 * s8pds_pwm_top.skip_out Filtered output
 */
#define PDSS_INTR15_STATUS_SKIP_OUT_FILT                    (1u << 27) /* <27:27> RW:R:0: */


/*
 * The status of s8pds_pwm_top.burst_exit_out
 */
#define PDSS_INTR15_STATUS_BURST_EXIT_OUT_STATUS            (1u << 28) /* <28:28> RW:R:0: */


/*
 * s8pds_pwm_top.burst_exit_out Filtered output
 */
#define PDSS_INTR15_STATUS_BURST_EXIT_OUT_FILT              (1u << 29) /* <29:29> RW:R:0: */


/*
 * INTR15 interrupt Cause.
 */
#define PDSS_INTR15_ADDRESS                                 (0x400a203c)
#define PDSS_INTR15                                         (*(volatile uint32_t *)(0x400a203c))
#define PDSS_INTR15_DEFAULT                                 (0x00000000)

/*
 * s8pds_ea_top.cc_flag changed (wakeup interrupt from deepsleep)
 * Check the  INTR15_STATUS.CC_FLAG_STATUS value
 */
#define PDSS_INTR15_CC_FLAG_CHANGED                         (1u << 0) /* <0:0> RW1S:RW1C:0: */


/*
 * vbus_det changed (wakeup interrupt from deepsleep)
 * Check the  INTR15_STATUS.PDS_VREG20_VBUS_STATUS value
 */
#define PDSS_INTR15_PDS_VREG20_VBUS_CHANGED                 (1u << 1) /* <1:1> RW1S:RW1C:0: */


/*
 * s8pds_scp_top.scp_out changed (wakeup interrupt from deepsleep)
 * Check the INTR15_STATUS.PDS_SCP_STATUS value
 */
#define PDSS_INTR15_PDS_SCP_CHANGED                         (1u << 2) /* <2:2> RW1S:RW1C:0: */


/*
 * s8pds_srsense_top.ff_uv changed (wakeup interrupt from deepsleep)
 * Check the  INTR15_STATUS.FF_UV_STATUS value
 */
#define PDSS_INTR15_FF_UV_CHANGED                           (1u << 3) /* <3:3> RW1S:RW1C:0: */


/*
 * s8pds_srsense_top.ff_ov changed (wakeup interrupt from deepsleep)
 * Check the  INTR15_STATUS.FF_OV_STATUS value
 */
#define PDSS_INTR15_FF_OV_CHANGED                           (1u << 4) /* <4:4> RW1S:RW1C:0: */


/*
 * s8pds_srsense_top.peakdet_out changed (wakeup interrupt from deepsleep)
 * Check the  INTR15_STATUS.PEACKDET_OUT_STATUS value
 */
#define PDSS_INTR15_PEAKDET_OUT_CHANGED                     (1u << 5) /* <5:5> RW1S:RW1C:0: */


/*
 * s8pds_srsense_top.peakdet_rst_out changed (wakeup interrupt from deepsleep)
 * Check the  INTR15_STATUS.PEACKDET_OUT_STATUS value
 */
#define PDSS_INTR15_PEAKDET_RST_OUT_CHANGED                 (1u << 6) /* <6:6> RW1S:RW1C:0: */


/*
 * s8pds_srsense_top.peakdet_clcmp_raw_out changed (wakeup interrupt from
 * deepsleep)
 * Check the  INTR15_STATUS.PEAKDET_CLCMP_RAW_OUT_STATUS value
 */
#define PDSS_INTR15_PEAKDET_CLCMP_RAW_OUT_CHANGED           (1u << 7) /* <7:7> RW1S:RW1C:0: */


/*
 * s8pds_srsense_top.zcdf_out changed (wakeup interrupt from deepsleep)
 * Check the  INTR15_STATUS.ZCDF_OUT_STATUS value
 */
#define PDSS_INTR15_ZCDF_OUT_CHANGED                        (1u << 8) /* <8:8> RW1S:RW1C:0: */


/*
 * s8pds_srsense_top.zcdf_out changed (wakeup interrupt from deepsleep)
 * Check the  INTR15_STATUS.SR_SEN_OVP_OUT_STATUS value
 */
#define PDSS_INTR15_SR_SEN_OVP_OUT_CHANGED                  (1u << 9) /* <9:9> RW1S:RW1C:0: */


/*
 * s8pds_srsense_top.nsn_out changed (wakeup interrupt from deepsleep)
 * Check the  INTR15_STATUS.NSN'_OUT_STATUS value
 */
#define PDSS_INTR15_NSN_OUT_CHANGED                         (1u << 10) /* <10:10> RW1S:RW1C:0: */


/*
 * s8pds_srsense_top.zcd_out changed (wakeup interrupt from deepsleep)
 * Check the  INTR15_STATUS.ZCD_OUT_STATUS value
 */
#define PDSS_INTR15_ZCD_OUT_CHANGED                         (1u << 11) /* <11:11> RW1S:RW1C:0: */


/*
 * s8pds_pwm_top.pwm_out changed (wakeup interrupt from deepsleep)
 * Check the  INTR15_STATUS.PWM_OUT_STATUS value
 */
#define PDSS_INTR15_PWM_OUT_CHANGED                         (1u << 12) /* <12:12> RW1S:RW1C:0: */


/*
 * s8pds_pwm_top.pwm_out changed (wakeup interrupt from deepsleep)
 * Check the  INTR15_STATUS.SKIP_OUT_STATUS value
 */
#define PDSS_INTR15_SKIP_OUT_CHANGED                        (1u << 13) /* <13:13> RW1S:RW1C:0: */


/*
 * s8pds_pwm_top.pwm_out changed (wakeup interrupt from deepsleep)
 * Check the  INTR15_STATUS.BURST_EXIT_OUT_STATUS value
 */
#define PDSS_INTR15_BURST_EXIT_OUT_CHANGED                  (1u << 14) /* <14:14> RW1S:RW1C:0: */


/*
 * INTR15 Interrupt Set
 */
#define PDSS_INTR15_SET_ADDRESS                             (0x400a2040)
#define PDSS_INTR15_SET                                     (*(volatile uint32_t *)(0x400a2040))
#define PDSS_INTR15_SET_DEFAULT                             (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_SET_CC_FLAG_CHANGED                     (1u << 0) /* <0:0> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_SET_PDS_VREG20_VBUS_CHANGED             (1u << 1) /* <1:1> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_SET_PDS_SCP_CHANGED                     (1u << 2) /* <2:2> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_SET_FF_UV_CHANGED                       (1u << 3) /* <3:3> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_SET_FF_OV_CHANGED                       (1u << 4) /* <4:4> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_SET_PEAKDET_OUT_CHANGED                 (1u << 5) /* <5:5> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_SET_PEAKDET_RST_OUT_CHANGED             (1u << 6) /* <6:6> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_SET_PEAKDET_CLCMP_RAW_OUT_CHANGED       (1u << 7) /* <7:7> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_SET_ZCDF_OUT_CHANGED                    (1u << 8) /* <8:8> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_SET_SR_SEN_OVP_OUT_CHANGED              (1u << 9) /* <9:9> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_SET_NSN_OUT_CHANGED                     (1u << 10) /* <10:10> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_SET_ZCD_OUT_CHANGED                     (1u << 11) /* <11:11> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_SET_PWM_OUT_CHANGED                     (1u << 12) /* <12:12> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_SET_SKIP_OUT_CHANGED                    (1u << 13) /* <13:13> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_SET_BURST_EXIT_OUT_CHANGED              (1u << 14) /* <14:14> A:RW1S:0: */


/*
 * INTR15 interrupt Mask
 */
#define PDSS_INTR15_MASK_ADDRESS                            (0x400a2044)
#define PDSS_INTR15_MASK                                    (*(volatile uint32_t *)(0x400a2044))
#define PDSS_INTR15_MASK_DEFAULT                            (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_MASK_CC_FLAG_CHANGED_MASK               (1u << 0) /* <0:0> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_MASK_PDS_VREG20_VBUS_CHANGED_MASK       (1u << 1) /* <1:1> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_MASK_PDS_SCP_CHANGED_MASK               (1u << 2) /* <2:2> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_MASK_FF_UV_CHANGED_MASK                 (1u << 3) /* <3:3> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_MASK_FF_OV_CHANGED_MASK                 (1u << 4) /* <4:4> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_MASK_PEAKDET_OUT_CHANGED_MASK           (1u << 5) /* <5:5> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_MASK_PEAKDET_RST_OUT_CHANGED_MASK       (1u << 6) /* <6:6> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_MASK_PEAKDET_CLCMP_RAW_OUT_CHANGED_MASK    (1u << 7) /* <7:7> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_MASK_ZCDF_OUT_CHANGED_MASK              (1u << 8) /* <8:8> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_MASK_SR_SEN_OVP_OUT_CHANGED_MASK        (1u << 9) /* <9:9> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_MASK_NSN_OUT_CHANGED_MASK               (1u << 10) /* <10:10> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_MASK_ZCD_OUT_CHANGED_MASK               (1u << 11) /* <11:11> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_MASK_PWM_OUT_CHANGED_MASK               (1u << 12) /* <12:12> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_MASK_SKIP_OUT_CHANGED_MASK              (1u << 13) /* <13:13> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR15_MASK_BURST_EXIT_OUT_CHANGED_MASK        (1u << 14) /* <14:14> R:RW:0: */


/*
 * INTR15 interrupt Masked
 */
#define PDSS_INTR15_MASKED_ADDRESS                          (0x400a2048)
#define PDSS_INTR15_MASKED                                  (*(volatile uint32_t *)(0x400a2048))
#define PDSS_INTR15_MASKED_DEFAULT                          (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR15_MASKED_CC_FLAG_CHANGED_MASKED           (1u << 0) /* <0:0> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR15_MASKED_PDS_VREG20_VBUS_CHANGED_MASKED    (1u << 1) /* <1:1> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR15_MASKED_PDS_SCP_CHANGED_MASKED           (1u << 2) /* <2:2> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR15_MASKED_FF_UV_CHANGED_MASKED             (1u << 3) /* <3:3> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR15_MASKED_FF_OV_CHANGED_MASKED             (1u << 4) /* <4:4> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR15_MASKED_PEAKDET_OUT_CHANGED_MASKED       (1u << 5) /* <5:5> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR15_MASKED_PEAKDET_RST_OUT_CHANGED_MASKED    (1u << 6) /* <6:6> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR15_MASKED_PEAKDET_CLCMP_RAW_OUT_CHANGED_MASKED    (1u << 7) /* <7:7> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR15_MASKED_ZCDF_OUT_CHANGED_MASKED          (1u << 8) /* <8:8> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR15_MASKED_SR_SEN_OVP_OUT_CHANGED_MASKED    (1u << 9) /* <9:9> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR15_MASKED_NSN_OUT_CHANGED_MASKED           (1u << 10) /* <10:10> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR15_MASKED_ZCD_OUT_CHANGED_MASKED           (1u << 11) /* <11:11> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR15_MASKED_PWM_OUT_CHANGED_MASKED           (1u << 12) /* <12:12> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR15_MASKED_SKIP_OUT_CHANGED_MASKED          (1u << 13) /* <13:13> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR15_MASKED_BURST_EXIT_OUT_CHANGED_MASKED    (1u << 14) /* <14:14> RW:R:0: */


/*
 * VBUS Transition config
 */
#define PDSS_VBTR_CFG_ADDRESS                               (0x400a204c)
#define PDSS_VBTR_CFG                                       (*(volatile uint32_t *)(0x400a204c))
#define PDSS_VBTR_CFG_DEFAULT                               (0x00000000)

/*
 * Enable/Disable SRC operation on isrc_dac_ctrl pin of s8pds_ea_top.
 * 1 - Enable
 * 0 - Disable
 */
#define PDSS_VBTR_CFG_SRC_EN                                (1u << 0) /* <0:0> R:RW:0: */


/*
 * SRC operation type
 * 1 - Increment
 * 0 - Decrement
 */
#define PDSS_VBTR_CFG_SRC_UP_DOWN                           (1u << 1) /* <1:1> R:RW:0: */


/*
 * Enable/Disable SNK operation on isnk_dac_ctrl pin of s8pds_ea_top.
 * 1 - Enable
 * 0 - Disable
 */
#define PDSS_VBTR_CFG_SNK_EN                                (1u << 8) /* <8:8> R:RW:0: */


/*
 * SNK operation type
 * 1 - Increment
 * 0 - Decrement
 */
#define PDSS_VBTR_CFG_SNK_UP_DOWN                           (1u << 9) /* <9:9> R:RW:0: */


/*
 * Selects which operation starts first
 * 0 - Source then Sink      (if SNK_EN==1)
 * 1 - Sink      then Source (if SRC_EN==1)
 */
#define PDSS_VBTR_CFG_VBTR_SEL                              (1u << 16) /* <16:16> R:RW:0: */


/*
 * VBUS Transition control
 */
#define PDSS_VBTR_CTRL_ADDRESS                              (0x400a2050)
#define PDSS_VBTR_CTRL                                      (*(volatile uint32_t *)(0x400a2050))
#define PDSS_VBTR_CTRL_DEFAULT                              (0x00000000)

/*
 * Start the VBTR Operation.
 * FW sets this bit to start the VBTR operation.
 * The operation starts about 3 VBTR clock cycle.
 * HW clears this bit when the current operation is complete or an exit request
 * has been processed and
 * the INTR8.VBTR_OPR_DONE/INTR8.VBTR_EXIT_DONE interrupt is cleared
 */
#define PDSS_VBTR_CTRL_START                                (1u << 0) /* <0:0> RW0C:RW1S:0: */


/*
 * Exit the current VBTR operation.
 * The operation (incr/decr) is immediately stoped.
 * The EXIT interrupt bit is getting set within one VBTR clock cycle of the
 * EXIT bit being set.
 * SW sets this bit to exit from the current VBTR operation.
 * HW clears this bit once the exit request has been processed and
 * the INTR8.VBTR_OPR_DONE/INTR8.VBTR_EXIT_DONE interrupt is cleared
 */
#define PDSS_VBTR_CTRL_EXIT                                 (1u << 1) /* <1:1> RW0C:RW1S:0: */


/*
 * VBUS Transition Source/Sink shadow registers
 */
#define PDSS_VBTR_SRC_SNK_OPR_VALUE_ADDRESS                 (0x400a2054)
#define PDSS_VBTR_SRC_SNK_OPR_VALUE                         (*(volatile uint32_t *)(0x400a2054))
#define PDSS_VBTR_SRC_SNK_OPR_VALUE_DEFAULT                 (0x00000000)

/*
 * Indicates Final SRC Register value after SRC Operation completion.
 */
#define PDSS_VBTR_SRC_SNK_OPR_VALUE_SRC_DAC_MASK            (0x0000007f) /* <0:6> RW:R:0: */
#define PDSS_VBTR_SRC_SNK_OPR_VALUE_SRC_DAC_POS             (0)


/*
 * Indicates Final SNK Register value after SNK Operation completion.
 */
#define PDSS_VBTR_SRC_SNK_OPR_VALUE_SNK_DAC_MASK            (0x03ff0000) /* <16:25> RW:R:0: */
#define PDSS_VBTR_SRC_SNK_OPR_VALUE_SNK_DAC_POS             (16)


/*
 * VBUS Transition Source Initial/Final value registers
 */
#define PDSS_VBTR_SRC_INIT_FIN_VALUE_ADDRESS                (0x400a2058)
#define PDSS_VBTR_SRC_INIT_FIN_VALUE                        (*(volatile uint32_t *)(0x400a2058))
#define PDSS_VBTR_SRC_INIT_FIN_VALUE_DEFAULT                (0x00000000)

/*
 * Indicates the Initial SRC Register value to be loaded into VBTR_SRC_SNK_OPR_VALUE
 * Register.
 */
#define PDSS_VBTR_SRC_INIT_FIN_VALUE_SRC_INIT_MASK          (0x0000007f) /* <0:6> R:RW:0: */
#define PDSS_VBTR_SRC_INIT_FIN_VALUE_SRC_INIT_POS           (0)


/*
 * Indicates the Final SRC Register value to be reached after operation completion.
 */
#define PDSS_VBTR_SRC_INIT_FIN_VALUE_SRC_FIN_MASK           (0x00007f00) /* <8:14> R:RW:0: */
#define PDSS_VBTR_SRC_INIT_FIN_VALUE_SRC_FIN_POS            (8)


/*
 * VBUS Transition Sink Initial/Final value registers
 */
#define PDSS_VBTR_SNK_INIT_FIN_VALUE_ADDRESS                (0x400a205c)
#define PDSS_VBTR_SNK_INIT_FIN_VALUE                        (*(volatile uint32_t *)(0x400a205c))
#define PDSS_VBTR_SNK_INIT_FIN_VALUE_DEFAULT                (0x00000000)

/*
 * Indicates the Initial SNK Register value to be loaded into VBTR_SRC_SNK_OPR_VALUE
 * Register.
 */
#define PDSS_VBTR_SNK_INIT_FIN_VALUE_SNK_INIT_MASK          (0x000003ff) /* <0:9> R:RW:0: */
#define PDSS_VBTR_SNK_INIT_FIN_VALUE_SNK_INIT_POS           (0)


/*
 * Indicates the Final SNK Register value to be reached after operation completion.
 */
#define PDSS_VBTR_SNK_INIT_FIN_VALUE_SNK_FIN_MASK           (0x03ff0000) /* <16:25> R:RW:0: */
#define PDSS_VBTR_SNK_INIT_FIN_VALUE_SNK_FIN_POS            (16)


/*
 * VBUS Transition Status registers
 */
#define PDSS_VBTR_STATUS_ADDRESS                            (0x400a2060)
#define PDSS_VBTR_STATUS                                    (*(volatile uint32_t *)(0x400a2060))
#define PDSS_VBTR_STATUS_DEFAULT                            (0x00000000)

/*
 * Status of Source vbus transition completion
 */
#define PDSS_VBTR_STATUS_SRC_DONE                           (1u << 0) /* <0:0> RW:R:0: */


/*
 * Status of Sink vbus transition completion
 */
#define PDSS_VBTR_STATUS_SNK_DONE                           (1u << 1) /* <1:1> RW:R:0: */


/*
 * Power Adapter Secondary Controller (PASC) Configuration Registers
 */
#define PDSS_PASC_CTRL_ADDRESS                              (0x400a2064)
#define PDSS_PASC_CTRL                                      (*(volatile uint32_t *)(0x400a2064))
#define PDSS_PASC_CTRL_DEFAULT                              (0x0000c144)

/*
 * Indicates Configuration of PASC.
 * 0 - PWM
 * 1 - SR_ONLY
 */
#define PDSS_PASC_CTRL_PASC_CFG                             (1u << 0) /* <0:0> R:RW:0: */


/*
 * Indicates operational mode of PASC. This register hasa shadow register
 * in logic
 * 00 - Fixed Frequency (FF)
 * 01 - Psuedo Fixed Frequency (PFF)
 * 10 - QR-Valley Mode (QR)
 */
#define PDSS_PASC_CTRL_PASC_MODE_MASK                       (0x00000006) /* <1:2> R:RW:2: */
#define PDSS_PASC_CTRL_PASC_MODE_POS                        (1)


/*
 * Indicates Dynamic Switching
 * 00 - No dynamic swicthing
 * 01 - FF <-> PFF
 * 10 - FF <-> QR
 */
#define PDSS_PASC_CTRL_PASC_DS_MASK                         (0x00000018) /* <3:4> R:RW:0: */
#define PDSS_PASC_CTRL_PASC_DS_POS                          (3)


/*
 * Indicates the width of PTDRV pulses during calibration.
 * Minimum value that can be programmed is 0x3
 */
#define PDSS_PASC_CTRL_CAL_WIDTH_MASK                       (0x000003e0) /* <5:9> R:RW:10: */
#define PDSS_PASC_CTRL_CAL_WIDTH_POS                        (5)


/*
 * Indicates the sepeartion time between consecutive calibration pulses.
 * Minimum value that can be programmed 0x5
 */
#define PDSS_PASC_CTRL_CAL_SEPARATION_MASK                  (0x0003fc00) /* <10:17> R:RW:48: */
#define PDSS_PASC_CTRL_CAL_SEPARATION_POS                   (10)


/*
 * Control bit for PWM out signal.
 * 0 - Logic uses Hard-Ip generated signal
 * 1 - Logic uses external GPIO signal.
 */
#define PDSS_PASC_CTRL_SEL_EXT_PWM_OUT                      (1u << 18) /* <18:18> R:RW:0: */


/*
 * Control bit for ZCD out signal.
 * 0 - Logic uses Hard-Ip generated signal
 * 1 - Logic uses external GPIO signal.
 */
#define PDSS_PASC_CTRL_SEL_EXT_ZCD_OUT                      (1u << 19) /* <19:19> R:RW:0: */


/*
 * Control bit for NSN out signal.
 * 0 - Logic uses Hard-Ip generated signal
 * 1 - Logic uses external GPIO signal.
 */
#define PDSS_PASC_CTRL_SEL_EXT_NSN_OUT                      (1u << 20) /* <20:20> R:RW:0: */


/*
 * Control bit for PEAKDET out signal.
 * 0 - Logic uses Hard-Ip generated signal
 * 1 - Logic uses external GPIO signal.
 */
#define PDSS_PASC_CTRL_SEL_EXT_PEAKDET_OUT                  (1u << 21) /* <21:21> R:RW:0: */


/*
 * Bit22: Disable nsn idle timeout check.
 * Bit30:23: Un-used
 */
#define PDSS_PASC_CTRL_PASC_SPARE_MASK                      (0x7fc00000) /* <22:30> R:RW:0: */
#define PDSS_PASC_CTRL_PASC_SPARE_POS                       (22)


/*
 * FW should set this bit to turn on the secondary controller power state
 * machine.
 */
#define PDSS_PASC_CTRL_PA_EN                                (1u << 31) /* <31:31> R:RW:0: */


/*
 * SR GDRV Control Register 0
 */
#define PDSS_SRGDRV_0_CTRL_ADDRESS                          (0x400a2068)
#define PDSS_SRGDRV_0_CTRL                                  (*(volatile uint32_t *)(0x400a2068))
#define PDSS_SRGDRV_0_CTRL_DEFAULT                          (0x0c980000)

/*
 * Controls the assertion delay of "gdrv_in" signal
 */
#define PDSS_SRGDRV_0_CTRL_GDRV_IN_ON_DLY_MASK              (0x0000003f) /* <0:5> R:RW:0: */
#define PDSS_SRGDRV_0_CTRL_GDRV_IN_ON_DLY_POS               (0)


/*
 * Controls the de-assertion delay of "gdrv_in" signal
 */
#define PDSS_SRGDRV_0_CTRL_GDRV_IN_OFF_DLY_MASK             (0x00000fc0) /* <6:11> R:RW:0: */
#define PDSS_SRGDRV_0_CTRL_GDRV_IN_OFF_DLY_POS              (6)


/*
 * Control bit for signal “gdrv_in”
 * 0 – Logic output is propagated as “gdrv_in”.
 * 1 – External GPIO input is propagated as “gdrv_in”
 */
#define PDSS_SRGDRV_0_CTRL_GDRV_IN_OUT_CTRL                 (1u << 12) /* <12:12> R:RW:0: */


/*
 * Override Register.
 * 1 - FW Override Enabled. GDRV_IN_OVR_VAL is driven on secondary gate driver
 */
#define PDSS_SRGDRV_0_CTRL_GDRV_IN_OVR                      (1u << 13) /* <13:13> R:RW:0: */


/*
 * Override Value for "gdrv_in" Signal
 */
#define PDSS_SRGDRV_0_CTRL_GDRV_IN_OVR_VAL                  (1u << 14) /* <14:14> R:RW:0: */


/*
 * Setting this override will prevent turn-off width check
 */
#define PDSS_SRGDRV_0_CTRL_GDRV_TURN_OFF_OVERRIDE           (1u << 15) /* <15:15> R:RW:0: */


/*
 * Secondary Width below which GDRV will be gated. The secondary width is
 * measured as the time from NSN to ZCD/ZCDF. This register has a shadow
 * register in Logic
 */
#define PDSS_SRGDRV_0_CTRL_GDRV_IN_TURN_OFF_WIDTH_MASK      (0x007f0000) /* <16:22> R:RW:24: */
#define PDSS_SRGDRV_0_CTRL_GDRV_IN_TURN_OFF_WIDTH_POS       (16)


/*
 * 0: When minimum width is violated GDRV is shut-off and only turned back
 * on when the INTR8.13 is cleared
 * 1: GDRV is only turned off for the current power cycle
 */
#define PDSS_SRGDRV_0_CTRL_GDRV_GATE_EVERY_CYCLE            (1u << 23) /* <23:23> R:RW:1: */


/*
 * Delay from gdrv max time out to nsn enable in SR-Mode only
 */
#define PDSS_SRGDRV_0_CTRL_GDRV_TMR_FOR_NSN_DLY_MASK        (0x7f000000) /* <24:30> R:RW:12: */
#define PDSS_SRGDRV_0_CTRL_GDRV_TMR_FOR_NSN_DLY_POS         (24)


/*
 * SR GDRV Control Register 1
 */
#define PDSS_SRGDRV_1_CTRL_ADDRESS                          (0x400a206c)
#define PDSS_SRGDRV_1_CTRL                                  (*(volatile uint32_t *)(0x400a206c))
#define PDSS_SRGDRV_1_CTRL_DEFAULT                          (0x00009c05)

/*
 * Minimum Pulse Width of "gdrv_in". This register has a shadow register
 * in Logic
 */
#define PDSS_SRGDRV_1_CTRL_GDRV_IN_MIN_WIDTH_MASK           (0x0000003f) /* <0:5> R:RW:5: */
#define PDSS_SRGDRV_1_CTRL_GDRV_IN_MIN_WIDTH_POS            (0)


/*
 * Maximum Pulse Width of "gdrv_in"
 */
#define PDSS_SRGDRV_1_CTRL_GDRV_IN_MAX_WIDTH_MASK           (0x0001ffc0) /* <6:16> R:RW:624: */
#define PDSS_SRGDRV_1_CTRL_GDRV_IN_MAX_WIDTH_POS            (6)


/*
 * Width of "gdrv_in" signal in the latest power cycle
 */
#define PDSS_SRGDRV_1_CTRL_GDRV_IN_STATUS_UPD_MASK          (0x0ffe0000) /* <17:27> RW:R:0: */
#define PDSS_SRGDRV_1_CTRL_GDRV_IN_STATUS_UPD_POS           (17)


/*
 * PASC-PWM Control Register 0
 */
#define PDSS_PASC_PWM_0_CTRL_ADDRESS                        (0x400a2070)
#define PDSS_PASC_PWM_0_CTRL                                (*(volatile uint32_t *)(0x400a2070))
#define PDSS_PASC_PWM_0_CTRL_DEFAULT                        (0x0c000000)

/*
 * Controls the assertion delay of "ptdrv_in" signal
 */
#define PDSS_PASC_PWM_0_CTRL_PTDRV_IN_ON_DLY_MASK           (0x0000003f) /* <0:5> R:RW:0: */
#define PDSS_PASC_PWM_0_CTRL_PTDRV_IN_ON_DLY_POS            (0)


/*
 * Controls the de-assertion delay of "ptdrv_in" signal
 */
#define PDSS_PASC_PWM_0_CTRL_PTDRV_IN_OFF_DLY_MASK          (0x00000fc0) /* <6:11> R:RW:0: */
#define PDSS_PASC_PWM_0_CTRL_PTDRV_IN_OFF_DLY_POS           (6)


/*
 * Setting this bit will gate the PTDRV on the negative edge of primary pulse.
 * FW can use this bit to smoothly transition control from logic. After setting
 * this bit, FW has to read the PTDRV GPIO to check if low. Then take control
 * of the GPIO
 */
#define PDSS_PASC_PWM_0_CTRL_PTDRV_GATE_FW                  (1u << 12) /* <12:12> R:RW:0: */


/*
 * Minimum Pulse width of "ptdrv_in" signal. This register has a shadow register
 * in logic.
 */
#define PDSS_PASC_PWM_0_CTRL_PTDRV_IN_MIN_WIDTH_MASK        (0x7f000000) /* <24:30> R:RW:12: */
#define PDSS_PASC_PWM_0_CTRL_PTDRV_IN_MIN_WIDTH_POS         (24)


/*
 * Control bit for signal “ptdrv_in”
 * 0 – Logic output is propagated as “ptdrv_in”.
 * 1 – External GPIO input is propagated as “ptdrv_in”
 */
#define PDSS_PASC_PWM_0_CTRL_PTDRV_IN_OUT_CTRL              (1u << 31) /* <31:31> R:RW:0: */


/*
 * PASC-PWM Control Register 1
 */
#define PDSS_PASC_PWM_1_CTRL_ADDRESS                        (0x400a2074)
#define PDSS_PASC_PWM_1_CTRL                                (*(volatile uint32_t *)(0x400a2074))
#define PDSS_PASC_PWM_1_CTRL_DEFAULT                        (0x000002a0)

/*
 * Maximum Pulse width of "ptdrv_in" signal. This register has a shadow register
 * in logic
 */
#define PDSS_PASC_PWM_1_CTRL_PTDRV_IN_MAX_WIDTH_MASK        (0x000007ff) /* <0:10> R:RW:672: */
#define PDSS_PASC_PWM_1_CTRL_PTDRV_IN_MAX_WIDTH_POS         (0)


/*
 * Width of "ptdrv_in" signal in the latest power cycle
 */
#define PDSS_PASC_PWM_1_CTRL_PTDRV_IN_STATUS_UPD_MASK       (0x003ff800) /* <11:21> RW:R:0: */
#define PDSS_PASC_PWM_1_CTRL_PTDRV_IN_STATUS_UPD_POS        (11)


/*
 * PASC-PWM Control Register 2
 */
#define PDSS_PASC_PWM_2_CTRL_ADDRESS                        (0x400a2078)
#define PDSS_PASC_PWM_2_CTRL                                (*(volatile uint32_t *)(0x400a2078))
#define PDSS_PASC_PWM_2_CTRL_DEFAULT                        (0x000bc218)

/*
 * If this bit is set, srgdrv is gated when primary is doing the burst pulses.
 */
#define PDSS_PASC_PWM_2_CTRL_GATE_SRGDRV_BURST_EN           (1u << 0) /* <0:0> R:RW:0: */


/*
 * Minimum pulse width of burst pulse.
 */
#define PDSS_PASC_PWM_2_CTRL_BURST_MIN_WIDTH_MASK           (0x000000fe) /* <1:7> R:RW:12: */
#define PDSS_PASC_PWM_2_CTRL_BURST_MIN_WIDTH_POS            (1)


/*
 * Burst frequency
 */
#define PDSS_PASC_PWM_2_CTRL_BURST_FREQ_MASK                (0x0007ff00) /* <8:18> R:RW:962: */
#define PDSS_PASC_PWM_2_CTRL_BURST_FREQ_POS                 (8)


/*
 * Burst pulse incremental delta value
 */
#define PDSS_PASC_PWM_2_CTRL_BURST_INCR_VAL_MASK            (0x00f80000) /* <19:23> R:RW:1: */
#define PDSS_PASC_PWM_2_CTRL_BURST_INCR_VAL_POS             (19)


/*
 * 1 – if burst exit happens when in audio range, move to normal mode
 */
#define PDSS_PASC_PWM_2_CTRL_ALLOW_BURST_EXIT_IN_AUDIO      (1u << 24) /* <24:24> R:RW:0: */


/*
 * Change_pwm – setting this bit will update the shadow registers for the
 * following when PTDRV is low
 * a. PWM_BURST_EXIT_SEL
 * b. PWM_DAC
 * c. PWM_LCLAMP_SEL
 * d. BURST_MIN_WIDTH
 * e. BURST_TRIM_VAL
 * f. SKIP_TRIM_VAL
 */
#define PDSS_PASC_PWM_2_CTRL_CHANGE_PWM                     (1u << 25) /* <25:25> RW1C:RW1S:0: */


/*
 * Setting this bit will update the shadow registres on DOUBLER_BYPASS at
 * GDRV negedge.
 */
#define PDSS_PASC_PWM_2_CTRL_DOUBLER_BYPASS_CHANGE          (1u << 26) /* <26:26> RW1C:RW1S:0: */


/*
 * PASC-MODE Control Register 0
 */
#define PDSS_MODE_0_CTRL_ADDRESS                            (0x400a207c)
#define PDSS_MODE_0_CTRL                                    (*(volatile uint32_t *)(0x400a207c))
#define PDSS_MODE_0_CTRL_DEFAULT                            (0x00000000)

/*
 * Controls the assertion delay of "pwm_reset" signal
 */
#define PDSS_MODE_0_CTRL_PWM_RESET_ON_DLY_MASK              (0x0000003f) /* <0:5> R:RW:0: */
#define PDSS_MODE_0_CTRL_PWM_RESET_ON_DLY_POS               (0)


/*
 * Controls the de-assertion delay of "pwm_reset" signal
 */
#define PDSS_MODE_0_CTRL_PWM_RESET_OFF_DLY_MASK             (0x00000fc0) /* <6:11> R:RW:0: */
#define PDSS_MODE_0_CTRL_PWM_RESET_OFF_DLY_POS              (6)


/*
 * Enables PWM CAP Modulation, dithering
 */
#define PDSS_MODE_0_CTRL_PWM_CAP_UPD_EN                     (1u << 20) /* <20:20> R:RW:0: */


/*
 * FW update set for "pwm_cap". FW can use this to set the PWM cap value
 * dynamically. When set, the value in PWM_CAP_FW_UPD_VAL is used.
 */
#define PDSS_MODE_0_CTRL_PWM_CAP_FW_UPD                     (1u << 21) /* <21:21> RW1C:RW1S:0: */


/*
 * FW update value for "pwm_cap"
 */
#define PDSS_MODE_0_CTRL_PWM_CAP_FW_UPD_VAL_MASK            (0x03c00000) /* <22:25> R:RW:0: */
#define PDSS_MODE_0_CTRL_PWM_CAP_FW_UPD_VAL_POS             (22)


/*
 * This register is used during trimming to create a pwm_reset deassertion.
 * This will be used to measure the PWM pulse width
 */
#define PDSS_MODE_0_CTRL_PWM_RESET_FW                       (1u << 27) /* <27:27> RW1C:RW1S:0: */


/*
 * Enables the frequency dithering in FF and PFF modes.
 */
#define PDSS_MODE_0_CTRL_FF_SPREAD_EN                       (1u << 28) /* <28:28> R:RW:0: */


/*
 * Increment value used in Frequency dithering.
 */
#define PDSS_MODE_0_CTRL_FF_SPREAD_INC_MASK                 (0xe0000000) /* <29:31> R:RW:0: */
#define PDSS_MODE_0_CTRL_FF_SPREAD_INC_POS                  (29)


/*
 * PASC-MODE Control Register 1
 */
#define PDSS_MODE_1_CTRL_ADDRESS                            (0x400a2080)
#define PDSS_MODE_1_CTRL                                    (*(volatile uint32_t *)(0x400a2080))
#define PDSS_MODE_1_CTRL_DEFAULT                            (0x000060f0)

/*
 * Fixed Frequency Value. This register has a shadow in logic
 */
#define PDSS_MODE_1_CTRL_FIX_FREQ_MASK                      (0x000007ff) /* <0:10> R:RW:240: */
#define PDSS_MODE_1_CTRL_FIX_FREQ_POS                       (0)


/*
 * Fixed Frequency Offset Value
 */
#define PDSS_MODE_1_CTRL_FIX_FREQ_OFFSET_MASK               (0x0001f800) /* <11:16> R:RW:12: */
#define PDSS_MODE_1_CTRL_FIX_FREQ_OFFSET_POS                (11)


/*
 * PASC-MODE Control Register 2
 */
#define PDSS_MODE_2_CTRL_ADDRESS                            (0x400a2084)
#define PDSS_MODE_2_CTRL                                    (*(volatile uint32_t *)(0x400a2084))
#define PDSS_MODE_2_CTRL_DEFAULT                            (0x001e10a0)

/*
 * This field set the max frequency possible in QR mode of operation. This
 * register has a shadow in logic
 */
#define PDSS_MODE_2_CTRL_VAR_TMIN_MASK                      (0x000007ff) /* <0:10> R:RW:160: */
#define PDSS_MODE_2_CTRL_VAR_TMIN_POS                       (0)


/*
 * This field set the min frequency possible in QR mode of operation. This
 * register has a shadow in logic
 */
#define PDSS_MODE_2_CTRL_VAR_TMAX_MASK                      (0x007ff800) /* <11:22> R:RW:962: */
#define PDSS_MODE_2_CTRL_VAR_TMAX_POS                       (11)


/*
 * PASC-MODE Control Register 3
 */
#define PDSS_MODE_3_CTRL_ADDRESS                            (0x400a2088)
#define PDSS_MODE_3_CTRL                                    (*(volatile uint32_t *)(0x400a2088))
#define PDSS_MODE_3_CTRL_DEFAULT                            (0x00005dc0)

/*
 * This register defines the min frequency of audio range.
 */
#define PDSS_MODE_3_CTRL_AUDIO_TMAX_MASK                    (0x001fffff) /* <0:20> R:RW:24000: */
#define PDSS_MODE_3_CTRL_AUDIO_TMAX_POS                     (0)


/*
 * PASC-MODE Control Register 4
 */
#define PDSS_MODE_4_CTRL_ADDRESS                            (0x400a208c)
#define PDSS_MODE_4_CTRL                                    (*(volatile uint32_t *)(0x400a208c))
#define PDSS_MODE_4_CTRL_DEFAULT                            (0x000004c3)

/*
 * This register defines the max frequency of audio range (default 20kHZ)
 */
#define PDSS_MODE_4_CTRL_AUDIO_TMIN_MASK                    (0x00001fff) /* <0:12> R:RW:1219: */
#define PDSS_MODE_4_CTRL_AUDIO_TMIN_POS                     (0)


/*
 * PEAK Generator Control Register 0
 */
#define PDSS_PEAKGEN_0_CTRL_ADDRESS                         (0x400a2090)
#define PDSS_PEAKGEN_0_CTRL                                 (*(volatile uint32_t *)(0x400a2090))
#define PDSS_PEAKGEN_0_CTRL_DEFAULT                         (0x00009024)

/*
 * Controls the assertion delay of "peak_reset" signal.
 */
#define PDSS_PEAKGEN_0_CTRL_PEAK_RESET_ON_DLY_MASK          (0x000001ff) /* <0:8> R:RW:36: */
#define PDSS_PEAKGEN_0_CTRL_PEAK_RESET_ON_DLY_POS           (0)


/*
 * Controls the "peak to peak" delay. This register has a shadow in logic.
 */
#define PDSS_PEAKGEN_0_CTRL_PEAK2PEAK_DLY_MASK              (0x0003fe00) /* <9:17> R:RW:72: */
#define PDSS_PEAKGEN_0_CTRL_PEAK2PEAK_DLY_POS               (9)


/*
 * Controls the Enabling of Valley based Peak Algorithm.
 * 1 - Valley based algorithm is used.
 * Note that this bit has to be disabled during the Peak memory update
 */
#define PDSS_PEAKGEN_0_CTRL_EN_PEAK_ALGO                    (1u << 18) /* <18:18> R:RW:0: */


/*
 * Enable bit for FW to update the peak.
 * 1 - FW Update Enabed
 */
#define PDSS_PEAKGEN_0_CTRL_FW_PEAK_UPD                     (1u << 19) /* <19:19> RW1C:RW1S:0: */


/*
 * FW specified peak. The peak used is programmed value + 1, i.e. 0 means
 * 1st peak
 */
#define PDSS_PEAKGEN_0_CTRL_FW_PEAK_NUM_MASK                (0x03f00000) /* <20:25> R:RW:0: */
#define PDSS_PEAKGEN_0_CTRL_FW_PEAK_NUM_POS                 (20)


/*
 * Setting this bit ignores the peaks from Hard-IP and uses only digital
 * computation
 */
#define PDSS_PEAKGEN_0_CTRL_PEAK_OVR                        (1u << 26) /* <26:26> R:RW:0: */


/*
 * This bit must be set before PA_EN is set to prevent calibration from happening.
 * When set, there will be 1 start pulse instead of 2
 */
#define PDSS_PEAKGEN_0_CTRL_CALIBRATE_OVERRIDE              (1u << 27) /* <27:27> R:RW:0: */


/*
 * This field specifies the pulse width of the peak reset signal in number
 * of clk_pasc clock cycles.
 */
#define PDSS_PEAKGEN_0_CTRL_PEAK_RESET_PULSE_MASK           (0xf0000000) /* <28:31> R:RW:0: */
#define PDSS_PEAKGEN_0_CTRL_PEAK_RESET_PULSE_POS            (28)


/*
 * PEAK Generator Control Register 1
 */
#define PDSS_PEAKGEN_1_CTRL_ADDRESS                         (0x400a2094)
#define PDSS_PEAKGEN_1_CTRL                                 (*(volatile uint32_t *)(0x400a2094))
#define PDSS_PEAKGEN_1_CTRL_DEFAULT                         (0x0541209b)

/*
 * If this bit is cleared, the peak_kill signal from Hard-IP is ignored.
 * The peaks to be used will be dynamically determined by either PEAK_EXPECT_DLY
 * timeout or STOP_HIP_ON_NTH_PEAK
 */
#define PDSS_PEAKGEN_1_CTRL_PEAK_KILL_EN                    (1u << 0) /* <0:0> R:RW:1: */


/*
 * Enables the use of updating the trim values for Skip.
 * 1 - Skip Trim Update is enabled.
 */
#define PDSS_PEAKGEN_1_CTRL_SKIP_TRIM_EN                    (1u << 1) /* <1:1> R:RW:1: */


/*
 * Indicates the value to be used for SKIP TRIM bits once "skip_out" is encountered.
 */
#define PDSS_PEAKGEN_1_CTRL_BURST_TRIM_VAL_MASK             (0x0000003c) /* <2:5> R:RW:6: */
#define PDSS_PEAKGEN_1_CTRL_BURST_TRIM_VAL_POS              (2)


/*
 * Indicates the value of SKIP TRIM bits before "skip_out" and after "burst_exit"
 * is encountered.
 */
#define PDSS_PEAKGEN_1_CTRL_SKIP_TRIM_VAL_MASK              (0x000003c0) /* <6:9> R:RW:2: */
#define PDSS_PEAKGEN_1_CTRL_SKIP_TRIM_VAL_POS               (6)


/*
 * Indicates the Time between "zcd_out" encountered to the first peak. This
 * is used if full digital scheme of peak generation is used. This register
 * has a shadow in logic
 */
#define PDSS_PEAKGEN_1_CTRL_ZCD_TO_PEAK_TIME_MASK           (0x000ff800) /* <11:19> R:RW:36: */
#define PDSS_PEAKGEN_1_CTRL_ZCD_TO_PEAK_TIME_POS            (11)


/*
 * Indicates the maximum time (expected) between two consecutive peaks. If
 * this timer times out, digital peak generation will take over.
 */
#define PDSS_PEAKGEN_1_CTRL_PEAK_EXPECT_DLY_MASK            (0x1ff00000) /* <20:28> R:RW:84: */
#define PDSS_PEAKGEN_1_CTRL_PEAK_EXPECT_DLY_POS             (20)


/*
 * PEAK Generator Control Register 2
 */
#define PDSS_PEAKGEN_2_CTRL_ADDRESS                         (0x400a2098)
#define PDSS_PEAKGEN_2_CTRL                                 (*(volatile uint32_t *)(0x400a2098))
#define PDSS_PEAKGEN_2_CTRL_DEFAULT                         (0x0000001f)

/*
 * Setting this bit will ignore Hard IP generated peaks after the programmed
 * number of STOP_PEAK
 */
#define PDSS_PEAKGEN_2_CTRL_STOP_HIP_ON_NTH_PEAK            (1u << 0) /* <0:0> R:RW:1: */


/*
 * Peak at which Hard IP output is ignored. Programmed peak + 1 is used.
 */
#define PDSS_PEAKGEN_2_CTRL_STOP_PEAK_MASK                  (0x0000007e) /* <1:6> R:RW:15: */
#define PDSS_PEAKGEN_2_CTRL_STOP_PEAK_POS                   (1)


/*
 * Feed Forward Control Register
 */
#define PDSS_FEEDFWD_CTRL_ADDRESS                           (0x400a209c)
#define PDSS_FEEDFWD_CTRL                                   (*(volatile uint32_t *)(0x400a209c))
#define PDSS_FEEDFWD_CTRL_DEFAULT                           (0x040c8700)

/*
 * Control bit for signal “feedfwd_pre_sw_en” signal
 * 0 – Logic output is propagated as “feedfwd_pre_sw_en”.
 * 1 – External GPIO input is propagated as “feedfwd_pre_sw_en”
 */
#define PDSS_FEEDFWD_CTRL_FEEDFWD_PRE_SW_EN_OUT_CTRL        (1u << 0) /* <0:0> R:RW:0: */


/*
 * Control bit for signal “feedfwd_sw_en” signal
 * 0 – Logic output is propagated as “feedfwd_sw_en”.
 * 1 – External GPIO input is propagated as “feedfwd_sw_en”
 */
#define PDSS_FEEDFWD_CTRL_FEEDFWD_SW_EN_OUT_CTRL            (1u << 1) /* <1:1> R:RW:0: */


/*
 * FW Override bit for "feedfwd_sw_en"
 */
#define PDSS_FEEDFWD_CTRL_FEEDFWD_SW_EN_OVR                 (1u << 2) /* <2:2> R:RW:0: */


/*
 * FW Override value for "feedfwd_sw_en"
 */
#define PDSS_FEEDFWD_CTRL_FEEDFWD_SW_EN_OVR_VAL             (1u << 3) /* <3:3> R:RW:0: */


/*
 * FW Override bit for "feedfwd_pre_sw_en"
 */
#define PDSS_FEEDFWD_CTRL_FEEDFWD_PRE_SW_EN_OVR             (1u << 4) /* <4:4> R:RW:0: */


/*
 * FW Override value for "feedfwd_pre_sw_en"
 */
#define PDSS_FEEDFWD_CTRL_FEEDFWD_PRE_SW_EN_OVR_VAL         (1u << 5) /* <5:5> R:RW:0: */


/*
 * Controls the assertion delay of "feedfwd_pre_sw_en" signal
 */
#define PDSS_FEEDFWD_CTRL_FEEDFWD_PRE_SW_EN_ON_DLY_MASK     (0x00003f00) /* <8:13> R:RW:7: */
#define PDSS_FEEDFWD_CTRL_FEEDFWD_PRE_SW_EN_ON_DLY_POS      (8)


/*
 * Controls the de-assertion delay of "feedfwd_pre_sw_en" signal
 */
#define PDSS_FEEDFWD_CTRL_FEEDFWD_PRE_SW_EN_OFF_DLY_MASK    (0x0003c000) /* <14:17> R:RW:2: */
#define PDSS_FEEDFWD_CTRL_FEEDFWD_PRE_SW_EN_OFF_DLY_POS     (14)


/*
 * Controls the width of "feedfwd_sw_en" signal.
 */
#define PDSS_FEEDFWD_CTRL_FEEDFWD_PRE_SW_EN_WIDTH_MASK      (0x01fc0000) /* <18:24> R:RW:3: */
#define PDSS_FEEDFWD_CTRL_FEEDFWD_PRE_SW_EN_WIDTH_POS       (18)


/*
 * Controls the assertion delay of "feedfwd_sw_en" signal after the assertion
 * of "feedfwd_pre_sw_en" signal.
 */
#define PDSS_FEEDFWD_CTRL_FEEDFWD_SW_EN_ON_DLY_MASK         (0x7e000000) /* <25:30> R:RW:2: */
#define PDSS_FEEDFWD_CTRL_FEEDFWD_SW_EN_ON_DLY_POS          (25)


/*
 * Disables Feedforward width calculation. Feedforward is disabled on PTDRV
 * negedge only
 */
#define PDSS_FEEDFWD_CTRL_FEEDFWD_NO_FEEDFWD_DEASSERT_ON_WIDTH    (1u << 31) /* <31:31> R:RW:0: */


/*
 * PASC Hard-Ip Sequence Generator Control Register 0
 */
#define PDSS_HIP_SEQ_GEN_0_CTRL_ADDRESS                     (0x400a20a0)
#define PDSS_HIP_SEQ_GEN_0_CTRL                             (*(volatile uint32_t *)(0x400a20a0))
#define PDSS_HIP_SEQ_GEN_0_CTRL_DEFAULT                     (0x00060000)

/*
 * Controls assertion delay of "nsn_en" signal
 */
#define PDSS_HIP_SEQ_GEN_0_CTRL_NSN_EN_ON_DLY_MASK          (0x000000ff) /* <0:7> R:RW:0: */
#define PDSS_HIP_SEQ_GEN_0_CTRL_NSN_EN_ON_DLY_POS           (0)


/*
 * Controls de-assertion delay of "nsn_en" signal
 */
#define PDSS_HIP_SEQ_GEN_0_CTRL_NSN_EN_OFF_DLY_MASK         (0x00003f00) /* <8:13> R:RW:0: */
#define PDSS_HIP_SEQ_GEN_0_CTRL_NSN_EN_OFF_DLY_POS          (8)


/*
 * Controls assertion delay of "zcd_en" signal
 */
#define PDSS_HIP_SEQ_GEN_0_CTRL_ZCD_EN_ON_DLY_MASK          (0x003fc000) /* <14:21> R:RW:24: */
#define PDSS_HIP_SEQ_GEN_0_CTRL_ZCD_EN_ON_DLY_POS           (14)


/*
 * Controls de-assertion delay of "zcd_en" signal
 */
#define PDSS_HIP_SEQ_GEN_0_CTRL_ZCD_EN_OFF_DLY_MASK         (0x0fc00000) /* <22:27> R:RW:0: */
#define PDSS_HIP_SEQ_GEN_0_CTRL_ZCD_EN_OFF_DLY_POS          (22)


/*
 * Override Register for "nsn_en" signal
 */
#define PDSS_HIP_SEQ_GEN_0_CTRL_NSN_EN_OVR                  (1u << 28) /* <28:28> R:RW:0: */


/*
 * Override value for "nsn_en" signal
 */
#define PDSS_HIP_SEQ_GEN_0_CTRL_NSN_EN_OVR_VAL              (1u << 29) /* <29:29> R:RW:0: */


/*
 * Override Register for "zcd_en" signal
 */
#define PDSS_HIP_SEQ_GEN_0_CTRL_ZCD_EN_OVR                  (1u << 30) /* <30:30> R:RW:0: */


/*
 * Override value for "zcd_en" signal
 */
#define PDSS_HIP_SEQ_GEN_0_CTRL_ZCD_EN_OVR_VAL              (1u << 31) /* <31:31> R:RW:0: */


/*
 * PASC Hard-Ip Sequence Generator Control Register 1
 */
#define PDSS_HIP_SEQ_GEN_1_CTRL_ADDRESS                     (0x400a20a4)
#define PDSS_HIP_SEQ_GEN_1_CTRL                             (*(volatile uint32_t *)(0x400a20a4))
#define PDSS_HIP_SEQ_GEN_1_CTRL_DEFAULT                     (0x0000c005)

/*
 * Controls assertion delay of "zcdf_en" signal
 */
#define PDSS_HIP_SEQ_GEN_1_CTRL_ZCDF_EN_ON_DLY_MASK         (0x000000ff) /* <0:7> R:RW:5: */
#define PDSS_HIP_SEQ_GEN_1_CTRL_ZCDF_EN_ON_DLY_POS          (0)


/*
 * Controls de-assertion delay of "zcdf_en" signal
 */
#define PDSS_HIP_SEQ_GEN_1_CTRL_ZCDF_EN_OFF_DLY_MASK        (0x00003f00) /* <8:13> R:RW:0: */
#define PDSS_HIP_SEQ_GEN_1_CTRL_ZCDF_EN_OFF_DLY_POS         (8)


/*
 * Controls assertion delay of "peakdet_sw_en" signal
 */
#define PDSS_HIP_SEQ_GEN_1_CTRL_PEAKDET_SW_EN_ON_DLY_MASK    (0x000fc000) /* <14:19> R:RW:3: */
#define PDSS_HIP_SEQ_GEN_1_CTRL_PEAKDET_SW_EN_ON_DLY_POS    (14)


/*
 * Controls de-assertion delay of "peakdet_sw_en" signal
 */
#define PDSS_HIP_SEQ_GEN_1_CTRL_PEAKDET_SW_EN_OFF_DLY_MASK    (0x03f00000) /* <20:25> R:RW:0: */
#define PDSS_HIP_SEQ_GEN_1_CTRL_PEAKDET_SW_EN_OFF_DLY_POS    (20)


/*
 * Override Register for "zcdf_en" signal
 */
#define PDSS_HIP_SEQ_GEN_1_CTRL_ZCDF_EN_OVR                 (1u << 26) /* <26:26> R:RW:0: */


/*
 * Override value for "zcdf_en" signal
 */
#define PDSS_HIP_SEQ_GEN_1_CTRL_ZCDF_EN_OVR_VAL             (1u << 27) /* <27:27> R:RW:0: */


/*
 * Override Register for "peakdet_sw_en" signal
 */
#define PDSS_HIP_SEQ_GEN_1_CTRL_PEAKDET_SW_EN_OVR           (1u << 28) /* <28:28> R:RW:0: */


/*
 * Override value for "peakdet_sw_en" signal
 */
#define PDSS_HIP_SEQ_GEN_1_CTRL_PEAKDET_SW_EN_OVR_VAL       (1u << 29) /* <29:29> R:RW:0: */


/*
 * Controls the use of either "zcd_out" or "zcdf" for the assertion of "peakdet_sw_en".
 * 0 - use zcdf
 * 1 - use zcd_out or zcdf_out
 */
#define PDSS_HIP_SEQ_GEN_1_CTRL_ZCD_OUT_FOR_PEAK            (1u << 30) /* <30:30> R:RW:0: */


/*
 * In SR-Only Mode wait for FF OV to enable NSN
 */
#define PDSS_HIP_SEQ_GEN_1_CTRL_FF_OV_FOR_NSN_EN            (1u << 31) /* <31:31> R:RW:0: */


/*
 * PASC Hard-Ip Sequence Generator Control Register 2
 */
#define PDSS_HIP_SEQ_GEN_2_CTRL_ADDRESS                     (0x400a20a8)
#define PDSS_HIP_SEQ_GEN_2_CTRL                             (*(volatile uint32_t *)(0x400a20a8))
#define PDSS_HIP_SEQ_GEN_2_CTRL_DEFAULT                     (0x0006000a)

/*
 * Controls assertion delay of "peakdet_en" signal
 */
#define PDSS_HIP_SEQ_GEN_2_CTRL_PEAKDET_EN_ON_DLY_MASK      (0x0000003f) /* <0:5> R:RW:10: */
#define PDSS_HIP_SEQ_GEN_2_CTRL_PEAKDET_EN_ON_DLY_POS       (0)


/*
 * Override Register for "peakdet_en" signal
 */
#define PDSS_HIP_SEQ_GEN_2_CTRL_PEAKDET_EN_OVR              (1u << 6) /* <6:6> R:RW:0: */


/*
 * Override value for "peakdet_en" signal
 */
#define PDSS_HIP_SEQ_GEN_2_CTRL_PEAKDET_EN_OVR_VAL          (1u << 7) /* <7:7> R:RW:0: */


/*
 * Override Register for HIP SEQ GEN State
 */
#define PDSS_HIP_SEQ_GEN_2_CTRL_STATE_OVR                   (1u << 8) /* <8:8> R:RW:0: */


/*
 * Override Value for HIP SEQ GEN State
 */
#define PDSS_HIP_SEQ_GEN_2_CTRL_STATE_OVR_VAL_MASK          (0x00000e00) /* <9:11> R:RW:0: */
#define PDSS_HIP_SEQ_GEN_2_CTRL_STATE_OVR_VAL_POS           (9)


/*
 * Controls the time duration for which state machine waits for "nsn_out"
 * to arrive.
 */
#define PDSS_HIP_SEQ_GEN_2_CTRL_NSN_IDLE_TIME_MASK          (0x001ff000) /* <12:20> R:RW:96: */
#define PDSS_HIP_SEQ_GEN_2_CTRL_NSN_IDLE_TIME_POS           (12)


/*
 * When set ZCD is used as ZCDF
 */
#define PDSS_HIP_SEQ_GEN_2_CTRL_USE_ZCDF_AS_ZCD             (1u << 30) /* <30:30> R:RW:0: */


/*
 * When set ZCDF is used as ZCD
 */
#define PDSS_HIP_SEQ_GEN_2_CTRL_USE_ZCD_AS_ZCDF             (1u << 31) /* <31:31> R:RW:0: */


/*
 * PASC Status registers 0
 */
#define PDSS_PASC_STATUS_0_ADDRESS                          (0x400a20ac)
#define PDSS_PASC_STATUS_0                                  (*(volatile uint32_t *)(0x400a20ac))
#define PDSS_PASC_STATUS_0_DEFAULT                          (0x00000000)

/*
 * Indicates the status of current state within Hard-Ip Sequence Generator
 * block.
 */
#define PDSS_PASC_STATUS_0_STATE_STATUS_MASK                (0x00000007) /* <0:2> RW:R:0: */
#define PDSS_PASC_STATUS_0_STATE_STATUS_POS                 (0)


/*
 * Indicates the Turn around delay measured by RTL during calibration.
 */
#define PDSS_PASC_STATUS_0_TURN_AROUND_DELAY_MASK           (0x00003ff8) /* <3:13> RW:R:0: */
#define PDSS_PASC_STATUS_0_TURN_AROUND_DELAY_POS            (3)


/*
 * Indicates the calibrated value of ZCD to first peak.
 */
#define PDSS_PASC_STATUS_0_ZCD_TO_PEAK_CAL_VAL_MASK         (0x7fc00000) /* <22:30> RW:R:0: */
#define PDSS_PASC_STATUS_0_ZCD_TO_PEAK_CAL_VAL_POS          (22)


/*
 * PASC Status registers 1
 */
#define PDSS_PASC_STATUS_1_ADDRESS                          (0x400a20b0)
#define PDSS_PASC_STATUS_1                                  (*(volatile uint32_t *)(0x400a20b0))
#define PDSS_PASC_STATUS_1_DEFAULT                          (0x00000000)

/*
 * Indicates the "pwm_reset" width. (Indirectly indicates the load)
 */
#define PDSS_PASC_STATUS_1_PWM_RESET_WIDTH_MASK             (0x000003ff) /* <0:9> RW:R:0: */
#define PDSS_PASC_STATUS_1_PWM_RESET_WIDTH_POS              (0)


/*
 * Indicates the "pwm_reset" width for current power cycle
 */
#define PDSS_PASC_STATUS_1_PWM_PULSE_WIDTH_FOR_PEAK_MASK    (0x001ffc00) /* <10:20> RW:R:0: */
#define PDSS_PASC_STATUS_1_PWM_PULSE_WIDTH_FOR_PEAK_POS     (10)


/*
 * Indicates the peak value for current power cycle
 */
#define PDSS_PASC_STATUS_1_CURR_PEAK_MASK                   (0x07e00000) /* <21:26> RW:R:0: */
#define PDSS_PASC_STATUS_1_CURR_PEAK_POS                    (21)


/*
 * PASC Status registers 2
 */
#define PDSS_PASC_STATUS_2_ADDRESS                          (0x400a20b4)
#define PDSS_PASC_STATUS_2                                  (*(volatile uint32_t *)(0x400a20b4))
#define PDSS_PASC_STATUS_2_DEFAULT                          (0x00000000)

/*
 * Indicates Peak to Peak to Calibration Value 0. Used by FW for average
 * calculation.
 */
#define PDSS_PASC_STATUS_2_P2P_CAL_VAL0_MASK                (0x000001ff) /* <0:8> RW:R:0: */
#define PDSS_PASC_STATUS_2_P2P_CAL_VAL0_POS                 (0)


/*
 * Indicates Peak to Peak to Calibration Value 1. Used by FW for average
 * calculation.
 */
#define PDSS_PASC_STATUS_2_P2P_CAL_VAL1_MASK                (0x0003fe00) /* <9:17> RW:R:0: */
#define PDSS_PASC_STATUS_2_P2P_CAL_VAL1_POS                 (9)


/*
 * Indicates Peak to Peak to Calibration Value 2. Used by FW for average
 * calculation.
 */
#define PDSS_PASC_STATUS_2_P2P_CAL_VAL2_MASK                (0x07fc0000) /* <18:26> RW:R:0: */
#define PDSS_PASC_STATUS_2_P2P_CAL_VAL2_POS                 (18)


/*
 * PASC Status registers 3
 */
#define PDSS_PASC_STATUS_3_ADDRESS                          (0x400a20b8)
#define PDSS_PASC_STATUS_3                                  (*(volatile uint32_t *)(0x400a20b8))
#define PDSS_PASC_STATUS_3_DEFAULT                          (0x00000000)

/*
 * Indicates the Peak to Reset (peak_reset) Calibration Value 0. Used by
 * FW for average calculation.
 */
#define PDSS_PASC_STATUS_3_PEAK_RST_CAL_VAL0_MASK           (0x000001ff) /* <0:8> RW:R:0: */
#define PDSS_PASC_STATUS_3_PEAK_RST_CAL_VAL0_POS            (0)


/*
 * Indicates the Peak to Reset (peak_reset) Calibration Value 1. Used by
 * FW for average calculation.
 */
#define PDSS_PASC_STATUS_3_PEAK_RST_CAL_VAL1_MASK           (0x0003fe00) /* <9:17> RW:R:0: */
#define PDSS_PASC_STATUS_3_PEAK_RST_CAL_VAL1_POS            (9)


/*
 * Indicates the Peak to Reset (peak_reset) Calibration Value 2. Used by
 * FW for average calculation.
 */
#define PDSS_PASC_STATUS_3_PEAK_RST_CAL_VAL2_MASK           (0x07fc0000) /* <18:26> RW:R:0: */
#define PDSS_PASC_STATUS_3_PEAK_RST_CAL_VAL2_POS            (18)


/*
 * PASC Status registers 4
 */
#define PDSS_PASC_STATUS_4_ADDRESS                          (0x400a20bc)
#define PDSS_PASC_STATUS_4                                  (*(volatile uint32_t *)(0x400a20bc))
#define PDSS_PASC_STATUS_4_DEFAULT                          (0x00000000)

/*
 * Indicates Peak to Peak to Calibration Value 3. Used by FW for average
 * calculation.
 */
#define PDSS_PASC_STATUS_4_P2P_CAL_VAL3_MASK                (0x000001ff) /* <0:8> RW:R:0: */
#define PDSS_PASC_STATUS_4_P2P_CAL_VAL3_POS                 (0)


/*
 * Indicates the Peak to Reset (peak_reset) Calibration Value 3. Used by
 * FW for average calculation.
 */
#define PDSS_PASC_STATUS_4_PEAK_RST_CAL_VAL3_MASK           (0x0003fe00) /* <9:17> RW:R:0: */
#define PDSS_PASC_STATUS_4_PEAK_RST_CAL_VAL3_POS            (9)


/*
 * PASC DDFT MUX
 */
#define PDSS_PASC_DDFT_MUX_ADDRESS                          (0x400a20c0)
#define PDSS_PASC_DDFT_MUX                                  (*(volatile uint32_t *)(0x400a20c0))
#define PDSS_PASC_DDFT_MUX_DEFAULT                          (0x00000000)

/*
 * 31:28   MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.skip_trim,
 * 27      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.feedfwd_pre_sw_en,
 * 26:22   MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.pwm_cap,
 * 21      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.nsn_en,
 * 20      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.zcd_en,
 * 19      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.zcdf_en,
 * 18      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.peakdet_sw_en,
 * 17      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.peakdet_en,
 * 16      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.peak_reset,
 * 15      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.pwm_reset,
 * 14      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.ptdrv_in,
 * 13      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.feedfwd_sw_en,
 * 12      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.gdrv_in,
 * 11      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.pulse_disable,
 * 10      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.burst_enable,
 * 9       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.intr_fix_freq_timeout_set,
 * 8       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.intr_var_tmin_timeout_set,
 * 7       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.intr_var_tmax_timeout_set,
 * 6       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.intr_aud_tmin_timeout_set,
 * 5       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.intr_aud_tmax_timeout_set,
 * 4       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.gdrv_in_max_width_timeout,
 * 3       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.ptdrv_in_max_width_timeout,
 * 2       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.skip_entry_filt_posedge_sync,
 * 1       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.skip_entry_filt_negedge_sync,
 * 0       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.burst_exit_filt_posedge_sync
 */
#define PDSS_PASC_DDFT_MUX_DDFT0_SEL_MASK                   (0x0000003f) /* <0:5> R:RW:0: */
#define PDSS_PASC_DDFT_MUX_DDFT0_SEL_POS                    (0)


/*
 * 31:28   MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.skip_trim,
 * 27      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.feedfwd_pre_sw_en,
 * 26:22   MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.pwm_cap,
 * 21      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.nsn_en,
 * 20      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.zcd_en,
 * 19      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.zcdf_en,
 * 18      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.peakdet_sw_en,
 * 17      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.peakdet_en,
 * 16      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.peak_reset,
 * 15      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.pwm_reset,
 * 14      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.ptdrv_in,
 * 13      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.feedfwd_sw_en,
 * 12      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.gdrv_in,
 * 11      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.pulse_disable,
 * 10      MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.burst_enable,
 * 9       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.intr_fix_freq_timeout_set,
 * 8       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.intr_var_tmin_timeout_set,
 * 7       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.intr_var_tmax_timeout_set,
 * 6       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.intr_aud_tmin_timeout_set,
 * 5       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.intr_aud_tmax_timeout_set,
 * 4       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.gdrv_in_max_width_timeout,
 * 3       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.ptdrv_in_max_width_timeout,
 * 2       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.skip_entry_filt_posedge_sync,
 * 1       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.skip_entry_filt_negedge_sync,
 * 0       MXUSBPD_MMIO_INST.y_pasc_en.u_mxusbpd_pasc.burst_exit_filt_posedge_sync
 */
#define PDSS_PASC_DDFT_MUX_DDFT1_SEL_MASK                   (0x00000fc0) /* <6:11> R:RW:0: */
#define PDSS_PASC_DDFT_MUX_DDFT1_SEL_POS                    (6)


/*
 * PASC GPIO DDFT Selections
 */
#define PDSS_PASC_GPIO_DDFT_MUX_ADDRESS                     (0x400a20c4)
#define PDSS_PASC_GPIO_DDFT_MUX                             (*(volatile uint32_t *)(0x400a20c4))
#define PDSS_PASC_GPIO_DDFT_MUX_DEFAULT                     (0x00000000)

/*
 * 14 MXUSBPD_MMIO_INST.y_pasc_en.u_srsense_nsn_out_change.filt_out_gpio
 * 13 MXUSBPD_MMIO_INST.y_pasc_en.u_srsense_zcd_out_change.filt_out_gpio
 * 12 MXUSBPD_MMIO_INST.y_pasc_en.u_pds_vreg_vbus_change.filt_out_gpio
 * 11  MXUSBPD_MMIO_INST.y_pasc_en.u_pds_scp_change.filt_out_gpio
 * 10 MXUSBPD_MMIO_INST.y_pasc_en.u_pwm_skip_out_change.filt_out_gpio
 * 9 MXUSBPD_MMIO_INST.y_pasc_en.u_pwm_burst_exit_out_change.filt_out_gpio
 * 8 MXUSBPD_MMIO_INST.y_pasc_en.u_pwm_out_change.filt_out_gpio
 * 7 MXUSBPD_MMIO_INST.y_pasc_en.u_srsense_zcdf_out_change.filt_out_gpio
 * 6 MXUSBPD_MMIO_INST.y_pasc_en.u_srsense_ovp_change.filt_out_gpio
 * 5 MXUSBPD_MMIO_INST.y_pasc_en.u_srsense_peakdet_out_change.filt_out_gpio
 * 4 MXUSBPD_MMIO_INST.y_pasc_en.u_srsense_peakdet_rst_out_change.filt_out_gpio
 * 3 MXUSBPD_MMIO_INST.y_pasc_en.u_srsense_peakdet_clcmp_raw_out_change.filt_out_gpio
 * 2 MXUSBPD_MMIO_INST.y_pasc_en.u_ea_cc_flag_change.filt_out_gpio
 * 1  MXUSBPD_MMIO_INST.y_pasc_en.u_ff_ov_change.filt_out_gpio
 * 0 MXUSBPD_MMIO_INST.y_pasc_en.u_ff_uv_change.filt_out_gpio
 */
#define PDSS_PASC_GPIO_DDFT_MUX_GPIO_DDFT0_SEL_MASK         (0x0000007f) /* <0:6> R:RW:0: */
#define PDSS_PASC_GPIO_DDFT_MUX_GPIO_DDFT0_SEL_POS          (0)


/*
 * 14 MXUSBPD_MMIO_INST.y_pasc_en.u_srsense_nsn_out_change.filt_out_gpio
 * 13 MXUSBPD_MMIO_INST.y_pasc_en.u_srsense_zcd_out_change.filt_out_gpio
 * 12 MXUSBPD_MMIO_INST.y_pasc_en.u_pds_vreg_vbus_change.filt_out_gpio
 * 11  MXUSBPD_MMIO_INST.y_pasc_en.u_pds_scp_change.filt_out_gpio
 * 10 MXUSBPD_MMIO_INST.y_pasc_en.u_pwm_skip_out_change.filt_out_gpio
 * 9 MXUSBPD_MMIO_INST.y_pasc_en.u_pwm_burst_exit_out_change.filt_out_gpio
 * 8 MXUSBPD_MMIO_INST.y_pasc_en.u_pwm_out_change.filt_out_gpio
 * 7 MXUSBPD_MMIO_INST.y_pasc_en.u_srsense_zcdf_out_change.filt_out_gpio
 * 6 MXUSBPD_MMIO_INST.y_pasc_en.u_srsense_ovp_change.filt_out_gpio
 * 5 MXUSBPD_MMIO_INST.y_pasc_en.u_srsense_peakdet_out_change.filt_out_gpio
 * 4 MXUSBPD_MMIO_INST.y_pasc_en.u_srsense_peakdet_rst_out_change.filt_out_gpio
 * 3 MXUSBPD_MMIO_INST.y_pasc_en.u_srsense_peakdet_clcmp_raw_out_change.filt_out_gpio
 * 2 MXUSBPD_MMIO_INST.y_pasc_en.u_ea_cc_flag_change.filt_out_gpio
 * 1  MXUSBPD_MMIO_INST.y_pasc_en.u_ff_ov_change.filt_out_gpio
 * 0 MXUSBPD_MMIO_INST.y_pasc_en.u_ff_uv_change.filt_out_gpio
 */
#define PDSS_PASC_GPIO_DDFT_MUX_GPIO_DDFT1_SEL_MASK         (0x00007f00) /* <8:14> R:RW:0: */
#define PDSS_PASC_GPIO_DDFT_MUX_GPIO_DDFT1_SEL_POS          (8)


/*
 * PEAK SRAM Data
 */
#define PDSS_PEAK_MEM_DATA_ADDRESS(n)                       (0x400a3010 + ((n) * (0x0004)))
#define PDSS_PEAK_MEM_DATA(n)                               (*(volatile uint32_t *)(0x400a3010 + ((n) * 0x0004)))
#define PDSS_PEAK_MEM_DATA_DEFAULT                          (0x00000000)

/*
 * Indicates the Data from Memory (SRAM) used for Valley - Based peak selection
 * algortihm. The memeory is organised as 16-rows of 2 bytes each. The rows
 * indicate the peak used. The MSB byte is the increment value used at that
 * peak and LSB byte is the decrement value used at that peak.
 */
#define PDSS_PEAK_MEM_DATA_DATA_MASK                        (0xffffffff) /* <0:31> R:RW:0: */
#define PDSS_PEAK_MEM_DATA_DATA_POS                         (0)


/*
 * USBPD Hard IP C-connector Trim Register0. Production trims stored in flash
 */
#define PDSS_TRIM_CC_0_ADDRESS                              (0x400aff00)
#define PDSS_TRIM_CC_0                                      (*(volatile uint32_t *)(0x400aff00))
#define PDSS_TRIM_CC_0_DEFAULT                              (0x00000000)

/*
 * Trim bits for Driver termination impedance. BROS describes the step
 */
#define PDSS_TRIM_CC_0_ZDRV_TRIM_MASK                       (0x00000007) /* <0:2> R:RW:0: */
#define PDSS_TRIM_CC_0_ZDRV_TRIM_POS                        (0)


/*
 * Trim bits for TX Driver rise/fall slew rate. 00 for minimum and 11 for
 * fastest slew rate.
 */
#define PDSS_TRIM_CC_0_TX_TRIM_MASK                         (0x00000018) /* <3:4> R:RW:0: */
#define PDSS_TRIM_CC_0_TX_TRIM_POS                          (3)


/*
 * USBPD Hard IP C-connector Trim Register1. Production trims stored in flash
 */
#define PDSS_TRIM_CC_1_ADDRESS                              (0x400aff04)
#define PDSS_TRIM_CC_1                                      (*(volatile uint32_t *)(0x400aff04))
#define PDSS_TRIM_CC_1_DEFAULT                              (0x00000000)

/*
 * Trim bits for CC1 Pull-up current source
 * Firmware must read SFlash and set this value for each Rp value (CC_CTRL_0.RP_MODE)
 * 0FFF_F079 : Port 0 - Default
 * 0FFF_F07A : Port 0 - 1p5
 * 0FFF_F07B : Port 0 - 3p0
 * 0FFF_F085 : Port 1 - Default
 * 0FFF_F086 : Port 1 - 1p5
 * 0FFF_F087 : Port 1 - 3p0
 */
#define PDSS_TRIM_CC_1_RP_CC1_TRIM_MASK                     (0x0000003f) /* <0:5> R:RW:0: */
#define PDSS_TRIM_CC_1_RP_CC1_TRIM_POS                      (0)


/*
 * USBPD Hard IP C-connector Trim Register2. Production trims stored in flash
 */
#define PDSS_TRIM_CC_2_ADDRESS                              (0x400aff08)
#define PDSS_TRIM_CC_2                                      (*(volatile uint32_t *)(0x400aff08))
#define PDSS_TRIM_CC_2_DEFAULT                              (0x00000000)

/*
 * Trim bits for CC2 Pull-up current source
 * Firmware must read SFlash and set this value for each Rp value (CC_CTRL_0.RP_MODE)
 * 0FFF_F079 : Port 0 - Default
 * 0FFF_F07A : Port 0 - 1p5
 * 0FFF_F07B : Port 0 - 3p0
 * 0FFF_F085 : Port 1 - Default
 * 0FFF_F086 : Port 1 - 1p5
 * 0FFF_F087 : Port 1 - 3p0
 */
#define PDSS_TRIM_CC_2_RP_CC2_TRIM_MASK                     (0x0000003f) /* <0:5> R:RW:0: */
#define PDSS_TRIM_CC_2_RP_CC2_TRIM_POS                      (0)


/*
 * USBPD Hard IP C-connector Trim Register3. Production trims stored in flash
 */
#define PDSS_TRIM_CC_3_ADDRESS                              (0x400aff0c)
#define PDSS_TRIM_CC_3                                      (*(volatile uint32_t *)(0x400aff0c))
#define PDSS_TRIM_CC_3_DEFAULT                              (0x00000000)

/*
 * Trim bits for 0.5V comparator reference. Default is 0.53. Check BROS for
 * complete description of the steps.
 * This should be programmed to 0x2 for CCG2*A Silicon. Leave default for
 * ** silicon.
 * Notes: This filed should be 0x2 for all application (DFP,UFP,AMA ,cable)
 */
#define PDSS_TRIM_CC_3_V0P5_TRIM_MASK                       (0x00000007) /* <0:2> R:RW:0: */
#define PDSS_TRIM_CC_3_V0P5_TRIM_POS                        (0)


/*
 * Trim bits for 0.655V comparator reference. Default value is 0.6475V
 */
#define PDSS_TRIM_CC_3_V0P655_TRIM_MASK                     (0x00000038) /* <3:5> R:RW:0: */
#define PDSS_TRIM_CC_3_V0P655_TRIM_POS                      (3)


/*
 * USBPD Hard IP C-connector Trim Register4. Production trims stored in flash
 */
#define PDSS_TRIM_CC_4_ADDRESS                              (0x400aff10)
#define PDSS_TRIM_CC_4                                      (*(volatile uint32_t *)(0x400aff10))
#define PDSS_TRIM_CC_4_DEFAULT                              (0x00000000)

/*
 * Trim bits for 0.74V comparator reference. Default value is 0.735V. Check
 * BROS for complete description of the steps.
 */
#define PDSS_TRIM_CC_4_V0P74_TRIM_MASK                      (0x0000000f) /* <0:3> R:RW:0: */
#define PDSS_TRIM_CC_4_V0P74_TRIM_POS                       (0)


/*
 * Trim bits for 1.63V comparator reference. Default value is 1.71V
 */
#define PDSS_TRIM_CC_4_V1P63_TRIM_MASK                      (0x00000070) /* <4:6> R:RW:0: */
#define PDSS_TRIM_CC_4_V1P63_TRIM_POS                       (4)


/*
 * USBPD Hard IP C-connector Trim Register5. Production trims stored in flash
 */
#define PDSS_TRIM_CC_5_ADDRESS                              (0x400aff14)
#define PDSS_TRIM_CC_5                                      (*(volatile uint32_t *)(0x400aff14))
#define PDSS_TRIM_CC_5_DEFAULT                              (0x00000000)

/*
 * Trim bits for 1.125V comparator reference. Default is 1.125. Check BROS
 * for complete description of the steps.
 */
#define PDSS_TRIM_CC_5_V1P125_TRIM_MASK                     (0x00000007) /* <0:2> R:RW:0: */
#define PDSS_TRIM_CC_5_V1P125_TRIM_POS                      (0)


/*
 * Trim bits for 1.235V comparator reference. Default is 1.22V. Check BROS
 * for complete description of the steps.
 */
#define PDSS_TRIM_CC_5_V1P235_TRIM_MASK                     (0x00000038) /* <3:5> R:RW:0: */
#define PDSS_TRIM_CC_5_V1P235_TRIM_POS                      (3)


/*
 * USBPD Hard IP C-connector Trim Register6. Production trims stored in flash
 */
#define PDSS_TRIM_CC_6_ADDRESS                              (0x400aff18)
#define PDSS_TRIM_CC_6                                      (*(volatile uint32_t *)(0x400aff18))
#define PDSS_TRIM_CC_6_DEFAULT                              (0x00000000)

/*
 * Trim bits for 1.575V comparator reference. Default value being 1.56V .
 * Check BROS for complete description for the trim.
 * This should be programmed to 0x3 for CCG2*A Silicon. Leave default for
 * ** silicon.
 * Notes: This field should be 0x3 for all application (DFP,UFP,AMA ,cable)
 */
#define PDSS_TRIM_CC_6_V1P575_TRIM_MASK                     (0x00000007) /* <0:2> R:RW:0: */
#define PDSS_TRIM_CC_6_V1P575_TRIM_POS                      (0)


/*
 * Trim bits for Rd pull-down resistor
 */
#define PDSS_TRIM_CC_6_RD_TRIM_MASK                         (0x00000078) /* <3:6> R:RW:0: */
#define PDSS_TRIM_CC_6_RD_TRIM_POS                          (3)


/*
 * USBPD Hard IP C-connector Trim Register7. Production trims stored in flash
 */
#define PDSS_TRIM_CC_7_ADDRESS                              (0x400aff1c)
#define PDSS_TRIM_CC_7                                      (*(volatile uint32_t *)(0x400aff1c))
#define PDSS_TRIM_CC_7_DEFAULT                              (0x00000000)

/*
 * Trim bits for 1.97V comparator reference. Default value being 2.09V. Check
 * BROS for complete description for the trim.
 */
#define PDSS_TRIM_CC_7_V1P97_TRIM_MASK                      (0x00000007) /* <0:2> R:RW:0: */
#define PDSS_TRIM_CC_7_V1P97_TRIM_POS                       (0)


/*
 * USBPD Hard IP 20V regulator Trim Register
 */
#define PDSS_TRIM_PDS_VREG20_ADDRESS                        (0x400aff20)
#define PDSS_TRIM_PDS_VREG20                                (*(volatile uint32_t *)(0x400aff20))
#define PDSS_TRIM_PDS_VREG20_DEFAULT                        (0x00000000)

/*
 * 20V regulator output voltage trim.  Refer to s8pds BROS for settings.
 */
#define PDSS_TRIM_PDS_VREG20_VREG_TRIM_MASK                 (0x00000007) /* <0:2> R:RW:0: */
#define PDSS_TRIM_PDS_VREG20_VREG_TRIM_POS                  (0)


/*
 * USBPD Hard IP battery charger Detect#1 Trim Register 0. Production trims
 * stored in flash
 */
#define PDSS_TRIM_BCH_DET1_0_ADDRESS                        (0x400aff24)
#define PDSS_TRIM_BCH_DET1_0                                (*(volatile uint32_t *)(0x400aff24))
#define PDSS_TRIM_BCH_DET1_0_DEFAULT                        (0x00000000)

/*
 * 0.6V Reference Voltage Trim
 * Refer to the s8pds BROS for more details
 */
#define PDSS_TRIM_BCH_DET1_0_V600M_TRIM_MASK                (0x00000007) /* <0:2> R:RW:0: */
#define PDSS_TRIM_BCH_DET1_0_V600M_TRIM_POS                 (0)


/*
 * Reference Generator trim control
 * Refer to the s8pds BROS for more details
 */
#define PDSS_TRIM_BCH_DET1_0_V740M_CHGDET_TRIM_MASK         (0x00000078) /* <3:6> R:RW:0: */
#define PDSS_TRIM_BCH_DET1_0_V740M_CHGDET_TRIM_POS          (3)


/*
 * USBPD Hard IP battery charger Detect#1 Trim Register 1. Production trims
 * stored in flash
 */
#define PDSS_TRIM_BCH_DET1_1_ADDRESS                        (0x400aff28)
#define PDSS_TRIM_BCH_DET1_1                                (*(volatile uint32_t *)(0x400aff28))
#define PDSS_TRIM_BCH_DET1_1_DEFAULT                        (0x00000000)

/*
 * 0.325V Reference Voltage Trim
 * Refer to the s8pds BROS for more details
 */
#define PDSS_TRIM_BCH_DET1_1_V325M_TRIM_MASK                (0x00000003) /* <0:1> R:RW:0: */
#define PDSS_TRIM_BCH_DET1_1_V325M_TRIM_POS                 (0)


/*
 * 2.0V Reference Voltage Trim
 * Refer to the s8pds BROS for more details
 */
#define PDSS_TRIM_BCH_DET1_1_V2P0V_TRIM_MASK                (0x0000000c) /* <2:3> R:RW:0: */
#define PDSS_TRIM_BCH_DET1_1_V2P0V_TRIM_POS                 (2)


/*
 * 1.2V Reference Voltage Trim
 * Refer to the s8pds BROS for more details
 */
#define PDSS_TRIM_BCH_DET1_1_V1P2V_TRIM_MASK                (0x000000f0) /* <4:7> R:RW:0: */
#define PDSS_TRIM_BCH_DET1_1_V1P2V_TRIM_POS                 (4)


/*
 * USBPD Hard IP battery charger Detect#1 Trim Register 2. Production trims
 * stored in flash
 */
#define PDSS_TRIM_BCH_DET1_2_ADDRESS                        (0x400aff2c)
#define PDSS_TRIM_BCH_DET1_2                                (*(volatile uint32_t *)(0x400aff2c))
#define PDSS_TRIM_BCH_DET1_2_DEFAULT                        (0x00000000)

/*
 * 0.325V Reference Voltage Trim
 * Refer to the s8pds BROS for more details
 */
#define PDSS_TRIM_BCH_DET1_2_V850M_TRIM_MASK                (0x00000003) /* <0:1> R:RW:0: */
#define PDSS_TRIM_BCH_DET1_2_V850M_TRIM_POS                 (0)


/*
 * 2.2V Reference Voltage Trim
 * Refer to the s8pds BROS for more details
 */
#define PDSS_TRIM_BCH_DET1_2_V2P2V_TRIM_MASK                (0x0000000c) /* <2:3> R:RW:0: */
#define PDSS_TRIM_BCH_DET1_2_V2P2V_TRIM_POS                 (2)


/*
 * 2.9V Reference Voltage Trim
 * Refer to the s8pds BROS for more details
 */
#define PDSS_TRIM_BCH_DET1_2_V2P9V_TRIM_MASK                (0x00000030) /* <4:5> R:RW:0: */
#define PDSS_TRIM_BCH_DET1_2_V2P9V_TRIM_POS                 (4)


/*
 * 2.5V Reference Voltage Trim
 * Refer to the s8pds BROS for more details
 */
#define PDSS_TRIM_BCH_DET1_2_V2P5V_TRIM_MASK                (0x000000c0) /* <6:7> R:RW:0: */
#define PDSS_TRIM_BCH_DET1_2_V2P5V_TRIM_POS                 (6)


/*
 * USBPD Hard IP battery charger Detect#1 Trim Register 3. Production trims
 * stored in flash
 */
#define PDSS_TRIM_BCH_DET1_3_ADDRESS                        (0x400aff30)
#define PDSS_TRIM_BCH_DET1_3                                (*(volatile uint32_t *)(0x400aff30))
#define PDSS_TRIM_BCH_DET1_3_DEFAULT                        (0x00000000)

/*
 * 0.700V Reference Voltage Trim
 * Refer to the s8pds BROS for more details
 */
#define PDSS_TRIM_BCH_DET1_3_V700M_TRIM_MASK                (0x00000007) /* <0:2> R:RW:0: */
#define PDSS_TRIM_BCH_DET1_3_V700M_TRIM_POS                 (0)


/*
 * 1.4V Reference Voltage Trim
 * Refer to the s8pds BROS for more details
 */
#define PDSS_TRIM_BCH_DET1_3_V1P4V_TRIM_MASK                (0x00000038) /* <3:5> R:RW:0: */
#define PDSS_TRIM_BCH_DET1_3_V1P4V_TRIM_POS                 (3)


/*
 * 1.7V Reference Voltage Trim
 * Refer to the s8pds BROS for more details
 */
#define PDSS_TRIM_BCH_DET1_3_V1P7V_TRIM_MASK                (0x000000c0) /* <6:7> R:RW:0: */
#define PDSS_TRIM_BCH_DET1_3_V1P7V_TRIM_POS                 (6)


/*
 * USBPD Hard IP LSCSA#1 Trim Register 0. Production trims stored in flash
 */
#define PDSS_TRIM_LSCSA1_0_ADDRESS                          (0x400aff34)
#define PDSS_TRIM_LSCSA1_0                                  (*(volatile uint32_t *)(0x400aff34))
#define PDSS_TRIM_LSCSA1_0_DEFAULT                          (0x00000000)

/*
 * Trim control for gain accuracy of stage1
 */
#define PDSS_TRIM_LSCSA1_0_AV1_TRIM_MASK                    (0x0000000f) /* <0:3> R:RW:0: */
#define PDSS_TRIM_LSCSA1_0_AV1_TRIM_POS                     (0)


/*
 * Trim control for gain accuracy of stage2
 */
#define PDSS_TRIM_LSCSA1_0_AV2_TRIM_MASK                    (0x000000f0) /* <4:7> R:RW:0: */
#define PDSS_TRIM_LSCSA1_0_AV2_TRIM_POS                     (4)


/*
 * USBPD Hard IP LSCSA#1 Trim Register 1. Production trims stored in flash
 */
#define PDSS_TRIM_LSCSA1_1_ADDRESS                          (0x400aff38)
#define PDSS_TRIM_LSCSA1_1                                  (*(volatile uint32_t *)(0x400aff38))
#define PDSS_TRIM_LSCSA1_1_DEFAULT                          (0x00000000)

/*
 * Trim control for stage1 input referred offset
 */
#define PDSS_TRIM_LSCSA1_1_OS1_TRIM_MASK                    (0x0000001f) /* <0:4> R:RW:0: */
#define PDSS_TRIM_LSCSA1_1_OS1_TRIM_POS                     (0)


/*
 * USBPD Hard IP LSCSA#1 Trim Register 2. Production trims stored in flash
 */
#define PDSS_TRIM_LSCSA1_2_ADDRESS                          (0x400aff3c)
#define PDSS_TRIM_LSCSA1_2                                  (*(volatile uint32_t *)(0x400aff3c))
#define PDSS_TRIM_LSCSA1_2_DEFAULT                          (0x00000000)

/*
 * Trim control for stage2 input referred offset
 */
#define PDSS_TRIM_LSCSA1_2_OS2_TRIM_MASK                    (0x0000001f) /* <0:4> R:RW:0: */
#define PDSS_TRIM_LSCSA1_2_OS2_TRIM_POS                     (0)


/*
 * USBPD Hard IP REFGEN#1 Trim Register 0. Production trims stored in flash
 */
#define PDSS_TRIM_REFGEN1_0_ADDRESS                         (0x400aff40)
#define PDSS_TRIM_REFGEN1_0                                 (*(volatile uint32_t *)(0x400aff40))
#define PDSS_TRIM_REFGEN1_0_DEFAULT                         (0x00000000)

/*
 * Trim for input reference voltage/ Offset of regulation amplifier. This
 * provides the reference for the regulation blocks in the CSA,EA, and other
 * blocks such as CC_SHVT reference and VBUS_dischg. Default value is 0 with
 * bits bit 1-7 trimming down and 9-15 trimming up. BROS shows complete description
 * for the trim.
 */
#define PDSS_TRIM_REFGEN1_0_REFGEN_VREF_TRIM_MASK           (0x0000000f) /* <0:3> R:RW:0: */
#define PDSS_TRIM_REFGEN1_0_REFGEN_VREF_TRIM_POS            (0)


/*
 * Trim for input reference voltage/ Offset of regulation amplifier. This
 * provides the reference for the regulation blocks in the CSA,EA, and other
 * blocks such as CC_SHVT reference and VBUS_dischg. Default value is 0 with
 * bits bit 1-7 trimming down and 9-15 trimming up. BROS shows complete description
 * for the trim.
 */
#define PDSS_TRIM_REFGEN1_0_REFGEN_VREF_MON_TRIM_MASK       (0x000000f0) /* <4:7> R:RW:0: */
#define PDSS_TRIM_REFGEN1_0_REFGEN_VREF_MON_TRIM_POS        (4)


/*
 * USBPD Hard IP REFGEN#1 Trim Register 1. Production trims stored in flash
 */
#define PDSS_TRIM_REFGEN1_1_ADDRESS                         (0x400aff44)
#define PDSS_TRIM_REFGEN1_1                                 (*(volatile uint32_t *)(0x400aff44))
#define PDSS_TRIM_REFGEN1_1_DEFAULT                         (0x00000000)

/*
 * Trim bit for internal clock generator
 * 0: No trim
 * 1: Higher Frequency
 */
#define PDSS_TRIM_REFGEN1_1_REFGEN_ICLK_TRIM                (1u << 0) /* <0:0> R:RW:0: */


/*
 * USBPD Hard IP EA#1 Trim Register 0. Production trims stored in flash
 */
#define PDSS_TRIM_PDS_EA_ADDRESS                            (0x400aff48)
#define PDSS_TRIM_PDS_EA                                    (*(volatile uint32_t *)(0x400aff48))
#define PDSS_TRIM_PDS_EA_DEFAULT                            (0x00000020)

/*
 * Trim bits to alter IDAC current resolution which would change VBUS resolution.
 */
#define PDSS_TRIM_PDS_EA_IREF_CV_TRIM_MASK                  (0x000000ff) /* <0:7> R:RW:32: */
#define PDSS_TRIM_PDS_EA_IREF_CV_TRIM_POS                   (0)


/*
 * USBPD Hard IP Spare1 Trim Register
 */
#define PDSS_TRIM_SPARE1_ADDRESS                            (0x400aff4c)
#define PDSS_TRIM_SPARE1                                    (*(volatile uint32_t *)(0x400aff4c))
#define PDSS_TRIM_SPARE1_DEFAULT                            (0x00000000)

/*
 * RESERVED FOR FUTURE USE
 */
#define PDSS_TRIM_SPARE1_SPARE1_TRIM_MASK                   (0x000000ff) /* <0:7> R:RW:0: */
#define PDSS_TRIM_SPARE1_SPARE1_TRIM_POS                    (0)


/*
 * USBPD Hard IP Comparator#0 Trim Register. Production trims stored in flash
 */
#define PDSS_TRIM_COMP1_0_ADDRESS                           (0x400aff50)
#define PDSS_TRIM_COMP1_0                                   (*(volatile uint32_t *)(0x400aff50))
#define PDSS_TRIM_COMP1_0_DEFAULT                           (0x00000000)

/*
 * Input offset trim.
 */
#define PDSS_TRIM_COMP1_0_VIOS_TRIM_MASK                    (0x0000003f) /* <0:5> R:RW:0: */
#define PDSS_TRIM_COMP1_0_VIOS_TRIM_POS                     (0)


/*
 * USBPD Hard IP Deep Sleep   Trim Register0. Production trims stored in
 * flash
 */
#define PDSS_TRIM_DPSLP_0_ADDRESS                           (0x400aff54)
#define PDSS_TRIM_DPSLP_0                                   (*(volatile uint32_t *)(0x400aff54))
#define PDSS_TRIM_DPSLP_0_DEFAULT                           (0x000000f1)

/*
 * DeepSleep 2.4uA current reference trim bit.
 * Refer to s8usbpd_ver3 BROS for bit settings.
 */
#define PDSS_TRIM_DPSLP_0_I_TRIM_MASK                       (0x000000ff) /* <0:7> R:RW:241: */
#define PDSS_TRIM_DPSLP_0_I_TRIM_POS                        (0)


/*
 * USBPD Hard IP Deep Sleep  Trim Register1. Production trims stored in flash
 */
#define PDSS_TRIM_DPSLP_1_ADDRESS                           (0x400aff58)
#define PDSS_TRIM_DPSLP_1                                   (*(volatile uint32_t *)(0x400aff58))
#define PDSS_TRIM_DPSLP_1_DEFAULT                           (0x00000000)

/*
 * Beta multiplier reference trim bits.
 * Refer to GPM-541 for trim settings.
 */
#define PDSS_TRIM_DPSLP_1_REF_TRIM_MASK                     (0x0000000f) /* <0:3> R:RW:0: */
#define PDSS_TRIM_DPSLP_1_REF_TRIM_POS                      (0)


/*
 * Deepsleep 2.4uA current reference temperature coefficient trim bits
 */
#define PDSS_TRIM_DPSLP_1_ITRIM_TC_MASK                     (0x000000f0) /* <4:7> R:RW:0: */
#define PDSS_TRIM_DPSLP_1_ITRIM_TC_POS                      (4)


/*
 * USBPD Hard IP SR Sense Trim Register0. Production trims stored in flash
 */
#define PDSS_TRIM_SR_SENSE_0_ADDRESS                        (0x400aff5c)
#define PDSS_TRIM_SR_SENSE_0                                (*(volatile uint32_t *)(0x400aff5c))
#define PDSS_TRIM_SR_SENSE_0_DEFAULT                        (0x00000006)

/*
 * trim bits for trimming resistive divider on SR_SENSE, SR_VSS and for OVP
 */
#define PDSS_TRIM_SR_SENSE_0_DRAIN_TRIM_MASK                (0x000000ff) /* <0:7> R:RW:6: */
#define PDSS_TRIM_SR_SENSE_0_DRAIN_TRIM_POS                 (0)


/*
 * USBPD Hard IP SR Sense Trim Register1. Production trims stored in flash
 */
#define PDSS_TRIM_SR_SENSE_1_ADDRESS                        (0x400aff60)
#define PDSS_TRIM_SR_SENSE_1                                (*(volatile uint32_t *)(0x400aff60))
#define PDSS_TRIM_SR_SENSE_1_DEFAULT                        (0x00000016)

/*
 * trim bits for feed-forward ictat
 */
#define PDSS_TRIM_SR_SENSE_1_FEEDFWD_ICTAT_TRIM_MASK        (0x0000003f) /* <0:5> R:RW:22: */
#define PDSS_TRIM_SR_SENSE_1_FEEDFWD_ICTAT_TRIM_POS         (0)


/*
 * USBPD Hard IP SR Sense Trim Register2. Production trims stored in flash
 */
#define PDSS_TRIM_SR_SENSE_2_ADDRESS                        (0x400aff64)
#define PDSS_TRIM_SR_SENSE_2                                (*(volatile uint32_t *)(0x400aff64))
#define PDSS_TRIM_SR_SENSE_2_DEFAULT                        (0x00000020)

/*
 * trim bits for feed-forward i_feedfwd
 */
#define PDSS_TRIM_SR_SENSE_2_FEEDFWD_IFF_TRIM_MASK          (0x0000003f) /* <0:5> R:RW:32: */
#define PDSS_TRIM_SR_SENSE_2_FEEDFWD_IFF_TRIM_POS           (0)


/*
 * USBPD Hard IP SR Sense Trim Register3. Production trims stored in flash
 */
#define PDSS_TRIM_SR_SENSE_3_ADDRESS                        (0x400aff68)
#define PDSS_TRIM_SR_SENSE_3                                (*(volatile uint32_t *)(0x400aff68))
#define PDSS_TRIM_SR_SENSE_3_DEFAULT                        (0x00000084)

/*
 * trim bits for feed-forward Iout vbus
 */
#define PDSS_TRIM_SR_SENSE_3_FEEDFWD_IOUT_VBUS_TRIM_MASK    (0x000000ff) /* <0:7> R:RW:132: */
#define PDSS_TRIM_SR_SENSE_3_FEEDFWD_IOUT_VBUS_TRIM_POS     (0)


/*
 * USBPD Hard IP SR Sense Trim Register4. Production trims stored in flash
 */
#define PDSS_TRIM_SR_SENSE_4_ADDRESS                        (0x400aff6c)
#define PDSS_TRIM_SR_SENSE_4                                (*(volatile uint32_t *)(0x400aff6c))
#define PDSS_TRIM_SR_SENSE_4_DEFAULT                        (0x00000088)

/*
 * trim bits for feed-forward V2I
 */
#define PDSS_TRIM_SR_SENSE_4_FEEDFWD_V2I_TRIM_MASK          (0x000000ff) /* <0:7> R:RW:136: */
#define PDSS_TRIM_SR_SENSE_4_FEEDFWD_V2I_TRIM_POS           (0)


/*
 * USBPD Hard IP SR Sense Trim Register5. Production trims stored in flash
 */
#define PDSS_TRIM_SR_SENSE_5_ADDRESS                        (0x400aff70)
#define PDSS_TRIM_SR_SENSE_5                                (*(volatile uint32_t *)(0x400aff70))
#define PDSS_TRIM_SR_SENSE_5_DEFAULT                        (0x00000008)

/*
 * trim bits for feed-forward V_feedfwd
 */
#define PDSS_TRIM_SR_SENSE_5_FEEDFWD_VFF_TRIM_MASK          (0x0000000f) /* <0:3> R:RW:8: */
#define PDSS_TRIM_SR_SENSE_5_FEEDFWD_VFF_TRIM_POS           (0)


/*
 * USBPD Hard IP SR Sense Trim Register6. Production trims stored in flash
 */
#define PDSS_TRIM_SR_SENSE_6_ADDRESS                        (0x400aff74)
#define PDSS_TRIM_SR_SENSE_6                                (*(volatile uint32_t *)(0x400aff74))
#define PDSS_TRIM_SR_SENSE_6_DEFAULT                        (0x00000034)

/*
 * trim bits for NSN comparator (Default of -100mV)
 */
#define PDSS_TRIM_SR_SENSE_6_NSN_TRIM_MASK                  (0x0000003f) /* <0:5> R:RW:52: */
#define PDSS_TRIM_SR_SENSE_6_NSN_TRIM_POS                   (0)


/*
 * USBPD Hard IP SR Sense Trim Register7. Production trims stored in flash
 */
#define PDSS_TRIM_SR_SENSE_7_ADDRESS                        (0x400aff78)
#define PDSS_TRIM_SR_SENSE_7                                (*(volatile uint32_t *)(0x400aff78))
#define PDSS_TRIM_SR_SENSE_7_DEFAULT                        (0x00000000)

/*
 * Trim bits for peakdet comparator
 */
#define PDSS_TRIM_SR_SENSE_7_PEAKDET_TRIM_MASK              (0x000000ff) /* <0:7> R:RW:0: */
#define PDSS_TRIM_SR_SENSE_7_PEAKDET_TRIM_POS               (0)


/*
 * USBPD Hard IP SR Sense Trim Register8. Production trims stored in flash
 */
#define PDSS_TRIM_SR_SENSE_8_ADDRESS                        (0x400aff7c)
#define PDSS_TRIM_SR_SENSE_8                                (*(volatile uint32_t *)(0x400aff7c))
#define PDSS_TRIM_SR_SENSE_8_DEFAULT                        (0x00000017)

/*
 * Trim bits for ZCD
 */
#define PDSS_TRIM_SR_SENSE_8_ZCD_TRIM_MASK                  (0x000000ff) /* <0:7> R:RW:23: */
#define PDSS_TRIM_SR_SENSE_8_ZCD_TRIM_POS                   (0)


/*
 * USBPD Hard IP SR Sense Trim Register9. Production trims stored in flash
 */
#define PDSS_TRIM_SR_SENSE_9_ADDRESS                        (0x400aff80)
#define PDSS_TRIM_SR_SENSE_9                                (*(volatile uint32_t *)(0x400aff80))
#define PDSS_TRIM_SR_SENSE_9_DEFAULT                        (0x00000000)

/*
 * Trim bits for ZCDF
 */
#define PDSS_TRIM_SR_SENSE_9_ZCDF_TRIM_MASK                 (0x000000ff) /* <0:7> R:RW:0: */
#define PDSS_TRIM_SR_SENSE_9_ZCDF_TRIM_POS                  (0)


/*
 * USBPD Hard IP PWM Trim Register0. Production trims stored in flash
 */
#define PDSS_TRIM_PWM_0_ADDRESS                             (0x400aff84)
#define PDSS_TRIM_PWM_0                                     (*(volatile uint32_t *)(0x400aff84))
#define PDSS_TRIM_PWM_0_DEFAULT                             (0x00000010)

/*
 * trim bits for PWM capacitor trimming in fixed DAC path
 */
#define PDSS_TRIM_PWM_0_CAP_DAC_TRIM_MASK                   (0x0000001f) /* <0:4> R:RW:16: */
#define PDSS_TRIM_PWM_0_CAP_DAC_TRIM_POS                    (0)


/*
 * USBPD Hard IP PWM Trim Register1. Production trims stored in flash
 */
#define PDSS_TRIM_PWM_1_ADDRESS                             (0x400aff88)
#define PDSS_TRIM_PWM_1                                     (*(volatile uint32_t *)(0x400aff88))
#define PDSS_TRIM_PWM_1_DEFAULT                             (0x00000010)

/*
 * trim bits for PWM capacitor trimming in feedforward DAC path
 */
#define PDSS_TRIM_PWM_1_CAP_DAC_FF_TRIM_MASK                (0x0000001f) /* <0:4> R:RW:16: */
#define PDSS_TRIM_PWM_1_CAP_DAC_FF_TRIM_POS                 (0)


/*
 * USBPD Hard IP PWM Trim Register2. Production trims stored in flash
 */
#define PDSS_TRIM_PWM_2_ADDRESS                             (0x400aff8c)
#define PDSS_TRIM_PWM_2                                     (*(volatile uint32_t *)(0x400aff8c))
#define PDSS_TRIM_PWM_2_DEFAULT                             (0x0000002a)

/*
 * pwm comparator hysteresis trimming
 */
#define PDSS_TRIM_PWM_2_TRIM_PWM_COMP_HYST_MASK             (0x00000003) /* <0:1> R:RW:2: */
#define PDSS_TRIM_PWM_2_TRIM_PWM_COMP_HYST_POS              (0)


/*
 * hclamp comparator hysteresis trimming
 */
#define PDSS_TRIM_PWM_2_TRIM_HCLAMP_COMP_HYST_MASK          (0x0000000c) /* <2:3> R:RW:2: */
#define PDSS_TRIM_PWM_2_TRIM_HCLAMP_COMP_HYST_POS           (2)


/*
 * burst exit comparator hysteresis trimming
 */
#define PDSS_TRIM_PWM_2_TRIM_BURST_EXIT_COMP_HYST_MASK      (0x00000030) /* <4:5> R:RW:2: */
#define PDSS_TRIM_PWM_2_TRIM_BURST_EXIT_COMP_HYST_POS       (4)


/*
 * USBPD Hard IP PWM Trim Register1. Production trims stored in flash
 */
#define PDSS_TRIM_SCP_ADDRESS                               (0x400aff90)
#define PDSS_TRIM_SCP                                       (*(volatile uint32_t *)(0x400aff90))
#define PDSS_TRIM_SCP_DEFAULT                               (0x0000001f)

/*
 * Trim bits for SCP offset to for 6A SCP (30mV with external 5mohm Rsense)
 */
#define PDSS_TRIM_SCP_SCP_TRIM_MASK                         (0x0000003f) /* <0:5> R:RW:31: */
#define PDSS_TRIM_SCP_SCP_TRIM_POS                          (0)


/*
 * Configuration register
 */
#define CPUSS_CONFIG_ADDRESS                                (0x40100000)
#define CPUSS_CONFIG                                        (*(volatile uint32_t *)(0x40100000))
#define CPUSS_CONFIG_DEFAULT                                (0x00000000)

/*
 * 0': Vector Table is located at 0x0000:0000 in flash
 * '1': Vector Table is located at 0x2000:0000 in SRAM
 * Note that vectors for RESET and FAULT are always fetched from ROM. Value
 * in flash/RAM is ignored for these vectors.
 */
#define CPUSS_CONFIG_VECT_IN_RAM                            (1u << 0) /* <0:0> R:RW:0: */


/*
 * SYSCALL control register
 * Used to make system requests to SROM code.  System Requests transition
 * from User Mode to Privileged Mode.  See SAS for more details.  Firmware/ATE
 * should write CPUSS_SYSARG first and CPUSS_SYSREQ register next.
 */
#define CPUSS_SYSREQ_ADDRESS                                (0x40100004)
#define CPUSS_SYSREQ                                        (*(volatile uint32_t *)(0x40100004))
#define CPUSS_SYSREQ_DEFAULT                                (0x30000000)

/*
 * Opcode of the system call being requested.
 */
#define CPUSS_SYSREQ_SYSCALL_COMMAND_MASK                   (0x0000ffff) /* <0:15> :RW:0: */
#define CPUSS_SYSREQ_SYSCALL_COMMAND_POS                    (0)


/*
 * Disable Reset Vector fetch relocation:
 * '0': CPU accesses to locations 0x0000:0000 - 0x0000:0007 are redirected
 * to ROM.
 * '1': CPU accesses to locations 0x0000:0000 - 0x0000:0007 are made to flash.
 * Note that this field defaults to '0' on reset, ensuring actual reset vector
 * fetches are always made to ROM. Note that this field does not affect DAP
 * accesses. Flash DfT routines may set this bit to '1' to enable uninhibited
 * read-back of programmed data in the first flash page.
 */
#define CPUSS_SYSREQ_DIS_RESET_VECT_REL                     (1u << 27) /* <27:27> R:RW:0: */


/*
 * Indicates whether the system is in privileged ('1') or user mode ('0').
 * Only CPU SW executing from ROM can set this field to '1' when ROM_ACCESS_EN
 * is '1' (the CPU is executing a SystemCall NMI interrupt handler). Any
 * other write to this field sets is to '0'. This field is used as the AHB-Lite
 * hprot[1] signal to implement Cypress proprietary user/privileged modes.
 * These modes are used to enable/disable access to specific MMIO registers
 * and memory regions.
 */
#define CPUSS_SYSREQ_PRIVILEGED                             (1u << 28) /* <28:28> A:RW:1: */


/*
 * Indicates that executing from Boot ROM is enabled. HW sets this field
 * to '1', on reset or when the SystemCall NMI vector is fetched from Boot
 * ROM. HW sets this field to '0', when the CPU is NOT executing from either
 * Boot or System ROM. This bit is used for debug purposes only.
 */
#define CPUSS_SYSREQ_ROM_ACCESS_EN                          (1u << 29) /* <29:29> RW:R:1: */


/*
 * Indicates the source of the write access to the SYSREQ register.
 * '0': CPU write access.
 * '1': DAP write access.
 * HW sets this field when the SYSREQ register is written to and SYSCALL_REQ
 * is '0' (the last time it is set is when SW sets SYSCALL_REQ from '0' to
 * '1').
 */
#define CPUSS_SYSREQ_HMASTER_0                              (1u << 30) /* <30:30> A:R:0: */


/*
 * CPU/DAP writes a '1' to this field to request a SystemCall. The HMASTER_0
 * field indicates the source of the write access. Setting this field to
 * '1' immediate results in a NMI. The SystemCall NMI interrupt handler sets
 * this field to '0' after servicing the request.
 */
#define CPUSS_SYSREQ_SYSCALL_REQ                            (1u << 31) /* <31:31> R:RW:0: */


/*
 * SYSARG control register
 * Used to make system requests to SROM code.  System Requests transition
 * from User Mode to Privileged Mode.  See SAS for more details.  Firmware/ATE
 * should write CPUSS_SYSARG first and CPUSS_SYSREQ register next.
 */
#define CPUSS_SYSARG_ADDRESS                                (0x40100008)
#define CPUSS_SYSARG                                        (*(volatile uint32_t *)(0x40100008))
#define CPUSS_SYSARG_DEFAULT                                (0x00000000)

/*
 * Argument to System Call specified in SYSREQ. Semantics of argument depends
 * on system call made. Typically a pointer to a parameter block.
 */
#define CPUSS_SYSARG_SYSCALL_ARG_MASK                       (0xffffffff) /* <0:31> :RW:0: */
#define CPUSS_SYSARG_SYSCALL_ARG_POS                        (0)


/*
 * Protection control register
 */
#define CPUSS_PROTECTION_ADDRESS                            (0x4010000c)
#define CPUSS_PROTECTION                                    (*(volatile uint32_t *)(0x4010000c))
#define CPUSS_PROTECTION_DEFAULT                            (0x0000000f)

/*
 * Current protection mode; this field is available as a global signal everywhere
 * in the system. Writes to this field are ignored when PROTECTION_LOCK is
 * '1':
 * 0b1xxx: BOOT
 * 0b01xx: KILL
 * 0b001x: PROTECTED
 * 0b0001: OPEN
 * 0b0000: VIRGIN (also used for DEAD mode, but then FLASH_LOCK is also set)
 */
#define CPUSS_PROTECTION_PROTECTION_MODE_MASK               (0x0000000f) /* <0:3> A:RW:15: */
#define CPUSS_PROTECTION_PROTECTION_MODE_POS                (0)


/*
 * Setting this bit will force SPCIF.ADDRESS.AXA to be ignored, which prevents
 * SM Flash from being erased or overwritten. It is used to indicate the
 * DEAD protection mode. Writes to this field are ignored when PROTECTION_LOCK
 * is '1'
 */
#define CPUSS_PROTECTION_FLASH_LOCK                         (1u << 30) /* <30:30> A:RW:0:SECURE_ACCESS */


/*
 * Setting this field will block (ignore) any further writes to the PROTECTION_MODE
 * field in this register. Once '1', this field cannot be cleared.
 */
#define CPUSS_PROTECTION_PROTECTION_LOCK                    (1u << 31) /* <31:31> R:RW1S:0: */


/*
 * ROM privilege register
 * ROM memory has a maximum capacity of 128 KByte. ROM is partitioned into
 * Boot ROM and System ROM. These two partitions are located back-to-back
 * in the system address space. The Boot ROM capacity is 4, 8, 16, or 32
 * KByte. The System ROM partition capacity equals the ROM capacity minus
 * the Boot ROM partition capacity.
 *
 * User mode accesses to a privileged address result in an AHB-Lite bus error.
 * If the ROM memory capacity is not a power of two, the ROM memory region
 * has an unpopulated/unaccounted memory are (at the end of the ROM memory
 * region). A user mode access to an unpopulated, privileged area (as indicated
 * by the LIMIT field(s)) address results in an AHB-Lite bus error. A user
 * mode access to a unpopulated area, without any access violations, behaves
 * as follows: Reads return "0" and writes are ignore (RZWI).
 */
#define CPUSS_PRIV_ROM_ADDRESS                              (0x40100010)
#define CPUSS_PRIV_ROM                                      (*(volatile uint32_t *)(0x40100010))
#define CPUSS_PRIV_ROM_DEFAULT                              (0x00000000)

/*
 * Indicates the limit where the privileged area of the Boot ROM partition
 * starts in increments of 256 Bytes.
 * "0":  Entire Boot ROM is Privileged.
 * "1":  First 256 Bytes are User accessable.
 * ...
 * BROM_PROT_LIMIT >= "Boot ROM partition capacity": Entire Boot ROM partition
 * is user mode accessible.
 */
#define CPUSS_PRIV_ROM_BROM_PROT_LIMIT_MASK                 (0x000000ff) /* <0:7> R:RW:0: */
#define CPUSS_PRIV_ROM_BROM_PROT_LIMIT_POS                  (0)


/*
 * RAM privilege register
 * User mode accesses to a privileged address result in an AHB-Lite bus error.
 * If the RAM memory capacity is not a power of two, the RAM memory region
 * has an unpopulated/unaccounted memory are (at the end of the RAM memory
 * region). A user mode access to an unpopulated, privileged area (as indicated
 * by the LIMIT field(s)) address results in an AHB-Lite bus error. A user
 * mode access to a unpopulated area, without any access violations, behaves
 * as follows: Reads return "0" and writes are ignore (RZWI).
 */
#define CPUSS_PRIV_RAM_ADDRESS                              (0x40100014)
#define CPUSS_PRIV_RAM                                      (*(volatile uint32_t *)(0x40100014))
#define CPUSS_PRIV_RAM_DEFAULT                              (0x00000000)

/*
 * Indicates the limit where the privileged area of SRAM starts in increments
 * of 256 Bytes.
 * "0":  Entire SRAM is Privileged.
 * "1":  First 256 Bytes are User accessable.
 *
 * Any number larger than the size of the SRAM indicates that the entire
 * SRAM is user mode accessible.
 */
#define CPUSS_PRIV_RAM_RAM_PROT_LIMIT_MASK                  (0x000001ff) /* <0:8> R:RW:0: */
#define CPUSS_PRIV_RAM_RAM_PROT_LIMIT_POS                   (0)


/*
 * Flash privilege register
 * User mode accesses to a privileged address result in an AHB-Lite bus error.
 * If the regular flash memory capacity is not a power of two, the regular
 * flash memory region has an unpopulated/unaccounted memory are (at the
 * end of the regular flash memory region). A user mode access to an unpopulated,
 * privileged area (as indicated by the LIMIT field(s)) address results in
 * an AHB-Lite bus error. A user mode access to a unpopulated area, without
 * any access violations, behaves as follows: Reads return "0" and writes
 * are ignore (RZWI).
 */
#define CPUSS_PRIV_FLASH_ADDRESS                            (0x40100018)
#define CPUSS_PRIV_FLASH                                    (*(volatile uint32_t *)(0x40100018))
#define CPUSS_PRIV_FLASH_DEFAULT                            (0x00000000)

/*
 * Indicates the limit where the privileged area of flash starts in increments
 * of 256 Bytes.
 * "0":  Entire flash is Privileged.
 * "1":  First 256 Bytes are User accessable.
 *
 * Any number larger than the size of the flash indicates that the entire
 * flash is user mode accessible. Note that SuperVisory rows are always User
 * accessable.
 *
 * If FLASH_PROT_LIMIT defines a non-empty privileged area, the boot ROM
 * will assume that a system call table exists at the beginning of the Flash
 * privileged area and use it for all SystemCalls made using SYSREQ.
 */
#define CPUSS_PRIV_FLASH_FLASH_PROT_LIMIT_MASK              (0x00000fff) /* <0:11> R:RW:0: */
#define CPUSS_PRIV_FLASH_FLASH_PROT_LIMIT_POS               (0)


/*
 * Wounding register
 * Wounding is based on the FLASH/SRAM memory address range. This range is
 * always the next power of 2 multiple of the FLASH/SRAM memory capacity.
 * E.g., a 48 KByte SRAM capacity has a 64 KByte memory address range. With
 * RAM_WOUND is "0", all 48 KByte SRAM capacity is accessible. With RAM_WOUND
 * is "1", the first 32 KByte SRAM capacity is accessible. With RAM_WOUND
 * is "2", the first 16 KByte SRAM capacity is accessible.
 */
#define CPUSS_WOUNDING_ADDRESS                              (0x4010001c)
#define CPUSS_WOUNDING                                      (*(volatile uint32_t *)(0x4010001c))
#define CPUSS_WOUNDING_DEFAULT                              (0x00000000)

/*
 * Indicates the amount of accessible RAM 0 memory capacitty in this part.
 * The value in this field is effectively write-once (it is only possible
 * to set bits, not clear them). The remainder portion of SRAM is not accessible
 * and will return an AHB-Lite bus error.
 * "0": entire memory accessible
 * "1": first 1/2 of the memory accessible
 * "2": first 1/4 of the memory accessible
 * "3": first 1/8 of the memory accessible
 * "4": first 1/16 of the memory accessible
 * "5": first 1/32 of the memory accessible
 * "6": first 1/64 of the memory accessible
 * "7": first 1/128 of the memory accessible
 */
#define CPUSS_WOUNDING_RAM_WOUND_MASK                       (0x00070000) /* <16:18> R:RW1S:0: */
#define CPUSS_WOUNDING_RAM_WOUND_POS                        (16)


/*
 * Indicates the amount of accessible flash in this part. The value in this
 * field is effectively write-once (it is only possible to set bits, not
 * clear them). The remainder portion of flash is not accessible and will
 * return an AHB-Lite bus error.
 * "0": entire memory accessible
 * "1": first 1/2 of the memory accessible
 * "2": first 1/4 of the memory accessible
 * "3": first 1/8 of the memory accessible
 * "4": first 1/16 of the memory accessible
 * "5": first 1/32 of the memory accessible
 * "6": first 1/64 of the memory accessible
 * "7": first 1/128 of the memory accessible (used for the DEAD protection
 * mode)
 */
#define CPUSS_WOUNDING_FLASH_WOUND_MASK                     (0x00700000) /* <20:22> R:RW1S:0:FLASHC_PRESENT */
#define CPUSS_WOUNDING_FLASH_WOUND_POS                      (20)


/*
 * FLASH control register
 */
#define CPUSS_FLASH_CTL_ADDRESS                             (0x40100030)
#define CPUSS_FLASH_CTL                                     (*(volatile uint32_t *)(0x40100030))
#define CPUSS_FLASH_CTL_DEFAULT                             (0x00000000)

/*
 * Amount of ROM wait states:
 * "0": 0 wait states (fast flash: [0, 24] MHz system frequency, slow flash:
 * [0, 16] MHz system frequency)
 * "1": 1 wait state (fast flash: [24, 48] MHz system frequency, slow flash:
 * [16, 32] MHz system frequency)
 * "2": 2 wait states (slow flash: [32, 48] MHz system frequency)
 * "3": 3 wait states (can be used to give more time for flash access if
 * 2 wait states are not sufficient)
 */
#define CPUSS_FLASH_CTL_FLASH_WS_MASK                       (0x00000003) /* <0:1> R:RW:0: */
#define CPUSS_FLASH_CTL_FLASH_WS_POS                        (0)


/*
 * Prefetch enable:
 * '0': disabled. This is a desirable seeting when FLASH_WS is "0" or when
 * predictable execution behavior is required.
 * '1': enabled.
 */
#define CPUSS_FLASH_CTL_PREF_EN                             (1u << 4) /* <4:4> R:RW:0: */


/*
 * 1': Invalidates the content of the flash controller's buffers.
 */
#define CPUSS_FLASH_CTL_FLASH_INVALIDATE                    (1u << 8) /* <8:8> RW1C:RW:0: */


/*
 * ROM control register
 */
#define CPUSS_ROM_CTL_ADDRESS                               (0x40100034)
#define CPUSS_ROM_CTL                                       (*(volatile uint32_t *)(0x40100034))
#define CPUSS_ROM_CTL_DEFAULT                               (0x00000000)

/*
 * Amount of ROM wait states:
 * '0': 0 wait states. Use this setting for newer, faster ROM design. Use
 * this setting for older, slower ROM design and frequencies in the range
 * [0, 24] MHz.
 * '1': 1 wait state. Use this setting for older, slower ROM design and frequencies
 * in the range <24, 48] MHz.
 *
 * CPUSSv2 supports two types of ROM memory: an older, slower design (operating
 * at up to 24 MHz) and a newer, faster design (operating at up to 48 MHz).
 * The older design requires 1 wait state for frequencies above 24 MHz. The
 * newer design never requires wait states. All chips after Street Fighter
 * will use the newer design. As a result, all chips after Street Fighter
 * can always use 0 wait states.
 */
#define CPUSS_ROM_CTL_ROM_WS                                (1u << 0) /* <0:0> R:RW:0: */


/*
 * Bist command register
 */
#define CPUSS_BIST_CMD_ADDRESS                              (0x40100040)
#define CPUSS_BIST_CMD                                      (*(volatile uint32_t *)(0x40100040))
#define CPUSS_BIST_CMD_DEFAULT                              (0x00000000)

/*
 * 1': Start SRAM BIST. Hardware set this field to '0' when BIST is completed.
 * SRAM BIST is functional up to 48 MHz. Note that this field is mutually
 * exclusive with the "SROM_GO" field.
 */
#define CPUSS_BIST_CMD_SRAM_GO                              (1u << 0) /* <0:0> RW1C:RW:0: */


/*
 * 1': Start ROM BIST. Hardware set this field to '0' when BIST is completed.
 * SROM BIST is functional up to 24 MHz.
 */
#define CPUSS_BIST_CMD_SROM_GO                              (1u << 1) /* <1:1> RW1C:RW:0: */


/*
 * BIST data register
 */
#define CPUSS_BIST_DATA_ADDRESS                             (0x40100044)
#define CPUSS_BIST_DATA                                     (*(volatile uint32_t *)(0x40100044))
#define CPUSS_BIST_DATA_DEFAULT                             (0x00000000)

/*
 * Data that is written into a SRAM during a W0 substep (~BIST_DATA.DATA
 * is written into a SRAM during a W1 substep).
 */
#define CPUSS_BIST_DATA_DATA_MASK                           (0xffffffff) /* <0:31> R:RW:0: */
#define CPUSS_BIST_DATA_DATA_POS                            (0)


/*
 * BIST control register
 */
#define CPUSS_BIST_CTL_ADDRESS                              (0x4010004c)
#define CPUSS_BIST_CTL                                      (*(volatile uint32_t *)(0x4010004c))
#define CPUSS_BIST_CTL_DEFAULT                              (0x00000000)

/*
 * Hot-one mask for the SRAMs for which the BIST is performed.
 */
#define CPUSS_BIST_CTL_SRAMS_ENABLED_MASK                   (0x000001ff) /* <0:8> R:RW:0:BIST_SRAM_NR */
#define CPUSS_BIST_CTL_SRAMS_ENABLED_POS                    (0)


/*
 * Specifies how the SRAM BIST addresses are generated (should be set to
 * '0' for SROM BIST):
 * '0': Column address is incremented/decremented till it reaches its maximum/minimum
 * value. Once it reach its maximum/minimum value, it is set to its mimimum/maximum
 * value and only then is the row address incremented/decremented.
 * '1': Row address is incremented/decremented till it reaches its maximum/minimum
 * value. Once it reach its maximum/minimum value, it is set to its mimimum/maximum
 * value and only then is the column address incremented/decremented.
 */
#define CPUSS_BIST_CTL_ROW_FIRST                            (1u << 20) /* <20:20> R:RW:0: */


/*
 * BIST step 0 control register
 */
#define CPUSS_BIST_STEP0_CTL_ADDRESS                        (0x40100050)
#define CPUSS_BIST_STEP0_CTL                                (*(volatile uint32_t *)(0x40100050))
#define CPUSS_BIST_STEP0_CTL_DEFAULT                        (0x00000000)

/*
 * Specifies what sequence of SRAM BIST steps (R0, R1, W0, W1) is performed
 * (not used for SROM BIST):
 * "0": W0. Write SRAM with BIST_DATA.DATA.
 * "1": W1. Write SRAM with ~BIST_DATA.DATA.
 * "2": R0. Read SRAM and compare output to BIST_DATA.DATA.
 * "3": R1. Read SRAM and compare output to ~BIST_DATA.DATA.
 * "4": W0, R0. Write SRAM with BIST_DATA.DATA, followed by read SRAM and
 * compare output to BIST_DATA.DATA (all to the same address).
 * "5": R0, W1.
 * "6": R1, W0.
 * "7": R0, W1, R1.
 * "8": R1, W0, R0.
 * "9": R0, W1, W0.
 * "10": R1, W0, W1.
 * "11": R0, W1, W0, W1.
 * "12": R1, W0, W1, W0.
 * "13": R0, W1, R1, W0.
 * "14": R1, W0, R0, W1.
 * "15": R0, W1, R1, W0, R0, W1.
 */
#define CPUSS_BIST_STEP0_CTL_OPCODE_MASK                    (0x0000000f) /* <0:3> R:RW:0: */
#define CPUSS_BIST_STEP0_CTL_OPCODE_POS                     (0)


/*
 * Specifies direction in which SRAM BIST steps through addresses (not used
 * for SROM BIST):
 * ''0': BIST steps through the SRAM from the maximum row and column addresses
 * (as specified by a design time configurtion parameter when ADDR_START_ENABLED
 * is '0' and as specified by BIST_ADDR_START when ADDR_START_ENABLED is
 * '1')  to the minimum row and column addresses.
 * '1': BIST steps through the SRAM from the minimum row and column addresses
 * ("0" when ADDR_START_ENABLED is '0' and as specified by BIST_ADDR_START
 * when ADDR_START_ENABLED is '1') to the maximum row and column addresses.
 */
#define CPUSS_BIST_STEP0_CTL_UP                             (1u << 4) /* <4:4> R:RW:0: */


/*
 * BIST status register
 */
#define CPUSS_BIST_STATUS_ADDRESS                           (0x40100080)
#define CPUSS_BIST_STATUS                                   (*(volatile uint32_t *)(0x40100080))
#define CPUSS_BIST_STATUS_DEFAULT                           (0x00000000)

/*
 * BIST substep (either R0, R1, W0, or W1).
 */
#define CPUSS_BIST_STATUS_SUB_STEP_MASK                     (0x00000007) /* <0:2> W:R:Undefined: */
#define CPUSS_BIST_STATUS_SUB_STEP_POS                      (0)


/*
 * BIST step (step i uses BIST_STEPi_CTL).
 */
#define CPUSS_BIST_STATUS_STEP_MASK                         (0x00000700) /* <8:10> W:R:Undefined: */
#define CPUSS_BIST_STATUS_STEP_POS                          (8)


/*
 * SRAM identifier.
 */
#define CPUSS_BIST_STATUS_SRAM_MASK                         (0x000f0000) /* <16:19> W:R:Undefined: */
#define CPUSS_BIST_STATUS_SRAM_POS                          (16)


/*
 * 0': BIST passed.
 * '1': BIST failed (SRAM match error or ROM MISR check error). BIST error
 * information is found in SRAM, STEP, SUB_STEP fields and BIST_DATA_ACT,
 * BIST_DATA_EXP and BIST_ADDR registers.
 */
#define CPUSS_BIST_STATUS_FAIL                              (1u << 24) /* <24:24> W1S:RW:0: */


/*
 * BIST data expected register
 */
#define CPUSS_BIST_DATA_ACT_ADDRESS                         (0x40100084)
#define CPUSS_BIST_DATA_ACT                                 (*(volatile uint32_t *)(0x40100084))
#define CPUSS_BIST_DATA_ACT_DEFAULT                         (0x00000000)

/*
 * For SRAM BIST, this field is the SRAM output data that caused a BIST failure.
 * For ROM BIST, this field is the calculated Mulitple Input Shift Register
 * (MISR).
 */
#define CPUSS_BIST_DATA_ACT_DATA_MASK                       (0xffffffff) /* <0:31> W:R:Undefined: */
#define CPUSS_BIST_DATA_ACT_DATA_POS                        (0)


/*
 * BIST data actual register
 */
#define CPUSS_BIST_DATA_EXP_ADDRESS                         (0x40100088)
#define CPUSS_BIST_DATA_EXP                                 (*(volatile uint32_t *)(0x40100088))
#define CPUSS_BIST_DATA_EXP_DEFAULT                         (0x00000000)

/*
 * For SRAM BIST. This field is the expected data from SRAM. This value is
 * BIST_DATA.DATA for a R0 substep and ~BIST_DATA.DATA for a R1 substep.
 */
#define CPUSS_BIST_DATA_EXP_DATA_MASK                       (0xffffffff) /* <0:31> W:R:Undefined: */
#define CPUSS_BIST_DATA_EXP_DATA_POS                        (0)


/*
 * BIST address register
 */
#define CPUSS_BIST_ADDR_ADDRESS                             (0x4010008c)
#define CPUSS_BIST_ADDR                                     (*(volatile uint32_t *)(0x4010008c))
#define CPUSS_BIST_ADDR_DEFAULT                             (0x00000000)

/*
 * Current column address.
 */
#define CPUSS_BIST_ADDR_COL_ADDR_MASK                       (0x00000fff) /* <0:11> W:R:Undefined: */
#define CPUSS_BIST_ADDR_COL_ADDR_POS                        (0)


/*
 * Current row address.
 */
#define CPUSS_BIST_ADDR_ROW_ADDR_MASK                       (0x0fff0000) /* <16:27> W:R:Undefined: */
#define CPUSS_BIST_ADDR_ROW_ADDR_POS                        (16)


/*
 * BIST SROM Multiple Input Shift Register (MISR)
 */
#define CPUSS_BIST_MISR_ADDRESS                             (0x40100090)
#define CPUSS_BIST_MISR                                     (*(volatile uint32_t *)(0x40100090))
#define CPUSS_BIST_MISR_DEFAULT                             (0x00000000)

/*
 * Current value of ROM Multiple Input Shift Register (MISR).
 */
#define CPUSS_BIST_MISR_DATA_MASK                           (0xffffffff) /* <0:31> W:R:Undefined: */
#define CPUSS_BIST_MISR_DATA_POS                            (0)


/*
 * Parallel Test Mode Control Register
 * This register determines the test interface (SWD or PTM) that connects
 * the ATE to the device.
 */
#define CPUSS_PTM_CTL_ADDRESS                               (0x401000c0)
#define CPUSS_PTM_CTL                                       (*(volatile uint32_t *)(0x401000c0))
#define CPUSS_PTM_CTL_DEFAULT                               (0x00000000)

/*
 * 0': SWD mode.
 * '1': PTM mode.
 * This bit is typically set to '1' through the SWD interface to switch to
 * the PTM interface and it is typically set to '0' through the PTM interface
 * to switch to the SWD interface. This bit replaces the m0s8tst IP's CTRL.PTM_MODE_EN
 * MMIO register field, which is rendered ineffective when using CPUSSv2.
 */
#define CPUSS_PTM_CTL_PTM_EN                                (1u << 0) /* <0:0> R:RW:0: */


/*
 * Flash/NVL geometry information
 * This register indicates the flash/NVLatch geometry parameters for the
 * current device. This does not take wounding into account, which may restrict
 * access to flash memory. Note that the default values of the read-only
 * fields are determined by the CPUSS design time configuration.
 */
#define SPCIF_GEOMETRY_ADDRESS                              (0x40110000)
#define SPCIF_GEOMETRY                                      (*(volatile uint32_t *)(0x40110000))
#define SPCIF_GEOMETRY_DEFAULT                              (0x00000000)

/*
 * Regular flash capacity in 256 Byte multiples (chip dependent). If multiple
 * flash macros are present, this field provides the flash capacity of all
 * flash macros together:
 * "0": 256 Bytes.
 * "1": 2*256 Bytes.
 * ...
 * "16383": 16384*256 Bytes.
 */
#define SPCIF_GEOMETRY_FLASH_MASK                           (0x00003fff) /* <0:13> W:R:Undefined:FLASHC_PRESENT */
#define SPCIF_GEOMETRY_FLASH_POS                            (0)


/*
 * Supervisory flash capacity in 256 Byte multiples (chip dependent). If
 * multiple flash macros are present, this field provides the supervisory
 * flash capacity of all flash macros together:
 * "0": 256 Bytes.
 * "1": 2*256 Bytes.
 * ...
 * "63": 64*256 Bytes.
 */
#define SPCIF_GEOMETRY_SFLASH_MASK                          (0x000fc000) /* <14:19> W:R:Undefined:FLASHC_PRESENT */
#define SPCIF_GEOMETRY_SFLASH_POS                           (14)


/*
 * Number of flash macros (chip dependent):
 * "0": 1 flash macro
 * "1": 2 flash macros
 * "2": 3 flash macros
 * "3": 4 flash macros
 */
#define SPCIF_GEOMETRY_NUM_FLASH_MASK                       (0x00300000) /* <20:21> W:R:Undefined:FLASHC_PRESENT */
#define SPCIF_GEOMETRY_NUM_FLASH_POS                        (20)


/*
 * Page size in 64 Byte multiples (chip dependent):
 * "0": 64 byte
 * "1": 128 byte
 * "2": 192 byte
 * "3": 256 byte
 *
 * The page size is used to detemine the number of Bytes in a page for Flash
 * page based operations (e.g. PGM_PAGE).
 *
 * Note: the field name FLASH_ROW is misleading, as this field specifies
 * the number of Bytes in a page, rather than the number of Bytes in a row.
 * In a single plane flash macro architecture, a page consists of a single
 * row. However, in a multi plane flash macro architecture, a page consists
 * of multiple rows from different planes.
 */
#define SPCIF_GEOMETRY_FLASH_ROW_MASK                       (0x00c00000) /* <22:23> W:R:Undefined:FLASHC_PRESENT */
#define SPCIF_GEOMETRY_FLASH_ROW_POS                        (22)


/*
 * 0': SRAM busy wait loop has not been copied.
 * '1': Busy wait loop has been written into SRAM.
 */
#define SPCIF_GEOMETRY_DE_CPD_LP                            (1u << 31) /* <31:31> :RW:0: */


/*
 * Flash/NVL address register
 * Specifies the flash macro, flash partition, flash page and flash byte
 * address on which a flash operation is performed or the NVLatch byte address
 * to be read/written.
 */
#define SPCIF_ADDRESS_ADDRESS                               (0x40110004)
#define SPCIF_ADDRESS                                       (*(volatile uint32_t *)(0x40110004))
#define SPCIF_ADDRESS_DEFAULT                               (0x00000000)

/*
 * Specifies a Byte address:
 * - For flash, this is the byte in a (macro, page) as specified by PAGE_ADDR
 * (for write operations to page byte latches). MSBs are ignored when they
 * exceed the page range; e.g. for 64 byte pages, BYTE_ADDR[7:6] are ignored.
 * - For NVLatch, this is the byte in the NVLatch array (for read/write operations).
 * MSBs are ignored when they exceed the NVLatch array; e.g. for 32 byte
 * array, BYTE_ADDR[7:5] are ignored.
 */
#define SPCIF_ADDRESS_BYTE_ADDR_MASK                        (0x000000ff) /* <0:7> RW:RW:0: */
#define SPCIF_ADDRESS_BYTE_ADDR_POS                         (0)


/*
 * 0': Don't change BYTE_ADDR after a write to FLASH_WR_DATA.
 * '1': Increment BYTE_ADDR after a write to FLASH_WR_DATA.
 */
#define SPCIF_ADDRESS_INC                                   (1u << 8) /* <8:8> R:RW:0:FLASHC_PRESENT */


/*
 * Flash page address for program/erase operations. Supports up to 2048 pages:
 * 512 KByte total flash capacity for 256 Byte pages, 256 KByte total flash
 * capacity for 128 Byte pages, 128 KByte total flash capacity for 64 Byte
 * pages.
 *
 * MSBs are used to identify a specific flash macro; e.g. for a 4 flash macro
 * configuration, with 64 pages per flash macro, PAGE_ADDR[7:6] identifies
 * the flash macro. Note that ADDRESS.ALL can be used to select all flash
 * macros.
 *
 * MSBs are ignored when they exceed the total flash capacity; e.g. for a
 * total page capacity of 256 pages, PAGE_ADDR[10:8] are ignored. In addition,
 * MSBs are ignored when ADDRESS.ALL is '1'
 */
#define SPCIF_ADDRESS_PAGE_ADDR_MASK                        (0x07ff0000) /* <16:26> R:RW:0:FLASHC_PRESENT */
#define SPCIF_ADDRESS_PAGE_ADDR_POS                         (16)


/*
 * Indicates whether address pertains to regular or supervisory flash pages.
 * This bit has no effect for NVLatch operations.
 */
#define SPCIF_ADDRESS_AXA                                   (1u << 28) /* <28:28> R:RW:0:FLASHC_PRESENT */


/*
 * SPCIF timer register
 * Provides the timer period reload value
 */
#define SPCIF_TIMER_ADDRESS                                 (0x40110008)
#define SPCIF_TIMER                                         (*(volatile uint32_t *)(0x40110008))
#define SPCIF_TIMER_DEFAULT                                 (0x00000000)

/*
 * Period of timer in "clk_timer" clock cycles; start of the time counter
 * value. When SW sets FLASH_CONTROL.START_COUNT to '1', the timer counter
 * is counted down to "0" from the start counter value as provided by this
 * register field. Note that TIMER.PERIOD should only be written when FLASH_CONTROL.START_COUNT
 * is '0'. If TIMER.PERIOD is written when FLASH_CONTOL.START_COUNT is '1',
 * an AHB-Lite bus error is generated and the write is dropped/does not take
 * effect.
 *
 * For SRSSv2, the timer counter operates in asynchronous mode and "clk_timer"
 * is the 36 MHz IMO clock ("clk_imo"), which is asynchronous to the system
 * clock ("clk_sys"). In asynchronous mode, a read from this register field
 * returns "0".
 *
 * For SRSS-Lite, the timer operates in asynchronous mode and "clk_timer"
 * is the system clock ("clk_sys"). In synchronous mode (for SRSS-Lite and
 * "clk_timer" is "clk_sys"), a read from this register field returns the
 * current timer counter value.
 */
#define SPCIF_TIMER_PERIOD_MASK                             (0xffffffff) /* <0:31> A:RW:0: */
#define SPCIF_TIMER_PERIOD_POS                              (0)


/*
 * Flash control register
 * Controls program/erase operations on the Flash
 */
#define SPCIF_FLASH_CONTROL_ADDRESS                         (0x4011000c)
#define SPCIF_FLASH_CONTROL                                 (*(volatile uint32_t *)(0x4011000c))
#define SPCIF_FLASH_CONTROL_DEFAULT                         (0x00000000)

/*
 * Flash mode:
 */
#define SPCIF_FLASH_CONTROL_MODE_MASK                       (0x0000000f) /* <0:3> R:RW:0:FLASHC_PRESENT */
#define SPCIF_FLASH_CONTROL_MODE_POS                        (0)


/*
 * Flash Seq input, see BROS for details on values.
 */
#define SPCIF_FLASH_CONTROL_SEQ_MASK                        (0x00000030) /* <4:5> R:RW:0:FLASHC_PRESENT */
#define SPCIF_FLASH_CONTROL_SEQ_POS                         (4)


/*
 * Enables a single cycle high time of the Aclk to the flash macro(s).  This
 * bit only has
 * effect when FLASH_CONTROL.NO_READS = '1'.
 */
#define SPCIF_FLASH_CONTROL_ACLK_EN                         (1u << 6) /* <6:6> RW:RW:0: */


/*
 * Configures the timer to assert the PE output while the timer is active:
 * '0': Pumps off.
 * '1': Pumps on while timer is counting.
 */
#define SPCIF_FLASH_CONTROL_PE_ENABLE                       (1u << 7) /* <7:7> R:RW:0: */


/*
 * Manual override for pump enables:
 * '0': Pumps operate normally (per PE_ENABLE).
 * '1': Pumps are enabled as long as this bit is set.
 */
#define SPCIF_FLASH_CONTROL_PE_OVERRIDE                     (1u << 8) /* <8:8> R:RW:0: */


/*
 * This bit should be enabled so that the timer can start counting down.
 * This bit is cleared by hardware when counter period expires. The start
 * timer counter value is provided by TIMER.PERIOD.
 */
#define SPCIF_FLASH_CONTROL_START_COUNT                     (1u << 9) /* <9:9> RW1C:RW1S:0: */


/*
 * Selects which of the 3 pump enable outputs to drive with the signal driven
 * from the PE_ENABLE and PE_OVERRIDE bits.  (3: Reserved)
 */
#define SPCIF_FLASH_CONTROL_PE_SELECT_MASK                  (0x00000c00) /* <10:11> R:RW:0: */
#define SPCIF_FLASH_CONTROL_PE_SELECT_POS                   (10)


/*
 * Firmware sets this bit to block the flash controller from issuing more
 * requests to flash. When '1', the flash controller returns an AHB-Lite
 * bus error on any read requests. Firmware must ensure no read accesses
 * are pending or must not take control of the flash immediately with setting
 * this bit.
 */
#define SPCIF_FLASH_CONTROL_NO_READS                        (1u << 14) /* <14:14> R:RW:0:FLASHC_PRESENT */


/*
 * Flash write data register
 * Writing to this register writes one byte into the Flash page latch (at
 * position ADDRESS.WR_ADDR).  Multiple bytes can be written in sequence
 * by using ADDRESS.INC. Writes to this register may block the CPU bus for
 * a few cycles to enable the flash page latches to keep up.
 */
#define SPCIF_FLASH_WR_DATA_ADDRESS                         (0x40110010)
#define SPCIF_FLASH_WR_DATA                                 (*(volatile uint32_t *)(0x40110010))
#define SPCIF_FLASH_WR_DATA_DEFAULT                         (0x00000000)

/*
 * Data byte for a specific page byte latch
 */
#define SPCIF_FLASH_WR_DATA_DATA_MASK                       (0x000000ff) /* <0:7> A:RW:0: */
#define SPCIF_FLASH_WR_DATA_DATA_POS                        (0)


/*
 * Bookmark pointer register
 * Used as storage space for the SROM to write a bookmark. Bookmarks are
 * used as a pointer for cases when the SPCIF timer is setup to generate
 * an interrupt and control of the CM0 is given back to user code. When the
 * SPCIF interrupt is activated, the SROM uses this value to know where to
 * resume its function execution.
 */
#define SPCIF_BOOKMARK_ADDRESS                              (0x40110028)
#define SPCIF_BOOKMARK                                      (*(volatile uint32_t *)(0x40110028))
#define SPCIF_BOOKMARK_DEFAULT                              (0x00000000)

/*
 * SROM bookmark.
 */
#define SPCIF_BOOKMARK_BOOKMARK_MASK                        (0xffffffff) /* <0:31> R:RW:0: */
#define SPCIF_BOOKMARK_BOOKMARK_POS                         (0)


/*
 * Flash-Lite test mode register
 * Configures test mode parameters and signals for Flash macro. See m0s8cpussv2/flash
 * memory BROS for the use of this register.
 */
#define SPCIF_FMLT_DFT_ADDRESS                              (0x40110030)
#define SPCIF_FMLT_DFT                                      (*(volatile uint32_t *)(0x40110030))
#define SPCIF_FMLT_DFT_DEFAULT                              (0x00000000)

/*
 * Flash Test Mode Select, see BROS for details on each value.
 */
#define SPCIF_FMLT_DFT_FMLT_TM_MASK                         (0x0000001f) /* <0:4> R:RW:0: */
#define SPCIF_FMLT_DFT_FMLT_TM_POS                          (0)


/*
 * Positive/Negative Margin Voltage select.
 */
#define SPCIF_FMLT_DFT_FMLT_PNB                             (1u << 5) /* <5:5> R:RW:0: */

/*
 * Use Vneg
 */
#define SPCIF_FMLT_DFT_FMLT_PNB_FMLT_PNB_VNEG               (0)
/*
 * Use Vpos
 */
#define SPCIF_FMLT_DFT_FMLT_PNB_FMLT_PNB_VPOS               (1)

/*
 * When DFT.TM_XYDEC = 1, if one and only one WL or Y select address is on,
 * this signal is asserted.
 */
#define SPCIF_FMLT_DFT_TM_XYOUT                             (1u << 17) /* <17:17> RW:R:0: */


/*
 * SystemCall routines in ROM provide flash programming functionality. Flash
 * programming requires proper trimming of certain on-chip components. This
 * trimming information is part of the supervisory flash memory. The supervisory
 * flash memory is programmed using the SystemCall routines. As a result,
 * the initial programming of trimming information in the supervisory flash
 * memory can NOT rely on the information in the supervisory flash memory
 * (supervisory flash memory trimming information is not initialized). Instead,
 * this trimming information is provided at specific SRAM locations (known
 * by the SystemCall routines). This bit field specifies where the SystemCall
 * routines can find the trimming information.
 */
#define SPCIF_FMLT_DFT_PARAM_LOC                            (1u << 31) /* <31:31> R:RW:0: */


/*
 * SPCIF interrupt request register
 */
#define SPCIF_INTR_ADDRESS                                  (0x401107f0)
#define SPCIF_INTR                                          (*(volatile uint32_t *)(0x401107f0))
#define SPCIF_INTR_DEFAULT                                  (0x00000000)

/*
 * Timer counter value reaches "0". Set to '1', when event is detected. Write
 * INTR field with '1', to clear bit. Write INTR_SET field with '1', to set
 * bit.
 */
#define SPCIF_INTR_TIMER                                    (1u << 0) /* <0:0> RW1S:RW1C:0: */


/*
 * SPCIF interrupt set request register
 * When read, this register reflects the interrupt request register.
 */
#define SPCIF_INTR_SET_ADDRESS                              (0x401107f4)
#define SPCIF_INTR_SET                                      (*(volatile uint32_t *)(0x401107f4))
#define SPCIF_INTR_SET_DEFAULT                              (0x00000000)

/*
 * Write INTR_SET field with '1' to set corresponding INTR field.
 */
#define SPCIF_INTR_SET_TIMER                                (1u << 0) /* <0:0> A:RW1S:0: */


/*
 * SPCIF interrupt mask register
 */
#define SPCIF_INTR_MASK_ADDRESS                             (0x401107f8)
#define SPCIF_INTR_MASK                                     (*(volatile uint32_t *)(0x401107f8))
#define SPCIF_INTR_MASK_DEFAULT                             (0x00000000)

/*
 * Mask for corresponding field in INTR register.
 */
#define SPCIF_INTR_MASK_TIMER                               (1u << 0) /* <0:0> R:RW:0: */


/*
 * SPCIF interrupt masked request register
 * When read, this register reflects a bitwise and between the interrupt
 * request and mask registers.
 */
#define SPCIF_INTR_MASKED_ADDRESS                           (0x401107fc)
#define SPCIF_INTR_MASKED                                   (*(volatile uint32_t *)(0x401107fc))
#define SPCIF_INTR_MASKED_DEFAULT                           (0x00000000)

/*
 * Logical and of corresponding request and mask fields.
 */
#define SPCIF_INTR_MASKED_TIMER                             (1u << 0) /* <0:0> W:R:0: */


/*
 * Flash macro 0 DAC trim register 0
 * Control/trim values for flash macro 0 DACs. These values are determined
 * at manufacturing and written in flash macro 0 supervisory memory. Note
 * that IDAC and SLOPE values for flash macro 0 are provided by the NVLatch
 * structure if this structure is present (after a DeepSleep reset "rst_sys_dpslp_n",
 * the HW copies the NVLatch structure IDAC and SLOPE values into this MMIO
 * register). If the NVLatch structure is NOT present, the boot process reads
 * flash macro 0 supervisory memory and writes to this register. This flash
 * macro read is an uncontrolled/untrimmed flash macro read in which the
 * default control/trim values apply. See m0s8cpussv2/flash memory BROS for
 * the use of this register.
 */
#define SPCIF_TRIM_M0_DAC0_ADDRESS                          (0x4011ff00)
#define SPCIF_TRIM_M0_DAC0                                  (*(volatile uint32_t *)(0x4011ff00))
#define SPCIF_TRIM_M0_DAC0_DEFAULT                          (0x00000000)

/*
 * Used as "idac[4:0]" flash macro 0 input. It enables trimming of the sense
 * amplifier reference current. Each increment causes an increase of ~0.6
 * µA:
 * "0": (0 + 2) * 0.6 µA = 1.2 µA
 * "1": (1 + 2) * 0.6 µA = 1.8 µA
 * ...
 * "31": (31 + 2) * 0.6 µA = 19.8 µA
 */
#define SPCIF_TRIM_M0_DAC0_IDAC_MASK                        (0x0000001f) /* <0:4> RW:RW:0: */
#define SPCIF_TRIM_M0_DAC0_IDAC_POS                         (0)


/*
 * Used as "slope[2:0]" flash macro 0 input (for regular FLASH) or used as
 * "sdac[2:0]" flash macro 0 input (for FLASH-Lite). It determines the slope
 * of the sense amplifier reference current as a function of temperature.
 * Each increment cause an increase of 5 nA/C:
 * "0": -5 nA/C
 * "1": 0 nA/C (no temperature dependency)
 * ...
 * "7":  30 nA/C
 */
#define SPCIF_TRIM_M0_DAC0_SLOPE_MASK                       (0x000000e0) /* <5:7> RW:RW:0: */
#define SPCIF_TRIM_M0_DAC0_SLOPE_POS                        (5)


/*
 * Flash macro 0 DAC trim register 1
 * Control/trim values for flash macro 0 DACs. These values are determined
 * at manufacturing and written in flash macro 0 supervisory memory. The
 * boot process reads flash macro 0 supervisory memory and writes to this
 * register. This flash macro read is an uncontrolled/untrimmed flash macro
 * read in which the default control/trim values apply. See m0s8cpussv2/flash
 * memory BROS for the use of this register.
 */
#define SPCIF_TRIM_M0_DAC1_ADDRESS                          (0x4011ff04)
#define SPCIF_TRIM_M0_DAC1                                  (*(volatile uint32_t *)(0x4011ff04))
#define SPCIF_TRIM_M0_DAC1_DEFAULT                          (0x00000000)

/*
 * Used as "mdac[7:0]" flash macro 0 input. It controls the output margin
 * voltage VMARG as a function of input supplies VPOS and VNEG. Each increment
 * increases the output margin voltage as a multiple of VPOS/255 (positive
 * margin mode) or VNEG/255 (negative margin mode):
 * "0": 0 * VPOS/255 (positive margin mode); 0 * VNEG/255 (negative margin
 * mode)
 * "1": 1 * VPOS/255 (positive margin mode); 1 * VNEG/255 (negative margin
 * mode)
 * ...
 * "255": VPOS (positive margin mode); VNEG (negative margin mode)
 *
 * The flash macro 0 pumps (VPOS and VNEG) are also used for the NVLatch
 * structure and therefore the MDAC field needs to be set for NVLatch operations.
 */
#define SPCIF_TRIM_M0_DAC1_MDAC_MASK                        (0x000000ff) /* <0:7> R:RW:0: */
#define SPCIF_TRIM_M0_DAC1_MDAC_POS                         (0)


/*
 * Flash macro 0 DAC trim register 2
 * Control/trim values for flash macro 0 DACs. These values are determined
 * at manufacturing and written in flash macro 0 supervisory memory. The
 * boot process reads flash macro 0 supervisory memory and writes to this
 * register. This flash macro read is an uncontrolled/untrimmed flash macro
 * read in which the default control/trim values apply. See m0s8cpussv2/flash
 * memory BROS for the use of this register.
 */
#define SPCIF_TRIM_M0_DAC2_ADDRESS                          (0x4011ff08)
#define SPCIF_TRIM_M0_DAC2                                  (*(volatile uint32_t *)(0x4011ff08))
#define SPCIF_TRIM_M0_DAC2_DEFAULT                          (0x00000000)

/*
 * Used as "pdac[3:0]" flash macro 0 input. It control the trimming of the
 * positive pump output voltage VPOS. Each increment causes an increase of
 * ~0.1 V. In normal mode:
 * "0": VPOS = 5.9 + 0 * 0.1 V = 5.9 V
 * "1": VPOS = 5.9 + 1 * 0.1 V = 6.0 V
 * ...
 * "15": VPOS = 5.9 + 15 * 0.1 V = 7.4 V
 *
 * The flash macro 0 pumps (VPOS and VNEG) are also used for the NVLatch
 * structure and therefore the PDAC field needs to be set for NVLatch operations.
 */
#define SPCIF_TRIM_M0_DAC2_PDAC_MASK                        (0x0000000f) /* <0:3> R:RW:0: */
#define SPCIF_TRIM_M0_DAC2_PDAC_POS                         (0)


/*
 * Used as "ndac[3:0]" flash macro 0 input. It control the trimming of the
 * negative pump output voltage VNEG. Each increment causes an decrease of
 * ~0.1 V. In normal mode:
 * "0": VNEG = -3.0 - 0 * 0.1 V = -3.0 V
 * "1": VNEG = -3.0 - 1 * 0.1 V = -3.1 V
 * ...
 * "15": VNEG = -3.0 - 15 * 0.1 V = -4.5 V
 *
 * The flash macro 0 pumps (VPOS and VNEG) are also used for the NVLatch
 * structure and therefore the NDAC field needs to be set for NVLatch operations.
 */
#define SPCIF_TRIM_M0_DAC2_NDAC_MASK                        (0x000000f0) /* <4:7> R:RW:0: */
#define SPCIF_TRIM_M0_DAC2_NDAC_POS                         (4)


/*
 * Flash macro 0 DAC trim register 3
 * Control/trim values for flash macro 0 DACs. These values are determined
 * at manufacturing and written in flash macro 0 supervisory memory. The
 * boot process reads flash macro 0 supervisory memory and writes to this
 * register. See m0s8cpussv2/flash memory BROS for the use of this register.
 */
#define SPCIF_TRIM_M0_DAC3_ADDRESS                          (0x4011ff0c)
#define SPCIF_TRIM_M0_DAC3                                  (*(volatile uint32_t *)(0x4011ff0c))
#define SPCIF_TRIM_M0_DAC3_DEFAULT                          (0x00000000)

/*
 * Used as "bdac[3:0]" flash macro 0 input.
 */
#define SPCIF_TRIM_M0_DAC3_BDAC_MASK                        (0x0000000f) /* <0:3> R:RW:0: */
#define SPCIF_TRIM_M0_DAC3_BDAC_POS                         (0)


/*
 * Used as "cdac[2:0]" flash macro 0 input (for FLASH-Lite). Temperature
 * compensated trim DAC. It controls Vctat slope for VPOS.
 */
#define SPCIF_TRIM_M0_DAC3_CDAC_MASK                        (0x00000070) /* <4:6> R:RW:0:FMLT_OR_S8FS */
#define SPCIF_TRIM_M0_DAC3_CDAC_POS                         (4)


/*
 * Watchpoint Comparator Configuration
 * Defines the number of comparators implemented
 */
#define CM0_DWT_CTRL_ADDRESS                                (0xe0001000)
#define CM0_DWT_CTRL                                        (*(volatile uint32_t *)(0xe0001000))
#define CM0_DWT_CTRL_DEFAULT                                (0x20000000)

/*
 * Number of comparators available
 */
#define CM0_DWT_CTRL_NUMCOMP_MASK                           (0xf0000000) /* <28:31> :R:2: */
#define CM0_DWT_CTRL_NUMCOMP_POS                            (28)


/*
 * Watchpoint Comparator PC Sample
 * Samples the current value of the program counter.  Unless DWT_PCSR reads
 * as 0xFFFFFFFF, under the conditions described in Program counter sampling
 * support on page C1-344, bit [0] is RAZ. When RAZ, bit [0] does not reflect
 * instruction set state as is the case with similar functionality in
 * other ARM architecture profiles.
 */
#define CM0_DWT_PCSR_ADDRESS                                (0xe000101c)
#define CM0_DWT_PCSR                                        (*(volatile uint32_t *)(0xe000101c))
#define CM0_DWT_PCSR_DEFAULT                                (0x00000000)

/*
 * Executed Instruction Address sample value
 */
#define CM0_DWT_PCSR_EIASAMPLE_MASK                         (0xffffffff) /* <0:31> W:R:X: */
#define CM0_DWT_PCSR_EIASAMPLE_POS                          (0)


/*
 * Watchpoint Comparator Compare Value
 * Provides a reference value for use by comparator
 */
#define CM0_DWT_COMP0_ADDRESS                               (0xe0001020)
#define CM0_DWT_COMP0                                       (*(volatile uint32_t *)(0xe0001020))
#define CM0_DWT_COMP0_DEFAULT                               (0x00000000)

/*
 * Reference value for comparison. See The DWT comparators on page C1-341.
 */
#define CM0_DWT_COMP0_COMP_MASK                             (0xffffffff) /* <0:31> R:RW:X: */
#define CM0_DWT_COMP0_COMP_POS                              (0)


/*
 * Watchpoint Comparator Mask
 * Provides the size of the ignore mask applied to the access address range
 * matching
 */
#define CM0_DWT_MASK0_ADDRESS                               (0xe0001024)
#define CM0_DWT_MASK0                                       (*(volatile uint32_t *)(0xe0001024))
#define CM0_DWT_MASK0_DEFAULT                               (0x00000000)

/*
 * The size of the ignore mask applied to address range matching. See The
 * DWT comparators on
 * page C1-341 for the usage model. The mask range is IMPLEMENTATION DEFINED.
 * Writing all ones to this field and reading it back can be used to determine
 * the maximum mask size supported.
 */
#define CM0_DWT_MASK0_MASK_MASK                             (0xffffffff) /* <0:31> R:RW:X: */
#define CM0_DWT_MASK0_MASK_POS                              (0)


/*
 * Watchpoint Comparator Function
 * Controls the operation of the comparator
 */
#define CM0_DWT_FUNCTION0_ADDRESS                           (0xe0001028)
#define CM0_DWT_FUNCTION0                                   (*(volatile uint32_t *)(0xe0001028))
#define CM0_DWT_FUNCTION0_DEFAULT                           (0x00000000)

/*
 * Select action on comparator match.
 */
#define CM0_DWT_FUNCTION0_FUNCTION_MASK                     (0x0000000f) /* <0:3> R:RW:0: */
#define CM0_DWT_FUNCTION0_FUNCTION_POS                      (0)


/*
 * Comparator match. It indicates that the operation defined by FUNCTION
 * has occurred since the bit was last read:
 * 0 the associated comparator has matched.
 * 1 the associated comparator has not matched.
 * Reading the register clears this bit to 0.
 */
#define CM0_DWT_FUNCTION0_MATCHED                           (1u << 24) /* <24:24> RW:R:0: */


/*
 * Watchpoint Comparator Compare Value
 * Provides a reference value for use by comparator
 */
#define CM0_DWT_COMP1_ADDRESS                               (0xe0001030)
#define CM0_DWT_COMP1                                       (*(volatile uint32_t *)(0xe0001030))
#define CM0_DWT_COMP1_DEFAULT                               (0x00000000)

/*
 * Reference value for comparison. See The DWT comparators on page C1-341.
 */
#define CM0_DWT_COMP1_COMP_MASK                             (0xffffffff) /* <0:31> R:RW:X: */
#define CM0_DWT_COMP1_COMP_POS                              (0)


/*
 * Watchpoint Comparator Mask
 * Provides the size of the ignore mask applied to the access address range
 * matching
 */
#define CM0_DWT_MASK1_ADDRESS                               (0xe0001034)
#define CM0_DWT_MASK1                                       (*(volatile uint32_t *)(0xe0001034))
#define CM0_DWT_MASK1_DEFAULT                               (0x00000000)

/*
 * The size of the ignore mask applied to address range matching. See The
 * DWT comparators on
 * page C1-341 for the usage model. The mask range is IMPLEMENTATION DEFINED.
 * Writing all ones to this field and reading it back can be used to determine
 * the maximum mask size supported.
 */
#define CM0_DWT_MASK1_MASK_MASK                             (0xffffffff) /* <0:31> R:RW:X: */
#define CM0_DWT_MASK1_MASK_POS                              (0)


/*
 * Watchpoint Comparator Function
 * Controls the operation of the comparator
 */
#define CM0_DWT_FUNCTION1_ADDRESS                           (0xe0001038)
#define CM0_DWT_FUNCTION1                                   (*(volatile uint32_t *)(0xe0001038))
#define CM0_DWT_FUNCTION1_DEFAULT                           (0x00000000)

/*
 * Select action on comparator match.
 */
#define CM0_DWT_FUNCTION1_FUNCTION_MASK                     (0x0000000f) /* <0:3> R:RW:0: */
#define CM0_DWT_FUNCTION1_FUNCTION_POS                      (0)


/*
 * Comparator match. It indicates that the operation defined by FUNCTION
 * has occurred since the bit was last read:
 * 0 the associated comparator has matched.
 * 1 the associated comparator has not matched.
 * Reading the register clears this bit to 0.
 */
#define CM0_DWT_FUNCTION1_MATCHED                           (1u << 24) /* <24:24> RW:R:0: */


/*
 * Watchpoint Unit CoreSight ROM Table Peripheral ID #4
 */
#define CM0_DWT_PID4_ADDRESS                                (0xe0001fd0)
#define CM0_DWT_PID4                                        (*(volatile uint32_t *)(0xe0001fd0))
#define CM0_DWT_PID4_DEFAULT                                (0x00000004)

/*
 * Peripheral ID #4
 */
#define CM0_DWT_PID4_VALUE_MASK                             (0xffffffff) /* <0:31> :R:4: */
#define CM0_DWT_PID4_VALUE_POS                              (0)


/*
 * Watchpoint Unit CoreSight ROM Table Peripheral ID #0
 */
#define CM0_DWT_PID0_ADDRESS                                (0xe0001fe0)
#define CM0_DWT_PID0                                        (*(volatile uint32_t *)(0xe0001fe0))
#define CM0_DWT_PID0_DEFAULT                                (0x0000000a)

/*
 * Peripheral ID #0
 */
#define CM0_DWT_PID0_VALUE_MASK                             (0xffffffff) /* <0:31> :R:10: */
#define CM0_DWT_PID0_VALUE_POS                              (0)


/*
 * Watchpoint Unit CoreSight ROM Table Peripheral ID #1
 */
#define CM0_DWT_PID1_ADDRESS                                (0xe0001fe4)
#define CM0_DWT_PID1                                        (*(volatile uint32_t *)(0xe0001fe4))
#define CM0_DWT_PID1_DEFAULT                                (0x000000b0)

/*
 * Peripheral ID #1
 */
#define CM0_DWT_PID1_VALUE_MASK                             (0xffffffff) /* <0:31> :R:176: */
#define CM0_DWT_PID1_VALUE_POS                              (0)


/*
 * Watchpoint Unit CoreSight ROM Table Peripheral ID #2
 */
#define CM0_DWT_PID2_ADDRESS                                (0xe0001fe8)
#define CM0_DWT_PID2                                        (*(volatile uint32_t *)(0xe0001fe8))
#define CM0_DWT_PID2_DEFAULT                                (0x0000000b)

/*
 * Peripheral ID #2
 */
#define CM0_DWT_PID2_VALUE_MASK                             (0xffffffff) /* <0:31> :R:11: */
#define CM0_DWT_PID2_VALUE_POS                              (0)


/*
 * Watchpoint Unit CoreSight ROM Table Peripheral ID #3
 */
#define CM0_DWT_PID3_ADDRESS                                (0xe0001fec)
#define CM0_DWT_PID3                                        (*(volatile uint32_t *)(0xe0001fec))
#define CM0_DWT_PID3_DEFAULT                                (0x00000000)

/*
 * Peripheral ID #3
 */
#define CM0_DWT_PID3_VALUE_MASK                             (0xffffffff) /* <0:31> :R:0: */
#define CM0_DWT_PID3_VALUE_POS                              (0)


/*
 * Watchpoint Unit CoreSight ROM Table Component ID #0
 */
#define CM0_DWT_CID0_ADDRESS                                (0xe0001ff0)
#define CM0_DWT_CID0                                        (*(volatile uint32_t *)(0xe0001ff0))
#define CM0_DWT_CID0_DEFAULT                                (0x0000000d)

/*
 * Component ID #0
 */
#define CM0_DWT_CID0_VALUE_MASK                             (0xffffffff) /* <0:31> :R:13: */
#define CM0_DWT_CID0_VALUE_POS                              (0)


/*
 * Watchpoint Unit CoreSight ROM Table Component ID #1
 */
#define CM0_DWT_CID1_ADDRESS                                (0xe0001ff4)
#define CM0_DWT_CID1                                        (*(volatile uint32_t *)(0xe0001ff4))
#define CM0_DWT_CID1_DEFAULT                                (0x000000e0)

/*
 * Component ID #1
 */
#define CM0_DWT_CID1_VALUE_MASK                             (0xffffffff) /* <0:31> :R:224: */
#define CM0_DWT_CID1_VALUE_POS                              (0)


/*
 * Watchpoint Unit CoreSight ROM Table Component ID #2
 */
#define CM0_DWT_CID2_ADDRESS                                (0xe0001ff8)
#define CM0_DWT_CID2                                        (*(volatile uint32_t *)(0xe0001ff8))
#define CM0_DWT_CID2_DEFAULT                                (0x00000005)

/*
 * Component ID #2
 */
#define CM0_DWT_CID2_VALUE_MASK                             (0xffffffff) /* <0:31> :R:5: */
#define CM0_DWT_CID2_VALUE_POS                              (0)


/*
 * Watchpoint Unit CoreSight ROM Table Component ID #3
 */
#define CM0_DWT_CID3_ADDRESS                                (0xe0001ffc)
#define CM0_DWT_CID3                                        (*(volatile uint32_t *)(0xe0001ffc))
#define CM0_DWT_CID3_DEFAULT                                (0x000000b1)

/*
 * Component ID #3
 */
#define CM0_DWT_CID3_VALUE_MASK                             (0xffffffff) /* <0:31> :R:177: */
#define CM0_DWT_CID3_VALUE_POS                              (0)


/*
 * Breakpoint Unit Control
 * Provides BPU implementation information, and the global enable for the
 * BPU.
 */
#define CM0_BP_CTRL_ADDRESS                                 (0xe0002000)
#define CM0_BP_CTRL                                         (*(volatile uint32_t *)(0xe0002000))
#define CM0_BP_CTRL_DEFAULT                                 (0x00000040)

/*
 * Enables the BPU:
 * 0 BPU is disabled.
 * 1 BPU is enabled.
 */
#define CM0_BP_CTRL_ENABLE                                  (1u << 0) /* <0:0> R:RW:0: */


/*
 * RAZ on reads, SBO for writes. If written as zero, the write to the register
 * is ignored.
 */
#define CM0_BP_CTRL_KEY                                     (1u << 1) /* <1:1> R:RW:0: */


/*
 * The number of breakpoint comparators. If NUM_CODE is zero, the implementation
 * does not support any comparators.
 */
#define CM0_BP_CTRL_NUM_CODE_MASK                           (0x000000f0) /* <4:7> :R:4: */
#define CM0_BP_CTRL_NUM_CODE_POS                            (4)


/*
 * Breakpoint Compare Register
 * Holds a breakpoint address for comparison with instruction addresses in
 * the Code
 * memory region, see The system address map on page B3-258 for more information.
 */
#define CM0_BP_COMP0_ADDRESS                                (0xe0002008)
#define CM0_BP_COMP0                                        (*(volatile uint32_t *)(0xe0002008))
#define CM0_BP_COMP0_DEFAULT                                (0x00000000)

/*
 * Enables the comparator:
 * Note BP_CTRL.ENABLE must also be set to 1 to enable a comparator.
 */
#define CM0_BP_COMP0_ENABLE                                 (1u << 0) /* <0:0> R:RW:0: */


/*
 * Stores bits [28:2] of the comparison address. The comparison address is
 * compared with the address from the Code memory region. Bits [31:29] and
 * [1:0] of the comparison address are zero.
 */
#define CM0_BP_COMP0_COMP_ADDR_MASK                         (0x1ffffffc) /* <2:28> R:RW:X: */
#define CM0_BP_COMP0_COMP_ADDR_POS                          (2)


/*
 * BP_MATCH defines the behavior when the COMP address is matched.
 */
#define CM0_BP_COMP0_MATCH_MASK                             (0xc0000000) /* <30:31> R:RW:X: */
#define CM0_BP_COMP0_MATCH_POS                              (30)


/*
 * Breakpoint Compare Register
 * Holds a breakpoint address for comparison with instruction addresses in
 * the Code
 * memory region, see The system address map on page B3-258 for more information.
 */
#define CM0_BP_COMP1_ADDRESS                                (0xe000200c)
#define CM0_BP_COMP1                                        (*(volatile uint32_t *)(0xe000200c))
#define CM0_BP_COMP1_DEFAULT                                (0x00000000)

/*
 * Enables the comparator:
 * Note BP_CTRL.ENABLE must also be set to 1 to enable a comparator.
 */
#define CM0_BP_COMP1_ENABLE                                 (1u << 0) /* <0:0> R:RW:0: */


/*
 * Stores bits [28:2] of the comparison address. The comparison address is
 * compared with the address from the Code memory region. Bits [31:29] and
 * [1:0] of the comparison address are zero.
 */
#define CM0_BP_COMP1_COMP_ADDR_MASK                         (0x1ffffffc) /* <2:28> R:RW:X: */
#define CM0_BP_COMP1_COMP_ADDR_POS                          (2)


/*
 * BP_MATCH defines the behavior when the COMP address is matched.
 */
#define CM0_BP_COMP1_MATCH_MASK                             (0xc0000000) /* <30:31> R:RW:X: */
#define CM0_BP_COMP1_MATCH_POS                              (30)


/*
 * Breakpoint Compare Register
 * Holds a breakpoint address for comparison with instruction addresses in
 * the Code
 * memory region, see The system address map on page B3-258 for more information.
 */
#define CM0_BP_COMP2_ADDRESS                                (0xe0002010)
#define CM0_BP_COMP2                                        (*(volatile uint32_t *)(0xe0002010))
#define CM0_BP_COMP2_DEFAULT                                (0x00000000)

/*
 * Enables the comparator:
 * Note BP_CTRL.ENABLE must also be set to 1 to enable a comparator.
 */
#define CM0_BP_COMP2_ENABLE                                 (1u << 0) /* <0:0> R:RW:0: */


/*
 * Stores bits [28:2] of the comparison address. The comparison address is
 * compared with the address from the Code memory region. Bits [31:29] and
 * [1:0] of the comparison address are zero.
 */
#define CM0_BP_COMP2_COMP_ADDR_MASK                         (0x1ffffffc) /* <2:28> R:RW:X: */
#define CM0_BP_COMP2_COMP_ADDR_POS                          (2)


/*
 * BP_MATCH defines the behavior when the COMP address is matched.
 */
#define CM0_BP_COMP2_MATCH_MASK                             (0xc0000000) /* <30:31> R:RW:X: */
#define CM0_BP_COMP2_MATCH_POS                              (30)


/*
 * Breakpoint Compare Register
 * Holds a breakpoint address for comparison with instruction addresses in
 * the Code
 * memory region, see The system address map on page B3-258 for more information.
 */
#define CM0_BP_COMP3_ADDRESS                                (0xe0002014)
#define CM0_BP_COMP3                                        (*(volatile uint32_t *)(0xe0002014))
#define CM0_BP_COMP3_DEFAULT                                (0x00000000)

/*
 * Enables the comparator.
 * Note BP_CTRL.ENABLE must also be set to 1 to enable a comparator.
 */
#define CM0_BP_COMP3_ENABLE                                 (1u << 0) /* <0:0> R:RW:0: */


/*
 * Stores bits [28:2] of the comparison address. The comparison address is
 * compared with the address from the Code memory region. Bits [31:29] and
 * [1:0] of the comparison address are zero.
 */
#define CM0_BP_COMP3_COMP_ADDR_MASK                         (0x1ffffffc) /* <2:28> R:RW:X: */
#define CM0_BP_COMP3_COMP_ADDR_POS                          (2)


/*
 * BP_MATCH defines the behavior when the COMP address is matched.
 */
#define CM0_BP_COMP3_MATCH_MASK                             (0xc0000000) /* <30:31> R:RW:X: */
#define CM0_BP_COMP3_MATCH_POS                              (30)


/*
 * Breakpoint Unit CoreSight ROM Table Peripheral ID #4
 */
#define CM0_BP_PID4_ADDRESS                                 (0xe0002fd0)
#define CM0_BP_PID4                                         (*(volatile uint32_t *)(0xe0002fd0))
#define CM0_BP_PID4_DEFAULT                                 (0x00000004)

/*
 * Peripheral ID #4
 */
#define CM0_BP_PID4_VALUE_MASK                              (0xffffffff) /* <0:31> :R:4: */
#define CM0_BP_PID4_VALUE_POS                               (0)


/*
 * Breakpoint Unit CoreSight ROM Table Peripheral ID #0
 */
#define CM0_BP_PID0_ADDRESS                                 (0xe0002fe0)
#define CM0_BP_PID0                                         (*(volatile uint32_t *)(0xe0002fe0))
#define CM0_BP_PID0_DEFAULT                                 (0x0000000b)

/*
 * Peripheral ID #0
 */
#define CM0_BP_PID0_VALUE_MASK                              (0xffffffff) /* <0:31> :R:11: */
#define CM0_BP_PID0_VALUE_POS                               (0)


/*
 * Breakpoint Unit CoreSight ROM Table Peripheral ID #1
 */
#define CM0_BP_PID1_ADDRESS                                 (0xe0002fe4)
#define CM0_BP_PID1                                         (*(volatile uint32_t *)(0xe0002fe4))
#define CM0_BP_PID1_DEFAULT                                 (0x000000b0)

/*
 * Peripheral ID #1
 */
#define CM0_BP_PID1_VALUE_MASK                              (0xffffffff) /* <0:31> :R:176: */
#define CM0_BP_PID1_VALUE_POS                               (0)


/*
 * Breakpoint Unit CoreSight ROM Table Peripheral ID #2
 */
#define CM0_BP_PID2_ADDRESS                                 (0xe0002fe8)
#define CM0_BP_PID2                                         (*(volatile uint32_t *)(0xe0002fe8))
#define CM0_BP_PID2_DEFAULT                                 (0x0000000b)

/*
 * Peripheral ID #2
 */
#define CM0_BP_PID2_VALUE_MASK                              (0xffffffff) /* <0:31> :R:11: */
#define CM0_BP_PID2_VALUE_POS                               (0)


/*
 * Breakpoint Unit CoreSight ROM Table Peripheral ID #3
 */
#define CM0_BP_PID3_ADDRESS                                 (0xe0002fec)
#define CM0_BP_PID3                                         (*(volatile uint32_t *)(0xe0002fec))
#define CM0_BP_PID3_DEFAULT                                 (0x00000000)

/*
 * Peripheral ID #3
 */
#define CM0_BP_PID3_VALUE_MASK                              (0xffffffff) /* <0:31> :R:0: */
#define CM0_BP_PID3_VALUE_POS                               (0)


/*
 * Breakpoint Unit CoreSight ROM Table Component ID #0
 */
#define CM0_BP_CID0_ADDRESS                                 (0xe0002ff0)
#define CM0_BP_CID0                                         (*(volatile uint32_t *)(0xe0002ff0))
#define CM0_BP_CID0_DEFAULT                                 (0x0000000d)

/*
 * Component ID #0
 */
#define CM0_BP_CID0_VALUE_MASK                              (0xffffffff) /* <0:31> :R:13: */
#define CM0_BP_CID0_VALUE_POS                               (0)


/*
 * Breakpoint Unit CoreSight ROM Table Component ID #1
 */
#define CM0_BP_CID1_ADDRESS                                 (0xe0002ff4)
#define CM0_BP_CID1                                         (*(volatile uint32_t *)(0xe0002ff4))
#define CM0_BP_CID1_DEFAULT                                 (0x000000e0)

/*
 * Component ID #1
 */
#define CM0_BP_CID1_VALUE_MASK                              (0xffffffff) /* <0:31> :R:224: */
#define CM0_BP_CID1_VALUE_POS                               (0)


/*
 * Breakpoint Unit CoreSight ROM Table Component ID #2
 */
#define CM0_BP_CID2_ADDRESS                                 (0xe0002ff8)
#define CM0_BP_CID2                                         (*(volatile uint32_t *)(0xe0002ff8))
#define CM0_BP_CID2_DEFAULT                                 (0x00000005)

/*
 * Component ID #2
 */
#define CM0_BP_CID2_VALUE_MASK                              (0xffffffff) /* <0:31> :R:5: */
#define CM0_BP_CID2_VALUE_POS                               (0)


/*
 * Breakpoint Unit CoreSight ROM Table Component ID #3
 */
#define CM0_BP_CID3_ADDRESS                                 (0xe0002ffc)
#define CM0_BP_CID3                                         (*(volatile uint32_t *)(0xe0002ffc))
#define CM0_BP_CID3_DEFAULT                                 (0x000000b1)

/*
 * Component ID #3
 */
#define CM0_BP_CID3_VALUE_MASK                              (0xffffffff) /* <0:31> :R:177: */
#define CM0_BP_CID3_VALUE_POS                               (0)


/*
 * Auxiliary Control Register
 * Provides configuration and control options.
 */
#define CM0_ACTLR_ADDRESS                                   (0xe000e008)
#define CM0_ACTLR                                           (*(volatile uint32_t *)(0xe000e008))
#define CM0_ACTLR_DEFAULT                                   (0x00000000)

/*
 * Always 0 for CM0r0p0
 */
#define CM0_ACTLR_ACTRL_MASK                                (0xffffffff) /* <0:31> :R:0: */
#define CM0_ACTLR_ACTRL_POS                                 (0)


/*
 * SysTick Control & Status
 * Controls the SysTick counter and provides status data. The SysTick counter
 * is functional in the Active and Sleep power modes (and not functional
 * in the DeepSleep, Hibernate and Stop power modes).
 */
#define CM0_SYST_CSR_ADDRESS                                (0xe000e010)
#define CM0_SYST_CSR                                        (*(volatile uint32_t *)(0xe000e010))
#define CM0_SYST_CSR_DEFAULT                                (0x00000000)

/*
 * Indicates the enabled status of the SysTick counter:
 * '0': counter is disabled.
 * '1': counter is operating.
 */
#define CM0_SYST_CSR_ENABLE                                 (1u << 0) /* <0:0> R:RW:0: */


/*
 * Indicates whether counting to "0" causes the status of the SysTick exception
 * to change to pending:
 * '0': count to "0" does not affect the SysTick exception status.
 * '1': count to "0" changes the SysTick exception status to pending.
 *
 * Changing the value of the counter to "0" by writing zero to the SYST_CVR
 * register to "0" never changes the status of the SysTick exception.
 */
#define CM0_SYST_CSR_TICKINT                                (1u << 1) /* <1:1> R:RW:0: */


/*
 * Indicates the SysTick counter clock source:
 * '0': SysTick uses the low frequency clock "clk_lf". For this mode to function,
 * "clk_lf" should be less than half the frequency of "clk_sys". Note that
 * "clk_lf" is generated by a low accuracy ILO (Internal Low power Oscillator),
 * with a target frequency of 32.768 kHz (frequency can be as low as 15 KHz
 * and as high as 60 kHz).
 * '1': SysTick uses the system/processor clock "clk_sys".
 *
 * In PSoC4A-BLE (and later products), SysTick counter functionality on the
 * low frequency clock is provided. in SF, TSG6M products, this functionality
 * is not provided. For these products, this field should be set to '1',
 * such that SysTick uses the system clock "clk_sys".
 */
#define CM0_SYST_CSR_CLKSOURCE                              (1u << 2) /* <2:2> R:RW:0: */


/*
 * Indicates whether the counter has counted to "0" since the last read of
 * this register:
 * '0': counter has not counted to "0".
 * '1': counter has counted to "0".
 *
 * COUNTFLAG is set to '1' by a count transition from "1" to "0".
 * COUNTFLAG is cleared to '0' by a read of this register, and by any write
 * to the SYST_CVR register.
 */
#define CM0_SYST_CSR_COUNTFLAG                              (1u << 16) /* <16:16> RW:R:0: */


/*
 * SysTick Reload Value
 * Sets or reads the reload value of the SYST_CVR register.
 */
#define CM0_SYST_RVR_ADDRESS                                (0xe000e014)
#define CM0_SYST_RVR                                        (*(volatile uint32_t *)(0xe000e014))
#define CM0_SYST_RVR_DEFAULT                                (0x00000000)

/*
 * The value to load into the SYST_CVR register when the counter reaches
 * 0.
 */
#define CM0_SYST_RVR_RELOAD_MASK                            (0x00ffffff) /* <0:23> R:RW:X: */
#define CM0_SYST_RVR_RELOAD_POS                             (0)


/*
 * SysTick Current Value
 * Reads or clears the current counter value.  Any write to the register
 * clears the register to 0.
 */
#define CM0_SYST_CVR_ADDRESS                                (0xe000e018)
#define CM0_SYST_CVR                                        (*(volatile uint32_t *)(0xe000e018))
#define CM0_SYST_CVR_DEFAULT                                (0x00000000)

/*
 * Current counter value.
 * This is the value of the counter at the time it is sampled.
 */
#define CM0_SYST_CVR_CURRENT_MASK                           (0x00ffffff) /* <0:23> R:RW:X: */
#define CM0_SYST_CVR_CURRENT_POS                            (0)


/*
 * SysTick Calibration Value
 * Reads the calibration value and parameters for SysTick.
 */
#define CM0_SYST_CALIB_ADDRESS                              (0xe000e01c)
#define CM0_SYST_CALIB                                      (*(volatile uint32_t *)(0xe000e01c))
#define CM0_SYST_CALIB_DEFAULT                              (0x00000000)

/*
 * Optionally, holds a reload value to be used for 10ms (100Hz) timing, subject
 * to system clock skew errors. If this field is "0", the calibration value
 * is not known.
 *
 * In PSoC4A-BLE (and later products), SysTick counter functionality on the
 * low frequency clock is provided and this field is 0x00:00147. In earlier
 * products, SysTick counter functionality on the low frequency clock is
 * NOT provided and this field is 0x00:0000.
 */
#define CM0_SYST_CALIB_TENMS_MASK                           (0x00ffffff) /* <0:23> RW:R:X: */
#define CM0_SYST_CALIB_TENMS_POS                            (0)


/*
 * Indicates whether the 10ms calibration value is exact:
 * '0': 10ms calibration value is exact.
 * '1': 10ms calibration value is inexact, because of the clock frequency.
 *
 * In PSoC4A-BLE (and later products), SysTick counter functionality on the
 * low frequency clock is provided and this field is '1' (due to the low
 * accuracy ILO). In earlier products, SysTick counter functionality on the
 * low frequency clock is NOT provided and this field is '0'.
 */
#define CM0_SYST_CALIB_SKEW                                 (1u << 30) /* <30:30> RW:R:X: */


/*
 * Indicates whether a implementation defined reference clock is provided:
 * '0': the reference clock is provided.
 * '1': the reference clock is not provided.
 * When this bit is '1', the SYST_CSR.CLKSOURCEis forced to '1' and cannot
 * be cleared to '0'.
 *
 * In PSoC4A-BLE (and later products), SysTick counter functionality  on
 * the low frequency clock is provided and this field is '0'. In earlier
 * products, SysTick counter functionality on the low frequency clock is
 * NOT provided and this field is '1'.
 */
#define CM0_SYST_CALIB_NOREF                                (1u << 31) /* <31:31> :R:0: */


/*
 * Interrupt Set-Enable Register
 * Enables, or reads the enabled state of one or more interrupts.
 */
#define CM0_ISER_ADDRESS                                    (0xe000e100)
#define CM0_ISER                                            (*(volatile uint32_t *)(0xe000e100))
#define CM0_ISER_DEFAULT                                    (0x00000000)

/*
 * Enables, or reads the enabled state of one or more interrupts. Each bit
 * corresponds to the same numbered interrupt.
 */
#define CM0_ISER_SETENA_MASK                                (0xffffffff) /* <0:31> R:RW1S:0: */
#define CM0_ISER_SETENA_POS                                 (0)


/*
 * Interrupt Clear Enable Register
 * Disables, or reads the enabled state of one or more interrupts
 */
#define CM0_ICER_ADDRESS                                    (0xe000e180)
#define CM0_ICER                                            (*(volatile uint32_t *)(0xe000e180))
#define CM0_ICER_DEFAULT                                    (0x00000000)

/*
 * Disables, or reads the enabled state of one or more interrupts. Each bit
 * corresponds to the same numbered interrupt.
 */
#define CM0_ICER_CLRENA_MASK                                (0xffffffff) /* <0:31> R:RW1C:0: */
#define CM0_ICER_CLRENA_POS                                 (0)


/*
 * Interrupt Set-Pending Register
 * On writes, sets the status of one or more interrupts to pending. On reads,
 * shows the
 * pending status of the interrupts.
 */
#define CM0_ISPR_ADDRESS                                    (0xe000e200)
#define CM0_ISPR                                            (*(volatile uint32_t *)(0xe000e200))
#define CM0_ISPR_DEFAULT                                    (0x00000000)

/*
 * Changes the state of one or more interrupts to pending. Each bit corresponds
 * to the same numbered interrupt.
 */
#define CM0_ISPR_SETPEND_MASK                               (0xffffffff) /* <0:31> R:RW1S:0: */
#define CM0_ISPR_SETPEND_POS                                (0)


/*
 * Interrupt Clear-Pending Register
 * On writes, clears the status of one or more interrupts to pending. On
 * reads, shows
 * the pending status of the interrupts.
 */
#define CM0_ICPR_ADDRESS                                    (0xe000e280)
#define CM0_ICPR                                            (*(volatile uint32_t *)(0xe000e280))
#define CM0_ICPR_DEFAULT                                    (0x00000000)

/*
 * Changes the state of one or more interrupts to not pending. Each bit corresponds
 * to the same numbered interrupt.
 */
#define CM0_ICPR_CLRPEND_MASK                               (0xffffffff) /* <0:31> R:RW1C:0: */
#define CM0_ICPR_CLRPEND_POS                                (0)


/*
 * Interrupt Priority Registers
 * Sets or reads interrupt priorities. Register n contains priorities for
 * interrupts N=4n .. 4n+3
 */
#define CM0_IPR_ADDRESS(n)                                  (0xe000e400 + ((n) * (0x0004)))
#define CM0_IPR(n)                                          (*(volatile uint32_t *)(0xe000e400 + ((n) * 0x0004)))
#define CM0_IPR_DEFAULT                                     (0x00000000)

/*
 * Priority of interrupt number N.
 */
#define CM0_IPR_PRI_N0_MASK                                 (0x000000c0) /* <6:7> R:RW:0: */
#define CM0_IPR_PRI_N0_POS                                  (6)


/*
 * Priority of interrupt number N+1.
 */
#define CM0_IPR_PRI_N1_MASK                                 (0x0000c000) /* <14:15> R:RW:0: */
#define CM0_IPR_PRI_N1_POS                                  (14)


/*
 * Priority of interrupt number N+2.
 */
#define CM0_IPR_PRI_N2_MASK                                 (0x00c00000) /* <22:23> R:RW:0: */
#define CM0_IPR_PRI_N2_POS                                  (22)


/*
 * Priority of interrupt number N+3.
 */
#define CM0_IPR_PRI_N3_MASK                                 (0xc0000000) /* <30:31> R:RW:0: */
#define CM0_IPR_PRI_N3_POS                                  (30)


/*
 * CPUID Register
 * Contains the part number, version, and implementation information that
 * is specific to this processor.
 */
#define CM0_CPUID_ADDRESS                                   (0xe000ed00)
#define CM0_CPUID                                           (*(volatile uint32_t *)(0xe000ed00))
#define CM0_CPUID_DEFAULT                                   (0x410cc200)

/*
 * Indicates revision. In ARM implementations this is the minor revision
 * number n in the pn part of the rnpn revision status, see Product revision
 * status on page xii. For release r0p0.
 */
#define CM0_CPUID_REVISION_MASK                             (0x0000000f) /* <0:3> :R:0: */
#define CM0_CPUID_REVISION_POS                              (0)


/*
 * Indicates part number, Cortex-M0
 */
#define CM0_CPUID_PARTNO_MASK                               (0x0000fff0) /* <4:15> :R:3104: */
#define CM0_CPUID_PARTNO_POS                                (4)


/*
 * Indicates the architecture, ARMv6-M
 */
#define CM0_CPUID_CONSTANT_MASK                             (0x000f0000) /* <16:19> :R:12: */
#define CM0_CPUID_CONSTANT_POS                              (16)


/*
 * Implementation defined. In ARM implementations this is the major revision
 * number n in the rn part of the rnpn revision status, Product revision
 * status on page xii.
 */
#define CM0_CPUID_VARIANT_MASK                              (0x00f00000) /* <20:23> :R:0: */
#define CM0_CPUID_VARIANT_POS                               (20)


/*
 * Implementer code for ARM.
 */
#define CM0_CPUID_IMPLEMENTER_MASK                          (0xff000000) /* <24:31> :R:65: */
#define CM0_CPUID_IMPLEMENTER_POS                           (24)


/*
 * Interrupt Control State Register
 * Controls and provides status information for the ARMv6-M.
 */
#define CM0_ICSR_ADDRESS                                    (0xe000ed04)
#define CM0_ICSR                                            (*(volatile uint32_t *)(0xe000ed04))
#define CM0_ICSR_DEFAULT                                    (0x00000000)

/*
 * The exception number for the current executing exception.  0= Thread mode.
 * This is the same value as IPSR[8:0]
 */
#define CM0_ICSR_VECTACTIVE_MASK                            (0x000001ff) /* <0:8> RW:R:0: */
#define CM0_ICSR_VECTACTIVE_POS                             (0)


/*
 * The exception number for the highest priority pending exception. 0= No
 * pending exceptions.
 * The pending state includes the effect of memory-mapped enable and mask
 * registers. It does not include the PRIMASK special-purpose register qualifier.
 */
#define CM0_ICSR_VECTPENDING_MASK                           (0x001ff000) /* <12:20> RW:R:0: */
#define CM0_ICSR_VECTPENDING_POS                            (12)


/*
 * Indicates if an external configurable, NVIC generated, interrupt is pending.
 */
#define CM0_ICSR_ISRPENDING                                 (1u << 22) /* <22:22> RW:R:0: */


/*
 * Indicates whether a pending exception will be serviced on exit from debug
 * halt state.
 */
#define CM0_ICSR_ISRPREEMPT                                 (1u << 23) /* <23:23> RW:R:0: */


/*
 * Clears a pending SysTick, whether set here or by the timer hardware.
 */
#define CM0_ICSR_PENDSTCLR                                  (1u << 25) /* <25:25> R:RW1C:0: */


/*
 * Sets a pending SysTick or reads back the current state. Writing PENDSTSET
 * and PENDSTCLR to '1' concurrently is UNPREDICTABLE.
 */
#define CM0_ICSR_PENDSTSETB                                 (1u << 26) /* <26:26> RW:RW1S:0: */


/*
 * Clears a pending PendSV interrupt.
 */
#define CM0_ICSR_PENDSVCLR                                  (1u << 27) /* <27:27> R:RW1C:0: */


/*
 * Sets a pending PendSV interrupt or reads back the current state.  Use
 * this normally to request a context switch.  Writing PENDSVSET and PENDSVCLR
 * to '1' concurrently is UNPREDICTABLE.
 */
#define CM0_ICSR_PENDSVSET                                  (1u << 28) /* <28:28> RW:RW1S:0: */


/*
 * Activates an NMI exception or reads back the current state.
 * Because NMI is the highest priority exception, it activates as soon as
 * it is registered.
 */
#define CM0_ICSR_NMIPENDSET                                 (1u << 31) /* <31:31> RW:RW1S:0: */


/*
 * Application Interrupt and Reset Control Register
 * Sets or returns interrupt control data.
 */
#define CM0_AIRCR_ADDRESS                                   (0xe000ed0c)
#define CM0_AIRCR                                           (*(volatile uint32_t *)(0xe000ed0c))
#define CM0_AIRCR_DEFAULT                                   (0x00000000)

/*
 * Clears all active state information for fixed and configurable
 * exceptions.  The effect of writing a 1 to this bit if the processor is
 * not halted in Debug state is UNPREDICTABLE.
 */
#define CM0_AIRCR_VECTCLRACTIVE                             (1u << 1) /* <1:1> R:RW1C:0: */


/*
 * System Reset Request.  Writing 1 to this bit asserts a signal to request
 * a reset by the external system. This will cause a full system reset of
 * the CPU and all other components in the device.  See Reset management
 * on page B1-240 for more information.
 */
#define CM0_AIRCR_SYSRESETREQ                               (1u << 2) /* <2:2> R:RW1S:0: */


/*
 * Indicates the memory system data endianness:
 * 0 little endian
 * 1 big endian.
 * See Endian support on page A3-44 for more information.
 */
#define CM0_AIRCR_ENDIANNESS                                (1u << 15) /* <15:15> :R:0: */


/*
 * Vector Key. The value 0x05FA must be written to this register, otherwise
 * the register write is UNPREDICTABLE.  Readback value is UNKNOWN.
 */
#define CM0_AIRCR_VECTKEY_MASK                              (0xffff0000) /* <16:31> R:RW:X: */
#define CM0_AIRCR_VECTKEY_POS                               (16)


/*
 * System Control Register
 * Sets or returns system control data.
 */
#define CM0_SCR_ADDRESS                                     (0xe000ed10)
#define CM0_SCR                                             (*(volatile uint32_t *)(0xe000ed10))
#define CM0_SCR_DEFAULT                                     (0x00000000)

/*
 * Determines whether, on an exit from an ISR that returns to the base level
 * of execution priority, the processor enters a sleep state:
 * 0 do not enter sleep state.
 * 1 enter sleep state.
 * See Power management on page B1-240 for more information.
 */
#define CM0_SCR_SLEEPONEXIT                                 (1u << 1) /* <1:1> R:RW:0: */


/*
 * An implementation can use this bit to select DeepSleep/Hibernate power
 * modes upon execution of WFI/WFE:
 * 0: Select Sleep mode
 * 1: Select DeepSleep/Hibernate (depends on PWR_CONTROL.HIBERNATE)
 */
#define CM0_SCR_SLEEPDEEP                                   (1u << 2) /* <2:2> R:RW:0: */


/*
 * Determines whether an interrupt transition from inactive state to pending
 * state is a wakeup event:
 * 0: transitions from inactive to pending are not wakeup events.
 * 1: transitions from inactive to pending are wakeup events.
 * See WFE on page A6-197 for more information.
 */
#define CM0_SCR_SEVONPEND                                   (1u << 4) /* <4:4> R:RW:0: */


/*
 * Configuration and Control Register
 * Returns configuration and control data.
 */
#define CM0_CCR_ADDRESS                                     (0xe000ed14)
#define CM0_CCR                                             (*(volatile uint32_t *)(0xe000ed14))
#define CM0_CCR_DEFAULT                                     (0x00000208)

/*
 * 1: unaligned word and halfword accesses generate a HardFault exception.
 */
#define CM0_CCR_UNALIGN_TRP                                 (1u << 3) /* <3:3> :R:1: */


/*
 * 1: On exception entry, the SP used prior to the exception is adjusted
 * to be 8-byte aligned and the context to restore it is saved. The SP is
 * restored on the associated exception return.
 */
#define CM0_CCR_STKALIGN                                    (1u << 9) /* <9:9> :R:1: */


/*
 * System Handler Priority Register 2
 * Sets or returns priority for system handler 11
 */
#define CM0_SHPR2_ADDRESS                                   (0xe000ed1c)
#define CM0_SHPR2                                           (*(volatile uint32_t *)(0xe000ed1c))
#define CM0_SHPR2_DEFAULT                                   (0x00000000)

/*
 * Priority of system handler 11, SVCall
 */
#define CM0_SHPR2_PRI_11_MASK                               (0xc0000000) /* <30:31> R:RW:0: */
#define CM0_SHPR2_PRI_11_POS                                (30)


/*
 * System Handler Priority Register 3
 * Sets or returns priority for system handlers 14-15.
 */
#define CM0_SHPR3_ADDRESS                                   (0xe000ed20)
#define CM0_SHPR3                                           (*(volatile uint32_t *)(0xe000ed20))
#define CM0_SHPR3_DEFAULT                                   (0x00000000)

/*
 * Priority of system handler 14, PendSV
 */
#define CM0_SHPR3_PRI_14_MASK                               (0x00c00000) /* <22:23> R:RW:0: */
#define CM0_SHPR3_PRI_14_POS                                (22)


/*
 * Priority of system handler 15, SysTick
 */
#define CM0_SHPR3_PRI_15_MASK                               (0xc0000000) /* <30:31> R:RW:0: */
#define CM0_SHPR3_PRI_15_POS                                (30)


/*
 * System Handler Control and State Register
 * Controls and provides the status of system handlers.
 */
#define CM0_SHCSR_ADDRESS                                   (0xe000ed24)
#define CM0_SHCSR                                           (*(volatile uint32_t *)(0xe000ed24))
#define CM0_SHCSR_DEFAULT                                   (0x00000000)

/*
 * 0 SVCall is not pending.
 * 1 SVCall is pending.
 * This bit reflects the pending state on a read, and updates the pending
 * state, to the value written, on a write. (Pending state bits are set to
 * 1 when an exception occurs, and are cleared to 0 when an exception becomes
 * active.)
 */
#define CM0_SHCSR_SVCALLPENDED                              (1u << 15) /* <15:15> RW:RW:0: */


/*
 * Debug Fault Status Register
 * Provides the top level reason why a debug event has occurred.  Writing
 * 1 to a register bit clears that bit to 0. A read of the HALTED bit by
 * an instruction executed by stepping returns an UNKNOWN value, For more
 * information see Debug stepping on page C1-325.
 */
#define CM0_DFSR_ADDRESS                                    (0xe000ed30)
#define CM0_DFSR                                            (*(volatile uint32_t *)(0xe000ed30))
#define CM0_DFSR_DEFAULT                                    (0x00000000)

/*
 * Indicates a debug event generated by a C_HALT or C_STEP request, triggered
 * by a write to the DHCSR:
 * 0 no active halt request debug event.
 * 1 halt request debug event active.
 * See Debug Halting Control and Status Register, DHCSR for more information.
 */
#define CM0_DFSR_HALTED                                     (1u << 0) /* <0:0> RW1S:RW1C:0: */


/*
 * Indicates a debug event generated by BKPT instruction execution or a breakpoint
 * match in the BPU:
 * 0 no breakpoint debug event.
 * 1 at least one breakpoint debug event.
 */
#define CM0_DFSR_BKPT                                       (1u << 1) /* <1:1> RW1S:RW1C:0: */


/*
 * Indicates a debug event generated by the DWT:
 * 0 no debug events generated by the DWT.
 * 1 at least one debug event generated by the DWT.
 */
#define CM0_DFSR_DWTTRAP                                    (1u << 2) /* <2:2> RW1S:RW1C:0: */


/*
 * Indicates whether a vector catch debug event was generated:
 * 0 no vector catch debug event generated.
 * 1 vector catch debug event generated.
 * The corresponding FSR shows the primary cause of the exception.
 */
#define CM0_DFSR_VCATCH                                     (1u << 3) /* <3:3> RW1S:RW1C:0: */


/*
 * Indicates an asynchronous debug event generated because of EDBGRQ being
 * asserted:
 * 0 no EDBGRQ debug event.
 * 1 EDBGRQ debug event.
 */
#define CM0_DFSR_EXTERNAL                                   (1u << 4) /* <4:4> RW1S:RW1C:0: */


/*
 * Debug Halting Control and Status Register
 * Controls halting debug.  When C_DEBUGEN is set to 1, C_STEP and C_MASKINTS
 * must not be
 * modified when the processor is running.  Note: S_HALT is 0 when the processor
 * is running. When C_DEBUGEN is set to 0, the processor ignores the values
 * of all other bits in this register. For more information on the use of
 * DHCSR, see Debug stepping on page C1-325.
 * Note: any write to this register must have [31:16]=0xA05F - if not, the
 * write is ignored.
 */
#define CM0_DHCSR_ADDRESS                                   (0xe000edf0)
#define CM0_DHCSR                                           (*(volatile uint32_t *)(0xe000edf0))
#define CM0_DHCSR_DEFAULT                                   (0x02000000)

/*
 * Halting debug enable bit.  If a debugger writes to DHCSR to change the
 * value of this bit from 0 to 1, it must also write 0 to the C_MASKINTS
 * bit, otherwise behavior is UNPREDICTABLE.  This bit can only be written
 * from the DAP. Access to the DHCSR from
 * software running on the processor is IMPLEMENTATION DEFINED.  However,
 * writes to this bit from software running on the processor are ignored.
 */
#define CM0_DHCSR_C_DEBUGEN                                 (1u << 0) /* <0:0> R:RW:0: */


/*
 * Processor halt bit. The effects of writes to this bit are:
 * 0 Request a halted processor to run.
 * 1 Request a running processor to halt.
 * Table C1-7 on page C1-326 shows the effect of writes to this bit when
 * the processor is in Debug state.
 */
#define CM0_DHCSR_C_HALT                                    (1u << 1) /* <1:1> R:RW:X: */


/*
 * Processor step bit. The effects of writes to this bit are:
 * 0 Single-stepping disabled.
 * 1 Single-stepping enabled.
 * For more information about the use of this bit see Table C1-7 on page
 * C1-326.
 */
#define CM0_DHCSR_C_STEP                                    (1u << 2) /* <2:2> R:RW:X: */


/*
 * When debug is enabled, the debugger can write to this bit to mask PendSV,
 * SysTick and external configurable interrupts. The effect of any attempt
 * to change the value of this bit is UNPREDICTABLE unless both:
 * - before the write to DHCSR, the value of the C_HALT bit is 1
 * - the write to the DHCSR that changes the C_MASKINTS bit also writes 1
 * to the C_HALT bit
 * This means that a single write to DHCSR cannot set the C_HALT to 0 and
 * change the value of the C_MASKINTS bit. The bit does not affect NMI. When
 * DHCSR.C_DEBUGEN is set to 0, the
 * value of this bit is UNKNOWN. For more information about the use of this
 * bit see Table C1-7 on page C1-326.
 */
#define CM0_DHCSR_C_MASKINTS                                (1u << 3) /* <3:3> R:RW:X: */


/*
 * A handshake flag for transfers through the DCRDR:
 * - Writing to DCRSR clears the bit to 0.
 * - Completion of the DCRDR transfer then sets the bit to 1.
 * For more information about DCRDR transfers see Debug Core Register Data
 * Register, DCRDR on page C1-337.
 * 0: There has been a write to the DCRDR, but the transfer is
 * not complete.
 * 1: The transfer to or from the DCRDR is complete.
 * This bit is only valid when the processor is in Debug state, otherwise
 * the bit is UNKNOWN.
 */
#define CM0_DHCSR_S_REGRDY                                  (1u << 16) /* <16:16> RW:R:X: */


/*
 * Indicates whether the processor is in Debug state.
 */
#define CM0_DHCSR_S_HALT                                    (1u << 17) /* <17:17> RW:R:0: */


/*
 * Indicates whether the processor is sleeping.  The debugger must set the
 * DHCSR.C_HALT bit to 1 to gain control, or
 * wait for an interrupt or other wakeup event to wakeup the system.
 */
#define CM0_DHCSR_S_SLEEP                                   (1u << 18) /* <18:18> RW:R:0: */


/*
 * Indicates whether the processor is locked up because of an unrecoverable
 * exception.  See Unrecoverable exception cases on page B1-238 for more
 * information.  This bit can only read as 1 when accessed by a remote debugger
 * using the
 * DAP.  The bit clears to 0 when the processor enters Debug state.
 */
#define CM0_DHCSR_S_LOCKUP                                  (1u << 19) /* <19:19> RW:R:0: */


/*
 * Debug key:
 * Software must write 0xA05F to [31:16] to enable write accesses to bits
 * [15:0], otherwise the processor ignores the write access.
 */
#define CM0_DHCSR_DBG_KEY_P1_MASK                           (0x00f00000) /* <20:23> R:W:X: */
#define CM0_DHCSR_DBG_KEY_P1_POS                            (20)


/*
 * When not in Debug state, indicates whether the processor has completed
 * execution of at least one instruction since the last read of DHCSR. This
 * is a sticky bit, that clears to 0 on a read of DHCSR.
 * This bit is UNKNOWN:
 * - after a Local reset, but is set to 1 as soon as the processor completes
 * execution of an instruction
 * - when S_LOCKUP is set to 1
 * - when S_HALT is set to 1.
 * When the processor is not in Debug state, a debugger can check this bit
 * to determine if the processor is stalled on a load, store or fetch access.
 */
#define CM0_DHCSR_S_RETIRE_ST                               (1u << 24) /* <24:24> RW:R:X: */


/*
 * Indicates whether the processor has been reset since the last read of
 * DHCSR. This is a sticky bit, that clears to 0 on a read of DHCSR
 */
#define CM0_DHCSR_S_RESET_ST                                (1u << 25) /* <25:25> RW:R:1: */


/*
 * See desciption for Bit 20
 */
#define CM0_DHCSR_DBG_KEY_P2_MASK                           (0xfc000000) /* <26:31> R:W:X: */
#define CM0_DHCSR_DBG_KEY_P2_POS                            (26)


/*
 * Debug Core Register Selector Register
 * With the DCRDR, see Debug Core Register Data Register, DCRDR on page C1-337,
 * the DCRSR provides debug access to the ARM core registers and special-purpose
 * registers. A write to DCRSR specifies the register to transfer, whether
 * the transfer is a read or a write, and starts the transfer.  This register
 * is only accessible in Debug state.  When the processor is in Debug state,
 * the debugger must preserve the Exception number bits in the IPSR, otherwise
 * behavior is UNPREDICTABLE
 */
#define CM0_DCRSR_ADDRESS                                   (0xe000edf4)
#define CM0_DCRSR                                           (*(volatile uint32_t *)(0xe000edf4))
#define CM0_DCRSR_DEFAULT                                   (0x00000000)

/*
 * Specifies the ARM core register or special-purpose register to transfer.
 */
#define CM0_DCRSR_REGSEL_MASK                               (0x0000001f) /* <0:4> R:RW:X: */
#define CM0_DCRSR_REGSEL_POS                                (0)


/*
 * Specifies the type of access for the transfer.
 */
#define CM0_DCRSR_REGWNR                                    (1u << 16) /* <16:16> R:RW:X: */


/*
 * Debug Core Register Data Register
 * With the DCRSR, see Debug Core Register Selector Register, DCRSR on page
 * C1-335, the DCRDR provides debug access to the ARM core registers and
 * special-purpose registers. The DCRDR is the data register for these accesses.
 */
#define CM0_DCRDR_ADDRESS                                   (0xe000edf8)
#define CM0_DCRDR                                           (*(volatile uint32_t *)(0xe000edf8))
#define CM0_DCRDR_DEFAULT                                   (0x00000000)

/*
 * Data temporary cache, for reading and writing CPU registers.
 * This register is UNKNOWN:
 * - on reset
 * - when DHCSR.S_HALT = 0.
 * - when DHCSR.S_REGRDY = 0 during execution of a DCRSR based transaction
 * that updates the register
 */
#define CM0_DCRDR_DBGTMP_MASK                               (0xffffffff) /* <0:31> RW:RW:X: */
#define CM0_DCRDR_DBGTMP_POS                                (0)


/*
 * Debug Exception and Monitor Control Register
 * Manages vector catch behavior and enables the DWT.
 */
#define CM0_DEMCR_ADDRESS                                   (0xe000edfc)
#define CM0_DEMCR                                           (*(volatile uint32_t *)(0xe000edfc))
#define CM0_DEMCR_DEFAULT                                   (0x00000000)

/*
 * Enable Reset Vector Catch. This causes a Local reset to halt a running
 * system. If DHCSR.C_DEBUGEN is set to 0, the processor ignores the value
 * of this bit.
 */
#define CM0_DEMCR_VC_CORERESET                              (1u << 0) /* <0:0> R:RW:0: */


/*
 * Enable halting debug trap on a HardFault exception. If DHCSR.C_DEBUGEN
 * is set to 0, the processor ignores the value of this bit.
 */
#define CM0_DEMCR_VC_HARDERR                                (1u << 10) /* <10:10> R:RW:0: */


/*
 * Global enable for all features configured and controlled by the DWT unit.
 *  When DWTENA is set to 0 DWT registers return UNKNOWN values on reads.
 * In addition, it is IMPLEMENTATION DEFINED whether the processor ignores
 * writes to the DWT while DWTENA is 0.
 */
#define CM0_DEMCR_DWTENA                                    (1u << 24) /* <24:24> R:RW:0: */


/*
 * System Control Space ROM Table Peripheral ID #4
 */
#define CM0_SCS_PID4_ADDRESS                                (0xe000efd0)
#define CM0_SCS_PID4                                        (*(volatile uint32_t *)(0xe000efd0))
#define CM0_SCS_PID4_DEFAULT                                (0x00000004)

/*
 * Peripheral ID #4
 */
#define CM0_SCS_PID4_VALUE_MASK                             (0xffffffff) /* <0:31> :R:4: */
#define CM0_SCS_PID4_VALUE_POS                              (0)


/*
 * System Control Space ROM Table Peripheral ID #0
 */
#define CM0_SCS_PID0_ADDRESS                                (0xe000efe0)
#define CM0_SCS_PID0                                        (*(volatile uint32_t *)(0xe000efe0))
#define CM0_SCS_PID0_DEFAULT                                (0x00000008)

/*
 * Peripheral ID #0
 */
#define CM0_SCS_PID0_VALUE_MASK                             (0xffffffff) /* <0:31> :R:8: */
#define CM0_SCS_PID0_VALUE_POS                              (0)


/*
 * System Control Space ROM Table Peripheral ID #1
 */
#define CM0_SCS_PID1_ADDRESS                                (0xe000efe4)
#define CM0_SCS_PID1                                        (*(volatile uint32_t *)(0xe000efe4))
#define CM0_SCS_PID1_DEFAULT                                (0x000000b0)

/*
 * Peripheral ID #1
 */
#define CM0_SCS_PID1_VALUE_MASK                             (0xffffffff) /* <0:31> :R:176: */
#define CM0_SCS_PID1_VALUE_POS                              (0)


/*
 * System Control Space ROM Table Peripheral ID #2
 */
#define CM0_SCS_PID2_ADDRESS                                (0xe000efe8)
#define CM0_SCS_PID2                                        (*(volatile uint32_t *)(0xe000efe8))
#define CM0_SCS_PID2_DEFAULT                                (0x0000000b)

/*
 * Peripheral ID #2
 */
#define CM0_SCS_PID2_VALUE_MASK                             (0xffffffff) /* <0:31> :R:11: */
#define CM0_SCS_PID2_VALUE_POS                              (0)


/*
 * System Control Space ROM Table Peripheral ID #3
 */
#define CM0_SCS_PID3_ADDRESS                                (0xe000efec)
#define CM0_SCS_PID3                                        (*(volatile uint32_t *)(0xe000efec))
#define CM0_SCS_PID3_DEFAULT                                (0x00000000)

/*
 * Peripheral ID #3
 */
#define CM0_SCS_PID3_VALUE_MASK                             (0xffffffff) /* <0:31> :R:0: */
#define CM0_SCS_PID3_VALUE_POS                              (0)


/*
 * System Control Space ROM Table Component ID #0
 */
#define CM0_SCS_CID0_ADDRESS                                (0xe000eff0)
#define CM0_SCS_CID0                                        (*(volatile uint32_t *)(0xe000eff0))
#define CM0_SCS_CID0_DEFAULT                                (0x0000000d)

/*
 * Component ID #0
 */
#define CM0_SCS_CID0_VALUE_MASK                             (0xffffffff) /* <0:31> :R:13: */
#define CM0_SCS_CID0_VALUE_POS                              (0)


/*
 * System Control Space ROM Table Component ID #1
 */
#define CM0_SCS_CID1_ADDRESS                                (0xe000eff4)
#define CM0_SCS_CID1                                        (*(volatile uint32_t *)(0xe000eff4))
#define CM0_SCS_CID1_DEFAULT                                (0x000000e0)

/*
 * Component ID #1
 */
#define CM0_SCS_CID1_VALUE_MASK                             (0xffffffff) /* <0:31> :R:224: */
#define CM0_SCS_CID1_VALUE_POS                              (0)


/*
 * System Control Space ROM Table Component ID #2
 */
#define CM0_SCS_CID2_ADDRESS                                (0xe000eff8)
#define CM0_SCS_CID2                                        (*(volatile uint32_t *)(0xe000eff8))
#define CM0_SCS_CID2_DEFAULT                                (0x00000005)

/*
 * Component ID #2
 */
#define CM0_SCS_CID2_VALUE_MASK                             (0xffffffff) /* <0:31> :R:5: */
#define CM0_SCS_CID2_VALUE_POS                              (0)


/*
 * System Control Space ROM Table Component ID #3
 */
#define CM0_SCS_CID3_ADDRESS                                (0xe000effc)
#define CM0_SCS_CID3                                        (*(volatile uint32_t *)(0xe000effc))
#define CM0_SCS_CID3_DEFAULT                                (0x000000b1)

/*
 * Component ID #3
 */
#define CM0_SCS_CID3_VALUE_MASK                             (0xffffffff) /* <0:31> :R:177: */
#define CM0_SCS_CID3_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Peripheral #0
 */
#define CM0_ROM_SCS_ADDRESS                                 (0xe00ff000)
#define CM0_ROM_SCS                                         (*(volatile uint32_t *)(0xe00ff000))
#define CM0_ROM_SCS_DEFAULT                                 (0xfff0f003)

/*
 * Offset to SCS ROM Table
 */
#define CM0_ROM_SCS_VALUE_MASK                              (0xffffffff) /* <0:31> :R:4293980163: */
#define CM0_ROM_SCS_VALUE_POS                               (0)


/*
 * CM0 CoreSight ROM Table Peripheral #1
 */
#define CM0_ROM_DWT_ADDRESS                                 (0xe00ff004)
#define CM0_ROM_DWT                                         (*(volatile uint32_t *)(0xe00ff004))
#define CM0_ROM_DWT_DEFAULT                                 (0xfff02003)

/*
 * Offset to DWT ROM Table
 */
#define CM0_ROM_DWT_VALUE_MASK                              (0xffffffff) /* <0:31> :R:4293926915: */
#define CM0_ROM_DWT_VALUE_POS                               (0)


/*
 * CM0 CoreSight ROM Table Peripheral #2
 */
#define CM0_ROM_BPU_ADDRESS                                 (0xe00ff008)
#define CM0_ROM_BPU                                         (*(volatile uint32_t *)(0xe00ff008))
#define CM0_ROM_BPU_DEFAULT                                 (0xfff03003)

/*
 * Offset to BPU ROM Table
 */
#define CM0_ROM_BPU_VALUE_MASK                              (0xffffffff) /* <0:31> :R:4293931011: */
#define CM0_ROM_BPU_VALUE_POS                               (0)


/*
 * CM0 CoreSight ROM Table End Marker
 */
#define CM0_ROM_END_ADDRESS                                 (0xe00ff00c)
#define CM0_ROM_END                                         (*(volatile uint32_t *)(0xe00ff00c))
#define CM0_ROM_END_DEFAULT                                 (0x00000000)

/*
 * End marker in peripheral list
 */
#define CM0_ROM_END_VALUE_MASK                              (0xffffffff) /* <0:31> :R:0: */
#define CM0_ROM_END_VALUE_POS                               (0)


/*
 * CM0 CoreSight ROM Table Memory Type
 */
#define CM0_ROM_CSMT_ADDRESS                                (0xe00fffcc)
#define CM0_ROM_CSMT                                        (*(volatile uint32_t *)(0xe00fffcc))
#define CM0_ROM_CSMT_DEFAULT                                (0x00000001)

/*
 * Memory Type
 */
#define CM0_ROM_CSMT_VALUE_MASK                             (0xffffffff) /* <0:31> :R:1: */
#define CM0_ROM_CSMT_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Peripheral ID #4
 */
#define CM0_ROM_PID4_ADDRESS                                (0xe00fffd0)
#define CM0_ROM_PID4                                        (*(volatile uint32_t *)(0xe00fffd0))
#define CM0_ROM_PID4_DEFAULT                                (0x00000004)

/*
 * Peripheral ID #4
 */
#define CM0_ROM_PID4_VALUE_MASK                             (0xffffffff) /* <0:31> :R:4: */
#define CM0_ROM_PID4_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Peripheral ID #0
 */
#define CM0_ROM_PID0_ADDRESS                                (0xe00fffe0)
#define CM0_ROM_PID0                                        (*(volatile uint32_t *)(0xe00fffe0))
#define CM0_ROM_PID0_DEFAULT                                (0x00000071)

/*
 * Peripheral ID #0
 */
#define CM0_ROM_PID0_VALUE_MASK                             (0xffffffff) /* <0:31> :R:113: */
#define CM0_ROM_PID0_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Peripheral ID #1
 */
#define CM0_ROM_PID1_ADDRESS                                (0xe00fffe4)
#define CM0_ROM_PID1                                        (*(volatile uint32_t *)(0xe00fffe4))
#define CM0_ROM_PID1_DEFAULT                                (0x000000b4)

/*
 * Peripheral ID #1
 */
#define CM0_ROM_PID1_VALUE_MASK                             (0xffffffff) /* <0:31> :R:180: */
#define CM0_ROM_PID1_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Peripheral ID #2
 */
#define CM0_ROM_PID2_ADDRESS                                (0xe00fffe8)
#define CM0_ROM_PID2                                        (*(volatile uint32_t *)(0xe00fffe8))
#define CM0_ROM_PID2_DEFAULT                                (0x0000000b)

/*
 * Peripheral ID #2
 */
#define CM0_ROM_PID2_VALUE_MASK                             (0xffffffff) /* <0:31> :R:11: */
#define CM0_ROM_PID2_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Peripheral ID #3
 */
#define CM0_ROM_PID3_ADDRESS                                (0xe00fffec)
#define CM0_ROM_PID3                                        (*(volatile uint32_t *)(0xe00fffec))
#define CM0_ROM_PID3_DEFAULT                                (0x00000000)

/*
 * Peripheral ID #3
 */
#define CM0_ROM_PID3_VALUE_MASK                             (0xffffffff) /* <0:31> :R:0: */
#define CM0_ROM_PID3_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Component ID #0
 */
#define CM0_ROM_CID0_ADDRESS                                (0xe00ffff0)
#define CM0_ROM_CID0                                        (*(volatile uint32_t *)(0xe00ffff0))
#define CM0_ROM_CID0_DEFAULT                                (0x0000000d)

/*
 * Component ID #0
 */
#define CM0_ROM_CID0_VALUE_MASK                             (0xffffffff) /* <0:31> :R:13: */
#define CM0_ROM_CID0_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Component ID #1
 */
#define CM0_ROM_CID1_ADDRESS                                (0xe00ffff4)
#define CM0_ROM_CID1                                        (*(volatile uint32_t *)(0xe00ffff4))
#define CM0_ROM_CID1_DEFAULT                                (0x00000010)

/*
 * Component ID #1
 */
#define CM0_ROM_CID1_VALUE_MASK                             (0xffffffff) /* <0:31> :R:16: */
#define CM0_ROM_CID1_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Component ID #2
 */
#define CM0_ROM_CID2_ADDRESS                                (0xe00ffff8)
#define CM0_ROM_CID2                                        (*(volatile uint32_t *)(0xe00ffff8))
#define CM0_ROM_CID2_DEFAULT                                (0x00000005)

/*
 * Component ID #2
 */
#define CM0_ROM_CID2_VALUE_MASK                             (0xffffffff) /* <0:31> :R:5: */
#define CM0_ROM_CID2_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Component ID #3
 */
#define CM0_ROM_CID3_ADDRESS                                (0xe00ffffc)
#define CM0_ROM_CID3                                        (*(volatile uint32_t *)(0xe00ffffc))
#define CM0_ROM_CID3_DEFAULT                                (0x000000b1)

/*
 * Component ID #3
 */
#define CM0_ROM_CID3_VALUE_MASK                             (0xffffffff) /* <0:31> :R:177: */
#define CM0_ROM_CID3_VALUE_POS                              (0)


/*
 * Link to Cortex M0 ROM Table.
 */
#define ROMTABLE_ADDR_ADDRESS                               (0xf0000000)
#define ROMTABLE_ADDR                                       (*(volatile uint32_t *)(0xf0000000))
#define ROMTABLE_ADDR_DEFAULT                               (0xf00ff003)

/*
 * Entry present.
 */
#define ROMTABLE_ADDR_PRESENT                               (1u << 0) /* <0:0> R:R:1: */


/*
 * ROM Table format:
 * '0: 8-bit format.
 * '1': 32-bit format.
 */
#define ROMTABLE_ADDR_FORMAT_32BIT                          (1u << 1) /* <1:1> R:R:1: */


/*
 * Address offset of the Cortex-M0 ROM Table base address (0xe00f:f000) wrt.
 * Cypress chip specific ROM Table base address (0xf000:0000). ADDR_OFFSET[19:0]
 * = 0xe00f:f - 0xf000:0 = 0xf00f:f.
 */
#define ROMTABLE_ADDR_ADDR_OFFSET_MASK                      (0xfffff000) /* <12:31> R:R:983295: */
#define ROMTABLE_ADDR_ADDR_OFFSET_POS                       (12)


/*
 * Device Type Identifier register.
 */
#define ROMTABLE_DID_ADDRESS                                (0xf0000fcc)
#define ROMTABLE_DID                                        (*(volatile uint32_t *)(0xf0000fcc))
#define ROMTABLE_DID_DEFAULT                                (0x00000001)

/*
 * .
 */
#define ROMTABLE_DID_VALUE_MASK                             (0xffffffff) /* <0:31> R:R:1: */
#define ROMTABLE_DID_VALUE_POS                              (0)


/*
 * Peripheral Identification Register 4.
 */
#define ROMTABLE_PID4_ADDRESS                               (0xf0000fd0)
#define ROMTABLE_PID4                                       (*(volatile uint32_t *)(0xf0000fd0))
#define ROMTABLE_PID4_DEFAULT                               (0x00000000)

/*
 * JEP106 continuation code.  This value is product specific and specified
 * as part of the product definition in the CPUSS.JEPCONTINUATION parameter.
 */
#define ROMTABLE_PID4_JEP_CONTINUATION_MASK                 (0x0000000f) /* <0:3> R:R:Undefined: */
#define ROMTABLE_PID4_JEP_CONTINUATION_POS                  (0)


/*
 * Size of ROM Table is 2^COUNT * 4 KByte.
 */
#define ROMTABLE_PID4_COUNT_MASK                            (0x000000f0) /* <4:7> R:R:0: */
#define ROMTABLE_PID4_COUNT_POS                             (4)


/*
 * Peripheral Identification Register 5.
 */
#define ROMTABLE_PID5_ADDRESS                               (0xf0000fd4)
#define ROMTABLE_PID5                                       (*(volatile uint32_t *)(0xf0000fd4))
#define ROMTABLE_PID5_DEFAULT                               (0x00000000)

/*
 * .
 */
#define ROMTABLE_PID5_VALUE_MASK                            (0xffffffff) /* <0:31> R:R:0: */
#define ROMTABLE_PID5_VALUE_POS                             (0)


/*
 * Peripheral Identification Register 6.
 */
#define ROMTABLE_PID6_ADDRESS                               (0xf0000fd8)
#define ROMTABLE_PID6                                       (*(volatile uint32_t *)(0xf0000fd8))
#define ROMTABLE_PID6_DEFAULT                               (0x00000000)

/*
 * .
 */
#define ROMTABLE_PID6_VALUE_MASK                            (0xffffffff) /* <0:31> R:R:0: */
#define ROMTABLE_PID6_VALUE_POS                             (0)


/*
 * Peripheral Identification Register 7.
 */
#define ROMTABLE_PID7_ADDRESS                               (0xf0000fdc)
#define ROMTABLE_PID7                                       (*(volatile uint32_t *)(0xf0000fdc))
#define ROMTABLE_PID7_DEFAULT                               (0x00000000)

/*
 * .
 */
#define ROMTABLE_PID7_VALUE_MASK                            (0xffffffff) /* <0:31> R:R:0: */
#define ROMTABLE_PID7_VALUE_POS                             (0)


/*
 * Peripheral Identification Register 0.
 */
#define ROMTABLE_PID0_ADDRESS                               (0xf0000fe0)
#define ROMTABLE_PID0                                       (*(volatile uint32_t *)(0xf0000fe0))
#define ROMTABLE_PID0_DEFAULT                               (0x00000000)

/*
 * JEP106 part number. 4 lsbs of CPUSS.PARTNUMBER parameter.  These part
 * numbers are maintained in spec 40-9500.
 */
#define ROMTABLE_PID0_PN_MIN_MASK                           (0x000000ff) /* <0:7> R:R:Undefined: */
#define ROMTABLE_PID0_PN_MIN_POS                            (0)


/*
 * Peripheral Identification Register 1.
 */
#define ROMTABLE_PID1_ADDRESS                               (0xf0000fe4)
#define ROMTABLE_PID1                                       (*(volatile uint32_t *)(0xf0000fe4))
#define ROMTABLE_PID1_DEFAULT                               (0x00000000)

/*
 * JEP106 part number. 4 msbs of CPUSS.PARTNUMBER parameter.  These part
 * numbers are maintained in spec 40-9500.
 */
#define ROMTABLE_PID1_PN_MAJ_MASK                           (0x0000000f) /* <0:3> R:R:Undefined: */
#define ROMTABLE_PID1_PN_MAJ_POS                            (0)


/*
 * JEP106 vendor id. 4 lsbs of CPUSS.JEPID parameter.  This number is maintained
 * in spec 40-9500.
 */
#define ROMTABLE_PID1_JEPID_MIN_MASK                        (0x000000f0) /* <4:7> R:R:Undefined: */
#define ROMTABLE_PID1_JEPID_MIN_POS                         (4)


/*
 * Peripheral Identification Register 2.
 */
#define ROMTABLE_PID2_ADDRESS                               (0xf0000fe8)
#define ROMTABLE_PID2                                       (*(volatile uint32_t *)(0xf0000fe8))
#define ROMTABLE_PID2_DEFAULT                               (0x00000000)

/*
 * JEP106 vendor id. 4 msbs of CPUSS.JEPID parameter.  This number is maintained
 * in spec 40-9500.
 */
#define ROMTABLE_PID2_JEPID_MAJ_MASK                        (0x00000007) /* <0:2> R:R:Undefined: */
#define ROMTABLE_PID2_JEPID_MAJ_POS                         (0)


/*
 * Major REVision number (chip specific). Identifies the design iteration
 * of the component. For first tape out: 0x1. This field is implemented in
 * RTL by an ECO-able tie-off structure and is incremented on subsequent
 * tape outs.
 */
#define ROMTABLE_PID2_REV_MASK                              (0x000000f0) /* <4:7> R:R:Undefined: */
#define ROMTABLE_PID2_REV_POS                               (4)


/*
 * Peripheral Identification Register 3.
 */
#define ROMTABLE_PID3_ADDRESS                               (0xf0000fec)
#define ROMTABLE_PID3                                       (*(volatile uint32_t *)(0xf0000fec))
#define ROMTABLE_PID3_DEFAULT                               (0x00000000)

/*
 * Customer modified field. This field is used to track modifications to
 * the original component design as a result of componenet IP reuse.
 */
#define ROMTABLE_PID3_CM_MASK                               (0x0000000f) /* <0:3> R:R:0: */
#define ROMTABLE_PID3_CM_POS                                (0)


/*
 * Minor REVision number (chip specific). For first tape out: 0x1. This field
 * is implemented in RTL by an ECO-able tie-off structure and is incremented
 * on subsequent tape outs.
 */
#define ROMTABLE_PID3_REV_AND_MASK                          (0x000000f0) /* <4:7> R:R:Undefined: */
#define ROMTABLE_PID3_REV_AND_POS                           (4)


/*
 * Component Identification Register 0.
 */
#define ROMTABLE_CID0_ADDRESS                               (0xf0000ff0)
#define ROMTABLE_CID0                                       (*(volatile uint32_t *)(0xf0000ff0))
#define ROMTABLE_CID0_DEFAULT                               (0x0000000d)

/*
 * Component identification byte 0 of 4-byte component identification 0xB105:100D.
 */
#define ROMTABLE_CID0_VALUE_MASK                            (0xffffffff) /* <0:31> R:R:13: */
#define ROMTABLE_CID0_VALUE_POS                             (0)


/*
 * Component Identification Register 1.
 */
#define ROMTABLE_CID1_ADDRESS                               (0xf0000ff4)
#define ROMTABLE_CID1                                       (*(volatile uint32_t *)(0xf0000ff4))
#define ROMTABLE_CID1_DEFAULT                               (0x00000010)

/*
 * Component identification byte 1 of 4-byte component identification 0xB105:100D.
 * Component class: "ROM Table".
 */
#define ROMTABLE_CID1_VALUE_MASK                            (0xffffffff) /* <0:31> R:R:16: */
#define ROMTABLE_CID1_VALUE_POS                             (0)


/*
 * Component Identification Register 2.
 */
#define ROMTABLE_CID2_ADDRESS                               (0xf0000ff8)
#define ROMTABLE_CID2                                       (*(volatile uint32_t *)(0xf0000ff8))
#define ROMTABLE_CID2_DEFAULT                               (0x00000005)

/*
 * Component identification byte 2 of 4-byte component identification 0xB105:100D.
 */
#define ROMTABLE_CID2_VALUE_MASK                            (0xffffffff) /* <0:31> R:R:5: */
#define ROMTABLE_CID2_VALUE_POS                             (0)


/*
 * Component Identification Register 3.
 */
#define ROMTABLE_CID3_ADDRESS                               (0xf0000ffc)
#define ROMTABLE_CID3                                       (*(volatile uint32_t *)(0xf0000ffc))
#define ROMTABLE_CID3_DEFAULT                               (0x000000b1)

/*
 * Component identification byte 3 of 4-byte component identification 0xB105:100D.
 */
#define ROMTABLE_CID3_VALUE_MASK                            (0xffffffff) /* <0:31> R:R:177: */
#define ROMTABLE_CID3_VALUE_POS                             (0)



#endif /* _PAG1S_REGS_H_ */

