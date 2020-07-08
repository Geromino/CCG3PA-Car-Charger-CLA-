/*******************************************************************************
* File Name: cyfitter.h
* 
* PSoC Creator  4.3
*
* Description:
* 
* This file is automatically generated by PSoC Creator.
*
********************************************************************************
* Copyright (c) 2007-2019 Cypress Semiconductor.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

#ifndef INCLUDED_CYFITTER_H
#define INCLUDED_CYFITTER_H
#include "cydevice_trm.h"

/* PWM_A */
#define PWM_A_cy_m0s8_tcpwm_1__CC CYREG_TCPWM_CNT1_CC
#define PWM_A_cy_m0s8_tcpwm_1__CC_BUFF CYREG_TCPWM_CNT1_CC_BUFF
#define PWM_A_cy_m0s8_tcpwm_1__COUNTER CYREG_TCPWM_CNT1_COUNTER
#define PWM_A_cy_m0s8_tcpwm_1__CTRL CYREG_TCPWM_CNT1_CTRL
#define PWM_A_cy_m0s8_tcpwm_1__INTR CYREG_TCPWM_CNT1_INTR
#define PWM_A_cy_m0s8_tcpwm_1__INTR_MASK CYREG_TCPWM_CNT1_INTR_MASK
#define PWM_A_cy_m0s8_tcpwm_1__INTR_MASKED CYREG_TCPWM_CNT1_INTR_MASKED
#define PWM_A_cy_m0s8_tcpwm_1__INTR_SET CYREG_TCPWM_CNT1_INTR_SET
#define PWM_A_cy_m0s8_tcpwm_1__PERIOD CYREG_TCPWM_CNT1_PERIOD
#define PWM_A_cy_m0s8_tcpwm_1__PERIOD_BUFF CYREG_TCPWM_CNT1_PERIOD_BUFF
#define PWM_A_cy_m0s8_tcpwm_1__STATUS CYREG_TCPWM_CNT1_STATUS
#define PWM_A_cy_m0s8_tcpwm_1__TCPWM_CMD CYREG_TCPWM_CMD
#define PWM_A_cy_m0s8_tcpwm_1__TCPWM_CMDCAPTURE_MASK 0x02u
#define PWM_A_cy_m0s8_tcpwm_1__TCPWM_CMDCAPTURE_SHIFT 1u
#define PWM_A_cy_m0s8_tcpwm_1__TCPWM_CMDRELOAD_MASK 0x200u
#define PWM_A_cy_m0s8_tcpwm_1__TCPWM_CMDRELOAD_SHIFT 9u
#define PWM_A_cy_m0s8_tcpwm_1__TCPWM_CMDSTART_MASK 0x2000000u
#define PWM_A_cy_m0s8_tcpwm_1__TCPWM_CMDSTART_SHIFT 25u
#define PWM_A_cy_m0s8_tcpwm_1__TCPWM_CMDSTOP_MASK 0x20000u
#define PWM_A_cy_m0s8_tcpwm_1__TCPWM_CMDSTOP_SHIFT 17u
#define PWM_A_cy_m0s8_tcpwm_1__TCPWM_CTRL CYREG_TCPWM_CTRL
#define PWM_A_cy_m0s8_tcpwm_1__TCPWM_CTRL_MASK 0x02u
#define PWM_A_cy_m0s8_tcpwm_1__TCPWM_CTRL_SHIFT 1u
#define PWM_A_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE CYREG_TCPWM_INTR_CAUSE
#define PWM_A_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE_MASK 0x02u
#define PWM_A_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE_SHIFT 1u
#define PWM_A_cy_m0s8_tcpwm_1__TCPWM_NUMBER 1u
#define PWM_A_cy_m0s8_tcpwm_1__TR_CTRL0 CYREG_TCPWM_CNT1_TR_CTRL0
#define PWM_A_cy_m0s8_tcpwm_1__TR_CTRL1 CYREG_TCPWM_CNT1_TR_CTRL1
#define PWM_A_cy_m0s8_tcpwm_1__TR_CTRL2 CYREG_TCPWM_CNT1_TR_CTRL2

/* Clock_2 */
#define Clock_2__CTRL_REGISTER CYREG_PERI_PCLK_CTL3
#define Clock_2__DIV_ID 0x00000000u
#define Clock_2__DIV_REGISTER CYREG_PERI_DIV_8_CTL0
#define Clock_2__PA_DIV_ID 0x000000FFu

/* PWM_OUT_A */
#define PWM_OUT_A__0__DR CYREG_GPIO_PRT2_DR
#define PWM_OUT_A__0__DR_CLR CYREG_GPIO_PRT2_DR_CLR
#define PWM_OUT_A__0__DR_INV CYREG_GPIO_PRT2_DR_INV
#define PWM_OUT_A__0__DR_SET CYREG_GPIO_PRT2_DR_SET
#define PWM_OUT_A__0__HSIOM CYREG_HSIOM_PORT_SEL2
#define PWM_OUT_A__0__HSIOM_MASK 0x0000F000u
#define PWM_OUT_A__0__HSIOM_SHIFT 12u
#define PWM_OUT_A__0__INTCFG CYREG_GPIO_PRT2_INTR_CFG
#define PWM_OUT_A__0__INTR CYREG_GPIO_PRT2_INTR
#define PWM_OUT_A__0__INTR_CFG CYREG_GPIO_PRT2_INTR_CFG
#define PWM_OUT_A__0__INTSTAT CYREG_GPIO_PRT2_INTR
#define PWM_OUT_A__0__MASK 0x08u
#define PWM_OUT_A__0__PC CYREG_GPIO_PRT2_PC
#define PWM_OUT_A__0__PC2 CYREG_GPIO_PRT2_PC2
#define PWM_OUT_A__0__PORT 2u
#define PWM_OUT_A__0__PS CYREG_GPIO_PRT2_PS
#define PWM_OUT_A__0__SHIFT 3u
#define PWM_OUT_A__DR CYREG_GPIO_PRT2_DR
#define PWM_OUT_A__DR_CLR CYREG_GPIO_PRT2_DR_CLR
#define PWM_OUT_A__DR_INV CYREG_GPIO_PRT2_DR_INV
#define PWM_OUT_A__DR_SET CYREG_GPIO_PRT2_DR_SET
#define PWM_OUT_A__INTCFG CYREG_GPIO_PRT2_INTR_CFG
#define PWM_OUT_A__INTR CYREG_GPIO_PRT2_INTR
#define PWM_OUT_A__INTR_CFG CYREG_GPIO_PRT2_INTR_CFG
#define PWM_OUT_A__INTSTAT CYREG_GPIO_PRT2_INTR
#define PWM_OUT_A__MASK 0x08u
#define PWM_OUT_A__PC CYREG_GPIO_PRT2_PC
#define PWM_OUT_A__PC2 CYREG_GPIO_PRT2_PC2
#define PWM_OUT_A__PORT 2u
#define PWM_OUT_A__PS CYREG_GPIO_PRT2_PS
#define PWM_OUT_A__SHIFT 3u

/* SW_Tx_UART_tx */
#define SW_Tx_UART_tx__0__DR CYREG_GPIO_PRT1_DR
#define SW_Tx_UART_tx__0__DR_CLR CYREG_GPIO_PRT1_DR_CLR
#define SW_Tx_UART_tx__0__DR_INV CYREG_GPIO_PRT1_DR_INV
#define SW_Tx_UART_tx__0__DR_SET CYREG_GPIO_PRT1_DR_SET
#define SW_Tx_UART_tx__0__HSIOM CYREG_HSIOM_PORT_SEL1
#define SW_Tx_UART_tx__0__HSIOM_MASK 0x00000F00u
#define SW_Tx_UART_tx__0__HSIOM_SHIFT 8u
#define SW_Tx_UART_tx__0__INTCFG CYREG_GPIO_PRT1_INTR_CFG
#define SW_Tx_UART_tx__0__INTR CYREG_GPIO_PRT1_INTR
#define SW_Tx_UART_tx__0__INTR_CFG CYREG_GPIO_PRT1_INTR_CFG
#define SW_Tx_UART_tx__0__INTSTAT CYREG_GPIO_PRT1_INTR
#define SW_Tx_UART_tx__0__MASK 0x04u
#define SW_Tx_UART_tx__0__PC CYREG_GPIO_PRT1_PC
#define SW_Tx_UART_tx__0__PC2 CYREG_GPIO_PRT1_PC2
#define SW_Tx_UART_tx__0__PORT 1u
#define SW_Tx_UART_tx__0__PS CYREG_GPIO_PRT1_PS
#define SW_Tx_UART_tx__0__SHIFT 2u
#define SW_Tx_UART_tx__DR CYREG_GPIO_PRT1_DR
#define SW_Tx_UART_tx__DR_CLR CYREG_GPIO_PRT1_DR_CLR
#define SW_Tx_UART_tx__DR_INV CYREG_GPIO_PRT1_DR_INV
#define SW_Tx_UART_tx__DR_SET CYREG_GPIO_PRT1_DR_SET
#define SW_Tx_UART_tx__INTCFG CYREG_GPIO_PRT1_INTR_CFG
#define SW_Tx_UART_tx__INTR CYREG_GPIO_PRT1_INTR
#define SW_Tx_UART_tx__INTR_CFG CYREG_GPIO_PRT1_INTR_CFG
#define SW_Tx_UART_tx__INTSTAT CYREG_GPIO_PRT1_INTR
#define SW_Tx_UART_tx__MASK 0x04u
#define SW_Tx_UART_tx__PC CYREG_GPIO_PRT1_PC
#define SW_Tx_UART_tx__PC2 CYREG_GPIO_PRT1_PC2
#define SW_Tx_UART_tx__PORT 1u
#define SW_Tx_UART_tx__PS CYREG_GPIO_PRT1_PS
#define SW_Tx_UART_tx__SHIFT 2u

/* TYPE_A_VBUS_EN */
#define TYPE_A_VBUS_EN__0__DR CYREG_GPIO_PRT1_DR
#define TYPE_A_VBUS_EN__0__DR_CLR CYREG_GPIO_PRT1_DR_CLR
#define TYPE_A_VBUS_EN__0__DR_INV CYREG_GPIO_PRT1_DR_INV
#define TYPE_A_VBUS_EN__0__DR_SET CYREG_GPIO_PRT1_DR_SET
#define TYPE_A_VBUS_EN__0__HSIOM CYREG_HSIOM_PORT_SEL1
#define TYPE_A_VBUS_EN__0__HSIOM_MASK 0x0000000Fu
#define TYPE_A_VBUS_EN__0__HSIOM_SHIFT 0u
#define TYPE_A_VBUS_EN__0__INTCFG CYREG_GPIO_PRT1_INTR_CFG
#define TYPE_A_VBUS_EN__0__INTR CYREG_GPIO_PRT1_INTR
#define TYPE_A_VBUS_EN__0__INTR_CFG CYREG_GPIO_PRT1_INTR_CFG
#define TYPE_A_VBUS_EN__0__INTSTAT CYREG_GPIO_PRT1_INTR
#define TYPE_A_VBUS_EN__0__MASK 0x01u
#define TYPE_A_VBUS_EN__0__PC CYREG_GPIO_PRT1_PC
#define TYPE_A_VBUS_EN__0__PC2 CYREG_GPIO_PRT1_PC2
#define TYPE_A_VBUS_EN__0__PORT 1u
#define TYPE_A_VBUS_EN__0__PS CYREG_GPIO_PRT1_PS
#define TYPE_A_VBUS_EN__0__SHIFT 0u
#define TYPE_A_VBUS_EN__DR CYREG_GPIO_PRT1_DR
#define TYPE_A_VBUS_EN__DR_CLR CYREG_GPIO_PRT1_DR_CLR
#define TYPE_A_VBUS_EN__DR_INV CYREG_GPIO_PRT1_DR_INV
#define TYPE_A_VBUS_EN__DR_SET CYREG_GPIO_PRT1_DR_SET
#define TYPE_A_VBUS_EN__INTCFG CYREG_GPIO_PRT1_INTR_CFG
#define TYPE_A_VBUS_EN__INTR CYREG_GPIO_PRT1_INTR
#define TYPE_A_VBUS_EN__INTR_CFG CYREG_GPIO_PRT1_INTR_CFG
#define TYPE_A_VBUS_EN__INTSTAT CYREG_GPIO_PRT1_INTR
#define TYPE_A_VBUS_EN__MASK 0x01u
#define TYPE_A_VBUS_EN__PC CYREG_GPIO_PRT1_PC
#define TYPE_A_VBUS_EN__PC2 CYREG_GPIO_PRT1_PC2
#define TYPE_A_VBUS_EN__PORT 1u
#define TYPE_A_VBUS_EN__PS CYREG_GPIO_PRT1_PS
#define TYPE_A_VBUS_EN__SHIFT 0u

/* BUCK_BOOST_EN_A */
#define BUCK_BOOST_EN_A__0__DR CYREG_GPIO_PRT2_DR
#define BUCK_BOOST_EN_A__0__DR_CLR CYREG_GPIO_PRT2_DR_CLR
#define BUCK_BOOST_EN_A__0__DR_INV CYREG_GPIO_PRT2_DR_INV
#define BUCK_BOOST_EN_A__0__DR_SET CYREG_GPIO_PRT2_DR_SET
#define BUCK_BOOST_EN_A__0__HSIOM CYREG_HSIOM_PORT_SEL2
#define BUCK_BOOST_EN_A__0__HSIOM_MASK 0x00000F00u
#define BUCK_BOOST_EN_A__0__HSIOM_SHIFT 8u
#define BUCK_BOOST_EN_A__0__INTCFG CYREG_GPIO_PRT2_INTR_CFG
#define BUCK_BOOST_EN_A__0__INTR CYREG_GPIO_PRT2_INTR
#define BUCK_BOOST_EN_A__0__INTR_CFG CYREG_GPIO_PRT2_INTR_CFG
#define BUCK_BOOST_EN_A__0__INTSTAT CYREG_GPIO_PRT2_INTR
#define BUCK_BOOST_EN_A__0__MASK 0x04u
#define BUCK_BOOST_EN_A__0__PC CYREG_GPIO_PRT2_PC
#define BUCK_BOOST_EN_A__0__PC2 CYREG_GPIO_PRT2_PC2
#define BUCK_BOOST_EN_A__0__PORT 2u
#define BUCK_BOOST_EN_A__0__PS CYREG_GPIO_PRT2_PS
#define BUCK_BOOST_EN_A__0__SHIFT 2u
#define BUCK_BOOST_EN_A__DR CYREG_GPIO_PRT2_DR
#define BUCK_BOOST_EN_A__DR_CLR CYREG_GPIO_PRT2_DR_CLR
#define BUCK_BOOST_EN_A__DR_INV CYREG_GPIO_PRT2_DR_INV
#define BUCK_BOOST_EN_A__DR_SET CYREG_GPIO_PRT2_DR_SET
#define BUCK_BOOST_EN_A__INTCFG CYREG_GPIO_PRT2_INTR_CFG
#define BUCK_BOOST_EN_A__INTR CYREG_GPIO_PRT2_INTR
#define BUCK_BOOST_EN_A__INTR_CFG CYREG_GPIO_PRT2_INTR_CFG
#define BUCK_BOOST_EN_A__INTSTAT CYREG_GPIO_PRT2_INTR
#define BUCK_BOOST_EN_A__MASK 0x04u
#define BUCK_BOOST_EN_A__PC CYREG_GPIO_PRT2_PC
#define BUCK_BOOST_EN_A__PC2 CYREG_GPIO_PRT2_PC2
#define BUCK_BOOST_EN_A__PORT 2u
#define BUCK_BOOST_EN_A__PS CYREG_GPIO_PRT2_PS
#define BUCK_BOOST_EN_A__SHIFT 2u

/* BUCK_BOOST_EN_C */
#define BUCK_BOOST_EN_C__0__DR CYREG_GPIO_PRT1_DR
#define BUCK_BOOST_EN_C__0__DR_CLR CYREG_GPIO_PRT1_DR_CLR
#define BUCK_BOOST_EN_C__0__DR_INV CYREG_GPIO_PRT1_DR_INV
#define BUCK_BOOST_EN_C__0__DR_SET CYREG_GPIO_PRT1_DR_SET
#define BUCK_BOOST_EN_C__0__HSIOM CYREG_HSIOM_PORT_SEL1
#define BUCK_BOOST_EN_C__0__HSIOM_MASK 0x000000F0u
#define BUCK_BOOST_EN_C__0__HSIOM_SHIFT 4u
#define BUCK_BOOST_EN_C__0__INTCFG CYREG_GPIO_PRT1_INTR_CFG
#define BUCK_BOOST_EN_C__0__INTR CYREG_GPIO_PRT1_INTR
#define BUCK_BOOST_EN_C__0__INTR_CFG CYREG_GPIO_PRT1_INTR_CFG
#define BUCK_BOOST_EN_C__0__INTSTAT CYREG_GPIO_PRT1_INTR
#define BUCK_BOOST_EN_C__0__MASK 0x02u
#define BUCK_BOOST_EN_C__0__PC CYREG_GPIO_PRT1_PC
#define BUCK_BOOST_EN_C__0__PC2 CYREG_GPIO_PRT1_PC2
#define BUCK_BOOST_EN_C__0__PORT 1u
#define BUCK_BOOST_EN_C__0__PS CYREG_GPIO_PRT1_PS
#define BUCK_BOOST_EN_C__0__SHIFT 1u
#define BUCK_BOOST_EN_C__DR CYREG_GPIO_PRT1_DR
#define BUCK_BOOST_EN_C__DR_CLR CYREG_GPIO_PRT1_DR_CLR
#define BUCK_BOOST_EN_C__DR_INV CYREG_GPIO_PRT1_DR_INV
#define BUCK_BOOST_EN_C__DR_SET CYREG_GPIO_PRT1_DR_SET
#define BUCK_BOOST_EN_C__INTCFG CYREG_GPIO_PRT1_INTR_CFG
#define BUCK_BOOST_EN_C__INTR CYREG_GPIO_PRT1_INTR
#define BUCK_BOOST_EN_C__INTR_CFG CYREG_GPIO_PRT1_INTR_CFG
#define BUCK_BOOST_EN_C__INTSTAT CYREG_GPIO_PRT1_INTR
#define BUCK_BOOST_EN_C__MASK 0x02u
#define BUCK_BOOST_EN_C__PC CYREG_GPIO_PRT1_PC
#define BUCK_BOOST_EN_C__PC2 CYREG_GPIO_PRT1_PC2
#define BUCK_BOOST_EN_C__PORT 1u
#define BUCK_BOOST_EN_C__PS CYREG_GPIO_PRT1_PS
#define BUCK_BOOST_EN_C__SHIFT 1u

/* PDSS_PORT0_RX_CLK */
#define PDSS_PORT0_RX_CLK__DIV_ID 0x00000001u
#define PDSS_PORT0_RX_CLK__DIV_REGISTER CYREG_PERI_DIV_8_CTL1
#define PDSS_PORT0_RX_CLK__PA_DIV_ID 0x000000FFu

/* PDSS_PORT0_TX_CLK */
#define PDSS_PORT0_TX_CLK__DIV_ID 0x00000040u
#define PDSS_PORT0_TX_CLK__DIV_REGISTER CYREG_PERI_DIV_16_CTL0
#define PDSS_PORT0_TX_CLK__PA_DIV_ID 0x000000FFu

/* PDSS_PORT0_BCH_CLK */
#define PDSS_PORT0_BCH_CLK__DIV_ID 0x00000003u
#define PDSS_PORT0_BCH_CLK__DIV_REGISTER CYREG_PERI_DIV_8_CTL3
#define PDSS_PORT0_BCH_CLK__PA_DIV_ID 0x000000FFu

/* PDSS_PORT0_SAR_CLK */
#define PDSS_PORT0_SAR_CLK__DIV_ID 0x00000002u
#define PDSS_PORT0_SAR_CLK__DIV_REGISTER CYREG_PERI_DIV_8_CTL2
#define PDSS_PORT0_SAR_CLK__PA_DIV_ID 0x000000FFu

/* PDSS_PORT0_ISINK_CLK */
#define PDSS_PORT0_ISINK_CLK__DIV_ID 0x00000043u
#define PDSS_PORT0_ISINK_CLK__DIV_REGISTER CYREG_PERI_DIV_16_CTL3
#define PDSS_PORT0_ISINK_CLK__PA_DIV_ID 0x000000FFu

/* PDSS_PORTX_REFGEN_CLK */
#define PDSS_PORTX_REFGEN_CLK__DIV_ID 0x00000041u
#define PDSS_PORTX_REFGEN_CLK__DIV_REGISTER CYREG_PERI_DIV_16_CTL1
#define PDSS_PORTX_REFGEN_CLK__PA_DIV_ID 0x000000FFu

/* PDSS_PORT0_FILT_CLK_SEL */
#define PDSS_PORT0_FILT_CLK_SEL__DIV_ID 0x00000042u
#define PDSS_PORT0_FILT_CLK_SEL__DIV_REGISTER CYREG_PERI_DIV_16_CTL2
#define PDSS_PORT0_FILT_CLK_SEL__PA_DIV_ID 0x000000FFu

/* Miscellaneous */
#define CY_PROJECT_NAME "noboot"
#define CY_VERSION "PSoC Creator  4.3"
#define CYDEV_BANDGAP_VOLTAGE 1.200
#define CYDEV_BCLK__HFCLK__HZ 48000000U
#define CYDEV_BCLK__HFCLK__KHZ 48000U
#define CYDEV_BCLK__HFCLK__MHZ 48U
#define CYDEV_BCLK__SYSCLK__HZ 24000000U
#define CYDEV_BCLK__SYSCLK__KHZ 24000U
#define CYDEV_BCLK__SYSCLK__MHZ 24U
#define CYDEV_CHIP_DIE_LEOPARD 1u
#define CYDEV_CHIP_DIE_PSOC4A 24u
#define CYDEV_CHIP_DIE_PSOC5LP 2u
#define CYDEV_CHIP_DIE_PSOC5TM 3u
#define CYDEV_CHIP_DIE_TMA4 4u
#define CYDEV_CHIP_DIE_UNKNOWN 0u
#define CYDEV_CHIP_FAMILY_FM0P 5u
#define CYDEV_CHIP_FAMILY_FM3 6u
#define CYDEV_CHIP_FAMILY_FM4 7u
#define CYDEV_CHIP_FAMILY_PSOC3 1u
#define CYDEV_CHIP_FAMILY_PSOC4 2u
#define CYDEV_CHIP_FAMILY_PSOC5 3u
#define CYDEV_CHIP_FAMILY_PSOC6 4u
#define CYDEV_CHIP_FAMILY_UNKNOWN 0u
#define CYDEV_CHIP_FAMILY_USED CYDEV_CHIP_FAMILY_PSOC4
#define CYDEV_CHIP_JTAG_ID 0x200311B0u
#define CYDEV_CHIP_MEMBER_3A 1u
#define CYDEV_CHIP_MEMBER_4A 24u
#define CYDEV_CHIP_MEMBER_4AA 23u
#define CYDEV_CHIP_MEMBER_4AB 28u
#define CYDEV_CHIP_MEMBER_4AC 14u
#define CYDEV_CHIP_MEMBER_4D 18u
#define CYDEV_CHIP_MEMBER_4E 6u
#define CYDEV_CHIP_MEMBER_4F 25u
#define CYDEV_CHIP_MEMBER_4G 4u
#define CYDEV_CHIP_MEMBER_4H 22u
#define CYDEV_CHIP_MEMBER_4I 30u
#define CYDEV_CHIP_MEMBER_4J 19u
#define CYDEV_CHIP_MEMBER_4K 20u
#define CYDEV_CHIP_MEMBER_4L 29u
#define CYDEV_CHIP_MEMBER_4M 27u
#define CYDEV_CHIP_MEMBER_4N 11u
#define CYDEV_CHIP_MEMBER_4O 8u
#define CYDEV_CHIP_MEMBER_4P 26u
#define CYDEV_CHIP_MEMBER_4Q 15u
#define CYDEV_CHIP_MEMBER_4R 9u
#define CYDEV_CHIP_MEMBER_4S 12u
#define CYDEV_CHIP_MEMBER_4T 10u
#define CYDEV_CHIP_MEMBER_4U 5u
#define CYDEV_CHIP_MEMBER_4V 21u
#define CYDEV_CHIP_MEMBER_4W 13u
#define CYDEV_CHIP_MEMBER_4X 7u
#define CYDEV_CHIP_MEMBER_4Y 16u
#define CYDEV_CHIP_MEMBER_4Z 17u
#define CYDEV_CHIP_MEMBER_5A 3u
#define CYDEV_CHIP_MEMBER_5B 2u
#define CYDEV_CHIP_MEMBER_6A 31u
#define CYDEV_CHIP_MEMBER_FM3 35u
#define CYDEV_CHIP_MEMBER_FM4 36u
#define CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE1 32u
#define CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE2 33u
#define CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE3 34u
#define CYDEV_CHIP_MEMBER_UNKNOWN 0u
#define CYDEV_CHIP_MEMBER_USED CYDEV_CHIP_MEMBER_4R
#define CYDEV_CHIP_DIE_EXPECT CYDEV_CHIP_MEMBER_USED
#define CYDEV_CHIP_DIE_ACTUAL CYDEV_CHIP_DIE_EXPECT
#define CYDEV_CHIP_REV_LEOPARD_ES1 0u
#define CYDEV_CHIP_REV_LEOPARD_ES2 1u
#define CYDEV_CHIP_REV_LEOPARD_ES3 3u
#define CYDEV_CHIP_REV_LEOPARD_PRODUCTION 3u
#define CYDEV_CHIP_REV_PSOC4A_ES0 17u
#define CYDEV_CHIP_REV_PSOC4A_PRODUCTION 17u
#define CYDEV_CHIP_REV_PSOC5LP_ES0 0u
#define CYDEV_CHIP_REV_PSOC5LP_PRODUCTION 0u
#define CYDEV_CHIP_REV_PSOC5TM_ES0 0u
#define CYDEV_CHIP_REV_PSOC5TM_ES1 1u
#define CYDEV_CHIP_REV_PSOC5TM_PRODUCTION 1u
#define CYDEV_CHIP_REV_TMA4_ES 17u
#define CYDEV_CHIP_REV_TMA4_ES2 33u
#define CYDEV_CHIP_REV_TMA4_PRODUCTION 17u
#define CYDEV_CHIP_REVISION_3A_ES1 0u
#define CYDEV_CHIP_REVISION_3A_ES2 1u
#define CYDEV_CHIP_REVISION_3A_ES3 3u
#define CYDEV_CHIP_REVISION_3A_PRODUCTION 3u
#define CYDEV_CHIP_REVISION_4A_ES0 17u
#define CYDEV_CHIP_REVISION_4A_PRODUCTION 17u
#define CYDEV_CHIP_REVISION_4AA_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4AB_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4AC_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4D_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4E_CCG2_NO_USBPD 0u
#define CYDEV_CHIP_REVISION_4E_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4F_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4F_PRODUCTION_256DMA 0u
#define CYDEV_CHIP_REVISION_4F_PRODUCTION_256K 0u
#define CYDEV_CHIP_REVISION_4G_ES 17u
#define CYDEV_CHIP_REVISION_4G_ES2 33u
#define CYDEV_CHIP_REVISION_4G_PRODUCTION 17u
#define CYDEV_CHIP_REVISION_4H_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4I_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4J_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4K_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4L_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4M_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4N_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4O_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4P_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4Q_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4R_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4S_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4T_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4U_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4V_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4W_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4X_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4Y_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4Z_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_5A_ES0 0u
#define CYDEV_CHIP_REVISION_5A_ES1 1u
#define CYDEV_CHIP_REVISION_5A_PRODUCTION 1u
#define CYDEV_CHIP_REVISION_5B_ES0 0u
#define CYDEV_CHIP_REVISION_5B_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_6A_ES 17u
#define CYDEV_CHIP_REVISION_6A_NO_UDB 33u
#define CYDEV_CHIP_REVISION_6A_PRODUCTION 33u
#define CYDEV_CHIP_REVISION_FM3_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_FM4_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_PDL_FM0P_TYPE1_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_PDL_FM0P_TYPE2_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_PDL_FM0P_TYPE3_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_USED CYDEV_CHIP_REVISION_4R_PRODUCTION
#define CYDEV_CHIP_REV_EXPECT CYDEV_CHIP_REVISION_USED
#define CYDEV_CONFIG_READ_ACCELERATOR 1
#define CYDEV_CONFIG_UNUSED_IO_AllowButWarn 0
#define CYDEV_CONFIG_UNUSED_IO_AllowWithInfo 1
#define CYDEV_CONFIG_UNUSED_IO_Disallowed 2
#define CYDEV_CONFIGURATION_COMPRESSED 1
#define CYDEV_CONFIGURATION_MODE_COMPRESSED 0
#define CYDEV_CONFIGURATION_MODE CYDEV_CONFIGURATION_MODE_COMPRESSED
#define CYDEV_CONFIGURATION_MODE_DMA 2
#define CYDEV_CONFIGURATION_MODE_UNCOMPRESSED 1
#define CYDEV_DEBUG_PROTECT_KILL 4
#define CYDEV_DEBUG_PROTECT_OPEN 1
#define CYDEV_DEBUG_PROTECT CYDEV_DEBUG_PROTECT_OPEN
#define CYDEV_DEBUG_PROTECT_PROTECTED 2
#define CYDEV_DEBUGGING_DPS_Disable 3
#define CYDEV_DEBUGGING_DPS CYDEV_DEBUGGING_DPS_Disable
#define CYDEV_DEBUGGING_DPS_SWD 2
#define CYDEV_DEBUGGING_ENABLE 0
#define CYDEV_DFT_SELECT_CLK0 8u
#define CYDEV_DFT_SELECT_CLK1 9u
#define CYDEV_HEAP_SIZE 0x80
#define CYDEV_IMO_TRIMMED_BY_USB 0u
#define CYDEV_IMO_TRIMMED_BY_WCO 0u
#define CYDEV_IS_EXPORTING_CODE 0
#define CYDEV_IS_IMPORTING_CODE 0
#define CYDEV_PROJ_TYPE 0
#define CYDEV_PROJ_TYPE_BOOTLOADER 1
#define CYDEV_PROJ_TYPE_LAUNCHER 5
#define CYDEV_PROJ_TYPE_LOADABLE 2
#define CYDEV_PROJ_TYPE_LOADABLEANDBOOTLOADER 4
#define CYDEV_PROJ_TYPE_MULTIAPPBOOTLOADER 3
#define CYDEV_PROJ_TYPE_STANDARD 0
#define CYDEV_STACK_SIZE 0x0800
#define CYDEV_USE_BUNDLED_CMSIS 1
#define CYDEV_VARIABLE_VDDA 1
#define CYDEV_VDDD 3.3
#define CYDEV_VDDD_MV 3300
#define CYDEV_WDT_GENERATE_ISR 0u
#define CYIPBLOCK_m0s8cpussv3_VERSION 1
#define CYIPBLOCK_m0s8ioss_VERSION 1
#define CYIPBLOCK_m0s8peri_VERSION 1
#define CYIPBLOCK_m0s8scb_VERSION 2
#define CYIPBLOCK_m0s8tcpwm_VERSION 2
#define CYIPBLOCK_mxusbpd_VERSION 1
#define CYIPBLOCK_s8srsslt_VERSION 1
#define CYDEV_BOOTLOADER_ENABLE 0

#endif /* INCLUDED_CYFITTER_H */
