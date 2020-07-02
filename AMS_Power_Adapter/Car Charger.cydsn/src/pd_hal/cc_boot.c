/*
 * @cc_boot.c
 * @brief CC bootloader policy source file.
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

#include <project.h>
#include <config.h>
#include "status.h"
#include "flash.h"
#include "pdss_hal.h"
#include "gpio.h"
#include "cc_boot.h"
#include "pd.h"
#include "ccgx_regs.h"
#include "uvdm.h"
#include "hal_ccgx.h"
#include "system.h"

/* Ordered set detection. SOP and Hard Reset. */
#define UFP_OS_CFG                      (0x0000410Bu)

/* USBPD header fields macros */
#define PDSS_MSG_SZ(x)                  ((((x) >> 12) & 0x7) + 1)

#define DATA_ROLE_DFP                   (0x0020u)

#define RX_INTERRUPTS1                  \
    (PDSS_INTR0_RCV_GOOD_PACKET_COMPLETE |      \
     PDSS_INTR0_RCV_BAD_PACKET_COMPLETE  |      \
     PDSS_INTR0_TX_GOODCRC_MSG_DONE      |      \
     PDSS_INTR0_RCV_RST                  |      \
     PDSS_INTR0_COLLISION_TYPE3)

/* Index of PD Header in data message */
#define PD_HDR_IDX                      (0u)

#define VBUS_P_NGDO_EN_LV_0             (1 << 13)
#define VBUS_P_NGDO_EN_LV_1             (1 << 12)
#define VBUS_P_PLDN_EN_LV_0             (1 << 15)
#define VBUS_P_PLDN_EN_LV_1             (1 << 14)

/* Hardware required averaging values - design team recommendation. */
#define PDSS_NUM_PREAMBLE_AVG_VALUE     (5u)
#define PDSS_NUM_TRANS_AVG_VALUE        (0x31u)

/*
 * The SYS_CLK counter theshold for 1ms timeout. Division is expected
 * to be optimized out by the compiler.
 */
#define SYSCLK_1MS_COUNT_VALUE          (CYDEV_BCLK__SYSCLK__HZ / 1000)

/* This translates to around 50ms of CC debounce. */
#define MAX_DEBOUNCE_ATTACH_COUNT       (2500)

#define MAX_DEBOUNCE_DETACH_COUNT       (2500)

/* Structured VDM Response values*/
#define SVDM_DSC_ID_HEADER              (0xFF008041)
#define SVDM_DSC_ID_HEADER_VDO          (0x920004B4)
#define SVDM_DSC_ID_PRODUCT_VDO         (0xF6600000)
#define SVDM_DSC_SVID_HEADER            (0xFF008042)
#define SVDM_DSC_SVID_VDO1              (0x04B40000)
#define SVDM_DSC_MODE_HEADER            (0x04B48043)
#define SVDM_DSC_MODE_VDO               (0x00000001)
#define SVDM_ENTER_MODE_HEADER          (0x04B48044)

/*
 * CCG3PA discharge drive strength setting. This setting is meant for *A or above
 * silicon. But we are using the same for all revisions to reduce code space.
 */
#define CCG3PA_DISCHG_DS_VBUS_C             (4u)

/*
 * VBUS_IN startup threshold voltage before starting PWM signal.
 */
#define PASC_VBUS_IN_STARTUP_THRES          (4000)

/*
 * VBUS_IN startup EA DAC value.
 */
#define PASC_VBUS_IN_STARTUP_SRC_EA         (85)

/* PAG1S VBUS_IN resistor divider for Type-C VBUS monitoring using ADC. */
#define AMUX_ADC_PAG1S_VBUS_IN_8P_EN_POS    (9)

#define VBUS_MON_DIV_8P_VAL                 (25u)   /* Multiplied by 2. */

/*
 * GPIO to be used as PWM signal to the primary. This is defined for PAG1S
 * silicon and should not be changed.
 */
#define PASC_PTDRV_GPIO                     (GPIO_PORT_0_PIN_1)

/* 
 * Number of manual pulses to be sent as the start up sequence to the primary
 * controller. This is defined by the PAG1P requirements and should not be
 * changed.
 */
#define PASC_PTDRV_PULSE_COUNT              (4)

/*
 * Delay to be applied in us as the OFF time for the start up pulsing.
 * This is defined for PAG1P and should not be modified.
 */
#define PASC_PTDRV_PULSE_OFF_TIME_US        (23)

/* Time in us taken for peak calibration to complete. */
#define PASC_PTDRV_CAL_TIME                 (100)

/* This is required only on PSVP due to a glitch on SR gate. */
#define PASC_HIP_SEQ_ZCD_EN_ON_DLY_VALUE    (0x1F)
#define PASC_HIP_SEQ_ZCDF_EN_ON_DLY_VALUE   (0x0F)

static pd_state_t gl_pd_state;
static uint32_t volatile gl_pd_event;
static uint32_t gl_rcvd_pkt[MAX_PD_PKT_WORDS];
static uint8_t volatile gl_tr_msg_id;

static uint8_t volatile gl_first_msg_rcvd ;

/* Stores Current received packet's message id */
static uint8_t gl_cur_rec_msg_id;

/*
 * Variable stores message Id of last processed packet.
 * It's used along with gl_cur_rec_msg_id to detect whether
 * new packet is received
 */
static uint8_t volatile gl_rec_msg_id ;

static pd_do_t dobject[8];
static volatile uint32_t gl_cc_count[2] = {0,0};
static volatile uint32_t gl_active_channel;

#if (SOURCE_BOOT)
 /* Port type DFP,UFP or DRP */
uint8_t gl_cur_port_type = PRT_TYPE_DFP;
pd_do_t src_pdo[1]= {{0x0A01905A}};
#endif /* SOURCE_BOOT */

void pd_reset_protocol(void)
{
    gl_rec_msg_id = 0;
    gl_tr_msg_id = 0;
    gl_first_msg_rcvd = false;
}

/**
 * @brief This function sends PD message
 * @param msg_type Type of PD message.
 * @param dobj Pointer to data object
 * @param count Number of data objects to send. It should be less than or equal to seven.
 * @return None
 */
static void send_message(uint8_t msg_type, uint32_t* pdo, uint8_t count)
{
    uint32_t i;          /* Buf load index */

    /*Wait 1ms to over previous message transmission */
    CyDelay(1);
    /* Update msg type, ID, count and data role(port type) in the packet header */
    PDSS->tx_header = PD_HEADER(msg_type, gl_tr_msg_id, count);
#if (SOURCE_BOOT)
    PDSS->tx_header |= PD_DR_PR_ROLE(gl_cur_port_type, PRT_ROLE_SOURCE);
#endif /*SOURCE_BOOT*/

    for (i = 0; i < count; i++)
    {
        PDSS->tx_mem_data[i]  = pdo[i];
    }

    /* Begin transmission. */
    PDSS->tx_ctrl |= PDSS_TX_CTRL_TX_GO;

    /* Delay to complete message transmission(~2ms) and good CRC (~ 0.5 ms) */
    CyDelay(3);
    /* Increment message IDs */
    gl_tr_msg_id = (gl_tr_msg_id + 1) & MAX_MESSAGE_ID;
}

#if (defined(PAG1S) && (CCG_REG_SEC_CTRL != 0))

/* Initilization value for the SRSNS_2_CTRL register. */
#define PDSS_SRSNS_2_CTRL_INIT_VALUE            (0x6048)

void pd_pasc_disable(uint8_t port)
{
    (void)port;

    /* Set HSIOM configuration for PWM for GPIO to prevent any false pulses. */
    gpio_hsiom_set_config(PASC_PTDRV_GPIO, HSIOM_MODE_GPIO, GPIO_DM_STRONG, false);

    /* Hold PASC in reset. */
    PDSS->debug_ctrl |= PDSS_DEBUG_CTRL_RESET_PASC;

    PDSS->pasc_ctrl &= ~PDSS_PASC_CTRL_PA_EN;

    /* Power down the EA block. */
    PDSS->ea_ctrl |= PDSS_EA_CTRL_EA_PD;

    /* Disable PWM block. */
    PDSS->pwm_0_ctrl &= ~(PDSS_PWM_0_CTRL_ENABLE_PWM | PDSS_PWM_0_CTRL_PWM_ISO_N |
            PDSS_PWM_0_CTRL_ENABLE_PWDM_DAC | PDSS_PWM_0_CTRL_ENABLE_PWM_DAC_FF | 
            PDSS_PWM_0_CTRL_ENABLE_PWM_HCLAMP | PDSS_PWM_0_CTRL_ENABLE_PWM_LCLAMP |
            PDSS_PWM_0_CTRL_ENABLE_PWM_SKIP | PDSS_PWM_0_CTRL_ENABLE_PWM_BURST_EXIT);
}

static void pd_pasc_init(void)
{
    uint32_t tmp = 0;

    /* PASC mode is set to QR by default */

    /* Enable the SR sense block. */
    tmp = PDSS->srsns_0_ctrl;
    tmp &= ~(PDSS_SRSNS_0_CTRL_ZCD_PD | PDSS_SRSNS_0_CTRL_ZCDF_PD |
        PDSS_SRSNS_0_CTRL_NSN_PD | PDSS_SRSNS_0_CTRL_PEAKDET_PD);
    tmp |= (PDSS_SRSNS_0_CTRL_SRSNS_ISO_N | PDSS_SRSNS_0_CTRL_FEEDFWD_EN);
    PDSS->srsns_0_ctrl = tmp;

    /* 
     * BIT14:13: To enable Fast NSN comparator with 200mV ref.
     * BIT6: To set NSN threshold to zero volt.
     * BIT3: To enable NSN hysterisis
     */
    PDSS->srsns_2_ctrl = PDSS_SRSNS_2_CTRL_INIT_VALUE;

    /* Configure EA for PWM mode operation. */
    PDSS->pds_ea_1_ctrl |= PDSS_PDS_EA_1_CTRL_EN_PWM_MODE;

    /* Setting for 9:1 turns ratio. */
    REG_FIELD_UPDATE(PDSS->srsns_1_ctrl, PDSS_SRSNS_1_CTRL_FFWD_TEST, 0x040F);

    /* Enable PWM block. */
    PDSS->pwm_0_ctrl |= (PDSS_PWM_0_CTRL_ENABLE_PWM | PDSS_PWM_0_CTRL_PWM_ISO_N |
            PDSS_PWM_0_CTRL_ENABLE_PWDM_DAC | PDSS_PWM_0_CTRL_ENABLE_PWM_DAC_FF | 
            PDSS_PWM_0_CTRL_ENABLE_PWM_HCLAMP | PDSS_PWM_0_CTRL_ENABLE_PWM_LCLAMP |
            PDSS_PWM_0_CTRL_ENABLE_PWM_SKIP | PDSS_PWM_0_CTRL_ENABLE_PWM_BURST_EXIT);

    /*
     * Skip entry = 450mV, Burst entry = 450mV, burst exit = 450mV, Lclamp = 350mV, Hclamp = 2.4V
     * Increase filter to 0x1F (maximum). ** silicon has a different minimum level for skip entry,
     * skip exit, burst exit, and VREF_LOW. So set them differently. The current configuration 
     * is set to avoid burst mode.
     */
    if (get_silicon_revision() != CCG_SILICON_REV00_VALUE)
    {
        REG_FIELD_UPDATE(PDSS->peakgen_1_ctrl, PDSS_PEAKGEN_1_CTRL_SKIP_TRIM_VAL, 8);
        REG_FIELD_UPDATE(PDSS->peakgen_1_ctrl, PDSS_PEAKGEN_1_CTRL_BURST_TRIM_VAL, 8);
        REG_FIELD_UPDATE(PDSS->pwm_0_ctrl, PDSS_PWM_0_CTRL_PWM_BURST_EXIT_SEL, 8);
        REG_FIELD_UPDATE(PDSS->pwm_1_ctrl, PDSS_PWM_1_CTRL_PWM_LCLAMP_SEL, 7);
    }
    else
    {
        REG_FIELD_UPDATE(PDSS->peakgen_1_ctrl, PDSS_PEAKGEN_1_CTRL_SKIP_TRIM_VAL, 4);
        REG_FIELD_UPDATE(PDSS->peakgen_1_ctrl, PDSS_PEAKGEN_1_CTRL_BURST_TRIM_VAL, 4);
        REG_FIELD_UPDATE(PDSS->pwm_0_ctrl, PDSS_PWM_0_CTRL_PWM_BURST_EXIT_SEL, 0);
        REG_FIELD_UPDATE(PDSS->pwm_1_ctrl, PDSS_PWM_1_CTRL_PWM_LCLAMP_SEL, 3);
    }
    REG_FIELD_UPDATE(PDSS->pwm_1_ctrl, PDSS_PWM_1_CTRL_PWM_HCLAMP_SEL, 3);
    REG_FIELD_UPDATE(PDSS->intr15_cfg_0_pwm, PDSS_INTR15_CFG_0_PWM_SKIP_OUT_FILT_SEL, 0x1F);
    
    REG_FIELD_UPDATE(PDSS_TRIM_PWM_0, PDSS_TRIM_PWM_0_CAP_DAC_TRIM,  24);
    REG_FIELD_UPDATE(PDSS_TRIM_PWM_1, PDSS_TRIM_PWM_1_CAP_DAC_FF_TRIM,  28);
    
    REG_FIELD_UPDATE(PDSS->hip_seq_gen_0_ctrl, PDSS_HIP_SEQ_GEN_0_CTRL_ZCD_EN_ON_DLY,
            PASC_HIP_SEQ_ZCD_EN_ON_DLY_VALUE);
    REG_FIELD_UPDATE(PDSS->hip_seq_gen_0_ctrl, PDSS_HIP_SEQ_GEN_0_CTRL_NSN_EN_ON_DLY,
            0x07);
    REG_FIELD_UPDATE(PDSS->hip_seq_gen_1_ctrl, PDSS_HIP_SEQ_GEN_1_CTRL_ZCDF_EN_ON_DLY,
            PASC_HIP_SEQ_ZCDF_EN_ON_DLY_VALUE);
    REG_FIELD_UPDATE(PDSS->hip_seq_gen_1_ctrl, PDSS_HIP_SEQ_GEN_1_CTRL_PEAKDET_SW_EN_ON_DLY,
            0x05);
    REG_FIELD_UPDATE(PDSS->hip_seq_gen_2_ctrl, PDSS_HIP_SEQ_GEN_2_CTRL_NSN_IDLE_TIME,
            0xF0);
    
    /*
     * PWM_DAC and PWM_DAC_FF setting need to be changed to support below
     * configuration.
     * For 85V : 0.5uA (fixed) + 0.1uA (FF) 
     * so that the ramp is ~28uS (from vreflo to vclamp_hi)
     * For 380V : 0.5uA (Fixed) + 0.5uA (FF) - and this will happen 
     * automatically with same setting of PWM DAC as used in 85V case.
     * So that the ramp is of 17uS  (from Vreflo to vclamp_hi)
     * Note: Fixed DAC gain set to 0.2x (2.4u*10/30) and 
     * feedfwd DAC set to 0.03x(2.4u*5/30).
     */
    tmp = PDSS->pwm_1_ctrl & ~(PDSS_PWM_1_CTRL_PWM_DAC_MASK |
            PDSS_PWM_1_CTRL_PWM_DAC_FF_MASK);
    tmp |= ((6 << PDSS_PWM_1_CTRL_PWM_DAC_POS) |
            (3 << PDSS_PWM_1_CTRL_PWM_DAC_FF_POS));
    PDSS->pwm_1_ctrl = tmp;

    /*
     * Configure the AUDIO_TMIN and AUDIO_TMAX to avoid audible buzz. These
     * registers do not configure the frequency but act as TRIM registers to
     * match avoid buzz due to burst mode.
     */
    REG_FIELD_UPDATE(PDSS->mode_3_ctrl, PDSS_MODE_3_CTRL_AUDIO_TMAX, 0x4FF);
    REG_FIELD_UPDATE(PDSS->mode_4_ctrl, PDSS_MODE_4_CTRL_AUDIO_TMIN, 0x4FA);

    /* Enable peak reset pulse */
    REG_FIELD_UPDATE(PDSS->peakgen_0_ctrl, PDSS_PEAKGEN_0_CTRL_PEAK_RESET_PULSE,
            1);

    /* Not performing the calibration */
    PDSS->peakgen_0_ctrl |= PDSS_PEAKGEN_0_CTRL_CALIBRATE_OVERRIDE;

    /* Peak kill needs to be disabled as silicon does not trigger correctly. */
    PDSS->peakgen_1_ctrl &= ~PDSS_PEAKGEN_1_CTRL_PEAK_KILL_EN;

    /* Allow PAG1S hardware to take over PWM control. */
    PDSS->pasc_ctrl |= PDSS_PASC_CTRL_PA_EN;

    /*
     * No calibration as it is being overridden. But, wait for the delay. Not mandatory.
     */
    CyDelayUs(PASC_PTDRV_CAL_TIME);
}
#endif /* (defined(PAG1S) && (CCG_REG_SEC_CTRL != 0)) */

void pd_send_ctl_msg(ctrl_msg_t msg_type)
{
    send_message(msg_type, NULL, 0);
}

void pd_send_data_msg(data_msg_t msg_type, pd_do_t* dobj, uint8_t count)
{
    send_message(msg_type, &(dobj->val), count);
}

void start_systick_timer(uint8_t timems)
{
    CM0->syst_cvr = 0;
    CM0->syst_rvr = ((timems * SYSCLK_1MS_COUNT_VALUE) - 1);
    CM0->syst_csr = (CM0_SYST_CSR_ENABLE | CM0_SYST_CSR_CLKSOURCE);
}

void analyze_rx(void)
{
    uint32_t msg_type ;
    uint32_t count ;
    uint8_t i, size;
    /* Get the size of RX packet from RX_HEADER register. */
    size = PDSS_MSG_SZ(PDSS->rx_header);
    /* Store the header. */
    gl_rcvd_pkt[0] = PDSS->rx_header;
    /* Copy the data into the specified buffer */
    for (i = 0; i < (size-1); i++)
    {
        *(gl_rcvd_pkt + i+1) = PDSS->rx_mem_data[i];
    }

    /* Update current message id */
    gl_cur_rec_msg_id = GET_PD_HDR_ID(gl_rcvd_pkt[PD_HDR_IDX]);
    msg_type = GET_PD_HDR_TYPE(gl_rcvd_pkt[PD_HDR_IDX]);
    count = GET_PD_HDR_CNT(gl_rcvd_pkt[PD_HDR_IDX]);
    /* Checks if a goodcrc message */
    if ((msg_type == CTRL_MSG_GOOD_CRC) && (count == 0))
    {
        return;
    }

    /* If first message received or valid new message received */
    if((gl_first_msg_rcvd == false)
        || ((gl_rec_msg_id) != gl_cur_rec_msg_id))
    {
        /* Valid packet received */
        gl_first_msg_rcvd = true;
        gl_rec_msg_id = gl_cur_rec_msg_id;
        gl_pd_event = PE_EVT_PKT_RCVD;
    }
}

/**
 * PDSS interrupt handler
 * Interrupt and bottom half handling for Tx and Rx paths.
 */
void pdss_phy_intr0_handler(void)
{
    uint32_t intr0_cause;

    /* Read interrupt causes. */
    intr0_cause = PDSS->intr0_masked;

    if (intr0_cause != 0)
    {
        /* Clear interrupts. */
        PDSS->intr0 = intr0_cause;

        if (intr0_cause & PDSS_INTR0_TX_GOODCRC_MSG_DONE)
        {
            analyze_rx();
        }
        if (intr0_cause & PDSS_INTR0_RCV_RST)
        {
            gl_pd_event = PE_EVT_HARD_RESET_RCVD;
        }
    }
}

void pdss_phy_init(void)
{
    uint32_t tmp;
    
#if (VBUS_CTRL_TYPE == VBUS_CTRL_OPTO_FB) 
    /* Enable shunt */
    PDSS->ea_ctrl |= PDSS_EA_CTRL_EN_SHNT | PDSS_EA_CTRL_SHNT_ST_OPAMP_ENB;
#ifndef PAG1S
    PDSS_TRIMS->trim_ea1_0 = 0xA0;
#endif /* !PAG1S */
#else /* (VBUS_CTRL_TYPE_P1 != VBUS_CTRL_OPTO_FB) */
#ifndef PAG1S
    PDSS_TRIMS->trim_ea1_0 = 0x20;
#endif /* !PAG1S */
#endif /* VBUS_CTRL_TYPE_P1 */

    PERI->pclk_ctl[PDSS_PORT0_PCLK_RX_IDX] = PDSS_PORT0_RX_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_TX_IDX] = PDSS_PORT0_TX_CLK_DIV_ID;
#ifdef PAG1S
  #if CCG_REG_SEC_CTRL
    PERI->pclk_ctl[PDSS_PORT0_PCLK_REFGEN_IDX] = PDSS_PORT0_REFGEN_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_SAR_IDX] = PDSS_PORT0_SAR_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_PASC_IDX] = PDSS_PORT0_PASC_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_VBTR_IDX] = PDSS_PORT0_VBTR_CLK_DIV_ID;
  #endif /* CCG_REG_SEC_CTRL */
#endif /* PAG1S */

    /* IP enable. */
    PDSS->ctrl &= ~PDSS_CTRL_IP_ENABLED;
    PDSS->ctrl = PDSS_CTRL_IP_ENABLED;

    /* Turn off PHY deepsleep. References require 100us to stabilize. */
    PDSS->dpslp_ref_ctrl = ((PDSS->dpslp_ref_ctrl & ~PDSS_DPSLP_REF_CTRL_PD_DPSLP) |
            PDSS_DPSLP_REF_CTRL_IGEN_EN);

#ifdef PAG1S
    /*
     * Also turn on the NGDO. Since this requires delay in stages, putting it
     * across the available delay to save code space.
     */
    PDSS->ngdo_ctrl |= (PDSS_NGDO_CTRL_NGDO_ISO_N | PDSS_NGDO_CTRL_NGDO_EN_LV);
#endif /* PAG1S */
    CyDelayUs(100);

#ifdef PAG1S
    /* Update Refgen setting */
    PDSS->refgen_0_ctrl &= ~PDSS_REFGEN_0_CTRL_REFGEN_PD;
    PDSS->refgen_0_ctrl  |= PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_SEL |
                          PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_MON_SEL;
    /*
     * Continue turn on of NGDO. Since this requires delay in stages, putting
     * it across the available delay to save code space.
     */
    PDSS->ngdo_ctrl |= (PDSS_NGDO_CTRL_NGDO_CP_EN);
#endif /* PAG1S */

     /*Configure CRC_COUNTER reg*/
    PDSS->crc_counter = CRC_COUNTER_CFG;

    /*Configure INTER_PACKET_COUNTER reg*/
    PDSS->inter_packet_counter = INTER_PACKET_COUNTER_CFG;

    /* Enable active circuitry and DC paths. */
    PDSS->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_PWR_DISABLE;

    /* REF-GEN updates from CDT 275028. */
    PDSS->refgen_0_ctrl &= ~PDSS_REFGEN_0_CTRL_REFGEN_PD;
    PDSS->refgen_2_ctrl  = 0x003D6461;

#if (SOURCE_BOOT)
    /*
     * Enable proper Rp and enable comparators.
     * Set Rp mode and enable references for source operation.
     */
    PDSS->cc_ctrl_0 |= (PDSS_CC_CTRL_0_RD_CC1_DB_DIS | PDSS_CC_CTRL_0_RD_CC2_DB_DIS |
        PDSS_CC_CTRL_0_RP_CC1_EN | PDSS_CC_CTRL_0_RP_CC2_EN | PDSS_CC_CTRL_0_RX_EN |
        PDSS_CC_CTRL_0_RP_MODE_MASK | PDSS_CC_CTRL_0_DFP_EN | PDSS_CC_CTRL_0_CMP_EN |
        (PDSS_CC_CTRL_0_CMP_LA_VSEL_CFG << PDSS_CC_CTRL_0_CMP_LA_VSEL_POS) |
        PDSS_CC_CTRL_0_CMP_UP_VSEL_MASK | (4u << PDSS_CC_CTRL_0_CMP_DN_VSEL_POS));
    PDSS->pump_ctrl &= ~ (PDSS_PUMP_CTRL_BYPASS_LV | PDSS_PUMP_CTRL_PD_PUMP);
    PDSS->tx_ctrl |= PD_DR_PR_ROLE(PRT_TYPE_DFP, PRT_ROLE_SOURCE);
#else

    /* Enable proper Rd and enable comparators.
     * Connect Up comparaator to CC2 and Down comparator to CC1
     */
    PDSS->cc_ctrl_0 |= (PDSS_CC_CTRL_0_RD_CC1_DB_DIS | PDSS_CC_CTRL_0_RD_CC2_DB_DIS |
            PDSS_CC_CTRL_0_RD_CC1_EN | PDSS_CC_CTRL_0_RD_CC2_EN | PDSS_CC_CTRL_0_RX_EN |
            (PDSS_CC_CTRL_0_CMP_LA_VSEL_CFG << PDSS_CC_CTRL_0_CMP_LA_VSEL_POS) |
            PDSS_CC_CTRL_0_CMP_EN);
#endif
    /*
     * GoodCRC auto response configuration. Since the receiver is controlled,
     * these registers can be enabled all the time.
     */
#if (defined(CCG3))
    PDSS->rx_default_sop_goodcrc_ctrl_0 = AUTO_CTRL_MESSAGE_GOODCRC_MASK_CFG;
    PDSS->rx_default_sop_goodcrc_ctrl_1 = 0xFFFFFFFF;
#endif /*CCG3*/
    /*Configure RX_SOP_GOOG_CRC_EN_CTRL*/
    PDSS->rx_sop_good_crc_en_ctrl = RX_SOP_GOOD_CRC_EN_CTRL_CFG;
    PDSS->rx_order_set_ctrl = UFP_OS_CFG;
    PDSS->tx_ctrl |= CTRL_MSG_GOOD_CRC;

    /* Receive C-Connector configuration. */
    PDSS->rx_cc_0_cfg = RX_CC_CFG;

    /* Update the averaging logic */
    tmp = PDSS->debug_cc_1 & ~(PDSS_DEBUG_CC_1_NUM_PREAMBLE_AVG_MASK |
            PDSS_DEBUG_CC_1_NUM_TRANS_AVG_MASK);
    tmp |= (PDSS_NUM_PREAMBLE_AVG_VALUE << PDSS_DEBUG_CC_1_NUM_PREAMBLE_AVG_POS) |
            (PDSS_NUM_TRANS_AVG_VALUE << PDSS_DEBUG_CC_1_NUM_TRANS_AVG_POS);
    PDSS->debug_cc_1 = tmp;

    /* Set trim settings. */
#if (defined (CCG3))
    tmp = PDSS->s8usbpd_trim_6 & ~PDSS_S8USBPD_TRIM_6_V1P575_TRIM_MASK;
    tmp |= SILICON_TRIM6_V1P575_TRIM_VALUE;
    PDSS->s8usbpd_trim_6 = tmp;

    tmp = PDSS->s8usbpd_trim_3 & ~PDSS_S8USBPD_TRIM_3_V0P55_TRIM_MASK;
    tmp |= SILICON_TRIM3_V0P55_TRIM_VALUE;
    PDSS->s8usbpd_trim_3 = tmp;

    /*
     * Clearing of TX_TRIM field is enough for Rp = 1.5A or Default termination
     * at initialization time. Later TX_TRIM could be updated for Rp = 3A
     * termination.
     */
    PDSS->s8usbpd_trim_0 &= (~(PDSS_S8USBPD_TRIM_0_TX_TRIM_MASK));
#endif /* CCG3 */

    /* Register sync interrupt handler. */
    CyIntDisable(PD_PORT0_INTR0);
    (void)CyIntSetVector(PD_PORT0_INTR0, &pdss_phy_intr0_handler);
    CyIntEnable(PD_PORT0_INTR0);

    /* Clear receive interrupts. */
    PDSS->intr0 = RX_INTERRUPTS1;

    /* Enable receive interrupts. */
    PDSS->intr0_mask |= RX_INTERRUPTS1;

#if (VBUS_CTRL_TYPE != VBUS_CTRL_OPTO_FB)
    {
        PDSS->ea_ctrl |= PDSS_EA_CTRL_RES_DIV_BYPASS;
    }
#endif /* (VBUS_CTRL_TYPE != VBUS_CTRL_OPTO_FB) */
}

#if (defined(PAG1S) && (CCG_REG_SEC_CTRL != 0))
void pdss_phy_regulation_init(void)
{
    uint32_t tmp = 0;

    /* Enable shunt */
    PDSS->ea_ctrl |= PDSS_EA_CTRL_EN_SHNT | PDSS_EA_CTRL_SHNT_ST_OPAMP_ENB;

    PERI->pclk_ctl[PDSS_PORT0_PCLK_REFGEN_IDX] = PDSS_PORT0_REFGEN_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_SAR_IDX] = PDSS_PORT0_SAR_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_PASC_IDX] = PDSS_PORT0_PASC_CLK_DIV_ID;
    PERI->pclk_ctl[PDSS_PORT0_PCLK_VBTR_IDX] = PDSS_PORT0_VBTR_CLK_DIV_ID;

    /* IP enable. */
    PDSS->ctrl &= ~PDSS_CTRL_IP_ENABLED;
    PDSS->ctrl = PDSS_CTRL_IP_ENABLED;

    /* Turn off PHY deepsleep. References require 100us to stabilize. */
    PDSS->dpslp_ref_ctrl = ((PDSS->dpslp_ref_ctrl & ~PDSS_DPSLP_REF_CTRL_PD_DPSLP) |
            PDSS_DPSLP_REF_CTRL_IGEN_EN);

    /* Update Refgen setting */
    PDSS->refgen_0_ctrl &= ~PDSS_REFGEN_0_CTRL_REFGEN_PD;
    PDSS->refgen_0_ctrl  |= PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_SEL |
                          PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_MON_SEL;

    /* Enable EA to hold the current VBUS_IN voltage */
    tmp = PDSS->ea_ctrl & ~(PDSS_EA_CTRL_ISNK_EN | PDSS_EA_CTRL_ISRC_EN |
    PDSS_EA_CTRL_ISRC_DAC_CTRL_MASK | PDSS_EA_CTRL_ISNK_DAC_CTRL_MASK);
    PDSS->ea_ctrl = (tmp | PDSS_EA_CTRL_EN_CV | PDSS_EA_CTRL_ISRC_EN);
    REG_FIELD_UPDATE(PDSS->ea_ctrl, PDSS_EA_CTRL_ISRC_DAC_CTRL, PASC_VBUS_IN_STARTUP_SRC_EA);
 
    /* When VBUS_IN reaches 4V, start voltage regulation */
    while(pd_hal_measure_vbus_in(0) < PASC_VBUS_IN_STARTUP_THRES);
    
    /* Send 4 pulses to PAG1P to stop incrementing voltage */
    gpio_hsiom_set_config(PASC_PTDRV_GPIO, HSIOM_MODE_GPIO, GPIO_DM_STRONG, false);
    for (tmp = 0; tmp < PASC_PTDRV_PULSE_COUNT; tmp++)
    {
        gpio_set_value(PASC_PTDRV_GPIO, true);
        __asm("NOP\nNOP\nNOP\nNOP\nNOP\nNOP\nNOP\nNOP\nNOP\nNOP\nNOP\nNOP\n");
        gpio_set_value(PASC_PTDRV_GPIO, false);
        CyDelayUs(PASC_PTDRV_PULSE_OFF_TIME_US);
    }
    gpio_hsiom_set_config(PASC_PTDRV_GPIO, HSIOM_MODE_PTDRV_IN,
            GPIO_DM_STRONG, false);

    /* Enbale PASC to take over voltage control */
    pd_pasc_init();

    /* Set VBUS_IN to VSAFE_5V */
    vbus_set_vsafe(0);
}
#endif /* (defined(PAG1S) && (CCG_REG_SEC_CTRL != 0)) */

#if (SOURCE_BOOT)
#if (defined (CCG3))
void turn_on_vbus(void)
{
    if (pd_pctrl_drive == PD_FET_DR_P_JN_FET)
    {
        PDSS->ngdo_ctrl_0 |= (VBUS_P_PLDN_EN_LV_0|VBUS_P_PLDN_EN_LV_1);
    }
    else /* PD_FET_DR_N_JN_FET */
    {
        PDSS->ngdo_ctrl_0 &= ~(VBUS_P_PLDN_EN_LV_0 | VBUS_P_PLDN_EN_LV_1);
        PDSS->ngdo_ctrl_0 |= (VBUS_P_NGDO_EN_LV_0 | VBUS_P_NGDO_EN_LV_1);
        PDSS->ngdo_ctrl_c &= ~PDSS_NGDO_CTRL_C_RST_EDGE_DET;
    }
}

void turn_off_vbus(void)
{
    if (pd_pctrl_drive == PD_FET_DR_P_JN_FET)
    {
        PDSS->ngdo_ctrl_0 &= ~(VBUS_P_PLDN_EN_LV_0|VBUS_P_PLDN_EN_LV_1);
    }
    else /* PD_FET_DR_N_JN_FET */
    {
        PDSS->ngdo_ctrl_0 &= ~(VBUS_P_NGDO_EN_LV_0 | VBUS_P_NGDO_EN_LV_1);
        PDSS->ngdo_ctrl_0 |= (VBUS_P_PLDN_EN_LV_0 | VBUS_P_PLDN_EN_LV_1);
    }
}

void pd_internal_vbus_discharge_on(uint8_t port)
{
    PDSS->vbus_ctrl |= PDSS_VBUS_CTRL_DISCHG_EN;
}

void pd_internal_vbus_discharge_off(uint8_t port)
{
    PDSS->vbus_ctrl &= ~PDSS_VBUS_CTRL_DISCHG_EN;
}

#elif (defined(CCG3PA) || defined(CCG3PA2))
void turn_on_vbus(void)
{
    PDSS->pgdo_pu_1_cfg = (PDSS_PGDO_PU_1_CFG_SEL_ON_OFF |
        PDSS_PGDO_PU_1_CFG_PGDO_PU_EN_LV_ON_VALUE);
}

void turn_off_vbus(void)
{
     PDSS->pgdo_pu_1_cfg = PDSS_PGDO_PU_1_CFG_DEFAULT;
}

void pd_internal_vbus_discharge_on(uint8_t port)
{
    PDSS->dischg_shv_ctrl[0] = (PDSS_DISCHG_SHV_CTRL_DISCHG_EN |
            PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG |
            (CCG3PA_DISCHG_DS_VBUS_C << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS));
}

void pd_internal_vbus_discharge_off(uint8_t port)
{
    PDSS->dischg_shv_ctrl[0] &= ~PDSS_DISCHG_SHV_CTRL_DISCHG_EN;
}
#elif (defined(PAG1S))
void turn_on_vbus(void)
{
#if (CCG_REG_SEC_CTRL != 0)
    uint32_t vbus_in = 0;
    uint16_t timeout = 1000;

    /* 
     * In case of secondary side controlled regulation, we need to
     * ensure that VBUS_IN is in safe 5V range before turning on the
     * FET. If VBUS_IN is above 5.25V, try to discharge for 100ms.
     * If VBUS_IN has not dropped below the threshold within this time,
     * stop discharging and wait until VBUS_IN is safe for connect.
     */
    while(1)
    {
        vbus_in = pd_hal_measure_vbus_in(0);

        if (vbus_in > (uint32_t)apply_threshold(VSAFE_5V, 5))
        {
            if(timeout > 0)
            {
                pd_internal_vbus_in_discharge_on(0);
                timeout--;
            }
            else
            {
                pd_internal_vbus_in_discharge_off(0);
            }
            CyDelayUs(100);
        }
        else
        {
            pd_internal_vbus_in_discharge_off(0);
            break;
        }
    }
#endif /* (CCG_REG_SEC_CTRL != 0) */

    PDSS->ngdo_1_cfg = (PDSS_NGDO_1_CFG_SEL_ON_OFF |
        PDSS_NGDO_1_CFG_GDRV_EN_ON_VALUE);
}

void turn_off_vbus(void)
{
    PDSS->ngdo_1_cfg = PDSS_NGDO_1_CFG_DEFAULT;
}

void pd_internal_vbus_discharge_on(uint8_t port)
{
    (void)port;
    PDSS->dischg_shv_ctrl[0] |= PDSS_DISCHG_SHV_CTRL_DISCHG_EN;
}

void pd_internal_vbus_discharge_off(uint8_t port)
{
    (void)port;
    PDSS->dischg_shv_ctrl[0] &= ~PDSS_DISCHG_SHV_CTRL_DISCHG_EN;
}

void pd_internal_vbus_in_discharge_on(uint8_t port)
{
    (void)port;
#if VBUS_IN_DISCHARGE_EN
    /* This is VBUS_IN discharge path. Only discharge to 5V. */
    PDSS->dischg_shv_ctrl[1] &= ~(PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
    /* Enable the comparator. */
    PDSS->comp_ctrl[COMP_ID_VBUS_DISCHARGE] &= ~PDSS_COMP_CTRL_COMP_PD;
#endif /* VBUS_IN_DISCHARGE_EN */
}

void pd_internal_vbus_in_discharge_off(uint8_t port)
{
    (void)port;
#if VBUS_IN_DISCHARGE_EN
    PDSS->comp_ctrl[COMP_ID_VBUS_DISCHARGE] |= PDSS_COMP_CTRL_COMP_PD;
    PDSS->dischg_shv_ctrl[1] = ((PDSS->dischg_shv_ctrl[1] & ~PDSS_DISCHG_SHV_CTRL_DISCHG_EN) |
        PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
#endif /* VBUS_IN_DISCHARGE_EN */
}

uint16_t pd_hal_measure_vbus_in(uint8_t port)
{
    (void)port;
    uint16_t tmp = 0;

    /* Enable the ADC and power it down. Select REFGEN as source */
    PDSS->adc_ctrl = PDSS_ADC_CTRL_ADC_ISO_N | PDSS_ADC_CTRL_PD_LV;
    PDSS->adc_ctrl &= ~PDSS_ADC_CTRL_VREF_DAC_SEL;

    /* Select VBUS_IN for voltage measurement */
    PDSS->amux_nhvn_ctrl |= (1 << AMUX_ADC_PAG1S_VBUS_IN_8P_EN_POS);
    CyDelayUs(20);

    /* Sample Vbus voltage using ADC. */
    PDSS->intr0 = PDSS_INTR0_SAR_DONE;
    PDSS->adc_ctrl &= ~PDSS_ADC_CTRL_VREF_DAC_SEL;
    PDSS->adc_ctrl = PDSS_ADC_CTRL_ADC_ISO_N |
        ((APP_VBUS_POLL_ADC_INPUT << PDSS_ADC_CTRL_VSEL_POS) & PDSS_ADC_CTRL_VSEL_MASK);
    PDSS->adc_sar_ctrl |= PDSS_ADC_SAR_CTRL_SAR_EN;
    /* Wait for SAR done interrupt status or timeout. */
    while (((PDSS->intr0 & PDSS_INTR0_SAR_DONE) == 0) && (tmp < PD_ADC_TIMEOUT_COUNT))
    {
        tmp++;
    }
    /* Delay required between SAR_EN bit to be cleared and value to be loaded. */
    CyDelayUs(2);
    tmp = ((PDSS->adc_sar_ctrl & PDSS_ADC_SAR_CTRL_SAR_OUT_MASK) >> PDSS_ADC_SAR_CTRL_SAR_OUT_POS);
    PDSS->intr0 = PDSS_INTR0_SAR_DONE;

    /* Convert ADC sample to voltage level */
    tmp = ((tmp * MX_PD_ADC_REF_VOLT_MV * VBUS_MON_DIV_8P_VAL)/(PD_ADC_NUM_LEVELS << 1u));

    /* Revert AMUX */
    PDSS->amux_nhvn_ctrl &= ~(1 << AMUX_ADC_PAG1S_VBUS_IN_8P_EN_POS);

    return tmp;
}

/* 
 * The function assumes that the EA iDAC is set below 5V and does not 
 * require transition from high to low.
 */
void vbus_set_vsafe(uint8_t port)
{
    (void)port;
    uint32_t regval;

    /* Ensure EA feedback is enabled. */
    PDSS->ea_ctrl |= (PDSS_EA_CTRL_EN_CV | PDSS_EA_CTRL_ISRC_EN);

    /* Initialize VBTR and set initial and final values for sink IDAC */
    PDSS->vbtr_cfg = (PDSS_VBTR_CFG_SRC_EN);
    PDSS->vbtr_snk_init_fin_value = 0;
    regval = REG_FIELD_GET(PDSS->ea_ctrl, PDSS_EA_CTRL_ISRC_DAC_CTRL);
    PDSS->vbtr_src_init_fin_value = (regval << PDSS_VBTR_SRC_INIT_FIN_VALUE_SRC_INIT_POS);

    /* Clear VBTR status and start the operation */
    PDSS->intr8 = PDSS_INTR8_VBTR_OPR_DONE;
    PDSS->vbtr_ctrl |= PDSS_VBTR_CTRL_START;

    /* Wait untill operation is done */
    while((PDSS->intr8 & PDSS_INTR8_VBTR_OPR_DONE) == 0);

    /* Clear the iDAC settings to make it zero. */
    PDSS->ea_ctrl &= ~(PDSS_EA_CTRL_ISRC_DAC_CTRL_MASK | PDSS_EA_CTRL_ISNK_DAC_CTRL_MASK);

    /* Clear VBTR status */
    PDSS->intr8 = PDSS_INTR8_VBTR_OPR_DONE;
    PDSS->vbtr_ctrl &= ~PDSS_VBTR_CTRL_START;

    /* Leave the EA CV mode disabled. */
    PDSS->ea_ctrl &= ~(PDSS_EA_CTRL_EN_CV | PDSS_EA_CTRL_ISRC_EN | PDSS_EA_CTRL_ISNK_EN);
}
#endif /* CCGx */

void dr_swap_to_UFP(void)
{
    /*Send Accept message in response to DR_SWAP request and change data role to UFP */
    pd_send_ctl_msg(CTRL_MSG_ACCEPT);
    PDSS->tx_ctrl &= ~DATA_ROLE_DFP;
    gl_cur_port_type = PRT_TYPE_UFP;
}

#endif/* (SOURCE_BOOT) */

void typec_state_machine(void)
{
    if (gl_pd_state == IDLE)
    {
        volatile bool status;
        uint8_t i;

        for(i=0; i < 2; i++)
        {
            status = false;

#if (SOURCE_BOOT)
            if(i == 0)
            {
                /* Connect both Down and Up comparator to CC1*/
                PDSS->cc_ctrl_0 &= ~(
                        PDSS_CC_CTRL_0_CMP_DN_CC1V2 |
                        PDSS_CC_CTRL_0_CMP_UP_CC1V2
                        );
            }
            else
            {
                /* Connect both Down and Up comparator to CC2*/
                PDSS->cc_ctrl_0 |= (
                        PDSS_CC_CTRL_0_CMP_DN_CC1V2 |
                        PDSS_CC_CTRL_0_CMP_UP_CC1V2
                        );
            }

            /* Let comparator output stabilize. */
            CyDelayUs(10);

            /* Check if CC line is in 0.2 - 2.6 V range which indicates that Rd is connected. */
            if((PDSS->intr1_status & (PDSS_INTR1_STATUS_VCMP_DN_STATUS |
                            PDSS_INTR1_STATUS_VCMP_UP_STATUS))
                    == PDSS_INTR1_STATUS_VCMP_DN_STATUS)
            {
                status = true;
            }
#else
            if(i== 0)
            {
                /* Connect CC1 to UP Comparator. */
                PDSS->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_CMP_UP_CC1V2;
                CyDelayUs(10);

                /* Check status of UP comparaor. */
                status = PDSS->intr1_status & PDSS_INTR1_STATUS_VCMP_UP_STATUS;
            }
            else
            {
                /* Connect CC2 to UP Comparator. */
                PDSS->cc_ctrl_0 |= PDSS_CC_CTRL_0_CMP_UP_CC1V2;
                CyDelayUs(10);

                /* Check status of UP comparaor. */
                status = PDSS->intr1_status & PDSS_INTR1_STATUS_VCMP_UP_STATUS;
            }
#endif /* SOURCE_BOOT */

            /* Is CC voltage in connected range. */
            if(status)
            {
                gl_cc_count[i]++;
                /* Debounce the status. */
                if(gl_cc_count[i] > MAX_DEBOUNCE_ATTACH_COUNT)
                {
                    /* Store active CC id. */
                    gl_active_channel = i;
                    /* Reset debounce count. */
                    gl_cc_count[0] = 0;
                    gl_cc_count[1] = 0;
                    pd_reset_protocol();
                    /*Change PD state from IDLE to CONNECTED.*/
                    gl_pd_state = CONNECTED;

                    /* Select active CC line. */
                    PDSS->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_CC_1V2;
                    PDSS->cc_ctrl_0 |= (i << 2);

#if (SOURCE_BOOT)
                    turn_on_vbus();
#endif /*SOURCE_BOOT*/
                    return;
                }
            }
            else
            {
                /* Reset debounce count. */
                gl_cc_count[i] = 0;
            }
        }
    }
    else
    {
        /* Check if CC line is no longer in connected state. */
#if SOURCE_BOOT
        if(PDSS->intr1_status & PDSS_INTR1_STATUS_VCMP_UP_STATUS)
#else
        if (!(PDSS->intr1_status & PDSS_INTR1_STATUS_VCMP_UP_STATUS))
#endif /* SOURCE_BOOT */
        {
            gl_cc_count[gl_active_channel]++;
            /* Detach status debounce. */
            if(gl_cc_count[gl_active_channel] > MAX_DEBOUNCE_DETACH_COUNT)
            {
                gl_cc_count[0] = 0;
                gl_cc_count[1] = 0;
#if (SOURCE_BOOT)
                turn_off_vbus();
#else /* SINK */
                /*
                 * In power bank case, if device is powered by external VDDD, VDDD gets 
                 * shorted to VBUS_IN line. This shall result connecting VDDD to the
                 * Type-C VBUS line. This also includes cases where we start as dead
                 * dead battery device and then get charged. So if any time VBUS has to
                 * be removed in course of PD / Type-C state machine, ensure that internal 
                 * VBUS regulator is disabled. In event of dead battery, this shall lead
                 * to device reset. This is the safest recovery path. CDT 276535.
                 * TODO: Check and disable only if it is enabled. This can be done inside
                 * the API or outside based on other call usage model.
                 * Also, this code can be removed if the VBATT monitoring can be done
                 * continously. But this code can still be in place to avoid any corner
                 * case handling.
                 */
                PDSS->vreg_vsys_ctrl &= ~(PDSS_VREG_VSYS_CTRL_VREG_EN);
#endif /* (SOURCE_BOOT) */
                gl_pd_state = IDLE;
                return;
            }
        }
        else
        {
            gl_cc_count[gl_active_channel] = 0;
        }
    }
}

void pd_state_machine(void)
{
    uint32_t msg_type ;
    uint32_t count ;
    pd_do_t* d_obj;
    uint8_t no_of_vdo=0;
    pd_do_t *vdm_resp;
    uvdm_response_state_t response_state = UVDM_NOT_HANDLED;
    uint8_t intr_state;

    if(gl_pd_event == PE_EVT_HARD_RESET_RCVD)
    {
        /*Clear PD event*/
        intr_state = CyEnterCriticalSection();
        gl_pd_event = 0;
        CyExitCriticalSection(intr_state);
        pd_reset_protocol();

#if (SOURCE_BOOT)
        /*Turn off VBUS. Wait for VBUS to discharge and then turn on VBUS*/
        turn_off_vbus();
        pd_internal_vbus_discharge_on(0);
        CyDelay(VBUS_DISCHARGE_TIME);
        pd_internal_vbus_discharge_off(0);
        turn_on_vbus();

        /*Update PD state to CONNECTED*/
        gl_pd_state = CONNECTED;
#endif /* SOURCE_BOOT */
    }
    else if(gl_pd_event == PE_EVT_PKT_RCVD)
    {
        /*Clear PD event*/
        intr_state = CyEnterCriticalSection();
        gl_pd_event = 0;
        CyExitCriticalSection(intr_state);

        msg_type = GET_PD_HDR_TYPE(gl_rcvd_pkt[PD_HDR_IDX]);
        count = GET_PD_HDR_CNT(gl_rcvd_pkt[PD_HDR_IDX]);
        /* Data Message */
        if(count != 0)
        {
#if (SOURCE_BOOT)
            /* Handle RDO request*/
            if(msg_type == DATA_MSG_REQUEST)
            {
                /* Send Accept message in reponse to RDO*/
                pd_send_ctl_msg(CTRL_MSG_ACCEPT);
                CyDelay(50);
                /* Send PS_RDY message */
                pd_send_ctl_msg(CTRL_MSG_PS_RDY);

                /* Change PD state from SEND_SRC_CAP to CONTRACT_ESTD */
                gl_pd_state = CONTRACT_ESTD;
            }
#else /* Sink Bootloader*/
            /* Send RDO after receving SRC_CAP message*/
            if(msg_type == DATA_MSG_SRC_CAP)
            {
                dobject[0].val = 0x10000000;
                pd_send_data_msg(DATA_MSG_REQUEST, dobject,1);
            }
#endif /* SOURCE_BOOT */
            if(msg_type == DATA_MSG_VDM)
            {
#if (SOURCE_BOOT)
                /* if datarole ==  DFP mode,  ignore vdm */
                if(gl_cur_port_type == PRT_TYPE_DFP)
                {
                    return;
                }
#endif /*SOURCE_BOOT*/
                d_obj = (pd_do_t*)(&(gl_rcvd_pkt[1]));
                if(d_obj->std_vdm_hdr.vdm_type == true)
                {
                    /* Handle Structured VDMs*/
                    switch(d_obj->std_vdm_hdr.cmd)
                    {
                        case VDM_CMD_DSC_IDENTITY:
                            dobject[0].val = SVDM_DSC_ID_HEADER;
                            dobject[1].val = SVDM_DSC_ID_HEADER_VDO;
                            dobject[2].val = 0;
                            dobject[3].val = SVDM_DSC_ID_PRODUCT_VDO;
                            pd_send_data_msg(DATA_MSG_VDM, dobject, 4);
                            break;
                        case VDM_CMD_DSC_SVIDS:
                            dobject[0].val = SVDM_DSC_SVID_HEADER;
                            dobject[1].val = SVDM_DSC_SVID_VDO1;
                            pd_send_data_msg(DATA_MSG_VDM, dobject, 2);
                            break;
                        case VDM_CMD_DSC_MODES:
                            dobject[0].val = SVDM_DSC_MODE_HEADER;
                            dobject[1].val = SVDM_DSC_MODE_VDO;
                            pd_send_data_msg(DATA_MSG_VDM, dobject, 2);
                            break;
                        case VDM_CMD_ENTER_MODE:
                            dobject[0].val = SVDM_ENTER_MODE_HEADER;
                            pd_send_data_msg(DATA_MSG_VDM, dobject, 1);
                            break;
                    }
                }
                else
                {
                    /* Handle Un-structured VDMs*/
                    response_state = uvdm_handle_cmd (gl_rcvd_pkt, &vdm_resp, &no_of_vdo, NULL);
                    /* Respond with UVDM response. */
                    if ((response_state == UVDM_HANDLED_RESPONSE_READY) && (no_of_vdo))
                    {
                        pd_send_data_msg(DATA_MSG_VDM, vdm_resp, no_of_vdo);
                    }
                    /* Respond with NAK if UVDM not recognized and VID is CY VID. */
                    else if (response_state == UVDM_NOT_HANDLED)
                    {
                        /* Check if header has CY VID. */
                        if ((gl_rcvd_pkt[1] >> 16) == CY_VID)
                        {
                            dobject[0].val = gl_rcvd_pkt[1];
                            /* Set NAK . */
                            dobject[0].ustd_vdm_hdr.cmd_type = CMD_TYPE_RESP_NAK;
                            pd_send_data_msg (DATA_MSG_VDM, dobject, 1);
                        }
                    }
                }
            }
        }
#if SOURCE_BOOT
        /* Control Message*/
        else
        {
            if(msg_type == CTRL_MSG_DR_SWAP)
            {
                /*Handle DR_SWAP request only when PD contract is established otherwise ignore the request*/
                if(gl_pd_state == CONTRACT_ESTD)
                {
                    /*Handle DR_Swap only if current data role is DFP */
                    if(gl_cur_port_type == PRT_TYPE_UFP)
                    {
                        pd_send_ctl_msg (CTRL_MSG_REJECT);
                    }
                    else
                    {
                        dr_swap_to_UFP();
                    }
                }
            }
        }
#endif /*SOURCE_BOOT*/
    }

#if SOURCE_BOOT
    if (gl_pd_state == CONNECTED)
    {
        /* After receiving hard request or detecting sink connection, Source bootloader enters into CONNECTED state
         * and its data role is set to DFP.*/
        PDSS->tx_ctrl |= DATA_ROLE_DFP;
        gl_cur_port_type = PRT_TYPE_DFP;
        /* Move to SEND_SRC_CAP state */
        gl_pd_state = SEND_SRC_CAP;
        /*Start systick timer with timeout 180 ms*/
        start_systick_timer (180);
    }
    else if (gl_pd_state == SEND_SRC_CAP)
    {
        /* When systick timer expires, Send SRC_CAP Message with 5V pdo and restart the timer with 180ms.
         * Bootloader sends SRC_CAP message after every 180 ms until it receives RDO request. */
        if(CM0->syst_csr & CM0_SYST_CSR_COUNTFLAG)
        {
            pd_send_data_msg (DATA_MSG_SRC_CAP, src_pdo, 1);
            start_systick_timer (180);
        }
    }
#endif /*SOURCE_BOOT*/
}
