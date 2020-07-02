/**
 * @file timer_id.h
 *
 * @brief @{Soft timer identifier definitions.@}
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

#ifndef _TIMER_ID_H_
#define _TIMER_ID_H_

#include <stdint.h>

/**
 * @typedef ccg_timer_id
 * @brief List of soft timer IDs defined for various applications. Values of the timer
 * IDs should not be changed. Timer IDs from USER_TIMERS_START_ID can be used by user
 * code.
 */
typedef enum ccg_timer_id {
    PD_TIMERS_START_ID            = 0,
    /**< 000: Start index for USB-PD stack timers. */

    PD_CABLE_TIMER,
    /**< 001: Timer used for cable capability check. */

    PD_NO_RESPONSE_TIMER,
    /**< 002: Response timer. */

    PD_CBL_DSC_ID_TIMER,
    /**< 003: Timer used for cable discovery state machine. */

    PD_CBL_DELAY_TIMER,
    /**< 004: Timer used to enforce cable delay. */

    PD_PHY_BUSY_TIMER,
    /**< 005: Timer used to handle PHY busy status. */

    PD_GOOD_CRC_TX_TIMER,
    /**< 006: GoodCRC timer. */

    PD_HARD_RESET_TX_TIMER,
    /**< 007: Hard reset transmit timer. */

    PD_VCONN_SWAP_INITIATOR_TIMER,
    /**< 008: VConn swap initiator timer. */

    PD_GENERIC_TIMER,
    /**< 009: Generic AMS timer. */

    PD_PPS_TIMER,
    /**< 010: PPS related timer. */

    PD_SINK_TX_TIMER,
    /**< 011: PD 3.0 sink Rp flow control timer. */

    PD_TIMER_RESERVED_12,
    /**< 012: Reserved for future use. */

    PD_TIMER_RESERVED_13,
    /**< 013: Reserved for future use. */

    PD_TIMERS_END_ID,
    /**< 014: End index (inclusive) for USB-PD stack timers. */

    TYPEC_TIMERS_START_ID         = 15u,
    /**< 015: Start index for Type-C timers. */

    TYPEC_GENERIC_TIMER2          = 15u,
    /**< 015: Generic Type-C state machine timer #2. */

    TYPEC_GENERIC_TIMER1,
    /**< 016: Generic Type-C state machine timer #1. */

    TYPEC_CC1_DEBOUNCE_TIMER,
    /**< 017: Timer used for CC1 debounce. */

    TYPEC_CC2_DEBOUNCE_TIMER,
    /**< 018: Timer used for CC2 debounce. */

    TYPEC_RD_DEBOUNCE_TIMER,
    /**< 019: Timer used for Rd debounce. */

    TYPEC_VBUS_DISCHARGE_TIMER,
    /**< 020: VBus discharge timer id. */

    TYPEC_ACTIVITY_TIMER,
    /**< 021: Type-C activity timer id. */

    TYPEC_RP_CHANGE_TIMER,
    /**< 022: Timer used to trigger current limit update after Rp change. */

    TYPEC_TIMER_RESERVED_23,
    /**< 023: Reserved for future use. */

    TYPEC_TIMERS_END_ID,
    /**< 024: End index (inclusive) for Type-C timers. */

    PD_OCP_DEBOUNCE_TIMER,
    /**< 025: Timer used for FW debounce of VBus OCP. */

    HPD_RX_ACTIVITY_TIMER_ID,
    /**< 026: Timer used for HPD receive handling. */

    PD_VCONN_OCP_DEBOUNCE_TIMER,
    /**< 027: Timer used for FW debounce of VConn OCP. */

    PD_TIMER_RESERVED_28,
    /**< 028: Reserved for future use. */

    VBUS_DISCHARGE_SCHEDULE_TIMER,
    /**< 029: Timer used to monitor VBus discharge operation. */

    APP_TIMERS_START_ID = 30u,
    /**< 030: Start index for Application level timers. */

    APP_PSOURCE_EN_TIMER = 30u,
    /**< 030: Timer used to ensure timely completion of power source enable operation. */

    APP_PSOURCE_EN_MONITOR_TIMER,
    /**< 031: Timer used to monitor voltage during power source enable operation. */

    APP_PSOURCE_EN_HYS_TIMER,
    /**< 032: Timer used to add hysteresis at the end of a power source enable operation. */

    APP_PSOURCE_DIS_TIMER,
    /**< 033: Timer used to ensure timely completion of power source disable operation. */

    APP_PSOURCE_DIS_MONITOR_TIMER,
    /**< 034: Timer used to monitor voltage during power source disable operation. */

    APP_PSOURCE_CF_TIMER,
    /**< 035: Power source Current foldback restart timer ID. */

    APP_PSOURCE_DIS_EXT_DIS_TIMER,
    /**< 036: Timer used to discharge VBus for some extra time at the end of a power source disable operation. */

    APP_DB_SNK_FET_DIS_DELAY_TIMER,
    /**< 037: Dead battery Sink Fet disable delay timer. */

    APP_PSINK_DIS_TIMER,
    /**< 038: Timer used to ensure timely completion of power sink disable operation. */

    APP_PSINK_DIS_MONITOR_TIMER,
    /**< 039: Timer used to monitor voltage during power sink disable operation. */

    APP_VDM_BUSY_TIMER,
    /**< 040: Timer used to delay retry of VDMs due to BUSY responses or errors. */

    APP_AME_TIMEOUT_TIMER,
    /**< 041: Timer used to implement AME timeout. */

    APP_VBUS_OCP_OFF_TIMER,
    /**< 042: Timer used to disable VBus supply after OC fault. */

    APP_VBUS_OVP_OFF_TIMER,
    /**< 043: Timer used to disable VBus supply after OV fault. */

    APP_VBUS_UVP_OFF_TIMER,
    /**< 044: Timer used to disable VBus supply after UV fault. */

    APP_VBUS_SCP_OFF_TIMER,
    /**< 045: Timer used to disable VBus supply after SC fault. */

    APP_FAULT_RECOVERY_TIMER,
    /**< 046: App timer used to delay port enable after detecting a fault. */

    APP_SBU_DELAYED_CONNECT_TIMER,
    /**< 047: Timer used to do delayed SBU connection in Thunderbolt mode. */

    APP_MUX_DELAY_TIMER,
    /**< 048: Timer used to delay VDM response. */

    APP_MUX_POLL_TIMER,
    /**< 049: Timer used to MUX status. */

    APP_CBL_DISC_TRIGGER_TIMER,
    /**< 050: Timer used to trigger cable discovery after a V5V supply change. */

    APP_V5V_CHANGE_DEBOUNCE_TIMER,
    /**< 051: Timer used to debounce V5V voltage changes. */

    APP_UFP_RECOV_VCONN_SWAP_TIMER,
    /**< 052: Timer used to run Vconn swap after V5V was lost and recovered while UFP. */

    APP_OT_DETECTION_TIMER,
    /**< 053: Timer used to call OT measurment handler. */

    APP_PASC_VALLEY_TIMER_ID,
    /**< Timer used to run PASC update task for valley algorithm. */

    APP_PASC_TASK_TIMER_ID,
    /**< Timer used to run PASC tasks periodically. */

    APP_TIMER_RESERVED_56,
    /**< 056: Timer ID reserved for future use. */

    APP_TIMER_RESERVED_57,
    /**< 057: Timer ID reserved for future use. */

    APP_TIMER_RESERVED_58,
    /**< 058: Timer ID reserved for future use. */

    APP_TIMER_RESERVED_59,
    /**< 059: Timer ID reserved for future use. */

    APP_BB_ON_TIMER,
    /**< 060: Timer used to provide delay between disabling the Billboard device and re-enabling it. */

    APP_BB_OFF_TIMER,
    /**< 061: Timer used to display USB billboard interface to save power. */

    APP_TIMER_RESERVED_62,
    /**< 062: Timer ID reserved for future use. */

    APP_INITIATE_SWAP_TIMER,
    /**< 063: Timer used to initiate SWAP operations in DRP applications with a power/data role preference. */

    APP_INITIATE_SEND_IRQ_CLEAR_ACK,
    /**< 064: Timer used to initiate Virtual HPD IRQ CLEAR ACK to the Thunderbolt Controller. */

    UCSI_CONNECT_EVENT_TIMER,
    /**< 065: UCSI Connect Event timer. This timer is used to Signal Connect event to OPM. */

    APP_VDM_NOT_SUPPORT_RESP_TIMER_ID,
    /**< 066: VDM Not supported response timer. */

    UCSI_DPM_RETRY_TIMER,
    /**< 067: Timer ID used to retry DPM commands initiated by UCSI layer. */

    APP_TIMER_RESERVED_68,
    /**< 068: Timer ID reserved for future use. */

    APP_TIMER_RESERVED_69,
    /**< 069: Timer ID reserved for future use. */

    APP_BC_TIMERS_START_ID,
    /**< 070: Start of Battery Charging State Machine timers. */

    APP_BC_GENERIC_TIMER1,
    /**< 071: Generic timer #1 used by the BC state machine. */

    APP_BC_GENERIC_TIMER2,
    /**< 072: Generic timer #2 used by the BC state machine. */

    APP_BC_DP_DM_DEBOUNCE_TIMER,
    /**< 073: Timer used to debounce voltage changes on DP and DM pins. */

    APP_BC_DETACH_DETECT_TIMER,
    /**< 074: Timer used to detect detach of a BC 1.2 sink while functioning as a CDP. */

    APP_TIMER_RESERVED_75,
    /**< 075: Timer ID reserved for future use. */

    APP_TIMER_RESERVED_76,
    /**< 076: Timer ID reserved for future use. */

    APP_TIMER_RESERVED_77,
    /**< 077: Timer ID reserved for future use. */

    APP_TIMER_RESERVED_78,
    /**< 078: Timer ID reserved for future use. */

    APP_TIMER_RESERVED_79,
    /**< 079: Timer ID reserved for future use. */

    APP_TIMER_RESERVED_80,
    /**< 080: Timer ID reserved for future use. */

    CCG_ACTIVITY_TIMER_ID,
    /**< 081: CCG activity timer ID. */

    OTP_DEBOUNCE_TIMER_ID,
    /**< 082: OTP debounce timer ID. */

    TYPE_A_CUR_SENSE_TIMER_ID,
    /**< 083: TYPE-A current sense timer ID. */

    TYPE_A_REG_SWITCH_TIMER_ID,
    /**< 084: TYPE-A regulator switch timer ID. */

    TYPE_A_PWM_STEP_TIMER_ID,
    /**< 085: TYPE-A port PWM step change timer. */

    PB_DEBOUNCE_TIMER_ID,
    /**< 086: Power Bank debounce timer ID. */

    APP_PSOURCE_VBUS_SET_TIMER_ID,
    /**< 087: Power source VBUS set timer ID */

    THROTTLE_TIMER_ID,
    /**< 088: Power Throttling timer ID. */

    THROTTLE_WAIT_FOR_PD_TIMER_ID,
    /**< 089: Power Throttling timer ID. */

    APP_BAD_SINK_TIMEOUT_TIMER,
    /**< 090: PD bad sink timeout timer ID. */

    APP_FET_SOFT_START_TIMER_ID,
    /**< 091: Timer used to control soft turn-on of power FET gate drivers. */

    APP_PPS_SNK_RECONTRACT_TIMER_ID,
    /**< 092: Timer used by PPS sinks to send periodic request messages. */

    APP_PSOURCE_SAFE_FET_ON_MONITOR_TIMER_ID,
    /**< 093: Timer to monitor voltage during FET On operation. */

    APP_PSOURCE_SAFE_FET_ON_TIMER_ID,
    /**< 094: Timeout timer to set safe voltage during FET On operation. */

    APP_TIMER_RESERVED_95,              /**< 095: Timer ID reserved for future use. */
    APP_TIMER_RESERVED_96,              /**< 096: Timer ID reserved for future use. */

    CCG_LS_MASTER_PORT_DEBOUNCE_TIMER_ID,
    /**< 097: Macro defines Master Debounce Timer ID. */

    CCG_LS_SLAVE_PORT_DEBOUNCE_TIMER_ID,
    /**< 098: Macro defines Slave Debounce Timer ID. */

    CCG_LS_MASTER_WRITE_TIMER_ID,
    /**< 099: Macro defines Master Write Timer ID. */

    CCG_LS_HEART_BEAT_TIMER_ID,
    /**< 100: Macro defines Heart Beat Timer ID. */

    USB_SUSPEND_TIMER,
    /**< 101: This timer is used to check for USB bus suspend condition. */

    USB_REMOTE_WAKEUP_TIMER,
    /**< 102: This timer is used to signal remote wakeup. */

    RIDGE_INIT_HPD_DEQUEUE_TIMER_ID,
    /**< 103: Timer for initiating virtual HPD dequeue. */

    APP_TIMER_RESERVED_104,             /**< 104: Timer ID reserved for future use. */
    APP_TIMER_RESERVED_105,             /**< 105: Timer ID reserved for future use. */
    APP_TIMER_RESERVED_106,             /**< 106: Timer ID reserved for future use. */
    APP_TIMER_RESERVED_107,             /**< 107: Timer ID reserved for future use. */
    APP_TIMER_RESERVED_108,             /**< 108: Timer ID reserved for future use. */
    APP_TIMER_RESERVED_109,             /**< 109: Timer ID reserved for future use. */

    HPI_PD_CMD_TIMER,
    /**< 110: Timer ID used for HPI command retry checks. */

    I2C_SLAVE_SCB0_TIMER,
    /**< 111: I2C transfer timeout for SCB0. */

    I2C_SLAVE_SCB1_TIMER,
    /**< 112: I2C transfer timeout for SCB0. */

    I2C_SLAVE_SCB2_TIMER,
    /**< 113: I2C transfer timeout for SCB0. */

    I2C_SLAVE_SCB3_TIMER,
    /**< 114: I2C transfer timeout for SCB0. */

    APP_TIMER_RESERVED_115,             /**< 115: Timer ID reserved for future use. */
    APP_TIMER_RESERVED_116,             /**< 116: Timer ID reserved for future use. */
    APP_TIMER_RESERVED_117,             /**< 117: Timer ID reserved for future use. */
    APP_TIMER_RESERVED_118,             /**< 118: Timer ID reserved for future use. */
    APP_TIMER_RESERVED_119,             /**< 119: Timer ID reserved for future use. */
    APP_TIMER_RESERVED_120,             /**< 120: Timer ID reserved for future use. */
    APP_TIMER_RESERVED_121,             /**< 121: Timer ID reserved for future use. */
    APP_TIMER_RESERVED_122,             /**< 122: Timer ID reserved for future use. */
    APP_TIMER_RESERVED_123,             /**< 123: Timer ID reserved for future use. */
    APP_TIMER_RESERVED_124,             /**< 124: Timer ID reserved for future use. */
    APP_TIMER_RESERVED_125,             /**< 125: Timer ID reserved for future use. */
    APP_TIMER_RESERVED_126,             /**< 126: Timer ID reserved for future use. */
    APP_TIMER_RESERVED_127,             /**< 127: Timer ID reserved for future use. */

    USER_TIMERS_START_ID = 128,          /**< 128: Start of timer IDs left for generic solution level usage. */
    
    APP_RETIMER_DISABLE_WAIT_TIMER      = 128,                 
    /**< 128: This timer is used to delay retimer disable. */
	
    APP_AUTO_DR_SWAP_TIMER,                          	
    /**< 129: This timer is used to trigger DR_Swap after PD Contract. */
	
    ICL_VSYS_STABLE_TIMER,                           
    /**< 130: This timer is let VSYS turn-on change to stablilze. */

    TBT_MODE_EXIT_CHECK_TIMER,                       	
    /**< 131: This timer is used to periodically check if TBT mode should be exited. */
	
    ICL_MUX_STATE_CHANGE_TIMER,
    /**< 132: This timer is used to check ICL mux state changes. */
	
    ICL_ADP_DETECT_DEB_TIMER,
    /**< 133: This timer is used debounce barrel state change. */
	
    BB_DBR_DEBUG_POLL_TIMER,
    /**< 134: This timer is used to retry reading the DEBUG register from the retimer. */
	
    BB_DBR_WAIT_TIMER,
    /**< 135: This timer is used to delay the retimer to start accepting the state change requests. */    
    
    APP_HPD_DELAY_TIMER,
    /**< 136: This timer is used to delay HPD events. */
} ccg_timer_id_t;

#endif /* _TIMER_ID_H_ */

/*[]*/
