/**
 ****************************************************************************************
 * @file    dbg_define.h
 * @brief   
 
 * @version 0.01
 * @date    2018/11/13
 * @history 
 * @note	   
 * detailed description 
 
 ****************************************************************************************
 * @attention
 *
 * Copyright (C) 2016 Shanghai Panchip Microelectronics Co., Ltd. All rights reserved.
 ****************************************************************************************
 */


#ifndef __DBG_DEFINE_H__
#define __DBG_DEFINE_H__

// base
#define DBG_LLD_GROUP                   0xD000

#define DBG_LLD_COMMON_INDEX            (DBG_LLD_GROUP + 0x00)
#define DBG_LLD_RESTART_POL             (DBG_LLD_COMMON_INDEX + 0)
#define DBG_LLD_ANCHOR_POINT_BASETIME_CNT       (DBG_LLD_COMMON_INDEX + 1)
#define DBG_LLD_ANCHOR_POINT_FINETIME_CNT       (DBG_LLD_COMMON_INDEX + 2)
#define DBG_LLD_HANDLE                  (DBG_LLD_COMMON_INDEX + 3)
#define DBG_LLD_INTERVAL                (DBG_LLD_COMMON_INDEX + 4)
#define DBG_LLD_WIN_SIZE                (DBG_LLD_COMMON_INDEX + 5)
#define DBG_LLD_SLEEP_COUNT             (DBG_LLD_COMMON_INDEX + 6)
#define DBG_LLD_SLEEP_TIME              (DBG_LLD_COMMON_INDEX + 7)

// connection paramter
#define DBG_LLD_CONNECTION_PARAMTERS_INDEX      (DBG_LLD_GROUP + 0x10)
#define DBG_LLD_EVENT_COUNT             (DBG_LLD_CONNECTION_PARAMTERS_INDEX + 0)
#define DBG_LLD_CHANNEL                 (DBG_LLD_CONNECTION_PARAMTERS_INDEX + 1)
#define DBG_LLD_MISSCNT                 (DBG_LLD_CONNECTION_PARAMTERS_INDEX + 2)
#define DBG_LLD_LATENCY                 (DBG_LLD_CONNECTION_PARAMTERS_INDEX + 3)
#define DBG_LLD_TIMEOUT                 (DBG_LLD_CONNECTION_PARAMTERS_INDEX + 4)
#define DBG_LLD_MASTER_SCA              (DBG_LLD_CONNECTION_PARAMTERS_INDEX + 5)
#define DBG_LLD_PEER_ADDR               (DBG_LLD_CONNECTION_PARAMTERS_INDEX + 6)
#define DBG_LLD_CHANNEL_MAP             (DBG_LLD_CONNECTION_PARAMTERS_INDEX + 7)
#define DBG_LLD_DISCONNECT_REASON       (DBG_LLD_CONNECTION_PARAMTERS_INDEX + 8)
#define DBG_LLD_EVT_WAITING             (DBG_LLD_CONNECTION_PARAMTERS_INDEX + 9)

// instant
#define DBG_LLD_INSTANT_INDEX           (DBG_LLD_GROUP + 0x20)
#define DBG_LLD_INSTANT_ACTION          (DBG_LLD_INSTANT_INDEX + 0)
#define DBG_LLD_INSTANT                 (DBG_LLD_INSTANT_INDEX + 1)


// test
#define DBG_LLD_TEST_INDEX              (DBG_LLD_GROUP + 0x80)
#define DBG_LLD_RX_WIN_USED             (DBG_LLD_TEST_INDEX + 0)
#define DBG_LLD_32K_RC_FREQ_CNT         (DBG_LLD_TEST_INDEX + 1)
#define DBG_IRQ_TYPE                    (DBG_LLD_TEST_INDEX + 2)    // tIRQ_TYPE @ sim.h
#define DBG_BLE_EVENT_SCHEDULE          (DBG_LLD_TEST_INDEX + 3)    // enum KE_EVENT_TYPE @ panip_config.h, (0x01 << KE_EVENT_TYPE)
#define DBG_FLAG_PREVENT_SLEEP          (DBG_LLD_TEST_INDEX + 4)    // enum prevent_sleep @ rwip.h

#define DBG_IRQ_ERROR_TYPE              (DBG_LLD_TEST_INDEX + 5)
#define DBG_SLEEP_CLOCK_ACCURACY        (DBG_LLD_TEST_INDEX + 6)
#define DBG_SLEEP_CLOCK_DRIFT           (DBG_LLD_TEST_INDEX + 7)
#define DBG_MISSCNT_CALCULATION         (DBG_LLD_TEST_INDEX + 8)
#define DBG_LATENCY_CALCULATION         (DBG_LLD_TEST_INDEX + 9)

// ble packet
#define DBG_BLE_PACKET                  (DBG_LLD_GROUP + 0x90)
#define DBG_BLE_RX_CNT                  (DBG_BLE_PACKET + 0)
// BLE_BDADDR_MATCH_BIT     0x0080
// BLE_NESN_ERR_BIT         0x0040
// BLE_SN_ERR_BIT           0x0020
// BLE_MIC_ERR_BIT          0x0010
// BLE_CRC_ERR_BIT          0x0008
// BLE_LEN_ERR_BIT          0x0004
// BLE_TYPE_ERR_BIT         0x0002
// BLE_SYNC_ERR_BIT         0x0001
#define DBG_BLE_RX_STATUS               (DBG_BLE_PACKET + 1)
// LLID_RFU                 0
// LLID_CONTINUE            1
// LLID_START               2
// LLID_CNTL                3
#define DBG_BLE_RX_TYPE                 (DBG_BLE_PACKET + 2)

#define DBG_BLE_RX_LENGTH               (DBG_BLE_PACKET + 3)

// LL_CONNECTION_UPDATE_REQ 0x00
// LL_CHANNEL_MAP_REQ       0x01
// LL_TERMINATE_IND         0x02
// LL_ENC_REQ               0x03
// LL_ENC_RSP               0x04
// LL_START_ENC_REQ         0x05
// LL_START_ENC_RSP         0x06
// LL_UNKNOWN_RSP           0x07
// LL_FEATURE_REQ           0x08
// LL_FEATURE_RSP           0x09
// LL_PAUSE_ENC_REQ         0x0A
// LL_PAUSE_ENC_RSP         0x0B
// LL_VERSION_IND           0x0C
// LL_REJECT_IND            0x0D
#define DBG_BLE_CNTL_OPCODE             (DBG_BLE_PACKET + 4)

#define DBG_BLE_RSSI                    (DBG_BLE_PACKET + 5)


#define DBG_PERIPHERALS_GROUP           0xDE00
#define DBG_PERI_ADC_TEMP_RESULT_RAW    (DBG_PERIPHERALS_GROUP + 0)
#define DBG_PERI_ADC_TEMP_RESULT        (DBG_PERIPHERALS_GROUP + 1)
#define DBG_PERI_ADC_TEMP_AVG           (DBG_PERIPHERALS_GROUP + 2)
#define DBG_PERI_ADC_TEMP_BACKUP        (DBG_PERIPHERALS_GROUP + 3)
#define DBG_PERI_32K_RC_FREQ_CNT        (DBG_PERIPHERALS_GROUP + 4)

#define DBG_RF_CALIB_GROUP              0xDE10
#define DBG_RF_CALIB_TIMES              (DBG_RF_CALIB_GROUP + 0x00)
#define DBG_RF_CALIB_DA_GAIN            (DBG_RF_CALIB_GROUP + 0x01)
#define DBG_RF_CALIB_DA_VREF_LB         (DBG_RF_CALIB_GROUP + 0x02)
#define DBG_RF_CALIB_DA_VREF_MB         (DBG_RF_CALIB_GROUP + 0x03)
#define DBG_RF_CALIB_DA_RESULT          (DBG_RF_CALIB_GROUP + 0x04)


#define DBG_SYSTEM_GROUP                0xDF00
#define DBG_SYSTEM_HARDFAULT            (DBG_SYSTEM_GROUP + 0)
#define DBG_SYSTEM_ASSERT_ERROR         (DBG_SYSTEM_GROUP + 1)

#endif /* __DBG_DEFINE_H__ */

