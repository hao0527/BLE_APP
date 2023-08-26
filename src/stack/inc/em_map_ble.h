
#ifndef EM_MAP_BLE_H_
#define EM_MAP_BLE_H_

/**
 ****************************************************************************************
 * @addtogroup EM EM
 * @ingroup CONTROLLER
 * @brief Mapping of the different descriptors in the exchange memory
 *
 * @{
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "panip_config.h"                // stack configuration


#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "co_bt.h"
#include "co_buf.h"
#include "_reg_ble_em_tx.h"
#include "_reg_ble_em_txe.h"
#include "_reg_ble_em_rx.h"
#include "_reg_ble_em_wpb.h"
#include "_reg_ble_em_wpv.h"
#include "_reg_ble_em_et.h"
#include "_reg_ble_em_cs.h"

/*
 * DEFINES
 ****************************************************************************************
 */
/// number of entries for the exchange table
#define EM_BLE_EXCH_TABLE_LEN      16

/// number of frequencies / Depends on RF target
#if defined(CFG_RF_ATLAS)
  /// IcyTRx requires 40 x 32-bit words for Frequency table + 40 byte for VCO sub-band table
  #define EM_BLE_FREQ_TABLE_LEN  160
  #define EM_BLE_VCO_TABLE_LEN 40
#else
  /// Ripple/ExtRC requires 40 x 8-bit words for Frequency table / No VCO sub-band table
  #define EM_BLE_FREQ_TABLE_LEN  40
  #define EM_BLE_VCO_TABLE_LEN 0
#endif



/// Size of the encryption area
#define EM_BLE_ENC_LEN         16

/// number of control structure entries for the exchange table
#define EM_BLE_CS_COUNT        (BLE_CONNECTION_MAX + 1)

/// number of control structure entries for the exchange table
#define EM_BLE_TXE_COUNT       (BLE_CONNECTION_MAX)

/// Status of the control structure in the exchange table
enum
{
    EM_BLE_ET_READY,
    EM_BLE_ET_PROCESSED,
    EM_BLE_ET_RESERVED,
    EM_BLE_ET_BYPASSED
};

/*
 * Mapping of the different elements in EM
 ****************************************************************************************
 */
/// Offset of the exchange table
#define EM_BLE_ET_OFFSET         0
/// Offset of the frequency table
#define EM_BLE_FT_OFFSET         (EM_BLE_ET_OFFSET + EM_BLE_EXCH_TABLE_LEN * REG_BLE_EM_ET_SIZE) //32Bytes


/// Offset of the plain data area (used for SW initiated encryption)
#define EM_BLE_ENC_PLAIN_OFFSET  (EM_BLE_FT_OFFSET + (EM_BLE_VCO_TABLE_LEN + EM_BLE_FREQ_TABLE_LEN) * sizeof(uint8_t))
/// Offset of the ciphered data area (used for SW initiated encryption)
#define EM_BLE_ENC_CIPHER_OFFSET (EM_BLE_ENC_PLAIN_OFFSET + EM_BLE_ENC_LEN * sizeof(uint8_t)) //16Bytes
/// Offset of the control structure area
#define EM_BLE_CS_OFFSET         (EM_BLE_ENC_CIPHER_OFFSET + EM_BLE_ENC_LEN * sizeof(uint8_t)) //16Bytes
/// Offset of the public white list area
#define EM_BLE_WPB_OFFSET        (EM_BLE_CS_OFFSET + EM_BLE_CS_COUNT * REG_BLE_EM_CS_SIZE)
/// Offset of the private white list area
#define EM_BLE_WPV_OFFSET        (EM_BLE_WPB_OFFSET + BLE_WHITELIST_MAX * REG_BLE_EM_WPB_SIZE)
/// Offset of the TX buffer area
#define EM_BLE_TX_OFFSET         (EM_BLE_WPV_OFFSET + BLE_WHITELIST_MAX * REG_BLE_EM_WPV_SIZE)
/// Offset of the RX buffer area
#define EM_BLE_RX_OFFSET         (EM_BLE_TX_OFFSET + BLE_TX_BUFFER_CNT * REG_BLE_EM_TX_SIZE)
/// Offset of the Connection Address
#define EM_BLE_CNXADD_OFFSET     (EM_BLE_RX_OFFSET + BLE_RX_BUFFER_CNT * REG_BLE_EM_RX_SIZE)
/// Offset of the empty TX buffer area
#define EM_BLE_TXE_OFFSET        (EM_BLE_CNXADD_OFFSET + BD_ADDR_LEN)
/// Offset of the RF parameter area
#define EM_BLE_RF_SPI_OFFSET     (EM_BLE_TXE_OFFSET + EM_BLE_TXE_COUNT * REG_BLE_EM_TXE_SIZE)

 
/// @} LLDEXMEM

#endif // EM_MAP_BLE_H_
