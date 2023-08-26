#ifndef MESH_PACKET_H__
#define MESH_PACKET_H__

#include <stdint.h>
#include <string.h>

#include "mesh_error.h"
#include "utils.h"


/**
 * @defgroup PACKET Packet Formats
 * @ingroup MESH_CORE
 * Provides definitions of the packet format and functions for manipulating
 * the provided structures.
 * @{
 */
/** @brief BLE address length. */
#define BLE_GAP_ADDR_LEN (6)
#define BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH    (31)                /**< Bluetooth Core Specification v4.0 adv packet payload max length */
#define BLE_ADV_PACKET_OVERHEAD              (6)  /**< BLE-packet overhead length in addition to length of payload. */
#define BLE_ADV_PACKET_HEADER_LENGTH         (2)                 /**< BLE-packet header length on air in bytes (pdu type + tx_add_type + length) */
#define BLE_ADV_PACKET_HEADER_PADDING_LENGTH (1)                 /**< Padding needed in the packet for the PAN Radio. */
#define BLE_ADV_PACKET_HEADER_BUFFER_LENGTH                                                        \
    (BLE_ADV_PACKET_HEADER_LENGTH +                                                                \
     BLE_ADV_PACKET_HEADER_PADDING_LENGTH) /**< BLE-packet header length in buffer in bytes */
#define BLE_ADV_PACKET_BUFFER_OVERHEAD                                                             \
    (BLE_ADV_PACKET_HEADER_BUFFER_LENGTH +                                                         \
     BLE_ADV_PACKET_OVERHEAD) /**< Overhead in an advertising packet buffer. */
#define BLE_ADV_PACKET_BUFFER_MAX_LENGTH                                                           \
    (BLE_ADV_PACKET_HEADER_BUFFER_LENGTH + BLE_ADV_PACKET_OVERHEAD +                               \
     BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH) /**< Longest possible BLE advertisement packet */
#define BLE_ADV_PACKET_MIN_LENGTH                                                                  \
    (BLE_ADV_PACKET_OVERHEAD) /**< Minimum length of a BLE advertisement packet. */
#define BLE_AD_DATA_OVERHEAD                 (1)                 /**< BLE AD data overhead per AD-data structure */
#define BLE_ADV_SERVICE_DATA_UUID_DFU        (0xfee4)            /**< Service data UUID for DFU packets. */

#define AD_TYPE_MESH                        (0x2a)              /**< AD type for Bluetooth mesh. */
#define AD_TYPE_BEACON                      (0x2b)              /**< AD type for Bluetooth mesh beacons. */
#define AD_TYPE_PB_ADV                      (0x29)              /**< AD type for PB-ADV messages. */
#define AD_TYPE_DFU                         (0x16)              /**< AD type for OpenMesh messages. */

/**
 * Generic packet type.
 *
 * This type is returned by the packet manager when allocating a packet buffer.
 * It can be cast to any of the other packet format structures.
 */
typedef void packet_generic_t;

/*lint -align_max(push) -align_max(1) */

/**
 * BLE advertisement packet types.
 */
typedef enum
{
    
    BLE_PACKET_TYPE_ADV_IND,            /**< Indication advertisement type */
    BLE_PACKET_TYPE_ADV_DIRECT_IND,     /**< Direct advertisement type */    
    BLE_PACKET_TYPE_ADV_NONCONN_IND,    /**< Nonconnectable advertisement type */
    BLE_PACKET_TYPE_SCAN_REQ,           /**< Scan request type */
    BLE_PACKET_TYPE_SCAN_RSP,           /**< Scan response type */
    BLE_PACKET_TYPE_CONN_REQ,           /**< Connection request type */
    BLE_PACKET_TYPE_ADV_DISCOVER_IND,   /**< Discoverable advertisement type */
    BLE_PACKET_TYPE_ADV_EXT             /**< Advertising extension packet type */
} ble_packet_type_t;

/**
 * BLE standard adv header.
 */
typedef struct __attribute((packed))
{
    uint8_t type        : 4;            /**< BLE packet type, from ble_packet_type_t enum */
    uint8_t _rfu1       : 2;            /**< Reserved for future use */
    uint8_t addr_type   : 1;            /**< BLE GAP address type */
    uint8_t _rfu2       : 1;            /**< Reserved for future use */
    uint8_t length;                     /**< Packet length */
    uint8_t _rfu3;                      /**< Padding needed for the S1 field on the PAN Radio*/
} ble_packet_hdr_t;

/**
 * BLE standard AD-data header.
 */
typedef struct __attribute((packed))
{
    uint8_t length;                     /**< Length of type + data */
    uint8_t type;                       /**< Advertisement type, assigned by Bluetooth SIG */
} ble_ad_header_t;

/**
 * BLE standard ad-data format.
 */
typedef struct __attribute((packed))
{
    uint8_t length;                     /**< Length of type + data */
    uint8_t type;                       /**< Advertisement type, assigned by Bluetooth SIG */
    uint8_t data[];                     /**< Advertisement data, is length - 1 long */
} ble_ad_data_t;

/** BLE Service data packets. */
typedef struct __attribute((packed))
{
    uint16_t uuid;      /**< UUID field. */
    uint8_t data[];     /**< Service data field. */
} ble_ad_data_service_data_t;

/**
 * BLE standard adv packet.
 */
typedef struct __attribute((packed))
{
    ble_packet_hdr_t header;                            /**< BLE packet header */
    uint8_t addr[BLE_GAP_ADDR_LEN];                     /**< BLE GAP advertisement address */
    uint8_t payload[BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH]; /**< BLE advertisement payload */
} packet_t;

/*lint -align_max(pop) */

/**
 * Get next AD data structure in the packet.
 *
 * @param[in] p_ad Pointer to current ad structure to iterate from.
 *
 * @return Pointer to next ad structure after @p p_ad.
 */
static inline ble_ad_data_t* packet_ad_type_get_next(ble_ad_data_t* p_ad)
{
    return (ble_ad_data_t*) ((uint8_t*) p_ad + p_ad->length + BLE_AD_DATA_OVERHEAD);
}

/**
 * Sets the payload size of an advertisement packet.
 *
 * @param[in,out] p_packet Pointer to the BLE advertising packet.
 * @param[in] size Size of the payload field of the packet.
 */
static inline void packet_payload_size_set(packet_t * p_packet, uint8_t size)
{
    p_packet->header.length = size + BLE_GAP_ADDR_LEN;
}

/**
 * Gets the length of the payload field of a packet.
 *
 * @param[in] p_packet Pointer to the BLE advertising packet.
 *
 * @return Length of the payload field of the packet.
 */
static inline uint8_t packet_payload_size_get(const packet_t * p_packet)
{
    return p_packet->header.length - BLE_GAP_ADDR_LEN;
}

/** @} */

#endif

