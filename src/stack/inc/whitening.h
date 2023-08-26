#ifndef __WHITENING_H__
#define __WHITENING_H__

#include "stdint.h"

#define BLE_PDU_HEADER_SIZE     2   // Defined by Bluetooth SIG
#define BLE_PDU_PAYLOAD_SIZE    37  // BLE Core 4.0 format

#define XN297_PREMABLE_SIZE     3
#define XN297_ADDRESS_SIZE      5   // range 3 to 5
#define XN297_PAYLOAD_SIZE      1
#define XN297_CRC_SIZE          2

#define XN297_PREMABLE_POS      0
#define XN297_ADDRESS_POS       3

/**********************************************
 * channel index (BLE): 37		38		39
 * frequency (MHz):		2402	2426	2480
 *
 * channel index (XN297L): 0x3F (63)
***********************************************/

extern uint8_t xn297_whitening_payload_generate ( uint8_t        rf_channel, 
                                                  const uint8_t* address, 
                                                  uint8_t        address_length, 
                                                  const uint8_t* payload, 
                                                  uint8_t        payload_length, 
                                                  uint8_t*       dest );

                                           
#endif /* __XN297_WHITENING_H__ */
