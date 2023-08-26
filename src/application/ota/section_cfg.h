/**
 ****************************************************************************************
 * @file    section_cfg.h
 * @brief

 * @version 0.01
 * @date    2018/11/22
 * @history
 * @note
 * detailed description

 ****************************************************************************************
 * @attention
 *
 * Copyright (C) 2016 Shanghai Panchip Microelectronics Co., Ltd. All rights reserved.
 ****************************************************************************************
 */
#ifndef __SECTION_CFG_H__
#define __SECTION_CFG_H__

#include "stdint.h"


#define BYTE_TO_DWORD(size)             ((size) >> 2)   // size / 4
#define FLASH_ADDR_TO_SECTION(addr)     ((addr) >> 8)   // addr / 512

#define FLASH_ADDR_START                (0x00000000)
#define FLASH_ADDR_END                  (0x00040000)

#define FLASH_PAGE_SIZE                 (0x0200)     /**< 512Bytes page Size */
#define FLASH_PAGE_SIZE_DWORD           BYTE_TO_DWORD(FLASH_PAGE_SIZE)


#define FLASH_MBR_ADDR                  (FLASH_ADDR_START)                                  /**< SoftDevice Start Address */
#define FLASH_MBR_SIZE                  (0x00000000)                                        /**< SoftDevice Size */

#define FLASH_SOFTDEVICE_ADDR           (FLASH_MBR_ADDR + FLASH_MBR_SIZE)                   /**< SoftDevice Start Address */
#define FLASH_SOFTDEVICE_SIZE           (0x00016800)                                        /**< SoftDevice Size */

#define FLASH_APP_ADDR                  (FLASH_SOFTDEVICE_ADDR + FLASH_SOFTDEVICE_SIZE)     /**< Application Start Address */
#define FLASH_APP_SIZE                  (0x00012C00)                                        /**< Application Size */

#define FLASH_DFU_ADDR                  (FLASH_APP_ADDR + FLASH_APP_SIZE)                   /**< DeviceFirmwareUpdate Start Address */
#define FLASH_DFU_SIZE                  (0x00019000)                                        /**< DeviceFirmwareUpdate Size */

#define FLASH_APP_DATA_ADDR             (FLASH_DFU_ADDR + FLASH_DFU_SIZE)                   /**< Application Data Start Address */
#define FLASH_APP_DATA_SIZE             (0x00001800)                                        /**< Application Data Size */

#define FLASH_BOOTLOADER_ADDR           (0x0003DC00)
#define FLASH_BOOTLOADER_SIZE           (0x00002000)

#define FLASH_ADDR_INFO                 (0x0003D800)
#define FLASH_ADDR_INFO_BACKUP          (0x0003DA00)
#define FLASH_ADDR_INFO_SIZE            (FLASH_PAGE_SIZE)

#define FLASH_OFFSET_INFO_DFU           (0x0000)
#define FLASH_OFFSET_INFO_SOFTDEVICE    (0x0010)
#define FLASH_OFFSET_INFO_APP           (0x0020)
#define FLASH_SIZE_INFO                 (0x000F)

#define FLASH_OFFSET_DFU_SOURCE         (0x0000)
#define FLASH_SIZE_DFU_SOURCE           (0x0004)

#define FLASH_OFFSET_DFU_FLAG           (0x0003)
#define FLASH_SIZE_DFU_FLAG             (0x0004)

#define FLASH_OFFSET_SOFTDEVICE_FLAG    (0x001C)
#define FLASH_SIZE_SOFTDEVICE_FLAG      (0x0004)

#define GET_FLAG(u32flag)               (((u32flag) >> 16) & 0xFFFF)


typedef enum dfu_type
{
    DFU_SOURCE_OTA              = (uint8_t)0x01,
    DFU_SOURCE_UART             = (uint8_t)0x02,
} dfu_source_t;

typedef enum dfu_section
{
    SECTION_BOOTLOADER          = (uint8_t)0x00,
    SECTION_SOFTDEVICE          = (uint8_t)0x01,
    SECTION_APP                 = (uint8_t)0x02,
    SECTION_APP_DATA            = (uint8_t)0x03,
    SECTION_DFU                 = (uint8_t)0x04,
    SECTION_MAX                 = (uint8_t)0x05,
} section_type_t;

#define SECTION_ROOT_FLAG               0xDBE1
#define SECTION_SOFTDEVICE_FLAG         0xDBD2
#define SECTION_APP_FLAG                0xDBC3
#define SECTION_APP_DATA_FLAG           0xDBB4
#define SECTION_INFO_FLAG               0xDBA5

typedef struct
{
    uint16_t        softdevice;
    uint16_t        appliction;
	uint16_t		project;
} version_t;

typedef struct
{
    uint32_t    size;
    uint32_t    crc;
    version_t   version;
    uint16_t    flag;
} section_info_t;

#endif // __SECTION_CFG_H__
