/**
 ****************************************************************************************
 * @file    dbg_sys.h
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
/** @defgroup Module Description
* @{
*/
#ifndef __DBG_SYS_H__
#define __DBG_SYS_H__

#include "stdint.h"
#include "dbg_define.h"


#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */


#if DBG_SYS_EN
#define DBG_SYS(address, data)  dbg_sys_write(address, data)
#define DBG_SYS_PRINT()         dbg_sys_print()
#else
#define DBG_SYS(address, data)
#define DBG_SYS_PRINT()
#endif


#define DBG_SYS_U32(address, data)  do { \
    DBG_SYS ( address, (data) ); \
    DBG_SYS ( address, ((data) >> 16) ); \
} while ( 0 )

#define DBG_SYS_BD_ADDR(address, addr_type, addr) do { \
    DBG_SYS ( address, addr_type); \
    DBG_SYS ( address, (addr[5] << 8) | addr[4]); \
    DBG_SYS ( address, (addr[3] << 8) | addr[2]); \
    DBG_SYS ( address, (addr[1] << 8) | addr[0]); \
} while ( 0 )

extern void dbg_sys_init ( void );
extern void dbg_sys_enable ( uint8_t enable );
extern void dbg_sys_write ( uint16_t address, uint16_t data  );
extern void dbg_sys_print ( void );


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __DBG_SYS_H__ */
/** @} */
