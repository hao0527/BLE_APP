/**
 ****************************************************************************************
 * @file    dbg_sys.c
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
#include "panip_config.h"

#include "dbg_sys.h"
#include "uart.h"

#include "string.h"

#if DBG_SYS_EN

#define DBG_SYS_BUFFER_SIZE      256
#define DBG_SYS_BUFFER_SIZE_MASK 0xFF

#if DBG_SYS_USE_UART0
    #define DBG_SYS_UART_PORT   UART0
#elif DBG_SYS_USE_UART1
    #define DBG_SYS_UART_PORT   UART1
#endif

typedef struct
{
    uint16_t wr;
    uint16_t rd;
    uint32_t buf[DBG_SYS_BUFFER_SIZE];
} dbg_ctrl_t;

static dbg_ctrl_t m_dbg_ctrl;

typedef enum
{
    DBG_SYS_UNINIT  = 0,
    DBG_SYS_DISABLE = 1,
    DBG_SYS_ENABLE  = 2,
} dbg_sys_status_t;

static uint8_t m_dbg_sys_status = 0;


void dbg_sys_init ( void )
{
    m_dbg_sys_status = DBG_SYS_DISABLE;
    
    m_dbg_ctrl.rd = 0;
    m_dbg_ctrl.wr = 0;
    
    memset( m_dbg_ctrl.buf, 0, DBG_SYS_BUFFER_SIZE );
}

void dbg_sys_enable ( uint8_t enable )
{
    m_dbg_sys_status = (enable == 0) ? DBG_SYS_DISABLE : DBG_SYS_ENABLE;
}

void dbg_sys_write ( uint16_t address, uint16_t data  )
{
    if ( DBG_SYS_ENABLE != m_dbg_sys_status )
    {
        return;
    }
    
    uint16 wr = m_dbg_ctrl.wr;
    
    m_dbg_ctrl.buf[wr] = ( address << 16 ) + data;

    wr = (wr + 1) & DBG_SYS_BUFFER_SIZE_MASK;

    m_dbg_ctrl.wr = wr;
}

void dbg_sys_print ( void )
{
    uint16_t rd = m_dbg_ctrl.rd;
    uint16_t wr = m_dbg_ctrl.wr;

    while ( rd != wr )
    {
        uint32_t data = m_dbg_ctrl.buf[rd];
        
        while (UART_IsTxFifoFull(DBG_SYS_UART_PORT));
        DBG_SYS_UART_PORT->RBR_THR_DLL = data >> 24;
        while (UART_IsTxFifoFull(DBG_SYS_UART_PORT));
        DBG_SYS_UART_PORT->RBR_THR_DLL = data >> 16;
        while (UART_IsTxFifoFull(DBG_SYS_UART_PORT));
        DBG_SYS_UART_PORT->RBR_THR_DLL = data >> 8;
        while (UART_IsTxFifoFull(DBG_SYS_UART_PORT));
        DBG_SYS_UART_PORT->RBR_THR_DLL = data >> 0;

        rd = (rd + 1) & DBG_SYS_BUFFER_SIZE_MASK;
    }
    
    m_dbg_ctrl.rd = wr;
    
    // wait uart transfer completed
    while ( !UART_IsTxFifoEmpty ( DBG_SYS_UART_PORT ) );
}

#endif /* DBG_SYS_EN */
