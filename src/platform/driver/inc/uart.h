/**************************************************************************//**
 * @file     UART.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date:    17/08/14 14:25 $
 * @brief    UART driver header file
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __UART_H__
#define __UART_H__

#ifdef __cplusplus
extern "C"
{
#endif
#include "PN102Series.h"

/** @addtogroup UART_Driver UART Driver
  @{
*/




/** @addtogroup UART_EXPORTED_CONSTANTS UART Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* UART_FIFO constants definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum UART_TxTrigger
{
    Uart_tt_empty_fifo = 0x0,
    Uart_tt_two_chars_in_fifo = 0x1,
    Uart_tt_quarter_full_fifo = 0x2,
    Uart_tt_half_full_fifo = 0x3
}UART_TxTriggerDef;

typedef enum UART_RxTrigger
{
    Uart_rt_one_char_in_fifo = 0x0,
    Uart_rt_fifo_quarter_full = 0x1,
    Uart_rt_fifo_half_full = 0x2,
    Uart_rt_fifo_two_less_full = 0x3
}UART_RxTriggerDef;

typedef enum UART_DMAMode
{
    Uart_DMA_Mode0 = 0x0,
    Uart_DMA_Mode1 = 0x1
}UART_DMAModeDef;

typedef enum UART_FifoEnable
{
    Uart_Fifo_Disable = 0x0,
    Uart_Fifo_Enable = 0x1
}UART_FifoEnableDef;

typedef enum UART_AddressMatchMode
{
    User_Addr_Match = 0x0,
    Hardware_Addr_Match = 0x1
}UART_AddressMatchModeDef;
/*---------------------------------------------------------------------------------------------------------*/
/* UART_LINE constants definitions                                                                            */
/*------------------------------------------------------------------------------------`---------------------*/
/*
 * DESCRIPTION
 *  This is the data type used for manipulation of the UART line control
 *  settings.
 */
typedef enum UART_LineCtrl
{
    Uart_line_5n1 = 0x00,   // 5 data bits, no parity, 1 stop bit
    Uart_line_5n1_5 = 0x04, // 5 data bits, no parity, 1.5 stop bits
    Uart_line_5e1 = 0x18,   // 5 data bits, even parity, 1 stop bit
    Uart_line_5e1_5 = 0x1c, // 5 data bits, even parity, 1.5 stop bits
    Uart_line_5o1 = 0x08,   // 5 data bits, odd parity, 1 stop bit
    Uart_line_5o1_5 = 0x0c, // 5 data bits, odd parity, 1.5 stop bits
    Uart_line_6n1 = 0x01,   // 6 data bits, no parity, 1 stop bit
    Uart_line_6n2 = 0x05,   // 6 data bits, no parity, 2 stop bits
    Uart_line_6e1 = 0x19,   // 6 data bits, even parity, 1 stop bit
    Uart_line_6e2 = 0x1d,   // 6 data bits, even parity, 2 stop bits
    Uart_line_6o1 = 0x09,   // 6 data bits, odd parity, 1 stop bit
    Uart_line_6o2 = 0x0d,   // 6 data bits, odd parity, 2 stop bits
    Uart_line_7n1 = 0x02,   // 7 data bits, no parity, 1 stop bit
    Uart_line_7n2 = 0x06,   // 7 data bits, no parity, 2 stop bits
    Uart_line_7e1 = 0x1a,   // 7 data bits, even parity, 1 stop bit
    Uart_line_7e2 = 0x1e,   // 7 data bits, even parity, 2 stop bits
    Uart_line_7o1 = 0x0a,   // 7 data bits, odd parity, 1 stop bit
    Uart_line_7o2 = 0x0e,   // 7 data bits, odd parity, 2 stop bits
    Uart_line_8n1 = 0x03,   // 8 data bits, no parity, 1 stop bit
    Uart_line_8n2 = 0x07,   // 8 data bits, no parity, 2 stop bits
    Uart_line_8e1 = 0x1b,   // 8 data bits, even parity, 1 stop bit
    Uart_line_8e2 = 0x1f,   // 8 data bits, even parity, 2 stop bits
    Uart_line_8o1 = 0x0b,   // 8 data bits, odd parity, 1 stop bit
    Uart_line_8o2 = 0x0f    // 8 data bits, odd parity, 2 stop bits
}UART_LineCtrlDef;

/*---------------------------------------------------------------------------------------------------------*/
/* UART Interrupt definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/

/*
 * DESCRIPTION
 *  These are the bit definitions used for managing UART interrupts.
 *  The functionality of ETBEI and ERBFI alters when programmable THRE
 *  interrupt mode is active (dw_uart_enablePtime()).  See the
 *  DW_apb_uart databook for a detailed description.
 */
typedef enum UART_Irq
{
    Uart_irq_erbfi = 0x01,      // receive data available
    Uart_irq_etbei = 0x02,      // transmitter holding register empty
    Uart_irq_elsi = 0x04,       // receiver line status
    Uart_irq_edssi = 0x08,      // modem status
    Uart_irq_all = 0x0f         // all interrupts
}UART_IrqDef;

/*
 * DESCRIPTION
 *  This is the data type used for specifying UART events.  An event is
 *  the occurrence of an interrupt which must be handled appropriately.
 *  One of these events is passed at a time to the user listener
 *  function to be processed.  The exception to this are the
 *  Uart_event_thre and Uart_event_timeout interrupts which are handled
 *  internally by the interrupt handler.
 */
typedef enum UART_Event
{
    Uart_event_modem = 0x0,     // CTR, DSR, RI or DCD status changed.
    Uart_event_none = 0x1,      // No event/interrupt pending.
    Uart_event_thre = 0x2,      // Transmit holding register empty or TX
                                // FIFO at or below trigger threshold.
    Uart_event_data = 0x4,      // Receive buffer register data
                                // available (non-FIFO mode) or RX FIFO
                                // trigger level reached.
    Uart_event_line = 0x6,      // Overrun/parity/framing error or break
                                // interrupt occurred.
#if 0
    Uart_event_busy = 0x7,      // Attempt to write to the LCR[7] while
                                // DW_apb_uart was busy (DLAB).
#endif
    Uart_event_timeout = 0xc    // No characters in or out of the
                                // receiver FIFO during the last four
                                // character times and there is at least
                                // one character in it during this time.
}UART_EventDef;

#if 0
#define UART_IRQ_ID_MODEM_STAS          (uint16_t)(0x0)     //0000 - modem status
#define UART_IRQ_ID_NONE                (uint16_t)(0x1)     //0001 - no interrupt pending
#define UART_IRQ_ID_THR_EMPTY           (uint16_t)(0x2)     //0010 - THR empty
#define UART_IRQ_ID_RDA                 (uint16_t)(0x4)     //0100 - received data available
#define UART_IRQ_ID_RCV_LINE_STAS       (uint16_t)(0x6)     //0110 - receiver line status
#define UART_IRQ_ID_CHAR_TIMEOUT        (uint16_t)(0xC)     //1100 - character timeout
#endif

/*
 * DESCRIPTION
 *  These are the definitions used for reporting the line status
 *  including errors, if any.  Note that they represent bits of an 8-bit
 *  register and more than one can be active at any time.
 */
typedef enum UART_LineStatus
{
    Uart_line_dr = 0x01,        // data ready
    Uart_line_oe = 0x02,        // overrun error
    Uart_line_pe = 0x04,        // parity error
    Uart_line_fe = 0x08,        // framing error
    Uart_line_bi = 0x10,        // break interrupt
    Uart_line_thre = 0x20,      // transmit holding register empty
    Uart_line_temt = 0x40,      // transmitter empty
    Uart_line_rfe = 0x80        // receiver FIFO error
}UART_LineStatusDef;
/*****/


/**
  * @brief  UART Init Structure definition
  */

typedef struct
{
    //baud rate
    uint32_t UART_BaudRate;
    //line control
    UART_LineCtrlDef UART_LineCtrl;

} UART_InitTypeDef;



/*@}*/ /* end of group UART_EXPORTED_CONSTANTS */



void UART_Init(UART_T* uart, UART_InitTypeDef* UART_InitStruct);
void UART_DeInit(UART_T* uart );

void UART_EnableStickParity(UART_T* uart);
void UART_DisableStickParity(UART_T* uart);

UART_LineStatusDef UART_GetLineStatus(UART_T* uart);
/* FIFO functions ********************************************************/

//void UART_EnableFifo(UART_T* uart);
//void UART_DisableFifo(UART_T* uart);
//bool UART_IsFifoEnabled(UART_T* uart);

void UART_ResetRxFifo(UART_T* uart);
void UART_ResetTxFifo(UART_T* uart);

uint8_t UART_GetTxFifoLevel(UART_T* uart);
uint8_t UART_GetRxFifoLevel(UART_T* uart);

//void UART_SetTxTrigger(UART_T *UARTx, UART_TxTriggerDef level);
//void UART_SetRxTrigger(UART_T *UARTx, UART_RxTriggerDef level);

void UART_EnableFifo(UART_T* UARTx);
void UART_SetFIFOControl(UART_T* UARTx,UART_FifoEnableDef fifoe,UART_RxTriggerDef rt,UART_TxTriggerDef tet,UART_DMAModeDef dmam);

//this is same as UART_EnableAfc. a little different from init rts.
void UART_EnableAutoFlow( UART_T* UARTx);
void UART_SetRTS(UART_T* UARTx);
void UART_UnSetRTS(UART_T* UARTx);

//9 bits data
void UART_Enable9BitData( UART_T* UARTx);
void UART_SetAddressMatchMode(UART_T* UARTx,UART_AddressMatchModeDef mode);
void UART_SetReceiveAddress( UART_T* UARTx,uint32_t addr);
void UART_SetTransmitAddress( UART_T* UARTx,uint32_t addr);

//auto flow control
void UART_EnableAfc(UART_T *UARTx);
void UART_DisableAfc(UART_T *UARTx);
bool UART_IsAfcEnabled(UART_T *UARTx);

/* Tx/Rx data functions ********************************************************/
void UART_SendData(UART_T* UARTx, uint8_t Data);
uint8_t UART_ReceiveData(UART_T* UARTx);

/* DMA functions ********************************************************/
void UART_SetDmaMode(UART_T* UARTx);
uint8_t UART_GetDmaMode(UART_T* UARTx);
void UART_DmaSwAck(UART_T* UARTx);

/* Interrupts and flags management functions **********************************/
void UART_EnablePtime(UART_T* UARTx);
void UART_DisablePtime(UART_T* UARTx);
bool UART_IsPtimeEnabled(UART_T* UARTx);

//Irq Mask
void UART_EnableIrq(UART_T* UARTx, UART_IrqDef irq);
void UART_DisableIrq(UART_T* UARTx, UART_IrqDef irq);
bool UART_IsIrqEnabled(UART_T* UARTx, UART_IrqDef irq);

uint8_t UART_GetIrqMasked(UART_T * UARTx);

//Get Interrupt occurred
UART_EventDef UART_GetActiveEvent(UART_T * UARTx);

//Get Line status.add by XXL 
bool UART_IsTransmitterEmpty(UART_T* UARTx);

//Fifo Status
bool UART_IsTxFifoEmpty(UART_T* UARTx);
bool UART_IsTxFifoFull(UART_T* UARTx);
bool UART_IsRxFifoEmpty(UART_T* UARTx);
bool UART_IsRxFifoFull(UART_T* UARTx);

/*@}*/ /* end of group UART_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group UART_Driver */


#ifdef __cplusplus
}
#endif

#endif //__UART_H__

/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/








