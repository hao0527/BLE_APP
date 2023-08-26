/**************************************************************************//**
* @file     uart.c
* @version  V1.00
* $Revision: 2 $
* $Date: 16/02/25 14:25 $
* @brief    PN102 series UART driver source file
*
* @note
* Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
*****************************************************************************/

//#include <stdio.h>
#include "PN102Series.h"

/**
  * @brief  Initializes the UARTx peripheral according to the specified
  *         parameters in the UART_InitStruct .
  * @param  UARTx: where x can be 1, 2 to select the
  *         UART peripheral.
  * @param  UART_InitStruct: pointer to a UART_InitTypeDef structure that contains
  *         the configuration information for the specified UART peripheral.
  * @retval None
  */

void UART_Init(UART_T* UARTx, UART_InitTypeDef* UART_InitStruct)
{
    uint32_t tmpreg = 0x00, apbclock = 0x00;
    uint32_t integerdivider = 0x00;
    uint32_t fractionaldivider = 0x00;

    /*---------------------------- UART BRR Configuration -----------------------*/
    /* Configure the UART Baud Rate */
    //apbclock = CLK_GetPLLClockFreq();
		//26M
		apbclock = CLK_GetHCLKFreq();
    UARTx->LCR |= UART_LCR_DLAB_Msk;
    /* Determine the integer part */
    integerdivider = ((25 * apbclock) / (4 * (UART_InitStruct->UART_BaudRate)));

    tmpreg = (integerdivider / 100);
    UARTx->RBR_THR_DLL = tmpreg & 0xFF;
    UARTx->IER_DLH = (tmpreg & 0xFF00 ) >> 8;

    /* Determine the fractional part */
    fractionaldivider = integerdivider - (100 * tmpreg);

    /* Implement the fractional part in the register */
    UARTx->DLF = ((((fractionaldivider * 16) + 50) / 100)) ;
    UARTx->LCR &=  ~UART_LCR_DLAB_Msk;

    /*---------------------------- UART Line Configuration -----------------------*/
    tmpreg = UARTx->LCR;
    tmpreg &= ~(UART_LCR_SP_Msk | UART_LCR_EPS_Msk | UART_LCR_PEN_Msk | UART_LCR_STOP_Msk | UART_LCR_DLS_Msk);
    tmpreg |= (UART_InitStruct->UART_LineCtrl);
    UARTx->LCR = tmpreg;

}
void UART_DeInit(UART_T* UARTx )
{

}
void UART_EnableStickParity(UART_T* UARTx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = UARTx->LCR;
    tmpreg |= (UART_LCR_SP_Msk);
    UARTx->LCR = tmpreg;

}
void UART_DisableStickParity(UART_T* UARTx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = UARTx->LCR;
    tmpreg &= ~(UART_LCR_SP_Msk);
    UARTx->LCR = tmpreg;

}

UART_LineStatusDef UART_GetLineStatus(UART_T* UARTx)
{
    uint32_t tmpreg,retval = 0x00;

    tmpreg = UARTx->LSR;
    retval = tmpreg & UART_LSR_LINE_STATUS_Msk ;

    return (UART_LineStatusDef)retval;
}

/****f* uart.functions/dw_uart_enableFifos
 * DESCRIPTION
 *  This function enables receive and transmit FIFOs, if they are
 *  available.
 *
 */
//instead by UART_SetFIFOControl.

//void UART_EnableFifo(UART_T* UARTx)
//{
//    uint32_t tmpreg = 0x00;

//    //tmpreg = UARTx->IIR_FCR;
//    tmpreg = UART_FCR_FIFOE_Msk;

//    UARTx->IIR_FCR = tmpreg;
//}


/****f* uart.functions/dw_uart_disableFifos
 * DESCRIPTION
 *  This function disables receive and transmit FIFOs.
 *
 */
//instead by UART_SetFIFOControl.

//void UART_DisableFifo(UART_T* UARTx)
//{
//    uint32_t tmpreg = 0x00;

//    //tmpreg = UARTx->IIR_FCR;
//    tmpreg &= ~UART_FCR_FIFOE_Msk;

//    UARTx->IIR_FCR = tmpreg;
//}

/****f* uart.functions/dw_uart_areFifosEnabled
 * DESCRIPTION
 *  Returns whether the FIFOs and enabled or not.
 */

//FCR can't be readed.

//bool UART_IsFifoEnabled(UART_T* UARTx)
//{
//    uint32_t tmpreg = 0x00;

//    tmpreg = UARTx->IIR_FCR;
//    if(tmpreg & UART_FCR_FIFOE_Msk)
//        return true;
//    else
//        return false;
//}

//Need call UART_SetFIFOControl after UART_ResetRxFifo. by XXL
void UART_ResetRxFifo(UART_T* UARTx)
{
    uint32_t tmpreg = 0x00;

    tmpreg |= UART_FCR_RFIFOR_Msk;

    UARTx->IIR_FCR = tmpreg;

}

//Need call UART_SetFIFOControl after UART_ResetTxFifo. by XXL
void UART_ResetTxFifo(UART_T* UARTx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = UART_FCR_XFIFOR_Msk;

    UARTx->IIR_FCR = tmpreg;

}

/****f* uart.functions/dw_uart_getFifoDepth
 * DESCRIPTION
 *  Returns how many bytes deep the transmitter and receiver FIFOs are.
 */
uint8_t UART_GetTxFifoLevel(UART_T* UARTx)
{

     return (uint8_t)((UARTx->TFL & UART_TFL_Msk) >> UART_TFL_Pos);
}
/****f* uart.functions/dw_uart_getRxFifoLevel
 * DESCRIPTION
 *  This function returns the number of characters currently present in
 *  the Rx FIFO.
 */
uint8_t UART_GetRxFifoLevel(UART_T* UARTx)
{
     return (uint8_t)((UARTx->TFL & UART_RFL_Msk) >> UART_RFL_Pos);

}
/****f* uart.functions/dw_uart_getTxTrigger
 * DESCRIPTION
 *  Gets the trigger level of the transmitter FIFO empty interrupt.
*/

//instead by UART_SetFIFOControl.

//void UART_SetTxTrigger(UART_T* UARTx, UART_TxTriggerDef level)
//{
//    uint32_t tmpreg = 0x00;

//    tmpreg = UARTx->IIR_FCR;
//    tmpreg &= ~UART_FCR_TET_Msk;
//    
//    tmpreg |= (level << UART_FCR_TET_Pos);
//    UARTx->IIR_FCR = tmpreg;
//}

/****f* uart.functions/dw_uart_setRxTrigger
 * DESCRIPTION
 *  Sets the trigger level for the receiver FIFO full interrupt.
*/

//instead by UART_SetFIFOControl.

//void UART_SetRxTrigger(UART_T* UARTx, UART_RxTriggerDef level)
//{
//    uint32_t tmpreg = 0x00;

//    tmpreg = UARTx->IIR_FCR;
//    tmpreg &= ~UART_FCR_RT_Msk;

//    tmpreg |= (level << UART_FCR_RT_Pos);
//    UARTx->IIR_FCR = tmpreg;
//}

//Compatible with the function.default use 1/4 quarter FIFO.
void UART_EnableFifo(UART_T* UARTx)
{
    UART_SetFIFOControl(UARTx,Uart_Fifo_Enable,Uart_rt_fifo_quarter_full,Uart_tt_quarter_full_fifo,(UART_DMAModeDef)0);
}

// Init FIFO. added by XXL.
void UART_SetFIFOControl(UART_T* UARTx,UART_FifoEnableDef fifoe,UART_RxTriggerDef rt,UART_TxTriggerDef tet,UART_DMAModeDef dmam)
{
    uint32_t tmpreg = 0x00;
    tmpreg |= (rt << UART_FCR_RT_Pos)|(tet << UART_FCR_TET_Pos)|(dmam << UART_FCR_DMAM_Pos) | (fifoe << UART_FCR_FIFOE_Pos);
    UARTx->IIR_FCR = tmpreg;
}

void UART_EnableAutoFlow( UART_T* UARTx)
{
   uint32_t tmpreg = 0x00;
   tmpreg = UARTx->MCR;
   tmpreg |= UART_MCR_RTS_Msk|UART_MCR_AFCE_Msk;
   UARTx->MCR = tmpreg;
}


void UART_SetRTS(UART_T* UARTx)
{
   UARTx->MCR |= UART_MCR_RTS_Msk;
}

void UART_UnSetRTS(UART_T* UARTx)
{
   UARTx->MCR &= ~UART_MCR_RTS_Msk;
}

//this will reset other value of bits to be 0.
void UART_Enable9BitData( UART_T* UARTx)
{
    UARTx->LCR_EXT = UART_LCR_EXT_DLS_E_Msk;
}

//this will reset all value of bits  to be 0.
void UART_Disable9BitData( UART_T* UARTx)
{
    UARTx->LCR_EXT = 0;
}

void UART_SetAddressMatchMode(UART_T* UARTx,UART_AddressMatchModeDef mode)
{
   UARTx->LCR_EXT |= (mode << UART_LCR_EXT_ADDR_MATCH_Pos);
}

void UART_SetReceiveAddress( UART_T* UARTx,uint32_t addr)
{
    UARTx->RAR = addr;
}

void UART_SetTransmitAddress( UART_T* UARTx,uint32_t addr)
{
    UARTx->TAR = addr;
}

void UART_EnableAfc(UART_T *UARTx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = UARTx->MCR;
    tmpreg |= UART_MCR_AFCE_Msk;

    UARTx->MCR = tmpreg;
}
void UART_DisableAfc(UART_T *UARTx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = UARTx->MCR;
    tmpreg &= ~UART_MCR_AFCE_Msk;

    UARTx->MCR = tmpreg;

}
bool UART_IsAfcEnabled(UART_T *UARTx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = UARTx->MCR;
    if(tmpreg & UART_MCR_AFCE_Msk)
        return true;
    else
        return false;

}

void UART_SendData(UART_T* UARTx, uint8_t Data)
{
    UARTx->RBR_THR_DLL = Data;
}

uint8_t UART_ReceiveData(UART_T* UARTx)
{
    uint8_t retval;

    retval = (uint8_t)UARTx->RBR_THR_DLL;

    return retval;
}


void UART_EnablePtime(UART_T* UARTx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = UARTx->IER_DLH;
    tmpreg |= UART_IER_EPTI_Msk;

    UARTx->IER_DLH = tmpreg;


}

void UART_DisablePtime(UART_T* UARTx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = UARTx->IER_DLH;
    tmpreg &= ~UART_IER_EPTI_Msk;

    UARTx->IER_DLH = tmpreg;

}

bool UART_IsPtimeEnabled(UART_T* UARTx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = UARTx->IER_DLH;
    if(tmpreg & UART_IER_EPTI_Msk)
        return true;
    else
        return false;


}

void UART_EnableIrq(UART_T* UARTx, UART_IrqDef irq)
{
    uint32_t tmpreg = 0x00;

    tmpreg = UARTx->IER_DLH;
    tmpreg |= ((irq << UART_IER_ALL_IRQ_Pos ) & UART_IER_ALL_IRQ_Msk);

    UARTx->IER_DLH = tmpreg;

}
void UART_DisableIrq(UART_T* UARTx, UART_IrqDef irq)
{
    uint32_t tmpreg = 0x00;

    tmpreg = UARTx->IER_DLH;
    tmpreg &= ~((irq << UART_IER_ALL_IRQ_Pos ) & UART_IER_ALL_IRQ_Msk );

    UARTx->IER_DLH = tmpreg;
}

bool UART_IsIrqEnabled(UART_T* UARTx, UART_IrqDef irq)
{
    uint32_t tmpreg,intmsk = 0x00;

    tmpreg = UARTx->IER_DLH;
    intmsk = (irq << UART_IER_ALL_IRQ_Pos ) & UART_IER_ALL_IRQ_Msk ;

    if(tmpreg & intmsk)
        return true;
    else
        return false;
}

uint8_t UART_GetIrqMasked(UART_T * UARTx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = UARTx->IER_DLH;

    return (uint8_t)( (tmpreg & UART_IER_ALL_IRQ_Msk) >> UART_IER_ALL_IRQ_Pos);

}
UART_EventDef UART_GetActiveEvent(UART_T * UARTx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = UARTx->IIR_FCR;

    return (UART_EventDef)( (tmpreg & UART_IIR_IID_Msk) >> UART_IIR_IID_Pos);

}


bool UART_IsTxFifoEmpty(UART_T* UARTx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = UARTx->USR;

    if(tmpreg & UART_USR_TFE_Msk)
        return true;
    else
        return false;

}


//add by XXL
bool UART_IsTransmitterEmpty(UART_T* UARTx)
{
    uint32_t tmpreg;

    tmpreg = UARTx->LSR;

    if(tmpreg & UART_LSR_TEMT_Msk)    
        return true;
    else
        return false;
}

bool UART_IsTxFifoFull(UART_T* UARTx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = UARTx->USR;

    if(tmpreg & UART_USR_TFNF_Msk)
        return false;
    else
        return true;
}
bool UART_IsRxFifoEmpty(UART_T* UARTx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = UARTx->USR;

    if(tmpreg & UART_USR_RFNE_Msk)
        return false;
    else
        return true;

}
bool UART_IsRxFifoFull(UART_T* UARTx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = UARTx->USR;

    if(tmpreg & UART_USR_RFF_Msk)
        return true;
    else
        return false;

}

/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/



