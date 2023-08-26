/**************************************************************************//**
 * @file     i2c.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/02/2216:24 $
 * @brief    PN102 series I2C driver source file
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
*****************************************************************************/
#include "PN102Series.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define FLAG_MASK         ((uint32_t)0x007F0FFF)  /*<! I2C FLAG mask */

#define ITEN_MASK         ((uint16_t)0x2FFF)  /*<! I2C Interrupt Enable mask */
#define IT_FLAG_MASK      ((uint16_t)0x2FFF)  /*<! I2C FLAG mask */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the I2Cx peripheral according to the specified
  *         parameters in the I2C_InitStruct.
  *
  * @note   To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency
  *         (I2C peripheral input clock) must be a multiple of 10 MHz.
  *
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  I2C_InitStruct: pointer to a I2C_InitTypeDef structure that contains
  *         the configuration information for the specified I2C peripheral.
  * @note   This function should be called before enabling
            the I2C Peripheral.

  * @retval None
  */
void I2C_Init(I2C_T* I2Cx, I2C_InitTypeDef* I2C_InitStruct)
{
	uint32_t tmpreg = 0;
	uint32_t result = 0x04;
	uint32_t pclk1;

	//26M
    pclk1 = CLK_GetHCLKFreq();

	/*---------------------------- I2Cx CON Configuration ------------------------*/
	/*Disable I2C */

	tmpreg = I2Cx->CON;

	/*restart enable*/
	tmpreg |= (I2C_CON_IC_RESTART_EN);
	/* slave address mode*/
	if(I2C_AcknowledgedAddress_7bit == I2C_InitStruct->I2C_AcknowledgedAddress)
	{
		tmpreg &= ~I2C_CON_IC_10BITADDR_SLAVE;
	}
	else//10-bit addr
	{
		tmpreg |= I2C_CON_IC_10BITADDR_SLAVE;
	}

    tmpreg &= ~(I2C_CON_SPEED);
	/*---------------------------- I2Cx HCNT/LCNT Configuration ------------------------*/
	/* Configure speed in standard mode */
	if (I2C_InitStruct->I2C_ClockSpeed <= 100000)
	{
		/* Standart mode speed calculate: Tlow/Thigh = 1 */
		result = (pclk1 / (I2C_InitStruct->I2C_ClockSpeed << 1));
		/*HCNT equals to LCNT*/
		I2Cx->SS_SCL_HCNT = result - I2Cx->FS_SPKLEN - 7;
		I2Cx->SS_SCL_LCNT = result - 1;

		tmpreg |= I2C_SPEED_STANDARD_MODE;
	}
	/* Configure speed in fast mode */
	else  if (I2C_InitStruct->I2C_ClockSpeed <= 1000000)
	{
		if (I2C_InitStruct->I2C_DutyCycle == I2C_DutyCycle_2)
		{
			/* Fast mode speed calculate: Tlow/Thigh = 2 */
			result = (pclk1 / (I2C_InitStruct->I2C_ClockSpeed * 3));
			I2Cx->FS_SCL_HCNT = result - I2Cx->FS_SPKLEN - 7;
			I2Cx->FS_SCL_LCNT = (result << 1) - 1;
		}
		else
		{
			/* Fast mode speed calculate: Tlow/Thigh = 16/9 */
			result = (pclk1 / (I2C_InitStruct->I2C_ClockSpeed * 25));
			/* Set DUTY bit */
			I2Cx->FS_SCL_HCNT = result * 9 - I2Cx->FS_SPKLEN - 7;
			I2Cx->FS_SCL_LCNT = (result << 4) - 1;
		}

		tmpreg |= I2C_SPEED_FAST_MODE;

	}
    /* To use the I2C at 1 MHz (in fast mode plus ) */
	else
	{

	}

	I2Cx->CON = tmpreg;

	/*data setup time: 250ns*/
	I2Cx->SDA_SETUP = (pclk1) / 4000000 + 1;

	/* Set I2Cx Own Address1 and acknowledged address */
	I2Cx->SAR &= ~( I2C_TAR_TAR);
	/* Get 7-bit address from I2C_OwnAddress1*/
	I2Cx->SAR |=  ((I2C_InitStruct ->I2C_OwnAddress1 )) ;
	/*fifo threshold */
	I2Cx->TX_TL = I2C_TX_TL_4;
	I2Cx->RX_TL = I2C_RX_TL_4;
}

/**
  * @brief  Set I2C mode .
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  Mode: role of the I2Cx peripheral.
  *          This parameter can be: I2C_MODE_MASTER or I2C_MODE_SLAVE.
  * @note   This function should be called before enabling
            the I2C Peripheral.
  * @retval None
  */
void I2C_SetMode(I2C_T* I2Cx, uint16_t Mode)
{
    uint32_t tmpreg = I2Cx->CON;

    /*master mode*/
    if(Mode == I2C_MODE_MASTER)
    {
        tmpreg |= (I2C_CON_IC_SLAVE_DISABLE | I2C_CON_MASTER_MODE);
    }
    else
    {
        tmpreg &= ~(I2C_CON_IC_SLAVE_DISABLE | I2C_CON_MASTER_MODE);
    }

    I2Cx->CON = tmpreg;

}

/**
  * @brief  Set I2C Tx fifo threshold value .
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  ThresholdValue:Transmit FIFO Threshold Level
  *          This parameter can be @I2C_Transmit_FIFO_Threshold_Value.
  * @note   Controls the level of entries (or below) that trigger the TX_EMPTY interrupt (bit 4 in
  *         IC_RAW_INTR_STAT register). The valid range is 0-8, with the additional
  *         restriction that it may not be set to value larger than the depth of the buffer. If an
  *         attempt is made to do that, the actual value set will be the maximum depth of the
  *         buffer.A value of 0 sets the threshold for 0 entries, and a value of 8 sets the threshold for
  *         8 entries.
  * @retval None
  */
void I2C_SetTxTirggerLevel(I2C_T* I2Cx, uint8_t ThresholdValue)
{
    uint32_t tmpreg = 0;
    /* Check the parameters */

    tmpreg = I2Cx->TX_TL;

    tmpreg &= ~(I2C_TX_TL);
    tmpreg |= (ThresholdValue & 0xF);

    I2Cx->TX_TL = tmpreg;

}

/**
  * @brief  Set I2C Rx fifo threshold value .
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  ThresholdValue:Receive FIFO Threshold Level
  *          This parameter can be @I2C_Receive_FIFO_Threshold_Value.
  * @note   Controls the level of entries (or below) that trigger the RX_FULL interrupt (bit 2 in
  *         IC_RAW_INTR_STAT register). The valid range is 0-8, with the additional
  *         restriction that it may not be set to value larger than the depth of the buffer. If an
  *         attempt is made to do that, the actual value set will be the maximum depth of the
  *         buffer.A value of 0 sets the threshold for 0 entries, and a value of 8 sets the threshold for
  *         8 entries.
  * @retval None
  */
void I2C_SetRxTirggerLevel(I2C_T* I2Cx, uint8_t ThresholdValue)
{
    uint32_t tmpreg = 0;
    /* Check the parameters */
    tmpreg = I2Cx->RX_TL;

    tmpreg &= ~(I2C_RX_TL);
    tmpreg |= (ThresholdValue & 0xF);

    I2Cx->RX_TL = tmpreg;

}

/**
  * @brief  Enables or disables the specified I2C peripheral.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  NewState: new state of the I2Cx peripheral.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void I2C_Cmd(I2C_T* I2Cx, FunctionalState NewState)
{
    /* Check the parameters */

    if (NewState != DISABLE)
    {
        /* Enable the selected I2C peripheral */
        I2Cx->IC_ENABLE |= I2C_ENABLE_ENABLE;
    }
    else
    {
        /* Disable the selected I2C peripheral */
        I2Cx->IC_ENABLE &= (uint32_t)~((uint32_t)I2C_ENABLE_ENABLE);
    }
}

/**
  * @brief  Configure the target address for any master transaction.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  Address: specifies the slave 7-bit address which will be transmitted
  * @retval None.
  */
void I2C_Set7bitAddress(I2C_T* I2Cx, uint8_t Address)
{
	uint32_t tmpreg;
	 /* Check the parameters */

	tmpreg = I2Cx->TAR;

    /*7-bits address mode*/
	tmpreg &= ~(I2C_TAR_IC_10BITADDR_MASTER | I2C_TAR_SPECIAL | I2C_TAR_GC_OR_START | I2C_TAR_TAR);

	tmpreg |= (Address & 0x07F);

	/* Send the address */
	I2Cx->TAR = tmpreg;

}

/**
  * @brief  Configure the target address for any master transaction.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  Address: specifies the slave 10-bit address which will be transmitted
  * @retval None.
  */
void I2C_Set10bitAddress(I2C_T* I2Cx, uint16_t Address)
{
	uint32_t tmpreg;

	tmpreg = I2Cx->TAR;

    /*7-bits address mode*/
	tmpreg &= ~(I2C_TAR_IC_10BITADDR_MASTER | I2C_TAR_SPECIAL | I2C_TAR_GC_OR_START | I2C_TAR_TAR);

	tmpreg |= (I2C_TAR_IC_10BITADDR_MASTER | (Address & 0x03ff));

	/* Send the address */
	I2Cx->TAR = tmpreg;

}

/**
  * @brief  Enables or disables the specified I2C general call feature.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  NewState: new state of the I2C General call.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void I2C_GeneralCallCmd(I2C_T* I2Cx, FunctionalState NewState)
{
	uint32_t tmpreg = 0;

	tmpreg = I2Cx->TAR;

	if(NewState != DISABLE)
	{
        tmpreg |= (I2C_TAR_SPECIAL);
        tmpreg &= ~(I2C_TAR_GC_OR_START);
        I2Cx->TAR = tmpreg;
	}

}



/**
  * @}
  */


/** @defgroup I2C_Group2 Data transfers functions
 *  @brief   Data transfers functions
 *
@verbatim
 ===============================================================================
                  ##### Data transfers functions #####
 ===============================================================================

@endverbatim
  * @{
  */

/**
  * @brief  Sends a data byte through the I2Cx peripheral.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  Data: Byte to be transmitted..
            Cmd : I2C_CMD_RD/I2C_CMD_WR, I2C_CMD_RESTART, or I2C_CMD_STOP
  * @retval None
  */
void I2C_SendDataCmd(I2C_T* I2Cx, uint8_t Data, uint8_t Cmd)
{
	uint32_t reg = 0;
    uint8_t data = Data;

	reg |= ((Cmd << 8 ) | (data & 0xFF)) ;
	/* Write in the DR register the data to be sent */
	I2Cx->DATACMD = reg;
}
/**
  * @brief  Sends a data byte through the I2Cx peripheral.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  Cmd: I2C_CMD_RD/I2C_CMD_WR,I2C_CMD_RESTART, or I2C_CMD_STOP
  * @retval None
  */
void I2C_SendCmd(I2C_T* I2Cx, uint8_t Cmd)
{
	uint32_t reg = 0;

	reg |= (Cmd << 8);
	/* Write in the DR register the data to be sent */
	I2Cx->DATACMD = reg;
}

/**
  * @brief  Returns the most recent received data by the I2Cx peripheral.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @retval The value of the received data.
  */
uint8_t I2C_ReceiveData(I2C_T* I2Cx)
{
  /* Return the data in the DR register */
  return (uint8_t)I2Cx->DATACMD;
}

/**
  * @brief  Aborting I2C Transfers.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @retval None
  */
void I2C_AbortTransfer(I2C_T* I2Cx)
{
	I2Cx->IC_ENABLE |= I2C_ENABLE_ABORT;
}
/**
  * @}
  */

/** @defgroup I2C_Group4 DMA transfers management functions
 *  @brief   DMA transfers management functions
 *
@verbatim
 ===============================================================================
                ##### DMA transfers management functions #####
 ===============================================================================
  This section provides functions allowing to configure the I2C DMA channels
  requests.

@endverbatim
  * @{
  */

/**
  * @brief  Enables or disables the specified I2C DMA requests.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  NewState: new state of the I2C DMA transfer.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void I2C_DMACmd(I2C_T* I2Cx, FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        /* Enable the selected I2C peripheral */
        I2Cx->DMA_CR |= (I2C_DMA_CR_RDMAE | I2C_DMA_CR_TDMAE);
    }
    else
    {
        /* Disable the selected I2C peripheral */
	    I2Cx->DMA_CR &= (uint32_t)~((uint32_t)(I2C_DMA_CR_RDMAE | I2C_DMA_CR_TDMAE));
    }
}
/**
  * @brief  DMA Transmit Data Level Register.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  Level: This parameter controls the level at which a DMA request is made by the transmit logic
  * @retval None
  */
void I2C_DMATransferDataLevel(I2C_T* I2Cx, uint8_t Level)
{
	I2Cx->DMA_TDLR |= (Level & 0x07);
}

/**
  * @brief  DMA Receive Data Level Register.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  Level: This parameter controls the level at which a DMA request is made by the receive logic.
  * @retval None
  */
void I2C_DMAReceiveDataLevel(I2C_T* I2Cx, uint8_t Level)
{
	I2Cx->DMA_TDLR |= (Level & 0x07);
}

/**
  * @}
  */

/**
  * @brief  Reads the specified I2C register and returns its value.
  * @param  I2C_Register: specifies the register to read.
  *          This parameter can be one of the following values
  *          @I2C_registers
  * @retval The value of the read register.
  */
uint16_t I2C_ReadRegister(I2C_T* I2Cx, uint8_t I2C_Register)
{
    __IO uint32_t tmp = 0;

    tmp = (uint32_t) I2Cx;
    tmp += I2C_Register;

    /* Return the selected register value */
    return (*(__IO uint16_t *) tmp);
}


/**
  * @brief  Enables or disables the specified I2C interrupts.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  I2C_IT: specifies the I2C interrupts sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *             @arg I2C_IT_RX_UNDER
  *             @arg I2C_IT_RX_OVER
  *             @arg I2C_IT_RX_FULL
  *             @arg I2C_IT_TX_OVER
  *             @arg I2C_IT_TX_EMPTY
  *             @arg I2C_IT_RD_REQ
  *             @arg I2C_IT_TX_ABRT
  *             @arg I2C_IT_RX_DONE
  *             @arg I2C_IT_ACTIVITY
  *             @arg I2C_IT_STOP_DET
  *             @arg I2C_IT_START_DET
  *             @arg I2C_IT_GEN_CALL
  * @param  NewState: new state of the specified I2C interrupts.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void I2C_ITConfig(I2C_T* I2Cx, uint16_t I2C_IT, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    /* Enable the selected I2C interrupts */
    I2Cx->INTR_MASK |= I2C_IT;
  }
  else
  {
    /* Disable the selected I2C interrupts */
    I2Cx->INTR_MASK &= (uint16_t)~I2C_IT;
  }
}

/**
  * @brief  Checks whether the specified I2C flag is set or not.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  I2C_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *          @arg I2C_FLAG_SLV_ACTIVITY
  *          @arg I2C_FLAG_MST_ACTIVITY
  *          @arg I2C_FLAG_RFF
  *          @arg I2C_FLAG_TFE
  *          @arg I2C_FLAG_TFNF
  *          @arg I2C_FLAG_ACTIVITY
  *          @arg I2C_FLAG_MST_ACTIVITY
  * @retval The new state of I2C_FLAG (SET or RESET).
  */
FlagStatus I2C_GetFlagStatus(I2C_T* I2Cx, uint32_t I2C_FLAG)
{
	FlagStatus bitstatus = RESET;
	__IO uint32_t i2cxbase = 0;

	/* Get the I2Cx peripheral base address */
	i2cxbase = (uint32_t)I2Cx;

	/* Get bit[23:0] of the flag */
	I2C_FLAG &= FLAG_MASK;
	i2cxbase += 0x70;

	if(((*(__IO uint32_t *)i2cxbase) & I2C_FLAG) != (uint32_t)RESET)
	{
		/* I2C_FLAG is set */
		bitstatus = SET;
	}
	else
	{
		/* I2C_FLAG is reset */
		bitstatus = RESET;
	}

	/* Return the I2C_FLAG status */
	return  bitstatus;
}

/**
  * @brief  Clears the I2Cx's pending flags.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  I2C_FLAG: specifies the flag to clear.
  *          This parameter can be any combination of the following values:
  *          @arg I2C_FLAG_SLV_ACTIVITY
  *          @arg I2C_FLAG_MST_ACTIVITY
  *          @arg I2C_FLAG_RFF
  *          @arg I2C_FLAG_TFE
  *          @arg I2C_FLAG_TFNF
  *          @arg I2C_FLAG_ACTIVITY
  *          @arg I2C_FLAG_MST_ACTIVITY
  *
  * @retval None
  */
void I2C_ClearFlag(I2C_T* I2Cx, uint32_t I2C_FLAG)
{
	  uint32_t flagpos = 0;

	  (void)flagpos;
}

/**
  * @brief  Checks whether the specified I2C interrupt has occurred or not.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  I2C_IT: specifies the interrupt source to check.
  *          This parameter can be one of the following values:
  *          @arg I2C_IT_RX_UNDER
  *          @arg I2C_IT_RX_OVER
  *          @arg I2C_IT_RX_FULL
  *          @arg I2C_IT_TX_OVER
  *          @arg I2C_IT_TX_EMPTY
  *          @arg I2C_IT_RD_REQ
  *          @arg I2C_IT_TX_ABRT
  *          @arg I2C_IT_RX_DONE
  *          @arg I2C_IT_ACTIVITY
  *          @arg I2C_IT_STOP_DET
  *          @arg I2C_IT_START_DET
  *          @arg I2C_IT_GEN_CALL
  *          @arg I2C_IT_MST_ON_HOLD
  * @retval The new state of I2C_IT (SET or RESET).
  */
ITStatus I2C_GetITStatus(I2C_T* I2Cx, uint16_t I2C_IT)
{
    ITStatus bitstatus = RESET;

    /* Get bit[11:0] of the interrupt flag */
    I2C_IT &= IT_FLAG_MASK;

    /* Check the status of the specified I2C flag */
    if ((I2Cx->INTR_STAT & I2C_IT) != (uint32_t)RESET)
    {
        /* I2C_IT is set */
        bitstatus = SET;
    }
    else
    {
        /* I2C_IT is reset */
        bitstatus = RESET;
    }

    /* Return the I2C_IT status */
    return  bitstatus;
}
/**
  * @brief  Checks whether the specified I2C interrupt has occurred or not.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  I2C_IT: specifies the interrupt source to check.
  *          This parameter can be one of the following values:
  *             @arg I2C_IT_RX_UNDER
  *             @arg I2C_IT_RX_OVER
  *             @arg I2C_IT_RX_FULL
  *             @arg I2C_IT_TX_OVER
  *             @arg I2C_IT_TX_EMPTY
  *             @arg I2C_IT_RD_REQ
  *             @arg I2C_IT_TX_ABRT
  *             @arg I2C_IT_RX_DONE
  *             @arg I2C_IT_ACTIVITY
  *             @arg I2C_IT_STOP_DET
  *             @arg I2C_IT_START_DET
  *             @arg I2C_IT_GEN_CALL
  *             @arg I2C_IT_MST_ON_HOLD
  * @retval The new state of I2C_IT (SET or RESET).
  */
ITStatus I2C_GetRawITStatus(I2C_T* I2Cx, uint16_t I2C_IT)
{
    ITStatus bitstatus = RESET;

    /* Get bit[11:0] of the interrupt flag */
    I2C_IT &= IT_FLAG_MASK;

    /* Check the status of the specified I2C flag */
    if ((I2Cx->RAW_INTR_STAT & I2C_IT) != (uint32_t)RESET)
    {
        /* I2C_IT is set */
        bitstatus = SET;
    }
    else
    {
        /* I2C_IT is reset */
        bitstatus = RESET;
    }
    /* Return the I2C_IT status */
    return  bitstatus;
}
/**
  * @brief  Clears the I2Cx's interrupt pending bits.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  I2C_IT: specifies the interrupt pending bit to clear.
  *             @arg I2C_IT_RX_UNDER
  *             @arg I2C_IT_RX_OVER
  *             @arg I2C_IT_RX_FULL
  *             @arg I2C_IT_TX_OVER
  *             @arg I2C_IT_TX_EMPTY
  *             @arg I2C_IT_RD_REQ
  *             @arg I2C_IT_TX_ABRT
  *             @arg I2C_IT_RX_DONE
  *             @arg I2C_IT_ACTIVITY
  *             @arg I2C_IT_STOP_DET
  *             @arg I2C_IT_START_DET
  *             @arg I2C_IT_GEN_CALL
  *             @arg I2C_IT_MST_ON_HOLD
  * @retval None
  */
void I2C_ClearITPendingBit(I2C_T* I2Cx, uint16_t I2C_IT)
{
    uint16_t flagpos = 0;

    /* Get the I2C flag position */
    flagpos = (I2C_IT & IT_FLAG_MASK);

    /* Clear the selected I2C flag */
    if(flagpos & I2C_IT_RX_UNDER)
	    (void)(I2Cx->CLR_RX_UND);

    if(flagpos & I2C_IT_RX_OVER)
	    (void)(I2Cx->CLR_RX_OVR);

    if(flagpos & I2C_IT_TX_OVER)
	    (void)(I2Cx->CLR_TX_OVR);

    if(flagpos & I2C_IT_RD_REQ)
	    (void)(I2Cx->CLR_RD_REQ);

    if(flagpos & I2C_IT_TX_ABORT)
	    (void)(I2Cx->CLR_TX_ABRT);

    if(flagpos & I2C_IT_RX_DONE)
	    (void)(I2Cx->CLR_RX_DONE);

    if(flagpos & I2C_IT_ACTIVITY)
	    (void)(I2Cx->CLR_ACTIVITY);

    if(flagpos & I2C_IT_STOP_DET)
	    (void)(I2Cx->CLR_STOP_DET);

    if(flagpos & I2C_IT_START_DET)
		(void)(I2Cx->CLR_START_DET);

    if(flagpos & I2C_IT_GEN_CALL)
		(void)(I2Cx->CLR_GEN_CALL);

	if(flagpos & I2C_IT_MST_ON_HOLD)
		(void)(I2Cx->CLR_RESTART_DET);
}

/**
  * @brief  Clears all the I2Cx's interrupt pending bits.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @retval None
  */

void I2C_ClearAllITPendingBit(I2C_T* I2Cx)
{
	(void)(I2Cx->CLR_INTR);
}

/*@}*/ /* end of group PN102_I2C_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group PN102_I2C_Driver */

/*@}*/ /* end of group PN102_Device_Driver */

/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/
