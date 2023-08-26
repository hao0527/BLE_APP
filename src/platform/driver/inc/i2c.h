/**************************************************************************//**
 * @file     i2c.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/02/22 7:44 $
 * @brief    PN102 series I2C driver header file
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup PN102_Device_Driver PN102 Device Driver
  @{
*/

/** @addtogroup I2C
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  I2C Init structure definition
  */

typedef struct
{
  uint32_t I2C_ClockSpeed;          /*!< Specifies the clock frequency.
                                         This parameter must be set to a value lower than 400kHz */

  uint16_t I2C_Mode;                /*!< Specifies the I2C mode.
                                         This parameter can be a value of @ref I2C_mode */

  uint16_t I2C_DutyCycle;           /*!< Specifies the I2C fast mode duty cycle.
                                         This parameter can be a value of @ref I2C_duty_cycle_in_fast_mode */

  uint16_t I2C_OwnAddress1;         /*!< Specifies the first device own address.
                                         This parameter can be a 7-bit or 10-bit address. */

  uint16_t I2C_AcknowledgedAddress; /*!< Specifies if 7-bit or 10-bit address is acknowledged.
                                         This parameter can be a value of @ref I2C_acknowledged_address */
}I2C_InitTypeDef;

/* Exported constants --------------------------------------------------------*/

/** @defgroup I2C_IP_Config_Parameter
  * @{
  */
#define I2C_DYNAMIC_TAR_UPDATE 0

/**
  * @}
  */

/** @defgroup I2C_mode
  * @{
  */

/**
  * @}
  */

/** @defgroup I2C_Exported_Constants
  * @{
  */
#define IS_I2C_ALL_PERIPH(PERIPH) ((PERIPH) == I2C1)
/**
  * @}
  */

/** @defgroup I2C_mode
  * @{
  */
#define I2C_MODE_MASTER                    ((uint16_t)0x0021)
#define I2C_MODE_SLAVE                     ((uint16_t)0x0000)
#define IS_I2C_MODE(MODE) (((MODE) == I2C_Mode_MASTER) || \
                           ((MODE) == I2C_Mode_SLAVE))
/**
  * @}
  */

/** @defgroup I2C_mode
  * @{
  */

#define I2C_SPEED_STANDARD_MODE   (uint16_t)(0x0002)
#define I2C_SPEED_FAST_MODE       (uint16_t)(0x0004)
#define I2C_SPEED_HIGH_MODE       (uint16_t)(0x0006)
/**
  * @}
  */

/** @defgroup I2C_duty_cycle_in_fast_mode
  * @{
  */

#define I2C_DutyCycle_16_9              ((uint16_t)0x4000) /*!< I2C fast mode Tlow/Thigh = 16/9 */
#define I2C_DutyCycle_2                 ((uint16_t)0xBFFF) /*!< I2C fast mode Tlow/Thigh = 2 */
#define IS_I2C_DUTY_CYCLE(CYCLE) (((CYCLE) == I2C_DutyCycle_16_9) || \
                                  ((CYCLE) == I2C_DutyCycle_2))
/**
  * @}
  */


/** @defgroup I2C_transfer_direction
  * @{
  */

#define  I2C_Direction_Transmitter      ((uint16_t)0x0000)
#define  I2C_Direction_Receiver         ((uint16_t)0x0100)
#define IS_I2C_DIRECTION(DIRECTION) (((DIRECTION) == I2C_Direction_Transmitter) || \
                                     ((DIRECTION) == I2C_Direction_Receiver))
/**
  * @}
  */

/** @defgroup I2C_acknowledged_address
  * @{
  */

#define I2C_AcknowledgedAddress_7bit    ((uint16_t)0x4000)
#define I2C_AcknowledgedAddress_10bit   ((uint16_t)0xC000)
#define IS_I2C_ACKNOWLEDGE_ADDRESS(ADDRESS) (((ADDRESS) == I2C_AcknowledgedAddress_7bit) || \
                                             ((ADDRESS) == I2C_AcknowledgedAddress_10bit))
/**
  * @}
  */



/** @defgroup I2C_Cmd
  * @{
  */

#define I2C_CMD_WR                                   ((uint8_t)0x00)
#define I2C_CMD_RD                                   ((uint8_t)0x01)
#define I2C_CMD_STOP                                 ((uint8_t)0x02)
#define I2C_CMD_RESTART                              ((uint8_t)0x04)
/**
  * @}
  */

/** @defgroup I2C_Receive_FIFO_Threshold_Value
  * @{
  */
#define I2C_RX_TL_0                                   ((uint8_t)0x00)
#define I2C_RX_TL_1                                   ((uint8_t)0x01)
#define I2C_RX_TL_2                                   ((uint8_t)0x02)
#define I2C_RX_TL_3                                   ((uint8_t)0x03)
#define I2C_RX_TL_4                                   ((uint8_t)0x04)
#define I2C_RX_TL_5                                   ((uint8_t)0x05)
#define I2C_RX_TL_6                                   ((uint8_t)0x06)
#define I2C_RX_TL_7                                   ((uint8_t)0x07)
#define I2C_RX_TL_8                                   ((uint8_t)0x08)
/**
  * @}
  */
/** @defgroup I2C_Transmit_FIFO_Threshold_Value
  * @{
  */
#define I2C_TX_TL_0                                   ((uint8_t)0x00)
#define I2C_TX_TL_1                                   ((uint8_t)0x01)
#define I2C_TX_TL_2                                   ((uint8_t)0x02)
#define I2C_TX_TL_3                                   ((uint8_t)0x03)
#define I2C_TX_TL_4                                   ((uint8_t)0x04)
#define I2C_TX_TL_5                                   ((uint8_t)0x05)
#define I2C_TX_TL_6                                   ((uint8_t)0x06)
#define I2C_TX_TL_7                                   ((uint8_t)0x07)
#define I2C_TX_TL_8                                   ((uint8_t)0x08)

/**
  * @}
  */

/** @defgroup I2C_interrupts_definition
  * @{
  */

#define I2C_IT_RX_UNDER                    ((uint16_t)0x0001)
#define I2C_IT_RX_OVER                     ((uint16_t)0x0002)
#define I2C_IT_RX_FULL                     ((uint16_t)0x0004)
#define I2C_IT_TX_OVER                     ((uint16_t)0x0008)
#define I2C_IT_TX_EMPTY                    ((uint16_t)0x0010)
#define I2C_IT_RD_REQ                      ((uint16_t)0x0020)
#define I2C_IT_TX_ABORT                    ((uint16_t)0x0040)
#define I2C_IT_RX_DONE                     ((uint16_t)0x0080)
#define I2C_IT_ACTIVITY                    ((uint16_t)0x0100)
#define I2C_IT_STOP_DET                    ((uint16_t)0x0200)
#define I2C_IT_START_DET                   ((uint16_t)0x0400)
#define I2C_IT_GEN_CALL                    ((uint16_t)0x0800)
#define I2C_IT_MST_ON_HOLD                 ((uint16_t)0x2000)

#define IS_I2C_CLEAR_IT(IT) ((((IT) & (uint16_t)0x0FFF) == 0x00) && ((IT) != (uint16_t)0x00))

#define IS_I2C_GET_IT(IT) (((IT) == I2C_IT_RX_UNDER) || ((IT) == I2C_IT_RX_OVER) || \
                           ((IT) == I2C_IT_RX_FULL) || ((IT) == I2C_IT_TX_OVER) || \
                           ((IT) == I2C_IT_TX_EMPTY) || ((IT) == I2C_IT_RD_REQ) || \
                           ((IT) == I2C_IT_TX_ABORT) || ((IT) == I2C_IT_RX_DONE) || \
                           ((IT) == I2C_IT_ACTIVITY) || ((IT) == I2C_IT_STOP_DET) || \
                           ((IT) == I2C_IT_START_DET) || ((IT) == I2C_IT_GEN_CALL)
/**
  * @}
  */

/** @defgroup I2C_flags_definition
  * @{
  */

/**
  * @brief  SR2 register flags
  */

#define I2C_FLAG_SLV_ACTIVITY                   ((uint32_t)0x00000040)
#define I2C_FLAG_MST_ACTIVITY                   ((uint32_t)0x00000020)
#define I2C_FLAG_RFF                  			((uint32_t)0x00000010)
#define I2C_FLAG_RFNE                  			((uint32_t)0x00000008)
#define I2C_FLAG_TFE                    		((uint32_t)0x00000004)
#define I2C_FLAG_TFNF                   		((uint32_t)0x00000002)
#define I2C_FLAG_ACTIVITY                     	((uint32_t)0x00000001)

#define IS_I2C_CLEAR_FLAG(FLAG) ((((FLAG) & (uint16_t)0x007F) == 0x00) && ((FLAG) != (uint16_t)0x00))

#define IS_I2C_GET_FLAG(FLAG) (((FLAG) == I2C_FLAG_ACTIVITY) || ((FLAG) == I2C_FLAG_TFNF) || \
                               ((FLAG) == I2C_FLAG_TFE) || ((FLAG) == I2C_FLAG_RFNE) || \
                               ((FLAG) == I2C_FLAG_RFF) || ((FLAG) == I2C_FLAG_MST_ACTIVITY) || \
                               ((FLAG) == I2C_FLAG_SLV_ACTIVITY))
/**
  * @}
  */

/** @defgroup I2C_own_address1
  * @{
  */

#define IS_I2C_OWN_ADDRESS1(ADDRESS1) ((ADDRESS1) <= 0x3FF)
/**
  * @}
  */

/** @defgroup I2C_clock_speed
  * @{
  */

#define IS_I2C_CLOCK_SPEED(SPEED) (((SPEED) >= 0x1) && ((SPEED) <= 400000))
/**
  * @}
  */

/** @addtogroup Exported_types
  * @{
  */




/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/* Initialization and Configuration functions *********************************/
void I2C_Init(I2C_T* I2Cx, I2C_InitTypeDef* I2C_InitStruct);
void I2C_Cmd(I2C_T* I2Cx, FunctionalState NewState);
void I2C_Set7bitAddress(I2C_T* I2Cx, uint8_t Address);
void I2C_Set10bitAddress(I2C_T* I2Cx, uint16_t Address);

void I2C_GeneralCallCmd(I2C_T* I2Cx, FunctionalState NewState);
void I2C_SetMode(I2C_T* I2Cx, uint16_t Mode);

void I2C_SetTxTirggerLevel(I2C_T* I2Cx, uint8_t ThresholdValue);
void I2C_SetRxTirggerLevel(I2C_T* I2Cx, uint8_t ThresholdValue);

/* Data transfers functions ***************************************************/
void I2C_SendCmd(I2C_T* I2Cx, uint8_t Cmd);
void I2C_SendDataCmd(I2C_T* I2Cx, uint8_t Data, uint8_t Cmd);
uint8_t I2C_ReceiveData(I2C_T* I2Cx);
void I2C_AbortTransfer(I2C_T* I2Cx);

/* DMA transfers management functions *****************************************/
void I2C_DMACmd(I2C_T* I2Cx, FunctionalState NewState);
void I2C_DMATransferDataLevel(I2C_T* I2Cx, uint8_t Level);
void I2C_DMAReceiveDataLevel(I2C_T* I2Cx, uint8_t Level);


/* Interrupts, events and flags management functions **************************/
uint16_t I2C_ReadRegister(I2C_T* I2Cx, uint8_t I2C_Register);

FlagStatus I2C_GetFlagStatus(I2C_T* I2Cx, uint32_t I2C_FLAG);
void I2C_ClearFlag(I2C_T* I2Cx, uint32_t I2C_FLAG);

void I2C_ITConfig(I2C_T* I2Cx, uint16_t I2C_IT, FunctionalState NewState);
ITStatus I2C_GetITStatus(I2C_T* I2Cx, uint16_t I2C_IT);
ITStatus I2C_GetRawITStatus(I2C_T* I2Cx, uint16_t I2C_IT);
void I2C_ClearITPendingBit(I2C_T* I2Cx, uint16_t I2C_IT);

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif

/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/
