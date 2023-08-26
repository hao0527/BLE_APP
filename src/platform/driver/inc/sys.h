/**************************************************************************//**
 * @file     sys.h
 * @version  V1.00
 * $Revision: 15 $
 * $Date: 15/06/04 5:18p $
 * @brief    PN102 series SYS driver header file
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __SYS_H__
#define __SYS_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup PN102_Device_Driver PN102 Device Driver
  @{
*/

/** @addtogroup PN102_SYS_Driver SYS Driver
  @{
*/

/** @addtogroup PN102_SYS_EXPORTED_CONSTANTS SYS Exported Constants
  @{
*/
/*---------------------------------------------------------------------------------------------------------*/
/*  Module Reset Control Resister constant definitions.                                                    */
/*---------------------------------------------------------------------------------------------------------*/

#define ADC_RST  ((0x4<<24) | SYS_IPRST1_ADCRST_Pos    ) /*!< ADC  reset is one of the SYS_ResetModule parameter */
#define ACMP_RST ((0x4<<24) | SYS_IPRST1_ACMPRST_Pos   ) /*!< ACMP reset is one of the SYS_ResetModule parameter */
#define PWM0_RST  ((0x4<<24) | SYS_IPRST1_PWM0RST_Pos    ) /*!< PWM  reset is one of the SYS_ResetModule parameter */
#define	WWDT_RST  ((0x4<<24) | SYS_IPRST1_WWDTRST_Pos    ) /*!< WWDT  reset is one of the SYS_ResetModule parameter */
#define	WDT_RST  ((0x4<<24) | SYS_IPRST1_WDTRST_Pos    ) /*!< WWDT  reset is one of the SYS_ResetModule parameter */
#define UART0_RST ((0x4<<24) | SYS_IPRST1_UART0RST_Pos ) /*!< UART0 reset is one of the SYS_ResetModule parameter */
#define UART1_RST ((0x4<<24) | SYS_IPRST1_UART1RST_Pos ) /*!< UART1 reset is one of the SYS_ResetModule parameter */
#define SPI0_RST ((0x4<<24) | SYS_IPRST1_SPI0RST_Pos    ) /*!< SPI0  reset is one of the SYS_ResetModule parameter */
#define SPI1_RST ((0x4<<24) | SYS_IPRST1_SPI1RST_Pos    ) /*!< SPI1  reset is one of the SYS_ResetModule parameter */
#define I2C0_RST ((0x4<<24) | SYS_IPRST1_I2C0RST_Pos  ) /*!< I2C0  reset is one of the SYS_ResetModule parameter */
#define I2C1_RST ((0x4<<24) | SYS_IPRST1_I2C1RST_Pos  ) /*!< I2C1  reset is one of the SYS_ResetModule parameter */
#define TMR2_RST ((0x4<<24) | SYS_IPRST1_TMR2RST_Pos   ) /*!< TMR2 reset is one of the SYS_ResetModule parameter */
#define TMR1_RST ((0x4<<24) | SYS_IPRST1_TMR1RST_Pos   ) /*!< TMR1 reset is one of the SYS_ResetModule parameter */
#define TMR0_RST ((0x4<<24) | SYS_IPRST1_TMR0RST_Pos   ) /*!< TMR0 reset is one of the SYS_ResetModule parameter */
#define GPIO_RST ((0x4<<24) | SYS_IPRST1_GPIORST_Pos   ) /*!< GPIO reset is one of the SYS_ResetModule parameter */


/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector Threshold Voltage Selection constant definitions.                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_BODCTL_BOD_RST_EN             (1UL<<SYS_BODCTL_BODRSTEN_Pos)     /*!< Brown-out Reset Enable */
#define SYS_BODCTL_BOD_INTERRUPT_EN       (0UL<<SYS_BODCTL_BODRSTEN_Pos)     /*!< Brown-out Interrupt Enable */
#define SYS_BODCTL_DISABLE             (3UL<<SYS_BODCTL_BODVL_Pos)              /*!< Setting Brown Out Detector Disable */
#define SYS_BODCTL_BODVL_2_67V             (2UL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 267V */
#define SYS_BODCTL_BODVL_2_4V             (1UL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 2.40V */
#define SYS_BODCTL_BODVL_2_15V             (0UL<<SYS_BODCTL_BODVL_Pos)     /*!< Setting Brown Out Detector Threshold Voltage as 2.15V */


/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_MFP_TYPE_Msk(bit)       (1UL << ((bit) +16)) /*!< TYPE mask for Multiple Function Port */
#define SYS_MFP_ALT_Msk(bit)        (1UL << ((bit) + 8)) /*!< ALT mask for Multiple Function Port */
#define SYS_MFP_MFP_Msk(bit)        (1UL << ((bit)    )) /*!< MFP mask for Multiple Function Port */

#define SYS_MFP_P00_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P00_TADC_CLK       	 0x00000001UL
#define SYS_MFP_P00_PWM0_CH2       	 0x00000100UL
#define SYS_MFP_P00_UART0_TXD        0x00000101UL     /*!< Data transmitter output pin for UART0.           */
#define SYS_MFP_P00_UART1_RTS        0x00010000UL     /*!< Uart RTS pin for UART1.                          */
#define SYS_MFP_P00_Msk              0x00010101UL     /*!< P0_MFP pin 0 mask                                */

#define SYS_MFP_P01_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P01_SPI0_SS          0x00000002UL     /*!< 1st SPI0 slave select pin.                       */
#define SYS_MFP_P01_PWM0_CH3       	 0x00000200UL
#define SYS_MFP_P01_UART0_RXD        0x00000202UL     /*!< Data receiver input pin for UART0.               */
#define SYS_MFP_P01_UART1_CTS        0x00020000UL     /*!< Uart CTS pin for UART1.                          */
#define SYS_MFP_P01_SPI2_SS          0x00020002UL     /*!< 1st SPI2 slave select pin.                       */
#define SYS_MFP_P01_Msk              0x00020202UL     /*!< P0_MFP pin 1 mask                                */

#define SYS_MFP_P02_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P02_Msk              0x00040404UL     /*!< General purpose digital I/O pin.                 */

#define SYS_MFP_P03_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P03_SPI1_SS          0x00000008UL     /*!< 1st SPI1 slave select pin.                       */
#define SYS_MFP_P03_BLE_DBG23        0x00000800UL
#define SYS_MFP_P03_BLE_DBG31        0x00000808UL
#define SYS_MFP_P03_SPI3_SS          0x00080000UL     /*!< 1st SPI3 slave select pin.                       */
#define SYS_MFP_P03_I2C0_SDA         0x00080008UL     /*!< I2C0 SDA pin.                                    */
#define SYS_MFP_P03_Msk              0x00080808UL

#define SYS_MFP_P04_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P04_TADC_DATH      	 0x00000010UL
#define SYS_MFP_P04_SPI0_SS          0x00001000UL     /*!< 1st SPI0 slave select pin.                       */
#define SYS_MFP_P04_PWM0_CH5         0x00001010UL     /*!< PWM0 channel5 output/capture input.              */
#define SYS_MFP_P04_SPI2_SS          0x00100000UL     /*!< 1st SPI2 slave select pin.                       */
#define SYS_MFP_P04_BLE_DBG08        0x00100010UL
#define SYS_MFP_P04_BLE_DBG00        0x00101000UL
#define SYS_MFP_P04_Msk              0x00101010UL     /*!< P0_MFP pin 4 mask                                */

#define SYS_MFP_P05_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P05_BLE_DBG14        0x00000020UL
#define SYS_MFP_P05_SPI0_MOSI        0x00002000UL     /*!< 1st SPI0 MOSI (Master Out, Slave In) pin.        */
#define SYS_MFP_P05_PWM0_CH4         0x00002020UL     /*!< PWM0 channel4 output/capture input.              */
#define SYS_MFP_P05_SPI2_MOSI        0x00200000UL     /*!< 1st SPI2 MOSI pin.                               */
#define SYS_MFP_P05_UART1_RXD        0x00200020UL     /*!< Uart CTS pin for UART0.                          */
#define SYS_MFP_P05_UART0_CTS        0x00202000UL     /*!< Uart CTS pin for UART0.                          */
#define SYS_MFP_P05_BLE_DBG06        0x00202020UL
#define SYS_MFP_P05_32K              0x00202020UL     /*!< P0_MFP pin 5 mask  								*/

#define SYS_MFP_P06_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P06_BLE_DBG13        0x00000040UL
#define SYS_MFP_P06_SPI0_MISO        0x00004000UL     /*!< 1st SPI0 MISO (Master In, Slave Out) pin.        */
#define SYS_MFP_P06_PWM0_CH1         0x00004040UL     /*!< PWM0 channel1 output/capture input.              */
#define SYS_MFP_P06_SPI2_MISO        0x00400000UL     /*!< 1st SPI2 MISO (Master In, Slave Out) pin.        */
#define SYS_MFP_P06_UART1_TXD        0x00400040UL     /*!< Uart TX Data pin for UART1.                      */
#define SYS_MFP_P06_UART0_RTS        0x00404000UL     /*!< Uart RTS pin for UART0.                          */
#define SYS_MFP_P06_BLE_DBG05        0x00404040UL
#define SYS_MFP_P06_32K              0x00404040UL     /*!< P0_MFP pin 6 mask                                */

#define SYS_MFP_P07_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P07_I2C1_SCL         0x00000080UL     /*!< I2C1 SCL.                                        */
#define SYS_MFP_P07_SPI0_CLK         0x00008000UL     /*!< SPI0 serial clock pin.                           */
#define SYS_MFP_P07_PWM0_CH0         0x00008080UL     /*!< PWM0 channel0 output/capture input.              */
#define SYS_MFP_P07_SPI2_CLK         0x00800000UL     /*!< SPI2 serial clock pin.                           */
#define SYS_MFP_P07_BLE_DBG12        0x00800080UL
#define SYS_MFP_P07_BLE_DBG04        0x00808000UL
#define SYS_MFP_P07_Msk              0x00808080UL     /*!< P0_MFP pin 7 mask                                */

#define SYS_MFP_P10_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P10_ADC_CH1          0x00000001UL     /*!< ADC channel1 analog input.                       */
#define SYS_MFP_P10_PWM0_CH3         0x00000100UL     /*!< PWM0 channel3 output/capture input.              */
#define SYS_MFP_P10_BLE_DBG01        0x00000101UL
#define SYS_MFP_P10_BLE_DBG09        0x00010000UL
#define SYS_MFP_P10_SPI1_MOSI        0x00010001UL     /*!< SPI1 MOSI pin.                                   */
#define SYS_MFP_P10_SPI2_MOSI        0x00010100UL     /*!< SPI2 MOSI pin.                                   */
#define SYS_MFP_P10_Msk              0x00010101UL     /*!< P1_MFP pin 0 mask                                */

#define SYS_MFP_P11_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P11_SPI1_CLK         0x00000002UL     /*!< SPI1 serial clock pin.                           */
#define SYS_MFP_P11_SPI3_CLK         0x00000202UL     /*!< SPI3 serial clock pin.                           */
#define SYS_MFP_P11_I2C0_SCL         0x00020000UL     /*!< I2C0 SCL clock pin.                           */
#define SYS_MFP_P11_BLE_DBG22        0x00020002UL
#define SYS_MFP_P11_Msk              0x00020202UL     /*!< P1_MFP pin 1 mask                                */

#define SYS_MFP_P12_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P12_ADC_CH2          0x00000004UL     /*!< ADC channel2 analog input.                       */
#define SYS_MFP_P12_UART0_RXD        0x00000400UL     /*!< Data receiver input pin for UART0.               */
#define SYS_MFP_P12_UART1_CTS        0x00000404UL     /*!< Uart CTS pin for UART1.                          */
#define SYS_MFP_P12_PWM0_CH0         0x00040000UL     /*!< PWM0 channel0 output/capture input.              */
#define SYS_MFP_P12_BLE_DBG02        0x00040004UL
#define SYS_MFP_P12_BLE_DBG10        0x00040400UL
#define SYS_MFP_P12_Msk              0x00040404UL     /*!< P1_MFP pin 2 mask                                */

#define SYS_MFP_P13_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P13_ADC_CH3          0x00000008UL     /*!< ADC channel3 analog input.                       */
#define SYS_MFP_P13_UART0_TXD        0x00000800UL     /*!< Data transmitter output pin for UART0.           */
#define SYS_MFP_P13_UART1_RTS        0x00000808UL     /*!< Uart RTS pin for UART1.                          */
#define SYS_MFP_P13_PWM0_CH1         0x00080000UL     /*!< PWM0 channel1 output/capture input.              */
#define SYS_MFP_P13_BLE_DBG03        0x00080008UL
#define SYS_MFP_P13_BLE_DBG11        0x00080800UL
#define SYS_MFP_P13_Msk              0x00080808UL     /*!< P1_MFP pin 3 mask                                */

#define SYS_MFP_P14_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P14_ADC_CH4          0x00000010UL     /*!< ADC channel4 analog input.                       */
#define SYS_MFP_P14_UART1_RXD        0x00001000UL     /*!< Data receiver input pin for UART1.               */
#define SYS_MFP_P14_UART0_CTS        0x00001010UL     /*!< Uart CTS pin for UART0.                          */
#define SYS_MFP_P14_PWM0_CH4         0x00100000UL     /*!< PWM0 channel4 output/capture input.              */
#define SYS_MFP_P14_BLE_DBG16        0x00100010UL
#define SYS_MFP_P14_BLE_DBG24        0x00101000UL
#define SYS_MFP_P14_Msk              0x00101010UL     /*!< P1_MFP pin 4 mask                                */

#define SYS_MFP_P15_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P15_ADC_CH5          0x00000020UL     /*!< ADC channel5 analog input.                       */
#define SYS_MFP_P15_UART1_TXD        0x00002000UL     /*!< Data transmitter output pin for UART1.           */
#define SYS_MFP_P15_UART0_RTS        0x00002020UL     /*!< Uart RTS pin for UART0.                          */
#define SYS_MFP_P15_PWM0_CH6         0x00200000UL     /*!< PWM0 channel6 output/capture input.              */
#define SYS_MFP_P15_BLE_DBG17        0x00200020UL
#define SYS_MFP_P15_BLE_DBG25        0x00202000UL
#define SYS_MFP_P15_Msk              0x00202020UL     /*!< P1_MFP pin 5 mask                                */

#define SYS_MFP_P16_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P16_SPI1_MOSI        0x00000040UL     /*!< SPI0 MOSI pin.                                   */
#define SYS_MFP_P16_BLE_DBG29        0x00004000UL
#define SYS_MFP_P16_SPI3_MOSI        0x00004040UL     /*!< SPI3 MOSI pin.                                   */
#define SYS_MFP_P16_UART0_RXD        0x00400000UL     /*!< Uart tx data pin for UART0.                      */
#define SYS_MFP_P16_UART1_CTS        0x00400040UL     /*!< Uart CTS pin for UART1.                          */
#define SYS_MFP_P16_BLE_DBG21        0x00404000UL
#define SYS_MFP_P16_Msk              0x00404040UL     /*!< P1_MFP pin 6 mask                                */

#define SYS_MFP_P17_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P17_SPI1_MISO        0x00000080UL     /*!< SPI0 MISO pin.                                   */
#define SYS_MFP_P17_BLE_DBG28        0x00008000UL
#define SYS_MFP_P17_SPI3_MISO        0x00008080UL     /*!< SPI3 MISO pin.                                   */
#define SYS_MFP_P17_UART0_TXD        0x00800000UL     /*!< Uart tx data pin for UART0.                      */
#define SYS_MFP_P17_UART1_RTS        0x00800080UL     /*!< Uart RTS pin for UART1.                          */
#define SYS_MFP_P17_BLE_DBG20        0x00808000UL
#define SYS_MFP_P17_Msk              0x00808080UL     /*!< P1_MFP pin 7 mask                                */

#define SYS_MFP_P20_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P20_SPI1_SS          0x00000001UL     /*!< 1st SPI1 slave select pin.                       */
#define SYS_MFP_P20_INT2             0x00000100UL     /*!< External interrupt2 input pin.                   */
#define SYS_MFP_P20_BLE_DBG15        0x00000101UL
#define SYS_MFP_P20_SPI3_SS          0x00010000UL     /*!< 1st SPI3 slave select pin.                       */
#define SYS_MFP_P20_I2C1_SDA         0x00010001UL     /*!< I2C1 SDA pin.                                  */
#define SYS_MFP_P20_BLE_DBG07        0x00010100UL
#define SYS_MFP_P20_Msk              0x00010101UL     /*!< P2_MFP pin 0 mask                                */

#define SYS_MFP_P21_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P21_EXTCLK           0x00000002UL     /*!< External clock select pin.                       */
#define SYS_MFP_P21_Msk              0x00020202UL     /*!< P2_MFP pin 1 mask                                */

#define SYS_MFP_P22_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P22_PWM0_CH0         0x00000400UL     /*!< PWM0 channel0 output/capture input.              */
#define SYS_MFP_P22_I2C1_SCL         0x00000404UL     /*!< I2C1 clock pin.                                  */
#define SYS_MFP_P22_SPI1_CLK         0x00040000UL     /*!< 1st SPI1 CLK pin.                                */
#define SYS_MFP_P22_SPI3_CLK         0x00040004UL     /*!< 1st SPI3 CLK pin.                                */
#define SYS_MFP_P22_BLE_DBG24        0x00040400UL
#define SYS_MFP_P22_BLE_DBG16        0x00040404UL
#define SYS_MFP_P22_Msk              0x00040404UL     /*!< P2_MFP pin 2 mask                                */

#define SYS_MFP_P23_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P23_BLE_DBG25        0x00000008UL
#define SYS_MFP_P23_PWM0_CH1         0x00000800UL     /*!< PWM0 channel1 output/capture input.              */
#define SYS_MFP_P23_I2C1_SDA         0x00000808UL     /*!< I2C1 data input/output pin.                      */
#define SYS_MFP_P23_SPI1_SS          0x00080000UL     /*!< 1st SPI1 slave select pin.                       */
#define SYS_MFP_P23_SPI3_SS          0x00080008UL     /*!< 1st SPI3 slave select pin.                       */
#define SYS_MFP_P23_BLE_DBG17        0x00080800UL
#define SYS_MFP_P23_Msk              0x00080808UL     /*!< P2_MFP pin 3 mask                                */

#define SYS_MFP_P24_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P24_UART1_RXD        0x00000010UL     /*!< Data receiver input pin for UART1.               */
#define SYS_MFP_P24_PWM0_CH2         0x00001000UL     /*!< PWM0 channel2 output/capture input.              */
#define SYS_MFP_P24_UART0_CTS        0x00001010UL     /*!< Uart CTS pin for UART0.                          */
#define SYS_MFP_P24_BLE_DBG26        0x00100000UL
#define SYS_MFP_P24_BLE_DBG18        0x00100010UL
#define SYS_MFP_P24_Msk              0x00101010UL     /*!< P2_MFP pin 4 mask                                */

#define SYS_MFP_P25_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P25_UART1_TXD        0x00000020UL     /*!< Data transmitter output pin for UART1.           */
#define SYS_MFP_P25_PWM0_CH3         0x00002000UL     /*!< PWM0 channel3 output/capture input.              */
#define SYS_MFP_P25_UART0_RTS        0x00002020UL     /*!< Uart RTS pin for UART1.                          */
#define SYS_MFP_P25_BLE_DBG27        0x00200000UL
#define SYS_MFP_P25_BLE_DBG19        0x00200020UL
#define SYS_MFP_P25_Msk              0x00202020UL     /*!< P2_MFP pin 5 mask                                */

#define SYS_MFP_P26_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P26_SPI0_SS          0x00000040UL     /*!< 1st SPI0 slave select pin.                       */
#define SYS_MFP_P26_PWM0_CH4         0x00004000UL     /*!< PWM0 channel4 output/capture input.              */
#define SYS_MFP_P26_SPI2_SS          0x00004040UL     /*!< 1st SPI2 slave select pin.                       */
#define SYS_MFP_P26_RF_SPI_CSK       0x00400000UL
#define SYS_MFP_P26_Msk              0x00404040UL     /*!< P2_MFP pin 6 mask                                */

#define SYS_MFP_P27_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P27_ADC_CH8          0x00000080UL     /*!< ADC channel8 analog input.                       */
#define SYS_MFP_P27_TM2_EXT          0x00008000UL     /*!< Timer2 external capture input.                   */
#define SYS_MFP_P27_TADC_VLD         0x00008080UL
#define SYS_MFP_P27_SPI0_CLK         0x00800000UL     /*!< 1st SPI0 CLK pin.                                */
#define SYS_MFP_P27_SPI2_CLK         0x00800080UL     /*!< 1st SPI2 CLK pin.                                */
#define SYS_MFP_P27_BLE_DBG06        0x00808000UL
#define SYS_MFP_P27_BLE_DBG14        0x00808080UL
#define SYS_MFP_P27_Msk              0x00008080UL     /*!< P2_MFP pin 7 mask                                */

#define SYS_MFP_P30_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P30_PWM0_CH7         0x00000001UL     /*!< PWM0 channel7 output/capture input.              */
#define SYS_MFP_P30_UART1_RXD        0x00000100UL     /*!< Uart RX Data pin for UART1.                      */
#define SYS_MFP_P30_ADC_CH6          0x00000101UL     /*!< ADC channel6 analog input.                       */
#define SYS_MFP_P30_UART0_CTS        0x00010000UL     /*!< Uart CTS pin for UART0.                          */
#define SYS_MFP_P30_BLE_DBG18        0x00010001UL
#define SYS_MFP_P30_BLE_DBG26        0x00010100UL
#define SYS_MFP_P30_Msk              0x00010101UL     /*!< P3_MFP pin 0 mask                                */

#define SYS_MFP_P31_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P31_PWM0_CH5         0x00000002UL     /*!< PWM0 channel5 output/capture input.              */
#define SYS_MFP_P31_UART1_TXD        0x00000200UL     /*!< Uart RX Data pin for UART1.                      */
#define SYS_MFP_P31_ADC_CH7          0x00000202UL     /*!< ADC channel7 analog input.                       */
#define SYS_MFP_P31_UART0_RTS        0x00020000UL     /*!< Uart CTS pin for UART1.                          */
#define SYS_MFP_P31_BLE_DBG19        0x00020002UL
#define SYS_MFP_P31_BLE_DBG27        0x00020200UL
#define SYS_MFP_P31_Msk              0x00020202UL     /*!< P3_MFP pin 1 mask                                */

#define SYS_MFP_P32_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P32_INT0             0x00000004UL     /*!< External interrupt0 input pin.                   */
#define SYS_MFP_P32_TM0_EXT          0x00000400UL     /*!< Timer0 external capture input.                   */
#define SYS_MFP_P32_STADC            0x00000404UL     /*!< ADC external trigger input.                      */
#define SYS_MFP_P32_SPI0_MISO        0x00040000UL     /*!< 1st SPI0 MISO pin.                               */
#define SYS_MFP_P32_SPI2_MISO        0x00040004UL     /*!< 1st SPI2 MISO pin.                               */
#define SYS_MFP_P32_BLE_DBG20        0x00040400UL
#define SYS_MFP_P32_BLE_DBG12        0x00040404UL
#define SYS_MFP_P32_Msk              0x00040404UL     /*!< P3_MFP pin 2 mask                                */

#define SYS_MFP_P34_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P34_TM0_CNT_OUT      0x00000010UL     /*!< Timer0  event counter input/toggle output.       */
#define SYS_MFP_P34_I2C0_SDA         0x00001000UL     /*!< I2C0 data input/output pin.                      */
#define SYS_MFP_P34_BLE_DBG21        0x00001010UL
#define SYS_MFP_P34_BLE_DBG13        0x00100000UL
#define SYS_MFP_P34_Msk              0x00101010UL     /*!< P3_MFP pin 4 mask                                */

#define SYS_MFP_P35_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P35_TM1_CNT_OUT      0x00000020UL     /*!< Timer1  event counter input/toggle output.       */
#define SYS_MFP_P35_I2C0_SCL         0x00002000UL     /*!< I2C0 clock pin.                                  */
#define SYS_MFP_P35_BLE_DBG22        0x00002020UL
#define SYS_MFP_P35_BLE_DBG14        0x00200000UL
#define SYS_MFP_P35_Msk              0x00202020UL     /*!< P3_MFP pin 5 mask                                */

#define SYS_MFP_P36_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P36_TM1_EXT          0x00000040UL     /*!< Timer1 external capture input.                   */
#define SYS_MFP_P36_CLKO             0x00004000UL     /*!< Clock output.                                    */
#define SYS_MFP_P36_SPI1_SS          0x00004040UL     /*!< 1st SPI0 SS pin.                                 */
#define SYS_MFP_P36_SPI3_SS          0x00400000UL     /*!< 1st SPI2 SS pin.                                 */
#define SYS_MFP_P36_BLE_DBG23        0x00400040UL
#define SYS_MFP_P36_BLE_DBG15        0x00404000UL
#define SYS_MFP_P36_Msk              0x00404040UL     /*!< P3_MFP pin 6 mask                                */

#define SYS_MFP_P46_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P46_ICE_CLK          0x00000040UL     /*!< Serial wired debugger clock pin.                 */
#define SYS_MFP_P46_UART1_RXD        0x00004000UL     /*!< Data receiver input pin for UART1.               */
#define SYS_MFP_P46_I2C0_SCL         0x00004040UL     /*!< I2C0 SCL.                                        */
#define SYS_MFP_P46_SPI1_CLK         0x00400000UL     /*!< SPI1 CLK.                                        */
#define SYS_MFP_P46_SPI2_CLK         0x00400040UL     /*!< SPI2 CLK.                                        */
#define SYS_MFP_P46_Msk              0x00404040UL     /*!< P4_MFP pin 6 mask                                */

#define SYS_MFP_P47_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P47_ICE_DAT          0x00000080UL     /*!< Serial wired debugger data pin.                  */
#define SYS_MFP_P47_UART1_TXD        0x00008000UL     /*!< Data transmitter output pin for UART1.           */
#define SYS_MFP_P47_I2C0_SDA         0x00008080UL     /*!< I2C0 SDA.                                        */
#define SYS_MFP_P47_SPI1_SS          0x00800000UL     /*!< SPI1 SS.                                        */
#define SYS_MFP_P47_SPI2_SS          0x00800080UL     /*!< SPI2 SS.                                        */
#define SYS_MFP_P47_Msk              0x00808080UL     /*!< P4_MFP pin 7 mask                                */

#define SYS_MFP_P50_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P50_UART1_RTS        0x00000001UL     /*!< Uart CTS pin for UART0.                          */
#define SYS_MFP_P50_I2C1_SDA         0x00000100UL     /*!< I2C1 data input/output pin.                      */
#define SYS_MFP_P50_UART0_TXD        0x00000101UL     /*!< Data transmitter output pin for UART0.           */
#define SYS_MFP_P50_PWM0_CH7         0x00010000UL     /*!< PWM0 channel7 output/capture input.              */
#define SYS_MFP_P50_BLE_DBG05        0x00010001UL
#define SYS_MFP_P50_BLE_DBG13        0x00010100UL
#define SYS_MFP_P50_Msk              0x00010101UL     /*!< P5_MFP pin 0 mask                                */

#define SYS_MFP_P51_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P51_UART1_CTS        0x00000002UL     /*!< Uart CTS pin for UART1.                          */
#define SYS_MFP_P51_I2C1_SCL         0x00000200UL     /*!< I2C1 clock pin.                                  */
#define SYS_MFP_P51_UART0_RXD        0x00000202UL     /*!< Data receiver input pin for UART0.               */
#define SYS_MFP_P51_PWM0_CH6         0x00020000UL     /*!< PWM0 channel6 output/capture input.              */
#define SYS_MFP_P51_BLE_DBG04        0x00020002UL
#define SYS_MFP_P51_BLE_DBG12        0x00020200UL
#define SYS_MFP_P51_Msk              0x00020202UL     /*!< P5_MFP pin 1 mask                                */

#define SYS_MFP_P52_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P52_INT1             0x00000004UL     /*!< External interrupt1 input pin.                   */
#define SYS_MFP_P52_EXT_WAKEUP       0x00000400UL
#define SYS_MFP_P52_Msk              0x00040404UL     /*!< P5_MFP pin 2 mask                                */

#define SYS_MFP_P53_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P53_ADC_CH0          0x00000008UL     /*!< ADC channel0 analog input.                       */
#define SYS_MFP_P53_PWM0_CH2         0x00000800UL     /*!< PWM0 channel2 output/capture input.              */
#define SYS_MFP_P53_RF_SPI_CSN       0x00000808UL
#define SYS_MFP_P53_BLE_DBG00        0x00080000UL
#define SYS_MFP_P53_BLE_DBG08        0x00080008UL
#define SYS_MFP_P53_SPI1_MISO        0x00080800UL     /*!< SPI1 MISO pin.                                   */
#define SYS_MFP_P53_SPI2_MISO        0x00080808UL     /*!< SPI2 MISO pin.                                   */
#define SYS_MFP_P53_Msk              0x00080808UL     /*!< P5_MFP pin 3 mask                                */

#define SYS_MFP_P54_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P54_TM2_CNT_OUT      0x00000010UL     /*!< Timer2  event counter input/toggle output.       */
#define SYS_MFP_P54_TADC_DATL        0x00001000UL
#define SYS_MFP_P54_RF_SPI_MOSI      0x00001010UL
#define SYS_MFP_P54_SPI0_MOSI        0x00100000UL     /*!< 1st SPI0 MOSI pin.                               */
#define SYS_MFP_P54_SPI2_MOSI        0x00100010UL     /*!< 1st SPI2 MOSI pin.                               */
#define SYS_MFP_P54_BLE_DBG07        0x00101000UL
#define SYS_MFP_P54_BLE_DBG15        0x00101010UL
#define SYS_MFP_P54_Msk              0x00101010UL     /*!< P5_MFP pin 4 mask                                */

#define SYS_MFP_P55_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P55_RF_SPI_MISO      0x00000020UL
#define SYS_MFP_P55_SPI0_SS          0x00002000UL     /*!< 1st SPI0 SS pin.                                 */
#define SYS_MFP_P55_PWM0_CH5         0x00002020UL     /*!< PWM0 channel5 output/capture input.              */
#define SYS_MFP_P55_SPI2_SS          0x00200000UL     /*!< 1st SPI2 SS pin.                                 */
#define SYS_MFP_P55_BLE_DBG09        0x00200020UL
#define SYS_MFP_P55_BLE_DBG01        0x00202000UL
#define SYS_MFP_P55_Msk              0x00202020UL     /*!< P5_MFP pin 5 mask                                */

#define SYS_MFP_P56_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P56_I2C0_SCL         0x00004000UL     /*!< I2C0 clock pin.                                  */
#define SYS_MFP_P56_PWM0_CH6         0x00004040UL     /*!< PWM0 channel6 output/capture input.              */
#define SYS_MFP_P56_SPI1_MOSI        0x00400000UL     /*!< 1st SPI1 MOSI pin.                               */
#define SYS_MFP_P56_SPI3_MOSI        0x00400040UL     /*!< 1st SPI3 MOSI pin.                               */
#define SYS_MFP_P56_BLE_DBG10        0x00404000UL
#define SYS_MFP_P56_BLE_DBG02        0x00404040UL
#define SYS_MFP_P56_Msk              0x00404040UL     /*!< P5_MFP pin 6 mask                                */

#define SYS_MFP_P57_GPIO             0x00000000UL     /*!< General purpose digital I/O pin.                 */
#define SYS_MFP_P57_I2C0_SDA         0x00008000UL     /*!< I2C0 data input/output pin.                      */
#define SYS_MFP_P57_PWM0_CH7         0x00008080UL     /*!< PWM0 channel7 output/capture input.              */
#define SYS_MFP_P57_SPI1_MISO        0x00800000UL     /*!< 1st SPI1 MISO pin.                               */
#define SYS_MFP_P57_SPI3_MISO        0x00800080UL     /*!< 1st SPI3 MISO pin.                               */
#define SYS_MFP_P57_BLE_DBG11        0x00808000UL
#define SYS_MFP_P57_BLE_DBG03        0x00808080UL
#define SYS_MFP_P57_Msk              0x00008080UL     /*!< P5_MFP pin 6 mask                                */


#define	SYS_BLDBCTL_BODDBSEL_2POW4	 0x00000001UL
#define	SYS_BLDBCTL_BODDBSEL_2POW7	 0x00000002UL
#define	SYS_BLDBCTL_BODDBSEL_2POW9	 0x00000004UL
#define	SYS_BLDBCTL_BODDBSEL_2POW11	 0x00000008UL
#define	SYS_BLDBCTL_BODDBSEL_2POW13	 0x00000010UL
#define	SYS_BLDBCTL_BODDBSEL_2POW15	 0x00000020UL

#define	SYS_BLDBCTL_LVRDBSEL_2POW4	 0x00000100UL
#define	SYS_BLDBCTL_LVRDBSEL_2POW7	 0x00000200UL
#define	SYS_BLDBCTL_LVRDBSEL_2POW9	 0x00000400UL
#define	SYS_BLDBCTL_LVRDBSEL_2POW11	 0x00000800UL
#define	SYS_BLDBCTL_LVRDBSEL_2POW13	 0x00001000UL
#define	SYS_BLDBCTL_LVRDBSEL_2POW15	 0x00002000UL

/*@}*/ /* end of group PN102_SYS_EXPORTED_CONSTANTS */

/** @addtogroup PN102_SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/
/**
  * @brief      Clear Brown-out detector interrupt flag
  * @param      None
  * @return     None
  * @details    This macro clear Brown-out detector interrupt flag.
  */
#define SYS_CLEAR_BOD_INT_FLAG()        (SYS->BODCTL |= SYS_BODCTL_BODIF_Msk)

/**
  * @brief      Set Brown-out detector function to normal mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to normal mode.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_CLEAR_BOD_LPM()             (SYS->BODCTL &= ~SYS_BODCTL_BODLPM_Msk)

/**
  * @brief      Disable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro disable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_BOD()               (SYS->BODCTL = (SYS->BODCTL &~(SYS_BODCTL_BODVL_Msk|SYS_BODCTL_BODEN_Msk))|SYS_BODCTL_BODVL_Msk)

/**
  * @brief      Enable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_BOD()                (SYS->BODCTL = (SYS->BODCTL | SYS_BODCTL_BODEN_Msk))

/**
  * @brief      Get Brown-out detector interrupt flag
  * @param      None
  * @retval     0   Brown-out detect interrupt flag is not set.
  * @retval     >=1 Brown-out detect interrupt flag is set.
  * @details    This macro get Brown-out detector interrupt flag.
  */
#define SYS_GET_BOD_INT_FLAG()          (SYS->BODCTL & SYS_BODCTL_BODIF_Msk)

/**
  * @brief      Get Brown-out detector status
  * @param      None
  * @retval     0   System voltage is higher than BOD threshold voltage setting or BOD function is disabled.
  * @retval     >=1 System voltage is lower than BOD threshold voltage setting.
  * @details    This macro get Brown-out detector output status.
  *             If the BOD function is disabled, this function always return 0.
  */
#define SYS_GET_BOD_OUTPUT()            (SYS->BODCTL & SYS_BODCTL_BODOUT_Msk)

/**
  * @brief      Enable Brown-out detector interrupt function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector interrupt function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_BOD_RST()           (SYS->BODCTL &= ~SYS_BODCTL_BODRSTEN_Msk)

/**
  * @brief      Enable Brown-out detector reset function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detect reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_BOD_RST()            (SYS->BODCTL |= SYS_BODCTL_BODRSTEN_Msk)

/**
  * @brief      Set Brown-out detector function low power mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to low power mode.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_SET_BOD_LPM()               (SYS->BODCTL |= SYS_BODCTL_BODLPM_Msk)

/**
  * @brief      Set Brown-out detector voltage level
  * @param[in]  u32Level is Brown-out voltage level. Including :
  *             - \ref SYS_BODCTL_BODVL_4_4V
  *             - \ref SYS_BODCTL_BODVL_3_7V
  *             - \ref SYS_BODCTL_BODVL_2_7V
  *             - \ref SYS_BODCTL_BODVL_2_2V
  * @return     None
  * @details    This macro set Brown-out detector voltage level.
  *             The write-protection function should be disabled before using this macro.
  */
#define SYS_SET_BOD_LEVEL(u32Level)     (SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODVL_Msk) | (u32Level))

/**
  * @brief      Get reset source is from Brown-out detector reset
  * @param      None
  * @retval     0   Previous reset source is not from Brown-out detector reset
  * @retval     >=1 Previous reset source is from Brown-out detector reset
  * @details    This macro get previous reset source is from Brown-out detect reset or not.
  */
#define SYS_IS_BOD_RST()                (SYS->RSTSTS & SYS_RSTSTS_BODRF_Msk)

/**
  * @brief      Set BOD De-Bounce (glitch) time Control register
  * @param      Mask bit of select.[5:0]
  *				ref:/ SYS_BLDBCTL_BODDBSEL_2POW4 ...
  * @details    This macro get previous reset source is from Brown-out detect reset or not.
  */
#define	SYS_SET_BODDBSEL(u32Msk)		(SYS->BLDBCTL = ((SYS->BLDBCTL & ~SYS_BLDBCTL_BODDBSEL_Msk) | (u32Msk )))

/**
  * @brief      Set LVR De-Bounce (glitch) time Control register
  * @param      Mask bit of select.[13:8]
  *				ref:/ SYS_BLDBCTL_LVRDBSEL_2POW4 ...
  * @details    This macro get previous reset source is from Brown-out detect reset or not.
  */
#define	SYS_SET_LVRDBSEL(u32Msk)		(SYS->BLDBCTL = ((SYS->BLDBCTL & ~SYS_BLDBCTL_LVRDBSEL_Msk) | (u32Msk )))

/**
  * @brief      Get reset source is from CPU reset
  * @param      None
  * @retval     0   Previous reset source is not from CPU reset
  * @retval     >=1 Previous reset source is from CPU reset
  * @details    This macro get previous reset source is from CPU reset.
  */
#define SYS_IS_CPU_RST()                (SYS->RSTSTS & SYS_RSTSTS_CPURF_Msk)

/**
  * @brief      Get reset source is from Power-on Reset
  * @param      None
  * @retval     0   Previous reset source is not from Power-on Reset
  * @retval     >=1 Previous reset source is from Power-on Reset
  * @details    This macro get previous reset source is from Power-on Reset.
  */
#define SYS_IS_POR_RST()                (SYS->RSTSTS & SYS_RSTSTS_PORF_Msk)

/**
  * @brief      Get reset source is from reset pin reset
  * @param      None
  * @retval     0   Previous reset source is not from reset pin reset
  * @retval     >=1 Previous reset source is from reset pin reset
  * @details    This macro get previous reset source is from reset pin reset.
  */
#define SYS_IS_RSTPIN_RST()             (SYS->RSTSTS & SYS_RSTSTS_PINRF_Msk)

/**
  * @brief      Get reset source is from system reset
  * @param      None
  * @retval     0   Previous reset source is not from system reset
  * @retval     >=1 Previous reset source is from system reset
  * @details    This macro get previous reset source is from system reset.
  */
#define SYS_IS_SYSTEM_RST()             (SYS->RSTSTS & SYS_RSTSTS_SYSRF_Msk)

/**
  * @brief      Get reset source is from window watch dog reset
  * @param      None
  * @retval     0   Previous reset source is not from window watch dog reset
  * @retval     >=1 Previous reset source is from window watch dog reset
  * @details    This macro get previous reset source is from window watch dog reset.
  */
#define SYS_IS_WDT_RST()                (SYS->RSTSTS & SYS_RSTSTS_WDTRF_Msk)

/**
  * @brief      Disable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_POR()               (SYS->PORCTL = 0x5AA5)

/**
  * @brief      Enable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_POR()                (SYS->PORCTL = 0)

/**
  * @brief      Clear reset source flag
  * @param[in]  u32RstSrc is reset source. Including :
  *             - \ref SYS_RSTSTS_PORF_Msk
  *             - \ref SYS_RSTSTS_PINRF_Msk
  *             - \ref SYS_RSTSTS_WDTRF_Msk
  *             - \ref SYS_RSTSTS_BODRF_Msk
  *             - \ref SYS_RSTSTS_SYSRF_Msk
  *             - \ref SYS_RSTSTS_CPURF_Msk
  * @return     None
  * @details    This macro clear reset source flag.
  */
#define SYS_CLEAR_RST_SOURCE(u32RstSrc) ((SYS->RSTSTS) = (u32RstSrc) )


/**
  * @brief      Disable register write-protection function
  * @param      None
  * @return     None
  * @details    This function disable register write-protection function.
  *             To unlock the protected register to allow write access.
  */
__STATIC_INLINE void SYS_UnlockReg(void)
{
    do
    {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }
    while(SYS->REGLCTL == 0);
}

/**
  * @brief      Enable register write-protection function
  * @param      None
  * @return     None
  * @details    This function is used to enable register write-protection function.
  *             To lock the protected register to forbid write access.
  */
__STATIC_INLINE void SYS_LockReg(void)
{
    SYS->REGLCTL = 0;
}

void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
uint32_t  SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex, uint32_t u32HoldTime);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);
void SYS_delay_10nop(uint32_t u32NopCnt);

/*@}*/ /* end of group PN102_SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group PN102_SYS_Driver */

/*@}*/ /* end of group PN102_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__SYS_H__
