/*
 * stm32f407xx_spi_driver.h
 * Created on: 2020. 12. 28.
 * Author: Josh W.
 * 
 * Required APIS:
 * 1. Clock Control
 * 2. Init and De-init
 * 3. Read and Write
 * 4. IRQ Configuration and ISR Handling (interrupt handling)
 * 5. Peripheral Clock Setup (EN/DI)
 * 
 * Summary: this file contains all the APIs for SPI peripheral driver for STM32F407xx microcontroller
 */

#ifndef STM32F407XX_SPI_DRIVER_H_
#define STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct {
    uint8_t SPI_DeviceMode; /* possible values from @SPI_DeviceMode = (Master/Slave) or (0/1)*/
    uint8_t SPI_BusConfig;  /* possible values from @SPI_BusConfig  */
    uint8_t SPI_SclkSpeed;  /* possible values from @SPI_SclkSpeed  */
    uint8_t SPI_DFF;        /* possible values from @SPI_DFF        */
    uint8_t SPI_CPOL;       /* possible values from @SPI_CPOL       */
    uint8_t SPI_CPHA;       /* possible values from @SPI_CPHA       */
    uint8_t SPI_SSM;        /* possible values from @SPI_SSM        */
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct {
    SPI_RegDef_t *pSPIx;    /* This holds the base address of SPIx(x:0,1,2) peripheral */
    SPI_Config_t SPIConfig; /* This holds SPIx peripheral configuration settings */
    uint8_t *pTxBuffer;     /* To store the app. Tx buffer address */
    uint8_t *pRxBuffer;     /* To store the app. Rx buffer address */
    uint32_t TxLen;         /* To store Tx len   */
    uint32_t RxLen;         /* To store Rx len   */
    uint8_t TxState;        /* To store Tx state */
    uint8_t RxState;        /* To store Rx state */
}SPI_Handle_t;

/*
 * @SPI_DeviceMode
 *
 * MSTR bit = 0 (Slave), MSTR = 1 (Master)
 * 
 * Slave does not generate the clock, it only receives the clock from the master
 */
#define SPI_DEVICE_MODE_MASTER 1
#define SPI_DEVICE_MODE_SLAVE  0

/*
 * @SPI_BusConfig
 *
 * BusConfig = Full Duplex (FD), Half Duplex (HD), Simplex RX Only, Simplex TX Only
 * 
 * Full Duplex (FD) = TX and RX both are enabled
 * Half Duplex (HD) = TX and RX both are enabled but RX is not used
 *      BIDIMODE = 0 (TX and RX both are enabled, 2 line uni-directional data mode), 
 *      BIDIMODE = 1 (TX and RX both are enabled but TX is not used, this must be used if in Half Duplex mode, 1 line bi-directional data mode)) 
 * 
 *      BIDIOE = 0 (RX only), this bit controls the output enable of bi-directional mode (one line tx or rx)
 *      BIDIOE = 1 (TX only)
 * 
 * if using Full Duplex then we DONT CARE about BIDIOE. if using Half Duplex then we MUST CARE about BIDIOE.
 */
#define SPI_BUS_CONFIG_FD             1
#define SPI_BUS_CONFIG_HD             2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2             	0
#define SPI_SCLK_SPEED_DIV4             	1
#define SPI_SCLK_SPEED_DIV8             	2
#define SPI_SCLK_SPEED_DIV16             	3
#define SPI_SCLK_SPEED_DIV32             	4
#define SPI_SCLK_SPEED_DIV64             	5
#define SPI_SCLK_SPEED_DIV128             	6
#define SPI_SCLK_SPEED_DIV256             	7

// #define SPI_CR1_BR_DIV2   0x0000
// #define SPI_CR1_BR_DIV4   0x0008
// #define SPI_CR1_BR_DIV8   0x0010
// #define SPI_CR1_BR_DIV16  0x0018
// #define SPI_CR1_BR_DIV32  0x0020
// #define SPI_CR1_BR_DIV64  0x0028
// #define SPI_CR1_BR_DIV128 0x0030
// #define SPI_CR1_BR_DIV256 0x0038

/*
 * @SPI_DFF
 *
 * Inside CR1 register, DFF bit = 0 (8-bit data frame format is selected for transmission/reception), DFF = 1 (16-bit data frame format is selected for transmission/reception)
 * if DFF = 1, then we must send 2 bytes of data for each transmission because the shift register is 16 bits wide.
 */
#define SPI_DFF_8BITS 0
#define SPI_DFF_16BITS 1

/*
 * @SPI_CPOL
 *
 * Inside CR1 register, CPOL bit = 0 (CLK to 0 when idle), CPOL = 1 (CLK to 1 when idle)
 */
#define SPI_CPOL_LOW 0
#define SPI_CPOL_HIGH 1

/*
 * @SPI_CPHA
 *
 * Inside CR1 register, CPHA bit = 0 (The first clock transition is the first data capture edge), CPHA = 1 (The second clock transition is the first data capture edge)
 */
#define SPI_CPHA_LOW 0
#define SPI_CPHA_HIGH 1

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI 0 /* Hardware Slave Management */
#define SPI_SSM_EN 1 /* Software Slave Management */

/*
 * Possible SPI Application states
 */
#define SPI_READY          0
#define SPI_BUSY_IN_RX     1
#define SPI_BUSY_IN_TX     2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT 1
#define SPI_EVENT_RX_CMPLT 2
#define SPI_EVENT_OVR_ERR  3
#define SPI_EVENT_CRC_ERR  4
// #define SPI_EVENT_FRE_ERR  5
// #define SPI_EVENT_MODF_ERR 6
// #define SPI_EVENT_UDR_ERR  7


/**********************************************************************************************
 *                          SPI related status flags definitions
 **********************************************************************************************/
#define SPI_RXNE_FLAG	(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG	(1 << SPI_SR_TXE)
#define SPI_BUSY_FLAG	(1 << SPI_SR_BSY)
#define SPI_OVR_FLAG	(1 << SPI_SR_OVR)
// #define SPI_FRE_FLAG	(1 << SPI_SR_FRE)
// #define SPI_MODF_FLAG	(1 << SPI_SR_MODF)
// #define SPI_UDR_FLAG	(1 << SPI_SR_UDR)
// #define SPI_CRCERR_FLAG	(1 << SPI_SR_CRCERR)

/**********************************************************************************************
 *                          API's supported by this driver (API Prototypes)
 *                  For more information about the APIs check the function definitions
 **********************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive (Polling based = Blocking API)
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len); /* length must always be in uint32_t (this is a standard) */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len); /* length must always be in uint32_t (this is a standard) */

/*
 * Data Send and Receive (Interrupt based = Non-Blocking API)
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * Data Send and Receive (DMA based = Non-Blocking API)
 */

/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* STM32F407XX_SPI_DRIVER_H_ */