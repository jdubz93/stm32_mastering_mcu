#ifndef  STM32F407XX_I2C_DRIVER_H_
#define  STM32F407XX_I2C_DRIVER_H_

/*
 * Header file for the I2C peripheral
 * Created on: 02/11/2023
 * Author: Josh Williams
 * Course: Udemy Embedded Systems Masterclass
 */

#include "stm32f407xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct {
    uint32_t I2C_SCLSpeed;
    uint8_t  I2C_DeviceAdrs;
    uint8_t  I2C_ACKCtrl;
    uint16_t I2C_FMDutyCycle;
    uint8_t  I2C_DeviceAdrsMode;
} I2C_Config_t;

/*
 * I2C_Handle structure for I2Cx peripheral
 * pI2Cx - base address of I2Cx peripheral
 * I2C_Config - I2C configuration settings
 * I2C_State - I2C application state
 * pTxBuffer - pointer to Tx buffer
 * pRxBuffer - pointer to Rx buffer
 * TxLen - length of Tx buffer
 * RxLen - length of Rx buffer
 * TxRxState - state of Tx/Rx
 * DevAddr - device address
 * RxSize - size of Rx buffer
 * Sr - repeated start
 */
typedef struct {
    I2C_RegDef_t *pI2Cx;  // base address of I2Cx peripheral
    I2C_Config_t I2C_Config; // I2C configuration settings
    uint8_t *pTxBuffer; // only used in master mode
    uint8_t *pRxBuffer; // only used in slave mode
    uint32_t TxLen; // length of Tx buffer
    uint32_t RxLen; // length of Rx buffer
    uint8_t TxRxState; // state of Tx/Rx
    uint8_t DevAddr; // device address
    uint32_t TxSize; // size of Tx buffer
    uint32_t RxSize; // size of Rx buffer
    uint8_t Sr;     // repeated start
} I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 * I2C_SCL_SPEED_SM - standard mode
 * I2C_SCL_SPEED_FM4K - fast mode 4KHz
 * I2C_SCL_SPEED_FM2K - fast mode 2KHz
 */
#define I2C_SCL_SPEED_SM   100000
#define I2C_SCL_SPEED_FM4K 400000
#define I2C_SCL_SPEED_FM2K 200000

/*
 * @I2C_ACKControl
 * I2C_ACK_ENABLE - enable ACK
 * I2C_ACK_DISABLE - disable ACK
 */
#define I2C_ACK_ENABLE  1
#define I2C_ACK_DISABLE 0

/*
 * @I2C_FMDutyCycle
 * I2C_FM_DUTY_2 - fast mode Tlow/Thigh = 2
 * I2C_FM_DUTY_16_9 - fast mode Tlow/Thigh = 16/9
 */
#define I2C_FM_DUTY_2    0
#define I2C_FM_DUTY_16_9 1

/*
 * I2C application states
 */
typedef enum {
    I2C_READY,
    I2C_BUSY_IN_RX,
    I2C_BUSY_IN_TX
} I2C_State_t;

/*
 * I2C application events
 * I2C_EVNT_TX_CMPLT - Tx complete
 * I2C_EVNT_RX_CMPLT - Rx complete
 * I2C_EVNT_STOP - stop detected
 * I2C_ERROR_BERR - bus error
 * I2C_ERROR_ARLO - arbitration lost
 * I2C_ERROR_AF - acknowledgment failure
 * I2C_ERROR_OVR - overrun/underrun
 * I2C_ERROR_TIMEOUT - timeout
 * I2C_EVNT_DATA_REQ - data request
 * I2C_EVNT_DATA_RCV - data received
 */
typedef enum {
    I2C_EVNT_TX_CMPLT,
    I2C_EVNT_RX_CMPLT,
    I2C_EVNT_STOP,
    I2C_ERROR_BERR,
    I2C_ERROR_ARLO,
    I2C_ERROR_AF,
    I2C_ERROR_OVR,
    I2C_ERROR_TIMEOUT,
    I2C_EVNT_DATA_REQ,
    I2C_EVNT_DATA_RCV
} I2C_AppEvent_t;

extern uint16_t AHB_PreScaler[8];
extern uint8_t APB1_PreScaler[4];
#define RCC_GetPLLOutputClock(void) (0)

/*
 * I2C status flags
 */
#define I2C_FLAG_SB             (1 << I2C_SR1_SB)           /* Start bit */
#define I2C_FLAG_ADDR           (1 << I2C_SR1_ADDR)         /* Address sent */
#define I2C_FLAG_BTF            (1 << I2C_SR1_BTF)          /* Byte transfer finished */
#define I2C_FLAG_ADD10          (1 << I2C_SR1_ADD10)        /* 10-bit header sent */
#define I2C_FLAG_STOPF          (1 << I2C_SR1_STOPF)        /* Stop detected */
#define I2C_FLAG_RXNE           (1 << I2C_SR1_RXNE)         /* Data register not empty */
#define I2C_FLAG_TXE            (1 << I2C_SR1_TXE)          /* Data register empty */
#define I2C_FLAG_BERR           (1 << I2C_SR1_BERR)         /* Bus error */
#define I2C_FLAG_ARLO           (1 << I2C_SR1_ARLO)         /* Arbitration lost condition for Master mode */
#define I2C_FLAG_AF             (1 << I2C_SR1_AF)           /* Acknowledgment failure after address/ data transmission */
#define I2C_FLAG_OVR            (1 << I2C_SR1_OVR)          /* Overrun/underrun */
#define I2C_FLAG_PECERR         (1 << I2C_SR1_PECERR)      /* PEC error in reception\
                                                            * packet error checking generation or verification\
                                                            *   PEC value can be transmitted as last byte in Tx mode\
                                                            *   PEC error checking for last received byte 
                                                            */
#define I2C_FLAG_TIMEOUT        (1 << I2C_SR1_TIMEOUT)      /* Timeout or Tlow error */
#define I2C_FLAG_SMBALERT       (1 << I2C_SR1_SMBALERT)     /* SMBus alert */
#define I2C_FLAG_SMBDEFAULT     (1 << I2C_SR1_SMBDEFAULT)   /* SMBus device default address */
#define I2C_FLAG_SMBHOSTADDR    (1 << I2C_SR1_SMBHOSTADDR)  /* SMBus host address */
#define I2C_FLAG_DUALF          (1 << I2C_SR1_DUALF)        /* Dual flag */
#define I2C_FLAG_SMBHOST        (1 << I2C_SR1_SMBHOST)      /* SMBus host header */
#define I2C_FLAG_GENCALL        (1 << I2C_SR1_GENCALL)      /* General call header */
#define I2C_CR2_10BITADDR       (1 << 10)                   /* 10-bit address */
// Detection of misplaced start or stop condition

/*   TRA: Transmitter/receiver
 *      0: Data bytes received
 *      1: Data bytes transmitted
 */
#define I2C_SR2_TRA_RX 0
#define I2C_SR2_TRA_TX 1


/************************************************************************************************
 *    								APIs supported by this driver
 *                For more information about the APIs check the function definitions
 *                                 in the stm32f407xx_i2c_driver.c file
 * **********************************************************************************************/

/* Init and De-init & Peripheral Setup */
void I2C_PeriClockCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi); /* Enable or Disable peripheral clock for I2C */
void I2C_PeriCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi); /* Enable or disable I2C peripheral */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/* Data Send and Receive */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t slaveAddr, uint8_t Sr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

/* IRQ Configuration and ISR Handling */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/* Other Peripheral Control APIs */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi); /* Enable or disable ACKing */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName); /* Get flag status */
void checkPeriCtrlDisabled(I2C_RegDef_t *pI2Cx); /* Check if peripheral is disabled */
uint32_t RCC_GetPCLK1Value(void);
void I2C_GenStopCond(I2C_RegDef_t *pI2Cx); /* Generate stop condition */

/* Application callback */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);
void I2C_AppErrorHandler(I2C_Handle_t *pI2CHandle);
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi); /* Enable or disable callback events */

#endif /* STM32F407XX_I2C_DRIVER_H_ */