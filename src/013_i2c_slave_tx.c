

#include "stm32f407xx.h"
#include <string.h>
#include <stdio.h>

/*
 * I2C slave send data to arduino
 * 
 * GPIO pins used: 
 * STM32 = PB6, PB7
 * Arduino = A4, A5
 * 
 * Date: 2023-03-13
 * Author: Josh Williams
 * Course: Udemy Embedded Systems Masterclass
 * 
 * PB6 - I2C1_SCL
 * PB7 - I2C1_SDA
 * 
 * Arduino Uno:
 * A4 - SDA
 * A5 - SCL
 * 
 * ADDRS:
 * STM32F4 = 0x69
 * ARDUINO = 0x61
 * 
 * Data flow summary:
 * 
 * master sends 0x51 to slave to read the length of data
 * I2C_EVNT_DATA_RCV triggers
 * 
 * slave sends the length of data to master 
 * I2C_EVNT_DATA_REQ
 * 
 * master sends 0x52 to slave to read the data
 * I2C_EVNT_DATA_RCV triggers
 * 
 * slave sends the data to master
 * I2C_EVNT_DATA_REQ
 * 
 */

#define SLAVE_ADDR  0x69

/* Global Variables */
I2C_Handle_t I2C1Handle;
uint8_t TxBuffer[32] = "stm32 slave mode testing.."; // holds 32 bytes

void delay(void) {
    for (uint32_t i = 0; i < 500000 / 2; i++);
}

void I2C1_GPIOInits(void) {
    GPIO_Handle_t I2CPins;
    I2CPins.pGPIOx                              = GPIOB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_ALTFN;
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode   = 4;
    I2CPins.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OP_TYPE_OD;
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_PIN_PU;
    I2CPins.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST;

    // SCL
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_PeriClockControl(GPIOB, ENABLE);
    GPIO_Init(&I2CPins);

    // SDA
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GPIO_Init(&I2CPins);
}

void I2C1_Inits(void) {
    I2C1Handle.pI2Cx                            = I2C1;                     // I2C1 peripheral
    I2C1Handle.I2C_Config.I2C_ACKCtrl           = I2C_ACK_ENABLE;           // Not relevant for standard mode
    I2C1Handle.I2C_Config.I2C_DeviceAdrs        = SLAVE_ADDR;               // Not relevant for standard mode
    I2C1Handle.I2C_Config.I2C_DeviceAdrsMode    = I2C_DEVICE_ADDRESS_7BIT;  // Not relevant for standard mode
    I2C1Handle.I2C_Config.I2C_FMDutyCycle       = I2C_FM_DUTY_2;            // Not relevant for standard mode
    I2C1Handle.I2C_Config.I2C_SCLSpeed          = I2C_SCL_SPEED_SM;         // Standard mode

    I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(void) {
    GPIO_Handle_t GPIOBtn;
    GPIOBtn.pGPIOx                              = GPIOA;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_0;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&GPIOBtn);
}

int main(void) {
    printf("Application is running\n");

    GPIO_ButtonInit();

    // I2C1 Peripheral Configuration
    I2C1_GPIOInits();
    I2C1_Inits();

    // I2C IRQ Configurations
    I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
    I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);
    I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);
    // I2C IRQ Priority Configurations

    // Enable the I2C peripheral
    I2C_PeriCtrl(I2C1, ENABLE); 
    I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);
    /* PE must be set to 1 before we can enable ack bit. Otherwise the bit is cleared by hardware */
    while(1);
}

void I2C1_EV_IRQHandler(void)
{
    I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler (void)
{
    I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
    // can be global or static. 
    // (static variables are kind of like global variables, 
    // the memory allocated for them will not be in the stack. 
    // The difference from global variables however,
    // is that you cannot access these variables from outside the function.)
    static uint8_t cmdCode = 0;
    static uint8_t data_index = 0;
    // static uint8_t RxBuffer[32];

    if(AppEv == I2C_EVNT_DATA_REQ)
    {
        // Master is requesting data. Slave sends it.
        if(cmdCode == 0x51)
        {
            // Send the length information to the master
            I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)TxBuffer));
        }
        else if(cmdCode == 0x52)
        {
            // Send the contents of TxBuffer
            I2C_SlaveSendData(pI2CHandle->pI2Cx, TxBuffer[data_index++]);
            printf("Data sent: %c\n", TxBuffer[data_index-1]);
        }

        // I2C_SlaveSendData(pI2CHandle->pI2Cx, TxBuffer[data_index++]);
        // printf("Data sent: %c\n", TxBuffer[data_index-1]);
    }
    else if(AppEv == I2C_EVNT_DATA_RCV)
    {
        // Master has sent some data. Slave reads it.
        cmdCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx); /* RxBuffer cannot be a local variable because will go out of scope */
    }
    else if(AppEv == I2C_ERROR_AF)
    {
        // Master has sent NACK. Slave should understand that master doesn't want more data.
        printf("NACK received from master\n");
        cmdCode = 0xFF; // invalidate the command code
        data_index = 0; // reset the data index
    }
    else if(AppEv == I2C_EVNT_STOP)
    {
        // Master has ended the communication.
        printf("Event: Stop condition detected\n");
    }
}