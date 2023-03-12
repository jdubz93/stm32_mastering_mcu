

#include "stm32f407xx.h"
#include <string.h>
#include <stdio.h>

// extern void initialise_monitor_handles(void);

/*
 * I2C_SCLSpeed = 100KHz (standard mode)
 * External pull-up resistors are used (3.3 KOhm) for SDA and SCL
 * 
 * Exercise: When user button is pressed on STM32F4, 
 * 1. Master sends 0x51 to slave which tells the slave to send the length,
 * 2. Master sends 0x52 to slave which tells the slave to send the rest of the data.
 * 3. Data received is displayed on semihosting printf. 
 *
 * Date: 2023-02-26
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
 * STM32F4 = 0x61
 * ARDUINO = 0x68
 */

/* Global Variables */
#define MY_ADDR     0x61
#define SLAVE_ADDR  0x68
I2C_Handle_t I2C1Handle;
uint8_t pRxbuffer[32]; // holds 32 bytes

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
    I2C1Handle.I2C_Config.I2C_DeviceAdrs        = 0x61;                     // Not relevant for standard mode
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
    // initialise_monitor_handles();
    printf("Application is running\n");

    uint8_t rqst_len = 0x51;
    uint8_t rqst_data = 0x52;
    uint8_t recv_len;
    GPIO_ButtonInit();
    I2C1_GPIOInits();
    I2C1_Inits();
    I2C_PeriCtrl(I2C1, ENABLE); 
    I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);
    /* PE must be set to 1 before we can enable ack bit. Otherwise the bit is cleared by hardware */
    while (1) {
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        delay(); // Debounce

        // length is 1 byte
        I2C_MasterSendData(&I2C1Handle, &rqst_len, 1, SLAVE_ADDR, I2C_ENABLE_SR);
        I2C_MasterReceiveData(&I2C1Handle, &recv_len, 1, SLAVE_ADDR, I2C_ENABLE_SR);

        I2C_MasterSendData(&I2C1Handle, &rqst_data, 1, SLAVE_ADDR, I2C_ENABLE_SR);
        I2C_MasterReceiveData(&I2C1Handle, pRxbuffer, recv_len, SLAVE_ADDR, I2C_DISABLE_SR);

        pRxbuffer[recv_len+1] = '\0';
        printf("Data : %s",pRxbuffer);
    }
}