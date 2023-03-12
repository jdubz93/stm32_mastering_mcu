

#include "stm32f407xx.h"
#include <string.h>
#include <stdio.h>

/*
 * I2C_SCLSpeed = 100KHz (standard mode)
 * External pull-up resistors are used (3.3 KOhm) for SDA and SCL
 * 
 * Exercise: When user button is pressed on STM32F4, data is sent to the Arduino and displayed on serial monitor. 
 *
 * Date: 2023-02-11
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
    char user_data[] = "Hello World!";
    GPIO_ButtonInit();
    I2C1_GPIOInits();
    I2C1_Inits();
    I2C_PeriCtrl(I2C1, ENABLE);

    while (1) {
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        delay(); // Debounce

        I2C_MasterSendData(&I2C1Handle, (uint8_t*)user_data, strlen(user_data), SLAVE_ADDR, I2C_ENABLE_SR);
    }
}