/*
 * Created by:     Josh Williams
 * Date Created:   12/29/2022
 * Last Modified:  01/03/2023
 * 
 * Description:    This file contains example code sending data over SPI to arduino uno r3.
 * 
 * PB13 -> SCLK
 * PB14 -> MISO
 * PB15 -> MOSI
 * PB12 -> NSS
 * 
 * Arduino Uno R3:
 * SPI pin numbers:
 * SCK   13  // Serial Clock.
 * MISO  12  // Master In Slave Out.
 * MOSI  11  // Master Out Slave In.
 * SS    10  // Slave Select . Arduino SPI pins respond only if SS pulled low by the master
 * 
 * Exercise:
 * 1. Use SPI Full Duplex mode to send data to Arduino.
 * 2. Use DFF = 0
 * 3. Use SSM = 0, hardware slave management (NSS pin)
 * 4. Use SCLK speed = 2MHz (generates clock of 2MHz), fPCLK = 16MHz
 * 5. Send "Hello World" to Arduino and confirm it is received.
 * 6. STM32 acting as master, Arduino acting as slave.
 * 
 * Note: slave does not know how many bytes it will receive, so you must send length data first.
 * 
 * Tip: When SSM = 0, NSS pin hardware managed by the SPI peripheral. SPE = 1, NSS pin will be pulled to low. SPE = 0, NSS pin will be pulled to high.
 * SPE (SPI Enable) is a control bit in the SPI_CR1 register.
 * SSOE (Slave Select Output Enable) is a control bit in the SPI_CR2 register.
 * SSOE must be set to 1 when SSM = 0.
 */

#include "stm32f407xx.h"
#include <string.h>
#include <stdio.h>

void delay(void)
{
    for (uint32_t i = 0; i < 500000 / 2; i++); // 500000 / 2 = 250000
}


int main(void)
{
    SPI_Handle_t Spi2Handle;
    char user_data[] = "Hello World";

    // SPI2 Global GPIO Configuration
    GPIO_Handle_t GpioSpi, GpioBtn;
    GpioSpi.pGPIOx = GPIOB;
    GpioSpi.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    GpioSpi.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    GpioSpi.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    // GpioSpi.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GpioSpi.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    GpioSpi.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    // Enable GPIO RCC Peripheral Clock
    GPIO_PeriClockControl(GPIOB, ENABLE);

    // SCLK
    GpioSpi.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&GpioSpi);

    // MOSI
    GpioSpi.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&GpioSpi);

    // // MISO
    // GpioSpi.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    // GPIO_Init(&GpioSpi);

    // NSS
    GpioSpi.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&GpioSpi);

    // BTN
    GpioBtn.pGPIOx = GPIOA;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // push pull
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // We are using external pull down resistor, so there is no need to activate the internal pull down resistor.
    GpioBtn.GPIO_PinConfig.GPIO_PinAltFunMode = 0;

    // Enable GPIOA RCC Peripheral Clock
    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&GpioBtn);

    // SPI2 Peripheral Configuration
    Spi2Handle.pSPIx = SPI2;
    Spi2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    Spi2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    Spi2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // Generates SCLK of 2MHz (16 / 8 = 2) (APB1 = 16MHz) (APB2 = 16MHz)
    Spi2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    Spi2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    // Spi2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
    Spi2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    Spi2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // Hardware slave management enabled by NSS pin

    SPI_Init(&Spi2Handle);

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
    SPI_SSOEConfig(SPI2, ENABLE); // SSOE must be set to 1 when SSM = 0

    // Btn press to send data
    while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
    delay();

	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,ENABLE);

    //first send length information
    uint8_t dataLen = strlen(user_data);
    SPI_SendData(SPI2, &dataLen, 1);

    SPI_SendData(SPI2, (uint8_t*)user_data, dataLen);

	//lets confirm SPI is not busy
	while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));

	//Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,DISABLE);

    while(1);
    
    return 0;
}