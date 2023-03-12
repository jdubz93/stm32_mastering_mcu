/*
 * Created by:     Josh Williams
 * Date Created:   12/29/2022
 * Last Modified:  01/03/2023
 * 
 * Description:    This file contains example code sending data over SPI.
 * 
 * PB13 -> SCLK
 * PB14 -> MISO
 * PB15 -> MOSI
 * PB12 -> NSS
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
    GPIO_Handle_t GpioSpi;
    GpioSpi.pGPIOx = GPIOB;
    GpioSpi.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    GpioSpi.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    GpioSpi.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioSpi.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
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

    // // NSS
    // GpioSpi.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    // GPIO_Init(&GpioSpi);

    // SPI2 Peripheral Configuration
    Spi2Handle.pSPIx = SPI2;
    Spi2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    Spi2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    Spi2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; // Generates SCLK of 8MHz
    Spi2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    // Spi2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    Spi2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
    Spi2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    Spi2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN; // Software slave management enabled for NSS pin

    // Enable SPI2 Peripheral
    // SPI_PeriClockControl(SPI2, ENABLE); /* Moved to SPI_Init() */

    SPI_Init(&Spi2Handle);

    SPI_SSIConfig(SPI2, ENABLE); // Enable SSI bit in CR1 register to enable NSS output (master mode)

	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,ENABLE);

    SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));

	//lets confirm SPI is not busy
	while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));

	//Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,DISABLE);

    while(1);
    
    return 0;
}