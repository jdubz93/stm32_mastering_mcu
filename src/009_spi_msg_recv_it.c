/*
 * Created by:     Josh Williams
 * Date Created:   01/15/2023
 * Last Modified:  01/15/2023
 * 
 * Description:    This file contains example code sending data over SPI to STM32 (master) from arduino uno r3 (slave)
 * 
 * PB13 -> SCLK
 * PB14 -> MISO
 * PB15 -> MOSI
 * PB12 -> NSS
 * PD8  -> IRQ (interrupt request)
 * 
 * Arduino Uno R3:
 * SPI pin numbers:
 * SCK   13  // Serial Clock.
 * MISO  12  // Master In Slave Out.
 * MOSI  11  // Master Out Slave In.
 * SS    10  // Slave Select . Arduino SPI pins respond only if SS pulled low by the master
 * IRQ   8   // Interrupt Request
 * 
 * Exercise:
 * Use IT (interrupt) to receive data from arduino uno r3
 */

/*
 * This application receives and prints the user message received from the Arduino peripheral in SPI interrupt mode
 * User sends the message through Arduino IDE's serial monitor tool
 * Monitor the message received in the SWV itm data console
 * 
 * Note : Follow the instructions to test this code
 * 1. Download this code on to STM32 board , acts as Master
 * 2. Download Slave code (003SPISlaveUartReadOverSPI.ino) on to Arduino board (Slave)
 * 3. Reset both the boards
 * 4. Enable SWV ITM data console to see the message
 * 5. Open Arduino IDE serial monitor tool
 * 6. Type anything and send the message (Make sure that in the serial monitor tool line ending set to carriage return)
 */

#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include <string.h>

// extern void initialise_monitor_handles();

#define MAX_DATA_LEN 500
// char user_data[MAX_DATA_LEN] = "Hello World!";
char recv_buff[MAX_DATA_LEN];
__vo char read_byte;
volatile uint8_t recvStop = 0;

/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;
SPI_Handle_t Spi2Handle;

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t GpioSpi;
    GpioSpi.pGPIOx 								= GPIOB;
    GpioSpi.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_ALTFN;
    GpioSpi.GPIO_PinConfig.GPIO_PinAltFunMode 	= 5;
    GpioSpi.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OP_TYPE_PP;
    GpioSpi.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PIN_PU;
    GpioSpi.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOB, ENABLE);

	/*SCLK*/
	GpioSpi.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_13;
	GPIO_Init(&GpioSpi);
	/*MOSI*/
    GpioSpi.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_15;
	GPIO_Init(&GpioSpi);
	/*MISO*/
	GpioSpi.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_14;
	GPIO_Init(&GpioSpi);
	/*NSS*/
	GpioSpi.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_12;
	GPIO_Init(&GpioSpi);
}

void SPI2_Inits(void)
{
    Spi2Handle.pSPIx = SPI2;
    Spi2Handle.SPIConfig.SPI_BusConfig  = SPI_BUS_CONFIG_FD;
    Spi2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    Spi2Handle.SPIConfig.SPI_SclkSpeed  = SPI_SCLK_SPEED_DIV8; /* generates  */
    Spi2Handle.SPIConfig.SPI_DFF        = SPI_DFF_8BITS;
    Spi2Handle.SPIConfig.SPI_CPOL       = SPI_CPOL_LOW;
    Spi2Handle.SPIConfig.SPI_CPHA       = SPI_CPHA_LOW;
    Spi2Handle.SPIConfig.SPI_SSM        = SPI_SSM_DI; /* Hardware slave management enabled for NSS pin */
    SPI_Init(&Spi2Handle);
   /* 
    * SSOE 1 does NSS output enable
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI2, ENABLE); // SSOE must be set to 1 when SSM = 0
}

/* This function configures the gpio pin over which SPI peripheral issues data available interrupt */
void SlaveGpioITPinInit(void)
{
    GPIO_Handle_t SPI_IT_PIN;
    memset(&SPI_IT_PIN,0,sizeof(SPI_IT_PIN)); 
    SPI_IT_PIN.pGPIOx                              = GPIOD;
    SPI_IT_PIN.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_6;
    SPI_IT_PIN.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_IT_FT;
    SPI_IT_PIN.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_LOW;
    SPI_IT_PIN.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_PIN_PU;
	GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&SPI_IT_PIN);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);
}

int main(void)
{
    // initialise_monitor_handles();
    // printf("Application started\n");

    uint8_t dummy = 0xff;
    SlaveGpioITPinInit();
    SPI2_GPIOInits();
    SPI2_Inits();
    SPI_IRQInterruptConfig(IRQ_NO_SPI2,ENABLE);

    while(1)
    {
        recvStop = 0;
        while(!dataAvailable)
		{
			/* wait till data available interrupt from transmitter device(slave) */
		}

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,DISABLE);

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		while(!recvStop)
		{
			/* fetch the data from the SPI peripheral byte by byte in interrupt mode */
			while ( SPI_SendDataIT(&Spi2Handle,&dummy,1) == SPI_BUSY_IN_TX);
			while ( SPI_ReceiveDataIT(&Spi2Handle,&read_byte,1) == SPI_BUSY_IN_RX );
		}

		// confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);

		printf("Rcvd data = %s\n",recv_buff);

		dataAvailable = 0;

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	}

    return 0;
}

/* Runs when a data byte is received from the peripheral over SPI*/
void SPI2_IRQHandler(void)
{

	SPI_IRQHandling(&Spi2Handle);
}



void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	static uint32_t i = 0;
	/* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1) */
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
				recv_buff[i++] = read_byte;
				if(read_byte == '\0' || ( i == MAX_DATA_LEN)){
					recvStop = 1;
					recv_buff[i-1] = '\0';
					i = 0;
				}
	}

}

/* Slave data available interrupt handler */
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	dataAvailable = 1;
}