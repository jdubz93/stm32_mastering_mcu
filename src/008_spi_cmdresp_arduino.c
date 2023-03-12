/*
 * Created by:     Josh Williams
 * Date Created:   12/29/2022
 * Last Modified:  01/03/2023
 * 
 * Description:    This file contains example code sending data over SPI to arduino uno r3.
 * 
 * PB13 = SCLK -> SCK   13  // Serial Clock.
 * PB14 = MISO -> MISO  12  // Master In Slave Out.
 * PB15 = MOSI -> MOSI  11  // Master Out Slave In.
 * PB12 = NSS  -> SS    10  // Slave Select
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

extern void initialise_monitor_handles();

// arduino cmds
#define CMD_LED_CTRL    0x50
#define CMD_SENSOR_READ 0x51
#define CMD_LED_READ    0x52
#define CMD_PRINT       0x53
#define CMD_ID_READ     0x54

// arduino led states
#define LED_ON      1
#define LED_OFF     0

// arduino analog pins
#define ANALOG_PIN0 0
#define ANALOG_PIN1 1
#define ANALOG_PIN2 2
#define ANALOG_PIN3 3
#define ANALOG_PIN4 4

// arduino led
#define LED_PIN     9



void delay(void)
{
    for (uint32_t i = 0; i < 500000 / 2; i++); // 500000 / 2 = 250000
}

uint8_t SPI_VerifyCmdResp(uint8_t ackbyte)
{
    if (ackbyte == 0xF5)
    {
        // ack
        printf("Command executed successfully");
        return 1;
    }
    else
    {
        // nack
        printf("Command not executed");
    }
    return 0;
}

int main(void)
{
    initialise_monitor_handles();
    printf("Application started\n");

    SPI_Handle_t Spi2Handle;
    uint8_t dummy_write = 0xff;
    uint8_t dummy_read;

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

    // MISO
    GpioSpi.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&GpioSpi);

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

    while(1) {
        // Btn press to send data
        while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        delay();

        //enable the SPI2 peripheral
        SPI_PeripheralControl(SPI2,ENABLE);

        // start of CMD_LED_CTRL
        //1. CMD_LED_CTRL  <pin no(1)>  <value(1)>
        uint8_t commandCode = CMD_LED_CTRL;
        uint8_t ackByte;
        uint8_t args[2];

        //send command
        SPI_SendData(SPI2, &commandCode, 1);
        /*
         * Note: after sending command, we need to read the ack byte from the slave. RXNE will have garbage code after sending data.
         * So we need to clear the RXNE flag by reading the data from the DR register.
         */
        SPI_ReceiveData(SPI2, &dummy_read, 1);
        /*
         * Note: Send dummy byte to fetch the ack byte from the slave
         */
        SPI_SendData(SPI2, &dummy_write, 1);

        SPI_ReceiveData(SPI2, &ackByte, 1);
        
        if(SPI_VerifyCmdResp(ackByte)){
            //send arguments
            args[0] = LED_PIN; // PIN9
            args[1] = LED_ON;  // ON
            SPI_SendData(SPI2, args, 2);
            printf("COMMAND_LED_CTRL executed successfully\n");
        }
        // end of CMD_LED_CTRL

        //2. CMD_SENSOR_READ <analog pin number(1)>
        while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        delay();
        commandCode = CMD_SENSOR_READ;
        uint8_t analogVal;

        //send cmd
        SPI_SendData(SPI2, &commandCode, 1);
        SPI_ReceiveData(SPI2, &dummy_read, 1); // clear RXANY flag (RXNE)
        SPI_SendData(SPI2, &dummy_write, 1);   // send dummy byte to fetch the ack byte from slave
        SPI_ReceiveData(SPI2, &ackByte, 1);    // read ack byte from slave (0xF5 = ACK, 0xA5 = NACK)
        if(SPI_VerifyCmdResp(ackByte)){
            //send arguments
            args[0] = ANALOG_PIN0;
            SPI_SendData(SPI2, args, 1);
            printf("COMMAND_SENSOR_READ executed successfully\n");

            // read the analog value from slave
            SPI_ReceiveData(SPI2, &dummy_read, 1); // clear RXNE flag
            delay(); // delay added because ADC conversion takes time. If we don't add delay, we will get wrong analog value.
            SPI_SendData(SPI2, &dummy_write, 1);   // send dummy byte
            SPI_ReceiveData(SPI2, &analogVal, 1);  // read analog value from slave
            /* If analog pin is connected to ground, analogVal = 0
            * If analog pin is connected to 5V, analogVal = 255
            * If analog pin is connected to 3.3V, analogVal = 165 (around this value 160-170)
            */
            printf("Analog value: %d\n", analogVal);
        }
        // end of CMD_SENSOR_READ

        //3. CMD_LED_READ <pin no(1)>
        while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)); // wait for button press
        delay(); // btn debouncing
        commandCode = CMD_LED_READ;
        uint8_t ledStatus;

        //send cmd
        SPI_SendData(SPI2, &commandCode, 1);
        SPI_ReceiveData(SPI2, &dummy_read, 1); // clear RXANY flag (RXNE)
        SPI_SendData(SPI2, &dummy_write, 1);
        SPI_ReceiveData(SPI2, &ackByte, 1);
        if(SPI_VerifyCmdResp(ackByte)){
            args[0] = LED_PIN;
            SPI_SendData(SPI2, args, 1);
            printf("COMMAND_LED_READ executed successfully\n");
            SPI_ReceiveData(SPI2, &dummy_read, 1); // clear RXNE flag
            SPI_SendData(SPI2, &dummy_write, 1);   // send dummy byte
            SPI_ReceiveData(SPI2, &ledStatus, 1);  // read led status from slave
            printf("LED status: %d\n", ledStatus); // 0 = OFF, 1 = ON
        }

        //4. CMD_PRINT <length of message(2)> <message(length)>
        while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        delay();
        commandCode = CMD_PRINT;
        uint8_t message[] = "Hello world";

        //send cmd
        SPI_SendData(SPI2, &commandCode, 1);
        SPI_ReceiveData(SPI2, &dummy_read, 1);
        SPI_SendData(SPI2, &dummy_write, 1);
        SPI_ReceiveData(SPI2, &ackByte, 1);
        if(SPI_VerifyCmdResp(ackByte)){
            //send length info
            args[0] = strlen((char*)message);
            SPI_SendData(SPI2, args, 1);
            // SPI_SendData(SPI2, message, args[0]);
			SPI_ReceiveData(SPI2,&dummy_read,1); //do dummy read to clear off the RXNE
			delay();

			//send message
			for(int i = 0 ; i < args[0] ; i++){
				SPI_SendData(SPI2,&message[i],1);
				SPI_ReceiveData(SPI2,&dummy_read,1);
			}
            printf("COMMAND_PRINT executed successfully\n");
        }

        //5. CMD_ID_READ
        while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        delay();
        commandCode = CMD_ID_READ;
        uint8_t id[11];

        //send cmd
        SPI_SendData(SPI2, &commandCode, 1);
        SPI_ReceiveData(SPI2, &dummy_read, 1);
        SPI_SendData(SPI2, &dummy_write, 1);
        SPI_ReceiveData(SPI2, &ackByte, 1);
        if(SPI_VerifyCmdResp(ackByte)){
            //read 10 bytes id from the slave
            for(int i = 0 ; i < 10 ; i++){
                SPI_SendData(SPI2, &dummy_write, 1);   // send dummy byte
                SPI_ReceiveData(SPI2, &id[i], 1);      // read id byte from slave
            }
            id[11] = '\0';
            printf("COMMAND_ID_READ executed successfully\n");
            printf("ID: %s\n", id);
        }

        //lets confirm SPI is not busy
        while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));

        //Disable the SPI2 peripheral
        SPI_PeripheralControl(SPI2,DISABLE);
    }

    while(1);
    
    return 0;
}