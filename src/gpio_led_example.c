/*
 * Created by:     Josh Williams
 * Date Created:   12/27/2022
 * Last Modified:  12/27/2022
 * 
 * Description:    This file contains example code to light the led on stm32f407-discovery board using gpio driver code.
 *               
 * Case 1. Use push-pull output mode to light the led.
 * Case 2. Use open-drain output mode to light the led.
 */

#include "stm32f407xx.h"

void delay(void)
{
    for (uint32_t i = 0; i < 500000 / 2; i++);
}

int main(void)
{
    GPIO_Handle_t GpioLed;

    GpioLed.pGPIOx = GPIOD;

    /* Pin config settings for Push-Pull */
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&GpioLed);
    while(1)
    {
        GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
        delay();
    }

    /* Pin config settings for Open Drain */
    // GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    // GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    // GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    // GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    // GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    // GPIO_PeriClockControl(GPIOD, ENABLE);
    // GPIO_Init(&GpioLed);
    // while(1)
    // {
    //     GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
    //     delay();
    // }

    return 0;
}