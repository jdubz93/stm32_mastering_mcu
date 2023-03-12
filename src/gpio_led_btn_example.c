/*
 * Created by:     Josh Williams
 * Date Created:   12/27/2022
 * Last Modified:  12/27/2022
 * 
 * Description:    This file contains example code of internal user btn to light the led on stm32f407-discovery board using gpio driver code.
 *               
 * Case 1. Use push-pull output mode to light the  using internal user btn.
 */

#include "stm32f407xx.h"

void delay(void)
{
    for (uint32_t i = 0; i < 500000 / 2; i++);
}

int main(void)
{
    GPIO_Handle_t GpioLed, GpioBtn;

    // LED
    GpioLed.pGPIOx = GPIOD;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GpioLed.GPIO_PinConfig.GPIO_PinAltFunMode = 0;

    // BTN
    GpioBtn.pGPIOx = GPIOA;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // push pull
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // We are using external pull down resistor, so there is no need to activate the internal pull down resistor.
    GpioBtn.GPIO_PinConfig.GPIO_PinAltFunMode = 0;

    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_PeriClockControl(GPIOA, ENABLE);

    GPIO_Init(&GpioLed);
    GPIO_Init(&GpioBtn);

    while(1)
    {
        if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == ENABLE)
            GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
        delay(); // Debounce delay to avoid button bounce.
    }

    return 0;
}