/*
 * Created by:     Josh Williams
 * Date Created:   12/27/2022
 * Last Modified:  12/27/2022
 * 
 * Description:    This file contains example code of external user btn & led on stm32f407-discovery board using gpio driver code.
 *               
 * Case 1. Use push-pull output mode to light external LED using external user btn.
 * External User Btn is connected to PB12
 * External LED is connected to PA14
 */

#include "stm32f407xx.h"

void delay(void)
{
    for (uint32_t i = 0; i < 500000 / 2; i++); // 500000 / 2 = 250000
}


int main(void)
{

    GPIO_Handle_t GpioLed, GpioBtn;

    // LED
    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GpioLed.GPIO_PinConfig.GPIO_PinAltFunMode = 0;

    // BTN
    GpioBtn.pGPIOx = GPIOB;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // push pull
    // GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // We are using external pull down resistor, no internal is needed.
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // We are using external pull down resistor, no internal is needed.
    GpioBtn.GPIO_PinConfig.GPIO_PinAltFunMode = 0;

    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_PeriClockControl(GPIOB, ENABLE);

    GPIO_Init(&GpioLed);
    GPIO_Init(&GpioBtn);

    while(1)
    {
        if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == DISABLE) {
            delay(); // Debounce delay to avoid button bounce.
            GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
        }
    }

    return 0;
}