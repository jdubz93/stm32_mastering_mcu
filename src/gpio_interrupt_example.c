/*
 * Created by:     Josh Williams
 * Date Created:   12/27/2022
 * Last Modified:  12/27/2022
 * 
 * Description:    This file contains example code of interrupt based external user btn & led on stm32f407-discovery board using gpio driver code.
 *               
 * Exercise: 
 * 1. Connect external user btn to PD5 and toggle external LED on PA8 using interrupt.
 * 2. Interrupt should trigger on falling edge of btn press.
 * 
 * External User Btn: PD5
 * External LED: PA8
 * Resistors used: 
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
    GpioLed.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_8;
    GpioLed.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_NO_PUPD;
    GpioLed.GPIO_PinConfig.GPIO_PinAltFunMode   = 0;

    // BTN
    GpioBtn.pGPIOx = GPIOD;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_5;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_IT_FT; // Interrupt on falling edge
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST;
    GpioBtn.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OP_TYPE_PP; // push pull
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_PIN_PU; // We are using external pull down resistor, no internal is needed.
    GpioBtn.GPIO_PinConfig.GPIO_PinAltFunMode   = 0;

    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_PeriClockControl(GPIOD, ENABLE);

    GPIO_Init(&GpioLed);
    GPIO_Init(&GpioBtn);

    // IRQ configurations
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15); // not required because only using one interrupt
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

    /* Infinite Loop */
    while(1);

    return 0;
}

/* This is the ISR. Overwite IRQ Handler (weak) in startup file */
void EXTI9_5_IRQHandler(void) 
{
    delay();
    GPIO_IRQHandling(GPIO_PIN_NO_5);
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
}