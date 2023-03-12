/*
 * Created by:     Josh Williams
 * Date Created:   12/27/2022
 * Last Modified:  12/27/2022
 * 
 * Description:    This file contains gpio driver implementation code for stm32f4-DISC1 board (stm32f407 mcu).
 *
 */

#include "stm32f407xx_gpio_driver.h"
#include <stdio.h>

// Peripheral Clock Init/Deinit

/******************************************************************************************
 * @fn				- GPIO_Init
 * @brief			- This function initializes the GPIO peripheral
 * @param[in]		- Base address of the GPIO peripheral
 * @return			- void
 * @note			- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
        // Configure the mode of gpio pin for non interrupt mode
        pGPIOHandle->pGPIOx->MODER &= ~(0b11 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing
        pGPIOHandle->pGPIOx->MODER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    } else {
        // Interrupt mode
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
            // Configure the FTSR
            EXTI->FTSR |= (0b1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // Clear the RTSR bit corresponding to the pin number (in case it was set previously)
            EXTI->RTSR &= ~(0b1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        } else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
            // Configure the RTSR
            EXTI->RTSR |= (0b1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // Clear the FTSR bit corresponding to the pin number (in case it was set previously)
            EXTI->FTSR &= ~(0b1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        } else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
            // Configure the both FTSR and RTSR
            EXTI->FTSR |= (0b1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->RTSR |= (0b1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        // 2. Configure the GPIO port selection in SYSCFG_EXTICR
        uint8_t ExtiPortIdx, ExtiPortPinPos;
        ExtiPortIdx    = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4; // this tells us which EXTI port index to use. 0-3
        ExtiPortPinPos = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4; // this tells us which EXTI port pin position to use.
        uint8_t gpioPortLetter = CONVERT_BASEADDR_TO_PORTLETTER(pGPIOHandle->pGPIOx);
        SYSCFG_PCLK_EN(); // Enable the SYSCFG peripheral clock before you can access the SYSCFG registers
        SYSCFG->EXTICR[ExtiPortIdx] &= ~(0b1111 << (4 * ExtiPortPinPos)); // Clearing
        // SYSCFG->EXTICR[ExtiPortIdx] = gpioPortLetter << (4 * ExtiPortPinPos);
        SYSCFG->EXTICR[ExtiPortIdx] |= (gpioPortLetter << (4 * ExtiPortPinPos));

        // 3. Enable the exti interrupt delivery using IMR
        EXTI->IMR |= (0b1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

    }

    // Configure the pupd settings
    pGPIOHandle->pGPIOx->PUPDR &= ~(0b11 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing
    pGPIOHandle->pGPIOx->PUPDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	if((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT) || (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN))
    {
        // Configure the speed
        pGPIOHandle->pGPIOx->OSPEEDR &= ~(0b11 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing
        pGPIOHandle->pGPIOx->OSPEEDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));


        // Configure the optype
        pGPIOHandle->pGPIOx->OTYPER &= ~(0b1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing
        pGPIOHandle->pGPIOx->OTYPER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }

    // Configure the alt functionality
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
        uint8_t AltFnIdx, AltFnPinPos;
        AltFnIdx    = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; // if pin > 8 ? return 1 : return 0
        AltFnPinPos = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8; // if pin > 8 ? return pin - 8 : return pin
        pGPIOHandle->pGPIOx->AFR[AltFnIdx] &= ~(0b1111 << (4 * AltFnPinPos)); // Clearing
        pGPIOHandle->pGPIOx->AFR[AltFnIdx] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * AltFnPinPos));
    }
}

/******************************************************************************************
 * @fn				- GPIO_DeInit
 * @brief			- This function de-initializes the GPIO peripheral
 * @param[in]		- Base address of the GPIO peripheral
 * @return			- void
 * @note			- none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if(pGPIOx == GPIOA){
        GPIOA_REG_RESET();
    } else if(pGPIOx == GPIOB){
        GPIOB_REG_RESET();
    } else if(pGPIOx == GPIOC){
        GPIOC_REG_RESET();
    } else if(pGPIOx == GPIOD){
        GPIOD_REG_RESET();
    } else if(pGPIOx == GPIOE){
        GPIOE_REG_RESET();
    } else if(pGPIOx == GPIOF){
        GPIOF_REG_RESET();
    } else if(pGPIOx == GPIOG){
        GPIOG_REG_RESET();
    } else if(pGPIOx == GPIOH){
        GPIOH_REG_RESET();
    }
}

// Peripheral Clock Setup

/******************************************************************************************
 * @fn				- GPIO_PeriClockControl
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @return			- void
 * @note			- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE) { /* Enable the peripheral clock */
        if(pGPIOx == GPIOA){
            GPIOA_PCLK_EN();
        } else if(pGPIOx == GPIOB){
            GPIOB_PCLK_EN();
        } else if(pGPIOx == GPIOC){
            GPIOC_PCLK_EN();
        } else if(pGPIOx == GPIOD){
            GPIOD_PCLK_EN();
        } else if(pGPIOx == GPIOE){
            GPIOE_PCLK_EN();
        } else if(pGPIOx == GPIOF){
            GPIOF_PCLK_EN();
        } else if(pGPIOx == GPIOG){
            GPIOG_PCLK_EN();
        } else if(pGPIOx == GPIOH){
            GPIOH_PCLK_EN();
        } else if(pGPIOx == GPIOI){
            GPIOI_PCLK_EN();
        } else {
            printf("GPIO_PeriClockControl: Invalid GPIO port\n");
        }
    } else { /* Disable the peripheral clock */
        if(pGPIOx == GPIOA){
            GPIOA_PCLK_DI();
        } else if(pGPIOx == GPIOB){
            GPIOB_PCLK_DI();
        } else if(pGPIOx == GPIOC){
            GPIOC_PCLK_DI();
        } else if(pGPIOx == GPIOD){
            GPIOD_PCLK_DI();
        } else if(pGPIOx == GPIOE){
            GPIOE_PCLK_DI();
        } else if(pGPIOx == GPIOF){
            GPIOF_PCLK_DI();
        } else if(pGPIOx == GPIOG){
            GPIOG_PCLK_DI();
        } else if(pGPIOx == GPIOH){
            GPIOH_PCLK_DI();
        } else if(pGPIOx == GPIOI){
            GPIOI_PCLK_DI();
        } else {
            printf("GPIO_PeriClockControl: Invalid GPIO Port\n");
        }
    }
}

// Data Read/Write

/******************************************************************************************
 * @fn				- GPIO_ReadFromInputPin
 * @brief			- This function reads the input data from the given GPIO pin
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- Pin number
 * @return			- 0 or 1
 * @note			- Check img folder (GPIO_ReadFromInputPin.png) for more info
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001); // Read the pin number (right shift by pin number and mask with 0x00000001)
    // value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x0000000000000001);
    return value;

	// //Note: STM code uses a 16 bit value for pin and a simple & (no bit shift needed)
	// uint8_t value;
	// uint16_t GPIO_value = (1 << PinNumber);
	// if (pGPIOx->IDR & GPIO_value){
	// 	value = SET;
	// } else {
	// 	value = RESET;
	// }
	// return value;
}

/******************************************************************************************
 * @fn				- GPIO_ReadFromInputPort
 * @brief			- This function reads the input data from the given GPIO port
 * @param[in]		- Base address of the GPIO peripheral
 * @return			- 16-bit value
 * @note			- none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;
    value = (uint16_t)pGPIOx->IDR;
    return value;
}

/******************************************************************************************
 * @fn				- GPIO_WriteToOutputPin
 * @brief			- This function writes the given value to the given GPIO pin
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- Pin number
 * @param[in]		- 0 or 1
 * @return			- void
 * @note			- none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if(Value == GPIO_PIN_SET){
        // Write 1 to the output data register at the bit field corresponding to the pin number
        pGPIOx->ODR |= (0b1 << PinNumber);
    } else {
        // Write 0 to the output data register at the bit field corresponding to the pin number
        pGPIOx->ODR &= ~(0b1 << PinNumber);
    }
}

/******************************************************************************************
 * @fn				- GPIO_WriteToOutputPort
 * @brief			- This function writes the given value to the given GPIO port
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- 16-bit value
 * @return			- void
 * @note			- none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value; // Write the value to the output data register
}

/******************************************************************************************
 * @fn				- GPIO_ToggleOutputPin
 * @brief			- This function toggles the given GPIO pin
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- Pin number
 * @return			- void
 * @note			- none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= 0b1 << PinNumber; // Toggle the pin number
    // pGPIOx->ODR ^= (1 << PinNumber); // Toggle the pin number
}

// IRQ Configuration and ISR Handling

/******************************************************************************************
 * @fn				- GPIO_IRQInterruptConfig
 * @brief			- This function configures the given IRQ number
 * @param[in]		- IRQ number
 * @param[in]		- ENABLE or DISABLE macros
 * @return			- void
 * @note			- none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    // if(EnorDi == ENABLE){
    //     if(IRQNumber <= 31){
    //         // Program ISER0 register
    //         *NVIC_ISER0 |= (1 << IRQNumber);
    //     } else if(IRQNumber > 31 && IRQNumber < 64){
    //         // Program ISER1 register
    //         *NVIC_ISER1 |= (1 << (IRQNumber % 32)); /* IRQNumber % 32 is the IRQNumber - 32 (e.g. IRQNumber = 33, IRQNumber % 32 = 33 - 32 = 1) (e.g. IRQNumber = 32, (IRQNumber % 32) = (32 - 32) = 0) */
    //     } else if(IRQNumber >= 64 && IRQNumber < 96){
    //         // Program ISER2 register
    //         *NVIC_ISER2 |= (1 << (IRQNumber % 64)); /* IRQNumber % 64 is the IRQNumber - 64 (e.g. IRQNumber = 65, IRQNumber % 64 = 65 - 64 = 1) (e.g. IRQNumber = 64, (IRQNumber % 64) = (64 - 64) = 0) */
    //     } else {
    //         printf("GPIO_IRQInterruptConfig: Invalid IRQ Number\n");
    //     }
    // } else {
    //     if(IRQNumber <= 31){
    //         // Program ICER0 register
    //         *NVIC_ICER0 |= (1 << IRQNumber);
    //     } else if(IRQNumber > 31 && IRQNumber < 64){
    //         // Program ICER1 register
    //         *NVIC_ICER1 |= (1 << (IRQNumber % 32));
    //     } else if(IRQNumber >= 64 && IRQNumber < 96){
    //         // Program ICER2 register
    //         *NVIC_ICER2 |= (1 << (IRQNumber % 64));
    //     } else {
    //         printf("GPIO_IRQInterruptConfig: Invalid IRQ Number\n");
    //     }
    // }

	uint8_t nvicIdx, nvicBitPos;
	nvicIdx = IRQNumber / 32; // IRQNumber / 32 (e.g. IRQNumber = 33, 33 / 32 = 1) (e.g. IRQNumber = 32, IRQNumber / 32 = 32 / 32 = 1)
	nvicBitPos = IRQNumber % 32; // IRQNumber % 32 (e.g. IRQNumber = 33, 33 % 32 = 1) (e.g. IRQNumber = 32, 32 % 32 = 0)

    if(EnorDi == ENABLE){
        // Program ISER0-3 register
        NVIC->ISER[nvicIdx] |= (1 << nvicBitPos);
    } else {
        // Program ICER0-3 register
        NVIC->ICER[nvicIdx] |= (1 << nvicBitPos);
    }
}

/******************************************************************************************
 * @fn				- GPIO_IRQPriorityConfig
 * @brief			- This function configures the given IRQ priority
 * @param[in]		- IRQ number
 * @param[in]		- IRQ priority
 * @return			- void
 * @note			- none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// IP register is of type uint8_t thus you can select the register directly
    NVIC->IP[IRQNumber]  = IRQPriority << NO_PR_BITS_IMPLEMENTED; /* shift by 4 is required because the lower 4 bits are not implemented */
}
// void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
// {
//     // 1. Find out the IPR register
//     uint8_t iprx = IRQNumber / 4;
//     // 2. Find out the section of the IPR register (0-3)
//     uint8_t iprx_section = IRQNumber % 4;
//     // 3. Find out the shift amount (section * 8) + (8 - NO_PR_BITS_IMPLEMENTED)
//     uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
//     // 4. Write to the IPR register
//     *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
// }

/******************************************************************************************
 * @fn				- GPIO_IRQHandling
 * @brief			- This function handles the given IRQ number
 * @param[in]		- Pin number
 * @return			- void
 * @note			- EXTI pointer to the EXTI module and PR is a register in the module that contains the pending request flags for each external interrupt line.
 *                  The & operator is a bitwise AND operator. It performs a bitwise AND operation on the value in the PR register and the value (1 << PinNumber).
 *                  ie: if the value in the PR register is 0b00000000 and the value (1 << PinNumber) is 0b00000001, the result of the bitwise AND operation is 0b00000000 and the if statement is false(skipped).
 *                  if the result is 1, the if statement is true and the pending request flag is cleared.        
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
    // Clear the EXTI PR register corresponding to the pin number
    if(EXTI->PR & (1 << PinNumber)){ // checking the state of a pin in the EXTI (External Interrupt). 
        EXTI->PR |= (1 << PinNumber); // (clear) writing a 1 to the PR register clears the pending request flag for the corresponding external interrupt line.
    }    
}