/*
 * stm32f407xx_gpio_driver.h
 * Created on: 2020. 12. 28.
 * Author: Josh W.
 * 
 * Required APIS:
 * 1. Clock Control
 * 2. Init and De-init
 * 3. Read and Write
 * 4. IRQ Configuration and ISR Handling (interrupt handling)
 * 5. Peripheral Clock Setup (EN/DI)
 * 6. Alternate Function Configuration
 * 7. Port Enable/Disable
 * 8. Port Read/Write
 * 9. Pin Read/Write
 *
 * 10. IRQ Configuration ? (maybe)
 * 11. IRQ Handling ? (maybe)
 * 12. IRQ Priority Configuration ? (maybe)
 * 
 * Summary: this file contains all the APIs for GPIO peripheral driver for STM32F407xx microcontroller
 */

#ifndef STM32F407XX_GPIO_DRIVER_H
#define STM32F407XX_GPIO_DRIVER_H

#include "stm32f407xx.h"

/*
 * Configuration structure for a GPIO pin
 */
typedef struct {
    uint8_t GPIO_PinNumber;      /* Possible values from @GPIO_PIN_NUMBERS (uint8_t is picked because pins go from 0 to 15) */
    uint8_t GPIO_PinMode;        /* Possible values from @GPIO_PIN_MODES */
    uint8_t GPIO_PinSpeed;       /* Possible values from @GPIO_PIN_SPEED */
    uint8_t GPIO_PinPuPdControl; /* Possible values from @GPIO_PIN_PUPD */
    uint8_t GPIO_PinOPType;      /* Possible values from @GPIO_PIN_OPTYPE */
    uint8_t GPIO_PinAltFunMode;  /* Possible values from @GPIO_PIN_ALT_FUN_MODE */
} GPIO_PinConfig_t;

/*
 * GPIO Handle structure
 */
typedef struct {
    GPIO_RegDef_t *pGPIOx;           /* This holds the base address of the GPIO port to which the pin belongs */
    GPIO_PinConfig_t GPIO_PinConfig; /* This holds GPIO pin configuration settings */
} GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0  0
#define GPIO_PIN_NO_1  1
#define GPIO_PIN_NO_2  2
#define GPIO_PIN_NO_3  3
#define GPIO_PIN_NO_4  4
#define GPIO_PIN_NO_5  5
#define GPIO_PIN_NO_6  6
#define GPIO_PIN_NO_7  7
#define GPIO_PIN_NO_8  8
#define GPIO_PIN_NO_9  9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN     0
#define GPIO_MODE_OUT    1
#define GPIO_MODE_ALTFN  2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT  4 // interrupt falling edge trigger
#define GPIO_MODE_IT_RT  5 // interrupt rising edge trigger
#define GPIO_MODE_IT_RFT 6 // interrupt rising/falling edge trigger

/*
 * @GPIO_PIN_OPTYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP 0 // push-pull
#define GPIO_OP_TYPE_OD 1 // open-drain

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW    0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_FAST   2
#define GPIO_SPEED_HIGH   3

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull-up pull-down configuration macros
 */
#define GPIO_NO_PUPD 0
#define GPIO_PIN_PU  1
#define GPIO_PIN_PD  2

/*
 * @GPIO_PIN_ALT_FUN_MODE
 * GPIO pin alternate function modes
 */
#define GPIO_AF0   0
#define GPIO_AF1   1
#define GPIO_AF2   2
#define GPIO_AF3   3
#define GPIO_AF4   4
#define GPIO_AF5   5
#define GPIO_AF6   6
#define GPIO_AF7   7
#define GPIO_AF8   8
#define GPIO_AF9   9
#define GPIO_AF10 10
#define GPIO_AF11 11
#define GPIO_AF12 12
#define GPIO_AF13 13
#define GPIO_AF14 14
#define GPIO_AF15 15

/**********************************************************************************************
 *                          API's supported by this driver (API Prototypes)
 *                  For more information about the APIs check the function definitions
 **********************************************************************************************/
/*
 * Peripheral Initialization and De-initialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Peripheral Clock Setup (EN/DI)
 * this function enables/disables the peripheral clock
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR Handling
 * this function configures the IRQ number and the IRQ priority (like enabling the IRQ)
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
// void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi); // this function combines the above two functions
void GPIO_IRQHandling(uint8_t PinNumber); // whenever the interrupt triggers this function can be called to handle the interrupt like a callback function.


#endif /* STM32F407XX_GPIO_DRIVER_H */