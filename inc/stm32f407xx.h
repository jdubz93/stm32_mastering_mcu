#ifndef STM32F407XX_H_
#define STM32F407XX_H_

/*
 * Created by:     Josh Williams
 * Date Created:   12/27/2022
 * Last Modified:  12/27/2022
 * 
 * Description:    This file contains all the peripheral register definitions
 *                for the STM32F407xx microcontroller.
 * 1. The base addresses of various memories of STM32F407xx MCU such as
 *   FLASH, SRAM, ROM, etc.
 * 2. The base addresses of various bus domains of STM32F407xx MCU such as
 *  AHBx, APBx, etc.
 * 3. The base addresses of various peripherals hanging on various bus
 * domains of STM32F407xx MCU.
 * 4. Clock management macros (ie: clock enable/disable)
 * 5. IRQ (Interrupt Request) numbers/defintions.
 * 6. Peripheral Register defintion structures.
 * 7. Peripheral register bit definitions.
 * 8. Other useful microcontroller configuration macros.
 * ie: generic macros.
 */

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#define __weak __attribute__((weak))
#define __vo volatile

/*************************************************************************************
 *                                 Processor Specific Details
 *  Core: ARM Cortex M4 Processor     (Cortex M4 Processor)
 * Clock: 168 MHz                     (168 MHz)
 * Cores: 1                           (1)
 * Vendor: ST Microelectronics        (ST Microelectronics)
 * Architecture: ARMv7E-M             (ARMv7E-M)
 * Series: STM32F4                    (STM32F4)
 * Line: STM32F407xx                  (STM32F407xx)
 * Package: LQFP100                   (LQFP100)
 * RAM: 192 KB                        (192 KB)
 * ROM: 1 MB                          (1 MB)
 * EEPROM: 64 KB                      (64 KB)
 * GPIOs: 16                          (16)
 * ADCs: 3                            (3)
 * DACs: 2                            (2)
 * UARTs: 3                           (3)
 * SPIs: 5                            (5)
 * I2Cs: 3                            (3)
 * CANs: 2                            (2)
 * USBs: 2                            (2)
 * Ethernet: 1                        (1)
 */

/*
 * NVIC Base Address
 */
#define NVIC_BASEADDR                   0xE000E100

/*
 * NVIC Register structure
 * this should be in another header file specific to the m4 coretx (e.g. core_m4.h)
 */
typedef struct {
    __vo uint32_t ISER[8];  /* Interrupt Set-Enable Registers (8 x 32 bits)   Address Offset: 0x00 - 0x1C */
    uint32_t RESERVED0[24]; /* Reserved                                       Address Offset: 0x20 - 0x9C */
    __vo uint32_t ICER[8];  /* Interrupt Clear-Enable Registers (8 x 32 bits) Address Offset: 0xA0 - 0xBC */
    uint32_t RSERVED1[24];  /* Reserved                                       Address Offset: 0xC0 - 0x11C */
    __vo uint32_t ISPR[8];  /* Interrupt Set-Pending Registers (8 x 32 bits)  Address Offset: 0x120 - 0x13C */
    uint32_t RESERVED2[24]; /* Reserved                                       Address Offset: 0x140 - 0x17C */
    __vo uint32_t ICPR[8];  /* Interrupt Clear-Pending Registers (8 x 32 bits)Address Offset: 0x180 - 0x19C */
    uint32_t RESERVED3[24]; /* Reserved                                       Address Offset: 0x1A0 - 0x1DC */
    __vo uint32_t IABR[8];  /* Interrupt Active Bit Registers (8 x 32 bits)   Address Offset: 0x1E0 - 0x1FC */
    uint32_t RESERVED4[56]; /* Reserved                                       Address Offset: 0x200 - 0x2FC */
    __vo uint8_t  IP[240];  /* Interrupt Priority Registers (240 x 8 bits)    Address Offset: 0x300 - 0x3EC */
    uint32_t RESERVED5[644];/* Reserved                                       Address Offset: 0x3F0 - 0xEFC */
    __vo uint32_t STIR;     /* Software Trigger Interrupt Register            Address Offset: 0xF00 */
}NVIC_RegDef_t;

#define NVIC			((NVIC_RegDef_t*)	NVIC_BASEADDR)

/*
 * Not used anymore
 */
/*
 * ARM Cortex M4 Processor NVIC ISERx register addresses
 */
#define NVIC_ISER0                  ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1                  ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2                  ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3                  ((__vo uint32_t*)0xE000E10C)
/*
 * ARM Cortex M4 Processor NVIC ICERx register addresses
 */
#define NVIC_ICER0                  ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1                  ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2                  ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3                  ((__vo uint32_t*)0xE000E18C)
/*
 * ARM Cortex M4 Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR           ((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED      4

/*
 * NVIC Priority Register Macros
 */
#define NVIC_IRQ_PRI0               0
#define NVIC_IRQ_PRI1               1
#define NVIC_IRQ_PRI2               2
#define NVIC_IRQ_PRI3               3
#define NVIC_IRQ_PRI4               4
#define NVIC_IRQ_PRI5               5
#define NVIC_IRQ_PRI6               6
#define NVIC_IRQ_PRI7               7
#define NVIC_IRQ_PRI8               8
#define NVIC_IRQ_PRI9               9
#define NVIC_IRQ_PRI10              10
#define NVIC_IRQ_PRI11              11
#define NVIC_IRQ_PRI12              12
#define NVIC_IRQ_PRI13              13
#define NVIC_IRQ_PRI14              14
#define NVIC_IRQ_PRI15              15

/*********************End of Processor Specific Details**********************/

#define FLASH_BASEADDR              0x08000000U     // Flash Memory
#define SRAM1_BASEADDR              0x20000000U     // SRAM1
#define SRAM2_BASEADDR              0x2001C000U     // SRAM2
#define SYSMEM_BASEADDR             0x1FFF0000U     // System Memory = ROM
#define ROM                         SYSMEM_BASEADDR // ROM
#define SRAM                        SRAM1_BASEADDR  // SRAM = SRAM1

// Exercise find which peripheral is on 0x40000000U
// #define TIM2_PERIPH          0x40000000U // TIM2 PERIPHERAL

#define PERIPH_BASE                 0x40000000U // Peripheral Base Address
#define APB1PERIPH_BASEADDR         PERIPH_BASE // APB1 Peripheral Base Address
#define APB2PERIPH_BASEADDR         0x40010000U // APB2 Peripheral Base Address
#define AHB1PERIPH_BASEADDR         0x40020000U // AHB1 Peripheral Base Address
#define AHB2PERIPH_BASEADDR         0x50000000U // AHB2 Peripheral Base Address

/*
 * AHB1 PERIPHERALS BASE ADDRESSES
 */
#define GPIOA_BASEADDR              (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR              (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR              (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR              (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR              (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR                (AHB1PERIPH_BASEADDR + 0x3800)

/*
 * APB1 PERIPHERALS BASE ADDRESSES
 */
#define TIM2_BASEADDR               (APB1PERIPH_BASEADDR + 0x0000)
#define TIM3_BASEADDR               (APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR               (APB1PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR               (APB1PERIPH_BASEADDR + 0x0C00)
#define TIM6_BASEADDR               (APB1PERIPH_BASEADDR + 0x1000)
#define TIM7_BASEADDR               (APB1PERIPH_BASEADDR + 0x1400)
#define TIM12_BASEADDR              (APB1PERIPH_BASEADDR + 0x1800)
#define TIM13_BASEADDR              (APB1PERIPH_BASEADDR + 0x1C00)
#define TIM14_BASEADDR              (APB1PERIPH_BASEADDR + 0x2000)
#define RTC_BKP_BASEADDR            (APB1PERIPH_BASEADDR + 0x2800)
#define WWDG_BASEADDR               (APB1PERIPH_BASEADDR + 0x2C00)
#define IWDG_BASEADDR               (APB1PERIPH_BASEADDR + 0x3000)
#define I2S2EXT_BASEADDR            (APB1PERIPH_BASEADDR + 0x3400)
#define SPI2_BASEADDR               (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR               (APB1PERIPH_BASEADDR + 0x3C00)
#define I2S3EXT_BASEADDR            (APB1PERIPH_BASEADDR + 0x4000)
#define USART2_BASEADDR             (APB1PERIPH_BASEADDR + 0x4400) // NOTE: USART not UART
#define USART3_BASEADDR             (APB1PERIPH_BASEADDR + 0x4800) // NOTE USART not UART
#define UART4_BASEADDR              (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR              (APB1PERIPH_BASEADDR + 0x5000)
#define I2C1_BASEADDR               (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR               (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR               (APB1PERIPH_BASEADDR + 0x5C00)
#define CAN1_BASEADDR               (APB1PERIPH_BASEADDR + 0x6400)
#define CAN2_BASEADDR               (APB1PERIPH_BASEADDR + 0x6800)
#define PWR_BASEADDR                (APB1PERIPH_BASEADDR + 0x7000)
#define DAC_BASEADDR                (APB1PERIPH_BASEADDR + 0x7400)
#define UART7_BASEADDR              (APB1PERIPH_BASEADDR + 0x7800)
#define UART8_BASEADDR              (APB1PERIPH_BASEADDR + 0x7C00)

/*
 * APB2 PERIPHERALS BASE ADDRESSES
 */
#define TIM1_BASEADDR               (APB2PERIPH_BASEADDR + 0x0000)
#define TIM8_BASEADDR               (APB2PERIPH_BASEADDR + 0x0400)
#define USART1_BASEADDR             (APB2PERIPH_BASEADDR + 0x1000) // NOTE USART not UART
#define USART6_BASEADDR             (APB2PERIPH_BASEADDR + 0x1400) // NOTE USART not UART
#define ADC1_BASEADDR               (APB2PERIPH_BASEADDR + 0x2000)
#define ADC2_BASEADDR               (APB2PERIPH_BASEADDR + 0x2100)
#define ADC3_BASEADDR               (APB2PERIPH_BASEADDR + 0x2200)
#define SDIO_BASEADDR               (APB2PERIPH_BASEADDR + 0x2C00)
#define SPI1_BASEADDR               (APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR               (APB2PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR             (APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR               (APB2PERIPH_BASEADDR + 0x3C00)
#define TIM9_BASEADDR               (APB2PERIPH_BASEADDR + 0x4000)
#define TIM10_BASEADDR              (APB2PERIPH_BASEADDR + 0x4400)
#define TIM11_BASEADDR              (APB2PERIPH_BASEADDR + 0x4800)
#define SPI5_BASEADDR               (APB2PERIPH_BASEADDR + 0x5000)
#define SPI6_BASEADDR               (APB2PERIPH_BASEADDR + 0x5400)
#define SAI1_BASEADDR               (APB2PERIPH_BASEADDR + 0x5800)
#define LTDC_BASEADDR               (APB2PERIPH_BASEADDR + 0x6800)


/*
 * PERIPHERAL REGISTER DEFINITION STRUCTURES
 */

/* GPIOx */
typedef struct {
    __vo uint32_t MODER;          // GPIO port mode register,               Address offset: 0x00
    __vo uint32_t OTYPER;         // GPIO port output type register,        Address offset: 0x04
    __vo uint32_t OSPEEDR;        // GPIO port output speed register,       Address offset: 0x08
    __vo uint32_t PUPDR;          // GPIO port pull-up/pull-down register,  Address offset: 0x0C
    __vo uint32_t IDR;            // GPIO port input data register,         Address offset: 0x10
    __vo uint32_t ODR;            // GPIO port output data register,        Address offset: 0x14
    __vo uint32_t BSRR;           // GPIO port bit set/reset register,      Address offset: 0x18
    __vo uint32_t LCKR;           // GPIO port configuration lock register, Address offset: 0x1C
    __vo uint32_t AFR[2];         // GPIO alternate function low/high registers,     Address offset: 0x20-0x24 (AFRL[0], AFRH[1])
}GPIO_RegDef_t;

/* RCC */
typedef struct {
    __vo uint32_t RCC_CR;            // RCC clock control register,          Address offset: 0x00
    __vo uint32_t RCC_PLLCFGR;       // RCC PLL configuration register,      Address offset: 0x04
    __vo uint32_t RCC_CFGR;          // RCC clock configuration register,    Address offset: 0x08
    __vo uint32_t RCC_CIR;           // RCC clock interrupt register,        Address offset: 0x0C
    __vo uint32_t RCC_AHB1RSTR;      // RCC AHB1 peripheral reset register,  Address offset: 0x10
    __vo uint32_t RCC_AHB2RSTR;      // RCC AHB2 peripheral reset register,  Address offset: 0x14
    __vo uint32_t RCC_AHB3RSTR;      // RCC AHB3 peripheral reset register,  Address offset: 0x18
    uint32_t      RESERVED0;         // Reserved, 0x1C
    __vo uint32_t RCC_APB1RSTR;      // RCC APB1 peripheral reset register,  Address offset: 0x20
    __vo uint32_t RCC_APB2RSTR;      // RCC APB2 peripheral reset register,  Address offset: 0x24
    uint32_t      RESERVED1[2];      // Reserved, 0x28-0x2C
    __vo uint32_t RCC_AHB1ENR;       // RCC AHB1 peripheral clock enable register,  Address offset: 0x30
    __vo uint32_t RCC_AHB2ENR;       // RCC AHB2 peripheral clock enable register,  Address offset: 0x34
    __vo uint32_t RCC_AHB3ENR;       // RCC AHB3 peripheral clock enable register,  Address offset: 0x38
    uint32_t      RESERVED2;         // Reserved, 0x3C
    __vo uint32_t RCC_APB1ENR;       // RCC APB1 peripheral clock enable register, Address offset: 0x40
    __vo uint32_t RCC_APB2ENR;       // RCC APB2 peripheral clock enable register, Address offset: 0x44
    uint32_t      RESERVED3[2];      // Reserved, 0x48-0x4C
    __vo uint32_t RCC_AHB1LPENR;     // RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50
    __vo uint32_t RCC_AHB2LPENR;     // RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54
    __vo uint32_t RCC_AHB3LPENR;     // RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58
    uint32_t      RESERVED4;         // Reserved, 0x5C
    __vo uint32_t RCC_APB1LPENR;     // RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60
    __vo uint32_t RCC_APB2LPENR;     // RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64
    uint32_t      RESERVED5[2];      // Reserved, 0x68-0x6C
    __vo uint32_t RCC_BDCR;          // RCC Backup domain control register,  Address offset: 0x70
    __vo uint32_t RCC_CSR;           // RCC clock control & status register, Address offset: 0x74
    uint32_t      RESERVED6[2];      // Reserved, 0x78-0x7C
    __vo uint32_t RCC_SSCGR;         // RCC spread spectrum clock generation register, Address offset: 0x80
    __vo uint32_t RCC_PLLI2SCFGR;    // RCC PLLI2S configuration register,   Address offset: 0x84
}RCC_RegDef_t;

/* EXTI */
typedef struct {
    __vo uint32_t IMR;            // Interrupt mask register,              Address offset: 0x00
    __vo uint32_t EMR;            // Event mask register,                  Address offset: 0x04
    __vo uint32_t RTSR;           // Rising trigger selection register,    Address offset: 0x08
    __vo uint32_t FTSR;           // Falling trigger selection register,   Address offset: 0x0C
    __vo uint32_t SWIER;          // Software interrupt event register,    Address offset: 0x10
    __vo uint32_t PR;             // Pending register,                     Address offset: 0x14
}EXTI_RegDef_t;

/* SYSCFG */
typedef struct {
    __vo uint32_t MEMRMP;    // SYSCFG memory remap register,                         Address offset: 0x00
    __vo uint32_t PMC;       // SYSCFG peripheral mode configuration register,        Address offset: 0x04
    __vo uint32_t EXTICR[4];   // SYSCFG external interrupt configuration register,   Address offset: 0x08 - 0x14
    uint32_t      RESERVED1[2];      // Reserved, 0x18-0x1C
    __vo uint32_t CMPCR;     // SYSCFG Compensation cell control register,            Address offset: 0x20
    uint32_t      RESERVED2[2];      // Reserved, 0x24-0x28
    __vo uint32_t CFGR;      // SYSCFG configuration register,                        Address offset: 0x2C
}SYSCFG_RegDef_t;

/* SPI */
typedef struct {
    __vo uint32_t CR1;           // SPI control register 1,               Address offset: 0x00
    __vo uint32_t CR2;           // SPI control register 2,               Address offset: 0x04
    __vo uint32_t SR;            // SPI status register,                  Address offset: 0x08
    __vo uint32_t DR;            // SPI data register,                    Address offset: 0x0C
    __vo uint32_t CRCPR;         // SPI CRC polynomial register,          Address offset: 0x10
    __vo uint32_t RXCRCR;        // SPI RX CRC register,                  Address offset: 0x14
    __vo uint32_t TXCRCR;        // SPI TX CRC register,                  Address offset: 0x18
    __vo uint32_t I2SCFGR;       // SPI_I2S configuration register,       Address offset: 0x1C
    __vo uint32_t I2SPR;         // SPI_I2S prescaler register,           Address offset: 0x20
}SPI_RegDef_t;

// /* SPI */
// typedef struct {
//     __vo uint32_t CR[2];         /* SPI control register 1 & 2,           Address offset: 0x00 - 0x04 */
//     __vo uint32_t SR;            /* SPI status register,                  Address offset: 0x08 */
//     __vo uint32_t DR;            /* SPI data register,                    Address offset: 0x0C */
//     __vo uint32_t CRCPR;         /* SPI CRC polynomial register,          Address offset: 0x10 */
//     __vo uint32_t RXCRCR;        /* SPI RX CRC register,                  Address offset: 0x14 */
//     __vo uint32_t TXCRCR;        /* SPI TX CRC register,                  Address offset: 0x18 */
//     __vo uint32_t I2SCFGR;       /* SPI_I2S configuration register,       Address offset: 0x1C */
//     __vo uint32_t I2SPR;         /* SPI_I2S prescaler register,           Address offset: 0x20 */
// }SPI_RegDef_t;

/* I2C */
typedef struct {
    __vo uint32_t CR[2];         // I2C control register 1 & 2,           Address offset: 0x00 - 0x04
    __vo uint32_t OAR[2];        // I2C own address register 1 & 2,       Address offset: 0x08 - 0x0C
    __vo uint32_t DR;            // I2C data register,                    Address offset: 0x10
    __vo uint32_t SR[2];         // I2C status register 1 & 2,            Address offset: 0x14 - 0x18
    __vo uint32_t CCR;           // I2C clock control register,           Address offset: 0x1C
    __vo uint32_t TRISE;         // I2C TRISE register,                   Address offset: 0x20
    __vo uint32_t FLTR;          // I2C FLTR register,                    Address offset: 0x24
}I2C_RegDef_t;

// typedef struct
// {
//   __vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
//   __vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
//   __vo uint32_t OAR1;       /*!< TODO,     										Address offset: 0x08 */
//   __vo uint32_t OAR2;       /*!< TODO,     										Address offset: 0x0C */
//   __vo uint32_t DR;         /*!< TODO,     										Address offset: 0x10 */
//   __vo uint32_t SR1;        /*!< TODO,     										Address offset: 0x14 */
//   __vo uint32_t SR2;        /*!< TODO,     										Address offset: 0x18 */
//   __vo uint32_t CCR;        /*!< TODO,     										Address offset: 0x1C */
//   __vo uint32_t TRISE;      /*!< TODO,     										Address offset: 0x20 */
//   __vo uint32_t FLTR;       /*!< TODO,     										Address offset: 0x24 */
// }I2C_RegDef_t;

/* Peripheral structure pointers */
#define GPIOA  ((GPIO_RegDef_t*)GPIOA_BASEADDR) // NOTE: GPIOA is a pointer to a GPIO_RegDef_t struct mentioned above.
#define GPIOB  ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  ((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define RCC    ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI   ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
#define SPI1   ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2   ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3   ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4   ((SPI_RegDef_t*)SPI4_BASEADDR)
#define I2C1   ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2   ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3   ((I2C_RegDef_t*)I2C3_BASEADDR)

/* Clock Enable Macros for GPIOx peripherals */
#define GPIOA_PCLK_EN() (RCC->RCC_AHB1ENR |= (1 << 0) ) // 0th bit of RCC_AHB1ENR register is set to 1 to enable clock for GPIOA
#define GPIOB_PCLK_EN() (RCC->RCC_AHB1ENR |= (1 << 1) ) // 1st bit of RCC_AHB1ENR register is set to 1 to enable clock for GPIOB
#define GPIOC_PCLK_EN() (RCC->RCC_AHB1ENR |= (1 << 2) ) // 2nd bit of RCC_AHB1ENR register is set to 1 to enable clock for GPIOC
#define GPIOD_PCLK_EN() (RCC->RCC_AHB1ENR |= (1 << 3) ) // GPIOD
#define GPIOE_PCLK_EN() (RCC->RCC_AHB1ENR |= (1 << 4) ) // 4th bit points to GPIOE
#define GPIOF_PCLK_EN() (RCC->RCC_AHB1ENR |= (1 << 5) ) // 5th bit points to GPIOF
#define GPIOG_PCLK_EN() (RCC->RCC_AHB1ENR |= (1 << 6) ) // 6th bit points to GPIOG
#define GPIOH_PCLK_EN() (RCC->RCC_AHB1ENR |= (1 << 7) ) // 7th bit points to GPIOH
#define GPIOI_PCLK_EN() (RCC->RCC_AHB1ENR |= (1 << 8) ) // 8th bit points to GPIOI
/* Clock Enable Macros for I2Cx peripherals */
#define I2C1_PCLK_EN() (RCC->RCC_APB1ENR |= (1 << 21) ) // APB1 21st bit points to I2C1
#define I2C2_PCLK_EN() (RCC->RCC_APB1ENR |= (1 << 22) ) // APB1 22nd bit points to I2C2
#define I2C3_PCLK_EN() (RCC->RCC_APB1ENR |= (1 << 23) ) // APB1 23rd bit points to I2C3
/* Clock Enable Macros for SPIx peripherals */
#define SPI1_PCLK_EN() (RCC->RCC_APB2ENR |= (1 << 12) ) // APB2 12th bit points to SPI1
#define SPI4_PCLK_EN() (RCC->RCC_APB2ENR |= (1 << 13) ) // APB2 13th bit points to SPI4
#define SPI2_PCLK_EN() (RCC->RCC_APB1ENR |= (1 << 14) ) // APB1 14th bit points to SPI2
#define SPI3_PCLK_EN() (RCC->RCC_APB1ENR |= (1 << 15) ) // APB1 15th bit points to SPI3
/* Clock Enable Macros for USARTx peripherals */
#define USART1_PCLK_EN() (RCC->RCC_APB2ENR |= (1 << 4) )  // APB2 4th bit points to USART1
#define USART2_PCLK_EN() (RCC->RCC_APB1ENR |= (1 << 17) ) // APB1 17th bit points to USART2
#define USART3_PCLK_EN() (RCC->RCC_APB1ENR |= (1 << 18) ) // APB1 18th bit points to USART3
#define UART4_PCLK_EN() (RCC->RCC_APB1ENR  |= (1 << 19) ) // APB1 19th bit points to UART4
/* Clock Enable Macros for SYSCFG peripherals */
#define SYSCFG_PCLK_EN() (RCC->RCC_APB2ENR |= (0b1 << 14) ) // APB2 14th bit points to SYSCFG
/* Clock Enable Macros for UARTx peripherals */
#define UART4_PCLK_EN() (RCC->RCC_APB1ENR |= (1 << 19) ) // APB1 19th bit points to UART4
#define UART5_PCLK_EN() (RCC->RCC_APB1ENR |= (1 << 20) ) // APB1 20th bit points to UART5

/* Clock Disable Macros for GPIOx peripherals */
#define GPIOA_PCLK_DI() (RCC->RCC_AHB1ENR &= ~(1 << 0) ) // clear (reset) the 0th bit of RCC_AHB1ENR register to disable clock for GPIOA
#define GPIOB_PCLK_DI() (RCC->RCC_AHB1ENR &= ~(1 << 1) ) // clear GPIOB
#define GPIOC_PCLK_DI() (RCC->RCC_AHB1ENR &= ~(1 << 2) ) // clear GPIOC
#define GPIOD_PCLK_DI() (RCC->RCC_AHB1ENR &= ~(1 << 3) ) // clear GPIOD
#define GPIOE_PCLK_DI() (RCC->RCC_AHB1ENR &= ~(1 << 4) ) // clear GPIOE
#define GPIOF_PCLK_DI() (RCC->RCC_AHB1ENR &= ~(1 << 5) ) // clear GPIOF
#define GPIOG_PCLK_DI() (RCC->RCC_AHB1ENR &= ~(1 << 6) ) // clear GPIOG
#define GPIOH_PCLK_DI() (RCC->RCC_AHB1ENR &= ~(1 << 7) ) // clear GPIOH
#define GPIOI_PCLK_DI() (RCC->RCC_AHB1ENR &= ~(1 << 8) ) // clear GPIOI
/* Clock Disable Macros for I2Cx peripherals */
#define I2C1_PCLK_DI() (RCC->RCC_APB1ENR &= ~(1 << 21) ) // clear APB1 21st bit to disable clock for I2C1
#define I2C2_PCLK_DI() (RCC->RCC_APB1ENR &= ~(1 << 22) ) // clear APB1 22nd bit to disable clock for I2C2
#define I2C3_PCLK_DI() (RCC->RCC_APB1ENR &= ~(1 << 23) ) // clear APB1 23rd bit to disable clock for I2C3
/* Clock Disable Macros for SPIx peripherals */
#define SPI1_PCLK_DI() (RCC->RCC_APB2ENR &= ~(1 << 12) ) // clear APB2 12th bit to disable clock for SPI1
#define SPI4_PCLK_DI() (RCC->RCC_APB2ENR &= ~(1 << 13) ) // clear APB2 13th bit to disable clock for SPI4
#define SPI2_PCLK_DI() (RCC->RCC_APB1ENR &= ~(1 << 14) ) // clear APB1 14th bit to disable clock for SPI2
#define SPI3_PCLK_DI() (RCC->RCC_APB1ENR &= ~(1 << 15) ) // clear APB1 15th bit to disable clock for SPI3
/* Clock Disable Macros for USARTx peripherals */
#define USART1_PCLK_DI() (RCC->RCC_APB2ENR &= ~(1 << 4) )  // clear APB2 4th bit to disable clock for USART1
#define USART2_PCLK_DI() (RCC->RCC_APB1ENR &= ~(1 << 17) ) // clear APB1 17th bit to disable clock for USART2
#define USART3_PCLK_DI() (RCC->RCC_APB1ENR &= ~(1 << 18) ) // clear APB1 18th bit to disable clock for USART3
#define UART4_PCLK_DI() (RCC->RCC_APB1ENR  &= ~(1 << 19) ) // clear APB1 19th bit to disable clock for UART4
/* Clock Disable Macros for SYSCFG peripherals */
#define SYSCFG_PCLK_DI() (RCC->RCC_APB2ENR &= ~(1 << 14) ) // clear APB2 14th bit to disable clock for SYSCFG
/* Clock Disable Macros for UARTx peripherals */
#define UART4_PCLK_DI() (RCC->RCC_APB1ENR &= ~(1 << 19) )  // clear APB1 19th bit to disable clock for UART5
#define UART5_PCLK_DI() (RCC->RCC_APB1ENR &= ~(1 << 20) )  // clear APB1 20th bit to disable clock for UART6

/* GPIOx Reset Macros */
#define GPIOA_REG_RESET() do{(RCC->RCC_AHB1RSTR |= (1 << 0) ); (RCC->RCC_AHB1RSTR &= ~(1 << 0) );}while(0) // set and clear the 0th bit of RCC_AHB1RSTR register to reset GPIOA peripheral port.
/* Note: we use a do-while loop here so that we can include two statements into one macro, since we have to first set the bit before we can (reset) the bit. 
  this is a trick that is used in C to execute multiple statements in single macro */
#define GPIOB_REG_RESET() do{(RCC->RCC_AHB1RSTR |= (1 << 1) ); (RCC->RCC_AHB1RSTR &= ~(1 << 1) );}while(0) // reset GPIOB
#define GPIOC_REG_RESET() do{(RCC->RCC_AHB1RSTR |= (1 << 2) ); (RCC->RCC_AHB1RSTR &= ~(1 << 2) );}while(0) // reset GPIOC
#define GPIOD_REG_RESET() do{(RCC->RCC_AHB1RSTR |= (1 << 3) ); (RCC->RCC_AHB1RSTR &= ~(1 << 3) );}while(0) // reset GPIOD
#define GPIOE_REG_RESET() do{(RCC->RCC_AHB1RSTR |= (1 << 4) ); (RCC->RCC_AHB1RSTR &= ~(1 << 4) );}while(0) // reset GPIOE
#define GPIOF_REG_RESET() do{(RCC->RCC_AHB1RSTR |= (1 << 5) ); (RCC->RCC_AHB1RSTR &= ~(1 << 5) );}while(0) // reset GPIOF
#define GPIOG_REG_RESET() do{(RCC->RCC_AHB1RSTR |= (1 << 6) ); (RCC->RCC_AHB1RSTR &= ~(1 << 6) );}while(0) // reset GPIOG
#define GPIOH_REG_RESET() do{(RCC->RCC_AHB1RSTR |= (1 << 7) ); (RCC->RCC_AHB1RSTR &= ~(1 << 7) );}while(0) // reset GPIOH
#define GPIOI_REG_RESET() do{(RCC->RCC_AHB1RSTR |= (1 << 8) ); (RCC->RCC_AHB1RSTR &= ~(1 << 8) );}while(0) // reset GPIOI
/* SPIx Reset Macros */
#define SPI1_REG_RESET() do{(RCC->RCC_APB2RSTR |= (1 << 12) ); (RCC->RCC_APB2RSTR &= ~(1 << 12) );}while(0) // reset SPI1
#define SPI2_REG_RESET() do{(RCC->RCC_APB1RSTR |= (1 << 14) ); (RCC->RCC_APB1RSTR &= ~(1 << 14) );}while(0) // reset SPI2
#define SPI3_REG_RESET() do{(RCC->RCC_APB1RSTR |= (1 << 15) ); (RCC->RCC_APB1RSTR &= ~(1 << 15) );}while(0) // reset SPI3
#define SPI4_REG_RESET() do{(RCC->RCC_APB2RSTR |= (1 << 13) ); (RCC->RCC_APB2RSTR &= ~(1 << 13) );}while(0) // reset SPI4
/* I2Cx Reset Macros */
#define I2C1_REG_RESET() do{(RCC->RCC_APB1RSTR |= (1 << 21) ); (RCC->RCC_APB1RSTR &= ~(1 << 21) );}while(0) // reset I2C1
#define I2C2_REG_RESET() do{(RCC->RCC_APB1RSTR |= (1 << 22) ); (RCC->RCC_APB1RSTR &= ~(1 << 22) );}while(0) // reset I2C2
#define I2C3_REG_RESET() do{(RCC->RCC_APB1RSTR |= (1 << 23) ); (RCC->RCC_APB1RSTR &= ~(1 << 23) );}while(0) // reset I2C3



/*
 * IRQ (Interrupt Request) Numbers of STM32F407x MCU mapped to the processor
 */
#define IRQ_NO_EXTI0        6
#define IRQ_NO_EXTI1        7
#define IRQ_NO_EXTI2        8
#define IRQ_NO_EXTI3        9
#define IRQ_NO_EXTI4        10
#define IRQ_NO_EXTI9_5      23
#define IRQ_NO_EXTI15_10    40

#define IRQ_NO_SPI1         35 /* address: 0x0000 00CC | priority 42 */
#define IRQ_NO_SPI2         36 /* address: 0x0000 00D0 | priority 43 */
#define IRQ_NO_SPI3         51 /* address: 0x0000 0144 | priority 58 */

#define IRQ_NO_I2C1_EV      31 /* Event Trigger | address: 0x0000 00B8 | priority 38 */
#define IRQ_NO_I2C1_ER      32 /* Error Trigger | address: 0x0000 00BC | priority 39 */



/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/* Bit position definitions SPI_CR1 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15
/* Bit position definitions SPI_CR2 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7
/* Bit position definitions SPI_SR & possible SPI interrupt errors | search "28.4.8 Error flags" in datasheet */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8


/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/* Bit position definitions I2C_CR1 */
#define I2C_CR1_PE						 0
#define I2C_CR1_SMBUS                    1
/* reserved bit 2 */
#define I2C_CR1_SMBTYPE                  3
#define I2C_CR1_ENARP                    4
#define I2C_CR1_ENPEC                    5
#define I2C_CR1_ENGC                     6
#define I2C_CR1_NOSTRETCH                7
#define I2C_CR1_START                    8
#define I2C_CR1_STOP                     9
#define I2C_CR1_ACK                      10
#define I2C_CR1_POS                      11
#define I2C_CR1_PEC                      12
#define I2C_CR1_ALERT                    13
/* reserved bit 14 */
#define I2C_CR1_SWRST                    15
/* Bit position definitions I2C_CR2 */
#define I2C_CR2_FREQ                     0 /* 6 bits */
/* reserved bits 6-7 */
#define I2C_CR2_ITERREN                  8
#define I2C_CR2_ITEVTEN                  9
#define I2C_CR2_ITBUFEN                  10
#define I2C_CR2_DMAEN                    11
#define I2C_CR2_LAST                     12
/* Bit position definitions I2C_OAR1 */
#define I2C_OAR1_ADD0                    0 /* 1 bit */
#define I2C_OAR1_ADD71                   1 /* 7 bits */
#define I2C_OAR1_ADD98                   8 /* 2 bits */
#define I2C_OAR1_MUSTBEONE               14 /* 14th bit should always be kept at 1 by software */
#define I2C_OAR1_ADDMODE                 15
/* Bit position definitions I2C_OAR2 */
#define I2C_OAR2_ENDUAL                  0
#define I2C_OAR2_ADD2                    1 /* 7 bits */
/* Bit position definitions I2C_DR */
#define I2C_DR_DR                        0 /* 8 bits */
/* Bit position definitions I2C_SR1 */
#define I2C_SR1_SB                       0
#define I2C_SR1_ADDR                     1
#define I2C_SR1_BTF                      2
#define I2C_SR1_ADD10                    3
#define I2C_SR1_STOPF                    4
#define I2C_SR1_RXNE                     6
#define I2C_SR1_TXE                      7
#define I2C_SR1_BERR                     8
#define I2C_SR1_ARLO                     9
#define I2C_SR1_AF                       10
#define I2C_SR1_OVR                      11
#define I2C_SR1_PECERR                   12
#define I2C_SR1_TIMEOUT                  14
#define I2C_SR1_SMBALERT                 15
/* Bit position definitions I2C_SR2 */
#define I2C_SR2_MSL                     0
#define I2C_SR2_BUSY                    1
#define I2C_SR2_TRA                     2
#define I2C_SR2_GENCALL                 4
#define I2C_SR2_SMBDEFAUL               5
#define I2C_SR2_SMBHOST                 6
#define I2C_SR2_DUALF                   7
#define I2C_SR2_PEC70                   8
/* Bit position definitions I2C_CCR */
#define I2C_CCR_CCR                     0 /* 12 bits */
#define I2C_CCR_DUTY                    14
#define I2C_CCR_FS                      15
/* Bit position definitions I2C_TRISE */
#define I2C_TRISE_TRISE                 0 /* 6 bits */

#define I2C_DEVICE_ADDRESS_7BIT         0

/******************************************************************************************
 * Generic Macros
 ******************************************************************************************/
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define FLAG_RESET      RESET
#define FLAG_SET        SET
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET
#define I2C_ENABLE_SR   SET
#define I2C_DISABLE_SR  RESET

/* Placed here because it could be used for other peripherals (I2C/SPI/etc..) */
#define CONVERT_BASEADDR_TO_PORTLETTER(x)  ((x == GPIOA) ? 0b0    :\
                                            (x == GPIOB) ? 0b1    :\
                                            (x == GPIOC) ? 0b10   :\
                                            (x == GPIOD) ? 0b11   :\
                                            (x == GPIOE) ? 0b100  :\
                                            (x == GPIOF) ? 0b101  :\
                                            (x == GPIOG) ? 0b110  :\
                                            (x == GPIOH) ? 0b111  :\
                                            (x == GPIOI) ? 0b1111 :0 ) // returns the GPIO port code for the given GPIO base address
                                    /*
                                    0000: PA[x] pin
                                    0001: PB[x] pin
                                    0010: PC[x] pin
                                    0011: PD[x] pin
                                    0100: PE[x] pin
                                    0101: PF[x] pin
                                    0110: PG[x] pin
                                    0111: PH[x] pin
                                    1000: PI[x] pin
                                    */

// includes
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"

#endif /* STM32F407XX_H_ */