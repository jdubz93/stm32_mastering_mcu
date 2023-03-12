/*
 * Created by:     Josh Williams
 * Date Created:   12/27/2022
 * Last Modified:  12/27/2022
 * 
 * Description:    This file contains spi driver implementation code for stm32f4-DISC1 board (stm32f407 mcu).
 *
 */

#include "stm32f407xx_spi_driver.h"

/* 
 * Uses the keyword static to indicate these are private functions, the user source code should not have access 
 * to these helper functions. If compiler tries to access these functions, it will throw an error.
 */
static void spi_txne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_crc_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

static void spi_close_tx_interrupt(SPI_Handle_t *pSPIHandle);
static void spi_close_rx_interrupt(SPI_Handle_t *pSPIHandle);

// // spi peripheral error interrupt handlers
// static void spi_crcerr_interrupt_handle(SPI_Handle_t *pSPIHandle);
// static void spi_fre_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
// static void spi_modf_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
// static void spi_udr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


// Peripheral Clock Init/Deinit

/******************************************************************************************
 * @fn				- SPI_Init
 * @brief			- This function initializes the SPI peripheral
 * @param[in]		- Base address of the SPI peripheral
 * @return			- void
 * @note			- none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){
    // Enable the peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    if(pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER){
        // Set the device mode to master
        pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

		// if the device is master mode, then the SSI must also be 1 to avoid an error (unless Multi master?)
		// this is only true if SSM is enabled, can we keep this functionality even is SSM is disabled?
		// based on stm generated code, the SSI stays enabled for SSM enabled and disabled
		// pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSI); // this should probably be done in another way
    } else {
        // Set the device mode to slave
        pSPIHandle->pSPIx->CR1 &= ~(pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);
    }

    // Configure the bus config
    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
        // bidi mode should be cleared
        pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
    } else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
        // bidi mode should be set
        pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_BusConfig << SPI_CR1_BIDIMODE);
    } else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
        // bdi mode should be cleared and rxonly should be set
        pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
        pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_BusConfig << SPI_CR1_RXONLY);
    } else {
        // Do nothing
    }

    // Configure the spi serial clock speed (baud rate)
    pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);
    // Configure the DFF
    pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);
    // Configure the CPOL
    pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);
    // Configure the CPHA
    pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);
    // Configure the SSM
    pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);
}

/******************************************************************************************
 * @fn				- SPI_DeInit
 * @brief			- This function de-initializes the SPI peripheral
 * @param[in]		- Base address of the SPI peripheral
 * @return			- void
 * @note			- none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx){
    if(pSPIx == SPI1){
        SPI1_REG_RESET();
    } else if(pSPIx == SPI2){
        SPI2_REG_RESET();
    } else if(pSPIx == SPI3){
        SPI3_REG_RESET();
    } else if(pSPIx == SPI4){
        SPI4_REG_RESET();
    } else {
        // Do nothing
    }
}

/******************************************************************************************
 * @fn				- SPI_GetFlagStatus
 * @brief			- This function returns the status of the given flag
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- Flag name
 * @return			- Flag status
 * @note			- for Tx and Rx, the status is 0 when the buffer is empty
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
    if(pSPIx->SR & FlagName){ // if the flag is set (1) then return 1
        return FLAG_SET;
    }
    /* */
    return FLAG_RESET;
}

/******************************************************************************************
 * @fn                - SPI_ClearOVRFlag
 * @brief             - This function clears SPI overrun flag
 * @param[in]         - SPI base address pointer
 * @return            - none
 * @note              - none
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
    uint8_t temp;
    // read the DR register to clear the OVR flag
    temp = pSPIx->DR;
    // read the SR register to clear the OVR flag
    temp = pSPIx->SR;
    (void)temp; // to avoid compiler warning
}

/******************************************************************************************
 * @fn                - SPI_CloseTransmission
 * @brief             - This function closes SPI transmission
 * @param[in]         - SPI handle pointer
 * @return            - none
 * @note              - none
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
    spi_close_tx_interrupt(pSPIHandle);
}

/******************************************************************************************
 * @fn                - SPI_CloseReception
 * @brief             - This function closes SPI reception
 * @param[in]         - SPI handle pointer
 * @return            - none
 * @note              - none
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
    spi_close_rx_interrupt(pSPIHandle);
}

// Peripheral Clock Setup

/******************************************************************************************
 * @fn				- SPI_PeriClockControl
 * @brief			- This function enables or disables peripheral clock for the given SPI port
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @return			- void
 * @note			- none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
    if(EnorDi == ENABLE){
        if(pSPIx == SPI1){
            SPI1_PCLK_EN();
        } else if(pSPIx == SPI2){
            SPI2_PCLK_EN();
        } else if(pSPIx == SPI3){
            SPI3_PCLK_EN();
        } else if(pSPIx == SPI4){
            SPI4_PCLK_EN();
        } else {
            // Do nothing
        }
    } else {
        if(pSPIx == SPI1){
            SPI1_PCLK_DI();
        } else if(pSPIx == SPI2){
            SPI2_PCLK_DI();
        } else if(pSPIx == SPI3){
            SPI3_PCLK_DI();
        } else if(pSPIx == SPI4){
            SPI4_PCLK_DI();
        } else {
            // Do nothing
        }
    }
}

/******************************************************************************************
 * @fn				- SPI_PeripheralControl
 * @brief			- This function enables or disables the SPI peripheral
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @return			- void
 * @note			- making SSOE 1 does NSS output enable.
 *                    The NSS pin is automatically managed by the hardware.
 *                    i.e when SPE=1 , NSS will be pulled to low
 *                    and NSS pin will be high when SPE=0
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
    if(EnorDi == ENABLE){
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    } else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

/******************************************************************************************
 * @fn				- SPI_SSIConfig
 * @brief			- This function enables or disables the SSI bit
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @return			- void
 * @note			- none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
    if(EnorDi == ENABLE){
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    } else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}

/******************************************************************************************
 * @fn				- SPI_SSOEConfig
 * @brief			- This function enables or disables the SSOE bit
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @return			- void
 * @note			- none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
    if(EnorDi == ENABLE){
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    } else {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}

// Data Read/Write (Blocking)

/******************************************************************************************
 * @fn				- SPI_ReceiveData
 * @brief			- This function reads the input data from the given SPI pin
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- Pin number
 * @return			- void
 * @note			- none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){
    while(Len > 0) {
        //1. wait until RXNE is set (RX buffer is not empty). Whenever RXNE is 1, it means RX buffer is not empty and read from DR is possible
        while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);
        //2. check the DFF bit in CR1
        if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
            //16 bit DFF
            //1. load the data from DR to RxBuffer
            *((uint16_t*)pRxBuffer) = pSPIx->DR;
            //2. increment the buffer address
            Len--;
            Len--;
            (uint16_t*)pRxBuffer++;
        } else {
            //8 bit DFF
            //1. load the data from DR to RxBuffer
            *pRxBuffer = pSPIx->DR;
            //2. increment the buffer address
            Len--;
            pRxBuffer++;
        }
    }
}

/******************************************************************************************
 * @fn				- SPI_SendData
 * @brief			- This function writes the given value to the given SPI pin
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- Pin number
 * @param[in]		- 0 or 1
 * @return			- void
 * @note			- none
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
    while(Len > 0) {
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);
        //2. check the DFF bit in CR1
        if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
            //16 bit DFF
            //1. load the data in to the DR
            pSPIx->DR = *((uint16_t*)pTxBuffer);
            Len--;
            Len--;
            (uint16_t*)pTxBuffer++;
        } else {
            //8 bit DFF
            pSPIx->DR = *pTxBuffer;
            Len--;
            pTxBuffer++;
        }
    }
}


// IRQ Configuration and ISR Handling

/******************************************************************************************
 * @fn				- SPI_IRQInterruptConfig
 * @brief			- This function configures the given IRQ number
 * @param[in]		- IRQ number
 * @param[in]		- ENABLE or DISABLE macros
 * @return			- void
 * @note			- none
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
    uint8_t nvicIdx, nvicBitPos;
    nvicIdx = IRQNumber / 32;
    nvicBitPos = IRQNumber % 32;
    if(EnorDi == ENABLE){
        NVIC->ISER[nvicIdx] |= 1 << nvicBitPos;
    } else {
        NVIC->ICER[nvicIdx] |= 1 << nvicBitPos;
    }
}


/******************************************************************************************
 * @fn				- SPI_IRQPriorityConfig
 * @brief			- This function configures the given IRQ priority
 * @param[in]		- IRQ number
 * @param[in]		- IRQ priority
 * @return			- void
 * @note			- none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
    //NOTE: because we used uint8_t for the IPR register, we can select the desired register directly
    NVIC->IP[IRQNumber]  = IRQPriority << NO_PR_BITS_IMPLEMENTED; //shift by 4 is required because the lower 4 bits are not implemented
}

/******************************************************************************************
 * @fn				- SPI_IRQHandling
 * @brief			- This function handles the given IRQ number
 * @param[in]		- SPI handle pointer and Pin number
 * @return			- void
 * @note			- EXTI pointer to the EXTI module and PR is a register in the module that contains the pending request flags for each external interrupt line.
 *                  The & operator is a bitwise AND operator. It performs a bitwise AND operation on the value in the PR register and the value (1 << PinNumber).
 *                  ie: if the value in the PR register is 0b00000000 and the value (1 << PinNumber) is 0b00000001, the result of the bitwise AND operation is 0b00000000 and the if statement is false(skipped).
 *                  if the result is 1, the if statement is true and the pending request flag is cleared.        
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
	uint8_t status, control;

    // check for TXE
	status = pSPIHandle->pSPIx->SR & SPI_TXE_FLAG; // will be 1 if TXE is set
	control = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE); // will be 1 if TXEIE is set

	if ( status && control ) {
		spi_txne_interrupt_handle(pSPIHandle);
	}

	// check for RXNE
	status = pSPIHandle->pSPIx->SR & SPI_RXNE_FLAG; // will be 1 if RXNE is set
	control = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE); // will be 1 if RXNEIE is set

	if ( status && control ) {
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	// check for Overun flag (OVR) this tells the application we just lost data because we didn't read the data fast enough
	status = pSPIHandle->pSPIx->SR & SPI_OVR_FLAG; // will be 1 if OVR is set
	control = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE); // will be 1 if ERRIE is set

	if ( status && control ) {
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}

    // // check for CRCERR flag
    // status = pSPIHandle->pSPIx->SR & SPI_CRCERR_FLAG; // will be 1 if CRCERR is set
    // control = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE); // will be 1 if ERRIE is set

    // if (status && control) {
    //     spi_crc_err_interrupt_handle(pSPIHandle);
    // }
}

/**********************************************************************************************
 * @fn                - SPI_SendDataIT
 * @brief             - This function sends data using interrupt
 * @param[in]         - SPI handle pointer
 * @param[in]         - Tx buffer pointer
 * @param[in]         - Length of the data
 * @return            - uint8_t
 * @note              - none
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){
    uint8_t state = pSPIHandle->TxState;
    /* Only do this if the TxState is free or ready */
    if(state != SPI_BUSY_IN_TX){
        // 1. Save the Tx buffer address and Len information in some global variables
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;
        // 2. Mark the SPI state as busy in the SPI handle, so no other code can take over same API peripheral until transmission is over
        pSPIHandle->TxState = SPI_BUSY_IN_TX;
        // 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
    }

	/* 
        why return state here and not pSPIHandle->TxState?
	    if the TxState is not busy, then we configure to send as an intterupt, which presumably executes
	    in this case, state will return as SPI_NOT_BUSY
	    this is intended because this function will only return once the data is all sent
	    if TxState is busy, then this will return, that it is busy, which will allow the while loop to try again
    */
    return state; // BUSY_IN_TX or READY or BUSY_IN_RX
}

/**********************************************************************************************
 * @fn                - SPI_ReceiveDataIT
 * @brief             - This function receives data using interrupt
 * @param[in]         - SPI handle pointer
 * @param[in]         - Rx buffer pointer
 * @param[in]         - Length of the data
 * @return            - uint8_t
 * @note              - none
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){
    uint8_t state = pSPIHandle->RxState;
    /* Only do this if the RxState is free or ready */
    if(state != SPI_BUSY_IN_RX)
    {                         
        /* 1. Save the Rx buffer address and Len information in some global variables */
        pSPIHandle->pRxBuffer   = pRxBuffer;             
        /* 2. Mark the SPI state as busy in the SPI handle */
        pSPIHandle->RxLen       = Len;
        pSPIHandle->RxState     = SPI_BUSY_IN_RX;
        /* 3. Enable the RXNEIE control bit to get interrupt whenever RXNE flag is set in SR */ 
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
    }
    return state; // BUSY_IN_RX or READY or BUSY_IN_TX
}

/******************************************************************
 * @fn                - SPI_ApplicationEventCallback
 * @brief             - This function is a weak implementation of the application callback 
 * @param[in]         - SPI handle pointer
 * @param[in]         - Event
 * @return            - none
 * @note              - none
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){
    // This is a weak implementation. The application may override this function.
}


/* Helper Functions */

/**********************************************************************************************
 * @fn                - spi_txe_interrupt_handle
 * @brief             - This function handles the TXE interrupt
 * @param[in]         - SPI handle pointer
 * @return            - none
 * @note              - none
 */
void spi_txne_interrupt_handle(SPI_Handle_t *pSPIHandle){
    /* check if 16 bit or 8 bit */
    if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){ /* 16-bit data frame format */
        pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
        pSPIHandle->TxLen--;
        pSPIHandle->TxLen--;
        (uint16_t*)pSPIHandle->pTxBuffer++;
    } else { /* 8-bit data frame format */
        pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
        pSPIHandle->TxLen--;
        pSPIHandle->pTxBuffer++;
    }
    /* if TxLen is 0, then close the SPI data transmission */
    if(!pSPIHandle->TxLen){
        /* 
         * Close SPI transmission and inform the application that Tx is over
         * This prevents interrupts from setting up of TXE flag 
         */
        SPI_CloseTransmission(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}

/**********************************************************************************************
 * @fn                - spi_rxne_interrupt_handle
 * @brief             - This function handles the RXNE interrupt
 * @param[in]         - SPI handle pointer
 * @return            - none
 * @note              - none
 */
void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	//do rxing as per the dff
	if(pSPIHandle->pSPIx->CR1 & ( 1 << 11))
	{
		//16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
        // pSPIHandle->RxLen--;
        // pSPIHandle->RxLen--;
        pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;

	}else
	{
		//8 bit
        *(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(!pSPIHandle->RxLen)
	{
		//reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}

/******************************************************************8
 * @fn                - spi_ovr_err_interrupt_handle
 * @brief             - this function handles SPI overrun err interrupt
 * @param[in]         - SPI handle pointer
 * @return            - none
 * @note              - none
 */
void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
    uint8_t temp;
    // 1. Clear the OVR flag
    if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
        temp = pSPIHandle->pSPIx->DR;
        temp = pSPIHandle->pSPIx->SR;
    }
    (void)temp;
    // 2. Inform the application
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

/**********************************************************************************************
 * @fn                - spi_crc_err_interrupt_handle
 * @brief             - This function handles the CRC error interrupt
 * @param[in]         - SPI handle pointer
 * @return            - none
 * @note              - none
 */
void spi_crc_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
    uint8_t temp;
    // 1. Clear the CRCERR flag
    temp = pSPIHandle->pSPIx->SR;
    temp = pSPIHandle->pSPIx->DR;
    (void)temp;
    // 2. Inform the application
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_CRC_ERR);
}

/******************************************************************8
 * @fn                - spi_close_tx_interrupt
 * @brief             - This function closes SPI transmission
 * @param[in]         - SPI handle pointer
 * @return            - none
 * @note              - none
 */
void spi_close_tx_interrupt(SPI_Handle_t *pSPIHandle){
    // Disable the TXEIE control bit
    /* This prevents interrupts setting of TXE flag */
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    pSPIHandle->pTxBuffer  = NULL;
    pSPIHandle->TxLen      = 0;
    pSPIHandle->TxState    = SPI_READY;
}

/******************************************************************8
 * @fn                - spi_close_rx_interrupt
 * @brief             - This function closes SPI reception
 * @param[in]         - SPI handle pointer
 * @return            - none
 * @note              - none
 */
void spi_close_rx_interrupt(SPI_Handle_t *pSPIHandle){
    // Disable the RXNEIE control bit
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pSPIHandle->pRxBuffer  = NULL;
    pSPIHandle->RxLen      = 0;
    pSPIHandle->RxState    = SPI_READY;
}

/**********************************************************************************************
 * @fn                - spi_modf_err_interrupt_handle
 * @brief             - This function handles the MODF error interrupt
 * @param[in]         - SPI handle pointer
 * @return            - none
 * @note              - only neede for multi master communication
 */
// void spi_modf_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
//     uint8_t temp;
//     // 1. Clear the MODF flag
//     temp = pSPIHandle->pSPIx->SR;
//     temp = pSPIHandle->pSPIx->DR;
//     (void)temp;
//     // 2. Inform the application
//     SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_MODF_ERR);
// }

// void spi_fre_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
//     uint8_t temp;
//     // 1. Clear the FRE flag
//     temp = pSPIHandle->pSPIx->SR;
//     temp = pSPIHandle->pSPIx->DR;
//     (void)temp;

//     // 2. Inform the application
//     SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_FRE_ERR);
// }

// void spi_udr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
//     uint8_t temp;
//     // 1. Clear the UDR flag
//     temp = pSPIHandle->pSPIx->SR;
//     temp = pSPIHandle->pSPIx->DR;
//     (void)temp;
//     // 2. Inform the application
//     SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_UDR_ERR);
// }