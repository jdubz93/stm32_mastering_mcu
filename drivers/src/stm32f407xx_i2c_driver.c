#include "stm32f407xx_i2c_driver.h"

/* prototypes for static helper functions */
static void I2C_GenStartCond(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
/* end of prototypes for static helper functions */

uint16_t AHB_PreScaler[8]  = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t  APB1_PreScaler[4] = {2, 4, 8, 16};

/**********************************************************************************************
 * @fn                - I2C_GenStartCond
 * @brief             - This is a static helper function that generates start condition on I2C bus
 * @param[in]         - base address of the I2C peripheral
 * @return            - none 
 * @Note              - static is specific to this file only.
 */
static void I2C_GenStartCond(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR[0] |= (1 << I2C_CR1_START);
}
/**********************************************************************************************
 * @fn                - I2C_GenStopCond
 * @brief             - This is a static helper function that generates stop condition on I2C bus
 * @param[in]         - base address of the I2C peripheral
 * @return            - none 
 * @Note              - static is specific to this file only.
 */
void I2C_GenStopCond(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR[0] |= (1 << I2C_CR1_STOP);
}
/**********************************************************************************************
 * @fn                - I2C_ExecuteAddressPhaseWrite
 * @brief             - This is a static helper function that executes address phase in write mode
 * @param[in]         - base address of the I2C peripheral
 * @param[in]         - slave address
 * @return            - none 
 * @Note              - static is specific to this file only.
 */
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    SlaveAddr = SlaveAddr << 1; // shift left by 1 to make room for r/w bit
    SlaveAddr &= ~(1);          // clear r/w bit = write
    pI2Cx->DR = SlaveAddr;      // write slave address to data register
}
/**********************************************************************************************
 * @fn                - I2C_ExecuteAddressPhaseRead
 * @brief             - This is a static helper function that executes address phase in read mode
 * @param[in]         - base address of the I2C peripheral
 * @param[in]         - slave address
 * @return            - none 
 * @Note              - static is specific to this file only.
 */
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; //SlaveAddr is Slave address + r/nw bit=1
	pI2Cx->DR = SlaveAddr;
}
/**********************************************************************************************
 * @fn                - I2C_ClearADDRFlag
 * @brief             - This is a static helper function that clears ADDR flag
 * @param[in]         - pointer to I2C handle structure
 * @return            - none 
 * @Note              - static is specific to this file only.
 */
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR[1] & ( 1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				//first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR[0];
				dummy_read = pI2CHandle->pI2Cx->SR[1];
				(void)dummy_read;
			}
		}
		else
		{
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR[0];
			dummy_read = pI2CHandle->pI2Cx->SR[1];
			(void)dummy_read;
		}
	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR[0];
		dummy_read = pI2CHandle->pI2Cx->SR[1];
		(void)dummy_read;
	}
}
/**********************************************************************************************
 * @fn                - I2C_MasterHandleTXEInterrupt
 * @brief             - This is a static helper function that handles TXE interrupt
 * @param[in]         - pointer to I2C handle structure
 * @return            - none 
 * @Note              - static is specific to this file only.
 */
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
    if(pI2CHandle->TxLen > 0)
    {
        //1. load the data into DR
        pI2CHandle->pI2Cx->DR = *pI2CHandle->pTxBuffer; // load data into DR, we dereference the pointer to get the data

        //2. Decrement the TxLen
        pI2CHandle->TxLen--;

        //3. Increment the buffer address
        pI2CHandle->pTxBuffer++;
    }
}
/**********************************************************************************************
 * @fn                - I2C_MasterHandleRXNEInterrupt
 * @brief             - This is a static helper function that handles RXNE interrupt
 * @param[in]         - pointer to I2C handle structure
 * @return            - none 
 * @Note              - static is specific to this file only.
 */
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
    // Keep receiving data until len becomes 0
    if(pI2CHandle->RxSize == 1)
    {
        // 1. Read the data from DR
        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        // 2. Decrement the RxLen
        pI2CHandle->RxLen--;
        // 3. Close the I2C data reception and notify the application
        I2C_CloseReceiveData(pI2CHandle);
    }

    if(pI2CHandle->RxSize > 1)
    {
        // Clear the ACK bit
        if(pI2CHandle->RxLen == 2)
            I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

        // 1. Read the data from DR
        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        // 2. Decrement the RxLen
        pI2CHandle->RxLen--;
        // 3. Increment the buffer address
        pI2CHandle->pRxBuffer++;
    }

    if(pI2CHandle->RxLen == 0)
    {
        // generate the STOP condition
        if(pI2CHandle->Sr == I2C_DISABLE_SR)
            I2C_GenStopCond(pI2CHandle->pI2Cx);
        
        // Close the I2C data reception and notify the application
        I2C_CloseReceiveData(pI2CHandle);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EVNT_RX_CMPLT);
    }
}

/**********************************************************************************************
 * @fn                - I2C_ManageAcking
 * @brief             - This function enables or disables ACK bit
 * @param[in]         - base address of the I2C peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @return            - none 
 * @Note              - none
 */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if(EnOrDi == I2C_ACK_ENABLE)
    {
        // Enable the ack
        pI2Cx->CR[0] |= (1 << I2C_CR1_ACK);
    }
    else {
        // Disable the ack
        pI2Cx->CR[0] &= ~(1 << I2C_CR1_ACK);
    }
}

/**********************************************************************************************
 * @fn                - I2C_PeriClockCtrl
 * @brief             - This function enables or disables peripheral clock for the given I2C port
 * @param[in]         - base address of the I2C peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @return            - none 
 * @Note              - none
 */
void I2C_PeriClockCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        if(pI2Cx == I2C1)
        {
            I2C1_PCLK_EN();
        }
        else if(pI2Cx == I2C2)
        {
            I2C2_PCLK_EN();
        }
        else if(pI2Cx == I2C3)
        {
            I2C3_PCLK_EN();
        }
    }
    else {
        if(pI2Cx == I2C1)
        {
            I2C1_PCLK_DI();
        }
        else if(pI2Cx == I2C2)
        {
            I2C2_PCLK_DI();
        }
        else if(pI2Cx == I2C3)
        {
            I2C3_PCLK_DI();
        }
    }
}

/**********************************************************************************************
 * @fn                - I2C_PeriCtrl
 * @brief             - This function enables or disables the I2C peripheral
 * @param[in]         - base address of the I2C peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @return            - none 
 * @Note              - none
 */
void I2C_PeriCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        pI2Cx->CR[0] |= (1 << I2C_CR1_PE);
    }
    else {
        pI2Cx->CR[0] &= ~(1 << 0);
    }
}


/**********************************************************************************************
 * @fn                - RCC_GetPCLK1Value
 * @brief             - This function gets the PCLK1 value based on the system clock and AHB prescaler + APB1 prescaler
 * @param[in]         - pointer to I2C handle structure
 * @return            - PCLK1 value
 * @Note              - none
 */
uint32_t RCC_GetPCLK1Value(void)
{
    uint32_t pclk1, SysClk;
    uint8_t clksrc, temp, ahbpre, apb1pre;

    // Get the value of CLKSRC
    clksrc = ((RCC->RCC_CFGR >> 2) & 0b11); /* Shift bits 2-3 to 0-1 position and then mask with 0b11 using & to read. Simple */
    if(clksrc == 0) /* HSI */
    {
        SysClk = 16000000; /* (16000000 = 16MHz) */
    } else if(clksrc == 1) /* HSE */
    {
        SysClk = 8000000; /* (8000000 = 8MHz) */
    } else if(clksrc == 2) /* PLL */
    {
        SysClk = RCC_GetPLLOutputClock(); /* Get PLL output clock */
    } else {
        // do nothing
    }

    // Get the value of AHB prescaler
    temp = ((RCC->RCC_CFGR >> 4) & 0xF); /* Shift bits 4-7 to 0-3 position and then mask with 0xF. */
    if(temp < 8) /* If AHB prescaler is less than 8, then it is not divided */
    {
        ahbpre = 1; /* (1 = not divided) */
    } else {
        ahbpre = AHB_PreScaler[temp - 8]; /* (temp - 8) to get the index of the AHB_PreScaler array */
    }

    // Get the value of APB1 prescaler
    temp = ((RCC->RCC_CFGR >> 10) & 0x7); /* Shift bits 10-12 to 0-2 position and then mask with 0x7. */

    if(temp < 4) /* If APB1 prescaler is less than 4, then it is not divided */
    {
        apb1pre = 1; /* (1 = not divided) */
    } else {
        apb1pre = APB1_PreScaler[temp - 4]; /* (temp - 4) to get the index of the APB1_PreScaler array */
    }

    // Calculate PCLK1
    pclk1 = (SysClk / ahbpre) / apb1pre; /* PCLK1 = (SysClk / AHB prescaler) / APB1 prescaler */
    return pclk1; /* Return PCLK1 value */
}

/**********************************************************************************************
 * @fn                - I2C_Deinit
 * @brief             - This function resets the I2C peripheral
 * @param[in]         - base address of the I2C peripheral
 * @return            - none
 * @Note              - none
 */
void I2C_Deinit(I2C_RegDef_t *pI2Cx)
{
    if(pI2Cx == I2C1)
    {
        I2C1_REG_RESET();
    }
    else if(pI2Cx == I2C2)
    {
        I2C2_REG_RESET();
    }
    else if(pI2Cx == I2C3)
    {
        I2C3_REG_RESET();
    }
    else {
        // do nothing
    }
    I2C_PeriClockCtrl(pI2Cx, DISABLE);
    I2C_PeriCtrl(pI2Cx, DISABLE);
}

/**********************************************************************************************
 * @fn                - I2C_Init
 * @brief             - This function initializes the I2C peripheral
 * @param[in]         - pointer to the I2C handle structure
 * @return            - none
 * @Note              - none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    // checkPeriCtrlDisabled(pI2CHandle->pI2Cx);

    // Enable peripheral clock
    I2C_PeriClockCtrl(pI2CHandle->pI2Cx, ENABLE);

    // Configure ACK
    // pI2CHandle->pI2Cx->CR[0] &= ~(1 << I2C_CR1_ACK);
    pI2CHandle->pI2Cx->CR[0] |= pI2CHandle->I2C_Config.I2C_ACKCtrl << I2C_CR1_ACK;

    // Configure FREQ
	uint32_t tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / (unsigned int)I2C_SCL_SPEED_SM;
	pI2CHandle->pI2Cx->CR[1] = (tempreg & 0x3F);

    // Configure device address
    pI2CHandle->pI2Cx->OAR[0] |= pI2CHandle->I2C_Config.I2C_DeviceAdrs << I2C_OAR1_ADD71;
    pI2CHandle->pI2Cx->OAR[0] |= 1 << I2C_OAR1_MUSTBEONE;

    // Configure CCR
    uint16_t ccr_value = 0;
    uint32_t temp_reg = 0;
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        // Standard mode
        ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        temp_reg |= (ccr_value & 0xFFF);
    } else {
        // Fast mode
		temp_reg |= ( 1 << 15);
		temp_reg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
        if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
        {
            // Fast mode duty cycle 2
            ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        } else {
            // Fast mode duty cycle 16/9
            ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
        }
        temp_reg |= (ccr_value & 0xFFF);
    }
    pI2CHandle->pI2Cx->CCR = temp_reg;
    temp_reg = 0;
    // can only configure trise when peripheral is disabled

    // Configure TRISE
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        // Standard mode
        temp_reg |= (RCC_GetPCLK1Value() / 1000000U) + 1; // divide by 1000000 to get MHz. 1000000 microseconds = 1MHz. 1000 nanoseconds = 1 microsecond.
    } else {
        // Fast mode
        temp_reg |= ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1; // 300 nanoseconds is the maximum rise time for fast mode. 1000000000 nanoseconds = 1 second. 
    }
    pI2CHandle->pI2Cx->TRISE |= (temp_reg & 0x3F);
}

/**********************************************************************************************
 * @fn                - checkPeriCtrlDisabled
 * @brief             - This function checks if the peripheral is disabled
 * @param[in]         - pointer to the I2C base address
 * @return            - void
 * @Note              - none
 */
void checkPeriCtrlDisabled(I2C_RegDef_t *pI2Cx)
{
    if(pI2Cx->CR[0] & (1 << I2C_CR1_PE))
    {
        I2C_PeriCtrl(pI2Cx, DISABLE);
    } else {
        // do nothing
    }
}
/**********************************************************************************************
 * @fn                - I2C_GetFlagStatus
 * @brief             - This function retrieves the status of a flag
 * @param[in]         - pointer to the I2C base address
 * @return            - void
 * @Note              - none
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
    if(pI2Cx->SR[0] & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}
/**********************************************************************************************
 * @fn                - I2C_MasterSendData
 * @brief             - This function sends data to the slave and is a blocking call
 * @param[in]         - pointer to the I2C handle structure, 
 *                                          ptr to txbuffer, 
 *                                          len of data, 
 *                                          slave addr, 
 *                                          repeated start
 * @return            - void
 * @Note              - none
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    // 1. Generate the START condition
    I2C_GenStartCond(pI2CHandle->pI2Cx);

    // 2. Confirm that start generation is completed by checking the SB flag in the SR1
    // Note: Until SB is cleared SCL will be stretched (pulled to low)
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

    // 3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
    I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

    // 4. Confirm that address phase is completed by checking the ADDR flag in the SR1
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

    // 5. Clear the ADDR flag according to its software sequence
    // Note: Until ADDR is cleared SCL will be stretched (pulled to low)
    I2C_ClearADDRFlag(pI2CHandle);

    // 6. Send the data until Len becomes 0
    while(Len > 0)
    {
        // 6.1 Check/Wait until TXE is set
        while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
        pI2CHandle->pI2Cx->DR = *pTxbuffer;
        // 6.3 Increment the buffer address
        pTxbuffer++;
        Len--;
    }

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

    // 9. Generate the STOP condition and master needs not to wait for the completion of stop condition
    // Note: generating STOP, automatically clears the BTF
    if(Sr == I2C_DISABLE_SR)
        I2C_GenStopCond(pI2CHandle->pI2Cx);

    // 10. Confirm that STOP generation is completed by checking the STOPF flag in the SR1
    // Note: Stop condition is automatically cleared by hardware when a STOPF is detected
    while(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_STOPF));

}

/**********************************************************************************************
 * @fn                - I2C_MasterReceiveData
 * @brief             - This function receives data from the slave
 * @param[in]         - pointer to the I2C handle structure, 
 *                                          ptr to rxbuffer, 
 *                                          len of data, 
 *                                          slave addr, 
 *                                          repeated start
 * @return            - void
 * @Note              - is a blocking call
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t slaveAddr, uint8_t Sr)
{
    /* 1. Generate the start condition */
    I2C_GenStartCond(pI2CHandle->pI2Cx);

    /* 2. Confirm the Start Bit generation is completed by checking SB Flag in SR1. 
        Until the Start bit is completd the SCL line will be held at low. */
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

    /* 3. Send the address byte to slave with the last bit (r/w) set to 1 for reading. */
    I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, slaveAddr);

    /* 4. wait until addr phase is completed by checking the addr flag in SR1 */
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));
    
    if(Len == 1)
    {
        // Disable ACKing
        I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

        // clear the ADDR flag
        I2C_ClearADDRFlag(pI2CHandle);

        // Check RXNE (wait until RXNE becomes 1)
        while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)); // the ! says wait until rxne becomes 1 then exit loop.

		//generate STOP condition
		if(Sr == I2C_DISABLE_SR)
			I2C_GenStopCond(pI2CHandle->pI2Cx);

        // Read the DR (read data in to buffer)
        *pRxbuffer = pI2CHandle->pI2Cx->DR;
    }

    /* read data when len is greater than 1 */
    if(Len > 1)
    {
        // clear the ADDR flag
        I2C_ClearADDRFlag(pI2CHandle);

        // read the data until len becomes zero
        for(uint32_t i = Len; i > 0; i--)
        {
            // wait until RXNE becomes one.
            while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

            if (i == 2) // if last two bytes of data
            {
                // clear the ACK bit (disable acking)
                I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//generate STOP condition
				if(Sr == I2C_DISABLE_SR)
					I2C_GenStopCond(pI2CHandle->pI2Cx);
            }

            // Read the data from data register into rx buffer.
            *pRxbuffer = pI2CHandle->pI2Cx->DR;

            // increment the buffer address
            pRxbuffer++;
        }
    }

    if(pI2CHandle->I2C_Config.I2C_ACKCtrl == I2C_ACK_ENABLE)
        I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE); // re-enable ACKing
}
/**********************************************************************************************
 * @fn                - I2C_MasterSendDataIT
 * @brief             - This function sends data to the slave
 * @param[in]         - pointer to the I2C handle structure, 
 *                                          ptr to txbuffer, 
 *                                          len of data, 
 *                                          slave addr, 
 *                                          repeated start
 * @return            - void
 * @Note              - is a non-blocking call
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

    if((busystate != I2C_BUSY_IN_RX) && (busystate != I2C_BUSY_IN_TX))
    {
        // 1. Save the Tx buffer address and Len information in some global variables
        pI2CHandle->pTxBuffer = pTxbuffer; // we dont dereference the pointer because we want to save the address of the buffer
        pI2CHandle->TxLen = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
        pI2CHandle->TxSize = Len; // Txsize is used in the ISR code to manage the data transmission
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        // 2. Generate the START condition
        I2C_GenStartCond(pI2CHandle->pI2Cx);

        // 3. Enable the ITBUFEN control bit to get interrupt whenever TXE flag is set in SR[0]
        pI2CHandle->pI2Cx->CR[1] |= (1 << I2C_CR2_ITBUFEN);

        // 4. Enable the ITEVFEN control bit to get interrupt whenever BTF, ADDR or STOPF flag is set in SR[0]
        pI2CHandle->pI2Cx->CR[1] |= (1 << I2C_CR2_ITEVTEN);

        // 5. Enable the ITERREN control bit to get interrupt whenever BERR or ARLO or AF or OVR or TIMEOUT flag is set in SR[0]
        pI2CHandle->pI2Cx->CR[1] |= (1 << I2C_CR2_ITERREN);
    }

    return busystate;
}
/**********************************************************************************************
 * @fn                - I2C_MasterReceiveDataIT
 * @brief             - This function receives data from the slave
 * @param[in]         - pointer to the I2C handle structure, 
 *                                          ptr to rxbuffer, 
 *                                          len of data, 
 *                                          slave addr, 
 *                                          repeated start
 * @return            - void
 * @Note              - is a non-blocking call
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

    if((busystate != I2C_BUSY_IN_RX) && (busystate != I2C_BUSY_IN_TX))
    {
        // 1. Save the Rx buffer address and Len information in some global variables
        pI2CHandle->pRxBuffer = pRxbuffer; // we dont dereference pRxbuffer because we want to save the address of the buffer
        pI2CHandle->RxLen = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
        pI2CHandle->RxSize = Len; // Rxsize is used in the ISR code to manage the data reception
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        // 2. Generate the START condition
        I2C_GenStartCond(pI2CHandle->pI2Cx);

        // 3. Enable the ITBUFEN control bit to get interrupt whenever TXE flag is set in SR1
        pI2CHandle->pI2Cx->CR[1] |= (1 << I2C_CR2_ITBUFEN);

        // 4. Enable the ITEVFEN control bit to get interrupt whenever BTF, ADDR or STOPF flag is set in SR1
        pI2CHandle->pI2Cx->CR[1] |= (1 << I2C_CR2_ITEVTEN);

        // 5. Enable the ITERREN control bit to get interrupt whenever BERR or ARLO or AF or OVR or TIMEOUT flag is set in SR1
        pI2CHandle->pI2Cx->CR[1] |= (1 << I2C_CR2_ITERREN);
    }

    return busystate;
}

/**********************************************************************************************
 * @fn                - I2C_SlaveSendData
 * @brief             - This function sends data to the master
 * @param[in]         - pointer to the I2C peripheral, 
 *                                          data to send
 * @return            - void
 * @Note              - none
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
    pI2Cx->DR = data;
}

/**********************************************************************************************
 * @fn                - I2C_SlaveReceiveData
 * @brief             - This function receives data from the master
 * @param[in]         - pointer to the I2C peripheral
 * @return            - data received
 * @Note              - none
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
    return (uint8_t)pI2Cx->DR;
}

/**********************************************************************************************
 * @fn                - I2C_IRQInterruptConfig
 * @brief             - This function configures the IRQ number and enables or disables it
 * @param[in]         - IRQ number, enable or disable, 
 * @return            - void
 * @Note              - none
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
    uint8_t nvicIdx, nvicBitPos;
    nvicIdx = IRQNumber / 32;
    nvicBitPos = IRQNumber % 32;
    if(EnOrDi == ENABLE){
        NVIC->ISER[nvicIdx] |= 1 << nvicBitPos;
    } else {
        NVIC->ICER[nvicIdx] |= 1 << nvicBitPos;
    }
}

/**********************************************************************************************
 * @fn                - I2C_IRQPriorityConfig
 * @brief             - This function configures the IRQ priority
 * @param[in]         - IRQ number, IRQ priority, 
 * @return            - void
 * @Note              - none
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    NVIC->IP[IRQNumber]  = IRQPriority << NO_PR_BITS_IMPLEMENTED; //shift by 4 is required because the lower 4 bits are not implemented
}

/**********************************************************************************************
 * @fn                - I2C_EV_IRQHandling
 * @brief             - This function handles the I2C event interrupt
 * @param[in]         - pointer to I2C handle structure, 
 * @return            - void
 * @Note              - none
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    // Interrupt Handling For Both Master & Slave Device
    uint32_t tmp1, tmp2, tmp3;

    // check if EVENT interrupt is set in CR2
    tmp1 = pI2CHandle->pI2Cx->CR[1] & (1 << I2C_CR2_ITEVTEN);
    tmp2 = pI2CHandle->pI2Cx->CR[1] & (1 << I2C_CR2_ITBUFEN);
    tmp3 = pI2CHandle->pI2Cx->SR[0] & (1 << I2C_SR1_SB);

    // 1. Handle for interrupt generated by SB event
    // Note: SB flag is only applicable in Master mode
    if(tmp1 && tmp3)
    {
        // SB flag is set
        // This block will not be executed in slave mode because for slave SB = 0
        // In this block lets execute the address phase
        uint8_t TxRxState = pI2CHandle->TxRxState;
        if(TxRxState == I2C_BUSY_IN_TX)
        {
            // 1. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
            I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
        else if(TxRxState == I2C_BUSY_IN_RX)
        {
            // 1. Send the address of the slave with r/nw bit set to r(1) (total 8 bits )
            I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
    }
    // 2. Handle for interrupt generated by ADDR event
    // Note: When master mode: Address is sent
    //       When slave mode: Address matched with own address
    tmp3 = pI2CHandle->pI2Cx->SR[0] & (1 << I2C_SR1_ADDR);
    if(tmp1 && tmp3)
    {
        // ADDR flag is set - when the ADDR flag is set the clock stretching is enabled and we must clear the ADDR flag
        // This block will not be executed in slave mode because for slave ADDR = 0
        I2C_ClearADDRFlag(pI2CHandle);

    }
    // 3. Handle for interrupt generated by BTF(Byte Transfer Finished) event
    // Note: This event is applicable only in Master mode
    //       This event is also called as "Transfer Complete" event
    tmp3 = pI2CHandle->pI2Cx->SR[0] & (1 << I2C_SR1_BTF);
    if(tmp1 && tmp3)
    {
        // BTF flag is set
        // This block will not be executed in slave mode because for slave BTF = 0
        // In this block lets close the I2C data transmission
        uint8_t TxRxState = pI2CHandle->TxRxState;
        if(TxRxState == I2C_BUSY_IN_TX)
        {
            // make sure that TXE is also set
            if(pI2CHandle->pI2Cx->SR[0] & (1 << I2C_SR1_TXE))
            {
                // BTF, TXE = 1
                // Generate the STOP condition
                if(pI2CHandle->Sr == I2C_DISABLE_SR)
                    I2C_GenStopCond(pI2CHandle->pI2Cx);

                // Close the I2C data transmission
                I2C_CloseSendData(pI2CHandle);

                // Inform the application that I2C transmission is over
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EVNT_TX_CMPLT);
            }
        }
        else if(TxRxState == I2C_BUSY_IN_RX)
        {
            // make sure that RXNE is also set
        }
        else { /* do nothing */ }
    }
    // 4. Handle for interrupt generated by STOPF event
    // Note: STOPF flag is only applicable in Slave mode
    //       When STOPF flag is set, it means slave has detected the STOP condition on the bus
    //       In this situation slave has to clear the STOPF flag
    //       Clearing of STOPF is done by software by reading the SR1 register followed by reading the DR register
    tmp3 = pI2CHandle->pI2Cx->SR[0] & (1 << I2C_SR1_STOPF);
    if(tmp1 && tmp3)
    {
        // STOPF flag is set
        // Clear the STOPF flag by reading SR1 and writing CR1
        pI2CHandle->pI2Cx->CR[0] |= 0x0000;
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EVNT_STOP);
    }
    // 5. Handle for interrupt generated by TXE event
    // Note: TXE flag is set by hardware when the data register is empty
    //       This event is applicable only in Master mode
    tmp3 = pI2CHandle->pI2Cx->SR[0] & (1 << I2C_SR1_TXE);
    if(tmp1 && tmp2 && tmp3) /* tmp2 is required because RXNE & TXE use both ITBUFEN & ITEVTEN before sending interrupt */
    {
        // TXE flag is set

        // Check to make sure the device is in master mode
        if(pI2CHandle->pI2Cx->SR[1] & (1 << I2C_SR2_MSL)) 
        {
            if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
                I2C_MasterHandleTXEInterrupt(pI2CHandle);
        }else {
            // slave mode
            
            /*  TRA: Transmitter/receiver
             *   0: Data bytes received
             *   1: Data bytes transmitted
             */

            // check the device is in TX mode, should return 1
            if(pI2CHandle->pI2Cx->SR[1] & (1 << I2C_SR2_TRA))
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EVNT_DATA_REQ);
    
        }

        // Note: if you need else make sure to add curly loops to avoid errors
    }
    // 6. Handle for interrupt generated by RXNE event
    // Note: RXNE flag is set by hardware when the data register is not empty
    //       This event is applicable only in Master mode
    tmp3 = pI2CHandle->pI2Cx->SR[0] & (1 << I2C_SR1_RXNE);
    if(tmp1 && tmp2 && tmp3) /* tmp2 is required because RXNE & TXE use both ITBUFEN & ITEVTEN before sending interrupt */
    {
        if(pI2CHandle->pI2Cx->SR[1] & (1 << I2C_SR2_MSL))
        {
            if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
                I2C_MasterHandleRXNEInterrupt(pI2CHandle);
        } else 
        {
            // slave mode

            /*  TRA: Transmitter/receiver
             *   0: Data bytes received
             *   1: Data bytes transmitted
             */

            // check the device is in RX mode, should return 0
            if(!(pI2CHandle->pI2Cx->SR[1] & (1 << I2C_SR2_TRA)))
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EVNT_DATA_RCV);
        }

        // Note: if you need else make sure to add curly loops to avoid errors
    }
}

/**********************************************************************
 * @fn				- I2C_ER_IRQHandling
 * @brief			- This function handles error interrupt for I2C peripheral
 * @param[in]		- Base address of the I2C peripheral
 * @return			- none
 * @Note			- none
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t tmp1, tmp2;

    tmp2 = pI2CHandle->pI2Cx->CR[1] & (1 << I2C_CR2_ITERREN);

    // 1. Handle for interrupt generated by BERR event
    // Note: BERR flag is set by hardware when a misplaced START or STOP condition is detected
    //       In this situation, SCL remains low until the situation is cleared
    //       This situation is cleared by software by reading the SR1 register followed by writing the CR1 register
    tmp1 = pI2CHandle->pI2Cx->SR[0] & (1 << I2C_SR1_BERR);
    if(tmp1 && tmp2)
    {
        // BERR flag is set

        // this is a bus error
        // clear the BERR flag by software writing 0 or by hardware when PE=0. Check Reference Manual.
        pI2CHandle->pI2Cx->SR[0] &= ~(1 << I2C_SR1_BERR);
        
        // inform the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
    }
    // 2. Handle for interrupt generated by ARLO event
    // Note: ARLO flag is set by hardware when a loss of arbitration occurs
    //       In this situation, SCL remains low until the situation is cleared
    //       This situation is cleared by software by reading the SR1 register followed by writing the CR1 register
    tmp1 = pI2CHandle->pI2Cx->SR[0] & (1 << I2C_SR1_ARLO);
    if(tmp1 && tmp2)
    {
        // ARLO flag is set

        // this is a arbitration lost error
        // clear the ARLO flag. Cleared by software writing 0, or by hardware when PE=0.
            // After an ARLO event the interface switches back automatically to Slave mode (MSL=0). 
        pI2CHandle->pI2Cx->SR[0] &= ~(1 << I2C_SR1_ARLO);

        // inform the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
    }
    // 3. Handle for interrupt generated by AF event
    // Note: AF flag is set by hardware when acknowledge failure occurs
    //       In this situation, SCL remains low until the situation is cleared
    //       This situation is cleared by software by reading the SR1 register followed by writing the CR1 register
    tmp1 = pI2CHandle->pI2Cx->SR[0] & (1 << I2C_SR1_AF);
    if(tmp1 && tmp2)
    {
        // AF flag is set

        // this is a acknowledge failure error. Set by hardware when the acknowledge is not received after the address or data byte is sent.
        // clear the AF flag. Cleared by software writing 0, or by hardware when PE=0.
        pI2CHandle->pI2Cx->SR[0] &= ~(1 << I2C_SR1_AF);

        // inform the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
    }
    // 4. Handle for interrupt generated by OVR event
    // Note: OVR flag is set by hardware when a data overrun/underrun occurs
    //       In this situation, SCL remains low until the situation is cleared
    //       This situation is cleared by software by reading the SR1 register followed by reading the DR register
    tmp1 = pI2CHandle->pI2Cx->SR[0] & (1 << I2C_SR1_OVR);
    if(tmp1 && tmp2)
    {
        // OVR flag is set

        // this is a overrun/underrun
        // clear the OVR flag
        pI2CHandle->pI2Cx->SR[0] &= ~(1 << I2C_SR1_OVR);

        // inform the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
    }
    // 5. Handle for interrupt generated by TIMEOUT event
    // Note: TIMEOUT flag is set by hardware when SCL remains low for 25ms
    //       In this situation, SCL remains low until the situation is cleared
    //       This situation is cleared by software by reading the SR1 register followed by writing the CR1 register
    tmp1 = pI2CHandle->pI2Cx->SR[0] & (1 << I2C_SR1_TIMEOUT);
    if(tmp1 && tmp2)
    {
        // TIMEOUT flag is set

        // this is a timeout error
        // clear the TIMEOUT flag
        pI2CHandle->pI2Cx->SR[0] &= ~(1 << I2C_SR1_TIMEOUT);

        // inform the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
    }
}

/**********************************************************************
 * @fn				- I2C_CloseReceiveData
 * @brief			- This function sends data in master mode
 * @param[in]		- I2C handle structure
 * @return			- none
 * @Note			- none
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
    // disable ITBUFEN control bit & ITEVTEN control bit
    pI2CHandle->pI2Cx->CR[1] &= ~(1 << I2C_CR2_ITBUFEN);
    pI2CHandle->pI2Cx->CR[1] &= ~(1 << I2C_CR2_ITEVTEN);
    
    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxSize = 0;
    pI2CHandle->RxLen = 0;

    if(pI2CHandle->I2C_Config.I2C_ACKCtrl == I2C_ACK_ENABLE)
        I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
}

/**********************************************************************
 * @fn				- I2C_CloseSendData
 * @brief			- This function sends data in master mode
 * @param[in]		- I2C handle structure
 * @return			- none
 * @Note			- none
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
    pI2CHandle->pI2Cx->CR[1] &= ~(1 << I2C_CR2_ITBUFEN);
    pI2CHandle->pI2Cx->CR[1] &= ~(1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TxSize = 0;
    pI2CHandle->TxLen = 0;
}

void I2C_AppErrorHandler(I2C_Handle_t *pI2CHandle)
{
    // Close the I2C data transmission
    if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        I2C_CloseSendData(pI2CHandle);
    
    if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        I2C_CloseReceiveData(pI2CHandle);

    // Generate the stop condition to release the SCL line
    I2C_GenStopCond(pI2CHandle->pI2Cx);
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        // CR2 register
        pI2Cx->CR[1] |= (1 << I2C_CR2_ITEVTEN);
        pI2Cx->CR[1] |= (1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR[1] |= (1 << I2C_CR2_ITERREN);
    }
    else
    {
        // CR2 register
        pI2Cx->CR[1] &= ~(1 << I2C_CR2_ITEVTEN);
        pI2Cx->CR[1] &= ~(1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR[1] &= ~(1 << I2C_CR2_ITERREN);
    }
}