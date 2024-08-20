#include "myspi.h"
#include "header.h"

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if( EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
		   SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPi3_PCLK_EN();
		}
	else
	{
		if(pSPIx == SPI1)
		{
		  SPI1_PCLK_DI();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3)
		{
			SPi3_PCLK_DI();
		}
	}	
}
}

/**
  * @brief  Initialize the SPI according to the specified parameters
  *         in the SPI_InitTypeDef and initialize the associated handle.
  * @param  pSPIHandler pointer to a SPI_Handle_t structure that contains
  *               the configuration information for SPI module.
  */
void SPI_Init(SPI_Handler_t *pSPIHandle)
{
	//enable clock for SPI
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	
	uint32_t tempreg = 0;
	
	//1. configure the bus device mode
	tempreg |= pSPIHandle->SPICongif.SPI_DeviceMode << SPI_CR1_MSTR;
	
	//2. configure the bus config
	
	if(pSPIHandle->SPICongif.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1<< SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPICongif.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= 1<<SPI_CR1_BIDIMODE;
	}
	else if(pSPIHandle->SPICongif.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		// Set RNONLY bit
		tempreg |= 1<<SPI_CR1_RXONLY;
	}
	//3. Configure the SPI serial clock speed
	tempreg |= pSPIHandle->SPICongif.SPI_SclkSpeed << SPI_CR1_BR;
	//4. Config the Data frame
	tempreg |= pSPIHandle->SPICongif.SPI_DFF << SPI_CR1_DFF;
	//5. configure the CPOL
	tempreg |= pSPIHandle->SPICongif.CPOL << SPI_CR1_CPOL;
	//6. configure the CPHA
	tempreg |= pSPIHandle->SPICongif.CPHA << SPI_CR1_CPHA;
	
	tempreg|= pSPIHandle->SPICongif.SSM << SPI_CR1_SSM;
	
	//config SPI CR1 register
	pSPIHandle->pSPIx->CR1 = tempreg;
	
}
/**
  * @brief  Checks whether the specified SPI flag is set or not.
  * @param  pSPIx: where x can be 1 to 17 to select the SPI peripheral.
  * @param  FlagName: specifies the flag to check.
 
  */
uint8_t SPI_GetFlagStatus( SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void SPI_SendData( SPI_RegDef_t *pSPIx, uint8_t* pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_SR_TXE) == FLAG_RESET);
		//2. check DFF bit in CR1
		if((pSPIx->CR1 & ( 1<< SPI_CR1_DFF)))
		{
			//16 bit DFF
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			Len--;
			Len--;
			(uint16_t *)pTxBuffer++;
		}
		else 
		{
			//8 bit DFF
			pSPIx->DR = *((uint8_t *)pTxBuffer);
			Len--;
			pTxBuffer++;
		}
	}
}

//
uint16_t SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer)
{
	//1. wait until RXNE is set
	while((SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET));
	return (uint16_t)pSPIx->DR;
}
//
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

//

void SPI_SSOEConfig( SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

//

void SPI_IRQIntercruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= 1<<IRQNumber;
		}
		else if((IRQNumber > 31) && (IRQNumber < 64))
		{
			//program ISER1 register
			*NVIC_ISER1 |= 1<<(IRQNumber%32);
		}
		else if((IRQNumber > 64) && (IRQNumber < 96))
		{
			//program ISER2 register
			*NVIC_ISER2 |= 1<<(IRQNumber%64);
		}
	}
  else
	{
		if(IRQNumber <= 31)
		{
			//program ISCER0 register
			*NVIC_ICER0 |= (1<<IRQNumber);
		}
		else if((IRQNumber > 31) && (IRQNumber < 64))
		{
			//program ICER1 register
			*NVIC_ICER1 |= 1<<(IRQNumber%32);
		}
		else if((IRQNumber > 64) && (IRQNumber < 96))
		{
			//program ISCER2 register
			*NVIC_ICER2 |= 1<<(IRQNumber%64);
		}
	}
}

//

uint8_t  SPI_SendDataIT( SPI_Handler_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state == SPI_BUSY_IN_TX)
	{
		//1. Save the Tx Buffer address and len information in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2. Mark the SPI states as busy in transmission
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//3. Enable the TXEIE control bit to get intercrupt whenever TXE flag is set
		pSPIHandle->pSPIx->CR2 |= 1 << SPI_CR2_TXEIE;
	}
	return state;
}

//

uint8_t  SPI_ReceiveDataIT( SPI_Handler_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state == SPI_BUSY_IN_RX)
	{
		//1. Save the Tx Buffer address and len information in global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2. Mark the SPI states as busy in transmission
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		//3. Enable the TXEIE control bit to get intercrupt whenever RXNE flag is set
		pSPIHandle->pSPIx->CR2 |= 1 << SPI_CR2_RXNEIE;
	}
	return state;
}

//

void SPI_IRQHandling(SPI_Handler_t *pHandle)
{
	uint8_t temp1, temp2;
	//TXF
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1<< SPI_CR2_TXEIE);
	if(temp1 && temp2)
	{
		//hanle TXE
	}
	//RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1<< SPI_CR2_RXNEIE);
	if(temp1 && temp2)
	{
		//hanle RXNE
	}
	//ovr flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1<< SPI_CR2_ERRIE);
	if(temp1 && temp2)
	{
		//ovr error
	}
}

//

static void spi_txe_intercrupt_handle(SPI_Handler_t * pSPIHandle)
{
	// Check the DFF bit CR1 register
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
	{
		//16 bit data 
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else
	{
		//8bit data
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	if(!pSPIHandle->TxLen)
	{
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

//

void SPI_CloseTransmission(SPI_Handler_t *pHandle)
{
	pHandle->pSPIx->CR2 &= ~( 1<< SPI_CR2_TXEIE);
	pHandle->TxLen = 0;
	pHandle->TxState = SPI_READY;
}

//

static void spi_rxne_intercrupt_handle(SPI_Handler_t * pSPIHandle)
{
	// Check the DFF bit CR1 register
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
	{
		//16 bit data 
		*((uint16_t *)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}
	else
	{
		//8bit data
		*(pSPIHandle->pRxBuffer)  = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}
	if(!pSPIHandle->RxLen)
	{
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

//

void SPI_CloseReception(SPI_Handler_t *pHandle)
{
	pHandle->pSPIx->CR2 &= ~( 1<< SPI_CR2_RXNEIE);
	pHandle->RxLen = 0;
	pHandle->RxState = SPI_READY;
}