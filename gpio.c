#include "gpio.h"
#include "header.h"

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if( EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
		   GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
		   GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}	
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	
	//enble the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	//config mode of gpio pin
	if(pGPIOHandle->GPIO_PinCongig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinCongig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinCongig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2*pGPIOHandle->GPIO_PinCongig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		//intercrupt
		if(pGPIOHandle->GPIO_PinCongig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// config the FTSR
			EXTI->FTSR |= ( 1<< pGPIOHandle->GPIO_PinCongig.GPIO_PinNumber);
			//Clear the corresspond RTSR bit
			EXTI->RTSR &= ~( 1<< pGPIOHandle->GPIO_PinCongig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinCongig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// config the RTSR
			EXTI->RTSR |= ( 1<< pGPIOHandle->GPIO_PinCongig.GPIO_PinNumber);
			//Clear the corresspond FTSR bit
			EXTI->FTSR &= ~( 1<< pGPIOHandle->GPIO_PinCongig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinCongig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// config the RTSR and FTSR
			EXTI->RTSR |= ( 1<< pGPIOHandle->GPIO_PinCongig.GPIO_PinNumber);
			
			EXTI->FTSR |= ( 1<< pGPIOHandle->GPIO_PinCongig.GPIO_PinNumber);
		}
		
		// configure InterCrupt Mask 
		EXTI->IMR |= 1<<pGPIOHandle->GPIO_PinCongig.GPIO_PinNumber;
		
		//congigure the GPIO port selecttion in SYSCFG_EXTICR 
		uint8_t temp1 = pGPIOHandle->GPIO_PinCongig.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinCongig.GPIO_PinNumber%4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PLCK_EN();
		SYSCFG->EXTICR[temp1] |= portcode << (temp2*4);
	}
	//config speed
	temp = (pGPIOHandle->GPIO_PinCongig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinCongig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinCongig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	
	//config pull up pull down
	
	temp = (pGPIOHandle->GPIO_PinCongig.GPIO_PinPulPdControl << (2*pGPIOHandle->GPIO_PinCongig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinCongig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	
	//config oput type
	
	temp = (pGPIOHandle->GPIO_PinCongig.GPIO_PinOPType << (2*pGPIOHandle->GPIO_PinCongig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (2*pGPIOHandle->GPIO_PinCongig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	
	//config the alt function
	if(pGPIOHandle->GPIO_PinCongig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
    temp1 = pGPIOHandle->GPIO_PinCongig.GPIO_PinNumber/8;
    temp2 = pGPIOHandle->GPIO_PinCongig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinCongig.GPIO_PinAltFunMode << (4*temp2));
	}
}

//
void GPIO_IRQIntercruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}
//
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= 1<< PinNumber;
	}
	else
	{
	  pGPIOx->ODR &= ~(1<<PinNumber);
	}
}

//
void GPIO_WriteToOutoutPort( GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}