#ifndef _GPIO_H_
#define _GPIO_H_
#include <stdint.h>
#include "header.h"

typedef struct
{
	uint32_t GPIO_PinNumber;
	uint32_t GPIO_PinMode;
	uint32_t GPIO_PinSpeed;
	uint32_t GPIO_PinPulPdControl;
	uint32_t GPIO_PinOPType;
	uint32_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;
 
typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinCongig;
}GPIO_Handle_t;

/*
 * this macro return a code between 0 to 7 for a gieven GPIO bade address
 */
#define GPIO_BASEADDR_TO_CODE(x) ((x == GPIOA) ? 0 : (x == GPIOB) ? 1 \
                                                   : (x == GPIOC) ? 2 \
                                                   : (x == GPIOD) ? 3 \
                                                   : (x == GPIOE) ? 4 \
                                                   : (x == GPIOF) ? 5 \
                                                   : (x == GPIOG) ? 6 \
                                                   : (x == GPIOH) ? 7 \
                                                                  : 0)
/*
 * GPIO pin number
 */
#define GPIO_PIN_0 0
#define GPIO_PIN_1 1
#define GPIO_PIN_2 2
#define GPIO_PIN_3 3
#define GPIO_PIN_4 4
#define GPIO_PIN_5 5
#define GPIO_PIN_6 6
#define GPIO_PIN_7 7
#define GPIO_PIN_8 8
#define GPIO_PIN_9 9
#define GPIO_PIN_10 10
#define GPIO_PIN_11 11
#define GPIO_PIN_12 12
#define GPIO_PIN_13 13
#define GPIO_PIN_14 14
#define GPIO_PIN_15 15

/*
 *GPIO pin mode
 */
#define GPIO_MODE_IN 0
#define GPIO_MODE OUT 1
#define GPIO_MODE_ALTFN 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT 4
#define GPIO_MODE_IT_RT 5
#define GPIO_MODE_IT_RFT 6

/*
 *GPIO speed
 */
#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_FAST 2
#define GPIO_SPEED_HIGH 3

/*
 *GPIO output type
 */
#define GPIO_OP_TYPE_PP 0
#define GPIO_OP_TYPE_OD 1

/*
 *GPIO pin pull up & pull down
 */
#define GPIO_NO_PUPD 0
#define GPIO_PIN_PU 1
#define GPIO_PIN_PD 2

#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESER

/*
 *prototype
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_IRQIntercruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutoutPort( GPIO_RegDef_t *pGPIOx, uint16_t value);
#endif
