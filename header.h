#ifndef _HEADER_H_
#define _HEADER_H_

#include <stdint.h>
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE

#define __vo volatile
#define __weak __attribute__((weak))

//
#define APB1PERIPH_BASEADDR 0x40000000U
#define APB2PERIPH_BASEADDR 0x40010000U
#define AHB1PERIPH_BASEADDR 0x40020000U
#define AHB2PERIPH_BASEADDR 0x50020000U

//
#define GPIOA_BASEADDR (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR (AHB1PERIPH_BASEADDR + 0x2000)

#define RCC_BASEADDR     (AHB1PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR    (APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR  (APB2PERIPH_BASEADDR + 0x3800)

#define SPI1_BASEADDR (APB2PERIPH_BASEADDR + 0x3000)
#define SPI2_BASEADDR (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR (APB1PERIPH_BASEADDR + 0x3C00)
#define SPI4_BASEADDR (APB2PERIPH_BASEADDR + 0x3400)

#define I2C1_BASEADDR (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR (APB1PERIPH_BASEADDR + 0x5C00)

typedef struct
{
	__vo uint32_t CR;             /*!< RCC clocl control register,                                */
	__vo uint32_t PLLCFGR;        /*!< RCC PLL configuration register,                            */
	__vo uint32_t CFGR;           /*!< RCC clock configuration register,                          */
	__vo uint32_t CIR;            /*!< RCC clock intercrupt register,                             */
	__vo uint32_t AHB1RSTR;       /*!< RCC AHB1 peripheral reset register,                        */
	__vo uint32_t AHB2RSTR;       /*!< RCC AHB2 peripheral reset register,                        */
	__vo uint32_t AHB3RSTR;       /*!< RCC AHB3 peripheral reset register,                        */
	uint32_t RESERVED0;           /*!< Reserved, 0x1C,                                            */
	__vo uint32_t APB1RSTR;       /*!< RCC APB1 peripheral reset register,                        */
	__vo uint32_t APB2RSTR;       /*!< RCC APB2 peripheral reset register,                        */
	uint32_t RESERVED1[2];        /*!< Reseved, 0x28-0x2C,                                        */
	__vo uint32_t AHB1ENR;        /*!< RCC AHB1 peripheral enable register,                       */
	__vo uint32_t AHB2ENR;        /*!< RCC AHB2 peripheral enable register,                       */
	__vo uint32_t AHB3ENR;        /*!< RCC AHB3 peripheral enable register,                       */
	uint32_t RESEVED2;            /*!< Reseverd, 0x3C,                                            */
	__vo uint32_t APB1ENR;        /*!< RCC APB1 peripheral enable register,                       */
	__vo uint32_t APB2ENR;        /*!< RCC APB2 peripheral enable register,                       */
	uint32_t RESERVED3[2];        /*!< Reseved, 0x48-0x4C,                                        */
	__vo uint32_t AHB1LPENR;      /*!< RCC AHB1 peripheral enable in low power mode register,     */
	__vo uint32_t AHB2LPENR;      /*!< RCC AHB2 peripheral enable in low power mode register,     */
	__vo uint32_t AHB3LPENR;      /*!< RCC AHB3 peripheral enable in low power mode register,     */
	uint32_t RESEVED4;            /*!< Reseved, 0x5C,                                             */
	__vo uint32_t APB1LPENR;      /*!< RCC APB1 peripheral enable in low power mode register,     */
	__vo uint32_t APB2LPENR;      /*!< RCC APB2 peripheral enable in low power mode register,     */
	uint32_t RESERVED5[2];        /*!< Reseved, 0x68-0x6C,                                        */
	__vo uint32_t BDCR;           /*!< RCC Backup domain control register,                        */
	__vo uint32_t CSR;            /*!< RCC clock control & status register,                       */
	uint32_t RESERVED6[2];        /*!< Reseved, 0x78-0x7C,                                        */
	__vo uint32_t SSCGR;          /*!< RCC spread spectrum clock generation register,             */
	__vo uint32_t PLLI2SCFGR;     /*!< RCC PLLI2S configuration register,                         */
	__vo uint32_t PLLSAICFGR;     /*!< RCC PLLSAI configuration register,                         */
	__vo uint32_t DCKCFGR;        /*!< RCC dedicated clock configuration register,                */
	__vo uint32_t CKGATENR;       /*!< RCC clock gated enable register,                           */
	__vo uint32_t DCKCFGR2;       /*!< RCC dedicated clocks configuration register 2,             */
	
}RCC_RegDef_t;

#define RCC ((RCC_RegDef_t *)RCC_BASEADDR)

typedef struct
{
	__vo uint32_t MODER;          /*!< GPIO port mode register,                  */
	__vo uint32_t OTYPER;         /*!< GPIO port output type register,           */
	__vo uint32_t OSPEEDR;        /*!< GPIO port speed register,                 */
	__vo uint32_t PUPDR;          /*!< GPIO port pull up pull down register,      */
	__vo uint32_t IDR;            /*!< GPIO port input data register,            */
	__vo uint32_t ODR;            /*!< GPIO port output data register,           */
	__vo uint32_t BSRR;           /*!< GPIO port bit set/reset register,         */
	__vo uint32_t LCKR;           /*!< GPIO port configuration clock register,   */
	__vo uint32_t AFR[2];         /*!< GIIO alternate function register,         */
}GPIO_RegDef_t;

#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t *)GPIOI_BASEADDR)

#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1<<7))

#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1<<7))

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
	
}EXTI_RegDef_t;

#define EXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)

typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
	uint32_t RESERVED2[2];
	__vo uint32_t CFGR;
}SYSCFG_RegDef_t;

#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SYSCFG_PLCK_EN() (RCC->APB2ENR |= (1<<14))
#define SYSCFG_PLCK_DI() (RCC->APB2ENR &= ~(1<<14))

/*
 * ARM Cortex Mx NVIC ISERs register address
 */
#define NVIC_ISER0 ((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1 ((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2 ((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3 ((__vo uint32_t *)0xE000E10C)
/*
 * ARM Cortex Mx NVIC ICERs register address
 */
#define NVIC_ICER0 ((__vo uint32_t *)0xE000E180)
#define NVIC_ICER1 ((__vo uint32_t *)0xE000E184)
#define NVIC_ICER2 ((__vo uint32_t *)0xE000E188)
#define NVIC_ICER3 ((__vo uint32_t *)0xE000E18C)

/*
 * Vecto IRQ_Handler
 */
#define IRQ_NO_EXTI0 6
#define IRQ_NO_EXTI1 7
#define IRQ_NO_EXTI2 8
#define IRQ_NO_EXTI3 9
#define IRQ_NO_EXTI4 10
#define IRQ_NO_EXTI9_5 23
#define IRQ_NO_EXTI15_10 40
#define IRQ_NO_SPI1 35
#define IRQ_NO_SPI2 36
#define IRQ_NO_SPI3 51
#define IRQ_NO_I2C1_EV 31
#define IRQ_NO_I2C1_ER 32
#define IRQ_NO_I2C2_EV 33
#define IRQ_NO_I2C2_ER 34
#define IRQ_NO_UART1 37
#define IRQ_NO_UART2 38
#define IRQ_NO_UART3 39
#define IRQ_NO_UART4 52
#define IRQ_NO_UART5 53
#define IRQ_NO_UART6 71
/*
 * SPI Struct register 
 */
typedef struct 
{
	__vo uint32_t CR1;     /*!< SPI control register,        */
	__vo uint32_t CR2;     /*!< SPI control register 2,      */
	__vo uint32_t SR;      /*!< SPI status register,         */
	__vo uint32_t DR;      /*!< SPI data register,           */
	__vo uint32_t CRCPR;   /*!< SPI CRC polynomial register, */
	__vo uint32_t RXCRCR;  /*!< SPI RX CRC register,         */
	__vo uint32_t TXCRCR;  /*!< SPI TX CRC register,         */
	__vo uint32_t I2SCFG;  /*!< SPI I2S confiuration register*/
	__vo uint32_t I2SPR;   /*!< SPI I2S Prescaler register,  */
}SPI_RegDef_t;

#define SPI1 ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t *)SPI3_BASEADDR)

#define SPI1_PLCK_EN() (RCC->APB2ENR |= 1<<12)
#define SPI1_PLCK_DI() (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PLCK_EN() (RCC->APB1ENR |= 1<<14)
#define SPI2_PLCK_DI() (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PLCK_EN() (RCC->APB1ENR |= 1<<15)
#define SPI3_PLCK_DI() (RCC->APB1ENR &= ~(1<<15))
/*
 *Bit definition for SPI_CR1 register
*/
#define SPI_CR1_CPHA 0
#define SPI_CR1_CPOL 1
#define SPI_CR1_MSTR 2
#define SPI_CR1_BR   3
#define SPI_CR1_SPE  6
#define SPI_CR1_LSBFIRST 6
#define SPI_CR1_SSI 8
#define SPI_CR1_SSM 9
#define SPI_CR1_RXONLY 10
#define SPI_CR1_DFF 11
#define SPI_CR1_CRCNEXT 12
#define SPI_CR1_CRCEN 13
#define SPI_CR1_BIDIOE 14
#define SPI_CR1_BIDIMODE 15
/*
 *Bit definition for SPI_CR2 register
*/
#define SPI_CR2_RXDMAEN 0
#define SPI_CR2_TXDMAEN 1
#define SPI_CR2_SSOE 2
#define SPI_CR2_FRF 4
#define SPI_CR2_ERRIE 5
#define SPI_CR2_RXNEIE 6
#define SPI_CR2_TXEIE 7
/*
 *Bit definition for SPI_SR register
*/
#define SPI_SR_RXNE 0
#define SPI_SR_TXE 1
#define SPI_SR_CHSIDE 2
#define SPI_SR_UDR 3
#define SPI_SR_CRCERR 4
#define SPI_SR_MODF 5
#define SPI_SR_OVR 6
#define SPI_SR_BSY 7
#define SPI_SR_FRE 8
/*
 *I2C Struct register 
*/
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
}I2C_RegDef_t;

#define I2C1 ((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3 ((I2C_RegDef_t *)I2C3_BASEADDR)
/*
 * Bit Definition for I2C_CR1 register
*/
#define I2C_CR1_PE 0
#define I2C_CR1_NOSTRETCH 7
#define I2C_CR1_START 8
#define I2C_CR1_STOP 9
#define I2C_CR1_ACK 10
#define I2C_CR1_SWRST 15
/*
 * Bit Definition for I2C_CR2 register
*/
#define I2C_CR2_FREQ 0
#define I2C_CR2_ITERREN 8
#define I2C_CR2_ITEVTEN 9
#define I2C_CR2_ITBUFEN 10
/*
 * Bit Definition for I2C_OAR1 register
*/
#define I2C_OAR1_ADD0 0
#define I2C_OAR1_ADD71 1
#define I2C_OAR1_ADD98 8
#define I2C_OAR1_ADDMODE 15
/*
 * Bit Definition for I2C_SR1 register
*/
#define I2C_SR1_SB 0
#define I2C_SR1_ADDR 1
#define I2C_SR1_BTF 2
#define I2C_SR1_ADD10 3
#define I2C_SR1_STOPF 4
#define I2C_SR1_RXNE 6
#define I2C_SR1_TXE 7
#define I2C_SR1_BERR 8
#define I2C_SR1_ARL0 9
#define I2C_SR1_AF 10
#define I2C_SR1_OVR 11
#define I2C_SR1_TIMOUT 14
/*
 * Bit Definition for I2C_SR2 register
*/
#define I2C_SR2_MSL 0
#define I2C_SR2_BUSY 1
#define I2C_SR2_TRA 2
#define I2C_SR2_GENCALL 4
#define I2C_SR2_DUALF 7

#define I2C_CCR_CCR 0
#define I2C_CCR_DUTY 14
#define I2C_CCR_FS 15

#endif
