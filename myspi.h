#ifndef _MYSPI_H_
#define _MYSPI_H_

#include <stdint.h>
#include "header.h"

/*
 *Bit definition for SPI_CR2 register
*/
typedef struct
{
	uint8_t SPI_DeviceMode;     /*!< Master/Slave Mode,                     */
	uint8_t SPI_BusConfig;      /*!< Full duplex, Half duplex, simplex,     */
	uint8_t SPI_SclkSpeed;      /*!< Clock Speed,                           */
	uint8_t SPI_DFF;            /*!< Data frame 8 bit or 16 bit,            */
	uint8_t CPOL;               /*!<                                        */
	uint8_t CPHA;               /*!<                                        */
	uint8_t SSM;                /*!<                                        */
}SPI_Config_t;                

/*
 *Bit definition for SPI_CR2 register
*/
typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPICongif;
	uint8_t *pTxBuffer;         /*!< To store the Tx buffer address          */
	uint8_t *pRxBuffer;         /*!< To stire the Rx buffer address          */
	uint32_t TxLen;             /*!< To store the Tx len                     */
	uint32_t RxLen;             /*!< To store the Rx len                     */
	uint8_t TxState;
	uint8_t RxState;
}SPI_Handler_t;

/*
 *SPI Application events
*/
#define SPI_EVENT_TX_CMPLT 1
#define SPI_EVENT_RX_CMPLT 2
#define SPI_EVENT_OVR_ERR  3
#define SPI_EVENT_CRC_ERR  4
/*
 *SPI State
*/
#define SPI_READY 0
#define SPI_BUSY_IN_RX 1
#define SPI_BUSY_IN_TX 2
/*
 *SPI device mode
*/
#define SPI_DEVICE_MODE_MASTER 1
#define SPI_DEVICE_MODE_SLAVE 0
/*
 *SPI bus config
*/
#define SPI_BUS_CONFIG_FD 1
#define SPI_BUS_CONFIG_HD 2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 3
/*
 *SPI clock speed divide
*/
#define SPI_CLOCK_SPEED_DIV2 0
#define SPI_CLOCK_SPEED_DIV4 1
#define SPI_CLOCK_SPEED_DIV8 2
#define SPI_CLOCK_CPEED_DIV16 3
#define SPI_CLOCK_SPEED_DIV32 4
#define SPI_CLOCK_SPEED_DIV64 5
#define SPI_CLOCK_SPEED_DIV128 6
#define SPI_CLOCK_SPEED_DIV256 7
/*
 *SPI data frame
*/
#define SPI_DATA_FRAME_8BITS 0
#define SPI_DATA_FRAME_16BITS 1
/*
 *SPI CPOL 
*/
#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW 0
/*
 *SPI CPHA
*/
#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW 0

#define SPI_SSM_EN 1
#define SPI_SSM_DI 0

#define SPI_TXE_FLAG (1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG (1<<SPI_SR_RXNE)
#define SPI_BUSY_FLAG (1<<SPI_SR_BUSY)

#define FLAG_SET 1
#define FLAG_RESET 0
/*
 *Prototype
*/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_Init(SPI_Handler_t *pSPIHandle);
uint8_t SPI_GetFlagStatus( SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_SendData( SPI_RegDef_t *pSPIx, uint8_t* pTxBuffer, uint32_t Len);
uint16_t SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig( SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_IRQIntercruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
uint8_t  SPI_SendDataIT( SPI_Handler_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t  SPI_ReceiveDataIT( SPI_Handler_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);
void SPI_IRQHandling(SPI_Handler_t *pHandle);
void SPI_CloseTransmission(SPI_Handler_t *pHandle);
void SPI_CloseReception(SPI_Handler_t *pHandle);
/*
 * Application Callback
 */
 void SPI_ApplicationEventCallback(SPI_Handler_t *pSPIHandle, uint8_t AppEvent);
#endif
