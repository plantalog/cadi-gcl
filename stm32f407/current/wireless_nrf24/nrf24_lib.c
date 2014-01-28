// #include "stm32f4xx_conf.h"
#include "nrf24_lib.h"

uint8_t aTxBuffer[NRF24_DMA_BUFFERSIZE] = "SPI Master/Slave : Communication between two SPI using Interrupts";
__IO uint8_t aRxBuffer [NRF24_DMA_BUFFERSIZE];
__IO uint8_t ubRxIndex = 0;
__IO uint8_t ubTxIndex = 0;
__IO uint32_t TimeOut = 0;

unsigned char nrfRxBuff[NRF24_RX_BUFF_SIZE];
unsigned char nrfTxBuff[NRF24_TX_BUFF_SIZE];
uint8_t nrf_rx_pointer=0, nrf_tx_pointer=0;

// copy-paste from ST.com example with Interrupts

void nrf24init(void){
	  GPIO_InitTypeDef GPIO_InitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
	  SPI_InitTypeDef  SPI_InitStructure;

	  /* Peripheral Clock Enable -------------------------------------------------*/
	  /* Enable the SPI clock */
	  NRF24_SPIx_CLK_INIT(NRF24_SPIx_CLK, ENABLE);

	  /* Enable GPIO clocks */
	  RCC_AHB1PeriphClockCmd(NRF24_SPIx_SCK_GPIO_CLK | NRF24_SPIx_MISO_GPIO_CLK | NRF24_SPIx_MOSI_GPIO_CLK, ENABLE);

	  RCC_AHB1PeriphClockCmd(NRF24_CEN_GPIO_CLK, ENABLE);               

	  /* Configure the NRF interrupt pin */
	  GPIO_InitStructure.GPIO_Pin = NRF24_IRQ_PIN;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	// GPIO_Mode_AF ??
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;             
	  GPIO_Init(NRF24_IRQ_PORT, &GPIO_InitStructure);
	  
	  /* Configure the  NRF Chip Enable (pin 3) pin */
	  GPIO_InitStructure.GPIO_Pin = NRF24_CEN_PIN;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                
	  GPIO_Init(NRF24_CEN_PORT, &GPIO_InitStructure);
	  
	  /* Configure NRF Chip Select (pin 4) pin */
	  GPIO_InitStructure.GPIO_Pin = NRF24_CSN_PIN;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                
	  GPIO_Init(NRF24_CSN_PORT, &GPIO_InitStructure);
	  
	  /* SPI GPIO Configuration --------------------------------------------------*/
	  /* GPIO Deinitialisation */
	  GPIO_DeInit(NRF24_SPIx_SCK_GPIO_PORT);
	  GPIO_DeInit(NRF24_SPIx_MISO_GPIO_PORT);
	  GPIO_DeInit(NRF24_SPIx_MOSI_GPIO_PORT);

	  // Connect SPI pins to AF5 (CHECK IF NEEDED)
	  GPIO_PinAFConfig(NRF24_SPIx_SCK_GPIO_PORT, NRF24_SPIx_SCK_SOURCE, NRF24_SPIx_SCK_AF);
	  GPIO_PinAFConfig(NRF24_SPIx_MISO_GPIO_PORT, NRF24_SPIx_MISO_SOURCE, NRF24_SPIx_MISO_AF);
	  GPIO_PinAFConfig(NRF24_SPIx_MOSI_GPIO_PORT, NRF24_SPIx_MOSI_SOURCE, NRF24_SPIx_MOSI_AF);

	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

	  /* SPI SCK pin configuration */
	  GPIO_InitStructure.GPIO_Pin = NRF24_SPIx_SCK_PIN;
	  GPIO_Init(NRF24_SPIx_SCK_GPIO_PORT, &GPIO_InitStructure);

	  /* SPI  MISO pin configuration */
	  GPIO_InitStructure.GPIO_Pin =  NRF24_SPIx_MISO_PIN;
	  GPIO_Init(NRF24_SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);

	  /* SPI  MOSI pin configuration */
	  GPIO_InitStructure.GPIO_Pin =  NRF24_SPIx_MOSI_PIN;
	  GPIO_Init(NRF24_SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);
	  

	  /* SPI configuration -------------------------------------------------------*/
	  SPI_I2S_DeInit(NRF24_SPIx);
	  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	  SPI_InitStructure.SPI_CRCPolynomial = 7;
	  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//	  NRF24_CEN_LOW;
//	  NRF24_CSN_LOW;
	  SPI_Init(NRF24_SPIx, &SPI_InitStructure);
	  	//  SPI1->CR2 |= SPI_CR2_SSOE;	
	  	  
	  	  SPI1->CR1    |= SPI_CR1_SSI;       // nss HIGH
	  	  SPI1->CR1    |= SPI_CR1_SSM;       // enable soft nss
	  NRF24_SPIx->CR2 |= (uint16_t)SPI_CR2_SSOE; // set nss pin as output
	  SPI1->CR2 |= SPI_CR2_SSOE;	
//	  NRF24_SPIx->CR1 |= SPI_NSSInternalSoft_Set;
	  
	  /* Configure the Priority Group to 1 bit */
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	  /* Configure the SPI interrupt priority */
	  NVIC_InitStructure.NVIC_IRQChannel = NRF24_SPIx_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
	  
	  SPI_Cmd(NRF24_SPIx, ENABLE);
	  
	  /* Initialize Buffer counters */
	  ubTxIndex = 0;
	  ubRxIndex = 0;
	  
	  /* Enable the Rx buffer not empty interrupt */
	  SPI_I2S_ITConfig(NRF24_SPIx, SPI_I2S_IT_RXNE, ENABLE);
	  
	  /* Enable the Tx buffer empty interrupt */
	  SPI_I2S_ITConfig(NRF24_SPIx, SPI_I2S_IT_TXE, ENABLE);
	  // set CSN to HIGH
	  NRF24_CSN_HIGH;
 }

// copy-paste from ST.com examples with DMA
void nrf24init_dma(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	  DMA_InitTypeDef DMA_InitStructure;

	  SPI_InitTypeDef  SPI_InitStructure;

	  /* Peripheral Clock Enable -------------------------------------------------*/
	  /* Enable the SPI clock */
	  SPIx_CLK_INIT(NRF24_SPIx_CLK, ENABLE);

	  /* Enable GPIO clocks */
	  RCC_AHB1PeriphClockCmd(NRF24_SPIx_SCK_GPIO_CLK | NRF24_SPIx_MISO_GPIO_CLK | NRF24_SPIx_MOSI_GPIO_CLK, ENABLE);

	  /* Enable DMA clock */
	  RCC_AHB1PeriphClockCmd(NRF24_SPIx_DMA_CLK, ENABLE);

	  /* SPI GPIO Configuration --------------------------------------------------*/
	  /* GPIO Deinitialisation */
	  GPIO_DeInit(NRF24_SPIx_SCK_GPIO_PORT);
	  GPIO_DeInit(NRF24_SPIx_MISO_GPIO_PORT);
	  GPIO_DeInit(NRF24_SPIx_MOSI_GPIO_PORT);

	  /* Connect SPI pins to AF5 */
	  GPIO_PinAFConfig(NRF24_SPIx_SCK_GPIO_PORT, NRF24_SPIx_SCK_SOURCE, NRF24_SPIx_SCK_AF);
	  GPIO_PinAFConfig(NRF24_SPIx_MISO_GPIO_PORT, NRF24_SPIx_MISO_SOURCE, NRF24_SPIx_MISO_AF);
	  GPIO_PinAFConfig(NRF24_SPIx_MOSI_GPIO_PORT, NRF24_SPIx_MOSI_SOURCE, NRF24_SPIx_MOSI_AF);

	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

	  /* SPI SCK pin configuration */
	  GPIO_InitStructure.GPIO_Pin = NRF24_SPIx_SCK_PIN;
	  GPIO_Init(NRF24_SPIx_SCK_GPIO_PORT, &GPIO_InitStructure);

	  /* SPI  MISO pin configuration */
	  GPIO_InitStructure.GPIO_Pin =  NRF24_SPIx_MISO_PIN;
	  GPIO_Init(NRF24_SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);

	  /* SPI  MOSI pin configuration */
	  GPIO_InitStructure.GPIO_Pin =  NRF24_SPIx_MOSI_PIN;
	  GPIO_Init(NRF24_SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);

	  /* SPI configuration -------------------------------------------------------*/
	  SPI_I2S_DeInit(NRF24_SPIx);
	  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	  SPI_InitStructure.SPI_CRCPolynomial = 7;
	  
	  NRF24_SPIx->CR2 |= (uint16_t)SPI_CR2_SSOE; // set nss pin as output
	  SPI1->CR2 |= SPI_CR2_SSOE;	
	  NRF24_SPIx->CR1 |= SPI_NSSInternalSoft_Set;
	  /* DMA configuration -------------------------------------------------------*/
	  /* Deinitialize DMA Streams */
	  DMA_DeInit(NRF24_SPIx_TX_DMA_STREAM);
	  DMA_DeInit(NRF24_SPIx_RX_DMA_STREAM);

	  

	  /* Configure DMA Initialization Structure */
	  DMA_InitStructure.DMA_BufferSize = NRF24_DMA_BUFFERSIZE;
	  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(NRF24_SPIx->DR));
	  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	  /* Configure TX DMA */
	  DMA_InitStructure.DMA_Channel = NRF24_SPIx_TX_DMA_CHANNEL;
	  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)aTxBuffer;
	  DMA_Init(NRF24_SPIx_TX_DMA_STREAM, &DMA_InitStructure);
	  /* Configure RX DMA */
	  DMA_InitStructure.DMA_Channel = NRF24_SPIx_RX_DMA_CHANNEL;
	  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)aRxBuffer;
	  DMA_Init(NRF24_SPIx_RX_DMA_STREAM, &DMA_InitStructure);

}


// copy-paste from SPI sd card init
static void nrf24init2(void)
{
/*	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable SPI & GPIO clocks 
	SPI_GPIO_CLK(ENABLE);
	SPI_PERIF_CLK(ENABLE);

#ifdef USE_PWRCTRL
	// Configure I/O for Power FET 
	SPIMMC_PWR_CLK(ENABLE);

	GPIO_InitStructure.GPIO_Pin		= SPIMMC_PIN_PWR;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_OD; // pull-up resistor at power FET
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_Init(GPIO_PWR, &GPIO_InitStructure);
	PWR_ON();
#endif

	//for (Timer1 = 25; Timer1; );	// Wait for 250ms
	Delay_mS(250);

	// Configure I/O for Flash Chip select 
	GPIO_InitStructure.GPIO_Pin		= SPIMMC_PIN_CS;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_Init(SPIMMC_PORT_CS, &GPIO_InitStructure);

	// Deselect the Card: Chip Select high 
	CS_HIGH();


	// Connect SPI pins to AF 
	SPI_AF_SET();
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

	// SPI SCK pin configuration
	GPIO_InitStructure.GPIO_Pin = SPIMMC_PIN_SCK;
	GPIO_Init(SPIMMC_PORT_SCK, &GPIO_InitStructure);

	// SPI MOSI pin configuration
	GPIO_InitStructure.GPIO_Pin =  SPIMMC_PIN_MOSI;
	GPIO_Init(SPIMMC_PORT_MOSI, &GPIO_InitStructure);

	// SPI MISO pin configuration
	GPIO_InitStructure.GPIO_Pin =  SPIMMC_PIN_MISO;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(SPIMMC_PORT_MISO, &GPIO_InitStructure);


	// SPI_MMC configuration
	SPI_InitStructure.SPI_Direction	 		= SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode		 		= SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize	 		= SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL		 		= SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA		 		= SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS		 		= SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // 84000kHz /256 = 328 < 400kHz
	SPI_InitStructure.SPI_FirstBit	 		= SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial 	= 7;
	SPI_Init(SPIMMC, &SPI_InitStructure);


	SPI_CalculateCRC(SPIMMC, DISABLE);

	// Enable SPIx 
	//SPIMMC->CR1 |= SPI_CR1_SPE;
	SPI_Cmd(SPIMMC, ENABLE);

	// drain SPI
	//while (!(SPIMMC->SR & SPI_I2S_FLAG_TXE)) { ; }
	while(SPI_I2S_GetFlagStatus(SPIMMC, SPI_I2S_FLAG_TXE) == RESET);
	SPIMMC->DR;

#ifdef USE_SPIMMC_DMA
	// Enable DMA clock
	RCC_AHB1PeriphClockCmd(SPIMMC_DMA_STREAM_CLOCK, ENABLE);

 	// Disable SPI_DMA_TX/RX
    SPIMMC_DMA_STREAM_RX->CR &= ~(uint32_t)DMA_SxCR_EN;
    SPIMMC_DMA_STREAM_TX->CR &= ~(uint32_t)DMA_SxCR_EN;
	DMA_DeInit(SPIMMC_DMA_STREAM_RX);
	DMA_DeInit(SPIMMC_DMA_STREAM_TX);

	// shared DMA configuration values
	DMA_InitStructure.DMA_Channel 				= SPIMMC_DMA_CHANNEL_RX;
	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)(&(SPIMMC->DR));
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority 				= DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst 			= DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;

#endif

*/
}

// https://github.com/wendlers/libemb/blob/master/libnrf24l01/src/nrf24l01.c
/*
* This file is part of the libemb project.
*
* Copyright (C) 2011 Stefan Wendler <sw@kaltpost.de>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/



int nrf_read_reg(unsigned char reg, nrf_reg_buf *buf)
{
     int i;

     // get register payload size
     unsigned char s = nrf_reg_def.data[reg].size;

     nrf_spi_csl();

     // send command
     nrf_spi_xfer_byte(NRF_CMD_RREG | reg);

     // receive response
     for(i = 0; i < s; i++) {
          buf->data[i] = nrf_spi_xfer_byte(NRF_CMD_NOP);
     }

     nrf_spi_csh();

     buf->size = i;

     return i;
}

int nrf_write_reg(unsigned char reg, nrf_reg_buf *buf)
{
     int i;

     // get register payload size
     unsigned char s = nrf_reg_def.data[reg].size;

     nrf_spi_csl();

     // send command
     nrf_spi_xfer_byte(NRF_CMD_WREG | reg);

     // send payload
     for(i = 0; i < s; i++) {
          nrf_spi_xfer_byte(buf->data[i]);
     }

     nrf_spi_csh();

     return i;
}

int nrf_send(nrf_payload *payload)
{
     int i;

     nrf_reg_buf status;
     nrf_read_reg(NRF_REG_STATUS, &status);

     // if TX buffer is full, indicate no bytes where sent, flush TX buffer
     if(nrf_get_reg_field(NRF_REG_STATUS, NRF_REGF_TX_FULL, &status) == 1) {
          nrf_spi_csl();
          nrf_spi_xfer_byte(NRF_CMD_FLUSH_TX);
          nrf_spi_csh();
          return NRF_ERR_TX_FULL;
     }

     // send command
     nrf_spi_csl();
     nrf_spi_xfer_byte(NRF_CMD_TX);

     // send payload
     for(i = 0; i < payload->size; i++) {
          nrf_spi_xfer_byte(payload->data[i]);
     }

     nrf_spi_csh();

     return i;
}

int nrf_send_blocking(nrf_payload *payload)
{
     int i;

     nrf_reg_buf status;
     nrf_read_reg(NRF_REG_STATUS, &status);

     // if TX buffer is full, indicate no bytes where sent, flush TX buffer
     if(nrf_get_reg_field(NRF_REG_STATUS, NRF_REGF_TX_FULL, &status) == 1) {
          nrf_spi_csl();
          nrf_spi_xfer_byte(NRF_CMD_FLUSH_TX);
          nrf_spi_csh();
          i = NRF_ERR_TX_FULL;
     }

     // send command
     nrf_spi_csl();
     nrf_spi_xfer_byte(NRF_CMD_TX);

     // send payload
     for(i = 0; i < payload->size; i++) {
          nrf_spi_xfer_byte(payload->data[i]);
     }
     nrf_spi_csh();

     // wait until payload passed TX FIFO
     do {
          nrf_read_reg(NRF_REG_STATUS, &status);

          // If MAX_RT is reached, indicate no bytes where sent ...
          if(nrf_get_reg_field(NRF_REG_STATUS, NRF_REGF_MAX_RT, &status) == 1) {
               i = NRF_ERR_MAX_RT;
               break;
          }
     } while(nrf_get_reg_field(NRF_REG_STATUS, NRF_REGF_TX_DS , &status) != 1);

     // clear TX_DS/MAX_RT bits (by writing 1 or just writing back the status)
     nrf_write_reg(NRF_REG_STATUS, &status);

     return i;
}


int nrf_receive(nrf_payload *payload)
{
     unsigned char i = 0;

     nrf_reg_buf status;

     nrf_read_reg(NRF_REG_STATUS, &status);

     // receive payload
     if(nrf_get_reg_field(NRF_REG_STATUS, NRF_REGF_RX_DR, &status) == 1) {

          nrf_spi_csl();
          nrf_spi_xfer_byte(NRF_CMD_RX);

          for(i = 0; i < payload->size; i++) {
               payload->data[i] = nrf_spi_xfer_byte(NRF_CMD_RREG);
          }

          nrf_spi_csh();

          // write back status to clean RX_DR
          nrf_write_reg(NRF_REG_STATUS, &status);
     }

     nrf_read_reg(NRF_REG_FIFO_STATUS, &status);

     // if RX buffer is full, indicate no bytes where receifed, flush RX buffer
     if(nrf_get_reg_field(NRF_REG_FIFO_STATUS, NRF_REGF_FIFO_RX_FULL, &status) == 1) {
          nrf_spi_csl();
          nrf_spi_xfer_byte(NRF_CMD_FLUSH_RX);
          nrf_spi_csh();
          return NRF_ERR_RX_FULL;
     }

     return i;
}

int nrf_receive_blocking(nrf_payload *payload)
{
     unsigned char i = 0;

     nrf_reg_buf status;

     // wait until data arrives
     do {
          nrf_read_reg(NRF_REG_STATUS, &status);
     } while(nrf_get_reg_field(NRF_REG_STATUS, NRF_REGF_RX_DR, &status) != 1);

     // receive payload
     nrf_spi_csl();
     nrf_spi_xfer_byte(NRF_CMD_RX);

     for(i = 0; i < payload->size; i++) {
          payload->data[i] = nrf_spi_xfer_byte(NRF_CMD_RREG);
     }

     nrf_spi_csh();

     // write back status to clean RX_DR
     nrf_write_reg(NRF_REG_STATUS, &status);

     nrf_read_reg(NRF_REG_FIFO_STATUS, &status);

     // if RX buffer is full, indicate no bytes where receifed, flush RX buffer
     if(nrf_get_reg_field(NRF_REG_FIFO_STATUS, NRF_REGF_FIFO_RX_FULL, &status) == 1) {
          nrf_spi_csl();
          nrf_spi_xfer_byte(NRF_CMD_FLUSH_RX);
          nrf_spi_csh();
          return NRF_ERR_RX_FULL;
     }

     return i;
}

int nrf_write_ack_pl(nrf_payload *payload, unsigned char pipe)
{
     int i;

     nrf_reg_buf status;
     nrf_read_reg(NRF_REG_STATUS, &status);

     // if TX buffer is full, indicate no bytes where sent, flush TX buffer
     if(nrf_get_reg_field(NRF_REG_STATUS, NRF_REGF_TX_FULL, &status) == 1) {
          nrf_spi_csl();
          nrf_spi_xfer_byte(NRF_CMD_FLUSH_TX);
          nrf_spi_csh();
          return NRF_ERR_TX_FULL;
     }

     nrf_spi_csl();
     nrf_spi_xfer_byte(NRF_CMD_WACKPL | (0b00000111 & pipe));

     for(i = 0; i < payload->size; i++) {
          nrf_spi_xfer_byte(payload->data[i]);
     }

     nrf_spi_csh();

     return i;
}

int nrf_read_ack_pl(nrf_payload *payload)
{
     unsigned char i = 0;

     nrf_reg_buf status;
     nrf_read_reg(NRF_REG_FIFO_STATUS, &status);

     // if RX buffer is full, indicate no bytes where receifed, flush RX buffer
     if(nrf_get_reg_field(NRF_REG_FIFO_STATUS, NRF_REGF_FIFO_RX_FULL, &status) == 1) {
          nrf_spi_csl();
          nrf_spi_xfer_byte(NRF_CMD_FLUSH_RX);
          nrf_spi_csh();
          return NRF_ERR_RX_FULL;
     }

     nrf_spi_csl();
     nrf_spi_xfer_byte(NRF_CMD_RX);

     for(i = 0; i < payload->size; i++) {
          payload->data[i] = nrf_spi_xfer_byte(NRF_CMD_RREG);
     }

     nrf_spi_csh();

     return i;
}

void nrf_preset_sb(unsigned char mode, unsigned char rf_ch, unsigned char pw, nrf_reg_buf *addr)
{

     nrf_reg_buf buf;

     // Disable auto ACK on all pipes
     nrf_read_reg(NRF_REG_EN_AA, &buf);
     nrf_set_reg_field(NRF_REG_EN_AA, NRF_REGF_ENAA_P0, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_AA, NRF_REGF_ENAA_P1, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_AA, NRF_REGF_ENAA_P2, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_AA, NRF_REGF_ENAA_P3, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_AA, NRF_REGF_ENAA_P4, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_AA, NRF_REGF_ENAA_P5, &buf, 0);
     nrf_write_reg(NRF_REG_EN_AA, &buf);

     // Disable RX addresses, except PIPE0
     nrf_read_reg(NRF_REG_EN_RXADDR, &buf);
     nrf_set_reg_field(NRF_REG_EN_RXADDR, NRF_REGF_ERX_P0, &buf, 1);
     nrf_set_reg_field(NRF_REG_EN_RXADDR, NRF_REGF_ERX_P1, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_RXADDR, NRF_REGF_ERX_P2, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_RXADDR, NRF_REGF_ERX_P3, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_RXADDR, NRF_REGF_ERX_P4, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_RXADDR, NRF_REGF_ERX_P5, &buf, 0);
     nrf_write_reg(NRF_REG_EN_RXADDR, &buf);

     // CONFIG - CRC enable, 2-Bit CRC, RX/TX mode, disable MAX_RT_IRQ + TX_DS/RX_DR
     nrf_read_reg(NRF_REG_CONFIG, &buf);
     nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_EN_CRC, &buf, 1);
     nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_CRCO, &buf, 1);
     nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_PRIM_RX, &buf, mode);
     nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_MASK_MAX_RT, &buf, 1);

     if(mode == NRF_MODE_PRX) {
          nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_MASK_TX_DS, &buf, 1);
     } else {
          nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_MASK_RX_DR, &buf, 1);
     }

     nrf_write_reg(NRF_REG_CONFIG, &buf);

     // Disable auto retry
     nrf_read_reg(NRF_REG_SETUP_RETR, &buf);
     nrf_set_reg_field(NRF_REG_SETUP_RETR, NRF_REGF_ARC, &buf, 0);
     nrf_set_reg_field(NRF_REG_SETUP_RETR, NRF_REGF_ARD, &buf, 0);
     nrf_write_reg(NRF_REG_SETUP_RETR, &buf);

     // Set address width to 5 bytes
     nrf_read_reg(NRF_REG_SETUP_AW, &buf);
     nrf_set_reg_field(NRF_REG_SETUP_AW, NRF_REGF_AW, &buf, 0b11);
     nrf_write_reg(NRF_REG_SETUP_AW, &buf);

     // RX_ADDR_P0 - set receive address data pipe0
     nrf_write_reg(NRF_REG_RX_ADDR_P0, addr);

     // TX_ADDR - transmit address
     nrf_write_reg(NRF_REG_TX_ADDR, addr);

     if(mode == NRF_MODE_PRX) {
          // RX_PW_P0 - set number of bytes in RX payload in data PIPE0
          nrf_read_reg(NRF_REG_RX_PW_P0, &buf);
          nrf_set_reg_field(NRF_REG_RX_PW_P0, NRF_REGF_PW, &buf, pw);
          nrf_write_reg(NRF_REG_RX_PW_P0, &buf);
     }

     // Set RF-channel
     nrf_read_reg(NRF_REG_RF_CH, &buf);
     nrf_set_reg_field(NRF_REG_RF_CH, NRF_REGF_RF_CH, &buf, rf_ch);
     nrf_write_reg(NRF_REG_RF_CH, &buf);
        
         // Setup Data-Rate to 1MBit and RF power to 0db
     nrf_read_reg(NRF_REG_RF_SETUP, &buf);
     nrf_set_reg_field(NRF_REG_RF_SETUP, NRF_REGF_RF_DR , &buf, 0);
     nrf_set_reg_field(NRF_REG_RF_SETUP, NRF_REGF_RF_PWR, &buf, 3);
     nrf_write_reg(NRF_REG_RF_SETUP, &buf);


     // Power up radio
     nrf_read_reg(NRF_REG_CONFIG, &buf);
     nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_PWR_UP, &buf, 1);
     nrf_write_reg(NRF_REG_CONFIG, &buf);
}

void nrf_preset_esb(
     unsigned char mode, unsigned char rf_ch, unsigned char pw,
     unsigned char retr, unsigned char delay, nrf_reg_buf *addr)
{

     nrf_reg_buf buf;

     // Disable auto ACK on all pipes, except PIPE0
     nrf_read_reg(NRF_REG_EN_AA, &buf);
     nrf_set_reg_field(NRF_REG_EN_AA, NRF_REGF_ENAA_P0, &buf, 1);
     nrf_set_reg_field(NRF_REG_EN_AA, NRF_REGF_ENAA_P1, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_AA, NRF_REGF_ENAA_P2, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_AA, NRF_REGF_ENAA_P3, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_AA, NRF_REGF_ENAA_P4, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_AA, NRF_REGF_ENAA_P5, &buf, 0);
     nrf_write_reg(NRF_REG_EN_AA, &buf);

     // Disable RX addresses, except PIPE0
     nrf_read_reg(NRF_REG_EN_RXADDR, &buf);
     nrf_set_reg_field(NRF_REG_EN_RXADDR, NRF_REGF_ERX_P0, &buf, 1);
     nrf_set_reg_field(NRF_REG_EN_RXADDR, NRF_REGF_ERX_P1, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_RXADDR, NRF_REGF_ERX_P2, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_RXADDR, NRF_REGF_ERX_P3, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_RXADDR, NRF_REGF_ERX_P4, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_RXADDR, NRF_REGF_ERX_P5, &buf, 0);
     nrf_write_reg(NRF_REG_EN_RXADDR, &buf);

     // CONFIG - CRC enable, 2-Bit CRC, RX/TX mode
     nrf_read_reg(NRF_REG_CONFIG, &buf);
     nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_EN_CRC, &buf, 1);
     nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_CRCO, &buf, 1);
     nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_PRIM_RX, &buf, mode);
     nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_MASK_MAX_RT, &buf, 1);

     if(mode == NRF_MODE_PRX) {
          nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_MASK_TX_DS, &buf, 1);
     } else {
          nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_MASK_RX_DR, &buf, 1);
     }

     nrf_write_reg(NRF_REG_CONFIG, &buf);

     // Enable auto retry and delay
     nrf_read_reg(NRF_REG_SETUP_RETR, &buf);
     nrf_set_reg_field(NRF_REG_SETUP_RETR, NRF_REGF_ARC, &buf, retr);
     nrf_set_reg_field(NRF_REG_SETUP_RETR, NRF_REGF_ARD, &buf, delay);
     nrf_write_reg(NRF_REG_SETUP_RETR, &buf);

     // Set address width to 5 bytes
     nrf_read_reg(NRF_REG_SETUP_AW, &buf);
     nrf_set_reg_field(NRF_REG_SETUP_AW, NRF_REGF_AW, &buf, 0b11);
     nrf_write_reg(NRF_REG_SETUP_AW, &buf);

     // RX_ADDR_P0 - set receive address data pipe0
     nrf_write_reg(NRF_REG_RX_ADDR_P0, addr);

     // TX_ADDR - transmit address
     nrf_write_reg(NRF_REG_TX_ADDR, addr);

     if(mode == NRF_MODE_PRX) {
          // RX_PW_P0 - set number of bytes in RX payload in data PIPE0
          nrf_read_reg(NRF_REG_RX_PW_P0, &buf);
          nrf_set_reg_field(NRF_REG_RX_PW_P0, NRF_REGF_PW, &buf, pw);
          nrf_write_reg(NRF_REG_RX_PW_P0, &buf);
     }

     // Set RF-channel
     nrf_read_reg(NRF_REG_RF_CH, &buf);
     nrf_set_reg_field(NRF_REG_RF_CH, NRF_REGF_RF_CH, &buf, rf_ch);
     nrf_write_reg(NRF_REG_RF_CH, &buf);
    
     // Setup Data-Rate to 1MBit and RF power to 0db
     nrf_read_reg(NRF_REG_RF_SETUP, &buf);
     nrf_set_reg_field(NRF_REG_RF_SETUP, NRF_REGF_RF_DR , &buf, 0);
     nrf_set_reg_field(NRF_REG_RF_SETUP, NRF_REGF_RF_PWR, &buf, 3);
     nrf_write_reg(NRF_REG_RF_SETUP, &buf);


     // Power up radio
     nrf_read_reg(NRF_REG_CONFIG, &buf);
     nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_PWR_UP, &buf, 1);
     nrf_write_reg(NRF_REG_CONFIG, &buf);
}

void nrf_preset_esbpl(
     unsigned char mode, unsigned char rf_ch, unsigned char pw,
     unsigned char retr, unsigned char delay, nrf_reg_buf *addr)
{

     nrf_reg_buf buf;

     // Disable auto ACK on all pipes, except PIPE0
     nrf_read_reg(NRF_REG_EN_AA, &buf);
     nrf_set_reg_field(NRF_REG_EN_AA, NRF_REGF_ENAA_P0, &buf, 1);
     nrf_set_reg_field(NRF_REG_EN_AA, NRF_REGF_ENAA_P1, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_AA, NRF_REGF_ENAA_P2, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_AA, NRF_REGF_ENAA_P3, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_AA, NRF_REGF_ENAA_P4, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_AA, NRF_REGF_ENAA_P5, &buf, 0);
     nrf_write_reg(NRF_REG_EN_AA, &buf);

     // Disable RX addresses, except PIPE0
     nrf_read_reg(NRF_REG_EN_RXADDR, &buf);
     nrf_set_reg_field(NRF_REG_EN_RXADDR, NRF_REGF_ERX_P0, &buf, 1);
     nrf_set_reg_field(NRF_REG_EN_RXADDR, NRF_REGF_ERX_P1, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_RXADDR, NRF_REGF_ERX_P2, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_RXADDR, NRF_REGF_ERX_P3, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_RXADDR, NRF_REGF_ERX_P4, &buf, 0);
     nrf_set_reg_field(NRF_REG_EN_RXADDR, NRF_REGF_ERX_P5, &buf, 0);
     nrf_write_reg(NRF_REG_EN_RXADDR, &buf);

     // CONFIG - CRC enable, 2-Bit CRC, RX/TX mode
     nrf_read_reg(NRF_REG_CONFIG, &buf);
     nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_EN_CRC, &buf, 1);
     nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_CRCO, &buf, 1);
     nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_PRIM_RX, &buf, mode);
     nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_MASK_MAX_RT, &buf, 1);

     if(mode == NRF_MODE_PRX) {
          nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_MASK_TX_DS, &buf, 1);
     } else {
          nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_MASK_RX_DR, &buf, 1);
     }

     nrf_write_reg(NRF_REG_CONFIG, &buf);

     // Enable auto retry and delay
     nrf_read_reg(NRF_REG_SETUP_RETR, &buf);
     nrf_set_reg_field(NRF_REG_SETUP_RETR, NRF_REGF_ARC, &buf, retr);
     nrf_set_reg_field(NRF_REG_SETUP_RETR, NRF_REGF_ARD, &buf, delay);
     nrf_write_reg(NRF_REG_SETUP_RETR, &buf);

     // Set address width to 5 bytes
     nrf_read_reg(NRF_REG_SETUP_AW, &buf);
     nrf_set_reg_field(NRF_REG_SETUP_AW, NRF_REGF_AW, &buf, 0b11);
     nrf_write_reg(NRF_REG_SETUP_AW, &buf);

     // RX_ADDR_P0 - set receive address data pipe0
     nrf_write_reg(NRF_REG_RX_ADDR_P0, addr);

     // TX_ADDR - transmit address
     nrf_write_reg(NRF_REG_TX_ADDR, addr);

     // Set ACK PL + DYN PL
     nrf_read_reg(NRF_REG_FEATURE, &buf);
     nrf_set_reg_field(NRF_REG_FEATURE, NRF_REGF_EN_DPL, &buf, 1);
     nrf_set_reg_field(NRF_REG_FEATURE, NRF_REGF_EN_ACK_PAY, &buf, 1);
     nrf_write_reg(NRF_REG_FEATURE, &buf);

     // Enable dynamic payload width on PIPE0
     nrf_read_reg(NRF_REG_DYNPD, &buf);
     nrf_set_reg_field(NRF_REG_DYNPD, NRF_REGF_DPL_P0, &buf, 1);
     nrf_set_reg_field(NRF_REG_DYNPD, NRF_REGF_DPL_P1, &buf, 0);
     nrf_set_reg_field(NRF_REG_DYNPD, NRF_REGF_DPL_P2, &buf, 0);
     nrf_set_reg_field(NRF_REG_DYNPD, NRF_REGF_DPL_P3, &buf, 0);
     nrf_set_reg_field(NRF_REG_DYNPD, NRF_REGF_DPL_P4, &buf, 0);
     nrf_set_reg_field(NRF_REG_DYNPD, NRF_REGF_DPL_P5, &buf, 0);
     nrf_write_reg(NRF_REG_DYNPD, &buf);

     if(mode == NRF_MODE_PRX) {
          // RX_PW_P0 - set number of bytes in RX payload in data PIPE0
          nrf_read_reg(NRF_REG_RX_PW_P0, &buf);
          nrf_set_reg_field(NRF_REG_RX_PW_P0, NRF_REGF_PW, &buf, pw);
          nrf_write_reg(NRF_REG_RX_PW_P0, &buf);
     }

     // Set RF-channel
     nrf_read_reg(NRF_REG_RF_CH, &buf);
     nrf_set_reg_field(NRF_REG_RF_CH, NRF_REGF_RF_CH, &buf, rf_ch);
     nrf_write_reg(NRF_REG_RF_CH, &buf);

     // Setup Data-Rate to 1MBit and RF power to 0db
     nrf_read_reg(NRF_REG_RF_SETUP, &buf);
     nrf_set_reg_field(NRF_REG_RF_SETUP, NRF_REGF_RF_DR , &buf, 0);
     nrf_set_reg_field(NRF_REG_RF_SETUP, NRF_REGF_RF_PWR, &buf, 3);
     nrf_write_reg(NRF_REG_RF_SETUP, &buf);

     // Power up radio
     nrf_read_reg(NRF_REG_CONFIG, &buf);
     nrf_set_reg_field(NRF_REG_CONFIG, NRF_REGF_PWR_UP, &buf, 1);
     nrf_write_reg(NRF_REG_CONFIG, &buf);
}

unsigned char nrf_spi_xfer_byte(unsigned char data)
{
	NRF24_SPIx->DR = data;
     //return spi_xfer(SPI2, data);
}
