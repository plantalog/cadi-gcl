#include "stm32f4xx_conf.h"

#define NRF24_DMA_BUFFERSIZE			  	 128
#define NRF24_RX_BUFF_SIZE					 128
#define NRF24_TX_BUFF_SIZE					 128

#define NRF24_IRQ_PIN						 GPIO_Pin_5
#define NRF24_IRQ_PORT						 GPIOC
#define NRF24_CEN_PIN						 GPIO_Pin_4
#define NRF24_CEN_PORT						 GPIOC
#define NRF24_CSN_PIN						 GPIO_Pin_3		// nss
#define NRF24_CSN_PORT						 GPIOC
#define NRF24_CEN_GPIO_CLK         	    	 RCC_AHB1Periph_GPIOC

#define NRF24_CSN_HIGH						 NRF24_CSN_PORT->BSRRH |= NRF24_CSN_PIN
#define NRF24_CSN_LOW						 NRF24_CSN_PORT->BSRRL |= NRF24_CSN_PIN
#define NRF24_CEN_HIGH						 NRF24_CEN_PORT->BSRRH |= NRF24_CEN_PIN
#define NRF24_CEN_LOW						 NRF24_CEN_PORT->BSRRL |= NRF24_CEN_PIN

#define NRF24_SPIx_CLK_INIT                  RCC_APB2PeriphClockCmd
#define NRF24_SPIx_IRQn						 SPI1_IRQn
#define NRF24_SPIx                           SPI1
#define NRF24_SPIx_CLK                       RCC_APB2Periph_SPI1
#define NRF24_SPIx_IRQHANDLER                SPI1_IRQHandler

#define NRF24_SPIx_SCK_PIN                   GPIO_Pin_5
#define NRF24_SPIx_SCK_GPIO_PORT             GPIOA
#define NRF24_SPIx_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOA
#define NRF24_SPIx_SCK_SOURCE                GPIO_PinSource5
#define NRF24_SPIx_SCK_AF                    GPIO_AF_SPI1

#define NRF24_SPIx_MISO_PIN                  GPIO_Pin_6
#define NRF24_SPIx_MISO_GPIO_PORT            GPIOA
#define NRF24_SPIx_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define NRF24_SPIx_MISO_SOURCE               GPIO_PinSource6
#define NRF24_SPIx_MISO_AF                   GPIO_AF_SPI1

#define NRF24_SPIx_MOSI_PIN                  GPIO_Pin_7
#define NRF24_SPIx_MOSI_GPIO_PORT            GPIOA
#define NRF24_SPIx_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define NRF24_SPIx_MOSI_SOURCE               GPIO_PinSource7
#define NRF24_SPIx_MOSI_AF                   GPIO_AF_SPI1

#define NRF24_SPIx_DMA                       DMA1
#define NRF24_SPIx_DMA_CLK                   RCC_AHB1Periph_DMA1
#define NRF24_SPIx_TX_DMA_CHANNEL            DMA_Channel_0
#define NRF24_SPIx_TX_DMA_STREAM             DMA1_Stream4
#define NRF24_SPIx_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF4
#define NRF24_SPIx_RX_DMA_CHANNEL            DMA_Channel_0
#define NRF24_SPIx_RX_DMA_STREAM             DMA1_Stream3
#define NRF24_SPIx_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF3


// NRF2401 Commands
#define NRF_CMD_RREG                0b00000000
#define NRF_CMD_WREG                0b00100000
#define NRF_CMD_TX                  0b10100000
#define NRF_CMD_RX                  0b01100001
#define NRF_CMD_NOP                 0b11111111
#define NRF_CMD_FLUSH_TX        	0b11100001
#define NRF_CMD_FLUSH_RX        	0b11100010
#define NRF_CMD_WACKPL              0b10101000


// Stefan Wendler's libemb driver pieces
/* This file is part of the libemb project.
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
* along with this program. If not, see <http://www.gnu.org/licenses/>. */

#ifndef __NRF24L01_H_
#define __NRF24L01_H_

// Maximum payload size the NRF is able to send in bytes.
#define NRF_MAX_PAYLOAD           	32

// Primary mode TX
#define NRF_MODE_PTX                0

// Primary mode RX
#define NRF_MODE_PRX                1

// Retry delay for auto retransmit 250us
#define NRF_RT_DELAY_250        	0

// Retry delay for auto retransmit 500us
#define NRF_RT_DELAY_500        	1

// Retry delay for auto retransmit 750us
#define NRF_RT_DELAY_750        	2


// Retry delay for auto retransmit 1000us
#define NRF_RT_DELAY_1000        	3

// Retry delay for auto retransmit 1250us
#define NRF_RT_DELAY_1250        	4

// Retry delay for auto retransmit 1500us
#define NRF_RT_DELAY_1500        	5

// Retry delay for auto retransmit 1750us
#define NRF_RT_DELAY_1750        	6

// Retry delay for auto retransmit 2000us
#define NRF_RT_DELAY_2000        	7

// Retry delay for auto retransmit 2250us
#define NRF_RT_DELAY_2250        	8

// Retry delay for auto retransmit 2500us
#define NRF_RT_DELAY_2500        	9

// Retry delay for auto retransmit 2750us
#define NRF_RT_DELAY_2750        	10

// Retry delay for auto retransmit 3000us
#define NRF_RT_DELAY_3000        	11

// Retry delay for auto retransmit 3250us
#define NRF_RT_DELAY_3250        	12

// Retry delay for auto retransmit 3500us
#define NRF_RT_DELAY_3500        	13

// Retry delay for auto retransmit 3750us
#define NRF_RT_DELAY_3750        	14

// Retry delay for auto retransmit 4000us
#define NRF_RT_DELAY_4000        	15

// Error code for max retry reached
#define NRF_ERR_MAX_RT             	(-1)

// Error code for TX buffer full
#define NRF_ERR_TX_FULL           	(-2)

// Error code for RX buffer full
#define NRF_ERR_RX_FULL          	(-3)

// Definition of payload send through the device.
typedef struct {
   unsigned char size; // Payload size in bytes (must not exceed NRF_MAX_PAYLOAD]).
   unsigned char data[NRF_MAX_PAYLOAD];		// payload
} nrf_payload;


/* Setting this to 1 enables compilation of meta information
* for the registers. Meta information contains readable register
* names which could be used for debugging or displaying.
* <br/><br/>
* NOTE: Use "-DNRF_REG_DEF_META" compiler switch from makefile
* is the preferred way to set this flag. */
// #define NRF_REG_DEF_META                1

/* Maximum number of bytes needed to store register information (largest
* block of data to be read from nNRF is the address register with 5 bytes). */
#define NRF_MAX_REG_BUF                        5

// === NRF2041 Registers ===

#define NRF_REG_CONFIG            	0x00
#define NRF_REG_EN_AA             	0x01
#define NRF_REG_EN_RXADDR         	0x02
#define NRF_REG_SETUP_AW           	0x03
#define NRF_REG_SETUP_RETR         	0x04
#define NRF_REG_RF_CH              	0x05
#define NRF_REG_RF_SETUP        	0x06
#define NRF_REG_STATUS             	0x07
#define NRF_REG_OBSERVE_TX       	0x08
#define NRF_REG_CD                	0x09
#define NRF_REG_RX_ADDR_P0         	0x0A
#define NRF_REG_RX_ADDR_P1        	0x0B
#define NRF_REG_RX_ADDR_P2       	0x0C
#define NRF_REG_RX_ADDR_P3      	0x0D
#define NRF_REG_RX_ADDR_P4    		0x0E
#define NRF_REG_RX_ADDR_P5        	0x1F
#define NRF_REG_TX_ADDR          	0x10
#define NRF_REG_RX_PW_P0           	0x11
#define NRF_REG_RX_PW_P1          	0x12
#define NRF_REG_RX_PW_P2          	0x13
#define NRF_REG_RX_PW_P3          	0x14
#define NRF_REG_RX_PW_P4          	0x15
#define NRF_REG_RX_PW_P5        	0x16
#define NRF_REG_FIFO_STATUS      	0x17
// N/A                            	0x18
// N/A                          	0x19
// N/A                           	0x1A
// N/A                            	0x1B
#define NRF_REG_DYNPD            	0x1C
#define NRF_REG_FEATURE            	0x1D

// === NRF2401 Register Fields ===

// CONFIG
#define NRF_REGF_PRIM_RX                        0
#define NRF_REGF_PWR_UP                        	1
#define NRF_REGF_CRCO                        	2
#define NRF_REGF_EN_CRC                      	3
#define NRF_REGF_MASK_MAX_RT                	4
#define NRF_REGF_MASK_TX_DS                		5
#define NRF_REGF_MASK_RX_DR                    	6

// EN_AA
#define NRF_REGF_ENAA_P0                        0
#define NRF_REGF_ENAA_P1                        1
#define NRF_REGF_ENAA_P2                        2
#define NRF_REGF_ENAA_P3                        3
#define NRF_REGF_ENAA_P4                        4
#define NRF_REGF_ENAA_P5                        5

// EN_RXADDR
#define NRF_REGF_ERX_P0                       	0
#define NRF_REGF_ERX_P1                        	1
#define NRF_REGF_ERX_P2                        	2
#define NRF_REGF_ERX_P3                       	3
#define NRF_REGF_ERX_P4                        	4
#define NRF_REGF_ERX_P5                        	5

// SETUP_AW
#define NRF_REGF_AW                             0

// SETUP_RETR
#define NRF_REGF_ARC                           	0
#define NRF_REGF_ARD                          	1

// RF_CH
#define NRF_REGF_RF_CH                        	0

// RF_SETUP
#define NRF_REGF_LNA_HCURR                    	0
#define NRF_REGF_RF_PWR                      	1
#define NRF_REGF_RF_DR                        	2
#define NRF_REGF_PLL_LOCK                   	3

// STATUS
#define NRF_REGF_TX_FULL                        0
#define NRF_REGF_RX_P_NO                        1
#define NRF_REGF_MAX_RT                    		2
#define NRF_REGF_TX_DS                       	3
#define NRF_REGF_RX_DR                         	4

// OBSERVE_TX
#define NRF_REGF_ARC_CNT                   		0
#define NRF_REGF_PLOS_CNT                       1

// CD
#define NRF_REGF_CD                         	0

// ADDR
#define NRF_REGF_ADDR_A                       	0
#define NRF_REGF_ADDR_B                        	1
#define NRF_REGF_ADDR_C                       	2
#define NRF_REGF_ADDR_D                       	3
#define NRF_REGF_ADDR_E                       	4

// RX_PW
#define NRF_REGF_PW                           	0

// FIFO_STATUS
#define NRF_REGF_FIFO_RX_EMPTY                	0
#define NRF_REGF_FIFO_RX_FULL                	1
#define NRF_REGF_FIFO_TX_EMPTY                	4
#define NRF_REGF_FIFO_TX_FULL                	5
#define NRF_REGF_FIFO_TX_REUSE                	6

// DYNPD
#define NRF_REGF_DPL_P0                        	0
#define NRF_REGF_DPL_P1                        	1
#define NRF_REGF_DPL_P2                        	2
#define NRF_REGF_DPL_P3                        	3
#define NRF_REGF_DPL_P4                        	4
#define NRF_REGF_DPL_P5                        	5

// FEATURE
#define NRF_REGF_EN_DYN_ACK                		0
#define NRF_REGF_EN_ACK_PAY                		1
#define NRF_REGF_EN_DPL                        	2

// Defines how many bits make up a certain value in the register.
typedef struct {
#ifdef NRF_REG_DEF_META
     char                         *name; // If META is enabled, the readable name for the bit fiels.

/* Defines if the bit field is read-/writable. If set to 1,
* it means read+write access is allowd, otherwise only read
* access is possible to that field. */
     unsigned char         rw;
#endif
     unsigned char         size;	// Number of bits which make up that field.
} nrf_reg_bits;


typedef struct {
     unsigned char        count;
     nrf_reg_bits         data[];
} nrf_reg_fields;

// Definitoin of a register.
typedef struct {
#ifdef NRF_REG_DEF_META
     char                  *name;	// If META is enabled, the readable name for the register.
#endif
     unsigned char          size;	// Size of the register in bytes.
     nrf_reg_fields        *fields; // Array of bit-fields which make up this register
} nrf_reg;

// Definition of all available registers on the nRF24l01.
typedef struct {
     unsigned char count;	// Number of registers in the "data" array.
     nrf_reg data[];		// Array with registers on the nRF24l01.
} nrf_regs;

// Buffer used to store the data for reading/writing a register.
typedef struct {
    unsigned char size; // Number of bytes to read/write from/to the "data" array.
    unsigned char data[NRF_MAX_REG_BUF]; // The data to read/write to/from the register.
} nrf_reg_buf;

extern nrf_reg_fields nrf_reg_config_fields; // Fields of the CONFIG register.
extern nrf_reg_fields nrf_reg_enaa_fields; // Fields of the ENAA register.
extern nrf_reg_fields nrf_reg_enrxaddr_fields; // Fields of the EN_RXADDR register.
extern nrf_reg_fields nrf_reg_setupaw_fields; // Fields of the SETUP_AW register.
extern nrf_reg_fields nrf_reg_setupretr_fields; // Fields of the SETUP_RETR register.
extern nrf_reg_fields nrf_reg_rfch_fields; // Fields of the RF_CH register.
extern nrf_reg_fields nrf_reg_rfsetup_fields; // Fields of the RF_SETUP register.
extern nrf_reg_fields nrf_reg_status_fields; // Fields of the STATUS register.
extern nrf_reg_fields nrf_reg_observetx_fields; // Fields of the OBSERVE_TX register.
extern nrf_reg_fields nrf_reg_cd_fields; // Fields of the CD register.
extern nrf_reg_fields nrf_reg_addr_fields; // Fields of the ADDR register.
extern nrf_reg_fields nrf_reg_rxpw_fields; // Fields of the RX_PW register.
extern nrf_reg_fields nrf_reg_fifostat_fields; // Fields of the FIFO_STAT register.
extern nrf_reg_fields nrf_reg_dynpd_fields; // Fields of the DYNPD register.
extern nrf_reg_fields nrf_reg_feature_fields; // Fields of the FEATURE register.
extern nrf_regs nrf_reg_def; // Complete register definition for the nNRF24l01.


// === NRF library functions prototypes ===

/* Extract the value of a given field for a given register out of the
* given buffer and return it. The buffer has to be read by nrf_read_reg
* first.
*
* @param[in]        reg                the register definition to use for extracting (one of NRF_REG_*)
* @param[in]        regf        the field definition to use for extracting (one of NRF_REGF_*)
* @param[in]        *buf        the buffer containing the content of the register
* @return                                the value extracted for the field  */
unsigned char nrf_get_reg_field(unsigned char reg, unsigned char regf, nrf_reg_buf *buf);


/* Write a value to a given field for a given register into the
* given buffer. The buffer has to be read by nrf_read_reg
* first, and written back by nrf_write_reg.
*
* @param[in]        reg                the register definition to use for extracting (one of NRF_REG_*)
* @param[in]        regf        the field definition to use for extracting (one of NRF_REGF_*)
* @param[out]        *buf        the buffer to write the field value to
* @param[in]        value        the value to write to the field
* @return                                the value extracted for the field */
void nrf_set_reg_field(unsigned char reg, unsigned char regf, nrf_reg_buf *buf, unsigned char value);


/* Read contents of a register form the nRF24l01.
*
* @param[in] reg        register to read (see NRF_REG_* form nrf24l01_regs.h)
* @param[out] *buf        the value(s) read from the register
* @return                         number of bytes read from the register */
int nrf_read_reg(unsigned char reg, nrf_reg_buf *buf);


/* Write contents of a register to the nRF24l01.
*
* @param[in] reg        register to write (see NRF_REG_* form nrf24l01_regs.h)
* @param[in] *buf        the value(s) to write the register
* @return                         number of bytes written to the register
*/
int nrf_write_reg(unsigned char reg, nrf_reg_buf *buf);


/* Send payload non-blocking through the nRF24l01.
*
* @param[in] *payload        the payload to send
* @return                                 number of bytes sent or NRF_ERR_TX_FULL if TX buffer is full
*                                                 or NRF_ERR_MAX_RT if auto ACK with retry was enabled and
*                                                 payload was not delivered after configured retrys
*/
int nrf_send(nrf_payload *payload);


/* Send payload blocking through the nRF24l01.
*
* @param[in] *payload        the payload to send
* @return                                 number of bytes sent or NRF_ERR_TX_FULL if TX buffer is full
*                                                 or NRF_ERR_MAX_RT if auto ACK with retry was enabled and
*                                                 payload was not delivered after configured retrys
*/
int nrf_send_blocking(nrf_payload *payload);


/* Receive payload non-blocking from nRF24l01.
*
* @param        *payload        the payload received, "size" member of payload
*                                                 must be set to correct payload size (as configured for nRF)
* @return                                number of bytes received or NRF_ERR_RX_FULL when RX buffer is full
d */
int nrf_receive(nrf_payload *payload);


/* Receive payload blocking from nRF24l01.
*
* @param        *payload        the payload received, "size" member of payload
*                                                 must be set to correct payload size (as configured for nRF)
* @return                                number of bytes received or NRF_ERR_RX_FULL when RX buffer is full */
int nrf_receive_blocking(nrf_payload *payload);


/* Set ACK payload for a given pipe to be sent back in ESB mode with ACK-payload.
*
* @param[in]        *payload        ACK payload to send back on next RX
* @return                                         number of bytes sent */
int nrf_write_ack_pl(nrf_payload *payload, unsigned char pipe);


/* Read ACK payload received as response to the last TX request (only ESB mode with
* ACK-payload enabled).
*
* @param[out]        *payload        ACK payload received
* @return                                         number of bytes received  */
int nrf_read_ack_pl(nrf_payload *payload);


/* Preset configuration to configure the nRF24l01 into SB (Shock Burst) mode.
*
* @param[in]        mode        primery device mode: NRF_MODE_PRX for RX, or NRF_MODE_PTX for TX
* @param[in]        rf_ch        RF channel to use
* @param[in]        pw                payload width in bytes (must not exceed NRF_MAX_PAYLOAD)
* @param[in]        *addr        Address to use for TX and RX on pipe 0 */
void nrf_preset_sb(unsigned char mode, unsigned char rf_ch, unsigned char pw, nrf_reg_buf *addr);


/* Preset configuration to configure the nRF24l01 into ESB (Enhenced Shock Burst) mode.
*
* @param[in]        mode        primery device mode: NRF_MODE_PRX for RX, or NRF_MODE_PTX for TX
* @param[in]        rf_ch        RF channel to use
* @param[in]        pw                payload width in bytes (must not exceed NRF_MAX_PAYLOAD)
* @param[in]        rert        number of retrys for receiving ACK
* @param[in]        delay        delay to wait before next retry (one of NRF_RT_DELAY_*)
* @param[in]        *addr        Address to use for TX and RX on pipe 0 */
void nrf_preset_esb(unsigned char mode, unsigned char rf_ch, unsigned char pw, unsigned char retr, unsigned char delay, nrf_reg_buf *addr);


/* Preset configuration to configure the nRF24l01 into ESB (Enhenced Shock Burst) mode with
* enabled ACL payload.
*
* @param[in]        mode        primery device mode: NRF_MODE_PRX for RX, or NRF_MODE_PTX for TX
* @param[in]        rf_ch        RF channel to use
* @param[in]        pw                payload width in bytes (must not exceed NRF_MAX_PAYLOAD)
* @param[in]        rert        number of retrys for receiving ACK
* @param[in]        delay        delay to wait before next retry (one of NRF_RT_DELAY_*)
* @param[in]        *addr        Address to use for TX and RX on pipe 0 */
void nrf_preset_esbpl(unsigned char mode, unsigned char rf_ch, unsigned char pw, unsigned char retr, unsigned char delay, nrf_reg_buf *addr);

#endif




void nrf24init(void);
unsigned char nrf_spi_xfer_byte(unsigned char data);


