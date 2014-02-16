/*
* Use this code free of charge, but leave this text box
* This code is distributed as is with no warranties.
* plantalog.livejournal.com
*
*
*
*
*/

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rtc.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_spi.h"
#include "eeprom.h"
#include "stm32f4_discovery.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hd44780_lib.h"
#include "stdio.h"
#include "wireless_nrf24/nrf24_lib.h"
#include "fatfs_easyelectronics/ff.h"
#include "fatfs_easyelectronics/mmc_stm32f4.h"
//#include "ff9a/src/diskio.h"
//#include "ff9a/src/ff.h"


uint8_t spi_irq_counter=0;


/*
*
*
*
*

/// SPI2 ///


PC2
SPI2_MISO /
OTG_HS_ULPI_DIR /
TH_MII_TXD2
/I2S2ext_SD/ EVENTOUT
ADC123_IN12


PC3
SPI2_MOSI / I2S2_SD /
OTG_HS_ULPI_NXT /
ETH_MII_TX_CLK/
EVENTOUT
ADC123_IN13

PB10
SPI2_SCK / I2S2_CK /
I2C2_SCL/ USART3_TX /
OTG_HS_ULPI_D3 /
ETH_MII_RX_ER /
TIM2_CH3/ EVENTOUT


PB12
SPI2_NSS / I2S2_WS /
I2C2_SMBA/
USART3_CK/ TIM1_BKIN /
CAN2_RX /
OTG_HS_ULPI_D5/
ETH_RMII_TXD0 /
ETH_MII_TXD0/
OTG_HS_ID/ EVENTOUT


PB13
SPI2_SCK / I2S2_CK /
USART3_CTS/
TIM1_CH1N /CAN2_TX /
OTG_HS_ULPI_D6 /
ETH_RMII_TXD1 /
ETH_MII_TXD1/
EVENTOUT
OTG_HS_V


PB14
SPI2_MISO/ TIM1_CH2N /
TIM12_CH1 /
OTG_HS_DM/
USART3_RTS /
TIM8_CH2N/I2S2ext_SD/
EVENTOUT

PB15
SPI2_MOSI / I2S2_SD/
TIM1_CH3N / TIM8_CH3N
/ TIM12_CH2 /
OTG_HS_DP/ EVENTOUT

PB9 I/O FT
SPI2_NSS/ I2S2_WS/
TIM4_CH4/ TIM11_CH1/
SDIO_D5 / DCMI_D7 /
I2C1_SDA / CAN1_TX/
EVENTOUT


/// SPI3 ///

PA15
(JTDI)
I/O FT
JTDI/ SPI3_NSS/
I2S3_WS/TIM2_CH1_ETR
/ SPI1_NSS / EVENTOUT


PC10 I/O FT
SPI3_SCK / I2S3_CK/
UART4_TX/SDIO_D2 /
DCMI_D8 / USART3_TX/
EVENTOUT


PC11 I/O FT
UART4_RX/ SPI3_MISO /
SDIO_D3 /
DCMI_D4/USART3_RX /
I2S3ext_SD/ EVENTOUT


PC12 I/O FT
UART5_TX/SDIO_CK /
DCMI_D9 / SPI3_MOSI
/I2S3_SD / USART3_CK/
EVENTOUT


PB3
(JTDO/
TRACESWO)
I/O FT
JTDO/ TRACESWO/
SPI3_SCK / I2S3_CK /
TIM2_CH2 / SPI1_SCK/
EVENTOUT

PB4
(NJTRST)
I/O FT
NJTRST/ SPI3_MISO /
TIM3_CH1 / SPI1_MISO /
I2S3ext_SD/ EVENTOUT

PB5 I/O FT
I2C1_SMBA/ CAN2_RX /
OTG_HS_ULPI_D7 /
ETH_PPS_OUT/TIM3_CH
2 / SPI1_MOSI/
SPI3_MOSI / DCMI_D10 /
I2S3_SD/ EVENTOUT


*
*



/// SPI1 ///

PA4
SPI1_NSS / SPI3_NSS /
USART2_CK /
DCMI_HSYNC /
OTG_HS_SOF/ I2S3_WS/
EVENTOUT
ADC12_IN4
/DAC1_OUT


PA5
SPI1_SCK/
OTG_HS_ULPI_CK /
TIM2_CH1_ETR/
TIM8_CHIN/ EVENTOUT
ADC12_IN5/
DAC2_OUT


PA6
SPI1_MISO /
TIM8_BKIN/TIM13_CH1 /
DCMI_PIXCLK / TIM3_CH1
/ TIM1_BKIN/EVENTOUT
ADC12_IN6

PA7
SPI1_MOSI/ TIM8_CH1N /
TIM14_CH1/TIM3_CH2/
ETH_MII_RX_DV /
TIM1_CH1N /
RMII_CRS_DV/
EVENTOUT
ADC12_IN7



/// TIM2 ///
*
*
* PB11
I2C2_SDA/USART3_RX/
OTG_HS_ULPI_D4 /
ETH_RMII_TX_EN/
ETH_MII_TX_EN /
TIM2_CH4/ EVENTOUT

*/


/*
* PA3
USART2_RX/TIM5_CH4 /
TIM9_CH2 / TIM2_CH4 /
OTG_HS_ULPI_D0 /
ETH_MII_COL/
EVENTOUT
*
*/




typedef struct
{
  uint16_t                         DHT_Temperature; // temperature
  uint16_t                         DHT_Humidity; // from 0 (0%) to 1000 (100%)
  uint8_t                         DHT_CRC; // from 0 (0%) to 1000 (100%)
  RTC_TimeTypeDef         Last_refresh;
}DHT_data;


#define BUTTON_RANGES_START_ADDR        0x0621        // button ranges (8 values in a row)
#define BUTTON_OK                                2
#define BUTTON_CNL                                3
#define BUTTON_BCK                                1
#define BUTTON_FWD                                4

#define DHT_TRIG_PLUG                        7                // PA7
#define DHT_DATA_START_POINTER        2                // sets the first bit number in captured sequence of DHT response bits


// analog inputs
#define JDR_BUTTONS        ADC1->JDR3                // continuous ADC channel for buttons
#define JDR_PH                ADC1->JDR1                // continuous ADC channel for pH

#define sonar_port                                GPIOA

#define sonar_echo_port                        GPIOA
#define sonar_trig_port                        GPIOA
#define sonar_pin_trig                        GPIO_Pin_11
#define sonar_pin_echo                        GPIO_Pin_13
#define sonarclk                                RCC_AHB1Periph_GPIOA
#define sonar_trig_clk                        RCC_AHB1Periph_GPIOA
#define sonar_echo_clk                        RCC_AHB1Periph_GPIOA

#define        sonar_EXTI_LINE EXTI_Line0
#define        sonar_port_source                EXTI_PortSourceGPIOA
#define        sonar_pin_source                EXTI_PinSource0
#define sonar_EXTI_IRQn                 EXTI0_IRQn
#define sonar_echo_timer                TIM2

#define LIGHT_RANGE_ADDR        0x7CFF        // the border for the light sensor trigger
#define EE_PLUG_SETTINGS        0x7D00
#define EE_TIMER1_ON                0x8000
// #define EE_CTIMER                        0x7E00
#define EE_CTIMER_DURATION        0x7E00
#define EE_CTIMER_INTERVAL        0x7E02
#define EE_TIMER_SIZE                10
#define EE_CTIMER_SIZE                5
#define PLUG_AMOUNT                        3                // this links to number of pins on PORTC, so not more then 16
#define LS2CT_ADDR                        0x7C00        // 64 tajmera tipa LS+2CT (16bit na tajmer)
#define        ADDLIGHT_ADDR                0x7C40        // 128 tajmerov dosveta (8 bit na tajmer)
#define        T2CT_ADDR                        0x7C80        // 63 tajmera tipa 1obychnyj+2ckilicheskih
#define PH4_ADDR                        0x0600
#define PH7_ADDR                        0x0601
#define PH_INTERVAL                        0x0602                // pH measurement interval in milliseconds
#define PH_BUFF_SIZE                0x0603                // pH buffer size
#define SD_LOG_INTERVAL                0x0620                // sd logging interval, seconds

// SD card defines
#define GO_IDLE_STATE 0
#define SEND_IF_COND 8
#define READ_SINGLE_BLOCK 17
#define WRITE_SINGLE_BLOCK 24
#define SD_SEND_OP_COND         41
#define APP_CMD                         55
#define READ_OCR                 58

#define CS_ENABLE GPIOB->BSRRL |= GPIO_Pin_5;
#define CS_DISABLE          GPIOB->BSRRH |= GPIO_Pin_5;


// === Chloe system registers ===

RTC_TimeTypeDef curtime;	// current time
RTC_DateTypeDef curdate;	// current date

uint8_t button_code = 0;	// button pressed code

DHT_data DHTValue;			// DHT values structure

// LCD string data to display
static uint8_t LCDLine1[16], LCDLine2[16];              // framebuffer
static uint8_t lcd_pointerx=0, lcd_pointery=0;			// framebuffer pointers

// Migrated from Cadi (F100) firmware

char curphstr[5];										// ph value string
uint16_t phAdcValue;									// pH ADC reading value
uint32_t timerStateFlags, cTimerStateFlags;				// timer and cyclic timer state flags
uint32_t plugStateFlags;								// plugs' (loads) states
uint8_t plugSettings[PLUG_AMOUNT] = {0, 1, 2};        	// PLUG_AMOUNT - number of plugs HARDCODE
// elementy massiva - nomera tajmerov, sootvetstvujushih Plug'am.
// 0 element - pervyj plug, 1 element - plug no 2, etc




// === Chloe system variables ===

// sonar variables
uint16_t        sonar_ccnt=0;
uint16_t        sonar_value=0;
uint16_t        IC1Value[84];
uint16_t        IC2Value[84];


// digital humidity and temperature data
uint16_t dht_bit_array[50];				// buffer to store pulse widths of DHT response
uint8_t dht_data[5];					// 40bits of DHT data extracted from DHT response
uint8_t dht_bit_position = 0;			// bit array read pointer
uint8_t dht_bit_ready = 0;				// bit read ready flag
uint8_t dht_data_ready = 0;				// DHT sensor data read ready flag
uint8_t dht_rh_str[4], dht_t_str[4];	// DHT22 humidity and temperature values strings

// Real-time clock variables
uint8_t rtc_time_now_str[16];	// time and date string

static uint8_t sonar_str[5];	// sonar value string
uint16_t capture1=0, capture2=0;	// sonar echo pulse width
uint8_t capture_is_ready = 0;

GPIO_InitTypeDef GPIO_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
TIM_ICInitTypeDef TIM_ICInitStructure;


volatile uint16_t systick_ms = 0;
/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[2] = {
                        EE_TIMER1_ON,
                        EE_CTIMER_DURATION
};

// --- Buttons system variables ---
uint16_t button_ranges[8];        // 0,2,4,6 - lower, 1,3,5,7 - higher values for buttons
uint8_t buttonReverse=0;		// used for both directions button calibration capability

// Chloe SD card support with FatFS
uint8_t SDHC;
static FIL cadizlog;		// file structure associated with logfile on SD card
uint8_t logString[64];		// log string to store on SD card

// --- NRF24 wireless module ---
extern unsigned char nrfRxBuff[NRF24_RX_BUFF_SIZE];		// RX FIFO buffer
extern unsigned char nrfTxBuff[NRF24_TX_BUFF_SIZE];		// TX FIFO buffer
extern uint8_t nrf_rx_pointer, nrf_tx_pointer;			// RX and TX FIFO pointers
uint8_t nrf_spi_busy=0;									// NRF24 SPI bus busy flag

//// MIGRATED FROM F100 ////
uint8_t logInterval=0;
ErrorStatus HSEStartUpStatus;
FLASH_Status FlashStatus;
uint8_t no_sd;
FATFS fs;
uint16_t VarValue = 0;
uint16_t ph_seven=0;
uint32_t ph0=0;
uint16_t ph4=0;
int cdel=0;
uint16_t phBuffer[10];
uint32_t lastWriteTime=0;
//uint16_t ph_seven, ph4;
uint8_t lightSensor;
uint16_t lightRange;


#define MENURECS        23

// menu items
const char menuItemArray[MENURECS][18]=
{
                {"MONITOR MODE"},                // 0
                {"TIMERS"},                                // 1
                {"Timer 1"},                        // 2
                {"Timer 2"},                        // 3
                {"Timer 3"},                        // 4
                {"SET CLOCK"},                        // 5
                {"CYCLIC TIMERS"},                        // 6
                {"Cyclic Timer 1"},                        // 7
                {"Cyclic Timer 2"},                        // 8
                {"Cyclic Timer 3"},                        // 9
                {"PLUG SETTINGS"},                        // 10
                {"Plug 1"},                        // 11
                {"Plug 2"},                        // 12
                {"Plug 3"},                        // 13
                {"pH-monitor"},                // 14
                {"Calibration"},        // 15
                {"pH-stabilizer"},        // 16
                {"DAYLIGHT SENSOR"},// 17
                {"COMBI TIMERS"},        // 18
                {"LS+2CT"},        // 19
                {"T+2CT"},        // 20
                {"Add Light"},        // 21
                {"Temp. & humidity"}        // 22

};

// 0 - nr zapisi, 1 - link na tekst, 2 - <, 3 - >, 4 - OK, 5 - CNCL, 6 - tip zapisi (0 - folder, 1 - program)
const int fatArray[MENURECS][7]=
{
                {0,        0,        22,        1,        1,        0,        1},
                {1,        1,        0,        5,        2,        1,        0},
                {2,        2,        4,        3,        2,        1,        1},
                {3,        3,        2,        4,        3,        1,        1},
                {4,        4,        3,        2,        4,        1,        1},
                {5,        5,        1,        6,        5,        5,        1},
                {6,        6,        5,        10,        7,        6,        0},
                {7,        7,        9,        8,        6,        6,        1},
                {8,        8,        7,        9,        7,        6,        1},
                {9,        9,        8,        7,        8,        6,        1},
                {10,10,        6,        14,        11,        10,        0},
                {11,11,        13,        12,        9,        10,        1},
                {12,12,        11,        13,        10,        10,        1},
                {13,13,        12,        11,        11,        10,        1},
                {14,14,        10,        17,        15,        14,        0},
                {15,15,        16,        16,        15,        14,        1},
                {16,16,        15,        15,        16,        14,        1},
                {17,17,        14,        18,        14,        17,        1},
                {18,18,        17,        22,        19,        18,        0},
                {19,19,        21,        20,        15,        19,        1},
                {20,20,        19,        21,        16,        20,        1},
                {21,21,        20,        19,        17,        21,        1},
                {22,22,        18,        0,        17,        22,        1}

};

//// EOF MIGRATED FROM F100 ////

void Lcd_write_str_fb(uc8 *STRING);
void dht_arr_displayer(void);
int menuSelector(void);
void programRunner(uint8_t programId);
void readButtonRanges(void);
void saveButtonRanges(void);
void loadSettings();
char* int32str(uint32_t d, char* out);
extern void EXTI15_10_IRQHandler(void);
extern void TIM2_IRQHandler(void);
static void TIM_Config(void);
void EXTI0_IRQHandler(void);
static void RTC_Config(void);
extern void RTC_WKUP_IRQHandler(void);
extern void TIM5_IRQHandler(void);
extern void Init_lcd(void);
extern void Lcd_write_arr(uc8 *STRING, uint8_t chars);
void buttonsInit(void);
FRESULT string2log(char* str, int a);
void buttonCalibration(void);
uint8_t readButtons(void);
void copy_arr(uint8_t *source, uint8_t *destination, uint8_t amount, uint8_t pos);
uint32_t EE_ReadWord(uint16_t Address);
void monitorDisplayer(void);
void flush_lcd_buffer(void);
void Lcd_go2(uint8_t y, uint8_t x);
void nrf24init(void);
void Lcd_print_digit(uint8_t numb);
void nrfTest(void);
void Lcd_write_digit(uint8_t numb);
void buttonCodeSupplier(uint16_t adc_value);

unsigned char nrf_spi_xfer_byte(unsigned char data);
int nrf_read_reg(unsigned char reg, nrf_reg_buf *buf, uint8_t plsize);



void Lcd_print_digit(uint8_t numb){
	if (numb<10) {
    	if (lcd_pointery==0) {
        	LCDLine1[lcd_pointerx++] = 48;
          	LCDLine1[lcd_pointerx++] = 48+numb;
     	}
      	else {
        	LCDLine2[lcd_pointerx++] = 48;
         	LCDLine2[lcd_pointerx++] = 48+numb;
     	}
	}
	else {
    	if (lcd_pointery==0) {
    		LCDLine1[lcd_pointerx++] = (numb/10)+48;
        	LCDLine1[lcd_pointerx++] = (numb-(numb/10)*10)+48;
    	}
    	else {
        	LCDLine2[lcd_pointerx++] = (numb/10)+48;
        	LCDLine2[lcd_pointerx++] = (numb-(numb/10)*10)+48;
    	}
	}
}

void Lcd_go2(uc8 y,uc8 x)
{
	lcd_pointery = y;
	lcd_pointerx = x;
}

void flush_lcd_buffer(void){
	Lcd_go2(0,0);
	uint8_t i=0;
	for (i=0;i<16;i++){
		LCDLine1[i]=32;
		LCDLine2[i]=32;
	}
}



void Lcd_write_str_fb(uc8 *STRING){
	char c;
	while (c=*STRING++){
		if (lcd_pointery==0) {
			LCDLine1[lcd_pointerx] = c;
			lcd_pointerx++;
		}
		else {
			LCDLine2[lcd_pointerx] = c;
			lcd_pointerx++;
		}
		// vTaskDelay(1);
	}

}


int menuSelector_f100(void)
{
	int curItem=0;	// default item to display entering the menu
	int programId = 0;
//	int buttonPressed=readButtons();
	int textId=fatArray[curItem][1];
	while (programId==0){
		textId=fatArray[curItem][1];
		Lcd_goto(0,0);
		Lcd_write_str(menuItemArray[textId]);
		int curButton=readButtons();
		if (curButton>0){
			vTaskDelay(100);
			Lcd_clear();
		}
		vTaskDelay(10);
		if (curButton==BUTTON_OK){
			if (fatArray[curItem][6]==1)		// run program
				{programId=fatArray[curItem][4];}
			else	{curItem=fatArray[curItem][4];}			// enter folder
			Lcd_clear();
		}
		else if (curButton==BUTTON_CNL){
			curItem=fatArray[curItem][5];
			Lcd_clear();
		}
		else if (curButton==BUTTON_BCK){
			curItem=fatArray[curItem][2];
			Lcd_clear();
		}
		else if (curButton==BUTTON_FWD){
			curItem=fatArray[curItem][3];
			Lcd_clear();
		}
	}
	vTaskDelay(1);
	return(programId);
}

// function returns the programId selected in menu to run the program
int menuSelector(void)
{
        int curItem=0;        // default item to display entering the menu
        int programId = 0;
//        int buttonPressed=readButtons();
        int textId=fatArray[curItem][1];
        while (programId==0){
                textId=fatArray[curItem][1];
                Lcd_go2(0,0);
                Lcd_write_str_fb(menuItemArray[textId]);
     /*           if (button_code>0){
                        vTaskDelay(100);
                        Lcd_clear();
                } */
                vTaskDelay(10);
                if (button_code==BUTTON_OK){
                        if (fatArray[curItem][6]==1)                // run program
                                {programId=fatArray[curItem][4];}
                        else        {curItem=fatArray[curItem][4];}                        // enter folder
                        Lcd_clear();
                }
                else if (button_code==BUTTON_CNL){
                        curItem=fatArray[curItem][5];
                        Lcd_clear();
                }
                else if (button_code==BUTTON_BCK){
                        curItem=fatArray[curItem][2];
                        Lcd_clear();
                }
                else if (button_code==BUTTON_FWD){
                        curItem=fatArray[curItem][3];
                        Lcd_clear();
                }

        }
        vTaskDelay(1);
        return(programId);
}

void programRunner(uint8_t programId){
        uint32_t tmp;
        switch (programId) {
        case 1:
                break;
        case 2:
         		nrfTest();
                break;
  /*        case 3:

                setTimer(1);
                break;
        case 4:
                setTimer(2);
                break;
        case 5:
                tmp = RTC_GetCounter();
                uint32_t unixtime = timeAdjust(tmp, 1);
                RTC_SetCounter(unixtime);
                Lcd_clear();
                break;
        case 6:
         setCTimer(0);
                break;
        case 7:
                setCTimer(1);
                break;
        case 8:
                setCTimer(2);
                break;
        case 9:
                setPlug(0);
                break;
        case 10:
                setPlug(1);
                break;
        case 11:
                setPlug(2);
                break;
        case 13:
                phMonSettings();
                break;
        case 14:
                lightRangeSet();
                break;
        case 15:
                calibratePh();
                break;
        case 16:
                phStabSettings();
                break;
                */
        case 17:
                dht_arr_displayer();
                displayAdcValues();
//                setDutyCycle();
                break;
        }
}


void buttonsInit(void) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //

        GPIOA->MODER |= (GPIO_MODER_MODER0 << (0 * 2)); //
        GPIOA->MODER |= (GPIO_MODER_MODER0 << (1 * 2)); //
        GPIOA->MODER |= (GPIO_MODER_MODER0 << (2 * 2)); //
        GPIOA->MODER |= (GPIO_MODER_MODER0 << (3 * 2)); //

        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //
        ADC1->CR1 = 0; //
        ADC1->CR2 = 0; //
        ADC1->JSQR = 0;

        ADC1->CR2 = ADC_CR2_JEXTSEL; //
        ADC1->CR2 |= ADC_CR2_CONT; //
        ADC1->CR1 |= ADC_CR1_SCAN; //
        ADC1->CR1 |= ADC_CR1_JAUTO; //
        ADC1->JSQR = ((4-1)<<20); //number of channels in injected group
        ADC1->JSQR |= (0<<(5*3)); //ch. number for first conversion
        ADC1->JSQR |= (1<<(5*2)); //ch. number for 2nd conversion
        ADC1->JSQR |= (2<<(5*1)); //ch. number for 2nd conversion
        ADC1->JSQR |= (3<<(5*0)); //ch. number for 2nd conversion

        ADC1->CR2 |= ADC_CR2_ADON; //enable ADC
        ADC1->CR2 |= ADC_CR2_JSWSTART; //start conversion
}

void adc_read(void){
        ADC1->CR2 |= ADC_CR2_JSWSTART;
        while (!(ADC1->SR & ADC_SR_JEOC));
}

extern void RTC_WKUP_IRQHandler(void)
{
  if(RTC_GetITStatus(RTC_IT_WUT) != RESET)
  {
    /* Toggle on LED1 */
// STM_EVAL_LEDToggle(LED1);
    RTC_ClearITPendingBit(RTC_IT_WUT);
    EXTI_ClearITPendingBit(EXTI_Line22);
  }
}

/**
* @brief This function handles TIM5 global interrupt request.
* @param None
* @retval None
*/
extern void TIM5_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM5, TIM_IT_CC4) != RESET)
  {
    /* Get the Input Capture value */
// tmpCC4[uwCaptureNumber++] = TIM_GetCapture4(TIM5);

    /* Clear CC4 Interrupt pending bit */
    TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);

 // if (uwCaptureNumber >= 2)
 // {
      /* Compute the period length */
 // uwPeriodValue = (uint16_t)(0xFFFF - tmpCC4[0] + tmpCC4[1] + 1);
// }
  }
}


void vTaskLED5(void *pvParameters);
void vTaskLED6(void *pvParameters);
void vTaskLCDdraw(void *pvParameters);
void vTaskSonar(void *pvParameters);
void vTaskReadButtons(void *pvParameters);
void vTaskDHT(void *pvParameters);
void vTaskSDLog(void *pvParameters);
void vTaskProgramRunner(void *pvParameters);

void init_gpio(void){        // do we need it?
         GPIO_InitTypeDef gpio_cfg,GPIO_InitStructure;
         GPIO_StructInit(&gpio_cfg);

         // vyvod ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¨trigÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¨ signala
         RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);                // PB13
         gpio_cfg.GPIO_Mode = GPIO_Mode_OUT;
         gpio_cfg.GPIO_OType = GPIO_OType_PP;
         gpio_cfg.GPIO_PuPd = GPIO_PuPd_UP;
         gpio_cfg.GPIO_Pin = GPIO_Pin_13;
         GPIO_Init(GPIOB, &gpio_cfg);

         // TIM5 na PA3 dlja lovli "echo" s podschetom ego dliny
         GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
         GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
         GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
         GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
         GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void dht_init(void){
         GPIO_InitTypeDef GPIO_InitStructure;
         NVIC_InitTypeDef NVIC_InitStructure;

         /* TIM4 clock enable */
         RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

         /* GPIOB clock enable */
         RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

         /* TIM4 chennel2 configuration : PB.07 */
         GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
         GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
         GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
         GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
         GPIO_Init(GPIOB, &GPIO_InitStructure);

         // CARD SELECT for SD card        // chto za porno????
         GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
         GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
         GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
         GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
         GPIO_Init(GPIOB, &GPIO_InitStructure);

         /* Connect TIM pin to AF2 */
         GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);

         /* Enable the TIM4 global Interrupt */
         NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
         NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
         NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
         NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
         NVIC_Init(&NVIC_InitStructure);
        TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
         TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
         TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
         TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
         TIM_ICInitStructure.TIM_ICFilter = 0x0;

         TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);

         /* Select the TIM4 Input Trigger: TI2FP2 */
         TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);

         /* Select the slave Mode: Reset Mode */
         TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
         TIM_SelectMasterSlaveMode(TIM4,TIM_MasterSlaveMode_Enable);

         /* Enable the CC2 Interrupt Request */
         TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
         /* TIM enable counter */
         TIM_Cmd(TIM4, ENABLE);

}

void dht_init_out(void){        // init GPIO pin as OUTput
         GPIO_InitTypeDef gpio_cfg,GPIO_InitStructure;
         GPIO_StructInit(&gpio_cfg);

         // vyvod ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¨trigÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¨ signala
         RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);                // PB13
         gpio_cfg.GPIO_Mode = GPIO_Mode_OUT;
         gpio_cfg.GPIO_OType = GPIO_OType_PP;
         gpio_cfg.GPIO_PuPd = GPIO_PuPd_UP;
         gpio_cfg.GPIO_Pin = GPIO_Pin_7;
         GPIO_Init(GPIOB, &gpio_cfg);
}

void dht_conv_data(void){ // convert DHT impulse lengths array into numbers and strings of T and rH
        uint8_t i, i2;
        uint16_t caps1[45], caps2[45];
        uint16_t zero_ticks, one_ticks;
        vTaskDelay(10);
        if (dht_bit_ready==1) {
                int dht_buf_pointer=DHT_DATA_START_POINTER;        // points the dht data start bit
                for (i=0;i<45;i++) {
                        caps1[i]=dht_bit_array[i];
                }
                dht_data_ready = 0;
                for (i=0; i<5; i++){
                        dht_data[i]=0;
                        vTaskDelay(10);
                        for (i2=8; i2>0; i2--) {
                                if (caps1[dht_buf_pointer]>40 && caps1[dht_buf_pointer]<80) {
                                        dht_data[i] |= (1<<(i2-1)); // set i2 bit in dht_data[i]
                                }
                                dht_buf_pointer++;
                        }
                }
                vTaskDelay(10);

                DHTValue.DHT_Humidity = dht_data[0]*256+dht_data[1];
                DHTValue.DHT_Temperature = dht_data[2]*256+dht_data[3];
                DHTValue.DHT_CRC = dht_data[5];


                vTaskDelay(10);
                dht_t_str[0] = (DHTValue.DHT_Temperature%1000)/100+48;
                dht_t_str[1] = (DHTValue.DHT_Temperature%100)/10+48;
                dht_t_str[2] = 46;
                dht_t_str[3] = (DHTValue.DHT_Temperature%10)+48;

                vTaskDelay(10);
                dht_rh_str[0] = (DHTValue.DHT_Humidity%1000)/100+48;
                dht_rh_str[1] = (DHTValue.DHT_Humidity%100)/10+48;
                dht_rh_str[2] = 46;
                dht_rh_str[3] = (DHTValue.DHT_Humidity%10)+48;

                dht_data_ready = 1;
        }
}

void Lcd_write_digit(uint8_t numb){
	/*
	if (raw>0) {
		if (numb<10) {
				Lcd_write_data(48);
				Lcd_write_data(48+numb);
		}
		else {
		     	Lcd_write_data((numb/10)+48);
		    	Lcd_write_data((numb-(numb/10)*10)+48);
		}
	}
	else { */
		if (numb<10) {
			if (lcd_pointery==0) {
				LCDLine1[lcd_pointerx++] = 48;
				LCDLine1[lcd_pointerx++] = (48+numb);
			}
			else {
				LCDLine2[lcd_pointerx++] = 48;
				LCDLine2[lcd_pointerx++] = (48+numb);
			}
		}
		else {
			if (lcd_pointery==0) {
				LCDLine1[lcd_pointerx++] = (numb/10)+48;
				LCDLine1[lcd_pointerx++] = (numb-(numb/10)*10)+48;
			}
			else {
				LCDLine2[lcd_pointerx++] = (numb/10)+48;
				LCDLine2[lcd_pointerx++] = (numb-(numb/10)*10)+48;
			}
		}

//	}

}

void dht_arr_displayer(void){
//        dht_get_data();
        uint8_t button=0;
        uint8_t arr_pointer=0;
        while (button!=BUTTON_OK) {
//                dht_get_data();
                vTaskDelay(30);
                Lcd_goto(0,0);
                Lcd_write_digit(arr_pointer);
                Lcd_write_str(": ");
                Lcd_write_digit((dht_bit_array[arr_pointer]/100));
                Lcd_write_digit(dht_bit_array[arr_pointer]);
                if (button_code==BUTTON_FWD) {
                        if (arr_pointer==255) {
                                arr_pointer==0;
                        }
                        else {
                                arr_pointer++;
                        }
                }
                if (button_code==BUTTON_BCK){
                        if (arr_pointer==0) {
                                arr_pointer==255;
                        }
                        else {
                                arr_pointer--;
                        }
                }
                Lcd_goto(1,0);
                Lcd_write_str("T:");
                copy_arr(&dht_t_str, &LCDLine2, 4, 2);
                Lcd_goto(1,7);
                Lcd_write_str("rH:");
                copy_arr(&dht_rh_str, &LCDLine2, 4, 10);
        }
        Lcd_clear();
}

void dht_conv_data2(void){
        uint8_t i, i2;
        uint16_t caps1[45], caps2[45];
        uint16_t zero_ticks, one_ticks;

        vTaskDelay(10);
        if (dht_bit_ready==1) {
                int dht_buf_pointer=2;
                for (i=0;i<45;i++) {
                        caps1[i]=IC1Value[i];
//                        caps2[i]=IC2Value[i];        //possibly not necessary
                }
                dht_data_ready = 0;
                for (i=0; i<5; i++){
                        dht_data[i]=0;
                        vTaskDelay(10);
                        for (i2=8; i2>0; i2--) {
                                if (caps1[dht_buf_pointer]>50 && caps1[dht_buf_pointer]<80) {
                                        dht_data[i] |= (1<<(i2-1));
                                }
                                dht_buf_pointer++;
                        }
                        DHTValue.DHT_Temperature = dht_data[0]*256+dht_data[1];
                        DHTValue.DHT_Humidity = dht_data[2]*256+dht_data[3];
                        DHTValue.DHT_CRC = dht_data[5];
                }
                dht_t_str[0] = (DHTValue.DHT_Temperature%1000)/100+48;
                dht_t_str[1] = (DHTValue.DHT_Temperature%100)/10+48;
                dht_t_str[2] = 46;
                dht_t_str[3] = (DHTValue.DHT_Temperature%10)+48;
                for (i=0;i<4;i++){
                        LCDLine1[i+5]=dht_t_str[i];
                }

                vTaskDelay(10);
                dht_rh_str[0] = (DHTValue.DHT_Humidity%1000)/100+48;
                dht_rh_str[1] = (DHTValue.DHT_Humidity%100)/10+48;
                dht_rh_str[2] = 46;
                dht_rh_str[3] = (DHTValue.DHT_Humidity%10)+48;

                for (i=0;i<4;i++){
    //                    LCDLine1[i]=dht_rh_str[i];
                }
                dht_data_ready = 1;
        }

}



void dht_get_data(void){
         //reset DHT11
        dht_init_out();
         vTaskDelay(50);
         GPIOB->BSRRH |= GPIO_Pin_7;
         vTaskDelay(120);
         GPIOB->BSRRL |= GPIO_Pin_7;
         vTaskDelay(120);
         GPIOB->BSRRH |= GPIO_Pin_7;
         dht_init();
         //start reading
         dht_bit_ready = 0;
         dht_bit_position = 0;
//         while(dht_bit_position<40){        // vozmozhna zasada v etom cikle s posledujushim zavisiom. togda nado perezapustit' cikl snova. kak variant, snizit' ciferku ozhidanija (i<38 ili kak-to tak) i mozhno dobavit' zaderzhku po vyhodu iz cikla
                 vTaskDelay(50);
//         }
         if (dht_bit_position>(38+DHT_DATA_START_POINTER)){
                 dht_bit_ready = 1;
         }
         else {
                 dht_bit_ready = 0;
         }
         vTaskDelay(5);
         dht_conv_data();
}


void TIM4_IRQHandler(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);

  /* Clear TIM4 Capture compare interrupt pending bit */
  TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);

  /* Get the Input Capture value */
  dht_bit_array[dht_bit_position] = TIM_GetCapture1(TIM4)/16;
         IC1Value[dht_bit_position] = TIM_GetCapture1(TIM4)/16;
         IC2Value[dht_bit_position] = TIM_GetCapture2(TIM4)/16;
         dht_bit_position++;
}



void rtc_init(void){
        RTC_InitTypeDef RTC_InitStructure;
        uint16_t        uwLsiFreq = 32786;

         RTC_Config();
         RTC_InitStructure.RTC_AsynchPrediv = 0x7F;
         RTC_InitStructure.RTC_SynchPrediv        = (uwLsiFreq/128) - 1;
         RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
         RTC_Init(&RTC_InitStructure);
}



static void RTC_Config(void)
{
        RTC_InitTypeDef RTC_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Enable the PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Allow access to RTC */
  PWR_BackupAccessCmd(ENABLE);

  /* LSI used as RTC source clock */
  /* The RTC Clock may varies due to LSI frequency dispersion. */
  /* Enable the LSI OSC */
  RCC_LSICmd(ENABLE);

  /* Wait till LSI is ready */
  while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {
  }

  /* Select the RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();

  /* Calendar Configuration with LSI supposed at 32KHz */
  RTC_InitStructure.RTC_AsynchPrediv = 0x7F;
  RTC_InitStructure.RTC_SynchPrediv = 0xFF; /* (32KHz / 128) - 1 = 0xFF*/
  RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
  RTC_Init(&RTC_InitStructure);

  /* EXTI configuration *******************************************************/
  EXTI_ClearITPendingBit(EXTI_Line22);
  EXTI_InitStructure.EXTI_Line = EXTI_Line22;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable the RTC Wakeup Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure the RTC WakeUp Clock source: CK_SPRE (1Hz) */
  RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
  RTC_SetWakeUpCounter(0x0);

  /* Enable the RTC Wakeup Interrupt */
  RTC_ITConfig(RTC_IT_WUT, ENABLE);

  /* Enable Wakeup Counter */
  RTC_WakeUpCmd(ENABLE);
}

void sonar_init(){
        GPIO_InitTypeDef gpio_cfg;
         GPIO_StructInit(&gpio_cfg);

         RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
         RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

                 // PB13
         gpio_cfg.GPIO_Mode = GPIO_Mode_OUT;
         gpio_cfg.GPIO_OType = GPIO_OType_PP;
         gpio_cfg.GPIO_PuPd = GPIO_PuPd_UP;
         gpio_cfg.GPIO_Pin = GPIO_Pin_13;
         GPIO_Init(GPIOB, &gpio_cfg);

         // TIM2 channel 4 pin (PB.11) configuration as PWM input
         GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
         GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
         GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
         GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
         GPIO_Init(GPIOB, &GPIO_InitStructure);

         GPIO_PinAFConfig(GPIOB, GPIO_PinSource11,GPIO_AF_TIM2);
         /* Connect TIM pins to AF2 */


         /* Enable the TIM2 global Interrupt */
         NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
         NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
         NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
         NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
         NVIC_Init(&NVIC_InitStructure);

         TIM_ICStructInit(&TIM_ICInitStructure);
         TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
         TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
         TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
         TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
         TIM_ICInitStructure.TIM_ICFilter = 0x0;
         // TIM enable counter
         TIM_ICInit(TIM2, &TIM_ICInitStructure);
         TIM_Cmd(TIM2, ENABLE);
         TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
         NVIC_EnableIRQ(TIM2_IRQn);

         RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
                 /* Connect "echo" EXTI Line to "echo" GPIO Pin */
         SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource11);

                 // Configure "echo" EXTI line
         EXTI_InitStructure.EXTI_Line = EXTI_Line11;
         EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
         EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
         EXTI_InitStructure.EXTI_LineCmd = ENABLE;
         EXTI_Init(&EXTI_InitStructure);
         NVIC_EnableIRQ(EXTI15_10_IRQn);
}



void sonar_trig(){        // send the signal to sonar to measure the distance
        GPIOB->BSRRL |= GPIO_Pin_13;
        vTaskDelay(10);
        GPIOB->BSRRH |= GPIO_Pin_13;
        vTaskDelay(20);
}
int main(void)
{
	int del=0;
    Delay_us(4500);
	Init_lcd();
//	flush_lcd_buffer();
    Lcd_write_str("1");


    Lcd_write_str("2");
        RTC_TimeTypeDef RTC_TimeStructure;
        RTC_DateTypeDef RTC_DateStructure;
        dht_init();
        Lcd_write_str("3");
        nrf24init();
        Lcd_write_str("4");
        buttonsInit();
        Lcd_write_str("5");
        Delay_us(500);
        uint8_t i=0, res=0;
        uint16_t i2;
        Delay_us(50);
        FLASH_Unlock();
        Lcd_write_str("6");
        Delay_us(500);
        EE_Init();
        Delay_us(50);
        Lcd_write_str("7");
        sonar_init();
        Lcd_write_str("8");
        rtc_init();
        Lcd_write_str("9");
         /* Set the date: Sunday April 21st 2013 */
         RTC_DateStructure.RTC_Year = 0x13;
         RTC_DateStructure.RTC_Month = RTC_Month_April;
         RTC_DateStructure.RTC_Date = 0x21;
         RTC_DateStructure.RTC_WeekDay = RTC_Weekday_Sunday;
         RTC_SetDate(RTC_Format_BCD, &RTC_DateStructure);

         /* Set the time to 20h 52mn 00s */
         RTC_TimeStructure.RTC_Hours = 0x20;
         RTC_TimeStructure.RTC_Minutes = 0x52;
         RTC_TimeStructure.RTC_Seconds = 0x00;

         RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure);

        uint16_t        tmpvar=0;
        Lcd_goto(0,0);
        Lcd_write_str("0");

        no_sd = disk_initialize(0);		// init card
        	if (no_sd==0){
        		no_sd = f_mount(0, &fs);
        		no_sd = string2log("System started\n", 15);
        	}
        Lcd_write_str("1");
        Lcd_clear();
        Lcd_write_str("Init OK!");

        buttonCalibration();



// blok proverki zapisi v emulated eeprom
        /*
        EE_WriteVariable(0xF000, 12321);
        EE_ReadVariable(0xF000, &tmpvar);
        Lcd_write_data((tmpvar%100000)/10000+48);
        Lcd_write_data((tmpvar%10000)/1000+48);
        Lcd_write_data((tmpvar%1000)/100+48);
        Lcd_write_data((tmpvar%100)/10+48);
        Lcd_write_data((tmpvar%10)+48); */

        Delay_us(500);

        STM32F4_Discovery_LEDInit(LED6);
        STM32F4_Discovery_LEDInit(LED5);

//        void vTaskDHT(void *pvParameters)
//        void vTaskProgramRunner(void *pvParameters)
        xTaskCreate( vTaskProgramRunner, ( signed char * ) "RUN", configMINIMAL_STACK_SIZE+100, NULL, 2, ( xTaskHandle * ) NULL);
        xTaskCreate( vTaskSDLog, ( signed char * ) "SDLOG", configMINIMAL_STACK_SIZE, NULL, 2, ( xTaskHandle * ) NULL);
        xTaskCreate( vTaskDHT, ( signed char * ) "DHT", configMINIMAL_STACK_SIZE, NULL, 2, ( xTaskHandle * ) NULL);
        xTaskCreate( vTaskReadButtons, ( signed char * ) "BTNS", configMINIMAL_STACK_SIZE, NULL, 2, ( xTaskHandle * ) NULL);
        xTaskCreate( vTaskLED5, ( signed char * ) "LED5", configMINIMAL_STACK_SIZE, NULL, 2, ( xTaskHandle * ) NULL);
        xTaskCreate( vTaskLED6, ( signed char * ) "LED6", configMINIMAL_STACK_SIZE, NULL, 2, ( xTaskHandle * ) NULL);
        xTaskCreate( vTaskLCDdraw, ( signed char * ) "LCD", configMINIMAL_STACK_SIZE+100, NULL, 2, ( xTaskHandle * ) NULL);
        xTaskCreate( vTaskSonar, ( signed char * ) "SONAR", configMINIMAL_STACK_SIZE, NULL, 2, ( xTaskHandle * ) NULL);
        vTaskStartScheduler();
}

void buttonCodeSupplier(uint16_t adc_value){
	uint8_t i=0;
	button_code=0;
	for (i=0;i<4;i++) {
		if (adc_value>button_ranges[i*2] && adc_value<button_ranges[i*2+1]) {
			button_code = i+1;
		}
	}
}

void vTaskReadButtons(void *pvParameters) {
        uint16_t curval = 0, avg=0;
        uint8_t i;
        for (;;) {
                adc_read();
                vTaskDelay(10);
                buttonCodeSupplier(JDR_BUTTONS);
        }
}

extern void TIM2_IRQHandler(void)
{
    capture1 = TIM_GetCapture4(TIM2);
    capture_is_ready = 0;
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);

}

extern void EXTI15_10_IRQHandler(void)
{
        capture2 = TIM2->CNT;
        capture_is_ready = 1;
    EXTI_ClearITPendingBit(EXTI_Line11);
}

uint8_t blinker=0;

nrf_reg_buf reg_buffer;

void nrfTest(void){
	uint8_t resp=0, cmd=0;
	vTaskDelay(200);
	flush_lcd_buffer();
	Lcd_write_str_fb("Send command");
	while (button_code!=BUTTON_CNL){
	//    resp=nrfRxBuff[nrf_rx_pointer];
	    Lcd_go2(0,0);
	    vTaskDelay(20);
	    Lcd_write_str_fb("Send command");
	    Lcd_go2(1,0);
	    vTaskDelay(20);
	    Lcd_write_str_fb("CMD:");
	    Lcd_print_digit(cmd);
	    Lcd_write_str_fb("rs:");
//	    Lcd_print_digit(resp);
	 //   resp = NRF24_SPIx->DR;
	    if (button_code==BUTTON_BCK) {
	  //  	--cmd;
	//    	nrf_read_reg(7, &reg_buffer, 5);
	    	vTaskDelay(5);
            copy_arr(&reg_buffer.data, &LCDLine2, 5, 7);
	    }
	    if (button_code==BUTTON_FWD) {
	    	++cmd;
	    	nrf_read_reg(0, &reg_buffer, 5);
	    	vTaskDelay(5);
            copy_arr(&reg_buffer.data, &LCDLine1, 5, 7);
            if (cmd>10){
            	cmd=0;
            }
	    }

	    if (button_code==BUTTON_OK) {
	 //   	NRF24_SPIx->DR=++cmd;
	 //   	NRF24_CSN_LOW;
	//    	SPI_I2S_ITConfig(NRF24_SPIx, SPI_I2S_IT_TXE, ENABLE);
	    	if (cmd>10){
	    		cmd=0;
	    	}
	    	vTaskDelay(5);
	    }
	    vTaskDelay(10);
	}
}

unsigned char nrf_spi_xfer_byte(unsigned char data){
	SPI_I2S_ITConfig(NRF24_SPIx, SPI_I2S_IT_TXE, ENABLE);
	SPI_I2S_ITConfig(NRF24_SPIx, SPI_I2S_IT_RXNE, ENABLE);
	NRF24_SPIx->DR=data;
	nrf_spi_busy=1;
	while (nrf_spi_busy==1) {
		vTaskDelay(1);
	}
}

int nrf_read_reg(unsigned char reg, nrf_reg_buf *buf, uint8_t plsize)
{
     int i;
     NRF24_CSN_LOW;

     // send command
     nrf_spi_xfer_byte(NRF_CMD_RREG | reg);

     // receive response
     for(i = 0; i < plsize; i++) {
          buf->data[i] = nrf_spi_xfer_byte(NRF_CMD_NOP);
          vTaskDelay(2);
     }

     NRF24_CSN_HIGH;

     buf->size = i;

     return i;
}

int nrf_write_reg(unsigned char reg, nrf_reg_buf *buf, uint8_t plsize)
{
     int i;

     NRF24_CSN_LOW;

     // send command
     nrf_spi_xfer_byte(NRF_CMD_WREG | reg);

     // send payload
     for(i = 0; i < plsize; i++) {
          nrf_spi_xfer_byte(buf->data[i]);
     }

     NRF24_CSN_HIGH;

     return i;
}

void monitorDisplayer(void){
        uint8_t i, prevseconds=0;

        NRF24_CSN_LOW;

		SPI_I2S_ITConfig(NRF24_SPIx, SPI_I2S_IT_TXE, ENABLE);
		vTaskDelay(1);
        Lcd_go2(0,3);
        Lcd_write_digit(NRF24_SPIx->DR);
        vTaskDelay(1);
        Lcd_write_digit(spi_irq_counter);
        NRF24_SPIx->DR=2;


        // --- main monitor mode programm ---
        // display temperature and humidity data
/*    for (i=0;i<4;i++){
      LCDLine1[i] = dht_t_str[i];
    }
    for (i=0;i<4;i++){
      LCDLine1[i+5] = dht_rh_str[i];
    }
         // sonar data
    for (i=0;i<3;i++){
     LCDLine1[i+11] = sonar_str[i];
    } */
    // "mm" for sonar
    for (i=0;i<16;i++) {
          LCDLine2[i]=rtc_time_now_str[i];
 //         vTaskDelay(5);
    }
 /*   LCDLine1[14]=109;
    LCDLine1[15]=109;
         // button code
    LCDLine1[9] = button_code+48;
         // date and time
*/
    RTC_GetTime(RTC_Format_BIN, &curtime);
    RTC_GetDate(RTC_Format_BIN, &curdate);
    if (curtime.RTC_Seconds!=prevseconds) {
            prevseconds=curtime.RTC_Seconds;
            rtc_time_now_str[0] = (curtime.RTC_Hours%100)/10+48;
            rtc_time_now_str[1] = curtime.RTC_Hours%10+48;
            rtc_time_now_str[2] = 58;
            rtc_time_now_str[3] = (curtime.RTC_Minutes%100)/10+48;
            vTaskDelay(1);
            rtc_time_now_str[4] = curtime.RTC_Minutes%10+48;
            rtc_time_now_str[5] = 58;
            rtc_time_now_str[6] = (curtime.RTC_Seconds%100)/10+48;
            rtc_time_now_str[7] = curtime.RTC_Seconds%10+48;
            vTaskDelay(1);
            rtc_time_now_str[8] = (curdate.RTC_Date%100)/10+48;
            rtc_time_now_str[9] = curdate.RTC_Date%10+48;
            rtc_time_now_str[10] = 45;
            rtc_time_now_str[11] = (curdate.RTC_Month%100)/10+48;
            vTaskDelay(1);
            rtc_time_now_str[12] = curdate.RTC_Month%10+48;
            rtc_time_now_str[13] = 45;
            rtc_time_now_str[14] = (curdate.RTC_Year%100)/10+48;
            rtc_time_now_str[15] = curdate.RTC_Year%10+48;
     }

}

void vTaskProgramRunner(void *pvParameters){
        uint8_t prevseconds = 0, i;
        while (1){

                monitorDisplayer();	// default foreground process

                if (button_code==BUTTON_OK)	// enter menu tree
                {
                        Lcd_clear();
                        vTaskDelay(100);
                        uint8_t progId=menuSelector();
                        programRunner(progId);
                        Lcd_clear();
                        vTaskDelay(100);
                }
                vTaskDelay(10);
        }
}

void vTaskLCDdraw(void *pvParameters) {        // draws lcd
        uint32_t tmp=0;
        uint8_t button=0, i=0;
//        RTC_TimeTypeDef curtime;
//        RTC_TimeStructInit(&curtime);
        for (;;) {
//                Lcd_write_cmd(Display_clear);
//                adc_read();

                if (i>99) {
                	i=0;
                }
                else {
                	i++;
                }
                vTaskDelay(5);
                vTaskSuspendAll();
   //             flush_lcd_buffer();
            //    Lcd_write_str("A");
                Lcd_write_cmd(0x80);	// lcd_goto(0,0)
                Lcd_write_arr(&LCDLine1, 16);
       //         xTaskResumeAll();
                //Lcd_goto(1,0);
           //     vTaskDelay(5);
      //          Delay_us(5);
        //        vTaskSuspendAll();
                Lcd_write_cmd(0x80+0x40);	// lcd_goto(1,0)
                Lcd_write_arr(&LCDLine2, 16);
                xTaskResumeAll();

                vTaskDelay(5);
                // 26.06.2013
//                button=readButtons();
        }
}



uint16_t del;
void vTaskLED5(void *pvParameters) {
        for (;;) {
                STM32F4_Discovery_LEDOn(LED5);
                vTaskDelay(5);
                STM32F4_Discovery_LEDOff(LED5);
                del = 65-sonar_value/1000;
                vTaskDelay(del);
        }
}

void vTaskLED6(void *pvParameters) {
        uint8_t i=0, prevseconds=0;
        for (;;) {
                STM32F4_Discovery_LEDOn(LED6);
                vTaskDelay(2);
                STM32F4_Discovery_LEDOff(LED6);
                vTaskDelay(2);
                RTC_GetTime(RTC_Format_BIN, &curtime);
                RTC_GetDate(RTC_Format_BIN, &curdate);
                vTaskDelay(5);

                vTaskDelay(5);
        }
}

uint16_t uint16_time_diff(uint16_t now, uint16_t before)
{
  return (now >= before) ? (now - before) : (UINT16_MAX - before + now);
}

void vTaskDHT(void *pvParameters){
        uint8_t i=0;
        dht_bit_position=0;
        while (1) {
                dht_get_data();
                vTaskDelay(30);
        }
}

void vTaskSDLog(void *pvParameters){
        while (1){
                vTaskDelay(200);
                logString[0] = (curtime.RTC_Hours%100)/10+48;
                logString[1] = curtime.RTC_Hours%10+48;
                logString[2] = 58;
                logString[3] = (curtime.RTC_Minutes%100)/10+48;
                vTaskDelay(5);
                logString[4] = curtime.RTC_Minutes%10+48;
                logString[5] = 58;
                logString[6] = (curtime.RTC_Seconds%100)/10+48;
                logString[7] = curtime.RTC_Seconds%10+48;
                logString[8] = 32;
                vTaskDelay(5);
                logString[9] = (curdate.RTC_Date%100)/10+48;
                logString[10] = curdate.RTC_Date%10+48;
                logString[11] = 45;
                logString[12] = (curdate.RTC_Month%100)/10+48;
                vTaskDelay(5);
                logString[13] = curdate.RTC_Month%10+48;
                logString[14] = 45;
                logString[15] = (curdate.RTC_Year%100)/10+48;
                logString[16] = curdate.RTC_Year%10+48;
                logString[17] = 32;
                copy_arr(&sonar_str, &logString, 5, 18);
                logString[23] = 32;
                copy_arr(&dht_rh_str, &logString, 4, 24);
                logString[28] = 32;
                copy_arr(&dht_t_str, &logString, 4, 29);
         logString[33] = 10;                // LF perenos stroki
         string2log(&logString, 34);
        }
}

void vTaskSonar(void *pvParameters){
        uint8_t        i=0;
        for (;;) {
                 sonar_trig();
                 vTaskDelay(10);
         sonar_value = (65535-uint16_time_diff(capture1,capture2))/80; // get millimeters
         sonar_str[0] = (sonar_value%1000)/100+48;
         sonar_str[1] = (sonar_value%100)/10+48;
         sonar_str[2] = (sonar_value%10)+48;
         sonar_str[3] = 32;
         vTaskDelay(10);
        }

}

void copy_arr(uint8_t *source, uint8_t *destination, uint8_t amount, uint8_t pos){
        int i=0;
        for (i=0; i<amount;i++) {
                destination[i+pos] = source[i];
        }
}


FRESULT string2log(char* str, int amount){
        uint8_t res=0; //Ã�Â´Ã�Â»Ã‘ï¿½ Ã�Â²Ã�Â¾Ã�Â·Ã�Â²Ã‘â‚¬Ã�Â°Ã‘â€°Ã�Â°Ã�ÂµÃ�Â¼Ã�Â¾Ã�Â³Ã�Â¾ Ã‘â€žÃ‘Æ’Ã�Â½Ã�ÂºÃ‘â€ Ã�Â¸Ã‘ï¿½Ã�Â¼Ã�Â¸ Ã‘â‚¬Ã�ÂµÃ�Â·Ã‘Æ’Ã�Â»Ã‘Å’Ã‘â€šÃ�Â°Ã‘â€šÃ�Â°
        UINT len; //Ã�Â´Ã�Â»Ã‘ï¿½ Ã‘â€¦Ã‘â‚¬Ã�Â°Ã�Â½Ã�ÂµÃ�Â½Ã�Â¸Ã‘ï¿½ Ã�ÂºÃ�Â¾Ã�Â»Ã�Â¸Ã‘â€¡Ã�ÂµÃ‘ï¿½Ã‘â€šÃ�Â²Ã�Â° Ã‘â‚¬Ã�ÂµÃ�Â°Ã�Â»Ã‘Å’Ã�Â½Ã�Â¾ Ã�Â·Ã�Â°Ã�Â¿Ã�Â¸Ã‘ï¿½Ã�Â°Ã�Â½Ã�Â½Ã‘â€¹Ã‘â€¦ Ã�Â±Ã�Â°Ã�Â¹Ã‘â€š
        f_write(&cadizlog, str, amount, &len);
        f_sync(&cadizlog);
        return res;
}



void loadSettings(){        //
        uint16_t i, Address, Data;
        uint32_t tmp;
        char tmpstr[11], putstring[50];
//        char *massiv[];
        for (i=0; i<PLUG_AMOUNT; i++) {
                Address=EE_PLUG_SETTINGS+i;
                EE_ReadVariable(Address, &Data);
                plugSettings[i]=Data;
                if (no_sd>1) {
                        string2log("plug no.", 8);
                        int32str(i,&tmpstr);
                        string2log(tmpstr, 11);
                        string2log(" has timer no.", 14);
                        int32str(plugSettings[i],&tmpstr);
                        tmpstr[10] = 0x0A;
                        string2log(tmpstr, 11);
                }
        }
        Address=LIGHT_RANGE_ADDR;
        EE_ReadVariable(Address, &lightRange);
        EE_ReadVariable(PH4_ADDR, &ph4);

//        Address=PH7_ADDR;
        EE_ReadVariable(PH7_ADDR, &ph_seven);
        cdel = (ph_seven - ph4)/3;
        ph0 = ph4 - cdel*4;

        if (no_sd>0) {
                // vnesti v log znachenie kalibrovshika ph4
                int32str(ph4,&tmpstr);
                for (i=0;i<10;i++){
                        putstring[i]=tmpstr[i];
                }
                putstring[10] = 0x0A;
                string2log("ph4 cal @ ", 10);
                string2log(putstring, 11);

                // vnesti v log znachenie kalibrovshika ph7
                int32str(ph_seven,&tmpstr);
                for (i=0;i<10;i++){
                        putstring[i]=tmpstr[i];
                }
                putstring[10] = 0x0A;
                string2log("ph7 cal @ ", 10);
                string2log(putstring, 11);

                // vnesti v log raschitayj uroven' ph0
                int32str(ph0,&tmpstr);
                for (i=0;i<10;i++){
                        putstring[i]=tmpstr[i];
                }
                putstring[14] = 10;
                string2log("ph0 counted @ ", 14);
                string2log(putstring, 11);

                // vnesti v log raschitanuju cenu delenija
                int32str(cdel,&tmpstr);
                for (i=0;i<10;i++){
                        putstring[i]=tmpstr[i];
                }
                putstring[14] = 10;
                string2log("cena delenija ", 14);
                string2log(putstring, 11);
        }

        EE_ReadVariable(SD_LOG_INTERVAL, &logInterval);
}


char* int32str(uint32_t d, char* out)
{
    out[10] = '\0';
    out[9] = '0' + ( d ) % 10;
    out[8] = '0' + ( d /= 10 ) % 10;
    out[7] = '0' + ( d /= 10 ) % 10;
    out[6] = '0' + ( d /= 10 ) % 10;
    out[5] = '0' + ( d /= 10 ) % 10;
    out[4] = '0' + ( d /= 10 ) % 10;
    out[3] = '0' + ( d /= 10 ) % 10;
    out[2] = '0' + ( d /= 10 ) % 10;
    out[1] = '0' + ( d /= 10 ) % 10;
    out[0] = '0' + ( d / 10 ) % 10;
    return out;
}

void buttonCalibration(void){        // buttons calibration function
		readButtonRanges();
        uint16_t button_val[4], diff;
        Lcd_goto(0,0);
        Lcd_write_arr("Press OK to skip", 16);
        Lcd_goto(1,0);
        Lcd_write_arr("calibration", 11);
        Delay_us(1500);
        adc_read();
		Delay_us(100);
        buttonCodeSupplier(JDR_BUTTONS);
        if (button_code!=BUTTON_OK) {
			Lcd_clear();
			Lcd_write_arr("<", 1);
			Delay_us(30000);
			adc_read();
			Delay_us(100);
			button_val[0] = JDR_BUTTONS;
			Lcd_goto(0,0);
			Lcd_write_arr("OK", 2);
			Delay_us(30000);
			adc_read();
			Delay_us(100);
			button_val[1] = JDR_BUTTONS;
			Lcd_goto(0,0);
			Lcd_write_arr("CANCEL", 6);
			Delay_us(30000);
			adc_read();
			Delay_us(100);
			Lcd_clear();
			button_val[2] = JDR_BUTTONS;
			Lcd_write_arr(">", 1);
			Delay_us(30000);
			adc_read();
			Delay_us(100);
			button_val[3] = JDR_BUTTONS;

			if ((button_val[3]>>3)<(button_val[0]>>3)) {
					buttonReverse = 1;
			}
			else if ((button_val[3]>>3)>(button_val[0]>>3)) {
					buttonReverse = 0;
			}
			else {
					// loadButtonSettings();        // no key pressed, loading settings from EEPROM
					buttonReverse = 2;        //means loading button settings from EEPROM
			}
			if (buttonReverse == 0) {
					diff = ((button_val[1]-button_val[0])/2)-5;
					button_ranges[0] = button_val[0]-diff;
					button_ranges[1] = button_val[0]+diff;
					button_ranges[2] = button_val[1]-diff;
					diff = ((button_val[2]-button_val[1])/2)-5;
					button_ranges[3] = button_val[1]+diff;
					button_ranges[4] = button_val[2]-diff;
					diff = ((button_val[3]-button_val[2])/2)-5;
					button_ranges[5] = button_val[2]+diff;
					button_ranges[6] = button_val[3]-diff;
					button_ranges[7] = button_val[3]+diff;
					saveButtonRanges();
			}
			else if (buttonReverse == 1) {
					diff = ((button_val[0]-button_val[1])/2)-5;
					button_ranges[0] = button_val[0]-diff;
					button_ranges[1] = button_val[0]+diff;
					button_ranges[2] = button_val[1]-diff;
					diff = ((button_val[1]-button_val[2])/2)-5;
					button_ranges[3] = button_val[1]+diff;
					button_ranges[4] = button_val[2]-diff;
					diff = ((button_val[2]-button_val[3])/2)-5;
					button_ranges[5] = button_val[2]+diff;
					button_ranges[6] = button_val[3]-diff;
					button_ranges[7] = button_val[3]+diff;
					saveButtonRanges();
					Lcd_goto(0,0);
					Lcd_write_str("Complete");
					Delay_us(10000);
			}
			else {
					Lcd_clear();
					Lcd_write_str("EEPROM buttons");
					readButtonRanges();
					Delay_us(10000);
					Lcd_clear();
			}

			if (button_ranges[0]>button_ranges[8]) {
					buttonReverse=1;
			}
        }
}

void readButtonRanges(void){
        uint8_t i;
        for (i=0; i<8; i++) {
                EE_ReadVariable(BUTTON_RANGES_START_ADDR+i, &button_ranges[i]);
        }
}

void saveButtonRanges(void){
        uint8_t i;
        for (i=0; i<8; i++) {
                EE_WriteVariable(BUTTON_RANGES_START_ADDR+i, button_ranges[i]);
        }
}


void displayAdcValues(void){
        uint8_t button=0;
        Lcd_clear();
        while (button!=5){
                button=readButtons();
                Lcd_goto(0,0);
                Lcd_write_str("1:");
                Lcd_write_digit(ADC1->JDR1/100);
                Lcd_write_digit(ADC1->JDR1);
                Lcd_write_str(" 2:");
                Lcd_write_digit(ADC1->JDR2/100);
                Lcd_write_digit(ADC1->JDR2);
                Lcd_goto(1,0);
                Lcd_write_str("3:");
                Lcd_write_digit(ADC1->JDR3/100);
                Lcd_write_digit(ADC1->JDR3);
                Lcd_write_str(" 4:");
                Lcd_write_digit(ADC1->JDR4/100);
                Lcd_write_digit(ADC1->JDR4);
                vTaskDelay(20);
        }
        Lcd_clear();
}

void buttonCalibration2(void){        // buttons calibration function
        uint8_t i=0;
        uint16_t button_val[4], diff, delay;
        Lcd_clear();
        Lcd_goto(0,0);
        Lcd_write_str("Press button 1");
        Delay_us(30000);
        adc_read();
        Delay_us(100);
        button_val[0] = ADC1->JDR1;
        Lcd_goto(0,0);
        Lcd_write_str("Press button 2");
        Delay_us(30000);
        adc_read();
        Delay_us(100);
        button_val[1] = ADC1->JDR1;
        diff = ((button_val[1]-button_val[0])/2)-5;
        button_ranges[0] = button_val[0]-diff;
        button_ranges[1] = button_val[0]+diff;
        button_ranges[2] = button_val[1]-diff;
        Lcd_goto(0,0);
        Lcd_write_str("Press button 3");
        Delay_us(30000);
        adc_read();
        Delay_us(100);
        button_val[2] = ADC1->JDR1;
        diff = ((button_val[2]-button_val[1])/2)-5;
        button_ranges[3] = button_val[1]+diff;
        button_ranges[4] = button_val[2]-diff;
        Lcd_goto(0,0);
        Lcd_write_str("Press button 4");
        Delay_us(30000);
        adc_read();
        Delay_us(100);
        button_val[3] = ADC1->JDR1;
        diff = ((button_val[3]-button_val[2])/2)-5;
        button_ranges[5] = button_val[2]+diff;
        button_ranges[6] = button_val[3]-diff;
        button_ranges[7] = button_val[3]+diff;
        Lcd_goto(0,0);
        Lcd_write_str("Calib. Complete");
}


uint8_t readButtons(void){
        uint16_t curval = 0;
        uint8_t i;
//        adc_read();
//        Delay_us(100);
        curval = button_code;
        for (i=0;i<4;i++) {
                if (curval>button_ranges[i*2] && curval<button_ranges[i*2+1]) {
                        return i+1;
                }
        }
        return 0;
}

uint32_t EE_ReadWord(uint16_t Address){
        uint16_t tmp, tmp2;
        uint32_t Data = 0;
        EE_ReadVariable(Address, &tmp);
        EE_ReadVariable(Address+1, &tmp2);
        Data = ((uint32_t)tmp << 16) + (uint32_t)tmp2;
        return Data;
}


SPI1_IRQHandler(void) {
	if (spi_irq_counter>99) {
		spi_irq_counter=0;
	}
	else {
		spi_irq_counter++;
	}
	if (NRF24_SPIx->SR & SPI_SR_TXE) {
		NRF24_SPIx->DR=nrfRxBuff[nrf_tx_pointer++];

			   /* Disable the Tx buffer empty interrupt */
		SPI_I2S_ITConfig(NRF24_SPIx, SPI_I2S_IT_TXE, DISABLE);
		if (nrf_tx_pointer>(NRF24_TX_BUFF_SIZE-1)) {
			nrf_tx_pointer=0;
		}
		nrf_spi_busy=0;

	}
	if ((NRF24_SPIx->SR & SPI_I2S_FLAG_RXNE) != RESET) {
		SPI_I2S_ITConfig(NRF24_SPIx, SPI_I2S_IT_RXNE, DISABLE);
		nrfRxBuff[nrf_rx_pointer++]=NRF24_SPIx->DR;
		NRF24_CSN_HIGH;
		if (nrf_rx_pointer>(NRF24_RX_BUFF_SIZE-1)) {
			nrf_rx_pointer=0;
		}
		nrf_spi_busy=0;
	}
	nrf_spi_busy=0;
}
