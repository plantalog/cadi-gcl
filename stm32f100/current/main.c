/*
 *  Use this code free of charge, but leave this text box here, 
 *  This code is distributed "as is" with no warranties.
 *  https://github.com/plantalog/ is main repository hub for Cadi project.
 *
 *	27.08.2013 changed the virtaddvartab usage to copying the whole row or variables 0x05C0-0x0679
 *
 *
 */

#include "stm32f10x_it.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32f10x_flash.h"
#include "eeprom.h"
#include "ff9a/src/diskio.h"
#include "ff9a/src/ff.h"
#include "driver_sonar.h"


// enabling test mode adds some test functionality
#define	TEST_MODE

#define PSI_STAB_ENABLE
#define TANK_STAB_ENABLE

// SENSOR AND CONTROL DEVICES DRIVERS
/*
 * Peripherals used with Cadi should be properly initialized before use.
 * Typical init has:
 * - Pin config (except integrated peripherals like RTC or Flash)
 * - Driver variable's pool definitions
 * - #defines for names used
 * Some drivers need interfacing and/or data processing functions to
 * be implemented
 */


// DRIVER: WATER FLOW METER
#define USE_WFM
#ifdef USE_WFM		//	START WFM DEFINITIONS
#define WFM_AMOUNT	4	// number of meters to work with
uint32_t water_counter[WFM_AMOUNT];
#define WFM_PINS_PB0_1	// WFM assigned to pins PB0 and PB1
#endif				//	EOF WFM DEFINITIONS


// DRIVER: Spherical valves with feedback
#define USE_VALVES
#ifdef USE_VALVES	// START VALVES DEFINITIONS
#define VALVE_DISABLE	GPIOA->BSRR
#define VALVE_ENABLE	GPIOA->BRR
#define VALVE_SENSOR_GPIO_SHIFT		4		// valve position sensor GPIO shift
#define VALVE_AMOUNT				4		// number of valves to process
#define VALVE_MOTOR_GPIO_SHIFT		11		// valve control motor GPIO out shift
#define VALVE_MOTOR_PORT			GPIOA
#define VALVE_SENSOR_PORT			GPIOA
#define VALVE_SENSOR_PORT_SOURCE	GPIO_PortSourceGPIOA
#define	VALVE_FAILURE_TIMEOUT		600	// timeout for valve open/close function to avoid hanging if valve broken
#define DRAIN_VALVE_ID				1
// Valve variables
uint8_t valveFlags;
uint8_t valve_failed;
uint8_t valve_busy=0;
#endif	// EOF VALVES DEFINITIONS


// DRIVER: Bluetooth module HC-06 USART
#define USE_BLUETOOTH
#ifdef USE_BLUETOOTH		// START BLUETOOTH USART DEFINITIONS
#define BT_USART	USART1
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USARTx_IRQHandler   USART1_IRQHandler
//#define TxBufferSize   (countof(TxBuffer) - 1)
//#define RxBufferSize   0x10

/* Private macro -------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))

/* Private variables ---------------------------------------------------------*/
// uint8_t TxBuffer[] = "\n\rCadi sends HELLO!\n\r";
#define PRODUCTION_BUILD
#ifndef PRODUCTION_BUILD
uint8_t RxBuffer[16];
#endif
#ifdef PRODUCTION_BUILD
uint8_t TxBuffer[40];
#endif
#ifndef PRODUCTION_BUILD
uint8_t TxBuffer[24];
#endif
uint8_t RxByte;
//uint8_t TxByte;
uint8_t comm_state=48;	// communication state
//uint8_t TxDataReady;
//uint8_t RxDataReady;
uint8_t NbrOfDataToTransfer = 16;
//uint8_t NbrOfDataToRead = RxBufferSize
uint8_t txbuff_ne = 0;
uint8_t TxCounter = 0;
uint8_t RxCounter = 0;
uint8_t logstr_crc=0;
#endif				// EOF BLUETOOTH USART DEFINITIONS


// DRIVER: Digital Temperature and Humidity sensor DHT22

#define USE_DHT
#ifdef USE_DHT			// START DHT DEFINITIONS
#define DHT_TRIG_PLUG			15		// PB15
#define DHT_DATA_START_POINTER	4		// DHT22 = 4, DHT11 = ?; sets the first bit number in captured sequence of DHT response bits
#define DHT_0					(GPIOB->BRR = (1<<DHT_TRIG_PLUG))
#define DHT_1					(GPIOB->BSRR = (1<<DHT_TRIG_PLUG))
typedef struct
{
  uint16_t 			DHT_Temperature;        // temperature
  uint16_t 			DHT_Humidity;  // from 0 (0%) to 1000 (100%)
  uint8_t 			DHT_CRC;  // from 0 (0%) to 1000 (100%)
}DHT_data;
__IO uint16_t IC2Value = 0;
__IO uint16_t DutyCycle = 0;
__IO uint32_t Frequency = 0;
uint8_t dht_shifter=DHT_DATA_START_POINTER;		// could be removed in production version
TIM_ICInitTypeDef  TIM_ICInitStructure;
// digital humidity and temperature data
//uint8_t dht_bit_array[50];
DHT_data DHTValue;
uint8_t		dht_data[5];
uint8_t		dht_data2[5];
uint8_t dht_bit_position = 0;
uint8_t dht_bit_ready = 0;
uint8_t	dht_data_ready = 0;
uint8_t  dht_byte_pointer;
uint8_t  dht_bit_pointer;
uint8_t 	dht_rh_str[4], dht_t_str[4];
uint16_t rhWindowTop, rhWindowBottom;
uint8_t rhUnderOver = 0;
#endif						// EOF DHT DEFINITIONS



#define CADI_MB
#define USE_LCD



#ifdef CADI_MB
#define lcd_shift	11				// seems to be not used here anymore?
#define use_gpio	GPIO_Pin_13		// last data pin number
#define pin_d7		use_gpio		// define pins from last
#define pin_d6		use_gpio>>1
#define pin_d5		use_gpio>>2
#define pin_d4		use_gpio>>3		// to d4 of 4bit bus of 1602 LCD

#define d7_0		GPIOA->BSRRH |= (GPIO_Pin_15);
#define d7_1		GPIOA->BSRRL |= (GPIO_Pin_15);
#define d6_0		GPIOC->BSRRH |= (GPIO_Pin_10);
#define d6_1		GPIOC->BSRRL |= (GPIO_Pin_10);
#define d5_0		GPIOC->BSRRH |= (GPIO_Pin_11);
#define d5_1		GPIOC->BSRRL |= (GPIO_Pin_11);
#define d4_0		GPIOC->BSRRH |= (GPIO_Pin_12);
#define d4_1		GPIOC->BSRRL |= (GPIO_Pin_12);

#define e_1 	GPIOD->ODR |=  GPIO_Pin_2
#define e_0		GPIOD->ODR &=~ GPIO_Pin_2
#define rw_1	GPIOB->ODR |=  GPIO_Pin_3
#define rw_0	GPIOB->ODR &=~ GPIO_Pin_3
#define rs_1	GPIOB->ODR |=  GPIO_Pin_4
#define rs_0	GPIOB->ODR &=~ GPIO_Pin_4



#endif




// #define PAPA_EDITION
#ifdef PAPA_EDITION
#define lcd_init_port_data			RCC_APB2Periph_GPIOB
#define lcd_init_port_cmd			RCC_APB2Periph_GPIOB
#define pin_e 					GPIO_Pin_7
#define pin_rw					GPIO_Pin_8
#define pin_rs					GPIO_Pin_9
#define lcd_port_data			GPIOB
#define lcd_port_cmd			GPIOB
#endif

#ifndef PAPA_EDITION
#define lcd_init_port_data			RCC_APB2Periph_GPIOB
#define lcd_init_port_cmd			RCC_APB2Periph_GPIOC
#define pin_e 					GPIO_Pin_10
#define pin_rw					GPIO_Pin_11
#define pin_rs					GPIO_Pin_12
#define lcd_port_data			GPIOB
#define lcd_port_cmd			GPIOC
#endif



#define Function_set 				0b00100000//4-bit,2 - line mode, 5*8 dots
#define Display_on_off_control		0b00001100/// display on,cursor off,blink off
#define Display_clear				0b00000001
#define Entry_mode_set				0b00000100//







u32 del;
static uint8_t		LCDLine1[16], LCDLine2[16];		// lcd frame buffer
uint8_t lcd_pointerx=0, lcd_pointery=0;


// DRIVER: Analog buttons
#define USE_BUTTONS
#ifdef USE_BUTTONS			// START BUTTONS DEFINITIONS
#define BUTTON_OK				2
#define BUTTON_CNL				3
#define BUTTON_BCK				1
#define BUTTON_FWD				4
#define BUTTON_RANGE_SHRINKER	30
uint16_t button_ranges[8];	// 0,2,4,6 - lower, 1,3,5,7 - higher values for buttons
uint8_t buttonReverse=0;
#endif						// EOF BUTTONS DEFINITIONS


#ifdef CADI_MB
#define JDR_BUTTONS	ADC1->JDR1		// continuous ADC channel for buttons
#define ADC_AVG_BUTTONS		0	// N=ADC_AVG_BUTTONS, for adcAverage[N]
#endif

#ifndef CADI_MB
#define ADC_AVG_BUTTONS		3
#define JDR_BUTTONS	ADC1->JDR4		// continuous ADC channel for buttons
#endif



// DRIVER:  RTC
#define USE_RTC
#ifdef USE_RTC				// START RTC DEFINITIONS
#define YEAR12SECS		1325462400	// vse raschety vedjom ot 01.01.2012
typedef struct
{
  unsigned char  hour;
  unsigned char  min;
  unsigned char  sec;
} RTC_Time;

typedef struct
{
  uint16_t  hour;
  uint16_t  min;
  uint16_t  sec;
  uint16_t  day;
  uint16_t  month;
  uint16_t  year;
  uint16_t  doy;	   //day of year

} RTC_DateTime;

RTC_Time toAdjust;
uint8_t auto_flags=254;	// enable all except the 0 bit - psi stab
uint32_t timerStateFlags, cTimerStateFlags;
#endif						// EOF RTC DEFINITIONS


// DRIVER: LOAD TRIGGERING
#define USE_LOADS
#ifdef USE_LOADS			// START LOADS DEFINITIONS
#define PLUG_DISABLE	GPIOC->BSRR
#define PLUG_ENABLE		GPIOC->BRR
#define	PLUG_INVERT		0		// enable reverse plugStateSet
#define PLUG_AMOUNT		4
uint32_t plugStateFlags;
uint8_t plugSettings[PLUG_AMOUNT] = {0, 1, 2, 3};	// PLUG_AMOUNT - number of plugs HARDCODE
// elementy massiva - nomera programm (ex "tajmerov"), sootvetstvujushih Plug'am.
// 0 element - pervyj plug, 1 element - plug no 2, etc
#endif						// EOF LOADS DEFINITIONS


// DRIVER: SD-card on SPI interface with FatFS
#define USE_SD
#ifdef USE_SD
	int no_sd;
	FATFS   fs;       // file system variable to mount SD card
	uint16_t logInterval=0;
#endif

/*
 *   =====  END OF DRIVER SECTION =====
 */




// define eeprom cells for keeping user settings. memory map
#define EE_PLUG_SETTINGS	0x05FF	//	each plug one value (16 plugs in total, range 7FF-809)
#define EE_TIMER1_ON		0x05DA	// for 4 timers 36 (hex=24) values (range: 7DA-7FF)
#define EE_CTIMER_DURATION	0x05C0	// 2 values for duration of cTimer.
#define EE_CTIMER_INTERVAL	0x05C2	// and 2 for interval. For 5 timers 25 (hex=19) values 7C0-7D9
#define EE_TIMER_SIZE		9
#define EE_CTIMER_SIZE		5
#define PH4_ADDR			0x0600
#define PH7_ADDR			0x0601
#define PH_INTERVAL			0x0602		// pH measurement interval in milliseconds
#define PH_BUFF_SIZE		0x0603		// pH buffer size
#define PH_WINDOW_TOP		0x0604		// pH window top adc value
#define PH_WINDOW_BOTTOM	0x0605	// pH window bottom adc value
#define SD_LOG_INTERVAL		0x0609		// sd logging interval, seconds
#define BUTTON_RANGES_START_ADDR	0x060A	// button ranges (8 values in a row) [60A-611]

#define EC1413_ADDR			0x0606
#define EC0_ADDR			0x067E	// after FMPs

#define RH_WINDOW_TOP		0x0607		// pH window top adc value
#define RH_WINDOW_BOTTOM	0x0608	// pH window bottom adc value

#define CIRCULATION_PUMP_ID_ADDR	0x0899		// seems not to be used due to overriding with timerId=70


// WATERING PROGRAMS SETTINGS ADRESSES
#define WP_AMOUNT					3		// 3x16=48(hex=30) values (range: 613-643)
#define WP_SIZE						16		// size of block of settings data of watering program
#define WP_OFFSET					0x0613	// watering program settings offset
#define TOP_WATER_SENSOR_SHIFT		1		// water sensor address offset inside watering program
#define BOTTOM_WATER_SENSOR_SHIFT	2		// water sensor address offset inside watering program
#define WATER_FILL_TIMEOUT_SHIFT	3		// if no sensor reached timout for water tank fill valve
#define WP_DURATION_SHIFT			4		// ready solution watering duration
#define WP_INTERVAL_SHIFT			5		// 2x16bit variables for program execution interval
#define WP_START_TIME_SHIFT			7		// 2x16bit variables for program start time
#define WP_LAST_RUN_SHIFT			9		// 2x16bit last run of watering program
// #define WP_CIRCULATION_PUMP_PLUG_SHIFT		11		// circulation pump plug nr
#define WP_WATERING_PUMP_PLUG_ID	11
#define WP_ENABLE_SHIFT				12
#define WP_END_TIME_SHIFT			13
#define WP_DRAIN_DURATION_SHIFT		15



// 9x7=63 (hex=39) 0x0644 - 0x067D

#define FMP_PROGRAMS_AMOUNT			9

// Fertilizer Mixing Program adresses	()
#define FMP_OFFSET								0x0644
#define FMP_SIZE								7
#define FMP_DOSING_PUMP_ID_SHIFT				1
#define FMP_DOSING_TIME_SHIFT					2
#define FMP_CIRCULATION_MIXING_TIME_SHIFT		3
#define FMP_2_WP_ASSOC_SHIFT					4
#define FMP_TRIG_FREQUENCY_SHIFT				5
#define FMP_ENABLE								6

#define COMM_MONITOR_MODE		48
#define COMM_GET_SETTINGS		49
#define COMM_SET_SETTINGS		50
#define COMM_DIRECT_DRIVE		51

// two following defines moved to eeprom.h
//#define SETTINGS_PACKET_SIZE	200	// look NumbOfVar define in eeprom.h
//#define SETTINGS_START_ADDR		0x05C0	// HARDCODED in eeprom.c look for this address

#define WATER_TANK_TOP			0x0622
#define WATER_TANK_BOTTOM		0x0632
#define WFM_CAL_OFFSET			0x0642
#define WATER_TANK_SUPPLY_VALVE	0x0644

#define DAY						1
#define NIGHT					0

#define BSRRL					BSRR
#define BSRRH					BRR


#define LOG_SHIFT	1

// analog inputs
#define JDR_EC		ADC1->JDR3		// continuous ADC channel for EC
#define JDR_PH		ADC1->JDR2		// continuous ADC channel for pH
#define JDR_PSI		ADC1->JDR3		// pressure sensor
#define JDR_BUFFER_SIZE 10


#define PSI_SENSOR_0PSI			0x0645
#define PSI_SENSOR_32PSI		0x0646
#define PSI_SENSOR_TOP			0x0647
#define PSI_SENSOR_BTM			0x0648
#define PSI_PUMP_LOAD_ID		0x0649
uint8_t psi_pump_load_id=0;
uint8_t psi_underOver=0;
uint16_t psi_pump_top_level=0;
uint16_t psi_pump_btm_level=0;
uint16_t psi_cur_adc_value=0;

#define AVG_ADC_EC		3			// adcAverage[AVG_ADC_EC]
#define AVG_ADC_PH		2			// adcAverage[2]
#define AVG_ADC_PSI		2			// adcAverage[2]


uint16_t tank_windows_top[1];
uint16_t tank_windows_bottom[1];

char log_str[64];


uint8_t circulationPumpId;
ErrorStatus  HSEStartUpStatus;
FLASH_Status FlashStatus;
uint16_t ph_seven=0;
uint32_t ph0=0;
uint16_t ph4=0;
uint16_t ec1413 = 0, ec0 = 0;
int cdel=0;

uint16_t adcAverage[4];
uint16_t jdrBuff1[JDR_BUFFER_SIZE];
uint16_t jdrBuff2[JDR_BUFFER_SIZE];
uint16_t jdrBuff3[JDR_BUFFER_SIZE];
uint16_t jdrBuff4[JDR_BUFFER_SIZE];
uint8_t curphstr[5];
uint8_t curecstr[5];

//uint16_t ecAdcValue;
uint8_t currentEc=0;
uint8_t currentPh=0;
uint8_t phUnderOver = 0;	// 0 - pH value is within window, 1 - under window, 2 - over window
uint8_t ecUnderOver = 0;	// 0 - pH value is within window, 1 - under window, 2 - over window
uint16_t phWindowTop, phWindowBottom;
uint16_t ecWindowTop, ecWindowBottom;
uint32_t lastWriteTime=0;
uint8_t wpStateFlags;
uint8_t dosingPumpStateFlags;	// this variable seems to be overwritten by some part of this firmware, therefore dosingPumpStateFlags2 used instead
uint8_t dosingPumpStateFlags2;
uint8_t waterSensorStateFlags;
// uint8_t wsl_buff[3];
uint16_t wfCalArray[WFM_AMOUNT];


#define STATE_NOTHING					0
#define STATE_CONFIGURATION_PAYLOAD		1
#define STATE_CONFIGURATION_CRC			2
#define STATE_COMMAND_TYPE				3
#define STATE_COMMAND_PAYLOAD			4
#define STATE_COMMAND_CRC				5


#define RXM_NONE						0
#define RXM_CMD							4
#define RXM_SET							3
uint8_t cmdbuf[16];		// command packet buffer
uint8_t cmdbufp=0;		// pointer
uint8_t prefixDetectionIdx=0;
uint8_t pb_pntr=0;			// packet buffer pointer
uint8_t packet_length=0;
uint8_t rxm_state=0;
uint8_t packet_ready=0;		// packet readiness flag. reset after command execution
uint8_t rx_packet_crc = 0;	// cmdbuf and ZXn packet header CRC

void hygroStatSettings(void);
uint8_t readPercentVal(uint8_t value);
uint32_t measureDelay(void);
void phMonSettings(void);
void setTimer(uint8_t timerId);
//void copy_arr(uint8_t *source, uint8_t *destination, uint8_t amount, uint8_t pos);
void copy_arr(uint8_t *source, uint8_t *destination, uint8_t amount, uint8_t pos);
void Lcd_write_arr(uc8 *STRING, uint8_t chars);
void Lcd_write_digit(uint8_t numb);
void Delay_us(uint32_t delay);
void buttonCalibration(void);
void Init_pin_out(void);
void Init_pin_in(void);
void Init_lcd(void);
void Lcd_write_data(uint8_t byte);
void Lcd_write_cmd(uint8_t byte);
void Lcd_clear(void);
void Return_home(void);
void Lcd_goto(uc8 x, uc8 y);
void Lcd_write_str(char *STRING);
char* adc2str(uint_fast16_t d, char* out);
void int32str(uint32_t d, char *out);
void AdcInit(void);
uint32_t RTC_GetCounter(void);
void RTC_SetCounter(uint32_t value);
unsigned char RtcInit(void);
uint8_t readButtons(void);
void focusMenuItem(int itemId);
int menuSelector(void);
uint32_t timeAdjust(uint32_t cnt, uint8_t includeDays);
void programRunner(uint8_t programId);
RTC_DateTime unix2DateTime(uint32_t unixtime);
uint32_t DateTime2unix(RTC_DateTime datetime);
void EE_WriteWord(uint16_t Address, uint32_t Data);
uint32_t EE_ReadWord(uint16_t Address);
//void Lcd_print(char *STRING);
void setCTimer(uint8_t timerId);
uint32_t CTimerAdjust(uint32_t time);
void plugStateSet(uint8_t plug, uint8_t state);
void getPh();
void getEc(void);
FRESULT string2log(char* str, int bytes);
FRESULT sdLog2(void);
uint8_t adjust8bit(uint8_t val);
void loggerSettings(void);
int yesNoSelector(char str, int curval);
void loadSettings(void);
void set4lowBits(uint8_t dta);
void set4highBits(uint8_t dta);
void flush_lcd_buffer(void);
void phStabSettings(void);
void Lcd_write_16int(uint16_t);
void saveButtonRanges(void);
void readButtonRanges(void);
void set16bit(uint16_t value);
void convPh2str(uint8_t ph, char* phstr);
uint8_t readPhVal(uint8_t value);
void dht_conv_data(void);
void dht_init(void);
void dht_init_out(void);
void TIM3_IRQHandler(void);
void TIM1_BRK_TIM15_IRQHandler(void);
void dht_arr_displayer(void);
void setPwmDc(uint8_t duty_cycle);
void setDutyCycle(void);
void displayAdcValues(void);
// uint16_t get_average_adc(uint8_t amount);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);

void water_program_setup(uint8_t progId);
void fertilization_setup(void);
void watering_setup(void);
void fertilizer_mixing_program_setup(uint8_t progId);
void run_watering_program(uint8_t progId);
void run_fertilizer_mixer(uint8_t progId);
// void calibratEc(void);
void startWp(void);
uint16_t adjust16bit_fast(uint16_t val, uint8_t speed);
char * utoa_fast_div(uint32_t value, char *buffer);

// count crc starting from input and taking buffer from start_byte for length bytes
uint8_t crc_block(uint8_t input, uint8_t *start_byte, uint8_t length);

static void uart_task(void *pvParameters);
static void prvSetupHardware( void );
static void displayClock( void *pvParameters );
static void timerStateTrigger(void *pvParameters);
static void plugStateTrigger(void *pvParameters);
static void sdLog(void *pvParameters);
static void phMonitor(void  *pvParameters);
static void vTaskLCDdraw(void *pvParameters);
static void watering_program_trigger(void *pvParameters);
// static void valveManager(void);
// static void valve_status_updater(void);
void bluetooth_init(void);
void display_usart_rx(void);
void display_usart_tx(void);


void device_open_valve(uint8_t device_id);						// wrapper for open_valve()
void device_close_valve(uint8_t device_id);						// wrapper for close_valve()
void device_plug_state_set(uint8_t device_id, uint8_t state);	// wrapper for  plugStateSet()
void device_enable_dosing_pump(uint8_t device_id, uint8_t state);	// wrapper for enable_dosing_pump()


void open_valve(uint8_t valveId);
void close_valve(uint8_t valveId);
void run_valve_motor(uint8_t valveId);
void stop_valve_motor(uint8_t valveId);
void valve_test(void);
void valve_feedback_init(void);
void dosing_motor_control_init(void);
void valve_motor_control_init(void);
void EXTI15_10_IRQHandler(void);
void water_level_input_init(void);
void fertilizer_mixing_program_setup(uint8_t progId);
uint16_t adjust16bit(uint16_t val);
void enable_dosing_pump(uint8_t pumpId, uint8_t state);
void valveMotorStateSet(uint8_t valveId, uint8_t state);
void USART1_IRQHandler(void);
void StartDMAChannel4(unsigned int LengthBufer);
unsigned char GetStateDMAChannel4(void);
void TIM1_TRG_COM_TIM17_IRQHandler(void);
void TIM4_IRQHandler(void);
void tankLevelStabSetup(void);
void tankLevelStab(void);
void Lcd_write_16b(uint16_t tmp);
void Lcd_write_8b(uint8_t tmp);
void run_uart_cmd(void);
void get_water(uint8_t valve, uint8_t counter_id, uint16_t amount);
void get_water_cl(uint8_t valve, uint8_t counter_id, uint16_t amount);
void send_packet();
void setTimerSelector(void);
uint8_t idSelector(uint8_t min, uint8_t max, uint8_t curid);
void printOk(void);
void get_fertilizer(uint8_t fertId, uint8_t secs);
void get_settings_block(uint8_t block_number);
void rx_flush(void);
void adcAverager(void);
void get_status_block(uint8_t blockId);
void run_circulation_pump(uint16_t time);
void send_ee_addr(uint16_t addr, uint8_t type);
void rx_ee(uint16_t addr, uint8_t type);



void device_open_valve(uint8_t device_id){
	open_valve(device_id-20);
}

void device_close_valve(uint8_t device_id){
	close_valve(device_id-20);
}

void device_plug_state_set(uint8_t device_id, uint8_t state){
	plugStateSet(device_id, state);
}

void device_enable_dosing_pump(uint8_t device_id, uint8_t state){
	enable_dosing_pump(device_id-10, state);
}


void Lcd_write_8b(uint8_t tmp) {
	if (lcd_pointery==0) {
		LCDLine1[lcd_pointerx++] = (tmp/100)+48;
	}
	else {
		LCDLine2[lcd_pointerx++] = (tmp/100)+48	;
	}
	Lcd_write_digit(tmp%100);

}

void calibrateFlowMeter(void){
	uint8_t button=0, valve=0, counter_id=0, volume=0;
	uint16_t ticks_in_10cl=0;
	uint32_t cnt_val1=0, cnt_val2=0;
	Lcd_clear();
	Lcd_write_str("Choose one:");
	vTaskDelay(500);

	while (button==0) {
		button = readButtons();
		Lcd_goto(0,0);
		Lcd_write_str("1:");
		Lcd_write_16b(water_counter[0]);
		Lcd_write_str("2:");
		Lcd_write_16b(water_counter[1]);
		Lcd_goto(1,0);
		vTaskDelay(10);
		Lcd_write_str("3:");
		Lcd_write_16b(water_counter[2]);
		Lcd_write_str("4:");
		Lcd_write_16b(water_counter[3]);
	}
	Lcd_clear();
	counter_id = button-1;
	button=0;
	Lcd_write_str("Choose valve");
	valve = adjust8bit(valve);
	close_valve(valve);
	Lcd_clear();
	Lcd_write_str("How much x10CL?");
	volume = adjust8bit(volume);
	cnt_val1 = water_counter[counter_id];
	Lcd_clear();
	Lcd_write_str("OK to finish");
	open_valve(valve);
	button=0;
	while (button!=BUTTON_OK) {
		vTaskDelay(10);
		button=readButtons();
		Lcd_clear();
		Lcd_write_str("Current value:");
		Lcd_goto(1,0);
		Lcd_write_16b(water_counter[counter_id]);
	}
	cnt_val2 = water_counter[counter_id];
	close_valve(valve);
	vTaskDelay(200);

	ticks_in_10cl=(uint16_t)(cnt_val2-cnt_val1)/(uint16_t)volume;	// how much ticks for 10 CL?
	wfCalArray[counter_id] = ticks_in_10cl;	// ticks in 10cl
	Lcd_clear();
	Lcd_write_str("Ticks:");
	Lcd_write_16b(cnt_val2);
	Lcd_goto(1,0);
	Lcd_write_str("10CL=");
	Lcd_write_16b(ticks_in_10cl);
	EE_WriteVariable(WFM_CAL_OFFSET, ticks_in_10cl);
	button=0;
	while (button!=BUTTON_OK) {
		vTaskDelay(10);
		button=readButtons();
	}
	Lcd_clear();
	vTaskDelay(200);
}

void get_water_cl(uint8_t valve, uint8_t counter_id, uint16_t amount)
{
	uint32_t cnt_to_reach=0;
	cnt_to_reach = ((uint32_t)amount*(uint32_t)wfCalArray[0])/10;
//	uint8_t button=0;
	open_valve(0);		// HARDCODE
	water_counter[counter_id] = 0;
	while (water_counter[0]<cnt_to_reach) {
		vTaskDelay(10);
	}
	close_valve(0);		// HARDCODE
}

void get_water_tick(uint8_t valve, uint8_t counter_id, uint32_t ticks)
{
//	uint8_t button=0;
	open_valve(0);		// HARDCODE
	water_counter[counter_id] = 0;
	while (water_counter[0]<ticks) {
		vTaskDelay(10);
	}
	close_valve(0);		// HARDCODE
}

void get_water(uint8_t valve, uint8_t counter_id, uint16_t amount){	// amount in CL (10ml) max 65535x10=650L
	uint32_t cnt_to_reach=0;
	uint16_t volume=0;
	uint8_t button=0;
	Lcd_clear();
	Lcd_write_str("Calibrate WFM?");
	while (button==0) {
		vTaskDelay(10);
		button=readButtons();
	}
	if (button==BUTTON_OK) {
		calibrateFlowMeter();
	}


//	vTaskDelay(10000);
	uint8_t repeat=1;
	while (repeat==1) {
		Lcd_clear();
		Lcd_write_str("CL amount:");
		Lcd_write_16b(wfCalArray[0]);
		volume = adjust16bit_fast(volume, 5);
	//	cnt1=water_counter[0];
	//	cnt_to_reach = water_counter[0]+(volume*wfCalArray[0])/10;
		cnt_to_reach = ((uint32_t)volume*(uint32_t)wfCalArray[0])/10;

		Lcd_clear();
		Lcd_write_str("Adding");
		Lcd_write_16b((uint16_t)cnt_to_reach);
		Lcd_goto(1,0);
		Lcd_write_str("to ");
		Lcd_write_16b((uint16_t)water_counter[water_counter[counter_id]]);
		button=0;
		open_valve(0);		// HARDCODE
		water_counter[counter_id] = 0;
		while (water_counter[0]<cnt_to_reach) {
			vTaskDelay(10);
			Lcd_clear();
			Lcd_write_str("Cur:");
			Lcd_write_16b((uint16_t)water_counter[counter_id]);
			Lcd_goto(1,0);
			Lcd_write_str("of:");
			Lcd_write_16b(cnt_to_reach);
		}
		close_valve(0);		// HARDCODE
		Lcd_clear();
		Lcd_write_str("Repeat?");
		while (button==0) {
				vTaskDelay(10);
				button=readButtons();
		}
		if (button==BUTTON_OK) {
			repeat=1;
			water_counter[0]=0;
		}
		else {
			repeat=0;
		}
	}
}

void Lcd_write_16b(uint16_t tmp) {
	Lcd_write_digit((uint16_t)tmp/10000);
	Lcd_write_digit((uint16_t)(tmp%10000)/100);
	Lcd_write_digit((uint16_t)tmp%100);
}

uint8_t crc_block(uint8_t input, uint8_t *start_byte, uint8_t length){
	uint8_t i=0;
	for (i=0; i<length; i++) {
		input ^= start_byte[i];
	}
	return input;
}


void USART1_IRQHandler(void)
{

	if (RxCounter>6) {
			RxCounter=0;
	}
	if (USART_GetITStatus(BT_USART, USART_IT_RXNE) != RESET) {
//			RxByte=(USART_ReceiveData(BT_USART) & 0x7F);
			RxByte = USART_ReceiveData(BT_USART);	// get byte from Data Register
#ifndef		PRODUCTION_BUILD
			RxBuffer[RxCounter++] = RxByte;			// write it into RxBuffer
#endif
			USART1->SR &= ~USART_FLAG_RXNE;			// clear Rx Not Empty flag
	}
	BT_USART->SR &= ~USART_SR_TC;

	// sending packet in case of Tx Not Empty flag is set and buffer not sent completely yet
	if (txbuff_ne && TxCounter<NbrOfDataToTransfer) {
		USART1->DR = TxBuffer[TxCounter++];	// write TxBuff byte into Data Register for sending
		USART1->SR &= ~USART_SR_TC;		// clear Transfer Complete flag
	}

	if (packet_ready==0) {
		if (rxm_state==RXM_CMD) {
			if (pb_pntr==0) {	// if packet buffer pointer is 0, it points to payload size (payload, including this size byte)
				packet_length=RxByte;	// get packet length
			}
			cmdbuf[pb_pntr++]=RxByte;	// receive byte into cmd buffer and increase command packet buffer pointer
			if (pb_pntr==packet_length) {	// if packet buffer pointer reached packet length (rely on RXM_NONE set later)
				// crc count
				rx_packet_crc = crc_block(48, &cmdbuf[0],packet_length);	// 48 is XOR of "ZX2"
				// crc check
				if (rx_packet_crc==0) {	//
					packet_ready=1;	// packet received correctly and is ready to process
				}
				else {
					rx_flush();	// discard broken packet
				}
				rxm_state=RXM_NONE; // RX machine state set to NONE, completes RX cycle
				pb_pntr=0;			// packet buffer pointer to 0

			}

		}


		if (RxByte==90) {
			// ready
			prefixDetectionIdx = 1;
		}
		else if (RxByte==88 && prefixDetectionIdx==1) {
			// steady
			prefixDetectionIdx = 2;
		}
		else if (prefixDetectionIdx==2) {
			if (RxByte==48) {
				// escaping zero
			}
			else if (RxByte==49){
				// settings
				rxm_state=RXM_SET;
			}
			else if (RxByte==50) {
				// command
				rxm_state=RXM_CMD;
				pb_pntr=0;	// packet buffer pointer to zero to start reading new payload
			}
			prefixDetectionIdx = 0;
		}
	}
}

static void uart_task(void *pvParameters){
	while (1) {
		if (packet_ready==1) {

			run_uart_cmd();	// when
			packet_ready=0;
		}
		vTaskDelay(10);
		if (txbuff_ne==1) {	// if TxBuff is not empty (flag set by, for example, get_settings_block())
			send_packet();	// send packet from TxBuff
		}
		vTaskDelay(10);
	}
}

void send_packet(){
	NbrOfDataToTransfer = TxBuffer[3];	// packet size
	TxCounter=0;						// reset tx buffer pointer
	while (TxCounter<NbrOfDataToTransfer){
		vTaskDelay(5);
	}
	txbuff_ne=0;	// packet sent, reset tx not empty flag
}

void rx_flush(void){
	uint8_t i=0;
	for (i=0; i<16; i++){
		cmdbuf[i] = 0;
	}
}


void run_uart_cmd(void){
	switch (cmdbuf[1]) {
		case 0:
			get_water_cl(cmdbuf[2],cmdbuf[3],cmdbuf[4]*256+cmdbuf[5]);
			break;
		case 1:
			plugStateSet(cmdbuf[2], cmdbuf[3]);
			break;
		case 2:
			comm_state=COMM_DIRECT_DRIVE;
			plugStateSet(psi_pump_load_id, 0);	// disables PSI pump when entered CDD mode
			break;
		case 3:
			comm_state=COMM_MONITOR_MODE;
			break;
		case 4:
			open_valve(cmdbuf[2]);
			break;
		case 5:
			close_valve(cmdbuf[2]);
			break;
		case 6:
			get_settings_block(cmdbuf[2]);
			break;
		case 7:
			get_status_block(cmdbuf[2]);
			break;
		case 8:
			if (comm_state==COMM_DIRECT_DRIVE) {
				auto_flags=cmdbuf[2];	// set auto-processing flags
			}
			break;
		case 9:
			enable_dosing_pump(cmdbuf[2], cmdbuf[3]);
			break;
		case 10:
			valve_failed = cmdbuf[2];
			break;
		case 11:	// stop all processes and force manual control
			auto_flags=0;
			comm_state=COMM_DIRECT_DRIVE;
			auto_flags = 0;
			plugStateSet(0,0);
			plugStateSet(1,0);
			plugStateSet(2,0);
			plugStateSet(3,0);
			close_valve(2);
			close_valve(3);
			close_valve(0);
			close_valve(1);
			break;
		case 12:
			RTC_SetCounter(cmdbuf[2]*16777216+cmdbuf[3]*65536+cmdbuf[4]*256+cmdbuf[5]);
			break;
		case 13:
			NVIC_SystemReset();
			break;
		case 14:
			get_fertilizer(cmdbuf[2], cmdbuf[3]);
			break;
		case 15:	// receive 16 bit value to store into EEPROM
			rx_ee((cmdbuf[2]&(cmdbuf[3]<<8)), 1);
			break;
		case 16:	// receive 32 bit value to store into EEPROM
			rx_ee((cmdbuf[2]&(cmdbuf[3]<<8)), 2);
			break;
		case 17:	// send 16 bit EEPROM value
			send_ee_addr(cmdbuf[2]+cmdbuf[3]*256, 1);
			break;
		case 18:	// send 32 bit EEPROM value
			send_ee_addr(cmdbuf[2]+cmdbuf[3]*256, 2);
			break;
		case 19:
			loadSettings();
			break;
		case 20:
			valve_busy=0;
			break;
	}
	rx_flush();
}

void rx_ee(uint16_t addr, uint8_t type){
	//addr=(cmdbuf[2]+cmdbuf[3]*256);
	uint32_t val32 = 0;
	uint16_t val16 = 0;
	if (type==1) {
		val16=(cmdbuf[4]&(cmdbuf[5]<<8));
		EE_WriteVariable(addr,val16);
	}
	if (type==2) {
		val32=(cmdbuf[4]&(cmdbuf[5]<<8)&(cmdbuf[6]<<16)&(cmdbuf[7]<<24));
		EE_WriteWord(addr,val32);
	}
}


void send_ee_addr(uint16_t addr, uint8_t type){	// sends EEPROM cell contents via BT-USART
	uint16_t val16;
	uint32_t val32;
	// types: 0 - 8 bit, 1 - 16bit, 2 - 32bit
	if (type==1) {
		addr=(cmdbuf[2]&(cmdbuf[3]<<8));
		EE_ReadVariable(addr, &val16);
		TxBuffer[0] = 90;	// "Z"
		TxBuffer[1] = 88;	// "X"
		TxBuffer[2] = 49;	// "1"
		TxBuffer[3] = 9;
		TxBuffer[4] = cmdbuf[2];
		TxBuffer[5] = cmdbuf[3];
		TxBuffer[6] = (uint8_t)(val16&0xFF);
		TxBuffer[7] = (uint8_t)((val16>>8)&0xFF);
	}
	if (type==2) {
		addr=(cmdbuf[2]&(cmdbuf[3]<<8));
		val32 = EE_ReadWord(addr);
		TxBuffer[0] = 90;	// "Z"
		TxBuffer[1] = 88;	// "X"
		TxBuffer[2] = 49;	// "1"
		TxBuffer[3] = 11;	// ZX1+size+4bytepayload+crc
		TxBuffer[4] = cmdbuf[2];
		TxBuffer[5] = cmdbuf[3];
		TxBuffer[6] = (uint8_t)(val32&0xFF);
		TxBuffer[7] = (uint8_t)((val32>>8)&0xFF);
		TxBuffer[8] = (uint8_t)((val32>>16)&0xFF);
		TxBuffer[9] = (uint8_t)((val32>>24)&0xFF);
	}
	TxBuffer[39] = crc_block(0, &TxBuffer[0],(TxBuffer[3]-1));
	NbrOfDataToTransfer = TxBuffer[3];
	TxCounter=0;
	txbuff_ne = 1;
}

void get_status_block(uint8_t blockId){	// sends block with Cadi STATUS data
//	uint8_t block[8];
	TxBuffer[0] = 90;	// Z
	TxBuffer[1] = 88;	// X
	TxBuffer[2] = 51;	// 3 (sending STATUS)
	TxBuffer[3] = 40;	// packet size (ZX0+packet_size+payload+crc). Payload is 16bytes
	if (blockId==1) {	// state block 1
		TxBuffer[4] = comm_state;
		TxBuffer[5] = ((uint8_t)timerStateFlags&0xFF);
		TxBuffer[6] = ((uint8_t)cTimerStateFlags&0xFF);
		TxBuffer[7] = valveFlags;
		TxBuffer[8] = ((uint8_t)plugStateFlags&0xFF);
		TxBuffer[9] = wpStateFlags;	// watering program run flags
		TxBuffer[10] = dht_data[0];
		TxBuffer[11] = dht_data[1];
		TxBuffer[12] = dht_data[2];
		TxBuffer[13] = dht_data[3];
		TxBuffer[14] = (uint8_t)(RTC->CNTH&(0xFF));	// RTC unixtime
		TxBuffer[15] = (uint8_t)((RTC->CNTH>>8)&(0xFF));
		TxBuffer[16] = (uint8_t)(RTC->CNTL&(0xFF));
		TxBuffer[17] = (uint8_t)(((RTC->CNTL)>>8)&(0xFF));
		TxBuffer[18] = (uint8_t)(sonar_read[0]&(0xFF));		// First sonar lower byte
		TxBuffer[19] = (uint8_t)((sonar_read[0]>>8)&(0xFF));	// first sonar higher byte
		TxBuffer[20] = (uint8_t)(sonar_read[1]&(0xFF));		// second sonar
		TxBuffer[21] = (uint8_t)((sonar_read[1]>>8)&(0xFF));
		TxBuffer[22] = (uint8_t)(adcAverage[0]&(0xFF));	// ADC1 average reading
		TxBuffer[23] = (uint8_t)(((adcAverage[0])>>8)&(0xFF));
		TxBuffer[24] = (uint8_t)(adcAverage[1]&(0xFF));	// ADC2 average reading
		TxBuffer[25] = (uint8_t)(((adcAverage[1])>>8)&(0xFF));
		TxBuffer[26] = (uint8_t)(adcAverage[2]&(0xFF));	// ADC3 average reading
		TxBuffer[27] = (uint8_t)(((adcAverage[2])>>8)&(0xFF));
		TxBuffer[28] = (uint8_t)(adcAverage[3]&(0xFF));	// ADC4 average reading
		TxBuffer[29] = (uint8_t)(((adcAverage[3])>>8)&(0xFF));
		TxBuffer[30] = (uint8_t)(water_counter[0]&(0xFF));		// first wfm counter
		TxBuffer[31] = (uint8_t)((water_counter[0]>>8)&(0xFF));
		TxBuffer[32] = (uint8_t)((water_counter[0]>>16)&(0xFF));
		TxBuffer[33] = (uint8_t)((water_counter[0]>>24)&(0xFF));
		TxBuffer[34] = dosingPumpStateFlags2;		// dosing pump flags (dosingPumpStateFlags overwritten by some part of FW)
		TxBuffer[35] = auto_flags;	// first sonar higher byte
		TxBuffer[36] = (uint8_t)(sonar_read[1]&(0xFF));		// second sonar
		TxBuffer[37] = (uint8_t)((sonar_read[1]>>8)&(0xFF));
//		adcAverage[AVG_ADC_PSI]>psi_pump_top_level
		TxBuffer[38] = blockId;
	}
	if (blockId==2) {	// state block 2
//		uint32_t now = RTC_GetCounter();
		TxBuffer[4] = (uint8_t)(psi_pump_top_level&(0xFF));
		TxBuffer[5] = (uint8_t)((psi_pump_top_level>>8)&(0xFF));
		TxBuffer[6] = (uint8_t)(psi_pump_btm_level&(0xFF));
		TxBuffer[7] = (uint8_t)(((psi_pump_btm_level)>>8)&(0xFF));
		TxBuffer[8] = (uint8_t)0;
		TxBuffer[9] = (uint8_t)(((adcAverage[0])>>8)&(0xFF));
		TxBuffer[10] = (uint8_t)(((adcAverage[1])>>0)&(0xFF));
		TxBuffer[11] = (uint8_t)(((adcAverage[1])>>8)&(0xFF));
		TxBuffer[12] = (uint8_t)(((adcAverage[2])>>0)&(0xFF));
		TxBuffer[13] = (uint8_t)(((adcAverage[2])>>8)&(0xFF));
		TxBuffer[38] = blockId;
	}
	if (blockId==3) {	// state block 3
		TxBuffer[4] = (uint8_t)(GPIOA->IDR&(0xFF));
		TxBuffer[5] = (uint8_t)(((GPIOA->IDR)>>8)&(0xFF));
		TxBuffer[6] = (uint8_t)(((GPIOA->IDR)>>16)&(0xFF));
		TxBuffer[7] = (uint8_t)(((GPIOA->IDR)>>24)&(0xFF));
		TxBuffer[8] = (uint8_t)(GPIOB->IDR&(0xFF));
		TxBuffer[9] = (uint8_t)(((GPIOB->IDR)>>8)&(0xFF));
		TxBuffer[10] = (uint8_t)(((GPIOB->IDR)>>16)&(0xFF));
		TxBuffer[11] = (uint8_t)(((GPIOB->IDR)>>24)&(0xFF));
		TxBuffer[12] = dht_data[2];
		TxBuffer[13] = dht_data[3];
		TxBuffer[38] = blockId;
	}

	if (blockId==4) {	// state block 4
		TxBuffer[4] = currentEc;
		TxBuffer[5] = currentPh;
		TxBuffer[6] = phUnderOver+ecUnderOver*4;
		TxBuffer[7] = (uint8_t)(phWindowTop&(0xFF));
		TxBuffer[8] = (uint8_t)((phWindowBottom>>8)&(0xFF));
		TxBuffer[9] = 00;		// EMPTY
		TxBuffer[10] = dosingPumpStateFlags2;
		TxBuffer[11] = (uint8_t)(wfCalArray[0]&(0xFF));
		TxBuffer[12] = (uint8_t)((wfCalArray[0]>>8)&(0xFF));
		TxBuffer[13] = waterSensorStateFlags;
		TxBuffer[38] = blockId;
	}

	if (blockId==5) {	// state block 5
		TxBuffer[4] = (uint8_t)(sonar_read[0]&(0xFF));		// First sonar lower byte
		TxBuffer[5] = (uint8_t)((sonar_read[0]>>8)&(0xFF));	// first sonar higher byte
		TxBuffer[6] = (uint8_t)(sonar_read[1]&(0xFF));		// second sonar
		TxBuffer[7] = (uint8_t)((sonar_read[1]>>8)&(0xFF));
		TxBuffer[8] = (uint8_t)(water_counter[0]&(0xFF));		// first wfm counter
		TxBuffer[9] = (uint8_t)((water_counter[0]>>8)&(0xFF));
		TxBuffer[10] = (uint8_t)((water_counter[0]>>16)&(0xFF));
		TxBuffer[11] = (uint8_t)((water_counter[0]>>24)&(0xFF));
		TxBuffer[12] = valve_failed;		//
		TxBuffer[13] = waterSensorStateFlags;
		TxBuffer[38] = blockId;
	}

	if (blockId==6) {	// block 6
		TxBuffer[4] = (uint8_t)(sonar_read[0]&(0xFF));		// First sonar lower byte
		TxBuffer[5] = (uint8_t)((sonar_read[0]>>8)&(0xFF));	// first sonar higher byte
		TxBuffer[6] = (uint8_t)(sonar_read[1]&(0xFF));		// second sonar
		TxBuffer[5] = (uint8_t)((sonar_read[1]>>8)&(0xFF));
		TxBuffer[8] = (uint8_t)(water_counter[1]&(0xFF));		// 2nd water flow meter counter
		TxBuffer[9] = (uint8_t)((water_counter[1]>>8)&(0xFF));
		TxBuffer[10] = (uint8_t)((water_counter[1]>>16)&(0xFF));
		TxBuffer[11] = (uint8_t)((water_counter[1]>>24)&(0xFF));
		TxBuffer[12] = (uint8_t)(wfCalArray[1]&(0xFF));
		TxBuffer[13] = (uint8_t)((wfCalArray[1]>>8)&(0xFF));
		TxBuffer[22] = blockId;
	}

	TxBuffer[39] = crc_block(0, &TxBuffer[0],39);
	NbrOfDataToTransfer = TxBuffer[3];
	TxCounter=0;
	txbuff_ne = 1;
}


void get_settings_block(uint8_t block_number){
	/* block_number is a number from 0 to N, where N is a
	 * Settings memory array length / 8
	 * 8 - is a default block size for this function
	 */
	uint8_t i=0;
	uint16_t addr = 0, tmpbuf = 0;
	TxBuffer[0] = "Z";
	TxBuffer[1] = "X";
	TxBuffer[2] = "0";	// sending settings
	TxBuffer[3] = 13;	// packet size (ZX0+packet_size+payload). Payload is 8bytes. Here, packet size means only payload with size byte, not like in STATUS packets, where size means full packet size
	TxBuffer[4] = block_number;
	for (i=0; i<4; i++) {	// 4 blocks x 2bytes = 8bytes
		// here goes EEPROM reading loop, that puts payload data into TX buff, equipped with appropriate packet header
		addr = SETTINGS_START_ADDR+block_number*4+i*2;	// address for
		EE_ReadVariable(addr, &tmpbuf);
		TxBuffer[5+i*2] = (uint8_t)(tmpbuf*0xFF00)>>8;
		TxBuffer[5+i*2+1] = (uint8_t)(tmpbuf*0xFF);
	}
	txbuff_ne=1;
}

/* void onPacketType(uint8_t b) {
        // depending on packet type we transit to next state
        switch (b) {
        case 1: // for configuration packet type...
            // uint8_t payload[400];
            payloadIdx = 0;
            rxm_state = STATE_CONFIGURATION_PAYLOAD;
            break;
        case 2: // for command packet type...
        	rxm_state = STATE_COMMAND_TYPE; // ...expect command type to be read next
            break;
        }
        crc = 0;
}

void onPayloadByte(uint8_t b, uint8_t nextState) {
         // on each payload byte - fill payload buffer and calculate crc
         payload[payloadIdx++] = b;
         if (payloadIdx == countof(payload)) {
             rxm_state = nextState;
         }
         crc(b);
} */


void DMA1_Channel4_IRQHandler (void)
{
  if(DMA1->ISR & DMA_ISR_TCIF4) { }
}

void DMA1_Channel5_IRQHandler (void)
{

 if(DMA1->ISR & DMA_ISR_TCIF5) { }
}

void bluetooth_init(void){
//	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

//    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);

    // Tx on PA9 as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Rx on PA10 as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitTypeDef USART_InitStructure;

  // USART 1 init
  USART_DeInit(USART1);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  USART_ClearFlag(USART1, USART_FLAG_CTS | USART_FLAG_LBD  |
                        USART_FLAG_TC  | USART_FLAG_RXNE
                );
  USART1->CR1  |= USART_CR1_RXNEIE;
  USART1->CR1  |= USART_CR1_TCIE;
  USART_Cmd(USART1, ENABLE);
  NVIC_EnableIRQ(USART1_IRQn);
}



unsigned char GetStateDMAChannel4(void)
{
  if(DMA1->ISR & DMA_ISR_TCIF4) return 1;   //transmission finished
  return 0;                                 //transmission in prograss
}
//********************************************************************************
//Function: start exchange in  direction "memory-DMA-USART1"                           //
//Argument: amount of data                                        //
//********************************************************************************
void StartDMAChannel4(unsigned int LengthBufer)
{
  DMA1_Channel4->CCR   = ~DMA_CCR4_EN;      //disable DMA channel
  DMA1_Channel4->CNDTR =  LengthBufer;      //load data amount to transmit
  DMA1->IFCR          |=  DMA_IFCR_CTCIF4;  //reset end of transmit flag
  DMA1_Channel4->CCR  |=  DMA_CCR4_EN;      //enable DMA channel
}


void valve_motor_control_init(void){		// init PA8-PA11 for valve motor control output

		GPIO_InitTypeDef init_pin;

#ifndef CADI_MB
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
		 GPIOA->CRH      &= ~GPIO_CRH_CNF14;
	     GPIOA->CRH   |= GPIO_CRH_MODE14_0;
		 GPIOA->CRH      &= ~GPIO_CRH_CNF13;
	     GPIOA->CRH   |= GPIO_CRH_MODE13_0;
		 GPIOA->CRH      &= ~GPIO_CRH_CNF12;
	     GPIOA->CRH   |= GPIO_CRH_MODE12_0;
		 GPIOA->CRH      &= ~GPIO_CRH_CNF11;
	     GPIOA->CRH   |= GPIO_CRH_MODE11_0;
#endif

#ifdef CADI_MB
	 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_AFIO, ENABLE);
	 	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
//	 	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
//	 	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
	 	init_pin.GPIO_Pin  = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
		init_pin.GPIO_Mode = GPIO_Mode_Out_PP;
		init_pin.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &init_pin);


/*		 GPIOA->CRH      &= ~GPIO_CRH_CNF14;
	     GPIOA->CRH   |= GPIO_CRH_MODE14_0;
		 GPIOA->CRH      &= ~GPIO_CRH_CNF14;
	     GPIOA->CRH   |= GPIO_CRH_MODE14_0;
		 GPIOA->CRH      &= ~GPIO_CRH_CNF13;
	     GPIOA->CRH   |= GPIO_CRH_MODE13_0;
		 GPIOA->CRH      &= ~GPIO_CRH_CNF12;
	     GPIOA->CRH   |= GPIO_CRH_MODE12_0;
		 GPIOA->CRH      &= ~GPIO_CRH_CNF11;
	     GPIOA->CRH   |= GPIO_CRH_MODE11_0; */

#endif
}

void valve_feedback_init(void){		// init PA6-8 as input for 3V valve feedback

	// v1.4 config assumes PA5-7 pins as feedback, implement init function
	// v2 config assumes PA6-8 pins as feedback

// PA5-7 setup
	GPIO_InitTypeDef GPIO_InitStructure;
//	EXTI_InitTypeDef EXTI_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
	  // Enable GPIOA clock
//	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	  // Configure PA5-7 pin as input pull-down
	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
//	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
//	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PD;
	  GPIO_Init(VALVE_SENSOR_PORT, &GPIO_InitStructure);

//	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
//	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
//	  GPIO_Init(VALVE_SENSOR_PORT, &GPIO_InitStructure);
}

void water_level_input_init(void){
#ifdef WFM_PINS_PB0_1			// Configuring PINs for Water Flow Meters
		GPIO_InitTypeDef GPIO_InitStructure;
		EXTI_InitTypeDef EXTI_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		  // Enable GPIO clock
		  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

		  // Configure PB0-2 pins as input pull-down
		  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
		  GPIO_Init(GPIOB, &GPIO_InitStructure);


// interrupt config for water level sensor inputs

		  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
		  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
		  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		  EXTI_Init(&EXTI_InitStructure);

		  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
		  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
		  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		  EXTI_Init(&EXTI_InitStructure);

		  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
		  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
		  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
		  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		  NVIC_Init(&NVIC_InitStructure);

		  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
		  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
		  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
		  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		  NVIC_Init(&NVIC_InitStructure);

#endif
}

void dosing_motor_control_init(void){	// init PC6-PC9 as PWM output for dosing pump control
	  GPIO_InitTypeDef GPIO_InitStructure;
	  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	  TIM_OCInitTypeDef TIM_OCInitStruct;


	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE );

	    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );

	    GPIO_StructInit(&GPIO_InitStructure); // Reset init structure

	    // Setup Blue & Green LED on STM32-Discovery Board to use PWM.
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;            // Alt Function - Push Pull
	    GPIO_Init(GPIOC, &GPIO_InitStructure );
	    GPIO_PinRemapConfig( GPIO_FullRemap_TIM3, ENABLE );        // Map TIM3_CH3 to GPIOC.Pin8, TIM3_CH4 to GPIOC.Pin9

	    // Let PWM frequency equal 100Hz.
	    // Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
	    // Solving for prescaler gives 240.
	    TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
	    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
	    TIM_TimeBaseInitStruct.TIM_Period = 1000 - 1;   // 0..999
	    TIM_TimeBaseInitStruct.TIM_Prescaler = 240 - 1; // Div 240
	    TIM_TimeBaseInit( TIM3, &TIM_TimeBaseInitStruct );

	    TIM_OCStructInit( &TIM_OCInitStruct );
	    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	    // Initial duty cycle equals 0%. Value can range from zero to 1000.
	    TIM_OCInitStruct.TIM_Pulse = 0; // 0 .. 1000 (0=Always Off, 1000=Always On)


	    TIM_OC1Init( TIM3, &TIM_OCInitStruct ); // Channel 3 Blue LED
	    TIM_OC2Init( TIM3, &TIM_OCInitStruct ); // Channel 3 Blue LED
	    TIM_OC3Init( TIM3, &TIM_OCInitStruct ); // Channel 3 Blue LED
	    TIM_OC4Init( TIM3, &TIM_OCInitStruct ); // Channel 4 Green LED

	    TIM_Cmd( TIM3, ENABLE );
}


void tankLevelStabSetup(void){
	Lcd_clear();
	uint16_t curlevel=0;
	uint8_t button=0, tmp=0;
	Lcd_clear();
	Lcd_write_str("Supply valve ID");
	EE_ReadVariable(WATER_TANK_SUPPLY_VALVE, &tmp);
	tmp = adjust8bit(tmp);
	EE_WriteVariable(WATER_TANK_SUPPLY_VALVE, tmp);
	printOk();
	vTaskDelay(10);
	Lcd_clear();
	Lcd_write_str("Set tank top lvl");
	while (button!=BUTTON_OK) {
		button = readButtons();
		curlevel = sonar_read[0];
		vTaskDelay(5);
		Lcd_goto(1,0);
		Lcd_write_digit(curlevel/100);
		Lcd_write_digit(curlevel);
	}
	vTaskDelay(10);
	EE_WriteVariable(WATER_TANK_TOP, curlevel);
	printOk();
	vTaskDelay(2000);
	Lcd_write_str("Set btm lvl");
	button = 0;
/*	while (button!=BUTTON_OK) {
		button = readButtons();
		curlevel = sonar_read[0];
		vTaskDelay(5);
		Lcd_goto(1,0);
		Lcd_write_digit(curlevel/100);
		Lcd_write_digit(curlevel);
	}
	button=0;
	vTaskDelay(10);
	EE_WriteVariable(WATER_TANK_BOTTOM, curlevel);
	printOk();
	vTaskDelay(2000); */
	Lcd_clear();
	loadSettings();
}

void tankLevelStab(void){
#ifdef TANK_STAB_ENABLE
	vTaskDelay(1);
	uint16_t tmp=0;
	uint8_t supply_valve=0;
	EE_ReadVariable(WATER_TANK_SUPPLY_VALVE,&tmp);
	vTaskDelay(1);
	supply_valve = (uint8_t)(tmp&0x00FF);
	if (((auto_flags&2)>>1)==1 && supply_valve<4) {		// TANK STAB flag 1
		uint8_t valvebusy = 0;
		valvebusy = (valve_busy>>supply_valve) & 1;
		if (valvebusy==0) {
			if (sonar_read[0]>(tank_windows_top[0]+2)) {
				open_valve(supply_valve);
			}
			if (sonar_read[0]<tank_windows_top[0]) {
				close_valve(supply_valve);
			}
		}
	}
#endif
}

void psiStab(void){
#ifdef PSI_STAB_ENABLE
/*	if (auto_flags&1==1) {			// PSI STAB flag 0
		if (adcAverage[AVG_ADC_PSI]>psi_pump_top_level) {
			plugStateSet(psi_pump_load_id, 0);
		}
		if (adcAverage[AVG_ADC_PSI]<psi_pump_btm_level) {
			plugStateSet(psi_pump_load_id, 1);
		}
	} */
#endif
}


void open_valve(uint8_t valveId){
	vTaskDelay(1);
#ifdef USE_VALVES
	uint8_t valvebusy = 0;
	valvebusy = (valve_busy>>valveId) & 1;
	if (valvebusy==0) {
		valve_busy|=(1<<valveId);
		if (valveId==0 || valveId==1) {
			uint8_t curstatus=0, cur_valve_failed=0, duration=0;
			if (valveId==0) {
				curstatus=(uint8_t)(((VALVE_SENSOR_PORT->IDR)>>4) & 1);
			}
			else {
				curstatus=(uint8_t)(((VALVE_SENSOR_PORT->IDR)>>8) & 1);
			}
			vTaskDelay(1);
			cur_valve_failed=(valve_failed>>valveId)&1;	// check if valve failure flag is off
			if (curstatus==0 && cur_valve_failed==0) {	// if no failure detected and current status is ok, for ball valves with feedback
				run_valve_motor(valveId);
				uint16_t timeout=VALVE_FAILURE_TIMEOUT;	// valve failure timeout
				while (duration<3 && timeout>0) {		// HARDCODE. duration needed for sure-read of valve feedback
					if (valveId==0) {
						curstatus=(uint8_t)(((VALVE_SENSOR_PORT->IDR)>>4) & 1);
					}
					else {
						curstatus=(uint8_t)(((VALVE_SENSOR_PORT->IDR)>>8) & 1);
					}
					if (curstatus==1) {
						duration++;
					}
					else {
						duration=0;
					}
					vTaskDelay(1);
					timeout--;
				}
				if (timeout==0) {
					valve_failed |= (1<<valveId);
				}
			}
			stop_valve_motor(valveId);
			valveFlags &= ~(1<<valveId); // sbrosit' flag
			valveFlags |= (curstatus<<valveId);
		}

		vTaskDelay(5);
		valve_busy&= ~(1<<valveId);
	}
	if (valveId==2 || valveId==3) {	// for solenoid valves
		VALVE_MOTOR_PORT->BRR |= (1<<valveId+VALVE_MOTOR_GPIO_SHIFT);
		valveFlags |= (1<<valveId); // set flag
	}
	vTaskDelay(1);
#endif
}

void close_valve(uint8_t valveId){
	vTaskDelay(1);
	uint8_t valvebusy = 0;
	valvebusy = (valve_busy>>valveId) & 1;
	if (valvebusy==0) {
		valve_busy|=(1<<valveId);
		uint8_t duration=0, curstatus=0, cur_valve_failed=0;
		uint16_t timeout=VALVE_FAILURE_TIMEOUT;	// uint8_t fixed to uint16_t
		if (valveId==0) {
			curstatus=(uint8_t)(((VALVE_SENSOR_PORT->IDR)>>4) & 1);
		}
		else {
			curstatus=(uint8_t)(((VALVE_SENSOR_PORT->IDR)>>8) & 1);
		}
		cur_valve_failed=valve_failed&(1<<valveId);	// check if valve failure flag is off
		vTaskDelay(1);
		cur_valve_failed>>=valveId;	// ostavit' toka flag
		if (curstatus==1 && cur_valve_failed==0 && (valveId==0 || valveId==1)) {	// if no failure detected and current status is ok
			run_valve_motor(valveId);
			while (duration<3 && timeout>0) {
				if (valveId==0) {
					curstatus=(uint8_t)(((VALVE_SENSOR_PORT->IDR)>>4) & 1);
				}
				else {
					curstatus=(uint8_t)(((VALVE_SENSOR_PORT->IDR)>>8) & 1);
				}
				// this if statement filters occasional "0"s on GPIO (more needed for "1"s on open_valve())
				if (curstatus==0) {
					duration++;
				}
				else {
					duration=0;
				}
				vTaskDelay(1);
				timeout--;
			}
		}
		stop_valve_motor(valveId);
		valveFlags &= ~(1<<valveId); // sbrosit' flag
//		valveFlags |= ((((VALVE_SENSOR_PORT->IDR)>>(VALVE_SENSOR_GPIO_SHIFT+valveId*4) & 1))<<valveId);
		valve_busy&= ~(1<<valveId);
	}
	if (valveId==2 || valveId==3) {	// for solenoid valves
		VALVE_MOTOR_PORT->BSRR |= (1<<valveId+VALVE_MOTOR_GPIO_SHIFT);
		valveFlags &= ~(1<<valveId); // reset flag
	}
	vTaskDelay(1);
}




void run_valve_motor(uint8_t valveId){
	VALVE_MOTOR_PORT->BRR |= (1<<valveId+VALVE_MOTOR_GPIO_SHIFT);
}

void stop_valve_motor(uint8_t valveId){
	VALVE_MOTOR_PORT->BSRR |= (1<<valveId+VALVE_MOTOR_GPIO_SHIFT);
}



void EXTI0_IRQHandler(void) {
	EXTI_ClearITPendingBit(EXTI_Line0);
#ifdef WFM_PINS_PB0_1	// water flow meter (WFM) driver actions
	water_counter[0]++;
#endif
}

void EXTI1_IRQHandler(void) {
	EXTI_ClearITPendingBit(EXTI_Line1);
#ifdef WFM_PINS_PB0_1
	water_counter[1]++;
#endif

}

void EXTI2_IRQHandler(void) {
	EXTI_ClearITPendingBit(EXTI_Line2);

}

void EXTI15_10_IRQHandler(void)
{
    // Clear the  EXTI line 10-12 pending bit
	EXTI_ClearITPendingBit(EXTI_Line10);
	EXTI_ClearITPendingBit(EXTI_Line11);
	EXTI_ClearITPendingBit(EXTI_Line12);
}

void EXTI9_5_IRQHandler(void)
{
    // Clear the  EXTI line 10-12 pending bit
	EXTI_ClearITPendingBit(EXTI_Line5);
	EXTI_ClearITPendingBit(EXTI_Line6);
	EXTI_ClearITPendingBit(EXTI_Line7);
}

void valve_test2(void){
	uint8_t button=0, curvalve=0;
	Lcd_clear();

	while (button!=BUTTON_OK) {
		vTaskDelay(20);
		button=readButtons();
#ifdef USE_VALVES
		Lcd_goto(0,0);
		Lcd_write_str("curvalve:");
		Lcd_write_digit(curvalve);



		if (button==BUTTON_FWD){
			//open_valve(curvalve);
			open_valve(curvalve);
		}
		if (button==BUTTON_CNL){
			//close_valve(curvalve);
			close_valve(curvalve);
		}
		if (button==BUTTON_BCK){
			if (curvalve<2) {
				curvalve++;
			}
			else {
				curvalve=0;
			}
		}
		vTaskDelay(1);
		Lcd_goto(1,0);
		Lcd_write_str("VF");
		Lcd_write_digit(valveFlags);

		vTaskDelay(1);

		Lcd_write_str("States:");

		if (VALVE_SENSOR_PORT->IDR&(1<<4)) {
			Lcd_write_str("1");
		}
		else {
			Lcd_write_str("0");
		}
		if (VALVE_SENSOR_PORT->IDR&(1<<5)) {
			Lcd_write_str("1");
		}
		else {
			Lcd_write_str("0");

		if (VALVE_SENSOR_PORT->IDR&(1<<6)) {
			Lcd_write_str("1");
		}
		else {
			Lcd_write_str("0");
		}

		if (VALVE_SENSOR_PORT->IDR&(1<<8)) {
			Lcd_write_str("1");
		}
		else {
			Lcd_write_str("0");
		}
		}

#endif
	}
	Lcd_clear();
}

void valve_test3(void){
	uint8_t button=0, curvalve=2;
	Lcd_clear();

	while (button!=BUTTON_OK) {
		vTaskDelay(20);
		button=readButtons();
#ifdef USE_VALVES
		Lcd_goto(0,0);
		Lcd_write_str("curvalve:");
		Lcd_write_digit(curvalve);
		if (button==BUTTON_FWD){
			//open_valve(curvalve);
			VALVE_MOTOR_PORT->BSRR |= (1<<(11+curvalve));
			run_valve_motor(curvalve);
		}
		if (button==BUTTON_CNL){
			//close_valve(curvalve);
			VALVE_MOTOR_PORT->BRR |= (1<<(11+curvalve));
			stop_valve_motor(curvalve);
		}
		if (button==BUTTON_BCK){
			if (curvalve<4) {
				curvalve++;
			}
			else {
				curvalve=2;
			}
		}
		vTaskDelay(1);
		Lcd_goto(1,0);
		Lcd_write_str("VF");
		Lcd_write_digit(valveFlags);

		vTaskDelay(1);

		Lcd_write_str("States:");

		if (GPIOA->ODR&(1<<13)) {
			Lcd_write_str("1");
		}
		else {
			Lcd_write_str("0");
		}
		if (GPIOA->ODR&(1<<14)) {
			Lcd_write_str("1");
		}
		else {
			Lcd_write_str("0");
		}

#endif
	}
	Lcd_clear();
}


void valve_test(void){
	uint8_t button=0;
	uint16_t tmp=0;
	Lcd_clear();
	while (button!=BUTTON_OK) {
		vTaskDelay(20);
		button=readButtons();
#ifdef USE_VALVES
		Lcd_goto(0,0);
//		tmp=water_counter[0];
//		Lcd_write_16b(tmp);

		tmp=sonar_read[0];
//		Lcd_write_digit(tmp/100);
		Lcd_write_digit(tmp);
		tmp=sonar_read[1];
//		Lcd_write_digit(tmp/100);
		Lcd_write_digit(tmp);
		Lcd_write_str("cm");

		tmp=TIM17->CNT;
		Lcd_write_16b(tmp);

//		for (i=0; i<3; i++) {
//			curState = valveFlags&(1<<i);
//			curState>>=i;	// ostavit' toka flag

			if (button==BUTTON_FWD){
					//if (curState==1){
						//close_valve(0);
					//}
					//else {
					//	run_valve_motor(0);
					//	run_valve_motor(1);
						open_valve(1);
					//	run_valve_motor(2);
					//}
			}
			if (button==BUTTON_BCK){
				//if (curState==1){
					//stop_valve_motor(0);
					stop_valve_motor(1);
					close_valve(1);
					//stop_valve_motor(2);
				//}
				//else {
				//	open_valve(1);
				//}
			}
//		}
		vTaskDelay(1);
		Lcd_goto(1,0);
		Lcd_write_digit(valveFlags);
		Lcd_write_str("VS");
		uint16_t val=0;
		val = VALVE_SENSOR_PORT->IDR>>5;
		Lcd_write_16b(val);
		Lcd_write_str(" ");

		vTaskDelay(1);
		if (VALVE_SENSOR_PORT->IDR>>5 & 1) {
			Lcd_write_str("1");
		}
		else {
			Lcd_write_str("0");
		}

		if (VALVE_SENSOR_PORT->IDR>>6 & 1) {
			Lcd_write_str("1");
		}
		else {
			Lcd_write_str("0");
		}

		if (VALVE_SENSOR_PORT->IDR>>7 & 1) {
			Lcd_write_str("1");
		}
		else {
			Lcd_write_str("0");
		}
#endif

		Lcd_write_str("W");

	}
	Lcd_clear();
	valve_test2();
	valve_test3();
}

void water_program_setup(uint8_t progId){
	uint16_t addr=0, tempvalue=0;
	uint32_t tmp32, interval;

	// choose tank top water level sensor
	addr = WP_OFFSET+progId*WP_SIZE+TOP_WATER_SENSOR_SHIFT;
	EE_ReadVariable(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("Set top sensor");
	tempvalue = adjust16bit(tempvalue);
	EE_WriteVariable(addr, tempvalue);


	// choose tank bottom water level sensor
	addr = WP_OFFSET+progId*WP_SIZE+BOTTOM_WATER_SENSOR_SHIFT;
	EE_ReadVariable(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("Set bottom sensor");
	tempvalue = adjust16bit(tempvalue);
	EE_WriteVariable(addr, tempvalue);

	// set fresh water fill timeout, seconds
	addr = WP_OFFSET+progId*WP_SIZE+WATER_FILL_TIMEOUT_SHIFT;
	EE_ReadVariable(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("Wtr fill timeout");
	tempvalue = adjust16bit(tempvalue);
	EE_WriteVariable(addr, tempvalue);

	// set circulation pump plug id
//	addr = WP_OFFSET+progId*WP_SIZE+WP_CIRCULATION_PUMP_PLUG_SHIFT;
	EE_ReadVariable(CIRCULATION_PUMP_ID_ADDR, &tempvalue);
	Lcd_clear();
	Lcd_write_str("CircPump plug ID");
	tempvalue = adjust8bit(tempvalue);
	EE_WriteVariable(CIRCULATION_PUMP_ID_ADDR, tempvalue);
	circulationPumpId = tempvalue;
	addr = EE_PLUG_SETTINGS+tempvalue;
	EE_WriteVariable(addr, 70);		// HARDCODE!!! program (timer) id for circulation pump
	plugSettings[tempvalue] = 70;

	// set watering pump plug id
	addr = WP_OFFSET+progId*WP_SIZE+WP_WATERING_PUMP_PLUG_ID;
	EE_ReadVariable(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("Wtrng pump plug");
	tempvalue = adjust8bit(tempvalue);
	EE_WriteVariable(addr, tempvalue);
	addr = EE_PLUG_SETTINGS+tempvalue;
	EE_WriteVariable(addr, 71);		// HARDCODE!!! program (timer) id for watering pump
	plugSettings[tempvalue] = 71;

	// set watering duration, seconds
	addr = WP_OFFSET+progId*WP_SIZE+WP_DURATION_SHIFT;
	EE_ReadVariable(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("Wtrng duration");
	tempvalue = adjust16bit(tempvalue);
	EE_WriteVariable(addr, tempvalue);

	// set watering duration, seconds
	addr = WP_OFFSET+progId*WP_SIZE+WP_INTERVAL_SHIFT;
	interval = EE_ReadWord(addr);
	Lcd_clear();
	Lcd_write_str("Set run interval");
	vTaskDelay(500);
	interval = CTimerAdjust(interval);
	EE_WriteWord(addr, interval);

	addr = WP_OFFSET+progId*WP_SIZE+WP_DRAIN_DURATION_SHIFT;
	EE_ReadVariable(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("Drain duration");
	vTaskDelay(500);
	tempvalue = adjust16bit(tempvalue);
	EE_WriteVariable(addr, tempvalue);

	// enable watering program (=1 enables the WP)
	addr = WP_OFFSET+progId*WP_SIZE+WP_ENABLE_SHIFT;
	EE_ReadVariable(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("Enable WP?");
	tempvalue = adjust8bit(tempvalue);
	EE_WriteVariable(addr, tempvalue);



//	EE_ReadWord(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("Start program?");
	uint8_t button=0;
	while (button==0) {
		button=readButtons();
		vTaskDelay(10);
	}
	if (button==BUTTON_OK) {
		// set initial time point of watering program
		addr = WP_OFFSET+progId*WP_SIZE+WP_START_TIME_SHIFT;
		tmp32 = RTC_GetCounter();
		EE_WriteWord(addr, tmp32);

		addr = WP_OFFSET+progId*WP_SIZE+WP_LAST_RUN_SHIFT;
		EE_WriteWord(addr, (RTC_GetCounter()-interval+5));
//			run_watering_program(progId);
	}
	if (button==BUTTON_CNL) {
		Lcd_clear();
		Lcd_write_str("Start time");
		vTaskDelay(1000);
		addr = WP_OFFSET+progId*WP_SIZE+WP_START_TIME_SHIFT;
		tmp32 = EE_ReadWord(addr);
		tmp32 = timeAdjust(tmp32, 1);
		EE_WriteWord(addr, tmp32);
	}
	Lcd_clear();
	Lcd_write_str("End time");
	vTaskDelay(1000);
	addr = WP_OFFSET+progId*WP_SIZE+WP_END_TIME_SHIFT;
	tmp32 = EE_ReadWord(addr);
	tmp32 = timeAdjust(tmp32, 1);
	EE_WriteWord(addr, tmp32);
	Lcd_clear();
	vTaskDelay(200);

	vTaskSuspendAll();
	loadSettings();
	xTaskResumeAll();
}

void fertilization_setup(void){
	uint8_t progId=0;
	Lcd_clear();
	Lcd_write_str("Program nr");
	vTaskDelay(200);
	progId = adjust8bit(progId);
	fertilizer_mixing_program_setup(progId);
}

void watering_setup(void){
	uint8_t progId=0;
	Lcd_clear();
	Lcd_write_str("Program nr");
	vTaskDelay(200);
	progId = adjust8bit(progId);
	water_program_setup(progId);
}

// set fertilizer mixing programs
void fertilizer_mixing_program_setup(uint8_t progId){
	uint16_t addr=0, tempvalue=0;

	// choose dosing pump to use
	addr = FMP_OFFSET+progId*FMP_SIZE+FMP_DOSING_PUMP_ID_SHIFT;
	EE_ReadVariable(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("Fertilizer 2 use");
	tempvalue = adjust16bit(tempvalue);
	EE_WriteVariable(addr, tempvalue);

	// set dosing time, seconds
	addr = FMP_OFFSET+progId*FMP_SIZE+FMP_DOSING_TIME_SHIFT;
	EE_ReadVariable(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("Dosing duration");
	tempvalue = adjust16bit(tempvalue);
	EE_WriteVariable(addr, tempvalue);

	// set circulation pump mixing time
	addr = FMP_OFFSET+progId*FMP_SIZE+FMP_CIRCULATION_MIXING_TIME_SHIFT;
	EE_ReadVariable(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("Aftermix time");		// setting the time for running circulation pump to mix the fertilizer into solution
	tempvalue = adjust16bit(tempvalue);
	EE_WriteVariable(addr, tempvalue);

	// set watering program to associate this fertilizer mix program with
	addr = FMP_OFFSET+progId*FMP_SIZE+FMP_2_WP_ASSOC_SHIFT;
	EE_ReadVariable(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("Link to WP");
	tempvalue = adjust16bit(tempvalue);
	EE_WriteVariable(addr, tempvalue);

	// set frequency (N), showing how often this fertilizer mixing program activates (every N watering program run)
	addr = FMP_OFFSET+progId*FMP_SIZE+FMP_TRIG_FREQUENCY_SHIFT;
	EE_ReadVariable(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("Trig. freq. N");
	tempvalue = adjust16bit(tempvalue);
	EE_WriteVariable(addr, tempvalue);

	// set ENABLED/DISABLED status for this program
	addr = FMP_OFFSET+progId*FMP_SIZE+FMP_ENABLE;
	EE_ReadVariable(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("ENABLE FMP?");
	tempvalue = adjust16bit(tempvalue);
	vTaskDelay(10);
	EE_WriteVariable(addr, tempvalue);
	Lcd_clear();
	vTaskDelay(10);
	loadSettings();
}

void startWp(void){
	uint8_t progId=0, button=0;
	uint32_t addr, tmp32, interval;
	Lcd_clear();
	Lcd_write_str("Program nr");
	vTaskDelay(200);
	progId = adjust8bit(progId);
	Lcd_clear();
	Lcd_write_str("Start WP#");
	Lcd_write_digit(progId);
	Lcd_write_str("?");
	while (button!=BUTTON_OK && button!=BUTTON_CNL) {
//		dht_get_data();
		vTaskDelay(30);
		button=readButtons();
	}
	vTaskDelay(200);
	if (button==BUTTON_OK) {
		addr = WP_OFFSET+progId*WP_SIZE+WP_INTERVAL_SHIFT;
		interval = EE_ReadWord(addr);
		// set initial time point of watering program
		addr = WP_OFFSET+progId*WP_SIZE+WP_START_TIME_SHIFT;
		tmp32 = RTC_GetCounter();
		EE_WriteWord(addr, tmp32);

		addr = WP_OFFSET+progId*WP_SIZE+WP_LAST_RUN_SHIFT;
		EE_WriteWord(addr, (RTC_GetCounter()-interval+5));
//			run_watering_program(progId);
	}
	Lcd_clear();
	vTaskDelay(200);
}

void run_watering_program(uint8_t progId){
	wpStateFlags|=(1<<progId);	// set active flag for this program
	close_valve(DRAIN_VALVE_ID);	// close drain valve
//	valveFlags=0;	// all valves to off;
	uint16_t addr, sensorId, fmpLink, wateringPlugId=0;
	uint8_t i, waterSensorFlag=0;
	uint32_t now=0, end=0, wateringDuration=0, lastTime=0;
	vTaskDelay(1);

	addr = WP_OFFSET+progId*WP_SIZE+TOP_WATER_SENSOR_SHIFT;
	EE_ReadVariable(addr, &sensorId);

	// water fill timeout
	addr = WP_OFFSET+progId*WP_SIZE+WATER_FILL_TIMEOUT_SHIFT;
	EE_ReadVariable(addr, &wateringDuration);
	now = RTC_GetCounter();
	end = now + wateringDuration;

	vTaskDelay(5);
	// fill the tank
	open_valve(0);
	while (waterSensorFlag==0 && now<end) {
		now = RTC_GetCounter();
		waterSensorFlag=waterSensorStateFlags&(1<<sensorId);	// check if water level reached top sensor
		waterSensorFlag>>=sensorId;	// ostavit' toka flag
		vTaskDelay(25);
	}
	close_valve(0);
	vTaskDelay(1000);


	// mix fertilizers
	for (i=0; i<FMP_PROGRAMS_AMOUNT; i++) {

		// count current N value
		uint16_t n=0;
		uint32_t curN=0, interval=0, startime=0;
		addr = WP_OFFSET+progId*WP_SIZE+WP_INTERVAL_SHIFT;
		interval = EE_ReadWord(addr);
		addr = WP_OFFSET+progId*WP_SIZE+WP_START_TIME_SHIFT;
		startime = EE_ReadWord(addr);
		addr = FMP_OFFSET+i*FMP_SIZE+FMP_TRIG_FREQUENCY_SHIFT;
		EE_ReadVariable(addr, &n);
		curN = (RTC_GetCounter()-startime)/interval;
		uint8_t rest = curN/n;

		uint16_t enabled=0;
		addr = FMP_OFFSET+i*FMP_SIZE+FMP_2_WP_ASSOC_SHIFT;
		EE_ReadVariable(addr, &fmpLink);
		addr = FMP_OFFSET+i*FMP_SIZE+FMP_ENABLE;
		EE_ReadVariable(addr, &enabled);
		if (fmpLink==progId && enabled==1 && rest==0) {
			run_fertilizer_mixer(i);
		}
		vTaskDelay(10);
	}

	// run watering
	addr = WP_OFFSET+progId*WP_SIZE+WP_DURATION_SHIFT;
	EE_ReadVariable(addr, &wateringDuration);
	addr = WP_OFFSET+progId*WP_SIZE+WP_WATERING_PUMP_PLUG_ID;
	EE_ReadVariable(addr, &wateringPlugId);
	vTaskDelay(10);
//	now=0;
	now = RTC_GetCounter();
	end = RTC_GetCounter() + wateringDuration;
	vTaskDelay(10);
	addr = WP_OFFSET+progId*WP_SIZE+BOTTOM_WATER_SENSOR_SHIFT;
	EE_ReadVariable(addr, &sensorId);
	i=0;
	waterSensorFlag = 1;
//	uint8_t cTimerStateFlag = 0;
	while (now<end) {
		vTaskDelay(10);
		waterSensorFlag=waterSensorStateFlags&(1<<sensorId);	// check if water level reached top sensor
		waterSensorFlag>>=sensorId;	// ostavit' toka flag
		i=cTimerStateFlags&1;	// HARDCODE!!! (0)
		if ((lastTime<RTC_GetCounter()) && i==1 && waterSensorFlag==1) {
			now++;
			lastTime=RTC_GetCounter();
		}
		vTaskDelay(5);
		plugStateSet(wateringPlugId, (1&i&waterSensorFlag));
	}
	plugStateSet(wateringPlugId, 0);
	// drain the rest of the tank

	vTaskDelay(10);
	open_valve(DRAIN_VALVE_ID);

	addr = WP_OFFSET+progId*WP_SIZE+WP_LAST_RUN_SHIFT;
	EE_WriteWord(addr, RTC_GetCounter());	// write last run time

	wpStateFlags &= ~(1<<progId); // sbrosit' flag
	vTaskDelay(200);
}

void run_fertilizer_mixer(uint8_t progId){
	uint16_t dosingTime, dosingPumpId, circulationMixingTime, addr;
	uint32_t dosingEndTime=0;

	addr = FMP_OFFSET+progId*FMP_SIZE+FMP_DOSING_TIME_SHIFT;
	EE_ReadVariable(addr, &dosingTime);

	addr = FMP_OFFSET+progId*FMP_SIZE+FMP_DOSING_PUMP_ID_SHIFT;
	EE_ReadVariable(addr, &dosingPumpId);

	addr = FMP_OFFSET+progId*FMP_SIZE+FMP_CIRCULATION_MIXING_TIME_SHIFT;
	EE_ReadVariable(addr, &circulationMixingTime);

	dosingEndTime = RTC_GetCounter()+dosingTime;
	enable_dosing_pump(dosingPumpId, 1);
	while (RTC_GetCounter()<dosingEndTime) {
		vTaskDelay(10);
	}
	enable_dosing_pump(dosingPumpId, 0);
	run_circulation_pump(circulationMixingTime);
}

void run_circulation_pump(uint16_t time){
	uint32_t endTime=0;
	endTime = RTC_GetCounter()+time;
	plugStateSet(circulationPumpId, 1);
	while (RTC_GetCounter()<endTime) {
		vTaskDelay(10);
	}
	plugStateSet(circulationPumpId, 0);
}


void enable_dosing_pump2(uint8_t pumpId, uint8_t state){
	if (state==1) {
		dosingPumpStateFlags |= (1<<pumpId);
	}
	else {
		dosingPumpStateFlags &= ~(1<<pumpId);
	}

	state = 1-state;	// 0% pwm means running motor

	if (pumpId==0) {
		TIM3->CCR1 = state*1000;
	}
	if (pumpId==1) {
		TIM3->CCR2 = state*1000;
	}
	if (pumpId==2) {
		TIM3->CCR3 = state*1000;
	}
	if (pumpId==3) {
		TIM3->CCR4 = state*1000;
	}
}

void enable_dosing_pump(uint8_t pumpId, uint8_t state){
//	uint8_t prcnt = 0;
	if (state>0) {
		dosingPumpStateFlags2 |= (1<<pumpId);
	}
	else {
		dosingPumpStateFlags2 &= ~(1<<pumpId);
	}

	if (state==1) {
		state = 0;
	}
	else {
		state = 100 - state;	// 0 - running motor, 100 - stopped
	}

	if (pumpId==0) {
		TIM3->CCR1 = state*10;
	}
	if (pumpId==1) {
		TIM3->CCR2 = state*10;
	}
	if (pumpId==2) {
		TIM3->CCR3 = state*10;
	}
	if (pumpId==3) {
		TIM3->CCR4 = state*10;
	}
}

void get_fertilizer(uint8_t fertId, uint8_t secs){	// secs - number of seconds to run the fertId pump
	uint32_t reach = 0;
	// to make counting more precise wait until the next second STARTS
	reach = RTC_GetCounter()+1;
	while (RTC_GetCounter()<reach) {
		vTaskDelay(2);
	}
	reach = RTC_GetCounter()+secs;
	enable_dosing_pump(fertId,1);	// enable dosing pump
	while (RTC_GetCounter()<reach) {	// now dose the fertilizer within secs seconds
		vTaskDelay(2);		//
	}
	enable_dosing_pump(fertId,0);
}

void watering_program_trigger(void *pvParameters){
	uint32_t curtime, lastRun, interval, diff, startTime, endTime;
	uint16_t addr;
	uint8_t wpStateFlag=0, progId, enabled;
	// reinit valves
	open_valve(DRAIN_VALVE_ID);
	close_valve(0);

	while (1) {
//
		for (progId=0; progId<3; progId++){		// HARDCODE!
			vTaskDelay(10);
			addr = WP_OFFSET+progId*WP_SIZE+WP_LAST_RUN_SHIFT;
			lastRun = EE_ReadWord(addr);
			curtime = RTC_GetCounter();
			addr = WP_OFFSET+progId*WP_SIZE+WP_INTERVAL_SHIFT;
			interval = EE_ReadWord(addr);
			diff = curtime-lastRun;
			addr = WP_OFFSET+progId*WP_SIZE+WP_START_TIME_SHIFT;
			startTime = EE_ReadWord(addr);	// program start time point
			addr = WP_OFFSET+progId*WP_SIZE+WP_END_TIME_SHIFT;
			endTime = EE_ReadWord(addr);	// program end time point
			addr = WP_OFFSET+progId*WP_SIZE+WP_ENABLE_SHIFT;
			EE_ReadVariable(addr, &enabled);	// program start time point

			vTaskDelay(10);
			wpStateFlag=wpStateFlags&(1<<progId);	// check if WP active now
			wpStateFlag>>=progId;	// ostavit' toka flag
			if (diff>interval && curtime>startTime && curtime<endTime && enabled==1){
				run_watering_program(progId);
			}
			vTaskDelay(10);
		}
	}

}




void dht_init(void){
	// RCC Config
	  /* TIM15 clock enable */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);

	  // GPIO Config
	  GPIO_InitTypeDef GPIO_InitStructure;

	  /* TIM15 channel 2 pin (PB15) configuration */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	// NVIC Config

	  GPIO_PinRemapConfig(GPIO_Remap_TIM15, ENABLE);
	  NVIC_InitTypeDef NVIC_InitStructure;

	   /* Enable the TIM15 global Interrupt */

	  NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM15_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	   TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	   TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	   TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	   TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	   TIM_ICInitStructure.TIM_ICFilter = 0x0;

	   TIM_PWMIConfig(TIM15, &TIM_ICInitStructure);

	   /* Select the TIM15 Input Trigger: TI2FP2 */
	   TIM_SelectInputTrigger(TIM15, TIM_TS_TI2FP2);

	   /* Select the slave Mode: Reset Mode */
	   TIM_SelectSlaveMode(TIM15, TIM_SlaveMode_Reset);

	   /* Enable the Master/Slave Mode */
	   TIM_SelectMasterSlaveMode(TIM15, TIM_MasterSlaveMode_Enable);

	   /* TIM enable counter */
	   TIM_Cmd(TIM15, ENABLE);

	   /* Enable the CC2 Interrupt Request */
	   TIM_ITConfig(TIM15, TIM_IT_CC2, ENABLE);

}

void dht_init_out(void){	// init GPIO pin as OUTput
	 GPIOB->CRH      &= ~GPIO_CRH_CNF15;		// ... and PB15 for DHT triggering
     GPIOB->CRH   |= GPIO_CRH_MODE15_0;
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
  /* Clear TIM3 Capture compare interrupt pending bit */
  TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

  /* Get the Input Capture value */
  IC2Value = TIM_GetCapture2(TIM3);

  if (IC2Value != 0)
  {
    /* Duty cycle computation */
    DutyCycle = (TIM_GetCapture1(TIM3) * 100) / IC2Value;

    /* Frequency computation */
    Frequency = SystemCoreClock / IC2Value;
  }
  else
  {
    DutyCycle = 0;
    Frequency = 0;
  }
  dht_bit_position++;		// holds current DHT response bit sequence number

	if (dht_bit_position>dht_shifter && dht_data_ready==0) {
		if (DutyCycle>25 && DutyCycle<35) {
			dht_data[dht_byte_pointer] &= ~(1<<(dht_bit_pointer));	// reset bit in dht_data[i]
		}
		else {
			dht_data[dht_byte_pointer] |= (1<<(dht_bit_pointer)); // set bit
		}
		if (dht_bit_pointer==0){
			dht_bit_pointer=7;
			dht_byte_pointer++;
			if (dht_byte_pointer==4) {
				dht_data_ready=1;
				dht_byte_pointer = 0;
				dht_bit_pointer = 7;
//				dht_bit_position = 0;
			}
		}
		else {
			dht_bit_pointer--;
		}
	}

}

void TIM1_BRK_TIM15_IRQHandler(void)		// DHT moved from PA7 to PB15. 11.07.2013
{
  /* Clear TIM3 Capture compare interrupt pending bit */
  TIM_ClearITPendingBit(TIM15, TIM_IT_CC2);

  /* Get the Input Capture value */
  IC2Value = TIM15->CCR2;

  if (IC2Value != 0)
  {
    /* Duty cycle computation */
    DutyCycle = ((TIM15->CCR1) * 100) / IC2Value;

    /* Frequency computation */
 //   Frequency = SystemCoreClock / IC2Value;
  }
  else
  {
    DutyCycle = 0;
    Frequency = 0;
  }



  // fill one more value of dht bits array
//  dht_bit_array[dht_bit_position]=TIM_GetCapture1(TIM15);
//  dht_bit_array[dht_bit_position]=DutyCycle;
//  dht_bit_position++;
//  dht_bit_array[dht_bit_position]=TIM15->CCR2;
  dht_bit_position++;		// holds current DHT response bit sequence number

	if (dht_bit_position>dht_shifter && dht_data_ready==0) {
		if (DutyCycle>25 && DutyCycle<35) {
			dht_data[dht_byte_pointer] &= ~(1<<(dht_bit_pointer));	// reset bit in dht_data[i]
		}
		else {
			dht_data[dht_byte_pointer] |= (1<<(dht_bit_pointer)); // set bit
		}
		if (dht_bit_pointer==0){
			dht_bit_pointer=7;
			dht_byte_pointer++;
			if (dht_byte_pointer==4) {
				dht_data_ready=1;
				dht_byte_pointer = 0;
				dht_bit_pointer = 7;
//				dht_bit_position = 0;
			}
		}
		else {
			dht_bit_pointer--;
		}
	}

}


void TIM1_UP_TIM16_IRQHandler(void)		// DHT moved from PA7 to PB15. 11.07.2013
{
  /* Clear TIM16 Capture compare interrupt pending bit */
  TIM_ClearITPendingBit(TIM16, TIM_IT_CC1);

  /* Get the Input Capture value */
  sonar_read[1] = TIM16->CNT;
  TIM16->CNT = 0;
}

void TIM1_TRG_COM_TIM17_IRQHandler(void)		// DHT moved from PA7 to PB15. 11.07.2013
{
  /* Clear TIM16 Capture compare interrupt pending bit */
  TIM_ClearITPendingBit(TIM17, TIM_IT_CC1);
//  TIM17->SR &= ~TIM_SR_CC1IF;
  /* Get the Input Capture value */
  if (!(GPIOB->IDR & (1<<9))) {
	  sonar_read[0]=SONAR1_TIM->CNT;
  }
  SONAR1_TIM->CNT = 0;
}


void dht_get_data(void){	// function starts getting data from DHT22 sensor
//	SONAR1_TIM->EGR |= TIM_EGR_CC1G;
	vTaskDelay(25);
	  uint8_t i;
	  for (i=0;i<5;i++) {
		dht_data[i]=0;
	  }
//	  dht_bit_ready = 0;
	  dht_bit_position = 0;
	  dht_data_ready=0;
	  dht_init_out();
	  DHT_0;
//	  GPIOA->BRR = (1<<DHT_TRIG_PLUG);		// set 0
	  vTaskDelay(5);
	  DHT_1;
	  dht_init();
	  vTaskDelay(200);

//	  dht_bit_ready = 1;
//	  dht_bit_position = 0;
//	  GPIOA->BSRR = (1<<DHT_TRIG_PLUG);	// set 1
	  vTaskDelay(5);
	  dht_conv_data();
}

void dht_conv_data(void){ // convert DHT impulse lengths array into numbers and strings of T and rH
	vTaskDelay(10);
	if (dht_data_ready==1) {

		vTaskDelay(10);

		DHTValue.DHT_Humidity = dht_data[0]*256+dht_data[1];
		DHTValue.DHT_Temperature = dht_data[2]*256+dht_data[3];
		DHTValue.DHT_CRC = dht_data[5];

		if (DHTValue.DHT_Humidity>rhWindowBottom && DHTValue.DHT_Humidity<rhWindowTop) {
			rhUnderOver=0;
		}
		if (DHTValue.DHT_Humidity<rhWindowBottom) {
			rhUnderOver=1;
		}
		if (DHTValue.DHT_Humidity>rhWindowTop) {
			rhUnderOver=2;
		}

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

		dht_data_ready = 0;
	}
}

void dht_arr_displayer(void){
//	dht_get_data();
	uint8_t button=0;
	while (button!=BUTTON_OK) {
//		dht_get_data();
		vTaskDelay(30);
#ifdef TEST_MODE
		button=readButtons();
		Lcd_goto(0,0);
		Lcd_write_digit(dht_shifter);
		Lcd_write_str(": ");
//		Lcd_write_digit(dht_data[arr_pointer]/100);
//		Lcd_write_digit(dht_bit_array[arr_pointer]);
		if (button==BUTTON_FWD) {
			dht_shifter++;
		}
		if (button==BUTTON_BCK){
			dht_shifter--;
		}
#endif


		Lcd_goto(1,0);
		Lcd_write_str("T:");
		copy_arr(&dht_t_str, &LCDLine2, 4, 2);
		Lcd_goto(1,7);
		Lcd_write_str("rH:");
		copy_arr(&dht_rh_str, &LCDLine2, 4, 10);
	}
	Lcd_clear();
}


uint8_t adc2ph(uint16_t adcval){
	return ((adcval - ph0)*10)/cdel;
}

uint16_t ph2adc(uint8_t ph){
	return (ph*cdel)/10+ph0;
}

void phStabSettings(void){	// function guides user through setting the pH level stabilizer window
	uint16_t phadcvalue=0;
	uint8_t phval=0;
	Lcd_clear();
	Lcd_write_str("Set top pH lvl");
	vTaskDelay(10);
	EE_ReadVariable(PH_WINDOW_TOP, &phadcvalue);
	phval = adc2ph(phadcvalue);
	phval = readPhVal(phval);
	vTaskDelay(10);
	EE_WriteVariable(PH_WINDOW_TOP, ph2adc(phval));
	Lcd_clear();
	Lcd_write_str("Set btm lvl");
	vTaskDelay(10);
	EE_ReadVariable(PH_WINDOW_BOTTOM, &phadcvalue);
	phval = adc2ph(phadcvalue);
	phval = readPhVal(phval);
	vTaskDelay(10);
	EE_WriteVariable(PH_WINDOW_BOTTOM, ph2adc(phval));
	Lcd_clear();
}

uint8_t readPercentVal(uint8_t value){
	uint8_t button=0;
	char prcntStr[5];
	value %= 101;	// drop all except 0..100
	while (button!=BUTTON_OK) {
		button = readButtons();
		Lcd_goto(1,3);
		prcntStr[0] = (value%1000)/100;
		prcntStr[1] = (value%100)/10;
		prcntStr[2] = (value%10);
		Lcd_write_str("<");
		copy_arr(prcntStr, LCDLine2, 3, 6);
		Lcd_goto(1,12);
		Lcd_write_str(">");
		vTaskDelay(10);
		if (button==BUTTON_FWD) {
			if (value<100) {
				value++;
			}
			else {
				value=0;
			}
		}
		if (button==BUTTON_BCK) {
			if (value<1) {
				value=0;
			}
			else {
				value--;
			}
		}
		vTaskDelay(10);
	}
	return value;
}

void hygroStatSettings(void){
	uint16_t rh_value=0;
	uint8_t rh_val=0;
	Lcd_clear();
	Lcd_write_str("Set top rH lvl");
	vTaskDelay(10);
	EE_ReadVariable(RH_WINDOW_TOP, &rh_value);
//	rh_val = adc2ph(phadcvalue);
	rh_val = readPercentVal(rh_value);
	vTaskDelay(10);
	EE_WriteVariable(RH_WINDOW_TOP, rh_val);
	Lcd_clear();
	Lcd_write_str("Set bottom lvl");
	vTaskDelay(10);
	EE_ReadVariable(RH_WINDOW_BOTTOM, &rh_value);
//	phval = adc2ph(phadcvalue);
	rh_val = readPercentVal(rh_value);
	vTaskDelay(10);
	EE_WriteVariable(RH_WINDOW_BOTTOM, rh_val);
	Lcd_clear();
}

void convPh2str(uint8_t ph, char* phstr){
	phstr[3] = ph-(ph/10)*10+48;
	vTaskDelay(5);
	ph/=10;
	phstr[2]=46;
	phstr[1] = ph-(ph/10)*10+48;
	vTaskDelay(5);
	ph/=10;
	phstr[0]=ph-(ph/10)*10+48;
	phstr[4] = 0;	// konec stroki
}

uint8_t readPhVal(uint8_t value){
	uint8_t button=0;

	char phStr[5];
	while (button!=BUTTON_OK) {
		button = readButtons();
		Lcd_goto(1,3);
		convPh2str(value, &phStr);
		Lcd_write_str("<");
		copy_arr(phStr, LCDLine2, 4, 6);
		Lcd_goto(1,12);
		Lcd_write_str(">");
		vTaskDelay(10);
		if (button==BUTTON_FWD) {
			if (value<254) {
				value++;
			}
			else {
				value=0;
			}
		}
		if (button==BUTTON_BCK) {
			if (value<1) {
				value=0;
			}
			else {
				value--;
			}
		}
		vTaskDelay(10);
	}
	return value;
}

void Lcd_write_16int(uint16_t d){
	int i;
	char out[5];
	out[5] = '\0';
	out[4] = '0' + ( d       )    % 10;
	out[3] = '0' + ( d /= 10 )    % 10;
	out[2] = '0' + ( d /= 10 )    % 10;
	out[1] = '0' + ( d /= 10 )    % 10;
	out[0] = '0' + ( d /= 10 )    % 10;
	copy_arr(&out, LCDLine2, 6, 0);
	for (i=0;i<5;i++) {
		Lcd_write_str(out[i]);
	}
}

void Lcd_write_32int(uint32_t d){
//	int i;
	char tmpstr[10];
	int32str(d, &tmpstr);
	copy_arr(&tmpstr, LCDLine2, 11, 0);
}

void phStab(){
	// compare current pH with top level
/*	if (curph>pHstab_topLevel && pHstab_min_interval) {
		addAcid();
	}
	if (curph<pHstab_bottomLevel){
		addBase();
	} */
}

void enablePlug5ms(uint8_t plug, uint16_t amount){	// 1 amount = 5ms
	 RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;   //enable TIM2 clock
	 TIM2->PSC     = 40000-1;               //set divider for 5 milliseconds
	 TIM2->ARR = amount;
	 TIM2->CR1     = TIM_CR1_OPM;          //one pulse mode
	 TIM2->CR1 |= TIM_CR1_CEN;
	 plugStateSet(plug, 1);
	 while((TIM2->SR & TIM_SR_UIF)==0){
		 vTaskDelay(10);
	 }
	 plugStateSet(plug, 0);
}

uint32_t measureDelay(void){	// maximum 300+ (65535/200) seconds
	 RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;   //enable TIM2 clock
	 TIM2->PSC     = 40000-1;               //set divider for 5 milliseconds
	 TIM2->CR1     = TIM_CR1_OPM;          //one pulse mode
	 TIM2->CR1 |= TIM_CR1_CEN;
	 uint8_t button=0;
	 while (button!=BUTTON_OK) {
		 button = readButtons();
		 // display counter value
		 Lcd_goto(1,0);
		 Lcd_write_16int(TIM2->CNT);
	 }
	 return TIM2->CNT;
}



typedef struct
{
    uint32_t quot;
    uint8_t rem;
} divmod10_t;

inline static divmod10_t divmodu10(uint32_t n)
{
    divmod10_t res;
// ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¾ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¼ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¾ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¶ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â°ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂµÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¼ ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â° 0.8
    res.quot = n >> 1;
    res.quot += res.quot >> 1;
    res.quot += res.quot >> 4;
    res.quot += res.quot >> 8;
    res.quot += res.quot >> 16;
    uint32_t qq = res.quot;
// ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â´ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂµÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â»ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¸ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¼ ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â° 8
    res.quot >>= 3;
// ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â²ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¹ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¸ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â»ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂµÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¼ ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¾ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€šÃ‚Â¦ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â°ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€šÃ‚Â¦ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¾ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Âº
    res.rem = n - ((res.quot << 1) + (qq & ~7ul));
// ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂºÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¾ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¦ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¦ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂµÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂºÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€šÃ‚Â¦ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¸ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¦ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¾ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚ÂµÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¼ ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¾ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€šÃ‚Â¦ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â°ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€šÃ‚Â¦ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¾ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Âº ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¸ ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â°ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€šÃ‚Â¦ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¾ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Âµ
    if(res.rem > 9)
    {
        res.rem -= 10;
        res.quot++;
    }
    return res;
}



/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[2] = {
			CIRCULATION_PUMP_ID_ADDR,
	/*		RH_WINDOW_BOTTOM,
			RH_WINDOW_TOP,
			PH_WINDOW_BOTTOM,
			PH_WINDOW_TOP,
			EC1413_ADDR,
			SD_LOG_INTERVAL,
			PH_BUFF_SIZE,
			PH_INTERVAL,
			PH7_ADDR, */
			PH4_ADDR
};

void fill_virtAddrTab(void){
	uint8_t i;
	for (i=0; i<17; i++) {
		VirtAddVarTab[i]=WP_OFFSET+i;
	}
	for (i=0; i<17; i++) {
		VirtAddVarTab[i+17]=FMP_OFFSET+i;
	}
	for (i=0; i<17; i++) {
		VirtAddVarTab[i+17]=EE_PLUG_SETTINGS+i;
	}
	for (i=0; i<15; i++) {
		VirtAddVarTab[i+17]=EE_CTIMER_DURATION+i;
	}

	for (i=0; i<8; i++) {
		VirtAddVarTab[i+17]=BUTTON_RANGES_START_ADDR+i;
	}

	for (i=0; i<8; i++) {
		VirtAddVarTab[i+17]=EE_TIMER1_ON+i;
	}

}

#define MENURECS	34

#ifndef TEST_MODE
// menu items
const char menuItemArray[MENURECS][18]=
{
		{"MONITOR MODE"},		// 0
		{"TIMERS"},				// 1
		{"Timer 1"},			// 2
		{"Timer 2"},			// 3
		{"Timer 3"},			// 4
		{"SET CLOCK"},			// 5
		{"CYCLIC TIMERS"},			// 6
		{"C Timer 1"},			// 7
		{"C Timer 2"},			// 8
		{"C Timer 3"},			// 9
		{"PLUG SETTINGS"},			// 10
		{"Plug 1"},			// 11
		{"Plug 2"},			// 12
		{"Plug 3"},			// 13
		{"pH-monitor"},		// 14
		{"Calibration"},	// 15
		{"pH-stabilizer"},	// 16

		{"EC-monitor"},		// 17
		{"Calibration"},	// 18
		{"EC-stabilizer"},	// 19

		{"DAYLIGHT SENSOR"},// 20
		{"Tank lvl. keeper"},	// 21
		{"Keeping window"},	// 22
		{"Get water"},	// 23
		{"Add Light"},	// 24
		{"Temp. & humidity"},	// 25
		{"Hygrostat"},	// 26
		{"Thermostat"},	// 27
		{"Valve test"},	// 28
		{"Watering progs"},	// 29
		{"Fertilization"},	// 30
		{"Button test"},	// 31
		{"pH&EC test"},	// 32
		{"Start watering"}	// 33

};

// 0 - nr zapisi, 1 - link na tekst, 2 - <, 3 - >, 4 - OK, 5 - CNCL, 6 - tip zapisi (0 - folder, 1 - program)
const int fatArray[MENURECS][7]=
{
		{0,	0,	33,	1,	1,	0,	1},
		{1,	1,	0,	5,	2,	1,	1},
		{2,	2,	4,	3,	2,	1,	1},
		{3,	3,	2,	4,	3,	1,	1},
		{4,	4,	3,	2,	4,	1,	1},
		{5,	5,	1,	6,	5,	5,	1},
		{6,	6,	5,	10,	7,	6,	0},
		{7,	7,	9,	8,	6,	6,	1},
		{8,	8,	7,	9,	7,	6,	1},
		{9,	9,	8,	7,	8,	6,	1},
		{10,10,	6,	14,	11,	10,	0},
		{11,11,	13,	12,	9,	10,	1},
		{12,12,	11,	13,	10,	10,	1},
		{13,13,	12,	11,	11,	10,	1},
		{14,14,	10,	17,	15,	14,	0},
		{15,15,	16,	16,	15,	14,	1},
		{16,16,	15,	15,	16,	14,	1},
		{17,17,	14,	20,	18,	17,	0},
		{18,18,	19,	19,	23,	17,	1},	// ec calibration
		{19,19,	18,	18,	24,	17,	1},	// ec stab settings




		{20,20,	17,	21,	14,	17,	1},	// daylight sensor
		{21,21,	20,	25,	22,	18,	0},	// combi timers
		{22,22,	24,	23,	15,	22,	1},	// Tank level Keeping window setup
		{23,23,	22,	24,	16,	23,	1},	// t+2ct
		{24,24,	23,	22,	17,	24,	1},	// add light
		{25,25,	21,	26,	17,	25,	1},	// temp and humidity
		{26,26,	25,	27,	18,	26,	1},	// hygrostat
		{27,27,	26,	28,	19,	27,	1},	// thermostat
		{28,28,	27,	29,	20,	28,	1},	// valve test
		{29,29,	28,	30,	21,	29,	1},	// watering progs
		{30,30,	29,	31,	22,	30,	1},	// fertilization
		{31,31,	30,	32,	25,	31,	1},	// button test
		{32,32,	31,	33,	26,	32,	1},	// pH and ec test
		{33,33,	32,	0,	27,	33,	1}	// test function

};

#endif

#ifdef TEST_MODE
// menu items
const char menuItemArray[MENURECS][18]=
{
		{"MONITOR MODE"},		// 0
		{"TIMERS"},				// 1
		{"Tests"},	    		// 2
		{"Plug test"},			// 3
		{"PSI sensor cal."},			// 4
		{"SET CLOCK"},			// 5
		{"CYCLIC TIMERS"},			// 6
		{"C Timer 1"},			// 7
		{"C Timer 2"},			// 8
		{"C Timer 3"},			// 9
		{"PLUG SETTINGS"},			// 10
		{"Plug 1"},			// 11
		{"Plug 2"},			// 12
		{"Plug 3"},			// 13
		{"pH-monitor"},		// 14
		{"Calibration"},	// 15
		{"pH-stabilizer"},	// 16

		{"EC-monitor"},		// 17
		{"Calibration"},	// 18
		{"EC-stabilizer"},	// 19

		{"DAYLIGHT SENSOR"},// 20
		{"Tank lvl keeper"},	// 21
		{"Keeper window"},	// 22
		{"Tank tests"},	// 23
		{"Add Light"},	// 24
		{"Temp. & humidity"},	// 25
		{"Hygrostat"},	// 26
		{"Thermostat"},	// 27
		{"Valve test"},	// 28
		{"Watering progs"},	// 29
		{"Fertilization"},	// 30
		{"Button test"},	// 31
		{"pH&EC test"},	// 32
		{"Start watering"}	// 33

};

// 0 - nr zapisi, 1 - link na tekst, 2 - <, 3 - >, 4 - OK, 5 - CNCL, 6 - tip zapisi (0 - folder, 1 - program)
const int fatArray[MENURECS][7]=
{
		{0,	0,	33,	1,	1,	0,	1},
		{1,	1,	0,	2,	2,	1,	1},
		{2,	2,	1,	5,	3,	2,	0},
		{3,	3,	4,	4,	3,	1,	1},
		{4,	4,	3,	3,	4,	1,	1},
		{5,	5,	1,	6,	5,	5,	1},
		{6,	6,	5,	10,	7,	6,	0},
		{7,	7,	9,	8,	6,	6,	1},
		{8,	8,	7,	9,	7,	6,	1},
		{9,	9,	8,	7,	8,	6,	1},
		{10,10,	6,	14,	9,	10,	1},
		{11,11,	13,	12,	9,	10,	1},
		{12,12,	11,	13,	10,	10,	1},
		{13,13,	12,	11,	11,	10,	1},
		{14,14,	10,	17,	15,	14,	0},
		{15,15,	16,	16,	15,	14,	1},
		{16,16,	15,	15,	16,	14,	1},

		{17,17,	14,	20,	18,	17,	0},
		{18,18,	19,	19,	23,	17,	1},	// ec calibration
		{19,19,	18,	18,	24,	17,	1},	// ec stab settings




		{20,20,	17,	21,	14,	17,	1},	// daylight sensor
		{21,21,	20,	25,	22,	18,	0},	// keepers
		{22,22,	24,	23,	28,	22,	1},	// tank keeper level window setup
		{23,23,	22,	24,	23,	23,	1},	// t+2ct
		{24,24,	23,	22,	17,	24,	1},	// add light
		{25,25,	21,	26,	17,	25,	1},	// temp and humidity
		{26,26,	25,	27,	18,	26,	1},	// hygrostat
		{27,27,	26,	28,	19,	27,	1},	// thermostat
		{28,28,	27,	29,	20,	28,	1},	// valve test
		{29,29,	28,	30,	21,	29,	1},	// watering progs
		{30,30,	29,	31,	22,	30,	1},	// fertilization
		{31,31,	30,	32,	25,	31,	1},	// button test
		{32,32,	31,	33,	26,	32,	1},	// pH and ec test
		{33,33,	32,	0,	27,	33,	1}	// test function

};

#endif

char* adc2str(uint_fast16_t d, char* out)
{
	char out2[17];
	int i, k, c;
    out2[16] = '\0';
    out2[15] = '0' + ( d       )    % 10;
    out2[14] = '0' + ( d /= 10 )    % 10;
    out2[13] = '0' + ( d /= 10 )    % 10;
    out2[12] = '0' + ( d /= 10 )    % 10;
    out2[11] = '0' + ( d /= 10 )    % 10;
    out2[10] = '0' + ( d /= 10 )    % 10;
    vTaskDelay(25);
    out2[9] = '0' + ( d /= 10 )    % 10;
    out2[8] = '0' + ( d /= 10 )    % 10;
    out2[7] = '0' + ( d /= 10 )    % 10;
    out2[6] = '0' + ( d /= 10 )    % 10;
    out2[5] = '0' + ( d /= 10 )    % 10;
    out2[4] = '0' + ( d /= 10 )    % 10;
    out2[3] = '0' + ( d /= 10 )    % 10;
    out2[2] = '0' + ( d /= 10 )    % 10;
    out2[1] = '0' + ( d /= 10 )    % 10;
    out2[0] = '0' + ( d /  10 )    % 10;
    k=0;
    c=out2[k];
    c=48;

	vTaskDelay(25);
	while (c==48){
    	k++;
    	c=out2[k];
	}
	vTaskDelay(25);
	for (i=k; i<17; i++){
		out[i-k]=out2[i];
	}

    return out;
}

/* FRESULT sdLog2(void){

	char str[20];
	char tmpstr[15];
	int i;
	uint32_t tmp;
	tmp = RTC_GetCounter();
	FRESULT resp=0;

		int32str(tmp, &tmpstr);
		for (i=0; i<10; i++){
			str[i]=tmpstr[i];
		}
		str[10]=44;
		int32str(JDR_BUTTONS, &tmpstr);
		for (i=0;i<4;i++){
			str[i+11]=tmpstr[i+6];
		}
		str[16]=44;
		str[17]=13;
		resp = string2log(&str, 17);
	return resp;
} */

void Delay_us(uint32_t delay){
	uint32_t del=delay*250; while (del--){}
}

/* uint16_t get_average_adc(uint8_t amount){
	uint8_t i=0;
	uint32_t sum = 0;
	for (i=0; i<amount; i++) {
		sum += JDR_BUTTONS;
	}
	return sum/amount;
} */

void buttonCalibration(void){	// buttons calibration function
	uint16_t button_val[4], diff;
	Lcd_clear();
	Lcd_goto(0,0);
	Lcd_write_arr("<", 1);
	Delay_us(30000);
	adcAverager();
	Delay_us(100);
//	uint16_t adcAverage[4];
	button_val[0] = adcAverage[ADC_AVG_BUTTONS];
	Lcd_goto(0,0);
	Lcd_write_arr("OK", 2);
	Delay_us(30000);
	adcAverager();
	Delay_us(100);
	button_val[1] = adcAverage[ADC_AVG_BUTTONS];
	Lcd_goto(0,0);
	Lcd_write_arr("CANCEL", 6);
	Delay_us(30000);
	adcAverager();
	Delay_us(100);
	button_val[2] = adcAverage[ADC_AVG_BUTTONS];
	Lcd_clear();
	Lcd_write_arr(">", 1);
	Delay_us(30000);
	adcAverager();
	Delay_us(100);
	button_val[3] = adcAverage[ADC_AVG_BUTTONS];

	if ((button_val[3]>>3)<(button_val[0]>>3)) {
		buttonReverse = 1;
	}
	else if ((button_val[3]>>3)>(button_val[0]>>3)) {
		buttonReverse = 0;
	}
	else {
		// loadButtonSettings();	// no key pressed, loading settings from EEPROM
		buttonReverse = 2;	//means loading button settings from EEPROM
	}
	if (buttonReverse == 0) {
		diff = ((button_val[1]-button_val[0])/2)-5;
		button_ranges[0] = (button_val[0]-diff/2);	// 05.09.13. NxtLn commented
//		button_ranges[0] = button_val[0]-diff;
		button_ranges[1] = button_val[0]+diff;
		button_ranges[2] = button_val[1]-diff;
		diff = ((button_val[2]-button_val[1])/2)-5;
		button_ranges[3] = button_val[1]+diff;
		button_ranges[4] = button_val[2]-diff;
		diff = ((button_val[3]-button_val[2])/2)-5;
		button_ranges[5] = button_val[2]+diff;
		button_ranges[6] = button_val[3]-diff;
		button_ranges[7] = (button_val[3]+diff/2); 	// 05.09.13. NxtLn commented
//		button_ranges[7] = button_val[3]+diff;
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
	}
	else {
		Lcd_write_str("EEPROM buttons");
		readButtonRanges();
	}

	if (button_ranges[0]>button_ranges[8]) {
				buttonReverse=1;
	}
	Lcd_goto(0,0);
	Lcd_write_str("Complete");
}

void displayAdcValues(void){
#ifdef TEST_MODE
	uint8_t button=0;
	Lcd_clear();
	vTaskDelay(500);
	while (button!=BUTTON_OK){
		button=readButtons();
		Lcd_goto(0,0);
		Lcd_write_str("1:");
		Lcd_write_digit(adcAverage[0]/100);
		Lcd_write_digit(adcAverage[0]);
		Lcd_write_str(" 2:");
		Lcd_write_digit(adcAverage[1]/100);
		Lcd_write_digit(adcAverage[1]);
		Lcd_goto(1,0);
		Lcd_write_str("3:");
		Lcd_write_digit(adcAverage[2]/100);
		Lcd_write_digit(adcAverage[2]);
		Lcd_write_str(" 4:");
		Lcd_write_digit(adcAverage[3]/100);
		Lcd_write_digit(adcAverage[3]);
		vTaskDelay(20);
	}
	Lcd_clear();
#endif
}

void displayAdcValues_bak(void){
#ifdef TEST_MODE
	uint8_t button=0;
	Lcd_clear();
	vTaskDelay(500);
	while (button!=BUTTON_OK){
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
#endif
}

void display_usart_rx2(void){	// another usart test fr displaying cmdbuf contents
		uint8_t button=0, i=0;
		Lcd_clear();
		vTaskDelay(500);
		while (button!=BUTTON_OK){
			button=readButtons();
			Lcd_goto(0,0);
			vTaskDelay(2);
			for (i=0; i<5;i++) {
				Lcd_write_8b(cmdbuf[i]);
			}
			vTaskDelay(1);
			Lcd_goto(1,0);
			Lcd_write_8b(cmdbuf[5]);
			Lcd_write_8b(cmdbuf[6]);

			Lcd_write_str(" ");
			Lcd_write_digit(rxm_state);
			Lcd_write_digit(RxCounter);
			Lcd_write_digit(prefixDetectionIdx);
			Lcd_write_digit(packet_length);
			vTaskDelay(20);
		}
		Lcd_clear();
}

void display_usart_rx(void){
		uint8_t button=0;
		Lcd_clear();
		vTaskDelay(500);
		while (button!=BUTTON_OK){
			button=readButtons();
			Lcd_goto(0,0);
			vTaskDelay(2);
#ifndef PRODUCTION_BUILD
			for (i=0; i<5;i++) {
				Lcd_write_8b(RxBuffer[i]);
			}
#endif
			vTaskDelay(1);
			Lcd_goto(1,0);
#ifndef PRODUCTION_BUILD
			Lcd_write_8b(RxBuffer[5]);
			Lcd_write_8b(RxBuffer[6]);
			Lcd_write_str(" ");
#endif
//			Lcd_write_digit(RxCounter);
			Lcd_write_digit(rx_packet_crc);
#ifndef PRODUCTION_BUILD
			copy_arr(&RxBuffer, &LCDLine2, 7,9);
#endif
//			Lcd_write_arr(&RxBuffer, 7);
			vTaskDelay(20);
		}
		Lcd_clear();
}

void display_usart_tx(void){
		uint8_t button=0, i=0;
		Lcd_clear();
		vTaskDelay(500);
		while (button!=BUTTON_OK){
			button=readButtons();
			Lcd_goto(0,0);
			vTaskDelay(2);
			for (i=0; i<5;i++) {
				Lcd_write_8b(TxBuffer[i]);
			}
			vTaskDelay(1);
			Lcd_goto(1,0);
			Lcd_write_8b(TxBuffer[5]);
			Lcd_write_8b(TxBuffer[6]);
			Lcd_write_str(" ");
			Lcd_write_digit(TxCounter);
			copy_arr(&TxBuffer, &LCDLine2, 7,9);
//			Lcd_write_arr(&RxBuffer, 7);
			vTaskDelay(20);
		}
		Lcd_clear();
}


void saveButtonRanges(void){
	uint8_t i;
	for (i=0; i<8; i++) {
		EE_WriteVariable(BUTTON_RANGES_START_ADDR+i, button_ranges[i]);
	}
}

void readButtonRanges(void){
	uint8_t i;
	for (i=0; i<8; i++) {
		EE_ReadVariable(BUTTON_RANGES_START_ADDR+i, &button_ranges[i]);
	}
}

static void sdLog(void *pvParameters){	// store current device data into log
	char tmpstr[15];
	int i;
	uint32_t tmp;
	vTaskDelay(150);
	while(1){
	tmp = RTC_GetCounter();
		vTaskDelay(2);
		if (tmp>lastWriteTime) {
			lastWriteTime=RTC_GetCounter();
			int32str(tmp, &tmpstr);
			vTaskDelay(2);
			for (i=0; i<10; i++){
				log_str[i]=tmpstr[i];	// current time
			}
			vTaskDelay(2);
			log_str[10]=44;	// ascii comma

			vTaskDelay(2);
			vTaskSuspendAll();
			for (i=0;i<4;i++){
				log_str[i+11]=curphstr[i];	// current pH value
				log_str[i+16]=curecstr[i];	// current EC value
			}
			xTaskResumeAll();
			vTaskDelay(2);
			log_str[15]=44;
			log_str[20]=44;
			uint16_t flg=0;
	    	for (i=0; i<3; i++) {
	    		flg=plugStateFlags&(1<<i);	// plug state flags
	    		flg>>=i;
	    		log_str[i+21]=flg+48;
	    	}

	    	log_str[24]=44;	// ascii comma
	    	copy_arr(&dht_t_str, &log_str, 4, 25);	// temperature
	    	log_str[29]=44;	// ascii comma
	    	copy_arr(&dht_rh_str, &log_str, 4, 30);	// humidity
//			str[20]=13;		// CR
	    	log_str[34]=44;	// ascii comma
	    	for (i=0; i<3; i++) {
	    		flg=waterSensorStateFlags&(1<<i);	// water sensor state
	    		flg>>=i;
	    		log_str[i+35]=flg+48;
	    	}
	    	log_str[38]=44;	// ascii comma
	      	for (i=0; i<3; i++) {
	      		flg = VALVE_SENSOR_PORT->IDR & (1<<i+VALVE_SENSOR_GPIO_SHIFT); // valve sensor actual state
	      		flg>>=i+VALVE_SENSOR_GPIO_SHIFT;
	      		log_str[i+39]=flg+48;
	    	}

	      	log_str[42]=44;	// ascii comma

#ifdef USE_VALVES
	    	for (i=0; i<3; i++) {
	    		flg=valveFlags&(1<<i);	// valves sensors flags
	    		flg>>=i;
	    		log_str[i+43]=flg+48;
	    	}

	    	log_str[46]=44;	// ascii comma
#endif
#ifndef USE_VALVES
	    	for (i=0; i<3; i++) {
	    		log_str[i+43]=48;
	    	}

	    	log_str[46]=44;	// ascii comma
#endif

	    	for (i=0; i<3; i++) {
	    		flg = VALVE_MOTOR_PORT->ODR & (1<<i+VALVE_MOTOR_GPIO_SHIFT); // valve motor actual state
	    		flg>>=i+VALVE_MOTOR_GPIO_SHIFT;
	    		log_str[i+47]=flg+48;
	    	}


	    	log_str[50]=44;	// ascii comma
#ifdef USE_VALVES
	    	for (i=0; i<3; i++) {
	    		flg = valveFlags & (1<<i); // valve motor state flags
	    		flg>>=i;
	    		log_str[i+51]=flg+48;
	    	}
#endif
#ifndef USE_VALVES
	    	for (i=0; i<3; i++) {
	    		log_str[i+51]=48;
	    	}
#endif

	    	log_str[54]=44;	// ascii comma

	    	// for (i=0; i<3; i++) {
	    		// flg = dosingPumpStateFlags2;
	    		log_str[55]=dosingPumpStateFlags2+48;
	    	// }

	    	log_str[58]=44;	// ascii comma
//	    	for (i=0; i<3; i++) {
//	    		flg = wpStateFlags & (1<<i); // watering program state flags
//	    		flg>>=i;
	    		log_str[i+59]=wpStateFlags+48;
//	    	}






//	    	str[41]=44;	// ascii comma
	    	log_str[62]=13;		// CRC

	    	//for (i=0;i<62;i++) {
	    	//	log_str[62] ^= log_str[i];
	    	//}

	    	log_str[63]=10;		// U
	    	logstr_crc=0;

			string2log(&log_str, 64);
			vTaskDelay(2);
		}//
		vTaskDelay(2);
		dht_get_data();
		tankLevelStab();
		psiStab();
		sonar_ping();
	}
}



FRESULT string2log(char* str, int bytes){
	FRESULT res;
	if (no_sd==0) {
		  static FIL logfile;
		  UINT len;
		  char FileName[]="0:CADIZLOG";
	//	  logfile = malloc(sizeof (FIL));
	//      taskENTER_CRITICAL();
		  vTaskSuspendAll();

		  res = f_open(&logfile, FileName, FA_WRITE | FA_READ);
	//	  vTaskDelay(50);
		  if(res) {
			  res = f_open(&logfile, FileName, FA_WRITE | FA_CREATE_ALWAYS);
	//		  vTaskDelay(50);
			  if(res) return res;
		  };
		  res = f_lseek(&logfile, f_size(&logfile));
		  if(res) return res;

		  res = f_write(&logfile, str, bytes, &len);
		  if(res) return res;
		  res = f_sync(&logfile);
		  if(res) return res;

		  res = f_close(&logfile);
	//      taskEXIT_CRITICAL();
		  xTaskResumeAll();
	}
	else {
		res=0;	// force 0 output for no functioning SD card
	}
      return res;
}

void push2str(char* str1, char* str2, char *out){
	int len1,len2,i;
	len1=sizeof(str1);
	len2=sizeof(str2);
	for (i=0;i<len1;i++) {
		out[i]=str1[i];
	}
	for (i=len1;i<(len1+len2);i++) {
		out[i]=str2[(i-len1)];
	}

}

void calibratePh(){
	int button = 0;
	uint16_t ph;
//	char phstr[4];
	char buffer[5];
//	char *pbuffer;
	while (button!=BUTTON_OK){
		button = readButtons();
//		Lcd_clear();
		vTaskDelay(5);
		Lcd_goto(0,0);
		Lcd_write_str("Use pH7");
//		Lcd_goto(0,8);
		vTaskDelay(5);
//		ph_seven32 = ph_seven;
#ifdef TEST_MODE
		EE_ReadVariable(PH7_ADDR, &ph_seven);
		int10str(ph_seven, &buffer);
		copy_arr(&buffer, &LCDLine1, 4, 8);
#endif
//		Lcd_write_str(buffer);
		Lcd_goto(1,5);
		int10str(adcAverage[AVG_ADC_PH], &buffer);
		Lcd_write_str(buffer);
		vTaskDelay(10);
		Lcd_goto(1,0);
    	getPh();
    	vTaskDelay(10);
    	Lcd_write_str(curphstr);
    	vTaskDelay(100);
    	if (button==button!=BUTTON_CNL) {	// are you sure???

    	}
	}
	ph=adcAverage[AVG_ADC_PH];
	EE_WriteVariable(PH7_ADDR, ph);
	button = 0;

	Lcd_goto(0,0);
    	Lcd_write_str("pH7 value saved");
	vTaskDelay(2000);
	while (button!=BUTTON_OK){
		button = readButtons();
		Lcd_clear();
		vTaskDelay(25);
		Lcd_goto(0,0);
		Lcd_write_str("Use pH4");
		Lcd_goto(0,8);
    	getPh();
		adc2str(ph4, &buffer);
		Lcd_write_str(buffer);
		Lcd_goto(1,5);
		adc2str(adcAverage[AVG_ADC_EC], &buffer);
		Lcd_write_str(buffer);
		vTaskDelay(25);
		Lcd_goto(1,0);
    	vTaskDelay(25);
    	Lcd_write_str(curphstr);
    	vTaskDelay(500);
	}

	ph=adcAverage[AVG_ADC_PH];
	EE_WriteVariable(PH4_ADDR, adcAverage[AVG_ADC_EC]);
	loadSettings();
}

void calibrateEc(void){
	int button = 0;
//	uint16_t ec;
//	char phstr[4];
	char buffer[5];
//	char *pbuffer;
	Lcd_goto(0,0);
	Lcd_write_str("Wipe the probe");
	vTaskDelay(2000);
	while (button!=BUTTON_OK){
		button = readButtons();
//		Lcd_clear();
//		vTaskDelay(5);
		Lcd_goto(0,0);
		vTaskDelay(5);
		Lcd_write_str("OK to set");
//		ph_seven32 = ph_seven;
#ifdef TEST_MODE
		EE_ReadVariable(EC0_ADDR, &ec0);
		int10str(ec0, &buffer);
		copy_arr(&buffer, &LCDLine1, 4, 12);
#endif
//		Lcd_write_str(buffer);
		Lcd_goto(1,6);
		int10str(adcAverage[AVG_ADC_EC], &buffer);
		Lcd_write_str(buffer);
		vTaskDelay(10);
		Lcd_goto(1,0);
    	getEc();
    	vTaskDelay(10);
    	Lcd_write_str("0mS = ");
    	vTaskDelay(100);
	}
	EE_WriteVariable(EC0_ADDR, adcAverage[AVG_ADC_EC]);
	button = 0;
	Lcd_clear();
    Lcd_write_str("0 mS saved");
	vTaskDelay(2000);

	while (button!=BUTTON_OK){
		button = readButtons();
//		Lcd_clear();
		vTaskDelay(5);
		Lcd_clear();
		Lcd_write_str("Use EC1.413");
//		Lcd_goto(0,8);
		vTaskDelay(5);
//		ph_seven32 = ph_seven;
#ifdef TEST_MODE
		EE_ReadVariable(EC1413_ADDR, &ec1413);
		int10str(ec1413, &buffer);
		copy_arr(&buffer, &LCDLine1, 4, 12);
#endif
//		Lcd_write_str(buffer);
		Lcd_goto(1,10);
		int10str(adcAverage[AVG_ADC_EC], &buffer);
		Lcd_write_str(buffer);
		vTaskDelay(10);
		Lcd_goto(1,0);
    	getEc();
    	vTaskDelay(10);
    	Lcd_write_str("1.413mS = ");
    	vTaskDelay(100);
//    	if (button==button!=BUTTON_CNL) {	// wtf?

//    	}
	}
	EE_WriteVariable(EC1413_ADDR, adcAverage[AVG_ADC_EC]);
	button = 0;

	Lcd_goto(0,0);
    	Lcd_write_str("1.413 mS saved");
	vTaskDelay(2000);

	loadSettings();
}


void adcAverager(void){
	int i=0, i2=0;
	uint32_t jdrBuff1Total=0, jdrBuff2Total=0, jdrBuff3Total=0, jdrBuff4Total=0;

	for(i2=0; i2<9; i2++){
	    for(i=0; i < 9; i++) {
	    	jdrBuff1[i] = jdrBuff1[i+1];
	    	jdrBuff2[i] = jdrBuff2[i+1];
	    	jdrBuff3[i] = jdrBuff3[i+1];
	    	jdrBuff4[i] = jdrBuff4[i+1];
 		}
 		jdrBuff1[JDR_BUFFER_SIZE-1] = ADC1->JDR1;
 		jdrBuff2[JDR_BUFFER_SIZE-1] = ADC1->JDR2;
 		jdrBuff3[JDR_BUFFER_SIZE-1] = ADC1->JDR3;
 		jdrBuff4[JDR_BUFFER_SIZE-1] = ADC1->JDR4;
    	jdrBuff1Total += jdrBuff1[JDR_BUFFER_SIZE-1];
    	jdrBuff2Total += jdrBuff2[JDR_BUFFER_SIZE-1];
    	jdrBuff3Total += jdrBuff3[JDR_BUFFER_SIZE-1];
    	jdrBuff4Total += jdrBuff4[JDR_BUFFER_SIZE-1];
	}
	    adcAverage[0] = jdrBuff1Total/10;
	    adcAverage[1] = jdrBuff2Total/10;
	    adcAverage[2] = jdrBuff3Total/10;
	    adcAverage[3] = jdrBuff4Total/10;
}

void getPh() {	// current PH, please!
	uint32_t ph;
	vTaskDelay(5);

    // count pH underOver flag
	if (adcAverage[AVG_ADC_PH]>phWindowTop) {
		phUnderOver = 2;
	}
	else if (adcAverage[AVG_ADC_PH]<phWindowBottom) {
		phUnderOver = 1;
	}
	else {
		phUnderOver = 0;
	}

	//count ec underOver flag
	if (adcAverage[AVG_ADC_EC]>ecWindowTop) {
		ecUnderOver = 2;
	}
	else if (adcAverage[AVG_ADC_EC]<ecWindowBottom) {
		ecUnderOver = 1;
	}
	else {
		ecUnderOver = 0;
	}

	vTaskDelay(5);
	ph = ((adcAverage[AVG_ADC_EC] - ph0)*10)/cdel;
	currentPh = ph;
	curphstr[3] = ph-(ph/10)*10+48;
	vTaskDelay(5);
	ph/=10;
	curphstr[2]=46;
	curphstr[1] = ph-(ph/10)*10+48;
	vTaskDelay(5);
	ph/=10;
	curphstr[0]=ph-(ph/10)*10+48;
	curphstr[4] = 0;	// konec stroki
}

void getEc(void){
	uint32_t total=0, ec, ec_div;
	vTaskDelay(5);
	uint8_t i;
	if (adcAverage[AVG_ADC_EC]>ecWindowTop) {
		ecUnderOver = 2;
	}
	else if (adcAverage[AVG_ADC_EC]<ecWindowBottom) {
		ecUnderOver = 1;
	}
	else {
		ecUnderOver = 0;
	}
	vTaskDelay(5);
//	ec1413 = 200;		// this is 1.413ms
	ec_div = ((ec1413-ec0)*100)/141;		// 0.01mS = ec1413/141. We store it 100 times more for precision

	if (adcAverage[AVG_ADC_EC]>ec0) {
		ec = ((adcAverage[AVG_ADC_EC]-ec0)*100)/ec_div;	//
	}
	else {
		ec = 0;
	}
	currentEc = ec;
	curecstr[4] = '\0';
	curecstr[3] = '0' + ( ec       )    % 10;
	curecstr[2] = '0' + ( ec /= 10 )    % 10;
	curecstr[1] = '.';
	curecstr[0] = '0' + ( ec /= 10 )    % 10;

/*	curecstr[3] = ec-(ec/10)*10+48;
	vTaskDelay(5);
	ec/=10;
	curecstr[1]=46;
	curecstr[2] = ec-(ec/10)*10+48;
	vTaskDelay(5);
	ec/=10;
	curecstr[0]=ec-(ec/10)*10+48;
	curecstr[4] = 0;	// konec stroki */


/*	curecstr[3] = ec-(ec/10)*10+48;
	vTaskDelay(5);
	ec/=10;
	curecstr[2]=46;
	curphstr[1] = ec-(ec/10)*10+48;
	vTaskDelay(5);
	ec/=10;
	curecstr[0]=ec-(ec/10)*10+48;
	curecstr[4] = 0;	// konec stroki */
}


char * utoa_fast_div(uint32_t value, char *buffer)
{
    buffer += 11;
    *--buffer = 0;
    do
    {
        divmod10_t res = divmodu10(value);
        *--buffer = res.rem + '0';
        value = res.quot;
    }
    while (value != 0);


    return buffer;
}

void set16bit(uint16_t value){
	uint8_t button;
//	uint16_t Address, timerId;
//	plugId--;	// pervomu plugu sootvetstvuet nulevaja zapis' v kode (especially for menu calls)
//	Lcd_clear();
//	Lcd_goto(0,0);
//	Lcd_write_str("Plug ");
//	Lcd_write_digit(plugId);
//	Lcd_write_str(" timer");
//	Address = EE_PLUG_SETTINGS+plugId;
//	EE_ReadVariable(Address, &timerId);
//	vTaskDelay(50);
	while (button!=BUTTON_OK) {
		button=readButtons();
		vTaskDelay(25);
		if (button==BUTTON_BCK) {
			if (value<1) {
				value=65535;
			}
			else {
				value--;
			}
		}
		if (button==BUTTON_FWD) {
			if (value>65534) {
				value=0;
			}
			else {
				value++;
			}
		}
		Lcd_goto(1,0);
		Lcd_write_str("<");
		Lcd_write_digit(value);
		Lcd_write_str(">");
		vTaskDelay(25);
	}
	vTaskDelay(50);
//	Address = EE_PLUG_SETTINGS+plugId;
//	EE_WriteVariable(Address, timerId);
//	loadSettings();
//	Lcd_clear();
	printOk();
}


void setPlug(uint8_t plugId){
	uint8_t button=0;
	uint16_t Address, timerId;
//	plugId--;	// pervomu plugu sootvetstvuet nulevaja zapis' v kode (especially for menu calls)
	Lcd_clear();
	Lcd_goto(0,0);
	Lcd_write_str("Plug ");
	Lcd_write_digit(++plugId);
	Lcd_write_str(" timer");
	plugId--;
	Address = EE_PLUG_SETTINGS+plugId;
	EE_ReadVariable(Address, &timerId);
	vTaskDelay(50);
	timerId = adjust8bit(timerId);
	vTaskDelay(50);
	Address = EE_PLUG_SETTINGS+plugId;
	EE_WriteVariable(Address, timerId);
	vTaskDelay(5);
	loadSettings();
	vTaskDelay(5);
}

void printOk(void){
	Lcd_clear();
	Lcd_write_str("OK");
	vTaskDelay(500);
	Lcd_clear();
}

void timerStateTrigger(void *pvParameters){
	uint8_t i, timerStateFlag;
	uint32_t now, timer1, timer2;
	uint16_t Address;
	while (1) {
//		valve_status_updater();
		now=RTC_GetCounter();
		for (i=0; i<3; i++){	// 32 tajmera
			Address = EE_TIMER1_ON+EE_TIMER_SIZE*i;
			timer1 = EE_ReadWord(Address);
			timer2 = EE_ReadWord(Address+2);

			// for everyday triggering
			timer1 = timer1%86400;
			timer2 = timer2%86400;
			now = now % 86400;

			vTaskDelay(1);
	//		comm_manager();
			vTaskDelay(1);

			// logika obychnogo tajmera dlja timerOn<timerOff
			if (now<timer2 && now>timer1) {
				timerStateFlag=1;
			}
			else {
				timerStateFlag=0;
			}

			// obratnaja logika tajmera
			if (timer1>timer2) {
				if (now<timer1 && now>timer2) {
					timerStateFlag=0;
				}
				else {
					timerStateFlag=1;
				}
			}

			if (timerStateFlag==1) {
				timerStateFlags|=(1<<i);
			}
			else {
				timerStateFlags &= ~(1<<i); // sbrosit' flag
			}
		}
		vTaskDelay(1);
		for (i=0; i<3; i++){	// do 3 tajmerov
			Address = EE_CTIMER_DURATION+EE_CTIMER_SIZE*i;
			timer1 = EE_ReadWord(Address);
			timer2 = EE_ReadWord(Address+2);
			vTaskDelay(1);
			now=RTC_GetCounter();

			// logika ciklicheskogo tajmera
			now %= timer2;
			if (now<timer1) {
				timerStateFlag=1;
				cTimerStateFlags|=(1<<i);
			}
			else {
				timerStateFlag=0;
				cTimerStateFlags &= ~(1<<i); // sbrosit' flag
			}
			vTaskDelay(1);
		}
	}
}

void plugStateTrigger(void *pvParameters){
	uint8_t plugStateFlag, plugTimerId, plugType;
	uint8_t i;
	while (1) {
		if (comm_state!=COMM_DIRECT_DRIVE) {
			for (i=0; i<PLUG_AMOUNT; i++){		// PC0 to PC2
				plugType=0;
				plugTimerId = plugSettings[i];	// get the ID of timer for this plug
				if (plugTimerId>=0 && plugTimerId<=31) {
	//				plugType=0;
					// Timer
					plugStateFlag=timerStateFlags&(1<<plugTimerId);	// check if timer active now
					plugStateFlag>>=plugTimerId;	// ostavit' toka flag
				}
				if (plugTimerId>63 && plugTimerId<67){	// ph up
					plugType=2;
					plugTimerId-=32;
				}
				if (plugTimerId>66 && plugTimerId<69){	// ph up
					plugType=3;
					plugTimerId-=35;
				}

				if (plugTimerId>69 && plugTimerId<72){	// ph up
					plugType=4;
	//				plugTimerId-=35;
				}
				if (plugTimerId>79 && plugTimerId<81){	// 80 - PSI stab
					plugType=5;
	//				plugTimerId-=35;
				}

				if (plugTimerId>31 && plugTimerId<64) {
				//	plugType=1;
					// CTimer
					plugTimerId-=32;
					plugStateFlag=cTimerStateFlags&(1<<plugTimerId);	// check if timer active now
					plugStateFlag>>=plugTimerId;	// ostavit' toka flag
				}

				if (plugType==2) {
					if (plugStateFlag==1 && ((plugStateFlags>>i)&1)==0 && plugTimerId==0 && phUnderOver==1) {
						plugStateSet(i, 1);	// enable plug for ph up pump
					}
					if (plugStateFlag==1 && ((plugStateFlags>>i)&1)==0 && plugTimerId==1 && phUnderOver==2) {
						plugStateSet(i, 1);	// enable plug for ph down pump
					}
					if (plugStateFlag==1 && ((plugStateFlags>>i)&1)==0 && plugTimerId==2 && phUnderOver>0) {
						plugStateSet(i, 1);	// enable plug for mixing pump
					}
					if (plugStateFlag==0 && ((plugStateFlags>>i)&1)==1) {
						plugStateSet(i, 0);	// disable plug
					}
				}
				else if (plugType==3) {	// mister (or another humidity "upper")
					if (plugStateFlag==1 && ((plugStateFlags>>i)&1)==0 && plugTimerId==0 && rhUnderOver==1) {
						plugStateSet(i, 1);	// enable mister for underwindow
					}
					if (plugStateFlag==1 && ((plugStateFlags>>i)&1)==0 && plugTimerId==0 && rhUnderOver==0) {
						plugStateSet(i, 1);	// enable mister for in-window
					}
					if (plugStateFlag==0 && ((plugStateFlags>>i)&1)==1) {
						plugStateSet(i, 0);	// disable plug
					}
				}
				else if (plugType==4) {
					// watering and circulation pumps for watering controller
				}
				else if (plugType==5) {
					if (auto_flags&1==1) {			// PSI STAB flag 0
						if (adcAverage[AVG_ADC_PSI]>psi_pump_top_level) {
							plugStateSet(psi_pump_load_id, 0);
						}
						vTaskDelay(1);
						if (adcAverage[AVG_ADC_PSI]<psi_pump_btm_level) {
							plugStateSet(psi_pump_load_id, 1);
						}
					}
									// psi pump pressure stabilizer
				}
				else {
					if (plugStateFlag==1 && ((plugStateFlags>>i)&1)==0) {
						plugStateSet(i, 1);	// enable plug
					}
					if (plugStateFlag==0 && ((plugStateFlags>>i)&1)==1) {
						plugStateSet(i, 0);	// disable plug
					}
				}
				vTaskDelay(1);
			}
		}
		vTaskDelay(1);
#ifdef USE_VALVES
//		uint8_t valveId=0, valveMotorStateFlag=0;
//		for (valveId=0; valveId<3; valveId++) {
//			valveMotorStateFlag=valveMotorStateFlags&(1<<valveId);	// check if timer active now
//			valveMotorStateFlag>>=valveId;	// ostavit' toka flag
//			if (valveMotorStateFlag==1) {
//				valveMotorStateSet(valveId,1);
//			}
//			else {
//				valveMotorStateSet(valveId,2);
//			}
//		}
#endif
//		vTaskDelay(1);
//		valveManager();
		vTaskDelay(1);
	}
}


void psiSetup(void){
	uint8_t curbutton=0, tmp;
	uint16_t curpsiadc=0, tmp2=0, tempvalue=0;
	comm_state=COMM_DIRECT_DRIVE;
	Lcd_clear();
	Lcd_write_str("PSI pump load id");
	EE_ReadVariable(PSI_PUMP_LOAD_ID, &tempvalue);
//	tempvalue = adjust8bit(tempvalue);
	psi_pump_load_id = idSelector(1,4,tempvalue);
	EE_WriteVariable(PSI_PUMP_LOAD_ID, psi_pump_load_id);
	tmp2 = EE_PLUG_SETTINGS+psi_pump_load_id;
	EE_WriteVariable(tmp2, 80);		// HARDCODE!!! program (timer) id for booster pump
	plugSettings[tempvalue] = 80;

//	psi_pump_load_id = idSelector(0,4,psi_pump_load_id);
//	tmp2 = (uint16_t)(psi_pump_load_id*0x00FF);
//	EE_WriteVariable(PSI_PUMP_LOAD_ID, tmp2);

	Lcd_write_str("ADC psi reading");
	while (curbutton!=BUTTON_FWD){
		Lcd_goto(1,0);
		Lcd_write_str("TOP:");
		if (curbutton==BUTTON_BCK){
			plugStateSet(psi_pump_load_id, 1);
		}
		if (curbutton==BUTTON_OK){
			plugStateSet(psi_pump_load_id, 0);
		}
		if (curbutton==BUTTON_CNL){
			psi_pump_top_level = adcAverage[AVG_ADC_PSI];
		}
		vTaskDelay(25);
		Lcd_write_16b(adcAverage[AVG_ADC_PSI]);
		Lcd_write_str("/");
		Lcd_write_16b(psi_pump_top_level);
		curbutton=readButtons();
	}
	plugStateSet(psi_pump_load_id, 0);
	printOk();
	curbutton=0;
	while (curbutton!=BUTTON_FWD){
		Lcd_goto(1,0);
		Lcd_write_str("BTM:");
		if (curbutton==BUTTON_BCK){
			plugStateSet(psi_pump_load_id, 1);
		}
		if (curbutton==BUTTON_OK){
			plugStateSet(psi_pump_load_id, 0);
		}
		if (curbutton==BUTTON_CNL){
			psi_pump_btm_level = adcAverage[AVG_ADC_PSI];
		}
		vTaskDelay(10);
		Lcd_write_16b(adcAverage[AVG_ADC_PSI]);
		Lcd_write_str("/");
		Lcd_write_16b(psi_pump_btm_level);
		curbutton=readButtons();
	}
	plugStateSet(psi_pump_load_id, 0);
	comm_state=COMM_MONITOR_MODE;
	EE_WriteVariable(PSI_SENSOR_TOP, psi_pump_top_level);
	EE_WriteVariable(PSI_SENSOR_BTM, psi_pump_btm_level);
	printOk();
}

void plugTest(void){
	uint8_t curplug=0, curbutton=0;
	comm_state=COMM_DIRECT_DRIVE;
	while (curbutton!=BUTTON_FWD){
		Lcd_goto(0,0);
		Lcd_write_str("Plug:");
		Lcd_write_digit(curplug);
		if (curbutton==BUTTON_BCK){
			curplug++;
			if (curplug==PLUG_AMOUNT) {
				curplug=0;
			}
		}
		if (curbutton==BUTTON_OK){
			plugStateSet(curplug, 1);
		}
		if (curbutton==BUTTON_CNL){
			plugStateSet(curplug, 0);
		}
		vTaskDelay(10);
		curbutton=readButtons();
	}
	comm_state=COMM_MONITOR_MODE;
}

void plugStateSet(uint8_t plug, uint8_t state){
//	if (PLUG_INVERT==1) {
//		state ^= (1<<0);
//	}
	if (state==1) {
		PLUG_DISABLE = (1<<plug);
		plugStateFlags |= (1<<plug);
	}
	else {
		PLUG_ENABLE = (1<<plug);
		plugStateFlags &= ~(1<<plug);
	}
}

void valveMotorStateSet(uint8_t valveId, uint8_t state){
#ifdef USE_VALVES
/*	if (state==1) {
//		VALVE_MOTOR_PORT->BRR |= (1<<valveId+VALVE_MOTOR_GPIO_SHIFT); // valve enable 146%
			valveMotorStateFlags |= (1<<valveId);
			VALVE_ENABLE = (1<<valveId+VALVE_MOTOR_GPIO_SHIFT);
		}
		else {
			valveMotorStateFlags &= ~(1<<valveId);
			VALVE_DISABLE = (1<<valveId+VALVE_MOTOR_GPIO_SHIFT);
		} */
#endif
}



void EE_WriteWord(uint16_t Address, uint32_t Data){
	uint16_t tmp, tmp2;
	tmp2 = Data & 0xFFFF;
	tmp = Data >> 16;
	EE_WriteVariable(Address+1, tmp2);
	EE_WriteVariable(Address, tmp);
}

void programRunner(uint8_t programId){

	uint32_t tmp;
	uint8_t tmp8;
	Lcd_clear();
	switch (programId) {
	case 1:
		break;
	case 2:
		Lcd_write_str("Timer to adjust:");
		tmp8 = idSelector(1,4,1);
	    setTimer(--tmp8);
		break;
	case 3:
		plugTest();
		break;
	case 4:
		psiSetup();
		break;
	case 5:
		tmp = RTC_GetCounter();
		uint32_t unixtime = timeAdjust(tmp, 1);
		RTC_SetCounter(unixtime);
		Lcd_clear();
		break;
	case 6:
		Lcd_write_str("CTimer 2 adjust:");
		tmp8 = idSelector(1,4,1);
	    setCTimer(--tmp8);
		break;
	case 7:
//		setCTimer(1);
		break;
	case 8:
//		setCTimer(2);
		break;
	case 9:
		Lcd_write_str("Choose plug");
		tmp8 = idSelector(1,4,1);
		setPlug(--tmp8);	// decrement needed because of actual start from 0
		break;
	case 10:
//		setPlug(1);
		break;
	case 11:
//		setPlug(2);
		break;
	case 13:
		phMonSettings();
		break;
	case 14:
//		lightRangeSet();
		break;
	case 15:
		calibratePh();
		break;
	case 16:
		phStabSettings();
		break;
	case 17:
		dht_arr_displayer();
#ifdef	TEST_MODE
		displayAdcValues();	// test function to display ADC values
#endif
		display_usart_rx();
		display_usart_tx();
//		setDutyCycle();
		break;
	case 18:
		hygroStatSettings();
		break;
	case 19:
//		thermoStatSetup();
		break;
	case 20:
		valve_test();
		valve_test();
		break;
	case 21:
		watering_setup();
		break;
	case 22:
		fertilization_setup();
		break;
	case 23:
		get_water(0,0,100);
		//calibrateEc();
		break;
	case 24:
		// ecStabSettings();
		break;
	case 25:
		// buttonTest();
		break;
	case 26:
		// phAndEcTest();
		break;
	case 27:
		startWp();
		break;
	case 28:
		tankLevelStabSetup();
		break;
	}
}

/* void phStabSettings(void){
	Lcd_clear();
	Lcd_write_str("Set top lvl");
	readPhVal(value);
	EE_WriteVariable(SD_LOG_INTERVAL, logSetting);
	Lcd_write_str("Set bottom lvl");
	readPhVal(value);
	EE_WriteVariable(SD_LOG_INTERVAL, logSetting);

} */

void loggerSettings(void){
	uint32_t logSetting;
//	char value[8];
	vTaskDelay(10);
	Lcd_clear();
	EE_ReadVariable(SD_LOG_INTERVAL, &logSetting);
	vTaskDelay(100);
	Lcd_goto(0,0);
	Lcd_write_str("Log frequency");
	vTaskDelay(3000);
	logSetting=adjust8bit(logSetting);	// poluchit' novoe znachenie ot user'a
	EE_WriteVariable(SD_LOG_INTERVAL, logSetting);
	vTaskDelay(50);
	Lcd_clear();
	Lcd_goto(0,2);
	Lcd_write_str("Complete!");
	vTaskDelay(500);
	loadSettings();
	Lcd_clear();
}

void phMonSettings(void) {
//	uint16_t Address;
	uint32_t phSetting;	// possibly 16 bit?
//	char value[8];
	vTaskDelay(10);
	Lcd_clear();
	EE_ReadVariable(PH_INTERVAL, &phSetting);
	vTaskDelay(100);
	Lcd_goto(0,0);
	Lcd_write_str("Probing interval");
	vTaskDelay(1000);
	phSetting=adjust8bit(phSetting);	// poluchit' novoe znachenie ot user'a
	EE_WriteVariable(PH_INTERVAL, phSetting);
	vTaskDelay(50);


	// analogichno dlja buffer size
	EE_ReadVariable(PH_BUFF_SIZE, &phSetting);
	Lcd_clear();
	Lcd_goto(0,2);
	Lcd_write_str("Buffer size");
	vTaskDelay(500);
	phSetting=adjust8bit(phSetting);	// poluchit' novoe znachenie ot user'a
	EE_WriteVariable(PH_BUFF_SIZE, phSetting);
	Lcd_clear();
	Lcd_goto(0,2);
	Lcd_write_str("Complete!");
	vTaskDelay(500);
	loadSettings();
	Lcd_clear();
}

uint8_t adjust8bit(uint8_t val){
	uint8_t button;
	if (val>99) {val=99;}
	while (button!=BUTTON_OK) {
		button=readButtons();
		vTaskDelay(25);
		if (button==BUTTON_BCK) {
			if (val<2) {
				val=255;
			}
			else {
				val--;
			}
		}
		if (button==BUTTON_FWD) {
			if (val>254) {
				val=1;
			}
			else {
				val++;
			}
		}
		Lcd_goto(1,0);
		Lcd_write_str("< ");
		Lcd_write_8b(val);
		Lcd_write_str(" >");
		vTaskDelay(25);
	}
	printOk();
	return val;
}

uint16_t adjust16bit(uint16_t val){
	uint8_t button;
	if (val>65534) {
		val=0;
	}
	while (button!=BUTTON_OK) {
		button=readButtons();
		vTaskDelay(25);
		if (button==BUTTON_BCK) {
			if (val<1) {
				val=65535;
			}
			else {
				val--;
			}
		}
		if (button==BUTTON_FWD) {
			if (val>65534) {

				val=0;
			}
			else {
				val++;
			}
		}
		Lcd_goto(1,4);
		Lcd_write_str("< ");
		Lcd_write_16b(val);
		Lcd_write_str(" >");
		vTaskDelay(25);
	}
	printOk();
	return val;
}

uint16_t adjust16bit_fast(uint16_t val, uint8_t speed){
	uint8_t button=0;
	char buffer[11];
	if (val>65534) {
		val=0;
	}
	vTaskDelay(200);
	while (button!=BUTTON_OK) {
		button=readButtons();
		vTaskDelay(speed);
		if (button==BUTTON_BCK) {
			if (val<1) {
				val=65535;
			}
			else {
				val--;
			}
		}
		if (button==BUTTON_FWD) {
			if (val>65534) {

				val=0;
			}
			else {
				val++;
			}
		}
		Lcd_goto(1,4);
		Lcd_write_str("< ");
//		utoa_fast_div(val, &buffer);
//		copy_arr(&buffer, &LCDLine2, 5, 2);
		Lcd_write_16b(val);
		Lcd_write_str(" >");
		vTaskDelay(speed);
	}
	printOk();
	return val;
}

uint32_t CTimerAdjust(uint32_t time){
	uint8_t hours, minutes, seconds, hours2, minutes2, seconds2, button;
	char timestr[6];
	seconds = time % 60;
	time /= 60;
	minutes = time % 60;
	time /= 60;
	hours = time % 99;
	Lcd_clear();
	vTaskDelay(20);
	while (button!=BUTTON_FWD)
	{
		button=readButtons();
		if (button==BUTTON_BCK)
			{
				if (hours>98) {hours=0;}	// chtoby pechatat' pobolee znakov, nado zamenit' Lcd_write_digit()
				else {hours++;}
			}
		if (button==BUTTON_OK)
			{
				if (minutes>58) {minutes=0;}
				else {minutes++;}
			}
		if (button==BUTTON_CNL)
			{
				if (seconds>58) {seconds=0;}
				else {seconds++;}
			}

    	vTaskDelay(5);
		Lcd_goto(0,4);
    	Lcd_write_digit(hours);
    	Lcd_write_str(":");
    	Lcd_write_digit(minutes);
    	Lcd_write_str(":");
    	Lcd_write_digit(seconds);
    	vTaskDelay(5);
    	hours2=hours;		// daby izbezhat' konversii tipov pri umnozhenii na 3600 i podobnom
    	minutes2=minutes;
    	seconds2=seconds;
    	time=hours2*3600;
    	time+=minutes2*60;
    	time+=seconds2;
    	vTaskDelay(5);
    	Lcd_goto(1,3);
    	int32str(time, &timestr);
//    	Lcd_write_arr(&timestr, 6);
    	copy_arr(&timestr, LCDLine2, 10, 2);
    	vTaskDelay(10);
	}
	printOk();
	return(time);
}

void setCTimer(uint8_t timerId){
	uint16_t Address;
	uint32_t CTimerData;
//	char value[8];
	vTaskDelay(10);
	Lcd_clear();
	Address = EE_CTIMER_DURATION+timerId*EE_CTIMER_SIZE;
	CTimerData = EE_ReadWord(Address);
	if (CTimerData>356400) {
		CTimerData=356400;
	}
	Lcd_goto(0,2);
	Lcd_write_str("Set DURATION");
	vTaskDelay(3000);
	CTimerData=CTimerAdjust(CTimerData);	// poluchit' novoe znachenie ot user'a
	Address = EE_CTIMER_DURATION+timerId*EE_CTIMER_SIZE;	// adres postojanno "napominaetsja", potomu chto po-hodu ispolnenija koda on kuda-to "terjaetsja". bylo by neploho razobratsja kuda
	EE_WriteWord(Address, CTimerData);
	vTaskDelay(50);


	// analogichno dlja INTERVALa
	Address = EE_CTIMER_INTERVAL+timerId*EE_CTIMER_SIZE;
	CTimerData = EE_ReadWord(Address);
	Lcd_clear();
	Lcd_goto(0,2);
	Lcd_write_str("Set INTERVAL");
	vTaskDelay(500);
	CTimerData=CTimerAdjust(CTimerData);	// poluchit' novoe znachenie ot user'a
	Address = EE_CTIMER_INTERVAL+timerId*EE_CTIMER_SIZE;
	EE_WriteWord(Address, CTimerData);
	printOk();
}

uint8_t idSelector(uint8_t min, uint8_t max, uint8_t curid){
	uint8_t curbutton=0;
	while (curbutton!=BUTTON_OK){
		vTaskDelay(25);
		Lcd_goto(1,0);
		Lcd_write_digit(curid);
		if (curbutton==BUTTON_FWD) {
			if (curid<max) {
				curid++;
			}
		}
		if (curbutton==BUTTON_BCK) {
			if (curid>min) {
				curid--;
			}
		}
		curbutton=readButtons();
		vTaskDelay(25);
	}
	printOk();
	return curid;
}


void setTimer(uint8_t timerId){
	uint32_t Data, adjusteDate;
	uint16_t Address;
	Address = EE_TIMER1_ON+timerId*EE_TIMER_SIZE;	// set ON for plain timer
	Lcd_clear();
	Lcd_goto(0,2);
	Lcd_write_str("Set ON time");
	vTaskDelay(3000);
	Lcd_clear();
	vTaskDelay(50);
	Data = EE_ReadWord(Address);
	vTaskDelay(50);
	adjusteDate = timeAdjust(Data, 1);
	EE_WriteWord(Address, adjusteDate);
	Lcd_goto(0,2);
	Lcd_write_str("Set OFF time");
	vTaskDelay(50);
	vTaskSuspendAll();
	Address = EE_TIMER1_ON+timerId*EE_TIMER_SIZE+2;	// set OFF for plain timer
	Data = EE_ReadWord(Address);
	xTaskResumeAll();
	vTaskDelay(3000);
	Lcd_clear();
	adjusteDate = timeAdjust(Data, 1);
	EE_WriteWord(Address, adjusteDate);
	vTaskSuspendAll();
	Address = EE_TIMER1_ON+timerId*EE_TIMER_SIZE+4;	// set daily flag
	Data = EE_ReadWord(Address);
	xTaskResumeAll();
	vTaskDelay(50);
}

int yesNoSelector(char str, int curval){
	Lcd_clear();
	Lcd_goto(0,0);
	Lcd_write_str(str);
	vTaskDelay(50);
	uint8_t button=0;
	vTaskDelay(50);
	while (button!=BUTTON_OK) {
		if (button==BUTTON_FWD || button==BUTTON_BCK) {
/*			Lcd_goto(1,5);
			curval |= 1;
			vTaskDelay(50);
			if (curval==0){
				Lcd_write_str("< NO >");
			}
			vTaskDelay(50);
			if (curval==1){
				Lcd_write_str("< SI >");
			}
			vTaskDelay(50);
			button=readButtons();
			vTaskDelay(50); */
		}
		vTaskDelay(50);

	}
//	vTaskDelay(50);
	return(curval);
}

uint32_t timeAdjust(uint32_t cnt, uint8_t includeDays)
{
	Lcd_clear();
	uint32_t unixtime2;
	uint8_t button=0;
	RTC_DateTime curtime, curtime2;
	char unixtimestr[11];
	curtime=unix2DateTime(cnt);

	while (button!=BUTTON_FWD)
	{
		button=readButtons();
		if (button==BUTTON_BCK)
			{
				if (curtime.hour>22) {curtime.hour=0;}
				else {curtime.hour++;}
			}
		if (button==BUTTON_OK)
			{
				if (curtime.min>58) {curtime.min=0;}
				else {curtime.min++;}
			}
		if (button==BUTTON_CNL)
			{
				if (curtime.sec>58) {curtime.sec=0;}
				else {curtime.sec++;}
			}
    	vTaskDelay(5);
		Lcd_goto(0,0);
    	Lcd_write_digit(curtime.hour);
    	Lcd_write_str(":");
    	Lcd_write_digit(curtime.min);
    	Lcd_write_str(":");
    	Lcd_write_digit(curtime.sec);
#ifdef	TEST_MODE
		unixtime2=DateTime2unix(curtime);
		curtime2=unix2DateTime(unixtime2);
    	int32str(unixtime2, &unixtimestr);
		Lcd_goto(0,10);
    	Lcd_write_digit(curtime2.hour);
    	Lcd_write_digit(curtime2.min);
    	Lcd_write_digit(curtime2.sec);
    	Lcd_goto(1,2);
    	vTaskDelay(5);
    	copy_arr(&unixtimestr, LCDLine2, 10, 2);
#endif
    	vTaskDelay(20);
	}
	button=0;
	printOk();
	if (includeDays==1) {
			while (button!=BUTTON_FWD)
				{
					button=readButtons();
					if (button==BUTTON_CNL)
						{
							if (curtime.year>50) {curtime.year=12;}
							else {curtime.year++;}
						}
					if (button==BUTTON_OK)
						{
							if (curtime.month>11) {curtime.month=1;}
							else {curtime.month++;}
						}
					if (button==BUTTON_BCK)
						{
							if (curtime.day>30) {curtime.day=1;}
							else {curtime.day++;}
						}

			    	vTaskDelay(5);
#ifdef	TEST_MODE
			    	// preobrazovanija tuda-sjuda dlja togo chtoby ubeditsja, chto funkcii rabotajut verno
					unixtime2=DateTime2unix(curtime);
					curtime2=unix2DateTime(unixtime2);
			    	int32str(unixtime2, &unixtimestr);
#endif
			    	//Lcd_clear();
			    	vTaskDelay(5);
					Lcd_goto(0,0);
			    	Lcd_write_digit(curtime.day);
			    	Lcd_write_str("-");
			    	Lcd_write_digit(curtime.month);
			    	Lcd_write_str("-");
			    	Lcd_write_digit(curtime.year);
			    	vTaskDelay(5);
#ifdef	TEST_MODE
					Lcd_goto(0,10);
			    	Lcd_write_digit(curtime2.day);
			    	Lcd_write_digit(curtime2.month);
			    	Lcd_write_digit(curtime2.year);
			    	vTaskDelay(5);
			    	copy_arr(&unixtimestr, LCDLine2, 10, 2);
#endif
					vTaskDelay(50);
			}
		}
	else {
		unixtime2 = 0;
		unixtime2 += curtime.hour*3600;
		unixtime2 += curtime.min*60;
		unixtime2 += curtime.sec;
	}
	printOk();
	return(unixtime2);
}

void Lcd_write_arr2(uc8 *STRING, uint8_t chars)
{
	char c;
	uint8_t i;
	for (i=0; i<chars; i++) {
		c=STRING[i];
//		vTaskDelay(5);
		Lcd_write_data(c);
	}
}

void Lcd_write_arr(uc8 *STRING, uint8_t chars)
{
	char c;
	uint8_t i;
	for (i=0; i<chars; i++) {
		c=STRING[i];
		Lcd_write_data(c);
	}
}




RTC_DateTime unix2DateTime(uint32_t unixtime)
{
	RTC_DateTime datetime;
	uint32_t tmp;
	uint8_t leaps, month, day;
	uint16_t daysFromLeap, dayOfYearLeaped;
	tmp = unixtime-YEAR12SECS;	// vse raschety vedutsja ot 01.01.2012 00:00:00

	datetime.sec = unixtime % 60;
	unixtime /= 60;
	datetime.min = unixtime % 60;
	unixtime /= 60;
	datetime.hour = unixtime % 24;
	tmp /= 86400;
	daysFromLeap = tmp % 1461;	// 86400*1461=126230400 secs in leap cycle
	leaps=tmp/1461;
	daysFromLeap++;

	if (daysFromLeap<59)	// 1..60
	{
		datetime.year=leaps*4+12;
		dayOfYearLeaped=daysFromLeap+1;	// for shifting days because of 29.03

	}
	if (daysFromLeap>=59 && daysFromLeap<=366)		// 61..366
	{
		datetime.year=leaps*4+(daysFromLeap-1)/365+12;
		dayOfYearLeaped=daysFromLeap;
	}
	if (daysFromLeap>=366) {								// 367..1461
		datetime.year=leaps*4+(daysFromLeap-1)/365+12;
		dayOfYearLeaped=((daysFromLeap-366) % 365)+1;
	}

	if (dayOfYearLeaped>=1 && dayOfYearLeaped<=31)
	{
		month=1;	// january
		day=dayOfYearLeaped;
	}
	if (dayOfYearLeaped>=32 && dayOfYearLeaped<=59)
	{
		month=2;	// february
		day=dayOfYearLeaped-31;
	}
	if (dayOfYearLeaped>=60 && dayOfYearLeaped<=90)
	{
		month=3;	// march
		day=dayOfYearLeaped-59;
	}
	if (dayOfYearLeaped>=91 && dayOfYearLeaped<=120)
	{
		month=4;	// april
		day=dayOfYearLeaped-90;
	}
	if (dayOfYearLeaped>=121 && dayOfYearLeaped<=151)
	{
		month=5;	// may
		day=dayOfYearLeaped-120;
	}
	if (dayOfYearLeaped>=152 && dayOfYearLeaped<=181)
	{
		month=6;	// june
		day=dayOfYearLeaped-151;
	}
	if (dayOfYearLeaped>=182 && dayOfYearLeaped<=212)
	{
		month=7;	// july
		day=dayOfYearLeaped-181;
	}
	if (dayOfYearLeaped>=213 && dayOfYearLeaped<=243)
	{
		month=8;	// august
		day=dayOfYearLeaped-212;
	}
	if (dayOfYearLeaped>=244 && dayOfYearLeaped<=273)
	{
		month=9;	// september
		day=dayOfYearLeaped-243;
	}
	if (dayOfYearLeaped>=274 && dayOfYearLeaped<=304)
	{
		month=10;	// october
		day=dayOfYearLeaped-273;
	}
	if (dayOfYearLeaped>=305 && dayOfYearLeaped<=334)
	{
		month=11;	// november
		day=dayOfYearLeaped-304;
	}
	if (dayOfYearLeaped>=335 && dayOfYearLeaped<=365)
	{
		month=12;	// december
		day=dayOfYearLeaped-334;
	}
	datetime.day=day;
	datetime.month=month;
	if (daysFromLeap==59) {
		datetime.day=29;
		datetime.month=2;
	}
	return(datetime);
}

uint32_t DateTime2unix(RTC_DateTime datetime)
{
	    uint32_t tmp;
///	    uint8_t i;
	    uint16_t days, dayFromYear;

	    switch (datetime.month) {
	    case 0:
	    	days=0;
	    case 1:
	    	days=0;
	    	break;
	    case 2:
	    	days=31;
	    	break;
	    case 3:
	    	days=59;
	    	break;
	    case 4:
	    	days=90;
	    	break;
	    case 5:
	    	days=120;
	    	break;
	    case 6:
	    	days=151;
	    	break;
	    case 7:
	    	days=181;
	    	break;
	    case 8:
	    	days=212;
	    	break;
	    case 9:
	    	days=243;
	    	break;
	    case 10:
	    	days=273;
	    	break;
	    case 11:
	    	days=304;
	    	break;
	    case 12:
	    	days=334;
	    	break;
	    }

	    dayFromYear = days+datetime.day;

	    if (dayFromYear<60 && (datetime.year % 4)==0){
	    	dayFromYear--;
	    }
	    if (datetime.month==2 && datetime.day>=29 && (datetime.year % 4)==0) {
	        dayFromYear--;
	    }
	    if (dayFromYear<1){dayFromYear=1;}
	    tmp = ((datetime.year-12) * 365 * 86400);
	    tmp += (datetime.hour * 3600);
	    tmp += (datetime.min * 60);
	    tmp += (datetime.sec + YEAR12SECS);
	    tmp += ((dayFromYear-2)*86400);
	    tmp+=(((datetime.year-8)/4)*86400);
	    return tmp;
}

// function returns the programId selected in menu to run the program
int menuSelector(void)
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






// mycontroller.ru code

uint32_t RTC_GetCounter(void)
{
  return  (uint32_t)((RTC->CNTH << 16) | RTC->CNTL);
}






void RTC_SetCounter(uint32_t value)
{
  RTC->CRL |= RTC_CRL_CNF;
  RTC->CNTH = value>>16;
  RTC->CNTL = value;
  RTC->CRL &= ~RTC_CRL_CNF;
}






unsigned char  RtcInit(void)
{

  RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;

  PWR->CR |= PWR_CR_DBP;

  if ((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN)
  {
    RCC->BDCR |=  RCC_BDCR_BDRST;
    RCC->BDCR &= ~RCC_BDCR_BDRST;
    RCC->BDCR |=  RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_LSE;
    RTC->CRL  |=  RTC_CRL_CNF;
    RTC->PRLL  = 0x7FFF;        // divider to 32786 hZ
    RTC->CRL  &=  ~RTC_CRL_CNF;
    RCC->BDCR |= RCC_BDCR_LSEON;
    while ((RCC->BDCR & RCC_BDCR_LSEON) != RCC_BDCR_LSEON){}

    RTC->CRL &= (uint16_t)~RTC_CRL_RSF;
    while((RTC->CRL & RTC_CRL_RSF) != RTC_CRL_RSF){}
//    RTC_SetCounter(YEAR12SECS+(212+20)*86400+10*3600+9*60);
    return 1;
  }
  return 0;
}


 uint32_t TimeToRtc(RTC_Time *time )
{
  uint32_t result;
  result  = (uint32_t)time->hour * 3600;
  result += (uint32_t)time->min * 60;
  result +=           time->sec;
  return result;
}

 void RtcToTime( uint32_t cnt, RTC_Time *time )
 {
   time->sec = cnt % 60;
   cnt /= 60;
   time->min = cnt % 60;
   cnt /= 60;
   time->hour = cnt % 24;
 }

 // EOF mycontroller.ru RTC functions



// function slides the buffer window for pH ADC values (and EC)
void phMonitor(void *pvParameters){
 	while(1){
 		vTaskDelay(1);
 		adcAverager();
 	}
}

void copy_arr(uint8_t *source, uint8_t *destination, uint8_t amount, uint8_t pos){
	int i=0;
	for (i=0; i<amount;i++) {
		destination[i+pos] = source[i];
	}
}


void displayClock(void *pvParameters)
{
		RTC_DateTime	DateTime;
		uint32_t tmp;
		uint8_t button;
//		char str[6];
		Lcd_clear();
    	while (1)
	    {


//    		if (GetStateDMAChannel4()==1){
//    			StartDMAChannel4(64);
//    		}

	    	vTaskDelay(10);
    		tmp = RTC_GetCounter();
    		DateTime=unix2DateTime(tmp);
	    	LCDLine1[0]= (DateTime.day / 10) + 48;
	    	LCDLine1[1]= (DateTime.day % 10) + 48;
	    	LCDLine1[2]= 45;
	    	LCDLine1[3]= (DateTime.month / 10) + 48;
	    	LCDLine1[4]= (DateTime.month % 10) + 48;
	    	LCDLine1[5]= 45;
	    	LCDLine1[6]= (DateTime.year / 10) + 48;
	    	LCDLine1[7]= (DateTime.year % 10) + 48;
	    	LCDLine2[6]= 32;
	    	LCDLine2[7]= 32;


	    	Lcd_goto(1,0);
	    	Lcd_write_digit(DateTime.hour);
	    	Lcd_write_digit(DateTime.min);
	    	Lcd_write_digit(DateTime.sec);
    	//	displayAdcValues();

	    	vTaskDelay(5);

	    	Lcd_goto(1,9);
	    	Lcd_write_str("pH:");
	    	vTaskDelay(5);
	    	getPh();
			copy_arr(&curphstr, LCDLine2, 4, 12);
	    	vTaskDelay(5);
	    	Lcd_goto(0,9);
#ifdef	TEST_MODE
	    	Lcd_write_str("EC:");
	    	vTaskDelay(5);
	    	getEc();
	    	vTaskDelay(9);
			copy_arr(&curecstr, LCDLine1, 4, 12);
#endif
//			setPwmDc(90);
#ifndef	TEST_MODE
	    	uint16_t flg;
	    	// vyvod flagov dozirujushih nasosov
	    	uint8_t i;
	    	for (i=0; i<3; i++) {
	    		flg=timerStateFlags&(1<<i);

//	    		flg=wpStateFlags&(1<<i);
//	    		flg=dosingPumpStateFlags2&(1<<i);
	    		flg>>=i;
	    		LCDLine1[i+10]=flg+48;
	    	}
	    	// vyvod flagov rozetok
	    	for (i=0; i<3; i++) {
	    		flg=plugStateFlags&(1<<i);
	    		flg>>=i;
	    		LCDLine1[i+13]=flg+48;
	    	}
	    	LCDLine1[9]=phUnderOver+48;
#endif

	    	vTaskDelay(14);
	    	button=readButtons();
	    	vTaskDelay(3);

	    	if (button==BUTTON_OK)
	    	{
	    		vTaskDelay(100);
	    		Lcd_clear();
	    		vTaskDelay(500);
	    		uint8_t progId=menuSelector();
	    		programRunner(progId);
	    	}
	    	// USART_SendData(BT_USART, 50);
//	    	USART1->DR=55;


	    	vTaskDelay(10);

	    }





    	while (1) {


    	}
}

uint8_t readButtons(void){
	uint16_t curval = 0;
	uint8_t i;
		curval = adcAverage[ADC_AVG_BUTTONS];
		for (i=0;i<4;i++) {
				if (curval>button_ranges[i*2]+BUTTON_RANGE_SHRINKER && curval<button_ranges[i*2+1]-BUTTON_RANGE_SHRINKER) {
					return i+1;
				}

	}
	return 0;


}

void adcRegularInit(void){

}



void AdcInit(void)
{
	// WARNING NTBU (needs to be updated)! shifting pins down to 0 from 1st, saves some space on discovery board
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    GPIOC->CRL      &= ~GPIO_CRL_CNF0;		// LOAD triggering from PC0...
    GPIOC->CRL      &= ~GPIO_CRL_CNF1;
    GPIOC->CRL      &= ~GPIO_CRL_CNF2;
    GPIOC->CRL      &= ~GPIO_CRL_CNF3;

	  GPIOA->CRL   &= ~GPIO_CRL_MODE0;
	  GPIOA->CRL   &= ~GPIO_CRL_CNF0;

	  GPIOA->CRL   &= ~GPIO_CRL_MODE1;
	  GPIOA->CRL   &= ~GPIO_CRL_CNF1;

	  GPIOA->CRL   &= ~GPIO_CRL_MODE2;
	  GPIOA->CRL   &= ~GPIO_CRL_CNF2;

	  GPIOA->CRL   &= ~GPIO_CRL_MODE3;
	  GPIOA->CRL   &= ~GPIO_CRL_CNF3;

//	  GPIOA->CRL   &= ~GPIO_CRL_MODE4;
//	  GPIOA->CRL   &= ~GPIO_CRL_CNF4;

	  RCC->APB2ENR |=  RCC_APB2ENR_ADC1EN;
	  RCC->CFGR    &= ~RCC_CFGR_ADCPRE;
	  ADC1->CR1     =  0;
	  ADC1->CR2    |=  ADC_CR2_CAL;
	  while (!(ADC1->CR2 & ADC_CR2_CAL)){};
	  ADC1->CR2     =  ADC_CR2_JEXTSEL;
	  ADC1->CR2    |=  ADC_CR2_JEXTTRIG;
	  ADC1->CR2    |=  ADC_CR2_CONT;
	  ADC1->CR1    |=  ADC_CR1_SCAN;
	  ADC1->CR1    |=  ADC_CR1_JAUTO;
	  ADC1->JSQR    =  (uint32_t)(4-1)<<20;
	  ADC1->JSQR   |=  (uint32_t)1<<(5*0);
	  ADC1->JSQR   |=  (uint32_t)2<<(5*1);
	  ADC1->JSQR   |=  (uint32_t)3<<(5*2);
	  ADC1->JSQR   |=  (uint32_t)4<<(5*3);
	  ADC1->SMPR1  |=	0x00FFFFFF;
	  ADC1->CR2    |=  ADC_CR2_ADON;
	  ADC1->CR2    |=  ADC_CR2_JSWSTART;
}

void Lcd_write_digit2(uint8_t numb){
	if (numb<10) {
		Lcd_write_data(48);
		Lcd_write_data(48+numb);
	}
	else {
		Lcd_write_data((numb/10)+48);
		Lcd_write_data((numb-(numb/10)*10)+48);
	}
}

void Lcd_write_digit(uint8_t numb){
	if (lcd_pointery==0) {
		LCDLine1[lcd_pointerx] = ((numb%100)/10)+48;
		lcd_pointerx++;
		LCDLine1[lcd_pointerx] = (numb%10)+48;
		lcd_pointerx++;
	}
	else {
		LCDLine2[lcd_pointerx] = ((numb%100)/10)+48;
		lcd_pointerx++;
		LCDLine2[lcd_pointerx] = (numb%10)+48;
		lcd_pointerx++;
	}
}





void prvSetupHardware()
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;     // Enable clock on GPIOC.
//	        GPIOC->CRH      &= ~GPIO_CRH_CNF8;              //  Push-Pull. LED PC8
//	        GPIOC->CRH      &= ~GPIO_CRH_CNF9;              //  Push-Pull. LED PC9
	        GPIOC->CRL      &= ~GPIO_CRL_CNF0;		// LOAD triggering from PC0...
	        GPIOC->CRL      &= ~GPIO_CRL_CNF1;
	        GPIOC->CRL      &= ~GPIO_CRL_CNF2;
	        GPIOC->CRL      &= ~GPIO_CRL_CNF3;		// ... to PC3


//	        GPIOC->CRH   |= GPIO_CRH_MODE8_0;       //  10MHz.
	        GPIOC->CRL   |= GPIO_CRL_MODE0_0;
	        GPIOC->CRL   |= GPIO_CRL_MODE1_0;
	        GPIOC->CRL   |= GPIO_CRL_MODE2_0;
	        GPIOC->CRL   |= GPIO_CRL_MODE3_0;



}


void vTaskLCDdraw(void *pvParameters) {	// draws lcd
//	uint32_t tmp=0;
//	int str;
	for (;;) {
		vTaskSuspendAll();
		Lcd_write_cmd(0x80);	// lcd_goto(0,0)
//		Lcd_goto(0,0);
		Lcd_write_arr(&LCDLine1, 16);
//		xTaskResumeAll();
//		vTaskDelay(15);
//		vTaskSuspendAll();
		Lcd_write_cmd(0x80+0x40);	// lcd_goto(1,0)
		Lcd_write_arr(&LCDLine2, 16);
		xTaskResumeAll();
		vTaskDelay(17);
	}
}


void int32str(uint32_t d, char *out)
{
    out[10] = '\0';
    out[9] = '0' + ( d       )    % 10;
    out[8] = '0' + ( d /= 10 )    % 10;
    out[7] = '0' + ( d /= 10 )    % 10;
    out[6] = '0' + ( d /= 10 )    % 10;
    out[5] = '0' + ( d /= 10 )    % 10;
    out[4] = '0' + ( d /= 10 )    % 10;
    out[3] = '0' + ( d /= 10 )    % 10;
    out[2] = '0' + ( d /= 10 )    % 10;
    out[1] = '0' + ( d /= 10 )    % 10;
    out[0] = '0' + ( d /  10 )    % 10;
//    return out;
}

void int10str(uint32_t d, char *out)
{
    out[4] = '\0';
    out[3] = '0' + ( d       )    % 10;
    out[2] = '0' + ( d /= 10 )    % 10;
    out[1] = '0' + ( d /= 10 )    % 10;
    out[0] = '0' + ( d /  10 )    % 10;
//    return out;
}

uint32_t EE_ReadWord(uint16_t Address){
	uint16_t tmp, tmp2;
	uint32_t Data = 0;
	EE_ReadVariable(Address, &tmp);
	EE_ReadVariable(Address+1, &tmp2);
	Data = ((uint32_t)tmp << 16) + (uint32_t)tmp2;
	return Data;
}

void flush_lcd_buffer(void){
	uint8_t i=0;
	for (i=0;i<16;i++){
		LCDLine1[i]=32;
		LCDLine2[i]=32;
	}
}


void setPwmDc(uint8_t duty_cycle){		// duty_cycle in %
	TIM3->CCR3 = duty_cycle*10;
	TIM3->CCR4 = 1000 - duty_cycle*10;
}

void setDutyCycle(void){
	uint8_t button=0, duty_cycle;
	duty_cycle = TIM3->CCR3/10;
	Lcd_clear();
	while (button!=BUTTON_OK) {
		button = readButtons();
		Lcd_goto(0,0);
		Lcd_write_str("Crnt duty cycle");
		Lcd_goto(1,0);
		Lcd_write_digit(duty_cycle);
		if (button==BUTTON_FWD) {
			if (duty_cycle==99) {
			}
			else {
				duty_cycle++;
				setPwmDc(duty_cycle);
			}
		}
		if (button==BUTTON_BCK) {
			if (duty_cycle==0) {
			}
			else {
				duty_cycle--;
				setPwmDc(duty_cycle);
			}
		}
		vTaskDelay(20);
	}
	Lcd_clear();
}

int main(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_AFIO, ENABLE);
	// VALVE INIT
	GPIO_InitTypeDef init_pin;
 	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
 	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
 	init_pin.GPIO_Pin  = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
	init_pin.GPIO_Mode = GPIO_Mode_Out_PP;
	init_pin.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &init_pin);
	// VALVE INIT
	SystemInit();
#ifdef USE_VALVES
	stop_valve_motor(0);	// ball valves
	stop_valve_motor(1);
#endif

	uint32_t i;
	dosing_motor_control_init();
	for (i=0;i<4;i++) {
		enable_dosing_pump(i,0);	// startup disable dosing pumps
	}
	/* Unlock the Flash Program Erase controller */
	FLASH_Unlock();
	/* EEPROM Init */
	EE_Init();
	AdcInit();
	dht_init();
	RtcInit();		//init real time clock

	DSTATUS resp=0;		// variable for status response from sd card init function
	no_sd = disk_initialize(0);		// init card
	if (no_sd==0){
		no_sd = f_mount(0, &fs);
		no_sd = string2log("System started\n", 15);
	}


#ifndef PAPA_EDITION
	sonar_init();
#endif
	// SOLENOID VALVE RE-INIT
 	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
 	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
 	init_pin.GPIO_Pin  = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
	init_pin.GPIO_Mode = GPIO_Mode_Out_PP;
	init_pin.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &init_pin);
	// EOF SOLENOID VALVE RE-INIT
	prvSetupHardware();

	bluetooth_init();
	Init_lcd();
#ifdef USE_VALVES
	valve_feedback_init();
#endif

	buttonCalibration();

	Lcd_clear();

	// setup gpio for load triggering and led indication
	// reinit valves HARDCODE
#ifdef USE_VALVES
	//

#endif


	Lcd_clear();
	loadSettings();
	flush_lcd_buffer();	// fills the LCD frame buffer with spaces
	xTaskCreate(uart_task,(signed char*)"uart",50,
	            NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(displayClock,(signed char*)"CLK",140,
            NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(timerStateTrigger,(signed char*)"TIMERS",configMINIMAL_STACK_SIZE,
            NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(plugStateTrigger,(signed char*)"PLUGS",configMINIMAL_STACK_SIZE+35,
            NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(phMonitor,(signed char*)"PHMON",configMINIMAL_STACK_SIZE,
            NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vTaskLCDdraw,(signed char*)"LCDDRW",configMINIMAL_STACK_SIZE,
            NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(watering_program_trigger,(signed char*)"WP",configMINIMAL_STACK_SIZE+80,
            NULL, tskIDLE_PRIORITY + 1, NULL);
if (resp==0){
	xTaskCreate(sdLog,(signed char*)"SDLOG",128,NULL, tskIDLE_PRIORITY + 1, NULL);
}

/* Start the scheduler. */

    vTaskStartScheduler();

    while(1);
}


void loadSettings(void){	// function loads the predefined data
	uint16_t i, Address, Data;
//	uint8_t i2=0;
	char tmpstr[11], putstring[50];
	string2log("build date: ", 12);
	string2log(__DATE__, 11);
	string2log(" build time: ", 12);
	string2log(__TIME__, 8);
	string2log(" build ver.: ", 12);
//	string2log(VERSION_BUILD, 6);





	for (i=0;i<8;i++) {
		int32str(button_ranges[i],&tmpstr);
		tmpstr[10] = 0x0A;
		string2log(tmpstr, 12);
	}

	for (i=0; i<PLUG_AMOUNT; i++) {
		Address=EE_PLUG_SETTINGS+i;
		EE_ReadVariable(Address, &Data);
		plugSettings[i]=Data;
		if (no_sd==0) {
			string2log("plug no.", 8);
			int32str(i,&tmpstr);
			string2log(tmpstr, 11);
			string2log(" has timer no.", 14);
			int32str(plugSettings[i],&tmpstr);
			tmpstr[10] = 0x0A;
			string2log(tmpstr, 11);
		}
	}

	EE_ReadVariable(WATER_TANK_TOP, &tank_windows_top[0]);
	EE_ReadVariable(WATER_TANK_BOTTOM, &tank_windows_bottom[0]);

	EE_ReadVariable(PSI_SENSOR_TOP, &psi_pump_top_level);
	EE_ReadVariable(PSI_SENSOR_BTM, &psi_pump_btm_level);
	EE_ReadVariable(PSI_PUMP_LOAD_ID, &psi_pump_load_id);

	for (i=0; i<WFM_AMOUNT; i++) {
		EE_ReadVariable(WFM_CAL_OFFSET+i, &wfCalArray[i]);
	}


	// log FMP data

	for (i=0; i<WP_AMOUNT; i++) {
		if (no_sd==0) {
			string2log("FMP no.", 6);
			int32str(i,&tmpstr);
			string2log(tmpstr, 11);
			string2log(" has:", 5);
			tmpstr[0] = 0x0A;
			string2log(tmpstr, 1);

			Address = FMP_OFFSET+i*FMP_SIZE+FMP_DOSING_PUMP_ID_SHIFT;
			EE_ReadVariable(Address, &Data);
			string2log("- fert pump id: ", 15);
			int32str(Data,&tmpstr);
			tmpstr[10] = 0x0A;
			string2log(tmpstr, 11);

			Address = FMP_OFFSET+i*FMP_SIZE+FMP_DOSING_TIME_SHIFT;
			EE_ReadVariable(Address, &Data);
			string2log("- dosing duration: ", 19);
			int32str(Data,&tmpstr);
			tmpstr[10] = 0x0A;
			string2log(tmpstr, 11);

			Address = FMP_OFFSET+i*FMP_SIZE+FMP_CIRCULATION_MIXING_TIME_SHIFT;
			EE_ReadVariable(Address, &Data);
			string2log("- aftermix time: ", 17);
			int32str(Data,&tmpstr);
			tmpstr[10] = 0x0A;
			string2log(tmpstr, 11);

			Address = FMP_OFFSET+i*FMP_SIZE+FMP_2_WP_ASSOC_SHIFT;;
			EE_ReadVariable(Address, &Data);
			string2log("- WP link: ", 11);
			int32str(Data,&tmpstr);
			tmpstr[10] = 0x0A;
			string2log(tmpstr, 11);

			Address = FMP_OFFSET+i*FMP_SIZE+FMP_TRIG_FREQUENCY_SHIFT;
			EE_ReadVariable(Address, &Data);
			string2log("- trig freq.: ", 14);
			int32str(Data,&tmpstr);
			tmpstr[10] = 0x0A;
			string2log(tmpstr, 11);

			Address = FMP_OFFSET+i*FMP_SIZE+FMP_ENABLE;
			EE_ReadVariable(Address, &Data);
			string2log("- enabled?: ", 12);
			int32str(Data,&tmpstr);
			tmpstr[10] = 0x0A;
			string2log(tmpstr, 11);


		}
	}


	// log WateringPrograms data

	for (i=0; i<WP_AMOUNT; i++) {
		if (no_sd==0) {
			string2log("WP no.", 6);
			int32str(i,&tmpstr);
			string2log(tmpstr, 11);
			string2log(" has:", 5);
			tmpstr[0] = 0x0A;
			string2log(tmpstr, 1);

			Address = WP_OFFSET+i*WP_SIZE+TOP_WATER_SENSOR_SHIFT;
			EE_ReadVariable(Address, &Data);
			string2log("- top sensor: ", 14);
			int32str(Data,&tmpstr);
			tmpstr[10] = 0x0A;
			string2log(tmpstr, 11);

			Address = WP_OFFSET+i*WP_SIZE+BOTTOM_WATER_SENSOR_SHIFT;
			EE_ReadVariable(Address, &Data);
			string2log("- btm sensor: ", 14);
			int32str(Data,&tmpstr);
			tmpstr[10] = 0x0A;
			string2log(tmpstr, 11);

			Address = WP_OFFSET+i*WP_SIZE+WATER_FILL_TIMEOUT_SHIFT;
			EE_ReadVariable(Address, &Data);
			string2log("- water fill timeout: ", 21);
			int32str(Data,&tmpstr);
			tmpstr[10] = 0x0A;
			string2log(tmpstr, 11);

			Address = WP_OFFSET+i*WP_SIZE+WP_WATERING_PUMP_PLUG_ID;
			EE_ReadVariable(Address, &Data);
			string2log("- wtrng pump plug id: ", 21);
			int32str(Data,&tmpstr);
			tmpstr[10] = 0x0A;
			string2log(tmpstr, 11);

			Address = WP_OFFSET+i*WP_SIZE+WP_DURATION_SHIFT;
			EE_ReadVariable(Address, &Data);
			string2log("- wtrng duration: ", 18);
			int32str(Data,&tmpstr);
			tmpstr[10] = 0x0A;
			string2log(tmpstr, 11);

			Address = WP_OFFSET+i*WP_SIZE+WP_INTERVAL_SHIFT;
			EE_ReadVariable(Address, &Data);
			string2log("- wtrng interval: ", 18);
			int32str(Data,&tmpstr);
			tmpstr[10] = 0x0A;
			string2log(tmpstr, 11);

			Address = WP_OFFSET+i*WP_SIZE+WP_ENABLE_SHIFT;
			EE_ReadVariable(Address, &Data);
			string2log("- enabled?: ", 12);
			int32str(Data,&tmpstr);
			tmpstr[10] = 0x0A;
			string2log(tmpstr, 11);


		}
	}


//	Address=LIGHT_RANGE_ADDR;
//	EE_ReadVariable(Address, &lightRange);
	EE_ReadVariable(PH4_ADDR, &ph4);

	EE_ReadVariable(CIRCULATION_PUMP_ID_ADDR, &circulationPumpId);

//	Address=PH7_ADDR;
	EE_ReadVariable(PH7_ADDR, &ph_seven);
	cdel = (ph_seven - ph4)/3;
	ph0 = ph4 - cdel*4;

	EE_ReadVariable(PH_WINDOW_TOP, &phWindowTop);
	EE_ReadVariable(PH_WINDOW_BOTTOM, &phWindowBottom);


	EE_ReadVariable(EC0_ADDR, &ec0);
	EE_ReadVariable(EC1413_ADDR, &ec1413);


	if (no_sd==0) {

		// vnesti v log znachenie kalibrovshika EC na 1.413mS
		string2log("EC1.413 set @: ", 15);
		int32str(ec1413,&tmpstr);
		tmpstr[10] = 0x0A;
		string2log(tmpstr, 11);

		// put the EC0 ADC level into log
		string2log("EC0 set @: ", 11);
		int32str(ec0,&tmpstr);
		tmpstr[10] = 0x0A;
		string2log(tmpstr, 11);

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

		// ph window top
		int10str(phWindowTop,&tmpstr);
		for (i=0;i<4;i++){
			putstring[i]=tmpstr[i];
		}
		putstring[4] = 10;
		string2log("ph window top ", 14);
		string2log(putstring, 5);

		// ph window bottom
		int10str(phWindowBottom,&tmpstr);
		for (i=0;i<4;i++){
			putstring[i]=tmpstr[i];
		}
		putstring[4] = 10;
		string2log("ph window btm ", 14);
		string2log(putstring, 5);

		string2log("EC 1.413 cal @", 14);
		int10str(ec1413,&tmpstr);
		for (i=0;i<4;i++){
			putstring[i]=tmpstr[i];
		}
		putstring[4] = 10;
		string2log(putstring, 5);
	}

	EE_ReadVariable(SD_LOG_INTERVAL, &logInterval);
	readButtonRanges();
}


void Lcd_write_str(char *STRING)
{
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
	}
}

void Lcd_goto(uc8 x,uc8 y)
{
	int str;
	str = y + 0x80;
	if(x == 1)
	{
	str+= 0x40;
	}
	Lcd_write_cmd(str);
	lcd_pointerx = y;
	lcd_pointery = x;
}


// Cadi mainboard LCD Init
void Init_pin_out()
{
	// Cadi MB pins for LCD are following:
	// data: PA15, PC10, PC11, PC12
	// cmd: PD2, PB3, PB4



	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD, ENABLE);
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
//	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
//	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
	GPIO_InitTypeDef init_pin;
	init_pin.GPIO_Pin  = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	init_pin.GPIO_Mode = GPIO_Mode_Out_PP;
	init_pin.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &init_pin);
	init_pin.GPIO_Pin  = GPIO_Pin_15;
	init_pin.GPIO_Mode = GPIO_Mode_Out_PP;
	init_pin.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &init_pin);
	init_pin.GPIO_Pin  = GPIO_Pin_3 | GPIO_Pin_4;
	init_pin.GPIO_Mode = GPIO_Mode_Out_PP;
	init_pin.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &init_pin);
	init_pin.GPIO_Pin  = GPIO_Pin_2;
	init_pin.GPIO_Mode = GPIO_Mode_Out_PP;
	init_pin.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &init_pin);
}


void Init_pin_out_bak()
{
	RCC_APB2PeriphClockCmd(lcd_init_port_data | lcd_init_port_cmd, ENABLE);
	GPIO_InitTypeDef init_pin;
	init_pin.GPIO_Pin  = pin_d7 | pin_d6 | pin_d5 | pin_d4;
	init_pin.GPIO_Mode = GPIO_Mode_Out_PP;
	init_pin.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(lcd_port_data, &init_pin);
	init_pin.GPIO_Pin  = pin_e | pin_rs | pin_rw;
	init_pin.GPIO_Mode = GPIO_Mode_Out_PP;
	init_pin.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(lcd_port_cmd, &init_pin);
}

void Init_pin_in()
{
	RCC_APB2PeriphClockCmd(lcd_init_port_data, ENABLE);
	GPIO_InitTypeDef init_pin;
	init_pin.GPIO_Pin  =  pin_d7 | pin_d6 | pin_d5 | pin_d4 ;
	init_pin.GPIO_Mode = GPIO_Mode_IPD;
	init_pin.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init (lcd_port_data, &init_pin);
}

void Lcd_write_cmd(uc8 cmd )
{
		Delay_us(6);	// stable 240
		rs_0;rw_0;e_1;
		Delay_us(1);
		set4highBits(cmd);
		e_0;
		Delay_us(1);
		e_1;
		Delay_us(1);
		set4lowBits(cmd);
		e_0;
		Delay_us(6);
}

void Lcd_write_data(uint8_t data)
{
	Delay_us(6);
	e_1;rs_1;rw_0;
	Delay_us(1);
	set4highBits(data);
	Delay_us(1);
	e_0;
	Delay_us(1);
	e_1;
	Delay_us(1);
	set4lowBits(data);
	Delay_us(1);
	e_0;
	Delay_us(1);
	e_0;rs_0;rw_0;
	Delay_us(6);
}

void set4highBits(uint8_t dta){		// setting higher 4 bits of word on corresponding GPIO pins
/*	if (dta&16) {
		d4_0;
	}
	else {
		d4_1;
	}
	if (dta&32) {
		d5_0;
	}
	else {
		d5_1;
	}
	if (dta&64) {
		d6_0;
	}
	else {
		d6_1;
	}
	if (dta&128) {
		d7_0;
	}
	else {
		d7_1;
	} */

	if (dta&16) {
		d4_1;
	}
	else {
		d4_0;
	}
	if (dta&32) {
		d5_1;
	}
	else {
		d5_0;
	}
	if (dta&64) {
		d6_1;
	}
	else {
		d6_0;
	}
	if (dta&128) {
		d7_1;
	}
	else {
		d7_0;
	}
}


void set4lowBits(uint8_t dta){
	if (dta&1) {
		d4_1;
	}
	else {
		d4_0;
	}
	if (dta&2) {
		d5_1;
	}
	else {
		d5_0;
	}
	if (dta&4) {
		d6_1;
	}
	else {
		d6_0;
	}
	if (dta&8) {
		d7_1;
	}
	else {
		d7_0;
	}
}

void Init_lcd()
{
	Init_pin_out();
//	  Delay_us(10000);
	  e_1;rs_0;rw_0;
	      Delay_us(100);	// assume 10ms
	      set4lowBits(0b0010);	// set 4 bit bus
	      e_0;
	      Delay_us(10);	// assume 10ms

	      Lcd_write_cmd(0b00101000);	// again, 4bit bus and the rest 4bits of whole command will get the destination now
	      Delay_us(10);
//	  	  del=72000; while (del--){}
	  	  Lcd_write_cmd(Display_clear);
//	      del=72000; while (del--){}

		  Lcd_write_cmd(0b00000110);	// function set
//		  del=72000; while (del--){}

		  Lcd_write_cmd(0b00001100);	// display on cursor off
//		  del=72000; while (del--){}


		  Lcd_write_cmd(Display_clear);	// function set
//		  del=72000; while (del--){}

//		  Lcd_write_str("A");



		  Lcd_write_str("12345678");


	  	Delay_us(10);
}

void Lcd_clear()
{
	Lcd_write_cmd(Display_clear);
	flush_lcd_buffer();
	Lcd_goto(0,0);
}

void Return_home()
{
	Lcd_write_cmd(0b0000001);
}


