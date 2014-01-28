#include "stm32f4xx_conf.h"

#define lcd_port 			GPIOE
#define init_port         	RCC_AHB1Periph_GPIOE
#define pin_e              	GPIO_Pin_10
#define pin_rw             	GPIO_Pin_12
#define pin_rs             	GPIO_Pin_14

#define SPEED_K                168

#define LOG_SHIFT        1

#define lcd_shift        1
#define pin_d4                GPIO_Pin_15
#define pin_d5                GPIO_Pin_13
#define pin_d6                GPIO_Pin_11
#define pin_d7                GPIO_Pin_9


#define Function_set                                 0b00100000//4-bit,2 - line mode, 5*8 dots
#define Display_on_off_control                0b00001100/// display on,cursor off,blink off
#define Display_clear                                0b00000001
#define Entry_mode_set                                0b00000100//


//#define Function_set 0b00100000//4-bit,2 - line mode, 5*8 dots
//#define Display_on_off_control 0b00001111// display on,cursor off,blink off
//#define Display_clear 0b00000001
//#define Entry_mode_set 0b00000100//increment mode,entrir

#define rs_1        lcd_port->ODR |= pin_rs
#define rs_0        lcd_port->ODR &=~ pin_rs

//#define e_1         port->BSRRL |= pin_e
//#define e_0         port->BSRRH &=~ pin_e
#define e_1         lcd_port->ODR |= pin_e
#define e_0         lcd_port->ODR &=~ pin_e
#define rw_1        lcd_port->ODR |= pin_rw
#define rw_0        lcd_port->ODR &=~ pin_rw

#define d40                lcd_port->ODR &=~ pin_d4
#define d41                lcd_port->ODR |= pin_d4
#define d50                lcd_port->ODR &=~ pin_d5
#define d51                lcd_port->ODR |= pin_d5
#define d60                lcd_port->ODR &=~ pin_d6
#define d61                lcd_port->ODR |= pin_d6
#define d70                lcd_port->ODR &=~ pin_d7
#define d71                lcd_port->ODR |= pin_d7


void flush_lcd_buffer(void);
void Init_pin_out(void);
void Init_lcd(void);
void Lcd_write_data(uint8_t byte);
void Lcd_write_cmd(uint8_t byte);
void Lcd_clear(void);
void Return_home(void);
void Lcd_goto(uc8 x, uc8 y);
void Lcd_write_str(uc8 *STRING);
void STM32F4_LCDInit(void);
void Delay_us(uint32_t delay);
void Lcd_write_arr(uc8 *STRING, uint8_t chars);
