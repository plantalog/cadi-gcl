#include "hd44780_lib.h"

u32 del;
const unsigned char russian[]={ 0x41, 0xA0, 0x42, 0xA1, 0xE0, 0x45,
0xA3, 0xA4, 0xA5,0xA6, 0x4B, 0xA7, 0x4D, 0x48, 0x4F, 0xA8, 0x50,0x43,
0x54, 0xA9, 0xAA, 0x58, 0xE1, 0xAB, 0xAC, 0xE2, 0xAD,0xAE, 0x62,
0xAF, 0xB0, 0xB1, 0x61, 0xB2, 0xB3, 0xB4, 0xE3, 0x65, 0xB6, 0xB7,
0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0x6F, 0xBE, 0x70, 0x63,0xBF,
0x79, 0xE4, 0x78, 0xE5, 0xC0, 0xC1, 0xE6, 0xC2,0xC3, 0xC4, 0xC5,
0xC6, 0xC7 };

// string data to display
extern uint8_t		LCDLine1[16], LCDLine2[16];		// framebuffer
extern uint8_t lcd_pointerx=0, lcd_pointery=0;

void Lcd_write_str(uc8 *STRING)
{
	char c;
	while (c=*STRING++){
		if(c>=192) {
			Lcd_write_data(russian[c-192]);
		}
		else {
			Lcd_write_data(c);
		}
	}
}


void Lcd_write_arr(uc8 *STRING, uint8_t chars)
{
	char c;
	uint8_t i;
	for (i=0; i<chars; i++) {
		c=STRING[i];
//		vTaskDelay(5);
	if(c>=192) Lcd_write_data(russian[c-192]);
	else Lcd_write_data(c);
	}
}

void Lcd_goto2(uc8 x,uc8 y)
{
	int str;
	str = y + 0x80;
	if(x == 1)
	{
	str+= 0x40;
	}
	Lcd_write_cmd(str);
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


void STM32F4_LCDInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);		// PD11, 13, 15

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = pin_d7 | pin_d6 | pin_d5 | pin_d4 | pin_rw | pin_rs | pin_e | GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// vozmozhno uvelichit'
  GPIO_Init(port, &GPIO_InitStructure);
}

void set4highBits(uint8_t dta){		// setting higher 4 bits of word on corresponding GPIO pins
	if (dta&16) lcd_port->BSRRL |= (pin_d4);
	else lcd_port->BSRRH |= (pin_d4);
	if (dta&32) lcd_port->BSRRL |= (pin_d5);
	else lcd_port->BSRRH |= (pin_d5);
	if (dta&64) lcd_port->BSRRL |= (pin_d6);
	else lcd_port->BSRRH |= (pin_d6);
	if (dta&128) lcd_port->BSRRL |= (pin_d7);
	else lcd_port->BSRRH |= (pin_d7);
}

void set4lowBits(uint8_t dta){
	if (dta&1) lcd_port->BSRRL |= (pin_d4);
	else lcd_port->BSRRH |= (pin_d4);
	if (dta&2) lcd_port->BSRRL |= (pin_d5);
	else lcd_port->BSRRH |= (pin_d5);
	if (dta&4) lcd_port->BSRRL |= (pin_d6);
	else lcd_port->BSRRH |= (pin_d6);
	if (dta&8) lcd_port->BSRRL |= (pin_d7);
	else lcd_port->BSRRH |= (pin_d7);

}

void Lcd_write_cmd(uc8 cmd )
{
    //  mercajushij kursor display on RABOCHIJ kod
/*	      e_1;
    Delay_us(100);	// assume 10ms
    set4lowBits(0b0000);	// 4 bit, chuvak!
    e_0;
    Delay_us(400);	// assume 10ms
    e_1;
    Delay_us(100);	// assume 10ms
    set4lowBits(0b1111);
    e_0;
    Delay_us(100);	// assume 10ms */
//	STM32F4_LCDInit();		// rabotaet ne vezde
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
//	STM32F4_LCDInit();		// rabotaet ne na vseh display'ah
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

/*
// FreeRTOS version of delay
void Delay_us(uint32_t delay){
	vTaskDelay(delay);
}
*/


void Delay_us(uint32_t delay){
	del=delay*SPEED_K; while (del--){}
}


void Init_lcd()
{
	STM32F4_LCDInit();

	  Delay_us(10000);
	  e_1;rs_0;rw_0;
	      Delay_us(100);	// assume 10ms
	      set4lowBits(0b0010);	// set 4 bit bus
	      e_0;
	      Delay_us(400);	// assume 10ms

	      Lcd_write_cmd(0b00101000);	// again, 4bit bus and the rest 4bits of whole command will get the destination now
	      Delay_us(1000);
	  	  del=72000; while (del--){}
	  	  Lcd_write_cmd(Display_clear);
	      del=72000; while (del--){}

		  Lcd_write_cmd(0b00000110);	// function set
		  del=72000; while (del--){}

		  Lcd_write_cmd(0b00001100);	// display on cursor off
		  del=72000; while (del--){}


		  Lcd_write_cmd(Display_clear);	// function set
		  del=72000; while (del--){}

//		  Lcd_write_str("A");



//		  Lcd_write_str("12345678");


	  	Delay_us(700);

/*
#define Function_set 				0b00000011//4-bit,2 - line mode, 5*8 dots
#define Display_on_off_control		0b00001100/// display on,cursor off,blink off
#define Display_clear				0b00000001
#define Entry_mode_set				0b00000100//



http://easyelectronics.ru/avr-uchebnyj-kurs-podklyuchenie-k-avr-lcd-displeya-hd44780.html
http://www.foxytronics.com/tutorials/_/pic-microcontrollers/hd44780-lcd-r14
http://makesystem.net/?p=9

*/
}

void Lcd_write_digit(uint8_t numb){
	  		if (numb<10) {
	  			Lcd_write_data(48);
	  			Lcd_write_data(48+numb);
	  		}
	  		else {
	  			Lcd_write_data((numb/10)+48);
	  			Lcd_write_data((numb-(numb/10)*10)+48);
	  		}
}

void Lcd_clear()
{
	Lcd_write_cmd(Display_clear);
	lcd_pointerx = 0;
	lcd_pointery = 0;
}

void Return_home()
{
	Lcd_write_cmd(0b0000001);
}
