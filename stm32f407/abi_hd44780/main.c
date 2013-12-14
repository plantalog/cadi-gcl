#include "stdio.h"
#include "hd44780.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4_discovery.h"




int main(void){

// Custom character definitions
uint8_t CustChar1[8] = {b00011,
						b00100,
						b01010,
						b10000,
						b10100,
						b01011,
						b00100,
						b00011};

uint8_t CustChar2[8] = {b11000,
						b00100,
						b01010,
						b00001,
						b00101,
						b11010,
						b00100,
						b11000};

int buttonPressed;


  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);
buttonPressed = 0;



// Initialize the LCD
	LCD_ConfigurePort(GPIOE,
					GPIO_Pin_6, GPIO_Pin_4, GPIO_Pin_5,
					NULL, NULL, NULL, NULL,
					GPIO_Pin_2, GPIO_Pin_3, GPIO_Pin_0, GPIO_Pin_1
					);

	LCD_Initalize(BUS_WIDTH_4, DISPLAY_LINES_2, FONT_5x8);






// Load in the custom characters into the LCD RAM.
	LCD_CustChar(CustChar1, 0);
	LCD_CustChar(CustChar2, 1);

// Clean up the screen a little.
	LCD_Clear();
	LCD_DisplayScroll(False);

// First message.
	LCD_Print("Abe was here \x08\x09\0");



	/*

	*/
	while(1) {
	// Wait for a button press
		while(STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET){
			buttonPressed = 1;
		}
		if(buttonPressed == 1){
			buttonPressed = 0;
			break;
		}
	}

	LCD_Home();
	//LCD_Clear();
	//LCD_MoveCursor(3);
	//LCD_MoveDisplay(3);
	//LCD_DisplayOn(False);
	//LCD_DisplayScroll(False);
	//LCD_CursorOn(True);
	//LCD_EntryIncrement(False);
	//LCD_CursorBlink(True);
	LCD_MoveToPosition(0x40);

	LCD_Print("I mean here\0");

	while(1) {
	// Wait for a button press
		while(STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET){
			buttonPressed = 1;
		}
		if(buttonPressed == 1){
			buttonPressed = 0;
			break;
		}
	}

	LCD_Clear();
	LCD_DisplayScroll(False);
	LCD_Print("Abe is the man!!\0");


	while(1) {

	}




}


