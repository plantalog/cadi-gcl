//******************************************************************************
//  HD44780 LCD Driver			Date: October 24, 2012
//	by Abe Barker				abebarker@hotmail.com
//******************************************************************************
//
//		User functions:
//
//		LCD_ConfigurePort(Port,		[GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH or GPIOI]
//							RS,		[GPIO_Pin_0 to GPIO_Pin_15]
//							RW,		[GPIO_Pin_0 to GPIO_Pin_15]
//							E,		[GPIO_Pin_0 to GPIO_Pin_15]
//							DB0,	[GPIO_Pin_0 to GPIO_Pin_15 or NULL (0)]
//							DB1,	[GPIO_Pin_0 to GPIO_Pin_15 or NULL (0)]
//							DB2,	[GPIO_Pin_0 to GPIO_Pin_15 or NULL (0)]
//							DB3,	[GPIO_Pin_0 to GPIO_Pin_15 or NULL (0)]
//							DB4,	[GPIO_Pin_0 to GPIO_Pin_15]
//							DB5, 	[GPIO_Pin_0 to GPIO_Pin_15]
//							DB6,	[GPIO_Pin_0 to GPIO_Pin_15]
//							DB7		[GPIO_Pin_0 to GPIO_Pin_15]
//					);
//		 LCD_Initalize(BusWidth, 	[BUS_WIDTH_4, BUS_WIDTH_8]
//					DisplayLines, 	[DISPLAY_LINES_1, DISPLAY_LINES_2]
//					Font	 		[FONT_5x8, FONT_5x10]
//					);
//		LCD_Home()		// Move cursor to home position.
//		LCD_Clear()		// Clear the entire display
//		LCD_Print(char* str)	// Print a null terminated string.	i.e. "String\0"
//		LCD_MoveCursor(int value)	// Move cursor +/- places from its current location
//		LCD_MoveDisplay(int value)	// Shift the display +/- places (+ = Right) (- = Left)
//		LCD_DisplayOn(Bool value)	// Turn screen on or off. (Could be used to flash message)
//		LCD_DisplayScroll(Bool value)	// False = Cursor moves, True = Display moves (Probably better to have cursor at end of screen or have the entry decrement)
//		LCD_EntryIncrement(Bool value)	// True = Characters entered left to right, False = Characters entered right to left. (Probably better to have cursor at right side of screen when doing entry if False)
//		LCD_CursorOn(Bool value)		// Underscore at cursor position
//		LCD_CursorBlink(Bool value)		// Character block blinks black, white
//		LCD_MoveToPosition(int value)	// Move the cursor to any position (for 2 line display second line starts at 0x40)
//		LCD_CustChar(uint8_t array[], int charNum)	// Custom characters are passed in an array of bytes (array[8])
//
//
//
//
//	   Two things must be done before using this utility:
//		ConfigurePort
//		InitalizeLCD
//
//	Example:
//		LCD_ConfigurePort(GPIOE,
//				GPIO_Pin_6,	GPIO_Pin_4,	GPIO_Pin_5,
//				NULL, NULL,	NULL, NULL,
//				GPIO_Pin_2,	GPIO_Pin_3, GPIO_Pin_0,	GPIO_Pin_1
//				);
//		LCD_Initalize(BUS_WIDTH_4, DISPLAY_LINES_2, FONT_5x8);
//
//
//
//
//
//		Custom Character array example
//
//			uint8_t CustChar1[8] = {x00100,
//			  						x01110,
//			  						x10101,
//			  						x00100,
//			  						x00100,
//			  						x00100,
//			  						x00100,
//			  						x00100};
//
//
//			LCD_CustChar(CustChar1, 0);
//			LCD_CustChar(CustChar1, 1);
//
//			LCD_Print("Some text. \x08\1\ \0");
//
//			* Note: The custom character in first RAM address must be accessed with escape
//					sequence "\x08" since "\0" is used to mark the end of text.
//					"\1" to "\7" form of escape sequence is understood by compiler but not "\8"
//					"\x08" to "\x0F" repeat the RAM addresses 0 through 7
//
//
//
//
//
//			* Additional Note: The Interrupt Handler is located in the
//				"stm32f4xx_it.c" file. It looks like the following code.
//
//				void TIM2_IRQHandler(void) {
//
//					LCD_ClockTick();
//					TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//				}
//
//
//
//******************************************************************************

#include "HD44780.h"
#include "stm32f4xx.h"

// Create a stack for commands
struct TASK Task_Queue[QUEUE_SIZE];
uint16_t TopOfQueue, BottomOfQueue;

// Internal variables for giving commands
uint16_t HD44780_Entry_Inc_Dec;
uint16_t HD44780_Display_Shift;
uint16_t HD44780_Display_On_Off;
uint16_t HD44780_Cursor_On_Off;
uint16_t HD44780_Cursor_Blink;
uint16_t HD44780_Bus_Width;
uint16_t HD44780_Display_Lines;
uint16_t HD44780_Font_Size;

// Port settings
GPIO_TypeDef* LCD_Port;
uint32_t RCC_AHB1Periph_GPIOx;
uint32_t LCD_RS;
uint32_t LCD_RW;
uint32_t LCD_E;
uint32_t LCD_DB0;
uint32_t LCD_DB1;
uint32_t LCD_DB2;
uint32_t LCD_DB3;
uint32_t LCD_DB4;
uint32_t LCD_DB5;
uint32_t LCD_DB6;
uint32_t LCD_DB7;
GPIO_InitTypeDef GPIO_InitStruct;

uint16_t LCD_Data;


////////////////////// Functions ///////////////////////////

void LCD_ConfigurePort(GPIO_TypeDef* Port,
						uint16_t RS,
						uint16_t RW,
						uint16_t E,
						uint16_t DB0,
						uint16_t DB1,
						uint16_t DB2,
						uint16_t DB3,
						uint16_t DB4,
						uint16_t DB5,
						uint16_t DB6,
						uint16_t DB7
						){
unsigned int tempPort, temp2, temp3, RotateVal;
	// We need to change the port from GPIOx (Address space port address)
	// to a RCC_AHB1Periph_GPIOx (Register based port identifier) form
	// to configure the clock.
	tempPort = (unsigned int)Port;		// Get the address of the port

	temp2 = tempPort	& 0x3FFF;		// Strip off the unneeded address part
									// and save the part that identifies the port.
	//temp4 = temp2;
	tempPort = (unsigned int)GPIOB;	// We need a none zero value to divide by so we
	temp3 = tempPort & 0x3FFF;		// use second port value and strip the address part.
	RotateVal = temp2 / temp3;		// Then see how many times it goes into our unknown port value.
	tempPort = 1;						// Start a new port identifier with the lowest value.
	tempPort = tempPort << RotateVal;	// Rotate it into the correct register bit value to obtain our identifier.


	LCD_Port = Port;
	RCC_AHB1Periph_GPIOx = tempPort;
	LCD_RS = RS;
	LCD_RW = RW;
	LCD_E = E;
	LCD_DB0 = DB0;
	LCD_DB1 = DB1;
	LCD_DB2 = DB2;
	LCD_DB3 = DB3;
	LCD_DB4 = DB4;
	LCD_DB5 = DB5;
	LCD_DB6 = DB6;
	LCD_DB7 = DB7;

	return;
}




void LCD_Initalize(int BusWidth, int DisplayLines, int FontSize){
struct TASK Task;
	TopOfQueue = 0;
	BottomOfQueue = 0;

	// Set Default values
	HD44780_Entry_Inc_Dec = ENTRY_INCREMENT;
	HD44780_Display_Shift = DISPLAY_SHIFT_OFF;
	HD44780_Display_On_Off = DISPLAY_ON;
	HD44780_Cursor_On_Off = CURSOR_ON;
	HD44780_Cursor_Blink = CURSOR_BLINK_ON;
	HD44780_Bus_Width = BusWidth;
	HD44780_Display_Lines = DisplayLines;
	HD44780_Font_Size = FontSize;


	LCD_InitalizeRCC();
	LCD_InitalizeNVIC();
	LCD_TIM_Config();

	LCD_HardInitalize();


	// Turn the display on to be nice.
	LCD_WaitNotBusy();	// Wait until we are not busy first

	Task.Command = WriteData;
	Task.Data = DisplayControl | HD44780_Display_On_Off |  HD44780_Cursor_On_Off | HD44780_Cursor_Blink;
	Task.Iter = 0;
	LCD_PushTask(Task);

	return;
}









void LCD_Home(){	// Move cursor to home position.
struct TASK Task;
	//LCD_WaitNotBusy();	// Wait until we are not busy first

	Task.Command = WriteData;
	Task.Data = ReturnHome;
	Task.Iter = 0;
	LCD_PushTask(Task);

return;
}

void LCD_Clear(){	// Clear the entire display
struct TASK Task;
	//LCD_WaitNotBusy();	// Wait until we are not busy first

	Task.Command = WriteData;
	Task.Data = ClearDisplay;
	Task.Iter = 0;
	LCD_PushTask(Task);
return;
}

void LCD_Print(char* str){	// Print a null terminated string.
struct TASK Task;
int strPtr;
	strPtr = 0;

	//LCD_WaitNotBusy();	// Wait until we are not busy first
	while(*(str + strPtr) != 0){

		Task.Command = WriteData;
		Task.Data = WriteData | *(str + strPtr);
		Task.Iter = 0;
		LCD_PushTask(Task);
		strPtr++;

	}
	return;
}

void LCD_MoveCursor(int value){
struct TASK Task;
	if(value == 0){
		return;		// No movement needed.
	}
	//LCD_WaitNotBusy();	// Wait until we are not busy first
	if(value > 0){
		while(value !=0){
			Task.Command = WriteData;
			Task.Data = CursorDisplayShift | SHIFT_CURSOR | SHIFT_DIRECTION_RIGHT;
			Task.Iter = 0;
			LCD_PushTask(Task);
			value--;
		}
	}
	else{
		while(value !=0){
			Task.Command = WriteData;
			Task.Data = CursorDisplayShift | SHIFT_CURSOR | SHIFT_DIRECTION_LEFT;
			Task.Iter = 0;
			LCD_PushTask(Task);
			value++;
		}
	}
	return;
}

void LCD_MoveDisplay(int value){	// Shift the entire display number of places +/-
struct TASK Task;
	if(value == 0){
		return;		// No movement needed.
	}
	//LCD_WaitNotBusy();	// Wait until we are not busy first
	if(value > 0){
		while(value !=0){
			Task.Command = WriteData;
			Task.Data = CursorDisplayShift | SHIFT_DISPLAY | SHIFT_DIRECTION_RIGHT;
			Task.Iter = 0;
			LCD_PushTask(Task);
			value--;
		}
	}
	else{
		while(value !=0){
			Task.Command = WriteData;
			Task.Data = CursorDisplayShift | SHIFT_DISPLAY | SHIFT_DIRECTION_LEFT;
			Task.Iter = 0;
			LCD_PushTask(Task);
			value++;
		}
	}
	return;
}
void LCD_DisplayOn(Bool value){	// Turn display on or off.
struct TASK Task;
	//LCD_WaitNotBusy();	// Wait until we are not busy first

	if(value){
		HD44780_Display_On_Off = DISPLAY_ON;
	}
	else{
		HD44780_Display_On_Off = DISPLAY_OFF;
	}
	Task.Command = WriteData;
	Task.Data = DisplayControl | HD44780_Display_On_Off |  HD44780_Cursor_On_Off | HD44780_Cursor_Blink;
	Task.Iter = 0;
	LCD_PushTask(Task);
	return;
}

void LCD_DisplayScroll(Bool value){	// Scroll the display on character entry.
struct TASK Task;
	//LCD_WaitNotBusy();	// Wait until we are not busy first

	if(value){
		HD44780_Display_Shift = DISPLAY_SHIFT_ON;
		}
	else{
		HD44780_Display_Shift = DISPLAY_SHIFT_OFF;
	}
	Task.Command = WriteData;
	Task.Data = EntryMode | HD44780_Entry_Inc_Dec |  HD44780_Display_Shift;
	Task.Iter = 0;
	LCD_PushTask(Task);
	return;
}

void LCD_EntryIncrement(Bool value){
struct TASK Task;
	//LCD_WaitNotBusy();	// Wait until we are not busy first

	if(value){
		HD44780_Entry_Inc_Dec = ENTRY_INCREMENT;
		}
	else{
		HD44780_Entry_Inc_Dec = ENTRY_DECREMENT;
	}
	Task.Command = WriteData;
	Task.Data = EntryMode | HD44780_Entry_Inc_Dec |  HD44780_Display_Shift;
	Task.Iter = 0;
	LCD_PushTask(Task);
	return;
}

void LCD_CursorOn(Bool value){
struct TASK Task;
	//LCD_WaitNotBusy();	// Wait until we are not busy first

	if(value){
		HD44780_Cursor_On_Off = CURSOR_ON;
		}
	else{
		HD44780_Cursor_On_Off = CURSOR_OFF;
	}
	Task.Command = WriteData;
	Task.Data = DisplayControl | HD44780_Display_On_Off |  HD44780_Cursor_On_Off | HD44780_Cursor_Blink;
	Task.Iter = 0;
	LCD_PushTask(Task);
	return;
}

void LCD_CursorBlink(Bool value){
struct TASK Task;
	//LCD_WaitNotBusy();	// Wait until we are not busy first

	if(value){
		HD44780_Cursor_Blink = CURSOR_BLINK_ON;
		}
	else{
		HD44780_Cursor_Blink = CURSOR_BLINK_OFF;
	}
	Task.Command = WriteData;
	Task.Data = DisplayControl | HD44780_Display_On_Off |  HD44780_Cursor_On_Off | HD44780_Cursor_Blink;
	Task.Iter = 0;
	LCD_PushTask(Task);
	return;
}

void LCD_MoveToPosition(int value){
struct TASK Task;
	//LCD_WaitNotBusy();	// Wait until we are not busy first

	Task.Command = WriteData;
	Task.Data = SetDDRamAddress | value;
	Task.Iter = 0;
	LCD_PushTask(Task);

	return;
}


void LCD_CustChar(uint8_t array[], int charNum){
// An array of 8 bytes, and the position in CGRAM to put the new custom character.
struct TASK Task;
int Iter;

	//LCD_WaitNotBusy();

	Task.Command = WriteData;   	// Set the CGRam address
	Task.Data = SetCGRamAddress | (charNum * 8);
	Task.Iter = 0;
	LCD_PushTask(Task);
	//LCD_WaitNotBusy();

	for(Iter = 0; Iter <= 7; Iter++){
		//LCD_WaitNotBusy();

		// Write the data to CGRam
		Task.Command = WriteData;
		Task.Data = WriteData | (array[Iter] & 0x1F);
		Task.Iter = 0;
		LCD_PushTask(Task);


		Task.Command = WAIT;   	// Wait until we are not busy first
		Task.Data = 0x0002;
		Task.Iter = 0;
		LCD_PushTask(Task);
/*		*/
	}
	//LCD_WaitNotBusy();

	Task.Command = WAIT;   	// Wait until we are not busy first
	Task.Data = 0x0002;
	Task.Iter = 0;
	LCD_PushTask(Task);
/*	*/
	Task.Command = WriteData;		// Set the address pointer back to DDRAM address space.
	Task.Data = SetDDRamAddress | 0;
	Task.Iter = 0;
	LCD_PushTask(Task);



	return;
}



void LCD_WaitNotBusy(){
struct TASK Task;
	Task.Command = WaitBusy;   	// Wait until we are not busy first
	Task.Data = 0;
	Task.Iter = 0;
	LCD_PushTask(Task);
	return;
}




void LCD_ClockTick(){
//struct TASK Task;
struct TASK *Task;
uint16_t Temp;

// Pull a command off the stack

Task = LCD_ExaminTask();


// Do command
switch (Task->Command){
	case WAIT:
		if(Task->Data == 0){		// Are we waiting for something?
			LCD_DeleteTask();
			break;
		}
			Task->Data--;		// Decrement the wait time and
		break;


	case WriteData:
		if(Task->Iter == 3){
			GPIO_ResetBits(LCD_Port, LCD_E); 	// Toggle the enable bit off
			LCD_DeleteTask();					// We are done with this task.
			break;
		}
		if(Task->Iter == 2){
			// Output the lower nibble of data
			LCD_Port_Set_Output();
			GPIO_WriteBit(LCD_Port, LCD_RS, (BitAction)((Task->Data & 0x0200)>>8));
			GPIO_WriteBit(LCD_Port, LCD_RW, (BitAction)((Task->Data & 0x0100)>>8));
			GPIO_WriteBit(LCD_Port, LCD_DB7, (BitAction)(Task->Data & 0x0008));
			GPIO_WriteBit(LCD_Port, LCD_DB6, (BitAction)(Task->Data & 0x0004));
			GPIO_WriteBit(LCD_Port, LCD_DB5, (BitAction)(Task->Data & 0x0002));
			GPIO_WriteBit(LCD_Port, LCD_DB4, (BitAction)(Task->Data & 0x0001));

			GPIO_SetBits(LCD_Port, LCD_E);		// Toggle Enable pin on
			Task->Iter ++;		// Increase the iteration value so we know we have been though once already
			break;
		}
		if(Task->Iter == 1){
			GPIO_ResetBits(LCD_Port, LCD_E); 	// Toggle the enable bit off
			if(HD44780_Bus_Width == BUS_WIDTH_8){
				LCD_DeleteTask();
				break;
			}
			Task->Iter++;
			break;
		}
		if(Task->Iter == 0){
			LCD_Port_Set_Output();
			GPIO_WriteBit(LCD_Port, LCD_RS, (BitAction)((Task->Data & 0x0200)>>8));
			GPIO_WriteBit(LCD_Port, LCD_RW, (BitAction)((Task->Data & 0x0100)>>8));
			GPIO_WriteBit(LCD_Port, LCD_DB7, (BitAction)(Task->Data & 0x0080));
			GPIO_WriteBit(LCD_Port, LCD_DB6, (BitAction)(Task->Data & 0x0040));
			GPIO_WriteBit(LCD_Port, LCD_DB5, (BitAction)(Task->Data & 0x0020));
			GPIO_WriteBit(LCD_Port, LCD_DB4, (BitAction)(Task->Data & 0x0010));

			// If it's the full 8-bit port write the lower nibble also.
			if(HD44780_Bus_Width == BUS_WIDTH_8){
				GPIO_WriteBit(LCD_Port, LCD_DB3, (BitAction)(Task->Data & 0x0008));
				GPIO_WriteBit(LCD_Port, LCD_DB2, (BitAction)(Task->Data & 0x0004));
				GPIO_WriteBit(LCD_Port, LCD_DB1, (BitAction)(Task->Data & 0x0002));
				GPIO_WriteBit(LCD_Port, LCD_DB0, (BitAction)(Task->Data & 0x0001));
			}
			GPIO_SetBits(LCD_Port, LCD_E);		// Toggle Enable pin on
			Task->Iter ++;		// Increase the iteration value so we know we have been though once already
			break;
		}
		break;
	case WaitBusy:
		switch(Task->Iter){
			case 0:
				// Output the command to send data
				LCD_Port_Set_Input();
				GPIO_ResetBits(LCD_Port, LCD_RS | LCD_DB7 | LCD_DB6 | LCD_DB5 | LCD_DB4);
				GPIO_SetBits(LCD_Port, LCD_RW);
				GPIO_SetBits(LCD_Port, LCD_E);		// Toggle Enable pin on
				Task->Iter ++;	// Increase the iteration value so we know we have been though once already
				break;
			case 1:
				// Read the data, bit by bit and shift it into our data byte.
				LCD_Data = 0;
				//Temp = 0;
				Temp = GPIO_ReadInputDataBit(LCD_Port, LCD_DB7);
				Temp = Temp << 1;
				Temp = Temp | GPIO_ReadInputDataBit(LCD_Port, LCD_DB6);
				Temp = Temp << 1;
				Temp = Temp | GPIO_ReadInputDataBit(LCD_Port, LCD_DB5);
				Temp = Temp << 1;
				Temp = Temp | GPIO_ReadInputDataBit(LCD_Port, LCD_DB4);
				Temp = Temp << 1;
				if(HD44780_Bus_Width == BUS_WIDTH_8){
					Temp = Temp | GPIO_ReadInputDataBit(LCD_Port, LCD_DB3);
					Temp = Temp << 1;
					Temp = Temp | GPIO_ReadInputDataBit(LCD_Port, LCD_DB2);
					Temp = Temp << 1;
					Temp = Temp | GPIO_ReadInputDataBit(LCD_Port, LCD_DB1);
					Temp = Temp << 1;
					Temp = Temp | GPIO_ReadInputDataBit(LCD_Port, LCD_DB0);
					Temp = Temp << 1;
					Task->Data = Temp;	// All data collected
					GPIO_ResetBits(LCD_Port, LCD_E); 	// Toggle the enable bit off
					if(Task->Data & 0xF0){	// Is the busy flag set?
						Task->Data = 0;
						Task->Iter = 0;	// Reset the task and try again.
						break;
					}
					LCD_DeleteTask();		// We are done with task
					break;
				}
				Task->Data = Temp << 4;	// Move data to more permanent place.
				Task->Iter ++;	// Increase the iteration value so we know we have been though once already
				GPIO_ResetBits(LCD_Port, LCD_E); 	// Toggle the enable bit off
				break;
			case 2:
				// Still need other half of data.
				Task->Iter ++;	// Increase the iteration value so we know we have been though once already
				GPIO_SetBits(LCD_Port, LCD_E);		// Toggle Enable pin on
				break;
			case 3:
				// Read lower half of data.
				Temp = GPIO_ReadInputDataBit(LCD_Port, LCD_DB3);
				Temp = Temp << 1;
				Temp = Temp | GPIO_ReadInputDataBit(LCD_Port, LCD_DB2);
				Temp = Temp << 1;
				Temp = Temp | GPIO_ReadInputDataBit(LCD_Port, LCD_DB1);
				Temp = Temp << 1;
				Temp = Temp | GPIO_ReadInputDataBit(LCD_Port, LCD_DB0);
				Temp = Temp << 1;
				Task->Data = Task->Data | Temp;	// Combine data collected with upper half of data
				GPIO_ResetBits(LCD_Port, LCD_E); 	// Toggle the enable bit off
				if(Task->Data & 0xF0){	// Is the busy flag set?
					Task->Data = 0;
					Task->Iter = 0;	// Reset the task and try again.
					break;
				}
				LCD_DeleteTask();		// We are done with task
				break;
			}
		break;
	case ReadData:
		break;
	case NO_TASK:
		LCD_DeleteTask();
		break;
	}
	return;
}

uint16_t LCD_ConvertDataToPortData(uint16_t Data){
uint16_t iter, PortData = 0;
uint16_t PortBits[8];

	PortBits[7] = LCD_DB7; PortBits[6] = LCD_DB6;
	PortBits[5] = LCD_DB5; PortBits[4] = LCD_DB4;
	PortBits[3] = LCD_DB3; PortBits[2] = LCD_DB2;
	PortBits[1] = LCD_DB1; PortBits[0] = LCD_DB0;

	for(iter = 0; iter <= 7; iter++){
		if(Data & 0x01)	// If this bit is set
			PortData = PortData | PortBits[iter];	// set our portData at the location of that port bit
		Data = Data >> 1;	// shift the actual data through the testing bit (bit 0).
	}
	return PortData;
}

uint16_t LCD_ConvertPortDataToData(uint16_t PortData){
uint16_t iter, OutputData = 0;
uint16_t PortBits[8];

	PortBits[7] = LCD_DB7; PortBits[6] = LCD_DB6;
	PortBits[5] = LCD_DB5; PortBits[4] = LCD_DB4;
	PortBits[3] = LCD_DB3; PortBits[2] = LCD_DB2;
	PortBits[1] = LCD_DB1; PortBits[0] = LCD_DB0;

	for(iter = 0; iter <= 7; iter++){
		if(PortData & PortBits[iter])	// If this bit is set
			OutputData ++;				// add one and
		OutputData = OutputData << 1;	// shift it into our actual data.
	}
	return OutputData;
}

void LCD_Port_Set_Input(){

	GPIO_InitStruct.GPIO_Pin = LCD_DB0 | LCD_DB1 | LCD_DB2 |
								LCD_DB3 | LCD_DB4 | LCD_DB5	|
								LCD_DB6 | LCD_DB7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(LCD_Port, &GPIO_InitStruct);
  return;
}


void LCD_Port_Set_Output(){
	GPIO_InitStruct.GPIO_Pin = LCD_RS | LCD_RW	| LCD_E	|
								LCD_DB0 | LCD_DB1 |
								LCD_DB2 | LCD_DB3 |
								LCD_DB4 | LCD_DB5	|
								LCD_DB6 | LCD_DB7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

  	GPIO_Init(LCD_Port, &GPIO_InitStruct);
	return;
}







struct TASK* LCD_ExaminTask(){
	return &Task_Queue[BottomOfQueue];
}

void LCD_DeleteTask(){

	if (BottomOfQueue == TopOfQueue){	// Is Queue empty?
		Task_Queue[BottomOfQueue].Command = NO_TASK;	// Indicate that this is not a real task.
		Task_Queue[BottomOfQueue].Data = 0;
		Task_Queue[BottomOfQueue].Iter = 0;
		return;}
	Task_Queue[BottomOfQueue].Command = NO_TASK;	// Indicate that this is not a real task.
	Task_Queue[BottomOfQueue].Data = 0;
	Task_Queue[BottomOfQueue].Iter = 0;
	BottomOfQueue++;
	if (BottomOfQueue >= QUEUE_SIZE){
		BottomOfQueue = 0;}		// Roll over the top of array back to bottom.
	return;
}

void LCD_PushTask(struct TASK Task){
	TopOfQueue++;
	if(TopOfQueue >= QUEUE_SIZE){
		TopOfQueue = 0;}		// Roll over the top of array back to bottom.
	Task_Queue[TopOfQueue].Command = Task.Command;
	Task_Queue[TopOfQueue].Data = Task.Data;
	Task_Queue[TopOfQueue].Iter = Task.Iter;
	return;
}










void LCD_InitalizeRCC(){
//////////////// RCC Initialize /////////////////////
  /* PCLK1 = HCLK */
  //RCC_PCLK1Config(RCC_HCLK_Div1);
  /* PCLK2 = HCLK */
  //RCC_PCLK2Config(RCC_HCLK_Div1);
  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  /* GPIO clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOx, ENABLE);
  return;
}

void LCD_InitalizeNVIC(){
/////////////// Variables /////////////////
  NVIC_InitTypeDef NVIC_InitStructure;
//////////////// NVIC Initialize ////////////////////
  /* Set vector table */
#ifdef FLASH_BOOT
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x00);
#else
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x00);
#endif
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  /* Enable TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  return;
}

void LCD_TIM_Config()
{
  TIM_TimeBaseInitTypeDef TIM_InitStructure;
  /* TIM configuration */
  //TIM_InitStructure.TIM_Period = SystemCoreClock/LCD_PROCESS_IO_FREQ;
  TIM_InitStructure.TIM_Period = SystemCoreClock/LCD_PROCESS_IO_FREQ;
  TIM_InitStructure.TIM_Prescaler = 0;
  TIM_InitStructure.TIM_ClockDivision = 0;
  TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_InitStructure);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
  return;
}



void LCD_HardInitalize(){
struct TASK Task;
/*
	Task.Command = WAIT;   	// Wait until we are not busy first
	Task.Data = 0x0FFF;
	Task.Iter = 0;
	LCD_PushTask(Task);
*/

	Task.Command = WAIT;   	// Wait until we are not busy first
	Task.Data = 0x0190;
	Task.Iter = 0;
	LCD_PushTask(Task);

	Task.Command = WriteData;
	Task.Data = 0x03;
	Task.Iter = 2;
	LCD_PushTask(Task);


	Task.Command = WAIT;   	// Wait until we are not busy first
	Task.Data = 0x0030;
	Task.Iter = 0;
	LCD_PushTask(Task);


	Task.Command = WriteData;
	Task.Data = 0x03;
	Task.Iter = 2;
	LCD_PushTask(Task);

	Task.Command = WAIT;   	// Wait until we are not busy first
	Task.Data = 0x000F;
	Task.Iter = 0;
	LCD_PushTask(Task);


	Task.Command = WriteData;
	Task.Data = 0x03;
	Task.Iter = 2;
	LCD_PushTask(Task);


	Task.Command = WAIT;   	// Wait until we are not busy first
	Task.Data = 0x000F;
	Task.Iter = 0;
	LCD_PushTask(Task);

//---------------------------------------

if(HD44780_Bus_Width == BUS_WIDTH_4){
	// Set the bus width
	//LCD_WaitNotBusy();	// Wait until we are not busy first
Task.Command = WriteData;
Task.Data = 0x0002;			// Write the lower nibble to set the bus width to 4-bit
Task.Iter = 2;				// Note the contents must be shifted over a nibble when
LCD_PushTask(Task);			// writing half a byte because only the lower nibble is written

}

	Task.Command = WAIT;   	// Wait until we are not busy first
	Task.Data = 0x000F;
	Task.Iter = 0;
	LCD_PushTask(Task);
Task.Command = WriteData;
Task.Data = FunctionSet | HD44780_Bus_Width | HD44780_Display_Lines | HD44780_Font_Size;
Task.Iter = 0;
LCD_PushTask(Task);

	Task.Command = WAIT;   	// Wait until we are not busy first
	Task.Data = 0x000F;
	Task.Iter = 0;
	LCD_PushTask(Task);
Task.Command = WriteData;
Task.Data = DisplayControl | DISPLAY_OFF;
Task.Iter = 0;
LCD_PushTask(Task);


	Task.Command = WAIT;   	// Wait until we are not busy first
	Task.Data = 0x000F;
	Task.Iter = 0;
	LCD_PushTask(Task);
Task.Command = WriteData;
Task.Data = ClearDisplay;
Task.Iter = 0;
LCD_PushTask(Task);


	Task.Command = WAIT;   	// Wait until we are not busy first
	Task.Data = 0x000F;
	Task.Iter = 0;
	LCD_PushTask(Task);
Task.Command = WriteData;
Task.Data = EntryMode | HD44780_Entry_Inc_Dec |  HD44780_Display_Shift;
Task.Iter = 0;
LCD_PushTask(Task);




	return;
}

