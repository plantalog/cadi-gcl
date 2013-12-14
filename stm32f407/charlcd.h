/* 
 * See pinout.txt in the root directory of this project for the
 * LCD pinout.
 */

#ifndef __CHARLCD_H__
#define __CHARLCD_H__

// Device Defines (set these for your device!)
#define Num_Lines 4
#define Num_Characters 20  // Per line

#define Set_RS GPIO_SetBits(GPIOD,GPIO_Pin_7)
#define Clr_RS GPIO_ResetBits(GPIOD,GPIO_Pin_7)

#define Set_RW GPIO_SetBits(GPIOD,GPIO_Pin_8)
#define Clr_RW GPIO_ResetBits(GPIOD,GPIO_Pin_8)

#define Set_Clk GPIO_SetBits(GPIOD,GPIO_Pin_9)
#define Clr_Clk GPIO_ResetBits(GPIOD,GPIO_Pin_9)

// May use pwm and a transistor on the 5V rail in the final version
#define Backlight_On GPIO_SetBits(GPIOD,GPIO_Pin_10)
#define Backlight_Off GPIO_ResetBits(GPIOD,GPIO_Pin_10)

extern u8 CharLCD_line,CharLCD_column;

typedef struct {
	u8 number;
	u8 line[8];
} CustomCharacter;

void CharLCD_Config(void);
void CharLCD_Init(void);
void CharLCD_WriteLineWrap(const char* string);
void CharLCD_WriteLineNoWrap(const char* string);
void CharLCD_WriteString(const char* line);
void CharLCD_SendCustom(CustomCharacter *character);
void CharLCD_WriteCustom(CustomCharacter *character);
void CharLCD_SetCursor(u8 line,u8 column);
void CharLCD_Clear(void);
void CharLCD_Delay(int Count);
void CharLCD_Test(void);
void CharLCD_WriteData(u8 data);
void CharLCD_Backlight(u8 status);
void CharLCD_IncrementCursorVariables(void);

#endif // endif for __CHARLCD_H__

