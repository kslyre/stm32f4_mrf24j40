#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"
#include "defines.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_disco.h"
#include "tm_stm32f4_ili9341.h"
#include "tm_stm32f4_stmpe811.h"
#include "tm_stm32f4_fonts.h"
#include "mrf_lib.h"
#include <stdio.h>
#include "buttons.h"

int main(void) {
  TM_STMPE811_TouchData touchData;
	uint8_t *d;
	char data[127];
	mrf_rx_info_t rx_info;
	int i;
	
  SystemInit();
	    
  //Initialize LCD
  TM_ILI9341_Init();
  TM_ILI9341_Fill(ILI9341_COLOR_ORANGE);
  TM_ILI9341_Rotate(TM_ILI9341_Orientation_Landscape_2);
  
  //Initialize Touch
  if (TM_STMPE811_Init() != TM_STMPE811_State_Ok) {
      TM_ILI9341_Puts(20, 20, "STMPE811 Error", &TM_Font_11x18, ILI9341_COLOR_ORANGE, ILI9341_COLOR_BLACK);      
      while (1);
  }
  touchData.orientation = TM_STMPE811_Orientation_Landscape_2;
    
	mrf_start();
		
  TM_ILI9341_Puts(5, 5, "Packet", &TM_Font_11x18, ILI9341_COLOR_ORANGE, ILI9341_COLOR_BLACK);
  TM_ILI9341_Puts(220, 0, "ZigBee Sniffer", &TM_Font_7x10, ILI9341_COLOR_GREEN, ILI9341_COLOR_BLACK);
		
	TM_ILI9341_Puts(10, 30, strcat("Data: ", ""), &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_ORANGE);
    
	TM_DELAY_Init();
	TM_DISCO_LedInit();
  TM_DISCO_LedOn(LED_ALL);
	Delayms(100);
	TM_DISCO_LedOff(LED_ALL);
	
	drawButton(0,   200, "<", touchData, 0);
	drawButton(160, 200, ">", touchData, 0);
	drawButton(240, 200, "Scan", touchData, 0);
	drawButton(250, 20, "chan", touchData, 0);
	
		
  while (1) {
		if (get_rx_flag())
		{
			d = get_rx_buf();
			for(i = 0; i < 127; i++)
			{
				data[i] = (char) *(d + i);
			}
			
			rx_info = get_rx_info();
			TM_ILI9341_Puts(10, 90, strcat("Data: ", data), &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_ORANGE);
			
			TM_DISCO_LedOn(LED_ALL);
			Delayms(100);
			TM_DISCO_LedOff(LED_ALL);
			set_rx_flag();
		}
    
			//int state = (TM_STMPE811_ReadTouch(&touchData) == TM_STMPE811_State_Pressed);
        /*if (TM_STMPE811_ReadTouch(&touchData) == TM_STMPE811_State_Pressed) {
            //Touch valid
            sprintf(str, "Pressed    \n\nX: %03d\nY: %03d", touchData.x, touchData.y);
            TM_ILI9341_Puts(20, 80, str, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_ORANGE);
 
            //TM_ILI9341_DrawPixel(touchData.x, touchData.y, 0x0000);
        } else {
            sprintf(str, "Not Pressed\n\n       \n      ");
            TM_ILI9341_Puts(20, 80, str, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_ORANGE);
        }*/
				//drawButton(0,   200, "<", touchData, state);
				//drawButton(160, 200, ">", touchData, state);
				//drawButton(240, 200, "Scan", touchData, state);
    }
	
	// blinks
	/*TM_DELAY_Init();
	TM_DISCO_LedInit();
	TM_DISCO_LedOn(LED_ALL);
	Delayms(2000);
	
	while (1) {
		TM_DISCO_LedToggle(LED_ALL);
		Delayms(500);
	}*/
}
