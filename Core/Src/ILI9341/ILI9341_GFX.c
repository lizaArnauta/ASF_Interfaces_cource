#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
#include "stm32f4xx_hal_spi.h"

#include "6x8_font.h"

void ILI9341_Draw_Char(char Character, uint16_t X, uint16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour)
{
		uint8_t 	function_char;
    uint8_t 	i,j;
		
		function_char = Character;
		
    if (function_char < ' ') {
        Character = 0;
    } else {
        function_char -= 32;
		}
   	
		char temp[CHAR_WIDTH];
		for(uint8_t k = 0; k<CHAR_WIDTH; k++)
		{
		temp[k] = font[function_char][k];
		}
		
	ILI9341_Draw_Rectangle(X, Y, CHAR_WIDTH*Size, CHAR_HEIGHT*Size, Background_Colour);
    for (j=0; j<CHAR_WIDTH; j++)
    {
        for (i=0; i<CHAR_HEIGHT; i++)
        {
            if (temp[j] & (1<<i))
            {
            	if(Size == 1)
				{
            		ILI9341_Draw_Pixel(X+j, Y+i, Colour);
				}
				else
				{
					ILI9341_Draw_Rectangle(X+(j*Size), Y+(i*Size), Size, Size, Colour);
				}
            }						
        }
    }
}

void ILI9341_Draw_Text(const char* Text, uint16_t X, uint16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour)
{
    while (*Text)
    {
        ILI9341_Draw_Char(*Text++, X, Y, Colour, Size, Background_Colour);
        X += CHAR_WIDTH*Size;
    }
}


