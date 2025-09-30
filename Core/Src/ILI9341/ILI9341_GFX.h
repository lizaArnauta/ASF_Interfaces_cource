#ifndef ILI9341_GFX_H
#define ILI9341_GFX_H

#include "stm32f4xx_hal.h"

void ILI9341_Draw_Char(char Character, uint16_t X, uint16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour);
void ILI9341_Draw_Text(const char* Text, uint16_t X, uint16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour);
#endif
