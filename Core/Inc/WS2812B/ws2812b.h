/*
 * ws1812b.h
 *
 *  Created on: Jan 27, 2023
 *      Author: odemki
 */

#ifndef INC_WS2812B_WS2812B_H_
#define INC_WS2812B_WS2812B_H_

#include "stdbool.h"

void Set_LED (int LEDnum, int Red, int Green, int Blue);
void cleanAllScreen(void);
void cleanAllScreenBuffer(void);
void simple_animation(uint8_t r, uint8_t g, uint8_t b, uint8_t dalay);
void whiteAll(void);
void print_test_digits(uint8_t startRow, uint8_t simbol, uint8_t red, uint8_t green, uint8_t blue);
void printRuningNumbersAndLetters(int delay);
void printString(char* str, uint8_t red, uint8_t green, uint8_t blue);

void WS2812_Send(void);
void printTime(uint8_t minutes, uint8_t hours, uint8_t red, uint8_t green, uint8_t blue);

void setStringOnScreen(char *simbolstr, uint8_t string_size, uint8_t red, uint8_t green, uint8_t blue);
void setStringOnScreenWithShift(uint8_t startPosition, char *simbolstr, uint8_t string_size, uint8_t red, uint8_t green, uint8_t blue);

void textAnimation(void);
void colorAnimation(void);


#endif /* INC_WS2812B_WS2812B_H_ */
