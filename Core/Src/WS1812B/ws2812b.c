/*
 * ws2812b.c
 *
 *  Created on: Jan 27, 2023
 *      Author: odemki
 */


#include "main.h"
#include "WS2812B/ws2812b.h"

#include "ctype.h"

//#define DELAY_LEN 48

#define COLUMNS 17
#define ROWS 5

#define MAX_LED 85
uint8_t BYTESINPATTERN = 15-1;

extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_tim2_ch2_ch4;

const bool digits_paterns[38][15] = {
		{1, 1, 1, 1, 1,			// 0
		1, 0, 0, 0, 1,
		1, 1, 1, 1, 1},

		{0, 0, 0, 0, 0,			// 1
		0, 1, 0, 0, 0,
		1, 1, 1, 1, 1},

		{1, 1, 1, 0, 1,			// 2
		1, 0, 1, 0, 1,
		1, 0, 1, 1, 1},

		{1, 0, 1, 0, 1,			// 3
		1, 0, 1, 0, 1,
		1, 1, 1, 1, 1},

		{0, 0, 1, 1, 1,			// 4
		0, 0, 1, 0, 0,
		1, 1, 1, 1, 1},

		{1, 0, 1, 1, 1,			// 5
		1, 0, 1, 0, 1,
		1, 1, 1, 0, 1},

		{1, 1, 1, 1, 1,			// 6
		1, 0, 1, 0, 1,
		1, 1, 1, 0, 1},

		{0, 0, 0, 0, 1,			// 7
		1, 0, 0, 0, 0,
		1, 1, 1, 1, 1},

		{1, 1, 1, 1, 1,			// 8
		1, 0, 1, 0, 1,
		1, 1, 1, 1, 1},

		{1, 0, 1, 1, 1,			// 9
		1, 0, 1, 0, 1,
		1, 1, 1, 1, 1},

		{0, 0, 0, 0, 0,			// :		10
		0, 1, 0, 1, 0,
		0, 0, 0, 0, 0},

		{0, 0, 0, 0, 0,			// spase		11
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0},

				{1, 1, 1, 1, 0,			// A			12
				1, 0, 1, 0, 0,
				1, 1, 1, 1, 0},

				{1, 1, 1, 1, 1,			// B		13
				1, 0, 1, 0, 1,
				1, 1, 0, 1, 1},

				{1, 1, 1, 1, 1,			// C		14
				1, 0, 0, 0, 1,
				1, 0, 0, 0, 1},

				{1, 1, 1, 1, 1,			// D		15
				1, 0, 0, 0, 1,
				0, 1, 1, 1, 0},

				{1, 1, 1, 1, 1,			// E		16
				1, 0, 1, 0, 1,
				1, 0, 1, 0, 1},

				{1, 1, 1, 1, 1,			// F		17
				1, 0, 1, 0, 0,
				0, 0, 0, 0, 1},

				{1, 1, 1, 1, 1,			// G		18
				1, 0, 0, 0, 1,
				1, 1, 0, 0, 1},

				{1, 1, 1, 1, 1,			// H		19
				0, 0, 1, 0, 0,
				1, 1, 1, 1, 1},

				{0, 0, 0, 0, 0,			// I		20
				1, 1, 1, 1, 1,
				0, 0, 0, 0, 0},

				{1, 1, 0, 0, 0,			// J		21
				0, 0, 0, 0, 1,
				1, 1, 1, 1, 1},

				{1, 1, 1, 1, 1,			// K		22
				0, 1, 0, 1, 0,
				1, 0, 0, 0, 1},

				{1, 1, 1, 1, 1,			// L		23
				0, 0, 0, 0, 1,
				1, 0, 0, 0, 0},

				{1, 1, 1, 1, 1,			// M		24
				0, 1, 0, 0, 0,
				1, 1, 1, 1, 1},

				{1, 1, 1, 1, 1,			// N		25
				0, 1, 1, 0, 0,
				1, 1, 1, 1, 1},

				{1, 1, 1, 1, 1,			// O		26
				1, 0, 0, 0, 1,
				1, 1, 1, 1, 1},

				{1, 1, 1, 1, 1,			// P		27
				1, 0, 1, 0, 0,
				0, 0, 1, 1, 1},

				{0, 1, 1, 1, 1,			// Q		28
				1, 0, 0, 1, 0,
				1, 1, 1, 1, 1},

				{1, 1, 1, 1, 1,			// R		29
				1, 0, 1, 0, 0,
				1, 1, 0, 1, 1},

				{1, 0, 1, 1, 1,			// S		30
				1, 0, 1, 0, 1,
				1, 1, 1, 0, 1},

				{0, 0, 0, 0, 1,			// T		31
				1, 1, 1, 1, 1,
				0, 0, 0, 0, 1},

				{1, 1, 1, 1, 1,			// U		32
				0, 0, 0, 0, 1,
				1, 1, 1, 1, 1},

				{0, 1, 1, 1, 1,			// V		33
				0, 0, 0, 0, 1,
				0, 1, 1, 1, 1},

				{1, 1, 1, 1, 1,			// W		34
				1, 1, 1, 0, 0,
				1, 1, 1, 1, 1},

				{1, 0, 0, 0, 1,			// X		35
				0, 1, 1, 1, 0,
				1, 0, 0, 0, 1},

				{0, 0, 1, 1, 1,			// Y		36
				0, 0, 0, 1, 1,
				0, 0, 1, 1, 1},

				{1, 1, 0, 0, 1,			// Z		37
				1, 0, 1, 0, 1,
				1, 0, 0, 1, 1}
};


#define DELAY 55
uint8_t LED_Data[MAX_LED][4];
uint16_t pwmData[(24*MAX_LED) + DELAY];


volatile int datasentflag = 0;

void setOnesimbol(uint8_t startRow, char simbolstr, uint8_t red, uint8_t green, uint8_t blue);
void setNumberOnScreen(uint8_t startRow, char *simbolstr, uint8_t string_size, uint8_t red, uint8_t green, uint8_t blue);
void setTimeString(char *simbolstr, uint8_t string_size, uint8_t red, uint8_t green, uint8_t blue);

static void fillInAllScreen(bool red, bool green, bool blue, uint8_t min_brightness, uint8_t max_brightness, int delay);
// void setStringOnScreen(uint8_t startRow, char *simbolstr, uint8_t string_size, uint8_t red, uint8_t green, uint8_t blue);

// -------------------------------------------------------------------------------------------
void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}
// -------------------------------------------------------------------------------------------
void WS2812_Send(void)
{
	//HAL_GPIO_WritePin(GPIOA, USER_OUT_Pin, GPIO_PIN_SET);

	uint32_t indx=0;
	uint32_t color;
	int i = 0;


	for (i=0; i<DELAY; i++)								// Make dalay
	{
		pwmData[indx] = 0;
		indx++;
	}

	for(int i= 0; i<MAX_LED; i++)													// go over all LEDs
	{
		color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));	// Save color data into one value

		for(int k=23; k>=0; k--)							// analysis every bites
		{
			if (color&(1<<k))								//	if was 1 make 1 signal
			{
				pwmData[indx] = 60;  						// 2/3 of 90
			}
			else											//	if was 0 make 0 signal
			{
				pwmData[indx] = 30; 						 // 1/3 of 90
			}
			indx++;
		}
	}

	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_4, (uint32_t *)pwmData, indx);
	while (!datasentflag){};
	datasentflag = 0;

}
// -------------------------------------------------------------------------------------------
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_4);
	datasentflag = 1;
}
// --------------------------------------------------------------------------------------------
void cleanAllScreenBuffer(void)
{
	for(int i = 0; i <= MAX_LED; i++)
	{
		Set_LED (i, 0, 0, 0);
	}
}
// --------------------------------------------------------------------------------------------
void cleanAllScreen(void)
{
	for(int i = 0; i <= MAX_LED; i++)
	{
		Set_LED (i, 0, 0, 0);
	}
	WS2812_Send();
}
// --------------------------------------------------------------------------------------------
void whiteAll(void)
{
	for(int i = 0; i <= MAX_LED; i++)
	{
		Set_LED (i, 255, 255, 255);
	}
	WS2812_Send();
}
// --------------------------------------------------------------------------------------------
void simple_animation(uint8_t r, uint8_t g, uint8_t b, uint8_t dalay)
{
	uint8_t i = 0;

	for(i = 0; i <= 84; i++)
	{
		Set_LED(i, r, g, b);
		WS2812_Send();
		osDelay(dalay);

		if(i >= 84)
		{
			for(i = 84; i > 0; i--)
			{
				Set_LED(i, 0, 0, 0);
				WS2812_Send();
				osDelay(dalay);
			}
		}
	}
}
//// ----------------------------------------------------------------------------------------------
void showStringOnRGB(void)
{
	char test_shar[6] = "1234";
	int delay = 100;
	int i = 0;
	for(i = 0; i <= 16; i++)
	{
		cleanAllScreen();
		setStringOnScreenWithShift(i, test_shar, sizeof(test_shar), 200, 0, 200);
		WS2812_Send();
		osDelay(delay);

		if(i >= 16)
		{
			for(i = 16; i > -16; i--)
			{
				cleanAllScreen();
				setStringOnScreenWithShift(i, test_shar, sizeof(test_shar), 200, 2000, 0);
				WS2812_Send();
				osDelay(delay);
			}
		}
	}
	WS2812_Send();
}

//// ----------------------------------------------------------------------------------------------

//// ----------------------------------------------------------------------------------------------
void print_test_digits(uint8_t startRow, uint8_t simbol, uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t PositionOnScreen = 0;
	uint8_t k = 0;

	// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);		//Подивитися на затримку, скільки часу готується массив для передачі

	cleanAllScreen();				//	якщо чистити весь екран, то не вийде виводити більше ніж одні цифру
	//тому потрібно спочатку готувати весь заповнений символами екран, а потім виводити все разом, при зміні хоча б одного символу повтороити

	startRow = startRow * ROWS;			// make one step(column) on screenЗ (one column = 5 LEDs)

	if((startRow == 5) || (startRow == 15) || (startRow == 25) || (startRow == 35) || (startRow == 45) || (startRow == 55) || (startRow == 65))  // Pair column ( LEDs startfrom top)
	{
		// Convert pattern
		uint8_t k, l = 0;
		bool t_buffer[15] = {0, };

		for(uint8_t i = 1; i <= 3; i++)
		{
			t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
			t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
			t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
			t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
			t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
			l = 0;
		}

//		t_buffer[4] = digits_paterns[simbol][0];
//		t_buffer[3] = digits_paterns[simbol][1];
//		t_buffer[2] = digits_paterns[simbol][2];
//		t_buffer[1] = digits_paterns[simbol][3];
//		t_buffer[0] = digits_paterns[simbol][4];
//
//		t_buffer[9] = digits_paterns[simbol][5];
//		t_buffer[8] = digits_paterns[simbol][6];
//		t_buffer[7] = digits_paterns[simbol][7];
//		t_buffer[6] = digits_paterns[simbol][8];
//		t_buffer[5] = digits_paterns[simbol][9];
//
//		t_buffer[14] = digits_paterns[simbol][10];
//		t_buffer[13] = digits_paterns[simbol][11];
//		t_buffer[12] = digits_paterns[simbol][12];
//		t_buffer[11] = digits_paterns[simbol][13];
//		t_buffer[10] = digits_paterns[simbol][14];

		for(k = 0; k <= BYTESINPATTERN; k++)								// Read all pattern bytes
		{
			if(t_buffer[k] == 1)											// compare with mask
			{
				LED_Data[startRow][1] = red;
				LED_Data[startRow][2] = green;
				LED_Data[startRow][3] = blue;
			}
			startRow++;
		}
	}
	else																	// Not pair column ( LEDs start from bottom)
	{
		for(k = 0; k <= BYTESINPATTERN; k++)								// Read all pattern bytes
		{
			if(digits_paterns[simbol][k] == 1)								// compare with mask
			{
				LED_Data[startRow][1] = red;
				LED_Data[startRow][2] = green;
				LED_Data[startRow][3] = blue;
			}
			startRow++;
		}
	}
	// HAL_GPIO_WritePin(GPIOC, USER_LED, GPIO_PIN_RESET);

	WS2812_Send();

}
//// ----------------------------------------------------------------------------------------------
void printRuningNumbersAndLetters(int delay)		// Test function
{
	uint8_t red = 0;
	uint8_t green = 0;
	uint8_t blue = 0;

	for(uint8_t d = 0; d <= 37; d++)						// step over digits
	{
		uint8_t ranrom_num = RandAlg_GetRandInt(1, 3);
		if(ranrom_num == 1)
		{
			red = 255;
			green = 0;
			blue = 0;
		}
		if(ranrom_num == 2)
		{
			red = 0;
			green = 255;
			blue = 0;
		}
		if(ranrom_num == 3)
		{
			red = 0;
			green = 0;
			blue = 255;
		}

		for(uint8_t i = 0; i < BYTESINPATTERN; i++)
		{
			print_test_digits(i, d, red, green, blue);     // digits_paterns[10][15] =
			osDelay(delay);
		}
	}
}
//// ----------------------------------------------------------------------------------------------
void printString(char* str, uint8_t red, uint8_t green, uint8_t blue)
{
	cleanAllScreen();
	setTimeString(str, 5, red, green, blue);

	WS2812_Send();
}

//// ----------------------------------------------------------------------------------------------
void printTime(uint8_t minutes, uint8_t hours,  uint8_t green, uint8_t red,uint8_t blue)
{

	char main_buffer_str[6] = {0};
	char minutes_str[3] = {0};
	char hours_str[3] = {0};
	char dobledot[2] = ":";

	char sero_str[2] = "0";

	sprintf(minutes_str, "%d", minutes);
	sprintf(hours_str, "%d", hours);

	// додати 0 якщо менше 9
	if(hours <= 9)
	{
		strcat(main_buffer_str, sero_str);
		strcat(main_buffer_str, hours_str);
	}
	else
	{
		strcat(main_buffer_str, hours_str);
	}

	strcat(main_buffer_str, dobledot);

	if(minutes <= 9)
	{
		strcat(main_buffer_str, sero_str);
		strcat(main_buffer_str, minutes_str);
	}
	else
	{
		strcat(main_buffer_str, minutes_str);
	}

	cleanAllScreenBuffer();

	setTimeString( main_buffer_str, 5, red, green, blue);

	WS2812_Send();
}
//// ----------------------------------------------------------------------------------------------
void setTimeString( char *simbolstr, uint8_t string_size, uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t startRow = 0;
	uint8_t k = 0;
	uint8_t simbol = 0;

	for(uint8_t q = 0; q <= string_size; q++)
	{
		if(((simbolstr[q] - 48) >= 0) && (simbolstr[q] - 48 <= 9))		// If it is digits
		{
			simbol = simbolstr[q] - 48;
		}
		else															// If it is not digits
		{
			if(simbolstr[q] == ':')						// For make blink ':'
			{
				static bool trigSeccond = true;
				if(trigSeccond == true)
				{
					trigSeccond = false;
					simbol = 10;						// ':' symbol
				}
				else
				{
					trigSeccond = true;
					simbol = 11;						//' ' symbol (empty space)
				}
			}
		}

		// Choosing position for each symbols on screen
		switch (q) {
			case 0:
				startRow = 0;
				break;
			case 1:
				startRow = 4;
				break;
			case 2:
				startRow = 7;
				break;
			case 3:
				startRow = 10;
				break;
			case 4:
				startRow = 14;
				break;
		}

		startRow = startRow * ROWS;					// make one step(column) on screenЗ (one column = 5 LEDs)

		if((startRow == 5) || (startRow == 15) || (startRow == 25) || (startRow == 35) || (startRow == 45) || (startRow == 55) || (startRow == 65))  // Pair column ( LEDs startfrom top)
		{
			// Convert pattern
			uint8_t k = 0, l = 0;
			bool t_buffer[15] = {0, };

			for(uint8_t i = 1; i <= 3; i++)
			{
					t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
					t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
					t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
					t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
					t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
					l = 0;
				}

				for(k = 0; k <= BYTESINPATTERN; k++)								// Read all pattern bytes
				{
					if(t_buffer[k] == 1)											// compare with mask
					{
						LED_Data[startRow][1] = red;
						LED_Data[startRow][2] = green;
						LED_Data[startRow][3] = blue;
					}
					startRow++;
				}
			}
			else																	// Not pair column ( LEDs start from bottom)
			{
				for(k = 0; k <= BYTESINPATTERN; k++)								// Read all pattern bytes
				{
					if(digits_paterns[simbol][k] == 1)								// compare with mask
					{
						LED_Data[startRow][1] = red;
						LED_Data[startRow][2] = green;
						LED_Data[startRow][3] = blue;
					}
					startRow++;
				}
			}
	}
}
//// ----------------------------------------------------------------------------------------------
void setStringOnScreenWithShift(uint8_t startPosition, char *simbolstr, uint8_t string_size, uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t k = 0;
	uint8_t simbol = 0;
	uint8_t startRow = 0;

	for(uint8_t q = 0; ((q <= string_size) && (isalnum(simbolstr[q])) || (simbolstr[q] == ':') || (simbolstr[q] == ' ')); q++)
	{
		if(isdigit(simbolstr[q]))					// Detect digit
		{
			simbol = simbolstr[q] - 48;
		}
		if(isupper(simbolstr[q]))					// Detect letters
		{
			simbol = simbolstr[q] - 53;
		}
		if(simbolstr[q] == ' ')
		{
			simbol = simbolstr[q] - 21;
		}
		if(simbolstr[q] == ':')
		{
			simbol = simbolstr[q] - 48;
		}

		// Select place for sign on screen
		switch (q) {
			case 0:
				startRow = 0 + startPosition;
				break;
			case 1:
				startRow = 4 + startPosition;
				break;
			case 2:
				startRow = 8 + startPosition;
				break;
			case 3:
				startRow = 12 + startPosition;
				break;
			case 4:
				startRow = 16 + startPosition;
				break;
		}

		startRow = startRow * ROWS;					// make one step(column) on screenЗ (one column = 5 LEDs)

		if((startRow == 5) || (startRow == 15) || (startRow == 25) || (startRow == 35) || (startRow == 45) || (startRow == 55) || (startRow == 65) || (startRow == 75))  // Pair column ( LEDs startfrom top)
		{
			// Convert pattern
			uint8_t k = 0;
			uint8_t l = 0;
			bool t_buffer[15] = {0, };

			for(uint8_t i = 1; i <= 3; i++)
			{
				t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
				t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
				t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
				t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
				t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
				l = 0;
			}

			for(k = 0; k <= BYTESINPATTERN; k++)								// Read all pattern bytes
			{
				if(t_buffer[k] == 1)											// compare with mask
				{
					LED_Data[startRow][1] = red;
					LED_Data[startRow][2] = green;
					LED_Data[startRow][3] = blue;
				}
				startRow++;
			}
		}
		else																	// Not pair column ( LEDs start from bottom)
		{
			for(k = 0; k <= BYTESINPATTERN; k++)								// Read all pattern bytes
			{
				if(digits_paterns[simbol][k] == 1)								// compare with mask
				{
					LED_Data[startRow][1] = red;
					LED_Data[startRow][2] = green;
					LED_Data[startRow][3] = blue;
				}
				startRow++;

			}
		}
	}
}
// -----------------------------------------------------------------------------------------------------------
// функція виводить і букви і цифри на весь екран від першоно стовпчика да останнього. Приклад: "H: 7"
void setStringOnScreen( char *simbolstr, uint8_t string_size, uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t k = 0;
	uint8_t simbol = 0;
	uint8_t startRow = 0;
	uint8_t typeOfSimbol = 0;

	for(uint8_t q = 0; ((q <= string_size) && (isalnum(simbolstr[q])) || (simbolstr[q] == ':') || (simbolstr[q] == ' ')); q++)
	{
		if(isdigit(simbolstr[q]))					// Detect digit
		{
			simbol = simbolstr[q] - 48;
			typeOfSimbol = 0;
		}
		if(isupper(simbolstr[q]))					// Detect letters
		{
			simbol = simbolstr[q] - 53;  //
			typeOfSimbol = 1;
		}
		if(simbolstr[q] == ' ')
		{
			simbol = simbolstr[q] - 21;
			typeOfSimbol = 2;
		}
		if(simbolstr[q] == ':')
		{
			simbol = simbolstr[q] - 48;
			typeOfSimbol = 3;
		}


		// Select place for sign on screen
		switch (q) {
			case 0:
				startRow = 0;
				break;
			case 1:
				startRow = 4;
				break;
			case 2:
				startRow = 8;
				break;
			case 3:
				startRow = 12;
				break;
		}

		startRow = startRow * ROWS;					// make one step(column) on screenЗ (one column = 5 LEDs)

		if((startRow == 5) || (startRow == 15) || (startRow == 25) || (startRow == 35) || (startRow == 45) || (startRow == 55) || (startRow == 65))  // Pair column ( LEDs startfrom top)
		{
			// Convert pattern
			uint8_t k, l = 0;
			bool t_buffer[15] = {0, };

			for(uint8_t i = 1; i <= 3; i++)
			{
				t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
				t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
				t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
				t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
				t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
				l = 0;
			}

			for(k = 0; k <= BYTESINPATTERN; k++)								// Read all pattern bytes
			{
				if(t_buffer[k] == 1)											// compare with mask
				{
					LED_Data[startRow][1] = red;
					LED_Data[startRow][2] = green;
					LED_Data[startRow][3] = blue;
				}
				startRow++;
			}
		}
		else																	// Not pair column ( LEDs start from bottom)
		{
			for(k = 0; k <= BYTESINPATTERN; k++)								// Read all pattern bytes
			{
				if(digits_paterns[simbol][k] == 1)								// compare with mask
				{
					LED_Data[startRow][1] = red;
					LED_Data[startRow][2] = green;
					LED_Data[startRow][3] = blue;
				}
				startRow++;

			}
		}
	}
}

//// ----------------------------------------------------------------------------------------------
// Функція виводить число в перший стовпчик
void setNumberOnScreen(uint8_t startRow, char *simbolstr, uint8_t string_size, uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t k = 0;
	uint8_t simbol = 0;

	//string_size = string_size - 1;

	for(uint8_t q = 0; (q <= string_size) && (isdigit(simbolstr[q])); q++)
	{
		if((simbolstr[q] >= '0') && (simbolstr[q] <= '9'))		// If it is digits
		{
			simbol = simbolstr[q] - 48;
		}
		else															// If it is not digits
		{
			if(simbolstr[q] == ':')
			{
				//simbol = 11;
			}
		}

		switch (q) {
			case 0:
				startRow = 0;
				break;
			case 1:
				startRow = 4;
				break;
			case 2:
				startRow = 8;
				break;
			case 3:
				startRow = 12;
				break;
		}

		startRow = startRow * ROWS;					// make one step(column) on screenЗ (one column = 5 LEDs)

		if((startRow == 5) || (startRow == 15) || (startRow == 25) || (startRow == 35) || (startRow == 45) || (startRow == 55) || (startRow == 65))  // Pair column ( LEDs startfrom top)
		{
			// Convert pattern
			uint8_t k, l = 0;
			bool t_buffer[15] = {0, };

			for(uint8_t i = 1; i <= 3; i++)
			{
					t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
					t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
					t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
					t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
					t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
					l = 0;
				}

				for(k = 0; k <= BYTESINPATTERN; k++)								// Read all pattern bytes
				{
					if(t_buffer[k] == 1)											// compare with mask
					{
						LED_Data[startRow][1] = red;
						LED_Data[startRow][2] = green;
						LED_Data[startRow][3] = blue;
					}
					startRow++;
				}
			}
			else																	// Not pair column ( LEDs start from bottom)
			{
				for(k = 0; k <= BYTESINPATTERN; k++)								// Read all pattern bytes
				{
					if(digits_paterns[simbol][k] == 1)								// compare with mask
					{
						LED_Data[startRow][1] = red;
						LED_Data[startRow][2] = green;
						LED_Data[startRow][3] = blue;
					}
					startRow++;
				}
			}
	}
}
//// ----------------------------------------------------------------------------------------------

void setOnesimbol(uint8_t startRow, char simbolstr, uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t PositionOnScreen = 0;
	uint8_t k = 0;

	uint8_t simbol = simbolstr - 48;

	startRow = startRow * ROWS;			// make one step(column) on screenЗ (one column = 5 LEDs)

	if((startRow == 5) || (startRow == 15) || (startRow == 25) || (startRow == 35) || (startRow == 45) || (startRow == 55) || (startRow == 65))  // Pair column ( LEDs startfrom top)
	{
		// Convert pattern
		uint8_t k, l = 0;
		bool t_buffer[15] = {0, };

		for(uint8_t i = 1; i <= 3; i++)
		{
			t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
			t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
			t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
			t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
			t_buffer[((i * ROWS) - 1) - (l++)] = digits_paterns[simbol][k++];
			l = 0;
		}

		for(k = 0; k <= BYTESINPATTERN; k++)								// Read all pattern bytes
		{
			if(t_buffer[k] == 1)											// compare with mask
			{
				LED_Data[startRow][1] = red;
				LED_Data[startRow][2] = green;
				LED_Data[startRow][3] = blue;
			}
			startRow++;
		}
	}
	else																	// Not pair column ( LEDs start from bottom)
	{
		for(k = 0; k <= BYTESINPATTERN; k++)								// Read all pattern bytes
		{
			if(digits_paterns[simbol][k] == 1)								// compare with mask
			{
				LED_Data[startRow][1] = red;
				LED_Data[startRow][2] = green;
				LED_Data[startRow][3] = blue;
			}
			startRow++;
		}
	}
}
//// ----------------------------------------------------------------------------------------------
void textAnimation(void)
{
	char testString[] = "HELLO";
	int i = 0;

	for(i = 17; i > -20; i--)
	{
		cleanAllScreenBuffer();
		setStringOnScreenWithShift(i, testString, 5, 250,  0, 0);
		WS2812_Send();
		osDelay(50);
	}
}
//// ----------------------------------------------------------------------------------------------
void colorAnimation(void)
{
	fillInAllScreen(true, false, false, 0, 200, 15);
	fillInAllScreen(false, true, false, 0, 200, 15);
	fillInAllScreen(false, false, true, 0, 200, 15);

	osDelay(500);
}
//// ----------------------------------------------------------------------------------------------
static void fillInAllScreen(bool red, bool green, bool blue, uint8_t min_brightness, uint8_t max_brightness, int delay)
{
	int brightness =0;
	int led = 0;

	for(brightness = min_brightness; brightness <= max_brightness; brightness = brightness + 5)
	{
		for(led =0; led <= 84; led++)
		{
			Set_LED (led, red*brightness, green*brightness, blue*brightness);
		}
		WS2812_Send();
		osDelay(delay);
		cleanAllScreenBuffer();

		if(brightness >= max_brightness)
		{
			for(brightness = max_brightness; brightness >= min_brightness; brightness = brightness - 5)
			{
				for(led = 0; led <= 84; led++)
				{
					Set_LED (led, red*brightness, green*brightness, blue*brightness);
				}
				WS2812_Send();
				osDelay(delay);
				cleanAllScreenBuffer();
			}
			break;
		}
	}
}
//// ----------------------------------------------------------------------------------------------

















