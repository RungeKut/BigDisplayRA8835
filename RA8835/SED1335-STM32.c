#include "stm32f1xx_hal.h"
#include "SED1335-STM32.h"
#include "gpio.h"

#define SED1335_PORT GPIOA

#define SED1335_A0	Display_A0_Pin
#define SED1335_WR	Display_WR_Pin
#define SED1335_RD	Display_RD_Pin
#define SED1335_CS	Display_CS_Pin
#define SED1335_RES	Display_RES_Pin

#define SED1335_D0   0

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
void GLCD_InitPorts(void)
{
	int i = 0;
	HAL_GPIO_WritePin(SED1335_PORT, SED1335_A0 | SED1335_WR | SED1335_RD | SED1335_CS, GPIO_PIN_RESET);
	while (i < 5) i++;
	
	HAL_GPIO_WritePin(SED1335_PORT, SED1335_RES, GPIO_PIN_SET);
}
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
void GLCD_WriteData(unsigned char dataToWrite)
{
	int i = 0;
	HAL_GPIO_WritePin(SED1335_PORT, (dataToWrite << SED1335_D0), GPIO_PIN_SET);
	dataToWrite ^= 0xFF;
	HAL_GPIO_WritePin(SED1335_PORT, (dataToWrite << SED1335_D0), GPIO_PIN_RESET);

	HAL_GPIO_WritePin(SED1335_PORT, (SED1335_CS | SED1335_A0 | SED1335_WR), GPIO_PIN_RESET);
	while (i < 3) i++;
	HAL_GPIO_WritePin(SED1335_PORT, (SED1335_CS | SED1335_A0 | SED1335_WR), GPIO_PIN_SET);
}
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
void GLCD_WriteCommand(unsigned char commandToWrite)
{
	int i = 0;
	HAL_GPIO_WritePin(SED1335_PORT, (commandToWrite << SED1335_D0), GPIO_PIN_SET);
	commandToWrite ^= 0xFF;
	HAL_GPIO_WritePin(SED1335_PORT, (commandToWrite << SED1335_D0), GPIO_PIN_RESET);

	HAL_GPIO_WritePin(SED1335_PORT, (SED1335_CS | SED1335_WR), GPIO_PIN_RESET);
	while (i < 3) i++;
	HAL_GPIO_WritePin(SED1335_PORT, (SED1335_CS | SED1335_WR), GPIO_PIN_SET);
}
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
unsigned char GLCD_ReadData(void)
{
	int i = 0;
	unsigned char tmp;
	{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = 0xFF << SED1335_D0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	HAL_GPIO_WritePin(SED1335_PORT, (SED1335_CS | SED1335_RD), GPIO_PIN_RESET);
	while (i < 3) i++;
	tmp = (SED1335_PORT->IDR >> SED1335_D0) & 0xFF;
	HAL_GPIO_WritePin(SED1335_PORT, (SED1335_CS | SED1335_RD), GPIO_PIN_SET);
	{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = 0xFF << SED1335_D0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	return tmp;
}
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
char GLCD_ReadByteFromROMMemory(char * ptr)
{
	return *(ptr);
}
