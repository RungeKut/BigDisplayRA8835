#include "stm32f1xx_hal.h"
#include "SED1335-STM32.h"
#include "gpio.h"

// Удобные макросы (можно не использовать, если предпочитаете вызывать HAL напрямую)
#define A0_HIGH   HAL_GPIO_WritePin(Display_A0_GPIO_Port, Display_A0_Pin, GPIO_PIN_SET)
#define A0_LOW    HAL_GPIO_WritePin(Display_A0_GPIO_Port, Display_A0_Pin, GPIO_PIN_RESET)

#define WR_HIGH   HAL_GPIO_WritePin(Display_WR_GPIO_Port, Display_WR_Pin, GPIO_PIN_SET)
#define WR_LOW    HAL_GPIO_WritePin(Display_WR_GPIO_Port, Display_WR_Pin, GPIO_PIN_RESET)

#define CS_HIGH   HAL_GPIO_WritePin(Display_CS_GPIO_Port, Display_CS_Pin, GPIO_PIN_SET)
#define CS_LOW    HAL_GPIO_WritePin(Display_CS_GPIO_Port, Display_CS_Pin, GPIO_PIN_RESET)

#define RES_HIGH  HAL_GPIO_WritePin(Display_RES_GPIO_Port, Display_RES_Pin, GPIO_PIN_SET)
#define RES_LOW   HAL_GPIO_WritePin(Display_RES_GPIO_Port, Display_RES_Pin, GPIO_PIN_RESET)

// Функция записи байта на шину данных
void display_write_byte(uint8_t data) {
    // Устанавливаем каждый бит на соответствующий пин
    HAL_GPIO_WritePin(Display_D0_GPIO_Port, Display_D0_Pin, (data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Display_D1_GPIO_Port, Display_D1_Pin, (data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Display_D2_GPIO_Port, Display_D2_Pin, (data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Display_D3_GPIO_Port, Display_D3_Pin, (data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Display_D4_GPIO_Port, Display_D4_Pin, (data & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Display_D5_GPIO_Port, Display_D5_Pin, (data & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Display_D6_GPIO_Port, Display_D6_Pin, (data & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Display_D7_GPIO_Port, Display_D7_Pin, (data & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Строб WR: импульс записи
    WR_LOW;
    __NOP(); __NOP(); // минимальная задержка (~100 нс при 72 МГц — обычно достаточно)
    WR_HIGH;
}

// Микрозадержка (для HCLK = 72 МГц)
void delay_us(uint8_t us) {
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while (ticks--) __NOP();
}

void GLCD_InitPorts(void)
{
	// Аппаратный сброс
//	A0_LOW;
//	WR_LOW;
//	CS_LOW;
    RES_LOW;
    HAL_Delay(10);
    RES_HIGH;
    HAL_Delay(50); // дать контроллеру проснуться
}

//Функции отправки команды и данных
void GLCD_WriteData(unsigned char dataToWrite)
{
	CS_LOW;
    A0_HIGH;             // Режим данных
    display_write_byte(dataToWrite);
    CS_HIGH;
}
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
void GLCD_WriteCommand(unsigned char commandToWrite)
{
	CS_LOW;
    A0_LOW;              // Режим команды
    display_write_byte(commandToWrite);
    CS_HIGH;
}

// Вспомогательная функция: перевести все D0-D7 в режим INPUT
static void display_set_data_pins_input(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    // D0: PA15
    GPIO_InitStruct.Pin = Display_D0_Pin;
    HAL_GPIO_Init(Display_D0_GPIO_Port, &GPIO_InitStruct);

    // D1: PA15? Нет — у вас D1 = PA15? Нет, по вашему описанию:
    // D0: PB15 → но вы написали: Display_D0_Pin - PB15 → значит:
    // На самом деле:
    // Display_D0_Pin = PB15
    // Display_D1_Pin = PA15
    // Display_D2_Pin = PB3
    // ... и т.д.
    // Поэтому инициализируем каждый пин отдельно

    GPIO_InitStruct.Pin = Display_D1_Pin;
    HAL_GPIO_Init(Display_D1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Display_D2_Pin;
    HAL_GPIO_Init(Display_D2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Display_D3_Pin;
    HAL_GPIO_Init(Display_D3_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Display_D4_Pin;
    HAL_GPIO_Init(Display_D4_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Display_D5_Pin;
    HAL_GPIO_Init(Display_D5_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Display_D6_Pin;
    HAL_GPIO_Init(Display_D6_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Display_D7_Pin;
    HAL_GPIO_Init(Display_D7_GPIO_Port, &GPIO_InitStruct);
}

// Вспомогательная функция: вернуть все D0-D7 в режим OUTPUT_PP
static void display_set_data_pins_output(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = Display_D0_Pin;
    HAL_GPIO_Init(Display_D0_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Display_D1_Pin;
    HAL_GPIO_Init(Display_D1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Display_D2_Pin;
    HAL_GPIO_Init(Display_D2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Display_D3_Pin;
    HAL_GPIO_Init(Display_D3_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Display_D4_Pin;
    HAL_GPIO_Init(Display_D4_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Display_D5_Pin;
    HAL_GPIO_Init(Display_D5_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Display_D6_Pin;
    HAL_GPIO_Init(Display_D6_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Display_D7_Pin;
    HAL_GPIO_Init(Display_D7_GPIO_Port, &GPIO_InitStruct);
}

// Основная функция чтения байта с дисплея
unsigned char GLCD_ReadData(void) {
    // Переводим шину данных в режим входа
    display_set_data_pins_input();

    // Активируем CS и RD
    HAL_GPIO_WritePin(Display_CS_GPIO_Port, Display_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Display_RD_GPIO_Port, Display_RD_Pin, GPIO_PIN_RESET);

    // Небольшая задержка для стабилизации сигнала (можно убрать или заменить на __NOP())
    for (volatile int i = 0; i < 10; i++) __NOP();

    // Считываем каждый бит отдельно (т.к. пины на разных портах!)
    uint8_t data = 0;
    if (HAL_GPIO_ReadPin(Display_D0_GPIO_Port, Display_D0_Pin) == GPIO_PIN_SET) data |= (1 << 0);
    if (HAL_GPIO_ReadPin(Display_D1_GPIO_Port, Display_D1_Pin) == GPIO_PIN_SET) data |= (1 << 1);
    if (HAL_GPIO_ReadPin(Display_D2_GPIO_Port, Display_D2_Pin) == GPIO_PIN_SET) data |= (1 << 2);
    if (HAL_GPIO_ReadPin(Display_D3_GPIO_Port, Display_D3_Pin) == GPIO_PIN_SET) data |= (1 << 3);
    if (HAL_GPIO_ReadPin(Display_D4_GPIO_Port, Display_D4_Pin) == GPIO_PIN_SET) data |= (1 << 4);
    if (HAL_GPIO_ReadPin(Display_D5_GPIO_Port, Display_D5_Pin) == GPIO_PIN_SET) data |= (1 << 5);
    if (HAL_GPIO_ReadPin(Display_D6_GPIO_Port, Display_D6_Pin) == GPIO_PIN_SET) data |= (1 << 6);
    if (HAL_GPIO_ReadPin(Display_D7_GPIO_Port, Display_D7_Pin) == GPIO_PIN_SET) data |= (1 << 7);

    // Деактивируем CS и RD
    HAL_GPIO_WritePin(Display_RD_GPIO_Port, Display_RD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Display_CS_GPIO_Port, Display_CS_Pin, GPIO_PIN_SET);

    // Возвращаем шину в режим вывода (обычно нужно для последующей записи)
    display_set_data_pins_output();

    return data;
}

char GLCD_ReadByteFromROMMemory(char * ptr)
{
	return *(ptr);
}
