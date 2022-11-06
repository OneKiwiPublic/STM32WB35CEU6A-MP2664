#include <stdio.h>
#include "board.h"
#include "stm32wbxx_ll_cortex.h"
#include "stm32wbxx_ll_usart.h"
#include "stm32wbxx_ll_gpio.h"
#include "stm32wbxx_ll_rcc.h"
#include "stm32wbxx_ll_bus.h"
#include "stm32wbxx_ll_exti.h"

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	//HAL_UART_Transmit(&huart?, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	LL_USART_TransmitData8(USART1, ch);
	while(LL_USART_IsActiveFlag_TXE(USART1) == 0){
	}
	return ch;
}

void led_init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

	LL_GPIO_ResetOutputPin(LED_PORT, LED_PIN);

	GPIO_InitStruct.Pin = LED_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
}

void led_on(void)
{
	LL_GPIO_ResetOutputPin(LED_PORT, LED_PIN);
}

void led_off(void)
{
	LL_GPIO_SetOutputPin(LED_PORT, LED_PIN);
}

void led_toggle(void)
{
	LL_GPIO_TogglePin(LED_PORT, LED_PIN);
}

void usart1_init(void)
{
	LL_USART_InitTypeDef USART_InitStruct = {0};

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);

	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	/**USART1 GPIO Configuration
	PB6   ------> USART1_TX
	PB7   ------> USART1_RX
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);


	USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
	USART_InitStruct.BaudRate = 115200;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART1, &USART_InitStruct);
	LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_DisableFIFO(USART1);
	LL_USART_ConfigAsyncMode(USART1);

	LL_USART_Enable(USART1);

	/* Polling USART1 initialisation */
	while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
	{
	}
}

