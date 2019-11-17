#include "stm32f10x.h"                  // Device header
#include "misc.h"                       // Keil::Device:StdPeriph Drivers:Framework
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART

#include <stdio.h>
#include <stdbool.h>

#include "main.h"

#define PPM_SYNC_THRESHOLD 3000
#define PPM_NUM_CHANNELS   10

typedef enum ppm_decoder_state
{
	PPM_STATE_START, 					// Initial unknown state
	PPM_STATE_RECEIVING,			// Receiving state
} ppm_decoder_state_t;

uint16_t ppm_buffer[PPM_NUM_CHANNELS];
uint8_t ppm_channel_index;
ppm_decoder_state_t ppm_current_state;


void gpio_init(void);
void tim_init(void);
void tim_process_event(void);
void usart_init(void);
	
int main()
{
	ppm_channel_index = 0;
	ppm_current_state = PPM_STATE_START;
	
	gpio_init();
	tim_init();
	usart_init();
	
  while(1) {
		GPIOC->ODR ^= GPIO_Pin_15;
		//printf("Hello\n");
		for(int i = 0; i < 1000000; i++);
	}
	return 0;
}


// Initialize user-defined GPIOs
void gpio_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef gpio_init = {0};
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_Init(GPIOC, &gpio_init);
}

void tim_init(void)
{
	// Timer init
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	TIM_TimeBaseInitTypeDef tim_init = {0};
	tim_init.TIM_CounterMode = TIM_CounterMode_Up;
	tim_init.TIM_Prescaler = 48;
	tim_init.TIM_ClockDivision = TIM_CKD_DIV1;
	tim_init.TIM_Period = 65535;
	tim_init.TIM_RepetitionCounter = 1;
	TIM_TimeBaseInit(TIM2, &tim_init);
	
	// Input capture GPIO init
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
	
	GPIO_InitTypeDef gpio_init = {0};
	gpio_init.GPIO_Mode = GPIO_Mode_IPD;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init.GPIO_Pin = GPIO_Pin_10;
	//TIM_ARRPreloadConfig(TIM2, DISABLE);
	GPIO_Init(GPIOB, &gpio_init);
	
	// Input capture channel init
	TIM_ICInitTypeDef ic_init = {0};
	ic_init.TIM_Channel = TIM_Channel_3;
	ic_init.TIM_ICFilter = 0;
	ic_init.TIM_ICPolarity = TIM_ICPolarity_Rising;
	ic_init.TIM_ICPrescaler = 0;
	ic_init.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM2, &ic_init);
	
	// Interrupts init
	NVIC_EnableIRQ(TIM2_IRQn);
	TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC3, ENABLE);
	
	// Start timer
	TIM_Cmd(TIM2, ENABLE);
	TIM_CCxCmd(TIM2, TIM_Channel_3, ENABLE);
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetFlagStatus(TIM2, TIM_FLAG_Update)){
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);
		//GPIOC->ODR ^= GPIO_Pin_13;
	}
	else if(TIM_GetFlagStatus(TIM2, TIM_FLAG_CC3)) {
		TIM_ClearFlag(TIM2, TIM_FLAG_CC3);
		tim_process_event();
	}
}

void ppm_send()
{
	//for(int i = 0; i<4; i++) {
	//	printf("%u ", ppm_buffer[i]);
	//}
	//printf("\n");
	printf("%u %u\n", ppm_buffer[0], ppm_buffer[1]);
}

void ppm_finite_automate(uint16_t duration)
{
	bool is_sync = duration > PPM_SYNC_THRESHOLD;
	
	switch(ppm_current_state)
	{
		case PPM_STATE_START:
			// Initial unknown state
			if(is_sync) {
				ppm_channel_index = 0;
				ppm_current_state = PPM_STATE_RECEIVING;
			}
			break;
		case PPM_STATE_RECEIVING:
			if(is_sync) {
				if(ppm_channel_index == PPM_NUM_CHANNELS) {
					// All channels received
					// All channels received, sync strobe received
					// TODO: send
					ppm_send();
					ppm_channel_index = 0;
					ppm_current_state = PPM_STATE_RECEIVING;
				}
				else {
					// Not enough channels received
					// Looks like synchronization is failed
					ppm_current_state = PPM_STATE_START;
				}
			}
			else {
				if(ppm_channel_index < PPM_NUM_CHANNELS) {
					// One more channel received
					ppm_buffer[ppm_channel_index++] = duration;
				} else {
					// Too many channels received
					// Looks like synchronization is failed
					ppm_current_state = PPM_STATE_START;
				}
			}
			break;
	};
}

void tim_process_event()
{
	GPIOC->ODR ^= GPIO_Pin_13;
	uint16_t duration = TIM2->CCR3;
	TIM2->CNT = 0;
	ppm_finite_automate(duration);
}

void usart_init(void)
{
	// USART GPIO init
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef gpio_init = {0};
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	// TX
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA, &gpio_init);
	
	// USART init
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	USART_InitTypeDef usart_init = {0};
	usart_init.USART_BaudRate = 115200;
	usart_init.USART_Mode = USART_Mode_Tx;
	usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart_init.USART_Parity = USART_Parity_No;
	usart_init.USART_StopBits = USART_StopBits_1;
	usart_init.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &usart_init);
	
	USART_Cmd(USART1, ENABLE);
}


void usart_send_byte(uint8_t byte)
{
	while(!USART_GetFlagStatus(USART1, USART_FLAG_TXE));
	USART1->DR = byte;
}
