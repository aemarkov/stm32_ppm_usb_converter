#include "ppm_reader.h"
#include "stm32f10x.h"                  // Device header
#include "misc.h"                       // Keil::Device:StdPeriph Drivers:Framework
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM

void tim_init(void);

impulse_received impulse_callback;

void ppm_reader_init(impulse_received callback)
{
	impulse_callback = callback;
	tim_init();
}

/* Initialize timer to capture impulses
*   Pin         PB10
*   Resolution  1 us
*/
void tim_init(void)
{
	// Timer init
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	TIM_TimeBaseInitTypeDef tim_init = {0};
	tim_init.TIM_CounterMode = TIM_CounterMode_Up;
	tim_init.TIM_Prescaler = 72;
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
	}
	else if(TIM_GetFlagStatus(TIM2, TIM_FLAG_CC3)) {
		TIM_ClearFlag(TIM2, TIM_FLAG_CC3);
		TIM2->CNT=0;
		impulse_callback(TIM2->CCR3);
	}
}
