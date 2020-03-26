/*
 * InitPerif.c
 *
 *  Created on: 25 мар. 2020 г.
 *      Author: Mitin Nikita
 */

/***************************** INCLUDES *******************************/
#include "main.h"
/*********************************************************************/


/***************************** FUNCTIONS ******************************/

void GPIO_Init(){
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN |
				   RCC_AHBENR_GPIOAEN;   						// подаем_тактирование_на_порты GPIOC и GPIOA

	GPIOC->MODER |= GPIO_MODER_MODER9_0;  						// устанавливаем_пин 9 в_режим OUTPUT
	GPIOA->MODER &= ~GPIO_MODER_MODER0_0;						// устанавливаем_пин 0 в_режим INPUT
}

void RCC_Init(){
	RCC->CR |= ((uint32_t)RCC_CR_HSEON);						//влючение_HSE(_кварц_)
	while(!(RCC->CR & RCC_CR_HSERDY)) GPIOC->ODR |= 1<<8;		//ожидание_включния
	GPIOC->ODR &= ~(1<<8);

	FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;			// Настройка_тактирования ДЛЯ ФЛЭШ

	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;							//APB_presc
	RCC->CFGR |= RCC_CFGR_PPRE_DIV1;							//APB1_presc

	RCC->CFGR &= ~(RCC_CFGR_PLLMUL|RCC_CFGR_PLLSRC|RCC_CFGR_PLLXTPRE);
//	RCC->CFGR |= RCC_CFGR_USB;

	RCC->CFGR |= RCC_CFGR_PLLXTPRE_PREDIV1_Div2;
	RCC->CFGR2|= RCC_CFGR2_PREDIV1_DIV2;
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSE_PREDIV;
	RCC->CFGR |= RCC_CFGR_PLLMULL6;								// УМНОЖИТЕЛЬ
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY)) GPIOC->ODR |= 1<<9;
	GPIOC->ODR &= ~(1<<9);

	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void TIM_Init(){
	/****** TIM6 millis for dellay***/
	NVIC_EnableIRQ (TIM14_IRQn);								// Разрешает_прерывания
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;						// Подаем_тактирование на_таймер

	TIM14->PSC = 480-1;											// Устанавливаем_предилитель
	TIM14->ARR = 100-1;											// Установка_регистра_сброса

//	TIM14->DIER |= TIM_DIER_UIE;								// комбинация_команд_запускает_таймер,_запись 0_приведет_к_останавке
//	TIM14->CR1  |= TIM_CR1_CEN;
	/*******************************/
}

void ADC_Init(){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  						// тактирование_на_порт_GPIOA
	GPIOA->MODER |= GPIO_MODER_MODER1;  						// аналог_вход;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; 						// тактирование_на_ADC

	ADC1->CFGR2 |= ADC_CFGR2_CKMODE_1;  						// Делитель_на 4 (48/4 = 12) ADC <= 14 мГц

	/*_____________Калибровка____________*/
	if ((ADC1->CR & ADC_CR_ADEN) != 0){
		ADC1->CR |= ADC_CR_ADDIS;
	}
	while ((ADC1->CR & ADC_CR_ADEN) != 0){}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
	ADC1->CR |= ADC_CR_ADCAL;
	while ((ADC1->CR & ADC_CR_ADCAL) != 0){}
	/*__________Конец колибровки__________*/

	ADC1->CR |= ADC_CR_ADEN;
	ADC1->SMPR |= ADC_SMPR1_SMPR_0|
				  ADC_SMPR1_SMPR_1|
			      ADC_SMPR1_SMPR_2; 							//0x111 239.5 cycles
	ADC1->CHSELR |= ADC_CHSELR_CHSEL16|ADC_CHSELR_CHSEL1;		// Выбор_двух_каналов 16 и 1. 16_термодатчик_камня
}

void PWM_Init(){
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	GPIOC->AFR[1] |= GPIO_AFRH_AFR8;
	GPIOC->MODER |= GPIO_MODER_MODER8_1;      					//Альтернативная_функция (ВАЖНО! Не_настраивать в GPIO порт_вывода_для ШИМ)

	TIM3->PSC = 47; /* (1) */
	TIM3->ARR = 80; /* (2) */
	TIM3->CCR3 = 40; /* (3) */									//Коэф_заполнения CCRx/ARR

	TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2			//110 - ШИМ 1
				| TIM_CCMR2_OC3PE;
	TIM3->CR1 |= TIM_CR1_ARPE;									//Auto-reload preload enable
	TIM3->CCER |= TIM_CCER_CC3E;								//output enable
	TIM3->BDTR |= TIM_BDTR_MOE;									//Разерешение_вывода_на GPIO
	TIM3->EGR |= TIM_EGR_UG;									//обновление_генерации

	TIM3->DIER |= TIM_DIER_UIE;
	TIM3->CR1 |= TIM_CR1_CEN;
}

void EXTI_Init(){

}

void UART_Init(){

}

void SPI_Init(){

}

void CRC_Init(){
	RCC->AHBENR |= RCC_AHBENR_CRCEN;

	CRC->CR = CRC_CR_RESET;
}

