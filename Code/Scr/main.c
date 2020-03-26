/*
 * main.c
 *
 *  Created on: 25 мар. 2020 г.
 *      Author: Mitin Nikita
 */

/***************************** INCLUDES *******************************/
#include "main.h"
/*********************************************************************/

uint64_t countT = 0;

int main(void){

	GPIO_Init();
	TIM_Init();
	RCC_Init();
	PWM_Init();
	CRC_Init();

	uint16_t i = 0;
	uint32_t crc = 0;
	while(1){
		TIM3->CCR3 = i;
		i = (i < 80) ? i + 1 : 0;
		DelayMillis(40);
		CRC->DR = CRC_DR_DR;
		CRC->DR = __REV(123456789);
		crc = __REV(CRC->DR);

	}
}


/***************************** IRQHandlers **************************/
void TIM14_IRQHandler(void){
	countT++;
	TIM14->SR &= ~TIM_SR_UIF;				//сброс флага прерываний
}

/********************************************************************/



/****************************** FUNCTIONS ***************************/
void DelayMillis(uint32_t time){
	TIM14->DIER |= TIM_DIER_UIE;									// комбинация команд запускает таймер, запись 0 приведет к останавке
	TIM14->CR1  |= TIM_CR1_CEN;
	while(countT < time){}
	countT = 0;
	TIM14->DIER &= ~TIM_DIER_UIE;								// комбинация команд запускает таймер, запись 0 приведет к останавке
	TIM14->CR1  &= ~TIM_CR1_CEN;
}




