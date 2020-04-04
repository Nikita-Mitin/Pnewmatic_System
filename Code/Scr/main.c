/*
 * main.c
 *
 *  Created on: 25 мар. 2020 г.
 *      Author: Mitin Nikita
 */

/***************************** INCLUDES *******************************/
#include "main.h"
/*********************************************************************/

/****************************** DEFINES ******************************/
#define klapan_1_pin 1
#define klapan_2_pin 2
#define klapan_3_pin 3
#define klapan_4_pin 4

#define inertDelay 100 //ms
#define cluthDelay 100 //ms

#define NumOfGear 4

#define klapan_1_ON GPIOC->ODR |= (1 << klapan_1_pin)
#define klapan_2_ON GPIOC->ODR |= (1 << klapan_2_pin)
#define klapan_3_ON GPIOC->ODR |= (1 << klapan_3_pin)
#define klapan_4_ON GPIOC->ODR |= (1 << klapan_4_pin)
#define klapan_1_OFF GPIOC->ODR &= ~(1 << klapan_1_pin)
#define klapan_2_OFF GPIOC->ODR &= ~(1 << klapan_2_pin)
#define klapan_3_OFF GPIOC->ODR &= ~(1 << klapan_3_pin)
#define klapan_4_OFF GPIOC->ODR &= ~(1 << klapan_4_pin)

/*********************************************************************/
uint64_t countT = 0;

int main(void){

	GPIO_Init();
	TIM_Init();
	RCC_Init();
	PWM_Init();



	while(1){



	}
}


/***************************** IRQHandlers **************************/
void TIM14_IRQHandler(void){
	countT++;
	TIM14->SR &= ~TIM_SR_UIF;										//сброс флага прерываний
}

/********************************************************************/



/****************************** FUNCTIONS ***************************/
void DelayMillis(uint32_t time){
	TIM14->DIER |= TIM_DIER_UIE;									// комбинация команд запускает таймер, запись 0 приведет к останавке
	TIM14->CR1  |= TIM_CR1_CEN;
	while(countT < time){}
	countT = 0;
	TIM14->DIER &= ~TIM_DIER_UIE;									// комбинация команд запускает таймер, запись 0 приведет к останавке
	TIM14->CR1  &= ~TIM_CR1_CEN;
}

void GearUP(){
	klapan_3_ON;
	DelayMillis(cluthDelay);
	klapan_3_OFF;

	klapan_1_ON;
	DelayMillis(inertDelay);
	klapan_1_OFF;
	DelayMillis(inertDelay);

	klapan_4_ON;
	DelayMillis(cluthDelay);
	klapan_4_OFF;
}

void GearDOWN(){
	klapan_3_ON;
	DelayMillis(cluthDelay);
	klapan_3_OFF;

	klapan_1_ON;
	DelayMillis(inertDelay);
	klapan_1_OFF;
	DelayMillis(inertDelay);

	klapan_4_ON;
	DelayMillis(cluthDelay);
	klapan_4_OFF;
}

