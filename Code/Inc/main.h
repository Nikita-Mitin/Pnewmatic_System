/*
 * main.h
 *
 *  Created on: 25 мар. 2020 г.
 *      Author: Mitin Nikita
 */

#ifndef CODE_INC_MAIN_H_
#define CODE_INC_MAIN_H_

/***************************** INCLUDES *******************************/
#include "stm32f0xx.h"
/*********************************************************************/



/**************************** PROTOTYPES *****************************/
void GPIO_Init();
void RCC_Init();
void TIM_Init();
void ADC_Init();
void PWM_Init();
void EXTI_Init();
void UART_Init();
void SPI_Init();

void DelayMillis(uint32_t);
/*********************************************************************/




#endif /* CODE_INC_MAIN_H_ */
