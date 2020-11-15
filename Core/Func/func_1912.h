/*
 * accelerate.h
 *
 *  Created on: May 25, 2020
 *      Author: test
 */

#ifndef _FUNC_ACCELERATE_H_
#define _FUNC_ACCELERATE_H_

#include "conf_usart.h"


#define Force_huart huart2
#define Force_uart USART2


extern float F_1912;
extern uint8_t myID;

float getCurrentForce(void);
void Force_1912_Init(int ID);
void Force_1912_Start(void);
void ClearForce_Device(uint8_t ID);


TEST test1912_forceCollector_communication(void);
TEST test_CheckSum(void);
TEST test_Command(void);
TEST test_searchID(void);
TEST test_GetForce(void);
TEST test_GetCurrentForce(void);
void IDLE_UART_IRQHandler(UART_HandleTypeDef *huart);

#endif /* 1_FUNC_ACCELERATE_H_ */
