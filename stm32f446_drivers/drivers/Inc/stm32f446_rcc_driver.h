/*
 * stm32f446_rcc_driver.h
 *
 *  Created on: Oct 19, 2022
 *      Author: phanb
 */
#include "stm32f446.h"

#ifndef INC_STM32F446_RCC_DRIVER_H_
#define INC_STM32F446_RCC_DRIVER_H_

//It return the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//It return the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);

#endif /* INC_STM32F446_RCC_DRIVER_H_ */
