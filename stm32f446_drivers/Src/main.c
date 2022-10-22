/*
 * main.c
 *
 *  Created on: Aug 5, 2022
 *      Author: phanb
 */
#include "stm32f446.h"
#include "stm32f446_gpio_driver.h"

int main(void)
{
	return 0;
}

void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(0);
}

