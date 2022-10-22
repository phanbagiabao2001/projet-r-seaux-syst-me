/*
 * 016uart_case.c
 *
 *  Created on: Oct 22, 2022
 *      Author: phanb
 */

#include <stdio.h>
#include "string.h"
#include "stm32f446.h"
#include "stm32f446_usart_driver.h"

char *msg[3] = {"hihihihihihi123", "Hello. how are you?", "Today is Monday"};

char rx_buf[1024];

USART_Handle_t usart2_handle;

//this flag indicates-reception completion
uint8_t rxCmplt = RESET;

uint8_t g_data=0;

extern void initialise_monitor_handles();

void delay(void){
	for(uint32_t i=0; i<500000/2; i++);
}

void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART1;
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2_handle);
}

void USART2_GPIOInit(void)
{
	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOBaseAddr = GPIOA;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

	//USART6 TX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&usart_gpios);

	//USART6 RX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart_gpios);
}
void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioLed, GpioBtn;
	GpioLed.pGPIOBaseAddr = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

//	GPIO_Handle_t GpioBtn;
	GpioBtn.pGPIOBaseAddr = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOC,ENABLE);
	GPIO_Init(&GpioBtn);
}

int main(void)
{
	uint32_t cnt = 0;


	initialise_monitor_handles();

	USART2_GPIOInit();
    USART2_Init();

    USART_IRQInterruptConfig(IRQ_NO_USART2,ENABLE);

    USART_PeripheralControl(USART2,ENABLE);

    printf("Application is running\n");


	while(1)
	{
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == 0);

			delay();

			// Next message index ; make sure that cnt value doesn't cross 2
			cnt = cnt % 3;

			//First lets enable the reception in interrupt mode
			//this code enables the receive interrupt
			while ( USART_ReceiveDataIT(&usart2_handle,rx_buf,strlen(msg[cnt])) != USART_READY );

			//Send the msg indexed by cnt in blocking mode
	    	USART_SendData(&usart2_handle,(uint8_t*)msg[cnt],strlen(msg[cnt]));

	    	printf("Transmitted : %s\n",msg[cnt]);


	    	//Now lets wait until all the bytes are received from the arduino .
	    	//When all the bytes are received rxCmplt will be SET in application callback
	    	while(rxCmplt != SET);

	    	//just make sure that last byte should be null otherwise %s fails while printing
	    	rx_buf[strlen(msg[cnt])+ 1] = '\0';

	    	//Print what we received from the arduino
	    	printf("Received    : %s\n",rx_buf);

	    	//invalidate the flag
	    	rxCmplt = RESET;

	    	//move on to next message indexed in msg[]
	    	cnt ++;

		return 0;
	}
}

void USART2_IRQHandler(void)
{
	USART_IRQHandling(&usart2_handle);
}

void USART_ApplicationEventCallback( USART_Handle_t *pUSARTHandle,uint8_t ApEv)
{
   if(ApEv == USART_EVENT_RX_CMPLT)
   {
			rxCmplt = SET;

   }else if (ApEv == USART_EVENT_TX_CMPLT)
   {
	   ;
   }
}
