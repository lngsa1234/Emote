/*
 * @file  PlatformP.nc
 * @author Thomas Schmidt
 *
 */
#include "hardware.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include <stm32_eval.h>
#include <stm3210e_eval.h>
#include <stm32f10x_bootstrap.h>



module PlatformP {
    provides {
        interface Init;
        interface PlatformReset;
    }
    uses {
        interface Init as MoteInit;
        interface Init as MoteClockInit;
        interface Init as McuSleepInit;
        interface HplSTM32Interrupt as Interrupt;
    }
}
implementation {
	
    command error_t Init.init() {

	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	LowLevelInit();

        call MoteClockInit.init();


	// Enabling Clocks associated with the gpio
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF , ENABLE);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
#ifndef _FOR_MICROFRAMEWORK_


        // Init clock system
        //call McuSleepInit.init();

#endif
        //call MoteInit.init();
	
	//STM_EVAL_LEDInit(LED1);
	//STM_EVAL_LEDInit(LED2);
	//STM_EVAL_LEDInit(LED3);
	//STM_EVAL_LEDInit(LED4);

	// Testing if gpio is working
	while(1)
	{
		GPIO_WriteBit(GPIOF, 6, Bit_SET);
		GPIO_WriteBit(GPIOF, 7, Bit_SET);
		GPIO_WriteBit(GPIOF, 8, Bit_SET);
		//STM_EVAL_LEDOn(LED1);		
		//STM_EVAL_LEDOn(LED2);		
		//STM_EVAL_LEDOn(LED3);		
		
		Delay(0xAFFFF);

		GPIO_WriteBit(GPIOF, 6, Bit_RESET);
		GPIO_WriteBit(GPIOF, 7, Bit_RESET);
		GPIO_WriteBit(GPIOF, 8, Bit_RESET);
		//STM_EVAL_LEDOff(LED1);
		//STM_EVAL_LEDOff(LED2);
		//STM_EVAL_LEDOff(LED3);
	}


        return SUCCESS;
    }


    async command void PlatformReset.reset() {
        while (1);
        return; // Should never get here.
    }

    void nmi_handler()
    {
        while(1) {};
        return ;
    }

    void hardfault_handler()
    {
        while(1) {};
        return ;
    }

    //Functions definitions
    void myDelay(unsigned long delay )
    {
        while(delay) delay--;
    }

    async event void Interrupt.fired()
{

}

}

