// @author Nived Sivadas 

#ifndef _H_STM32SPI_h
#define _H_STM32SPI_h

#include "/opt/tinyos-2.x/tos/chips/stm32/fwlib/inc/stm32f10x_spi.h"
#include "/opt/tinyos-2.x/tos/chips/stm32/fwlib/inc/stm32f10x_map.h"

//====================== SPI Bus ==================================

enum {
  STM32_SPI_CLK_DIVIDE_4 = 0,
  STM32_SPI_CLK_DIVIDE_16 = 1,
  STM32_SPI_CLK_DIVIDE_64 = 2,
  STM32_SPI_CLK_DIVIDE_128 = 3,
};

SPI_InitTypeDef SPI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

extern uint16_t SPI_I2S_ReceiveData(SPI_TypeDef* SPIx);

#endif
