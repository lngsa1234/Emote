#ifndef STM32_TIMER_H
#define STM32_TIMER_H


#include <Timer.h>


typedef struct { } T64khz;
typedef struct { } T128khz;
typedef struct { } T2mhz;
typedef struct { } T4mhz;

typedef TMicro TOne;
typedef TMicro TThree;
typedef uint32_t counter_one_overflow_t;
typedef uint16_t counter_three_overflow_t;

extern uint16_t TIM_GetPrescalar(TIM_TypeDef* TIMx);

#endif
