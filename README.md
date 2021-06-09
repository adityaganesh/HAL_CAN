# HAL_CAN
Implementation of CAN Bus on a stm32f4 discovery board using the HAL_CAN API

## Steps for setting up timer :
1. Board used :- STM32F401 
2. Enable the TIM10 in the ioc file 
3. In parameter settings set Prescaler as 8400 - 1 as prescaler start from 0 in my case the clock frequency is 84 MHz that's why prescaler is 8400 - 1 
4. The above parameter gives the frequency of timer as 10KHz , so to make the timer count till 250th millisecond we will set counter period as 2500 - 1 we are subtracting one since at the last count interrupt will be generated 
5. Calculation : 250ms * (84MHz / 8400)
