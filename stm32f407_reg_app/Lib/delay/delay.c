#include "delay.h" // delay k�t�ohanesi

uint32_t SysTickCounter;

void SysTick_Handler()
{
	if(SysTickCounter !=0x00)
	{
	
	SysTickCounter--;
	
	}


}


void delay_ms(uint32_t delay) 
{
	SysTickCounter=delay;
	while(SysTickCounter != 0){
	
	}


}

void delay_s(uint8_t delay)  
{
	SysTickCounter=delay*1000;
	while(SysTickCounter != 0){
	
	}


}
