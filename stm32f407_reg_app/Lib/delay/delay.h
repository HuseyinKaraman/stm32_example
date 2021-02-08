#ifndef __DELAY_H
#define __DEFINE_H

#include "stm32f4xx.h"  // sistem lib
#include "stdint.h"  // Integer tipinde degiskenler için lib

extern uint32_t SysTickCounter;		// Systick Handler içerisindeki sayaç

void delay_ms(uint32_t);  // mili-saniye parametreli bekletme fonksiyonu

void delay_s (uint8_t); // saniye cinsinden bekletme fonksiyonu 

#endif
