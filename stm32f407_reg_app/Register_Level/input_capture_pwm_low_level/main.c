#include<stm32f4xx.h>  // uygulama_5 LOW LEVEL


void setClock(void) ;

void Tim2_InputCapture_CH1_CH2_Init()
{
	SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOAEN);
	
	GPIOA->MODER |= (0x2 << GPIO_MODER_MODE5_Pos); // MODER5[1:0] 11:Analog Mode.

	GPIOA->AFR[0] |= (0x1<< GPIO_AFRL_AFSEL5_Pos);   //  GPIO_AFRL_AFSEL5_Pos => 20-23. bitler   .   0x01  => 0001: AF1(TIM1/TIM2)
	// AFRLy: Alternate function selection for port x bit y (y = 0..7 pini ifade ediyor)    AFRLy selection: 0000...1111 (AF0=>AF15)
	// GPIOA portunun 5. pin alternate function registeri ile tim2 iliskilendirilmesi yapiliyor.
	
	SET_BIT(RCC->APB1ENR,RCC_APB1ENR_TIM2EN);
	
	TIM2->SMCR = (0x05<<TIM_SMCR_TS_Pos) | (0x04<<TIM_SMCR_SMS_Pos);  // TIMx slave mode control register(SMCR)  
	// TS(Trigger Selectin) => 101: Filtered Timer Input 1 (TI1FP1)      This bit-field selects the trigger input to be used to synchronize the counter.
	//  SMS(SLAVE MODE SELECTION) => 100: Reset Mode - Rising edge of the selected trigger input (TRGI) reinitializes the counter and generates an update of the registers.
	
	TIM2->CCMR1 = (0x01<<TIM_CCMR1_CC1S_Pos) | (0x02<<TIM_CCMR1_CC2S_Pos);  // 01: CC1 channel is configured as input, IC1 is mapped on TI1.
																																					// 10: CC2 channel is configured as input, IC2 is mapped on TI1
	
	TIM2->CCER = (0x01<<TIM_CCER_CC2P_Pos); //  CC2 CHANNEL INPUT => 01: inverted/falling edge Circuit is sensitive to TIxFP1 falling edge (capture, trigger in reset, external clock or trigger mode)  ,
																						// TIxFP1 is inverted (trigger in gated mode, encoder mode).
	// KANAL 2 input-capture modunda düsen kenarlari yakalama modunda çalistirilmasi için polarity ayari yapiliyor. 
/*  
(Hiç degisiklik yapmadigimiz için 00) CC1 CHANNEL INPUT =>00: noninverted/rising edge   
Circuit is sensitive to TIxFP1 rising edge (capture, trigger in reset, external clock or trigger mode), TIxFP1 is not inverted (trigger in gated mode, encoder mode).
*/
	
	TIM2->CCER |=  TIM_CCER_CC1E | TIM_CCER_CC2E ;  // 1: Capture enabled
	
	TIM2->PSC=84-1;
	
	TIM2->ARR=10000000;
	
	TIM2->CR1|=0X0001;
	
	
}

void Tim4_PWM_CH1_Init()
{
				
				SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIODEN);
		
	      GPIOD->MODER |=(0x2 <<GPIO_MODER_MODE12_Pos);  // 10:alternate function mode
	
				GPIOD->AFR[1] = (0x2<<GPIO_AFRH_AFSEL12_Pos); // AFR[1]=  For pins 8..15   10:AF2(TIM3..5)  ..  12.pin alternate function ile Tim4 iliskilendirilmesi yapildi.
	
		    SET_BIT(RCC->APB1ENR,RCC_APB1ENR_TIM4EN);
	
	     TIM4->CCMR1 = TIM_CCMR1_OC1PE | (0x06 <<TIM_CCMR1_OC1M_Pos); // 110: pwm mode 1 
	/* 
	  OC1PE: Output compare 1 preload enable
		0: Preload register on TIMx_CCR1 disabled. TIMx_CCR1 can be written at anytime, the new value is taken in account immediately.
    1: Preload register on TIMx_CCR1 enabled. Read/Write operations access the preload register. TIMx_CCR1 preload value is loaded in the active register at each update event. 
	*/
		
				TIM4->CCER=TIM_CCER_CC1E;  // 1: On - OC1 signal is output on the corresponding output pin
	
				TIM4->PSC=42-1;
	
				TIM4->ARR=1000;
	
				TIM4->CCR1=250;  // %25 duty-cycle
	
				TIM4->CR1 |=0X0001;
		

}



	  uint32_t  ch1_rising,ch2_falling;  // ch1_rising periyodu saymakta. ch2_falling ise pwm in on olma süresi.
		float pwm_freq, pwm_duty_cycle;


int main(void)
{

	setClock();  // clock ayarlari eklendi. kitapta yaziyor ayrintisi.
	
	Tim2_InputCapture_CH1_CH2_Init();
	Tim4_PWM_CH1_Init();
	
	
	
	while(1)
	{
		
		if( ((TIM2->SR & TIM_SR_CC1OF)==TIM_SR_CC1OF) & ((TIM2->SR & TIM_SR_CC2OF)==TIM_SR_CC2OF))
		{
			ch1_rising= TIM2->CCR1;
			ch2_falling=TIM2->CCR2;
			
			pwm_freq=	(1/(float)ch1_rising)* (84000000/(TIM2->PSC+1));
			
			pwm_duty_cycle= ((float)ch2_falling/(float)ch1_rising)*100;
		
		
		}
	 
	
	}
	

}








void setClock(void) //keil ile olusturulan bos projelerde versiyonuna bagli olarak clock ayari fonksiyonu bulunmamaktadir.
{
/******************************************************************************/
/*            PLL (clocked by HSE) used as System clock source                */
/******************************************************************************/
  #if !defined  (HSE_STARTUP_TIMEOUT) 
  #define HSE_STARTUP_TIMEOUT    ((uint16_t)0x0500)   /*!< Time out for HSE start up */
#endif /* HSE_STARTUP_TIMEOUT */ 
	
	uint16_t PLL_M=8, PLL_P=2,PLL_Q=4,PLL_N=336;
	
	__IO uint32_t StartUpCounter = 0, HSEStatus = 0;
  
  /* Enable HSE */
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
 
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
    StartUpCounter++;
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }

  if (HSEStatus == (uint32_t)0x01)
  {
    /* Select regulator voltage output Scale 1 mode, System frequency up to 168 MHz */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    /* HCLK = SYSCLK / 1*/
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
      
    /* PCLK2 = HCLK / 2*/
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
    
    /* PCLK1 = HCLK / 4*/
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

    /* Configure the main PLL */
    RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
                   (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

    /* Enable the main PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till the main PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }
   
    /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
    FLASH->ACR = FLASH_ACR_PRFTEN |FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;

    /* Select the main PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    /* Wait till the main PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL);
    {
			
    }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock
         configuration. User can add here some code to deal with this error */
  }




SystemCoreClockUpdate();


}

