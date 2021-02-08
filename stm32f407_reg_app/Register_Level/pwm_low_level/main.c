#include<stm32f4xx.h>  // uygulama_5 LOW LEVEL


void setClock(void) ;




int main(void)
{

	setClock();  // clock ayarlari eklendi. kitapta yaziyor ayrintisi.

	SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIODEN); // AHB hattina bagli olan  Gpio_D clock hatti aktif edildi.
	GPIOD->MODER|=GPIO_MODER_MODE12_1 | GPIO_MODER_MODE13_1 |GPIO_MODER_MODE14_1 |GPIO_MODER_MODE15_1;   // GPIOD portunun 12,13,14,15. pinler alternate moduna ayarlaniyor.
	GPIOD->AFR[1]= GPIO_AFRH_AFSEL12_1 |GPIO_AFRH_AFSEL13_1 |GPIO_AFRH_AFSEL14_1 |GPIO_AFRH_AFSEL15_1;   // 12 13 14 15 . pinler timer ile iliskilendirildi.AFR[1] AFR nin HIGH tarafini temsil ediyor. 
			// GPIO_AFRH_AFSELX_1 ise 0x02 ye esittir.Yani pine timer 4 ü bagla dedik.
	  
	
	SET_BIT(RCC->APB1ENR,RCC_APB1ENR_TIM4EN);  // APB1 clock hattinda Timer 4 birimi aktiflestiriliyor.
	
	TIM4->CCMR1= TIM_CCMR1_OC1PE | (0x06 << TIM_CCMR1_OC1M_Pos) ;    // Bits 6:4 OC1M'e  pwm mode-1 0x06 girilerek ayarlanir. OC1PE: Output Compare 1 Preload enable
	TIM4->CCMR1|= TIM_CCMR1_OC2PE | (0x06 << TIM_CCMR1_OC2M_Pos);   //  Bits14:12 OC2M[2:0]'e pwm mode-1 Bits 0x06 girilerek ayarlanir. OC2PE: Output Compare 2 Preload enable
																																		// 2 side CCMR1 registerinda bulunuyor.	 // cc1 cc2 çikis olarak ayarlanir. CC1S VE CC2S bitleri.
	
	TIM4->CCMR2 = TIM_CCMR2_OC3PE | (0x06 << TIM_CCMR2_OC3M_Pos); // Bits 6:4 OC3M'e  pwm mode-1 0x06 girilerek ayarlanir. OC3PE: Output Compare 3 Preload enable
	TIM4->CCMR2|= TIM_CCMR2_OC4PE | (0x06 << TIM_CCMR2_OC4M_Pos);  //  Bits 14:12 OC4M[2:0]'e pwm mode-1 Bits 0x06 girilerek ayarlanir. OC4PE: Output Compare 4 Preload enable
																															// 2 side CCMR2 registerinda bulunuyor.	 // cc3 cc4 çikis olarak ayarlanir. CC3S VE CC4S bitleri.
	
	TIM4->CCER= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E |TIM_CCER_CC4E ;   // TIM4 biriminin kanallarinin çikislari aktiflestiriliyor.
	
	TIM4->PSC=84-1;  // bölücü degeri . timerin Clock hizi 84Mhz  / bölücü degeri (PSC+1)=  1Mhz
	
	TIM4->ARR=1000; // auto reload degeri girildi.  PWM frekansini belirleyen deger budur.  fPWM = fCNT/fARR = 10 Khz. (100 ve 1000 denediö)
	
	TIM4->CCR1=50;  // % 75 DUTY-CYCLE (degistim.)
	TIM4->CCR2=800;  // % 50 DUTY-CYCLE
	TIM4->CCR3=500;  // % 25 DUTY-CYCLE
	TIM4->CCR4=250;  // % 10 DUTY-CYCLE
	
	TIM4->CR1|=0X0001;												// Sayaç aktiflestirme biti 
	
	while(1)
	{
		
		
	 
	
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

