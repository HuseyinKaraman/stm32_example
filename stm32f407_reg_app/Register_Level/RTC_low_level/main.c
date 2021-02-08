#include<stm32f4xx.h>  // uygulama_10 LOW LEVEL

/* RTC Ay Tanimlamalari (BCD): */
#define OCAK 		((uint8_t) 0x01)
#define SUBAT 		((uint8_t) 0x02)
#define MART		((uint8_t) 0x03)
#define NISAN 		((uint8_t) 0x04)
#define MAYIS 		((uint8_t) 0x05)
#define HAZIRAN		((uint8_t) 0x06)
#define TEMMUZ 		((uint8_t) 0x07)
#define AGUSTOS 	((uint8_t) 0x08)
#define EYLUL 		((uint8_t) 0x09)
#define EKIM 		((uint8_t) 0x10)
#define KASIM 		((uint8_t) 0x11)
#define ARALIK 		((uint8_t) 0x12)



/* RTC Hafta Tanimlamalari (BCD): */
#define PAZARTESI 	((uint8_t) 0x01)
#define SALI 		((uint8_t) 0x02)
#define CARSAMBA 	((uint8_t) 0x03)
#define PERSEMBE 	((uint8_t) 0x04)
#define CUMA 	    ((uint8_t) 0x05)
#define CUMARTESI 	((uint8_t) 0x06)
#define PAZAR	    ((uint8_t) 0x07)



void RTC_init()
{
    PWR->CR |=PWR_CR_DBP;     /*!< Disable Backup Domain write protection     */
	
	RCC->CSR |=RCC_CSR_LSION;  // Bit 0 LSION: Internal low-speed oscillator enable
	
	while((RCC->CSR & RCC_CSR_LSIRDY) != RCC_CSR_LSIRDY);   // Bit 1 LSIRDY: Internal low-speed oscillator ready
															// Hazir olana kadar bekle.(LSI'nin hazir olmasini bekliyoruz)
	
	RCC->BDCR |=RCC_BDCR_BDRST ; // Bit 16 BDRST: Backup domain software reset
	RCC->BDCR &= ~RCC_BDCR_BDRST;  // 0: Reset not activated | 1: Resets the entire Backup domain

	
	RCC->BDCR |=RCC_BDCR_RTCEN |(0x02<<RCC_BDCR_RTCSEL_Pos);    // (RTCEN: RTC clock enable) | (RTCSEL[1:0]: RTC clock source selection => 10: LSI oscillator clock used as the RTC clock) ;
	
	RTC->WPR= 0xCA;
	RTC->WPR= 0x53; // keyleri girerek korumayi kaldiriyoruz.
	
	RTC->ISR |=RTC_ISR_INIT; 			 /* Bit 7 INIT: Initialization mode
										0: Free running mode
										1: Initialization mode used to program time and date register (RTC_TR and RTC_DR), and
										prescaler register (RTC_PRER). Counters are stopped and start counting from the new
										value when INIT is reset.*/
	
	while((RTC->ISR & RTC_ISR_INITF) != RTC_ISR_INITF);  /* INITF: Initialization flag
															When this bit is set to 1, the RTC is in initialization state, and the time, date and prescaler
															registers can be updated.  
															0: Calendar registers update is not allowed
															1: Calendar registers update is allowed.*/ 
	
	RTC->PRER = ((128-1)<<RTC_PRER_PREDIV_A_Pos) | ((250-1)<<RTC_PRER_PREDIV_S_Pos);
	
	RTC->ISR &= ~RTC_ISR_INIT;
	
	RTC->WPR=0x23;  // geçersiz key girilerek tekrar kilitlendi . 
	
	
}


void Set_Time_BCD(uint8_t saat,uint8_t dakika,uint8_t saniye)
{
	RTC->WPR= 0xCA;
	RTC->WPR= 0x53;

	
	RTC->ISR |=RTC_ISR_INIT;
	
	while((RTC->ISR & RTC_ISR_INITF) != RTC_ISR_INITF);
	
	RTC->TR=(saat<<RTC_TR_HU_Pos) |(dakika<<RTC_TR_MNU_Pos)|(saniye<<RTC_TR_SU_Pos);
	
	RTC->ISR &=~RTC_ISR_INIT;

	RTC->WPR=0x23;
		
	
}

void Set_Date_BCD(uint8_t yil, uint8_t ay,uint8_t gun, uint8_t hafta_gun)
{
    RTC->WPR= 0xCA;
	RTC->WPR= 0x53;

	
	RTC->ISR |=RTC_ISR_INIT;
	
	while((RTC->ISR & RTC_ISR_INITF) != RTC_ISR_INITF);
	
	RTC->DR=(yil<<RTC_DR_YU_Pos) |(ay<<RTC_DR_MU_Pos)|(gun<<RTC_DR_DU_Pos)|(hafta_gun<<RTC_DR_WDU_Pos);
	
	RTC->ISR &=~RTC_ISR_INIT;

	RTC->WPR=0x23;
		
	
}


void Set_Alarm_A_BCD(uint8_t gun,uint8_t saat,uint8_t dakika,uint8_t saniye)
{
	RTC->WPR= 0xCA;
	RTC->WPR= 0x53;

	
	RTC->ISR |=RTC_ISR_INIT;
	
	while((RTC->ISR & RTC_ISR_INITF) != RTC_ISR_INITF);
	
	RTC->CR &= ~RTC_CR_ALRAE;

	RTC->ALRMAR=(gun<<RTC_ALRMAR_DU_Pos) 
				|(saat<<RTC_ALRMAR_HU_Pos)
				|(dakika<<RTC_ALRMAR_MNU_Pos)
				|(saniye<<RTC_ALRMAR_SU_Pos);
	
	RTC->CR |= RTC_CR_ALRAE;
	
															/*  ALRAWF: Alarm A write flag
															This bit is set by hardware when Alarm A values can be changed, after the ALRAE bit has
															been set to 0 in RTC_CR
															0: Alarm A update not allowed 
															1: Alarm A update allowed*/ 
	if((RTC->ISR & RTC_ISR_ALRAWF)==RTC_ISR_ALRAWF) 														
	{
		RTC->CR |=RTC_CR_ALRAIE;
		NVIC_EnableIRQ(RTC_Alarm_IRQn);
		EXTI->IMR |=EXTI_IMR_IM17;
		EXTI->RTSR|=EXTI_RTSR_TR17;
	}
	
	RTC->ISR &=~RTC_ISR_INIT;

	RTC->WPR=0x23;
	
}


void 	RTC_Alarm_IRQHandler(void)
{
	
	
   if((RTC->ISR & RTC_ISR_ALRAF)==RTC_ISR_ALRAF)
   {
	   
  
	   RTC->ISR &=~RTC_ISR_ALRAF;  // Alarm A gerçeklesince Aktif Olur.
    
	   GPIOD->ODR = GPIO_ODR_OD12 | GPIO_ODR_OD13 | GPIO_ODR_OD14 | GPIO_ODR_OD15 ;	  	
	   
		   
	 }
    
	 
}

void setClock(void) ;



int main(void)
{

	setClock();  // clock ayarlari eklendi. kitapta yaziyor ayrintisi.
	
	SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIODEN);  // VEYA  RCC->AHB1ENR |=0x08;
	
	GPIOD->MODER |=(1<<GPIO_MODER_MODE12_Pos)
					|(1<<GPIO_MODER_MODE13_Pos)
					|(1<<GPIO_MODER_MODE14_Pos)
					|(1<<GPIO_MODER_MODE15_Pos);
	
	RTC_init();
	
	Set_Time_BCD(0x09,0x4,0x50);   // saat 9:04:50 
	
	Set_Date_BCD(0x18,KASIM,0x10,CUMARTESI);
	
	Set_Alarm_A_BCD(0x10,0x09,0x05,0x00);  // alarm 10. gun saat 9:05:00
	
	
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
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL)
    {
			
    }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock
         configuration. User can add here some code to deal with this error */
  }




SystemCoreClockUpdate();


}

