#include "stm32f4xx.h" // gerekli sistem dosyalari



void TIM7_IRQHandler()
{
	TIM7->SR=0;			// status register flag biti=0; (yanlizca sifir yazilir.) (syf:206)
	GPIOD->ODR^=1<<12;  // her interrupta 12. bitin durumunu tersler.

}

void setClock(void);


int main(void)
{

setClock();

	
	SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIODEN); // RCC->AHB1ENR|=0x08;     (sfy:136)   	GPIOD portunun clock hatti aktif yapildi.
	GPIOD->MODER|=0x1000000;     // 6*4 =24 yani 0 ile 23 arasi  bit .. 24 ve 25 nolu bit 12. pine ait..12 pini çikis yaptik [01] çikis yapar.
															// 12. pin için bitler= (2y:2y+1 ) olmali.
	
	SET_BIT(RCC->APB1ENR,RCC_APB1ENR_TIM7EN); // RCC->APB1ENR|=1<<5; 			(syf:138)	  TIM7 biriminin clock hatti aktif yapildi. 
	
	TIM7->CR1=0x80;					// auto-reload preload (ön yukleme) degeri aktif
	
	TIM7->DIER=0x01;			// (Dma/Interrupt Enable Reg) Update intterrupt aktif edildi. (syf:205)
	
	TIM7->PSC=42000-1;		// psc degeri girildi. (syf:207)   okunabilir/yazilabilir.
	
	TIM7->ARR=2000-1;			// auto-reload degeri girildi.  okunabilir/yazilabilir.
	
	__NVIC_EnableIRQ(TIM7_IRQn); // NVIC ->ISER[55>>5]=1<<(55 & 0x1F);		// NVIC birimi ile tim7 vektoru aktiflestiriliyor. Tim7 interrupt pozisyon numarasi => 55 dir.
															// NVIC  fonksiyonlari syf=> 191.				Tim7 interrupt pozisyon numarasi => 55 dir.
	
	TIM7->CR1|=0x1;			// Tim7 kontrol reg den sayac aktiflestirme biti açiliyor ve tim7 sayaci saymaya basliyor.. (syf:204)
	
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