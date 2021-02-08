#include<stm32f4xx.h>  // uygulama_4
#include "delay.h" // uygulama2'de olusturuluyor


uint16_t channel_1,channel_2,channel_3; // kanal1, kanal2 , kanal3 adc_data       data_degerleri bu degiskenlere atanacak.

float channel_1_voltage;	//kanal_1 adc_voltage   voltaj degeri atanacak degisken 


void ADC_IRQHandler(void)				
{
  if( (ADC1->SR & ADC_SR_JEOC) !=0 )		// interrupt olusunca içine gir.
	{
		channel_1=ADC1->JDR3;  // JQS4 REG 'ine yazilan 1. kanalin datasi JDR3 registerindan degiskene yazildi.
		channel_1_voltage=((float)channel_1*3)/4095;
		
		channel_2=ADC1->JDR2; // JQS3 REG 'ine yazilan 2. kanalin datasi JDR3 registerindan degiskene yazildi.
		
		channel_3=ADC1->JDR1;  // JQS2 REG 'ine yazilan 3. kanalin datasi JDR3 registerindan degiskene yazildi.
		
		CLEAR_BIT(ADC1->SR,ADC_SR_JEOC); // interrupt biti temizleniyor.
	
	}

}

void setClock(void);



int main(void)
{

	setClock();  // clock ayarlari eklendi. kitapta yaziyor ayrintisi.

	SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOCEN); //AHB1ENR hattinda olan GPIO_C portu aktif
	
	GPIOC->MODER|= (0x03 << GPIO_MODER_MODE0_Pos) | (0x03 <<GPIO_MODER_MODE1_Pos) | (0x03 <<GPIO_MODER_MODE2_Pos);   // moder (0b11) -> analog mod olarak ayarliyor.PC0,PC1,PC2
	
	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_ADC1EN);   // adc clock hatti aktif
	ADC1->CR1 |= ADC_CR1_SCAN | ADC_CR1_JEOCIE; // scan mod(1 den fazla kanal kullanilmasi durumunda aktif edilmeli) ve injected in çevrim sonu interrupt i aktif ediliyor.
	ADC1->JSQR|= ADC_JSQR_JL_1;        // 3 kanal kullanilacagi bildiriliyor.      When JL=2 (3 injected conversions in the sequencer), the ADC converts the channels in the following order: JSQ2[4:0], JSQ3[4:0], and JSQ4[4:0].
	ADC1->JSQR|= (10<<ADC_JSQR_JSQ4_Pos) | (11<<ADC_JSQR_JSQ3_Pos) | (12<<ADC_JSQR_JSQ2_Pos);  // (0....18) arasi sayi yazilir reg'e .JSQ4 E 10 nolu pini ,JSQ3 E 11 i,JSQ2 YE 12 NOLU pini atadik :)
	// ADC_JSQR_JSQ4_Pos         (15U)     jsq4 15 . bitte
	// ADC_JSQR_JSQ3_Pos         (10U)      jsq3 10. bitte baslio vs vs ..
			
	
	NVIC_EnableIRQ(ADC_IRQn);   // NVIC->ISER[18>>5]=1<<(18 & 0x1F);            //interrupt aktif.
	
	SysTick_Config(SystemCoreClock/1000); // systick 1ms ile ayarlandi. // interrupt aktif edilir. //interrupt'a öncelik verilir 
	
	ADC1->CR2 |= ADC_CR2_ADON;           // adc1 birimini aktiflestirmek için adon biti 1 yapilir.
	
	
	
	while(1)
	{
	 
	 delay_ms(500);  
	 ADC1->CR2 |= ADC_CR2_JSWSTART;         // injected mod çevrimi starti veriliyor.
	
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

