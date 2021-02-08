#include<stm32f4xx.h>  // uygulama_5 LOW LEVEL


void setClock(void) ;

uint16_t channel_2; // kanal_2 regular grup adc_data
float channel_2_voltage; // kanal_2 adc_voltage



void ADC_IRQHandler()
{

  channel_2= ADC1->DR;            //EOC biti=> This bit is set by hardware at the end of the conversion of a regular group of channels. It is cleared by software or by reading the ADC_DR register.
	
	
	channel_2_voltage=((float)channel_2*3)/4095;


}


void Adc_Init()
{

	SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOAEN); // AHB1 hattina bagli olan  Gpio_A clock hatti aktif edildi.
	
	GPIOA->MODER |=(0x03<< GPIO_MODER_MODE2_Pos);    // A2 yi analog olarak ayarladik.
	 
	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_ADC1EN);  // APB2 hattina bagli olan  ADC1 clock hatti aktif edildi.

	// adc1 ayarlari :
	
	ADC1->CR1 |= ADC_CR1_EOCIE ;   // interrupt enable . Regular channel end of conversion interrupt. 1: EOC interrupt enabled. An interrupt is generated when the EOC bit is set.
	
	ADC1->CR2 |=(0x01<<ADC_CR2_EXTEN_Pos) | (0x07 <<ADC_CR2_EXTSEL_Pos);      // 01: Trigger detection on the rising edge | 0111: Timer 3 CC1 event
	
	ADC1->SQR3 |= (0x02 <<ADC_SQR3_SQ1_Pos);     // Bits 4:0 SQ1[4:0]: 1st conversion in regular sequence      These bits are written by software with the channel number (0..18) assigned as the 16th in the conversion sequence.
	// ADC regular sequence register 3       1. çevrime kanal_2(pa2) yazildi. 
	 	
	
	ADC1->CR2 |=ADC_CR2_ADON;  //adc1'i aktif ettik
	
	NVIC_EnableIRQ(ADC_IRQn);  // adc_ interrupt i enable.

}

void Tim3_PWM_CH1_Init()
{

  SET_BIT(RCC->APB1ENR,RCC_APB1ENR_TIM3EN); // APB1 hattina bagli olan  TIM3 clock hatti aktif edildi.
	
	// tim3 ayarlar:
	
  TIM3->CR2 |= (0x04<<TIM_CR2_MMS_Pos);  //  100: Compare - OC1REF signal is used as trigger output // 1. kanal çikisi tetik olarak ayarladik.
	
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE 	|(0x06<<TIM_CCMR1_OC1M_Pos);              // Output Compare 1 Preload enable    | pwm_mode_1
	
	TIM3->CCER=TIM_CCER_CC1E;           // CC1 channel configured as output: 1: On - OC1 signal is output on the corresponding output pin
	// tim 3 biriminin 1. kanal çikisi aktiflestiriliyor.
	
	TIM3->PSC=42000-1;
	
	TIM3->ARR=48000;    // 48 bine kadar say dedim.  25 sn idi galiba . ayarlandi. 1 hz için farkli ayar lazim.
	
	TIM3->CCR1=24000; // %50 Duty;
	
	TIM3->CR1 |=0X0001;
	

}






int main(void)
{

	setClock();  // clock ayarlari eklendi. kitapta yaziyor ayrintisi.

	
	Adc_Init();
	Tim3_PWM_CH1_Init();
	  
  
	
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

