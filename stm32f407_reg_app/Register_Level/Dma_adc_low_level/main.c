#include<stm32f4xx.h> // Gerekli sistem kutuphanesi
/* Syf=> 576 da.....*/  // uygulama_13 LOW LEVEL

void setClock(void) ;



void DMA_Init(uint16_t * adr,uint8_t lenght)    // adr parametresi DMA biriminde memory adresine yazilacaktir.    // syf=>578
{												// lenght parametresi transfer isleminin kaç defa yapilacagi bilgisidir.
	

	SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_DMA2EN);  // DMA2 init fonksiyonu

	DMA2_Stream0->CR  = (0x01 << DMA_SxCR_MSIZE_Pos);   // hafiza data genisligi seçme biti                   // Half word seçildi.
	DMA2_Stream0->CR |= (0x01 << DMA_SxCR_PSIZE_Pos);  //çevresel birim data genisligi seçme biti            // Half word seçildi.
	DMA2_Stream0->CR |= (DMA_SxCR_MINC);  // hafiza adresi arttirma modu           // her transferden sonra degeri 1 artacaktir
	DMA2_Stream0->CR |= (DMA_SxCR_CIRC);   // DMA2 Stream0 için circular mod aktiflestirildi. 
	
	DMA2_Stream0->PAR = (uint32_t) &ADC1->DR;    /*!< PAR => DMA stream x peripheral address register */  // =>Çevresel birim adresi registerlari ,burada=>ADC1 biriminin data registeri olarak girildi

	DMA2_Stream0->M0AR = (uint32_t) adr ;      /*!< MOAR=> DMA stream x memory 0 address register   */ // => Hafiza 0. adresi registerlari // Örnekte adc'nin degerini tutacak olan adresi M0AR'a atayacagiz.
																																		  // hafiza adresine adr degiskeni giriliyor
	DMA2_Stream0->NDTR = lenght;      // Data sayisi Registerlari, Transfer edilecek data sayisi bitleri.Her transferden sonra bu sayi dusmektedir.
}

// Not : ADC1 birimi 0. Kanal'da oldugundan dolayi DMA ayarlari yapilirken girilmemistir.Baslangiç degeri olarak 0. kanal olarak seçilir.

void ADC_Init()
{
	

	SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOAEN);		//A portunu clock hatti aktif edildi.
	
	GPIOA->MODER |= (0x03<<GPIO_MODER_MODE1_Pos) |         		 /*!< GPIO port mode register  */
					(0x03<<GPIO_MODER_MODE2_Pos) | 				//A portunun  1,2,3,4 . Pinleri analog giris olarak ayarlandi.
					(0x03<<GPIO_MODER_MODE3_Pos) |
					(0x03<<GPIO_MODER_MODE4_Pos);

	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_ADC1EN);    // ADC1 birimini clock hatti aktiflestirildi.
	
	ADC1->CR1 |= ADC_CR1_SCAN;         // ADC1 birimi scan modda ayarlaniyor.
		
	ADC1->CR2 |= ADC_CR2_EOCS |ADC_CR2_DDS | ADC_CR2_DMA | ADC_CR2_CONT;  //ADC1 biriminde çevrim tmmlandi bayragi her kanaldan sonra gerçeklessin secenegi aktiflestiriliyor(EOCS),DMA aktiflestiriliyor.
																		  // Continuous modu aktiflestirildi.
	
	ADC1->SQR1 = (0x03 << ADC_SQR1_L_Pos); // Regular grupta kullanilacak , çevrime girecek kanal sayisi.

	ADC1->SQR3  = (0x01<<ADC_SQR3_SQ1_Pos);  /* (1st conversion in regular sequence) */        // ilk 4 posizyona kanal numaralari giriliyor.
	ADC1->SQR3 |= (0x02<<ADC_SQR3_SQ2_Pos);  /* (2st conversion in regular sequence) */
	ADC1->SQR3 |= (0x03<<ADC_SQR3_SQ3_Pos);  /* (3st conversion in regular sequence) */
	ADC1->SQR3 |= (0x04<<ADC_SQR3_SQ4_Pos);  /* (4st conversion in regular sequence) */
	
}

void Start_ADC_DMA ()
{
	

	DMA2_Stream0->CR |= DMA_SxCR_EN;           // DMA2 Stream0 aktiflestiriliyor.
	
	ADC1->CR2 |= ADC_CR2_ADON;      /*!<A/D Converter ON / OFF */        // ADC1 aktiflestiriliyor.
	
	ADC1->CR2 |= ADC_CR2_SWSTART;     /*!<Start Conversion of regular channels */     //ADC1 regular kanal çevrimi baslatiliyor.Continuous mod aktif old'dan çevrim sürekli devam edecektir.	


}

void Stop_ADC_DMA ()
{
	

 DMA2_Stream0->CR &= ~DMA_SxCR_EN;      // DMA2 Stream0 pasif yapiliyor. 
	
 ADC1->CR2 &= ~ADC_CR2_ADON;       // ADC1 pasif yapiliyor. 

}


uint16_t adc_values[4];

int main(void)
{

	setClock();  // clock ayarlari eklendi. kitapta yaziyor ayrintisi.
	
	ADC_Init();
	
	DMA_Init(adc_values,4);
	
	Start_ADC_DMA();

	
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

