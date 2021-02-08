#include<stm32f4xx.h>  // uygulama_11 LOW LEVEL  TR.kitap sayfa=> 523

uint8_t data_array[2];
uint8_t chr_counter=0;

void uart5_send(uint8_t* data, uint8_t size)
{
	
	while(size>0)
	{
		size--;
		while( (UART5->SR & USART_SR_TXE) != USART_SR_TXE ) ; // TXE (Tx buffer bos) bayragi aktif olana kadar bekle 
		
		UART5->DR = *data++;
	
		while( (UART5->SR & USART_SR_TC) != USART_SR_TC ) ; // TC (transmit complete) bayragi 
	}


}


void UART5_IRQHandler()				// pc den 2-byte data paketi alinacak 
{
														
	if((UART5->SR & USART_SR_RXNE) == USART_SR_RXNE ) // RXNE(Rx buffer bos deil) interrupt olusma sebebi bumu kontorl ediliyor.
	{
	
		data_array[chr_counter++]=UART5->DR;
	
		if(chr_counter==(2) && (data_array[chr_counter-1]=='\n'))
		{
		
			GPIOD->ODR ^= ((1<<12) << ((data_array[0]-49) & 0x0F ));
			
			uart5_send((uint8_t *)"OK",2);
			uart5_send((uint8_t *)"\n",1);
			chr_counter=0;
		}
		else if(chr_counter>(2))
		{
			uart5_send((uint8_t *)"ERROR",5);
			uart5_send((uint8_t *)"\n",1);
			chr_counter=0;
		
		}
		
		
	
	}
}

void uart5_init()
{

	SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOCEN);
	SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIODEN);
	SET_BIT(RCC->APB1ENR,RCC_APB1ENR_UART5EN);
	
	GPIOC->MODER |= (0x2 << GPIO_MODER_MODE12_Pos);
	GPIOD->MODER |= (0x2 << GPIO_MODER_MODE2_Pos);
	
	GPIOC->AFR[1] |= (0x08 << GPIO_AFRH_AFSEL12_Pos);
	GPIOD->AFR[0] |= (0x08 << GPIO_AFRL_AFSEL2_Pos);

	UART5->BRR = 0x16D;   // sayfa 522' de hesaplanmasi. calc_uartdivide(42,115200);
    UART5->CR1 = USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE;
	UART5->CR1 |= USART_CR1_UE;
	
	NVIC_EnableIRQ(UART5_IRQn);
}

void setClock(void) ;



int main(void)
{

	setClock();  // clock ayarlari eklendi. kitapta yaziyor ayrintisi.
	
	SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIODEN);
	GPIOD->MODER |= (0x01<<GPIO_MODER_MODE12_Pos) |
					(0x01<<GPIO_MODER_MODE13_Pos) |
					(0x01<<GPIO_MODER_MODE14_Pos) |
					(0x01<<GPIO_MODER_MODE15_Pos);
	
	uart5_init();
	
			uart5_send((uint8_t *)"Merhaba!",8);
			uart5_send((uint8_t *)"\n",1);
	
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

