// uygulama_14 LOW LEVEL

#include<stm32f4xx.h>  // gerekli sistem kütüphanesi


uint32_t sector1=0x8004000;

void setClock(void) ;

void FLASH_Unlock(void)
{
  if(READ_BIT(FLASH->CR,FLASH_CR_LOCK) != RESET)
  { 
	/*The following values must be programmed consecutively to unlock the FLASH_CR register and allow programming/erasing it: */ 
	WRITE_REG(FLASH->KEYR,0x45670123); // a) KEY1 = 0x45670123 b) KEY2 = 0xCDEF89AB
	WRITE_REG(FLASH->KEYR,0xCDEF89AB);
  }

}

void FLASH_Lock(void)
{
 
	FLASH->CR |=FLASH_CR_LOCK;  // Write to 1 only. When it is set, this bit indicates that the FLASH_CR register is locked.

}

static void FLASH_Program_Word(uint32_t Address,uint32_t Data)
{
	CLEAR_BIT(FLASH->CR,FLASH_CR_PSIZE);
	
	FLASH->CR |=(0x02 <<FLASH_CR_PSIZE_Pos);  // PSIZE[1:0]: Program size  => 10: program x32
	
	FLASH->CR |=FLASH_CR_PG;  // PG: Programming => Flash programming activated
	
	*(__IO uint32_t*)Address= Data;
	
}


void FLASH_Erase_Sector(uint32_t sector_number)
{
	CLEAR_BIT(FLASH->CR, FLASH_CR_SNB);

	FLASH->CR |= FLASH_CR_SER | (sector_number<<FLASH_CR_SNB_Pos); // SER: Sector Erase (Sector Erase activated) | SNB[3:0]: Sector number
	
	FLASH->CR |= FLASH_CR_STRT; // STRT: Start=> This bit triggers an erase operation when set.
}


uint32_t flash_value[2];

int main(void)
{

	setClock();  // clock ayarlari eklendi. kitapta yaziyor ayrintisi.

  FLASH_Unlock();
 
  FLASH_Program_Word( sector1 ,0x12345678);
 
  FLASH_Program_Word(sector1+4 ,0x10203040);
  
  flash_value[0]= *(uint32_t *)(sector1);
  
  flash_value[1]= *(uint32_t *)(sector1+4);
  
  FLASH_Erase_Sector(1);
  
  FLASH_Lock();
	
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

