#include<stm32f4xx.h> // uyg_12 low level spi uygulamasi.

#define WHO_AM_I (0X0F)
#define CTRL_REG3 (0x23)
#define CTRL_REG4 (0x20)
#define CTRL_REG5 (0x24)
#define CTRL_REG6 (0x25)
#define FIFO_CTRL (0x2E) 
#define OUT_X (0x29)
#define OUT_Y (0x2B)
#define OUT_Z (0x2D)
#define FIFO_SRC (0x2F)



void SPI_LIS3DSH_Tx(uint8_t adr,uint8_t data){

	uint8_t void_Rx=0; // Modulun gondermedigi ama RXNE bayragi için hattan alinacak data için olusturulan degisken!
	 
	GPIOE->BSRR|=GPIO_BSRR_BR3;   // GPIO port bit set/reset register RESET. Modulun CS pini haberlesmeyi baslatabilmek için low yapiliyor.
	
	SPI1->DR=adr; // Gonderi yaparken 7.bit degeri 0 olarak gonderildigi için modulde yazma islemi gerceklesecektir......
	
	while(!(SPI1->SR & SPI_SR_RXNE));   /*!<RXNE=> Receive buffer Not Empty */ // Data  Gonderildikten sonra hattin okunmasi bekleniyor.RXNE bayragi set olana kadar sonsuz dongu içerisinde duruluyor.
	
	void_Rx=SPI1->DR; // gecersiz data degiskene aliniyor ve RXNE bayragi temizleniyor.
	
	SPI1->DR =data;
	
	while(!(SPI1->SR & SPI_SR_RXNE));
	
	void_Rx=SPI1->DR;
	
	GPIOE->BSRR|=GPIO_BSRR_BS3; // GPIO port bit set/reset register SET. // yazma bittigi için tekrar high.
}

uint8_t SPI_LIS3DSH_Rx(uint8_t adr){  // geri dönüs degeri  uint8_t

	adr|=0x80;  //Modulden okuma yapilacagi için=> 7. bitin degerini 1 yapiyoruz.
	
	uint8_t rx=0;
	
	GPIOE->BSRR|=GPIO_BSRR_BR3; // Modulun CS pini haberlesmeyi baslatabilmek için low yapiliyor.
	
	SPI1->DR=adr;
	
	while(!(SPI1->SR & SPI_SR_RXNE));		 /*Data gonderildikten sonra hattin okunmasi bekleniyor */
	
	rx=SPI1->DR;  // Gecersiz data degiskene aliniyor ve RXNE bayragi temizleniyor......
	
	SPI1->DR=0; // SPI1 data registerina 0 yaziliyor ve module gonderiliyor.Budata gondermenin amaci SPI1 biriminin 8 clock sinyali üretmesini saglamaktir.DEVAMI SAYFA =>564 de.
	
	while(!(SPI1->SR & SPI_SR_RXNE)); // hattin okunmasi bekleniyor.
	
	rx=SPI1->DR; // geçerli data.
	
	GPIOE->BSRR|=GPIO_BSRR_BS3; // Okuma bitti tekrar high yaptik.
	
	return rx;

}


void spi1_init(){
	
	// SCK,MISO,MOSI pinleri için A portu aktiflestiriliyor:
	
	SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOAEN);
	
	SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOEEN); // CS pini için E portu aktiflestiriliyor.
	
	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_SPI1EN); // SPI1 clock hatti aktiflestiriliyor.

	GPIOA->MODER|= (0x02<<GPIO_MODER_MODE5_Pos)	; // PA5,PA6,PA7 pinleri alternate modunda ayarlaniyor.
	GPIOA->MODER|= (0x02<<GPIO_MODER_MODE6_Pos)	; 
	GPIOA->MODER|= (0x02<<GPIO_MODER_MODE7_Pos)	;


	GPIOA->AFR[0] |=(0x05<<GPIO_AFRL_AFSEL5_Pos); // PA5,PA6,PA7 pinleri SPI1 ile bagdastiriliyor. SPI1 AFR registerinda AF5'e tanimlidir.  /* SPI1_SCK */
	GPIOA->AFR[0] |=(0x05<<GPIO_AFRL_AFSEL6_Pos); 	/* SPI1_MISO */
	GPIOA->AFR[0] |=(0x05<<GPIO_AFRL_AFSEL7_Pos); 	/* SPI1_MOSI */	

	GPIOE->BSRR|=GPIO_BSRR_BS3;
	GPIOE->MODER|=(0x01<<GPIO_MODER_MODE3_Pos);
	
	SPI1->CR1= (SPI_CR1_SSM) |(SPI_CR1_SSI)             // NSS pin yonetimi yazilimsal yapilmaktadir.SSM  ve SSI bitleri set ediliyor.
				|(0x07<<SPI_CR1_BR_Pos)| (SPI_CR1_MSTR)  //Baud rate ayari için prescaler degeri 256(0x07) seçiliyor.Ve Spi1 master modunda seçiliyor.
				|(SPI_CR1_CPOL)|(SPI_CR1_CPHA);  // Spi1 CPOL ayari set edliyor ve clock hatti haberlesme hatti olmadigi zaman yuksek yapiliyor.
												// SPI1 CPHA biti set edilerek data yakalama islemi clock sinyalinin 2. kenarinda gerceklesmesi aktif ediliyor.
				
	SPI1->CR1 |=(SPI_CR1_SPE);	 // SPI1 aktif ediliyor.
	
	
}

void SPI_LIS3DSH_Init(){ // Kitap Sayfa => 565
												// kitap syfa => 263 ayrintili bilgi
	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_SYSCFGEN); //Modulun INT1 pini stm'in PE0 pinine baglidir. Bu pin EXTI uretmek için EXTI birimi ve SYSCFG birimi kullaniliyor. 
	                   
	// EXTI0 configuration SYSCFG_EXTICR1_EXTI0_PE=>/*!<PE[0] pin */  // 0. hat için PE0 ayarlamasi yapildi. 
	SYSCFG->EXTICR[0]|= (SYSCFG_EXTICR1_EXTI0_PE<<SYSCFG_EXTICR1_EXTI0_Pos); // External interrupt port ayarlama registeri 0.hat için  PE0 ayarlamasi yapildi.
	/*System configuration controller => EXTICR=> SYSCFG external interrupt configuration registers */
	
	// External Interrupt/Event Controller:  
	EXTI->IMR |= EXTI_IMR_IM0; // IMR;  EXTI Interrupt mask register,  External interrupt 0.hat aktiflesmesi için IMR registerinda 0. bite 1 yazilir. 
	
	EXTI->RTSR|=EXTI_RTSR_TR0; // RTSR; EXTI Rising trigger selection register, EXTI 0.hat yükselen kenarda tetiklenmesi ayarlandi.
	
	NVIC_EnableIRQ((EXTI0_IRQn)); // EXTI 0. hat Interrupti NVIC  biriminde aktiflestiriliyor. EXTI Line0 Interrupt
	
	SPI_LIS3DSH_Tx(CTRL_REG6,0x80); // reboot islemi yapiliyor.
	
	SPI_LIS3DSH_Tx(CTRL_REG3,1); // yazilimsal reset atiliyor.

	for(int i=0; i<0xFFFF;i++);
	
	if( SPI_LIS3DSH_Rx(WHO_AM_I)==0x3F )
	{                                        // açiklamalar sayfa 565.
		
		SPI_LIS3DSH_Tx(CTRL_REG3,0x48); // Interrupt kutbu yuksek yapiliyor ve INT1 pinin interrupt aktiflestirilmesi yapiliyor.
		
		SPI_LIS3DSH_Tx(CTRL_REG4,0x47); // Cikis data olcegi ayarlaniyor ve x,y,z eksenleri aktiflestiriliyor.
	
		SPI_LIS3DSH_Tx(CTRL_REG5,0x48); // tam ölçek ve filtre ayari yapiliyor.Bu komut modulun eksenleri okuma performasini arttirmak için yazilmistir. Tam ölçek ve filtre ayarlarini kavrayabilmek için ivme ölçum sistemlerinin arastirilmasi gerekmektedir. 
		
		SPI_LIS3DSH_Tx(CTRL_REG6,0x64); // FIFO  ve Fifo derinliginin ayarlanabildigi watermark secenegi aktiflestirildi.INT1 pini FIFO watermark interrupt secenegi aktif ediliyor.
		
		SPI_LIS3DSH_Tx(FIFO_CTRL,0x41); //FIFO akis modunda aktiflestiriliyor,FIFO boyutu olan watermark degerine 1 giriliyor.
										// BU degere 1 girilmesi bir defa x,y,z, eksenleri okundugunda FIFO watermark interrupti olacagi anlamina gelmektedir.
										// Watermark degerini 1'den farkli girerek interrupt olusma suresini ve interrupt iöerisinde FIFO genisligi kadar x,y,z, eksenlerinin datalarini okuyabilirler.
	
	}

	
}

int8_t x,y,z;
uint8_t status,status_1;

void EXTI0_IRQHandler(void){

	EXTI->PR |=EXTI_PR_PR0; // EXTI Pending register, EXTI biriminden EXTI0 bayragi temizleniyor. // Bu bit okunabilir , yazlnizca '1' yazilabilir
																								 //	 '1' yazldiktan sonra bit degeri temizlenir.	
	
	// status= SPI_LIS3DSH_Rx(FIFO_SRC);

	x=SPI_LIS3DSH_Rx(OUT_X);
	y=SPI_LIS3DSH_Rx(OUT_Y);
	z=SPI_LIS3DSH_Rx(OUT_Z);

	// status_1=SPI_LIS3DSH_Rx(FIFO_SRC);
	
	if(x<-10) GPIOD->BSRR= GPIO_BSRR_BS12;
	else GPIOD->BSRR= GPIO_BSRR_BR12;
	
	if(x>10) GPIOD->BSRR= GPIO_BSRR_BS14;
	else GPIOD->BSRR= GPIO_BSRR_BR14;
	
	if(y<-10) GPIOD->BSRR= GPIO_BSRR_BS15;
	else GPIOD->BSRR= GPIO_BSRR_BR15;
	
	if(y>10) GPIOD->BSRR= GPIO_BSRR_BS13;
	else GPIOD->BSRR= GPIO_BSRR_BR13;
}

void setClock(void) ;


int main(void) {



	setClock();


	SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIODEN);


	GPIOD->MODER|= (0x01<<GPIO_MODER_MODE12_Pos)  | 
	(0x01<<GPIO_MODER_MODE13_Pos)  | 
	(0x01<<GPIO_MODER_MODE14_Pos)  | 
    (0x01<<GPIO_MODER_MODE15_Pos); 
	
	spi1_init();
	
	SPI_LIS3DSH_Init();
	
	while(1){





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
