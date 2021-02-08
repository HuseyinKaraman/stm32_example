/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WHO_AM_I (0x0F)  // LIS3DSH register adresleri
#define CTRL_REG3 (0x23)
#define CTRL_REG4 (0x20)
#define CTRL_REG5 (0x24)
#define CTRL_REG6 (0x25)
#define FIFO_CTRL (0x2E)
#define OUT_X (0x29)
#define OUT_Y (0x2B)
#define OUT_Z (0x2D)
#define FIFO_SRC (0x2F)
#define INFO1 (0x0D)         // who_am_i gibi sabit degerli register.
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

int8_t x,y,z,t;
uint8_t status,status_1;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void SPI_LIS3DSH_Tx(uint8_t adr,uint8_t data)    //  7. bitin basina 0 degeri koyuyoruz ki okuma yapalim yani sadece adres degeri gidicek => WAAA AAAA       A=> Adres demek!  R=> Read W=>Write
{																																				     //Read için =>1 / Write için =>0 olmali MSB degeri.
	
HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);  // Modulun CS pini haberlesmeyi baslatabilmesi çin Low yapiliyor.
	
HAL_SPI_Transmit(&hspi1,&adr,1,10); // &adr (Adresin bulundugu yerin adresini gönderdik.) , 1 byte , timeout (milisaniye)
	
HAL_SPI_Transmit(&hspi1,&data,1,10); // &data (adr'ye yazilacak olan data gönderilir.) , 1 byte , timeout (milisaniye)

HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);   // Yazma islemi gerçeklestigi için CS pini tekrar HIGH yapiliyor.

}


uint8_t SPI_LIS3DSH_Rx(uint8_t adr)
{
	
uint8_t rx=0;      
adr|=0x80;  // 7. bitin basina 1 degeri koyuyoruz ki okuma yapalim  => RAAA AAAA    A=> Adres demek!  R=> Read W=>Write  |  Read için =>1 / Write için =>0 olmali MSB degeri. 
	
HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);   // Modulun CS pini haberlesmeyi baslatabilmesi çin LOW yapiliyor.

HAL_SPI_Transmit(&hspi1,&adr,1,10);       // &adr (Adresin bulundugu yerin adresini gönderdik.) , 1 byte , timeout (milisaniye)

HAL_SPI_Receive(&hspi1,&rx,1,10);      // &rx (rx'e adresten gelen deger kaydedilir.       // (gelen data rx'e yazilacak) , 1 byte , timeout (milisaniye)
	
HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);   // Yazma islemi gerçeklestigi için CS pini tekrar HIGH yapiliyor.
	
return rx;

}

void SPI_LIS3DSH_Init()
{


 SPI_LIS3DSH_Tx(CTRL_REG6,0x80);   // 0x80 yazilarak REBOOT(Yenidenbaslatma) YAPILDI.
	
 SPI_LIS3DSH_Tx(CTRL_REG3,1);  // 1 yazilarak yazilimsal reset atiliyor.

 HAL_Delay(100);
	
	if(SPI_LIS3DSH_Rx(INFO1)==0x21)     // haberlesmeyi test etmek için cevabi sabit olan who_am_i veya INFO adresi okunuyor.
	{
		SPI_LIS3DSH_Tx(CTRL_REG3,0x48);  // ctrl_reg3 adresine 0x48 degeri yazilarak Interrupt kutbu High yapiliyor ve INT1 pinin interrupt aktiflestirilmesi yapiliyor.
	
		SPI_LIS3DSH_Tx(CTRL_REG4,0x47); // ctrl_reg4 adresine 0x47 degeri yazilarak Çikis data ölçegi ayarlaniyor ve x,y,z, eksenleri aktiflestiriliyor.
		
		SPI_LIS3DSH_Tx(CTRL_REG5,0x48); // ctrl_reg5 adresine 0x48 degeri yazilarak tam ölçek secenegi ve filtre ayari yapiliyor.
		
		SPI_LIS3DSH_Tx(CTRL_REG6,0x64); // ctrl_reg6 adresine 0x64 degeri yazilarak FIFO ve FIFO derinligi ayarlanabildigi watermak secenegi aktiflestiriliyor.INT1 pini FIFO watermak interrupt secenegi aktiflestiriliyor
		
		SPI_LIS3DSH_Tx(FIFO_CTRL,0x41); // Fifo_ctrl adresine 0x41 degeri yazilarak FIFO akis modunda aktiflestiriliyor. 
										//FIFO boyutu olan watermak degerine 1 giriliyor... Devami önemli KITAP=>556 
                                     //		Stream mode. If the FIFO is full, the new sample overwrites the older one.
	}
}



/*  Bildigimiz üzere harici kesme herhangi bir pini(Bu özelligi olan pinler) High to low yada Low to high geçisini algilayarak olusturulan kesmeye denir. 
    Bu pin logic konumda olmalidir.Ayarlarimiza göre bu pini lojik 0 dan lojik 1’E geçtigi anda kesme üretebilecek sekilde kontrol edebiliriz.
    Buton okuma islemi gibi düsünebiliriz ama butonu sürekli kontrol etmemize gerek yoktur çünkü herhangi bir geçiste otomatik kesme üretir
*/
void EXTI0_IRQHandler(void)    // INT1/DRDY' ye PE0 pini bagli tetikleme gelince pine interrupt üretecek!
{
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0); //  ilgili bayragi temizledik.
	
	status= SPI_LIS3DSH_Rx(FIFO_SRC); // yorum idi
	
	x=SPI_LIS3DSH_Rx(OUT_X);
	
	y=SPI_LIS3DSH_Rx(OUT_Y);
	
	z=SPI_LIS3DSH_Rx(OUT_Z);

	 status_1=SPI_LIS3DSH_Rx(FIFO_SRC); // yorum idi
	 
	if(x<-10) HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
	else HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
	
	if(x>10) HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
	else HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
	
	if(y<-10) HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
	else HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
	
	if(y>10) HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
	else HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
	

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
 __HAL_SPI_ENABLE(&hspi1);      // Spi'i enable ettik.
  /* USER CODE END 2 */
   
   SPI_LIS3DSH_Init(); // Kitap'ta ayrintili bi anlatim var ozellikle watermark ile ilgili.*************************************************
   
   /*  	  *********************Sensoru baslattik ve Sonra  Who_Am_I' in adresini gönderip onun tuttugu degeri aldik: ****************************
								Baslatmadan yapinca hata verdi!

    SPI_LIS3DSH_Tx(CTRL_REG6,0x80); 
	
	SPI_LIS3DSH_Tx(CTRL_REG3,1);

	HAL_Delay(100);
	
	t=SPI_LIS3DSH_Rx(WHO_AM_I);
	
	 ***************************************************************************************************
*/
  
   
    
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;            
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

 
  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
