/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"            // bunuda ekledik ve neden ?
/* USER CODE BEGIN Includes */

// ADC KISMI:

#define VREFIN_Cal ((uint16_t*)((uint32_t)0x1FFF7A2A)) // Vrefin calibration degeri memoriden okunur
#define V25 (float)0.76     // volt
#define avg_slope (float) 0.0025   //    volt/Centigrade

uint16_t adc1_data[3],adc2_data[1];

float Vref=0,Vadc1_ch5=0,Vsense=0, temperature=0,Vadc2_ch1=0;           

// UART KISMI:
// hem transmit hem receive yapacagiz... 
// stmstduio da gözlemlemek için degilkenler yazdik...

char rx0,rx1,rx2,rx3,rx4,rx5,rx6,rx7,rx8,rx9;
char db0,db1,db2,db3,db4,db5,db6,db7,db8,db9;
char rx[10];
char rx_buffer[10],data_buffer[10];
char tx[50];
uint8_t x=0,i=0,len=0,y=0,indeks=0,z=0,a=0;
uint8_t newline[100],rx_full=0,rx_size=10,d1,d2,d3,d4,d5,d6,d7,d8,d9,d10;
// char* tx_buffer="tx value";
// char* rx_buffer;


/* USER CODE END Includes */
                                             
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/


 
 // ADC için : 
 
 void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) 
 {
		Vref=(float)3.3*(*VREFIN_Cal)/adc1_data[0];
	  Vadc1_ch5=Vref*adc1_data[1]/4095;
	  Vsense=Vref*adc1_data[2]/4095;
    Vadc2_ch1=Vref*adc2_data[0]/4095;

		temperature=(Vsense-V25)/avg_slope+25;
  
 }

 // uart Tx Kismi :
 
 // Herbir data iletildiginde Pin toggle olucak :) intterpt ile :)
 //  tx complete call back ine dallaniyor  
 // normal mod da yok :=>
 
// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
// {
//  if (huart->Instance==USART2)
//		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
//	
//	  i++;       // ilk örnekteki gönderme islemin kullaniliyor..
//	  if(i==4)
//		i=0;

// }

 // uart Rx Kismi : -- Aktif 
 // -ilk basta transmit yapiyoruz ilk örnekteki degerler ile 
  
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//  {
// 
//  if( huart->Instance==USART2 )
//	{
//		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
//		rx_full=1;
//	} // Rx in buffer'i doldugu zaman buraya dallaniyor..
//  
// 
// }
// 
 

// 2. örnek için rx comp. callback i 
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {
  if (huart->Instance==USART2)
	{
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
		
		rx_full=1;
		
	}

 }



/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
 // MX_ADC1_Init();
 // MX_ADC2_Init();
  MX_USART2_UART_Init();

//  /* USER CODE BEGIN 2 */
//HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc1_data,3); // start ettik.
//HAL_ADC_Start_DMA(&hadc2,(uint32_t*)adc2_data,1);





  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /////////////////////////SECOND EXAMPLE/////////////////////////////
		
		// ilk örnek diger projede yazili.....
		HAL_UART_Receive_IT(&huart2,(uint8_t*)rx_buffer, rx_size);
		HAL_Delay(100);
		
		len=huart2.RxXferSize-huart2.RxXferCount; // received buffer size
		
		
	  for(x=y;x<len;x++)
		{
		
			if((int)rx_buffer[x]==10)  // newline dedection / int e dönüstürdük.. chardi.
		{
		 
			indeks++; // received data number
			newline[indeks]=x; // rx_buffer indeks when newline is dedected.
// TR  // x+1 in nedeni mesela 3. dizi elemani newline (10) idi bir dahaki
			 // newline aramada x in önceki degerlerini degilde bir sonrakinden baslamasi
			 // için y=x+1 yapildi..
			y=x+1; // next scan indeks within the buffer by skipping 1 size
		
			
			for(a=0;a<rx_size;a++)
				data_buffer[a]=0; // make  the all data_buffer 0 before new data 
			
			a=0; // make the data_buffer indeks 0.
			newline[0]=0; // make the first newline indeks 0.
			
			if(indeks==1) // for first data 
			{
			  for(z=newline[indeks-1];z<newline[indeks];z++)
				{
					 data_buffer[a]=rx_buffer[z];
					 a++;
				}
			
			 }
			else // other data
			{ 
				for(z=newline[indeks-1]+1; z<newline[indeks]; z++)
				{
					 data_buffer[a]=rx_buffer[z];
					 a++;
				}
			 }
			
			 HAL_UART_Transmit_IT(&huart2,(uint8_t*)tx,sprintf(tx,"data%d=%s\n",indeks,data_buffer));
			 HAL_Delay(20); 
			 // sprintf ile  ("data%d=%s\n",indeks,data_buffer)  <=bu bilgi tx e aktarildi.
			 
			 rx_buffer[x]=0; //newline dedection indeks reset, newline tespit edilen eleman sifirlandi.
		 }
	 }
		 
		 
		  if(rx_full==1)
			{
			  indeks=0;
				x=0;
				y=0;
			
				for(int i=0;i<rx_size;i++)
				{
				 rx_buffer[i]=0;
				 newline[i]=0;
				
				}
				
				rx_full=0; // reset the rx_full....
				
			}
			
			d1=newline[0];
			d2=newline[1];
			d3=newline[2];
			d4=newline[3];
			d5=newline[4];
			d6=newline[5];
			d7=newline[6];
			d8=newline[7];
			d9=newline[8];
			d10=newline[9];
		
			
			rx0=rx_buffer[0];
			rx1=rx_buffer[1];
			rx2=rx_buffer[2];
			rx3=rx_buffer[3];
			rx4=rx_buffer[4];
			rx5=rx_buffer[5];
			rx6=rx_buffer[6];
			rx7=rx_buffer[7];
			rx8=rx_buffer[8];
			rx9=rx_buffer[9];
			
			
			
			
			
			
		  db0=data_buffer[0];
			db1=data_buffer[1];
			db2=data_buffer[2];
			db3=data_buffer[3];
			db4=data_buffer[4];
			db5=data_buffer[5];
			db6=data_buffer[6];
			db7=data_buffer[7];
			db8=data_buffer[8];
			db9=data_buffer[9];
			
			
			
			
			
			
			
			
  }
 

	
	
	
	
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
