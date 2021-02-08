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

/* USER CODE BEGIN Includes */
 #define mpu6050address 0xD0
 #define REG_CHIP_ID_1  0x75   // who_am_i
 #define REG_CHIP_ID_2  0x1C   // acce_config
 // datasheetteki adres kendimiz manuelde bulabiliriz.       
//  slave cihazin adresi 0X68 NORMALDE

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

uint8_t i2cTx[2];
uint8_t i2cRx[2];
int16_t ax,ay,az;
//float Xaccel,Yaccel,Zaccel;
uint32_t i=0;
uint32_t a=0;
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
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
 
// // 1. Scan the I2C addresses   // datasheet yazanin 2 kati kodda saga kaydirilmasi gerektigi yaziyor.
//for( i=0;i<255;i++)    
//{
//  if(HAL_I2C_IsDeviceReady(&hi2c1,i,1,10)==HAL_OK) 
//		// manuel olarak adresi belirlemeye çalisiyoruz.
//	{
//			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
//	}
//} //belirlendi....

		if(HAL_I2C_IsDeviceReady(&hi2c1,0xD0,2,100)==HAL_OK) // cihazi test ediyoruz çalisiyormu diye.
		{
			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
		}

  
		
		HAL_I2C_Mem_Read(&hi2c1,mpu6050address,REG_CHIP_ID_1,1,i2cRx,1,100); // çalisti yazmayi deneyelim : 
		HAL_Delay(20);
		
		
		i2cTx[0]=0x08;
	  HAL_I2C_Mem_Write(&hi2c1,mpu6050address,REG_CHIP_ID_2,1,(uint8_t*)i2cTx,1,100);
		HAL_Delay(20);
	 	HAL_I2C_Mem_Read(&hi2c1,mpu6050address,REG_CHIP_ID_2,1,&i2cRx[1],1,100); 
  









//		//2. I2C Write Example     // register a deger yazamadim sikinti orda ......
//   // a.Set accelerometer range (reg28)
//	 i2cTx[0] =28; // register address: Accelerometer config 1
//   i2cTx[1]=0x08;  // data to write , +-4g range 
//	 HAL_I2C_Master_Transmit(&hi2c1,0xD0,i2cTx,2,100 );
//	 HAL_Delay(20);
//		
//	 // 3. I2C Read example:üstekini oku 
//   // a.request to read from a register (reg75)
//	 HAL_I2C_Master_Transmit(&hi2c1,0xD0,i2cTx,1,100);
//   // b.read data:
//	 HAL_Delay(20);
//   i2cRx[0]=0x00; // 0 olmali 
//	 HAL_I2C_Master_Receive(&hi2c1,0xD0,i2cRx,1,100);
//	 HAL_Delay(20);	
//		
///****************************************************** */	
//		
//   // 3. I2C Read example: Who_am_i
//   // a.request to read from a register (reg75)
//   i2cTx2[0]=117; //register address accelerometer config 1 
//   HAL_I2C_Master_Transmit(&hi2c1,0xD0,i2cTx2,1,100);
//   // b.read data:
//	 HAL_Delay(20);
//   i2cRx2[0]=0x00; // 0 olmali 
//   i2cRx2[1]=0x00;
//	 HAL_I2C_Master_Receive(&hi2c1,0xD0,&i2cRx2[1],2,100);
/////****************************************************** */
//	
  
	 
	     

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//  /* USER CODE END WHILE */
//  // 3. I2C Read example
//   // a.request to read from a register (reg59)
//   i2cBuf[0]=59; //register address X_axis H 
//   HAL_I2C_Master_Transmit(&hi2c1,mpu6050address,i2cBuf,1,1000);
//   // b.read data:
//   i2cBuf[1]=0x00; // 0 olmali 
//   HAL_I2C_Master_Receive(&hi2c1,mpu6050address,&i2cBuf[1],6,1000);

//		ax = -(i2cBuf[1]<<8 | i2cBuf[2]);
//		ay = -(i2cBuf[3]<<8 | i2cBuf[4]);
//		az = -(i2cBuf[5]<<8 | i2cBuf[6]);
//		
//		Xaccel = ax/8192.0;
//		Yaccel = ay/8192.0;
//		Zaccel = az/8192.0;
//  /* USER CODE BEGIN 3 */
//HAL_Delay(300);
  }
  /* USER CODE END 3 */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
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
