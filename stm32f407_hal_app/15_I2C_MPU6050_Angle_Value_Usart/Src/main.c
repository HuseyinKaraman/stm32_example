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
// eklenen libraryler:
#include "defines.h"
#include "tm_stm32_mpu6050.h"
#include <math.h> // önceki projeden farki .matematiksel hesaplamalar için....


/* USER CODE BEGIN Includes */
 float temperature=0;
 TM_MPU6050_t MPU6050_Sensor;
 char  data[120];
 
 unsigned long last_read_time;
 float        last_x_angle;  // these are the filtered angles 
 float        last_y_angle;
 float        last_z_angle;
 float        last_gyro_x_angle;  // Store the gyro angles to compare drift
 float        last_gyro_y_angle;
 float        last_gyro_z_angle;
 
 void set_last_read_angle_data(unsigned long time,float x,float y,float z, float x_gyro,float y_gyro,float z_gyro)
 {
  last_read_time= time;
	last_x_angle=x;
	last_y_angle=y;
  last_z_angle=z;
  last_gyro_x_angle=x_gyro;
  last_gyro_y_angle=y_gyro;
  last_gyro_z_angle=z_gyro;
	 
 }
 
 inline unsigned long get_last_time() {return last_read_time;}
 inline float get_last_x_angle() {return last_x_angle;}
 inline float get_last_y_angle(){return last_y_angle;}
 inline float get_last_z_angle(){return last_z_angle;}
 inline float get_last_gyro_x_angle() {return last_gyro_x_angle;}
 inline float get_last_gyro_y_angle(){return last_gyro_y_angle;}
 inline float get_last_gyro_z_angle(){return last_gyro_z_angle;}
		 
 
 float base_x_gyro=0;
 float base_y_gyro=0;
 float base_z_gyro=0;
 float base_x_acce1=0;
 float base_y_acce1=0;
 float base_z_acce1=0;
 
 float gyro_angle_z=0;
 
 // this globasl variable tells us how to scale gyroscope data
 float GYRO_FACTOR=0;
 
 unsigned long t_now=0,t_last=0;
 float dt=0;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

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
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
while(TM_MPU6050_Init(&MPU6050_Sensor, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_8G, TM_MPU6050_Gyroscope_250s) != TM_MPU6050_Result_Ok)
 {

 } 

 GYRO_FACTOR=131.0;
 const float RADIANS_TO_DEGREES= 57.2958;  // 180/pi
  /* USER CODE END 2 */
 set_last_read_angle_data(0,0,0,0,0,0,0); // ilk önce sifira setledik
 HAL_Delay(5000);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
    HAL_Delay(200);
		
		t_last=HAL_GetTick(); // buraya kadar geçen süre
		// Read all data from sensor1;
		TM_MPU6050_ReadAll(&MPU6050_Sensor);
		t_now=HAL_GetTick();   // degerleri okuduktan sonraki süre 
  /* USER CODE BEGIN 3 */
		float gyro_x=(MPU6050_Sensor.Gyroscope_X)/GYRO_FACTOR;
		float gyro_y=(MPU6050_Sensor.Gyroscope_Y)/GYRO_FACTOR;
		float gyro_z=(MPU6050_Sensor.Gyroscope_Z)/GYRO_FACTOR;
		
		float acce1_x= MPU6050_Sensor.Accelerometer_X; 
    float acce1_y= MPU6050_Sensor.Accelerometer_Y; 
    float acce1_z= MPU6050_Sensor.Accelerometer_Z; 

		float acce1_angle_y=atan(-1*acce1_x/sqrt(pow(acce1_y,2) +pow(acce1_z,2)))*RADIANS_TO_DEGREES;
    float acce1_angle_x=atan(-1*acce1_y/sqrt(pow(acce1_x,2) +pow(acce1_z,2)))*RADIANS_TO_DEGREES;
    float acce1_angle_z=0;	 // accelerometer doens't give z-angle
		
		/* Compute the (filtered) gyro angles */ 
    dt=(t_now-get_last_time())/1000.0;		
		 
		float gyro_angle_x =gyro_x*dt+ get_last_x_angle();
		float gyro_angle_y =gyro_y*dt+ get_last_y_angle();
		
		int gz_threshold=2; // gyro z raw data fluctuation threshold value when gyro doens't move.it is up to ypue mpu6050.
		if(gyro_z<gz_threshold && gyro_z >-gz_threshold)
						gyro_z=0;
		
		
		gyro_angle_z=gyro_z*dt+get_last_gyro_z_angle();
		
		/* Compute the drifting gyro angles */
		
	  float unfiltered_gyro_angle_x=gyro_x*dt+get_last_gyro_x_angle();
		float unfiltered_gyro_angle_y=gyro_y*dt+get_last_gyro_y_angle();
		float unfiltered_gyro_angle_z=gyro_z*dt+get_last_gyro_z_angle();
		
		 // Apply the complemantary filter to figure out the change in angle - choice of alpha is
		  // estimated now. Alpha depends on the sampling rate...
		const float alpha=0.96;
		
		float angle_x=alpha*gyro_angle_x+(1.0f-alpha)*acce1_angle_x;
		float angle_y=alpha*gyro_angle_y+(1.0f-alpha)*acce1_angle_y;
		float angle_z=gyro_angle_z; // accelerometer doesn't give z-angle.
		
		// Update the saved data with the latest value:
		set_last_read_angle_data(t_now,angle_x,angle_y,angle_z,unfiltered_gyro_angle_x,unfiltered_gyro_angle_y,unfiltered_gyro_angle_z);
		
		/* Send the angle values to serial buffer  */
		HAL_UART_Transmit(&huart2,(uint8_t*)data,sprintf(data,
		"Angle values\n- X:%3.4f\n- Y:%3.4f\n- Z:%3.4f\nTemperature\n- %3.4f\n\n\n",
		angle_x,
		angle_y,
		angle_z,
		MPU6050_Sensor.Temperature
		),1000);
		
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
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

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
