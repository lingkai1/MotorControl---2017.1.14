/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* USER CODE BEGIN Includes */
#include "pi.h"
#include "app.h"
#include "math.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint16_t pwm_value=0;
void user_pwm_setvalue(uint16_t value);
uint8_t  uartCheck = 0;
uint8_t RxBuffer[10];
uint8_t Rx422_Buffer[10];
float Speed = 0;

PI_CONTROLLER pi_spd = PI_CONTROLLER_DEFAULTS;
extern DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM12_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_DMA(&huart2, (uint8_t *)RxBuffer, 10);
	HAL_UART_Receive_DMA(&huart4, (uint8_t *)Rx422_Buffer, 10);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim2);

// Initialize the PI module for spd
  pi_spd.Kp= 2.8f;
	pi_spd.Ki= 0.004f;
	pi_spd.Umax = 1.0f;
	pi_spd.Umin = 0.0f;	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	int32_t i=0,j=rand();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	if((RxBuffer[0]==0xAA)&&(RxBuffer[1]==0x55))
	{ 
		uartCheck = 1;	
	}
	else
	{		
		HAL_Delay(RxBuffer[i]/3);
		MX_USART2_UART_Init();
		uartCheck = 0;
		i++;
		if(i>9)
			i=0;
	}
//    

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void user_pwm_setvalue(uint16_t value)
{
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);  
}

typedef union Rx_Dat
{
	uint16_t AD[4];
	uint8_t y[8];
}Rx_Dat;

__IO uint16_t My_AD[4];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  
{  
	Rx_Dat My_Rx_Dat;
	if((RxBuffer[0]==0xAA)&&(RxBuffer[1]==0x55))
	{
		My_Rx_Dat.y[0] = RxBuffer[2];
		My_Rx_Dat.y[1] = RxBuffer[3];
		My_Rx_Dat.y[2] = RxBuffer[4];
		My_Rx_Dat.y[3] = RxBuffer[5];
		My_Rx_Dat.y[4] = RxBuffer[6];
		My_Rx_Dat.y[5] = RxBuffer[7];
		My_Rx_Dat.y[6] = RxBuffer[8];
		My_Rx_Dat.y[7] = RxBuffer[9];
//		Speed = My_Rx_Dat.x;
		My_AD[0] = My_Rx_Dat.AD[0];
		My_AD[1] = My_Rx_Dat.AD[1];
		My_AD[2] = My_Rx_Dat.AD[2];
		My_AD[3] = My_Rx_Dat.AD[3];
		
		timer3DataProcess();
		
		printf("%d,%d,%d,%d\r\n",My_AD[0],My_AD[1],My_AD[2],My_AD[3]);
		
//		HAL_UART_Transmit_DMA(&huart4, RxBuffer, 10);
	}
	else
	{
		Speed = 9999;
		MX_DMA_Init();  
		HAL_UART_Receive_DMA(huart, (uint8_t *)RxBuffer, 10);
	}
	
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	MX_DMA_Init();  
	HAL_UART_Receive_DMA(huart, (uint8_t *)RxBuffer, 10);
}


#define Time_Interval 1
/* Captured Values */
uint32_t               uwIC2Value1 = 0;
uint32_t               uwIC2Value2 = 0;
uint32_t               uwDiffCapture = 0;
/* Capture index */
uint16_t               uhCaptureIndex = 0;
/* Frequency Value */
__IO float               	 uwFrequency = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
    if(uhCaptureIndex == 0)
    {
      /* Get the 1st Input Capture value */
      uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      uhCaptureIndex = 1;
		}
    else if(uhCaptureIndex == 1)
    {
      /* Get the 2nd Input Capture value */
      uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); 

      /* Capture computation */
      if (uwIC2Value2 > uwIC2Value1)
      {
        uwDiffCapture = (uwIC2Value2 - uwIC2Value1); 
      }
      else if (uwIC2Value2 < uwIC2Value1)
      {
        uwDiffCapture = ((0xFFFF - uwIC2Value1) + uwIC2Value2); 
      }
      else
      {
        /* If capture values are equal, we have reached the limit of frequency
           measures */
        Error_Handler();
      }
      /* Frequency computation: for this example TIMx (TIM1) is clocked by
         APB1Clk */      
			uwFrequency = (1000000.0f/uwDiffCapture/Time_Interval);
      uhCaptureIndex = 0;
			_IQsat(uwFrequency, 500, 0);
		}
  }
}

//#define DEBUG
uint16_t time=2000;
uint16_t Flag=0;
uint16_t Is_Control = 0;
float Fre_Set=500;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
			uint16_t s = htim12.Init.Period;
			if(htim12.Instance == TIM12)
			{
				#ifdef DEBUG
					if(Flag == 1)
					{
						Flag = 0;
						user_pwm_setvalue(s-time);
					}
					else if (Flag == 2)
					{
						user_pwm_setvalue(s-time);
					}
					else
						user_pwm_setvalue(s);
				#else		
				if(Is_Control == 1)
				{
					pi_spd.Ref = Fre_Set/500.0f;			//between 500Hz
					pi_spd.Fbk = uwFrequency/500.0f;		
					PI_MACRO(pi_spd)
					pwm_value = s-(uint16_t)(pi_spd.Out*s);
					user_pwm_setvalue(pwm_value);		
				}					
				else
					user_pwm_setvalue(s);		
				#endif
			}
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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
