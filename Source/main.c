/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Generic application start

*/

#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"

TIM_HandleTypeDef timHandler;
TIM_OC_InitTypeDef sConfig;
HAL_StatusTypeDef status;

void setUpTimerforRGBLed(void);

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    HAL_Delay(100);
    BSP_LED_Toggle(LED2);
  }
}


void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
14 => TIM2_CH1 => PA0 (R) B     (Breadboard Pin #1)
15 => TIM2_CH2 => PA1 (G) G     (Breadboard Pin #2)
GND => Cathode => GND (GND) GND (Breadboard Pin #3)
16 => TIM2_CH3 => PA2 (B) R     (Breadboard Pin #4)


84 MHz / 255 => 329411
PSC = 329411 / 255 => 1291
*/
void setUpTimerforRGBLed(void) {  
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();

  GPIO_InitTypeDef gpio_rgb = {0};

  gpio_rgb.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3;
  gpio_rgb.Mode = GPIO_MODE_AF_PP;
  gpio_rgb.Pull = GPIO_NOPULL;
  gpio_rgb.Speed = GPIO_SPEED_FREQ_LOW;
  gpio_rgb.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA,&gpio_rgb);

  timHandler.Instance = TIM2;
  timHandler.Init.Period = 255 - 1;
  timHandler.Init.Prescaler = 1291 - 1;
  timHandler.Init.ClockDivision = 0;
  timHandler.Init.CounterMode = TIM_COUNTERMODE_UP;
  timHandler.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  timHandler.Init.RepetitionCounter = 0;

  status = HAL_TIM_Base_Init(&timHandler);

  if(status != HAL_OK) {
    Error_Handler();
  }

  sConfig.OCMode = TIM_OCMODE_PWM1;
  sConfig.OCFastMode = TIM_OCFAST_DISABLE;
  sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfig.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;

  sConfig.Pulse = 0;
  status = HAL_TIM_PWM_ConfigChannel(&timHandler,&sConfig,TIM_CHANNEL_1);

  if(status != HAL_OK) {
    Error_Handler();
  }
  
  sConfig.Pulse = 0;
  status = HAL_TIM_PWM_ConfigChannel(&timHandler,&sConfig,TIM_CHANNEL_2);

  if(status != HAL_OK) {
    Error_Handler();
  }

  sConfig.Pulse = 0;
  status = HAL_TIM_PWM_ConfigChannel(&timHandler,&sConfig,TIM_CHANNEL_3);

  if(status != HAL_OK) {
    Error_Handler();
  }

  status = HAL_TIM_PWM_Start(&timHandler,TIM_CHANNEL_1); // Blue

  if(status != HAL_OK) {
    Error_Handler();
  }

  status = HAL_TIM_PWM_Start(&timHandler,TIM_CHANNEL_2); // Green

  if(status != HAL_OK) {
    Error_Handler();
  }

  status = HAL_TIM_PWM_Start(&timHandler,TIM_CHANNEL_3); // Red. Not working for the moment

  if(status != HAL_OK) {
    Error_Handler();
  }


}

/*********************************************************************
*
*       main()
*
*  Function description
*   Application entry point.
*/
int main(void) {
  uint16_t r = 0;
  uint16_t g = 0;
  uint16_t b = 0;

  /* STM32xx HAL library initialization */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  BSP_LED_Init(LED2);

  printf("Version HAL=%X\n", HAL_GetHalVersion());
  printf("Version BSP=%X\n", BSP_GetVersion());
  /** RCC - Lecture frequences differentes clocks */
  printf("HCLK =%d\n\r", HAL_RCC_GetHCLKFreq());
  printf("PCLK1 =%d\n\r", HAL_RCC_GetPCLK1Freq());
  printf("PCLK2 =%d\n\r", HAL_RCC_GetPCLK2Freq());
  printf("SYSCLK=%d\n\r", HAL_RCC_GetSysClockFreq());

  //Initialise LED
  BSP_LED_Init(LED2);

  setUpTimerforRGBLed();

  do {
    
    //BSP_LED_Toggle(LED2);

    TIM2->CCR1 = b;
    TIM2->CCR2 = g;
    TIM2->CCR3 = r;

    r += 5;
    g += 10;
    b += 15;

    if(r == 255)
        r = 0;
    
    if(g == 255)
        g = 0;
    
    if(b == 255)
        b = 0;

    HAL_Delay(100);
  } while (1);

}


/*************************** End of file ****************************/
