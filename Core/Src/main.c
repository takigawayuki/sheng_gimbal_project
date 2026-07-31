/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ZhangDaTou.h"
#include "common.h"
#include "user_tim.h"
// #include "mpu9250_app.h"
#include "oled.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void OLED_Show(sys_t *sys);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_byte;

// ����OLED
char display_buf[32];
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  ZDT_UART_RxStart();
  HAL_Delay(100);
  gimbal_init();
  HAL_Delay(500);

  // while (MPU9250_UserInit() != 0U)
  // {
  //   HAL_Delay(100);
  // }

  // H
  // ZhangDaTou_Enable(&pitchmotor, 0);
  // HAL_Delay(5);
  // ZhangDaTou_Enable(&yawmotor, 0);
  // HAL_Delay(5);

  // H
  ZhangDaTou_Enable(&pitchmotor, 1);
  HAL_Delay(5);
  // ZhangDaTou_Enable(&yawmotor, 1);
  // HAL_Delay(5);

  // H
  ZhangDaTou_StartPosFeedback(&pitchmotor, 5);
  HAL_Delay(50);
  // ZhangDaTou_StartPosFeedback(&yawmotor, 5);
  // HAL_Delay(50);

  // CarSpeak_UART_RxStart();

  key_init();
  menu_init();
  //	yawmotor.setSpeed = 0;

  User_TIM_Init();

  OLED_Init();
  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"Hello, World!", 16);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // HAL_GPIO_WritePin(text_io_GPIO_Port,text_io_Pin,GPIO_PIN_SET);
    // camera_x_pid_ctrl(&sys, 0.0f);
    // camera_y_pid_ctrl(&sys, 0.0f);
    // HAL_GPIO_WritePin(text_io_GPIO_Port,text_io_Pin,GPIO_PIN_RESET);
    // HAL_Delay(50);

    //    uint8_t text[8] = {11,22,33,44,55,66,77,88};
    //    HAL_UART_Transmit(&huart3, text, 8, 0xFFFF);
    //    HAL_Delay(100);

    //  if (HAL_UART_Receive(&huart3, &rx_byte, 1, 10) == HAL_OK)
    //  {
    //    HAL_UART_Transmit(&huart3, &rx_byte, 1, 0xFFFF);
    //  }
    //  HAL_Delay(100);

    //  ZhangDaTou_Task();

    // uint8_t text[8] = {11,22,33,44,55,66,77,88};
    // HAL_UART_Transmit(&huart6, text, 8, 0xff);
    // HAL_Delay(100);

    // HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);

    OLED_Show(&sys);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Initializes the CPU, AHB and APB buses clocks
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

/* USER CODE BEGIN 4 */
// static const char *menu_item_to_str(uint8_t item)
// {
//   switch (item)
//   {
//   case 0:
//     return "STBY  ";
//   case 1:
//     return "T_S_L ";
//   case 2:
//     return "T_S_R ";
//   case 3:
//     return "T_DYN ";
//   case 4:
//     return "R_DYN ";
//   default:
//     return "UNK   ";
//   }
// }

static const char *menu_item_to_str(menu_item_t item)
{
  switch (item)
  {
  case MENU_ITEM_STANDBY:
    return "STBY";
  case MENU_ITEM_TASK3_STATIC_PM5:
    return "T3";
  case MENU_ITEM_TASK4_AB_CENTER:
    return "T4";
  case MENU_ITEM_TASK5_LAP_CENTER:
    return "T5";
  case MENU_ITEM_TASK6_LAP_SETPOINT:
    return "T6";
  default:
    return "UNK";
  }
}

static const char *run_to_str(void)
{
  if (gimbal_sm_obj.finished)
    return "DONE";

  return menu.in_running ? "ON" : "OFF";
}

static const char *gimbal_state_to_str(gimbal_state state)
{
  switch (state)
  {
  case GIMBAL_IDLE:
    return "IDLE";
  case BALANCE_TASK3_STATIC_PLUS_TO_MINUS:
    return "T3";
  case BALANCE_TASK4_CAR_TO_B_CENTER:
    return "T4";
  case BALANCE_TASK5_CAR_LAP_CENTER:
    return "T5";
  case BALANCE_TASK6_CAR_LAP_SETPOINT:
    return "T6";
  default:
    return "UNK";
  }
}

void OLED_Show(sys_t *sys)
{
  // if ((key2_short_press_cnt & 1))
  // {
  //   if (key2_short_press_cnt == 1)
  //   {
  //     HAL_Delay(200);
  //     sys->task_state = hunter;
  //     key2_short_press_cnt = 3;
  //   }
  //   sprintf(display_buf, "Yaw: %.2f    ", sys->car.m_yaw);
  //   OLED_ShowString(0, 0, (uint8_t *)display_buf, 16);
  //   sprintf(display_buf, "lap: %d      ", sys->car.rev_yaw);
  //   OLED_ShowString(0, 2, (uint8_t *)display_buf, 16);
  //   sprintf(display_buf, "ctrl:%d     ", sys->ctrl.rev_preset_count);
  //   OLED_ShowString(0, 4, (uint8_t *)display_buf, 16);
  //   // sprintf(display_buf, "Yaw: %.2f", sys->car.m_yaw);
  //   // OLED_ShowString(0, 6, (uint8_t *)display_buf, 16);
  // }
  // else
  // {
  //   //		sprintf(display_buf, "                ");
  //   //    OLED_ShowString(0, 0, (uint8_t *)display_buf, 16);
  //   sprintf(display_buf, "      REV   ");
  //   OLED_ShowString(0, 2, (uint8_t *)display_buf, 16);
  //   sprintf(display_buf, "       %d     ", sys->ctrl.rev_preset_count);
  //   OLED_ShowString(0, 4, (uint8_t *)display_buf, 16);
  //   OLED_Clear();
  //   sys->ctrl.rev_preset_count = key1_short_press_cnt % 6;
  //   //    sprintf(display_buf, "               ");
  //   //    OLED_ShowString(4, 0, (uint8_t *)display_buf, 16);
  //   //    sprintf(display_buf, "               ");
  //   //    OLED_ShowString(6, 0, (uint8_t *)display_buf, 16);
  // }

  sprintf(display_buf, "Item:%-4s      ", menu_item_to_str(menu.cur_item));
  OLED_ShowString(0, 0, (uint8_t *)display_buf, 16);

  sprintf(display_buf, "Run :%-4s      ", run_to_str());
  OLED_ShowString(0, 2, (uint8_t *)display_buf, 16);

  if (gimbal_sm_obj.state == BALANCE_TASK3_STATIC_PLUS_TO_MINUS)
  {
    sprintf(display_buf, "GSM :T3   P%d   ", gimbal_sm_obj.task3_phase);
  }
  else if ((gimbal_sm_obj.state == BALANCE_TASK4_CAR_TO_B_CENTER) ||
           (gimbal_sm_obj.state == BALANCE_TASK5_CAR_LAP_CENTER))
  {
    sprintf(display_buf, "GSM :%-4s O    ", gimbal_state_to_str(gimbal_sm_obj.state));
  }
  else if (gimbal_sm_obj.state == BALANCE_TASK6_CAR_LAP_SETPOINT)
  {
    sprintf(display_buf, "GSM :T6   SET  ");
  }
  else
  {
    sprintf(display_buf, "GSM :%-4s     ", gimbal_state_to_str(gimbal_sm_obj.state));
  }
  OLED_ShowString(0, 4, (uint8_t *)display_buf, 16);

  sprintf(display_buf, "Time:%5lums  ", gimbal_sm_obj.elapsed_ms);
  OLED_ShowString(0, 6, (uint8_t *)display_buf, 16);

  // sprintf(display_buf, "Item:%-11s", menu_item_to_str(menu.cur_item));
  // OLED_ShowString(0, 0, (uint8_t *)display_buf, 16);

  // sprintf(display_buf, "Run :%-11s", run_to_str(menu.in_running));
  // OLED_ShowString(0, 2, (uint8_t *)display_buf, 16);

  // sprintf(display_buf, "GSM :%-11s", gimbal_state_to_str(gimbal_sm_obj.state));
  // OLED_ShowString(0, 4, (uint8_t *)display_buf, 16);

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
