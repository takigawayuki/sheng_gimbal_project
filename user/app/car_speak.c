#include "common.h"
#include "user_usart.h"
#include <string.h>

#define CAR_SPEAK_UART_RX_BUF_SIZE 64U
#define CAR_SPEAK_FRAME_SIZE 16U
#define CAR_SPEAK_FRAME_HEAD_1 0xAAU
#define CAR_SPEAK_FRAME_HEAD_2 0xBBU
#define CAR_SPEAK_FRAME_TAIL 0xEEU

uint8_t car_speak_uart2_rx_buf[CAR_SPEAK_UART_RX_BUF_SIZE];

car_speak_rx_t car_speak_rx;

static void CarSpeak_UART_ClearError(UART_HandleTypeDef *huart)
{
  __HAL_UART_CLEAR_PEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_IDLEFLAG(huart);
}

void CarSpeak_UART_RxStart(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, car_speak_uart2_rx_buf,
                                   CAR_SPEAK_UART_RX_BUF_SIZE) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
  }
}

void CarSpeak_UART_RecoverReceive(void)
{
  car_speak_rx.error_count++;

  CarSpeak_UART_ClearError(&huart2);
  HAL_UART_AbortReceive(&huart2);
  CarSpeak_UART_RxStart();
}

void CarSpeak_UART_ParseData(uint16_t size)
{
  uint16_t index = 0;

  car_speak_rx.last_rx_size = size;

  while ((index + CAR_SPEAK_FRAME_SIZE) <= size)
  {
    if (car_speak_uart2_rx_buf[index] == CAR_SPEAK_FRAME_HEAD_1 &&
        car_speak_uart2_rx_buf[index + 1] == CAR_SPEAK_FRAME_HEAD_2 &&
        car_speak_uart2_rx_buf[index + 15] == CAR_SPEAK_FRAME_TAIL)
    {
      memcpy((void *)car_speak_rx.raw, &car_speak_uart2_rx_buf[index],
             CAR_SPEAK_FRAME_SIZE);
      memcpy((void *)&car_speak_rx.data_1, &car_speak_uart2_rx_buf[index + 2],
             sizeof(float));
      memcpy((void *)&car_speak_rx.data_2, &car_speak_uart2_rx_buf[index + 6],
             sizeof(float));
      memcpy((void *)&car_speak_rx.data_3, &car_speak_uart2_rx_buf[index + 10],
             sizeof(float));

      car_speak_rx.flag = car_speak_uart2_rx_buf[index + 14];
      car_speak_rx.frame_ok = 1U;
      car_speak_rx.rx_count++;
      return;
    }

    index++;
  }

  car_speak_rx.frame_ok = 0U;
}
