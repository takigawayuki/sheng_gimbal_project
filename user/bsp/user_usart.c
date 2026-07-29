#include "user_usart.h"
#include "ZhangDaTou.h"
#include "common.h"
#include <string.h>

extern void camera_data_update(float dx, float dy);

#define ZDT_UART_RX_BUF_SIZE 64
#define VISION_FRAME_SIZE 11

static uint8_t zdt_uart1_rx_buf[ZDT_UART_RX_BUF_SIZE];
static uint8_t zdt_uart3_rx_buf[ZDT_UART_RX_BUF_SIZE];
static uint8_t vision_uart6_rx_buf[ZDT_UART_RX_BUF_SIZE];

volatile uint32_t dbg_uart1_error_cnt = 0;
volatile uint32_t dbg_uart3_error_cnt = 0;
volatile uint32_t dbg_uart6_error_cnt = 0;
volatile uint32_t dbg_uart_last_error = 0;

// 开启DMA空闲中断接收
static void ZDT_UART_StartReceive(UART_HandleTypeDef *huart, uint8_t *rx_buf)
{
	if (HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buf, ZDT_UART_RX_BUF_SIZE) == HAL_OK) {
		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
	}
}

static void ZDT_UART_ClearError(UART_HandleTypeDef *huart)
{
	__HAL_UART_CLEAR_PEFLAG(huart);
	__HAL_UART_CLEAR_FEFLAG(huart);
	__HAL_UART_CLEAR_NEFLAG(huart);
	__HAL_UART_CLEAR_OREFLAG(huart);
	__HAL_UART_CLEAR_IDLEFLAG(huart);
}

static void ZDT_UART_RecoverReceive(UART_HandleTypeDef *huart)
{
	dbg_uart_last_error = huart->ErrorCode;

	ZDT_UART_ClearError(huart);
	HAL_UART_AbortReceive(huart);

	if (huart->Instance == USART1)
	{
		dbg_uart1_error_cnt++;
		ZDT_UART_StartReceive(&huart1, zdt_uart1_rx_buf);
	} else if (huart->Instance == USART2)
	{
		CarSpeak_UART_RecoverReceive();
	} else if (huart->Instance == USART3)
	{
		dbg_uart3_error_cnt++;
		ZDT_UART_StartReceive(&huart3, zdt_uart3_rx_buf);
	} else if (huart->Instance == USART6)
	{
		dbg_uart6_error_cnt++;
		ZDT_UART_StartReceive(&huart6, vision_uart6_rx_buf);
	}
}

// 判断电机串口
static void ZDT_UART_ParseMotorFrame(UART_HandleTypeDef *huart, uint8_t *data)
{
	if (pitchmotor.huart == huart) {
		ZhangDaTou_DataParm(data, &pitchmotor);
	}

	if (yawmotor.huart == huart) {
		ZhangDaTou_DataParm(data, &yawmotor);
	}
}

static void ZDT_UART_ParseFeedback(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
	uint16_t index = 0;

	while ((index + 2) <= size) {
		uint16_t frame_len = 0;

		switch (data[index + 1]) {
			case 0x35:
				frame_len = 6;
				break;

			case 0x36:
				frame_len = 8;
				break;

			default:
				index++;
				continue;
		}

		if ((index + frame_len) > size) {
			return;
		}

		if (data[index + frame_len - 1] == 0x6B) {
			ZDT_UART_ParseMotorFrame(huart, &data[index]);
			index += frame_len;
		} else {
			index++;
		}
	}

}

// 解析视觉数据帧
static void Vision_UART_ParseData(uint8_t *data, uint16_t size)
{
	uint16_t index = 0;

	while ((index + VISION_FRAME_SIZE) <= size) {
		if (data[index] == 0xAA && data[index + 1] == 0x55) {
			uint8_t checksum = 0;
			for (int i = 2; i < 10; i++) {
				checksum += data[index + i];
			}

			if (checksum == data[index + 10]) {
				float dx, dy;
				memcpy(&dx, &data[index + 2], sizeof(float));
				memcpy(&dy, &data[index + 6], sizeof(float));

				camera_data_update(dx, dy);
				return;
			}
		}
		index++;
	}
}

// 启动接收
void ZDT_UART_RxStart(void)
{
	ZDT_UART_StartReceive(&huart1, zdt_uart1_rx_buf);
	CarSpeak_UART_RxStart();
	ZDT_UART_StartReceive(&huart3, zdt_uart3_rx_buf);
	ZDT_UART_StartReceive(&huart6, vision_uart6_rx_buf);
}

// UART接收完成回调
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART1)
	{
		ZDT_UART_ParseFeedback(huart, zdt_uart1_rx_buf, Size);
		ZDT_UART_StartReceive(&huart1, zdt_uart1_rx_buf);
	} else if (huart->Instance == USART2)
	{
		CarSpeak_UART_ParseData(Size);
		CarSpeak_UART_RxStart();
	} else if (huart->Instance == USART3)
	{
		ZDT_UART_ParseFeedback(huart, zdt_uart3_rx_buf, Size);
		ZDT_UART_StartReceive(&huart3, zdt_uart3_rx_buf);
	} 
	else if (huart->Instance == USART6)
	{
		Vision_UART_ParseData(vision_uart6_rx_buf, Size);
		ZDT_UART_StartReceive(&huart6, vision_uart6_rx_buf);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	ZDT_UART_RecoverReceive(huart);
}


