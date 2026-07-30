#include "user_usart.h"
#include "ZhangDaTou.h"
#include "common.h"
#include <string.h>

extern void camera_data_update(float dx, float dy);

#define ZDT_UART_RX_BUF_SIZE 64

#define VISION_FRAME_SIZE          18U
#define VISION_PAYLOAD_SIZE        16U
#define VISION_SOF                 0xA5U
#define VISION_STATUS_LOST         0x00U
#define VISION_STATUS_DETECTED     0x20U
#define VISION_STATUS_PREDICTED    0x21U
#define VISION_STREAM_BUF_SIZE     64U

static uint8_t zdt_uart1_rx_buf[ZDT_UART_RX_BUF_SIZE];
static uint8_t zdt_uart3_rx_buf[ZDT_UART_RX_BUF_SIZE];
static uint8_t vision_uart6_rx_buf[ZDT_UART_RX_BUF_SIZE];
static uint8_t vision_stream_buf[VISION_STREAM_BUF_SIZE];
static uint16_t vision_stream_len = 0U;

volatile uint32_t dbg_uart1_error_cnt = 0;
volatile uint32_t dbg_uart3_error_cnt = 0;
volatile uint32_t dbg_uart6_error_cnt = 0;
volatile uint32_t dbg_uart_last_error = 0;

volatile uint32_t dbg_vision_rx_cnt = 0;
volatile uint32_t dbg_vision_frame_ok_cnt = 0;
volatile uint32_t dbg_vision_crc_error_cnt = 0;
volatile uint32_t dbg_vision_status_error_cnt = 0;
volatile uint32_t dbg_vision_lost_cnt = 0;
volatile uint8_t dbg_vision_status = 0U;
volatile uint16_t dbg_vision_sequence = 0U;
volatile float dbg_vision_error_cm = 0.0f;
volatile float dbg_vision_position_cm = 0.0f;
volatile float dbg_vision_velocity_cm_s = 0.0f;

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

static uint16_t Vision_CRC16(const uint8_t *data, uint16_t length)
{
	uint16_t crc = 0xFFFFU;
	uint16_t i;
	uint8_t bit;

	for (i = 0U; i < length; i++) {
		crc ^= data[i];
		for (bit = 0U; bit < 8U; bit++) {
			if ((crc & 0x0001U) != 0U) {
				crc = (uint16_t)((crc >> 1U) ^ 0x8408U);
			} else {
				crc >>= 1U;
			}
		}
	}

	return crc;
}

static uint8_t Vision_StatusValid(uint8_t status)
{
	return (status == VISION_STATUS_LOST ||
			status == VISION_STATUS_DETECTED ||
			status == VISION_STATUS_PREDICTED) ? 1U : 0U;
}

static void Vision_HandleFrame(const uint8_t *frame)
{
	uint8_t status = frame[1];
	float error_cm = 0.0f;
	float position_cm = 0.0f;
	float velocity_cm_s = 0.0f;
	uint16_t sequence = 0U;

	memcpy(&error_cm, &frame[2], sizeof(float));
	memcpy(&position_cm, &frame[6], sizeof(float));
	memcpy(&velocity_cm_s, &frame[10], sizeof(float));
	sequence = (uint16_t)frame[14] | ((uint16_t)frame[15] << 8U);

	dbg_vision_status = status;
	dbg_vision_sequence = sequence;
	dbg_vision_error_cm = error_cm;
	dbg_vision_position_cm = position_cm;
	dbg_vision_velocity_cm_s = velocity_cm_s;
	dbg_vision_frame_ok_cnt++;

	if ((status & VISION_STATUS_DETECTED) != 0U) {
		/* H题闭环使用钢球实测位置 position_cm。
		 * error_cm 是上位机按 target-position 算出的误差，这里只保存到调试变量。
		 */
		camera_data_update(position_cm, 0.0f);
		sys.value.ball_vel_cm_s = velocity_cm_s;
	} else {
		/* LOST 时不能把 0 当成钢球在中心，只记录丢失状态并清速度。 */
		dbg_vision_lost_cnt++;
		sys.value.ball_vel_cm_s = 0.0f;
		target_lost_cnt++;
	}
}

// 解析 USART6 视觉 18 字节帧：A5 + status + 3个float + sequence + CRC16
static void Vision_UART_ParseData(uint8_t *data, uint16_t size)
{
	uint16_t copy_size;
	uint16_t index = 0U;

	if (size == 0U) {
		return;
	}

	dbg_vision_rx_cnt++;

	if (size > VISION_STREAM_BUF_SIZE) {
		data += (size - VISION_STREAM_BUF_SIZE);
		size = VISION_STREAM_BUF_SIZE;
		vision_stream_len = 0U;
	}

	if ((vision_stream_len + size) > VISION_STREAM_BUF_SIZE) {
		copy_size = VISION_FRAME_SIZE - 1U;
		if (copy_size > vision_stream_len) {
			copy_size = vision_stream_len;
		}
		memmove(vision_stream_buf, &vision_stream_buf[vision_stream_len - copy_size], copy_size);
		vision_stream_len = copy_size;
	}

	memcpy(&vision_stream_buf[vision_stream_len], data, size);
	vision_stream_len += size;

	while ((index + VISION_FRAME_SIZE) <= vision_stream_len) {
		uint16_t rx_crc;
		uint16_t calc_crc;

		if (vision_stream_buf[index] != VISION_SOF) {
			index++;
			continue;
		}

		rx_crc = (uint16_t)vision_stream_buf[index + 16U] |
				 ((uint16_t)vision_stream_buf[index + 17U] << 8U);
		calc_crc = Vision_CRC16(&vision_stream_buf[index], VISION_PAYLOAD_SIZE);

		if (calc_crc != rx_crc) {
			dbg_vision_crc_error_cnt++;
			index++;
			continue;
		}

		if (!Vision_StatusValid(vision_stream_buf[index + 1U])) {
			dbg_vision_status_error_cnt++;
			index++;
			continue;
		}

		Vision_HandleFrame(&vision_stream_buf[index]);
		index += VISION_FRAME_SIZE;
	}

	if (index > 0U) {
		vision_stream_len -= index;
		if (vision_stream_len > 0U) {
			memmove(vision_stream_buf, &vision_stream_buf[index], vision_stream_len);
		}
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


