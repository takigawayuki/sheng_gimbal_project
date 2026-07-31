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
#define VISION_STREAM_BUF_SIZE     128U
#define VISION_DMA_BUF_SIZE        256U   /* Circular DMA ring buffer, large enough to avoid wraparound between 1ms polls */

static uint8_t zdt_uart1_rx_buf[ZDT_UART_RX_BUF_SIZE];
static uint8_t zdt_uart3_rx_buf[ZDT_UART_RX_BUF_SIZE];
static uint8_t vision_uart6_rx_buf[VISION_DMA_BUF_SIZE];
static uint32_t vision_dma_last_ndtr = VISION_DMA_BUF_SIZE; /* Track DMA write pointer for manual polling */
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

// ����DMA�����жϽ���
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
		Vision_UART_Start();  /* Restart circular DMA */
	}
}

// �жϵ������
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
		/* H��ջ�ʹ�ø���ʵ��λ�� position_cm��
		 * error_cm ����λ���� target-position �����������ֻ���浽���Ա�����
		 */
		camera_data_update(position_cm, 0.0f);
		sys.value.ball_vel_cm_s = velocity_cm_s;
	} else {
		/* LOST ʱ���ܰ� 0 ���ɸ��������ģ�ֻ��¼��ʧ״̬�����ٶȡ� */
		dbg_vision_lost_cnt++;
		sys.value.ball_vel_cm_s = 0.0f;
		target_lost_cnt++;
	}
}

// ���� USART6 �Ӿ� 18 �ֽ�֡��A5 + status + 3��float + sequence + CRC16
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

/* ── USART6 视觉 Circular DMA 专用函数 ───────────────────────────
 * USART6 使用 HAL_UART_Receive_DMA (Circular) + 手动 NDTR 轮询，
 * 不依赖 HAL 的 IDLE 中断。TIM7 ISR 每 1ms 调用 Vision_UART_Poll()。
 */
void Vision_UART_Start(void)
{
    vision_dma_last_ndtr = VISION_DMA_BUF_SIZE;
    vision_stream_len = 0U;
    if (HAL_UART_Receive_DMA(&huart6, vision_uart6_rx_buf, VISION_DMA_BUF_SIZE) == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
        /* TC interrupt also not needed; we poll NDTR instead */
        __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_TC);
    }
}

void Vision_UART_Poll(void)
{
    uint32_t cur_ndtr;
    uint32_t new_bytes;
    uint32_t seg_start;   /* for wraparound case */

    cur_ndtr = __HAL_DMA_GET_COUNTER(huart6.hdmarx);

    if (cur_ndtr == vision_dma_last_ndtr) {
        return;  /* No new data */
    }

    if (cur_ndtr < vision_dma_last_ndtr) {
        /* ── 正常情况：DMA 未回绕，数据连续 ── */
        new_bytes = vision_dma_last_ndtr - cur_ndtr;
        /* 新数据在 buf[(BUF_SIZE - last_ndtr) ... (BUF_SIZE - cur_ndtr - 1)] */
        Vision_UART_ParseData(&vision_uart6_rx_buf[VISION_DMA_BUF_SIZE - vision_dma_last_ndtr],
                              (uint16_t)new_bytes);
    } else {
        /* ── DMA 回绕：新数据分两段 ── */
        /* 第一段：buf 末尾 (从上次位置到 buf 结束) */
        seg_start = VISION_DMA_BUF_SIZE - vision_dma_last_ndtr;
        Vision_UART_ParseData(&vision_uart6_rx_buf[seg_start], (uint16_t)vision_dma_last_ndtr);
        /* 第二段：buf 开头 (从 0 到当前写入位置) */
        Vision_UART_ParseData(&vision_uart6_rx_buf[0], (uint16_t)(VISION_DMA_BUF_SIZE - cur_ndtr));
    }

    vision_dma_last_ndtr = cur_ndtr;
}

/* 统一上电初始化所有 UART 接收 */
void ZDT_UART_RxStart(void)
{
	ZDT_UART_StartReceive(&huart1, zdt_uart1_rx_buf);
	CarSpeak_UART_RxStart();
	ZDT_UART_StartReceive(&huart3, zdt_uart3_rx_buf);
	Vision_UART_Start();  /* USART6: Circular DMA, no IDLE */
}

// UART������ɻص�
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
	/* USART6 使用 Circular DMA + NDTR 轮询（Vision_UART_Poll），不使用 IDLE 回调 */
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	ZDT_UART_RecoverReceive(huart);
}


