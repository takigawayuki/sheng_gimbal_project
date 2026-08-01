#include "ZhangDaTou.h"
#include "X_V2.h"
#include "usart.h"
#include "string.h"
ZDTMotor_Typedef pitchmotor={
.huart = &huart1,
.id =2,
.mod=1,
.setSpeed =100,
.setPosition = 0.0f,
.setAcc = ZHANGDATOU_DEFAULT_ACC,
.reduction_ratio = 6,
.microStep =256,
};
ZDTMotor_Typedef yawmotor={
.huart = &huart3,
.id =1,
.mod=1,
.setSpeed =100,
.setPosition = 0.0f,
.setAcc = ZHANGDATOU_DEFAULT_ACC,
.reduction_ratio = 6,
.microStep =256,
};

uint32_t swap_endian_32(uint32_t val) {
    return ((val >> 24) & 0x000000FF) |  // 移动最高字节到最低位
           ((val >> 8)  & 0x0000FF00) |  // 移动次高字节到次低位
           ((val << 8)  & 0x00FF0000) |  // 移动次低字节到次高位
           ((val << 24) & 0xFF000000);    // 移动最低字节到最高位
}
uint16_t swap_endian_16(uint16_t val) {
    return ((val >> 8) & 0x00FF) |  // 移动高字节到低位
           ((val << 8) & 0xFF00);    // 移动低字节到高位
}

static float ZhangDaTou_GetReductionRatio(ZDTMotor_Typedef* object)
{
	return (object->reduction_ratio == 0) ? 1.0f : (float)object->reduction_ratio;
}

static float ZhangDaTou_DegPerSecToMotorRPM(ZDTMotor_Typedef* object, float deg_per_sec)
{
	return ABS(deg_per_sec) * ZhangDaTou_GetReductionRatio(object) / 6.0f;
}

static float ZhangDaTou_OutputDegToMotorDeg(ZDTMotor_Typedef* object, float output_deg)
{
	return ABS(output_deg) * ZhangDaTou_GetReductionRatio(object);
}

static uint16_t ZhangDaTou_GetAcc(ZDTMotor_Typedef* object)
{
	return (object->setAcc == 0) ? ZHANGDATOU_DEFAULT_ACC : object->setAcc;
}

void ZhangDaTou_DataParm(uint8_t* Data,ZDTMotor_Typedef* object)
{
	if(Data[0]==object->id)
	{
		switch(Data[1])
		{
			case 0x35://速度
				if(Data[6-1]==0x6B)
				{
					static signed char sign=1;
					static uint16_t temp;
					if(Data[3-1]==1)
					{
						sign =-1;
					}
					else
					{
						sign =1;
					}
					memcpy(&temp, &Data[3], sizeof(uint16_t));
					object->Speed = (float)swap_endian_16(temp) * 0.1f * 6.0f / ZhangDaTou_GetReductionRatio(object) * sign;
				}
				break;
			case 0x36://位置
				if(Data[7]==0x6B)
				{
					static signed char sign=1;
					static uint32_t temp;
					if(Data[3-1]==1)
					{
						sign =-1;
					}
					else
					{
						sign =1;
					}
					memcpy(&temp, &Data[3], sizeof(uint32_t));
					object->Position = (float)swap_endian_32(temp) * 0.1f / ZhangDaTou_GetReductionRatio(object) * sign;
				}
				break;
			default:
				break;
		}
	}
}

void ZhangDaTou_Control(ZDTMotor_Typedef* object)
{
	if (object->huart == NULL) {
		object->huart = &huart1;
	}
	if (object->setAcc == 0) {
		object->setAcc = ZHANGDATOU_DEFAULT_ACC;
	}

	if(object->mod==0)//速度模式
	{
		uint8_t dir = (object->setSpeed >= 0.0f) ? 0 : 1;
		float vel_rpm = ZhangDaTou_DegPerSecToMotorRPM(object, object->setSpeed);
		X_V2_Vel_Control_UART(object->huart, object->id, dir, ZhangDaTou_GetAcc(object), vel_rpm, 0);
	}
	else//速度位置模式
	{
		uint8_t dir = (object->setPosition >= 0.0f) ? 0 : 1;
		float vel_rpm = ZhangDaTou_DegPerSecToMotorRPM(object, object->setSpeed);
		float pos_deg = ZhangDaTou_OutputDegToMotorDeg(object, object->setPosition);
		X_V2_Traj_Pos_Control_UART(object->huart, object->id, dir, ZhangDaTou_GetAcc(object), ZhangDaTou_GetAcc(object), vel_rpm, pos_deg, 1, 0);
	}
}

//速度模式控制
void ZhangDaTou_Speedctr(ZDTMotor_Typedef* object,float SpeedVal,uint16_t AccVal)
{
	object->mod = 0;
	object->setSpeed = SpeedVal;
	object->setAcc = (AccVal == 0) ? ZHANGDATOU_DEFAULT_ACC : AccVal;
}	
//速度位置控制
void ZhangDaTou_PositionSpeedctr(ZDTMotor_Typedef* object,float SpeedVal,float PositionVal,uint16_t AccVal)
{
	object->mod = 1;
	object->setSpeed = SpeedVal;
	object->setPosition = PositionVal;
	object->setAcc = (AccVal == 0) ? ZHANGDATOU_DEFAULT_ACC : AccVal;
}
//获取速度
float ZhangDaTou_getSpeedDate(ZDTMotor_Typedef* object)
{
	return object->Speed;
}
//获取位置
float ZhangDaTou_getPositionDate(ZDTMotor_Typedef* object)
{
	return object->Position;
}
//设置变量所代表的电机id,
void ZhangDaTou_init(ZDTMotor_Typedef* object,uint8_t id)
{
	object->id = id;
	if (object->huart == NULL) {
		object->huart = &huart1;
	}
	if (object->setAcc == 0) {
		object->setAcc = ZHANGDATOU_DEFAULT_ACC;
	}
}
void ZhangDaTou_SetUart(ZDTMotor_Typedef* object, UART_HandleTypeDef *huart)
{
	object->huart = huart;
}
void ZhangDaTou_SetAcc(ZDTMotor_Typedef* object,uint16_t AccVal)
{
	object->setAcc = (AccVal == 0) ? ZHANGDATOU_DEFAULT_ACC : AccVal;
}
//任务函数
void ZhangDaTou_Task()
{	
	// ZhangDaTou_Control(&pitchmotor);//控制电机1
	// ZhangDaTou_Control(&yawmotor);//控制电机2
}
void ZhangDaTou_StartPosFeedback(ZDTMotor_Typedef* object, uint16_t interval_ms)
{
	uint8_t i = 0;
	static uint8_t cmd[8];

	cmd[i++] = object->id;
	cmd[i++] = 0x11;
	cmd[i++] = 0x18;
	cmd[i++] = 0x36;
	cmd[i++] = (uint8_t)(interval_ms >> 8);
	cmd[i++] = (uint8_t)(interval_ms & 0xFF);
	cmd[i++] = 0x6B;

	HAL_UART_Transmit_DMA(object->huart, cmd, i);
}
void ZhangDaTou_Enable(ZDTMotor_Typedef* object, uint8_t state)
{
	uint8_t cmd[6];
	cmd[0] = object->id;
	cmd[1] = 0xF3;
	cmd[2] = 0xAB;
	cmd[3] = state;
	cmd[4] = 0x00;
	cmd[5] = 0x6B;
	HAL_UART_Transmit_DMA(object->huart, cmd, 6);
}
