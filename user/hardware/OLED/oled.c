#include "oled.h"
#include "oledfont.h"          //头文件

static uint8_t CMD_Data[] = {
0xAE, 0x00, 0x10, 0x40, 0xB0, 0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F,
					
0xC8, 0xD3, 0x00, 0xD5, 0x80, 0xD8, 0x05, 0xD9, 0xF1, 0xDA, 0x12,
					
0xD8, 0x30, 0x8D, 0x14, 0xAF};      //初始化命令

enum
{
	OLED_I2C_ADDR = 0x78U,
	OLED_I2C_CMD = 0x00U,
	OLED_I2C_DATA = 0x40U,
	OLED_I2C_DELAY_COUNT = 16U
};

static void oled_i2c_delay(void)
{
	volatile uint32_t i;

	for (i = 0U; i < OLED_I2C_DELAY_COUNT; i++)
	{
		__NOP();
	}
}

static void oled_i2c_start(void)
{
	OLED_SDA_ON();
	OLED_SCK_ON();
	oled_i2c_delay();
	OLED_SDA_OFF();
	oled_i2c_delay();
	OLED_SCK_OFF();
}

static void oled_i2c_stop(void)
{
	OLED_SDA_OFF();
	oled_i2c_delay();
	OLED_SCK_ON();
	oled_i2c_delay();
	OLED_SDA_ON();
	oled_i2c_delay();
}

static void oled_i2c_write_byte(uint8_t data)
{
	uint8_t i;

	for (i = 0U; i < 8U; i++)
	{
		if ((data & 0x80U) != 0U)
		{
			OLED_SDA_ON();
		}
		else
		{
			OLED_SDA_OFF();
		}

		oled_i2c_delay();
		OLED_SCK_ON();
		oled_i2c_delay();
		OLED_SCK_OFF();
		data <<= 1;
	}

	/* 释放 SDA，忽略 OLED 的 ACK。 */
	OLED_SDA_ON();
	oled_i2c_delay();
	OLED_SCK_ON();
	oled_i2c_delay();
	OLED_SCK_OFF();
}

static void oled_write_bytes(uint8_t control, const uint8_t *buf, uint16_t len)
{
	uint16_t i;

	oled_i2c_start();
	oled_i2c_write_byte(OLED_I2C_ADDR);
	oled_i2c_write_byte(control);

	for (i = 0U; i < len; i++)
	{
		oled_i2c_write_byte(buf[i]);
	}

	oled_i2c_stop();
}

static void oled_write_data_buffer(const uint8_t *buf, uint16_t len)
{
	oled_write_bytes(OLED_I2C_DATA, buf, len);
}


void WriteCmd(void)
{
	oled_write_bytes(OLED_I2C_CMD, CMD_Data, sizeof(CMD_Data));
}
//向设备写控制命令
void OLED_WR_CMD(uint8_t cmd)
{
	oled_write_bytes(OLED_I2C_CMD, &cmd, 1U);
}
//向设备写数据
void OLED_WR_DATA(uint8_t data)
{
	oled_write_bytes(OLED_I2C_DATA, &data, 1U);
}
//初始化oled屏幕
void OLED_Init(void)
{ 	
	OLED_SDA_ON();
	OLED_SCK_ON();
	HAL_Delay(200);
 
	WriteCmd();
}
//清屏
void OLED_Clear(void)
{
	uint8_t i;
	uint8_t empty_line[128] = {0};		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_CMD(0xb0+i);
		OLED_WR_CMD (0x00); 
		OLED_WR_CMD (0x10); 
		oled_write_data_buffer(empty_line, sizeof(empty_line));
	} 
}
//开启OLED显示    
void OLED_Display_On(void)
{
	OLED_WR_CMD(0X8D);  //SET DCDC命令
	OLED_WR_CMD(0X14);  //DCDC ON
	OLED_WR_CMD(0XAF);  //DISPLAY ON
}
//关闭OLED显示     
void OLED_Display_Off(void)
{
	OLED_WR_CMD(0X8D);  //SET DCDC命令
	OLED_WR_CMD(0X10);  //DCDC OFF
	OLED_WR_CMD(0XAE);  //DISPLAY OFF
}		   			 
void OLED_Set_Pos(uint8_t x, uint8_t y) 
{ 	
	OLED_WR_CMD(0xb0+y);
	OLED_WR_CMD(((x&0xf0)>>4)|0x10);
	OLED_WR_CMD(x&0x0f);
} 
 
void OLED_On(void)  
{  
	uint8_t i;
	uint8_t fill_line[128];

	for (i = 0U; i < sizeof(fill_line); i++)
	{
		fill_line[i] = 1U;
	}

	for(i=0;i<8;i++)  
	{  
		OLED_WR_CMD(0xb0+i);    //设置页地址（0~7）
		OLED_WR_CMD(0x00);      //设置显示位置—列低地址
		OLED_WR_CMD(0x10);      //设置显示位置—列高地址   
		oled_write_data_buffer(fill_line, sizeof(fill_line));
	} //更新显示
}
unsigned int oled_pow(uint8_t m,uint8_t n)
{
	unsigned int result=1;	 
	while(n--)result*=m;    
	return result;
}
//显示2个数字
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);	 		  
void OLED_ShowNum(uint8_t x,uint8_t y,unsigned int num,uint8_t len,uint8_t size2)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size2/2)*t,y,' ',size2);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size2/2)*t,y,temp+'0',size2); 
	}
} 
//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示				 
//size:选择字体 16/12 
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size)
{      	
	unsigned char c=0;	
		c=chr-' ';//得到偏移后的值			
		if(x>128-1){x=0;y=y+2;}
		if(Char_Size ==16)
			{
			OLED_Set_Pos(x,y);	
			oled_write_data_buffer(&F8X16[c*16], 8U);
			OLED_Set_Pos(x,y+1);
			oled_write_data_buffer(&F8X16[c*16+8], 8U);
			}
			else {	
				OLED_Set_Pos(x,y);
				oled_write_data_buffer(F6x8[c], 6U);
				
			}
}
 
//显示一个字符号串
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t Char_Size)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		OLED_ShowChar(x,y,chr[j],Char_Size);
			x+=8;
		if(x>120){x=0;y+=2;}
			j++;
	}
}
//显示汉字
//hzk 用取模软件得出的数组
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no)
{      			    
	OLED_Set_Pos(x,y);	
	oled_write_data_buffer(Hzk[2*no], 16U);
		OLED_Set_Pos(x,y+1);	
	oled_write_data_buffer(Hzk[2*no+1], 16U);
}

