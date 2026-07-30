# 串口数据帧协议

## 串口参数

- 波特率：921600
- 数据位：8
- 停止位：1
- 校验位：无

## 帧格式

单帧总长度为 16 字节。

| 字节偏移 | 长度 | 类型 | 内容 |
| --- | --- | --- | --- |
| 0 | 1 | uint8_t | 帧头 0xAA |
| 1 | 1 | uint8_t | 帧头 0xBB |
| 2 - 5 | 4 | float | 数据 1 |
| 6 - 9 | 4 | float | 数据 2 |
| 10 - 13 | 4 | float | 数据 3 |
| 14 | 1 | uint8_t | 状态/标志数据 |
| 15 | 1 | uint8_t | 帧尾 0xEE |

## 字节顺序

`float` 按 MCU 常用的小端格式发送，低字节在前，高字节在后。

例如发送 3 个 `float` 数据 `f1`、`f2`、`f3` 和 1 个 `uint8_t` 数据 `flag`：

```c
uint8_t frame[16];

frame[0] = 0xAA;
frame[1] = 0xBB;

memcpy(&frame[2],  &f1, 4);
memcpy(&frame[6],  &f2, 4);
memcpy(&frame[10], &f3, 4);

frame[14] = flag;
frame[15] = 0xEE;

HAL_UART_Transmit(&huart, frame, sizeof(frame), timeout);
```

## 数据校验

本协议当前未包含校验和或 CRC。接收端可通过以下规则判断一帧数据是否有效：

1. 第 0 字节必须为 `0xAA`
2. 第 1 字节必须为 `0xBB`
3. 第 15 字节必须为 `0xEE`
4. 总长度必须为 16 字节

