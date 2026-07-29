#ifndef __MPU9250_APP_H
#define __MPU9250_APP_H

#include "main.h"

typedef struct
{
    float pitch;
    float roll;
    float yaw;
    short gyro_x;
    short gyro_y;
    short gyro_z;
    short accel_x;
    short accel_y;
    short accel_z;
    uint8_t init_ok;
    uint8_t dmp_error;
    uint32_t update_count;
} MPU9250_Data_t;

extern volatile MPU9250_Data_t mpu9250_data;

uint8_t MPU9250_UserInit(void);
void MPU9250_Update10ms(void);

#endif
