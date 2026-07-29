#include "mpu9250_app.h"

#include "common.h"
#include "inv_mpu.h"
#include "mpu9250.h"

volatile MPU9250_Data_t mpu9250_data;

uint8_t MPU9250_UserInit(void)
{
    uint8_t ret = mpu_dmp_init();

    mpu9250_data.dmp_error = ret;
    mpu9250_data.init_ok = (ret == 0U) ? 1U : 0U;
    mpu9250_data.update_count = 0U;

    return ret;
}


void MPU9250_Update10ms(void)
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
    uint8_t ret;

    if (mpu9250_data.init_ok == 0U)
    {
        return;
    }

    ret = mpu_dmp_get_data(&pitch, &roll, &yaw);
    mpu9250_data.dmp_error = ret;
    if (ret == 0U)
    {
        mpu9250_data.pitch = pitch;
        mpu9250_data.roll = roll;
        mpu9250_data.yaw = yaw;
        mpu9250_data.update_count++;

        sys.value.gimbal_pitch = pitch;
        sys.value.gimbal_yaw = yaw;
    }

    if (MPU_Get_Gyroscope(&gyro_x, &gyro_y, &gyro_z) == 0U)
    {
        mpu9250_data.gyro_x = gyro_x;
        mpu9250_data.gyro_y = gyro_y;
        mpu9250_data.gyro_z = gyro_z;
    }

    if (MPU_Get_Accelerometer(&accel_x, &accel_y, &accel_z) == 0U)
    {
        mpu9250_data.accel_x = accel_x;
        mpu9250_data.accel_y = accel_y;
        mpu9250_data.accel_z = accel_z;
    }
}
