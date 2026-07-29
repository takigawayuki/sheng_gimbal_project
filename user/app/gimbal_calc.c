#include "common.h"
#include "math.h"

// extern volatile uint8_t target_valid;
// extern volatile uint32_t target_lost_cnt;
// extern volatile uint8_t aim_stable_frames = 0;

void camera_data_update(float dx, float dy)
{
    sys.value.camera_x = dx;
    sys.value.camera_y = dy;

    // target_valid = 1; // 收到数据 → 有目标(不管 dx/dy 是否为 0)
    // target_lost_cnt = 0;

    if (dx != 0.0f || dy != 0.0f)
        target_lost_cnt = 0;

    // 如果这一帧里，靶心偏差很小（|dx|<10 且 |dy|<10）
    if (fabsf(dx) < 10.0f && fabsf(dy) < 10.0f && (dx != 0.0f || dy != 0.0f))
    {
        if (aim_stable_frames < 255)
            aim_stable_frames++;
    }
    else
    {
        aim_stable_frames = 0;
    }
}
