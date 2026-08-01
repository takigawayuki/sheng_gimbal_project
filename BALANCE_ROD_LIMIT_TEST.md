# 摆杆限幅实测记录

限幅参数在 `user/app/gimbal_drv.c` 文件末尾：

```c
#define ROD_CENTER_DEG          0.0f
#define ROD_MIN_DEG            -8.0f
#define ROD_MAX_DEG             8.0f
```

测试顺序：

1. 先不要放钢球，一只手准备断电。
2. 上电后停在待机，不启动题 3/4/5/6。
3. 把摆杆调到物理水平，读取 `pitchmotor.Position`，填入 `ROD_CENTER_DEG`。
4. 用很小角度逐步往正方向试，比如 `+1°`、`+2°`，到机械安全边界前停止，填 `ROD_MAX_DEG`。
5. 同样往负方向试，填 `ROD_MIN_DEG`。
6. PID 输出限幅要比机械限幅更小，例如机械 `-10° ~ +10°`，先用输出 `±3°` 或 `±5°`。
7. 放入钢球后先测题 3，确认方向正确。若球越跑越远，把 `pid_init()` 里 H 题控制的 `kp/ki/kd` 符号取反。

记录表：

| 项目 | 实测值 |
| --- | --- |
| 水平位置 | `ROD_CENTER_DEG = ____` |
| 负方向安全限幅 | `ROD_MIN_DEG = ____` |
| 正方向安全限幅 | `ROD_MAX_DEG = ____` |
| 视觉比例 | `BALL_VISION_SCALE_CM_PER_UNIT = ____` |