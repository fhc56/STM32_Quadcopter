#ifndef __IMU_H
#define __IMU_H

#include "stm32f10x.h"

/*
 * 重置四元数和积分修正
 */
void IMU_Reset(void);

/*
 * 根据当前加速度方向初始化四元数
 * 上电静止校准完成以后调用。
 */
void IMU_InitQuaternion(float ax, float ay, float az);

/*
 * 6轴姿态融合
 * 算法结构参考江协科技：
 * Acc重力方向
 *      +
 * Gyro
 *      ↓
 * Mahony PI误差修正
 *      ↓
 * Quaternion
 *      ↓
 * Roll/Pitch/Yaw
 * UseAcc：
 * 1 = 当前允许使用加速度计修正
 * 0 = 当前只依靠陀螺仪传播姿态
 */
void IMU_6Axis_Gyro_Acc(float ax, float ay, float az, float gx, float gy, float gz, float *Roll,
						float *Pitch, float *Yaw, float dt, uint8_t UseAcc);

/*
 * 获取Mahony积分项估计出来的
 * 动态Gyro偏差修正。
 * 单位：
 * °/s
 */
void IMU_GetGyroCorrection(float *CorrX, float *CorrY, float *CorrZ);

#endif /* IMU_H */
