#ifndef __BMP280_H
#define __BMP280_H

#include "stm32f10x.h"

/*
 * 注意：
 * BMP280和MPU6050共用：
 * PB10 = I2C2_SCL
 * PB11 = I2C2_SDA
 * 所以BMP280_Init()
 * 不会重新初始化I2C2。
 * 必须先：
 * Attitude_Init();
 * 再：
 * BMP280_Init();
 */
/*
 * 初始化成功：
 * return 1
 * 失败：
 * return 0
 */
uint8_t BMP280_Init(void);

/*
 * 芯片ID
 * 正常BMP280应该返回：
 * 0x58
 */
uint8_t BMP280_GetID(void);

/*
 * 获取数据
 * Temperature：
 * 0.01°C
 * Pressure：
 * Pa
 * 成功 = 1
 * 失败 = 0
 */
uint8_t BMP280_GetData(int32_t *Temperature, uint32_t *Pressure);

#endif
