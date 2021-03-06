/*
 * Odometry.h
 *
 *  Created on: Feb 14, 2018
 *      Author: yusaku
 */
#pragma once

#include "MPU9250.h"
#include <cmath>

#define SPI_MPU9250  SPI2
#define GPIO_MPU9250 GPIOB
#define PIN_MPU9250  GPIO_PIN_12

#undef FOR_PR
#undef FOR_TR

#define FOR_PR
//#define FOR_TR

#if (defined(FOR_PR) && defined(FOR_TR))
#error "Cannot compile with both FOR_TR and FOR_PR set."
#endif
#if !(defined(FOR_PR) || defined(FOR_TR))
#error "Cannot compile with both FOR_TR and FOR_PR are not set."
#endif

class Odometry
{
private:
	float x;
	float y;
	float yaw;

	float offset_yaw;

	MPU9250 *mpu9250 = nullptr;

	// diameter of wheels in metre
	static constexpr float WheelDiameter = 0.0508; //タイヤの直径によって変更
	// pulse/rev
	static constexpr float PulsePerRevolution =
#ifdef FOR_TR
			500.0 * 4;
#endif
#ifdef FOR_PR
			500.0 * 4;
#endif

	/// Kpd = 2_pi_r[mm/rev] / Kp[pulse/rev]
	static constexpr float MPerPulse = M_PI * WheelDiameter / PulsePerRevolution;

	// milli degree per sec // milli G per sec //Gxyz,Axyz
//	int movavg[6];
//	int biased[6];

	void GetBias(float * const avg, float * const stdev) const;
	bool InitGyro(void);

	void ReadEncoder(void);
	void ReadAccGyro(void);

public:
	Odometry(void);

	bool Initialize(void);

	int raw[6]; //後で戻す
	int movavg[6];
	int biased[6];

	void Sample(void);
	void SetPose(float x, float y, float yaw);
	void GetPose(float *x, float *y, float *yaw);

	void SetOffsetYaw(const float offset);
	static constexpr int32_t SamplingFrequency = 1000; //TIM2の割り込み周波数と一致
	//正直madgwickfilterがそんなに周波数出るかわからん
};



















