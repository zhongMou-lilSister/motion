#pragma once

#ifndef _CONTROL_H
#define _CONTROL_H

#include <Servo.h>
#include <Arduino.h>

#endif

enum LIFT_STATE
{
	ON_DRAW,
	BETWEEN_NUMS,
	LIFTED,
	CLEAR
};

class Control
{
public:
	MControl(int pin_lift, int pin_left, int pin_right);
	void Calibrate();
  void DetachServo();
	void DrawNumber(float bx, float by, int num, float scale);
	void DrawWeatherIcon(float bx, float by, int type, float scale);
  
	void Lift(LIFT_STATE s);
	void GoTo(float pX, float pY);
  void Clear();
  void GoHome();
  uint8_t  rubberx = 82, rubbery = 48; //【此数值可能需要调节】
  
  Servo servo_lift;
  Servo servo_left;
  Servo servo_right;
  
private:
	// When in calibration mode, adjust the following factor until the servos move exactly 90 degrees
	// int SERVOFAKTOR = 550;
	int SERVOFAKTORLEFT = 670; //数值加大，顺时针旋转，减小则逆时针旋转 默认650
	int SERVOFAKTORRIGHT = 650; //数值减小，顺时针旋转，加大则逆时针旋转 默认650

	// Zero-position of left and right servo
	// When in calibration mode, adjust the NULL-values so that the servo arms are at all times parallel
	// either to the X or Y axis
	int SERVOLEFTNULL = 2070; //数值减小，顺时针旋转，加大则逆时针旋转 默认2000
	int SERVORIGHTNULL = 1000; //数值减小，顺时针旋转，加大则逆时针旋转  默认1000

	int LIFT0 = 2090; //落笔写字 
	int LIFT1 = 1950; //写字时抬臂动作 一般比lift0 小100左右
	int LIFT2 = 1850; //高抬笔架
	int LIFT3 = 2010; // erase


	uint8_t SERVOPINLIFT = 2; //抬笔舵机
	uint8_t SERVOPINLEFT = 3; //左臂舵机
	uint8_t SERVOPINRIGHT = 4; //右臂舵机

	// speed of lifting arm, higher is slower
	int LIFTSPEED = 2200;

	//摆臂的长度，根据图纸测量，无需改变
	int L1 = 35;
	int L2 = 57.1;
	int L3 = 14;

	//左右舵机轴心的位置，根据图纸测量，无需改变
	int O1X = 28.7; //box版 28.7，普通版22
	int O1Y = -25;
	int O2X = 53.9; //box版53.9， 普通版47
	int O2Y = -25;

	volatile float lastX = rubberx;
	volatile float lastY = rubbery;
	volatile int servoLift = 2000;
  
  volatile float inkr = -0.05;
  volatile float count = 0;

	void circleClockwise(float bx, float by, float radius, int start, int ende, float sqee);

	void circleAntiClockwise(float bx, float by, float radius, int start, int ende, float sqee);

	float return_angle(float a, float b, float c);

	void set_XY(float Tx, float Ty);
};
