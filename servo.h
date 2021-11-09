/*
 * servo.h
 *
 *  Created on: 30 Oct 2021
 *      Author: carlos
 */

#ifndef SERVO_H_
#define SERVO_H_

#include <bcm2835.h>
#include <stdio.h>
#include <unistd.h>
#include <cstdint>  // para el uint8_t --> typedef unsigned char uint8_t;

#include "PCA9685.h"
/**********************************************************************
* Filename    : miServo.h
* Description : Driver module for servo, with RpiCar S
* Author      : Carlos Grijalvo
* Version     : 1.0
**********************************************************************/

//#define SIG   0		//Pin de salida PWM

#define	MIN_PULSE_WIDTH 	600
#define	MAX_PULSE_WIDTH 	2400
#define	DEFAULT_PULSE_WIDTH 1500
#define	FREQUENCY 			60

#include "PCA9685.h"

class Servo{
public:


//	Servo (uint8_t channel, float offset=0, bool lock=1, uint8_t address =0x40);
	Servo (uint8_t channel, uint8_t offset, bool lock, uint8_t address=0x40);
	~Servo();
	//void servoCtrl(int dutyCycle);
	void setup();
	uint8_t angle_to_analog(uint8_t angle);
	float getFrequency();
	void setFrequency(float frequency);
	uint8_t getOffset();
	void setOffset(uint8_t offset);
	void write_angle(uint8_t angle);

	float frequency;
	uint8_t channel; //  1 - 16

	bool lock;
	uint8_t address = 0x40;
	PCA9685 pwm = PCA9685(address);

	int offset;
	void test();

};
#endif /* SERVO_H_ */
