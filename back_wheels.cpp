
/**********************************************************************
* Filename    : back_wheels.c
* Description : Driver module for back wheels, with RpiCar S
* Author      : Carlos Grijalvo
* Version     : 1.0
**********************************************************************/
#include <stdio.h>
#include "PCA9685.h"   //class PCA9685 (PWM)
#include "TB6612.h"    //class Motor
#include "back_wheels.h"

#define PWM_A 27
#define PWM_B 22

Back_Wheels :: Back_Wheels(){

//	left_wheel.setpwm = this->pwm.write(PWM_A, 0, int(this->pwm.map(value, 0, 100, 0, 4095)));
//	right_wheel.setpwm = this->pwm.write(PWM_B, 0, int(this->pwm.map(value, 0, 100, 0, 4095)));
	}

void Back_Wheels :: forward(){
	this->left_wheel.forward();
	this->right_wheel.forward();
	printf("Running forward");
}

void Back_Wheels :: backward(){
	this->left_wheel.backward();
	this->right_wheel.backward();
	printf("Running backward");
}

void Back_Wheels :: stop(){
	this->left_wheel.stop();
	this->right_wheel.stop();
	printf("Stop");
}

int Back_Wheels :: getspeed(){
	return this->speed;
}

void Back_Wheels :: setspeed(int velocidad){
	this->speed = velocidad;
	this->left_wheel.setspeed(this->speed);
	this->right_wheel.setspeed(this->speed);
	printf("Set speed to %d", this->speed);
}

Back_Wheels :: ~Back_Wheels(){
	bcm2835_close();
}

/*
	@property
	def debug(this):
		return this._DEBUG

	@debug.setter
	def debug(this, debug):
		// Set if debug information shows '''
		if debug in (True, False):
			this._DEBUG = debug
		else:
			raise ValueError('debug must be "True" (Set debug on) or "False" (Set debug off), not "{0}"'.format(debug))

		if this._DEBUG:
			printf(this._DEBUG_INFO, "Set debug on")
			this.left_wheel.debug = True
			this.right_wheel.debug = True
			this.pwm.debug = True
		else:
			printf(this._DEBUG_INFO, "Set debug off")
			this.left_wheel.debug = False
			this.right_wheel.debug = False
			this.pwm.debug = False
*/

/*
	def ready(this):
		// Get the back wheels to the ready position. (stop) '''
		this._debug_('Turn to "Ready" position')
		this.left_wheel.offset = this.forward_A
		this.right_wheel.offset = this.forward_B
		this.stop()

	def calibration(this):
		// Get the front wheels to the calibration position. '''
		this._debug_('Turn to "Calibration" position')
		this.speed = 50
		this.forward()
		this.cali_forward_A = this.forward_A
		this.cali_forward_B = this.forward_B

	def cali_left(this):
		// Reverse the left wheels forward direction in calibration '''
		this.cali_forward_A = (1 + this.cali_forward_A) & 1
		this.left_wheel.offset = this.cali_forward_A
		this.forward()

	def cali_right(this):
		// Reverse the right wheels forward direction in calibration '''
		this.cali_forward_B = (1 + this.cali_forward_B) & 1
		this.right_wheel.offset = this.cali_forward_B
		this.forward()

	def cali_ok(this):
		//Save the calibration value '''
		this.forward_A = this.cali_forward_A
		this.forward_B = this.cali_forward_B
		this.db.set('forward_A', this.forward_A)
		this.db.set('forward_B', this.forward_B)
		this.stop()
*/

	void Back_Wheels ::test(){
		//import time
		Back_Wheels back_wheels = Back_Wheels();
		sleep (1);
		try{
			int i;
			back_wheels.forward();
			for (i=0;i<101;i++){
				back_wheels.setspeed(i);
				printf("Forward, speed =%d", i);
				sleep(1);
			}
			for (i=100;i<-1;i--){
				back_wheels.setspeed(i);
				printf("Forward, speed =%d", i);
				sleep(1);
			}

			back_wheels.backward();
			for (i=0;i<101;i++){
				back_wheels.setspeed(i);
				printf("Backward, speed =%d", i);
				sleep(1);
			}
			for (i=100;i<-1;i--){
				back_wheels.setspeed(i);
				printf("Backward, speed =%d", i);
				sleep(1);
			}
		}
		catch (...) {
			//finally:
			back_wheels.stop();
			back_wheels.~Back_Wheels();
			printf("motores DC parados");
		}
	}

int main_back_wheels(void){
	printf("main 1\n");
	Back_Wheels Objeto;
	try {
    	printf("try i2c\n");
    	Objeto.test();
		printf("fin test\n");
    }
	catch (...) {

		Objeto.~Back_Wheels();
	}
	printf("fin main ^_^\n");
	return 0;
}

