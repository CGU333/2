/**********************************************************************
* Filename    : front_wheels.c
* Description : Driver module for front wheels, with RpiCar S
* Author      : Carlos Grijalvo
* Version     : 1.0
**********************************************************************/

#include <stdio.h>
#include "servo.h"
#include "front_wheels.h"

	Front_Wheels :: Front_Wheels(uint8_t chnl){
		channel=chnl;
		int turning_offset=0;
	}

	void Front_Wheels :: turn_left(){
		printf("Turn left");
		this->wheel.write_angle(this->angle[0]);   //write_angle == metodo write en  Servo
	}

	void Front_Wheels :: turn_straight(){
		printf("Turn straight");
		this->wheel.write_angle(this->angle[1]);
	}

	void Front_Wheels :: turn_right(){
		printf("Turn right");
		this->wheel.write_angle(this->angle[2]);
	}

	void Front_Wheels :: turn(uint8_t angle){
		printf("Turn to %d " , angle);
		if (angle < this->angle[0])
			angle = this->angle[0];
		if (angle > this->angle[2])
			angle = this->angle[2];
		this->wheel.write_angle(angle);
	}

	uint8_t Front_Wheels :: getchannel(){
		return this->channel;
	}

	uint8_t Front_Wheels :: get_turning_max(){
		return this->turning_max;
	}

	void Front_Wheels :: setchannel(uint8_t chn){
		this->channel = chn;
	}

	void Front_Wheels :: set_turning_max(uint8_t angle){
		this->turning_max = angle;
		this->min_angle = this->straight_angle - angle;
		this->max_angle = this->straight_angle + angle;
		//angle[3]={min_angle, straight_angle, max_angle};
		this->angle[0]=min_angle;
		this->angle[1]=straight_angle;
		this->angle[2]=max_angle;
	}

	uint8_t Front_Wheels :: get_turning_offset(){
		return this->_turning_offset;
	}

	void Front_Wheels :: set_turning_offset(uint8_t value){
		this->_turning_offset = value;
		this->wheel.offset = value;
		this->turn_straight();
	}
/*
	void Front_Wheels :: ready(){
		// Get the front wheels to the ready position.
		//this._debug_('Turn to "Ready" position');
		this->wheel.offset = this->turning_offset;
		this->turn_straight();
	}

	void Front_Wheels :: calibration(){
		// Get the front wheels to the calibration position.
		//this._debug_('Turn to "Calibration" position');
		this->turn_straight();
		this->cali_turning_offset = this->turning_offset;
	}

	void Front_Wheels :: cali_left(){
		this->cali_turning_offset -= 1;
		this->wheel.offset = this->cali_turning_offset;
		this->turn_straight();
	}

	void Front_Wheels :: cali_right(){
		this->cali_turning_offset += 1;
		this->wheel.offset = this->cali_turning_offset;
		this->turn_straight();
	}

	void Front_Wheels :: cali_ok(){
		this->turning_offset = this->cali_turning_offset;

	}
*/
	Front_Wheels :: ~Front_Wheels() {
		printf("Borramos objeto");
	}

	void Front_Wheels :: test(){
		//Front_Wheels front_wheels;
		//try{
			while (1){
				printf("turn_left");
				this->turn_left();
				sleep(1);
				printf("turn_straight");
				this->turn_straight();
				sleep(1);
				printf("turn_right");
				this->turn_right();
				sleep(1);
				printf("turn_straight");
				this->turn_straight();
				sleep(1);
			}
		//}
		//catch(...){
			this->~Front_Wheels();
			printf("motor servo parado");
		//}
	}

	int main_front_wheels(void){
		printf("inicializar clase\n");
		Front_Wheels Objeto = Front_Wheels(); //  Front_Wheels Objeto(0x40);
		try {
	    	printf("try i2c y testeo \n");
	    	Objeto.test();   // las ruedas delanteras se configuran en el canal 0  en la direccion 0x40
			printf("fin test\n");
	    }
		catch (...) {
			printf("main catch\n");
			Objeto.~Front_Wheels();
		}
		printf("fin main ^_^\n");
		return 0;
	}


