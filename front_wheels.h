/*
 * front_wheels.h
 *
 *  Created on: 2 Nov 2021
 *      Author: carlos
 */

#ifndef FRONT_WHEELS_H_
#define FRONT_WHEELS_H_
#include <unistd.h>
#include <ctime>
#include <cstdio>
#include <cstdint>  // para el uint8_t --> typedef unsigned char uint8_t;
#include <bcm2835.h>  //equivalente en Python a importar SMBUS para los pines de entrada/salida.
				//existe opcion para habilitar esta biblioteca en Buildroot
				//En el futuro se sustituira por ficheros linux
#include <vector>  //añadimos esta libreria para poder utilizar vectores ya que en c++ es mejor que los arrays.
					// vectores en c++ vienen con varias funciones que podemos utilizar. es un equivalente a usar listas
using namespace std; //para evitar poner std:: en cada definicion de vectores

#include "servo.h"

#define FRONT_WHEEL_CHANNEL 0 // establece canal ruedas delanteras en 0

class Front_Wheels{
public:
	Front_Wheels(uint8_t chnl= FRONT_WHEEL_CHANNEL);
	~Front_Wheels();
	void turn_left();
	void turn_straight();
	void turn_right();
	void turn(uint8_t angle);
	uint8_t getchannel();
	void setchannel(uint8_t chn);

	uint8_t get_turning_max();
	void set_turning_max(uint8_t angle);
	uint8_t get_turning_offset();
	void set_turning_offset(uint8_t value);

	void ready();
	void calibration();
	void cali_left();
	void cali_right();
	void cali_ok();
	void test();

	uint8_t channel;  //  1 - 16
	Servo wheel = Servo(0, 0, 1, 0x40); // 1er parametro es channel
	uint8_t angle[3] = {0, 90, 180};

	uint8_t cali_turning_offset=0;
	uint8_t _turning_offset=0;
	uint8_t turning_max=45;
	uint8_t straight_angle=90;
	uint8_t min_angle=0;
	uint8_t max_angle=180;
};

#endif /* FRONT_WHEELS_H_ */
