/*
 * back_wheels.h
 *
 *  Created on: 2 Nov 2021
 *      Author: carlos
 */
#ifndef BACK_WHEELS_H_
#define BACK_WHEELS_H_

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

#include "PCA9685.h"
#include "TB6612.h"
#define CHANNEL_MOTOR_A  17  //channel
#define CHANNEL_MOTOR_B  18


class Back_Wheels{
	public:
		Back_Wheels();
		//void set_a_pwm(value)
		//void set_b_pwm(value)
		void forward();
		void backward();
		void stop();
		int getspeed();
		void setspeed(int velocidad);
		void test();
		~Back_Wheels();
		 //aqui irian los metodos de calibracion

		uint8_t turning_offset=0;
		int speed=0;
		bool forward_A=true;
		bool forward_B=true;
		uint8_t value =0;

		PCA9685 pwm = PCA9685 (0x40);

		Motor left_wheel= Motor(CHANNEL_MOTOR_A, 0, forward_A);
		Motor right_wheel= Motor(CHANNEL_MOTOR_B, 0, forward_B);
};

#endif /* BACK_WHEELS_H_ */
