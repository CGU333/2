/*
**********************************************************************
* Filename    : TB6612.h
* Description : A module to read the analog value with ADC PCF859
* Created by  : Carlos Grijalvo
*/

#ifndef TB6612_H_
#define TB6612_H_

#include <unistd.h>
#include <ctime>
#include <cstdio>
#include <cstdint>     // para el uint8_t --> typedef unsigned char uint8_t;
#include <bcm2835.h>  //equivalente en Python a importar SMBUS para los pines de entrada/salida.
				//existe opcion para habilitar esta biblioteca en Buildroot
				//En el futuro se sustituira por ficheros linux
#include <vector>     //añadimos esta libreria para poder utilizar vectores ya que en c++ es mejor que los arrays.
					  // vectores en c++ vienen con varias funciones que podemos utilizar. es un equivalente a usar listas
using namespace std;  //para evitar poner std:: en cada definicion de vectores

//Clase para control de los 2 motores DC traseros
class Motor{
	public:
		Motor(uint8_t direction_channel, float pwm, bool offset);
		//void _debug_(char mensaje);
		int getspeed();
		void setspeed(int velocidad);
		void forward();
		void backward();
		void stop();
		bool getoffset();
		void setoffset(bool offset);
		uint8_t getpwm();
		void setpwm(uint8_t value);
		~Motor();

		uint8_t direction_channel;
		int forward_offset;
		int backward_offset;
		int speed=0;
		uint8_t pwm;
		bool offset;

};

#endif /* TB6612_H_ */
