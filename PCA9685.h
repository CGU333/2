#ifndef PCA9685_H_
#define PCA9685_H_
/*
 * PCA9685.h
 *
 *  Created on: 29 Oct 2021
 *      Author: carlos
 */

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


// Clase para el control del PWM
class PCA9685{

	public:
			PCA9685(uint8_t address=0x40);
			void setup();
			void set_PWM_freq(float freq);
			float get_PWM_freq();
			void write (uint8_t channel, uint32_t pulseWidth);
			void write (uint8_t channel, uint16_t on, uint16_t off);
			void write_all_value (uint16_t on, uint16_t off);
			uint8_t map(int x, int in_min, int in_max, int out_min, int out_max);
			~PCA9685();

			float frequency;

	private:
		uint8_t addr;
		uint32_t T;  //PWM Period[us]
		char sendBuf[5]; // Inicializamos todos los valores a 0 (3 es la longitud total del array de chars)
		uint8_t errCode;

		uint8_t read_byte_data(uint8_t addr);
		void write_byte_data(uint8_t addr, uint8_t d);
		//void check_i2c_issues();  //este metodo solo busca errores en i2c. no necesario por el momento

};


#endif /* PCA9685_H_ */
