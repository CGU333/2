/*
 * servoc.cpp
 *
 *  Created on: 30 Oct 2021
 *      Author: carlos
*/
#include <bcm2835.h>
#include <stdio.h>
#include <unistd.h>
#include <cstdint>

#include "servo.h"
#include "PCA9685.h"

#define	FREQUENCY 			60
#define MIN_PULSE_WIDTH		600
#define MAX_PULSE_WIDTH		2400
#define DEFAULT_PULSE_WIDTH	1500

Servo :: Servo(uint8_t channel, uint8_t offst = 0, bool lck = true, uint8_t address){
	if (channel < 0 || channel > 16)
		printf ("Servo channel  is not in range (0, 15)\n");
	//self._debug_("Debug on")

	this->channel=channel; //se indica con this ya que tienen mismo nombre
	offset=offst;
	lock=lck;
	printf("servo 1.2 FREQUENCY\n");
	frequency = FREQUENCY;
		printf("servo 1.3 inicializar i2c para pwm\n");
	setup();
		printf("servo 1.4 write_angle(90)\n");
	write_angle(90);
		printf(" ********* servo inicializado y configurado a 90 grados :) ***********\n");
}

/**
*	Convierte un ángulo de giro al correspondiente valor de ancho de ciclo
**/
void Servo :: setup()
{
	this->pwm.setup();
}

/**
*	Devuelve la frecuencia actual
*	@return frequency
**/
float Servo :: getFrequency()
{
	return this->frequency;
}

/**
*	Establece la frecuencia de funcionamiento
*	@param frequency
**/
void Servo :: setFrequency(float frequency)
{
	this->frequency = frequency;
	this->pwm.frequency = frequency;
}

/**
*	Devuelve la offset actual
*	@return offset
**/
uint8_t Servo :: getOffset()
{
	return this->offset;
}

/**
*	Establece el offset
*	@param offset
**/
void Servo :: setOffset(uint8_t offset)
{
	/* Set offset for much user-friendly */
	this->offset = offset;
	//debug('Set offset to %d' % this.offset);
}

/**
*	Establece el ángulo de giro
**/
void Servo :: write_angle(uint8_t angle)
{
	/* Turn the servo with giving angle. */
	if (this->lock){		//--> Terminar el lock
		if (angle > 180)
			angle = 180;
		if (angle < 0)
			angle = 0;
	}
	else {
		if (angle <0 || angle>180){
			printf("Servo %i turn angle %i is not in (0, 180).", channel, angle);
		}
	}
	uint8_t val = this->angle_to_analog(angle);
	val += offset;
	this->pwm.write(channel, 0, val);
	printf("Servo::write_angle() el valor a setear en las ruedas delanteras es de %d y el offset es  %d *_* \n", val, offset);
}

uint8_t Servo :: angle_to_analog(uint8_t angle) {
	uint8_t pulse_wide = this->pwm.map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
	uint8_t analog_value = int(float(pulse_wide) / 1000000 * this->frequency * 4096);
	printf("el valor analogico es %d   **  \n", analog_value);
	return analog_value;
	//printf("el valor analogico introducido es %d   **  \n", pulse_wide);
	//return pulse_wide;
	//https://robojax.com/learn/arduino/?vid=robojax_PCA9685-V1

}

Servo :: ~Servo() {
	printf("~Servo() Borramos objeto\n");
}

int main_servo(void){

	return 0;
}


