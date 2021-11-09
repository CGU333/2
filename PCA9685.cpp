/*
 * PCA9685.cpp
 *
 *  Created on: 29 Oct 2021
 *      Author: carlos
 */
using namespace std; //para evitar poner std:: en cada definicion de vectores

#include <bcm2835.h>
#include "PCA9685.h"
#include <unistd.h>
#include <stdio.h>      /* printf */
#include <math.h>       /* floor */

#define _MODE1            0x00
#define _PRESCALE         0xFE
#define _MODE2            0x01

#define _SUBADR1          0x02
#define _SUBADR2          0x03
#define _SUBADR3          0x04

#define LED0_ON_L         0x6
#define LED0_ON_H         0x7
#define LED0_OFF_L        0x8
#define LED0_OFF_H        0x9
#define _ALL_LED_ON_L     0xFA
#define _ALL_LED_ON_H     0xFB
#define _ALL_LED_OFF_L    0xFC
#define _ALL_LED_OFF_H    0xFD

#define _RESTART          0x80
#define _SLEEP            0x10
#define _ALLCALL          0x01
#define _INVRT            0x10
#define _OUTDRV           0x04

/*
* https://www.airspayce.com/mikem/bcm2835/pwm_8c-example.html
* https://programmerclick.com/article/1519244875/
* https://robojax.com/learn/arduino/?vid=robojax_PCA9685-V1
*/

// control del PWM
PCA9685 :: PCA9685(uint8_t address){ //addr=0x40 (PWM)
		this->addr = address;        //se podria pasar la direccion 0x48 directamente al crear el metodo pero al pasarlo desde el .h esta mas encapsulado
		this->sendBuf[5] = {'0'};   // Inicializamos todos los valores a 0 (3 es la longitud total del array de chars)
		this->errCode = 0;
		this->T = 0 ; //para empezar asi
		this->frequency = 60;
}

void PCA9685 :: setup(){
	//Inicializacion  Bus I2C
	printf("PCA9685 setup() 1.1 Init...\n");
	bcm2835_init();

	printf("PCA9685 setup() 1.2 i2c begin\n");
	if(!bcm2835_i2c_begin())
	{
		printf("bcm2835_i2c_begin failed");
		exit(0);
	}
	bcm2835_i2c_setSlaveAddress(this->addr); //direccion I2C = 0x40
	bcm2835_i2c_set_baudrate(100000);  //frecuencia por el I2C = 100MHz (100000) standard RPI3B+

//	printf("Inicializa PWM = 0 en todos los canales \n");
//	write(_MODE1, 0x00);
  /*  set_freq(50);
    write(0, 0, 4095);
    for (int i = 0; i < 16; i++) {
    	write(i, 0, 0);
	}
*/
//	printf("PCA9685 setup() 1.3 set_freq(60)\n");
//	this->set_PWM_freq(60);
	printf("PCA9685 setup() 1.4  FIN  ***** \n");
}


//https://programmerclick.com/article/1519244875/
void PCA9685 :: set_PWM_freq(float freq){
		// Adafruit servo driver:
		//http://wiki.sunfounder.cc/index.php?title=PCA9685_16_Channel_12_Bit_PWM_Servo_Driver
//freq *= 0.9;  //Correct for overshoot in the frequency setting (see issue #11).
	this->frequency=freq;
	T =static_cast<uint32_t>(1/freq*1000000);
	double osc_clock = 25000000;
	float prescaleval = static_cast<unsigned char>(osc_clock/4096/freq-1);
	uint8_t prescale = static_cast<uint8_t>(floor(prescaleval+0.5));

	uint8_t oldmode = read_byte_data(_MODE1);
	uint8_t newmode = (oldmode&0x7f) | 0x10;
	write_byte_data(_MODE1, newmode); // go to sleep

	write_byte_data(_PRESCALE, prescale); // set the prescaler
	oldmode &= 0xef;
	write_byte_data(_MODE1, oldmode);
	delay(2);
	write_byte_data(_MODE1, (oldmode|0x80));// 0x80  sunfounder picar-s python

}

float PCA9685 :: get_PWM_freq(){
	return (this->frequency);
}

uint8_t PCA9685 :: read_byte_data(uint8_t channel){

		bcm2835_i2c_setSlaveAddress(this->addr); // establecido I2C en  0x40
		//pasar el canal como el valor entero de un array (0,1,2,...) ademas de encapsular, facilita la forma de indicar las direcciones
		sendBuf[0] =  channel; // en este caso se indica asi ya que channel = _MODE1 = 0x00 que es el canal que queremos
		if((errCode = bcm2835_i2c_write(sendBuf,1)))  //escritura de un char byte en el canal indicado
			printf("(read_byte_data) bcm2835_i2c_write failed at %s%d, errCode = 0x%x\n",__FILE__,__LINE__, errCode);
		char byte;
		if((errCode = bcm2835_i2c_read(&byte,1))) //lectura de un char byte en el canal indicado
			printf("(read_byte_data) bcm2835_i2c_read failed at %s%d, errCode = 0x%x\n",__FILE__,__LINE__, errCode);
		return byte;  // return = 0-255
}

void PCA9685 :: write_byte_data(uint8_t addr, uint8_t data){
	printf("PCA9685::write_byte_data(...)\n");
	try {
		bcm2835_i2c_setSlaveAddress(addr);
		sendBuf[0] = addr;
		sendBuf[1] = data;
		if((errCode = bcm2835_i2c_write(sendBuf,2)))
			printf("(write_byte_data) bcm2835_i2c_write failed at %s%d, errCode = 0x%x\n",__FILE__,__LINE__, errCode);
	}
	catch (...) {
		printf("Error write_byte_data(...)\n");
	}
}


// EScribir el valor del PWM
void PCA9685 :: write(uint8_t channel, uint32_t pulseWidth){
	uint16_t off = static_cast<uint16_t> ((pulseWidth* 4096.0/T)*1.01);
	write(channel,0,off);
}

void PCA9685 :: write(uint8_t channel, uint16_t on, uint16_t off){
	bcm2835_i2c_setSlaveAddress(addr);
	sendBuf[0] = LED0_ON_L+4*channel;
	sendBuf[1] = on & 0x00FF;
	sendBuf[2] = on >> 8;
	sendBuf[3] = off & 0x00FF;
	sendBuf[4] = off >> 8;
	if((errCode = bcm2835_i2c_write(sendBuf,5)))
		printf("bcm2835_i2c_write failed at %s%d, errCode = 0x%x\n",__FILE__,__LINE__, errCode);
}

void PCA9685 :: write_all_value (uint16_t on, uint16_t off){
	try {
		bcm2835_i2c_setSlaveAddress(addr);
		sendBuf[0] = _ALL_LED_ON_L;
		sendBuf[1] = on & 0x00FF;
		sendBuf[2] = on >> 8;
		sendBuf[3] = off & 0x00FF;
		sendBuf[4] = off >> 8;
		if((errCode = bcm2835_i2c_write(sendBuf,5)))
			printf("bcm2835_i2c_write failed at %s%d, errCode = 0x%x\n",__FILE__,__LINE__, errCode);
	}
	catch (...) {
		printf("Error write_all_value(...)\n");
	}
}

uint8_t PCA9685 :: map(int x, int in_min, int in_max, int out_min, int out_max){
    //'''To map the value from arange to another'''
    return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

PCA9685 :: ~PCA9685(){
	bcm2835_i2c_end();
	bcm2835_close();
}

int main_PCA9685(void){
	printf("main_pca9685\n");
	PCA9685 pwm = PCA9685(0x40);  //I2C addr = 0x40 para el control de ruedas
	pwm.setup();
	uint8_t i;
	try {
		for (i=0;i<16;i++) {
        sleep(0.5);
        printf("\nChannel %d\n", i);
        sleep(0.5);
        uint8_t j;
        	for (j=0; j< 4096; j++) {
            pwm.write(i, 0, j);
            printf("PWM value: %d", j);
            sleep(0.0003);
        	}
		}
	}
	catch (...) {
		pwm.~PCA9685();
	}

    return 0;
}



/*
 *
 *
 * bcm2835_gpio_fsel(pin, BCM2835_GPIO_FSEL_ALTn)	Selecciona la función alternativa n del pin.
bcm2835_pwm_set_clock(div)	Configura el divisor para el reloj de 19.2MHz.
bcm2835_pwm_set_mode(canal, ms, activo)	Configura el modo del canal PWM y/o lo activa.
bcm2835_pwm_set_range(canal, range)	Configura el rango del pulso en un canal PWM.
bcm2835_pwm_set_data(canal, v)	Configura la anchura de pulso del canal PWM.
 *
 *
 *
 *
 *
 * */



