#ifndef PCF8591_H_
#define PCF8591_H_
/*
**********************************************************************
* Filename    : PCF8591.h
* Description : read the analog value with ADC
* Created by  : Carlos Grijalvo
*/
#include <unistd.h>
#include <ctime>
#include <cstdio>
//#include <iostream.h> //esta es de C++ para sacar mensajes por pantalla en la consola, equivalente al printf en C
#include <cstdint>  // para el uint8_t --> typedef unsigned char uint8_t;
#include <bcm2835.h>  //equivalente en Python a importar SMBUS para los pines de entrada/salida.
				//existe opcion para habilitar esta biblioteca en Buildroot
				//En el futuro se sustituira por ficheros linux
#include <vector>  //añadimos esta libreria para poder utilizar vectores ya que en c++ es mejor que los arrays.
					// vectores en c++ vienen con varias funciones que podemos utilizar. es un equivalente a usar listas
using namespace std; //para evitar poner std:: en cada definicion de vectores

// Definir Clase de tipo PCF8591 (lectura i2c 3 senssores light follower)
class PCF8591{
	public:

		PCF8591(uint8_t addr = 0x48);  		// constructor de la clase
    	//bool __init__();	//esto se debe incluir en el constructor en C (en python sí existe)
		uint8_t read(uint8_t chn);  // lectura del ADC (para los sensores), en el canal indicado como parametro
				// pasamos el canal como un entero en lugar de como uint8_t

// A0, A1, A2 y A3 metodos directos para el ejemplo de test
		uint8_t A0();    // llamar directamente al valor leido de los canales especificados al definir el metodo (0x43)
		uint8_t A1();    // llamar directamente al valor leido de los canales especificados al definir el metodo (0x42)
		uint8_t A2();    // llamar directamente al valor leido de los canales especificados al definir el metodo (0x41)
		uint8_t A3();    // llamar directamente al valor leido de los canales especificados al definir el metodo (0x40)

	void test();             //ejemplo para comprobar funcionamiento de este modulo

	         //std::vector<uint8_t> analog_result(); //vectores, explicado por Victor en laboratorio de instrumentacion --> en c++ mejor usar vectores que arrays
	vector<uint8_t> analog_result();  //no hace falta poner std:: como en a linea anterior al definir arriba "using namespace std"
	vector<bool> digital_result();

	//destructor de clase
	~PCF8591();        //destructor de la clase PCF8591

//private:
		uint8_t addr;
		float refV=0;
		char sendBuf[3];
		uint8_t errCode=0;
		int referencias[3] ={50,50,50}; //valores para ligh follower bien iluminado detectando oscuridad

};

#endif

