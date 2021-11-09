/*
**********************************************************************
* Filename    : PCF8591.cpp
* Description : A module to read the analog value with ADC PCF859
* by: Carlos Grijalvo
*/

using namespace std; //para evitar poner std:: en cada definicion de vectores

#include <bcm2835.h>
#include "PCF8591.h"
#include <unistd.h>

//#define AD_CHANNEL [] = [0x43, 0x42, 0x41, 0x40] // direcciones de lectura de bytes del SMBUS para el ADC

//Declaracion de los metodos de la clase PCF8591

	//definicion del constructor
PCF8591 :: PCF8591(uint8_t addr){  //adrr = 0x48
	this->addr = addr;        //se podria pasar la direccion 0x48 directamente al crear el metodo pero al pasarlo desde el .h esta mas encapsulado
	this->refV = 3.3f;        //en fichero origen indican 5.0f (5 voltios);pongo 3.3V para probar
	this->sendBuf[3] = {0};   // Inicializamos todos los valores a 0 (3 es la longitud total del array de chars)
	this->errCode = 0;
	   // Inicializar el i2c
	printf("PCF8591 Init...\n");
	bcm2835_init();
	printf("PCF8591 i2c begin\n");
	if(!bcm2835_i2c_begin())
	{
		printf("bcm2835_i2c_begin failed");
		exit(0);
	}
	bcm2835_i2c_setSlaveAddress(this->addr); //direccion I2C = 0x48
	bcm2835_i2c_set_baudrate(100000);  //frecuencia por el I2C = 100MHz
}
uint8_t PCF8591 :: read(uint8_t channel){ // channel = 0 o 1 o 2 o 3
	//si se pasa como canal en lugar de como valor de array, se declararia como: unint8_t chn
	// al pasar el parametro como entero, llamaremos a la funcion asi: read(0), read(1), etc

	bcm2835_i2c_setSlaveAddress(this->addr); // establecido I2C en  0x48

	//pasar el canal como el valor entero de un array (0,1,2,...) ademas de encapsular, facilita la forma de indicar las direcciones
	sendBuf[0] = 0x40 | channel; //para leer, primero se comprueba no hay error al escribir en el canal indicado

	if((errCode = bcm2835_i2c_write(sendBuf,1)))  //lectura de tipo char
		printf("bcm2835_i2c_write failed at %s%d, errCode = 0x%x\n",__FILE__,__LINE__, errCode);
	char byte;
	if((errCode = bcm2835_i2c_read(&byte,1))) //lectura de un char byte en el canal indicado
		printf("bcm2835_i2c_read failed at %s%d, errCode = 0x%x\n",__FILE__,__LINE__, errCode);
	return byte;  // return = 0-255
}
// mostrar el valor de cada sensor, leido desde la direccion especificada.
// al definirlos de esta manera, los llamaremos directamente:  sensor_1= PCF8591.A1 por ejemplo
uint8_t PCF8591 :: A0(){  // lectura I2C de la direccion 0x43
	return this->read(2);
}
uint8_t PCF8591 :: A1(){  // lectura I2C de la direccion 0x42
	return this->read(1);
}
uint8_t PCF8591 :: A2(){  // lectura I2C de la direccion 0x41
    return this->read(3); //o
}

/*
//los siguientes metodos proceden del controlador de ligh follower (ficheros python, carpeta picar)
//metodo para convertir los datos leidos de los 3 sensores en una lista tipo array y mostrar los 3 valores leidos. 
std::vector<uint8_t> PCF8591 :: analog_result() {
	std::vector <uint8_t> lista_analogica(3);           //Inicializamos el array de char unsigned int7
	lista_analogica[0] = A0();
	lista_analogica[1] = A1();
	lista_analogica[2] = A2();
	return lista_analogica; // Devolvemos el objeto
}
*/
vector<bool> PCF8591 :: digital_result(){
//	std::vector<uint8_t> analog_list = this->analog_result();
	vector <uint8_t> lista_analogica(3);
	vector<bool> digital_list(3);

	//digital_list.push_back(0);  //inicializar vector
	//digital_list= {0,0,0};      //inicializar vector
	lista_analogica[0] = A0();
	lista_analogica[1] = A1();
	lista_analogica[2] = A2();
	//printf("A0 = %d\n", lista_analogica[0]);
	//printf("A1 = %d\n", lista_analogica[1]);
	//printf("A2 = %d\n", lista_analogica[2]);
	int i=0;
	/*************************************************************************
		//si cuando hay menos luz detectada, aumenta el valor, configurar valor de referencia
		// se puede configurar a un poco mas del valor leído, para darle más sensibilidad
		this->referencias[0]=lista_analogica[0]+5 ;
		this->referencias[1]=lista_analogica[1]+5 ;
		this->referencias[2]=lista_analogica[2]+5 ;
	****************************************************************/

	for (i=0;i<3;i++){
		if (lista_analogica[i] >= this->referencias[i]){
			//digital_list.insert(i)=0;              //añade a la lista el valor 0 en la posicion i del array
			digital_list[i]=0;
		}
		else if	(lista_analogica[i] < this->referencias[i]){
			//digital_list.insert(i)=1 ;             // añade a la lista el valor 1 en la posicion i del array
			digital_list[i]=1;
		}
	}
	return digital_list;
}


//Metodo para salir
PCF8591 :: ~PCF8591(){
	// Cerramos i2c
	bcm2835_i2c_end();
	bcm2835_close();
}

// Metodo para testear el I2C (los 3 sensores del modulo Ligh Follower)
void PCF8591 :: test(){
    printf("test running...\n");
	uint8_t a0; //sensor izquierdo en sentido marcha adelante
	uint8_t a1; //centro
	uint8_t a2; //sensor derecho en sentido marcha hacia adelante

	bool digital_a0;  //valor digital (0 o 1) de cada sensor
	bool digital_a1;
	bool digital_a2;

	printf("\n init \n");
	//printf("\n %d \n", lista_digital[0]);
	//if(!PCF8591.__init__()){
	printf(" antes de leer \n");
	int i = 0;
	for (i=0;i<2000;i++){
		// obtener el valor leido del I2C para los 3 sensores correspondientes al modulo Ligh Follower
    	a0 = this->A0();
    	a1 = this->A1();
    	a2 = this->A2();
       	printf("%d , %d , %d\n", a0, a1, a2);
    	vector<bool> lista_digital = this->digital_result();
       	//mostrar los valores leidos en formato digital
    	digital_a0=lista_digital[0];
    	digital_a1=lista_digital[1];
    	digital_a2=lista_digital[2];

     	printf("[%d] [%d] [%d]\n", digital_a0, digital_a1, digital_a2);
     	sleep(1);
	}
	//printf("valores %s", this->analog_result());
    sleep(1);
}

int main_PCF8591(void){
	printf("main 1\n");
	PCF8591 Objeto(0x48);
	//miObjeto.__init__();
	printf("main 2\n");
    try {
    	printf("try i2c\n");
    	Objeto.test();
		printf("fin test\n");
    }
	catch (...) {
		printf("main 3\n");
		Objeto.~PCF8591();
		printf("main 4\n");
	}
	printf("fin main ^_^\n");
	return 0;
}

//*****************************************//
/*
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define DEV_I2C     "/dev/i2c-1"

#define SLAVE_ADDR 0x48 // EMC1414 I2C slave address

//#define COMBINED_TRANSCTION
int main2(int argc, char *argv[])
{
    int fd;
    int ret;
    unsigned char buf[2];

    // open device node
    fd = open(DEV_I2C, O_RDWR);
    if (fd < 0) {
        printf("ERROR open %s ret=%d\n", DEV_I2C, fd);
        return -1;
    }

    if (ioctl(fd, I2C_SLAVE, SLAVE_ADDR) < 0) {
        printf("ERROR ioctl() set slave address\n");
        return -1;
    }

#ifdef COMBINED_TRANSCTION
    struct i2c_rdwr_ioctl_data data;
    struct i2c_msg messages[2];

    // Set conversion rate
    buf[0] = 0x04; // Conversion rate register address
    buf[1] = 0x04; // Set conversion rate to 1 second
    messages[0].addr  = SLAVE_ADDR; //device address
    messages[0].flags = 0; //write
    messages[0].len   = 2; //longitud
    messages[0].buf   = buf; //data address

    data.msgs  = &messages[0];
    data.nmsgs = 1;
    if (ioctl(fd, I2C_RDWR, &data) < 0) {
        printf("ERROR ioctl() conversion rate\n");
        return -1;
    }

    // Read temperature
    buf[0] = 0x00; // Internal Diode High Byte register address
    buf[1] = 0;    // clear receive buffer
    messages[0].addr  = SLAVE_ADDR; //device address
    messages[0].flags = 0; //write
    messages[0].len   = 1;
    messages[0].buf   = &buf[0]; //data address

    messages[1].addr  = SLAVE_ADDR; //device address
    messages[1].flags = I2C_M_RD; //read
    messages[1].len   = 1;
    messages[1].buf   = &buf[1];

    data.msgs  = messages;
    data.nmsgs = 2;
    while (1) {
        if (ioctl(fd, I2C_RDWR, &data) < 0) {
            printf("ERROR ioctl() read data\n");
            return -1;
        }

        printf("Temperature is %d\n", buf[1]);
        sleep(1);
    }

#else
    // Set conversion rate
    buf[0] = 0x04; // Conversion rate register address
    buf[1] = 0x04; // Set conversion rate to 1 second
    ret = write(fd, buf, 2);
    if (ret != 2) {
        printf("ERROR write() conversion rate\n");
        return -1;
    }

    // Read temperature
    // Set internal address register pointer
    buf[0] = 0x00; // Internal Diode High Byte register address
    ret = write(fd, &buf[0], 1);
    if (ret != 1) {
        printf("ERROR write() register address\n");
        return -1;
    }

    while (1) {
        // Read temperature
        // Read data
        buf[1] = 0; // clear receive buffer
        ret = read(fd, &buf[1], 1);
        if (ret != 1) {
            printf("ERROR read() data\n");
            return -1;
        }

        printf("Temperature is %d\n", buf[1]);
        sleep(1);
    }
#endif

    // close device node
    close(fd);

    return 0;
}
*/
