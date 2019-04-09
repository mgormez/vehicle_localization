



#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <SPI.h>	// arduino library
#include <stdint.h>	//uint8_t

#include "constants.h"

class SENSOR
{
//private:
public:
	SENSOR();

	/*
	 * initial configuration of the sensor
 	*/
 	void dwm_reset();
	void config_init();
	void dwm_init();
	void get_device_ID();

};
void read_spi(uint8_t address, uint16_t offset, uint8_t* data, uint16_t len);
void write_spi(uint8_t address, uint16_t offset, uint8_t* data, uint16_t len);

#endif
