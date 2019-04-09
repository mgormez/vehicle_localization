



#ifndef UC_H
#define UC_H

#include <Arduino.h>
#include "sensor.h"

class uC
{

	SENSOR* dwm_1000;	//private reference to sensor object

public:
	uC();

	/*
	 * configure the sensor
 	*/
	void config_sensor();
	void use_sensor();
};

#endif