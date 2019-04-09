



#include <Arduino.h>	//should be in header file?

#include "uC.h"

uC::uC()
{
	dwm_1000 = new SENSOR();// create object on the heap because we will use it for a long time
							// so we don't waste the stack
}

void uC::config_sensor()
{
	dwm_1000 -> dwm_init();
	Serial.println("end config sensor -- uC class");
}

void uC::use_sensor()
{
	dwm_1000 -> get_device_ID();
}