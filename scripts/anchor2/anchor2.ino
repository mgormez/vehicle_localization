

/*
 * Version 05-5 : implement a ranging between tag and anchor
 * This version will compute the correct times
*/

#include <SPI.h>

#include <esp_pins.h>
#include <uC.h>
#include <constants.h>

// NLOS PARAMETERS: (should be done with a define)
// - receiver.cpp line321 (receiver_update_prf)
// - sensor.cpp line80 (correct_default_configuration)

// location of temporary .o files: C:\Users\mgorm\AppData\Local\Temp\arduino_build_816826\libraries
// location of Arduino.h library: 
// location of SPI.h library(arduino library): C:\Program Files (x86)\Arduino\hardware\arduino\avr\libraries\SPI\src
// C:\Users\mgorm\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\2.4.2\libraries\SPI

uC* esp = new uC(ANCHOR_WAIT_INIT); 	//initialise an instance of our microcontroller which is an esp

static int loop_counter = 0;

void setup()
{
	//UART & pin configuration of uC (here, esp8266)
	Serial.begin(9600);
	Serial.println("");	//the beginning of the communication is always bashed, offer this space as sacrifice
	Serial.println("---------------------------------------------------------");
	Serial.println("----------------------Michael Gormez---------------------");
	Serial.println("------------Vehicle localization master thesis-----------");
	Serial.println("---------------------------------------------------------");
	Serial.println("");
	delay(10);	//time for dw to go from wake up to init to idle
	esp -> set_number(0x2);
	esp -> config_pins();
	//TO DO: check the state of the uC object to turn on or off the LED to translate the RX or TX mode
	Serial.print("===> starting the ANCHOR");
	Serial.println(esp -> get_anchor_number());
	digitalWrite(LED,HIGH);	//turn LED OFF (=high) for RECEIVER

	esp -> config_sensor();
	esp -> correct_anchor_delay();
	esp -> get_device_ID();	//confirms good SPI communcation between microcontroller and sensor
	delay(10);
	esp -> start_reception();
}

void loop()
{
	esp -> ranging_loop();
}