



#include <SPI.h>

#include <esp_pins.h>
#include <uC.h>

// location of temporary .o files: C:\Users\mgorm\AppData\Local\Temp\arduino_build_816826\libraries
// location of Arduino.h library: 
// location of SPI.h library(arduino library): C:\Program Files (x86)\Arduino\hardware\arduino\avr\libraries\SPI\src

uC esp;	//initialise an instance of our microcontroller which is an esp

void setup()
{

	Serial.begin(9600);
	Serial.println(" ");
	Serial.println("---------------------------------------------------------");
	Serial.println("---------------------Michael Gormez----------------------");
	Serial.println("------------Vehicle localization master thesis-----------");
	Serial.println("---------------------------------------------------------");
	configure_pins();	//configure pin modes
	delay(10);
	// digitalWrite(LED,HIGH);
	// delay(1500);
	// digitalWrite(LED,LOW);
	// delay(1500);

	esp.config_sensor();
}

void configure_pins() 
{
	/*
	 * Configure pins of the microcontroller for the SPI communication
	*/
	// pinMode(SCK, INPUT);
	// pinMode(MISO, INPUT);
	// pinMode(MOSI, OUTPUT);
	// pinMode(SS, OUTPUT);
	SPI.begin();	// configures SS,SCK and MOSI as output and MISO as input
	pinMode(LED, OUTPUT);
	// pinMode(IRQ, INPUT);	// attach interrupt?
}

void loop()
{
	digitalWrite(LED,HIGH);
	delay(300);
	digitalWrite(LED,LOW);
	delay(600);
	esp.use_sensor();

}