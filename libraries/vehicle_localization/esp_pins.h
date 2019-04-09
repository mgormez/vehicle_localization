/*
 * This file is part of the master thesis "Vehicle localization in underground
 * tunnels using ultra-wideband ranging" submitted in order to be awarded the
 * Master's Degree in Electrical Engineering.
 *
 * This file defines maps the useful pins of the @microcontroller used 
 * throughout the project and maps
 *
 * @microcontroller: esp8266, mounted on development board esp12e devkit v2, DoIt (old)
 * @microcontroller: esp8266, mounted on development board esp12e devkit v2, Adafruit Feather HUZZAH ESP8266 (new)
 * @sensor: dwm1000
 *
 * @author: Michael Gormez - michaelgormez@gmail.com;mgormez@ulb.ac.be
 * @supervisor: Prof. Dr. Ir. Francois Quitin
 * @cosupervisor: Dr. Ir. Michel Osee
 * @date: Academic year 2018-2019
*/

#ifndef ESP_PINS_H
#define ESP_PINS_H

/*******************************************************************************
* PIN VALUES
*******************************************************************************/
// pin names
#define LED 	2	// both esp8266 boards (old - DoIt, new - Adafruit Feather) have a led on pin 2
#define IRQ 	0	// interrupt pin connected to GPIO0 (CANNOT BE CONNECTED TO GPIO16!!!)

#define SCK 	14	//HSCLK on GPIO14
#define MOSI 	13	// Master Output Slave Input: The master uses this pin to send data to the slave
#define MISO 	12	// Master Input Slave Output: The slave uses this pin to send data to the master
#define SS 		15	// Slave Select: The master sets the SS pin low for the slave he wants to communicate with

#endif
