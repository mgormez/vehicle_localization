/*
 * This file is part of the master thesis "Vehicle localization in underground
 * tunnels using ultra-wideband ranging" submitted in order to be awarded the
 * Master's Degree in Electrical Engineering.
 *
 * This file defines global constants used throughout the project, maps the 
 * useful pins of the @microcontroller used throughout the project and maps
 * the register names of the @sensor to their respective addresses
 *
 * @microcontroller: esp8266, mounted on development board esp12e devkit v2
 * @sensor: dwm1000
 *
 * @author: Michael Gormez - michaelgormez@gmail.com;mgormez@ulb.ac.be
 * @supervisor: Prof. Dr. Ir. Francois Quitin
 * @cosupervisor: Dr. Ir. Michel Osee
 * @date: Academic year 2018-2019
*/
// don't think this include guard is necessary? or maybe because it will say 
// that I redefine thoses constants?
// #ifndef ESP_PINS_H
// #define ESP_PINS_H
/*******************************************************************************
* PIN VALUES
*******************************************************************************/
// pin names
#define LED 16	//esp12e uC led connected on pin GPIO16
// #define LED2 2 // other led on GPIO2 
#define SCK 14	//HSCLK on GPIO14
#define MOSI 13	// Master Output Slave Input: The master uses this pin to send 
				// data to the slave
#define MISO 12	// Master Input Slave Output: The slave uses this pin to send 
				// data to the master
#define SS 15	// Slave Select: The master sets the SS pin low for the slave he
				// wants to communicate with
/*******************************************************************************
* REMARKABLE VALUES
*******************************************************************************/

// //pin input-output direction
// #define OUTPUT 1
// #define INPUT 0

// //pin value definition
// #define HIGH 1
// #define LOW 0

// // #endif
