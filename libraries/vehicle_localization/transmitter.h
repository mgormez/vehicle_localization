



#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include "sensor.h"

// should we maybe make a reference to the super class (Sensor host = s, where s is the host sensor that created transmitter/receiver)
// to be able to keep track of the state of the program
class Transmitter : public Sensor
{
private:
	//TO DO
public:
	Transmitter();

	/*
	 * set default values of the channel. Need to be corrected taking into account 2.5.5 page 22/244
	*/
	void transmitter_init();
	/*
	 * clear system event status register of any TX flags that may have been raised
	*/
	// void clear_tx_interrupts(); // static to be able to call during static interrupt
	void tx_start();
	void tx_buffer_fill_data(uint8_t* data, uint8_t data_size);
	uint8_t channel_valid_preamble_code(uint8_t preamble_code);
	PE channel_valid_preamble_size( PE preamble_size);

	/*
	 * send default values sent to the dw
	*/
	void transmitter_update_tx_power();
	/*
	 * 7.2.41.5 p152/244
 	*/
	void transmitter_update_channel();
	void transmitter_update_bitrate();
	void transmitter_update_prf();
	void transmitter_update_preamble_code();
	void transmitter_update_preamble_size();
};

#endif
