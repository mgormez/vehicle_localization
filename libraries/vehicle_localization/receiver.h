



#ifndef RECEIVER_H
#define RECEIVER_H

#include "sensor.h"

class Receiver : public Sensor
{

private:
	uint16_t frame_timeout_delay;
	uint16_t pac;
	uint16_t sfd;

public:
	Receiver();

	void clear_rx_interrupts();
	void auto_receive();

	void rx_start();

	/*
	 * receiver only reset (p195/244, last line of SOFTRESET section)
	*/
	void rx_reset();

	/*
	 * set default values of the channel. Need to be corrected taking into account 2.5.5 page 22/244
	*/
	void receiver_init();
	void rx_receive_data();
	void rx_receive_data(uint8_t * rx_buffer);

	uint8_t channel_valid_preamble_code(uint8_t preamble_code);
	PE channel_valid_preamble_size( PE preamble_size);


	void receiver_update_channel();
	void receiver_update_bitrate();
	void receiver_update_prf();
	void receiver_update_preamble_code();
	void receiver_update_preamble_size();
	void receiver_update_frame_timeout_delay();

	/*
	 * 7.2.40.7, update automatic timeout of detecting the SFD after having detected the preamble
	*/
	void receiver_update_SFD_timeout();
	
	void receiver_update_pac_size();

	void show_yourself();

	void polling_read_data();

};

#endif
