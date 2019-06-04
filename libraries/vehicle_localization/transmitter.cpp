


// todo : put slow spi
#include <Arduino.h>
#include <cstring>
#include "transmitter.h"
#include "constants.h"

Transmitter::Transmitter()
{

}

void Transmitter::transmitter_init()
{
	idle();
	// set default values for channel parameters
	// set_default_ranging_parameters();
	// transmitter_update_values();
	// delay(10);

	// set desired values for channel parameters
	set_ranging_parameters();
	transmitter_update_values();
	
	char transmitter_init_config[128] = {0};
	Serial.println("transmitter state: ");
	sprintf(transmitter_init_config,"channel %u, prf %u, bit rate %u, preamble_code %u, preamble_size %u",channel_nb,prf,bit_rate,preamble_code,preamble_size);
	Serial.println(transmitter_init_config);
}

void Transmitter::transmitter_update_values()
{
	transmitter_update_channel();
	transmitter_update_bitrate();
	transmitter_update_prf();
	transmitter_update_preamble_code();
	transmitter_update_preamble_size();
}

void Transmitter::tx_start()
{
	// p79/244. Call once all the configuration has been done and the data putt in TX_BUFFER.
	// This is the last step before the message is sent
	// device should be in IDLE mode before this
	uint8_t tx_start[1] = {0};	//FIXME change name 
	
	set_bit(tx_start,1,TXSTRT_BIT,1);		// start transmission bit
	set_bit(tx_start,1,WAIT4RESP_BIT,1);	// wait for response bit
	// Serial.print("tx_start:: ");
	// Serial.println(tx_start[0],BIN);
	// we can check WAIT4RESP here !! 
	//If the transmitter is expecting a response, because dwm1000 can go automatically from this transmit to receive mode then.
	write_spi(SYS_CTRL,NO_SUB,tx_start,1);
}

void Transmitter::tx_buffer_fill_data(uint8_t* data, uint8_t data_size)
{	
	// p77/244
	uint8_t frame_length[1] = {0};	// should hold on 7 bits
	frame_length[0] = data_size + 2;
	frame_length[0] &= 0x7F;

	// p73/244. Transmit Frame Control - frame length
	// write only 1 byte otherwise the other elements will be overwritten!
	write_spi(TX_FCTRL,NO_SUB,frame_length,1);
	write_spi(TX_BUFFER,NO_SUB,data,frame_length[0]);
}

void Transmitter::transmitter_update_tx_power() 
{
	//7.2.31.4 Transmit Power Control Reference Values, p110/244
	//table 20
	uint8_t tx_power[4] = {0};
	switch (this -> bit_rate) {
	case _6800KBPS:
		switch (this ->prf) {
		case _16MHZ:
			switch (this ->channel_nb) {
			case _1: case _2:
				int2Bytes(0x15355575U, tx_power);
				break;
			case _3:
				int2Bytes(0x0F2F4F6FU, tx_power);
				break;
			case _4:
				int2Bytes(0x1F1F3F5FU, tx_power);
				break;
			case _5:
				int2Bytes(0x0E082848U, tx_power);
				break;
			case _7:
				int2Bytes(0x32527292U, tx_power);
				break;
			}
			break;
		case _64MHZ:
			switch (this ->channel_nb) {
			case _1: case _2:
				int2Bytes(0x07274767U, tx_power);
				break;
			case _3:
				int2Bytes(0x2B4B6B8BU, tx_power);
				break;
			case _4:
				int2Bytes(0x3A5A7A9AU, tx_power);
				break;
			case _5:
				int2Bytes(0x25456585U, tx_power);
				break;
			case _7:
				int2Bytes(0x5171B1D1U, tx_power);
				break;
			}
			break;
		}
		break;
	default:	// not 6800 kbps
		switch (this ->prf) {
		case _16MHZ:
			switch (this ->channel_nb) {
			case _1: case _2:
				int2Bytes(0x75757575U, tx_power);
				break;
			case _3:
				int2Bytes(0x6F6F6F6FU, tx_power);
				break;
			case _4:
				int2Bytes(0x5F5F5F5FU, tx_power);
				break;
			case _5:
				int2Bytes(0x48484848U, tx_power);
				break;
			case _7:
				int2Bytes(0x92929292U, tx_power);
				break;
			}
			break;
		case _64MHZ:
			switch (this ->channel_nb) {
			case _1: case _2:
				int2Bytes(0x67676767U, tx_power);
				break;
			case _3:
				int2Bytes(0x8B8B8B8BU, tx_power);
				break;
			case _4:
				int2Bytes(0x9A9A9A9AU, tx_power);
				break;
			case _5:
				int2Bytes(0x85858585U, tx_power);
				break;
			case _7:
				int2Bytes(0xD1D1D1D1U, tx_power);
				break;
			}
			break;
		}
		break;
	}
	slow_write_spi(TX_POWER, NO_SUB, tx_power, 4);
}

void Transmitter::transmitter_update_channel()
{
	uint8_t tc[1] = {0};
	uint8_t txctrl[4] = {0};
	switch (this -> channel_nb) {
		case _1:
			int2Bytes(0x00005C40U, txctrl);
			tc[0] =  0xC9U;
			break;
		case _2:
			int2Bytes(0x00045CA0U, txctrl);
			tc[0] =  0xC2U;
			break;
		case _3:
			int2Bytes(0x00086CC0U, txctrl);
			tc[0] =  0xC5U;
			break;
		case _4:
			int2Bytes(0x00045C80U, txctrl);
			tc[0] =  0x95U;
			break;
		case _5:
			int2Bytes(0x001E3FE0U, txctrl);
			tc[0] =  0xC0U;
			break;
		case _7:
			int2Bytes(0x001E7DE0U, txctrl);
			tc[0] =  0x93U;
			break;
	}
	slow_write_spi(RF_CONF,  0x0C, txctrl,  3);
	slow_write_spi(TX_CAL,  0x0B, tc,  1);
	transmitter_update_tx_power();
}

void Transmitter::transmitter_update_bitrate()
{
	uint8_t bit_rate [] = {(uint8_t)((this -> bit_rate << 5) | (1<<7))};
	write_spi(TX_FCTRL,  1, bit_rate,1);
	transmitter_update_tx_power();
}

void Transmitter::transmitter_update_prf()
{
	uint8_t prf [] = {(uint8_t)(this -> prf | (this -> preamble_size << 2))};
	write_spi(TX_FCTRL, 2, prf, 1);
	transmitter_update_tx_power();
}


void Transmitter::transmitter_update_preamble_code()
{
	uint8_t chan[2] = {0};
	read_spi(CHAN_CTRL, 2, chan, 2);
	uint8_t preamble_code = this -> preamble_code;
	chan[0] &= 0x3F;
	chan[1] &= 0xF8;
	chan[0] |= preamble_code << 6;
	chan[1] |= preamble_code >> 2;
	write_spi(CHAN_CTRL, 2, chan, 2);
}

void Transmitter::transmitter_update_preamble_size()
{
	uint8_t preamble_size [] = {(uint8_t)(this -> prf |(this -> preamble_size << 2))};
	write_spi(TX_FCTRL, 2,preamble_size, 1);
}
