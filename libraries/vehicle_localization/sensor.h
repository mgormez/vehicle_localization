



#ifndef SENSOR_H
#define SENSOR_H

#define Bitset(var, bitno) ((var) |= 1UL<<(bitno))
#define Bitclear(var, bitno) ((var) &= ~(1UL<<(bitno)))

#include <stdint.h>

typedef enum
{ 
	_1 = 1U, 
	_2 = 2U, 
	_3 = 3U, 
	_4 = 4U, 
	_5 = 5U, 
	_7 = 7U 
}CHANNEL_NB;

typedef enum
{ 
	_16MHZ = 0b01U, 
	_64MHZ = 0b10U 
}PRF;

typedef enum
{ 
	_110KBPS = 0b00U,
	 _850KBPS = 0b01U,
	  _6800KBPS = 0b10U 
}BITRATE;

typedef enum
{
	_64 = 0b0001U, 
	_128 = 0b0101U, 
	_256 = 0b1001U,
	_512 = 0b1101U,
	_1024 = 0b0010U, 
	_1536 = 0b0110U,
	_2048 = 0b1010U, 
	_4096 = 0b0011U 
}PE;// preamble extension (p76/244) (preamble size)

class Sensor
{
protected:
	CHANNEL_NB channel_nb;
	PRF prf;
	BITRATE bit_rate;
	uint8_t preamble_code;
	PE preamble_size;
	void int2Bytes(const uint32_t data_int, uint8_t* data);
	void short2Bytes(const uint16_t data_short, uint8_t* data);

public:
	// A LOT OF THESE CAN BE PRIVATE!(not even needed to be protected)
	/*
	 * constructor
	*/
	Sensor();

	void wait_4_response();
	/*
	 * Default values for channel parameters 
	 * (channel #5, prf 16 MHz, bit rate 6800 kBps, preamble code 4, preamble size 128)
	*/
	void set_default_ranging_parameters();
	/*
	 * Set parameters of the communication (channel, prf, bit rate, preamble)
	*/
	void set_ranging_parameters();
	/*
	 * 
	*/
	void dwm_init();

	/*
	 * put sensor into IDLE mode
	*/
	void idle();
	/*
	 * soft reset of the decawave (p33/244)
	*/
	void soft_reset();

	/*
	 * for load LDE
	*/

	void load_LDE();
	
	void set_channel(CHANNEL_NB c);
	/*
	 * The transmitter antenna delay may be set to zero and a combined receiver 
	 * and transmitter delay value may be used for the receiver antenna
	*/
	void set_anchor_antenna_delay();

	/*
	 * 2.5.5 Default Configurations that should be modified
	*/
	void correct_default_configuration();

	void dwm_update_channel();

	uint8_t channel_valid_preamble_code(uint8_t preamble_code);

	PE channel_valid_preamble_size(PE preamble_size);
	/*
	 * set and unset the desired interrupts mask for tx and rx operations in SYS_MASK
	*/
	static void set_tx_interrupt_mask();
	static void unset_tx_interrupt_mask();
	static void set_rx_interrupt_mask();
	static void unset_rx_interrupt_mask();

	/*
	 *	clear the register at given address of given length
	*/
	static void clear_register(uint8_t register_address, uint16_t register_len);
	static void set_register(uint8_t register_address, uint16_t register_len);
	
	/*
	 * page 70, edge on which SPI communication starts
	 * maybe this is not correct and is also related to the SPI frequency used?
	*/
	void spi_init();
	/*
	 * SPI read and write functions. Need to be static to be used during the (static) interrupt
	*/
	static void read_spi(uint8_t address, uint16_t offset, uint8_t* spi_buffer, uint16_t len);
	static void write_spi(uint8_t address, uint16_t offset, uint8_t* spi_buffer, uint16_t len);
	/*
	 * 3MHz SPI used during the configuration step of the DW1000
	*/
	static void slow_read_spi(uint8_t address, uint16_t offset, uint8_t* spi_buffer, uint16_t len);
	static void slow_write_spi(uint8_t address, uint16_t offset, uint8_t* spi_buffer, uint16_t len);
	/*
	 * print register from MSB to LSB
	*/
	static void print_register(uint8_t* register_buffer, uint16_t register_len);
	static void set_bit(uint8_t* data, uint16_t len, uint8_t bit, uint8_t val);






};

#endif
