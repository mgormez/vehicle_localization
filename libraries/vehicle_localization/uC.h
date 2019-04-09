




#ifndef UC_H
#define UC_H

#include "receiver.h"
#include "transmitter.h"
#include "constants.h"

// typedef enum{IDLE_STATE = 0x00U, RX_STATE = 0x01U, TX_STATE = 0x10U}STATE; // legacy (not suited for ranging)

typedef enum
{
	COMPUTING_STATE 		= 0x00U,	// used at the end of the ranging, while we compute the distance

	TAG_SEND_INIT 			= 0x01U,
	TAG_WAIT_MESSAGE_1 		= 0x04U,
	TAG_SEND_MESSAGE_2 		= 0x05U,
	TAG_WAIT_TIMES			= 0x08U,

	ANCHOR_WAIT_INIT 		= 0x02U,
	ANCHOR_SEND_MESSAGE_1 	= 0x03U,
	ANCHOR_WAIT_MESSAGE_2	= 0x06U,
	ANCHOR_SEND_TIMES		= 0x07U
}STATE;

typedef enum
{
	MESSAGE_INIT 	= 0x00U,
	MESSAGE_1 		= 0x01U,
	MESSAGE_2 		= 0x02U
}MESSAGE;

class uC
{
private:

	STATE state;
	Sensor* dwm_1000;	
	Receiver* receiver;	
	Transmitter* transmitter;
	uint8_t anchor_number;	// used by anchor nodes
	uint8_t current_ranging_anchor_number;	// used by tag
	uint64_t TOF_val;
	double TOF;
	double distance;
	uint8_t tag_tx_init_time_stamp[TIME_STAMP_LEN];
	uint8_t tag_rx_message1_time_stamp[TIME_STAMP_LEN];
	uint8_t tag_tx_message2_time_stamp[TIME_STAMP_LEN];
	uint64_t t0;
	uint64_t t3;
	uint64_t t4;

	uint8_t anchor_rx_init_time_stamp[TIME_STAMP_LEN];
	uint8_t anchor_tx_message1_time_stamp[TIME_STAMP_LEN];
	uint8_t anchor_rx_message2_time_stamp[TIME_STAMP_LEN];
	uint64_t t1;
	uint64_t t2;
	uint64_t t5;

	uint64_t t_round_tag;
	uint64_t t_reply_tag;
	uint64_t t_round_anchor;
	uint64_t t_reply_anchor;

	static uint8_t msg [128];	// length encoded on 7 bits
	static uint8_t msg_length;
	static volatile bool tx_done;
	static volatile bool rx_done;
	static uint8_t isr_status_register[SYS_STATUS_LEN];
	static uint16_t isr_read_counter;

	uint8_t time1[TIME_STAMP_LEN] = {0};
	uint8_t time2[TIME_STAMP_LEN] = {0};

public:
	/*
	 * constructors
 	*/
	uC();
	uC(STATE initial_state);
	
	/*
	 * Main program loop. It implements the state machine necessary for performing the 
	 * Symmetric Double Sided Two Way Ranging (SDS-TWR). steps:
	 * 1. tag sends SEND_INIT (initial state of the tag). Tag goes to WAIT_MESSAGE_1
	 * 2. anchor waits for SEND_INIT message being in WAIT_INIT state (initial state of anchor)
	 * 3. once anchor receives SEND_INIT, it goes to SEND_MESSAGE_1. Once it's sent, it goes to WAIT_MESSAGE_2
	 * 4. once tag receives SEND_MESSAGE_1, it goes to SEND_MESSAGE_2. The tag then waits for the anchor's times in WAIT_ANCHOR_TIMES
	 * 5. once the anchor receives SEND_MESSAGE_2, the ranging is done. The anchor then sends it times in SEND_ANCHOR_TIMES
	*/
	void use_sensor();
	/*
	 * Computes the distance (on the tag side) based on the time stamps collected by the tag
	 * and those sent by the anchors
	*/
	void compute_distance();
	/*
	 * Determines based on the received message if the anchor receiving this message is the 
	 * recipient of this message or if it is addressed to another anchor.
	*/
	bool is_recipient();
	/*
	 * contains all the steps to follow for making a transmission. This function is called by the esp
	 * if the sensor is in TX mode. It is called once at configuration and each time there is an 
	 * interrupt (in the correct state)
	*/
	void start_transmission();
	/*
	 * contains all the steps to follow for making a transmission. This function is called by the esp
	 * if the sensor is in TX mode. It is called once at configuration and each time there is an 
	 * interrupt (in the correct state)
	*/
	void start_reception();

	/*
	 * clear any left over TX interrupts in the status register
	*/
	static void clear_tx_interrupts();
	static void clear_rx_interrupts();

	/*
	 * configure pins of the microcontroller for the SPI communication
	 * and interrupt line
	*/
	void config_pins();
	/*
	 * configure the communication between the (slave) sensor and the (master) microcontroller:
	 * interrupt conditions etc.
	*/
	void config_uc_to_sensor_communication();
	/*
	 * configure the sensor to sensor communication, meaning the different channel parameters:
	 * channel number, data rate, pac size etc.
	*/
	void config_sensor_to_sensor_communication();

	void config_sensor();
	/*
	 * set message according to the state of the system
	*/
	void set_message();

	/*
	 * This function is used for debugging purposes only. 
	 * Refair to the user manual for SYS_STATUS and SYS_MASK, and to APS022 for SYS_STATE
	 * print the state of the system (SYS_STATE,SYS_STATUS and SYS_MASK registers)
	*/
	static void system_state();

	void print_time_stamps();
	/*
	 * read the device id (register 0x00 of the decawave) to check if the SPI
	 * communication SEEMS to be working (although a correct DEV ID read is not a guarantee of correct working).
	 * A problem that can go by undetected is for example a problem involving interrupts happening during SPI
	 * communication.
	*/
	void get_device_ID();
	/*
	 * for now, hard code number of anchor like this
	*/
	void set_number(uint8_t number);
	/*
	 * Interrupt Service Routine (ISR). Must be static, or not an instance of the class
	 * It cannot have inputs nor outputs
	*/
	static void dwm_isr();
	static bool is_rx_done();
	static bool is_tx_done();
	/*
	 * clear the status register variable and the SYS_STATUS to 0xFF
	*/
	static void clear_stat_reg();
};
#endif