



#include <Arduino.h>
#include <SPI.h>

#include "esp_pins.h"	// for attaching interrupt
#include "constants.h"
#include "uC.h"

// static members definitions (declaration done in header file)
static int irq_counter = 0;	//for debug purposes
volatile bool uC::tx_done = false;	
volatile bool uC::rx_done = false;

uint16_t uC::isr_read_counter = 0;	//count how many times we read the status register (cf uC::good_isr_read())
uint8_t uC::isr_status_register[SYS_STATUS_LEN] = {0};
uint8_t uC::msg[128] = {0};
uint8_t uC::msg_length = MESSAGE_SIZE;	// all message are 1 byte (for now at least, probably different for the times)

uC::uC()
{
	Sensor* dwm_1000 = new Sensor();	// create object on the heap because we will use it for a long time so we don't waste the stack
	Receiver* receiver = new Receiver();
	Transmitter* transmitter = new Transmitter();
}

uC::uC(STATE initial_state)	// PROBABLY MAKE A CONSTRUCTOR with additionnal parameter being the first message (or we can reuse the message that is previously received. Je m'égare.)
{
	dwm_1000 = new Sensor();	// create object on the heap because we will use it for a long time so we don't waste the stack
	receiver = new Receiver();
	transmitter = new Transmitter();
	state = initial_state;
}

/*
 * #############################################################################
 * ##############################   RANGING    #################################
 * #############################################################################
*/
// should have a reception & emission function called everytime, simply that the state will be different between the different calls in the state machine
//--!! WHAT IS THE REFERENCE FOR THE TIME? A potential problem would be if the timer that the RX and TX stamping are based on overflows and therefore the 
// newer value is actually smaller than the previous value. Reinitialise the system counter I think? (SYS_TIME)
void uC::use_sensor()
{
	// problem: when removing the prints from the interrupt, the interrupt doesn't work anymore!' 
	// This clearly shows there must be a problem somewhere that is illustrated by this symptom

	// also... maybe use the value from the message that is sent (from TX_BUFFER) to check if indeed we're in the good state?
	switch(state)
	{
		// ---------------------------------------------------------------------
		// --------------------------------Tag cases----------------------------
		// ---------------------------------------------------------------------
		case TAG_SEND_INIT:	// Tag step 0: start the ranging by sending the initial message
			if(!tx_done)
			{
				break;
			}	
			Sensor::read_spi(TX_TIME,NO_SUB,tag_tx_init_time_stamp,TIME_STAMP_LEN);
			Serial.println("1->>>>TAG_WAIT_MESSAGE_1");
			// Sensor::print_register(tag_tx_init_time_stamp,TIME_STAMP_LEN);
			tx_done = false;
			state = TAG_WAIT_MESSAGE_1;	// next state
			start_reception();
			break;
		case TAG_WAIT_MESSAGE_1:
			if(!rx_done)
			{
				break;
			}
			Sensor::read_spi(RX_TIME,NO_SUB,tag_rx_message1_time_stamp,TIME_STAMP_LEN);
			Serial.println("2->>>>TAG_SEND_MESSAGE_2");
			// Sensor::print_register(tag_rx_message1_time_stamp,TIME_STAMP_LEN);
			rx_done = false;
			receiver -> rx_receive_data();
			state = TAG_SEND_MESSAGE_2;
			set_message();
			start_transmission();
			break;		
		case TAG_SEND_MESSAGE_2:
			if(!tx_done)
			{
				break;
			}
			Sensor::read_spi(TX_TIME,NO_SUB,tag_tx_message2_time_stamp,TIME_STAMP_LEN);
			Serial.println("3->>>>TAG_WAIT_TIMES done. Computing distances, state = TAG_SEND_MESSAGE_2");
			// Sensor::print_register(tag_tx_message2_time_stamp,TIME_STAMP_LEN);
			tx_done = false;
			state = TAG_WAIT_TIMES;
			start_reception();
			break;		
		case TAG_WAIT_TIMES:
			if(!rx_done)
			{
				break;
			}
			// VERSION < 5.3 : WAS READING TX BUFFER!
			// translated 1 byte to the left to take anchor ID and state machine state byte into account 
			Sensor::read_spi(RX_BUFFER,0x01,anchor_rx_init_time_stamp,TIME_STAMP_LEN);
			Sensor::read_spi(RX_BUFFER,0x06,anchor_tx_message1_time_stamp,TIME_STAMP_LEN);
			Sensor::read_spi(RX_BUFFER,0x0B,anchor_rx_message2_time_stamp,TIME_STAMP_LEN);
			// Sensor::read_spi(RX_TIME,NO_SUB,no this is a useless time,TIME_STAMP_LEN);
			Serial.println("computing state................");
			// Sensor::print_register(no this is a useless time,TIME_STAMP_LEN);
			rx_done = false;
			receiver -> rx_receive_data();
			state = COMPUTING_STATE;
			compute_distance();
			break;
		// ---------------------------------------------------------------------
		// ------------------------------Anchor cases---------------------------
		// ---------------------------------------------------------------------
		case ANCHOR_WAIT_INIT:	// Anchor step 1: wait to start the ranging
			if(!rx_done)
			{
				break;
			}
			rx_done = false;
			if (is_recipient())	// if the message received is for this anchor 
			{
				Sensor::read_spi(RX_TIME,NO_SUB,anchor_rx_init_time_stamp,TIME_STAMP_LEN);
				Serial.println("5->>>>ANCHOR_SEND_MESSAGE_1");
				// Sensor::print_register(anchor_rx_init_time_stamp,TIME_STAMP_LEN);
				receiver -> rx_receive_data();
				state = ANCHOR_SEND_MESSAGE_1;
				set_message();
				start_transmission();
			}
			else
			{
				start_reception();				
			}
			break;		
		case ANCHOR_SEND_MESSAGE_1:
			if(!tx_done)
			{
				break;
			}
			Sensor::read_spi(TX_TIME,NO_SUB,anchor_tx_message1_time_stamp,TIME_STAMP_LEN);
			Serial.println("6->>>>ANCHOR_WAIT_MESSAGE_2");
			// Sensor::print_register(anchor_tx_message1_time_stamp,TIME_STAMP_LEN);
			tx_done = false;
			state = ANCHOR_WAIT_MESSAGE_2;
			start_reception();
			break;
		case ANCHOR_WAIT_MESSAGE_2:
			if(!rx_done)
			{
				break;
			}
			rx_done = false;
			if(is_recipient())
			{
				Sensor::read_spi(RX_TIME,NO_SUB,anchor_rx_message2_time_stamp,TIME_STAMP_LEN);
				Serial.println("7->>>>ANCHOR_SEND_TIMES");
				Sensor::print_register(anchor_rx_message2_time_stamp,TIME_STAMP_LEN);
				receiver -> rx_receive_data();
				state = ANCHOR_SEND_TIMES;
				Serial.println("anchor times, anchor side:");
				Sensor::print_register(anchor_rx_init_time_stamp,TIME_STAMP_LEN);
				Sensor::print_register(anchor_tx_message1_time_stamp,TIME_STAMP_LEN);
				Sensor::print_register(anchor_rx_message2_time_stamp,TIME_STAMP_LEN);
				set_message();
				start_transmission();
			}
			else
			{
				start_reception();				
			}
			break;
		case ANCHOR_SEND_TIMES:
			if(!tx_done)
			{
				break;
			}
			// Sensor::read_spi(RX_TIME,NO_SUB,no this is useless time,TIME_STAMP_LEN);
			Serial.println("4->>>>ANCHOR_WAIT_INIT");
			// Sensor::print_register(no this is useless time,TIME_STAMP_LEN);
			tx_done = false;
			state = ANCHOR_WAIT_INIT;
			start_reception();
			break;
		// ----------------------- idle state while computing distance---------------------------
		case COMPUTING_STATE:
			Serial.println("computing state................");
			delay(500);
			current_ranging_anchor_number ++;	//ranging with next anchor
			if (current_ranging_anchor_number > NB_OF_ANCHORS)
			{
				current_ranging_anchor_number = 1;
			} 
			state = TAG_SEND_INIT;
			set_message();
			start_transmission();
			break;
		default:
			Serial.println("unknown state -- end program");
			while(1)
			{
				yield();
			}
	}
}

void uC::compute_distance()
{
	// format of a time stamp:
	// - 5 element array of uint8_t, to be converted in one uint64_t
	// - LSByte: LSB of the uint64_t => should NOT be shifted left
	// - MSByte: MSB of the uint64_t => should be shifted left 4*8 = 32 times
	
	// initilize the final times (uint64_t)
	t0 = 0;	
	t1 = 0;	
	t2 = 0;	
	t3 = 0;	
	t4 = 0;	
	t5 = 0;

	//initialize the temporary copies of the time array used for shifting the values
	uint64_t temp_t0;	
	uint64_t temp_t1;	
	uint64_t temp_t2;	
	uint64_t temp_t3;	
	uint64_t temp_t4;	
	uint64_t temp_t5;

	Sensor::print_register(tag_tx_init_time_stamp,TIME_STAMP_LEN);
	Sensor::print_register(tag_rx_message1_time_stamp,TIME_STAMP_LEN);
	Sensor::print_register(tag_tx_message2_time_stamp,TIME_STAMP_LEN);
	Sensor::print_register(anchor_rx_init_time_stamp,TIME_STAMP_LEN);
	Sensor::print_register(anchor_tx_message1_time_stamp,TIME_STAMP_LEN);
	Sensor::print_register(anchor_rx_message2_time_stamp,TIME_STAMP_LEN);

	for (int i = 0; i < TIME_STAMP_LEN; i++)//i = 0...4
	{

		temp_t0 = tag_tx_init_time_stamp[i];
		temp_t0 = temp_t0 << (8*i);
		temp_t3 = tag_rx_message1_time_stamp[i];
		temp_t3 = temp_t3 << (8*i);
		temp_t4 = tag_tx_message2_time_stamp[i];
		temp_t4 = temp_t4 << (8*i);

		temp_t1 = anchor_rx_init_time_stamp[i];
		temp_t1 = temp_t1 << (8*i);
		temp_t2 = anchor_tx_message1_time_stamp[i];
		temp_t2 = temp_t2 << (8*i);
		temp_t5 = anchor_rx_message2_time_stamp[i];
		temp_t5 = temp_t5 << (8*i);

		t0 += temp_t0;
		t1 += temp_t1;
		t2 += temp_t2;
		t3 += temp_t3;
		t4 += temp_t4;
		t5 += temp_t5;
	}
	// print_time_stamps();

	t_round_tag 	= t3-t0;
	t_reply_tag 	= t4-t3;
	t_round_anchor 	= t5-t2;
	t_reply_anchor 	= t2-t1;

	TOF_val = (t_round_tag - t_reply_anchor) + (t_round_anchor - t_reply_tag);
	uint32_t temp_tof_high = TOF_val >> 32;
	uint32_t temp_tof_low  = TOF_val;
	Serial.println("after computations:");
	Serial.print(temp_tof_high);
	Serial.println(temp_tof_low);

	TOF = TOF_val / 4.0;
	Serial.println("tof divided by 4: ");
	Serial.println(TOF,6);
	TOF = TOF*REGISTER_VALUE_TO_S;	//in seconds

	Serial.println("TOF value in seconds:: ");
	Serial.println(TOF,20);
	distance = TOF*SPEED_OF_LIGHT;
	Serial.print(" distance (m) : ");
	Serial.println(distance,20);
	Serial.println("======================end of compute distance=======================");
	delay(3000);
	// while(1)
	// {
	// 	yield();
	// }
}

bool uC::is_recipient()
{
	uint8_t recipient_anchor_number[1];	//	max number of anchors is 16, cf set_message, stored on nibble
	receiver -> rx_receive_data(recipient_anchor_number);	
	// Sensor::read_spi(TX_BUFFER,NO_SUB,recipient_anchor_number,1);
	Serial.print("recipient_anchor_number(recipient & message)::");
	Serial.println(recipient_anchor_number[0]);
	recipient_anchor_number[0] &= 0xF0;	// high nible of first byte
	recipient_anchor_number[0] = recipient_anchor_number[0] >> 4;
	Serial.print("recipient_anchor_number::");
	Serial.println(recipient_anchor_number[0]);
	Serial.print("this anchor number::");
	Serial.println(anchor_number);
	if (recipient_anchor_number[0] == anchor_number)
	{
		Serial.println("this anchor is the recipient_anchor_number!!");
		return true;
	}
	Serial.println("this anchor is not the recipient!");
	return false;
}

void uC::start_transmission()
{
	Serial.println("start_transmission");
	dwm_1000 -> idle();
	clear_tx_interrupts();
	Serial.print("message: ");
	Serial.print(msg[0]);
	transmitter -> tx_buffer_fill_data(msg,msg_length);
	Sensor::clear_register(SYS_CTRL,SYS_CTRL_LEN); //clear any RX or TX pinMode// check this. KILLS IDLE BIT
	transmitter -> tx_start();
	// system_state();
}

void uC::start_reception()
{
	Serial.println("start_reception");
	dwm_1000 -> idle();
	clear_rx_interrupts();
	// receiver -> auto_receive();
	Sensor::clear_register(SYS_CTRL,SYS_CTRL_LEN); //clear any RX or TX mode
	// receiver -> auto_receive();
	receiver -> rx_start();
}

void uC::clear_tx_interrupts()
{
	//set_bit not defined in this class. Be barbaric
	Sensor::set_bit(isr_status_register,SYS_MASK_LEN,7,1);	//TX OK
	// Sensor::set_bit(isr_status_register,SYS_MASK_LEN,6,1);
	// Sensor::set_bit(isr_status_register,SYS_MASK_LEN,5,1);
	// Sensor::set_bit(isr_status_register,SYS_MASK_LEN,4,1);
	// isr_status_register[0] |= 0xF0;
	Sensor::write_spi(SYS_STATUS, NO_SUB, isr_status_register, 1);
}

void uC::clear_rx_interrupts()
{
	Sensor::set_bit(isr_status_register,SYS_STATUS_LEN,13,1);// RX OK
	Sensor::set_bit(isr_status_register,SYS_STATUS_LEN,10,1);// RX OK
	Sensor::set_bit(isr_status_register,SYS_STATUS_LEN,18,1);// RX ERROR
	Sensor::set_bit(isr_status_register,SYS_STATUS_LEN,12,1);// RX ERROR
	Sensor::set_bit(isr_status_register,SYS_STATUS_LEN,15,1);// RX ERROR
	Sensor::set_bit(isr_status_register,SYS_STATUS_LEN,14,1);// RX ERROR
	Sensor::set_bit(isr_status_register,SYS_STATUS_LEN,16,1);// RX ERROR
	Sensor::write_spi(SYS_STATUS, NO_SUB, isr_status_register, SYS_STATUS_LEN);

}

void uC::set_message()
{
	switch(state)
	{
		case TAG_SEND_INIT:
			msg[0] = MESSAGE_INIT;
			msg_length = MESSAGE_SIZE;	// rename MESSAGE_SIZE
			break;	
		case ANCHOR_SEND_MESSAGE_1:
			msg[0] = MESSAGE_1;
			msg_length = MESSAGE_SIZE;
			break;
		case TAG_SEND_MESSAGE_2:
			msg[0] = MESSAGE_2;
			msg_length = MESSAGE_SIZE;
			break;
		case ANCHOR_SEND_TIMES:
			for (int i = 0; i < TIME_STAMP_LEN; i++)	
			{
				// The Times stamps start at index 1 since the first byte contains the ID of the anchor node (and the state of the state machine optionally)
				// Hence the +1
				msg[i+1] = anchor_rx_init_time_stamp[i];						
				msg[i+1+TIME_STAMP_LEN] = anchor_tx_message1_time_stamp[i];
				msg[i+1+2*TIME_STAMP_LEN] = anchor_rx_message2_time_stamp[i];
			}
			msg_length = TIME_MESSAGE_SIZE + MESSAGE_SIZE;
			break;
		default:
			Serial.println("This state doesn't have a message to set");
			while(1)
			{
				yield();
			}
	}
	msg[0] += (current_ranging_anchor_number << 4);
	Serial.println("message is SET-------------------------");
	Serial.println(msg[0]);
}
/*
 * #############################################################################
 * #############################   CONFIGURATIONS   ############################
 * #############################################################################
*/

void uC::config_pins() 
{
	pinMode(LED,OUTPUT);
	pinMode(IRQ,INPUT);

	SPI.begin();	// configures SS,SCK and MOSI as output and MISO as input
	// SPI.usingInterrupt((uint8_t)0);
	
	attachInterrupt(digitalPinToInterrupt(IRQ),uC::dwm_isr,RISING);

	delay(1);

	pinMode(SS,OUTPUT);
	digitalWrite(SS,HIGH);
}

void uC::config_uc_to_sensor_communication()
{
	// chek in spi communication bis. There, we do not need to use this sys_cfg, just
	// put speed at 20 MHz. Might it solve the problem of the unreadable status register??
	// Who knows
	// attach_interrupt();
	dwm_1000 -> dwm_init();	// this has to be the first function! (softreset)
	// dwm_1000 -> spi_init(); //correct spi config BEFORE attaching interrupt to avoid unwanted interrupt here
	dwm_1000 -> idle();
	// Sensor::read_spi(SYS_MASK,NO_SUB,p_sys_mask,SYS_MASK_LEN);
	// Sensor::clear_register(SYS_MASK,SYS_MASK_LEN);
	// dwm_1000 -> set_register(SYS_STATUS,SYS_STATUS_LEN);
	delay(1);
	dwm_1000 -> set_tx_interrupt_mask();
	dwm_1000 -> set_rx_interrupt_mask();
}

void uC::config_sensor_to_sensor_communication()
{
	//TO DO: rename transmitter to set parameters
	transmitter -> transmitter_init(); // Transmitter Configuration Procedure: page 202
	receiver -> receiver_init();
	dwm_1000 -> correct_default_configuration();
}

void uC::config_sensor()
{
	Serial.println("configuration...");
	config_uc_to_sensor_communication();
	config_sensor_to_sensor_communication();
	// if (state == ANCHOR_WAIT_INIT)	//during configuration, the state of the anchor is ANCHOR_WAIT_INIT 
	// {
	// 	dwm_1000 -> set_anchor_antenna_delay();
	// }
	Serial.println("done");
	if (state == TAG_SEND_INIT)
	{
		current_ranging_anchor_number = 1;	// first tag starts at 1...
		Serial.println("0->>>>TAG_SEND_INIT");
	}
	else if (state == ANCHOR_WAIT_INIT)
	{
		Serial.println("4->>>>ANCHOR_WAIT_INIT");
	}
}

/*
 * #############################################################################
 * ########################   HELPER FUNCTIONS   ###############################
 * #############################################################################
*/
void uC::system_state()
{
	Serial.println("system state function..........");
	uint8_t sys_state[SYS_STATE_LEN] = {0};
	uint8_t sys_status[SYS_STATUS_LEN] = {0};
	uint8_t sys_mask[SYS_MASK_LEN] = {0};
	// the dw clears the TXROFF as soon as it's read -> we can't read it!
	// uint8_t sys_ctrl[SYS_CTRL_LEN] = {0};

	Sensor::read_spi(SYS_STATUS,NO_SUB,sys_status,SYS_STATUS_LEN);
	Sensor::read_spi(SYS_MASK,NO_SUB,sys_mask,SYS_MASK_LEN);
	Sensor::read_spi(SYS_STATE,NO_SUB,sys_state,SYS_STATE_LEN);

	Serial.print("system state  : ");
	for (int i = 0; i < SYS_STATE_LEN; i++)
	{
		Serial.print(sys_state[SYS_STATE_LEN-1-i],HEX);
		Serial.print(" ");
	}
	Serial.println("");
	Serial.print("system status : ");	
	for (int i = 0; i < SYS_STATUS_LEN; i++)
	{
		Serial.print(sys_status[SYS_STATUS_LEN-1-i],HEX);
		Serial.print(" ");
	}
	Serial.println("");	
	Serial.print("system mask   : ");
	for (int i = 0; i < SYS_MASK_LEN; i++)
	{
		Serial.print(sys_mask[SYS_MASK_LEN-1-i],HEX);
		Serial.print(" ");
	}
	Serial.println("");	
}

void uC::print_time_stamps()
{
	// This is a little bit barbaric, please don't judge me..
	// Arduino cannot print uint64_t
	// Serial.print("t0  :: ");
	// Serial.print((uint32_t) (t0 >> 32),HEX);
	// Serial.println((uint32_t)t0,HEX);

	// Serial.println("times ::");	//print high and low halves, because can't print uint64_t!
	// Serial.print((uint32_t)( (t0 & 0x000000FF00000000) >> 32),HEX);
	// Serial.println((uint32_t)(t0 & 0x00000000FFFFFFFF),HEX);

	// Serial.print((uint32_t)( (t1 & 0x000000FF00000000) >> 32),HEX);
	// Serial.println((uint32_t)(t1 & 0x00000000FFFFFFFF),HEX);

	// Serial.print((uint32_t)( (t2 & 0x000000FF00000000) >> 32),HEX);
	// Serial.println((uint32_t)(t2 & 0x00000000FFFFFFFF),HEX);

	// Serial.print((uint32_t)( (t3 & 0x000000FF00000000) >> 32),HEX);
	// Serial.println((uint32_t)(t3 & 0x00000000FFFFFFFF),HEX);

	// Serial.print((uint32_t)( (t4 & 0x000000FF00000000) >> 32),HEX);
	// Serial.println((uint32_t)(t4 & 0x00000000FFFFFFFF),HEX);

	// Serial.print((uint32_t)( (t5 & 0x000000FF00000000) >> 32),HEX);
	// Serial.println((uint32_t)(t5 & 0x00000000FFFFFFFF),HEX);

}

void uC::get_device_ID()
{
	uint8_t spi_buffer[4] = {0};
	Sensor::read_spi(DEV_ID,NO_SUB,spi_buffer,DEV_ID_LEN);
	uint32_t dev_id =(spi_buffer[3] << 24) | (spi_buffer[2] << 16) | (spi_buffer[1] << 8) | spi_buffer[0];
	// dev_id ++; // testing with a wrong device ID
	Serial.print("uC :: ");
	if (dev_id != 0xDECA0130)
	{
		Serial.print(" BAD device ID: ");
		Serial.println(dev_id,HEX);
		while(1)
		{
			setup();	//start setup again since something went wrong
		}
	}
	Serial.print(" GOOD device ID: ");
	Serial.println(dev_id,HEX);
}

void uC::set_number(uint8_t number)
{
	anchor_number = number;
	Serial.print("anchor number set ::");
	Serial.println(anchor_number);
}

/*
 * #############################################################################
 * ################   INTERRUPT SERVICE ROUTINE & helpers   ####################
 * #############################################################################
*/

void uC::dwm_isr()	// static method, can only access static members
{
	Serial.println("##########IRQ##########");
	irq_counter++;
	Serial.print(irq_counter);
	Serial.println(" interrupt - dwm_isr");
	Sensor::read_spi(SYS_STATUS,NO_SUB,isr_status_register,SYS_STATUS_LEN);
	Sensor::print_register(isr_status_register,SYS_STATUS_LEN);
	
	if(is_tx_done())
	{
		tx_done = true;
		// Sensor::set_bit(isr_status_register,SYS_STATUS_LEN,7,1);
	}
	else if(is_rx_done())
	{
		rx_done = true;
	}
	else
	{
		Serial.println("INTERRUPT YIELDED NO RX OR TX FINISHED");
		system_state();	//to see what went wrong
	}
	clear_stat_reg();
}

bool uC::is_tx_done()
{
	//bitRead: arduino function(). Redefined in header :)
	uint8_t txfrs_register = isr_status_register[0];
	uint8_t txfrs = bitRead(txfrs_register,7);	//TXDONE is MSB of first byte. But also redefined in header
	Serial.print("...is tx done? ");
	Serial.println(txfrs);
	if (txfrs)
	{
		// Serial.println(" - yes!");
		return true;
	}
	else
	{
		return false;
	}
}

bool uC::is_rx_done()
{
	//bitRead: arduino function(). Redefined in header :)
	uint8_t rxfdr_register = isr_status_register[1];
	uint8_t rxfdr = bitRead(rxfdr_register,6);	//RX OK: bit 13 (starting from 0) = 8+6
	Serial.print("...is rx done? ");
	Serial.println(rxfdr);
	if (rxfdr)
	{
		// Serial.println(" - yes!");
		return true;
	}
	else
	{
		return false;
	}
}

void uC::clear_stat_reg()
{
	memset(isr_status_register,0xFF,SYS_STATUS_LEN);
	Sensor::write_spi(SYS_STATUS,NO_SUB,isr_status_register,SYS_STATUS_LEN);
}
