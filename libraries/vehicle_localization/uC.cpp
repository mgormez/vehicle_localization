



#include <Arduino.h>
#include <SPI.h>
#include <math.h>
// #include <time.h>	// for random noise when testing trilat algorithm


#include "esp_pins.h"	// for attaching interrupt
#include "constants.h"
#include "uC.h"

// static members definitions (declaration done in header file)
volatile bool uC::tx_done = false;	
volatile bool uC::rx_done = false;

uint8_t uC::isr_status_register[SYS_STATUS_LEN] = {0};
uint8_t uC::msg[128] = {0};
uint8_t uC::msg_length = MESSAGE_SIZE;
const uint8_t uC::rx_error_status[SYS_STATUS_LEN] = {0x00,0x04,0x027,0x90,0};

uC::uC()
{
	Sensor* dwm_1000 = new Sensor();	// create object on heap
	Receiver* receiver = new Receiver();
	Transmitter* transmitter = new Transmitter();
}

uC::uC(STATE initial_state)	
{
	dwm_1000 = new Sensor();	
	receiver = new Receiver();
	transmitter = new Transmitter();
	state = initial_state;
	tag_tx_init_time_stamp[TIME_STAMP_LEN] = {0};
	tag_rx_message1_time_stamp[TIME_STAMP_LEN] = {0};
	tag_tx_message2_time_stamp[TIME_STAMP_LEN] = {0};

	anchor_rx_init_time_stamp[TIME_STAMP_LEN] = {0};
	anchor_tx_message1_time_stamp[TIME_STAMP_LEN] = {0};
	anchor_rx_message2_time_stamp[TIME_STAMP_LEN] = {0};

    anchor_positions[0][0] = ANCHOR_1_POS_X;	
    anchor_positions[0][1] = ANCHOR_1_POS_Y;	
    anchor_positions[1][0] = ANCHOR_2_POS_X;	
    anchor_positions[1][1] = ANCHOR_2_POS_Y;	
    anchor_positions[2][0] = ANCHOR_3_POS_X;	
    anchor_positions[2][1] = ANCHOR_3_POS_Y;	
}

/*
 * #############################################################################
 * ##############################   RANGING    #################################
 * #############################################################################
*/

void uC::ranging_loop()
{
	if (millis() - ranging_init_time < TIMEOUT)
	{
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
				// Serial.print("NEW RANGING: message sent to :");
				// Serial.println(current_ranging_anchor_number);
				init_system_time = millis();
				Sensor::read_spi(TX_TIME,NO_SUB,tag_tx_init_time_stamp,TIME_STAMP_LEN);
				tx_done = false;
				state = TAG_WAIT_MESSAGE_1;	// next state
				break;
			case TAG_WAIT_MESSAGE_1:
				if(!rx_done)
				{
					break;
				}
				rx_done = false;
				receiver -> rx_receive_data(received_message_buffer);
				if (good_response())
				{
					state = TAG_SEND_MESSAGE_2;
					Sensor::read_spi(RX_TIME,NO_SUB,tag_rx_message1_time_stamp,TIME_STAMP_LEN);
				}
				else
				{
					state = TAG_SEND_INIT;
				}
				memset(received_message_buffer,0x00,MESSAGE_SIZE+TIME_MESSAGE_SIZE);
				set_message();
				start_transmission();
				break;		
			case TAG_SEND_MESSAGE_2:
				if(!tx_done)
				{
					break;
				}
				Sensor::read_spi(TX_TIME,NO_SUB,tag_tx_message2_time_stamp,TIME_STAMP_LEN);
				tx_done = false;
				state = TAG_WAIT_TIMES;
				break;		
			case TAG_WAIT_TIMES:
				if(!rx_done)
				{
					break;
				}
				t_ranging_end = micros();
				// TODO: ifdef with the goal of the code (measuring computing
				// time, distance or localization)

				// Serial.print("times:(ranging,distance,position): ");
				// Serial.print((int)(t_ranging_end - t_ranging_start));
				// Serial.print(" ");
				// Serial.print((int)(t_distance_end - t_distance_start));
				// Serial.print(" ");
				// Serial.print((int)(t_position_end - t_position_start));
				// Serial.println(";");
				final_system_time = millis();	
				rx_done = false;
				receiver -> rx_receive_data(received_message_buffer);
				if(good_response())
				{
					//TO DO: RX_BUFFER was already read in good_response(), 
					// reuse the data instead of reading again
					Sensor::read_spi(RX_BUFFER,0x01,anchor_rx_init_time_stamp,TIME_STAMP_LEN);
					Sensor::read_spi(RX_BUFFER,0x06,anchor_tx_message1_time_stamp,TIME_STAMP_LEN);
					Sensor::read_spi(RX_BUFFER,0x0B,anchor_rx_message2_time_stamp,TIME_STAMP_LEN);

					if (current_ranging_anchor_number >= NB_OF_ANCHORS)
					{
						compute_distance();
						compute_position();
						state = COMPUTING_STATE;
						current_ranging_anchor_number = 1;
					}
					else
					{
						compute_distance();
						state = COMPUTING_STATE;
						current_ranging_anchor_number ++;	//ranging with next anchor
					}
				}
				else
				{
					state = TAG_SEND_INIT;
					set_message();
					start_transmission();
				}
				memset(received_message_buffer,0x00,MESSAGE_SIZE+TIME_MESSAGE_SIZE);
				break;
			case COMPUTING_STATE:
				//Serial.println("computing state................");
				delay(80);	// necessary delay between 2 rangings. Still don't know why
				state = TAG_SEND_INIT;
				set_message();
				ranging_init_time = millis();
				t_ranging_start = micros();
				start_transmission();
				break;
			// ---------------------------------------------------------------------
			// ------------------------------Anchor cases---------------------------
			// ---------------------------------------------------------------------
			case ANCHOR_WAIT_INIT:	// Anchor step 1: wait to start the ranging
				if(!rx_done)
				{
					break;
				}
				// get_impulse_response();
				rx_done = false;
				t_ranging_start = micros();
				receiver -> rx_receive_data(received_message_buffer);
				if (is_recipient())	// if the message received is for this anchor 
				{
					Sensor::read_spi(RX_TIME,NO_SUB,anchor_rx_init_time_stamp,TIME_STAMP_LEN);
					receiver -> rx_receive_data();
					state = ANCHOR_SEND_MESSAGE_1;
					set_message();
					start_transmission();
				}
				else
				{
					// manually restart reception
					start_reception();				
				}
				break;		
			case ANCHOR_SEND_MESSAGE_1:
				if(!tx_done)
				{
					break;
				}
				Sensor::read_spi(TX_TIME,NO_SUB,anchor_tx_message1_time_stamp,TIME_STAMP_LEN);
				tx_done = false;
				state = ANCHOR_WAIT_MESSAGE_2;
				// start_reception();
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
					// Serial.println("step: 7->>>>ANCHOR_SEND_TIMES");
					// Sensor::print_register(anchor_rx_message2_time_stamp,TIME_STAMP_LEN);
					state = ANCHOR_SEND_TIMES;
					//Serial.println("anchor times, anchor side:");
					// Sensor::print_register(anchor_rx_init_time_stamp,TIME_STAMP_LEN);
					// Sensor::print_register(anchor_tx_message1_time_stamp,TIME_STAMP_LEN);
					// Sensor::print_register(anchor_rx_message2_time_stamp,TIME_STAMP_LEN);
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
				t_ranging_end = micros();

				// Serial.print("anchor ranging time: ");
				// Serial.println((int)(t_ranging_end - t_ranging_start));
				Serial.println("Anchor side -- done");
				tx_done = false;
				ranging_init_time = millis();	// restart ranging duration timeout
				state = ANCHOR_WAIT_INIT;
				// start_reception();
				break;
			default:
				Serial.println("unknown state -- end program");
				while(1)
				{
					yield();
				}
		}
	}
	else if(state <= 4)	//tag timeout
	{
		Serial.println("TAG TIME OUT!");
		system_state();
		config_sensor();
		delay(100);	// necessary
		state = TAG_SEND_INIT;
		tx_done = false;
		rx_done = false;
		current_ranging_anchor_number = 1;	// restart the whole ranging
		ranging_init_time = millis();	//restart timeout counter
		set_message();
		start_transmission();
		// set_message();
		// start_transmission();
	}
	else if(state >= 5)//anchor timeout
	{
		Serial.println("ANCHOR TIME OUT!");
		system_state();
		ranging_init_time = millis();
		tx_done = false;
		rx_done = false;
		state = ANCHOR_WAIT_INIT;
		start_reception();
	}
}

void uC::compute_distance()
{
	// format of a time stamp:
	// - 5 element array of uint8_t, to be converted in one uint64_t
	// - LSByte: LSB of the uint64_t => should NOT be shifted left
	// - MSByte: MSB of the uint64_t => should be shifted left 4*8 = 32 times

	// system_duration = final_system_time - init_system_time;
	t_distance_start = micros();
	// Serial.print("DURATION OF A RANGING:: ");
	// Serial.print((uint32_t)system_duration);
	// Serial.println(" ms");
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

	// Sensor::print_register(tag_tx_init_time_stamp,TIME_STAMP_LEN);
	// Sensor::print_register(tag_rx_message1_time_stamp,TIME_STAMP_LEN);
	// Sensor::print_register(tag_tx_message2_time_stamp,TIME_STAMP_LEN);
	// Sensor::print_register(anchor_rx_init_time_stamp,TIME_STAMP_LEN);
	// Sensor::print_register(anchor_tx_message1_time_stamp,TIME_STAMP_LEN);
	// Sensor::print_register(anchor_rx_message2_time_stamp,TIME_STAMP_LEN);

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
	// Serial.println("Time stamps from array: ");
	// Serial.println((uint32_t)(t0>>32));
	// Serial.println((uint32_t)t0);
	// Serial.println((uint32_t)(t1>>32));
	// Serial.println((uint32_t)t1);
	// Serial.println((uint32_t)(t2>>32));
	// Serial.println((uint32_t)t2);
	// Serial.println((uint32_t)(t3>>32));
	// Serial.println((uint32_t)t3);
	// Serial.println((uint32_t)(t4>>32));
	// Serial.println((uint32_t)t4);
	// Serial.println((uint32_t)(t5>>32));
	// Serial.println((uint32_t)t5);

	t_round_tag 	= t3-t0;
	t_reply_tag 	= t4-t3;
	t_round_anchor 	= t5-t2;
	t_reply_anchor 	= t2-t1;

	// Serial.println("Times for computing tof:");
	// Serial.println((uint32_t)(t_round_tag >> 32));
	// Serial.println((uint32_t)t_round_tag);
	// Serial.println((uint32_t)(t_reply_anchor >> 32));
	// Serial.println((uint32_t)t_reply_anchor);
	// Serial.println((uint32_t)(t_round_anchor >> 32));
	// Serial.println((uint32_t)t_round_anchor);
	// Serial.println((uint32_t)(t_reply_tag >> 32));
	// Serial.println((uint32_t)t_reply_tag);

	// Serial.println((uint32_t)t_round_tag);
	// Serial.println((uint32_t)t_reply_anchor);
	// Serial.println((uint32_t)t_round_anchor);
	// Serial.println((uint32_t)t_reply_tag);
	// Serial.println("Times of each message exchange:");
	// Serial.println((uint32_t)(t_round_tag - t_reply_anchor));
	// Serial.println((uint32_t)(t_round_anchor - t_reply_tag));

	// Serial.println((double)t_round_tag);
	//method 1
	TOF_val = (t_round_tag - t_reply_anchor) + (t_round_anchor - t_reply_tag);
	TOF = ((double)(TOF_val))/4;

	// // method 2 (asymetric)
	// TOF = (t_round_tag*t_round_anchor - t_reply_tag*t_reply_anchor)/((double)t_round_tag + t_round_anchor + t_reply_tag + t_reply_anchor);

	TOF = TOF*REGISTER_VALUE_TO_S;	//in seconds

	// Serial.println("TOF value in seconds:: ");
	// Serial.println(TOF,20);
	distances[current_ranging_anchor_number] = TOF*SPEED_OF_LIGHT;
	// Serial.print("distance (m) : ");
	// Serial.println(distances[current_ranging_anchor_number],5);
	// Serial.println("======================end of compute distance=======================");

	memset(tag_tx_init_time_stamp,0x00,TIME_STAMP_LEN);
	memset(tag_rx_message1_time_stamp,0x00,TIME_STAMP_LEN);
	memset(tag_tx_message2_time_stamp,0x00,TIME_STAMP_LEN);
	memset(anchor_rx_init_time_stamp,0x00,TIME_STAMP_LEN);
	memset(anchor_tx_message1_time_stamp,0x00,TIME_STAMP_LEN);
	memset(anchor_rx_message2_time_stamp,0x00,TIME_STAMP_LEN);

	t_distance_end = micros();
	// Serial.print("duration of distance comp:");
	// Serial.println((int)(t_distance_end-t_distance_start));
}


void uC::compute_position()
{
	// Serial.println("computing position.......................................");
	// Based on An Efficient Least-Squares Trilateration Algorithm for Mobile Robot Localization
	// Yu Zhou, Member, IEEE

	// for testing algorithm, add random noise on artificial measures
    // srand(time(NULL));
    // double random_n;
    // distances[0] = 0;
    // distances[1] = 2.06+((double)(rand()%100))/300;	// random value between 0 and 1/3
    // distances[2] = 2.06+((double)(rand()%100))/300;
    // distances[3] = 1.50+((double)(rand()%100))/300;

	t_position_start = micros();
    // computing a
    double a[2] = {0,0};
    double ppTp[2] = {0,0};
    double rp[2] = {0,0};

    // for (int i = 0; i < 3; i ++)
    // {
    // 	for (int j = 0; j < 2; j ++)
    // 	{
    // 		Serial.println(anchor_positions[i][j]);
    // 	}
    // }
    // Serial.print("distances recorded:");
    // for (int i = 1; i <= 3; i ++)
    // {
    // 	// distances[0] is not used! (since the anchors are 1 2 3 and not 0 1 2)
    // 	Serial.print(distances[i],5);
    // 	Serial.print(" ");
    // }

    for (int i = 0; i < 3; i++)
    {
    	//ppTp = (ppTp[0] ppTp[1]) = (xi^3+xi*yi^2 xi^2*yi+yi^3)
        ppTp[0] += pow(anchor_positions[i][0],3) + anchor_positions[i][0] * pow(anchor_positions[i][1],2);
        ppTp[1] += pow(anchor_positions[i][1],3) + anchor_positions[i][1] * pow(anchor_positions[i][0],2);
        rp[0] += anchor_positions[i][0] * pow(distances[i+1],2);
        rp[1] += anchor_positions[i][1] * pow(distances[i+1],2);
    }

    // Serial.println("a,ppTp, rp");
    // for (int i = 0; i < 2; i ++)
    // {
    // 	Serial.println(a[i]);
    // 	Serial.println(ppTp[i]);
    // 	Serial.println(rp[i]);
    // }
    
    a[0] = (ppTp[0]-rp[0])/NB_OF_ANCHORS;
    a[1] = (ppTp[1]-rp[1])/NB_OF_ANCHORS;

    // computing B 2x2 matrix
    double B[2][2] = {{0,0},{0,0}};
    double ppT[2][2] = {{0,0},{0,0}};
    double pTpI[2][2] = {{0,0},{0,0}};
    double rI[2][2] = {{0,0},{0,0}};

    for (int i = 0; i < NB_OF_ANCHORS; i++)	// browse anchors 
    {
        for(int j = 0; j < 2; j++)	// browse positions of B
        {
            for(int k = 0; k < 2; k++)
            {
            	// {xi^2, xi*yi},{yi*xi, yi*yi}
                ppT[j][k] += anchor_positions[i][j] * anchor_positions[i][k];
            }
            //{xi^2+yi^2,0},{0,xi^2+yi^2}
            pTpI[j][j] += pow(anchor_positions[i][0],2) + pow(anchor_positions[i][1],2);
            //{ri^2,0},{0,ri^2}
            rI[j][j] += pow(distances[i+1],2);
        }
    }

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            B[i][j] = (-2.0*ppT[i][j] - pTpI[i][j] + rI[i][j])/NB_OF_ANCHORS;
        }
    }

    // computing c
    double c[2] = {0,0};
    for (int i = 0; i < NB_OF_ANCHORS; i++)
    {
        for(int j = 0; j < 2; j++)
        {
            c[j] += anchor_positions[i][j];
        }
    }
    for (int i = 0; i < 2; i++)
    {
        c[i] = c[i]/NB_OF_ANCHORS;
    }

    // computing f
    double f[2] = {0,0};
    double Bc[2] = {0,0};
    double ccTc[2] = {0,0};

    Bc[0] = B[0][0]*c[0] + B[0][1]*c[1];
    Bc[1] = B[1][0]*c[0] + B[1][1]*c[1];

    ccTc[0] = pow(c[0],3) + c[0] * pow(c[1],2);
    ccTc[1] = pow(c[1],3) + c[1] * pow(c[0],2);

    for (int i = 0; i < 2; i++)
    {
        f[i] = a[i] + Bc[i] + 2*ccTc[i];
    }

    // H
    double H[2][2] = {{0,0},{0,0}};
    double ccT[2][2] = {{0,0},{0,0}};

    for(int i = 0; i < 2; i++)
    {
        for(int j = 0; j < 2; j++)
        {
            ccT[i][j] = c[i]*c[j];
            H[i][j] = -2.0/NB_OF_ANCHORS * ppT[i][j] + 2*ccT[i][j];
        }
    }
	// q = -(H^-1)*f . use cofactor method to inverse H
	// det(H) 
	double detH = H[0][0]*H[1][1] - H[0][1]*H[1][0];
	double invH[2][2] = {{H[1][1]/detH, -H[0][1]/detH},{-H[1][0]/detH,H[0][0]/detH}};
	
	//q = -(H^-1)*f
	double q[2] = {-invH[0][0]*f[0] - invH[0][1]*f[1],-invH[1][0]*f[0] - invH[1][1]*f[1]};
	
	// final tag position
	tag_position[0] = q[0] + c[0];
	tag_position[1] = q[1] + c[1];
	// Serial.println("DONE COMPUTING POSITION");
	// Serial.print("Absolute position of tag:");
	// Serial.print(tag_position[0]);
	// Serial.print(" ");
	// Serial.print(tag_position[1]);
	// Serial.println(";");
	// while(1)
	// {
	// 	yield();
	// }

	t_position_end = micros();
	// Serial.print("duration of position computation:");
	// Serial.println((int)(t_position_end - t_position_start));
}

bool uC::is_recipient()
{
	uint8_t destination_anchor_number = (received_message_buffer[0] & 0xF0) >>4;
	if (destination_anchor_number == anchor_number)
	{
		// Serial.println("this anchor is the recipient_anchor_number!!");
		return true;
	}
	// Serial.println("this anchor is not the recipient_anchor_number!");
	return false;
}

bool uC::good_response()
{
	uint8_t message = received_message_buffer[0] & 0x0F;
	uint8_t anchor  = (received_message_buffer[0] & 0xF0) >> 4;
	// Serial.println("in good response, elements are:");
	// Serial.println(message);
	// Serial.println(state);
	// Serial.println(anchor);
	// Serial.println(current_ranging_anchor_number);
	// When receiving a message i, the anchor is in the state i+1 which is 
	// waiting for message i to arrive
	if (message + 1 == state && anchor == current_ranging_anchor_number)
	{
		// Serial.println("good response");
		return true;
	}
	// Serial.println("bad response");
	return false;
}

void uC::start_transmission()
{
	// Serial.println("start_transmission");
	// dwm_1000 -> idle();
	// clear_tx_interrupts();
	// Serial.print("message: ");
	// Serial.println(msg[0]);
	transmitter -> tx_buffer_fill_data(msg,msg_length);
	Sensor::clear_register(SYS_CTRL,SYS_CTRL_LEN); //clear any RX or TX pinMode// check this. KILLS IDLE BIT
	transmitter -> tx_start();
	// system_state();
}

void uC::start_reception()
{
	// Serial.println("start_reception");
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
	// MESSAGE_SIZE: length of the message needed to be sent (state of the 
	// ranging and anchor number)
	switch(state)
	{
		case TAG_SEND_INIT:
			msg[0] = MESSAGE_INIT;
			msg[0] += (current_ranging_anchor_number << 4);
			msg_length = MESSAGE_SIZE;
			break;	
		case TAG_SEND_MESSAGE_2:
			msg[0] = MESSAGE_2;
			msg[0] += (current_ranging_anchor_number << 4);
			msg_length = MESSAGE_SIZE;
			break;
		case ANCHOR_SEND_MESSAGE_1:
			msg[0] = MESSAGE_1;
			msg[0] += (anchor_number << 4);
			msg_length = MESSAGE_SIZE;
			break;
		case ANCHOR_SEND_TIMES:
			msg[0] = TIME_MESSAGE;
			msg[0] += (anchor_number << 4);
			for (int i = 0; i < TIME_STAMP_LEN; i++)	
			{
				// The times stamps start at index 1 since the first byte 
				// contains the ID of the anchor node 
				// (and the state of the state machine optionally), hence the +1
				msg[i+1] = anchor_rx_init_time_stamp[i];						
				msg[i+1+TIME_STAMP_LEN] = anchor_tx_message1_time_stamp[i];
				msg[i+1+2*TIME_STAMP_LEN] = anchor_rx_message2_time_stamp[i];
			}
			msg_length = TIME_MESSAGE_SIZE + MESSAGE_SIZE;
			break;
		default:
			Serial.println(state);
			Serial.println("This state doesn't have a message to set");
			while(1)
			{
				yield();
			}
	}
	// Serial.println("message is SET-------------------------");
	// Serial.println(msg[0]);
}

void uC::get_impulse_response()
{
	// reseting the watchdog timer is only necessary if we want to print the
	// whole register, but since it's only zeros, not usefull
	wdt_disable();			// disable watchdog timer
	Serial.println("--------------IMPULSE RESPONSE---------------------");
	uint8_t impulse_response[4064];
	uint8_t current_elem = 0;
	uint16_t current_index = 0;
	Sensor::read_spi(ACC_MEM,NO_SUB,impulse_response,4064);
	Serial.println("-----------------GOOD SPI READ---------------------");
	for (uint8_t i = 0; i < 16; i++)
	{
		for (uint16_t j = 0; j < 256; j++)
		{
			current_index = i*256+j;
			current_elem = impulse_response[current_index];
			// if (current_elem != 0)
			// {
				Serial.println(impulse_response[i*256+j]);
			// }
		}
		// ESP.wdtFeed();		// prevent Watchdog timer from overflowing
	}
	wdt_enable(2000);
	// ESP.wdtDisable();		// reset Watchdog timer
	// ESP.wdtEnable(0);
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
	dwm_1000 -> dwm_init();	// this has to be the first function! (softreset)
	// legacy: spi_init(). Was used before fixing the read and write SPI
	// functions (protect from interrupts).
	// dwm_1000 -> spi_init();
	// dwm_1000 -> idle();
	// Sensor::read_spi(SYS_MASK,NO_SUB,p_sys_mask,SYS_MASK_LEN);
	// Sensor::clear_register(SYS_MASK,SYS_MASK_LEN);
	// dwm_1000 -> set_register(SYS_STATUS,SYS_STATUS_LEN);
	delay(1);
	dwm_1000 -> set_tx_interrupt_mask();
	dwm_1000 -> set_rx_interrupt_mask();
}

void uC::config_sensor_to_sensor_communication()
{
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
	// dwm_1000 -> set_anchor_antenna_delay();
	// }
	Serial.println("done");
	if (state == TAG_SEND_INIT)
	{
		current_ranging_anchor_number = 1;	// first anchor starts at 1...
		Serial.println("0->>>>TAG_SEND_INIT");
		ranging_init_time = millis();
	}
	else if (state == ANCHOR_WAIT_INIT)
	{
		Serial.println("4->>>>ANCHOR_WAIT_INIT");
	}
}

void uC::correct_anchor_delay()
{
	uint8_t delayuint8[2];
    uint16_t antennaDelay = ANTENNA_DELAY >> 1;
    delayuint8[1] = (antennaDelay & 0xFF00) >>8;
    delayuint8[0] = (antennaDelay & 0xFF);
    Sensor::write_spi(TX_ANTD, NO_SUB, delayuint8, 2);
    Sensor::write_spi(LDE_CTRL, 0x1804, delayuint8, 2);
}

/*
 * #############################################################################
 * ########################   HELPER FUNCTIONS   ###############################
 * #############################################################################
*/
void uC::rx_reset()
{
	// receiver only reset p195/244
	uint8_t pmscCtrl0[] = {0xE0};	// clear bit 28
	Sensor::write_spi(PMSC, 0x03, pmscCtrl0, 1);

	pmscCtrl0[0] = 0x10;	//set bit 28
	Sensor::write_spi(PMSC, 0x03, pmscCtrl0, 1);
}

void uC::system_state()
{
	// for debugging purposes, cf aps022
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

uint8_t uC::get_anchor_number()
{
	return anchor_number;
}

/*
 * #############################################################################
 * ################   INTERRUPT SERVICE ROUTINE & helpers   ####################
 * #############################################################################
*/

void uC::dwm_isr()	// static method, can only access static members
{
	Serial.println("##########IRQ##########");
	Sensor::read_spi(SYS_STATUS,NO_SUB,isr_status_register,SYS_STATUS_LEN);
	// Sensor::print_register(isr_status_register,SYS_STATUS_LEN);
	if(is_tx_done())
	{
		tx_done = true;
		// Sensor::set_bit(isr_status_register,SYS_STATUS_LEN,7,1);
	}
	else if(is_rx_done())
	{
		rx_done = true;
	}
	else if(rx_error())
	{
		Serial.println("RX ERROR!");
		system_state();
		// rx_reset();	//can't call it here (or make it static). RX error
						// almost never occurs
	}
	else
	{
		Serial.println("Other irq error, see system state");
		system_state();	//to see what went wrong
	}
	clear_stat_reg();	//clear all interrupts
}

bool uC::is_tx_done()
{
	//bitRead: arduino function(). Redefined in header :)
	uint8_t txfrs_register = isr_status_register[0];
	uint8_t txfrs = bitRead(txfrs_register,7);	//TXDONE is MSB of first byte. But also redefined in header
	// Serial.print("...is tx done? ");
	// Serial.println(txfrs);
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
	// Serial.print("...is rx done? ");
	// Serial.println(rxfdr);
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

bool uC::rx_error()
{
	for (int i = 0; i < SYS_STATUS_LEN; i++)
	{
		//if there is (at least) one of the errors, there is an error
		if(isr_status_register[i] & rx_error_status[i])
		{
			return true;
		}	
	}
	return false;		// no RX error
}

void uC::clear_stat_reg()
{
	uint8_t temp_status[SYS_STATUS_LEN];
	memset(temp_status,0xFF,SYS_STATUS_LEN);
	// memset(isr_status_register,0xFF,SYS_STATUS_LEN);
	Sensor::write_spi(SYS_STATUS,NO_SUB,temp_status,SYS_STATUS_LEN);
	// Sensor::write_spi(SYS_STATUS,NO_SUB,isr_status_register,SYS_STATUS_LEN);
}
