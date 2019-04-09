



#include <Arduino.h>
#include <SPI.h>	// arduino library


#include "constants.h"
#include "sensor.h"



Sensor::Sensor()
{

}

void Sensor::dwm_init()
{
	// Serial.println("dwm_init::");
	soft_reset();	//check for changing the clock speed during the softreset, cf dw API  XXXXXXXXXXXXXXXX
	delay(5);
	//change clock to XTI clock
	// uint8_t pmsc_ctrl0[2] = {0};// spi_buffer buffer to put in PMSC_CTRL0_SUB register
	// slow_read_spi(PMSC, PMSC_SYSCLKS_SUB, pmsc_ctrl0, 2);
	// pmsc_ctrl0[0] &= 0xFC;
	// pmsc_ctrl0[0] |= 0x01;
	// slow_write_spi(PMSC, PMSC_SYSCLKS_SUB, pmsc_ctrl0, 2);
	
	delay(10);
	load_LDE();
	delay(10);

	//change clock back to AUTO clock
	// slow_read_spi(PMSC, PMSC_SYSCLKS_SUB, pmsc_ctrl0, 2);
	// pmsc_ctrl0[0] = 0x00;
	// pmsc_ctrl0[1] &= 0xFE;
	// slow_write_spi(PMSC, PMSC_SYSCLKS_SUB, pmsc_ctrl0, 2);

}

void Sensor::channel_init()
{
	//set default values
	channel_nb = _5;
	prf = _16MHZ;
	bit_rate = _6800KBPS;
	preamble_code = channel_valid_preamble_code(4U);
	preamble_size = channel_valid_preamble_size(_128);
}

void Sensor::idle()
{
	//idle mode p81/244
	uint8_t sys_ctrl[SYS_CTRL_LEN];
	memset(sys_ctrl,0x00,SYS_CTRL_LEN);
	set_bit(sys_ctrl,SYS_CTRL_LEN,TRXOFF_BIT,1);
	// Serial.println("idle:::::::::::");
	write_spi(SYS_CTRL, NO_SUB, sys_ctrl, 4);
	// state = IDLE_STATE;	// to check! Not too sure that this is good as it might also make wrong behavior in the ranging!
}

/*
 * #############################################################################
 * ##########################   CONFIGURATION   ################################
 * #############################################################################
*/

void Sensor::soft_reset()
{

	uint8_t pmsc_ctrl0[2] = {0};// spi_buffer buffer to put in PMSC_CTRL0_SUB register
	// softreset - datasheet v2.15 7.2.50.1 page 195/244
	// 1) set SYSCLKS bits to 01
	pmsc_ctrl0[0] = 0x01;
	slow_write_spi(PMSC, PMSC_SYSCLKS_SUB, pmsc_ctrl0, 1);
	delay(1);

	// 2) clear SOFTRESET bits
	pmsc_ctrl0[0] = 0x00;// should read bit 0 of this byte first? because it's related to the mode of operation, and this overrides
	slow_write_spi(PMSC, PMSC_SOFTRESET_SUB, pmsc_ctrl0, 1),

	delay(1);
	// 3) set SOFTRESET bits
	pmsc_ctrl0[0] = 0xF0;
	slow_write_spi(PMSC, PMSC_SOFTRESET_SUB, pmsc_ctrl0, 1);

	delay(5);
}

void Sensor::load_LDE()
{
	//https://decaforum.decawave.com/t/help-with-basic-tx-rx-example/2355
	//spi spead has to be less than 3 MHz for this 
	// load the LDE microcode from ROM into RAM, table 4, 2.5.5.10 page23/244

	uint8_t pmsc_ctrl0[2] = {0};// spi_buffer buffer to put in PMSC_CTRL0_SUB register

	//load LDE
	pmsc_ctrl0[0] = 0x01;
	pmsc_ctrl0[1] = 0x03;
	slow_write_spi(PMSC, PMSC_SYSCLKS_SUB, pmsc_ctrl0, 2);
	delay(1);

	uint8_t opt_ctrl[2] = {0};
	opt_ctrl[0] = 0x00;
	opt_ctrl[1] = 0x80;

	slow_write_spi(OTP_IF, OTP_CTRL_SUB, opt_ctrl, 2);
	delay(5);	//wait 150 us, take more

	pmsc_ctrl0[0] = 0x00;//because we didn't clear pmsc_cltr0 variable! so put it back to 0x00
	pmsc_ctrl0[1] = 0x02;

	slow_write_spi(PMSC, PMSC_SYSCLKS_SUB, pmsc_ctrl0, 2);
	delay(1);
}

void Sensor::set_anchor_antenna_delay()
{
	uint8_t delayuint8[2];
    uint16_t antennaDelay = ANTENNA_DELAY >> 1;
    delayuint8[1] = (antennaDelay & 0xFF00) >>8;
    delayuint8[0] = (antennaDelay & 0xFF);
    write_spi(TX_ANTD, NO_SUB, delayuint8, 2);
    write_spi(LDE_CTRL, 0x1804, delayuint8, 2);
}

void Sensor::spi_init()
{
	Serial.println("spi init");
	uint8_t sys_cfg[4] = {0}; //page 70
	Sensor::slow_read_spi(SYS_CFG,NO_SUB,sys_cfg,4);
	// sys_cfg[0] |= 0x00;
	Sensor::print_register(sys_cfg,4);
	sys_cfg[1] |= 0x04;
	Sensor::print_register(sys_cfg,4);
	// sys_cfg[2] |= 0x00;
	// sys_cfg[3] |= 0x00;
	Sensor::slow_write_spi(SYS_CFG, NO_SUB,sys_cfg, 4);
}

void Sensor::correct_default_configuration()
{
	//user manual 2.5.5 p21/244

	//AGC_TUNE1 (depends if frequency is 16 MHz or 64 MHz)
	// cf Receiver::receiver_update_prf()

	//AGC_TUNE2, p123/244
	uint8_t agc_tune2[] = {0x07, 0xA9, 0x02, 0x25};
	slow_write_spi(AGC_CTRL, 0x0C, agc_tune2, 4);

	//DRX_TUNE2
	// cf Receiver::receiver_update_pac_size()

    // LDE_CFG1: NTM
    uint8_t lde_cfg1[] = {0x6d};	// LOS CONFIGURATION
	//uint8_t lde_cfg1[] = {0x07};	// NLOS CONFIGURATION
    slow_write_spi(LDE_CTRL, 0x0806, lde_cfg1, 1);

	//LDE_CFG2
    //cf Receiver::receiver_update_prf()

	//TX_POWER
    // cf Transmitter::transmitter_update_tx_power() 

	//RF_TXCTRL
    // cf Transmitter::transmitter_update_channel()

	//TC_PGDELAY
    // cf Transmitter::transmitter_update_channel()

	//FS_PLLTUNE
	//Sensor::dwm_update_channel()

	//LDE_LOAD
    //Sensor::load_LDE()

	//LDO_TUNE
    // DO LDO_TUNE ! (none of the guys did it though)
    Serial.println("To Do: LDO_TUNE");
    Serial.println("Also: maybe throw away the first frame as there is always a locking problem? Make a dummy frame with value 0xFF that tags know to discard");
    // Serial.println("To Do: also other config things :( ");
}

void Sensor::dwm_update_channel()
{
	uint8_t chan[] = {((this -> channel_nb << 4)|this -> channel_nb)};
	write_spi(CHAN_CTRL, NO_SUB, chan, 1);
	uint8_t pllTune[1] = {0};
	uint8_t pllCfg[4] = {0};
	switch (this -> channel_nb) {
		case _1:
			int2Bytes(0x09000407U, pllCfg);
			pllTune[0] = 0x1EU;
			break;
		case _2: case _4:
			int2Bytes(0x08400508U, pllCfg);
			pllTune[0] = 0x26U;
			break;
		case _3:
			int2Bytes(0x08401009U, pllCfg);
			pllTune[0] = 0x56U;
			break;
		case _5: case _7:
			int2Bytes(0x0800041DU, pllCfg);
			pllTune[0] = 0xBEU;
			break;
	}
	write_spi(FS_CTRL, 0x07, pllCfg, 4);
	write_spi(FS_CTRL, 0x0B, pllTune, 1);
}

uint8_t Sensor::channel_valid_preamble_code(uint8_t preamble_code) 
{

	switch (prf) {
	case _16MHZ:
		switch (this->channel_nb) {
		case _1:
			if (preamble_code != 1U && preamble_code != 2U) {
				preamble_code = 1U;
			}
			break;
		case _2: case _5:
			if (preamble_code != 3U && preamble_code != 4U) {
				preamble_code = 3U;
			}
			break;
		case _3:
			if (preamble_code != 5U && preamble_code != 6U) {
				preamble_code = 5U;
			}
			break;
		case _4: case _7:
			if (preamble_code != 7U && preamble_code != 8U) {
				preamble_code = 7U;
			}
			break;
		}
		break;
	case _64MHZ:
		switch (this->channel_nb) {
		case _1: case _2: case _3: case _5:
			if (preamble_code < 9U || preamble_code >  12U) {
				preamble_code = 9U;
			}
			break;
		case _4: case _7:
			if (preamble_code < 17U || preamble_code >  20U) {
				preamble_code = 17U;
			}
			break;
		}
		break;
	}
	return preamble_code;
}

PE Sensor::channel_valid_preamble_size(PE preamble_size) 
{
	switch (preamble_size) {
	case _1536: case _2048: case _4096:
		if (this->bit_rate != _110KBPS) {
			preamble_size = _1024;
		}
		break;
	case _128: case _256: case _512: case _1024:
		if (this->bit_rate == _110KBPS) {
			preamble_size = _1536;
		}
		break;
	case _64:
		switch (this->bit_rate) {
		case _850KBPS:
			preamble_size = _128;
			break;
		case _110KBPS:
			preamble_size = _1536;
			break;
		case _6800KBPS:
			break;
		}
		break;
	}
	return preamble_size;
}

/*
 * #############################################################################
 * ############################   SPI FUNCTIONS   ##############################
 * #############################################################################
*/
// no interrupt in the SPI function
// attach & detach interrupt before doing the SPI transaction
void Sensor::read_spi(uint8_t address, uint16_t offset, uint8_t* spi_buffer, uint16_t len)
{
	uint8_t header[3] = {0};
	uint8_t headerLen = 1;
	uint16_t i = 0;

	//SPI header formatting, depending on length of register accessed
	if (offset == NO_SUB) {
		header[0] = address | READ;
	} else {
		header[0] = address | READ_SUB;
		if(offset < 128) {
			header[1] = (uint8_t)offset;
			headerLen++;
		} else {
			header[1] = (uint8_t)offset | RW_SUB_EXT;
			header[2] = (uint8_t)(offset >> 7);
			headerLen += 2;
		}
	}
	//MSBFIRST and SPI_MODE0 are defined in the SPI arduino library 
	noInterrupts();
	SPI.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));

	digitalWrite(SS, LOW);
	for(i = 0; i < headerLen; i++) {
		SPI.transfer(header[i]); // send header
	}
	for(i = 0; i < len; i++) {
		spi_buffer[i] = SPI.transfer(JUNK); // read values
	}
	delayMicroseconds(5);	// leave time for last character to be read
	digitalWrite(SS, HIGH);

	SPI.endTransaction();
	interrupts();
}

void Sensor::write_spi(uint8_t address, uint16_t offset, uint8_t* spi_buffer, uint16_t len)
{
	uint8_t header[3] = {0};
	uint8_t headerLen = 1;
	uint16_t i = 0;
	//SPI header formatting, depending on length of register accessed
	if (offset == NO_SUB) {
		header[0] = address | WRITE;
	} else {
		header[0] = address | WRITE_SUB;
		if(offset < 128) {
			header[1] = (uint8_t)offset;
			headerLen++;
		} else {
			header[1] = (uint8_t)offset | RW_SUB_EXT;
			header[2] = (uint8_t)(offset >> 7);
			headerLen += 2;
		}
	}
	//MSBFIRST and SPI_MODE0 are defined in the SPI arduino library 
	noInterrupts();
	SPI.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));

	digitalWrite(SS,LOW); //slave select low to start the communication
	for(i = 0; i < headerLen; i++) {
		SPI.transfer(header[i]); // send header
	}

	for(i = 0; i < len; i++) {
		SPI.transfer(spi_buffer[i]); // write values
	}
	delayMicroseconds(5);	// leave time for last character to be sent
	digitalWrite(SS,HIGH);	// de-select slave

	SPI.endTransaction();
	interrupts();

}


void Sensor::slow_read_spi(uint8_t address, uint16_t offset, uint8_t* spi_buffer, uint16_t len)
{
	uint8_t header[3] = {0};
	uint8_t headerLen = 1;
	uint16_t i = 0;

	//SPI header formatting, depending on length of register accessed
	if (offset == NO_SUB) {
		header[0] = address | READ;
	} else {
		header[0] = address | READ_SUB;
		if(offset < 128) {
			header[1] = (uint8_t)offset;
			headerLen++;
		} else {
			header[1] = (uint8_t)offset | RW_SUB_EXT;
			header[2] = (uint8_t)(offset >> 7);
			headerLen += 2;
		}
	}
	noInterrupts();
	
	//MSBFIRST and SPI_MODE0 are defined in the SPI arduino library 
	SPI.beginTransaction(SPISettings(3000000L, MSBFIRST, SPI_MODE0));

	digitalWrite(SS, LOW);
	for(i = 0; i < headerLen; i++) {
		SPI.transfer(header[i]); // send header
	}
	for(i = 0; i < len; i++) {
		spi_buffer[i] = SPI.transfer(JUNK); // read values
	}
	delayMicroseconds(5);	// leave time for last character to be read
	digitalWrite(SS, HIGH);

	SPI.endTransaction();
	interrupts();

}

void Sensor::slow_write_spi(uint8_t address, uint16_t offset, uint8_t* spi_buffer, uint16_t len)
{
	uint8_t header[3] = {0};
	uint8_t headerLen = 1;
	uint16_t i = 0;
	//SPI header formatting, depending on length of register accessed
	if (offset == NO_SUB) {
		header[0] = address | WRITE;
	} else {
		header[0] = address | WRITE_SUB;
		if(offset < 128) {
			header[1] = (uint8_t)offset;
			headerLen++;
		} else {
			header[1] = (uint8_t)offset | RW_SUB_EXT;
			header[2] = (uint8_t)(offset >> 7);
			headerLen += 2;
		}
	}
	noInterrupts();

	//MSBFIRST and SPI_MODE0 are defined in the SPI arduino library 
	SPI.beginTransaction(SPISettings(3000000L, MSBFIRST, SPI_MODE0));

	digitalWrite(SS,LOW); //slave select low to start the communication
	for(i = 0; i < headerLen; i++) {
		SPI.transfer(header[i]); // send header
	}

	for(i = 0; i < len; i++) {
		SPI.transfer(spi_buffer[i]); // write values
	}
	delayMicroseconds(5);	// leave time for last character to be sent
	digitalWrite(SS,HIGH);	// de-select slave

	SPI.endTransaction();
	interrupts();
}
/*
 * #############################################################################
 * ########################   HELPER FUNCTIONS   ###############################
 * #############################################################################
*/
void Sensor::print_register(uint8_t* register_buffer, uint16_t register_len)
{
	//print from MSB to LSB
	for(uint16_t i = 0; i < register_len; i++)
	{
		Serial.print(register_buffer[register_len-1-i],HEX);
		Serial.print(" ");
	}
	Serial.println("");
}

//could do a register that also takes the value as input but I think this is fine :)
void Sensor::clear_register(uint8_t register_address, uint16_t register_len)
{
	uint8_t cleared_register[register_len];
	memset(cleared_register,0x00,register_len);
	write_spi(register_address, NO_SUB, cleared_register, register_len);
}

void Sensor::set_register(uint8_t register_address, uint16_t register_len)
{
	uint8_t set_reg[register_len];
	memset(set_reg,0xFF,register_len);
	write_spi(register_address, NO_SUB, set_reg, register_len);
}

void Sensor::set_tx_interrupt_mask()
{
	uint8_t tx_sys_mask[SYS_MASK_LEN];
	read_spi(SYS_MASK, NO_SUB, tx_sys_mask, SYS_MASK_LEN);
	set_bit(tx_sys_mask,SYS_MASK_LEN,7,1);// TX OK
	write_spi(SYS_MASK, NO_SUB, tx_sys_mask, SYS_MASK_LEN);
}

void Sensor::unset_tx_interrupt_mask()
{
	// uint8_t tx_sys_mask[SYS_MASK_LEN];
	// read_spi(SYS_MASK, NO_SUB, tx_sys_mask, SYS_MASK_LEN);
	// set_bit(tx_sys_mask,SYS_MASK_LEN,7,0);// TX OK
	// write_spi(SYS_MASK, NO_SUB, tx_sys_mask, SYS_MASK_LEN);
}

void Sensor::set_rx_interrupt_mask()
{
	//check page 33/244
	// rx active and no time out (to check )
	uint8_t rx_sys_mask[SYS_MASK_LEN];
	read_spi(SYS_MASK, NO_SUB, rx_sys_mask, SYS_MASK_LEN);
	// set_bit(rx_sys_mask,SYS_MASK_LEN,8,1);// RX Preamble Detected
	// set_bit(rx_sys_mask,SYS_MASK_LEN,9,1);// RX SFD detected 
	// set_bit(rx_sys_mask,SYS_MASK_LEN,10,1);// RX LDE Done
	// set_bit(rx_sys_mask,SYS_MASK_LEN,11,1);// RX PHY header detect
	set_bit(rx_sys_mask,SYS_MASK_LEN,13,1);// RX data frame ready
	// set_bit(rx_sys_mask,SYS_MASK_LEN,14,1);// RX FCS Good
	// set_bit(rx_sys_mask,SYS_MASK_LEN,12,1);// RX ERROR
	// set_bit(rx_sys_mask,SYS_MASK_LEN,15,1);// RX ERROR
	// set_bit(rx_sys_mask,SYS_MASK_LEN,16,1);// RX ERROR
	// set_bit(rx_sys_mask,SYS_MASK_LEN,17,1);// RX TIMEOUT
	// set_bit(rx_sys_mask,SYS_MASK_LEN,18,1);// RX ERROR
	// set_bit(rx_sys_mask,SYS_MASK_LEN,21,1);// RX ERROR
	// set_bit(rx_sys_mask,SYS_MASK_LEN,26,1);// RX ERROR
	write_spi(SYS_MASK, NO_SUB, rx_sys_mask, SYS_MASK_LEN);
}

void Sensor::unset_rx_interrupt_mask()
{
	uint8_t rx_sys_mask[SYS_MASK_LEN];
	read_spi(SYS_MASK, NO_SUB, rx_sys_mask, SYS_MASK_LEN);
	//set_bit(rx_sys_mask,SYS_MASK_LEN,10,0);// RX OK
	set_bit(rx_sys_mask,SYS_MASK_LEN,13,0);// RX OK
	// set_bit(rx_sys_mask,SYS_MASK_LEN,14,0);// RX OK
	// set_bit(rx_sys_mask,SYS_MASK_LEN,12,0);// RX ERROR
	// set_bit(rx_sys_mask,SYS_MASK_LEN,15,0);// RX ERROR
	// set_bit(rx_sys_mask,SYS_MASK_LEN,16,0);// RX ERROR
	// set_bit(rx_sys_mask,SYS_MASK_LEN,17,0);// RX TIMEOUT
	// set_bit(rx_sys_mask,SYS_MASK_LEN,18,0);// RX ERROR
	// set_bit(rx_sys_mask,SYS_MASK_LEN,21,0);// RX ERROR
	// set_bit(rx_sys_mask,SYS_MASK_LEN,26,0);// RX ERROR
	write_spi(SYS_MASK, NO_SUB, rx_sys_mask, SYS_MASK_LEN);
}

void Sensor::int2Bytes(const uint32_t data_int, uint8_t* data) 
{
	data[0] = data_int;
	data[1] = data_int >> 8;
	data[2] = data_int >> 16;
	data[3] = data_int >> 24;
}

void Sensor::short2Bytes(const uint16_t data_short, uint8_t* data) 
{
	data[0] = data_short;
	data[1] = data_short >> 8;
}

void Sensor::set_bit(uint8_t* data, uint16_t len, uint8_t bit, uint8_t val) 
{
	uint16_t element_number;		// in which byte (element of the uint8_t array, uint8_t = 8 bits data) will the shifted bit be
	uint8_t shift;				// how many times does it need to be shifted in that byte

	element_number = bit>>3;		// >>3 because 2^3 = 8 and there are 8 bits in a uint8_t element
	if (element_number >= len)	// the shift asked exceeds the amount of elements of the array
	{
		return;
	} 

	uint8_t* target_element = &data[element_number];
	shift = bit%8;				// in the selected element, shift the good number of times
	if (val) {
		Bitset(*target_element, shift);
	} else {
		Bitclear(*target_element, shift);
	}
}