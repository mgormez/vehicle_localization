



#include <Arduino.h>

#include "receiver.h"
#include "constants.h"

Receiver::Receiver()
{

}

void Receiver::receiver_init()
{
	idle();
	// set default values
	channel_init();
	this -> frame_timeout_delay = 0;
	//write default values to sensor
	receiver_update_channel();
	receiver_update_bitrate();
	receiver_update_prf();
	receiver_update_preamble_code();
	receiver_update_preamble_size();
	receiver_update_frame_timeout_delay();

	// delay(5);
	// this -> bit_rate = _110KBPS;
	// this -> preamble_code = channel_valid_preamble_code(4U);
	// this -> preamble_size = channel_valid_preamble_size(_2048);
	// receiver_update_channel();
	// receiver_update_bitrate();
	// receiver_update_prf();
	// receiver_update_preamble_code();
	// receiver_update_preamble_size();
	// receiver_update_frame_timeout_delay();
	// // //auto receive (permanently in receive mode)
	// // //p72/244
	// uint8_t sys_cfg[SYS_CFG_LEN] = {0};
	// read_spi(SYS_CFG,NO_SUB,sys_cfg,SYS_CFG_LEN);//to not over write!
	// // set_bit(sys_cfg,SYS_CFG_LEN,29,1);//RXAUTR, receiver auto-re-enable
	// if (this -> bit_rate == _110KBPS)
	// {
	// 	set_bit(sys_cfg,4,22,1);//RXM110K
	// 	// write_spi(SYS_CFG, 0x03, sys_cfg, 1);
	// }	
	// write_spi(SYS_CFG,NO_SUB,sys_cfg,SYS_CFG_LEN);

	char msg[128];
	Serial.println("receiver state: ");
	sprintf(msg,"channel %u, prf %u, bit rate %u, preamble_code %u, preamble_size %u",channel_nb,prf,bit_rate,preamble_code,preamble_size);
	Serial.println(msg);
}

void Receiver::rx_start()
{
	//enable receiver p82/244
	uint8_t sys_ctrl[1] = {0};
	// set_bit(sys_ctrl, 4, 0, 0);//no frame suppression (of the 2 FCS frames)
	sys_ctrl[0] = 0x01;
	write_spi(SYS_CTRL, 0x01, sys_ctrl, 1);
	// Serial.println("rx start");
}

void Receiver::rx_reset()
{
	// receiver only reset p195/244
	Serial.println("I think -> unused function");
	uint8_t pmscCtrl0[] = {0xE0};	// clear bit 28
	write_spi(PMSC, 0x03, pmscCtrl0, 1);

	pmscCtrl0[0] = 0x10;	//set bit 28
	write_spi(PMSC, 0x03, pmscCtrl0, 1);
}

void Receiver::clear_rx_interrupts()
{
	uint8_t sys_status[SYS_STATUS_LEN] = {0};
	
	set_bit(sys_status,SYS_STATUS_LEN,13,1);// RX OK
	set_bit(sys_status,SYS_STATUS_LEN,10,1);// RX OK
	set_bit(sys_status,SYS_STATUS_LEN,18,1);// RX ERROR
	set_bit(sys_status,SYS_STATUS_LEN,12,1);// RX ERROR
	set_bit(sys_status,SYS_STATUS_LEN,15,1);// RX ERROR
	set_bit(sys_status,SYS_STATUS_LEN,14,1);// RX ERROR
	set_bit(sys_status,SYS_STATUS_LEN,16,1);// RX ERROR

	write_spi(SYS_STATUS, NO_SUB, sys_status, SYS_STATUS_LEN);
}

void Receiver::auto_receive()
{
	uint8_t sys_cfg[SYS_CFG_LEN] = {0};
	read_spi(SYS_CFG,NO_SUB,sys_cfg,SYS_CFG_LEN);
	set_bit(sys_cfg,SYS_CFG_LEN,29,1);//RXAUTR, receiver auto-re-enable
	write_spi(SYS_CFG,NO_SUB,sys_cfg,SYS_CFG_LEN);
}

void Receiver::rx_receive_data()
{
	// Get frame length
	uint8_t flen;	// can we have more than 256 bytes of data?
	read_spi(RX_FINFO, NO_SUB, &flen, 1);
	// flen = flen & 0x007F;	//frame length is 7 LSB
	// Serial.print("length of received frame (BEFORE FCS clean up): ");
	// Serial.println(flen[0]);
	flen -= 2; // remove the 2 Frame check bytes (FCS)
	// Serial.print("length of received frame (after FCS clean up): ");
	// Serial.println(flen[0]);
	uint8_t rx_data[2] = {0};

	read_spi(RX_BUFFER, NO_SUB, rx_data, flen);
	Serial.println("======= DATA READ::");
	Serial.println("read data from data buffer: ");	// hardcoded length. Should loop on the message length (that comes as an input)
	Serial.println(rx_data[0]);
}
void Receiver::rx_receive_data(uint8_t* rx_buffer)
{
	// Get frame length
	uint8_t flen;	// can we have more than 256 bytes of data?
	read_spi(RX_FINFO, NO_SUB, &flen, RX_FLEN_LEN);
	flen = flen & 0x7F;	//frame length is 7 LSB
	// Serial.print("length of received frame (BEFORE FCS clean up): ");
	// Serial.println(flen[0]);
	flen -= 2; // remove the 2 Frame check bytes (FCS)
	// Serial.print("length of received frame (after FCS clean up): ");
	// Serial.println(flen[0]);

	read_spi(RX_BUFFER, NO_SUB, rx_buffer, flen);
	Serial.println("======= DATA READ::");
	Serial.println("read data from data buffer: ");	// hardcoded length. Should loop on the message length (that comes as an input)
	Serial.println(rx_buffer[0]);
}

void Receiver::receiver_update_SFD_timeout() 
{
	uint16_t preambleSize = 4096;
	uint8_t timeout[2] = {0};
	switch(this -> preamble_size) 
	{
		case _64:
			preambleSize = 64;
			break;
		case _128:
			preambleSize = 128;
			break;
		case _256:
			preambleSize = 256;
			break;
		case _512:
			preambleSize = 512;
			break;
		case _1024:
			preambleSize = 1024;
			break;
		case _1536:
			preambleSize = 1536;
			break;
		case _2048:
			preambleSize = 2048;
			break;
		case _4096:
			preambleSize = 4096;
			break;
	}
	short2Bytes(preambleSize + this -> sfd + 1 - this -> pac, timeout);
    write_spi(DRX_CONF, 0x20, timeout, 2);
}

void Receiver::receiver_update_pac_size() 
{
	uint8_t drxTune2[4] = {0};
	switch (this -> prf)
	{
	case _16MHZ:
		switch (this -> preamble_size) 
		{
		case _64: case _128:
			int2Bytes(0x311A002DU, drxTune2);
			this -> pac = 8;
			break;
		case _256: case _512:
			int2Bytes(0x331A0052U, drxTune2);
			this -> pac = 16;
			break;
		case _1024:
			int2Bytes(0x351A009AU, drxTune2);
			this -> pac = 32;
			break;
		case _1536: case _2048: case _4096:
			int2Bytes(0x371A011DU, drxTune2);
			this -> pac = 64;
			break;
		}
		break;
	case _64MHZ:
		switch (this -> preamble_size) 
		{
		case _64: case _128:
			int2Bytes(0x313B006BU, drxTune2);
			this -> pac = 8;
			break;
		case _256: case _512:
			int2Bytes(0x333B00BEU, drxTune2);
			this -> pac = 16;
			break;
		case _1024:
			int2Bytes(0x353B015EU, drxTune2);
			this -> pac = 32;
			break;
		case _1536: case _2048: case _4096:
			int2Bytes(0x373B0296U, drxTune2);
			this -> pac = 64;
			break;
		}
		break;
	}
	write_spi(DRX_CONF, 0x08, drxTune2, 4);
}
void Receiver::receiver_update_channel()
{
	uint8_t rxCtrl[1] = {0};
	switch (this -> channel_nb) 
	{
	case _1: case _2: case _3: case _5:
		rxCtrl[0] = 0xD8;
		break;
	case _4: case _7:
		rxCtrl[0] = 0xBC;
		break;
	}
	write_spi(RF_CONF, 0x0B, rxCtrl, 1);
}

void Receiver::receiver_update_bitrate()
{
	uint8_t sys[1] = {0};
	uint8_t drxTune0b[2] = {0};
	switch (this ->bit_rate) 
	{
	case _110KBPS:
		short2Bytes(0x000AU, drxTune0b);
		sys[0] = 1 << 6;
		this -> sfd = 64;
		break;
	case _850KBPS: case _6800KBPS:
		short2Bytes(0x0001U, drxTune0b);
		this -> sfd = 8;
		break;
	}
	write_spi(SYS_CFG, 2, sys, 1);
	write_spi(DRX_CONF, 0x02, drxTune0b, 2);
	this -> receiver_update_SFD_timeout();
}

void Receiver::receiver_update_prf()
{
	uint8_t prf[1] = {0};
	read_spi(CHAN_CTRL, 0x02, prf, 1);
	prf[0] &= 0xF3;
	prf[0] |= this -> prf << 2;
	write_spi(CHAN_CTRL, 2, prf, 1);
	uint8_t drxTune1a[2] = {0};
	uint8_t agcTune1[2] = {0};
	uint8_t ldeCfg2[2] = {0};
	switch (this -> prf) 
	{
	case _16MHZ:
		short2Bytes(0x0087U, drxTune1a);
		short2Bytes(0x8870U, agcTune1);
		// short2Bytes(0x0003U, ldeCfg2); // for NLOS
		short2Bytes(0x1607U, ldeCfg2);
		break;
	case _64MHZ:
		short2Bytes(0x008DU, drxTune1a);
		short2Bytes(0x889BU, agcTune1);
		short2Bytes(0x0607U, ldeCfg2);
		break;
	}
	write_spi(DRX_CONF, 0x04, drxTune1a, 2);
	write_spi(AGC_CTRL, 0x04, agcTune1, 2);
	write_spi(LDE_CTRL, 0x1806, ldeCfg2, 2);
	this -> receiver_update_pac_size();
}


void Receiver::receiver_update_preamble_code()
{
	uint8_t chan[1] = {0};
	read_spi(CHAN_CTRL, 3, chan, 1);
	uint8_t preamble_code = this -> preamble_code;
	chan[0] &= 0x07;
	chan[0] |= preamble_code << 3;
	write_spi(CHAN_CTRL, 3, chan, 1);
}

void Receiver::receiver_update_preamble_size()
{
	uint8_t drxTune1b[2] = {0};
	switch (this -> preamble_size) 
	{
	case _64:
		short2Bytes(0x0064, drxTune1b);
		break;
	case _128: case _256: case _512: case _1024:
		short2Bytes(0x0020, drxTune1b);
		break;
	case _1536: case _2048: case _4096:
		short2Bytes(0x0010, drxTune1b);
		break;
	}
	write_spi(DRX_CONF, 0x06, drxTune1b, 2);
	this -> receiver_update_pac_size();
	this -> receiver_update_SFD_timeout();
}

void Receiver::receiver_update_frame_timeout_delay()
{
	uint8_t sysCfg[1] = {0};
	read_spi(SYS_CFG, 0x03, sysCfg, 1);
	sysCfg[0] |= (1 << 4);
	uint8_t rxfwto[2] = {0};
	short2Bytes(this -> frame_timeout_delay, rxfwto);
	if (this -> frame_timeout_delay == 0) {
		sysCfg[0] &= ~(1 << 4);
	}
	write_spi(RX_FWTO, NO_SUB, rxfwto, 2);
	write_spi(SYS_CFG, 0x03, sysCfg, 1);
}

void Receiver::show_yourself()
{
	Serial.println("receiver object here!!");	
}

void Receiver::polling_read_data()
{
	// Get frame length
	uint8_t flen;	// can we have more than 256 bytes of data?
	read_spi(RX_FINFO, NO_SUB, &flen, 1);
	flen = flen & 0x7F;	//frame length is 7 LSB
	Serial.print("length of received frame (BEFORE FCS clean up): ");
	Serial.println(flen);
	flen -= 2; // remove the 2 Frame check bytes (FCS)
	Serial.print("length of received frame (after FCS clean up): ");
	Serial.println(flen);
	
	uint8_t rx_data[2] = {0};
	read_spi(RX_BUFFER, NO_SUB, rx_data, 2);
	Serial.println("read data from buffer: ");
	Serial.println(rx_data[0]);
	Serial.println(rx_data[1]);
	Serial.println("-----------------------");
}