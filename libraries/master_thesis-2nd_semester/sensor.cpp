




#include "sensor.h"


SENSOR::SENSOR()
{

}

void SENSOR::dwm_reset()
{

	delay(5);
	uint8_t pmsc_ctrl0[2];// spi_buffer buffer to put in PMSC_CTRL0_SUB register

	// softreset - datasheet v2.15 7.2.50.1 page 195/244
	// 1) set SYSCLKS bits to 01
	pmsc_ctrl0[0] = 0x01;
	write_spi(PMSC, PMSC_SYSCLKS_SUB, pmsc_ctrl0, 1);
	delay(1);

	// 2) clear SOFTRESET bits
	pmsc_ctrl0[0] = 0x00;// should read bit 0 of this byte first? because it's related to the mode of operation, and this overrides
	write_spi(PMSC, PMSC_SOFTRESET_SUB, pmsc_ctrl0, 1),
	//delay(1);

	delay(10);
	// 3) set SOFTRESET bits
	pmsc_ctrl0[0] = 0xF0;
	write_spi(PMSC, PMSC_SOFTRESET_SUB, pmsc_ctrl0, 1);

	// load the LDE microcode from ROM into RAM, table 4, 2.5.5.10 page23/244
    
    // Load the LDE algorithm microcode into LDE RAM or disable LDE execution (clear LDERUNE)
	delay(5);

	pmsc_ctrl0[0] = 0x01;
	pmsc_ctrl0[1] = 0x03;

	write_spi(PMSC, PMSC_SYSCLKS_SUB, pmsc_ctrl0, 2);

	// delay(1);

	uint8_t opt_ctrl[2];
	opt_ctrl[0] = 0x00;
	opt_ctrl[1] = 0x80;

	write_spi(OTP_IF, OTP_CTRL_SUB, opt_ctrl, 2);

	delay(5);	//wait 150 us, take more

	pmsc_ctrl0[0] = 0x00;//because we didn't clear pmsc_cltr0 variable!
	pmsc_ctrl0[1] = 0x02;

	write_spi(PMSC, PMSC_SYSCLKS_SUB, pmsc_ctrl0, 2);

	delay(10);
}

void SENSOR::config_init()
{
	pinMode(SS, OUTPUT);	//otherwise we get FFFF-255-15-15
	digitalWrite(SS,HIGH); // unselect slave
	delay(1);
	dwm_reset();			//not necessary. Also, a lot of delay!
	pinMode(SS, OUTPUT);	//otherwise we get FFFF-255-15-15
	digitalWrite(SS,HIGH); // unselect slave
	Serial.println("end config sensor -- sensor class");
}

void SENSOR::dwm_init()
{
	this -> config_init();//this -> is not necessary (it's implied already)
	uint8_t sys_cfg [4];
	uint8_t spi_edge[4]; //page 70
	spi_edge[0] = 0x00;
	spi_edge[1] = 0x04;
	spi_edge[2] = 0x00;
	spi_edge[3] = 0x00;

	write_spi(SYS_CFG, NO_SUB, spi_edge, 4);

	read_spi(SYS_CFG, NO_SUB, sys_cfg, 4);
	Serial.println("read configuration, sys_cfg attribute:");
	Serial.println(sys_cfg[0]);
	Serial.println(sys_cfg[1]);
	Serial.println(sys_cfg[2]);
	Serial.println(sys_cfg[3]);
}

void SENSOR::get_device_ID()
{
	uint8_t spi_buffer[4];
	char msg[128];	//we can also format the result with sprintf in this string
	read_spi(DEV_ID, NO_SUB, spi_buffer, 4);
	uint32_t deviceID =(spi_buffer[3] << 24) | (spi_buffer[2] << 16) | (spi_buffer[1] << 8) | spi_buffer[0];

	sprintf(msg, "%02X - model: %d, version: %d, revision: %d",
					(uint16_t)((spi_buffer[3] << 8) | spi_buffer[2]), spi_buffer[1], (spi_buffer[0] >> 4) & 0x0F, spi_buffer[0] & 0x0F);

	Serial.println("dwm1000 ::");
	Serial.println(msg);
	// transmitter_init();
	// receiver_Init();
	// // config.channel = _5;
	// DWM_UpdateChannel();
}
// void SENSOR::idle(){

// }
// these functions are not accessible outside of this file. Could just put them as private methods in the class?
void read_spi(uint8_t address, uint16_t offset, uint8_t* spi_buffer, uint16_t len)
{
	uint8_t header[3];
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
	SPI.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));

	digitalWrite(SS, LOW);
	for(i = 0; i < headerLen; i++) {
		SPI.transfer(header[i]); // send header
	}
	for(i = 0; i < len; i++) {
		spi_buffer[i] = SPI.transfer(JUNK); // read values
	}
	delayMicroseconds(5);
	digitalWrite(SS, HIGH);

	SPI.endTransaction();
	// interrupts(); ?? begin transactions turns them off but are not turned back on?

}

void write_spi(uint8_t address, uint16_t offset, uint8_t* spi_buffer, uint16_t len)
{
	uint8_t header[3];
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
	SPI.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));

	digitalWrite(SS,LOW); //slave select low to start the communication
	for(i = 0; i < headerLen; i++) {
		SPI.transfer(header[i]); // send header
	}

	for(i = 0; i < len; i++) {
		SPI.transfer(spi_buffer[i]); // write values
	}
	delayMicroseconds(5);//find out why
	digitalWrite(SS,HIGH);	//de-select slave

	SPI.endTransaction();
	// interrupts(); ?? begin transactions turns them off but are not turned back on?
}