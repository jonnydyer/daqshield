/*
 * \brief Library for operating the DAQShield.
 *
 * This library has all necessary functions for using the ADS1248
 * and DATAFLASH part on the Arduino DAQShield.
 * Copyright (c) 2012 by Jonny Dyer <jonny.dyer@gmail.com>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include "pins_arduino.h"
#include "DAQShield.h"
#include "ADS1248.h"
#include "Dataflash.h"

#define SPI_MODE_MASK 0x0C  	// CPOL = bit 3, CPHA = bit 2 on SPCR

static inline uint8_t spiTransferByte(uint8_t data);
static inline void setSPIMode(uint8_t mode);

DAQShieldClass DAQShield;

bool DAQShieldClass::begin(void)
{
	uint8_t temp;
	
	// Set pin idle states
	digitalWrite(SCK, LOW);
	digitalWrite(MOSI, LOW);
	digitalWrite(SS, HIGH);
	digitalWrite(MISO, LOW);
	digitalWrite(DATAFLASH_SS, HIGH);
	digitalWrite(DATAFLASH_RESET, HIGH);
	digitalWrite(ADS1248_RESET, HIGH);
	digitalWrite(ADS1248_START, LOW);
	digitalWrite(STAT_LED_1, LED_OFF);
	digitalWrite(STAT_LED_2, LED_OFF);
	
	// Set pin directions
	pinMode(SCK, OUTPUT);
	pinMode(MOSI, OUTPUT);
	pinMode(SS, OUTPUT);
	pinMode(MISO,INPUT);
	pinMode(DATAFLASH_SS, OUTPUT);
	pinMode(DATAFLASH_RESET,OUTPUT);
	pinMode(ADS1248_RESET,OUTPUT);
	pinMode(ADS1248_START,OUTPUT);
	pinMode(ADS1248_DRDY,INPUT);
	pinMode(STAT_LED_1, OUTPUT);
	pinMode(STAT_LED_2, OUTPUT);
	
	// Init SPI - f_cpu/128
	SPCR = (1<<MSTR) | (1<<SPE) | (0<<SPR1) | (1<<SPR0);
	SPSR |= (1<<SPI2X);				//2x speed
	// clear status by doing a read
	SPSR;SPDR;
	
	// Reset components
	//resetDataFlash();
	reset();
	
	temp = readReg(ADS1248_TEST_REG);
	writeReg(ADS1248_TEST_REG, 0x30);
	if(readReg(ADS1248_TEST_REG) == 0x30)
	{
		// Indicate that we are inited
		digitalWrite(STAT_LED_1, LED_ON);
		writeReg(ADS1248_TEST_REG,temp); 		//Return to reset state
		// Perform self offset calibration
		selfOffsetCal();
		return true;
	}
	return false;
	
}

void DAQShieldClass::end(void)
{
	SPCR &= ~(1<<SPE);
	digitalWrite(STAT_LED_1, LED_OFF);
	digitalWrite(STAT_LED_2, LED_OFF);
}

void DAQShieldClass::resetDataFlash(void)
{
	digitalWrite(DATAFLASH_RESET,LOW);
	delay(1);			// Min 10uS
	digitalWrite(DATAFLASH_RESET,HIGH);
	delay(2);			// Min 1uS
}

void DAQShieldClass::enableIntRef(void)
{
	writeReg(ADS1248_MUX1,readReg(ADS1248_MUX1) | ADC_VREF_ON);
}

void DAQShieldClass::selectRef(uint8_t ref_mux)
{
	writeReg(ADS1248_MUX1,(readReg(ADS1248_MUX1) & ~ADS1248_REFSEL_MASK) | (ref_mux & ADS1248_REFSEL_MASK));
}

void DAQShieldClass::selectMuxCal(uint8_t muxcal)
{
	writeReg(ADS1248_MUX1,(readReg(ADS1248_MUX1) & ~ADS1248_MUXCAL_MASK) | (muxcal & ADS1248_MUXCAL_MASK));
}

void DAQShieldClass::reset(void)
{
	digitalWrite(ADS1248_RESET,LOW);
	delay(1);			// Min 4*t_osc
	digitalWrite(ADS1248_RESET,HIGH);
	delay(1);			// min .6ms
}

/* float sample(uint8_t ch_pos, uint8_t ch_neg, uint8_t gain, uint8_t ref)
 * 
 * Perform a blocking read of one channel pair of the ADC.  Enables the internal reference
 * if not already enabled.
 * TODO: put timeout on DRDY in case the ADS1248 doesn't return
 */
float DAQShieldClass::sample(uint8_t ch_pos, uint8_t ch_neg, uint8_t gain, uint8_t sampleRate, uint8_t ref)
{
	int32_t res;
	// Enable START to allow writing to registers
	digitalWrite(ADS1248_START, HIGH);
	// Check if reference on.  If not, turn on and wait...
	if(readReg(ADS1248_MUX1) & ADC_VREF_ON)
		writeReg(ADS1248_MUX1, ADC_VREF_ON | (ADS1248_REFSEL_MASK & ref));
	else
	{
		writeReg(ADS1248_MUX1, ADC_VREF_ON | (ADS1248_REFSEL_MASK & ref));
		// Delay 1ms to allow reference to settle
		delay(1);
	}
	// Setup gain and sample rate
	writeReg(ADS1248_SYS0, (ADS1248_PGA_MASK & gain) | (ADS1248_SPS_MASK & sampleRate));
	// Choose channels
	writeReg(ADS1248_MUX0, (ADS1248_CH_MASK & ch_neg) | ((ADS1248_CH_MASK & ch_pos) << 3));
	// Disable START to reset sample
	digitalWrite(ADS1248_START, LOW);
	delayMicroseconds(1);
	// Start conversion
	startSingle();
	// Wait for completion - TODO: put timeout on DRDY in case the ADS1248 doesn't return
	while(digitalRead(ADS1248_DRDY) == HIGH){};
	// Retrieve result
	res = readData();
	// Scale result
	if((ref == ADC_INTREF) || (ref == ADC_INTREF_REF0))
	{
		return (float)res / ADS1248_MAX_VAL * 0.001 * ADS1248_INT_REF_MV;// / (float)(1 << (gain >> 4));
	}
	else
	{
		return (float)res / ADS1248_MAX_VAL / (float)(1 << (gain >> 4));
	}
}

/* float sample(uint8_t muxcal, uint8_t sampleRate)
 * 
 * Perform a blocking read of one of the MUXCAL inputs.  Enables the internal reference
 * if not already enabled.
 * TODO: put timeout on DRDY in case the ADS1248 doesn't return
 */
float DAQShieldClass::sample(uint8_t muxcal, uint8_t sampleRate)
{
	int32_t res;
	// Enable START to allow writing to registers
	digitalWrite(ADS1248_START, HIGH);
	// Check if reference on.  If not, turn on and wait...
	if(readReg(ADS1248_MUX1) & ADC_VREF_ON)
		writeReg(ADS1248_MUX1, ADC_VREF_ON | ADC_INTREF | (ADS1248_MUXCAL_MASK & muxcal));
	else
	{
		writeReg(ADS1248_MUX1, ADC_VREF_ON | ADC_INTREF | (ADS1248_MUXCAL_MASK & muxcal));
		// Delay 1ms to allow reference to settle
		delay(1);
	}
	// Setup gain and sample rate
	writeReg(ADS1248_SYS0, ADS1248_SPS_MASK & sampleRate);
	// Disable START to reset sample
	digitalWrite(ADS1248_START, LOW);
	delayMicroseconds(1);
	// Start conversion
	startSingle();
	// Wait for completion - TODO: put timeout on DRDY in case the ADS1248 doesn't return
	while(digitalRead(ADS1248_DRDY) == HIGH){};
	// Retrieve result
	res = readData();
	// Scale result
	return (float)res / ADS1248_MAX_VAL * 0.001 * ADS1248_INT_REF_MV;
}

void DAQShieldClass::startSingle(void)
{
	digitalWrite(ADS1248_START, HIGH);
	delayMicroseconds(2);				// Datasheet specifies minimum 3*t_osc which would really be < 1us
	digitalWrite(ADS1248_START, LOW);
}

void DAQShieldClass::setPGA(uint8_t gain)
{
	writeReg(ADS1248_SYS0,(readReg(ADS1248_SYS0) & ADS1248_SPS_MASK) | (gain & ADS1248_PGA_MASK));
}

void DAQShieldClass::setSampleRate(uint8_t rate)
{
	writeReg(ADS1248_SYS0,(readReg(ADS1248_SYS0) & ADS1248_PGA_MASK) | (rate & ADS1248_SPS_MASK));
}

// len can be no more than 16 bytes
uint8_t DAQShieldClass::readReg(uint8_t addr)
{
	uint8_t ret;
	setSPIMode(ADS1248_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	spiTransferByte((addr & 0x0F) | ADS1248_CMD_RREG);
	spiTransferByte(0x00);
	ret = spiTransferByte(ADS1248_CMD_NOP);
	digitalWrite(SS, HIGH);  // Pull CS high
	return ret;
}

int32_t DAQShieldClass::readData(void)
{
	uint32_t ret;
	uint8_t byte;
	setSPIMode(ADS1248_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	byte = spiTransferByte(ADS1248_CMD_NOP);
	ret = (((uint32_t)byte) << 16) | ((byte & 0x80) ? 0xFF000000:0x00000000);
	ret |= (((uint32_t)spiTransferByte(ADS1248_CMD_NOP)) << 8);
	ret |= (uint32_t)spiTransferByte(ADS1248_CMD_NOP);
	digitalWrite(SS, HIGH);  // Pull CS high
	return ret;
}

void DAQShieldClass::stopContinuous(void)
{
	setSPIMode(ADS1248_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	spiTransferByte(ADS1248_CMD_SDATAC);
	digitalWrite(SS, HIGH);  // Pull CS high
}

void DAQShieldClass::writeReg(uint8_t addr, uint8_t data)
{
	digitalWrite(SS, HIGH);  // Just to be sure
	setSPIMode(ADS1248_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	spiTransferByte((addr & 0x0F) | ADS1248_CMD_WREG);
	spiTransferByte(0x00);
	spiTransferByte(data);
	digitalWrite(SS, HIGH);  // Pull CS high
}

void DAQShieldClass::wake(void)
{
	setSPIMode(ADS1248_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	spiTransferByte(ADS1248_CMD_WAKE);
	digitalWrite(SS, HIGH);  // Pull CS high
}

void DAQShieldClass::sleep(void)
{
	setSPIMode(ADS1248_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	spiTransferByte(ADS1248_CMD_SELFOCAL);
	digitalWrite(SS, HIGH);  // Pull CS high
}

void DAQShieldClass::selfOffsetCal(void)
{
	uint8_t MUX1_old = readReg(ADS1248_MUX1);
	digitalWrite(ADS1248_START, HIGH);
	enableIntRef();
	selectRef(ADC_INTREF);
	digitalWrite(ADS1248_START, LOW);
	delay(1);			// Wait for ref to settle
	setSPIMode(ADS1248_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	spiTransferByte(ADS1248_CMD_SLEEP);
	digitalWrite(SS, HIGH);  // Pull CS high
	// TODO add timeout here
	while(digitalRead(ADS1248_DRDY) == HIGH){};
	digitalWrite(ADS1248_START, HIGH);
	writeReg(ADS1248_MUX1,MUX1_old);
	digitalWrite(ADS1248_START, LOW);
}

void DAQShieldClass::dumpRegs(void)
{
	digitalWrite(ADS1248_START, HIGH);
	for(uint8_t i = 0;i<0x0F;i++)
	{
		Serial.print("Reg 0x");
		Serial.print(i,16);
		Serial.print(" : 0x");
		Serial.print(readReg(i),16);
		Serial.print("\n");
	}
	digitalWrite(ADS1248_START, LOW);
}

// Remember that this function does NOT control the chip select line
uint8_t spiTransferByte(uint8_t data)
{
	SPDR = data;
	// wait for transfer to complete
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}
// Mode is 0 or 3 for dataflash and 1 for ADS1248
void setSPIMode(uint8_t mode)
{
	SPCR = (SPCR & ~SPI_MODE_MASK) | mode;
}