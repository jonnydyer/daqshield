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

#ifndef _DAQSHIELD_H_INCLUDED
#define _DAQSHIELD_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "pins_arduino.h"

// LEDS
#define STAT_LED_1 6
#define STAT_LED_2 7
#define LED_ON 0				// Open drain configuration
#define LED_OFF 1

// Setting defines for ADS1248
#define ADC_5_SPS 0x00
#define ADC_10_SPS 0x01
#define ADC_20_SPS 0x02
#define ADC_40_SPS 0x03
#define ADC_80_SPS 0x04
#define ADC_160_SPS 0x05
#define ADC_320_SPS 0x06
#define ADC_640_SPS 0x07
#define ADC_1000_SPS 0x08
#define ADC_2000_SPS 0x09
#define ADC_PGA_1 (0x00 << 4)
#define ADC_PGA_2 (0x01 << 4)
#define ADC_PGA_4 (0x02 << 4)
#define ADC_PGA_8 (0x03 << 4)
#define ADC_PGA_16 (0x04 << 4)
#define ADC_PGA_32 (0x05 << 4)
#define ADC_PGA_64 (0x06 << 4)
#define ADC_PGA_128 (0x07 << 4)
#define ADC_IDAC_0uA 0x00
#define ADC_IDAC_50uA 0x01
#define ADC_IDAC_100uA 0x02
#define ADC_IDAC_250uA 0x03
#define ADC_IDAC_500uA 0x04
#define ADC_IDAC_750uA 0x05
#define ADC_IDAC_1000uA 0x06
#define ADC_IDAC_1500uA 0x07
#define ADC_IDAC1_IEXT1 (0x80 << 4)
#define ADC_IDAC1_IEXT2 (0x81 << 4)
#define ADC_IDAC1_DISCONNECTED (0xC0 << 4)
#define ADC_IDAC2_IEXT1 0x80
#define ADC_IDAC2_IEXT2 0x81
#define ADC_IDAC2_DISCONNECTED 0xC0
#define ADC_REF0 (0x00 << 3)
#define ADC_REF1 (0x01 << 3)
#define ADC_INTREF (0x02 << 3)
#define ADC_INTREF_REF0 (0x03 << 3)
#define ADC_VREF_OFF (0x00 << 5)
#define ADC_VREF_ON (0x01 << 5)
#define ADC_VREF_ON_LP (0x02 << 5)
#define ADC_MUXCAL_NORM 0x00
#define ADC_MUXCAL_OFFSET 0x01
#define ADC_MUXCAL_GAIN 0x02
#define ADC_MUXCAL_TEMP 0x03
#define ADC_MUXCAL_REF1 0x04
#define ADC_MUXCAL_REF0 0x05
#define ADC_MUXCAL_AVDD 0x06
#define ADC_MUXCAL_DVDD 0x07


class DAQShieldClass {
public:
	bool begin(void);
	void end(void);
	
	void resetDataFlash(void);
	void reset(void);
	
	void wake(void);
	void sleep(void);
	void stopContinuous(void);
	
	void startSingle(void);
	float sample(uint8_t ch_pos, uint8_t ch_neg, uint8_t gain, uint8_t sampleRate, uint8_t ref);
	float sample(uint8_t muxcal, uint8_t sampleRate=ADC_5_SPS);
	
	void setPGA(uint8_t gain);
	void setSampleRate(uint8_t rate);
	void enableIntRef(void);
	void selectRef(uint8_t ref_mux);
	void selectMuxCal(uint8_t muxcal);
	void selfOffsetCal(void);
	void dumpRegs(void);
	
	uint8_t readReg(uint8_t addr);
	int32_t readData(void);
	void writeReg(uint8_t addr, uint8_t data);
private:
	void initSPI(void);
};

extern DAQShieldClass DAQShield;

#endif
