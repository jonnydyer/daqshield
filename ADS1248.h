/*
 * \brief Definitions for the ADS1248 part
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

#ifndef _ADS1248_H_INCLUDED
#define _ADS1248_H_INCLUDED

// Commands
#define ADS1248_RESET 2
#define ADS1248_START 3
#define ADS1248_DRDY 4
#define ADS1248_SPI_MODE ((0<<CPOL) | (1<<CPHA))
#define ADS1248_CMD_WAKE 0x00
#define ADS1248_CMD_SLEEP 0x02
#define ADS1248_CMD_SYNC 0x04
#define ADS1248_CMD_RESET 0x06
#define ADS1248_CMD_RDATA 0x12
#define ADS1248_CMD_RDATAC 0x14
#define ADS1248_CMD_SDATAC 0x16
#define ADS1248_CMD_RREG 0x20
#define ADS1248_CMD_WREG 0x40
#define ADS1248_CMD_SYSOCAL 0x60
#define ADS1248_CMD_SYSGCAL 0x61
#define ADS1248_CMD_SELFOCAL 0x62
#define ADS1248_CMD_NOP	0xFF

// Registers
#define ADS1248_MUX0 0x00
#define ADS1248_VBIAS 0x01
#define ADS1248_MUX1 0x02
#define ADS1248_SYS0 0x03
#define ADS1248_OFC0 0x04
#define ADS1248_OFC1 0x05
#define ADS1248_OFC2 0x06
#define ADS1248_FSC0 0x07
#define ADS1248_FSC1 0x08
#define ADS1248_FSC2 0x09
#define ADS1248_IDAC0 0x0A
#define ADS1248_IDAC1 0x0B
#define ADS1248_GPIOCFG 0x0C
#define ADS1248_GPIODIR 0x0D
#define ADS1248_GPIODAT 0x0E

// Register Bit Masks
#define ADS1248_CH_MASK 0x07			// Low 3 bits used for MUX in MUX0, IDAC1 and IDAC0
#define ADS1248_PGA_MASK 0x70
#define ADS1248_SPS_MASK 0x0F
#define ADS1248_REFSEL_MASK (0x03 << 3)
#define ADS1248_VREF_MASK (0x03 << 5)
#define ADS1248_MUXCAL_MASK 0x07

#define ADS1248_MAX_VAL 0x7FFFFF
#define ADS1248_MIN_VAL 0x800000
#define ADS1248_RANGE 0xFFFFFF
#define ADS1248_INT_REF_MV 2048

#define ADS1248_TEST_REG ADS1248_MUX1

#define ADS1248_DELAY delayMicroseconds(10)

#endif
