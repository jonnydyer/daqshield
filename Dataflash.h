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

#ifndef _DATAFLASH_H_INCLUDED
#define _DATAFLASH_H_INCLUDED

// DATAFLASH
#define DATAFLASH_SS A3 
#define DATAFLASH_RESET A2
#define DATAFLASH_SPI_MODE ((1<<CPOL) | (1<<CPHA))

#endif
