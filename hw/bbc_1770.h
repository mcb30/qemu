/*
 * Acorn BBC Micro 1770-based floppy disc controller
 *
 * Copyright (C) 2013 Michael Brown <mbrown@fensystems.co.uk>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_BBC_1770_H
#define HW_BBC_1770_H

#include "wd1770.h"

/* Floppy disc controller */
#define BBC1770_OFFSET_LOW 0x00
#define BBC1770_OFFSET_HIGH 0x04
#define BBC1770_CONTROL_SIZE 0x04
#define BBC1770_WD1770_SIZE 0x04
#define BBC1770_SIZE 0x08

/**
 * Control register bit names
 *
 * These values show up in the user-visible "controls" property; do
 * not reassign values.
 */
enum {
	BBC1770_CTRL_DRIVE0	= 0x00,
	BBC1770_CTRL_DRIVE1	= 0x04,
	BBC1770_CTRL_SIDE1	= 0x08,
	BBC1770_CTRL_DOUBLE	= 0x0c,
	BBC1770_CTRL_RESET	= 0x10,
};

/**
 * Define a control register bit
 *
 * @v name		Control register bit name
 * @v bit		Bit number within control register
 * @v inverted		Bit is inverted
 * @ret control		Control register bit definition (ORed into "controls")
 */
#define BBC1770_CTRL( name, bit, active_low ) \
	( ( ( (active_low) << 3 ) | bit ) << (name) )

/**
 * Get control register bit position
 *
 * @v controls		Control register bit definitions
 * @v name		Control register bit name
 * @ret bit		Bit number within control register
 */
#define BBC1770_CTRL_BIT( controls, name ) \
	( ( (controls) >> (name) ) & 0x07 )

/**
 * Get control register bit invertedness
 *
 * @v controls		Control register bit definitions
 * @v name		Control register bit name
 * @ret inverted	Bit is inverted
 */
#define BBC1770_CTRL_INVERTED( controls, name ) \
	( ( (controls) >> ( 3 + (name) ) ) & 0x01 )

/**
 * Configuration bits   
 *
 * These values show up in the user-visible "config" property; do not
 * reassign values.
 */
enum {
	BBC1770_CFG_CTRL_HIGH	= 0x00000001UL,
	BBC1770_CFG_HAS_RESET	= 0x00000002UL,
};

/**
 * BBC 1770 floppy disc controller
 */
typedef struct {
	/** BBC FDC device */
	BBCFDCDevice bbcfdc;
	/** Memory region */
	MemoryRegion mr;
	/** WD1770 FDC */
	WD1770FDC *fdc;

	/** Control register */
	uint8_t control;
	/** Control register bit definitions */
	uint32_t controls;
	/** Configuration */
	uint32_t config;
} BBC1770FDC;

/* Known BBC 1770 floppy disc controller types */
#define TYPE_BBC1770_ACORN "acorn1770"
#define TYPE_BBC1770_OPUS "opus1770"

#endif

