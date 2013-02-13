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
#define BBC1770_CONTROL_OFFSET 0x00
#define BBC1770_CONTROL_SIZE 0x04
#define BBC1770_WD1770_OFFSET 0x04
#define BBC1770_WD1770_SIZE 0x04
#define BBC1770_SIZE 0x08

/* Floppy disc control register */
#define BBC1770_DRIVE_MASK 0x03
#define BBC1770_DRIVE_0 0x01
#define BBC1770_DRIVE_1 0x02
#define BBC1770_SIDE_1 0x04
#define BBC1770_SINGLE_DENSITY 0x08
#define BBC1770_NOT_MASTER_RESET 0x20

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
} BBC1770FDC;

#endif
