/*
 * Acorn BBC Micro 8272-based floppy disc controller
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

#ifndef HW_BBC_8272_H
#define HW_BBC_8272_H

#include "i8272.h"

/* Floppy disc controller */
#define BBC8272_CONTROL_OFFSET 0x00
#define BBC8272_CONTROL_SIZE 0x04
#define BBC8272_I8272_OFFSET 0x04
#define BBC8272_I8272_SIZE 0x04
#define BBC8272_SIZE 0x08

/**
 * BBC 8272 floppy disc controller
 */
typedef struct {
	/** BBC FDC device */
	BBCFDCDevice bbcfdc;
	/** Memory region */
	MemoryRegion mr;
	/** 8272 FDC */
	I8272FDC *fdc;

	/** Control register */
	uint8_t control;
} BBC8272FDC;

/* Known BBC 8272 floppy disc controller types */
#define TYPE_BBC8272_UDM "udm8272"

#endif
