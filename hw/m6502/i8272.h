/*
 * Intel 8272 floppy disk controller
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

#ifndef HW_I8272_H
#define HW_I8272_H

#include "hw/sysbus.h"

typedef struct I8272FDC I8272FDC;

/** Number of drives */
#define I8272_DRIVE_COUNT 2

/** 8272 FDC attached drive */
typedef struct {
	/** Controller */
	I8272FDC *fdc;
	/** Block device */
	BlockDriverState *block;
} I8272FDD;

/** 8272 FDC */
struct I8272FDC {
	/** Name */
	const char *name;
	/** Memory region */
	MemoryRegion mr;
	/** Data request IRQ */
	qemu_irq drq;
	/** Completion IRQ */
	qemu_irq intrq;
	/** Attached drives */
	I8272FDD fdds[I8272_DRIVE_COUNT];
};

/** 8272 FDC system bus device */
typedef struct {
	/** System bus device */
	SysBusDevice busdev;
	/** 8272 FDC */
	I8272FDC fdc;
} I8272FDCSysBus;

/** Size of memory region */
#define I8272_SIZE 0x02

/** I8272 type name */
#define TYPE_I8272 "i8272"

extern I8272FDC * i8272_create ( MemoryRegion *mr, hwaddr offset,
				 const char *name, qemu_irq drq,
				 qemu_irq intrq, BlockDriverState **block );
extern I8272FDC * i8272_sysbus_create ( hwaddr addr, qemu_irq drq,
					qemu_irq intrq,
					BlockDriverState **block );

#endif
