/*
 * Motorola 6850 virtual ACIA
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

#include "hw.h"
#include "boards.h"
#include "exec/address-spaces.h"
#include "mc6850.h"

#define DEBUG_MC6850 1

/* Debug messages */
#define LOG_MC6850(...) do {						\
		if ( DEBUG_MC6850 ) {					\
			qemu_log_mask ( CPU_LOG_IOPORT, __VA_ARGS__ );	\
		}							\
	} while ( 0 )

/**
 * Read from 6850 ACIA
 *
 * @v opaque		6850 ACIA
 * @v addr		Register address
 * @v size		Size of read
 * @ret data		Read data
 */
static uint64_t mc6850_read ( void *opaque, hwaddr addr, unsigned int size ) {
	MC6850ACIA *acia = opaque;
	uint8_t data;

	/* Read from specified register */
	switch ( addr & ( MC6850_SIZE - 1 ) ) {
	default:
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented read from "
				"0x%02lx\n", acia->name, addr );
		data = 0;
		break;
	}
	return data;
}

/**
 * Write to 6850 ACIA
 *
 * @v opaque		6850 ACIA
 * @v addr		Register address
 * @v data64		Data to write
 * @v size		Size of write
 */
static void mc6850_write ( void *opaque, hwaddr addr, uint64_t data64,
			   unsigned int size ) {
	MC6850ACIA *acia = opaque;
	uint8_t data = data64;

	/* Write to specified register */
	switch ( addr & ( MC6850_SIZE - 1 ) ) {
	default:
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented write 0x%02x to "
				"0x%02lx\n", acia->name, data, addr );
		break;
	}
}

/** 6850 ACIA operations */
static const MemoryRegionOps mc6850_ops = {
	.read = mc6850_read,
	.write = mc6850_write,
};

/** Virtual machine state description */
static const VMStateDescription vmstate_mc6850 = {
	.name = "mc6850",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Initialise 6850 ACIA
 *
 * @v parent		Parent memory region
 * @v offset		Offset within parent memory region
 * @v size		Size of memory region
 * @v name		Device name
 * @v chr		Character device
 * @ret acia		6850 ACIA
 */
MC6850ACIA * mc6850_init ( MemoryRegion *parent, hwaddr offset, uint64_t size,
			   const char *name, CharDriverState *chr ) {
	MC6850ACIA *acia = g_new0 ( MC6850ACIA, 1 );

	/* Initialise ACIA */
	acia->name = name;
	acia->chr = chr;

	/* Register memory region */
	memory_region_init_io ( &acia->mr, &mc6850_ops, acia, acia->name,
				size );
	memory_region_add_subregion ( parent, offset, &acia->mr );

	/* Register virtual machine state */
	vmstate_register ( NULL, offset, &vmstate_mc6850, acia );

	return acia;
}
