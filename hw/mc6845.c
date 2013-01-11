/*
 * Motorola 6845 virtual CRTC
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
#include "mc6845.h"
#define DEBUG_MC6850 1

/* Debug messages */
#define LOG_MC6845(...) do {						\
		if ( DEBUG_MC6845 ) {					\
			qemu_log_mask ( CPU_LOG_IOPORT, __VA_ARGS__ );	\
		}							\
	} while ( 0 )

/**
 * Read from 6845 CRTC
 *
 * @v opaque		6845 CRTC
 * @v addr		Register address
 * @v size		Size of read
 * @ret data		Read data
 */
static uint64_t mc6845_read ( void *opaque, hwaddr addr, unsigned int size ) {
	MC6845CRTC *crtc = opaque;
	uint8_t data;

	/* Read from specified register */
	switch ( addr & ( MC6845_SIZE - 1 ) ) {
	default:
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented read from "
				"0x%02lx\n", crtc->name, addr );
		data = 0;
		break;
	}
	return data;
}

/**
 * Write to 6845 CRTC
 *
 * @v opaque		6845 CRTC
 * @v addr		Register address
 * @v data64		Data to write
 * @v size		Size of write
 */
static void mc6845_write ( void *opaque, hwaddr addr, uint64_t data64,
			   unsigned int size ) {
	MC6845CRTC *crtc = opaque;
	uint8_t data = data64;

	/* Write to specified register */
	switch ( addr & ( MC6845_SIZE - 1 ) ) {
	default:
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented write 0x%02x to "
				"0x%02lx\n", crtc->name, data, addr );
		break;
	}
}

/** 6845 CRTC operations */
static const MemoryRegionOps mc6845_ops = {
	.read = mc6845_read,
	.write = mc6845_write,
};

/** Virtual machine state description */
static const VMStateDescription vmstate_mc6845 = {
	.name = "mc6845",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Initialise 6845 CRTC
 *
 * @v parent		Parent memory region
 * @v offset		Offset within parent memory region
 * @v size		Size of memory region
 * @v name		Device name
 */
void mc6845_init ( MemoryRegion *parent, hwaddr offset, hwaddr size,
		   const char *name ) {
	MC6845CRTC *crtc = g_new0 ( MC6845CRTC, 1 );

	/* Initialise CRTC */
	crtc->name = name;

	/* Register memory region */
	memory_region_init_io ( &crtc->mr, &mc6845_ops, crtc, crtc->name,
				size );
	memory_region_add_subregion ( parent, offset, &crtc->mr );

	/* Register virtual machine state */
	vmstate_register ( NULL, offset, &vmstate_mc6845, crtc );
}
