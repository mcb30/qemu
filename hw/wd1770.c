/*
 * Western Digital 1770 floppy disk controller
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
#include "wd1770.h"

#define DEBUG_WD1770 1

/* Debug messages */
#define LOG_WD1770(...) do {						\
		if ( DEBUG_WD1770 ) {					\
			qemu_log_mask ( CPU_LOG_IOPORT, __VA_ARGS__ );	\
		}							\
	} while ( 0 )

/**
 * Read from 1770 FDC track register
 *
 * @v fdc		1770 FDC
 * @ret data		Data
 */
static uint8_t wd1770_track_read ( WD1770FDC *fdc ) {
	uint8_t data;

	/* Read track register */
	data = fdc->track;

	return data;
}

/**
 * Write to 1770 FDC track register
 *
 * @v fdc		1770 FDC
 * @v data		Data
 */
static void wd1770_track_write ( WD1770FDC *fdc, uint8_t data ) {

	LOG_WD1770 ( "%s: TRACK=0x%02x\n", fdc->name, data );

	/* Write track register */
	fdc->track = data;
}

/**
 * Read from 1770 FDC sector register
 *
 * @v fdc		1770 FDC
 * @ret data		Data
 */
static uint8_t wd1770_sector_read ( WD1770FDC *fdc ) {
	uint8_t data;

	/* Read sector register */
	data = fdc->sector;

	return data;
}

/**
 * Write to 1770 FDC sector register
 *
 * @v fdc		1770 FDC
 * @v data		Data
 */
static void wd1770_sector_write ( WD1770FDC *fdc, uint8_t data ) {

	LOG_WD1770 ( "%s: SECTOR=0x%02x\n", fdc->name, data );

	/* Write sector register */
	fdc->sector = data;
}

/**
 * Read from 1770 FDC
 *
 * @v opaque		1770 FDC
 * @v addr		Register address
 * @v size		Size of read
 * @ret data		Read data
 */
static uint64_t wd1770_read ( void *opaque, hwaddr addr, unsigned int size ) {
	WD1770FDC *fdc = opaque;
	uint8_t data;

	/* Read from specified register */
	switch ( addr & ( WD1770_SIZE - 1 ) ) {
	case WD1770_TRACK:
		data = wd1770_track_read ( fdc );
		break;
	case WD1770_SECTOR:
		data = wd1770_sector_read ( fdc );
		break;
	default:
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented read from "
				"0x%02lx\n", fdc->name, addr );
		data = 0xff;
		break;
	}
	return data;
}

/**
 * Write to 1770 FDC
 *
 * @v opaque		1770 FDC
 * @v addr		Register address
 * @v data64		Data to write
 * @v size		Size of write
 */
static void wd1770_write ( void *opaque, hwaddr addr, uint64_t data64,
			   unsigned int size ) {
	WD1770FDC *fdc = opaque;
	uint8_t data = data64;

	/* Write to specified register */
	switch ( addr & ( WD1770_SIZE - 1 ) ) {
	case WD1770_TRACK:
		wd1770_track_write ( fdc, data );
		break;
	case WD1770_SECTOR:
		wd1770_sector_write ( fdc, data );
		break;
	default:
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented write 0x%02x to "
				"0x%02lx\n", fdc->name, data, addr );
		break;
	}
}

/** 1770 FDC operations */
static const MemoryRegionOps wd1770_ops = {
	.read = wd1770_read,
	.write = wd1770_write,
};

/** Virtual machine state description */
static const VMStateDescription vmstate_wd1770 = {
	.name = "wd1770",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_UINT8 ( phys_track, WD1770FDC ),
		VMSTATE_UINT8 ( track, WD1770FDC ),
		VMSTATE_UINT8 ( sector, WD1770FDC ),
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Initialise 1770 FDC
 *
 * @v parent		Parent memory region
 * @v offset		Offset within parent memory region
 * @v size		Size of memory region
 * @v name		Device name
 * @v drq		Data request interrupt request line
 * @v intrq		Completion interrupt request line
 * @ret fdc		1770 FDC
 */
WD1770FDC * wd1770_init ( MemoryRegion *parent, hwaddr offset, uint64_t size,
			  const char *name, qemu_irq drq, qemu_irq intrq ) {
	WD1770FDC *fdc = g_new0 ( WD1770FDC, 1 );

	/* Initialise FDC */
	fdc->name = name;
	fdc->drq = drq;
	fdc->intrq = intrq;

	/* Register memory region */
	memory_region_init_io ( &fdc->mr, &wd1770_ops, fdc, fdc->name, size );
	memory_region_add_subregion ( parent, offset, &fdc->mr );

	/* Register virtual machine state */
	vmstate_register ( NULL, offset, &vmstate_wd1770, fdc );

	return fdc;
}
