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

#include "hw.h"
#include "boards.h"
#include "exec/address-spaces.h"
#include "block/block_int.h"
#include "bbc.h"
#include "bbc_1770.h"

#define DEBUG_BBC 1

/* Debug messages */
#define LOG_BBC( ... ) do {						\
		if ( DEBUG_BBC ) {					\
			qemu_log_mask ( CPU_LOG_IOPORT, __VA_ARGS__ );	\
		}							\
	} while ( 0 )

/**
 * Get device name for log messages
 *
 * @v fdc		1770 FDC
 * @ret name		Device name
 */
static inline const char * bbc1770_name ( BBC1770FDC *fdc ) {

	return qdev_fw_name ( &fdc->bbcfdc.qdev );
}

/**
 * Guess disk geometry
 *
 * @v opaque		1770 FDC
 * @v fdd		FDD
 * @ret rc		Return status
 */
static int bbc1770_fdc_guess_geometry ( void *opaque, WD1770FDD *fdd ) {
	BBC1770FDC *fdc = opaque;
	static const uint8_t try_tracks[] = { 80, 40 };
	static const uint8_t try_sides[] = { 2, 1 };
	static const uint8_t try_sectors[] = { 18, 16, 10 };
	char *filename;
	char *suffix;
	int64_t length;
	unsigned int cyl_size;
	unsigned int i;
	unsigned int j;
	unsigned int k;

	/* Get size of block device */
	length = bdrv_getlength ( fdd->block );
	if ( length < 0 ) {
		LOG_BBC ( "%s: could not determine length for %s\n",
			  bbc1770_name ( fdc ),
			  bdrv_get_device_name ( fdd->block ) );
		return -1;
	}

	/* Determine file suffix, if any */
	filename = fdd->block->filename;
	if ( filename ) {
		suffix = strrchr ( filename, '.' );
		if ( suffix )
			suffix++;
	} else {
		suffix = NULL;
	}

	/* Treat ".ssd" and ".dsd" filename suffixes as being in the
	 * standard single-density format common to downloadable
	 * images.
	 */
	if ( suffix && ( ( strcasecmp ( suffix, "ssd" ) == 0 ) ||
			 ( strcasecmp ( suffix, "dsd" ) == 0 ) ) ) {
		fdd->single_density = true;
		fdd->sides = ( ( tolower ( suffix[0] ) == 's' ) ? 1 : 2 );
		fdd->sectors = BBC_DFS_SECTORS;
		fdd->sector_size = BBC_DFS_SECTOR_SIZE;
		cyl_size = ( fdd->sides * fdd->sectors * fdd->sector_size );
		fdd->tracks = ( ( length + cyl_size - 1 ) / cyl_size );
		return 0;
	}

	/* Otherwise, try various standard geometry combinations to
	 * see if one matches the size.
	 */
	fdd->sector_size = BBC_DFS_SECTOR_SIZE;
	for ( i = 0 ; i < ARRAY_SIZE ( try_tracks ) ; i++ ) {
		fdd->tracks = try_tracks[i];
		for ( j = 0 ; j < ARRAY_SIZE ( try_sides ) ; j++ ) {
			fdd->sides = try_sides[j];
			for ( k = 0 ; k < ARRAY_SIZE ( try_sectors ) ; k++ ) {
				fdd->sectors = try_sectors[k];
				fdd->single_density =
					( fdd->sectors == BBC_DFS_SECTORS );
				if ( ( fdd->sides * fdd->tracks *
				       fdd->sectors * fdd->sector_size )
				     == length ) {
					return 0;
				}
			}
		}
	}

	return -1;
}

/** 1770 FDC 1770 operations */
static WD1770Ops bbc1770_fdc_1770_ops = {
	.guess_geometry = bbc1770_fdc_guess_geometry,
};

/**
 * Read from 1770 FDC control register
 *
 * @v opaque		1770 FDC
 * @v addr		Register address
 * @v size		Size of read
 * @ret data		Read data
 */
static uint64_t bbc1770_fdc_read ( void *opaque, hwaddr addr,
				   unsigned int size ) {

	/* This is a write-only register */
	return 0xff;
}

/**
 * Write to 1770 FDC control register
 *
 * @v opaque		1770 FDC
 * @v addr		Register address
 * @v data64		Data to write
 * @v size		Size of write
 */
static void bbc1770_fdc_write ( void *opaque, hwaddr addr, uint64_t data64,
				unsigned int size ) {
	BBC1770FDC *fdc = opaque;
	uint8_t data = data64;
	int drive;
	unsigned int side;
	bool single_density;
	bool master_reset;

	/* Write to control register */
	fdc->control = data;
	LOG_BBC ( "%s: control register (&%02lX) set to &%02X\n",
		  bbc1770_name ( fdc ), addr, data );

	/* Select drive number */
	switch ( data & BBC1770_DRIVE_MASK ) {
	case 0:
		/* No drive selected */
		drive = WD1770_NO_DRIVE;
		break;
	case BBC1770_DRIVE_0:
		drive = 0;
		break;
	case BBC1770_DRIVE_1:
		drive = 1;
		break;
	default:
		/* Invalid combination */
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented simultaneous "
				"operation of both drives\n",
				bbc1770_name ( fdc ) );
		drive = WD1770_NO_DRIVE;
		break;
	}
	wd1770_set_drive ( fdc->fdc, drive );

	/* Decode side number */
	side = ( ( data & BBC1770_SIDE_1 ) ? 1 : 0 );
	wd1770_set_side ( fdc->fdc, side );

	/* Decode density */
	single_density = ( !! ( data & BBC1770_SINGLE_DENSITY ) );
	wd1770_set_single_density ( fdc->fdc, single_density );

	/* Decode master reset.  This line is supposed to be
	 * level-sensitive; the real hardware would be held in reset
	 * while the line remained active.
	 */
	master_reset = ( ! ( data & BBC1770_NOT_MASTER_RESET ) );
	if ( master_reset )
		wd1770_reset ( fdc->fdc );
}

/** 1770 FDC operations */
static const MemoryRegionOps bbc1770_fdc_ops = {
	.read = bbc1770_fdc_read,
	.write = bbc1770_fdc_write,
};

/**
 * Initialise 1770 FDC
 *
 * @v bbcfdc		BBC FDC device
 * @v mr		Memory region
 * @v drq		Data request non-maskable interrupt request line
 * @v intrq		Completion non-maskable interrupt request line
 * @ret fdc		1770 FDC
 */
static int bbc1770_init ( BBCFDCDevice *bbcfdc, MemoryRegion *mr, qemu_irq drq,
			  qemu_irq intrq, BlockDriverState **block ) {
	BBC1770FDC *fdc = DO_UPCAST ( BBC1770FDC, bbcfdc, bbcfdc );
	const char *control_name;
	const char *wd1770_name;

	/* The memory map for this peripheral is a little strange.
	 * The BBC was originally designed to take an Intel 8271 FDC,
	 * occupying eight bytes of address space.  The WD1770 FDC was
	 * mapped in to the latter four byte of this address space,
	 * with the first four bytes being occupied by a 74LS174 hex
	 * D-type which provides additional signals to the WD1770.
	 */

	/* Initialise control register */
	control_name = g_strdup_printf ( "%s.control", bbc1770_name ( fdc ) );
	memory_region_init_io ( &fdc->mr, &bbc1770_fdc_ops, fdc, control_name,
				BBC1770_CONTROL_SIZE );
	memory_region_add_subregion ( mr, BBC1770_CONTROL_OFFSET, &fdc->mr );

	/* Initialise 1770 FDC */
	wd1770_name = g_strdup_printf ( "%s.wd1770", bbc1770_name ( fdc ) );
	fdc->fdc = wd1770_create ( mr, BBC1770_WD1770_OFFSET, wd1770_name,
				   drq, intrq, block );
	wd1770_set_ops ( fdc->fdc, &bbc1770_fdc_1770_ops, fdc );

	return 0;
}

/** 1770 FDC state description */
static const VMStateDescription vmstate_bbc1770 = {
	.name = "bbc1770",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_UINT8 ( control, BBC1770FDC ),
		VMSTATE_END_OF_LIST()
	},
};

/** 1770 FDC properties */
static Property bbc1770_properties[] = {
	DEFINE_PROP_END_OF_LIST(),
};

/** 1770 FDC class initialiser */
static void bbc1770_class_init ( ObjectClass *class, void *data ) {
	DeviceClass *dc = DEVICE_CLASS ( class );
	BBCFDCDeviceClass *fdc_class = BBC_FDC_DEVICE_CLASS ( class );

	fdc_class->init = bbc1770_init;
	dc->vmsd = &vmstate_bbc1770;
	dc->props = bbc1770_properties;
}

/** 1770 FDC information */
static TypeInfo bbc1770_info = {
	.name = "bbc1770",
	.parent = TYPE_BBC_FDC_DEVICE,
	.instance_size = sizeof ( BBC1770FDC ),
	.class_init = bbc1770_class_init,
};

/** Type registrar */
static void bbc1770_register_types ( void ) {
	type_register_static ( &bbc1770_info );
}

/** Type initialiser */
type_init ( bbc1770_register_types );
