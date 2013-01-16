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
#include "sysbus.h"
#include "wd1770.h"

#define DEBUG_WD1770 1

/* Debug messages */
#define DBG_WD1770( fdc, fmt, ... ) do {				\
	if ( DEBUG_WD1770 ) {						\
		qemu_log_mask ( CPU_LOG_IOPORT, "%s: " fmt,		\
				qdev_fw_name ( &(fdc)->busdev.qdev ),	\
				##__VA_ARGS__ );			\
	}								\
	} while ( 0 )

/* Unimplemented functionality messages */
#define LOG_WD1770_UNIMP( fdc, fmt, ... ) do {				\
	qemu_log_mask ( LOG_UNIMP, "%s: " fmt,				\
			qdev_fw_name ( &(fdc)->busdev.qdev ),		\
			##__VA_ARGS__ );				\
	} while ( 0 )

/**
 * Get currently selected drive
 *
 * @v fdc		1770 FDC
 * @ret fdd		Currently selected drive, or NULL
 */
static WD1770FDD * wd1770_fdd ( WD1770FDC *fdc ) {
	int drive = fdc->drive;

	/* Return drive if drive number is valid and block device exists */
	if ( ( drive >= 0 ) && ( drive < ARRAY_SIZE ( fdc->fdds ) ) &&
	     ( fdc->fdds[drive].block != NULL ) ) {
		return &fdc->fdds[drive];
	} else {
		return NULL;
	}
}

/**
 * Reset controller
 *
 * @v fdc		1770 FDC
 * @v drive		Drive number
 *
 * The reset line may be under software control, to allow the
 * controller to be reset in the event of an error.
 */
void wd1770_reset ( WD1770FDC *fdc ) {

	DBG_WD1770 ( fdc, "Reset\n" );

	/* Abort current command */
	fdc->command = 0;

	/* Reset track and sector registers */
	fdc->track = 0;
	fdc->sector = 0;

	/* Switch off motor */
	fdc->motor = false;
}

/**
 * Set currently selected drive
 *
 * @v fdc		1770 FDC
 * @v drive		Drive number
 *
 * The WD1770 chip does not generate a drive select signal; this
 * information must be provided externally.
 */
void wd1770_set_drive ( WD1770FDC *fdc, int drive ) {
	WD1770FDD *fdd;

	/* Record drive */
	fdc->drive = drive;
	fdd = wd1770_fdd ( fdc );
	DBG_WD1770 ( fdc, "DRIVE=%d (%s)\n", drive,
		     ( ( drive < 0 ) ? "<unselected>" :
		       ( fdd ? bdrv_get_device_name ( fdd->block ) :
			 "<missing>" ) ) );
}

/**
 * Set currently selected side
 *
 * @v fdc		1770 FDC
 * @v side		Side number
 *
 * The WD1770 chip does not generate a side select signal; this
 * information must be provided externally.
 */
void wd1770_set_side ( WD1770FDC *fdc, unsigned int side ) {

	DBG_WD1770 ( fdc, "SIDE=%d\n", side );

	/* Record side */
	fdc->side = side;
}

/**
 * Set single/double density
 *
 * @v fdc		1770 FDC
 * @v single_density	Use single density
 *
 * The WD1770 chip does not provide a register to select disk density;
 * this information must be provided externally.
 */
void wd1770_set_single_density ( WD1770FDC *fdc, bool single_density ) {

	DBG_WD1770 ( fdc, "DENSITY=%s\n",
		     ( single_density ? "SINGLE" : "DOUBLE" ) );

	/* Record side */
	fdc->single_density = single_density;
}

/**
 * Handle issued command
 *
 * @v fdc		1770 FDC
 * @v command		Command
 */
static void wd1770_command_write ( WD1770FDC *fdc, uint8_t command ) {

	

	/* Record command */
	fdc->command = command;
}

/**
 * Read from 1770 FDC status register
 *
 * @v fdc		1770 FDC
 * @ret data		Data
 */
static uint8_t wd1770_status_read ( WD1770FDC *fdc ) {
	uint8_t data;

	data = ( ( fdc->motor ? WD1770_STAT_MOTOR_ON : 0 ) |
		 0 );
		 

	return data;
}

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

	DBG_WD1770 ( fdc, "TRACK=%d\n", data );

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

	DBG_WD1770 ( fdc, "SECTOR=%d\n", data );

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
	case WD1770_STATUS:
		data = wd1770_status_read ( fdc );
		break;
	case WD1770_TRACK:
		data = wd1770_track_read ( fdc );
		break;
	case WD1770_SECTOR:
		data = wd1770_sector_read ( fdc );
		break;
	default:
		LOG_WD1770_UNIMP ( fdc, "unimplemented read from "
				   "0x%02lx\n", addr );
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
	case WD1770_COMMAND:
		wd1770_command_write ( fdc, data );
		break;
	case WD1770_TRACK:
		wd1770_track_write ( fdc, data );
		break;
	case WD1770_SECTOR:
		wd1770_sector_write ( fdc, data );
		break;
	default:
		LOG_WD1770_UNIMP ( fdc, "unimplemented write 0x%02x to "
				   "0x%02lx\n", data, addr );
		break;
	}
}

/** 1770 FDC operations */
static const MemoryRegionOps wd1770_ops = {
	.read = wd1770_read,
	.write = wd1770_write,
};

/**
 * Initialise 1770 FDC system bus device
 *
 * @v busdev		System bus device
 */
static int wd1770_sysbus_init ( SysBusDevice *busdev ) {
	WD1770FDC *fdc = DO_UPCAST ( WD1770FDC, busdev, busdev );
	unsigned int i;

	/* Initialise FDC */
	for ( i = 0 ; i < ARRAY_SIZE ( fdc->fdds ) ; i++ ) {
		fdc->fdds[i].fdc = fdc;
	}

	/* Initialise memory region */
	memory_region_init_io ( &fdc->mr, &wd1770_ops, fdc, "wd1770",
				WD1770_SIZE );
	sysbus_init_mmio ( busdev, &fdc->mr );

	/* Initialise interrupts */
	sysbus_init_irq ( busdev, &fdc->drq );
	sysbus_init_irq ( busdev, &fdc->intrq );

	return 0;
}

/**
 * Initialise 1770 FDC
 *
 * @v addr		Address
 * @v drq		Data request IRQ
 * @v intrq		Completion IRQ
 * @v fds		Floppy disk drive information
 * @ret fdc		1770 FDC
 */
WD1770FDC * wd1770_init ( hwaddr addr, qemu_irq drq, qemu_irq intrq,
			  DriveInfo **fds ) {
	DeviceState *dev;
	WD1770FDC *fdc;

	/* Create device */
	dev = qdev_create ( NULL, "wd1770" );
	fdc = DO_UPCAST ( WD1770FDC, busdev.qdev, dev );
	if ( fds[0] )
		qdev_prop_set_drive_nofail ( dev, "driveA", fds[0]->bdrv );
	if ( fds[1] )
		qdev_prop_set_drive_nofail ( dev, "driveB", fds[1]->bdrv );
	qdev_init_nofail ( dev );

	/* Connect to system bus */
	sysbus_mmio_map ( &fdc->busdev, 0, addr );
	sysbus_connect_irq ( &fdc->busdev, 0, drq );
	sysbus_connect_irq ( &fdc->busdev, 1, intrq );

	return fdc;
}
			  
/** Virtual machine state description (drive) */
static const VMStateDescription vmstate_wd1770_fdd = {
	.name = "wd1770fdd",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_UINT8 ( track, WD1770FDD ),
		VMSTATE_END_OF_LIST()
	},
};

/** Virtual machine state description (controller) */
static const VMStateDescription vmstate_wd1770 = {
	.name = "wd1770",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_STRUCT_ARRAY ( fdds, WD1770FDC, WD1770_DRIVE_COUNT,
				       1, vmstate_wd1770_fdd, WD1770FDD ),
		VMSTATE_INT8 ( drive, WD1770FDC ),
		VMSTATE_UINT8 ( side, WD1770FDC ),
		VMSTATE_BOOL ( single_density, WD1770FDC ),
		VMSTATE_UINT8 ( track, WD1770FDC ),
		VMSTATE_UINT8 ( sector, WD1770FDC ),
		VMSTATE_UINT8 ( command, WD1770FDC ),
		VMSTATE_BOOL ( motor, WD1770FDC ),
		VMSTATE_END_OF_LIST()
	},
};

/** Properties */
static Property wd1770_properties[] = {
	DEFINE_PROP_DRIVE ( "driveA", WD1770FDC, fdds[0].block ),
	DEFINE_PROP_DRIVE ( "driveB", WD1770FDC, fdds[1].block ),
	DEFINE_PROP_END_OF_LIST(),
};

/** Class initialiser */
static void wd1770_class_init ( ObjectClass *class, void *data ) {
	DeviceClass *dc = DEVICE_CLASS ( class );
	SysBusDeviceClass *busdev_class = SYS_BUS_DEVICE_CLASS ( class );

	busdev_class->init = wd1770_sysbus_init;
	dc->vmsd = &vmstate_wd1770;
	dc->props = wd1770_properties;
}

/** Type information */
static TypeInfo wd1770_info = {
	.name = "wd1770",
	.parent = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof ( WD1770FDC ),
	.class_init = wd1770_class_init,
};

/** Type registrar */
static void wd1770_register_types ( void ) {
	type_register_static ( &wd1770_info );
}

/** Type initialiser */
type_init ( wd1770_register_types );
