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

#include "hw.h"
#include "boards.h"
#include "exec/address-spaces.h"
#include "sysbus.h"
#include "i8272.h"

#define DEBUG_I8272 1

/* Debug messages */
#define LOG_I8272( ... ) do {						\
		if ( DEBUG_I8272 ) {					\
			qemu_log_mask ( CPU_LOG_IOPORT, __VA_ARGS__ );	\
		}							\
	} while ( 0 )

/**
 * Reset controller
 *
 * @v fdc		8272 FDC
 * @v drive		Drive number
 *
 * The reset line may be under software control, to allow the
 * controller to be reset in the event of an error.
 */
static void i8272_reset ( I8272FDC *fdc ) {

	LOG_I8272 ( "%s: reset\n", fdc->name );
}

/**
 * Read from 8272 FDC
 *
 * @v opaque		8272 FDC
 * @v addr		Register address
 * @v size		Size of read
 * @ret data		Read data
 */
static uint64_t i8272_read ( void *opaque, hwaddr addr, unsigned int size ) {
	I8272FDC *fdc = opaque;
	uint8_t data;

	LOG_I8272 ( "%s: read 0x%02lx\n", fdc->name, addr );
	data = 0x80;

	return data;
}

/**
 * Write to 8272 FDC
 *
 * @v opaque		8272 FDC
 * @v addr		Register address
 * @v data64		Data to write
 * @v size		Size of write
 */
static void i8272_write ( void *opaque, hwaddr addr, uint64_t data64,
			  unsigned int size ) {
	I8272FDC *fdc = opaque;
	uint8_t data = data64;

	LOG_I8272 ( "%s: write 0x%02lx <= 0x%02x\n", fdc->name, addr, data );
}

/** 8272 FDC operations */
static const MemoryRegionOps i8272_ops = {
	.read = i8272_read,
	.write = i8272_write,
};

/** 8272 FDC block device operations */
static const BlockDevOps i8272_block_ops = {
	//	.change_media_cb = i8272_change_media_cb,
	//	.resize_cb = i8272_resize_cb,
};

/**
 * Initialise 8272 FDC
 *
 * @v fdc		8272 FDC
 * @v name		Device name
 */
static void i8272_core_init ( I8272FDC *fdc, const char *name ) {
	I8272FDD *fdd;
	unsigned int i;

	/* Initialise FDC */
	fdc->name = name;

	/* Initialise drives */
	for ( i = 0 ; i < ARRAY_SIZE ( fdc->fdds ) ; i++ ) {
		fdd = &fdc->fdds[i];
		fdd->fdc = fdc;
		if ( fdd->block )
			bdrv_set_dev_ops ( fdd->block, &i8272_block_ops, fdd );
	}

	/* Initialise memory region */
	memory_region_init_io ( &fdc->mr, &i8272_ops, fdc, name, I8272_SIZE );

	/* Reset device */
	i8272_reset ( fdc );
}

/** Virtual machine state description (drive) */
static const VMStateDescription vmstate_i8272_fdd = {
	.name = "i8272fdd",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_END_OF_LIST()
	},
};

/** Virtual machine state description (controller) */
static const VMStateDescription vmstate_i8272 = {
	.name = "i8272",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_STRUCT_ARRAY ( fdds, I8272FDC, I8272_DRIVE_COUNT,
				       1, vmstate_i8272_fdd, I8272FDD ),
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Create 8272 FDC embedded device
 *
 * @v mr		Memory region
 * @v offset		Offset within memory region
 * @v name		Name
 * @v drq		Data request IRQ
 * @v intrq		Completion IRQ
 * @v block		Block devices
 * @ret fdc		8272 FDC
 */
I8272FDC * i8272_create ( MemoryRegion *mr, hwaddr offset, const char *name,
			  qemu_irq drq, qemu_irq intrq,
			  BlockDriverState **block ) {
	I8272FDC *fdc = g_new0 ( I8272FDC, 1 );
	unsigned int i;

	/* Associate block devices */
	for ( i = 0 ; i < ARRAY_SIZE ( fdc->fdds ) ; i++ )
		fdc->fdds[i].block = block[i];

	/* Initialise device */
	i8272_core_init ( fdc, name );

	/* Connect to embedded bus */
	memory_region_add_subregion ( mr, offset, &fdc->mr );
	fdc->drq = drq;
	fdc->intrq = intrq;

	/* Register virtual machine state */
	vmstate_register ( NULL, offset, &vmstate_i8272, fdc );

	return fdc;
}
			  
/**
 * Initialise 8272 FDC system bus device
 *
 * @v busdev		System bus device
 */
static int i8272_sysbus_init ( SysBusDevice *busdev ) {
	I8272FDCSysBus *fdc_sysbus =
		DO_UPCAST ( I8272FDCSysBus, busdev, busdev );
	I8272FDC *fdc = &fdc_sysbus->fdc;

	/* Initialise 8272 FDC */
	i8272_core_init ( fdc, qdev_fw_name ( &busdev->qdev ) );

	/* Register system bus memory region */
	sysbus_init_mmio ( busdev, &fdc->mr );

	/* Register system bus interrupts */
	sysbus_init_irq ( busdev, &fdc->drq );
	sysbus_init_irq ( busdev, &fdc->intrq );

	return 0;
}

/**
 * Create 8272 FDC system bus device
 *
 * @v addr		Address
 * @v drq		Data request IRQ
 * @v intrq		Completion IRQ
 * @v block		Block devices
 * @ret fdc		8272 FDC
 */
I8272FDC * i8272_sysbus_create ( hwaddr addr, qemu_irq drq, qemu_irq intrq,
				 BlockDriverState **block ) {
	DeviceState *qdev;
	I8272FDCSysBus *fdc_sysbus;
	I8272FDC *fdc;

	/* Create device */
	qdev = qdev_create ( NULL, TYPE_I8272 );
	fdc_sysbus = DO_UPCAST ( I8272FDCSysBus, busdev.qdev, qdev );
	fdc = &fdc_sysbus->fdc;
	if ( block[0] )
		qdev_prop_set_drive_nofail ( qdev, "driveA", block[0] );
	if ( block[1] )
		qdev_prop_set_drive_nofail ( qdev, "driveB", block[1] );
	qdev_init_nofail ( qdev );

	/* Connect to system bus */
	sysbus_mmio_map ( &fdc_sysbus->busdev, 0, addr );
	sysbus_connect_irq ( &fdc_sysbus->busdev, 0, drq );
	sysbus_connect_irq ( &fdc_sysbus->busdev, 1, intrq );

	return fdc;
}

/** 8272 FDC system bus device virtual machine state description */
static const VMStateDescription vmstate_i8272_sysbus = {
	.name = "i8272",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_STRUCT ( fdc, I8272FDCSysBus, 0,
				 vmstate_i8272, I8272FDC ),
		VMSTATE_END_OF_LIST()
	},
};

/** 8272 FDC system bus device properties */
static Property i8272_sysbus_properties[] = {
	DEFINE_PROP_DRIVE ( "driveA", I8272FDC, fdds[0].block ),
	DEFINE_PROP_DRIVE ( "driveB", I8272FDC, fdds[1].block ),
	DEFINE_PROP_END_OF_LIST(),
};

/** 8272 FDC system bus device class initialiser */
static void i8272_sysbus_class_init ( ObjectClass *class, void *data ) {
	DeviceClass *dc = DEVICE_CLASS ( class );
	SysBusDeviceClass *busdev_class = SYS_BUS_DEVICE_CLASS ( class );

	busdev_class->init = i8272_sysbus_init;
	dc->vmsd = &vmstate_i8272_sysbus;
	dc->props = i8272_sysbus_properties;
}

/** 8272 FDC system bus device type information */
static TypeInfo i8272_sysbus_info = {
	.name = TYPE_I8272,
	.parent = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof ( I8272FDCSysBus ),
	.class_init = i8272_sysbus_class_init,
};

/** Type registrar */
static void i8272_register_types ( void ) {
	type_register_static ( &i8272_sysbus_info );
}

/** Type initialiser */
type_init ( i8272_register_types );
