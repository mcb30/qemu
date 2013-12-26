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

#include "hw.h"
#include "boards.h"
#include "exec/address-spaces.h"
#include "block/block_int.h"
#include "bbc.h"
#include "bbc_8272.h"

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
 * @v fdc		8272 FDC
 * @ret name		Device name
 */
static inline const char * bbc8272_name ( BBC8272FDC *fdc ) {

	return qdev_fw_name ( &fdc->bbcfdc.qdev );
}

/**
 * Read from 8272 FDC control register
 *
 * @v opaque		8272 FDC
 * @v addr		Register address
 * @v size		Size of read
 * @ret data		Read data
 */
static uint64_t bbc8272_fdc_read ( void *opaque, hwaddr addr,
				   unsigned int size ) {
	BBC8272FDC *fdc = opaque;

	//
	LOG_BBC ( "%s: control register (&%02lX) read\n",
		  bbc8272_name ( fdc ), addr );

	/* This is a write-only register */
	return 0xff;
}

/**
 * Write to 8272 FDC control register
 *
 * @v opaque		8272 FDC
 * @v addr		Register address
 * @v data64		Data to write
 * @v size		Size of write
 */
static void bbc8272_fdc_write ( void *opaque, hwaddr addr, uint64_t data64,
				unsigned int size ) {
	BBC8272FDC *fdc = opaque;
	uint8_t data = data64;

	/* Write to control register */
	fdc->control = data;
	LOG_BBC ( "%s: control register (&%02lX) set to &%02X\n",
		  bbc8272_name ( fdc ), addr, data );	
}

/** 8272 FDC operations */
static const MemoryRegionOps bbc8272_fdc_ops = {
	.read = bbc8272_fdc_read,
	.write = bbc8272_fdc_write,
};

/**
 * Initialise 8272 FDC
 *
 * @v bbcfdc		BBC FDC device
 * @v mr		Memory region
 * @v drq		Data request non-maskable interrupt request line
 * @v intrq		Completion non-maskable interrupt request line
 * @ret fdc		8272 FDC
 */
static int bbc8272_init ( BBCFDCDevice *bbcfdc, MemoryRegion *mr, qemu_irq drq,
			  qemu_irq intrq, BlockDriverState **block ) {
	BBC8272FDC *fdc = DO_UPCAST ( BBC8272FDC, bbcfdc, bbcfdc );
	const char *control_name;
	const char *i8272_name;

	/* Initialise control register */
	control_name = g_strdup_printf ( "%s.control", bbc8272_name ( fdc ) );
	memory_region_init_io ( &fdc->mr, &bbc8272_fdc_ops, fdc, control_name,
				BBC8272_CONTROL_SIZE );
	memory_region_add_subregion ( mr, BBC8272_CONTROL_OFFSET, &fdc->mr );

	/* Initialise 8272 FDC */
	i8272_name = g_strdup_printf ( "%s.i8272", bbc8272_name ( fdc ) );
	fdc->fdc = i8272_create ( mr, BBC8272_I8272_OFFSET, i8272_name,
				  drq, intrq, block );

	return 0;
}

/** 8272 FDC state description */
static const VMStateDescription vmstate_bbc8272 = {
	.name = "bbc8272",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_UINT8 ( control, BBC8272FDC ),
		VMSTATE_END_OF_LIST()
	},
};

/** UDM 8272 FDC properties */
static Property udm8272_properties[] = {
	DEFINE_PROP_END_OF_LIST(),
};

/** 8272 FDC class initialiser */
static void bbc8272_class_init ( ObjectClass *class, void *data,
				 Property *props ) {
	DeviceClass *dc = DEVICE_CLASS ( class );
	BBCFDCDeviceClass *fdc_class = BBC_FDC_DEVICE_CLASS ( class );

	fdc_class->init = bbc8272_init;
	dc->vmsd = &vmstate_bbc8272;
	dc->props = props;
}

/** UDM 8272 FDC class initialiser */
static void udm8272_class_init ( ObjectClass *class, void *data ) {
	bbc8272_class_init ( class, data, udm8272_properties );
}

/** UDM 8272 FDC information */
static TypeInfo udm8272_info = {
	.name = TYPE_BBC8272_UDM,
	.parent = TYPE_BBC_FDC_DEVICE,
	.instance_size = sizeof ( BBC8272FDC ),
	.class_init = udm8272_class_init,
};

/** Type registrar */
static void bbc8272_register_types ( void ) {
	type_register_static ( &udm8272_info );
}

/** Type initialiser */
type_init ( bbc8272_register_types );
