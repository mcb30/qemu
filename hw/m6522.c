/*
 * MOS Technology 6522 virtual VIA
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
#include "m6522.h"

#define DEBUG_M6522 1

/* Debug messages */
#define LOG_M6522(...) do {						\
		if ( DEBUG_M6522 ) {					\
			qemu_log_mask ( CPU_LOG_IOPORT, __VA_ARGS__ );	\
		}							\
	} while ( 0 )

/**
 * Output to 6522 VIA PORT
 *
 * @v via		6522 VIA
 * @v port		Port
 * @v data		Output data
 */
static void m6522_output ( M6522VIA *via, M6522VIAPort *port, uint8_t data ) {

	/* Update stored OR */
	port->or = data;

	/* Output to port */
	if ( port->ops->output )
		port->ops->output ( via, port, data );
}

/**
 * Read from 6522 VIA
 *
 * @v opaque		6522 VIA
 * @v addr		Register address
 * @v size		Size of read
 * @ret data		Read data
 */
static uint64_t m6522_read ( void *opaque, hwaddr addr, unsigned int size ) {
	M6522VIA *via = opaque;
	uint8_t data;

	/* Read from specified register */
	switch ( addr ) {
	case M6522_IRB:
		data = ( via->b.ops->input ?
			 via->b.ops->input ( via, &via->b ) : 0 );
		/* For IRB (but not IRA), pins programmed as outputs will
		 * always read back the programmed output value.
		 */
		data = ( ( data & ~via->b.ddr ) | ( via->b.or & via->b.ddr ) );
		LOG_M6522 ( "%s: IRB=0x%02x\n", via->name, data );
		break;
	case M6522_IRA:
	case M6522_IRA_NO_HS:
		data = ( via->a.ops->input ?
			 via->a.ops->input ( via, &via->a ) : 0 );
		LOG_M6522 ( "%s: IRA=0x%02x\n", via->name, data );
		break;
	case M6522_DDRB:
		data = via->b.ddr;
		break;
	case M6522_DDRA:
		data = via->a.ddr;
		break;
	default:
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented read from "
				"0x%02lx\n", via->name, addr );
		data = 0;
		break;
	}
	return data;
}

/**
 * Write to 6522 VIA
 *
 * @v opaque		6522 VIA
 * @v addr		Register address
 * @v data64		Data to write
 * @v size		Size of write
 */
static void m6522_write ( void *opaque, hwaddr addr, uint64_t data64,
			  unsigned int size ) {
	M6522VIA *via = opaque;
	uint8_t data = data64;

	/* Write to specified register */
	switch ( addr ) {
	case M6522_ORB:
		LOG_M6522 ( "%s: ORB=0x%02x\n", via->name, data );
		m6522_output ( via, &via->b, data );
		break;
	case M6522_ORA:
	case M6522_ORA_NO_HS:
		LOG_M6522 ( "%s: ORA=0x%02x\n", via->name, data );
		m6522_output ( via, &via->a, data );
		break;
	case M6522_DDRB:
		LOG_M6522 ( "%s: DDRB=0x%02x\n", via->name, data );
		via->b.ddr = data;
		break;
	case M6522_DDRA:
		LOG_M6522 ( "%s: DDRA=0x%02x\n", via->name, data );
		via->a.ddr = data;
		break;
	default:
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented write 0x%02x to "
				"0x%02lx\n", via->name, data, addr );
		break;
	}
}

/** 6522 VIA operations */
static const MemoryRegionOps m6522_ops = {
	.read = m6522_read,
	.write = m6522_write,
};

/** Virtual machine state description */
static const VMStateDescription vmstate_m6522 = {
	.name = "m6522",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_UINT8 ( b.or, M6522VIA ),
		VMSTATE_UINT8 ( b.ddr, M6522VIA ),
		VMSTATE_UINT8 ( a.or, M6522VIA ),
		VMSTATE_UINT8 ( a.ddr, M6522VIA ),
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Initialise 6522 VIA
 *
 * @v parent		Parent memory region
 * @v offset		Offset within memory region
 * @v name		Device name
 * @v ops		VIA operations
 */
void m6522_init ( MemoryRegion *parent, hwaddr offset, const char *name,
		  const M6522VIAOps *ops ) {
	M6522VIA *via = g_new0 ( M6522VIA, 1 );

	/* Initialise VIA */
	via->name = name;
	via->b.ops = &ops->b;
	via->a.ops = &ops->a;

	/* Register memory region */
	memory_region_init_io ( &via->mr, &m6522_ops, via, via->name,
				M6522_SIZE );
	memory_region_add_subregion ( parent, offset, &via->mr );

	/* Register virtual machine state */
	vmstate_register ( NULL, offset, &vmstate_m6522, via );
}
