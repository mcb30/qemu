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

#ifndef HW_M6522_H
#define HW_M6522_H

typedef struct M6522VIA M6522VIA;
typedef struct M6522VIAPort M6522VIAPort;

/** A 6522 VIA port */
struct M6522VIAPort {
	/**
	 * Output to port
	 *
	 * @v via		6522 VIA
	 * @v port		Port
	 * @v data		Output data
	 */
	void ( * output ) ( M6522VIA *via, M6522VIAPort *port, uint8_t data );
	/**
	 * Input from port
	 *
	 * @v via		6522 VIA
	 * @v port		Port
	 * @ret data		Input data
	 */
	uint8_t ( * input ) ( M6522VIA *via, M6522VIAPort *port );
	/** Output register */
	uint8_t or;
	/** Data direction register */
	uint8_t ddr;
};

/** A 6522 VIA */
struct M6522VIA {
	/** Memory region name */
	const char *name;
	/** Port B */
	M6522VIAPort b;
	/** Port A */
	M6522VIAPort a;
};

/** Size of memory region */
#define M6522_VIA_SIZE 16

/* Register addresses */
#define M6522_ORB 0x0
#define M6522_IRB 0x0
#define M6522_ORA 0x1
#define M6522_IRA 0x1
#define M6522_DDRB 0x2
#define M6522_DDRA 0x3
#define M6522_ORA_NO_HS 0xf
#define M6522_IRA_NO_HS 0xf

extern void m6522_init ( M6522VIA *via, MemoryRegion *parent, hwaddr offset,
			 unsigned int priority );

#endif
