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

#ifndef HW_MC6850_H
#define HW_MC6850_H

typedef struct MC6850ACIA MC6850ACIA;

/** A 6850 ACIA */
struct MC6850ACIA {
	/** Name */
	const char *name;
	/** Memory region */
	MemoryRegion mr;
	/** Character device */
	CharDriverState *chr;
};

/** Size of memory region */
#define MC6850_SIZE 16

extern void mc6850_init ( MemoryRegion *parent, hwaddr offset,
			  const char *name, CharDriverState *chr );

#endif
