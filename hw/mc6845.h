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

#ifndef HW_MC6845_H
#define HW_MC6845_H

typedef struct MC6845CRTC MC6845CRTC;

/** A 6845 CRTC */
struct MC6845CRTC {
	/** Name */
	const char *name;
	/** Memory region */
	MemoryRegion mr;
};

/** Size of memory region */
#define MC6845_SIZE 0x02

extern void mc6845_init ( MemoryRegion *parent, hwaddr offset, hwaddr size,
			  const char *name );

#endif
