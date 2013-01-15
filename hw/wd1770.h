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

#ifndef HW_WD1770_H
#define HW_WD1770_H

/** 1770 FDC */
typedef struct {
	/** Name */
	const char *name;
	/** Memory region */
	MemoryRegion mr;
	/* Data request interrupt request line */
	qemu_irq drq;
	/* Completion interrupt request line */
	qemu_irq intrq;

	/** Current track position */
	uint8_t phys_track;
	/** Track register */
	uint8_t track;
	/** Sector register */
	uint8_t sector;
} WD1770FDC;

/** Size of memory region */
#define WD1770_SIZE 0x04

/* Register addresses */
#define WD1770_COMMAND 0x00
#define WD1770_STATUS 0x00
#define WD1770_TRACK 0x01
#define WD1770_SECTOR 0x02
#define WD1770_DATA 0x03

extern WD1770FDC * wd1770_init ( MemoryRegion *parent, hwaddr offset,
				 uint64_t size, const char *name,
				 qemu_irq drq, qemu_irq intrq );

#endif
