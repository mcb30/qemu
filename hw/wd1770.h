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

#include "sysbus.h"

typedef struct WD1770FDC WD1770FDC;

/** Number of drives */
#define WD1770_DRIVE_COUNT 2

/** Maximum sector size */
#define WD1770_MAX_SECTOR_SIZE 1024

/** 1770 FDC drive */
typedef struct {
	/** Controller */
	WD1770FDC *fdc;
	/** Block device */
	BlockDriverState *block;
	/** Number of tracks on this block device */
	uint8_t track_count;

	/** Current track position (may exceed limit of block device) */
	uint8_t track;
} WD1770FDD;

/** 1770 FDC */
struct WD1770FDC {
	/** System bus device */
	SysBusDevice busdev;
	/** Memory region */
	MemoryRegion mr;
	/** Data request IRQ */
	qemu_irq drq;
	/** Completion IRQ */
	qemu_irq intrq;
	/** Attached drives */
	WD1770FDD fdds[WD1770_DRIVE_COUNT];
	/** Sector buffer */
	uint8_t *buf;
	/** Motor switch-off timer */
	QEMUTimer *motor_off;

	/** Selected drive (negative for no drive) */
	int8_t drive;
	/** Selected side */
	uint8_t side;
	/** Selected density */
	bool single_density;
	/** Command register */
	uint8_t command;
	/** Status register */
	uint8_t status;
	/** Track register (may not match actual track position) */
	uint8_t track;
	/** Sector register */
	uint8_t sector;
	/** Data register */
	uint8_t data;
	/** Step direction */
	int8_t step;
};

/** Size of memory region */
#define WD1770_SIZE 0x04

/* Register addresses */
#define WD1770_COMMAND 0x00
#define WD1770_STATUS 0x00
#define WD1770_TRACK 0x01
#define WD1770_SECTOR 0x02
#define WD1770_DATA 0x03

/* Command register */
#define WD1770_CMD_VERIFY 0x04
#define WD1770_CMD_DISABLE_SPIN_UP 0x08
#define WD1770_CMD_UPDATE_TRACK 0x10

/* Status register */
#define WD1770_STAT_BUSY 0x01
#define WD1770_STAT_INDEX 0x02
#define WD1770_STAT_DRQ 0x02
#define WD1770_STAT_NOT_TR00 0x04
#define WD1770_STAT_LOST 0x04
#define WD1770_STAT_CRC_ERROR 0x08
#define WD1770_STAT_NOT_FOUND 0x10
#define WD1770_STAT_SPUN_UP 0x20
#define WD1770_STAT_DELETED_DATA 0x20
#define WD1770_STAT_PROTECTED 0x40
#define WD1770_STAT_MOTOR_ON 0x80

/** Drive number to use when no drive is selected */
#define WD1770_NO_DRIVE -1

/** Assumed maximum track number
 *
 * This is a property of the physical drive, rather than the
 * controller.  It is independent of the maximum track present on the
 * disk, and so should not be deduced from the size of the disk image
 * file (if any).  We choose a number that represents a common maximum
 * track number for physical drives.
 */
#define WD1770_MAX_TRACK 83

/** Motor switch-off delay in milliseconds
 *
 * The datasheet specifies 9 revolutions at 300rpm => 1800ms
 */
#define WD1770_MOTOR_OFF_DELAY_MS 1800

extern void wd1770_reset ( WD1770FDC *fdc );
extern void wd1770_set_drive ( WD1770FDC *fdc, int drive );
extern void wd1770_set_side ( WD1770FDC *fdc, unsigned int side );
extern void wd1770_set_single_density ( WD1770FDC *fdc, bool single_density );
extern WD1770FDC * wd1770_init ( hwaddr addr, qemu_irq drq, qemu_irq intrq,
				 DriveInfo **fds );

#endif
