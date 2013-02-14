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

/** Number of raw bytes in a single-density track (approximate)
 *
 * Derived from the "recommended single-density format with 128
 * bytes/sector" example on the datasheet.
 */
#define WD1770_TRACK_SIZE_SINGLE 3113

/** Number of raw bytes in a double-density track (approximate)
 *
 * Derived from the "recommended double-density format with 256
 * bytes/sector" example on the datasheet.
 */
#define WD1770_TRACK_SIZE_DOUBLE 6168

/** Size of data buffer
 *
 * The highest capacity requirement is for a write track operation on
 * a double density drive.
 */
#define WD1770_BUF_SIZE WD1770_TRACK_SIZE_DOUBLE

/** ID address marker byte */
#define WD1770_ID_ADDRESS_MARK 0xfe

/** Data address marker byte */
#define WD1770_DATA_ADDRESS_MARK 0xfb

/** ID address mark */
typedef struct {
	/** Track number */
	uint8_t track;
	/** Side number */
	uint8_t side;
	/** Sector number */
	uint8_t sector;
	/** Sector size */
	uint8_t sector_size_128_log2;
} QEMU_PACKED WD1770IdAddressMark;

/**
 * Calculate sector size from ID address mark
 *
 * @v id		ID address mark
 * @ret sector_size	Sector size
 */
static inline unsigned int wd1770_sector_size ( WD1770IdAddressMark *id ) {
	return ( 128 << id->sector_size_128_log2 );
}

/** Command delay in nanoseconds
 *
 * Some programs (e.g. disk formatters) don't cope properly if the
 * virtual hardware runs with essentially zero seek times.  Slow down
 * operations by delaying the start of each command.
 */
#define WD1770_CMD_DELAY_NS 1000

/** Data request delay in nanoseconds
 *
 * Some programs (e.g. disk formatters) don't cope properly if the
 * virtual hardware runs with essentially zero time between each data
 * byte.  Slow down some (not all) operations by delaying each
 * individual data request.
 */
#define WD1770_DRQ_DELAY_NS 1

/** Motor switch-off delay in milliseconds
 *
 * The datasheet specifies 9 revolutions at 300rpm => 1800ms
 */
#define WD1770_MOTOR_OFF_DELAY_MS 1800

/** 1770 FDC attached drive */
typedef struct {
	/** Controller */
	WD1770FDC *fdc;
	/** Block device */
	BlockDriverState *block;
	/** Density of media */
	bool double_density;
	/** Number of sides on media */
	uint8_t sides;
	/** Number of tracks on media */
	uint8_t tracks;
	/** Number of sectors per track on media */
	uint8_t sectors;
	/** Sector size of media */
	uint16_t sector_size;

	/** Current track position (may exceed limit of block device) */
	uint8_t track;
} WD1770FDD;

/** 1770 FDC operations */
typedef struct {
	/** Guess geometry */
	int ( * guess_geometry ) ( void *opaque, WD1770FDD *fdd );
} WD1770Ops;

/** 1770 FDC */
struct WD1770FDC {
	/** Name */
	const char *name;
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
	/** Command timer */
	QEMUTimer *command_timer;
	/** Data request timer */
	QEMUTimer *drq_timer;
	/** Motor switch-off timer */
	QEMUTimer *motor_off_timer;
	/** Operations */
	WD1770Ops *ops;
	/** Opaque pointer for operations */
	void *opaque;

	/** Selected drive (negative for no drive) */
	int8_t drive;
	/** Selected side */
	uint8_t side;
	/** Selected density */
	bool double_density;

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
	/** Interrupt is forced on */
	bool forced_intrq;
	/** Offset within current operation */
	uint32_t offset;
	/** Data bytes remaining within current operation */
	uint32_t remaining;
};

/** 1770 FDC system bus device */
typedef struct {
	/** System bus device */
	SysBusDevice busdev;
	/** 1770 FDC */
	WD1770FDC fdc;
} WD1770FDCSysBus;

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
#define WD1770_CMD_MULTIPLE 0x10
#define WD1770_CMD_WRITE 0x20
#define WD1770_CMD_FORCE_INTRQ 0x08

/* Status register */
#define WD1770_STAT_BUSY 0x01
#define WD1770_STAT_DRQ 0x02
#define WD1770_STAT_TR00 0x04
#define WD1770_STAT_LOST 0x04
#define WD1770_STAT_CRC_ERROR 0x08
#define WD1770_STAT_NOT_FOUND 0x10
#define WD1770_STAT_SPUN_UP 0x20
#define WD1770_STAT_DELETED_DATA 0x20
#define WD1770_STAT_PROTECTED 0x40
#define WD1770_STAT_MOTOR_ON 0x80

/** WD1770 type name */
#define TYPE_WD1770 "wd1770"

extern void wd1770_reset ( WD1770FDC *fdc );
extern void wd1770_set_drive ( WD1770FDC *fdc, int drive );
extern void wd1770_set_side ( WD1770FDC *fdc, unsigned int side );
extern void wd1770_set_double_density ( WD1770FDC *fdc, bool double_density );
extern void wd1770_set_ops ( WD1770FDC *fdc, WD1770Ops *ops, void *opaque );
extern WD1770FDC * wd1770_create ( MemoryRegion *mr, hwaddr offset,
				   const char *name, qemu_irq drq,
				   qemu_irq intrq, BlockDriverState **block );
extern WD1770FDC * wd1770_sysbus_create ( hwaddr addr, qemu_irq drq,
					  qemu_irq intrq,
					  BlockDriverState **block );

#endif
