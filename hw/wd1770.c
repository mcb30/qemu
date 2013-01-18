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
#define LOG_WD1770( ... ) do {						\
		if ( DEBUG_WD1770 ) {					\
			qemu_log_mask ( CPU_LOG_IOPORT, __VA_ARGS__ );	\
		}							\
	} while ( 0 )

/** A WD1770 command handler */
typedef void ( * wd1770_command_handler ) ( WD1770FDC *fdc );

/**
 * Get device name for log messages
 *
 * @v fdc		1770 FDC
 * @ret name		Device name
 */
static inline const char * wd1770_name ( WD1770FDC *fdc ) {

	return qdev_fw_name ( &fdc->busdev.qdev );
}

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
 * Calculate logical block address
 *
 * @v fdc		1770 FDC
 * @v sector		Sector number
 * @v block_sector	Logical block address, or negative on error
 *
 * Check to see if the (track,sector) tuple can be found on the
 * current track.  If so, return the logical block address.
 */
static int wd1770_lba ( WD1770FDC *fdc, uint8_t sector ) {
	WD1770FDD *fdd = wd1770_fdd ( fdc );
	int lba;

	/* Check if drive exists */
	if ( ! fdd ) {
		LOG_WD1770 ( "%s: track %d.%d sector %d not found: no drive "
			     "selected\n", wd1770_name ( fdc ), fdc->track,
			     fdc->side, sector );
		return -1;
	}

	/* Check if motor is on.  If the motor is off, no sectors will
	 * be detected.
	 */
	if ( ! ( fdc->status & WD1770_STAT_MOTOR_ON ) ) {
		LOG_WD1770 ( "%s: track %d.%d sector %d not found: motor "
			     "off\n", wd1770_name ( fdc ), fdc->track,
			     fdc->side, sector );
		return -1;
	}

	/* Check if density is correct for the media.  If the density
	 * is incorrect, then the wrong decoding will be applied and
	 * the (track,sector) tuple will not be detected.
	 */
	if ( fdd->single_density != fdc->single_density ) {
		LOG_WD1770 ( "%s: track %d.%d sector %d not found: wrong "
			     "density\n", wd1770_name ( fdc ), fdc->track,
			     fdc->side, sector );
		return -1;
	}

	/* Check if track register matches physical track.  If the
	 * register does not match, then the (track,sector) tuple will
	 * not be detected.
	 */
	if ( fdd->track != fdc->track ) {
		LOG_WD1770 ( "%s: track %d.%d sector %d not found: mismatch "
			     "(physical track %d)\n", wd1770_name ( fdc ),
			     fdc->track, fdc->side, sector, fdd->track );
		return -1;
	}

	/* Check if side is present on media */
	if ( fdc->side >= fdd->sides ) {
		LOG_WD1770 ( "%s: track %d.%d sector %d not found: maximum "
			     "side is %d\n", wd1770_name ( fdc ), fdc->track,
			     fdc->side, sector, ( fdd->sides - 1 ) );
		return -1;
	}

	/* Check if track is present on media */
	if ( fdd->track >= fdd->tracks ) {
		LOG_WD1770 ( "%s: track %d.%d sector %d not found: maximum "
			     "track is %d\n", wd1770_name ( fdc ), fdc->track,
			     fdc->side, sector, ( fdd->tracks - 1 ) );
		return -1;
	}

	/* Check if sector is present on media */
	if ( sector >= fdd->sectors ) {
		LOG_WD1770 ( "%s: track %d.%d sector %d not found: maximum "
			     "sector is %d\n", wd1770_name ( fdc ), fdc->track,
			     fdc->side, sector, ( fdd->sectors - 1 ) );
		return -1;
	}

	/* Calculate logical block address */
	lba = ( ( fdd->sectors * ( fdc->track * fdd->sides + fdc->side ) )
		+ sector );
	LOG_WD1770 ( "%s: track %d.%d sector %d is LBA %d\n",
		     wd1770_name ( fdc ), fdc->track, fdc->side, sector, lba );

	return lba;
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

	LOG_WD1770 ( "%s: reset\n", wd1770_name ( fdc ) );

	/* Reset registers */
	fdc->command = 0;
	fdc->status = 0;
	fdc->track = 0;
	fdc->sector = 0;
	fdc->data = 0;
	fdc->step = +1;
}

/**
 * Turn off motor
 *
 * @v opaque		1770 FDC
 */
static void wd1770_motor_off ( void *opaque ) {
	WD1770FDC *fdc = opaque;

	/* Turn off motor */
	fdc->status &= ~WD1770_STAT_MOTOR_ON;
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
	LOG_WD1770 ( "%s: drive=%d (%s)\n", wd1770_name ( fdc ), drive,
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

	LOG_WD1770 ( "%s: side=%d\n", wd1770_name ( fdc ), side );

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

	LOG_WD1770 ( "%s: density=%s\n", wd1770_name ( fdc ),
		     ( single_density ? "single" : "double" ) );

	/* Record side */
	fdc->single_density = single_density;
}

/**
 * Complete operation
 *
 * @v fdc		1770 FDC
 * @v intrq		Generate completion interrupt
 */
static void wd1770_done ( WD1770FDC *fdc, bool intrq ) {
	int64_t motor_off_time_ms;

	/* Clear busy and DRQ flags, and deassert data interrupt */
	fdc->status &= ~( WD1770_STAT_BUSY | WD1770_STAT_DRQ );
	qemu_irq_lower ( fdc->drq );

	/* Start timer to switch off motor (if currently on) */
	if ( fdc->status & WD1770_STAT_MOTOR_ON ) {
		motor_off_time_ms = ( qemu_get_clock_ms ( vm_clock ) +
				      WD1770_MOTOR_OFF_DELAY_MS );
		qemu_mod_timer ( fdc->motor_off, motor_off_time_ms );
	}

	/* Assert completion interrupt, if applicable */
	if ( intrq )
		qemu_irq_raise ( fdc->intrq );
}

/**
 * Seek to a new track
 *
 * @v fdc		1770 FDC
 * @v track_phys_delta	Amount by which to move the physical drive head
 * @v track_reg_delta	Amount by which to change the track register
 */
static void wd1770_seek ( WD1770FDC *fdc, int track_phys_delta,
			  int track_reg_delta ) {
	WD1770FDD *fdd = wd1770_fdd ( fdc );
	int new_track_phys;

	/* Switch on motor if instructed to do so */
	if ( ! ( fdc->command & WD1770_CMD_DISABLE_SPIN_UP ) )
		fdc->status |= ( WD1770_STAT_MOTOR_ON | WD1770_STAT_SPUN_UP );

	/* Calculate new physical track, limited to maximum track
	 * supported by the drive (not the disk).
	 */
	if ( fdd != NULL ) {
		new_track_phys = ( fdd->track + track_phys_delta );
		if ( new_track_phys < 0 )
			new_track_phys = 0;
		if ( new_track_phys > WD1770_MAX_TRACK )
			new_track_phys = WD1770_MAX_TRACK;
		fdd->track = new_track_phys;
	}

	/* Calculate new track register content */
	fdc->track += track_reg_delta;
	LOG_WD1770 ( "%s: now on track %d.%d (register %d)%s\n",
		     wd1770_name ( fdc ), ( fdd ? fdd->track : -1 ),
		     fdc->side, fdc->track,
		     ( fdd ? ( ( fdd->track == fdc->track ) ?
			       "" : " mismatch" ) : " no drive" ) );

	/* Indicate whether or not physical head is on track 0 */
	if ( ( fdd == NULL ) || ( fdd->track != 0 ) )
		fdc->status |= WD1770_STAT_NOT_TR00;

	/* Verify track presence if asked to do so */
	if ( ( fdc->command & WD1770_CMD_VERIFY ) &&
	     ( wd1770_lba ( fdc, 0 ) < 0 ) ) {
		fdc->status |= WD1770_STAT_NOT_FOUND;
		wd1770_done ( fdc, true );
		return;
	}

	/* Succeed */
	wd1770_done ( fdc, true );
}

/**
 * Start reading from current sector
 *
 * @v fdc		1770 FDC
 */
static void wd1770_read_sector ( WD1770FDC *fdc ) {
	WD1770FDD *fdd = wd1770_fdd ( fdc );
	int lba;
	unsigned int shift;
	int64_t block_sector;
	int block_count;
	uint16_t offset;

	/* Switch on motor if instructed to do so */
	if ( ! ( fdc->command & WD1770_CMD_DISABLE_SPIN_UP ) )
		fdc->status |= WD1770_STAT_MOTOR_ON;

	/* Check that sector can be found, and get LBA */
	lba = wd1770_lba ( fdc, fdc->sector );
	if ( lba < 0 ) {
		fdc->status |= WD1770_STAT_NOT_FOUND;
		wd1770_done ( fdc, true );
		return;
	}

	/* Calculate block sector address, count, and offset */
	if ( fdd->sector_size_log2 >= BDRV_SECTOR_BITS ) {
		/* Floppy sector contains one or more block sectors */
		shift = ( fdd->sector_size_log2 - BDRV_SECTOR_BITS );
		block_sector = ( lba << shift );
		block_count = ( 1 << shift );
		offset = 0;
	} else {
		/* Floppy sector is less than one block sector */
		shift = ( BDRV_SECTOR_BITS - fdd->sector_size_log2 );
		block_sector = ( lba >> shift );
		block_count = 1;
		offset = ( ( lba << fdd->sector_size_log2 ) &
			   ~BDRV_SECTOR_MASK );
	}
	LOG_WD1770 ( "%s: LBA %d is block sector %ld-%ld offset %d\n",
		     wd1770_name ( fdc ), lba, block_sector,
		     ( block_sector + block_count - 1 ), offset );

	/* Read data into sector buffer */
	if ( bdrv_read ( fdd->block, block_sector, fdc->buf,
			 block_count ) < 0 ) {
		LOG_WD1770 ( "%s: could not read from %s\n",
			     wd1770_name ( fdc ),
			     bdrv_get_device_name ( fdd->block ) );
		fdc->status |= WD1770_STAT_NOT_FOUND;
		wd1770_done ( fdc, true );
		return;
	}

	/* Reset offset and place first byte in data register */
	fdc->offset = offset;
	fdc->remaining = ( 1 << fdd->sector_size_log2 );
	fdc->data = fdc->buf[offset];

	/* Assert data interrupt */
	fdc->status |= WD1770_STAT_DRQ;
	qemu_irq_raise ( fdc->drq );
}

/**
 * Read next byte
 *
 * @v fdc		1770 FDC
 */
static void wd1770_read_next ( WD1770FDC *fdc ) {

	/* Deassert data interrupt */
	fdc->status &= ~WD1770_STAT_DRQ;
	qemu_irq_lower ( fdc->drq );

	/* Increment offset */
	fdc->offset++;
	fdc->remaining--;

	/* If we have not yet reached the end of the sector, place the
	 * next byte into the data register and return, reasserting
	 * the data interrupt.
	 */
	if ( fdc->remaining ) {
		fdc->data = fdc->buf[fdc->offset];
		fdc->status |= WD1770_STAT_DRQ;
		qemu_irq_raise ( fdc->drq );
		return;
	}

	/* If we are reading multiple sectors, then increment the
	 * sector register and start reading the next sector.
	 */
	if ( fdc->command & WD1770_CMD_MULTIPLE ) {
		fdc->sector++;
		wd1770_read_sector ( fdc );
		return;
	}

	/* Otherwise, mark the operation as complete */
	wd1770_done ( fdc, true );
}

/**
 * Restore head to track zero
 *
 * @v fdc		1770 FDC
 */
static void wd1770_command_restore ( WD1770FDC *fdc ) {

	LOG_WD1770 ( "%s: (0x%02x) restore to track 0\n", wd1770_name ( fdc ),
		     fdc->command );

	/* Seek to track zero */
	wd1770_seek ( fdc, -WD1770_MAX_TRACK, -(fdc->track) );
}

/**
 * Seek to specified track
 *
 * @v fdc		1770 FDC
 */
static void wd1770_command_seek ( WD1770FDC *fdc ) {
	int delta;

	LOG_WD1770 ( "%s: (0x%02x) seek from track %d.%d to track %d.%d\n",
		     wd1770_name ( fdc ), fdc->command, fdc->track, fdc->side,
		     fdc->data, fdc->side );

	/* Seek to specified track number */
	delta = ( fdc->data - fdc->track );
	wd1770_seek ( fdc, delta, delta );
}

/**
 * Step one track in same direction as previous step
 *
 * @v fdc		1770 FDC
 */
static void wd1770_command_step ( WD1770FDC *fdc ) {

	LOG_WD1770 ( "%s: (0x%02x) step\n", wd1770_name ( fdc ), fdc->command );

	/* Seek in same direction as previous step */
	wd1770_seek ( fdc, fdc->step,
		      ( ( fdc->command & WD1770_CMD_UPDATE_TRACK ) ?
			fdc->step : 0 ) );
}

/**
 * Step one track inwards
 *
 * @v fdc		1770 FDC
 */
static void wd1770_command_step_in ( WD1770FDC *fdc ) {

	LOG_WD1770 ( "%s: (0x%02x) step inwards\n", wd1770_name ( fdc ),
		     fdc->command );

	/* Seek in direction of increasing track number */
	fdc->step = +1;
	wd1770_seek ( fdc, fdc->step,
		      ( ( fdc->command & WD1770_CMD_UPDATE_TRACK ) ?
			fdc->step : 0 ) );
}

/**
 * Step one track outwards
 *
 * @v fdc		1770 FDC
 */
static void wd1770_command_step_out ( WD1770FDC *fdc ) {

	LOG_WD1770 ( "%s: (0x%02x), step outwards\n", wd1770_name ( fdc ),
		     fdc->command );

	/* Seek in direction of decreasing track number */
	fdc->step = -1;
	wd1770_seek ( fdc, fdc->step,
		      ( ( fdc->command & WD1770_CMD_UPDATE_TRACK ) ?
			fdc->step : 0 ) );
}

/**
 * Read sector(s)
 *
 * @v fdc		1770 FDC
 */
static void wd1770_command_read_sector ( WD1770FDC *fdc ) {

	LOG_WD1770 ( "%s: (0x%02x) read\n", wd1770_name ( fdc ), fdc->command );

	/* Start reading current sector */
	wd1770_read_sector ( fdc );
}

/**
 * Write sector(s)
 *
 * @v fdc		1770 FDC
 */
static void wd1770_command_write_sector ( WD1770FDC *fdc ) {

	LOG_WD1770 ( "%s: (0x%02x) write\n", wd1770_name ( fdc ),
		     fdc->command );
}

/**
 * Read address
 *
 * @v fdc		1770 FDC
 */
static void wd1770_command_read_address ( WD1770FDC *fdc ) {

	qemu_log_mask ( LOG_UNIMP, "%s: (0x%02x) unimplemented read address\n",
			wd1770_name ( fdc ), fdc->command );

	/* Fake a "lost data" error */
	fdc->status |= WD1770_STAT_LOST;
	wd1770_done ( fdc, true );
}

/**
 * Read track
 *
 * @v fdc		1770 FDC
 */
static void wd1770_command_read_track ( WD1770FDC *fdc ) {

	qemu_log_mask ( LOG_UNIMP, "%s: (0x%02x) unimplemented read track\n",
			wd1770_name ( fdc ), fdc->command );

	/* Fake a "lost data" error */
	fdc->status |= WD1770_STAT_LOST;
	wd1770_done ( fdc, true );
}

/**
 * Write track
 *
 * @v fdc		1770 FDC
 */
static void wd1770_command_write_track ( WD1770FDC *fdc ) {

	qemu_log_mask ( LOG_UNIMP, "%s: (0x%02x) unimplemented write track\n",
			wd1770_name ( fdc ), fdc->command );
	
	/* Fake a "lost data" error */
	fdc->status |= WD1770_STAT_LOST;
	wd1770_done ( fdc, true );
}

/**
 * Force interrupt
 *
 * @v fdc		1770 FDC
 */
static void wd1770_command_force_interrupt ( WD1770FDC *fdc ) {

	LOG_WD1770 ( "%s: (0x%02x) force interrupt\n", wd1770_name ( fdc ),
		     fdc->command );

	/* Terminate current command */
	wd1770_done ( fdc, false );

	/* Force interrupts as specified.  We don't emulate the "every
	 * index pulse" interrupt.
	 */
	fdc->forced_intrq = ( !! ( fdc->command & WD1770_CMD_FORCE_INTRQ ) );
	if ( fdc->forced_intrq )
		qemu_irq_raise ( fdc->intrq );
}

/**
 * Handle issued command
 *
 * @v fdc		1770 FDC
 * @v command		Command
 */
static void wd1770_command_write ( WD1770FDC *fdc, uint8_t command ) {
	static const wd1770_command_handler handlers[16] = {
		wd1770_command_restore,		wd1770_command_seek,
		wd1770_command_step,		wd1770_command_step,
		wd1770_command_step_in,		wd1770_command_step_in,
		wd1770_command_step_out,	wd1770_command_step_out,
		wd1770_command_read_sector,	wd1770_command_read_sector,
		wd1770_command_write_sector,	wd1770_command_write_sector,
		wd1770_command_read_address,	wd1770_command_force_interrupt,
		wd1770_command_read_track,	wd1770_command_write_track,
	};
	wd1770_command_handler handler = handlers[ command >> 4 ];

	/* Forced interrupts are a special case */
	if ( handler == wd1770_command_force_interrupt ) {
		fdc->command = command;
		wd1770_command_force_interrupt ( fdc );
		return;
	}

	/* If controller is busy, then ignore the command */
	if ( fdc->status & WD1770_STAT_BUSY ) {
		LOG_WD1770 ( "%s: (0x%02x) command ignored while busy (%d "
			     "bytes remaining)\n", wd1770_name ( fdc ),
			     command, fdc->remaining );
		return;
	}

	/* Record command */
	fdc->command = command;

	/* Clear all bits in status register apart from motor status */
	fdc->status &= WD1770_STAT_MOTOR_ON;

	/* Clear completion interrupt */
	if ( ! fdc->forced_intrq )
		qemu_irq_lower ( fdc->intrq );

	/* Mark controller as busy */
	fdc->status |= WD1770_STAT_BUSY;

	/* Hand off to individual command handler */
	handler ( fdc );
}

/**
 * Read from 1770 FDC status register
 *
 * @v fdc		1770 FDC
 * @ret data		Data
 */
static uint8_t wd1770_status_read ( WD1770FDC *fdc ) {
	uint8_t data;

	/* Read status register */
	data = fdc->status;

	/* Clear completion interrupt */
	if ( ! fdc->forced_intrq )
		qemu_irq_lower ( fdc->intrq );

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

	/* Write sector register */
	fdc->sector = data;
}

/**
 * Read from 1770 FDC data register
 *
 * @v fdc		1770 FDC
 * @ret data		Data
 */
static uint8_t wd1770_data_read ( WD1770FDC *fdc ) {
	uint8_t data;

	/* Read data register */
	data = fdc->data;

	/* If a read operation is in progress, get the next data byte
	 * into the data register.
	 */
	if ( fdc->remaining && ! ( fdc->command & WD1770_CMD_WRITE ) )
		wd1770_read_next ( fdc );

	return data;
}

/**
 * Write to 1770 FDC data register
 *
 * @v fdc		1770 FDC
 * @v data		Data
 */
static void wd1770_data_write ( WD1770FDC *fdc, uint8_t data ) {

	/* Write data register */
	fdc->data = data;
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
	case WD1770_DATA:
		data = wd1770_data_read ( fdc );
		break;
	default:
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented read from "
				"0x%02lx\n", wd1770_name ( fdc ), addr );
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
	case WD1770_DATA:
		wd1770_data_write ( fdc, data );
		break;
	default:
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented write 0x%02x to "
				"0x%02lx\n", wd1770_name ( fdc ), data, addr );
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
	fdc->buf = qemu_memalign ( BDRV_SECTOR_SIZE, WD1770_MAX_SECTOR_SIZE );
	fdc->motor_off = qemu_new_timer_ms ( vm_clock, wd1770_motor_off, fdc );

	/* Initialise drives */
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

	/* Reset device */
	wd1770_reset ( fdc );

	//
	// HACK
	//

#if 0
	// ADFS / DDFS
	fdc->fdds[0].single_density = false;
	fdc->fdds[0].sides = 2;
	fdc->fdds[0].tracks = 80;
	fdc->fdds[0].sectors = 16;
	fdc->fdds[0].sector_size_log2 = 8;
#endif
#if 1
	// DFS single-sided
	fdc->fdds[0].single_density = true;
	fdc->fdds[0].sides = 1;
	fdc->fdds[0].tracks = 80;
	fdc->fdds[0].sectors = 10;
	fdc->fdds[0].sector_size_log2 = 8;
#endif


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
		VMSTATE_UINT8 ( command, WD1770FDC ),
		VMSTATE_UINT8 ( status, WD1770FDC ),
		VMSTATE_UINT8 ( track, WD1770FDC ),
		VMSTATE_UINT8 ( sector, WD1770FDC ),
		VMSTATE_UINT8 ( data, WD1770FDC ),
		VMSTATE_INT8 ( step, WD1770FDC ),
		VMSTATE_BOOL ( forced_intrq, WD1770FDC ),
		VMSTATE_UINT16 ( offset, WD1770FDC ),
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
