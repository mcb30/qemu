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
typedef void ( * WD1770CommandHandler ) ( WD1770FDC *fdc );

/** Identify command handler */
static WD1770CommandHandler wd1770_command_handler ( uint8_t command );

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
 * Issue command
 *
 * @v fdc		1770 FDC
 */
static void wd1770_command ( WD1770FDC *fdc ) {
	WD1770CommandHandler handler;

	/* Identify command handler */
	handler = wd1770_command_handler ( fdc->command );

	/* Hand off to individual command handler */
	handler ( fdc );
}

/**
 * Start command timer to issue command
 *
 * @v fdc		1770 FDC
 */
static void wd1770_command_delayed ( WD1770FDC *fdc ) {
	int64_t command_time_ns;

	/* Start timer to issue command */
	command_time_ns = ( qemu_get_clock_ns ( vm_clock ) +
			    WD1770_CMD_DELAY_NS );
	qemu_mod_timer ( fdc->command_timer, command_time_ns );
}

/**
 * Issue command upon command timer expiry
 *
 * @v opaque		1770 FDC
 */
static void wd1770_command_expired ( void *opaque ) {
	WD1770FDC *fdc = opaque;

	/* Issue command */
	wd1770_command ( fdc );
}

/**
 * Request next data byte
 *
 * @v fdc		1770 FDC
 */
static inline void wd1770_drq ( WD1770FDC *fdc ) {

	/* Assert data interrupt */
	fdc->status |= WD1770_STAT_DRQ;
	qemu_irq_raise ( fdc->drq );	
}

/**
 * Start data request timer for next data byte
 *
 * @v fdc		1770 FDC
 */
static void wd1770_drq_delayed ( WD1770FDC *fdc ) {
	int64_t drq_time_ns;

	/* Start timer to request next data byte */
	drq_time_ns = ( qemu_get_clock_ns ( vm_clock ) + WD1770_DRQ_DELAY_NS );
	qemu_mod_timer ( fdc->drq_timer, drq_time_ns );
}

/**
 * Request next data byte upon data request timer expiry
 *
 * @v opaque		1770 FDC
 */
static void wd1770_drq_expired ( void *opaque ) {
	WD1770FDC *fdc = opaque;

	/* Assert data interrupt */
	wd1770_drq ( fdc );
}

/**
 * Turn on motor
 *
 * @v fdc		1770 FDC
 */
static void wd1770_motor_on ( WD1770FDC *fdc ) {

	/* Cancel motor-off timer, if running */
	qemu_del_timer ( fdc->motor_off_timer );

	/* Mark motor as on */
	fdc->status |= WD1770_STAT_MOTOR_ON;
}

/**
 * Turn off motor
 *
 * @v fdc		1770 FDC
 */
static void wd1770_motor_off ( WD1770FDC *fdc ) {

	/* Mark motor as off */
	fdc->status &= ~WD1770_STAT_MOTOR_ON;
}

/**
 * Start timer to turn off motor
 *
 * @v fdc		1770 FDC
 */
static void wd1770_motor_off_delayed ( WD1770FDC *fdc ) {
	int64_t motor_off_time_ms;

	/* Start timer to switch off motor (if currently on) */
	if ( fdc->status & WD1770_STAT_MOTOR_ON ) {
		motor_off_time_ms = ( qemu_get_clock_ms ( vm_clock ) +
				      WD1770_MOTOR_OFF_DELAY_MS );
		qemu_mod_timer ( fdc->motor_off_timer, motor_off_time_ms );
	}
}

/**
 * Turn off motor upon motor timer expiry
 *
 * @v opaque		1770 FDC
 */
static void wd1770_motor_off_expired ( void *opaque ) {
	WD1770FDC *fdc = opaque;

	/* Turn off motor */
	wd1770_motor_off ( fdc );
}

/**
 * Complete command
 *
 * @v fdc		1770 FDC
 * @v intrq		Generate completion interrupt
 */
static void wd1770_done ( WD1770FDC *fdc, bool intrq ) {

	/* Stop DRQ timer, clear busy and DRQ flags, and deassert data
	 * interrupt.
	 */
	qemu_del_timer ( fdc->drq_timer );
	fdc->status &= ~( WD1770_STAT_BUSY | WD1770_STAT_DRQ );
	qemu_irq_lower ( fdc->drq );

	/* Stop command timer to cancel any pending command, and mark
	 * any ongoing command as complete.
	 */
	qemu_del_timer ( fdc->command_timer );
	fdc->remaining = 0;

	/* Start timer to switch off motor (if currently on) */
	wd1770_motor_off_delayed ( fdc );

	/* Assert completion interrupt, if applicable */
	if ( intrq )
		qemu_irq_raise ( fdc->intrq );
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

	/* Cancel any ongoing command */
	wd1770_done ( fdc, false );

	/* Reset registers (excluding those provided by external sources) */
	fdc->command = 0;
	fdc->status = 0;
	fdc->track = 0;
	fdc->sector = 0;
	fdc->data = 0;
	fdc->step = +1;
	fdc->forced_intrq = false;
	fdc->offset = 0;
	fdc->remaining = 0;

	/* Deassert interrupts */
	qemu_irq_lower ( fdc->drq );
	qemu_irq_lower ( fdc->intrq );
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
 * Calculate offset within block device
 *
 * @v fdc		1770 FDC
 * @v sector		Sector number
 * @ret offset		Offset, or negative on error
 *
 * Check to see if the (track,sector) tuple can be found on the
 * current track of the currently-selected drive.  If so, return the
 * byte offset to the start of the sector.
 */
static int64_t wd1770_offset ( WD1770FDC *fdc, uint8_t sector ) {
	WD1770FDD *fdd;
	unsigned int lba;
	int64_t offset;

	/* Check if drive exists */
	fdd = wd1770_fdd ( fdc );
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

	/* Calculate logical block address, if applicable */
	lba = ( ( fdd->sectors * ( fdc->track * fdd->sides + fdc->side ) )
		+ sector );
	offset = ( lba * fdd->sector_size );
	LOG_WD1770 ( "%s: track %d.%d sector %d is LBA %d offset %" PRId64 "\n",
		     wd1770_name ( fdc ), fdc->track, fdc->side, sector,
		     lba, offset );

	return offset;
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
	WD1770FDD *fdd = wd1770_fdd ( fdc );

	/* Record drive */
	fdc->drive = drive;
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
	if ( ! ( fdc->command & WD1770_CMD_DISABLE_SPIN_UP ) ) {
		wd1770_motor_on ( fdc );
		fdc->status |= WD1770_STAT_SPUN_UP;
	}

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
	LOG_WD1770 ( "%s: now on track %d.%d (physical track %d)%s\n",
		     wd1770_name ( fdc ), fdc->track, fdc->side,
		     ( fdd ? fdd->track : -1 ),
		     ( fdd ? ( ( fdd->track == fdc->track ) ?
			       "" : " mismatch" ) : " no drive" ) );

	/* Indicate whether or not physical head is on track 0 */
	if ( ( fdd != NULL ) && ( fdd->track == 0 ) )
		fdc->status |= WD1770_STAT_TR00;

	/* Verify track presence if asked to do so */
	if ( ( fdc->command & WD1770_CMD_VERIFY ) &&
	     ( wd1770_offset ( fdc, 0 ) < 0 ) ) {
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
	WD1770FDD *fdd;
	int64_t offset;

	/* Switch on motor.  Do this unconditionally, ignoring the
	 * WD1770_CMD_DISABLE_SPIN_UP bit since this bit has a
	 * different meaning on WD1773.
	 */
	wd1770_motor_on ( fdc );

	/* Check that sector can be found, and calculate the block
	 * device offset.
	 */
	offset = wd1770_offset ( fdc, fdc->sector );
	if ( offset < 0 ) {
		fdc->status |= WD1770_STAT_NOT_FOUND;
		wd1770_done ( fdc, true );
		return;
	}

	/* At this point, we know that we have a block device since
	 * the offset calculation succeeded.
	 */
	fdd = wd1770_fdd ( fdc );
	assert ( fdd != NULL );

	/* Read data into data buffer */
	if ( bdrv_pread ( fdd->block, offset, fdc->buf,
			  fdd->sector_size ) < 0 ) {
		LOG_WD1770 ( "%s: could not read from %s\n",
			     wd1770_name ( fdc ),
			     bdrv_get_device_name ( fdd->block ) );
		fdc->status |= WD1770_STAT_NOT_FOUND;
		wd1770_done ( fdc, true );
		return;
	}

	/* Start ongoing command */
	fdc->offset = 0;
	fdc->remaining = fdd->sector_size;

	/* Place first byte in data register and issue data request */
	fdc->data = fdc->buf[0];
	wd1770_drq ( fdc );
}

/**
 * Read next byte
 *
 * @v fdc		1770 FDC
 */
static void wd1770_read_sector_next ( WD1770FDC *fdc ) {

	/* Deassert data interrupt */
	fdc->status &= ~WD1770_STAT_DRQ;
	qemu_irq_lower ( fdc->drq );

	/* Increment offset */
	fdc->offset++;
	fdc->remaining--;

	/* If we have not yet reached the end of the sector, place the
	 * next byte into the data register, reissue data request, and
	 * return.
	 */
	if ( fdc->remaining ) {
		fdc->data = fdc->buf[fdc->offset];
		wd1770_drq ( fdc );
		return;
	}

	/* If we are reading multiple sectors, then increment the
	 * sector register and reissue command to start reading the
	 * next sector.
	 */
	if ( fdc->command & WD1770_CMD_MULTIPLE ) {
		fdc->sector++;
		wd1770_command_delayed ( fdc );
		return;
	}

	/* Otherwise, mark the command as complete */
	wd1770_done ( fdc, true );
}

/**
 * Start writing to current sector
 *
 * @v fdc		1770 FDC
 */
static void wd1770_write_sector ( WD1770FDC *fdc ) {
	WD1770FDD *fdd;

	/* Switch on motor.  Do this unconditionally, ignoring the
	 * WD1770_CMD_DISABLE_SPIN_UP bit since this bit has a
	 * different meaning on WD1773.
	 */
	wd1770_motor_on ( fdc );

	/* Fail if block device is read-only */
	fdd = wd1770_fdd ( fdc );
	if ( fdd && ( bdrv_is_read_only ( fdd->block ) ) ) {
		fdc->status |= WD1770_STAT_PROTECTED;
		wd1770_done ( fdc, true );
		return;
	}

	/* Check that sector can be found */
	if ( wd1770_offset ( fdc, fdc->sector ) < 0 ) {
		fdc->status |= WD1770_STAT_NOT_FOUND;
		wd1770_done ( fdc, true );
		return;
	}

	/* At this point, we know that we have a block device since
	 * the offset calculation succeeded.
	 */
	assert ( fdd != NULL );

	/* Start ongoing command */
	fdc->offset = 0;
	fdc->remaining = fdd->sector_size;

	/* Request first byte of data */
	wd1770_drq ( fdc );
}

/**
 * Write next byte
 *
 * @v fdc		1770 FDC
 */
static void wd1770_write_sector_next ( WD1770FDC *fdc ) {
	WD1770FDD *fdd;
	int64_t offset;

	/* Deassert data interrupt */
	fdc->status &= ~WD1770_STAT_DRQ;
	qemu_irq_lower ( fdc->drq );

	/* Read byte into data buffer and increment offset */
	fdc->buf[fdc->offset] = fdc->data;
	fdc->offset++;
	fdc->remaining--;

	/* If we have not yet reached the end of the sector, request
	 * the next byte.
	 */
	if ( fdc->remaining ) {
		wd1770_drq ( fdc );
		return;
	}

	/* Calculate the block device offset */
	offset = wd1770_offset ( fdc, fdc->sector );
	if ( offset < 0 ) {
		fdc->status |= WD1770_STAT_NOT_FOUND;
		wd1770_done ( fdc, true );
		return;
	}

	/* At this point, we know that we have a block device since
	 * the offset calculation succeeded.
	 */
	fdd = wd1770_fdd ( fdc );
	assert ( fdd != NULL );

	/* Write the completed sector to the block device */
	if ( bdrv_pwrite ( fdd->block, offset, fdc->buf,
			   fdd->sector_size ) < 0 ) {
		LOG_WD1770 ( "%s: could not write to %s\n",
			     wd1770_name ( fdc ),
			     bdrv_get_device_name ( fdd->block ) );
		fdc->status |= WD1770_STAT_NOT_FOUND;
		wd1770_done ( fdc, true );
		return;
	}

	/* If we are writing multiple sectors, then increment the
	 * sector register and reissue command to start writing the
	 * next sector.
	 */
	if ( fdc->command & WD1770_CMD_MULTIPLE ) {
		fdc->sector++;
		wd1770_command_delayed ( fdc );
		return;
	}

	/* Otherwise, mark the command as complete */
	wd1770_done ( fdc, true );
}

/**
 * Start writing (i.e. formatting) the current track
 *
 * @v fdc		1770 FDC
 */
static void wd1770_write_track ( WD1770FDC *fdc ) {
	WD1770FDD *fdd;

	/* Switch on motor.  Do this unconditionally, ignoring the
	 * WD1770_CMD_DISABLE_SPIN_UP bit since this bit has a
	 * different meaning on WD1773.
	 */
	wd1770_motor_on ( fdc );

	/* Fail if no media is present */
	fdd = wd1770_fdd ( fdc );
	if ( ! fdd ) {
		LOG_WD1770 ( "%s: drive %d has no media\n",
			     wd1770_name ( fdc ), fdc->drive );
		fdc->status |= WD1770_STAT_NOT_FOUND;
		wd1770_done ( fdc, true );
		return;
	}

	/* Fail if block device is read-only */
	if ( bdrv_is_read_only ( fdd->block ) ) {
		fdc->status |= WD1770_STAT_PROTECTED;
		wd1770_done ( fdc, true );
		return;
	}

	/* Start ongoing command */
	fdc->offset = 0;
	fdc->remaining = ( fdc->single_density ? WD1770_TRACK_SIZE_SINGLE :
			   WD1770_TRACK_SIZE_DOUBLE );

	/* Start data request timer for first byte of raw track data */
	wd1770_drq_delayed ( fdc );
}

/**
 * Decode raw track geometry
 *
 * @v fdc		1770 FDC
 * @v decode		Decode
 * @ret rc		Return code
 */
static int wd1770_decode_geometry ( WD1770FDC *fdc, WD1770IdAddressMark *id,
				    const uint8_t *data ) {
	WD1770FDD *fdd = wd1770_fdd ( fdc );
	unsigned int sector_size;

	/* This function can never be called without a valid drive */
	assert ( fdd != NULL );

	/* Update disk geometry */
	sector_size = wd1770_sector_size ( id );
	if ( fdd->sector_size < sector_size )
		fdd->sector_size = sector_size;
	if ( fdd->sectors <= id->sector )
		fdd->sectors = ( id->sector + 1 );
	LOG_WD1770 ( "%s: track %d.%d decoded track %d.%d sector %d length "
		     "%d\n", wd1770_name ( fdc ), fdc->track, fdc->side,
		     id->track, id->side, id->sector, sector_size );

	return 0;
}

/**
 * Decode raw track data
 *
 * @v fdc		1770 FDC
 * @v decode		Decode
 * @ret rc		Return code
 */
static int wd1770_decode_data ( WD1770FDC *fdc, WD1770IdAddressMark *id,
				const uint8_t *data ) {
	WD1770FDD *fdd = wd1770_fdd ( fdc );	
	unsigned int sector_size;
	int64_t offset;

	/* This function can never be called without a valid drive */
	assert ( fdd != NULL );

	/* Check that sector can be found, and calculate the block
	 * device offset,
	 */
	offset = wd1770_offset ( fdc, id->sector );
	if ( offset < 0 )
		return -1;

	/* Write data to block device */
	sector_size = wd1770_sector_size ( id );
	if ( bdrv_pwrite ( fdd->block, offset, data, sector_size ) < 0 ) {
		LOG_WD1770 ( "%s: could not write to %s\n", wd1770_name ( fdc ),
			     bdrv_get_device_name ( fdd->block ) );
		return -1;
	}

	return 0;
}

/**
 * Decode raw track
 *
 * @v fdc		1770 FDC
 * @v decode		Decode
 * @ret rc		Return code
 */
static int wd1770_decode_track ( WD1770FDC *fdc,
				 int ( * decode ) ( WD1770FDC *fdc,
						    WD1770IdAddressMark *id,
						    const uint8_t *data ) ) {
	unsigned int raw_offset = 0;
	int found_id = 0;
	WD1770IdAddressMark id;
	unsigned int remaining;
	unsigned int sector_size;
	uint8_t data;

	/* Scan raw data for sectors */
	while ( raw_offset < fdc->offset ) {

		/* Read raw data byte and calculate remaining length */
		data = fdc->buf[raw_offset++];
		remaining = ( fdc->offset - raw_offset );

		/* Look for address markers */
		switch ( data ) {

		case WD1770_ID_ADDRESS_MARK:
			/* Found an ID address mark: record the address */
			if ( remaining < sizeof ( id ) ) {
				LOG_WD1770 ( "%s: truncated ID address mark "
					     "at %d/%d\n", wd1770_name ( fdc ),
					     raw_offset, fdc->offset );
				break;
			}
			memcpy ( &id, &fdc->buf[raw_offset], sizeof ( id ) );
			raw_offset += sizeof ( id );
			found_id = 1;
			break;

		case WD1770_DATA_ADDRESS_MARK:
			/* Found a data address mark: process the data */
			if ( ! found_id ) {
				LOG_WD1770 ( "%s: ID-less data address mark "
					     "at %d/%d\n", wd1770_name ( fdc ),
					     raw_offset, fdc->offset );
				break;
			}
			found_id = 0;
			sector_size = wd1770_sector_size ( &id );
			if ( remaining < sector_size ) {
				LOG_WD1770 ( "%s: truncated data address mark "
					     "at %d/%d\n", wd1770_name ( fdc ),
					     raw_offset, fdc->offset );
				break;
			}
			if ( decode ( fdc, &id, &fdc->buf[raw_offset] ) < 0 )
				return -1;
			raw_offset += sector_size;
			break;

		default:
			/* Ignore this byte */
			break;
		}
	}

	return 0;
}

/**
 * Write next track byte
 *
 * @v fdc		1770 FDC
 */
static void wd1770_write_track_next ( WD1770FDC *fdc ) {
	WD1770FDD *fdd;
	int64_t fdd_size;

	/* Deassert data interrupt */
	fdc->status &= ~WD1770_STAT_DRQ;
	qemu_irq_lower ( fdc->drq );

	/* Read byte into data buffer and increment offset */
	fdc->buf[fdc->offset] = fdc->data;
	fdc->offset++;
	fdc->remaining--;

	/* If we have not yet reached the end of the track, restart
	 * the data request timer for the next byte.
	 */
	if ( fdc->remaining ) {
		wd1770_drq_delayed ( fdc );
		return;
	}

	/* Fail if no media is present (which should not happen here) */
	fdd = wd1770_fdd ( fdc );
	if ( ! fdd ) {
		LOG_WD1770 ( "%s: drive %d has no media\n",
			     wd1770_name ( fdc ), fdc->drive );
		fdc->status |= WD1770_STAT_NOT_FOUND;
		wd1770_done ( fdc, true );
		return;
	}

	/* Decode raw track data to determine new disk geometry */
	fdd->single_density = fdc->single_density;
	if ( fdc->side >= fdd->sides )
		fdd->sides = ( fdc->side + 1 );
	if ( fdd->track >= fdd->tracks )
		fdd->tracks = ( fdd->track + 1 );
	fdd->sectors = 0;
	fdd->sector_size = 0;
	if ( wd1770_decode_track ( fdc, wd1770_decode_geometry ) < 0 ) {
		LOG_WD1770 ( "%s: could not decode track geometry\n",
			     wd1770_name ( fdc ) );
		fdc->status |= WD1770_STAT_NOT_FOUND;
		wd1770_done ( fdc, true );
		return;
	}

	/* Resize block device to new geometry */
	fdd_size = ( fdd->sides * fdd->tracks * fdd->sectors *
		     fdd->sector_size );
	if ( bdrv_truncate ( fdd->block, fdd_size ) < 0 ) {
		LOG_WD1770 ( "%s: could not resize %s to %" PRId64 "\n",
			     wd1770_name ( fdc ),
			     bdrv_get_device_name ( fdd->block ), fdd_size );
		fdc->status |= WD1770_STAT_NOT_FOUND;
		wd1770_done ( fdc, true );
		return;
	}

	/* Decode raw track data to write sectors to the block device */
	if ( wd1770_decode_track ( fdc, wd1770_decode_data ) < 0 ) {
		LOG_WD1770 ( "%s: could not write track data\n",
			     wd1770_name ( fdc ) );
		fdc->status |= WD1770_STAT_NOT_FOUND;
		wd1770_done ( fdc, true );
		return;
	}

	/* Mark command as complete */
	wd1770_done ( fdc, true );
}

/**
 * Restore head to track zero
 *
 * @v fdc		1770 FDC
 */
static void wd1770_command_restore ( WD1770FDC *fdc ) {

	LOG_WD1770 ( "%s: command 0x%02x restore to track 0\n",
		     wd1770_name ( fdc ), fdc->command );

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

	LOG_WD1770 ( "%s: command 0x%02x seek from track %d.%d to track "
		     "%d.%d\n", wd1770_name ( fdc ), fdc->command, fdc->track,
		     fdc->side, fdc->data, fdc->side );

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

	LOG_WD1770 ( "%s: command 0x%02x step (%s) to track %d.%d\n",
		     wd1770_name ( fdc ), fdc->command,
		     ( ( fdc->step > 0 ) ? "inwards" : "outwards" ),
		     ( fdc->track + fdc->step ), fdc->side );

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

	LOG_WD1770 ( "%s: command 0x%02x step inwards to track %d.%d\n",
		     wd1770_name ( fdc ), fdc->command, ( fdc->track + 1 ),
		     fdc->side );

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

	LOG_WD1770 ( "%s: command 0x%02x step outwards to track %d.%d\n",
		     wd1770_name ( fdc ), fdc->command, ( fdc->track - 1 ),
		     fdc->side );

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

	LOG_WD1770 ( "%s: command 0x%02x read from track %d.%d sector %d%s\n",
		     wd1770_name ( fdc ), fdc->command, fdc->track,
		     fdc->side, fdc->sector,
		     ( ( fdc->command & WD1770_CMD_MULTIPLE ) ? "+" : "" ) );

	/* Start reading current sector */
	wd1770_read_sector ( fdc );
}

/**
 * Write sector(s)
 *
 * @v fdc		1770 FDC
 */
static void wd1770_command_write_sector ( WD1770FDC *fdc ) {

	LOG_WD1770 ( "%s: command 0x%02x write to track %d.%d sector %d%s\n",
		     wd1770_name ( fdc ), fdc->command, fdc->track,
		     fdc->side, fdc->sector,
		     ( ( fdc->command & WD1770_CMD_MULTIPLE ) ? "+" : "" ) );

	/* Start writing current sector */
	wd1770_write_sector ( fdc );
}

/**
 * Read address
 *
 * @v fdc		1770 FDC
 */
static void wd1770_command_read_address ( WD1770FDC *fdc ) {

	qemu_log_mask ( LOG_UNIMP, "%s: command 0x%02x unimplemented read "
			"address\n", wd1770_name ( fdc ), fdc->command );

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

	qemu_log_mask ( LOG_UNIMP, "%s: command 0x%02x unimplemented read "
			"track\n", wd1770_name ( fdc ), fdc->command );

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

	LOG_WD1770 ( "%s: command 0x%02x write track %d.%d\n",
		     wd1770_name ( fdc ), fdc->command, fdc->track, fdc->side );

	/* Start writing current track */
	wd1770_write_track ( fdc );
}

/**
 * Force interrupt
 *
 * @v fdc		1770 FDC
 */
static void wd1770_command_force_interrupt ( WD1770FDC *fdc ) {

	LOG_WD1770 ( "%s: command 0x%02x force interrupt (%d remaining)\n",
		     wd1770_name ( fdc ), fdc->command, fdc->remaining );

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
 * Identify command handler
 *
 * @v command		Command
 * @ret handler		Command handler
 */
static WD1770CommandHandler wd1770_command_handler ( uint8_t command ) {
	static const WD1770CommandHandler handlers[16] = {
		wd1770_command_restore,		wd1770_command_seek,
		wd1770_command_step,		wd1770_command_step,
		wd1770_command_step_in,		wd1770_command_step_in,
		wd1770_command_step_out,	wd1770_command_step_out,
		wd1770_command_read_sector,	wd1770_command_read_sector,
		wd1770_command_write_sector,	wd1770_command_write_sector,
		wd1770_command_read_address,	wd1770_command_force_interrupt,
		wd1770_command_read_track,	wd1770_command_write_track,
	};

	return ( handlers[ command >> 4 ] );
}

/**
 * Handle issued command
 *
 * @v fdc		1770 FDC
 * @v command		Command
 */
static void wd1770_command_write ( WD1770FDC *fdc, uint8_t command ) {
	WD1770CommandHandler handler;

	/* Identify command handler */
	handler = wd1770_command_handler ( command );

	/* Forced interrupts are a special case */
	if ( handler == wd1770_command_force_interrupt ) {
		fdc->command = command;
		wd1770_command_force_interrupt ( fdc );
		return;
	}

	/* If controller is busy, then ignore the command */
	if ( fdc->status & WD1770_STAT_BUSY ) {
		LOG_WD1770 ( "%s: command 0x%02x ignored while busy (%d "
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

	/* Start command timer */
	wd1770_command_delayed ( fdc );

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
	WD1770CommandHandler handler;
	uint8_t data;

	/* Read data register */
	data = fdc->data;

	/* If a read command is in progress, read the next data byte
	 * into the data register.
	 */
	if ( fdc->remaining ) {
		handler = wd1770_command_handler ( fdc->command );
		if ( handler == wd1770_command_read_sector ) {
			wd1770_read_sector_next ( fdc );
		}
	}

	return data;
}

/**
 * Write to 1770 FDC data register
 *
 * @v fdc		1770 FDC
 * @v data		Data
 */
static void wd1770_data_write ( WD1770FDC *fdc, uint8_t data ) {
	WD1770CommandHandler handler;

	/* Write data register */
	fdc->data = data;

	/* If a write command is in progress, write this byte from
	 * the data register.
	 */
	if ( fdc->remaining ) {
		handler = wd1770_command_handler ( fdc->command );
		if ( handler == wd1770_command_write_sector ) {
			wd1770_write_sector_next ( fdc );
		} else if ( handler == wd1770_command_write_track ) {
			wd1770_write_track_next ( fdc );
		}
	}
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
	fdc->buf = qemu_memalign ( BDRV_SECTOR_SIZE, WD1770_BUF_SIZE );
	fdc->command_timer =
		qemu_new_timer_ns ( vm_clock, wd1770_command_expired, fdc );
	fdc->drq_timer =
		qemu_new_timer_ns ( vm_clock, wd1770_drq_expired, fdc );
	fdc->motor_off_timer =
		qemu_new_timer_ms ( vm_clock, wd1770_motor_off_expired, fdc );

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
	fdc->fdds[0].sector_size = 256;
#endif
#if 1
	// DFS single-sided
	fdc->fdds[0].single_density = true;
	fdc->fdds[0].sides = 1;
	fdc->fdds[0].tracks = 80;
	fdc->fdds[0].sectors = 10;
	fdc->fdds[0].sector_size = 256;
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
		VMSTATE_TIMER ( command_timer, WD1770FDC ),
		VMSTATE_TIMER ( drq_timer, WD1770FDC ),
		VMSTATE_TIMER ( motor_off_timer, WD1770FDC ),
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
		VMSTATE_UINT32 ( offset, WD1770FDC ),
		VMSTATE_UINT32 ( remaining, WD1770FDC ),
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
