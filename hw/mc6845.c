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

#include "hw.h"
#include "boards.h"
#include "exec/address-spaces.h"
#include "mc6845.h"
#define DEBUG_MC6845 1

/* Debug messages */
#define LOG_MC6845(...) do {						\
		if ( DEBUG_MC6845 ) {					\
			qemu_log_mask ( CPU_LOG_IOPORT, __VA_ARGS__ );	\
		}							\
	} while ( 0 )

/**
 * Read from 6845 CRTC
 *
 * @v opaque		6845 CRTC
 * @v addr		Register address
 * @v size		Size of read
 * @ret data		Read data
 */
static uint64_t mc6845_read ( void *opaque, hwaddr addr, unsigned int size ) {
	MC6845CRTC *crtc = opaque;
	uint8_t data;

	/* Ignore reads from the write-only address register */
	if ( ( addr & ( MC6845_SIZE - 1 ) ) == MC6845_ADDRESS )
		return 0;

	/* Read from specified register.  Most registers are write-only */
	switch ( crtc->address ) {
	case MC6845_CURSOR_H:
		data = ( ( crtc->cursor >> 0 ) & 0xff );
		break;
	case MC6845_CURSOR_L:
		data = ( ( crtc->cursor >> 8 ) & 0xff );
		break;
	case MC6845_PEN_H:
		data = ( ( crtc->pen >> 0 ) & 0xff );
		break;
	case MC6845_PEN_L:
		data = ( ( crtc->pen >> 8 ) & 0xff );
		break;
	default:
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented read from "
				"reg 0x%02x\n", crtc->name, crtc->address );
		data = 0;
		break;
	}
	return data;
}

/**
 * Write to 6845 CRTC
 *
 * @v opaque		6845 CRTC
 * @v addr		Register address
 * @v data64		Data to write
 * @v size		Size of write
 */
static void mc6845_write ( void *opaque, hwaddr addr, uint64_t data64,
			   unsigned int size ) {
	MC6845CRTC *crtc = opaque;
	uint8_t data = data64;

	/* Handle writes to address register */
	if ( ( addr & ( MC6845_SIZE - 1 ) ) == MC6845_ADDRESS ) {
		crtc->address = ( data & MC6845_ADDRESS_MASK );
		return;
	}

	/* Handle writes to data register */
	switch ( crtc->address ) {
	case MC6845_HORIZ_TOTAL:
		crtc->horiz_total = data;
		break;
	case MC6845_HORIZ_DISPLAYED:
		crtc->horiz_displayed = data;
		break;
	case MC6845_HORIZ_SYNC_POS:
		crtc->horiz_sync_pos = data;
		break;
	case MC6845_HORIZ_SYNC_WIDTH:
		crtc->horiz_sync_width = ( ( data >> 0 ) & 0x0f );
		crtc->vert_sync_width = ( ( data >> 4 ) & 0x0f );
		break;
	case MC6845_VERT_TOTAL:
		crtc->vert_total = ( data & 0x7f );
		break;
	case MC6845_VERT_ADJUST:
		crtc->vert_adjust = ( data & 0x1f );
		break;
	case MC6845_VERT_DISPLAYED:
		crtc->vert_displayed = ( data & 0x7f );
		break;
	case MC6845_VERT_SYNC_POS:
		crtc->vert_sync_pos = ( data & 0x7f );
		break;
	case MC6845_INTERLACE:
		crtc->interlace = ( ( data >> 0 ) & 0x01 );
		crtc->interlace_video = ( ( data >> 1 ) & 0x01 );
		crtc->display_delay = ( ( data >> 4 ) & 0x03 );
		crtc->display_disabled = ( crtc->display_delay == 0x03 );
		crtc->cursor_delay = ( ( data >> 6 ) & 0x03 );
		crtc->cursor_disabled = ( crtc->cursor_delay == 0x03 );
		break;
	case MC6845_MAX_SCAN_LINE:
		crtc->max_scan_line = ( data & 0x1f );
		break;
	case MC6845_CURSOR_START:
		crtc->cursor_blink = ( ( data >> 6 ) & 0x01 );
		crtc->cursor_blink_slow = ( ( data >> 5 ) & 0x01 );
		crtc->cursor_start = ( ( data >> 0 ) & 0x1f );
		break;
	case MC6845_CURSOR_END:
		crtc->cursor_end = ( data & 0x1f );
		break;
	case MC6845_START_H:
		crtc->start &= ~0xff00;
		crtc->start |= ( ( data & 0x3f ) << 8 );
		break;
	case MC6845_START_L:
		crtc->start &= ~0x00ff;
		crtc->start |= data;
		break;
	case MC6845_CURSOR_H:
		crtc->cursor &= ~0xff00;
		crtc->cursor |= ( ( data & 0x3f ) << 8 );
		break;
	case MC6845_CURSOR_L:
		crtc->cursor &= ~0x00ff;
		crtc->cursor |= data;
		break;
	default:
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented write 0x%02x to "
				"reg 0x%02x\n", crtc->name, data,
				crtc->address );
		break;
	}
	LOG_MC6845 ( "%s: %dx%d of %dx(%d+%d/%d) hsync %d+%d vsync %d+%d"
		     "%s%s delay %d%s\n", crtc->name,
		     crtc->horiz_displayed, crtc->vert_displayed,
		     crtc->horiz_total, crtc->vert_total, crtc->vert_adjust,
		     ( crtc->max_scan_line + 1 ),
		     crtc->horiz_sync_pos, crtc->horiz_sync_width,
		     crtc->vert_sync_pos, crtc->vert_sync_width,
		     ( crtc->interlace ? " int.sync" : "" ),
		     ( ( crtc->interlace && crtc->interlace_video ) ?
		       "+video" : "" ),
		     crtc->display_delay,
		     ( crtc->display_disabled ? " disabled" : "" ) );
	LOG_MC6845 ( "%s: start 0x%04x cursor 0x%04x=(%d,%d) shape %d-%d%s%s"
		     " delay %d%s\n",
		     crtc->name, crtc->start, crtc->cursor,
		     mc6845_cursor_horiz ( crtc ), mc6845_cursor_vert ( crtc ),
		     crtc->cursor_start, crtc->cursor_end,
		     ( crtc->cursor_blink ? " blink" : "" ),
		     ( crtc->cursor_blink ?
		       ( crtc->cursor_blink_slow ? " slow" : " fast" ) : "" ),
		     crtc->cursor_delay,
		     ( crtc->cursor_disabled ? " disabled" : "" ) );

}

/** 6845 CRTC operations */
static const MemoryRegionOps mc6845_ops = {
	.read = mc6845_read,
	.write = mc6845_write,
};

/** Virtual machine state description */
static const VMStateDescription vmstate_mc6845 = {
	.name = "mc6845",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_UINT8 ( address, MC6845CRTC ),
		VMSTATE_UINT8 ( horiz_total, MC6845CRTC ),
		VMSTATE_UINT8 ( horiz_displayed, MC6845CRTC ),
		VMSTATE_UINT8 ( horiz_sync_pos, MC6845CRTC ),
		VMSTATE_UINT8 ( horiz_sync_width, MC6845CRTC ),
		VMSTATE_UINT8 ( vert_total, MC6845CRTC ),
		VMSTATE_UINT8 ( vert_adjust, MC6845CRTC ),
		VMSTATE_UINT8 ( vert_displayed, MC6845CRTC ),
		VMSTATE_UINT8 ( vert_sync_pos, MC6845CRTC ),
		VMSTATE_UINT8 ( vert_sync_width, MC6845CRTC ),
		VMSTATE_UINT8 ( interlace, MC6845CRTC ),
		VMSTATE_UINT8 ( interlace_video, MC6845CRTC ),
		VMSTATE_UINT8 ( display_delay, MC6845CRTC ),
		VMSTATE_UINT8 ( display_disabled, MC6845CRTC ),
		VMSTATE_UINT8 ( cursor_delay, MC6845CRTC ),
		VMSTATE_UINT8 ( cursor_disabled, MC6845CRTC ),
		VMSTATE_UINT8 ( max_scan_line, MC6845CRTC ),
		VMSTATE_UINT8 ( cursor_blink, MC6845CRTC ),
		VMSTATE_UINT8 ( cursor_blink_slow, MC6845CRTC ),
		VMSTATE_UINT8 ( cursor_start, MC6845CRTC ),
		VMSTATE_UINT8 ( cursor_end, MC6845CRTC ),
		VMSTATE_UINT16 ( start, MC6845CRTC ),
		VMSTATE_UINT16 ( cursor, MC6845CRTC ),
		VMSTATE_UINT16 ( pen, MC6845CRTC ),
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Initialise 6845 CRTC
 *
 * @v parent		Parent memory region
 * @v offset		Offset within parent memory region
 * @v size		Size of memory region
 * @v name		Device name
 * @ret crtc		6845 CRTC
 */
MC6845CRTC * mc6845_init ( MemoryRegion *parent, hwaddr offset, uint64_t size,
			   const char *name ) {
	MC6845CRTC *crtc = g_new0 ( MC6845CRTC, 1 );

	/* Initialise CRTC */
	crtc->name = name;

	/* Register memory region */
	memory_region_init_io ( &crtc->mr, &mc6845_ops, crtc, crtc->name,
				size );
	memory_region_add_subregion ( parent, offset, &crtc->mr );

	/* Register virtual machine state */
	vmstate_register ( NULL, offset, &vmstate_mc6845, crtc );

	return crtc;
}
