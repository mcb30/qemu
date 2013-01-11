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

	/** Address register */
	uint8_t address;
	/** Horizontal total */
	uint8_t horiz_total;
	/** Horizontal displayed */
	uint8_t horiz_displayed;
	/** Horizontal sync position */
	uint8_t horiz_sync_pos;
	/** Horizontal sync width */
	uint8_t horiz_sync_width;
	/** Vertical total */
	uint8_t vert_total;
	/** Vertical total adjust */
	uint8_t vert_adjust;
	/** Vertical displayed */
	uint8_t vert_displayed;
	/** Vertical sync position */
	uint8_t vert_sync_pos;
	/** Vertical sync width */
	uint8_t vert_sync_width;
	/** Interlace sync */
	uint8_t interlace;
	/** Interlace video */
	uint8_t interlace_video;
	/** Display delay */
	uint8_t display_delay;
	/** Display disabled */
	uint8_t display_disabled;
	/** Cursor delay */
	uint8_t cursor_delay;
	/** Cursor disabled */
	uint8_t cursor_disabled;
	/** Max scan line address */
	uint8_t max_scan_line;
	/** Cursor blink enabled */
	uint8_t cursor_blink;
	/** Cursor blink slow */
	uint8_t cursor_blink_slow;
	/** Cursor start */
	uint8_t cursor_start;
	/** Cursor end */
	uint8_t cursor_end;
	/** Start address */
	uint16_t start;
	/** Cursor */
	uint16_t cursor;
	/** Light pen */
	uint16_t pen;
};

/**
 * Calculate cursor horizontal position
 *
 * @v crtc		6845 CRTC
 * @ret hpos		Horizontal position (or -1 if undefined)
 */
static inline int mc6845_cursor_horiz ( MC6845CRTC *crtc ) {
	unsigned int offset = ( crtc->cursor - crtc->start );

	return ( crtc->horiz_displayed ?
		 ( offset % crtc->horiz_displayed ) : -1 );
}

/**
 * Calculate cursor vertical position
 *
 * @v crtc		6845 CRTC
 * @ret hpos		Vertical position (or -1 if undefined)
 */
static inline int mc6845_cursor_vert ( MC6845CRTC *crtc ) {
	unsigned int offset = ( crtc->cursor - crtc->start );

	return ( crtc->horiz_displayed ?
		 ( offset / crtc->horiz_displayed ) : -1 );
}

/** Size of memory region */
#define MC6845_SIZE 0x02

/* Register addresses */
#define MC6845_ADDRESS 0x00
#define MC6845_DATA 0x01

/* Internal register addresses */
#define MC6845_ADDRESS_MASK 0x1f
#define MC6845_HORIZ_TOTAL 0x00
#define MC6845_HORIZ_DISPLAYED 0x01
#define MC6845_HORIZ_SYNC_POS 0x02
#define MC6845_HORIZ_SYNC_WIDTH 0x03
#define MC6845_VERT_TOTAL 0x04
#define MC6845_VERT_ADJUST 0x05
#define MC6845_VERT_DISPLAYED 0x06
#define MC6845_VERT_SYNC_POS 0x07
#define MC6845_INTERLACE 0x08
#define MC6845_MAX_SCAN_LINE 0x09
#define MC6845_CURSOR_START 0x0a
#define MC6845_CURSOR_END 0x0b
#define MC6845_START_H 0x0c
#define MC6845_START_L 0x0d
#define MC6845_CURSOR_H 0x0e
#define MC6845_CURSOR_L 0x0f
#define MC6845_PEN_H 0x10
#define MC6845_PEN_L 0x11

extern void mc6845_init ( MemoryRegion *parent, hwaddr offset, hwaddr size,
			  const char *name );

#endif
