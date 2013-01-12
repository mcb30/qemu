/*
 * Acorn BBC Micro CRT
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
#include "sysemu/sysemu.h"
#include "ui/console.h"
#include "vga_int.h"
#include "bbc.h"

#define DEBUG_CRT 1

/* Debug messages */
#define LOG_CRT(...) do {						\
		if ( DEBUG_CRT ) {					\
			qemu_log_mask ( CPU_LOG_IOPORT, __VA_ARGS__ );	\
		}							\
	} while ( 0 )

/**
 * Translate a CRTC address into a video RAM address
 *
 * @v crt		BBC display
 * @v crtc_address	Refresh memory address
 * @ret teletext	Teletext address translation applies
 * @ret address		Video RAM address
 */
static hwaddr bbc_crt_translate ( BBCDisplay *crt, unsigned int crtc_address,
				  bool *teletext ) {
	static const unsigned int adjustments[4] = BBC_CRTC_ADDR_ADJUSTMENTS;
	unsigned int adjustment;
	unsigned int adjusted_address;
	hwaddr address;

	/* CRTC address bit 12 causes the address to be adjusted by
	 * subtracting an amount configured using the system VIA's
	 * addressable latch.
	 */
	adjusted_address = ( crtc_address << 3 );
	if ( crtc_address & BBC_CRTC_ADDR_ADJUST ) {
		adjustment = ( ( crt->via->addressable_latch >> BBC_LATCH_C0 )
			       & 0x03 );
		adjusted_address -= adjustments[adjustment];
	}

	/* CRTC address bit 13 selects teletext address translation */
	if ( crtc_address & BBC_CRTC_ADDR_TELETEXT ) {
		/* Teletext address translation:
		 *
		 *   CRTC address:      AA3    All 1   MA9..MA0
		 *   Video RAM address:	A14  A13..A10  A9....A0
		 */
		address = ( ( adjusted_address & 0x4000 ) | 0x3c00 |
			    ( crtc_address & 0x3ff ) );
		*teletext = true;
	} else {
		/* Standard address translation:
		 *
		 *   CRTC address:	AA3..AA0  MA7..MA0  RA2..RA0
		 *   Video RAM address:	A13..A11  A10...A3  A2....A0
		 */
		address = ( ( adjusted_address & 0x7800 ) |
			    ( ( crtc_address & 0x00ff ) << 3 ) );
		*teletext = false;
	}

	/* Addresses wrap around within the available RAM */
	address &= ( memory_region_size ( crt->ram ) - 1 );

	LOG_CRT ( "%s: CRTC address &%04X maps to video RAM address &%04lX\n",
		  crt->name, crtc_address, address );
	return address;
}

/**
 * Update video RAM mappings
 *
 * @v crt		BBC display
 */
static void bbc_crt_vram_update ( BBCDisplay *crt ) {
	MC6845CRTC *crtc = crt->crtc;
	unsigned int crtc_transition;
	unsigned int screen_count;
	unsigned int first_count;
	unsigned int second_count;
	hwaddr start;
	hwaddr end;
	unsigned int size;
	void *parent_ptr;

	/* Calculate transition CRTC address, i.e. the first address
	 * after crtc->start with a different value for MA12.  Note
	 * that (a) this address may exceed the 14-bit address range
	 * of the CRTC, and (b) the CRTC may stop before reaching this
	 * address.
	 */
	crtc_transition =
		( ( crtc->start | ( BBC_CRTC_ADDR_ADJUST - 1 ) ) + 1 );

	/* Calcuate screen size (in characters) */
	screen_count = ( crtc->horiz_displayed * crtc->vert_displayed );

	/* Calculate start address and translation for first display region */
	crt->first.start = bbc_crt_translate ( crt, crtc->start,
					       &crt->first.teletext );
	first_count = ( crtc_transition - crtc->start );
	if ( first_count > screen_count )
		first_count = screen_count;
	crt->first.size = ( crt->first.teletext ? first_count :
			    ( first_count * BBC_CRT_CHAR_SIZE ) );

	/* Calculate start address and translation for second display region */
	if ( first_count < screen_count ) {
		/* Second region starts at transition address */
		crt->second.start = bbc_crt_translate ( crt, crtc_transition,
							&crt->second.teletext );
		second_count = ( screen_count - first_count );
		crt->second.size = ( crt->second.teletext ? second_count :
				     ( second_count * BBC_CRT_CHAR_SIZE ) );
	} else {
		/* No second region */
		crt->second.start = 0;
		crt->second.size = 0;
		crt->second.teletext = false;
	}

	/* Calculate bounds of regions used by screen memory */
	start = crt->first.start;
	end = ( crt->first.start + crt->first.size );
	if ( crt->second.size ) {
		if ( start > crt->second.start ) {
			start = crt->second.start;
		} else {
			end = ( crt->second.start + crt->second.size );
		}
	}
	start &= ~( BBC_CRT_VIDEO_RAM_ALIGN - 1 );
	end = ( ( end + BBC_CRT_VIDEO_RAM_ALIGN - 1 ) &
		~( BBC_CRT_VIDEO_RAM_ALIGN - 1 ) );
	size = ( end - start );

	/* Update video RAM memory region */
	if ( ( start != crt->start ) || ( size != crt->size ) ) {

		/* Destroy existing memory region */
		memory_region_del_subregion ( crt->ram, &crt->vram );
		memory_region_destroy ( &crt->vram );

		/* Create new memory region */
		parent_ptr = memory_region_get_ram_ptr ( crt->ram );
		memory_region_init_ram_ptr ( &crt->vram, "vram", size,
					     ( parent_ptr + start ) );
		memory_region_add_subregion ( crt->ram, start, &crt->vram );
		memory_region_set_log ( &crt->vram, true, DIRTY_MEMORY_VGA );

		/* Record new region bounds */
		crt->start = start;
		crt->size = size;
	}

	/* Debug output */
	LOG_CRT ( "%s: [&%04lx,&%04lx) regions [&%04lX,&%04lX) %s",
		  crt->name, crt->start, ( crt->start + crt->size - 1 ),
		  crt->first.start, ( crt->first.start + crt->first.size - 1 ),
		  ( crt->first.teletext ? "teletext" : "graphic" ) );
	if ( crt->second.size ) {
		LOG_CRT ( " [&%04lX,&%04lX) %s",
			  crt->second.start,
			  ( crt->second.start + crt->second.size - 1 ),
			  ( crt->second.teletext ? "teletext" : "graphic" ) );
	}
	LOG_CRT ( "\n" );
}

/**
 * Resize display
 *
 * @v crt		BBC display
 */
static void bbc_crt_resize ( BBCDisplay *crt ) {
	unsigned int width;
	unsigned int height;

	/* Calculate display geometry */
	width = ( ( crt->crtc->horiz_displayed * BBC_CRT_PIXEL_CLOCK ) /
		  bbc_video_ula_crtc_clock ( crt->ula ) );
	height = ( crt->crtc->vert_displayed *
		   ( crt->crtc->max_scan_line + 1 ) );
	if ( ! ( crt->crtc->interlace && crt->crtc->interlace_video ) )
		height <<= 1;

	/* Resize display */
	qemu_console_resize ( crt->ds, width, height );
}

static void bbc_crt_update ( void *opaque ) {
	BBCDisplay *crt = opaque;

	//
	bbc_crt_vram_update ( crt );
	bbc_crt_resize ( crt );

	LOG_CRT ( "%s: update\n", crt->name );
}

static void bbc_crt_invalidate ( void *opaque ) {
	BBCDisplay *crt = opaque;

	LOG_CRT ( "%s: invalidate\n", crt->name );
}

/**
 * Create a screen dump
 *
 * @v opaque		BBC display
 * @v filename		Filename
 * @v cswitch		Console is not currently displayed
 * @ret errp		Error pointer
 */
static void bbc_crt_screen_dump ( void *opaque, const char *filename,
				  bool cswitch, Error **errp ) {
	BBCDisplay *crt = opaque;

	LOG_CRT ( "%s: screen dump\n", crt->name );

	/* Invalidate display if currently displaying a different console */
	if ( cswitch )
		bbc_crt_invalidate ( crt );

	/* Update display */
	bbc_crt_update ( crt );

	/* Dump display to PPM file */
	ppm_save ( filename, crt->ds->surface, errp );
}

/**
 * Initialise CRT
 *
 * @v name		Name
 * @v ram		System RAM
 * @v crtc		6845 CRTC
 * @v ula		Video ULA
 * @v via		System VIA
 * @ret crt		CRT
 */
BBCDisplay * bbc_crt_init ( const char *name, MemoryRegion *ram,
			    MC6845CRTC *crtc, BBCVideoULA *ula,
			    BBCSystemVIA *via ) {
	BBCDisplay *crt = g_new0 ( BBCDisplay, 1 );

	/* Initialise CRT */
	crt->name = name;
	crt->ram = ram;
	crt->crtc = crtc;
	crt->ula = ula;
	crt->via = via;

	/* Initialise graphic console */
	crt->ds = graphic_console_init ( bbc_crt_update,
					 bbc_crt_invalidate,
					 bbc_crt_screen_dump, NULL, crt );

	/* Initialise a dummy video RAM memory region */
	memory_region_init ( &crt->vram, "vram", 0 );
	memory_region_add_subregion ( crt->ram, 0, &crt->vram );
	memory_region_set_enabled ( &crt->vram, false );

	return crt;
}
