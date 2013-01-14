/*
 * Acorn BBC Micro
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
#include "loader.h"
#include "bbc.h"

/******************************************************************************
 *
 * Video ULA (SHEILA &20-&2F)
 *
 */

/**
 * Register update notification function
 *
 * @v ula		Video ULA
 * @v updated		Update notification function
 * @v opaque		Opaque pointer
 */
void bbc_video_ula_update_register ( BBCVideoULA *ula,
				     BBCVideoULAUpdated updated,
				     void *opaque ) {
	BBCVideoULAUpdateEntry *entry = g_new0 ( BBCVideoULAUpdateEntry, 1 );

	entry->updated = updated;
	entry->opaque = opaque;
	QTAILQ_INSERT_TAIL ( &ula->updates, entry, next );
}

/**
 * Unregister update notification function
 *
 * @v ula		Video ULA
 * @v updated		Update notification function
 * @v opaque		Opaque pointer
 */
void bbc_video_ula_update_unregister ( BBCVideoULA *ula,
				       BBCVideoULAUpdated updated,
				       void *opaque ) {
	BBCVideoULAUpdateEntry *entry;

	QTAILQ_FOREACH ( entry, &ula->updates, next ) {
		if ( ( entry->updated == updated ) &&
		     ( entry->opaque == opaque ) ) {
			QTAILQ_REMOVE ( &ula->updates, entry, next );
			g_free ( entry );
			return;
		}
	}
}

/**
 * Call update notification functions
 *
 * @v ula		Video ULA
 */
static void bbc_video_ula_updated ( BBCVideoULA *ula ) {
	BBCVideoULAUpdateEntry *entry;

	QTAILQ_FOREACH ( entry, &ula->updates, next ) {
		entry->updated ( entry->opaque );
	}
}

/**
 * Read from video ULA register
 *
 * @v opaque		Video ULA
 * @v addr		Register address
 * @v size		Size of read
 * @ret data		Read data
 */
static uint64_t bbc_video_ula_read ( void *opaque, hwaddr addr,
				     unsigned int size ) {

	/* These are write-only registers */
	return 0xff;
}

/**
 * Write to video ULA control register
 *
 * @v ula		Video ULA
 * @v data		Data to write
 */
static void bbc_video_ula_control_write ( BBCVideoULA *ula, uint8_t data ) {

	/* Decode register */
	ula->cursor_mask = ( ( ( ( data >> 7 ) & 0x01 ) << 0 ) |
			     ( ( ( data >> 6 ) & 0x01 ) << 1 ) |
			     ( ( ( data >> 5 ) & 0x01 ) << 2 ) |
			     ( ( ( data >> 5 /* sic */ ) & 1 ) << 3 ) );
	ula->crtc_clock_log2 = ( ( data >> 4 ) & 0x01 );
	ula->pixel_clock_log2 = ( ( ( data >> 2 ) & 0x03 ) + 1 );
	ula->teletext = ( ( data >> 1 ) & 0x01 );
	ula->flash = ( ( data >> 0 ) & 0x01 );
	qemu_log_mask ( CPU_LOG_IOPORT, "%s: cursor mask %c%c%c%c CRTC %dMHz "
			"pixel %dMHz%s%s\n", ula->name,
			( ( ula->cursor_mask & 0x01 ) ? 'X' : '-' ),
			( ( ula->cursor_mask & 0x02 ) ? 'X' : '-' ),
			( ( ula->cursor_mask & 0x04 ) ? 'X' : '-' ),
			( ( ula->cursor_mask & 0x08 ) ? 'X' : '-' ),
			( 1 << ula->crtc_clock_log2 ),
			( 1 << ula->pixel_clock_log2 ),
			( ula->teletext ? " teletext" : "" ),
			( ula->flash ? " flash" : "" ) );

	/* Call update notification functions */
	bbc_video_ula_updated ( ula );
}

/**
 * Write to video ULA palette register
 *
 * @v ula		Video ULA
 * @v data		Data to write
 */
static void bbc_video_ula_palette_write ( BBCVideoULA *ula, uint8_t data ) {
	unsigned int logical;
	unsigned int actual;

	/* Decode register and update palette */
	logical = ( ( data >> 4 ) & 0x0f );
	actual = ( ( data >> 0 ) & 0x0f );
	ula->palette[logical] = actual;
	qemu_log_mask ( CPU_LOG_IOPORT, "%s: logical colour &%x maps to actual "
			"colour &%x\n", ula->name, logical, actual );
}

/**
 * Write to video ULA register
 *
 * @v opaque		Video ULA
 * @v addr		Register address
 * @v data64		Data to write
 * @v size		Size of write
 */
static void bbc_video_ula_write ( void *opaque, hwaddr addr,
				  uint64_t data64, unsigned int size ) {
	BBCVideoULA *ula = opaque;
	uint8_t data = data64;

	/* Write to specified register */
	switch ( addr & ( BBC_VIDEO_ULA_SIZE - 1 ) ) {
	case BBC_VIDEO_ULA_CONTROL:
		bbc_video_ula_control_write ( ula, data );
		break;
	case BBC_VIDEO_ULA_PALETTE:
		bbc_video_ula_palette_write ( ula, data );
		break;
	default:
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented write &%02x to "
				"&%02lx\n", ula->name, data, addr );
		break;

	}
}

/** Video ULA operations */
static const MemoryRegionOps bbc_video_ula_ops = {
	.read = bbc_video_ula_read,
	.write = bbc_video_ula_write,
};

/** Video ULA state description */
static const VMStateDescription vmstate_bbc_video_ula = {
	.name = "bbc_video_ula",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_UINT8 ( cursor_mask, BBCVideoULA ),
		VMSTATE_UINT8 ( crtc_clock_log2, BBCVideoULA ),
		VMSTATE_UINT8 ( pixel_clock_log2, BBCVideoULA ),
		VMSTATE_UINT8 ( teletext, BBCVideoULA ),
		VMSTATE_UINT8 ( flash, BBCVideoULA ),
		VMSTATE_UINT8_ARRAY ( palette, BBCVideoULA, 16 ),
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Initialise video ULA
 *
 * @v parent		Parent memory region
 * @v offset		Offset within parent memory region
 * @v size		Size of memory region
 * @v name		Device name
 * @ret ula		Video ULA
 */
static BBCVideoULA * bbc_video_ula_init ( MemoryRegion *parent, hwaddr offset,
					  uint64_t size, const char *name ) {
	BBCVideoULA *ula = g_new0 ( BBCVideoULA, 1 );

	/* Initialise ULA */
	ula->name = name;
	QTAILQ_INIT ( &ula->updates );

	/* Register memory region */
	memory_region_init_io ( &ula->mr, &bbc_video_ula_ops, ula, name, size );
	memory_region_add_subregion ( parent, offset, &ula->mr );

	/* Register virtual machine state */
	vmstate_register ( NULL, offset, &vmstate_bbc_video_ula, ula );

	return ula;
}

/******************************************************************************
 *
 * Paged ROMs
 *
 */

/**
 * Calculate paged ROM bank offset
 *
 * @v paged		Paged ROM
 * @v page		Page
 * @ret offset		Offset within page ROM bank
 */
static inline hwaddr bbc_paged_rom_offset ( BBCPagedROM *paged,
					    unsigned int page ) {

	return ( page * paged->size );
}

/**
 * Calculate paged ROM bank physical address for load_image_targphys()
 *
 * @v paged		Paged ROM
 * @v page		Page
 * @ret targphys	Target physical address
 */
static inline hwaddr bbc_paged_rom_targphys ( BBCPagedROM *paged,
					      unsigned int page ) {

	return ( paged->targphys + bbc_paged_rom_offset ( paged, page ) );
}

/**
 * Update paged ROM memory alias
 *
 * @v paged		Paged ROM
 */
static void bbc_paged_rom_update_alias ( BBCPagedROM *paged ) {
	hwaddr offset;

	/* Change offset address into paged ROM virtual memory region */
	offset = bbc_paged_rom_offset ( paged, paged->page );
	memory_region_set_alias_offset ( &paged->rom, offset );
}

/**
 * Read from paged ROM select register
 *
 * @v opaque		Paged ROM
 * @v addr		Register address
 * @v size		Size of read
 * @ret data		Read data
 */
static uint64_t bbc_paged_rom_select_read ( void *opaque, hwaddr addr,
					    unsigned int size ) {

	/* This is a write-only register */
	return 0xff;
}

/**
 * Write to paged ROM select register
 *
 * @v opaque		Paged ROM
 * @v addr		Register address
 * @v data64		Data to write
 * @v size		Size of write
 */
static void bbc_paged_rom_select_write ( void *opaque, hwaddr addr,
					 uint64_t data64, unsigned int size ) {
	BBCPagedROM *paged = opaque;
	uint8_t data = data64;

	/* Store page and update memory alias */
	paged->page = ( data & ( paged->count - 1 ) );
	bbc_paged_rom_update_alias ( paged );
}

/** Paged ROM select register operations */
static const MemoryRegionOps bbc_paged_rom_select_ops = {
	.read = bbc_paged_rom_select_read,
	.write = bbc_paged_rom_select_write,
};

/**
 * Update paged ROM state after loading from snapshot
 *
 * @v opaque		Paged ROM
 * @v version_id	State description version ID
 */
static int bbc_paged_rom_post_load ( void *opaque, int version_id ) {
	BBCPagedROM *paged = opaque;

	/* Update memory alias */
	bbc_paged_rom_update_alias ( paged );

	return 0;
}

/** Paged ROM state description */
static const VMStateDescription vmstate_bbc_paged_rom = {
	.name = "bbc_paged_rom",
	.version_id = 1,
	.minimum_version_id = 1,
	.post_load = bbc_paged_rom_post_load,
	.fields = ( VMStateField[] ) {
		VMSTATE_UINT8 ( page, BBCPagedROM ),
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Initialise paged ROM
 *
 * @v parent		Parent memory region
 * @v offset		Offset within parent memory region
 * @v size		Size of memory region
 * @v name		Device name
 * @v roms_offset	Offset within parent memory region of ROM bank
 * @v targphys		Base physical address of ROM bank
 * @v count		Number of paged ROMs
 * @ret paged		Paged ROM
 */
static BBCPagedROM * bbc_paged_rom_init ( MemoryRegion *parent, hwaddr offset,
					  uint64_t size, const char *name,
					  hwaddr roms_offset, hwaddr targphys,
					  unsigned int count ) {
	BBCPagedROM *paged = g_new0 ( BBCPagedROM, 1 );
	const char *roms_name = g_strdup_printf ( "%s.roms", name );

	/* Initialise paged ROM */
	paged->name = name;
	paged->targphys = targphys;
	paged->size = size;
	paged->count = count;

	/* Initialise ROM bank memory region.  There is no way for the
	 * CPU to access this region directly, but it gives us an
	 * address to pass to load_image_targphys().
	 */
	memory_region_init ( &paged->roms, roms_name,
			     ( paged->count * paged->size ) );
	memory_region_set_readonly ( &paged->roms, true );
	memory_region_add_subregion ( parent, roms_offset, &paged->roms );

	/* Initialise paged ROM memory region */
	memory_region_init_alias ( &paged->rom, name, &paged->roms,
				   0, paged->size );
	memory_region_set_readonly ( &paged->rom, true );
	memory_region_add_subregion ( parent, offset, &paged->rom );

	/* Register virtual machine state */
	vmstate_register ( NULL, offset, &vmstate_bbc_paged_rom, paged );

	return paged;
}

/**
 * Initialise paged ROM select register
 *
 * @v parent		Parent memory region
 * @v offset		Offset within parent memory region
 * @v size		Size of memory region
 * @v paged		Paged ROM
 */
static void bbc_paged_rom_select_init ( MemoryRegion *parent, hwaddr offset,
					uint64_t size, BBCPagedROM *paged ) {
	const char *select_name = g_strdup_printf ( "%s.select", paged->name );

	/* Initialise select register memory region */
	memory_region_init_io ( &paged->select, &bbc_paged_rom_select_ops,
				paged, select_name, size );
	memory_region_add_subregion ( parent, offset, &paged->select );	
}

/******************************************************************************
 *
 * Keyboard
 *
 */

/** Startup DIP switches */
static const uint8_t bbc_startup = 0x07;

/**
 * Check if CAPS LOCK is enabled
 *
 * @v via		System VIA
 * @ret caps_lock	CAPS LOCK is enabled
 */
static inline int bbc_caps_lock ( BBCSystemVIA *via ) {

	/* CAPS LOCK is controlled via the addressable latch */
	return ( via->addressable_latch & ( 1 << BBC_LATCH_CAPS_LOCK ) );
}

/**
 * Check if SHIFT LOCK is enabled
 *
 * @v via		System VIA
 * @ret shift_lock	SHIFT LOCK is enabled
 */
static inline int bbc_shift_lock ( BBCSystemVIA *via ) {

	/* SHIFT LOCK is controlled via the addressable latch */
	return ( via->addressable_latch & ( 1 << BBC_LATCH_SHIFT_LOCK ) );
}

/**
 * Check if keyboard autoscan is enabled
 *
 * @v via		System VIA
 * @ret autoscan	Autoscan is enabled
 */
static inline int bbc_keyboard_autoscan ( BBCSystemVIA *via ) {

	/* Autoscan is controlled via the addressable latch */
	return ( via->addressable_latch & ( 1 << BBC_LATCH_KB_WE ) );
}

/**
 * Get keyboard row address (valid only if autoscan is disabled)
 *
 * @v via		System VIA
 * @ret row		Row address
 */
static inline unsigned int bbc_keyboard_row ( BBCSystemVIA *via ) {

	/* Row address is in PA6:4 */
	return ( ( via->slow_data >> 4 ) & 0x07 );
}

/**
 * Get keyboard column address (valid only if autoscan is disabled)
 *
 * @v via		System VIA
 * @ret column		Column address
 */
static inline unsigned int bbc_keyboard_column ( BBCSystemVIA *via ) {

	/* Column address is in PA3:0 */
	return ( ( via->slow_data >> 0 ) & 0x0f );
}

/** A BBC key */
typedef struct {
	/** Scancode (possibly extended) */
	uint16_t scancode;
	/** Column */
	uint8_t column;
	/** Row */
	uint8_t row;
	/** Name */
	const char *name;
} BBCKey;

/** BBC keyboard map */
static BBCKey bbc_keys[] = {

	/* Basic keyboard grid (rows 1-7) */
	{ 0x0001, 0, 7, "ESC" },		{ 0x003b, 1, 7, "F1" },
	{ 0x003c, 2, 7, "F2" },			{ 0x003d, 3, 7, "F3" },
	{ 0x003f, 4, 7, "F5" },			{ 0x0040, 5, 7, "F6" },
	{ 0x0042, 6, 7, "F8" },			{ 0x0043, 7, 7, "F9" },
	{ 0x002b, 8, 7, "\\|" },		{ 0xe04d, 9, 7, "Right" },
	{ 0x0010, 0, 6, "Q" },			{ 0x0004, 1, 6, "3" },
	{ 0x0005, 2, 6, "4" },			{ 0x0006, 3, 6, "5" },
	{ 0x003e, 4, 6, "F4" },			{ 0x0009, 5, 6, "8" },
	{ 0x0041, 6, 6, "F7" },			{ 0x000c, 7, 6, "-=" /* -_ */ },
	{ 0x000d, 8, 6, "^~" /* =+ */ },	{ 0xe04b, 9, 6, "Left" },
	{ 0x0044, 0, 5, "F0" /* F10 */ },	{ 0x0011, 1, 5, "W" },
	{ 0x0012, 2, 5, "E" },			{ 0x0014, 3, 5, "T" },
	{ 0x0008, 4, 5, "7" },			{ 0x000a, 5, 5, "9" },
	{ 0x0017, 6, 5, "I" },			{ 0x000b, 7, 5, "0" },
	{ 0x0029, 8, 5, "_£" /* `~ */ },	{ 0xe050, 9, 5, "Down" },
	{ 0x0002, 0, 4, "1" },			{ 0x0003, 1, 4, "2" },
	{ 0x0020, 2, 4, "D" },			{ 0x0013, 3, 4, "R" },
	{ 0x0007, 4, 4, "6" },			{ 0x0016, 5, 4, "U" },
	{ 0x0018, 6, 4, "O" },			{ 0x0019, 7, 4, "P" },
	{ 0x001a, 8, 4, "[{" },			{ 0xe048, 9, 4, "Up" },
	{ 0x003a, 0, 3, "CapsLock" },		{ 0x001e, 1, 3, "A" },
	{ 0x002c, 2, 3, "Z" },			{ 0x0021, 3, 3, "F" },
	{ 0x0015, 4, 3, "Y" },			{ 0x0024, 5, 3, "J" },
	{ 0x002d, 6, 3, "X" },			{ 0x0058, 7, 3, "@" /* F12 */ },
	{ 0x0028, 8, 3, ":*" /* '" */ },	{ 0x001c, 9, 3, "Return" },
	{ 0x0045, 0, 2, "ShiftLock" /* NumLock */ }, { 0x001f, 1, 2, "S" },
	{ 0x002e, 2, 2, "C" },			{ 0x0022, 3, 2, "G" },
	{ 0x0023, 4, 2, "H" },			{ 0x0031, 5, 2, "N" },
	{ 0x0026, 6, 2, "L" },			{ 0x0027, 7, 2, ";+" /* ;: */ },
	{ 0x001b, 8, 2, "]}" },			{ 0xe053, 9, 2, "Delete" },
	{ 0x000f, 0, 1, "Tab" },		{ 0x002c, 1, 1, "Z" },
	{ 0x0039, 2, 1, "Space" },		{ 0x002f, 3, 1, "V" },
	{ 0x0030, 4, 1, "B" },			{ 0x0032, 5, 1, "M" },
	{ 0x0033, 6, 1, ",<" },			{ 0x0034, 7, 1, ".>" },
	{ 0x0035, 8, 1, "/?" },		 { 0xe052, 9, 1, "Copy" /* Insert */ },

	/* Modifier keys and DIP switches (row 0) */
	{ 0x002a /* LShift */, 0, 0, "Shift" },
	{ 0x001d /* LCtrl */, 1, 0, "Ctrl" },
	{ 0, 2, 0, "SW1" },			{ 0, 3, 0, "SW2" },
	{ 0, 4, 0, "SW3" },			{ 0, 5, 0, "SW4" },
	{ 0, 6, 0, "SW5" },			{ 0, 7, 0, "SW6" },
	{ 0, 8, 0, "SW7" },			{ 0, 9, 0, "SW8" },

	/* Allow left/right modifier keys to function as equivalents */
	{ 0x0036, 0, 0, "Shift (RShift)" },
	{ 0xe01d, 1, 0, "Ctrl (RCtrl)" },

	/* Allow Backspace to function as equivalent to Delete */
	{ 0xe053, 9, 2, "Delete (Backspace)" },

	/* Allow numeric keypad to function as nearest equivalent keys */
	{ 0x0052, 7, 5, "0 (Keypad0)" },
	{ 0x004f, 0, 4, "1 (Keypad1)" },
	{ 0x0050, 1, 4, "2 (Keypad2)" },
	{ 0x0051, 1, 6, "3 (Keypad3)" },
	{ 0x004b, 2, 6, "4 (Keypad4)" },
	{ 0x004c, 3, 6, "5 (Keypad5)" },
	{ 0x004d, 4, 4, "6 (Keypad6)" },
	{ 0x0047, 4, 5, "7 (Keypad7)" },
	{ 0x0048, 5, 6, "8 (Keypad8)" },
	{ 0x0049, 5, 5, "9 (Keypad9)" },
	{ 0x0053, 7, 1, ".> (Keypad.)" },
	{ 0x004e, 7, 2, ";+ (Keypad+)" },
	{ 0x004a, 7, 6, "-= (Keypad-)" },
	{ 0x0037, 8, 3, ":* (Keypad*)" },
	{ 0xe035, 8, 1, "/? (Keypad/)" },
	{ 0xe01c, 9, 3, "Return (KeypadEnter)" },

	/* Allow "\|" key on UK keyboards ("<>" on many international
	 * keyboards) to function as equivalent to US "\|".
	 */
	{ 0x0056, 8, 7, "\\| (UK)" },

	/* Allow left Windows key to function as equivalent to
	 * CapsLock, and right Windows key as equivalent to ShiftLock.
	 */
	{ 0xe05b, 0, 3, "CapsLock (LeftWindow)" },
	{ 0xe05c, 0, 2, "ShiftLock (RightWindow)" },
};

/** Break key is a hardwired reset line on the BBC */
#define BBC_KEY_BREAK 0xe11d

/** Extended keypresses are prefixed by 0xeX */
#define BBC_KEY_IS_PREFIX(keycode) ( ( (keycode) & 0xf0) == 0xe0 )

/** Key releases are indicated by bit 7 */
#define BBC_KEY_PRESSED(keycode) ( ! ( (keycode) & 0x80 ) )

/** Scancode is constructed from prefix and keycode (ignoring bit 7) */
#define BBC_KEY_SCANCODE( prefix, scancode ) \
	( ( (prefix) << 8 ) | ( (scancode) & 0x7f ) )

/**
 * Update keyboard interrupt line
 *
 * @v via		System VIA
 */
static void bbc_keyboard_update_irq ( BBCSystemVIA *via ) {
	uint8_t keys_pressed;
	unsigned int column;

	/* Get bitmask of contributing keypresses */
	if ( bbc_keyboard_autoscan ( via ) ) {
		keys_pressed = 0;
		for ( column = 0 ; column < BBC_KEYBOARD_COLUMNS ; column++ )
			keys_pressed |= via->keys_pressed[column];
	} else {
		column = bbc_keyboard_column ( via );
		keys_pressed = via->keys_pressed[column];
	}

	/* Only rows 1-7 contribute towards the interrupt */
	keys_pressed &= BBC_KEYBOARD_IRQ_ROW_MASK;

	/* Update VIA's CA2 interrupt line */
	qemu_set_irq ( via->via->a.c2.irq, keys_pressed );
}

/**
 * Handle keyboard event
 *
 * @v opaque		System VIA
 * @v keycode		Keycode
 */
static void bbc_keyboard_event ( void *opaque, int keycode ) {
	BBCSystemVIA *via = opaque;
	BBCKey *key = NULL;
	bool pressed;
	uint16_t scancode;
	unsigned int i;

	/* Handle extended keypresses */
	if ( BBC_KEY_IS_PREFIX ( keycode ) ) {
		via->keycode_prefix = keycode;
		return;
	}

	/* Separate out key press/release indicator */
	pressed = BBC_KEY_PRESSED ( keycode );

	/* Construct extended scancode */
	scancode = BBC_KEY_SCANCODE ( via->keycode_prefix, keycode );
	via->keycode_prefix = 0;

	/* BREAK isn't part of the keyboard; it's a hardware reset switch */
	if ( scancode == BBC_KEY_BREAK ) {
		qemu_system_reset_request();
		return;
	}

	/* Identify key */
	for ( i = 0 ; i < ARRAY_SIZE ( bbc_keys ) ; i++ ) {
		if ( bbc_keys[i].scancode == scancode ) {
			key = &bbc_keys[i];
			break;
		}
	}

	/* Ignore unknown keys */
	if ( ! key ) {
		qemu_log_mask ( LOG_UNIMP, "%s: unknown scancode 0x%04x\n",
				via->name, scancode );
		return;
	}

	/* Ignore duplicate press/release events */
	if ( pressed == ( !! ( via->keys_pressed[key->column] &
			       ( 1 << key->row ) ) ) ) {
		return;
	}

	/* Record key as pressed/released */
	via->keys_pressed[key->column] &= ~( 1 << key->row );
	if ( pressed )
		via->keys_pressed[key->column] |= ( 1 << key->row );
	qemu_log_mask ( CPU_LOG_IOPORT, "%s: key %s %s\n", via->name,
			key->name, ( pressed ? "pressed" : "released" ) );

	/* Update keyboard interrupt line */
	bbc_keyboard_update_irq ( via );
}

/**
 * Check if currently-selected key is pressed
 *
 * @v via		System VIA
 * @ret pressed		Key is pressed
 */
static int bbc_keyboard_pressed ( BBCSystemVIA *via ) {
	unsigned int row = bbc_keyboard_row ( via );
	unsigned int column = bbc_keyboard_column ( via );
	int pressed;

	/* Startup DIP switches are mapped to row 0 columns 2-9 */
	if ( ( row == 0 ) && ( column >= 2 ) ) {
		pressed = ( ( bbc_startup >> ( 9 - column ) ) & 0x01 );
	} else {
		pressed = ( via->keys_pressed[column] & ( 1 << row ) );
	}

	qemu_log_mask ( CPU_LOG_IOPORT, "BBC: keyboard column %d row %d %s\n",
			column, row, ( pressed ? "pressed" : "not pressed" ) );
	return pressed;
}

/**
 * Update keyboard LEDs
 *
 * @v via		System VIA
 */
static void bbc_keyboard_leds ( BBCSystemVIA *via ) {
	int caps_lock = bbc_caps_lock ( via );
	int shift_lock = bbc_shift_lock ( via );
	int ledstate;

	/* Update LEDs based on control bits in addressable latch */
	ledstate = ( ( caps_lock ? QEMU_CAPS_LOCK_LED : 0 ) |
		     ( shift_lock ? QEMU_NUM_LOCK_LED : 0 ) );
	kbd_put_ledstate ( ledstate );
	qemu_log_mask ( CPU_LOG_IOPORT, "BBC: keyboard leds %s %s\n",
			( caps_lock ? "CAPSLOCK" : "capslock" ),
			( shift_lock ? "SHIFTLOCK" : "shiftlock" ) );
}

/******************************************************************************
 *
 * System VIA (SHEILA &40-&4F)
 *
 */

/**
 * Get contents of slow data bus (system VIA port A)
 *
 * @v opaque		System VIA
 * @ret data		Slow data bus contents
 */
static uint8_t bbc_slow_data_read ( void *opaque ) {
	BBCSystemVIA *via = opaque;
	M6522VIAPort *port = &via->via->a;
	uint8_t data;

	/* Set data equal to outputs for all pins configured as outputs */
	data = ( via->slow_data & port->ddr );

	/* Read from keyboard into PA7 if keyboard is enabled */
	if ( ! bbc_keyboard_autoscan ( via ) ) {
		data &= ~( 1 << 7 );
		if ( bbc_keyboard_pressed ( via ) )
			data |= ( 1 << 7 );
	}

	return data;
}

/**
 * Write to slow data bus (system VIA port A)
 *
 * @v opaque		System VIA
 * @v data		Data
 */
static void bbc_slow_data_write ( void *opaque, uint8_t data ) {
	BBCSystemVIA *via = opaque;

	/* Record slow data bus contents */
	via->slow_data = data;

	/* Update keyboard interrupt line */
	bbc_keyboard_update_irq ( via );
}

/**
 * Write to 76489 sound chip
 *
 * @v via		System VIA
 * @v port		Port
 */
static void bbc_sound_write ( BBCSystemVIA *via ) {

	qemu_log_mask ( LOG_UNIMP, "%s: unimplemented sound write &%02x\n",
			via->name, via->slow_data );
}

/**
 * Write to addressable latch (system VIA port B)
 *
 * @v opaque		System VIA
 * @v data		Output data
 */
static void bbc_addressable_latch_write ( void *opaque, uint8_t data ) {
	BBCSystemVIA *via = opaque;
	static const char * names[8] = {
		[BBC_LATCH_SOUND_WE] = "SOUND_WE",
		[BBC_LATCH_SPEECH_RS] = "SPEECH_RS",
		[BBC_LATCH_SPEECH_WS] = "SPEECH_WS",
		[BBC_LATCH_KB_WE] = "KB_WE",
		[BBC_LATCH_C0] = "C0",
		[BBC_LATCH_C1] = "C1",
		[BBC_LATCH_CAPS_LOCK] = "CAPS_LOCK",
		[BBC_LATCH_SHIFT_LOCK] = "SHIFT_LOCK",
	};
	unsigned int latch_address;
	unsigned int latch_data;

	/* Update addressable latch stored value */
	latch_address = ( ( data >> 0 ) & 0x07 );
	latch_data = ( ( data >> 3 ) & 0x01 );
	via->addressable_latch &= ~( 1 << latch_address );
	via->addressable_latch |= ( latch_data << latch_address );
	qemu_log_mask ( CPU_LOG_IOPORT, "%s: addressable latch now &%02X "
			"(bit %d %s %s)\n", via->name, via->addressable_latch,
			latch_address, names[latch_address],
			( latch_data ? "high" : "low" ) );

	/* Handle write events */
	switch ( latch_address ) {
	case BBC_LATCH_SOUND_WE:
		if ( ! latch_data )
			bbc_sound_write ( via );
		break;
	case BBC_LATCH_KB_WE:
		bbc_keyboard_update_irq ( via );
		break;
	case BBC_LATCH_CAPS_LOCK:
	case BBC_LATCH_SHIFT_LOCK:
		bbc_keyboard_leds ( via );
		break;
	}
}

/** System VIA operations */
static M6522VIAOps bbc_system_via_ops = {
	.b = {
		.output = bbc_addressable_latch_write,
	},
	.a = {
		.input = bbc_slow_data_read,
		.output = bbc_slow_data_write,
	},
};

/** System VIA state description */
static const VMStateDescription vmstate_bbc_system_via = {
	.name = "bbc_system_via",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_UINT8 ( slow_data, BBCSystemVIA ),
		VMSTATE_UINT8 ( addressable_latch, BBCSystemVIA ),
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Initialise system VIA
 *
 * @v parent		Parent memory region
 * @v offset		Offset within parent memory region
 * @v size		Size of memory region
 * @v name		Device name
 * @v irq		Interrupt request line
 * @ret via		System VIA
 */
static BBCSystemVIA * bbc_system_via_init ( MemoryRegion *parent, hwaddr offset,
					    uint64_t size, const char *name,
					    qemu_irq irq ) {
	BBCSystemVIA *via = g_new0 ( BBCSystemVIA, 1 );

	/* Initialise system VIA */
	via->name = name;
	via->via = m6522_init ( parent, offset, size, name, via,
				&bbc_system_via_ops, irq, BBC_1MHZ_TICK_NS );

	/* Initialise keyboard */
	qemu_add_kbd_event_handler ( bbc_keyboard_event, via );

	/* Register virtual machine state */
	vmstate_register ( NULL, offset, &vmstate_bbc_system_via, via );

	return via;
}

/******************************************************************************
 *
 * User VIA (SHEILA &60-&6F)
 *
 */

/**
 * Write to parallel port (user VIA port A)
 *
 * @v opaque		User VIA
 * @v data		Output data
 */
static void bbc_parallel_write ( void *opaque, uint8_t data ) {
	BBCUserVIA *via = opaque;

	qemu_log_mask ( CPU_LOG_IOPORT, "%s: print character &%02x '%c'\n",
			via->name, data, ( isprint ( data ) ? data : '.' ) );
}

/** User VIA operations */
static M6522VIAOps bbc_user_via_ops = {
	.a = {
		.output = bbc_parallel_write,
	},
};

/** User VIA state description */
static const VMStateDescription vmstate_bbc_user_via = {
	.name = "bbc_user_via",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Initialise user VIA
 *
 * @v parent		Parent memory region
 * @v offset		Offset within parent memory region
 * @v size		Size of memory region
 * @v name		Device name
 * @v irq		Interrupt request line
 * @ret via		User VIA
 */
static BBCUserVIA * bbc_user_via_init ( MemoryRegion *parent, hwaddr offset,
					uint64_t size, const char *name,
					qemu_irq irq ) {
	BBCUserVIA *via = g_new0 ( BBCUserVIA, 1 );

	/* Initialise user VIA */
	via->name = name;
	via->via = m6522_init ( parent, offset, size, name, via,
				&bbc_user_via_ops, irq, BBC_1MHZ_TICK_NS );

	/* Register virtual machine state */
	vmstate_register ( NULL, offset, &vmstate_bbc_user_via, via );

	return via;
}

/******************************************************************************
 *
 * FRED, JIM and SHEILA
 *
 */

/**
 * Read from unimplemented I/O region
 *
 * @v opaque		Unimplemented memory region
 * @v addr		Address within region
 * @v size		Size of read
 * @ret data		Read data
 */
static uint64_t bbc_unimplemented_read ( void *opaque, hwaddr addr,
					 unsigned int size ) {
	BBCUnimplementedMemoryRegion *region = opaque;

	qemu_log_mask ( LOG_UNIMP, "%s: unimplemented read from &%02lX\n",
			region->name, addr );
	return 0;
}

/**
 * Write to unimplemented I/O region
 *
 * @v opaque		Unimplemented memory region
 * @v addr		Address within region
 * @v data		Data to write
 * @v size		Size of write
 */
static void bbc_unimplemented_write ( void *opaque, hwaddr addr, uint64_t data,
				      unsigned int size ) {
	BBCUnimplementedMemoryRegion *region = opaque;

	qemu_log_mask ( LOG_UNIMP, "%s: unimplemented write to &%02lX\n",
			region->name, addr );
}

/** Unimplemented I/O region operations */
static const MemoryRegionOps bbc_unimplemented_ops = {
	.read = bbc_unimplemented_read,
	.write = bbc_unimplemented_write,
};

/**
 * Initialise FRED
 *
 * @v parent		Parent memory region
 * @v offset		Offset within parent memory region
 * @v name		Device name
 * @ret fred		FRED
 */
static BBCFRED * bbc_fred_init ( MemoryRegion *parent, hwaddr offset,
				 const char *name ) {
	BBCFRED *fred = g_new0 ( BBCFRED, 1 );

	/* Initialise FRED */
	fred->unimp.name = name;

	/* Set up FRED as an unimplemented I/O region */
	memory_region_init_io ( &fred->mr, &bbc_unimplemented_ops, &fred->unimp,
				name, BBC_FRED_SIZE );
	memory_region_add_subregion_overlap ( parent, offset, &fred->mr, 1 );

	return fred;
}

/**
 * Initialise JIM
 *
 * @v parent		Parent memory region
 * @v offset		Offset within parent memory region
 * @v name		Device name
 * @ret jim		JIM
 */
static BBCJIM * bbc_jim_init ( MemoryRegion *parent, hwaddr offset,
			       const char *name ) {
	BBCJIM *jim = g_new0 ( BBCJIM, 1 );

	/* Initialise JIM */
	jim->unimp.name = name;

	/* Set up JIM as an unimplemented I/O region */
	memory_region_init_io ( &jim->mr, &bbc_unimplemented_ops, &jim->unimp,
				name, BBC_JIM_SIZE );
	memory_region_add_subregion_overlap ( parent, offset, &jim->mr, 1 );

	return jim;
}

/**
 * Initialise SHEILA
 *
 * @v parent		Parent memory region
 * @v offset		Offset within parent memory region
 * @v name		Device name
 * @v paged		Paged ROM
 * @v system_via_irq	System VIA interrupt request line
 * @v user_via_irq	User VIA interrupt request line
 * @ret sheila		SHEILA
 */
static BBCSHEILA * bbc_sheila_init ( MemoryRegion *parent, hwaddr offset,
				     const char *name, BBCPagedROM *paged,
				     qemu_irq system_via_irq,
				     qemu_irq user_via_irq ) {
	BBCSHEILA *sheila = g_new0 ( BBCSHEILA, 1 );

	/* Initialise SHEILA */
	sheila->name = name;
	sheila->unimp.name = name;
	sheila->paged = paged;

	/* Set up SHEILA as an unimplemented I/O container region to
	 * catch any unimplemented accesses.
	 */
	memory_region_init_io ( &sheila->mr, &bbc_unimplemented_ops,
				&sheila->unimp, name, BBC_SHEILA_SIZE );
	memory_region_add_subregion_overlap ( parent, offset, &sheila->mr, 1 );

	/* Initialise CRTC */
	sheila->crtc = mc6845_init ( &sheila->mr, BBC_SHEILA_CRTC_BASE,
				     BBC_SHEILA_CRTC_SIZE, "crtc" );

	/* Initialise serial system */
	sheila->acia = mc6850_init ( &sheila->mr, BBC_SHEILA_ACIA_BASE,
				     BBC_SHEILA_ACIA_SIZE, "acia",
				     serial_hds[0] );

	/* Initialise video ULA */
	sheila->video_ula = bbc_video_ula_init ( &sheila->mr,
						 BBC_SHEILA_VIDEO_ULA_BASE,
						 BBC_SHEILA_VIDEO_ULA_SIZE,
						 "video_ula" );

	/* Initialise paged ROM select register */
	bbc_paged_rom_select_init ( &sheila->mr,
				    BBC_SHEILA_PAGED_ROM_SELECT_BASE,
				    BBC_SHEILA_PAGED_ROM_SELECT_SIZE,
				    sheila->paged );

	/* Initialise system and user VIAs */
	sheila->system_via =
		bbc_system_via_init ( &sheila->mr, BBC_SHEILA_SYSTEM_VIA_BASE,
				      BBC_SHEILA_SYSTEM_VIA_SIZE,
				      "system_via", system_via_irq );
	sheila->user_via =
		bbc_user_via_init ( &sheila->mr, BBC_SHEILA_USER_VIA_BASE,
				    BBC_SHEILA_USER_VIA_SIZE,
				    "user_via", user_via_irq );

	return sheila;
}

/******************************************************************************
 *
 * Interrupts
 *
 */

/**
 * Interrupt handler
 *
 * @v bbc		BBC Micro
 * @v active		Interrupt set status
 * @v count		Number of interrupts within set
 * @v cpu_irq_type	CPU interrupt type
 * @v n			Interrupt number
 * @v level		Interrupt level
 */
static void bbc_interrupt ( BBCMicro *bbc, bool *active, unsigned int count,
			    int cpu_irq_type, int n, int level ) {
	unsigned int i;

	/* Record status of this interrupt */
	active[n] = level;

	/* If this interrupt is active, assert the CPU interrupt */
	if ( level ) {
		cpu_interrupt ( bbc->cpu, cpu_irq_type );
		return;
	}

	/* Otherwise, if any interrupt is active, leave the CPU
	 * interrupt asserted.
	 */
	for ( i = 0 ; i < count ; i++ ) {
		if ( active[i] )
			return;
	}

	/* Otherwise, deassert the CPU interrupt */
	cpu_reset_interrupt ( bbc->cpu, cpu_irq_type );
}

/**
 * IRQ handler
 *
 * @v opaque		BBC Micro
 * @v n			Interrupt number
 * @v level		Interrupt level
 */
static void bbc_irq_handler ( void *opaque, int n, int level ) {
	BBCMicro *bbc = opaque;

	/* Control CPU IRQ pin */
	bbc_interrupt ( bbc, bbc->irq_active, BBC_IRQ_COUNT,
			CPU_INTERRUPT_HARD, n, level );
}

/**
 * NMI handler
 *
 * @v opaque		BBC Micro
 * @v n			Interrupt number
 * @v level		Interrupt level
 */
static void bbc_nmi_handler ( void *opaque, int n, int level ) {
	BBCMicro *bbc = opaque;

	/* Control CPU NMI pin */
	bbc_interrupt ( bbc, bbc->nmi_active, BBC_NMI_COUNT,
			CPU_INTERRUPT_NMI, n, level );
}

/**
 * Initialise interrupts
 *
 * @v bbc		BBC Micro
 */
static void bbc_interrupts_init ( BBCMicro *bbc ) {

	/* Allocate IRQ and NMI interrupts and set inactive (high) */
	bbc->irq = qemu_allocate_irqs ( bbc_irq_handler, bbc, BBC_IRQ_COUNT );
	bbc->nmi = qemu_allocate_irqs ( bbc_nmi_handler, bbc, BBC_NMI_COUNT );
}

/******************************************************************************
 *
 * ROM loading
 *
 */

/**
 * Load ROM
 *
 * @v parent		Parent memory region
 * @v offset		Offset within parent memory region
 * @v size		Size of memory region
 * @v name		Name
 * @v filename		Filename
 * @v targphys		Base address within system memory
 * @v max_size		Maximum allowed size of ROM
 * @ret rom		ROM
 */
static BBCROM * bbc_load_rom ( MemoryRegion *parent, hwaddr offset,
			       uint64_t size, const char *name,
			       const char *filename, hwaddr targphys ) {
	BBCROM *rom = g_new0 ( BBCROM, 1 );
	const char *actual_filename;
	int actual_size;

	/* Initialise ROM */
	rom->name = name;

	/* Initialise memory region */
	memory_region_init_ram ( &rom->mr, name, size );
	vmstate_register_ram_global ( &rom->mr );
	memory_region_set_readonly ( &rom->mr, true );
	memory_region_add_subregion ( parent, offset, &rom->mr );

	/* Locate ROM file */
	actual_filename = qemu_find_file ( QEMU_FILE_TYPE_BIOS, filename );
	if ( ! actual_filename ) {
		fprintf ( stderr, "qemu: could not find ROM '%s'\n",
			  filename );
		exit ( 1 );
	}

	/* Check size */
	actual_size = load_image_targphys ( actual_filename, targphys, size );
	if ( actual_size < 0 ) {
		fprintf ( stderr, "qemu: could not load (or bad size) ROM "
			  "'%s'\n", actual_filename );
		exit ( 1 );
	}

	return rom;
}

/**
 * Load MOS ROM
 *
 * @v parent		Parent memory region
 * @v offset		Offset within parent memory region
 * @v size		Size of memory region
 * @v default_filename	Default filename to use if none specified
 * @v targphys		Target physical address
 * @ret rom		ROM
 */
static BBCROM * bbc_load_mos ( MemoryRegion *parent, hwaddr offset,
			       uint64_t size, const char *default_filename,
			       hwaddr targphys ) {
	const char *name;

	/* Use default filename if no 'BIOS' name is specified */
	if ( bios_name == NULL )
		bios_name = default_filename;

	/* Load MOS ROM */
	name = g_strdup_printf ( "mos %s", bios_name );
	return bbc_load_rom ( parent, offset, size, name, bios_name, targphys );
}

/**
 * Load paged ROMs
 *
 * @v paged		Paged ROM
 * @v basic_filename	Filename for BASIC ROM (always loaded)
 */
static void bbc_load_paged_roms ( BBCPagedROM *paged,
				  const char *basic_filename ) {
	unsigned int i;
	unsigned int page;
	const char *filename;
	const char *name;

	/* Load any specified option ROMs plus the BASIC ROM */
	for ( i = 0 ; ( ( i < paged->count ) &&
			( i < ( nb_option_roms + 1 ) ) ) ; i++ ) {
		page = ( paged->count - i - 1 );
		filename = ( ( i == nb_option_roms ) ? basic_filename :
			     option_rom[0].name );
		name = g_strdup_printf ( "rom%d %s", page, filename );
		bbc_load_rom ( &paged->roms,
			       bbc_paged_rom_offset ( paged, page ),
			       paged->size, name, filename,
			       bbc_paged_rom_targphys ( paged, page ) );
	}
}

/******************************************************************************
 *
 * Machine initialisation
 *
 */

/** BBC Micro state description */
static const VMStateDescription vmstate_bbc = {
	.name = "bbc",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Initialise BBC Model B
 *
 * @v args		Machine arguments
 */
static void bbcb_init ( QEMUMachineInitArgs *args ) {
	BBCMicro *bbc = g_new0 ( BBCMicro, 1 );
	const char *cpu_model = args->cpu_model;
	MemoryRegion *address_space_mem = get_system_memory();

	/* Initialise machine */
	bbc->name = "bbcb";

	/* Initialise RAM */
	memory_region_init_ram ( &bbc->ram, "ram", BBC_B_RAM_SIZE );
	vmstate_register_ram_global ( &bbc->ram );
	memory_region_add_subregion ( address_space_mem, BBC_B_RAM_BASE,
				      &bbc->ram );

	/* Initialise ROMs */
	bbc->mos = bbc_load_mos ( address_space_mem, BBC_B_MOS_BASE,
				  BBC_B_MOS_SIZE, BBC_B_MOS_FILENAME,
				  BBC_B_MOS_BASE );
	bbc->paged = bbc_paged_rom_init ( address_space_mem,
					  BBC_PAGED_ROM_BASE,
					  BBC_PAGED_ROM_SIZE, "paged_rom",
					  BBC_PAGED_ROM_VIRTUAL_BASE,
					  BBC_PAGED_ROM_VIRTUAL_BASE,
					  BBC_PAGED_ROM_COUNT );
	bbc_load_paged_roms ( bbc->paged, BBC_B_BASIC_FILENAME );

	/* Initialise CPU */
	if ( cpu_model == NULL )
		cpu_model = "6502";
	bbc->cpu = m6502_init ( cpu_model );

	/* Initialise interrupts */
	bbc_interrupts_init ( bbc );

	/* Initialise FRED, JIM, and SHEILA */
	bbc->fred = bbc_fred_init ( address_space_mem, BBC_FRED_BASE, "fred" );
	bbc->jim = bbc_jim_init ( address_space_mem, BBC_JIM_BASE, "jim" );
	bbc->sheila = bbc_sheila_init ( address_space_mem, BBC_SHEILA_BASE,
					"sheila", bbc->paged,
					bbc->irq[BBC_IRQ_SYSTEM_VIA],
					bbc->irq[BBC_IRQ_USER_VIA] );

	/* Initialise display */
	bbc->crt = bbc_crt_init ( "crt", &bbc->ram, bbc->sheila->crtc,
				  bbc->sheila->video_ula,
				  bbc->sheila->system_via );

	/* Register virtual machine state */
	vmstate_register ( NULL, 0, &vmstate_bbc, bbc );
}

/** BBC Model B */
static QEMUMachine bbc_model_b = {
	.name = "bbcb",
	.desc = "Acorn BBC Micro Model B",
	.init = bbcb_init,
	.is_default = 1,
};

/**
 * Register BBC machines
 */
static void bbc_machine_init ( void ) {
	qemu_register_machine ( &bbc_model_b );
}

machine_init ( bbc_machine_init );
