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
#include "mc6845.h"
#include "mc6850.h"
#include "m6522.h"
#include "bbc.h"

/******************************************************************************
 *
 * System state
 *
 */

/** System state definition */
typedef struct {
	/** Addressable latch */
	uint8_t addressable_latch;
} BBCState;

/** System state description */
static const VMStateDescription vmstate_bbc = {
	.name = "bbc",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_UINT8 ( addressable_latch, BBCState ),
		VMSTATE_END_OF_LIST()
	},
};

/** System state */
static BBCState bbc;

/******************************************************************************
 *
 * Interrupts
 *
 */

/** Maskable interrupt */
static qemu_irq bbc_irq;

/** Non-maskable interrupt */
static qemu_irq bbc_nmi;

/**
 * IRQ handler
 *
 */
static void bbc_irq_handler ( void *opaque, int n, int level ) {

	/* Control CPU IRQ pin */
	if ( level ) {
		cpu_interrupt ( first_cpu, CPU_INTERRUPT_HARD );
	} else {
		cpu_reset_interrupt ( first_cpu, CPU_INTERRUPT_HARD );
	}
}

/**
 * NMI handler
 *
 */
static void bbc_nmi_handler ( void *opaque, int n, int level ) {

	/* Control CPU NMI pin */
	if ( level ) {
		cpu_interrupt ( first_cpu, CPU_INTERRUPT_NMI );
	} else {
		cpu_reset_interrupt ( first_cpu, CPU_INTERRUPT_NMI );
	}
}

/**
 * Initialise interrupts
 *
 */
static void bbc_interrupts_init ( void ) {

	/* Allocate IRQ and NMI interrupts */
	bbc_irq = qemu_allocate_irqs ( bbc_irq_handler, NULL, 1 )[0];
	bbc_nmi = qemu_allocate_irqs ( bbc_nmi_handler, NULL, 1 )[0];
}

/******************************************************************************
 *
 * Video ULA (SHEILA &20-&2F)
 *
 */

/** Video ULA */
typedef struct {
	/** Name */
	const char *name;
	/** Memory region */
	MemoryRegion mr;
} BBCVideoULA;

/** Video ULA state description */
static const VMStateDescription vmstate_bbc_video_ula = {
	.name = "bbc_video_ula",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_END_OF_LIST()
	},
};

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
	return 0;
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
		//	case BBC_VIDEO_ULA_SIZE:
		
		//		break;
	default:
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented write 0x%02x to "
				"0x%02lx\n", ula->name, data, addr );
		break;

	}
}

/** Video ULA operations */
static const MemoryRegionOps bbc_video_ula_ops = {
	.read = bbc_video_ula_read,
	.write = bbc_video_ula_write,
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
					  hwaddr size, const char *name ) {
	BBCVideoULA *ula = g_new0 ( BBCVideoULA, 1 );

	/* Initialise ULA */
	ula->name = name;

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

/** Paged ROM */
typedef struct {
	/** Name */
	const char *name;
	/** Paged ROM memory region */
	MemoryRegion rom;
	/** Paged ROM bank memory region */
	MemoryRegion roms;
	/** Paged ROM select register memory region */
	MemoryRegion select;
	/** Base target physical address */
	hwaddr targphys;
	/** Paged ROM size */
	hwaddr size;
	/** Number of paged ROMs */
	unsigned int count;

	/** Paged ROM select register */
	uint8_t page;
} BBCPagedROM;

/** Paged ROM state description */
static const VMStateDescription vmstate_bbc_paged_rom = {
	.name = "bbc_paged_rom",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_UINT8 ( page, BBCPagedROM ),
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Calculate paged ROM bank offset
 *
 * @v paged		Paged ROM
 * @v page		Page
 * @ret offset		Offset within page ROM bank
 */
static hwaddr bbc_paged_rom_offset ( BBCPagedROM *paged, unsigned int page ) {

	return ( page * paged->size );
}

/**
 * Calculate paged ROM bank physical address for load_image_targphys()
 *
 * @v paged		Paged ROM
 * @v page		Page
 * @ret targphys	Target physical address
 */
static hwaddr bbc_paged_rom_targphys ( BBCPagedROM *paged, unsigned int page ) {

	return ( paged->targphys + bbc_paged_rom_offset ( paged, page ) );
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
	return 0;
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
	hwaddr offset;

	/* Change offset address into paged ROM virtual memory region */
	paged->page = ( data & ( paged->count - 1 ) );
	offset = bbc_paged_rom_offset ( paged, paged->page );
	memory_region_set_alias_offset ( &paged->rom, offset );
}

/** Paged ROM select register operations */
static const MemoryRegionOps bbc_paged_rom_select_ops = {
	.read = bbc_paged_rom_select_read,
	.write = bbc_paged_rom_select_write,
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
					  hwaddr size, const char *name,
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
					hwaddr size, BBCPagedROM *paged ) {
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

/**
 * Check if currently-selected key is pressed
 *
 * @v data		Slow data bus contents (excluding PA7)
 * @ret pressed		Key is pressed
 */
static int bbc_keyboard_pressed ( uint8_t data ) {
	unsigned int row;
	unsigned int column;
	int pressed;

	/* Calculate row (PA6:4) and column (PA3:0) addresses */
	row = ( ( data >> 4 ) & 0x07 );
	column = ( ( data >> 0 ) & 0x0f );
	pressed = 0;

	qemu_log_mask ( CPU_LOG_IOPORT, "BBC: keyboard column %d row %d %s\n",
			column, row, ( pressed ? "pressed" : "not pressed" ) );
	return 0;
}

/**
 * Check if CAPS LOCK is enabled
 *
 * @ret caps_lock	CAPS LOCK is enabled
 */
static inline int bbc_caps_lock ( void ) {

	return ( bbc.addressable_latch & ( 1 << BBC_LATCH_CAPS_LOCK ) );
}

/**
 * Check if SHIFT LOCK is enabled
 *
 * @ret shift_lock	SHIFT LOCK is enabled
 */
static inline int bbc_shift_lock ( void ) {

	return ( bbc.addressable_latch & ( 1 << BBC_LATCH_SHIFT_LOCK ) );
}

/**
 * Update keyboard LEDs
 *
 */
static void bbc_keyboard_leds ( void ) {
	int caps_lock = bbc_caps_lock();
	int shift_lock = bbc_shift_lock();
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
 * @v via		6522 VIA
 * @v port		Port
 * @ret data		Slow data bus contents
 */
static uint8_t bbc_slow_data ( M6522VIA *via, M6522VIAPort *port ) {
	uint8_t data;

	/* Set data equal to outputs for all pins configured as outputs */
	data = ( port->or & port->ddr );

	/* Read from keyboard into PA7 if keyboard is enabled */
	if ( ! ( bbc.addressable_latch & ( 1 << BBC_LATCH_KB_WE ) ) ) {
		data &= ~( 1 << 7 );
		if ( bbc_keyboard_pressed ( data ) )
			data |= ( 1 << 7 );
	}

	return data;
}

/**
 * Write to 76489 sound chip
 * 
 * @v via		6522 VIA
 * @v port		Port
 */
static void bbc_sound_write ( M6522VIA *via, M6522VIAPort *port ) {

	qemu_log_mask ( LOG_UNIMP, "BBC: unimplemented sound write &%02x\n",
			bbc_slow_data ( via, port ) );
}

/**
 * Write to addressable latch (system VIA port B)
 *
 * @v via		6522 VIA
 * @v port		Port
 * @v data		Output data
 */
static void bbc_addressable_latch_write ( M6522VIA *via, M6522VIAPort *port,
					  uint8_t data ) {
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
	bbc.addressable_latch &= ~( 1 << latch_address );
	bbc.addressable_latch |= ( latch_data << latch_address );
	qemu_log_mask ( CPU_LOG_IOPORT, "BBC: addressable latch now &%02X "
			"(bit %d %s %s)\n", bbc.addressable_latch,
			latch_address, names[latch_address],
			( latch_data ? "high" : "low" ) );

	/* Handle write events */
	switch ( latch_address ) {
	case BBC_LATCH_SOUND_WE:
		if ( ! latch_data )
			bbc_sound_write ( via, port );
		break;
	case BBC_LATCH_CAPS_LOCK:
	case BBC_LATCH_SHIFT_LOCK:
		bbc_keyboard_leds();
		break;
	}
}

/** System VIA operations */
static M6522VIAOps bbc_system_via_ops = {
	.b = {
		.output = bbc_addressable_latch_write,
	},
	.a = {
		.input = bbc_slow_data,
	},
};

/******************************************************************************
 *
 * User VIA (SHEILA &60-&6F)
 *
 */

/**
 * Write to parallel port (user VIA port A)
 *
 * @v via		6522 VIA
 * @v port		Port
 * @v data		Output data
 */
static void bbc_parallel_write ( M6522VIA *via, M6522VIAPort *port,
				 uint8_t data ) {

	qemu_log_mask ( CPU_LOG_IOPORT, "BBC: print character &%02x '%c'\n",
			data, ( isprint ( data ) ? data : '.' ) );
}

/** User VIA operations */
static M6522VIAOps bbc_user_via_ops = {
	.a = {
		.output = bbc_parallel_write,
	},
};

/******************************************************************************
 *
 * FRED, JIM and SHEILA
 *
 */

/** Unimplemented memory region */
typedef struct {
	/** Name */
	const char *name;
} BBCUnimplementedMemoryRegion;

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

/** FRED */
typedef struct {
	/** Memory region */
	MemoryRegion mr;
	/** Unimplemented memory region */
	BBCUnimplementedMemoryRegion unimp;	
} BBCFRED;

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

/** JIM */
typedef struct {
	/** Memory region */
	MemoryRegion mr;
	/** Unimplemented memory region */
	BBCUnimplementedMemoryRegion unimp;	
} BBCJIM;

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

/** SHEILA */
typedef struct {
	/** Name */
	const char *name;
	/** Memory region */
	MemoryRegion mr;
	/** Unimplemented memory region */
	BBCUnimplementedMemoryRegion unimp;
	/** CRTC */
	MC6845CRTC *crtc;
	/** ACIA */
	MC6850ACIA *acia;
	/** Video ULA */
	BBCVideoULA *video_ula;
	/** Paged ROM */
	BBCPagedROM *paged;
	/** System VIA */
	M6522VIA *system_via;
	/** User VIA */
	M6522VIA *user_via;
} BBCSHEILA;

/**
 * Initialise SHEILA
 *
 * @v parent		Parent memory region
 * @v offset		Offset within parent memory region
 * @v name		Device name
 * @ret sheila		SHEILA
 */
static BBCSHEILA * bbc_sheila_init ( MemoryRegion *parent, hwaddr offset,
				     const char *name, BBCPagedROM *paged ) {
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
	sheila->system_via = m6522_init ( &sheila->mr,
					  BBC_SHEILA_SYSTEM_VIA_BASE,
					  BBC_SHEILA_SYSTEM_VIA_SIZE,
					  "system_via", &bbc_system_via_ops,
					  bbc_irq );
	sheila->user_via = m6522_init ( &sheila->mr,
					BBC_SHEILA_USER_VIA_BASE,
					BBC_SHEILA_USER_VIA_SIZE,
					"user_via", &bbc_user_via_ops,
					bbc_irq );

	return sheila;
}

/******************************************************************************
 *
 * Machine initialisation
 *
 */

/**
 * Load ROM
 *
 * @v parent		Parent memory region
 * @v name		Name of memory region
 * @v filename		Filename
 * @v base		Base address within memory region
 * @v targphys		Base address within system memory
 * @v max_size		Maximum allowed size of ROM
 */
static void bbc_load_rom ( MemoryRegion *parent, const char *name,
			   const char *filename, hwaddr base, hwaddr targphys,
			   int max_size ) {
	MemoryRegion *rom = g_new ( MemoryRegion, 1 );
	const char *actual_filename;
	int size;

	/* Initialise memory region */
	memory_region_init_ram ( rom, name, max_size );
	vmstate_register_ram_global ( rom );
	memory_region_set_readonly ( rom, true );
	memory_region_add_subregion ( parent, base, rom );

	/* Locate ROM file */
	actual_filename = qemu_find_file ( QEMU_FILE_TYPE_BIOS, filename );
	if ( ! actual_filename ) {
		fprintf ( stderr, "qemu: could not find ROM '%s'\n",
			  filename );
		exit ( 1 );
	}

	/* Check size */
	size = load_image_targphys ( actual_filename, targphys, max_size );
	if ( size < 0 ) {
		fprintf ( stderr, "qemu: could not load (or bad size) ROM "
			  "'%s'\n", actual_filename );
		exit ( 1 );
	}
}

/**
 * Load MOS ROM
 *
 * @v default_filename	Default filename to use if none specified
 * @v base		Base address
 * @v max_size		Maximum allowed size of ROM
 */
static void bbc_load_mos ( const char *default_filename, hwaddr base,
			   int max_size ) {
	MemoryRegion *address_space_mem = get_system_memory();
	const char *name;

	/* Use default filename if no 'BIOS' name is specified */
	if ( bios_name == NULL )
		bios_name = default_filename;

	/* Load MOS ROM */
	name = g_strdup_printf ( "mos %s", bios_name );
	bbc_load_rom ( address_space_mem, name, bios_name, base, base,
		       max_size );
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
		bbc_load_rom ( &paged->roms, name, filename,
			       bbc_paged_rom_offset ( paged, page ),
			       bbc_paged_rom_targphys ( paged, page ),
			       paged->size );
	}
}

/**
 * Initialise BBC Model B
 *
 * @v args		Machine arguments
 */
static void bbcb_init ( QEMUMachineInitArgs *args ) {
	const char *cpu_model = args->cpu_model;
	CPUM6502State *env;
	MemoryRegion *address_space_mem = get_system_memory();
	MemoryRegion *ram = g_new ( MemoryRegion, 1 );
	BBCPagedROM *paged;

	/* Initialise RAM */
	memory_region_init_ram ( ram, "ram", BBC_B_RAM_SIZE );
	vmstate_register_ram_global ( ram );
	memory_region_add_subregion ( address_space_mem, BBC_B_RAM_BASE, ram );

	/* Initialise ROMs */
	bbc_load_mos ( BBC_B_MOS_FILENAME, BBC_B_MOS_BASE, BBC_B_MOS_SIZE );
	paged = bbc_paged_rom_init ( address_space_mem,
				     BBC_PAGED_ROM_BASE, BBC_PAGED_ROM_SIZE,
				     "paged_rom",
				     BBC_PAGED_ROM_VIRTUAL_BASE,
				     BBC_PAGED_ROM_VIRTUAL_BASE,
				     BBC_PAGED_ROM_COUNT );
	bbc_load_paged_roms ( paged, BBC_B_BASIC_FILENAME );

	/* Initialise interrupts */
	bbc_interrupts_init();

	/* Initialise FRED, JIM, and SHEILA */
	bbc_fred_init ( address_space_mem, BBC_FRED_BASE, "fred" );
	bbc_jim_init ( address_space_mem, BBC_JIM_BASE, "jim" );
	bbc_sheila_init ( address_space_mem, BBC_SHEILA_BASE, "sheila", paged );

	/* Initialise CPU */
	if ( cpu_model == NULL )
		cpu_model = "6502";
	env = m6502_init ( cpu_model );

	/* Register virtual machine state */
	vmstate_register ( NULL, 0, &vmstate_bbc, &bbc );
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
