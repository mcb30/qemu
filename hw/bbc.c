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
	/** Paged ROM select register */
	uint8_t paged_rom;
} BBCState;

/** System state description */
static const VMStateDescription vmstate_bbc = {
	.name = "bbc",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_UINT8 ( addressable_latch, BBCState ),
		VMSTATE_UINT8 ( paged_rom, BBCState ),
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
static void bbc_init_interrupts ( void ) {

	/* Allocate IRQ and NMI interrupts */
	bbc_irq = qemu_allocate_irqs ( bbc_irq_handler, NULL, 1 )[0];
	bbc_nmi = qemu_allocate_irqs ( bbc_nmi_handler, NULL, 1 )[0];
}

/******************************************************************************
 *
 * Paged ROMs
 *
 */

/** Paged ROM memory region */
static MemoryRegion bbc_paged_rom_mr;

/**
 * Read from paged ROM select register
 *
 * @v opaque		Opaque pointer
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
 * @v opaque		Opaque pointer
 * @v addr		Register address
 * @v data64		Data to write
 * @v size		Size of write
 */
static void bbc_paged_rom_select_write ( void *opaque, hwaddr addr,
					 uint64_t data, unsigned int size ) {
	unsigned int page;

	/* Change offset address into paged ROM virtual memory region */
	page = ( data & ( BBC_PAGED_ROM_COUNT - 1 ) );
	memory_region_set_alias_offset ( &bbc_paged_rom_mr,
					 ( page * BBC_PAGED_ROM_SIZE ) );
}

/** Paged ROM select register operations */
static const MemoryRegionOps bbc_paged_rom_select_ops = {
	.read = bbc_paged_rom_select_read,
	.write = bbc_paged_rom_select_write,
};

/**
 * Initialise paged ROM select register
 *
 * @v parent		Parent memory region
 * @v offset		Offset within parent memory region
 * @v size		Size of memory region
 * @v name		Device name
 */
static void bbc_paged_rom_select_init ( MemoryRegion *parent, hwaddr offset,
					hwaddr size, const char *name ) {
	MemoryRegion *mr = g_new ( MemoryRegion, 1 );

	/* Register memory region */
	memory_region_init_io ( mr, &bbc_paged_rom_select_ops, NULL, name,
				size );
	memory_region_add_subregion ( parent, offset, mr );
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
 * System VIA (SHEILA &40-&4f)
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
 * User VIA (SHEILA &60-&6f)
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

/**
 * Dummy read from unimplemented I/O region
 *
 * @v opaque		Region name
 * @v addr		Address within region
 * @v size		Size of read
 * @ret data		Read data
 */
static uint64_t bbc_dummy_read ( void *opaque, hwaddr addr,
				 unsigned int size ) {
	char *name = opaque;

	qemu_log_mask ( LOG_UNIMP, "%s: unimplemented read from &%02lX\n",
			name, addr );
	return 0;
}

/**
 * Dummy write to unimplemented I/O region
 *
 * @v opaque		Region name
 * @v addr		Address within region
 * @v data		Data to write
 * @v size		Size of write
 */
static void bbc_dummy_write ( void *opaque, hwaddr addr, uint64_t data,
			      unsigned int size ) {
	char *name = opaque;

	qemu_log_mask ( LOG_UNIMP, "%s: unimplemented write to &%02lX\n",
			name, addr );
}

/** Dummy I/O operations */
static const MemoryRegionOps bbc_dummy_ops = {
	.read = bbc_dummy_read,
	.write = bbc_dummy_write,
};

/**
 * Initialise FRED
 *
 */
static void bbc_init_fred ( void ) {
	MemoryRegion *address_space_mem = get_system_memory();
	MemoryRegion *fred = g_new ( MemoryRegion, 1 );
	static char name[] = "FRED";

	/* Set up FRED as a dummy I/O region */
	memory_region_init_io ( fred, &bbc_dummy_ops, name, "bbc.fred",
				BBC_FRED_SIZE );
	memory_region_add_subregion_overlap ( address_space_mem, BBC_FRED_BASE,
					      fred, 1 );
}

/**
 * Initialise JIM
 *
 */
static void bbc_init_jim ( void ) {
	MemoryRegion *address_space_mem = get_system_memory();
	MemoryRegion *jim = g_new ( MemoryRegion, 1 );
	static char name[] = "JIM";

	/* Set up JIM as a dummy I/O region */
	memory_region_init_io ( jim, &bbc_dummy_ops, name, "bbc.jim",
				BBC_JIM_SIZE );
	memory_region_add_subregion_overlap ( address_space_mem, BBC_JIM_BASE,
					      jim, 1 );
}

/**
 * Initialise SHEILA
 *
 */
static void bbc_init_sheila ( void ) {
	MemoryRegion *address_space_mem = get_system_memory();
	MemoryRegion *sheila = g_new ( MemoryRegion, 1 );
	static char name[] = "SHEILA";

	/* Set up SHEILA as a dummy I/O region */
	memory_region_init_io ( sheila, &bbc_dummy_ops, name, "bbc.sheila",
				BBC_SHEILA_SIZE );
	memory_region_add_subregion_overlap ( address_space_mem,
					      BBC_SHEILA_BASE, sheila, 1 );

	/* Initialise CRTC */
	mc6845_init ( sheila, BBC_SHEILA_CRTC_BASE, BBC_SHEILA_CRTC_SIZE,
		      "bbc.crtc" );

	/* Initialise serial system */
	mc6850_init ( sheila, BBC_SHEILA_ACIA_BASE, BBC_SHEILA_ACIA_SIZE,
		      "bbc.acia", serial_hds[0] );

	/* Initialise paged ROM select register */
	bbc_paged_rom_select_init ( sheila, BBC_SHEILA_PAGED_ROM_SELECT_BASE,
				    BBC_SHEILA_PAGED_ROM_SELECT_SIZE,
				    "bbc.paged_rom_select" );

	/* Initialise system and user VIAs */
	m6522_init ( sheila, BBC_SHEILA_SYSTEM_VIA_BASE,
		     BBC_SHEILA_SYSTEM_VIA_SIZE, "bbc.system_via",
		     &bbc_system_via_ops, bbc_irq );
	m6522_init ( sheila, BBC_SHEILA_USER_VIA_BASE,
		     BBC_SHEILA_USER_VIA_SIZE, "bbc.user_via",
		     &bbc_user_via_ops, bbc_irq );
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
	name = g_strdup_printf ( "bbc.mos %s", bios_name );
	bbc_load_rom ( address_space_mem, name, bios_name, base, base,
		       max_size );
}

/**
 * Load paged ROMs
 *
 * @v basic_filename	Filename for BASIC ROM (always loaded)
 */
static void bbc_load_paged_roms ( const char *basic_filename ) {
	MemoryRegion *address_space_mem = get_system_memory();
	MemoryRegion *roms = g_new ( MemoryRegion, 1 );
	unsigned int i;
	unsigned int page;
	const char *filename;
	const char *name;

	/* Initialise paged ROM virtual memory region.  There is no
	 * way for the CPU to access this region directly, but it
	 * gives us an address to pass to load_image_targphys().
	 */
	memory_region_init ( roms, "bbc.roms",
			     ( BBC_PAGED_ROM_COUNT * BBC_PAGED_ROM_SIZE ) );
	memory_region_set_readonly ( roms, true );
	memory_region_add_subregion ( address_space_mem,
				      BBC_PAGED_ROM_VIRTUAL_BASE, roms );

	/* Initialise paged ROM memory region */
	memory_region_init_alias ( &bbc_paged_rom_mr, "bbc.paged_rom", roms,
				   0, BBC_PAGED_ROM_SIZE );
	memory_region_set_readonly ( &bbc_paged_rom_mr, true );
	memory_region_add_subregion ( address_space_mem, BBC_PAGED_ROM_BASE,
				      &bbc_paged_rom_mr );

	/* Load any specified option ROMs plus the BASIC ROM */
	for ( i = 0 ; ( ( i < BBC_PAGED_ROM_COUNT ) &&
			( i < ( nb_option_roms + 1 ) ) ) ; i++ ) {
		page = ( BBC_PAGED_ROM_COUNT - i - 1 );
		filename = ( ( i == nb_option_roms ) ? basic_filename :
			     option_rom[0].name );
		name = g_strdup_printf ( "bbc.rom%d %s", page, filename );
		bbc_load_rom ( roms, name, filename,
			       ( page * BBC_PAGED_ROM_SIZE ),
			       ( BBC_PAGED_ROM_VIRTUAL_BASE +
				 ( page * BBC_PAGED_ROM_SIZE ) ),
			       BBC_PAGED_ROM_SIZE );
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

	/* Initialise RAM */
	memory_region_init_ram ( ram, "bbc.ram", BBC_B_RAM_SIZE );
	vmstate_register_ram_global ( ram );
	memory_region_add_subregion ( address_space_mem, BBC_B_RAM_BASE, ram );

	/* Initialise ROM */
	bbc_load_mos ( BBC_B_MOS_FILENAME, BBC_B_MOS_BASE, BBC_B_MOS_SIZE );
	bbc_load_paged_roms ( BBC_B_BASIC_FILENAME );

	/* Initialise interrupts */
	bbc_init_interrupts();

	/* Initialise FRED, JIM, and SHEILA */
	bbc_init_fred();
	bbc_init_jim();
	bbc_init_sheila();

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
