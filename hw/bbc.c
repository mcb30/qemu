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
#include "m6522.h"
#include "bbc.h"

/**
 * Load MOS ROM
 *
 * @v default_bios_name	Default MOS name to use if none specified
 * @v base		Base address
 * @v expected_size	Expected size of MOS ROM
 */
static void bbc_load_mos ( const char *default_bios_name, hwaddr base,
			   int expected_size ) {
	MemoryRegion *address_space_mem = get_system_memory();
	MemoryRegion *mos = g_new ( MemoryRegion, 1 );
	const char *filename;
	int size;

	/* Initialise memory region */
	memory_region_init_ram ( mos, "bbc.mos", expected_size );
	vmstate_register_ram_global ( mos );
	memory_region_set_readonly ( mos, true );
	memory_region_add_subregion ( address_space_mem, base, mos );

	/* Locate MOS file */
	if ( bios_name == NULL )
		bios_name = default_bios_name;
	filename = qemu_find_file ( QEMU_FILE_TYPE_BIOS, bios_name );
	if ( ! filename ) {
		fprintf ( stderr, "qemu: could not find MOS '%s'\n",
			  bios_name );
		exit ( 1 );
	}

	/* Check size */
	size = load_image_targphys ( filename, base, expected_size );
	if ( size != expected_size ) {
		fprintf ( stderr, "qemu: could not load (or bad size) MOS "
			  "'%s'\n", filename );
		exit ( 1 );
	}
}

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

	memory_region_init_io ( jim, &bbc_dummy_ops, name, "bbc.jim",
				BBC_JIM_SIZE );
	memory_region_add_subregion_overlap ( address_space_mem, BBC_JIM_BASE,
					      jim, 1 );
}

/** Slow data bus */
static uint8_t bbc_slow_data;

/** Addressable latch */
static uint8_t bbc_addressable_latch;

/**
 * Check if CAPS LOCK is enabled
 *
 * @ret caps_lock	CAPS LOCK is enabled
 */
static inline int bbc_caps_lock ( void ) {

	return ( bbc_addressable_latch & ( 1 << BBC_LATCH_CAPS_LOCK ) );
}

/**
 * Check if SHIFT LOCK is enabled
 *
 * @ret shift_lock	SHIFT LOCK is enabled
 */
static inline int bbc_shift_lock ( void ) {

	return ( bbc_addressable_latch & ( 1 << BBC_LATCH_SHIFT_LOCK ) );
}

/**
 * Update keyboard LEDs
 *
 */
static void bbc_keyboard_leds ( void ) {
	int ledstate;

	/* Update LEDs based on control bits in addressable latch */
	ledstate = ( ( bbc_caps_lock() ? QEMU_CAPS_LOCK_LED : 0 ) |
		     ( bbc_shift_lock() ? QEMU_NUM_LOCK_LED : 0 ) );
	kbd_put_ledstate ( ledstate );
}

/**
 * Check if currently-selected key is pressed
 *
 * @ret pressed		Key is pressed
 */
static int bbc_keyboard_pressed ( void ) {
	unsigned int row;
	unsigned int column;

	/* Calculate row (PA6:4) and column (PA3:0) addresses */
	row = ( ( bbc_slow_data >> 4 ) & 0x07 );
	column = ( ( bbc_slow_data >> 0 ) & 0x0f );

	printf ( "Reading key (%d,%d)\n", column, row );

	return 0;
}

/**
 * Write to 76489 sound chip
 *
 * 
 */
static void bbc_sound_write ( void ) {

	qemu_log_mask ( LOG_UNIMP, "BBC: unimplemented sound write &%02x\n",
			bbc_slow_data );
}

/**
 * Write to slow data bus (system VIA port A)
 *
 * @v via		6522 VIA
 * @v port		Port
 * @v data		Output data
 */
static void bbc_system_via_output_a ( M6522VIA *via, M6522VIAPort *port,
				      uint8_t data ) {

	/* Store data written to slow data bus */
	bbc_slow_data = data;
}

/**
 * Read from slow data bus (system VIA port A)
 *
 * @v via		6522 VIA
 * @v port		Port
 * @ret data		Input data
 */
static uint8_t bbc_system_via_input_a ( M6522VIA *via, M6522VIAPort *port ) {
	uint8_t data;

	/* Set data equal to outputs for all pins configured as outputs */
	data = ( port->or & port->ddr );

	//
	printf ( "Reading from PA\n" );

	/* Read from keyboard into PA7 if keyboard is enabled */
	if ( ! ( bbc_addressable_latch & ( 1 << BBC_LATCH_KB_WE ) ) ) {
		data &= ( 1 << 7 );
		if ( bbc_keyboard_pressed() )
			data |= ( 1 << 7 );
	}

	return data;
}

/**
 * Write to addressable latch (system VIA port B)
 *
 * @v via		6522 VIA
 * @v port		Port
 * @v data		Output data
 */
static void bbc_system_via_output_b ( M6522VIA *via, M6522VIAPort *port,
				      uint8_t data ) {
	unsigned int latch_address;
	unsigned int latch_data;

	/* Update addressable latch stored value */
	latch_address = ( ( data >> 0 ) & 0x07 );
	latch_data = ( ( data >> 3 ) & 0x01 );
	bbc_addressable_latch &= ~( 1 << latch_address );
	bbc_addressable_latch |= ( latch_data << latch_address );

	/* Handle write events */
	switch ( latch_address ) {
	case BBC_LATCH_SOUND_WE:
		if ( ! latch_data )
			bbc_sound_write();
		break;
	case BBC_LATCH_CAPS_LOCK:
	case BBC_LATCH_SHIFT_LOCK:
		bbc_keyboard_leds();
		break;
	}
}

/** System VIA */
static M6522VIA bbc_system_via = {
	.name = "bbc.sheila.system_via",
	.b = {
		.output = bbc_system_via_output_b,
	},
	.a = {
		.output = bbc_system_via_output_a,
		.input = bbc_system_via_input_a,
	},
};

/**
 * Initialise SHEILA
 *
 */
static void bbc_init_sheila ( void ) {
	MemoryRegion *address_space_mem = get_system_memory();
	MemoryRegion *sheila = g_new ( MemoryRegion, 1 );
	static char name[] = "SHEILA";

	memory_region_init_io ( sheila, &bbc_dummy_ops, name, "bbc.sheila",
				BBC_SHEILA_SIZE );
	memory_region_add_subregion_overlap ( address_space_mem,
					      BBC_SHEILA_BASE, sheila, 1 );

	m6522_init ( &bbc_system_via, address_space_mem,
		     ( BBC_SHEILA_BASE + BBC_SHEILA_SYSTEM_VIA ), 2 );
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

	/* Initialise MOS ROM */
	bbc_load_mos ( BBC_B_MOS_NAME, BBC_B_MOS_BASE, BBC_B_MOS_SIZE );

	/* Initialise FRED, JIM, and SHEILA */
	bbc_init_fred();
	bbc_init_jim();
	bbc_init_sheila();

	/* Initialise CPU */
	if ( cpu_model == NULL )
		cpu_model = "6502";
	env = m6502_init ( cpu_model );
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
