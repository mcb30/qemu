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
#include "loader.h"
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

	memory_region_init_io ( fred, &bbc_dummy_ops, name, "fred",
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

	memory_region_init_io ( jim, &bbc_dummy_ops, name, "jim",
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

	memory_region_init_io ( sheila, &bbc_dummy_ops, name, "sheila",
				BBC_SHEILA_SIZE );
	memory_region_add_subregion_overlap ( address_space_mem,
					      BBC_SHEILA_BASE, sheila, 1 );
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
