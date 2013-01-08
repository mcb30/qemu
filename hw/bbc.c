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

	/* Initialise CPU */
	if ( cpu_model == NULL )
		cpu_model = "6502";
	env = m6502_init ( cpu_model );
}

static QEMUMachine bbc_model_b = {
	.name = "bbcb",
	.desc = "Acorn BBC Micro Model B",
	.init = bbcb_init,
	.is_default = 1,
};

static void bbc_machine_init ( void ) {
	qemu_register_machine ( &bbc_model_b );
}

machine_init ( bbc_machine_init );
