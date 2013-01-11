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
#include "mc6850.h"
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
static void bbc_init_interrupts ( void ) {

	/* Allocate IRQ and NMI interrupts */
	bbc_irq = qemu_allocate_irqs ( bbc_irq_handler, NULL, 1 )[0];
	bbc_nmi = qemu_allocate_irqs ( bbc_nmi_handler, NULL, 1 )[0];
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

	/* Initialise serial system */
	mc6850_init ( sheila, BBC_SHEILA_SERIAL, "bbc.serial", serial_hds[0] );

	/* Initialise system and user VIAs */
	m6522_init ( sheila, BBC_SHEILA_SYSTEM_VIA, "bbc.system_via",
		     &bbc_system_via_ops, bbc_irq );
	m6522_init ( sheila, BBC_SHEILA_USER_VIA, "bbc.user_via",
		     &bbc_user_via_ops, bbc_irq );
}

/******************************************************************************
 *
 * Machine initialisation
 *
 */

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
