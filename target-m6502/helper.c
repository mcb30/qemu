/*
 * MOS Technologies 6502 virtual CPU
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

#include "cpu.h"
#include "qemu/host-utils.h"

/** A 6502 CPU model */
struct m6502_model {
	/** Name */
	const char *name;
	/** Features */
	unsigned int features;
};

/** Known 6502 CPU models */
static const struct m6502_model m6502_model[] = {
	{
		.name = "6502",
		.features = 0,
	},
	{
		.name = "65c02",
		.features = M6502_CMOS,
	},
};

/**
 * List CPU models
 *
 * @v f			Output stream
 * @v fprintf		fprintf() function to use
 */
void m6502_list ( FILE *f, fprintf_function fprintf ) {
	const struct m6502_model *model;
	int i;

	fprintf ( f, "Available CPUs:\n" );
	for ( i = 0 ; i < ARRAY_SIZE ( m6502_model ) ; i++ ) {
		model = &m6502_model[i];
		fprintf ( f, "  %s\n", model->name );
	}
}

/**
 * Identify CPU model by name
 *
 * @v name		Model name
 * @ret model		Model, or NULL if not found
 */
static const struct m6502_model * m6502_find ( const char *name ) {
	const struct m6502_model *model;
	int i;

	for ( i = 0 ; i < ARRAY_SIZE ( m6502_model ) ; i++ ) {
		model = &m6502_model[i];
		if ( strcasecmp ( name, model->name ) == 0 )
			return model;
	}
	return NULL;
}

/**
 * Initialize CPU
 *
 * @v name		Model name
 * @ret m6502		CPU state
 */
struct m6502 * m6502_init ( const char *name ) {
	const struct m6502_model *model;
	struct m6502 *m6502;
	static int tcg_initialized = 0;

	/* Identify model */
	model = m6502_find ( name );
	if ( ! model )
		return NULL;

	/* Create CPU */
	m6502 = M6502_CHECK ( object_new ( TYPE_M6502_CPU ) );
	m6502->features = model->features;

	/* Initialise CPU */
	qemu_init_vcpu ( m6502 );

	/* Initialise TCG, if applicable */
	if ( tcg_enabled() && ! tcg_initialized ) {
		tcg_initialized = 1;
		m6502_translate_init();
	}

	return m6502;
}

/**
 * Dump CPU state
 *
 * @v m6502		CPU state
 */
void m6502_dump_state ( struct m6502 *m6502, FILE *f, fprintf_function fprintf,
			int flags ) {

	fprintf ( f, "PC=%04x A=%02x X=%02x Y=%02x S=01%02x "
		  "P=%02x(%c%c%c%c%c%c%c)\n", m6502->pc, m6502->a, m6502->x,
		  m6502->y, m6502->s, m6502->p,
		  ( ( m6502->p & P_N ) ? 'N' : 'n' ),
		  ( ( m6502->p & P_V ) ? 'V' : 'v' ),
		  ( ( m6502->p & P_B ) ? 'B' : 'b' ),
		  ( ( m6502->p & P_D ) ? 'D' : 'd' ),
		  ( ( m6502->p & P_I ) ? 'I' : 'i' ),
		  ( ( m6502->p & P_Z ) ? 'Z' : 'z' ),
		  ( ( m6502->p & P_C ) ? 'C' : 'c' ) );
}

void m6502_tlb_fill ( struct m6502 *m6502, target_ulong addr,
		      int is_write, int mmu_idx, uintptr_t retaddr ) {
	/* Do nothing */
}

void m6502_interrupt ( struct m6502 *m6502 ) {
	cpu_abort ( m6502, "Unhandled interrupt" );
}
