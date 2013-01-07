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
typedef struct {
	/** Name */
	const char *name;
	/** Features */
	unsigned int features;
} M6502Model;

/** Known 6502 CPU models */
static const M6502Model m6502_model[] = {
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
	const M6502Model *model;
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
static const M6502Model * m6502_find ( const char *name ) {
	const M6502Model *model;
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
 * @ret env		CPU state
 */
CPUM6502State * m6502_init ( const char *name ) {
	const M6502Model *model;
	M6502CPU *cpu;
	CPUM6502State *env;
	static int tcg_initialized = 0;

	/* Identify model */
	model = m6502_find ( name );
	if ( ! model )
		return NULL;

	/* Create CPU */
	cpu = M6502_CPU ( object_new ( TYPE_M6502_CPU ) );
	env = &cpu->env;
	env->features = model->features;

	/* Initialise CPU */
	qemu_init_vcpu ( env );

	/* Initialise TCG, if applicable */
	if ( tcg_enabled() && ! tcg_initialized ) {
		tcg_initialized = 1;
		m6502_translate_init();
	}

	return env;
}

/**
 * Dump CPU state
 *
 * @v env		CPU state
 */
void m6502_dump_state ( CPUM6502State *env, FILE *f, fprintf_function fprintf,
			int flags ) {

	fprintf ( f, "PC=%04x A=%02x X=%02x Y=%02x S=01%02x "
		  "P=%02x(%c%c%c%c%c%c%c)\n", env->pc, env->a, env->x,
		  env->y, env->s, env->p,
		  ( ( env->p & P_N ) ? 'N' : 'n' ),
		  ( ( env->p & P_V ) ? 'V' : 'v' ),
		  ( ( env->p & P_B ) ? 'B' : 'b' ),
		  ( ( env->p & P_D ) ? 'D' : 'd' ),
		  ( ( env->p & P_I ) ? 'I' : 'i' ),
		  ( ( env->p & P_Z ) ? 'Z' : 'z' ),
		  ( ( env->p & P_C ) ? 'C' : 'c' ) );
}

void m6502_tlb_fill ( CPUM6502State *env, target_ulong addr,
		      int is_write, int mmu_idx, uintptr_t retaddr ) {
	/* Do nothing */
}

void m6502_interrupt ( CPUM6502State *env ) {
	cpu_abort ( env, "Unhandled interrupt" );
}

hwaddr m6502_get_phys_page_debug ( CPUM6502State *env, target_ulong addr ) {
    return addr & TARGET_PAGE_MASK;
}

#if !defined(CONFIG_USER_ONLY)
#define MMUSUFFIX _mmu
#define SHIFT 0
#include "exec/softmmu_template.h"
#define SHIFT 1
#include "exec/softmmu_template.h"
#define SHIFT 2
#include "exec/softmmu_template.h"
#define SHIFT 3
#include "exec/softmmu_template.h"
#endif
