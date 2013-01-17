/*
 * MOS Technology 6502 virtual CPU
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
#include "exec/softmmu_exec.h"

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

	/* Dump state */
	fprintf ( f, "PC=&%04X A=&%02X X=&%02X Y=&%02X S=&%04X "
		  "P=&%02X(%c%c%c%c%c%c)\n", env->pc, env->a, env->x,
		  env->y, ( M6502_STACK_BASE + env->s ), m6502_get_p ( env ),
		  ( env->p_n ? 'N' : 'n' ),
		  ( env->p_v ? 'V' : 'v' ),
		  ( env->p_d ? 'D' : 'd' ),
		  ( env->p_i ? 'I' : 'i' ),
		  ( ( ! env->p_nz ) ? 'Z' : 'z' ),
		  ( env->p_c ? 'C' : 'c' ) );
}

/**
 * Dump stack contents
 *
 * @v env		CPU state
 */
void m6502_dump_stack ( CPUM6502State *env, FILE *f,
			fprintf_function fprintf ) {
	uint16_t stack;
	uint16_t i;

	/* Dump stack */
	stack = ( M6502_STACK_BASE + env->s );
	fprintf ( f, "PC=&%04X S=&%04X", env->pc, stack );
	for ( i = M6502_STACK_TOP ; i > stack ; i-- ) {
		fprintf ( f, " &%02X", cpu_ldub_data ( env, i ) );
	}
	fprintf ( f, "\n" );
}

void m6502_tlb_fill ( CPUM6502State *env, target_ulong addr,
		      int is_write, int mmu_idx, uintptr_t retaddr ) {

	addr &= TARGET_PAGE_MASK;
	tlb_set_page ( env, addr, addr, PAGE_BITS, mmu_idx, TARGET_PAGE_SIZE );
}

static void m6502_irq ( CPUM6502State *env, uint16_t vector ) {
	uint16_t stack;

	/* Push program counter and status register onto stack */
	stack = ( M6502_STACK_BASE + env->s );
	cpu_stw_data ( env, ( stack - 1 ), env->pc );
	cpu_stb_data ( env, ( stack - 2 ), m6502_get_p ( env ) );
	env->s = ( ( env->s - 0x03 ) & 0xff );

	/* Disable interrupts */
	env->p_i = 1;

	/* Load program counter from vector */
	env->pc = cpu_lduw_data ( env, vector );
}

void m6502_interrupt ( CPUM6502State *env ) {

	switch ( env->exception_index ) {
	case EXCP_IRQ:
		qemu_log_mask ( CPU_LOG_INT, "IRQ at &%04X\n", env->pc );
		m6502_irq ( env, M6502_IRQ_VECTOR );
		break;
	case EXCP_NMI:
		qemu_log_mask ( CPU_LOG_INT, "NMI at &%04X\n", env->pc );
		env->in_nmi = 1;
		m6502_irq ( env, M6502_NMI_VECTOR );
		break;
	default:
		cpu_abort ( env, "Unhandled exception %d\n",
			    env->exception_index );
	}
}

hwaddr m6502_get_phys_page_debug ( CPUM6502State *env, target_ulong addr ) {
    return addr & TARGET_PAGE_MASK;
}
