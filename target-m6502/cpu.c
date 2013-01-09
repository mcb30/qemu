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
#include "qemu-common.h"
#include "hw/loader.h"

static void m6502_reset ( CPUState *s ) {
	M6502CPU *cpu = M6502_CPU ( s );
	M6502CPUClass *mcc = M6502_CPU_GET_CLASS ( cpu );
	CPUM6502State *env = &cpu->env;
	uint16_t *reset;

	/* Call parent reset() method */
	mcc->parent_reset ( s );

	/* Reset CPU */
	env->a = 0;
	env->x = 0;
	env->y = 0;
	memset ( env->p, 0, sizeof ( env->p ) );
	env->p[P_I] = 1;
	env->s = 0;
	env->pc = 0;
	tlb_flush ( env, 1 );

	/* Load initial program counter from reset vector */
	reset = rom_ptr ( M6502_RESET_VECTOR );
	if ( ! reset ) {
		fprintf ( stderr, "No ROM present to provide reset vector\n" );
		exit ( 1 );
	}
	env->pc = *reset;
}

static void m6502_instance_init ( Object *obj ) {
	M6502CPU *cpu = M6502_CPU ( obj );
	CPUM6502State *env = &cpu->env;

	cpu_exec_init ( env );
	cpu_reset ( CPU ( cpu ) );
}

static void m6502_class_init ( ObjectClass *oc, void *data ) {
	M6502CPUClass *mcc = M6502_CPU_CLASS ( oc );
	CPUClass *cc = CPU_CLASS ( oc );

	/* Intercept reset() method */
	mcc->parent_reset = cc->reset;
	cc->reset = m6502_reset;
}

static const TypeInfo m6502_type_info = {
	.name = TYPE_M6502_CPU,
	.parent = TYPE_CPU,
	.instance_size = sizeof ( M6502CPU ),
	.instance_init = m6502_instance_init,
	.abstract = false,
	.class_size = sizeof ( M6502CPUClass ),
	.class_init = m6502_class_init,
};

static void m6502_register_types ( void ) {
	type_register_static ( &m6502_type_info );
}

type_init ( m6502_register_types );
