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

static void m6502_reset ( CPUState *cpu ) {
	struct m6502 *m6502 = M6502_CHECK ( container_of ( cpu, struct m6502,
							   parent ) );
	struct m6502_class *class = M6502_CLASS ( m6502 );

	/* Call parent reset() method */
	class->parent_reset ( cpu );

	/* Reset CPU */
	m6502->a = 0;
	m6502->x = 0;
	m6502->y = 0;
	m6502->p = 0;
	m6502->s = 0;
	m6502->pc = 0;
}

static void m6502_instance_init ( Object *obj ) {
	struct m6502 *m6502 = M6502_CHECK ( obj );

	cpu_exec_init ( m6502 );
	cpu_reset ( CPU ( m6502 ) );
}

static void m6502_class_init ( ObjectClass *oc, void *data ) {
	struct m6502_class *class = M6502_CLASS_CHECK ( oc );
	CPUClass *cc = CPU_CLASS ( oc );

	/* Intercept reset() method */
	class->parent_reset = cc->reset;
	cc->reset = m6502_reset;
}

static const TypeInfo m6502_type_info = {
	.name = TYPE_M6502_CPU,
	.parent = TYPE_CPU,
	.instance_size = sizeof ( struct m6502 ),
	.instance_init = m6502_instance_init,
	.abstract = false,
	.class_size = sizeof ( struct m6502_class ),
	.class_init = m6502_class_init,
};

static void m6502_register_types ( void ) {
	type_register_static ( &m6502_type_info );
}

type_init ( m6502_register_types );
