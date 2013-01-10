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

#ifndef QEMU_LM32_CPU_QOM_H
#define QEMU_LM32_CPU_QOM_H

#include "qom/cpu.h"
#include "cpu.h"

#define TYPE_M6502_CPU "m6502-cpu"

#define M6502_CPU_CLASS( class ) \
    OBJECT_CLASS_CHECK ( M6502CPUClass, (class), TYPE_M6502_CPU )
#define M6502_CPU( obj ) \
    OBJECT_CHECK ( M6502CPU, (obj), TYPE_M6502_CPU )
#define M6502_CPU_GET_CLASS( obj ) \
    OBJECT_GET_CLASS ( M6502CPUClass, (obj), TYPE_M6502_CPU )

/**
 * M6502CPUClass:
 * @parent_reset: The parent class' reset handler.
 *
 * A 6502 CPU model.
 */
typedef struct M6502CPUClass {
    /*< private >*/
    CPUClass parent_class;
    /*< public >*/
    void (*parent_reset)(CPUState *cpu);
} M6502CPUClass;

/**
 * M6502CPU:
 * @env: #CPUM6502State
 *
 * A 6502 CPU.
 */
typedef struct M6502CPU {
    /*< private >*/
    CPUState parent_obj;
    /*< public >*/
    CPUM6502State env;
} M6502CPU;

static inline M6502CPU * m6502_env_get_cpu ( CPUM6502State *env ) {
	return M6502_CPU ( container_of ( env, M6502CPU, env ) );
}

#define ENV_GET_CPU( env ) CPU ( m6502_env_get_cpu ( env ) )

#endif
