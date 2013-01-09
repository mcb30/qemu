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
#include "helper.h"

#if ! defined ( CONFIG_USER_ONLY )
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

void helper_hlt ( CPUM6502State *env ) {

	env->halted = 1;
	env->exception_index = EXCP_HLT;
	cpu_loop_exit ( env );
}

uint32_t helper_get_p ( CPUM6502State *env ) {
	uint8_t p = 0;
	unsigned int i;

	/* Assemble status register value */
	for ( i = 0 ; i < 8 ; i++ )
		p |= ( ( !! env->p[i] ) << i );

	return p;
}
