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

#include "hw/hw.h"

static const VMStateDescription vmstate_cpu = {
	.name = "cpu",
	.version_id = CPU_SAVE_VERSION,
	.minimum_version_id = 1,
	.minimum_version_id_old = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_UINT32 ( a, CPUM6502State ),
		VMSTATE_UINT32 ( x, CPUM6502State ),
		VMSTATE_UINT32 ( y, CPUM6502State ),
		VMSTATE_UINT32 ( p_c, CPUM6502State ),
		VMSTATE_UINT32 ( p_nz, CPUM6502State ),
		VMSTATE_UINT32 ( p_i, CPUM6502State ),
		VMSTATE_UINT32 ( p_d, CPUM6502State ),
		VMSTATE_UINT32 ( p_b, CPUM6502State ),
		VMSTATE_UINT32 ( p_u, CPUM6502State ),
		VMSTATE_UINT32 ( p_v, CPUM6502State ),
		VMSTATE_UINT32 ( p_n, CPUM6502State ),
		VMSTATE_UINT32 ( s, CPUM6502State ),
		VMSTATE_UINT32 ( pc, CPUM6502State ),
		VMSTATE_END_OF_LIST()
	},
};

void m6502_save ( QEMUFile *f, void *opaque ) {
	vmstate_save_state ( f, &vmstate_cpu, opaque );
}

int m6502_load ( QEMUFile *f, void *opaque, int version_id ) {
	return vmstate_load_state ( f, &vmstate_cpu, opaque, version_id );
}
