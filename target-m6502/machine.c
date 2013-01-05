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
		VMSTATE_UINT8 ( a, struct m6502 ),
		VMSTATE_UINT8 ( x, struct m6502 ),
		VMSTATE_UINT8 ( y, struct m6502 ),
		VMSTATE_UINT8 ( p, struct m6502 ),
		VMSTATE_UINT8 ( s, struct m6502 ),
		VMSTATE_UINT16 ( pc, struct m6502 ),
		VMSTATE_END_OF_LIST()
	},
};

void m6502_save ( QEMUFile *f, void *opaque ) {
	vmstate_save_state ( f, &vmstate_cpu, opaque );
}

int m6502_load ( QEMUFile *f, void *opaque, int version_id ) {
	return vmstate_load_state ( f, &vmstate_cpu, opaque, version_id );
}
