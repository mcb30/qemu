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
#include "disas/disas.h"
#include "helper.h"
#include "tcg-op.h"

#define GEN_HELPER 1
#include "helper.h"

void m6502_translate_init ( void ) {
}

void m6502_gen_intermediate_code ( CPUM6502State *env,
				   struct TranslationBlock *tb ) {
}

void m6502_gen_intermediate_code_pc ( CPUM6502State *env,
				      struct TranslationBlock *tb ) {
}

void m6502_restore_state_to_opc ( CPUM6502State *env,
				  TranslationBlock *tb, int pc_pos ) {
    env->pc = tcg_ctx.gen_opc_pc[pc_pos];
}
