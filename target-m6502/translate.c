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

#define LOG_DIS( ... ) qemu_log_mask ( CPU_LOG_TB_IN_ASM, ## __VA_ARGS__ )

#define MEM_INDEX 0

typedef struct {
	/** TCG variable */
	TCGv_i32 var;
	/** Name (as used in disassembly) */
	char name[2];
} M6502Register;

static TCGv_ptr cpu_env;
static M6502Register cpu_a = { .name = "A" };
static M6502Register cpu_x = { .name = "X" };
static M6502Register cpu_y = { .name = "Y" };
static M6502Register cpu_p = { .name = "P" }; /* Dummy register for PHP, PLP */
static M6502Register cpu_p_c = { .name = "C" };
static M6502Register cpu_p_z = { .name = "Z" };
static M6502Register cpu_p_i = { .name = "I" };
static M6502Register cpu_p_d = { .name = "D" };
static M6502Register cpu_p_b = { .name = "B" };
static M6502Register cpu_p_u = { .name = "U" };
static M6502Register cpu_p_v = { .name = "V" };
static M6502Register cpu_p_n = { .name = "N" };
static M6502Register cpu_s = { .name = "S" };
static TCGv_i32 cpu_pc;

#include "exec/gen-icount.h"

typedef struct DisasContext DisasContext;

/** An instruction definition */
typedef struct {
	/** Generate instruction */
	void ( * gen ) ( DisasContext *dc );
	/** Destination register (if any) */
	M6502Register *dest;
	/** Source register (if any) */
	M6502Register *src;
	/** Memory addressing mode (if any) */
	void ( * mem ) ( DisasContext *dc );
	/** Length of instruction */
	size_t len;
} M6502Instruction;

struct DisasContext {
	CPUM6502State *env;
	uint32_t pc;
	TranslationBlock *tb;

	const M6502Instruction *insn;
	TCGv_i32 address;
	char address_desc[16];

	int is_jmp;
};

/* Generate address description */
static void m6502_address_desc ( DisasContext *dc, const char *fmt, ... ) {
	va_list args;

	if ( qemu_loglevel_mask ( CPU_LOG_TB_IN_ASM ) ) {
		va_start ( args, fmt );
		vsnprintf ( dc->address_desc, sizeof ( dc->address_desc ),
			    fmt, args );
		va_end ( args );
	}
}

/* Absolute addressing mode */
static void m6502_abs ( DisasContext *dc ) {
	uint16_t base = cpu_lduw_code ( dc->env, ( dc->pc + 1 ) );

	/* Generate address */
	dc->address = tcg_const_i32 ( base );

	/* Generate address description for disassembly */
	m6502_address_desc ( dc, "&%04X", base );
}

/* Absolute,X addressing mode */
static void m6502_abs_x ( DisasContext *dc ) {
	uint16_t base = cpu_lduw_code ( dc->env, ( dc->pc + 1 ) );

	/* Generate address */
	dc->address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( dc->address, cpu_x.var, base );
	tcg_gen_andi_i32 ( dc->address, dc->address, M6502_ADDRESS_MASK );

	/* Generate address description for disassembly */
	m6502_address_desc ( dc, "&%04X,X", base );
}

/* Absolute,Y addressing mode */
static void m6502_abs_y ( DisasContext *dc ) {
	uint16_t base = cpu_lduw_code ( dc->env, ( dc->pc + 1 ) );

	/* Generate address */
	dc->address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( dc->address, cpu_y.var, base );
	tcg_gen_andi_i32 ( dc->address, dc->address, M6502_ADDRESS_MASK );

	/* Generate address description for disassembly */
	m6502_address_desc ( dc, "&%04X,Y", base );
}

/* Zero-page addressing mode */
static void m6502_zero ( DisasContext *dc ) {
	uint8_t base = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	/* Generate address */
	dc->address = tcg_const_i32 ( base );

	/* Generate address description for disassembly */
	m6502_address_desc ( dc, "&%02X", base );
}

/* Zero-page,X addressing mode */
static void m6502_zero_x ( DisasContext *dc ) {
	uint8_t base = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	/* Generate address */
	dc->address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( dc->address, cpu_x.var, base );
	tcg_gen_andi_i32 ( dc->address, dc->address, M6502_ZERO_PAGE_MASK );

	/* Generate address description for disassembly */
	m6502_address_desc ( dc, "&%02X,X", base );
}

/* Zero-page,Y addressing mode */
static void m6502_zero_y ( DisasContext *dc ) {
	uint8_t base = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	/* Generate address */
	dc->address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( dc->address, cpu_y.var, base );
	tcg_gen_andi_i32 ( dc->address, dc->address, M6502_ZERO_PAGE_MASK );

	/* Generate address description for disassembly */
	m6502_address_desc ( dc, "&%02X,Y", base );
}

/* (Indirect,X) addressing mode */
static void m6502_ind_x ( DisasContext *dc ) {
	uint8_t base = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	/* Generate address */
	dc->address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( dc->address, cpu_x.var, base );
	tcg_gen_andi_i32 ( dc->address, dc->address, M6502_ZERO_PAGE_MASK );
	tcg_gen_qemu_ld16u ( dc->address, dc->address, MEM_INDEX );

	/* Generate address description for disassembly */
	m6502_address_desc ( dc, "(&%02X,X)", base );
}

/* (Indirect),Y addressing mode */
static void m6502_ind_y ( DisasContext *dc ) {
	uint8_t base = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	/* Generate address */
	dc->address = tcg_const_i32 ( base );
	tcg_gen_qemu_ld16u ( dc->address, dc->address, MEM_INDEX );
	tcg_gen_add_i32 ( dc->address, dc->address, cpu_y.var );

	/* Generate address description for disassembly */
	m6502_address_desc ( dc, "(&%02X),Y", base );
}

/* LDA, LDX, LDY (immediate) */
static void m6502_gen_load_imm ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;
	uint8_t value;

	/* Load register */
	value = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );
	tcg_gen_movi_i32 ( dest->var, value );
	tcg_gen_movi_i32 ( cpu_p_z.var, ( ( value == 0 ) ? 1 : 0 ) );
	tcg_gen_movi_i32 ( cpu_p_n.var, ( ( value & 0x80 ) ? 1 : 0 ) );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : LD%s #&%02X\n", dc->pc, dest->name, value );
}

/* LDA, LDX, LDY (memory) */
static void m6502_gen_load ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;

	/* Load register */
	tcg_gen_qemu_ld8u ( dest->var, dc->address, MEM_INDEX );
	tcg_gen_setcondi_i32 ( TCG_COND_EQ, cpu_p_z.var, dest->var, 0 );
	tcg_gen_andi_i32 ( cpu_p_n.var, dest->var, 0x80 );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : LD%s %s\n", dc->pc, dest->name, dc->address_desc );
}

/* STA, STX, STY */
static void m6502_gen_store ( DisasContext *dc ) {
	M6502Register *src = dc->insn->src;

	/* Store register */
	tcg_gen_qemu_st8 ( src->var, dc->address, MEM_INDEX );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : ST%s %s\n", dc->pc, src->name, dc->address_desc );
}

/* TAX, TAY, TSX, TXA, TXS, TYA */
static void m6502_gen_transfer ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;
	M6502Register *src = dc->insn->src;

	/* Transfer register */
	tcg_gen_mov_i32 ( dest->var, src->var );
	tcg_gen_setcondi_i32 ( TCG_COND_EQ, cpu_p_z.var, dest->var, 0 );
	tcg_gen_andi_i32 ( cpu_p_n.var, dest->var, 0x80 );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : T%s%s\n", dc->pc, src->name, dest->name );
}

/* CLC, CLD, CLI, CLV */
static void m6502_gen_clear ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;

	/* Set flag */
	tcg_gen_movi_i32 ( dest->var, 0 );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : CL%s\n", dc->pc, dest->name );
}

/* SEC, SED, SEI */
static void m6502_gen_set ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;

	/* Set flag */
	tcg_gen_movi_i32 ( dest->var, 1 );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : SE%s\n", dc->pc, dest->name );
}

/**
 * Load value for read/modify/write instruction
 *
 * @v dc		Disassembly context
 * @ret value		Value (must be passed to m6502_rmw_store())
 */
static TCGv_i32 m6502_rmw_load ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;
	TCGv_i32 value;

	/* Load value from memory if applicable */
	if ( dc->insn->mem ) {
		value = tcg_temp_new_i32();
		tcg_gen_qemu_ld8u ( value, dc->address, MEM_INDEX );
	} else {
		value = dest->var;
	}

	return value;
}

/**
 * Store value for read/modify/write instruction
 *
 * @v dc		Disassembly context
 * @v value		Value (created by m6502_rmw_load())
 */
static void m6502_rmw_store ( DisasContext *dc, TCGv_i32 value ) {

	/* Store value back to memory if applicable */
	if ( dc->insn->mem ) {
		tcg_gen_qemu_st8 ( value, dc->address, MEM_INDEX );
		tcg_temp_free_i32 ( value );
	}
}

/* ASL */
static void m6502_gen_asl ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;
	TCGv_i32 value;

	/* Perform shift */
	value = m6502_rmw_load ( dc );
	tcg_gen_andi_i32 ( cpu_p_c.var, value, 0x80 );
	tcg_gen_shli_i32 ( value, value, 1 );
	tcg_gen_andi_i32 ( value, value, 0xff );
	tcg_gen_setcondi_i32 ( TCG_COND_EQ, cpu_p_z.var, dest->var, 0 );
	tcg_gen_andi_i32 ( cpu_p_n.var, value, 0x80 );
	m6502_rmw_store ( dc, value );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : ASL%s%s\n", dc->pc,
		  ( dc->insn->mem ? " " : "" ),
		  ( dc->insn->mem ? dc->address_desc : "" ) );
}

/* INC, INX, INY */
static void m6502_gen_inc ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;
	TCGv_i32 value;

	/* Perform increment */
	value = m6502_rmw_load ( dc );
	tcg_gen_addi_i32 ( value, value, 1 );
	tcg_gen_andi_i32 ( value, value, 0xff );
	tcg_gen_setcondi_i32 ( TCG_COND_EQ, cpu_p_z.var, value, 0 );
	tcg_gen_andi_i32 ( cpu_p_n.var, value, 0x80 );
	m6502_rmw_store ( dc, value );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : IN%s%s%s\n", dc->pc,
		  ( dc->insn->mem ? "C" : dest->name ),
		  ( dc->insn->mem ? " " : "" ),
		  ( dc->insn->mem ? dc->address_desc : "" ) );
}

/* DEC, DEX, DEY */
static void m6502_gen_dec ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;
	TCGv_i32 value;

	/* Perform increment */
	value = m6502_rmw_load ( dc );
	tcg_gen_subi_i32 ( value, value, 1 );
	tcg_gen_andi_i32 ( value, value, 0xff );
	tcg_gen_setcondi_i32 ( TCG_COND_EQ, cpu_p_z.var, value, 0 );
	tcg_gen_andi_i32 ( cpu_p_n.var, value, 0x80 );
	m6502_rmw_store ( dc, value );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : DE%s%s%s\n", dc->pc,
		  ( dc->insn->mem ? "C" : dest->name ),
		  ( dc->insn->mem ? " " : "" ),
		  ( dc->insn->mem ? dc->address_desc : "" ) );
}

/* PHA, PHP */
static void m6502_gen_push ( DisasContext *dc ) {
	M6502Register *src = dc->insn->src;
	TCGv_i32 address;

	/* Synthesise P value, if applicable */
	if ( src == &cpu_p ) {
		cpu_p.var = tcg_temp_new_i32();
		gen_helper_get_p ( cpu_p.var, cpu_env );
	}

	/* Store value at current stack address */
	address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( address, cpu_s.var, M6502_STACK_BASE );
	tcg_gen_qemu_st8 ( src->var, address, MEM_INDEX );
	tcg_temp_free_i32 ( address );

	/* Decrement stack pointer (with wrap-around) */
	tcg_gen_subi_i32 ( cpu_s.var, cpu_s.var, 1 );
	tcg_gen_andi_i32 ( cpu_s.var, cpu_s.var, 0xff );

	/* Free temporary P value, if applicable */
	if ( src == &cpu_p )
		tcg_temp_free_i32 ( cpu_p.var );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : PH%s\n", dc->pc, src->name );
}

/* CMP, CPX, CPY (immediate) */
static void m6502_gen_comp_imm ( DisasContext *dc ) {
	M6502Register *src = dc->insn->src;
	uint8_t value;

	/* Generate comparison */
	value = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );
	tcg_gen_setcondi_i32 ( TCG_COND_GE, cpu_p_c.var, src->var, value );
	tcg_gen_setcondi_i32 ( TCG_COND_EQ, cpu_p_z.var, src->var, value );
	tcg_gen_subi_i32 ( cpu_p_n.var, src->var, value );
	tcg_gen_andi_i32 ( cpu_p_n.var, cpu_p_n.var, 0x80 );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : C%s%s #%02X\n", dc->pc,
		  ( ( src == &cpu_a ) ? "M" : "P" ),
		  ( ( src == &cpu_a ) ? "P" : src->name ), value );
}

/* CMP, CPX, CPY */
static void m6502_gen_comp ( DisasContext *dc ) {
	M6502Register *src = dc->insn->src;
	TCGv_i32 value;

	/* Generate comparison */
	value = tcg_temp_new_i32();
	tcg_gen_qemu_ld8u ( value, dc->address, MEM_INDEX );
	tcg_gen_setcond_i32 ( TCG_COND_GE, cpu_p_c.var, src->var, value );
	tcg_gen_setcond_i32 ( TCG_COND_EQ, cpu_p_z.var, src->var, value );
	tcg_gen_sub_i32 ( cpu_p_n.var, src->var, value );
	tcg_gen_andi_i32 ( cpu_p_n.var, cpu_p_n.var, 0x80 );
	tcg_temp_free_i32 ( value );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : C%s%s %s\n", dc->pc,
		  ( ( src == &cpu_a ) ? "M" : "P" ),
		  ( ( src == &cpu_a ) ? "P" : src->name ), dc->address_desc );
}

/* BCC, BCS, BEQ, BMI, BNE, BPL, BVC, BVS */
static void m6502_gen_branch ( DisasContext *dc, TCGCond condition ) {
	M6502Register *src = dc->insn->src;
	int offset = cpu_ldsb_code ( dc->env, ( dc->pc + 1 ) );
	uint16_t target = ( dc->pc + 2 + offset );
	static char condition_desc_buf[3];
	static char condition_eq[] = "EQ";
	static char condition_ne[] = "NE";
	static char condition_pl[] = "PL";
	static char condition_mi[] = "MI";
	char * condition_desc;
	int label;

	/* Generate branch */
	label = gen_new_label();
	tcg_gen_brcondi_i32 ( condition, src->var, 0, label );
	tcg_gen_goto_tb ( 0 );
	tcg_gen_movi_i32 ( cpu_pc, ( dc->pc + 2 ) );
	tcg_gen_exit_tb ( ( tcg_target_long ) dc->tb + 0 );
	gen_set_label ( label );
	tcg_gen_goto_tb ( 1 );
	tcg_gen_movi_i32 ( cpu_pc, target );
	tcg_gen_exit_tb ( ( tcg_target_long ) dc->tb + 1 );

	/* Stop translation */
	dc->is_jmp = DISAS_TB_JUMP;

	/* Generate disassembly */
	if ( qemu_loglevel_mask ( CPU_LOG_TB_IN_ASM ) ) {
		condition_desc_buf[0] = src->name[0];
		condition_desc_buf[1] = ( ( condition == TCG_COND_NE ) ?
					  'S' : 'C' );
		condition_desc = condition_desc_buf;
		if ( src == &cpu_p_z ) {
			condition_desc = ( ( condition == TCG_COND_NE ) ?
					   condition_eq : condition_ne );
		} else if ( src == &cpu_p_n ) {
			condition_desc = ( ( condition == TCG_COND_NE ) ?
					   condition_mi : condition_pl );
		}
		LOG_DIS ( "&%04X : B%s &%04X\n", dc->pc, condition_desc,
			  target );
	}
}

/* BCS, BEQ, BMI, BVS */
static void m6502_gen_br_set ( DisasContext *dc ) {
	m6502_gen_branch ( dc, TCG_COND_NE );
}

/* BCC, BNE, BPL, BVC */
static void m6502_gen_br_clear ( DisasContext *dc ) {
	m6502_gen_branch ( dc, TCG_COND_EQ );
}

/** Instruction table */
static const M6502Instruction m6502_instructions[256] = {
	/* ASL */
	[0x0a] = { m6502_gen_asl,	&cpu_a,	NULL,	NULL,		1 },
	[0x06] = { m6502_gen_asl,	NULL,	NULL,	m6502_zero,	2 },
	[0x16] = { m6502_gen_asl,	NULL,	NULL,	m6502_zero_x,	2 },
	[0x0e] = { m6502_gen_asl,	NULL,	NULL,	m6502_abs,	3 },
	[0x1e] = { m6502_gen_asl,	NULL,	NULL,	m6502_abs_x,	3 },
	/* BCC */
	[0x90] = { m6502_gen_br_clear,	NULL, &cpu_p_c,	NULL,		2 },
	/* BCS */
	[0xb0] = { m6502_gen_br_set,	NULL, &cpu_p_c,	NULL,		2 },
	/* BEQ */
	[0xf0] = { m6502_gen_br_set,	NULL, &cpu_p_z,	NULL,		2 },
	/* BMI */
	[0x30] = { m6502_gen_br_set,	NULL, &cpu_p_n,	NULL,		2 },
	/* BNE */
	[0xd0] = { m6502_gen_br_clear,	NULL, &cpu_p_z,	NULL,		2 },
	/* BPL */
	[0x10] = { m6502_gen_br_clear,	NULL, &cpu_p_n,	NULL,		2 },
	/* BVC */
	[0x50] = { m6502_gen_br_clear,	NULL, &cpu_p_v,	NULL,		2 },
	/* BVS */
	[0x70] = { m6502_gen_br_set,	NULL, &cpu_p_v,	NULL,		2 },
	/* CLC */
	[0x18] = { m6502_gen_clear,	&cpu_p_c, NULL,	NULL,		1 },
	/* CLD */
	[0xd8] = { m6502_gen_clear,	&cpu_p_d, NULL,	NULL,		1 },
	/* CLI */
	[0x58] = { m6502_gen_clear,	&cpu_p_i, NULL,	NULL,		1 },
	/* CLV */
	[0xb8] = { m6502_gen_clear,	&cpu_p_v, NULL,	NULL,		1 },
	/* CMP */
	[0xc9] = { m6502_gen_comp_imm,	NULL,	&cpu_a,	NULL,		2 },
	[0xc5] = { m6502_gen_comp,	NULL,	&cpu_a,	m6502_zero,	2 },
	[0xd5] = { m6502_gen_comp,	NULL,	&cpu_a,	m6502_zero_x,	2 },
	[0xcd] = { m6502_gen_comp,	NULL,	&cpu_a,	m6502_abs,	3 },
	[0xdd] = { m6502_gen_comp,	NULL,	&cpu_a,	m6502_abs_x,	3 },
	[0xd9] = { m6502_gen_comp,	NULL,	&cpu_a,	m6502_abs_y,	3 },
	[0xc1] = { m6502_gen_comp,	NULL,	&cpu_a,	m6502_ind_x,	2 },
	[0xd1] = { m6502_gen_comp,	NULL,	&cpu_a,	m6502_ind_y,	2 },
	/* CPX */
	[0xe0] = { m6502_gen_comp_imm,	NULL,	&cpu_x,	NULL,		2 },
	[0xe4] = { m6502_gen_comp,	NULL,	&cpu_x,	m6502_zero,	2 },
	[0xec] = { m6502_gen_comp,	NULL,	&cpu_x,	m6502_abs,	3 },
	/* CPY */
	[0xc0] = { m6502_gen_comp_imm,	NULL,	&cpu_y,	NULL,		2 },
	[0xc4] = { m6502_gen_comp,	NULL,	&cpu_y,	m6502_zero,	2 },
	[0xcc] = { m6502_gen_comp,	NULL,	&cpu_y,	m6502_abs,	3 },
	/* DEC */
	[0xc6] = { m6502_gen_dec,	NULL,	NULL,	m6502_zero,	2 },
	[0xd6] = { m6502_gen_dec,	NULL,	NULL,	m6502_zero_x,	2 },
	[0xce] = { m6502_gen_dec,	NULL,	NULL,	m6502_abs,	3 },
	[0xde] = { m6502_gen_dec,	NULL,	NULL,	m6502_abs_x,	3 },
	/* DEX */
	[0xca] = { m6502_gen_dec,	&cpu_x,	NULL,	NULL,		1 },
	/* DEY */
	[0x88] = { m6502_gen_dec,	&cpu_y,	NULL,	NULL,		1 },
	/* INC */
	[0xe6] = { m6502_gen_inc,	NULL,	NULL,	m6502_zero,	2 },
	[0xf6] = { m6502_gen_inc,	NULL,	NULL,	m6502_zero_x,	2 },
	[0xee] = { m6502_gen_inc,	NULL,	NULL,	m6502_abs,	3 },
	[0xfe] = { m6502_gen_inc,	NULL,	NULL,	m6502_abs_x,	3 },
	/* INX */
	[0xe8] = { m6502_gen_inc,	&cpu_x,	NULL,	NULL,		1 },
	/* INY */
	[0xc8] = { m6502_gen_inc,	&cpu_y,	NULL,	NULL,		1 },
	/* LDA */
	[0xa9] = { m6502_gen_load_imm,	&cpu_a,	NULL,	NULL,		2 },
	[0xa5] = { m6502_gen_load,	&cpu_a,	NULL,	m6502_zero,	2 },
	[0xb5] = { m6502_gen_load,	&cpu_a,	NULL,	m6502_zero_x,	2 },
	[0xad] = { m6502_gen_load,	&cpu_a,	NULL,	m6502_abs,	3 },
	[0xbd] = { m6502_gen_load,	&cpu_a,	NULL,	m6502_abs_x,	3 },
	[0xb9] = { m6502_gen_load,	&cpu_a,	NULL,	m6502_abs_y,	3 },
	[0xa1] = { m6502_gen_load,	&cpu_a,	NULL,	m6502_ind_x,	2 },
	[0xb1] = { m6502_gen_load,	&cpu_a,	NULL,	m6502_ind_y,	2 },
	/* LDX */
	[0xa2] = { m6502_gen_load_imm,	&cpu_x,	NULL,	NULL,		2 },
	[0xa6] = { m6502_gen_load,	&cpu_x,	NULL,	m6502_zero,	2 },
	[0xb6] = { m6502_gen_load,	&cpu_x,	NULL,	m6502_zero_y,	2 },
	[0xae] = { m6502_gen_load,	&cpu_x,	NULL,	m6502_abs,	3 },
	[0xbe] = { m6502_gen_load,	&cpu_x,	NULL,	m6502_abs_y,	3 },
	/* LDY */
	[0xa0] = { m6502_gen_load_imm,	&cpu_y,	NULL,	NULL,		2 },
	[0xa4] = { m6502_gen_load,	&cpu_y,	NULL,	m6502_zero,	2 },
	[0xb4] = { m6502_gen_load,	&cpu_y,	NULL,	m6502_zero_x,	2 },
	[0xac] = { m6502_gen_load,	&cpu_y,	NULL,	m6502_abs,	3 },
	[0xbc] = { m6502_gen_load,	&cpu_y,	NULL,	m6502_abs_x,	3 },
	/* PHA */
	[0x48] = { m6502_gen_push,	NULL,	&cpu_a,	NULL,		1 },
	/* PHP */
	[0x08] = { m6502_gen_push,	NULL,	&cpu_p,	NULL,		1 },
	/* SEC */
	[0x38] = { m6502_gen_set,	&cpu_p_c, NULL,	NULL,		1 },
	/* SED */
	[0xf8] = { m6502_gen_set,	&cpu_p_d, NULL,	NULL,		1 },
	/* SEI */
	[0x78] = { m6502_gen_set,	&cpu_p_i, NULL,	NULL,		1 },
	/* STA */
	[0x85] = { m6502_gen_store,	NULL,	&cpu_a,	m6502_zero,	2 },
	[0x95] = { m6502_gen_store,	NULL,	&cpu_a,	m6502_zero_x,	2 },
	[0x8d] = { m6502_gen_store,	NULL,	&cpu_a,	m6502_abs,	3 },
	[0x9d] = { m6502_gen_store,	NULL,	&cpu_a,	m6502_abs_x,	3 },
	[0x99] = { m6502_gen_store,	NULL,	&cpu_a,	m6502_abs_y,	3 },
	[0x81] = { m6502_gen_store,	NULL,	&cpu_a,	m6502_ind_x,	2 },
	[0x91] = { m6502_gen_store,	NULL,	&cpu_a,	m6502_ind_y,	2 },
	/* STX */
	[0x86] = { m6502_gen_store,	NULL,	&cpu_x,	m6502_zero,	2 },
	[0x96] = { m6502_gen_store,	NULL,	&cpu_x,	m6502_zero_y,	2 },
	[0x8e] = { m6502_gen_store,	NULL,	&cpu_x,	m6502_abs,	3 },
	/* STY */
	[0x84] = { m6502_gen_store,	NULL,	&cpu_y,	m6502_zero,	2 },
	[0x94] = { m6502_gen_store,	NULL,	&cpu_y,	m6502_zero_x,	2 },
	[0x8c] = { m6502_gen_store,	NULL,	&cpu_y,	m6502_abs,	3 },
	/* TAX */
	[0xaa] = { m6502_gen_transfer,	&cpu_x,	&cpu_a,	NULL,		1 },
	/* TAY */
	[0xa8] = { m6502_gen_transfer,	&cpu_y,	&cpu_a,	NULL,		1 },
	/* TSX */
	[0xba] = { m6502_gen_transfer,	&cpu_x,	&cpu_s,	NULL,		1 },
	/* TXA */
	[0x8a] = { m6502_gen_transfer,	&cpu_a,	&cpu_x,	NULL,		1 },
	/* TXS */
	[0x9a] = { m6502_gen_transfer,	&cpu_s,	&cpu_x,	NULL,		1 },
	/* TYA */
	[0x98] = { m6502_gen_transfer,	&cpu_a,	&cpu_y,	NULL,		1 },
};

static size_t m6502_gen_instruction ( DisasContext *dc ) {
	const M6502Instruction *insn;
	uint8_t opcode;

	/* Fetch and validate opcode */
	opcode = cpu_ldub_code ( dc->env, dc->pc );
	dc->insn = insn = &m6502_instructions[opcode];
	if ( ! insn->gen ) {

		//
		LOG_DIS ( "&%04X : unknown opcode &%02X\n", dc->pc, opcode );
		return 0;

		cpu_abort ( dc->env, "Unknown opcode %02x at %04x\n",
			    opcode, dc->pc );
		return 0;
	}

	/* Generate memory address, if applicable */
	if ( insn->mem )
		insn->mem ( dc );

	/* Generate instruction */
	insn->gen ( dc );

	/* Free address, if one was generated */
	if ( insn->mem )
		tcg_temp_free_i32 ( dc->address );

	return insn->len;
}

void m6502_gen_intermediate_code ( CPUM6502State *env,
				   struct TranslationBlock *tb ) {
	DisasContext ctx;
	DisasContext *dc = &ctx;
	hwaddr pc_start = tb->pc;
	unsigned int num_insns = 0;
	size_t len;

	printf ( "gen_intermediate_code() pc=%04x\n", tb->pc );

	dc->env = env;
	dc->pc = pc_start;
	dc->tb = tb;
	dc->is_jmp = DISAS_NEXT;

	gen_icount_start();
	do {
		len = m6502_gen_instruction ( dc );
		dc->pc += len;
		num_insns++;
	} while ( len && ( dc->is_jmp == DISAS_NEXT ) );

	if ( dc->is_jmp == DISAS_NEXT )
		tcg_gen_movi_i32 ( cpu_pc, dc->pc );
	if ( dc->pc == tb->pc )
		gen_helper_hlt ( cpu_env );
	tcg_gen_exit_tb ( 0 );

	gen_icount_end ( tb, num_insns );
	*tcg_ctx.gen_opc_ptr = INDEX_op_end;
	tb->size = ( dc->pc - pc_start );
	tb->icount = num_insns;
}

void m6502_gen_intermediate_code_pc ( CPUM6502State *env,
				      struct TranslationBlock *tb ) {

	printf ( "gen_intermediate_code_pc()\n" );
}

void m6502_restore_state_to_opc ( CPUM6502State *env,
				  TranslationBlock *tb, int pc_pos ) {

	printf ( "restore_state_to_opc\n" );

	env->pc = tcg_ctx.gen_opc_pc[pc_pos];
}

void m6502_translate_init ( void ) {

	printf ( "translate_init()\n" );

	cpu_env = tcg_global_reg_new_ptr ( TCG_AREG0, "env" );
	cpu_a.var = tcg_global_mem_new ( TCG_AREG0,
					 offsetof ( CPUM6502State, a ), "a" );
	cpu_x.var = tcg_global_mem_new ( TCG_AREG0,
					 offsetof ( CPUM6502State, x ), "x" );
	cpu_y.var = tcg_global_mem_new ( TCG_AREG0,
					 offsetof ( CPUM6502State, y ), "y" );
	cpu_p_c.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p[P_C] ),
					   "p.c" );
	cpu_p_z.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p[P_Z] ),
					   "p.c" );
	cpu_p_i.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p[P_I] ),
					   "p.c" );
	cpu_p_d.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p[P_D] ),
					   "p.c" );
	cpu_p_b.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p[P_B] ),
					   "p.c" );
	cpu_p_u.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p[P_U] ),
					   "p.c" );
	cpu_p_v.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p[P_V] ),
					   "p.c" );
	cpu_p_n.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p[P_N] ),
					   "p.c" );
	cpu_s.var = tcg_global_mem_new ( TCG_AREG0,
					 offsetof ( CPUM6502State, s ), "s" );
	cpu_pc = tcg_global_mem_new ( TCG_AREG0,
				      offsetof ( CPUM6502State, pc ), "pc" );
}
