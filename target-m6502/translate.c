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
static M6502Register cpu_p_nz = { .name = "Z" };
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
	unsigned int num_insns;
	unsigned int offset;
};

/******************************************************************************
 *
 * Status register assembly/disassembly
 *
 * See comments in CPUM6502State for an explanation of why the flags
 * are stored this way.
 */

/**
 * Get status register value (P)
 *
 * @v env		CPU state
 * @ret p		Status register value (P)
 */
unsigned int m6502_get_p ( CPUM6502State *env ) {

	return ( ( env->p_c << P_C ) | ( ( ! env->p_nz ) << P_Z ) |
		 ( env->p_i << P_I ) | ( env->p_d << P_D ) |
		 ( env->p_b << P_B ) | ( env->p_u << P_U ) |
		 ( ( !! env->p_v ) << P_V ) | env->p_n );
}

/**
 * Set status register value (P)
 *
 * @v env		CPU state
 * @v p			Status register value (P)
 */
void m6502_set_p ( CPUM6502State *env, unsigned int p ) {

	env->p_c = ( ( p >> P_C ) & 1 );
	env->p_nz = ( ~p & ( 1 << P_Z ) );
	env->p_i = ( ( p >> P_I ) & 1 );
	env->p_d = ( ( p >> P_D ) & 1 );
	env->p_b = ( ( p >> P_B ) & 1 );
	env->p_u = ( ( p >> P_U ) & 1 );
	env->p_v = ( p & ( 1 << P_V ) );
	env->p_n = ( p & ( 1 << P_N ) );
}

/******************************************************************************
 *
 * Addressing modes
 *
 */

/**
 * Generate address description
 *
 * @v dc		Disassembly context
 * @v fmt		Format string
 * @v ...		Arguments
 */
static void m6502_address_desc ( DisasContext *dc, const char *fmt, ... ) {
	va_list args;

	if ( qemu_loglevel_mask ( CPU_LOG_TB_IN_ASM ) ) {
		va_start ( args, fmt );
		vsnprintf ( dc->address_desc, sizeof ( dc->address_desc ),
			    fmt, args );
		va_end ( args );
	}
}

/**
 * Generate address for absolute addressing mode
 *
 * @v dc		Disassembly context
 */
static void m6502_abs ( DisasContext *dc ) {
	uint16_t base = cpu_lduw_code ( dc->env, ( dc->pc + 1 ) );

	/* Generate address */
	dc->address = tcg_const_i32 ( base );

	/* Generate address description for disassembly */
	m6502_address_desc ( dc, "&%04X", base );
}

/**
 * Generate address for absolute,X addressing mode
 *
 * @v dc		Disassembly context
 */
static void m6502_abs_x ( DisasContext *dc ) {
	uint16_t base = cpu_lduw_code ( dc->env, ( dc->pc + 1 ) );

	/* Generate address */
	dc->address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( dc->address, cpu_x.var, base );
	tcg_gen_andi_i32 ( dc->address, dc->address, M6502_ADDRESS_MASK );

	/* Generate address description for disassembly */
	m6502_address_desc ( dc, "&%04X,X", base );
}

/**
 * Generate address for absolute,Y addressing mode
 *
 * @v dc		Disassembly context
 */
static void m6502_abs_y ( DisasContext *dc ) {
	uint16_t base = cpu_lduw_code ( dc->env, ( dc->pc + 1 ) );

	/* Generate address */
	dc->address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( dc->address, cpu_y.var, base );
	tcg_gen_andi_i32 ( dc->address, dc->address, M6502_ADDRESS_MASK );

	/* Generate address description for disassembly */
	m6502_address_desc ( dc, "&%04X,Y", base );
}

/**
 * Generate address for zero-page addressing mode
 *
 * @v dc		Disassembly context
 */
static void m6502_zero ( DisasContext *dc ) {
	uint8_t base = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	/* Generate address */
	dc->address = tcg_const_i32 ( base );

	/* Generate address description for disassembly */
	m6502_address_desc ( dc, "&%02X", base );
}

/**
 * Generate address for zero-page,X addressing mode
 *
 * @v dc		Disassembly context
 */
static void m6502_zero_x ( DisasContext *dc ) {
	uint8_t base = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	/* Generate address */
	dc->address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( dc->address, cpu_x.var, base );
	tcg_gen_andi_i32 ( dc->address, dc->address, M6502_ZERO_PAGE_MASK );

	/* Generate address description for disassembly */
	m6502_address_desc ( dc, "&%02X,X", base );
}

/**
 * Generate address for zero-page,Y addressing mode
 *
 * @v dc		Disassembly context
 */
static void m6502_zero_y ( DisasContext *dc ) {
	uint8_t base = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	/* Generate address */
	dc->address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( dc->address, cpu_y.var, base );
	tcg_gen_andi_i32 ( dc->address, dc->address, M6502_ZERO_PAGE_MASK );

	/* Generate address description for disassembly */
	m6502_address_desc ( dc, "&%02X,Y", base );
}

/**
 * Generate address for (indirect,X) addressing mode
 *
 * @v dc		Disassembly context
 */
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

/**
 * Generate address for (indirect),Y addressing mode
 *
 * @v dc		Disassembly context
 */
static void m6502_ind_y ( DisasContext *dc ) {
	uint8_t base = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	/* Generate address */
	dc->address = tcg_const_i32 ( base );
	tcg_gen_qemu_ld16u ( dc->address, dc->address, MEM_INDEX );
	tcg_gen_add_i32 ( dc->address, dc->address, cpu_y.var );

	/* Generate address description for disassembly */
	m6502_address_desc ( dc, "(&%02X),Y", base );
}

/******************************************************************************
 *
 * Load/store/transfer instructions
 *
 */

/**
 * Generate load (immediate) instruction (LDA, LDX, LDY)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_load_imm ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;
	uint8_t value;

	/* Load register */
	value = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );
	tcg_gen_movi_i32 ( dest->var, value );
	tcg_gen_movi_i32 ( cpu_p_nz.var, value );
	tcg_gen_movi_i32 ( cpu_p_n.var, ( value & 0x80 ) );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : LD%s #&%02X\n", dc->pc, dest->name, value );
}

/**
 * Generate load (memory) instruction (LDA, LDX, LDY)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_load ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;

	/* Load register */
	tcg_gen_qemu_ld8u ( dest->var, dc->address, MEM_INDEX );
	tcg_gen_mov_i32 ( cpu_p_nz.var, dest->var );
	tcg_gen_andi_i32 ( cpu_p_n.var, dest->var, 0x80 );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : LD%s %s\n", dc->pc, dest->name, dc->address_desc );
}

/**
 * Generate store instruction (STA, STX, STY)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_store ( DisasContext *dc ) {
	M6502Register *src = dc->insn->src;

	/* Store register */
	tcg_gen_qemu_st8 ( src->var, dc->address, MEM_INDEX );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : ST%s %s\n", dc->pc, src->name, dc->address_desc );
}

/**
 * Generate transfer instruction (TAX, TAY, TSX, TXA, TXS, TYA)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_transfer ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;
	M6502Register *src = dc->insn->src;

	/* Transfer register */
	tcg_gen_mov_i32 ( dest->var, src->var );
	tcg_gen_mov_i32 ( cpu_p_nz.var, dest->var );
	tcg_gen_andi_i32 ( cpu_p_n.var, dest->var, 0x80 );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : T%s%s\n", dc->pc, src->name, dest->name );
}

/******************************************************************************
 *
 * Flag set/clear instructions
 *
 */

/**
 * Generate clear instruction (CLC, CLD, CLI, CLV)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_clear ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;

	/* Clear flag */
	tcg_gen_movi_i32 ( dest->var, 0 );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : CL%s\n", dc->pc, dest->name );
}

/**
 * Generate set instruction (SEC, SED, SEI)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_set ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;

	/* Set flag */
	tcg_gen_movi_i32 ( dest->var, 1 );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : SE%s\n", dc->pc, dest->name );
}

/******************************************************************************
 *
 * Arithmetic/logical instructions
 *
 */

/**
 * Generate add/subtract (immediate) instruction (SBC)
 *
 * @v dc		Disassembly context
 * @v invert		This is a subtraction
 */
static void m6502_gen_add_sub_imm ( DisasContext *dc, int invert ) {
	M6502Register *dest = dc->insn->dest;
	uint8_t value = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );
	TCGv_i32 overflow_before;
	TCGv_i32 overflow_after;

	/* Generate add/subtract */
	overflow_before = tcg_temp_new_i32();
	overflow_after = tcg_temp_new_i32();
	if ( invert )
		value = ( value ^ 0xff );
	tcg_gen_xori_i32 ( overflow_before, dest->var, value );
	tcg_gen_addi_i32 ( dest->var, dest->var, value );
	tcg_gen_add_i32 ( dest->var, dest->var, cpu_p_c.var );
	tcg_gen_shri_i32 ( cpu_p_c.var, dest->var, 8 );
	tcg_gen_andi_i32 ( dest->var, dest->var, 0xff );
	tcg_gen_xori_i32 ( overflow_after, dest->var, value );
	tcg_gen_andc_i32 ( cpu_p_v.var, overflow_after, overflow_before );
	tcg_gen_andi_i32 ( cpu_p_v.var, cpu_p_v.var, 0x80 );
	tcg_gen_mov_i32 ( cpu_p_nz.var, dest->var );
	tcg_gen_andi_i32 ( cpu_p_n.var, dest->var, 0x80 );
	tcg_temp_free_i32 ( overflow_after );
	tcg_temp_free_i32 ( overflow_before );

	/* FIXME : support decimal mode */

	/* Generate disassembly */
	LOG_DIS ( "&%04X : %sC #&%02X\n", dc->pc,
		  ( invert ? "SB" : "AD" ), value );
}

/**
 * Generate add/subtract (memory) instruction (ADC, SBC)
 *
 * @v dc		Disassembly context
 * @v invert		This is a subtraction
 */
static void m6502_gen_add_sub ( DisasContext *dc, int invert ) {
	M6502Register *dest = dc->insn->dest;
	TCGv_i32 value;
	TCGv_i32 overflow_before;
	TCGv_i32 overflow_after;

	/* Generate add/subtract */
	value = tcg_temp_new_i32();
	overflow_before = tcg_temp_new_i32();
	overflow_after = tcg_temp_new_i32();
	tcg_gen_qemu_ld8u ( value, dc->address, MEM_INDEX );
	if ( invert )
		tcg_gen_xori_i32 ( value, value, 0xff );
	tcg_gen_xor_i32 ( overflow_before, dest->var, value );
	tcg_gen_add_i32 ( dest->var, dest->var, value );
	tcg_gen_add_i32 ( dest->var, dest->var, cpu_p_c.var );
	tcg_gen_shri_i32 ( cpu_p_c.var, dest->var, 8 );
	tcg_gen_andi_i32 ( dest->var, dest->var, 0xff );
	tcg_gen_xor_i32 ( overflow_after, dest->var, value );
	tcg_gen_andc_i32 ( cpu_p_v.var, overflow_after, overflow_before );
	tcg_gen_andi_i32 ( cpu_p_v.var, cpu_p_v.var, 0x80 );
	tcg_gen_mov_i32 ( cpu_p_nz.var, dest->var );
	tcg_gen_andi_i32 ( cpu_p_n.var, dest->var, 0x80 );
	tcg_temp_free_i32 ( overflow_after );
	tcg_temp_free_i32 ( overflow_before );
	tcg_temp_free_i32 ( value );

	/* FIXME : support decimal mode */

	/* Generate disassembly */
	LOG_DIS ( "&%04X : %sC %s\n", dc->pc,
		  ( invert ? "SB" : "AD" ), dc->address_desc );
}

/**
 * Generate add (immediate) instruction (ADC)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_add_imm ( DisasContext *dc ) {
	m6502_gen_add_sub_imm ( dc, 0 );
}

/**
 * Generate add (memory) instruction (ADC)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_add ( DisasContext *dc ) {
	m6502_gen_add_sub ( dc, 0 );
}
/**
 * Generate subtract (immediate) instruction (SBC)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_sub_imm ( DisasContext *dc ) {
	m6502_gen_add_sub_imm ( dc, 1 );
}

/**
 * Generate subtract (memory) instruction (SBC)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_sub ( DisasContext *dc ) {
	m6502_gen_add_sub ( dc, 1 );
}

/**
 * Generate logical AND (immediate) instruction (AND)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_and_imm ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;
	uint8_t value = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	/* Generate logical AND */
	tcg_gen_andi_i32 ( dest->var, dest->var, value );
	tcg_gen_mov_i32 ( cpu_p_nz.var, dest->var );
	tcg_gen_andi_i32 ( cpu_p_n.var, dest->var, 0x80 );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : AND #&%02X\n", dc->pc, value );
}

/**
 * Generate logical AND (memory) instruction (AND)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_and ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;
	TCGv_i32 value;

	/* Generate logical AND */
	value = tcg_temp_new_i32();
	tcg_gen_qemu_ld8u ( value, dc->address, MEM_INDEX );
	tcg_gen_and_i32 ( dest->var, dest->var, value );
	tcg_gen_mov_i32 ( cpu_p_nz.var, dest->var );
	tcg_gen_andi_i32 ( cpu_p_n.var, dest->var, 0x80 );	
	tcg_temp_free_i32 ( value );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : AND %s\n", dc->pc, dc->address_desc );
}

/**
 * Generate logical OR (immediate) instruction (ORA)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_or_imm ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;
	uint8_t value = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	/* Generate logical OR */
	tcg_gen_ori_i32 ( dest->var, dest->var, value );
	tcg_gen_mov_i32 ( cpu_p_nz.var, dest->var );
	tcg_gen_andi_i32 ( cpu_p_n.var, dest->var, 0x80 );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : ORA #&%02X\n", dc->pc, value );
}

/**
 * Generate logical OR (memory) instruction (ORA)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_or ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;
	TCGv_i32 value;

	/* Generate logical OR */
	value = tcg_temp_new_i32();
	tcg_gen_qemu_ld8u ( value, dc->address, MEM_INDEX );
	tcg_gen_or_i32 ( dest->var, dest->var, value );
	tcg_gen_mov_i32 ( cpu_p_nz.var, dest->var );
	tcg_gen_andi_i32 ( cpu_p_n.var, dest->var, 0x80 );	
	tcg_temp_free_i32 ( value );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : ORA %s\n", dc->pc, dc->address_desc );
}

/**
 * Generate logical exclusive OR (immediate) instruction (EOR)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_xor_imm ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;
	uint8_t value = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	/* Generate logical XOR */
	tcg_gen_xori_i32 ( dest->var, dest->var, value );
	tcg_gen_mov_i32 ( cpu_p_nz.var, dest->var );
	tcg_gen_andi_i32 ( cpu_p_n.var, dest->var, 0x80 );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : EOR #&%02X\n", dc->pc, value );
}

/**
 * Generate logical exclusive OR (memory) instruction (EOR)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_xor ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;
	TCGv_i32 value;

	/* Generate logical XOR */
	value = tcg_temp_new_i32();
	tcg_gen_qemu_ld8u ( value, dc->address, MEM_INDEX );
	tcg_gen_xor_i32 ( dest->var, dest->var, value );
	tcg_gen_mov_i32 ( cpu_p_nz.var, dest->var );
	tcg_gen_andi_i32 ( cpu_p_n.var, dest->var, 0x80 );	
	tcg_temp_free_i32 ( value );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : EOR %s\n", dc->pc, dc->address_desc );
}

/******************************************************************************
 *
 * Read/modify/write instructions
 *
 */

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

/**
 * Generate arithmetic shift left instruction (ASL)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_asl ( DisasContext *dc ) {
	TCGv_i32 value;

	/* Perform shift left */
	value = m6502_rmw_load ( dc );
	tcg_gen_shri_i32 ( cpu_p_c.var, value, 7 );
	tcg_gen_shli_i32 ( value, value, 1 );
	tcg_gen_andi_i32 ( value, value, 0xff );
	tcg_gen_mov_i32 ( cpu_p_nz.var, value );
	tcg_gen_andi_i32 ( cpu_p_n.var, value, 0x80 );
	m6502_rmw_store ( dc, value );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : ASL%s%s\n", dc->pc,
		  ( dc->insn->mem ? " " : "" ),
		  ( dc->insn->mem ? dc->address_desc : "" ) );
}

/**
 * Generate logical shift right instruction (LSR)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_lsr ( DisasContext *dc ) {
	TCGv_i32 value;

	/* Perform shift right */
	value = m6502_rmw_load ( dc );
	tcg_gen_andi_i32 ( cpu_p_c.var, value, 0x01 );
	tcg_gen_shri_i32 ( value, value, 1 );
	tcg_gen_mov_i32 ( cpu_p_nz.var, value );
	tcg_gen_movi_i32 ( cpu_p_n.var, 0 );
	m6502_rmw_store ( dc, value );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : LSR%s%s\n", dc->pc,
		  ( dc->insn->mem ? " " : "" ),
		  ( dc->insn->mem ? dc->address_desc : "" ) );
}

/**
 * Generate rotate left instruction (ROL)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_rol ( DisasContext *dc ) {
	TCGv_i32 value;

	/* Perform rotate left */
	value = m6502_rmw_load ( dc );
	tcg_gen_shli_i32 ( value, value, 1 );
	tcg_gen_or_i32 ( value, value, cpu_p_c.var );
	tcg_gen_shri_i32 ( cpu_p_c.var, value, 8 );
	tcg_gen_andi_i32 ( value, value, 0xff );
	tcg_gen_mov_i32 ( cpu_p_nz.var, value );
	tcg_gen_andi_i32 ( cpu_p_n.var, value, 0x80 );
	m6502_rmw_store ( dc, value );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : ROL%s%s\n", dc->pc,
		  ( dc->insn->mem ? " " : "" ),
		  ( dc->insn->mem ? dc->address_desc : "" ) );
}

/**
 * Generate rotate right instruction (ROR)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_ror ( DisasContext *dc ) {
	TCGv_i32 value;

	/* Perform rotate right */
	value = m6502_rmw_load ( dc );
	tcg_gen_deposit_i32 ( value, value, cpu_p_c.var, 8, 1 );
	tcg_gen_andi_i32 ( cpu_p_c.var, value, 0x01 );
	tcg_gen_shri_i32 ( value, value, 1 );
	tcg_gen_mov_i32 ( cpu_p_nz.var, value );
	tcg_gen_andi_i32 ( cpu_p_n.var, value, 0x80 );
	m6502_rmw_store ( dc, value );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : ROR%s%s\n", dc->pc,
		  ( dc->insn->mem ? " " : "" ),
		  ( dc->insn->mem ? dc->address_desc : "" ) );
}

/**
 * Generate increment instruction (INC, INX, INY)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_inc ( DisasContext *dc ) {
	TCGv_i32 value;

	/* Perform increment */
	value = m6502_rmw_load ( dc );
	tcg_gen_addi_i32 ( value, value, 1 );
	tcg_gen_andi_i32 ( value, value, 0xff );
	tcg_gen_mov_i32 ( cpu_p_nz.var, value );
	tcg_gen_andi_i32 ( cpu_p_n.var, value, 0x80 );
	m6502_rmw_store ( dc, value );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : IN%s%s%s\n", dc->pc,
		  ( dc->insn->mem ? "C" : dc->insn->dest->name ),
		  ( dc->insn->mem ? " " : "" ),
		  ( dc->insn->mem ? dc->address_desc : "" ) );
}

/**
 * Generate decrement instruction (DEC, DEX, DEY)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_dec ( DisasContext *dc ) {
	TCGv_i32 value;

	/* Perform increment */
	value = m6502_rmw_load ( dc );
	tcg_gen_subi_i32 ( value, value, 1 );
	tcg_gen_andi_i32 ( value, value, 0xff );
	tcg_gen_mov_i32 ( cpu_p_nz.var, value );
	tcg_gen_andi_i32 ( cpu_p_n.var, value, 0x80 );
	m6502_rmw_store ( dc, value );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : DE%s%s%s\n", dc->pc,
		  ( dc->insn->mem ? "C" : dc->insn->dest->name ),
		  ( dc->insn->mem ? " " : "" ),
		  ( dc->insn->mem ? dc->address_desc : "" ) );
}

/******************************************************************************
 *
 * Stack push/pull instructions
 *
 */

/**
 * Generate push instruction (PHA, PHP)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_push ( DisasContext *dc ) {
	M6502Register *src = dc->insn->src;
	TCGv_i32 stack;

	/* Generate P value from flags, if applicable */
	if ( src == &cpu_p ) {
		cpu_p.var = tcg_temp_new_i32();
		gen_helper_get_p ( cpu_p.var, cpu_env );
	}

	/* Store value at current stack address */
	stack = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( stack, cpu_s.var, M6502_STACK_BASE );
	tcg_gen_qemu_st8 ( src->var, stack, MEM_INDEX );
	tcg_temp_free_i32 ( stack );

	/* Decrement stack pointer (with wrap-around) */
	tcg_gen_subi_i32 ( cpu_s.var, cpu_s.var, 1 );
	tcg_gen_andi_i32 ( cpu_s.var, cpu_s.var, 0xff );

	/* Free temporary P value, if applicable */
	if ( src == &cpu_p )
		tcg_temp_free_i32 ( cpu_p.var );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : PH%s\n", dc->pc, src->name );
}

/**
 * Generate pull instruction (PLA, PHP)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_pull ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;
	TCGv_i32 stack;

	/* Create temporary P value, if applicable */
	if ( dest == &cpu_p )
		cpu_p.var = tcg_temp_new_i32();

	/* Increment stack pointer (with wrap-around) */
	tcg_gen_addi_i32 ( cpu_s.var, cpu_s.var, 1 );
	tcg_gen_andi_i32 ( cpu_s.var, cpu_s.var, 0xff );

	/* Retrieve value from current stack address */
	stack = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( stack, cpu_s.var, M6502_STACK_BASE );
	tcg_gen_qemu_ld8u ( dest->var, stack, MEM_INDEX );
	tcg_temp_free_i32 ( stack );

	/* Generate flags from P value, or set flags based on A */
	if ( dest == &cpu_p ) {
		gen_helper_set_p ( cpu_env, cpu_p.var );
		tcg_temp_free_i32 ( cpu_p.var );
	} else {
		tcg_gen_mov_i32 ( cpu_p_nz.var, dest->var );
		tcg_gen_andi_i32 ( cpu_p_n.var, dest->var, 0x80 );
	}

	/* Generate disassembly */
	LOG_DIS ( "&%04X : PL%s\n", dc->pc, dest->name );
}

/******************************************************************************
 *
 * Comparison instructions
 *
 */

/**
 * Generate compare (immediate) instruction (CMP, CPX, CPY)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_comp_imm ( DisasContext *dc ) {
	M6502Register *src = dc->insn->src;
	uint8_t value;

	/* Generate comparison */
	value = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );
	tcg_gen_setcondi_i32 ( TCG_COND_GE, cpu_p_c.var, src->var, value );
	tcg_gen_subi_i32 ( cpu_p_nz.var, src->var, value );
	tcg_gen_subi_i32 ( cpu_p_n.var, src->var, value );
	tcg_gen_andi_i32 ( cpu_p_n.var, cpu_p_n.var, 0x80 );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : C%s%s #%02X\n", dc->pc,
		  ( ( src == &cpu_a ) ? "M" : "P" ),
		  ( ( src == &cpu_a ) ? "P" : src->name ), value );
}

/**
 * Generate compare (memory) instruction (CMP, CPX, CPY)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_comp ( DisasContext *dc ) {
	M6502Register *src = dc->insn->src;
	TCGv_i32 value;

	/* Generate comparison */
	value = tcg_temp_new_i32();
	tcg_gen_qemu_ld8u ( value, dc->address, MEM_INDEX );
	tcg_gen_setcond_i32 ( TCG_COND_GE, cpu_p_c.var, src->var, value );
	tcg_gen_sub_i32 ( cpu_p_nz.var, src->var, value );
	tcg_gen_sub_i32 ( cpu_p_n.var, src->var, value );
	tcg_gen_andi_i32 ( cpu_p_n.var, cpu_p_n.var, 0x80 );
	tcg_temp_free_i32 ( value );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : C%s%s %s\n", dc->pc,
		  ( ( src == &cpu_a ) ? "M" : "P" ),
		  ( ( src == &cpu_a ) ? "P" : src->name ), dc->address_desc );
}

/**
 * Generate bit test instruction (BIT)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_bit ( DisasContext *dc ) {
	M6502Register *src = dc->insn->src;
	TCGv_i32 value;

	/* Generate bit test */
	value = tcg_temp_new_i32();
	tcg_gen_qemu_ld8u ( value, dc->address, MEM_INDEX );
	tcg_gen_andi_i32 ( cpu_p_v.var, value, 0x40 );
	tcg_gen_andi_i32 ( cpu_p_n.var, value, 0x80 );
	tcg_gen_and_i32 ( cpu_p_nz.var, value, src->var );
	tcg_temp_free_i32 ( value );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : BIT %s\n", dc->pc, dc->address_desc );
}

/******************************************************************************
 *
 * Branch/jump instructions
 *
 */

/**
 * Generate branch instruction (BCC, BCS, BEQ, BMI, BNE, BPL, BVC, BVS)
 *
 * @v dc		Disassembly context
 */
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
	tcg_gen_movi_i32 ( cpu_pc, ( dc->pc + 2 ) );
	tcg_gen_goto_tb ( 0 );
	tcg_gen_exit_tb ( ( tcg_target_long ) dc->tb + 0 );
	gen_set_label ( label );
	tcg_gen_movi_i32 ( cpu_pc, target );
	tcg_gen_goto_tb ( 1 );
	tcg_gen_exit_tb ( ( tcg_target_long ) dc->tb + 1 );

	/* Stop translation */
	dc->is_jmp = DISAS_TB_JUMP;

	/* Generate disassembly */
	if ( qemu_loglevel_mask ( CPU_LOG_TB_IN_ASM ) ) {
		condition_desc_buf[0] = src->name[0];
		condition_desc_buf[1] = ( ( condition == TCG_COND_NE ) ?
					  'S' : 'C' );
		condition_desc = condition_desc_buf;
		if ( src == &cpu_p_nz ) {
			condition_desc = ( ( condition == TCG_COND_NE ) ?
					   condition_ne : condition_eq );
		} else if ( src == &cpu_p_n ) {
			condition_desc = ( ( condition == TCG_COND_NE ) ?
					   condition_mi : condition_pl );
		}
		LOG_DIS ( "&%04X : B%s &%04X\n", dc->pc, condition_desc,
			  target );
	}
}

/**
 * Generate branch instruction (BCS, BNE, BMI, BVS)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_br_set ( DisasContext *dc ) {
	m6502_gen_branch ( dc, TCG_COND_NE );
}

/**
 * Generate branch instruction (BCC, BEQ, BPL, BVC)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_br_clear ( DisasContext *dc ) {
	m6502_gen_branch ( dc, TCG_COND_EQ );
}

/**
 * Generate absolute jump instruction (JMP)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_jump_abs ( DisasContext *dc ) {
	uint16_t target = cpu_lduw_code ( dc->env, ( dc->pc + 1 ) );

	/* Generate jump */
	tcg_gen_movi_i32 ( cpu_pc, target );
	tcg_gen_goto_tb ( 0 );
	tcg_gen_exit_tb ( 0 );

	/* Stop translation */
	dc->is_jmp = DISAS_TB_JUMP;

	/* Generate disassembly */
	LOG_DIS ( "&%04X : JMP &%04X\n", dc->pc, target );
}

/**
 * Generate indirect jump instruction (JMP)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_jump_ind ( DisasContext *dc ) {

	/* Generate jump */
	tcg_gen_qemu_ld16u ( cpu_pc, dc->address, MEM_INDEX );
	tcg_gen_goto_tb ( 0 );
	tcg_gen_exit_tb ( 0 );

	/* Stop translation */
	dc->is_jmp = DISAS_JUMP;

	/* Generate disassembly */
	LOG_DIS ( "&%04X : JMP (%s)\n", dc->pc, dc->address_desc );
}

/**
 * Generate jump to subroutine instruction (JSR)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_jsr ( DisasContext *dc ) {
	uint16_t target = cpu_lduw_code ( dc->env, ( dc->pc + 1 ) );
	TCGv_i32 stack;
	TCGv_i32 retaddr;

	/* Store return address on stack */
	stack = tcg_temp_new_i32();
	retaddr = tcg_const_i32 ( dc->pc + 2 );
	tcg_gen_addi_i32 ( stack, cpu_s.var, M6502_STACK_BASE - 1 );
	tcg_gen_qemu_st16 ( retaddr, stack, MEM_INDEX );
	tcg_temp_free_i32 ( retaddr );
	tcg_temp_free_i32 ( stack );

	/* Decrement stack pointer by two (with wrap-around) */
	tcg_gen_subi_i32 ( cpu_s.var, cpu_s.var, 2 );
	tcg_gen_andi_i32 ( cpu_s.var, cpu_s.var, 0xff );

	/* Generate jump */
	tcg_gen_movi_i32 ( cpu_pc, target );
	tcg_gen_goto_tb ( 0 );
	tcg_gen_exit_tb ( 0 );

	/* Stop translation */
	dc->is_jmp = DISAS_TB_JUMP;

	/* Generate disassembly */
	LOG_DIS ( "&%04X : JSR &%04X\n", dc->pc, target );
}

/**
 * Generate return from subroutine instruction (RTS)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_rts ( DisasContext *dc ) {
	TCGv_i32 stack;

	/* Increment stack pointer by two (with wrap-around) */
	tcg_gen_addi_i32 ( cpu_s.var, cpu_s.var, 2 );
	tcg_gen_andi_i32 ( cpu_s.var, cpu_s.var, 0xff );

	/* Retrieve return address from stack */
	stack = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( stack, cpu_s.var, M6502_STACK_BASE - 1 );
	tcg_gen_qemu_ld16u ( cpu_pc, stack, MEM_INDEX );
	tcg_gen_addi_i32 ( cpu_pc, cpu_pc, 1 );
	tcg_temp_free_i32 ( stack );

	/* Generate jump */
	tcg_gen_goto_tb ( 0 );
	tcg_gen_exit_tb ( 0 );

	/* Stop translation */
	dc->is_jmp = DISAS_JUMP;

	/* Generate disassembly */
	LOG_DIS ( "&%04X : RTS\n", dc->pc );
}

/**
 * Generate break instruction (BRK)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_break ( DisasContext *dc ) {
	TCGv_i32 stack;
	TCGv_i32 retaddr;
	TCGv_i32 vector;

	/* Set break flag */
	tcg_gen_movi_i32 ( cpu_p_b.var, 1 );

	/* Store return address and status register on stack */
	stack = tcg_temp_new_i32();
	retaddr = tcg_const_i32 ( dc->pc + 2 );
	cpu_p.var = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( stack, cpu_s.var, M6502_STACK_BASE - 1 );
	tcg_gen_qemu_st16 ( retaddr, stack, MEM_INDEX );
	tcg_gen_addi_i32 ( stack, cpu_s.var, M6502_STACK_BASE - 2 );
	gen_helper_get_p ( cpu_p.var, cpu_env );
	tcg_gen_qemu_st8 ( cpu_p.var, stack, MEM_INDEX );
	tcg_temp_free_i32 ( cpu_p.var );
	tcg_temp_free_i32 ( retaddr );
	tcg_temp_free_i32 ( stack );

	/* Decrement stack pointer by three (with wrap-around) */
	tcg_gen_subi_i32 ( cpu_s.var, cpu_s.var, 3 );
	tcg_gen_andi_i32 ( cpu_s.var, cpu_s.var, 0xff );

	/* Jump to break vector */
	vector = tcg_const_i32 ( M6502_IRQ_VECTOR );
	tcg_gen_qemu_ld16u ( cpu_pc, vector, MEM_INDEX );
	tcg_temp_free_i32 ( vector );
	tcg_gen_goto_tb ( 0 );
	tcg_gen_exit_tb ( 0 );

	/* Stop translation */
	dc->is_jmp = DISAS_JUMP;

	/* Generate disassembly */
	LOG_DIS ( "&%04X : BRK\n", dc->pc );
}

/**
 * Generate return from interrupt instruction (RTI)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_rti ( DisasContext *dc ) {
	TCGv_i32 stack;

	/* Increment stack pointer by three (with wrap-around) */
	tcg_gen_addi_i32 ( cpu_s.var, cpu_s.var, 3 );
	tcg_gen_andi_i32 ( cpu_s.var, cpu_s.var, 0xff );

	/* Retrieve status register and return address from stack */
	stack = tcg_temp_new_i32();
	cpu_p.var = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( stack, cpu_s.var, M6502_STACK_BASE - 1 );
	tcg_gen_qemu_ld16u ( cpu_pc, stack, MEM_INDEX );
	tcg_gen_addi_i32 ( stack, cpu_s.var, M6502_STACK_BASE - 2 );
	tcg_gen_qemu_ld8u ( cpu_p.var, stack, MEM_INDEX );
	gen_helper_set_p ( cpu_env, cpu_p.var );
	tcg_temp_free_i32 ( cpu_p.var );
	tcg_temp_free_i32 ( stack );

	/* Generate jump */
	tcg_gen_goto_tb ( 0 );
	tcg_gen_exit_tb ( 0 );

	/* Stop translation */
	dc->is_jmp = DISAS_JUMP;

	/* Generate disassembly */
	LOG_DIS ( "&%04X : RTI\n", dc->pc );
}

/**
 * Generate no-operation instruction (NOP)
 *
 * @v dc		Disassembly context
 */
static void m6502_gen_nop ( DisasContext *dc ) {
	/* Nothing to do */
}

/******************************************************************************
 *
 * Instruction generator
 *
 */

/** Instruction table */
static const M6502Instruction m6502_instructions[256] = {
	/* BRK */
	[0x00] = { m6502_gen_break,	NULL,	NULL,	NULL,		1 },
	/* ADC */
	[0x69] = { m6502_gen_add_imm,	&cpu_a,	NULL,	NULL,		2 },
	[0x65] = { m6502_gen_add,	&cpu_a,	NULL,	m6502_zero,	2 },
	[0x75] = { m6502_gen_add,	&cpu_a,	NULL,	m6502_zero_x,	2 },
	[0x6d] = { m6502_gen_add,	&cpu_a,	NULL,	m6502_abs,	3 },
	[0x7d] = { m6502_gen_add,	&cpu_a,	NULL,	m6502_abs_x,	3 },
	[0x79] = { m6502_gen_add,	&cpu_a,	NULL,	m6502_abs_y,	3 },
	[0x61] = { m6502_gen_add,	&cpu_a,	NULL,	m6502_ind_x,	2 },
	[0x71] = { m6502_gen_add,	&cpu_a,	NULL,	m6502_ind_y,	2 },
	/* AND */
	[0x29] = { m6502_gen_and_imm,	&cpu_a,	NULL,	NULL,		2 },
	[0x25] = { m6502_gen_and,	&cpu_a,	NULL,	m6502_zero,	2 },
	[0x35] = { m6502_gen_and,	&cpu_a,	NULL,	m6502_zero_x,	2 },
	[0x2d] = { m6502_gen_and,	&cpu_a,	NULL,	m6502_abs,	3 },
	[0x3d] = { m6502_gen_and,	&cpu_a,	NULL,	m6502_abs_x,	3 },
	[0x39] = { m6502_gen_and,	&cpu_a,	NULL,	m6502_abs_y,	3 },
	[0x21] = { m6502_gen_and,	&cpu_a,	NULL,	m6502_ind_x,	2 },
	[0x31] = { m6502_gen_and,	&cpu_a,	NULL,	m6502_ind_y,	2 },
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
	[0xf0] = { m6502_gen_br_clear,	NULL, &cpu_p_nz, NULL,		2 },
	/* BIT */
	[0x24] = { m6502_gen_bit,	NULL,	&cpu_a,	m6502_zero,	2 },
	[0x2c] = { m6502_gen_bit,	NULL,	&cpu_a,	m6502_abs,	3 },
	/* BMI */
	[0x30] = { m6502_gen_br_set,	NULL, &cpu_p_n,	NULL,		2 },
	/* BNE */
	[0xd0] = { m6502_gen_br_set,	NULL, &cpu_p_nz, NULL,		2 },
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
	/* EOR */
	[0x49] = { m6502_gen_xor_imm,	&cpu_a,	NULL,	NULL,		2 },
	[0x45] = { m6502_gen_xor,	&cpu_a,	NULL,	m6502_zero,	2 },
	[0x55] = { m6502_gen_xor,	&cpu_a,	NULL,	m6502_zero_x,	2 },
	[0x4d] = { m6502_gen_xor,	&cpu_a,	NULL,	m6502_abs,	3 },
	[0x5d] = { m6502_gen_xor,	&cpu_a,	NULL,	m6502_abs_x,	3 },
	[0x59] = { m6502_gen_xor,	&cpu_a,	NULL,	m6502_abs_y,	3 },
	[0x41] = { m6502_gen_xor,	&cpu_a,	NULL,	m6502_ind_x,	2 },
	[0x51] = { m6502_gen_xor,	&cpu_a,	NULL,	m6502_ind_y,	2 },
	/* INC */
	[0xe6] = { m6502_gen_inc,	NULL,	NULL,	m6502_zero,	2 },
	[0xf6] = { m6502_gen_inc,	NULL,	NULL,	m6502_zero_x,	2 },
	[0xee] = { m6502_gen_inc,	NULL,	NULL,	m6502_abs,	3 },
	[0xfe] = { m6502_gen_inc,	NULL,	NULL,	m6502_abs_x,	3 },
	/* INX */
	[0xe8] = { m6502_gen_inc,	&cpu_x,	NULL,	NULL,		1 },
	/* INY */
	[0xc8] = { m6502_gen_inc,	&cpu_y,	NULL,	NULL,		1 },
	/* JMP */
	[0x4c] = { m6502_gen_jump_abs,	NULL,	NULL,	NULL,		3 },
	[0x6c] = { m6502_gen_jump_ind,	NULL,	NULL,	m6502_abs,	3 },
	/* JSR */
	[0x20] = { m6502_gen_jsr,	NULL,	NULL,	NULL,		1 },
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
	/* LSR */
	[0x4a] = { m6502_gen_lsr,	&cpu_a,	NULL,	NULL,		1 },
	[0x46] = { m6502_gen_lsr,	NULL,	NULL,	m6502_zero,	2 },
	[0x56] = { m6502_gen_lsr,	NULL,	NULL,	m6502_zero_x,	2 },
	[0x4e] = { m6502_gen_lsr,	NULL,	NULL,	m6502_abs,	3 },
	[0x5e] = { m6502_gen_lsr,	NULL,	NULL,	m6502_abs_x,	3 },
	/* NOP */
	[0xea] = { m6502_gen_nop,	NULL,	NULL,	NULL,		1 },
	/* OR */
	[0x09] = { m6502_gen_or_imm,	&cpu_a,	NULL,	NULL,		2 },
	[0x05] = { m6502_gen_or,	&cpu_a,	NULL,	m6502_zero,	2 },
	[0x15] = { m6502_gen_or,	&cpu_a,	NULL,	m6502_zero_x,	2 },
	[0x0d] = { m6502_gen_or,	&cpu_a,	NULL,	m6502_abs,	3 },
	[0x1d] = { m6502_gen_or,	&cpu_a,	NULL,	m6502_abs_x,	3 },
	[0x19] = { m6502_gen_or,	&cpu_a,	NULL,	m6502_abs_y,	3 },
	[0x01] = { m6502_gen_or,	&cpu_a,	NULL,	m6502_ind_x,	2 },
	[0x11] = { m6502_gen_or,	&cpu_a,	NULL,	m6502_ind_y,	2 },
	/* PHA */
	[0x48] = { m6502_gen_push,	NULL,	&cpu_a,	NULL,		1 },
	/* PHP */
	[0x08] = { m6502_gen_push,	NULL,	&cpu_p,	NULL,		1 },
	/* PLA */
	[0x68] = { m6502_gen_pull,	&cpu_a,	NULL,	NULL,		1 },
	/* PLP */
	[0x28] = { m6502_gen_pull,	&cpu_p,	NULL,	NULL,		1 },
	/* ROL */
	[0x2a] = { m6502_gen_rol,	&cpu_a,	NULL,	NULL,		1 },
	[0x26] = { m6502_gen_rol,	NULL,	NULL,	m6502_zero,	2 },
	[0x36] = { m6502_gen_rol,	NULL,	NULL,	m6502_zero_x,	2 },
	[0x2e] = { m6502_gen_rol,	NULL,	NULL,	m6502_abs,	3 },
	[0x3e] = { m6502_gen_rol,	NULL,	NULL,	m6502_abs_x,	3 },	
	/* ROR */
	[0x6a] = { m6502_gen_ror,	&cpu_a,	NULL,	NULL,		1 },
	[0x66] = { m6502_gen_ror,	NULL,	NULL,	m6502_zero,	2 },
	[0x76] = { m6502_gen_ror,	NULL,	NULL,	m6502_zero_x,	2 },
	[0x6e] = { m6502_gen_ror,	NULL,	NULL,	m6502_abs,	3 },
	[0x7e] = { m6502_gen_ror,	NULL,	NULL,	m6502_abs_x,	3 },
	/* RTI */
	[0x40] = { m6502_gen_rti,	NULL,	NULL,	NULL,		1 },
	/* RTS */
	[0x60] = { m6502_gen_rts,	NULL,	NULL,	NULL,		1 },
	/* SBC */
	[0xe9] = { m6502_gen_sub_imm,	&cpu_a,	NULL,	NULL,		2 },
	[0xe5] = { m6502_gen_sub,	&cpu_a,	NULL,	m6502_zero,	2 },
	[0xf5] = { m6502_gen_sub,	&cpu_a,	NULL,	m6502_zero_x,	2 },
	[0xed] = { m6502_gen_sub,	&cpu_a,	NULL,	m6502_abs,	3 },
	[0xfd] = { m6502_gen_sub,	&cpu_a,	NULL,	m6502_abs_x,	3 },
	[0xf9] = { m6502_gen_sub,	&cpu_a,	NULL,	m6502_abs_y,	3 },
	[0xe1] = { m6502_gen_sub,	&cpu_a,	NULL,	m6502_ind_x,	2 },
	[0xf1] = { m6502_gen_sub,	&cpu_a,	NULL,	m6502_ind_y,	2 },
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

/**
 * Check for breakpoints
 *
 * @v dc		Disassembly context
 * @ret hit		Breakpoint hit
 */
static int m6502_check_breakpoints ( DisasContext *dc ) {
	CPUBreakpoint *bp;
	TCGv_i32 excp;

	/* Do nothing if there are no breakpoints */
	if ( likely ( QTAILQ_EMPTY ( &dc->env->breakpoints ) ) )
		return 0;

	/* Scan through list of breakpoints */
	QTAILQ_FOREACH ( bp, &dc->env->breakpoints, entry ) {
		if ( bp->pc != dc->pc ) {

			/* Update program counter */
			tcg_gen_movi_i32 ( cpu_pc, dc->pc );

			/* Raise exception */
			excp = tcg_const_i32 ( EXCP_DEBUG );
			gen_helper_exception ( cpu_env, excp );
			tcg_temp_free_i32 ( excp );

			/* Stop translation */
			dc->is_jmp = DISAS_UPDATE;

			return 1;
		}
	}

	return 0;
}

/**
 * Fill in TCG opcode metadata
 *
 * @v dc		Disassembly context
 */
static void m6502_opcode_metadata ( DisasContext *dc ) {
	unsigned int offset;

	/* Clear the "start of instruction" flag for all opcodes since the
	 * start of the preceding instruction.
	 */
	offset = ( tcg_ctx.gen_opc_ptr - tcg_ctx.gen_opc_buf );
	if ( dc->offset < offset ) {
		dc->offset++;
		while ( dc->offset < offset )
			tcg_ctx.gen_opc_instr_start[dc->offset++] = 0;
	}

	/* Record metadata for this opcode's instruction */
	tcg_ctx.gen_opc_pc[offset] = dc->pc;
	tcg_ctx.gen_opc_instr_start[offset] = 1;
	tcg_ctx.gen_opc_icount[offset] = dc->num_insns;
}

/**
 * Generate a single instruction
 *
 * @v dc		Disassembly context
 */
static size_t m6502_gen_instruction ( DisasContext *dc ) {
	const M6502Instruction *insn;
	uint8_t opcode;

	/* Fetch and validate opcode */
	opcode = cpu_ldub_code ( dc->env, dc->pc );
	dc->insn = insn = &m6502_instructions[opcode];
	if ( ! insn->gen ) {
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

/**
 * Generate a block of instructions
 *
 * @v env		CPU state
 * @v tb		Translation block
 * @v search_pc		Generate opcode metadata
 */
static void m6502_gen_intermediate_code_internal ( CPUM6502State *env,
						   struct TranslationBlock *tb,
						   int search_pc ) {
	DisasContext ctx;
	DisasContext *dc = &ctx;
	hwaddr pc_start = tb->pc;
	uint16_t *gen_opc_end;
	unsigned int max_insns;
	size_t len;

	/* Initialise disassembly context */
	memset ( dc, 0, sizeof ( *dc ) );
	dc->env = env;
	dc->pc = pc_start;
	dc->tb = tb;
	dc->is_jmp = DISAS_NEXT;
	dc->offset = -1;

	/* Initialise parameters */
	gen_opc_end = ( tcg_ctx.gen_opc_buf + OPC_MAX_SIZE );
	max_insns = ( tb->cflags & CF_COUNT_MASK );
	if ( max_insns == 0 )
		max_insns = CF_COUNT_MASK;
	gen_icount_start();

	/* Main instruction generation loop */
	do {
		/* Check for breakpoints */
		if ( m6502_check_breakpoints ( dc ) )
			break;

		/* Fill in opcode metadata, if applicable */
		if ( search_pc )
			m6502_opcode_metadata ( dc );

		/* Allow I/O on last instruction if applicable(?) */
		if ( ( ( dc->num_insns + 1 ) == max_insns ) &&
		     ( tb->cflags & CF_LAST_IO ) ) {
			gen_io_start();
		}

		/* Generate instruction */
		len = m6502_gen_instruction ( dc );
		dc->pc += len;
		dc->num_insns++;

	} while ( ( dc->is_jmp == DISAS_NEXT ) &&
		  ( tcg_ctx.gen_opc_ptr < gen_opc_end ) &&
		  ( ! env->singlestep_enabled ) &&
		  ( ! singlestep ) &&
		  ( dc->num_insns < max_insns ) );

	/* Disallow I/O if applicable(?) */
	if ( tb->cflags & CF_LAST_IO )
		gen_io_end();

	/* Update program counter and stop translation, if not already done */
	if ( dc->is_jmp == DISAS_NEXT ) {
		tcg_gen_movi_i32 ( cpu_pc, dc->pc );
		tcg_gen_exit_tb ( 0 );
	}

	/* Finalise translation block */
	gen_icount_end ( tb, dc->num_insns );
	*tcg_ctx.gen_opc_ptr = INDEX_op_end;
	if ( search_pc ) {
		m6502_opcode_metadata ( dc );
		tcg_ctx.gen_opc_instr_start[dc->offset] = 0;
	}
	tb->size = ( dc->pc - pc_start );
	tb->icount = dc->num_insns;
}

/**
 * Generate a block of instructions
 *
 * @v env		CPU state
 * @v tb		Translation block
 */
void m6502_gen_intermediate_code ( CPUM6502State *env,
				   struct TranslationBlock *tb ) {
	m6502_gen_intermediate_code_internal ( env, tb, 0 );
}

/**
 * Generate a block of instructions and program counter information
 *
 * @v env		CPU state
 * @v tb		Translation block
 */
void m6502_gen_intermediate_code_pc ( CPUM6502State *env,
				      struct TranslationBlock *tb ) {
	m6502_gen_intermediate_code_internal ( env, tb, 1 );
}

/**
 * Restore state to opcode within translation block
 *
 * @v env		CPU state
 * @v tb		Translation block
 * @v pc_pos		Offset within opcode list
 */
void m6502_restore_state_to_opc ( CPUM6502State *env,
				  TranslationBlock *tb, int pc_pos ) {
	env->pc = tcg_ctx.gen_opc_pc[pc_pos];
}

/**
 * Initialise translation
 *
 */
void m6502_translate_init ( void ) {

	/* Create TCG variables for all CPU registers */
	cpu_env = tcg_global_reg_new_ptr ( TCG_AREG0, "env" );
	cpu_a.var = tcg_global_mem_new ( TCG_AREG0,
					 offsetof ( CPUM6502State, a ), "a" );
	cpu_x.var = tcg_global_mem_new ( TCG_AREG0,
					 offsetof ( CPUM6502State, x ), "x" );
	cpu_y.var = tcg_global_mem_new ( TCG_AREG0,
					 offsetof ( CPUM6502State, y ), "y" );
	cpu_p_c.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p_c ),
					   "p.c" );
	cpu_p_nz.var = tcg_global_mem_new ( TCG_AREG0,
					    offsetof ( CPUM6502State, p_nz ),
					    "p.nz" );
	cpu_p_i.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p_i ),
					   "p.i" );
	cpu_p_d.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p_d ),
					   "p.d" );
	cpu_p_b.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p_b ),
					   "p.b" );
	cpu_p_u.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p_u ),
					   "p.u" );
	cpu_p_v.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p_v ),
					   "p.v" );
	cpu_p_n.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p_n ),
					   "p.n" );
	cpu_s.var = tcg_global_mem_new ( TCG_AREG0,
					 offsetof ( CPUM6502State, s ), "s" );
	cpu_pc = tcg_global_mem_new ( TCG_AREG0,
				      offsetof ( CPUM6502State, pc ), "pc" );
}
