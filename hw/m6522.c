/*
 * MOS Technology 6522 virtual VIA
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

#include "hw.h"
#include "boards.h"
#include "exec/address-spaces.h"
#include "m6522.h"

#define DEBUG_M6522 1

/* Debug messages */
#define LOG_M6522(...) do {						\
		if ( DEBUG_M6522 ) {					\
			qemu_log_mask ( CPU_LOG_IOPORT, __VA_ARGS__ );	\
		}							\
	} while ( 0 )

/******************************************************************************
 *
 * Interrupts
 *
 */

/**
 * Update 6522 VIA interrupt request
 *
 * @v via		6522 VIA
 */
static void m6522_update_irq ( M6522VIA *via ) {
	int irq;

	/* Recalculate IRQ */
	via->ifr &= ~M6522_INT_IRQ;
	irq = ( !! ( via->ifr & via->ier ) );
	if ( irq )
		via->ifr |= M6522_INT_IRQ;

	/* Report IRQ to CPU */
	qemu_set_irq ( via->irq, irq );
}

/**
 * Set 6522 VIA interrupt flags
 *
 * @v via		6522 VIA
 * @v mask		Mask of interrupts to set
 */
static inline void m6522_set_interrupts ( M6522VIA *via, uint8_t mask ) {

	/* Set interrupts in IFR */
	via->ifr |= ( mask & ~M6522_INT_IRQ );

	/* Update interrupt request */
	m6522_update_irq ( via );
}

/**
 * Clear 6522 VIA interrupt flags
 *
 * @v via		6522 VIA
 * @v mask		Mask of interrupts to clear
 */
static inline void m6522_clear_interrupts ( M6522VIA *via, uint8_t mask ) {

	/* Clear interrupts in IFR */
	via->ifr &= ~( mask & ~M6522_INT_IRQ );

	/* Update interrupt request */
	m6522_update_irq ( via );
}

/**
 * Read from 6522 VIA IFR
 *
 * @v via		6522 VIA
 * @ret data		Data
 */
static uint8_t m6522_ifr_read ( M6522VIA *via ) {

	/* Read IFR */
	return via->ifr;
}

/**
 * Write to 6522 VIA IFR
 *
 * @v via		6522 VIA
 * @v data		Data
 */
static void m6522_ifr_write ( M6522VIA *via, uint8_t data ) {
	uint8_t mask;

	/* Decode data */
	mask = ( data & ~M6522_INT_IRQ );
	LOG_M6522 ( "%s: IFR clearing 0x%02x\n", via->name, mask );

	/* Clear interrupts */
	m6522_clear_interrupts ( via, mask );
}

/**
 * Read from 6522 VIA IER
 *
 * @v via		6522 VIA
 * @ret data		Data
 */
static uint8_t m6522_ier_read ( M6522VIA *via ) {

	/* Read IER.  Documentation conflicts on whether bit 7
	 * (M6522_INT_IRQ) reads as zero or one; we arbitrarily leave
	 * it as zero.
	 */
	return via->ier;
}

/**
 * Write to 6522 VIA IER
 *
 * @v via		6522 VIA
 * @v data		Data
 */
static void m6522_ier_write ( M6522VIA *via, uint8_t data ) {
	int enable;
	uint8_t mask;

	/* Decode data */
	enable = ( data & M6522_INT_IRQ );
	mask = ( data & ~M6522_INT_IRQ );
	LOG_M6522 ( "%s: IER %s 0x%02x\n", via->name,
		    ( enable ? "enabling" : "disabling" ), mask );

	/* Enable or disable interrupts as specified */
	if ( enable ) {
		via->ier |= mask;
	} else {
		via->ier &= ~mask;
	}

	/* Update interrupt request */
	m6522_update_irq ( via );
}

/**
 * Handle control line 1 interrupt
 *
 * @v opaque		Port
 * @v n			Interrupt number
 * @v level		Interrupt level
 */
static void m6522_c1_irq ( void *opaque, int n, int level ) {
	M6522VIAPort *port = opaque;
	M6522VIA *via = port->via;
	unsigned int pcr = ( via->pcr >> port->pcr_shift );

	/* Do nothing unless level has changed */
	level = ( !! level );
	if ( level == port->c1.previous )
		return;
	port->c1.previous = level;
	LOG_M6522 ( "%s: C%s1 %s\n", via->name, port->name,
		    ( level ? "high" : "low" ) );

	/* Assert interrupt if new level matches sensitive edge */
	if ( level == ( !! ( pcr & M6522_PCR_C1_POSITIVE ) ) )
		m6522_set_interrupts ( via, port->c1.ifr );
}

/**
 * Handle control line 2 interrupt
 *
 * @v opaque		Port
 * @v n			Interrupt number
 * @v level		Interrupt level
 */
static void m6522_c2_irq ( void *opaque, int n, int level ) {
	M6522VIAPort *port = opaque;
	M6522VIA *via = port->via;
	unsigned int pcr = ( via->pcr >> port->pcr_shift );

	/* Do nothing if line is configured as an output */
	if ( pcr & M6522_PCR_C2_OUTPUT )
		return;

	/* Do nothing unless level has changed */
	level = ( !! level );
	if ( level == port->c2.previous )
		return;
	port->c2.previous = level;
	LOG_M6522 ( "%s: C%s2 %s\n", via->name, port->name,
		    ( level ? "high" : "low" ) );

	/* Assert interrupt if new level matches sensitive edge */
	if ( level == ( !! ( pcr & M6522_PCR_C2_INPUT_POSITIVE ) ) )
		m6522_set_interrupts ( via, port->c2.ifr );
}

/******************************************************************************
 *
 * Port I/O
 *
 */

/**
 * Clear interrupts in response to IRA/IRB read or ORA/ORB write
 *
 * @v via		6522 VIA
 * @v port		Port
 */
static void m6522_iror_clear_interrupts ( M6522VIA *via, M6522VIAPort *port ) {
	unsigned int pcr = ( via->pcr >> port->pcr_shift );

	/* Always clear C1 interrupt */
	m6522_clear_interrupts ( via, port->c1.ifr );

	/* Clear C2 interrupt unless configured as an "independent" input */
	if ( ( pcr & ( M6522_PCR_C2_OUTPUT | M6522_PCR_C2_INPUT_INDEPENDENT ) )
	     != M6522_PCR_C2_INPUT_INDEPENDENT ) {
		m6522_clear_interrupts ( via, port->c2.ifr );
	}
}

/**
 * Read from 6522 VIA IRA/IRB
 *
 * @v via		6522 VIA
 * @v port		Port
 * @v handshake		Handshaking enabled
 * @ret data		Data
 */
static uint8_t m6522_ir_read ( M6522VIA *via, M6522VIAPort *port,
			       int handshake ) {
	uint8_t data;

	/* Input from port */
	data = ( port->ops->input ? port->ops->input ( via->opaque ) : 0 );

	/* For IRB (but not IRA), pins programmed as outputs will
	 * always read back the programmed output value.
	 */
	if ( port == &via->b ) {
		data = ( ( data & ~port->ddr ) | ( port->or & port->ddr ) );
	}

	/* Clear interrupts */
	if ( handshake )
		m6522_iror_clear_interrupts ( via, port );

	LOG_M6522 ( "%s: IR%s%s=0x%02x\n", via->name, port->name,
		    ( handshake ? "" : "_NO_HS" ), data );
	return data;
}

/**
 * Write to 6522 VIA ORA/ORB
 *
 * @v via		6522 VIA
 * @v port		Port
 * @v data		Data
 * @v handshake		Handshaking enabled
 */
static void m6522_or_write ( M6522VIA *via, M6522VIAPort *port, uint8_t data,
			     int handshake ) {

	LOG_M6522 ( "%s: OR%s%s=0x%02x\n", via->name, port->name,
		    ( handshake ? "" : "_NO_HS" ), data );

	/* Update stored OR */
	port->or = data;

	/* Output to port */
	if ( port->ops->output )
		port->ops->output ( via->opaque, data );

	/* Clear interrupts */
	if ( handshake )
		m6522_iror_clear_interrupts ( via, port );
}

/**
 * Read from 6522 VIA DDRA/DDRB
 *
 * @v via		6522 VIA
 * @v port		Port
 * @ret data		Data
 */
static uint8_t m6522_ddr_read ( M6522VIA *via, M6522VIAPort *port ) {
	uint8_t data;

	/* Read data direction register */
	data = port->ddr;

	return data;
}

/**
 * Write to 6522 VIA DDRA/DDRB
 *
 * @v via		6522 VIA
 * @v port		Port
 * @v data		Data
 */
static void m6522_ddr_write ( M6522VIA *via, M6522VIAPort *port,
			      uint8_t data ) {

	LOG_M6522 ( "%s: DDR%s=0x%02x\n", via->name, port->name, data );

	/* Write data direction register */
	port->ddr = data;
}

/******************************************************************************
 *
 * Timers
 *
 */

/**
 * Start timer
 *
 * @v via		6522 VIA
 * @v timer		Timer
 */
static void m6522_timer_start ( M6522VIA *via, M6522VIATimer *timer ) {
	int64_t expire_time;

	/* Do nothing if no timeout is set */
	if ( ! timer->counter )
		return;

	/* For timer 2, do nothing if configured to count pulses */
	if ( ( timer == &via->t2 ) && ( via->acr & M6522_ACR_T2_COUNT ) )
		return;

	/* Calculate expiry time */
	expire_time = ( qemu_get_clock_ns ( vm_clock ) +
			( timer->counter * via->tick_ns ) );

	/* Modify timer */
	qemu_mod_timer ( timer->timer, expire_time );
}

/**
 * Handle timer expiry
 *
 * @v opaque		Timer
 */
static void m6522_timer_expired ( void *opaque ) {
	M6522VIATimer *timer = opaque;
	M6522VIA *via = timer->via;

	/* Set interrupt */
	m6522_set_interrupts ( via, timer->ifr );

	/* Automatically restart timer 1 if configured to do so */
	if ( ( timer == &via->t1 ) && ( via->acr & M6522_ACR_T1_CONTINUOUS ) ) {
		timer->counter = timer->latch;
		m6522_timer_start ( via, timer );
	}
}

/**
 * Read from 6522 VIA timer latch low byte
 *
 * @v via		6522 VIA
 * @v timer		Timer
 * @ret data		Data
 */
static uint8_t m6522_latch_read_low ( M6522VIA *via, M6522VIATimer *timer ) {
	uint8_t data;

	/* Read latch low byte */
	data = ( ( timer->latch >> 0 ) & 0xff );

	return data;
}

/**
 * Read from 6522 VIA timer latch high byte
 *
 * @v via		6522 VIA
 * @v timer		Timer
 * @ret data		Data
 */
static uint8_t m6522_latch_read_high ( M6522VIA *via, M6522VIATimer *timer ) {
	uint8_t data;

	/* Read latch high byte */
	data = ( ( timer->latch >> 8 ) & 0xff );

	return data;
}

/**
 * Read from 6522 VIA timer counter low byte
 *
 * @v via		6522 VIA
 * @v timer		Timer
 * @ret data		Data
 */
static uint8_t m6522_counter_read_low ( M6522VIA *via, M6522VIATimer *timer ) {
	uint8_t data;

	/* Read counter low byte */
	data = ( ( timer->counter >> 0 ) & 0xff );

	/* Clear interrupt */
	m6522_clear_interrupts ( via, timer->ifr );

	return data;
}

/**
 * Read from 6522 VIA timer counter high byte
 *
 * @v via		6522 VIA
 * @v timer		Timer
 * @ret data		Data
 */
static uint8_t m6522_counter_read_high ( M6522VIA *via, M6522VIATimer *timer ) {
	uint8_t data;

	/* Read counter high byte */
	data = ( ( timer->counter >> 8 ) & 0xff );

	return data;
}

/**
 * Write to 6522 VIA timer latch low byte
 *
 * @v via		6522 VIA
 * @v timer		Timer
 * @v data		Data
 */
static void m6522_latch_write_low ( M6522VIA *via, M6522VIATimer *timer,
				    uint8_t data ) {

	LOG_M6522 ( "%s: T%sL_L=0x%02x\n", via->name, timer->name, data );

	/* Update latch low byte */
	timer->latch &= ~0xff;
	timer->latch |= data;
}

/**
 * Write to 6522 VIA timer latch high byte
 *
 * @v via		6522 VIA
 * @v timer		Timer
 * @v data		Data
 */
static void m6522_latch_write_high ( M6522VIA *via, M6522VIATimer *timer,
				     uint8_t data ) {

	LOG_M6522 ( "%s: T%sL_H=0x%02x\n", via->name, timer->name, data );

	/* Update latch high byte */
	timer->latch &= ~0xff00;
	timer->latch |= ( data << 8 );

	/* Clear interrupt */
	m6522_clear_interrupts ( via, timer->ifr );
}

/**
 * Write to 6522 VIA timer counter high byte
 *
 * @v via		6522 VIA
 * @v timer		Timer
 * @v data		Data
 */
static void m6522_counter_write_high ( M6522VIA *via, M6522VIATimer *timer,
				       uint8_t data ) {

	LOG_M6522 ( "%s: T%sC_H=0x%02x\n", via->name, timer->name, data );

	/* Update counter high byte */
	timer->counter &= ~0xff00;
	timer->counter |= ( data << 8 );

	/* Update latch high byte (T1 only) */
	if ( timer == &via->t1 ) {
		timer->latch &= ~0xff00;
		timer->latch |= ( data << 8 );
	}

	/* Load counter from latch */
	timer->counter = timer->latch;

	/* Clear interrupts */
	m6522_clear_interrupts ( via, timer->ifr );

	/* Start timer */
	m6522_timer_start ( via, timer );
}

/******************************************************************************
 *
 * Shift register
 *
 */

/**
 * Read from 6522 VIA SR
 *
 * @v via		6522 VIA
 * @ret data		Data
 */
static uint8_t m6522_sr_read ( M6522VIA *via ) {

	/* Clear interrupts */
	m6522_clear_interrupts ( via, M6522_INT_SR );

	qemu_log_mask ( LOG_UNIMP, "%s: unimplemented read from shift "
			"register\n", via->name );
	return 0;
}

/**
 * Write to 6522 VIA SR
 *
 * @v via		6522 VIA
 * @v data		Data
 */
static void m6522_sr_write ( M6522VIA *via, uint8_t data ) {

	/* Clear interrupts */
	m6522_clear_interrupts ( via, M6522_INT_SR );

	qemu_log_mask ( LOG_UNIMP, "%s: unimplemented write 0x%02x to shift "
			"register\n", via->name, data );
}

/******************************************************************************
 *
 * Auxiliary control register
 *
 */

/**
 * Read from 6522 VIA ACR
 *
 * @v via		6522 VIA
 * @ret data		Data
 */
static uint8_t m6522_acr_read ( M6522VIA *via ) {
	uint8_t data;

	/* Read auxiliary control register */
	data = via->acr;

	return data;
}

/**
 * Write to 6522 VIA ACR
 *
 * @v via		6522 VIA
 * @v data		Data
 */
static void m6522_acr_write ( M6522VIA *via, uint8_t data ) {

	LOG_M6522 ( "%s: ACR=0x%02x\n", via->name, data );

	/* Write auxiliary control register */
	via->acr = data;
}

/******************************************************************************
 *
 * Peripheral control register
 *
 */

/**
 * Read from 6522 VIA PCR
 *
 * @v via		6522 VIA
 * @ret data		Data
 */
static uint8_t m6522_pcr_read ( M6522VIA *via ) {
	uint8_t data;

	/* Read peripheral control register */
	data = via->pcr;

	return data;
}

/**
 * Write to 6522 VIA PCR
 *
 * @v via		6522 VIA
 * @v data		Data
 */
static void m6522_pcr_write ( M6522VIA *via, uint8_t data ) {

	LOG_M6522 ( "%s: PCR=0x%02x\n", via->name, data );

	/* Write peripheral control register */
	via->pcr = data;
}

/******************************************************************************
 *
 * VIA
 *
 */

/**
 * Read from 6522 VIA
 *
 * @v opaque		6522 VIA
 * @v addr		Register address
 * @v size		Size of read
 * @ret data		Read data
 */
static uint64_t m6522_read ( void *opaque, hwaddr addr, unsigned int size ) {
	M6522VIA *via = opaque;
	uint8_t data;

	/* Read from specified register */
	switch ( addr ) {
	case M6522_IRB:
		data = m6522_ir_read ( via, &via->b, 1 );
		break;
	case M6522_IRA:
		data = m6522_ir_read ( via, &via->a, 1 );
		break;
	case M6522_DDRB:
		data = m6522_ddr_read ( via, &via->b );
		break;
	case M6522_DDRA:
		data = m6522_ddr_read ( via, &via->a );
		break;
	case M6522_T1C_L:
		data = m6522_counter_read_low ( via, &via->t1 );
		break;
	case M6522_T1C_H:
		data = m6522_counter_read_high ( via, &via->t1 );
		break;
	case M6522_T1L_L:
		data = m6522_latch_read_low ( via, &via->t1 );
		break;
	case M6522_T1L_H:
		data = m6522_latch_read_high ( via, &via->t1 );
		break;
	case M6522_T2C_L:
		data = m6522_counter_read_low ( via, &via->t2 );
		break;
	case M6522_T2C_H:
		data = m6522_counter_read_high ( via, &via->t2 );
		break;
	case M6522_SR:
		data = m6522_sr_read ( via );
		break;
	case M6522_ACR:
		data = m6522_acr_read ( via );
		break;
	case M6522_PCR:
		data = m6522_pcr_read ( via );
		break;
	case M6522_IER:
		data = m6522_ier_read ( via );
		break;
	case M6522_IFR:
		data = m6522_ifr_read ( via );
		break;
	case M6522_IRA_NO_HS:
		data = m6522_ir_read ( via, &via->a, 0 );
		break;
	default:
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented read from "
				"0x%02lx\n", via->name, addr );
		data = 0;
		break;
	}

	return data;
}

/**
 * Write to 6522 VIA
 *
 * @v opaque		6522 VIA
 * @v addr		Register address
 * @v data64		Data to write
 * @v size		Size of write
 */
static void m6522_write ( void *opaque, hwaddr addr, uint64_t data64,
			  unsigned int size ) {
	M6522VIA *via = opaque;
	uint8_t data = data64;

	/* Write to specified register */
	switch ( addr ) {
	case M6522_ORB:
		m6522_or_write ( via, &via->b, data, 1 );
		break;
	case M6522_ORA:
		m6522_or_write ( via, &via->a, data, 1 );
		break;
	case M6522_DDRB:
		m6522_ddr_write ( via, &via->b, data );
		break;
	case M6522_DDRA:
		m6522_ddr_write ( via, &via->a, data );
		break;
	case M6522_T1C_L:
		m6522_latch_write_low ( via, &via->t1, data );
		break;
	case M6522_T1C_H:
		m6522_counter_write_high ( via, &via->t1, data );
		break;
	case M6522_T1L_L:
		m6522_latch_write_low ( via, &via->t1, data );
		break;
	case M6522_T1L_H:
		m6522_latch_write_high ( via, &via->t1, data );
		break;
	case M6522_T2C_L:
		m6522_latch_write_low ( via, &via->t2, data );
		break;
	case M6522_T2C_H:
		m6522_counter_write_high ( via, &via->t2, data );
		break;
	case M6522_SR:
		m6522_sr_write ( via, data );
		break;
	case M6522_PCR:
		m6522_pcr_write ( via, data );
		break;
	case M6522_ACR:
		m6522_acr_write ( via, data );
		break;
	case M6522_IFR:
		m6522_ifr_write ( via, data );
		break;
	case M6522_IER:
		m6522_ier_write ( via, data );
		break;
	case M6522_ORA_NO_HS:
		m6522_or_write ( via, &via->a, data, 0 );
		break;
	default:
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented write 0x%02x to "
				"0x%02lx\n", via->name, data, addr );
		break;
	}
}

/** 6522 VIA operations */
static const MemoryRegionOps m6522_ops = {
	.read = m6522_read,
	.write = m6522_write,
};

/** Virtual machine state description */
static const VMStateDescription vmstate_m6522 = {
	.name = "m6522",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_UINT8 ( b.or, M6522VIA ),
		VMSTATE_UINT8 ( b.ddr, M6522VIA ),
		VMSTATE_UINT8 ( a.or, M6522VIA ),
		VMSTATE_UINT8 ( a.ddr, M6522VIA ),
		VMSTATE_UINT16 ( t1.latch, M6522VIA ),
		VMSTATE_UINT16 ( t1.counter, M6522VIA ),
		VMSTATE_UINT16 ( t2.latch, M6522VIA ),
		VMSTATE_UINT16 ( t2.counter, M6522VIA ),
		VMSTATE_UINT8 ( sr, M6522VIA ),
		VMSTATE_UINT8 ( acr, M6522VIA ),
		VMSTATE_UINT8 ( pcr, M6522VIA ),
		VMSTATE_UINT8 ( ifr, M6522VIA ),
		VMSTATE_UINT8 ( ier, M6522VIA ),
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Initialise 6522 VIA
 *
 * @v parent		Parent memory region
 * @v offset		Offset within parent memory region
 * @v name		Device name
 * @v opaque		Opaque pointer
 * @v ops		VIA operations
 * @v irq		Interrupt request line
 * @v tick_ns		Clock tick duration (in nanoseconds)
 * @ret via		6522 VIA
 */
M6522VIA * m6522_init ( MemoryRegion *parent, hwaddr offset,
			const char *name, void *opaque, const M6522VIAOps *ops,
			qemu_irq irq, unsigned long tick_ns ) {
	M6522VIA *via = g_new0 ( M6522VIA, 1 );

	/* Initialise VIA */
	via->name = name;
	via->opaque = opaque;
	via->irq = irq;
	via->tick_ns = tick_ns;

	/* Initialise ports */
	via->a.via = via;
	via->b.via = via;
	via->a.name = "A";
	via->b.name = "B";
	via->a.ops = &ops->a;
	via->b.ops = &ops->b;
	via->a.pcr_shift = M6522_PCR_CA_SHIFT;
	via->b.pcr_shift = M6522_PCR_CB_SHIFT;
	via->a.c1.ifr = M6522_INT_CA1;
	via->a.c2.ifr = M6522_INT_CA2;
	via->b.c1.ifr = M6522_INT_CB1;
	via->b.c2.ifr = M6522_INT_CB2;
	via->a.c1.irq = qemu_allocate_irqs ( m6522_c1_irq, &via->a, 1 )[0];
	via->b.c1.irq = qemu_allocate_irqs ( m6522_c1_irq, &via->b, 1 )[0];
	via->a.c2.irq = qemu_allocate_irqs ( m6522_c2_irq, &via->a, 1 )[0];
	via->b.c2.irq = qemu_allocate_irqs ( m6522_c2_irq, &via->b, 1 )[0];

	/* Initialise timers */
	via->t1.via = via;
	via->t2.via = via;
	via->t1.name = "1";
	via->t2.name = "2";
	via->t1.ifr = M6522_INT_T1;
	via->t2.ifr = M6522_INT_T2;
	via->t1.timer = qemu_new_timer_ns ( vm_clock, m6522_timer_expired,
					    &via->t1 );
	via->t2.timer = qemu_new_timer_ns ( vm_clock, m6522_timer_expired,
					    &via->t2 );

	/* Register memory region */
	memory_region_init_io ( &via->mr, &m6522_ops, via, via->name,
				M6522_SIZE );
	memory_region_add_subregion ( parent, offset, &via->mr );

	/* Register virtual machine state */
	vmstate_register ( NULL, offset, &vmstate_m6522, via );

	return via;
}
