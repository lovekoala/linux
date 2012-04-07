/*
 * arch/arm/mach-sun4i/include/mach/debug-macro.S
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <mach/hardware.h>
#include <mach/uart_regs.h>

	.macro	addruart, rx, tmp
	mrc	p15, 0, \rx, c1, c0
	tst	\rx, #1					@ MMU enabled?
	ldreq	\rx, =PA_UARTS_BASE
	ldrne	\rx, =VA_UARTS_BASE
#if CONFIG_SW_DEBUG_UART != 0
	add	\rp, \rp, #(0x400 * CONFIG_SW_DEBUG_UART)
	add	\rv, \rv, #(0x400 * CONFIG_SW_DEBUG_UART)
#endif
	.endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>
