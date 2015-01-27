/*
 * seL4 debug serial driver
 *
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See the COPYING file in the top-level directory.
 */

#include "sel4.h"
#include <linux/types.h>

void seL4_DebugPutChar(char c){
	register uint32_t arg0 asm("r0") = c;
	register uint32_t scno asm("r7") = seL4_SysDebugPutChar;
	asm volatile(HVC "%[swi_num]" : : "r"(arg0), [swi_num] "i" __SWINUM(seL4_SysDebugPutChar), "r"(scno));
}

void seL4_DebugPuts(const char *str){
	for (; *str != '\0'; str++) {
		seL4_DebugPutChar(*str);
	}
}

