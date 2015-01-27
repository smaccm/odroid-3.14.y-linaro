/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See the COPYING file in the top-level directory.
 */

#include <linux/types.h>

#define HVC ".arch_extension virt\n hvc "
#define __SWINUM(x) ((x) & 0x00ffffff)

typedef enum {
	seL4_SysCall = -1,
	seL4_SysReplyWait = -2,
	seL4_SysSend = -3,
	seL4_SysNBSend = -4,
	seL4_SysWait = -5,
	seL4_SysReply = -6,
	seL4_SysYield = -7,
	seL4_SysDebugPutChar = -8,
} seL4_Syscall_ID;

static inline void
seL4_Notify(uint32_t dest)
{
	register uint32_t destptr asm("r0") = dest;
	register uint32_t info asm("r1") = 1;
	register uint32_t msg0 asm("r2") = 0;

	/* Perform the system call. */
	register uint32_t scno asm("r7") = seL4_SysSend;
	asm volatile (HVC "%[swi_num]"
			: "+r" (destptr), "+r" (msg0), "+r" (info)
			: [swi_num] "i" __SWINUM(seL4_SysSend), "r"(scno)
			: "memory");
}


void seL4_DebugPuts(const char *str);
void seL4_DebugPutChar(char c);
