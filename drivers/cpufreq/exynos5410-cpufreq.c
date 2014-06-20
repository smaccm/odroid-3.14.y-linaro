/*
 * Copyright (c) 2014 Hardkernel Co., Ltd.
 *		http://www.hardkernel.com
 * Author: Hakjoo Kim<ruppi.kim@hardkernel.com>
 *
 * EXYNOS5410 - CPU frequency scaling support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/cpufreq.h>

#include <mach/map.h>
#include <mach/regs-clock.h>

#include <plat/clock.h>

#include <linux/power/asv-driver.h>
#include "exynos-cpufreq.h"

#define CPUFREQ_NUM_LEVELS	(L19 + 1)
#define CPUFREQ_NUM_LEVELS_CA7	(L13 + 1)
#define EXYNOS5_CLKDIV_STATCPU0_MASK	0x11111111
#define EXYNOS5_CLKDIV_STATCPU1_MASK	0x111
#define EXYNOS5410_EVT2_REV_NUM	0x20

static int max_support_idx;
static int max_support_idx_CA7;
static int min_support_idx = (CPUFREQ_NUM_LEVELS - 1);
static int min_support_idx_CA7 = (CPUFREQ_NUM_LEVELS_CA7 - 1);

static struct clk *mout_cpu;
static struct clk *mout_mpll;
static struct clk *mout_apll;
static struct clk *fout_apll;
static struct clk *mout_kfc;
static struct clk *mout_kpll;
static struct clk *fout_kpll;

struct cpufreq_clkdiv {
	unsigned int	index;
	unsigned int	clkdiv;
	unsigned int	clkdiv1;
};

static unsigned int exynos5410_volt_table[CPUFREQ_NUM_LEVELS];
static struct cpufreq_frequency_table exynos5410_freq_table[] = {
	{L0,  2100 * 1000},
	{L1,  2000 * 1000},
	{L2,  1900 * 1000},
	{L3,  1800 * 1000},
	{L4,  1700 * 1000},
	{L5,  1600 * 1000},
	{L6,  1500 * 1000},
	{L7,  1400 * 1000},
	{L8,  1300 * 1000},
	{L9,  1200 * 1000},
	{L10, 1100 * 1000},
	{L11, 1000 * 1000},
	{L12,  900 * 1000},
	{L13,  800 * 1000},
	{L14,  700 * 1000},
	{L15,  600 * 1000},
	{L16,  500 * 1000},
	{L17,  400 * 1000},
	{L18,  300 * 1000},
	{L19,  200 * 1000},
	{0, CPUFREQ_TABLE_END},
};

static unsigned int exynos5410_volt_table_CA7[CPUFREQ_NUM_LEVELS_CA7];
static struct cpufreq_frequency_table exynos5410_freq_table_CA7[] = {
	{L0, 1500 * 1000},
	{L1, 1400 * 1000},
	{L2, 1300 * 1000},
	{L3, 1200 * 1000},
	{L4, 1100 * 1000},
	{L5, 1000 * 1000},
	{L6,  900 * 1000},
	{L7,  800 * 1000},
	{L8,  700 * 1000},
	{L9,  600 * 1000},
	{L10, 500 * 1000},
	{L11, 400 * 1000},
	{L12, 300 * 1000},
	{L13, 200 * 1000},
	{0, CPUFREQ_TABLE_END},
};

static struct cpufreq_clkdiv
			exynos5410_clkdiv_table_CA7[CPUFREQ_NUM_LEVELS_CA7];
static struct cpufreq_clkdiv exynos5410_clkdiv_table[CPUFREQ_NUM_LEVELS];

static unsigned int clkdiv_cpu0_5410[CPUFREQ_NUM_LEVELS][5] = {
	/*
	 *  Clock divider values for
	 *  {CPUD, ATB, PCLK_DBG, APLL, ARM2}
	 */
	{ 2, 7, 7, 3, 0 }, /* ARM L0:  2.1GHz */
	{ 2, 7, 7, 3, 0 }, /* ARM L1:  2.0GHz */
	{ 2, 7, 7, 3, 0 }, /* ARM L2:  1.9GHz */
	{ 2, 7, 7, 3, 0 }, /* ARM L3:  1.8GHz */
	{ 2, 7, 7, 3, 0 }, /* ARM L4:  1.7GHz */
	{ 2, 7, 7, 3, 0 }, /* ARM L5:  1.6GHz */
	{ 2, 7, 7, 3, 0 }, /* ARM L6:  1.5GHz */
	{ 2, 7, 7, 3, 0 }, /* ARM L7:  1.4GHz */
	{ 2, 7, 7, 3, 0 }, /* ARM L8:  1.3GHz */
	{ 2, 7, 7, 3, 0 }, /* ARM L9:  1.2GHz */
	{ 2, 7, 7, 3, 0 }, /* ARM L10: 1.1GHz */
	{ 2, 6, 6, 3, 0 }, /* ARM L11: 1.0GHz */
	{ 2, 6, 6, 3, 0 }, /* ARM L12: 900MHz */
	{ 2, 5, 5, 3, 0 }, /* ARM L13: 800MHz */
	{ 2, 5, 5, 3, 0 }, /* ARM L14: 700MHz */
	{ 2, 4, 4, 3, 0 }, /* ARM L15: 600MHz */
	{ 2, 3, 3, 3, 0 }, /* ARM L16: 500MHz */
	{ 2, 3, 3, 3, 0 }, /* ARM L17: 400MHz */
	{ 2, 3, 3, 3, 0 }, /* ARM L18: 300MHz */
	{ 2, 3, 3, 3, 0 }, /* ARM L19: 200MHz */
};

static unsigned int clkdiv_cpu0_5410_CA7[CPUFREQ_NUM_LEVELS_CA7][5] = {
	/*
	 * Clock divider values for
	 * {KFC, ACLK, HPM, PCLK, KPLL}
	 */
	{ 0, 2, 7, 5, 3 },  /* KFC L0:  1.5GHz */
	{ 0, 2, 7, 5, 3 },  /* KFC L1:  1.4GHz */
	{ 0, 2, 7, 5, 3 },  /* KFC L2:  1.3GHz */
	{ 0, 2, 7, 5, 3 },  /* KFC L3:  1.2GHz */
	{ 0, 2, 7, 5, 3 },  /* KFC L4:  1.1GHz */
	{ 0, 2, 7, 5, 3 },  /* KFC L5:  1.0GHz */
	{ 0, 2, 7, 5, 3 },  /* KFC L6:  900MHz */
	{ 0, 2, 7, 5, 3 },  /* KFC L7:  800MHz */
	{ 0, 2, 7, 4, 3 },  /* KFC L8:  700MHz */
	{ 0, 2, 7, 4, 3 },  /* KFC L9:  600MHz */
	{ 0, 2, 7, 4, 3 },  /* KFC L10: 500MHz */
	{ 0, 2, 7, 3, 3 },  /* KFC L11: 400MHz */
	{ 0, 2, 7, 3, 3 },  /* KFC L12: 300MHz */
	{ 0, 2, 7, 3, 3 },  /* KFC L13: 200MHz */
};

unsigned int clkdiv_cpu1_5410[CPUFREQ_NUM_LEVELS][2] = {
	/* Clock divider values for { copy, HPM } */
	{ 7, 7 }, /* ARM L0:  2.1GHz */
	{ 7, 7 }, /* ARM L1:  2.0GHz */
	{ 7, 7 }, /* ARM L2:  1.9GHz */
	{ 7, 7 }, /* ARM L3:  1.8GHz */
	{ 7, 7 }, /* ARM L4:  1.7GHz */
	{ 7, 7 }, /* ARM L5:  1.6GHz */
	{ 7, 7 }, /* ARM L6:  1.5GHz */
	{ 7, 7 }, /* ARM L7:  1.4GHz */
	{ 7, 7 }, /* ARM L8:  1.3GHz */
	{ 7, 7 }, /* ARM L9:  1.2GHz */
	{ 7, 7 }, /* ARM L10: 1.1GHz */
	{ 7, 7 }, /* ARM L11: 1.0GHz */
	{ 7, 7 }, /* ARM L12: 900MHz */
	{ 7, 7 }, /* ARM L13: 800MHz */
	{ 7, 7 }, /* ARM L14: 700MHz */
	{ 7, 7 }, /* ARM L15: 600MHz */
	{ 7, 7 }, /* ARM L16: 500MHz */
	{ 7, 7 }, /* ARM L17: 400MHz */
	{ 7, 7 }, /* ARM L18: 300MHz */
	{ 7, 7 }, /* ARM L19: 200MHz */
};

/*
 * Default ASV table
 */
static const unsigned int asv_voltage_5410[CPUFREQ_NUM_LEVELS] = {
	1362500,	/* L0  2100 */
	1312500,	/* L1  2000 */
	1300000,	/* L2  1900 */
	1300000,	/* L3  1800 */
	1300000,	/* L4  1700 */
	1300000,	/* L5  1600 */
	1300000,	/* L6  1500 */
	1300000,	/* L7  1400 */
	1300000,	/* L8  1300 */
	1300000,	/* L9  1200 */
	1300000,	/* L10 1100 */
	1225000,	/* L11 1000 */
	1175000,	/* L12  900 */
	1100000,	/* L13  800 */
	1075000,	/* L14  700 */
	1000000,	/* L15  600 */
	 950000,	/* L16  500 */
	 900000,	/* L17  400 */
	 825000,	/* L18  300 */
	 800000,	/* L19  200 */
};

static const unsigned int asv_voltage_5410_CA7[CPUFREQ_NUM_LEVELS_CA7] = {
	1387500,	/* L0 1500 */
	1387500,	/* L1 1400 */
	1387500,	/* L2 1300 */
	1387500,	/* L3 1200 */
	1387500,	/* L4 1100 */
	1387500,	/* L5 1000 */
	1387500,	/* L6  900 */
	1387500,	/* L7  800 */
	1300000,	/* L8  700 */
	1200000,	/* L9  600 */
	1087500,	/* L10 500 */
	1012500,	/* L11 400 */
	 937500,	/* L12 300 */
	 900000,	/* L13 200 */
};

static void exynos5410_set_clkdiv(unsigned int div_index)
{
	unsigned int tmp;

	/* Change Divider - CPU0 for CMU_CPU */
	tmp = exynos5410_clkdiv_table[div_index].clkdiv;
	__raw_writel(tmp, EXYNOS5_CLKDIV_CPU0);

	do {
		cpu_relax();
		tmp = __raw_readl(EXYNOS5_CLKDIV_STATCPU0);
	} while (tmp & EXYNOS5_CLKDIV_STATCPU0_MASK);
	pr_debug("DIV_CPU0[0x%x]\n", __raw_readl(EXYNOS5_CLKDIV_CPU0));

	/* Change Divider - CPU1 for CMU_CPU */
	tmp = exynos5410_clkdiv_table[div_index].clkdiv1;
	__raw_writel(tmp, EXYNOS5_CLKDIV_CPU1);

	do {
		cpu_relax();
		tmp = __raw_readl(EXYNOS5_CLKDIV_STATCPU1);
	} while (tmp & EXYNOS5_CLKDIV_STATCPU1_MASK);
	pr_debug("DIV_CPU1[0x%x]\n", __raw_readl(EXYNOS5_CLKDIV_CPU1));
}

static void exynos5410_set_clkdiv_CA7(unsigned int div_index)
{
	unsigned int tmp;

	/* Change Divider - KFC0 */
	tmp = exynos5410_clkdiv_table_CA7[div_index].clkdiv;
	__raw_writel(tmp, EXYNOS5_CLKDIV_KFC0);

	do {
		cpu_relax();
		tmp = __raw_readl(EXYNOS5_CLKDIV_STAT_KFC0);
	} while (tmp & EXYNOS5_CLKDIV_STATCPU0_MASK);
	pr_debug("DIV_KFC0[0x%x]\n", __raw_readl(EXYNOS5_CLKDIV_KFC0));
}

static void exynos5410_set_apll(unsigned int new_index,
				unsigned int old_index)
{
	unsigned int tmp;
	unsigned long rate;

	if (clk_set_parent(mout_cpu, mout_mpll)) {
		pr_err(KERN_ERR "Unable to set parent %s of clock %s.\n",
			mout_mpll->name, mout_cpu->name);
	}

	do {
		cpu_relax();
		tmp = __raw_readl(EXYNOS5_CLKMUX_STATCPU);
		tmp &= EXYNOS5_CLKMUX_STATCPU_MUXCORE_MASK;
	} while (tmp != (0x2 << EXYNOS5_CLKSRC_CPU_MUXCORE_SHIFT));

	/* 2. Set APLL rate */
	rate = exynos5410_freq_table[new_index].frequency * 1000;
	if (clk_set_rate(fout_apll, rate))
		pr_err("Unable to change apll rate to %lu\n", rate);

	/* 3. MUX_CORE_SEL = APLL */
	if (clk_set_parent(mout_cpu, mout_apll)) {
		pr_err("Unable to set parent %s of clock %s.\n",
				mout_apll->name, mout_cpu->name);
	}

	do {
		cpu_relax();
		tmp = __raw_readl(EXYNOS5_CLKMUX_STATCPU);
		tmp &= EXYNOS5_CLKMUX_STATCPU_MUXCORE_MASK;
	} while (tmp != (0x1 << EXYNOS5_CLKSRC_CPU_MUXCORE_SHIFT));
}

static void exynos5410_set_kpll(unsigned int new_index,
				unsigned int old_index)
{
	unsigned int tmp;
	unsigned long rate;

	/* 0. before change to MPLL, set div for MPLL output */
	if ((new_index < L5) && (old_index < L5))
		exynos5410_set_clkdiv_CA7(L5); /* pll_safe_index of CA7 */

	/* 1. MUX_CORE_SEL = MPLL, KFCCLK uses MPLL for lock time */
	if (clk_set_parent(mout_kfc, mout_mpll))
		pr_err("Unable to set mout_mpll_kfc as parent of mout_kfc\n");

	do {
		cpu_relax();
		tmp = __raw_readl(EXYNOS5_CLKMUX_STAT_KFC);
		tmp &= EXYNOS5_CLKMUX_STATKFC_MUXCORE_MASK;
	} while (tmp != (0x2 << EXYNOS5_CLKSRC_KFC_MUXCORE_SHIFT));

	/* 2. Set KPLL rate */
	rate = exynos5410_freq_table_CA7[new_index].frequency * 1000;
	if (clk_set_rate(fout_kpll, rate))
		pr_err("Unable to change kpll rate to %lu\n", rate);

	/* 3. MUX_CORE_SEL = KPLL */
	if (clk_set_parent(mout_kfc, mout_kpll))
		pr_err("Unable to set mout_kpll as parent of mout_kfc\n");

	do {
		cpu_relax();
		tmp = __raw_readl(EXYNOS5_CLKMUX_STAT_KFC);
		tmp &= EXYNOS5_CLKMUX_STATKFC_MUXCORE_MASK;
	} while (tmp != (0x1 << EXYNOS5_CLKSRC_KFC_MUXCORE_SHIFT));

	/* 4. restore original div value */
	if ((new_index < L5) && (old_index < L5))
		exynos5410_set_clkdiv_CA7(new_index);
}

static bool exynos5410_pms_change(unsigned int old_index,
				  unsigned int new_index)
{
	/*
	 * The Exynos cpufreq driver uses this to determine if it can
	 * avoid changing the CPU voltage and re-parenting the CPU clock
	 * while chaning the PLL rate.  Because we're using CCF to change
	 * the PLL rate, we no longer have access to the PLL divider table,
	 * so we can't tell whether or not we can take the fast path from
	 * here and must always take the slow path.  Since this only affects
	 * a few transitions, there should hopefully be no impact on
	 * performance.
	 */
	return (old_index != new_index);
}
static void exynos5410_set_frequency(unsigned int old_index,
				     unsigned int new_index)
{
	if (old_index > new_index) {
		/* 1. Change the system clock divider values */
		exynos5410_set_clkdiv(new_index);
		/* 2. Change the apll rate */
		exynos5410_set_apll(new_index, old_index);
	} else if (old_index < new_index) {
		/* 1. Change the apll rate */
		exynos5410_set_apll(new_index, old_index);
		/* 2. Change the system clock divider values */
		exynos5410_set_clkdiv(new_index);
	}
}

static void exynos5410_set_frequency_CA7(unsigned int old_index,
					 unsigned int new_index)
{
	if (old_index > new_index) {
		/* 1. Change the system clock divider values */
		exynos5410_set_clkdiv_CA7(new_index);
		/* 2. Change the kpll rate */
		exynos5410_set_kpll(new_index, old_index);
	} else if (old_index < new_index) {
		/* 1. Change the kpll rate */
		exynos5410_set_kpll(new_index, old_index);
		/* 2. Change the system clock divider values */
		exynos5410_set_clkdiv_CA7(new_index);
	}
}

static void __init set_volt_table(void)
{
	unsigned int i;
	unsigned int asv_volt = 0;
	unsigned int exynos5410_rev_num;

	for (i = 0; i < CPUFREQ_NUM_LEVELS; i++) {
#ifdef CONFIG_POWER_EXYNOS_ASV
		asv_volt = asv_get_volt
			(ASV_ARM, exynos5410_freq_table[i].frequency);
#endif
		if (!asv_volt)
			exynos5410_volt_table[i] = asv_voltage_5410[i];
		else
			exynos5410_volt_table[i] = asv_volt;
		pr_debug("CPUFREQ of CA15 L%d : %d uV\n", i,
				exynos5410_volt_table[i]);
	}

	if (soc_is_exynos5410()) {
		exynos5410_rev_num = samsung_rev();
		max_support_idx = L5;
		min_support_idx = L19;
	} else {
		max_support_idx = L3;
		min_support_idx = L13;
	}

	for (i = L0; i < max_support_idx; i++)
		exynos5410_freq_table[i].frequency = CPUFREQ_ENTRY_INVALID;

	for (i = (min_support_idx + 1); i < CPUFREQ_NUM_LEVELS; i++)
		exynos5410_freq_table[i].frequency = CPUFREQ_ENTRY_INVALID;
}

static void __init set_volt_table_CA7(void)
{
	unsigned int i;
	unsigned int asv_volt = 0;

	for (i = 0; i < CPUFREQ_NUM_LEVELS_CA7; i++) {
#ifdef CONFIG_POWER_EXYNOS_ASV
		asv_volt = asv_get_volt(ASV_KFC,
				exynos5410_freq_table_CA7[i].frequency);
#endif
		if (!asv_volt)
			exynos5410_volt_table_CA7[i] = asv_voltage_5410_CA7[i];
		else
			exynos5410_volt_table_CA7[i] = asv_volt;
		pr_debug("CPUFREQ of CA7  L%d : %d uV\n", i,
				exynos5410_volt_table_CA7[i]);
	}

	if (soc_is_exynos5410())
		max_support_idx_CA7 = L4;
	else
		max_support_idx_CA7 = L0;
	min_support_idx_CA7 = L9;

	for (i = L0; i < max_support_idx_CA7; i++)
		exynos5410_freq_table_CA7[i].frequency = CPUFREQ_ENTRY_INVALID;

	for (i = (min_support_idx_CA7 + 1); i < CPUFREQ_NUM_LEVELS_CA7; i++)
		exynos5410_freq_table_CA7[i].frequency = CPUFREQ_ENTRY_INVALID;
}

int exynos5410_cpufreq_CA7_init(struct exynos_dvfs_info *info)
{
	int i;
	unsigned int tmp;
	unsigned long rate;

	set_volt_table_CA7();

	mout_kfc = clk_get(NULL, "mout_kfc");
	if (IS_ERR(mout_kfc))
		goto err_mout_kfc;

	mout_mpll = clk_get(NULL, "mout_mpll");
	if (IS_ERR(mout_mpll)) {
		pr_err("Clock mout_mpll not found\n");
		goto err_mout_mpll;
	}

	rate = clk_get_rate(mout_mpll) / 1000;

	mout_kpll = clk_get(NULL, "mout_kpll");
	if (IS_ERR(mout_kpll))
		goto err_mout_kpll;

	fout_kpll = clk_get(NULL, "fout_kpll");
	if (IS_ERR(fout_kpll))
		goto err_fout_kpll;

	for (i = L0; i < CPUFREQ_NUM_LEVELS_CA7; i++) {
		exynos5410_clkdiv_table_CA7[i].index = i;

		tmp = __raw_readl(EXYNOS5_CLKDIV_KFC0);

		tmp &= ~(EXYNOS5_CLKDIV_KFC0_CORE_MASK |
			EXYNOS5_CLKDIV_KFC0_ACLK_MASK |
			EXYNOS5_CLKDIV_KFC0_HPM_MASK |
			EXYNOS5_CLKDIV_KFC0_PCLK_MASK |
			EXYNOS5_CLKDIV_KFC0_KPLL_MASK);

		tmp |= ((clkdiv_cpu0_5410_CA7[i][0] <<
				EXYNOS5_CLKDIV_KFC0_CORE_SHIFT) |
			(clkdiv_cpu0_5410_CA7[i][1] <<
				EXYNOS5_CLKDIV_KFC0_ACLK_SHIFT) |
			(clkdiv_cpu0_5410_CA7[i][2] <<
				EXYNOS5_CLKDIV_KFC0_HPM_SHIFT) |
			(clkdiv_cpu0_5410_CA7[i][3] <<
				EXYNOS5_CLKDIV_KFC0_PCLK_SHIFT) |
			(clkdiv_cpu0_5410_CA7[i][4] <<
				EXYNOS5_CLKDIV_KFC0_KPLL_SHIFT));

		exynos5410_clkdiv_table_CA7[i].clkdiv = tmp;
	}

	info->mpll_freq_khz = rate;
	info->pm_lock_idx = L0;
	info->pll_safe_idx = L5;
	info->max_support_idx = max_support_idx_CA7;
	info->min_support_idx = min_support_idx_CA7;
	info->cpu_clk = fout_kpll;
	pr_debug("fout_kpll[%lu]\n", clk_get_rate(fout_kpll));
	info->volt_table = exynos5410_volt_table_CA7;
	info->freq_table = exynos5410_freq_table_CA7;
	info->set_freq = exynos5410_set_frequency_CA7;
	info->need_apll_change = exynos5410_pms_change;

	return 0;

err_fout_kpll:
	clk_put(mout_kpll);
err_mout_kpll:
	clk_put(mout_mpll);
err_mout_mpll:
	clk_put(mout_kfc);
err_mout_kfc:

	pr_err("%s: failed initialization\n", __func__);
	return -EINVAL;
}

int exynos5410_cpufreq_init(struct exynos_dvfs_info *info)
{
	int i;
	unsigned int tmp;
	unsigned long rate;

	set_volt_table();

	mout_cpu = clk_get(NULL, "mout_cpu");
	if (IS_ERR(mout_cpu)) {
		pr_err("Clock mout_cpu not found\n");
		goto err_mout_cpu;
	}

	mout_mpll = clk_get(NULL, "mout_mpll");
	if (IS_ERR(mout_mpll)) {
		pr_err("Clock mout_mpll not found\n");
		goto err_mout_mpll_p;
	}

	rate = clk_get_rate(mout_cpu) / 1000;

	mout_apll = clk_get(NULL, "mout_apll");
	if (IS_ERR(mout_apll)) {
		pr_err("Clock mout_apll not found\n");
		goto err_mout_apll_p;
	}

	fout_apll = clk_get(NULL, "fout_apll");
	if (IS_ERR(fout_apll)) {
		pr_err("Clock fout_apll not found\n");
		goto err_fout_apll;
	}

	for (i = L0; i < CPUFREQ_NUM_LEVELS; i++) {
		exynos5410_clkdiv_table[i].index = i;

		tmp = __raw_readl(EXYNOS5_CLKDIV_CPU0);
		tmp &= ~(EXYNOS5_CLKDIV_CPU0_CPUD_MASK |
			EXYNOS5_CLKDIV_CPU0_ATB_MASK |
			EXYNOS5_CLKDIV_CPU0_PCLKDBG_MASK |
			EXYNOS5_CLKDIV_CPU0_APLL_MASK |
			EXYNOS5_CLKDIV_CPU0_CORE2_MASK);

		tmp |= ((clkdiv_cpu0_5410[i][0] <<
			EXYNOS5_CLKDIV_CPU0_CPUD_SHIFT) |
			(clkdiv_cpu0_5410[i][1] <<
			EXYNOS5_CLKDIV_CPU0_ATB_SHIFT) |
			(clkdiv_cpu0_5410[i][2] <<
			EXYNOS5_CLKDIV_CPU0_PCLKDBG_SHIFT) |
			(clkdiv_cpu0_5410[i][3] <<
			EXYNOS5_CLKDIV_CPU0_APLL_SHIFT) |
			(clkdiv_cpu0_5410[i][4] <<
			EXYNOS5_CLKDIV_CPU0_CORE2_SHIFT));

		exynos5410_clkdiv_table[i].clkdiv = tmp;

		tmp = __raw_readl(EXYNOS5_CLKDIV_CPU1);
		tmp &= ~(EXYNOS5_CLKDIV_CPU1_COPY_MASK |
			EXYNOS5_CLKDIV_CPU1_HPM_MASK);
		tmp |= ((clkdiv_cpu1_5410[i][0] <<
				EXYNOS5_CLKDIV_CPU1_COPY_SHIFT) |
			(clkdiv_cpu1_5410[i][1] <<
				EXYNOS5_CLKDIV_CPU1_HPM_SHIFT));

		exynos5410_clkdiv_table[i].clkdiv1 = tmp;
	}

	info->mpll_freq_khz = rate;
	info->pm_lock_idx = L0;
	info->pll_safe_idx = L12;
	info->max_support_idx = max_support_idx;
	info->min_support_idx = min_support_idx;
	info->cpu_clk = fout_apll;
	pr_debug("fout_apll[%lu]\n", clk_get_rate(fout_apll));
	info->volt_table = exynos5410_volt_table;
	info->freq_table = exynos5410_freq_table;
	info->set_freq = exynos5410_set_frequency;
	info->need_apll_change = exynos5410_pms_change;

#if ENABLE_CLKOUT
	tmp = __raw_readl(EXYNOS5_CLKOUT_CMU_CPU);
	tmp &= ~0xffff;
	tmp |= 0x1904;
	__raw_writel(tmp, EXYNOS5_CLKOUT_CMU_CPU);
#endif 
	return 0;

err_fout_apll:
	clk_put(mout_apll);
err_mout_apll_p:
	clk_put(mout_mpll);
err_mout_mpll_p:
	clk_put(mout_cpu);
err_mout_cpu:

	pr_err("%s: failed initialization\n", __func__);
	return -EINVAL;
}
