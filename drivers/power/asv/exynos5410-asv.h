/*
 * Exynos 5410 - Adaptive Supply Voltage Driver Header File
 *
 * copyright (c) 2014 hardkernel., ltd.
 *		http://www.hardkernel.com/
 *
 * Author: Hakjoo Kim <ruppi.kim@hardkernel.com>
 *
 * this program is free software; you can redistribute it and/or modify
 * it under the terms of the gnu general public license version 2 as
 * published by the free software foundation.
*/

#ifndef __EXYNOS5410_ASV_D_H
#define __EXYNOS5410_ASV_D_H __FILE__

#define EXYNOS5410_MAX_GRP_NR   13
#define ARM_GRP_NR		13
#define ARM_LEVEL_NR \
			ARRAY_SIZE(arm_asv_voltage)
#define ARM_MAX_VOLT			1362500

#define KFC_GRP_NR		13
#define KFC_LEVEL_NR \
			ARRAY_SIZE(kfc_asv_voltage)
#define KFC_MAX_VOLT			1312500

#define INT_GRP_NR		13

#define CHIP_ID_OFFSET		0x4
#define CHIP_ID0_OFFSET		0x14
#define CHIP_ID1_OFFSET		0x18
#define CHIP_AUXINFO_OFFSET	0x1c

#define EXYNOS5410_USESG_OFFSET 3
#define EXYNOS5410_USESG_MASK	0x01
#define EXYNOS5410_SG_OFFSET	0
#define EXYNOS5410_SG_MASK	0x07
#define EXYNOS5410_TABLE_OFFSET 8
#define EXYNOS5410_TABLE_MASK	0x03
#define EXYNOS5410_SG_A_OFFSET	17
#define EXYNOS5410_SG_A_MASK	0x0F
#define EXYNOS5410_SG_B_OFFSET	21
#define EXYNOS5410_SG_B_MASK	0x03
#define EXYNOS5410_SG_BSIGN_OFFSET	23
#define EXYNOS5410_SG_BSIGN_MASK	0x01
#define EXYNOS5410_IDS_OFFSET	24
#define EXYNOS5410_IDS_MASK	0xFF
#define EXYNOS5410_TMCB_OFFSET	0
#define EXYNOS5410_TMCB_MASK	0x7F
#define EXYNOS5410_EGLLOCK_UP_OFFSET    (8)
#define EXYNOS5410_EGLLOCK_UP_MASK      (0x03)
#define EXYNOS5410_EGLLOCK_DN_OFFSET    (10)
#define EXYNOS5410_EGLLOCK_DN_MASK      (0x03)
#define EXYNOS5410_KFCLOCK_UP_OFFSET    (12)
#define EXYNOS5410_KFCLOCK_UP_MASK      (0x03)
#define EXYNOS5410_KFCLOCK_DN_OFFSET    (14)
#define EXYNOS5410_KFCLOCK_DN_MASK      (0x03)
#define EXYNOS5410_INTLOCK_UP_OFFSET    (16)
#define EXYNOS5410_INTLOCK_UP_MASK      (0x03)
#define EXYNOS5410_INTLOCK_DN_OFFSET    (18)
#define EXYNOS5410_INTLOCK_DN_MASK      (0x03)
#define EXYNOS5410_MIFLOCK_UP_OFFSET    (20)
#define EXYNOS5410_MIFLOCK_UP_MASK      (0x03)
#define EXYNOS5410_MIFLOCK_DN_OFFSET    (22)
#define EXYNOS5410_MIFLOCK_DN_MASK      (0x03)
#define EXYNOS5410_G3DLOCK_UP_OFFSET    (24)
#define EXYNOS5410_G3DLOCK_UP_MASK      (0x03)
#define EXYNOS5410_G3DLOCK_DN_OFFSET    (26)
#define EXYNOS5410_G3DLOCK_DN_MASK      (0x03)
#define BASE_VOLTAGE_OFFSET 1000000

static const unsigned int arm_asv_voltage
[][ARM_GRP_NR + 1] = {
	{ 2100000,
		1362500, 1362500, 1350000, 1337500, 1325000,
		1312500, 1300000, 1275000, 1262500, 1250000,
		1237500, 1225000, 1212500 },
	{ 2000000,
		1287500, 1275000, 1262500, 1250000, 1237500,
		1250000, 1237500, 1225000, 1212500, 1187500,
		1162500, 1162500, 1150000 },
	{ 1900000,
		1287500, 1275000, 1262500, 1250000, 1237500,
		1250000, 1237500, 1225000, 1212500, 1187500,
		1162500, 1162500, 1150000 },
	{ 1800000,
		1287500, 1275000, 1262500, 1250000, 1237500,
		1250000, 1150000, 1212500, 1212500, 1187500,
		1162500, 1162500, 1150000 },
	{ 1700000,
		1237500, 1225000, 1212500, 1200000, 1187500,
		1200000, 1187500, 1175000, 1162500, 1137500,
		1112500, 1112500, 1062500 },
	{ 1600000,
		1200000, 1187500, 1175000, 1162500, 1150000,
		1162500, 1150000, 1137000, 1125000, 1100000,
		1075000, 1075000, 1062500 },
	{ 1500000,
		1175000, 1162500, 1150000, 1137500, 1125000,
		1137500, 1125000, 1112500, 1100000, 1075000,
		1050000, 1050000,  1037500 },
	{ 1400000,
		1137500, 1125000, 1112500, 1100000, 1087500,
		1100000, 1087500, 1075000, 1062500, 1037500,
		1012500, 1012500, 1000000 },
	{ 1300000,
		1100000, 1087500, 1075000, 1062500, 1050000,
		1062500, 1050000, 1037500, 1025000, 1000000,
		 975000,  975000,  975000 },
	{ 1200000,
		1062500, 1050000, 1037500, 1012500, 1012500,
		1025000, 1012500, 1000000,  987500,  975000,
		 962500,  962500,  950000 },
	{ 1100000,
		1037500, 1025000, 1025000, 1000000,  987500,
		1000000,  987500,  975000,  962500,  950000,
		 900000,  900000,  900000 },
	{ 1000000,
		1000000,  987500,  975000,  962500,  950000,
		 962500,  950000,  937500,  937500,  925000,
		 925000,  925000,  912500 },
	{ 900000,
		 962500,  950000,  937500,  925000,  912500,
		 912500,  912500,  912500,  912500,  912500,
		 912500,  912500,  912500 },
	{ 800000,
		 925000,  912500,  900000,  900000,  900000,
		 900000,  900000,  900000,  900000,  900000,
		 900000,  900000,  900000 },
	{ 700000,
		 925000,  912500,  900000,  900000,  900000,
		 900000,  900000,  900000,  900000,  900000,
		 900000,  900000,  900000 },
	{ 600000,
		 925000,  912500,  900000,  900000,  900000,
		 900000,  900000,  900000,  900000,  900000,
		 900000,  900000,  900000 },
	{ 500000,
		 925000,  912500,  900000,  900000,  900000,
		 900000,  900000,  900000,  900000,  900000,
		 900000,  900000,  900000 },
	{ 400000,
		 925000,  912500,  900000,  900000,  900000,
		 900000,  900000,  900000,  900000,  900000,
		 900000,  900000,  900000 },
	{ 300000,
		 925000,  912500,  900000,  900000,  900000,
		 900000,  900000,  900000,  900000,  900000,
		 900000,  900000,  900000 },
	{ 200000,
		 925000,  925000,  912500,  900000,  900000,
		 900000,  900000,  900000,  900000,  900000,
		 900000,  900000,  900000 },
};

static const unsigned int kfc_asv_voltage
[][KFC_GRP_NR + 1] = {
	{ 1500000,
		1312500, 1312500, 1312500, 1312500, 1312500,
		1312500, 1312500, 1312500, 1287500, 1287500,
		1262500, 1250000, 1237500 },
	{ 1400000,
		1312500, 1312500, 1312500, 1312500, 1287500,
		1312500, 1287500, 1287500, 1262500, 1250000,
		1237500, 1225000, 1212500 },
	{ 1300000,
		1312500, 1312500, 1312500, 1312500, 1287500,
		1262500, 1250000, 1337500, 1225000, 1212500,
		1200000, 1187500, 1175000 },
	{ 1200000,
		1312500, 1287500, 1262500, 1250000, 1237500,
		1225000, 1212500, 1200000, 1187500, 1175000,
		1175000, 1175000, 1175000 },
	{ 1100000,
		1250000, 1225000, 1200000, 1187500, 1175000,
		1162500, 1150000, 1137500, 1125000, 1112500,
		1112500, 1112500, 1112500 },
	{ 1000000,
		1187500, 1162500, 1137500, 1125000, 1112500,
	    1100000, 1087500, 1075000, 1000000, 987500,
		 987500,  987500,  987500 },
	{ 900000,
	    1125000, 1100000, 1075000, 1062500, 1050000,
		1037500, 1025000, 1012500, 1000000,  987500,
		 987500,  987500,  987500 },
	{ 800000,
		1075000, 1050000, 1025000, 1012500, 1000000,
		 987500,  975000,  962500,  950000,  950000,
		 950000,  950000,  950000 },
	{ 700000,
		1025000, 1000000,  975000,  962500,  950000,
		 950000,  950000,  950000,  950000,  950000,
		 950000,  950000,  950000 },
	{ 600000,
		 975000,  950000,  950000,  950000,  950000,
		 950000,  950000,  950000,  950000,  950000,
		 950000,  950000,  950000 },
	{ 500000,
		 950000,  950000,  950000,  950000,  950000,
		 950000,  950000,  950000,  950000,  950000,
		 950000,  950000,  950000 },
	{ 400000,
		 950000,  950000,  950000,  950000,  950000,
		 950000,  950000,  950000,  950000,  950000,
		 950000,  950000,  950000 },
	{ 300000,
		 950000,  950000,  950000,  950000,  950000,
		 950000,  950000,  950000,  950000,  950000,
		 950000,  950000,  950000 },
	{ 200000,
		 950000,  950000,  950000,  950000,  950000,
		 950000,  950000,  950000,  950000,  950000,
		 950000,  950000,  950000 },
};

static const unsigned int 
exynos5410_refer_table_get_asv[2] [ARM_LEVEL_NR] = {
    { 0, 9, 13, 17, 22, 29, 37, 47, 58, 72,  87, 100, 999},
    { 0, 0, 40, 43, 46, 49, 52, 55, 58, 60, 999, 999, 999},
};

static const unsigned int 
exynos5410_refer_use_table_get_asv[2] [ARM_LEVEL_NR] = {
    { 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    { 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
};

static unsigned int arm_asv_abb_info[ARM_GRP_NR] = {
    ABB_X080, ABB_X080, ABB_X080, ABB_X080, ABB_X080, ABB_BYPASS, ABB_BYPASS, ABB_BYPASS, ABB_BYPASS, ABB_BYPASS, ABB_BYPASS, ABB_X120, ABB_X120
};

static unsigned int int_asv_abb_info[INT_GRP_NR] = {
    ABB_X120, ABB_X120, ABB_X120, ABB_X120, ABB_X120, ABB_X120, ABB_X120, ABB_X120, ABB_X120, ABB_X120, ABB_X120, ABB_X120, ABB_X120
};

struct exynos5410_asv_info {
	unsigned int product_id;
	unsigned int lot_id;
	unsigned int package_id;
	unsigned int aux_info;
	unsigned int ids;
	unsigned int hpm;
	/* we may need more info as global data */
};

enum table_version {
    ASV_TABLE_VER0,
    ASV_TABLE_VER1,
    ASV_TABLE_VER2,
    ASV_TABLE_BIN2,
};

enum volt_offset {
    VOLT_OFFSET_0MV,
    VOLT_OFFSET_25MV,
    VOLT_OFFSET_50MV,
    VOLT_OFFSET_75MV,
};

#endif	/* __EXYNOS5410_ASV_D_H */
