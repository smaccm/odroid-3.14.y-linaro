#ifndef _DT_BINDINGS_CLOCK_EXYNOS_5410_H
#define _DT_BINDINGS_CLOCK_EXYNOS_5410_H

/* core clocks */
#define CLK_FOUT_APLL 1
#define CLK_FOUT_CPLL 2
#define CLK_FOUT_MPLL 3
#define CLK_FOUT_BPLL 4
#define CLK_FOUT_KPLL 5
#define CLK_FOUT_VPLL 6
#define CLK_FOUT_DPLL 7
#define CLK_FOUT_EPLL 8
#define CLK_FOUT_IPLL 9
#define CLK_FIN_PLL 10

/* gate for special clocks (sclk) */
#define CLK_SCLK_UART0 128
#define CLK_SCLK_UART1 129
#define CLK_SCLK_UART2 130
#define CLK_SCLK_UART3 131
#define CLK_SCLK_MMC0 132
#define CLK_SCLK_MMC1 133
#define CLK_SCLK_MMC2 134
#define CLK_SCLK_HDMIPHY 135
#define CLK_SCLK_PIXEL 136
#define CLK_SCLK_HDMI 137
#define CLK_SCLK_FIMD1 138
#define CLK_SCLK_DP1 139
#define CLK_SCLK_I2S1 140
#define CLK_SCLK_I2S2 141
#define CLK_SCLK_PCM1 142
#define CLK_SCLK_PCM2 143
#define CLK_SCLK_SPDIF 144
#define CLK_SCLK_MAUDIO0 148
#define CLK_SCLK_MAUPCM0 149
#define CLK_SCLK_USBD300 150
#define CLK_SCLK_USBD301 151
#define CLK_SCLK_USBPHY300 152
#define CLK_SCLK_USBPHY301 153
#define CLK_SCLK_EPLL 154
#define CLK_SCLK_SPI0 155
#define CLK_SCLK_SPI1 156
#define CLK_SCLK_SPI2 157

/* gate clocks */
#define CLK_UART0 257
#define CLK_UART1 258
#define CLK_UART2 259
#define CLK_UART3 260
#define CLK_I2C0 261
#define CLK_I2C1 262
#define CLK_I2C2 263
#define CLK_I2C3 264
#define CLK_I2C4 265
#define CLK_I2C5 266
#define CLK_I2C6 267
#define CLK_I2C7 268
#define CLK_I2C_HDMI 269
#define CLK_I2S1 270
#define CLK_I2S2 271
#define CLK_CHIPID 272

#define CLK_PDMA0 275
#define CLK_PDMA1 276

#define CLK_SPI0		277
#define CLK_SPI1		278
#define CLK_SPI2		279

#define CLK_MCT 315
#define CLK_TMU_APBIF 318

#define CLK_MDMA0 346
#define CLK_MDMA1 347

#define CLK_MMC0 351
#define CLK_MMC1 352
#define CLK_MMC2 353
#define CLK_MIXER 354
#define CLK_HDMI 355
#define CLK_FIMD1 356
#define CLK_MIE1 357
#define CLK_DSIM1 358
#define CLK_DP 359
#define CLK_GSCL0 360
#define CLK_GSCL1 361
#define CLK_GSCL2 362
#define CLK_GSCL3 363
#define CLK_GSCL4 364
#define CLK_USBH20 365
#define CLK_USBD300 366
#define CLK_USBD301 367
#define CLK_MFC 401
#define CLK_SMMU_MFCL 402
#define CLK_SMMU_MFCR 403
#define CLK_PWM 404

/* Div clocks */
#define CLK_DIV_HDMI_PIXEL 450

/* mux clocks */
#define CLK_MOUT_HDMI 500
#define CLK_MOUT_AUDIO0 501
#define CLK_NR_CLKS 512

#endif /* _DT_BINDINGS_CLOCK_EXYNOS_5410_H */
