/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef HLUSB_REG_H
#define HLUSB_REG_H

#define UDELAY_LEVEL1                (10)
#define UDELAY_LEVEL2                (20)
#define UDELAY_LEVEL3                (50)
#define UDELAY_LEVEL4                (100)
#define UDELAY_LEVEL5                (1000)
#define UDELAY_LEVEL6                (3000)

#define COMBOPHY0_MODE_MASK          (0x3<<8)
#define COMBOPHY0_MODE_USB           (0x1<<8)
#define PERI_COMBOPHY1_CTRL          0x490
#define COMBOPHY1_MODE_MASK          (0x3<<8)
#define COMBOPHY1_MODE_USB           (0x1<<8)
#define PERI_COMBOPHY2_CTRL          0x064
#define USB_LANE_MODE_MASK           (0xf<<28)
#define USB_LANE_MODE_USB            (0x2<<28)
#define COMBOPHY2_2_MODE_USB         (0x1<<20)
#define COMBOPHY2_2_MODE_MASK        (0x3<<20)
#define COMBOPHY2_3_MODE_USB         (0x1<<22)
#define COMBOPHY2_3_MODE_MASK        (0x3<<22)

#define PERI_COMBOPHY1_CTRL1         0x494
#define PERI_COMBOPHY1_TEST_O        (0xff << 24)
#define PERI_COMBOPHY1_TEST_RST      (0x01 << 17)
#define PERI_COMBOPHY1_TEST_WRITE    (0x01 << 16)
#define PERI_COMBOPHY1_TEST_I        (0xff << 8)
#define PERI_COMBOPHY1_TEST_ADD      0xff

#define PERI_USB2_CTRL_CFG0          0x600
#define PERI_USB2_CTRL_CFG1          0x610
#define PERI_USB31_CTRL_CFG0         0x620
#define PERI_USB31_CTRL_CFG1         0x640
#define PERI_USB31_CTRL_CFG2         0x660
#define PERI_CTRL_CFG_DEFAULT_VAL    0x020002ff
#define PERI_PORT_PWR_CTL_EN         (0x01<<4)
#define PERI_PORT_PWREN_POL          (0x01<<6)
#define PERI_PORT_OVRCUR_EN          (0x01<<5)
#define PERI_PORT_OVRCUR_POL         (0x01<<7)
#define PERI_USB3_PORT_DISABLE       (0x01<<9)
#define PERI_USB3_PORT_FORCE_5G      (0x01<<10)
#define PERI_USB3_PORT_DISABLE_CSU30 (0x01<<12)

/* phy eye related */
#define PERI_COMBOPHY0_CTRL1     0x484
#define PERI_COMBOPHY1_CTRL1     0x494
#define PERI_COMBOPHY2_CTRL1     0x0

#define PERI_COMBPHY_TEST_O           (0xffU << 24)
#define PERI_COMBPHY23_TEST_RST_N     BIT(20)
#define PERI_COMBPHY22_TEST_RST_N     BIT(19)
#define PERI_COMBPHY21_TEST_RST_N     BIT(18)
#define PERI_COMBPHY20_TEST_RST_N     BIT(17)
#define PERI_COMBPHY1_TEST_RST_N      BIT(17)
#define PERI_COMBPHY0_TEST_RST_N      BIT(17)
#define PERI_COMBPHY_TEST_WRITE       BIT(16)
#define PERI_COMBPHY_TEST_I           (0xffU << 8)
#define PERI_COMBPHY_TEST_ADDR        0xffU
#define PERI_COMBPHY_DATA_OFFSET      0x8U
#define PERI_COMBPHY_ADDR_OFFSET      0x0U

#define X1_SLEW_ASSIST_DIS_AND_SSC_ADDR        0x04
#define X1_SLEW_ASSIST_DIS_AND_SSC_VAL         0x12
#define X1_TX_SWING_COMP_ADDR                  0x0B
#define X1_TX_SWING_COMP_VAL                   0xA0
#define X1_CDR_DIRECT_TRIM_EQ_PEAKING_ADDR     0x10
#define X1_CDR_DIRECT_TRIM_EQ_PEAKING_VAL      0x39
#define X1_DFE_DIS_8G10G_ADDR                  0x1B
#define X1_DFE_DIS_8G10G_VAL                   0x1F
#define EQ_SWING_INC_PEAK_FREQ_ADDR            0x12
#define EQ_SWING_INC_PEAK_FREQ_VAL             0x96
#define X1_TXPLL_TRIM_ADDR                     0x00
#define X1_TXPLL_TRIM_VAL                      0x30
#define X1_REF_CLK_100N250_ADDR                0x01
#define X1_REF_CLK_100N250_VAL                 0x40
#define X1_EQ_INIT_MANUAL_SET1_ADDR            0x14
#define X1_EQ_INIT_MANUAL_SET1_VAL             0x6A
#define X1_EQ_INIT_PWON_CDR_MANUAL_SET1_ADDR   0x05
#define X1_EQ_INIT_PWON_CDR_MANUAL_SET1_VAL    0x10
#define X1_EQ_INIT_MANUAL_SET0_ADDR            0x14
#define X1_EQ_INIT_MANUAL_SET0_VAL             0x2A
#define X1_EQ_INIT_PWON_CDR_MANUAL_SET0_ADDR   0x05
#define X1_EQ_INIT_PWON_CDR_MANUAL_SET0_VAL    0x00

#define X4_LANED_SLEW_ASSIST_DIS_AND_SSC_ADDR  0xC4
#define X4_LANED_SLEW_ASSIST_DIS_AND_SSC_VAL   0x12
#define X4_LANED_TW_SWING_COMP_ADDR            0xCB
#define X4_LANED_TW_SWING_COMP_VAL             0xA0
#define X4_LANED_CDR_DIRECT_TRIM_ADDR          0xD0
#define X4_LANED_CDR_DIRECT_TRIM_VAL           0x39
#define X4_LANED_DFE_DIS_8G10G_ADDR            0xDB
#define X4_LANED_DFE_DIS_8G10G_VAL             0x1F
#define X4_LANED_EQ_SWING_ADDR                 0xD2
#define X4_LANED_EQ_SWING_VAL                  0x96
#define X4_LANED_TXPLL_TRIM_ADDR               0xC0
#define X4_LANED_TXPLL_TRIM_VAL                0x00
#define X4_LANED_REF_CLK_100N250_ADDR          0xC1
#define X4_LANED_REF_CLK_100N250_VAL           0x40
#define X4_LANED_INV_RXCDRCLK_ADDR             0xC8
#define X4_LANED_INV_RXCDRCLK_VAL              0x08
#define X4_LANED_EQ_INIT_MANUAL_SET1_ADDR      0xD4
#define X4_LANED_EQ_INIT_MANUAL_SET1_VAL       0x6A
#define X4_LANED_EQ_INIT_PWON_CDR_MANUAL_SET1_ADDR 0xC5
#define X4_LANED_EQ_INIT_PWON_CDR_MANUAL_SET1_VAL  0x10
#define X4_LANED_EQ_INIT_MANUAL_SET0_ADDR          0xD4
#define X4_LANED_EQ_INIT_MANUAL_SET0_VAL           0x2A
#define X4_LANED_EQ_INIT_PWON_CDR_MANUAL_SET0_ADDR 0xC5
#define X4_LANED_EQ_INIT_PWON_CDR_MANUAL_SET0_VAL  0x00

#define X4_LANEC_SLEW_ASSIST_DIS_AND_SSC_ADDR  0x84
#define X4_LANEC_SLEW_ASSIST_DIS_AND_SSC_VAL   0x12
#define X4_LANEC_TW_SWING_COMP_ADDR            0x8B
#define X4_LANEC_TW_SWING_COMP_VAL             0xA0
#define X4_LANEC_CDR_DIRECT_TRIM_ADDR          0x90
#define X4_LANEC_CDR_DIRECT_TRIM_VAL           0x39
#define X4_LANEC_DFE_DIS_8G10G_ADDR            0x9B
#define X4_LANEC_DFE_DIS_8G10G_VAL             0x1F
#define X4_LANEC_EQ_SWING_ADDR                 0x92
#define X4_LANEC_EQ_SWING_VAL                  0x96
#define X4_LANEC_TXPLL_TRIM_ADDR               0x80
#define X4_LANEC_TXPLL_TRIM_VAL                0x00
#define X4_LANEC_REF_CLK_100N250_ADDR          0x81
#define X4_LANEC_REF_CLK_100N250_VAL           0x40
#define X4_LANEC_EQ_INIT_MANUAL_SET1_ADDR      0x94
#define X4_LANEC_EQ_INIT_MANUAL_SET1_VAL       0x6A
#define X4_LANEC_EQ_INIT_PWON_CDR_MANUAL_SET1_ADDR 0x85
#define X4_LANEC_EQ_INIT_PWON_CDR_MANUAL_SET1_VAL  0x10
#define X4_LANEC_EQ_INIT_MANUAL_SET0_ADDR          0x94
#define X4_LANEC_EQ_INIT_MANUAL_SET0_VAL           0x2A
#define X4_LANEC_EQ_INIT_PWON_CDR_MANUAL_SET0_ADDR 0x85
#define X4_LANEC_EQ_INIT_PWON_CDR_MANUAL_SET0_VAL  0x00

#endif
