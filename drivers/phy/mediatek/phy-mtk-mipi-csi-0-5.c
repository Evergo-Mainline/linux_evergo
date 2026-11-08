// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MIPI CSI v0.5 driver
 *
 * Copyright (c) 2023, MediaTek Inc.
 * Copyright (c) 2023, BayLibre Inc.
 */

#include <dt-bindings/phy/phy.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "phy-mtk-io.h"
#include "phy-mtk-mipi-csi-0-5-rx-reg.h"

#define CSIXB_OFFSET		0x1000

struct mtk_mipi_cdphy_port {
	struct device *dev;
	void __iomem *base;
	struct phy *phy;
	u32 type;
	u32 mode;
	u32 num_lanes;
	bool mt6833;
};

enum PHY_TYPE {
	DPHY = 0,
	CPHY,
	CDPHY,
};

static void mtk_phy_csi_cdphy_ana_eq_tune(void __iomem *base)
{
	mtk_phy_update_field(base + MIPI_RX_ANA18_CSIXA, RG_CSI0A_L0_T0AB_EQ_IS, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA18_CSIXA, RG_CSI0A_L0_T0AB_EQ_BW, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA1C_CSIXA, RG_CSI0A_L1_T1AB_EQ_IS, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA1C_CSIXA, RG_CSI0A_L1_T1AB_EQ_BW, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA20_CSI0A, RG_CSI0A_L2_T1BC_EQ_IS, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA20_CSI0A, RG_CSI0A_L2_T1BC_EQ_BW, 1);

	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA18_CSIXA, RG_CSI0A_L0_T0AB_EQ_IS, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA18_CSIXA, RG_CSI0A_L0_T0AB_EQ_BW, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA1C_CSIXA, RG_CSI0A_L1_T1AB_EQ_IS, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA1C_CSIXA, RG_CSI0A_L1_T1AB_EQ_BW, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA20_CSI0A, RG_CSI0A_L2_T1BC_EQ_IS, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA20_CSI0A, RG_CSI0A_L2_T1BC_EQ_BW, 1);
}

static void mtk_phy_csi_dphy_ana_eq_tune(void __iomem *base)
{
	mtk_phy_update_field(base + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L0_EQ_IS, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L0_EQ_BW, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L1_EQ_IS, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L1_EQ_BW, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA1C_CSIXA, RG_CSI1A_L2_EQ_IS, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA1C_CSIXA, RG_CSI1A_L2_EQ_BW, 1);

	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L0_EQ_IS, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L0_EQ_BW, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L1_EQ_IS, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L1_EQ_BW, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA1C_CSIXA, RG_CSI1A_L2_EQ_IS, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA1C_CSIXA, RG_CSI1A_L2_EQ_BW, 1);
}

struct mtk_phy_reg {
	u16 off;
	u32 val;
};

static const struct mtk_phy_reg mt6833_phy_golden[] = {
	{ 0x0000, 0x00070010 },
	{ 0x0004, 0x00420844 },
	{ 0x0008, 0x44031010 },
	{ 0x000c, 0x02001010 },
	{ 0x0010, 0x00001010 },
	{ 0x0014, 0x00000045 },
	{ 0x0018, 0x10101010 },
	{ 0x001c, 0x10101010 },
	{ 0x0020, 0x20000000 },
	{ 0x0054, 0x00000111 },
	{ 0x00f0, 0x000000e2 },
	{ 0x1000, 0x00070010 },
	{ 0x1004, 0x00420844 },
	{ 0x1008, 0x44031010 },
	{ 0x100c, 0x02001010 },
	{ 0x1010, 0x00001010 },
	{ 0x1014, 0x00000045 },
	{ 0x1018, 0x10101010 },
	{ 0x101c, 0x10101010 },
	{ 0x1020, 0x20000000 },
	{ 0x1054, 0x00000111 },
	{ 0x10f0, 0x000000e2 },
	{ 0x2010, 0x10000000 },
	{ 0x2014, 0x10000000 },
	{ 0x2020, 0x10000000 },
	{ 0x2024, 0x10000000 },
	{ 0x2028, 0x10000000 },
	{ 0x202c, 0x10000000 },
	{ 0x2030, 0x00000101 },
	{ 0x2040, 0x0000001d },
	{ 0x2044, 0x0000ffff },
	{ 0x2048, 0x000000f0 },
	{ 0x204c, 0x0000ff1d },
	{ 0x2050, 0x3f000404 },
	{ 0x2054, 0x00030000 },
	{ 0x2058, 0x020000e4 },
	{ 0x2060, 0x00700000 },
	{ 0x2064, 0x00700000 },
	{ 0x2068, 0x00700000 },
	{ 0x206c, 0x00700000 },
	{ 0x2080, 0x80000000 },
	{ 0x2088, 0x80000000 },
	{ 0x20f0, 0x000000f0 },
	{ 0x20f4, 0xffffffff },
	{ 0x2110, 0x00005f00 },
	{ 0x2114, 0x00040387 },
	{ 0x2118, 0x00000704 },
	{ 0x2120, 0x00007d00 },
	{ 0x2124, 0x00040387 },
	{ 0x2128, 0x00000704 },
	{ 0x2130, 0x0000af00 },
	{ 0x2134, 0x00040387 },
	{ 0x2138, 0x00000704 },
	{ 0x2140, 0x0000be00 },
	{ 0x2144, 0x00040387 },
	{ 0x2148, 0x00000704 },
	{ 0x2184, 0x00000001 },
	{ 0x2188, 0x01050701 },
	{ 0x2190, 0x00000002 },
	{ 0x2198, 0x0000ffff },
	{ 0x21a0, 0x00000002 },
	{ 0x21a8, 0x0000ffff },
	{ 0x21b0, 0x00000002 },
	{ 0x21b8, 0x0000ffff },
	{ 0x21c0, 0x00000002 },
	{ 0x21c8, 0x0000ffff },
	{ 0x21f0, 0x000000ff },
	{ 0x221c, 0xffff0000 },
	{ 0x2264, 0xffffffff },
	{ 0x2284, 0x0000ffff },
	{ 0x228c, 0x00005161 },
	{ 0x3008, 0x44440012 },
	{ 0x3010, 0x0e492300 },
	{ 0x3014, 0x0e492301 },
	{ 0x3018, 0x12492400 },
	{ 0x3024, 0x09000000 },
	{ 0x3034, 0x09000000 },
	{ 0x3044, 0x09000000 },
	{ 0x3054, 0x09000000 },
	{ 0x3080, 0x0003ffff },
	{ 0x3084, 0x0003ffff },
	{ 0x3088, 0x0003ffff },
	{ 0x308c, 0x0003ffff },
	{ 0x30f0, 0x80000000 },
	{ 0x3100, 0x10100000 },
	{ 0x3104, 0xffffffff },
	{ 0x3130, 0x0f0f0380 },
	{ 0x3134, 0x00005161 },
	{ 0x4000, 0x00074713 },
	{ 0x4004, 0x00420844 },
	{ 0x4008, 0x44001111 },
	{ 0x400c, 0x02001111 },
	{ 0x4010, 0x00001010 },
	{ 0x4014, 0x00000045 },
	{ 0x4018, 0x00090909 },
	{ 0x401c, 0x00090909 },
	{ 0x4020, 0x203f403f },
	{ 0x4024, 0x30030000 },
	{ 0x4040, 0x03330000 },
	{ 0x4054, 0x00000111 },
	{ 0x4060, 0x03330000 },
	{ 0x4070, 0x02210000 },
	{ 0x4080, 0x01110000 },
	{ 0x4090, 0x03070000 },
	{ 0x40a0, 0x231d0000 },
	{ 0x40a4, 0x00001d02 },
	{ 0x40f0, 0x000200e2 },
	{ 0x5000, 0x00070713 },
	{ 0x5004, 0x00420844 },
	{ 0x5008, 0x44001111 },
	{ 0x500c, 0x02001111 },
	{ 0x5014, 0x00000045 },
	{ 0x5018, 0x00090909 },
	{ 0x501c, 0x00090909 },
	{ 0x5020, 0x203f613f },
	{ 0x5024, 0x30030000 },
	{ 0x5040, 0x00330000 },
	{ 0x5054, 0x00000111 },
	{ 0x5060, 0x00330000 },
	{ 0x5070, 0x00210000 },
	{ 0x5080, 0x00110000 },
	{ 0x5090, 0x03030000 },
	{ 0x50a0, 0x52f80000 },
	{ 0x50a4, 0x00007313 },
	{ 0x50f0, 0x000200e2 },
	{ 0x6000, 0x00000f01 },
	{ 0x6004, 0x80403102 },
	{ 0x6010, 0x10000000 },
	{ 0x6014, 0x10000000 },
	{ 0x6020, 0x10201702 },
	{ 0x6024, 0x10201702 },
	{ 0x6028, 0x10201702 },
	{ 0x602c, 0x10201702 },
	{ 0x6030, 0x00000110 },
	{ 0x6034, 0x10101010 },
	{ 0x6040, 0x0000001d },
	{ 0x6044, 0x0000ffff },
	{ 0x6048, 0x000000f0 },
	{ 0x604c, 0x0000ff1d },
	{ 0x6050, 0x3f000404 },
	{ 0x6054, 0x00030000 },
	{ 0x6058, 0x020000e4 },
	{ 0x6060, 0x00700000 },
	{ 0x6064, 0x00700000 },
	{ 0x6068, 0x00700000 },
	{ 0x606c, 0x00700000 },
	{ 0x6080, 0x80000000 },
	{ 0x6088, 0x80000000 },
	{ 0x60a0, 0x0000bbbb },
	{ 0x60f0, 0x000000f0 },
	{ 0x60f4, 0xffffffff },
	{ 0x6110, 0x00005f00 },
	{ 0x6114, 0x00040387 },
	{ 0x6118, 0x00000704 },
	{ 0x6120, 0x00007d00 },
	{ 0x6124, 0x00040387 },
	{ 0x6128, 0x00000704 },
	{ 0x6130, 0x0000af00 },
	{ 0x6134, 0x00040387 },
	{ 0x6138, 0x00000704 },
	{ 0x6140, 0x0000be00 },
	{ 0x6144, 0x00040387 },
	{ 0x6148, 0x00000704 },
	{ 0x6184, 0x00000001 },
	{ 0x6188, 0x01050701 },
	{ 0x6190, 0x00000002 },
	{ 0x6198, 0x0000ffff },
	{ 0x61a0, 0x00000002 },
	{ 0x61a8, 0x0000ffff },
	{ 0x61b0, 0x00000002 },
	{ 0x61b8, 0x0000ffff },
	{ 0x61c0, 0x00000002 },
	{ 0x61c8, 0x0000ffff },
	{ 0x61f0, 0x000000ff },
	{ 0x6264, 0xffffffff },
	{ 0x6284, 0x0000ffff },
	{ 0x628c, 0x00005161 },
	{ 0x7008, 0x44440012 },
	{ 0x7010, 0x0e492300 },
	{ 0x7014, 0x0e492301 },
	{ 0x7018, 0x12492410 },
	{ 0x7024, 0x09000000 },
	{ 0x7034, 0x09000000 },
	{ 0x7044, 0x09000000 },
	{ 0x7054, 0x09000000 },
	{ 0x7080, 0x0003ffff },
	{ 0x7084, 0x0003ffff },
	{ 0x7088, 0x0003ffff },
	{ 0x708c, 0x0003ffff },
	{ 0x70f0, 0x80000000 },
	{ 0x7100, 0x10100000 },
	{ 0x7104, 0xffffffff },
	{ 0x7130, 0x0f0f0380 },
	{ 0x7134, 0x00005161 },
	{ 0x8000, 0x00070010 },
	{ 0x8004, 0x00420844 },
	{ 0x8008, 0x44031010 },
	{ 0x800c, 0x02001010 },
	{ 0x8010, 0x00001010 },
	{ 0x8014, 0x00000045 },
	{ 0x8018, 0x10101010 },
	{ 0x801c, 0x10101010 },
	{ 0x8020, 0x20000000 },
	{ 0x8054, 0x00000111 },
	{ 0x80f0, 0x000000e2 },
	{ 0x9000, 0x00070010 },
	{ 0x9004, 0x00420844 },
	{ 0x9008, 0x44031010 },
	{ 0x900c, 0x02001010 },
	{ 0x9010, 0x00001010 },
	{ 0x9014, 0x00000045 },
	{ 0x9018, 0x10101010 },
	{ 0x901c, 0x10101010 },
	{ 0x9020, 0x20000000 },
	{ 0x9054, 0x00000111 },
	{ 0x90f0, 0x000000e2 },
	{ 0xa010, 0x10000000 },
	{ 0xa014, 0x10000000 },
	{ 0xa020, 0x10000000 },
	{ 0xa024, 0x10000000 },
	{ 0xa028, 0x10000000 },
	{ 0xa02c, 0x10000000 },
	{ 0xa030, 0x00000101 },
	{ 0xa040, 0x0000001d },
	{ 0xa044, 0x0000ffff },
	{ 0xa048, 0x000000f0 },
	{ 0xa04c, 0x0000ff1d },
	{ 0xa050, 0x3f000404 },
	{ 0xa054, 0x00030000 },
	{ 0xa058, 0x020000e4 },
	{ 0xa060, 0x00700000 },
	{ 0xa064, 0x00700000 },
	{ 0xa068, 0x00700000 },
	{ 0xa06c, 0x00700000 },
	{ 0xa080, 0x80000000 },
	{ 0xa088, 0x80000000 },
	{ 0xa0f0, 0x000000f0 },
	{ 0xa0f4, 0xffffffff },
	{ 0xa110, 0x00005f00 },
	{ 0xa114, 0x00040387 },
	{ 0xa118, 0x00000704 },
	{ 0xa120, 0x00007d00 },
	{ 0xa124, 0x00040387 },
	{ 0xa128, 0x00000704 },
	{ 0xa130, 0x0000af00 },
	{ 0xa134, 0x00040387 },
	{ 0xa138, 0x00000704 },
	{ 0xa140, 0x0000be00 },
	{ 0xa144, 0x00040387 },
	{ 0xa148, 0x00000704 },
	{ 0xa184, 0x00000001 },
	{ 0xa188, 0x01050701 },
	{ 0xa190, 0x00000002 },
	{ 0xa198, 0x0000ffff },
	{ 0xa1a0, 0x00000002 },
	{ 0xa1a8, 0x0000ffff },
	{ 0xa1b0, 0x00000002 },
	{ 0xa1b8, 0x0000ffff },
	{ 0xa1c0, 0x00000002 },
	{ 0xa1c8, 0x0000ffff },
	{ 0xa1f0, 0x000000ff },
	{ 0xa21c, 0xffff0000 },
	{ 0xa264, 0xffffffff },
	{ 0xa284, 0x0000ffff },
	{ 0xa28c, 0x00005161 },
	{ 0xb008, 0x44440012 },
	{ 0xb010, 0x0e492300 },
	{ 0xb014, 0x0e492301 },
	{ 0xb018, 0x12492400 },
	{ 0xb024, 0x09000000 },
	{ 0xb034, 0x09000000 },
	{ 0xb044, 0x09000000 },
	{ 0xb054, 0x09000000 },
	{ 0xb080, 0x0003ffff },
	{ 0xb084, 0x0003ffff },
	{ 0xb088, 0x0003ffff },
	{ 0xb08c, 0x0003ffff },
	{ 0xb0f0, 0x80000000 },
	{ 0xb100, 0x10100000 },
	{ 0xb104, 0xffffffff },
	{ 0xb130, 0x0f0f0380 },
	{ 0xb134, 0x00005161 },
};


static int mtk_mipi_phy_power_on(struct phy *phy)
{
	struct mtk_mipi_cdphy_port *port = phy_get_drvdata(phy);
	void __iomem *base = port->base;

	/*
	 * The driver currently supports DPHY and CD-PHY phys,
	 * but the only mode supported is DPHY,
	 * so CD-PHY capable phys must be configured in DPHY mode
	 */
	if (port->type == CDPHY) {
		mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSI0A_CPHY_EN, 0);
		mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA,
				     RG_CSI0A_CPHY_EN, 0);
	}

	/*
	 * Lane configuration:
	 *
	 * Only 4 data + 1 clock is supported for now with the following mapping:
	 *
	 * CSIXA_LNR0 --> D2
	 * CSIXA_LNR1 --> D0
	 * CSIXA_LNR2 --> C
	 * CSIXB_LNR0 --> D1
	 * CSIXB_LNR1 --> D3
	 */
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L0_CKMODE_EN, 0);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L0_CKSEL, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L1_CKMODE_EN, 0);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L1_CKSEL, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L2_CKMODE_EN, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L2_CKSEL, 1);

	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA,
			     RG_CSIXA_DPHY_L0_CKMODE_EN, 0);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L0_CKSEL, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA,
			     RG_CSIXA_DPHY_L1_CKMODE_EN, 0);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L1_CKSEL, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA,
			     RG_CSIXA_DPHY_L2_CKMODE_EN, 0);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L2_CKSEL, 1);

	/* Byte clock invert */
	mtk_phy_update_field(base + MIPI_RX_ANAA8_CSIXA, RG_CSIXA_CDPHY_L0_T0_BYTECK_INVERT, 1);
	mtk_phy_update_field(base + MIPI_RX_ANAA8_CSIXA, RG_CSIXA_DPHY_L1_BYTECK_INVERT, 1);
	mtk_phy_update_field(base + MIPI_RX_ANAA8_CSIXA, RG_CSIXA_CDPHY_L2_T1_BYTECK_INVERT, 1);

	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANAA8_CSIXA,
			     RG_CSIXA_CDPHY_L0_T0_BYTECK_INVERT, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANAA8_CSIXA,
			     RG_CSIXA_DPHY_L1_BYTECK_INVERT, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANAA8_CSIXA,
			     RG_CSIXA_CDPHY_L2_T1_BYTECK_INVERT, 1);

	/* Start ANA EQ tuning */
	if (port->type == CDPHY)
		mtk_phy_csi_cdphy_ana_eq_tune(base);
	else
		mtk_phy_csi_dphy_ana_eq_tune(base);

	/* End ANA EQ tuning */
	mtk_phy_set_bits(base + MIPI_RX_ANA40_CSIXA, 0x90);

	mtk_phy_update_field(base + MIPI_RX_ANA24_CSIXA, RG_CSIXA_RESERVE, 0x40);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA24_CSIXA, RG_CSIXA_RESERVE, 0x40);
	mtk_phy_update_field(base + MIPI_RX_WRAPPER80_CSIXA, CSR_CSI_RST_MODE, 0);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_WRAPPER80_CSIXA, CSR_CSI_RST_MODE, 0);
	/* ANA power on */
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_CORE_EN, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_CORE_EN, 1);
	usleep_range(20, 40);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_LPF_EN, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_LPF_EN, 1);

	return 0;
	if (port->mt6833) {
		unsigned int i;

		for (i = 0; i < ARRAY_SIZE(mt6833_phy_golden); i++)
			writel(mt6833_phy_golden[i].val,
			       base + mt6833_phy_golden[i].off);
	}

}


void mtk_csi_phy_mt6833_apply_vendor(struct phy *phy)
{
	struct mtk_mipi_cdphy_port *port = phy_get_drvdata(phy);
	void __iomem *base = port->base;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mt6833_phy_golden); i++)
		writel(mt6833_phy_golden[i].val, base + mt6833_phy_golden[i].off);
}
EXPORT_SYMBOL_GPL(mtk_csi_phy_mt6833_apply_vendor);

static int mtk_mipi_phy_power_off(struct phy *phy)
{
	struct mtk_mipi_cdphy_port *port = phy_get_drvdata(phy);
	void __iomem *base = port->base;

	/* Disable MIPI BG. */
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_CORE_EN, 0);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_LPF_EN, 0);

	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_CORE_EN, 0);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_LPF_EN, 0);

	return 0;
}

static struct phy *mtk_mipi_cdphy_xlate(struct device *dev,
					const struct of_phandle_args *args)
{
	struct mtk_mipi_cdphy_port *priv = dev_get_drvdata(dev);

	/*
	 * If PHY is CD-PHY then we need to get the operating mode
	 * For now only D-PHY mode is supported
	 */
	if (priv->type == CDPHY) {
		if (args->args_count != 1) {
			dev_err(dev, "invalid number of arguments\n");
			return ERR_PTR(-EINVAL);
		}
		switch (args->args[0]) {
		case PHY_TYPE_DPHY:
			priv->mode = DPHY;
			if (priv->num_lanes != 4) {
				dev_err(dev, "Only 4D1C mode is supported for now!\n");
				return ERR_PTR(-EINVAL);
			}
			break;
		default:
			dev_err(dev, "Unsupported PHY type: %i\n", args->args[0]);
			return ERR_PTR(-EINVAL);
		}
	} else {
		if (args->args_count) {
			dev_err(dev, "invalid number of arguments\n");
			return ERR_PTR(-EINVAL);
		}
		priv->mode = DPHY;
	}

	return priv->phy;
}

static const struct phy_ops mtk_cdphy_ops = {
	.power_on	= mtk_mipi_phy_power_on,
	.power_off	= mtk_mipi_phy_power_off,
	.owner		= THIS_MODULE,
};

static int mtk_mipi_cdphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *phy_provider;
	struct mtk_mipi_cdphy_port *port;
	struct phy *phy;
	int ret;
	u32 phy_type;

	port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	dev_set_drvdata(dev, port);

	port->dev = dev;
	port->mt6833 = of_device_is_compatible(dev->of_node,
					       "mediatek,mt6833-csi-rx");

	port->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(port->base))
		return PTR_ERR(port->base);

	ret = of_property_read_u32(dev->of_node, "num-lanes", &port->num_lanes);
	if (ret) {
		dev_err(dev, "Failed to read num-lanes property: %i\n", ret);
		return ret;
	}

	/*
	 * phy-type is optional, if not present, PHY is considered to be CD-PHY
	 */
	if (device_property_present(dev, "phy-type")) {
		ret = of_property_read_u32(dev->of_node, "phy-type", &phy_type);
		if (ret) {
			dev_err(dev, "Failed to read phy-type property: %i\n", ret);
			return ret;
		}
		switch (phy_type) {
		case PHY_TYPE_DPHY:
			port->type = DPHY;
			break;
		default:
			dev_err(dev, "Unsupported PHY type: %i\n", phy_type);
			return -EINVAL;
		}
	} else {
		port->type = CDPHY;
	}

	phy = devm_phy_create(dev, NULL, &mtk_cdphy_ops);
	if (IS_ERR(phy)) {
		dev_err(dev, "Failed to create PHY: %ld\n", PTR_ERR(phy));
		return PTR_ERR(phy);
	}

	port->phy = phy;
	phy_set_drvdata(phy, port);

	phy_provider = devm_of_phy_provider_register(dev, mtk_mipi_cdphy_xlate);
	if (IS_ERR(phy_provider)) {
		dev_err(dev, "Failed to register PHY provider: %ld\n",
			PTR_ERR(phy_provider));
		return PTR_ERR(phy_provider);
	}

	return 0;
}

static const struct of_device_id mtk_mipi_cdphy_of_match[] = {
	{ .compatible = "mediatek,mt8365-csi-rx" },
	{ .compatible = "mediatek,mt6833-csi-rx" },
	{ /* sentinel */},
};
MODULE_DEVICE_TABLE(of, mtk_mipi_cdphy_of_match);

static struct platform_driver mipi_cdphy_pdrv = {
	.probe = mtk_mipi_cdphy_probe,
	.driver	= {
		.name	= "mtk-mipi-csi-0-5",
		.of_match_table = mtk_mipi_cdphy_of_match,
	},
};
module_platform_driver(mipi_cdphy_pdrv);

MODULE_DESCRIPTION("MediaTek MIPI CSI CD-PHY v0.5 Driver");
MODULE_AUTHOR("Louis Kuo <louis.kuo@mediatek.com>");
MODULE_LICENSE("GPL");
