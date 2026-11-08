// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2025 Linaro Ltd

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define S5KJN1_LINK_FREQ_700MHZ		(700ULL * HZ_PER_MHZ)
#define S5KJN1_MCLK_FREQ_24MHZ		(24 * HZ_PER_MHZ)
#define S5KJN1_DATA_LANES		4

/* Register map is similar to MIPI CCS compliant camera sensors */
#define S5KJN1_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5KJN1_CHIP_ID			0x38e1

#define S5KJN1_REG_CTRL_MODE		CCI_REG8(0x0100)
#define S5KJN1_MODE_STREAMING		BIT(0)

#define S5KJN1_REG_ORIENTATION		CCI_REG8(0x0101)
#define S5KJN1_VFLIP			BIT(1)
#define S5KJN1_HFLIP			BIT(0)

#define S5KJN1_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5KJN1_EXPOSURE_MIN		8
#define S5KJN1_EXPOSURE_STEP		1

#define S5KJN1_REG_AGAIN		CCI_REG16(0x0204)
#define S5KJN1_AGAIN_MIN		1
#define S5KJN1_AGAIN_MAX		64
#define S5KJN1_AGAIN_STEP		1
#define S5KJN1_AGAIN_DEFAULT		6
#define S5KJN1_AGAIN_SHIFT		5

#define S5KJN1_REG_VTS			CCI_REG16(0x0340)
#define S5KJN1_VTS_MAX			0xffff

#define S5KJN1_REG_HTS			CCI_REG16(0x0342)
#define S5KJN1_REG_X_ADDR_START		CCI_REG16(0x0344)
#define S5KJN1_REG_Y_ADDR_START		CCI_REG16(0x0346)
#define S5KJN1_REG_X_ADDR_END		CCI_REG16(0x0348)
#define S5KJN1_REG_Y_ADDR_END		CCI_REG16(0x034a)
#define S5KJN1_REG_X_OUTPUT_SIZE	CCI_REG16(0x034c)
#define S5KJN1_REG_Y_OUTPUT_SIZE	CCI_REG16(0x034e)

#define S5KJN1_REG_TEST_PATTERN		CCI_REG16(0x0600)

#define to_s5kjn1(_sd)			container_of(_sd, struct s5kjn1, sd)

static const s64 s5kjn1_link_freq_menu[] = {
	S5KJN1_LINK_FREQ_700MHZ,
};

/* List of supported formats to cover horizontal and vertical flip controls */
static const u32 s5kjn1_mbus_formats[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,	MEDIA_BUS_FMT_SGBRG10_1X10,
};

struct s5kjn1_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct s5kjn1_mode {
	u32 width;			/* Frame width in pixels */
	u32 height;			/* Frame height in pixels */
	u32 hts;			/* Horizontal timing size */
	u32 vts;			/* Default vertical timing size */
	u32 exposure;			/* Default exposure value */
	u32 exposure_margin;		/* Exposure margin */

	const struct s5kjn1_reg_list reg_list;	/* Sensor register setting */
};

static const char * const s5kjn1_test_pattern_menu[] = {
	"Disabled",
	"Solid color",
	"Color bars",
	"Fade to grey color bars",
	"PN9",
};

struct s5kjn1 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator *afvdd;	/* Autofocus actuator power */
	struct regulator *vdda;		/* Analog power */
	struct regulator *vddd;		/* Digital core power */
	struct regulator *vddio;	/* Digital I/O power */

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;

	const struct s5kjn1_mode *mode;
};

static const struct cci_reg_sequence init_array_setting[] = {
	{ CCI_REG16(0x6028), 0x2400 },
	{ CCI_REG16(0x602a), 0x7700 },
	{ CCI_REG16(0x6f12), 0x1753 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0xe702 },
	{ CCI_REG16(0x6f12), 0x6385 },
	{ CCI_REG16(0x6f12), 0xb787 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9386 },
	{ CCI_REG16(0x6f12), 0xc701 },
	{ CCI_REG16(0x6f12), 0xb7b7 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x3777 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9387 },
	{ CCI_REG16(0x6f12), 0xc77f },
	{ CCI_REG16(0x6f12), 0x1307 },
	{ CCI_REG16(0x6f12), 0x07c7 },
	{ CCI_REG16(0x6f12), 0x958f },
	{ CCI_REG16(0x6f12), 0x2328 },
	{ CCI_REG16(0x6f12), 0xd774 },
	{ CCI_REG16(0x6f12), 0x231a },
	{ CCI_REG16(0x6f12), 0xf774 },
	{ CCI_REG16(0x6f12), 0x012f },
	{ CCI_REG16(0x6f12), 0xb777 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x1307 },
	{ CCI_REG16(0x6f12), 0xd003 },
	{ CCI_REG16(0x6f12), 0x23a0 },
	{ CCI_REG16(0x6f12), 0xe774 },
	{ CCI_REG16(0x6f12), 0x1753 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0x6700 },
	{ CCI_REG16(0x6f12), 0x2384 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x8547 },
	{ CCI_REG16(0x6f12), 0x6310 },
	{ CCI_REG16(0x6f12), 0xf506 },
	{ CCI_REG16(0x6f12), 0x3786 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9306 },
	{ CCI_REG16(0x6f12), 0x4601 },
	{ CCI_REG16(0x6f12), 0x83c7 },
	{ CCI_REG16(0x6f12), 0x4600 },
	{ CCI_REG16(0x6f12), 0xa1cb },
	{ CCI_REG16(0x6f12), 0xb737 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x83a7 },
	{ CCI_REG16(0x6f12), 0x875c },
	{ CCI_REG16(0x6f12), 0x83d6 },
	{ CCI_REG16(0x6f12), 0x2600 },
	{ CCI_REG16(0x6f12), 0x83d7 },
	{ CCI_REG16(0x6f12), 0x271e },
	{ CCI_REG16(0x6f12), 0x13d7 },
	{ CCI_REG16(0x6f12), 0x2700 },
	{ CCI_REG16(0x6f12), 0xb707 },
	{ CCI_REG16(0x6f12), 0x0140 },
	{ CCI_REG16(0x6f12), 0x83d5 },
	{ CCI_REG16(0x6f12), 0x27f0 },
	{ CCI_REG16(0x6f12), 0x8357 },
	{ CCI_REG16(0x6f12), 0x4601 },
	{ CCI_REG16(0x6f12), 0x1306 },
	{ CCI_REG16(0x6f12), 0xe7ff },
	{ CCI_REG16(0x6f12), 0xb697 },
	{ CCI_REG16(0x6f12), 0x8d8f },
	{ CCI_REG16(0x6f12), 0xc207 },
	{ CCI_REG16(0x6f12), 0xc183 },
	{ CCI_REG16(0x6f12), 0x9396 },
	{ CCI_REG16(0x6f12), 0x0701 },
	{ CCI_REG16(0x6f12), 0xc186 },
	{ CCI_REG16(0x6f12), 0x635f },
	{ CCI_REG16(0x6f12), 0xd600 },
	{ CCI_REG16(0x6f12), 0x8907 },
	{ CCI_REG16(0x6f12), 0x998f },
	{ CCI_REG16(0x6f12), 0x9396 },
	{ CCI_REG16(0x6f12), 0x0701 },
	{ CCI_REG16(0x6f12), 0xc186 },
	{ CCI_REG16(0x6f12), 0x9397 },
	{ CCI_REG16(0x6f12), 0x0601 },
	{ CCI_REG16(0x6f12), 0xc183 },
	{ CCI_REG16(0x6f12), 0x37b7 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x2311 },
	{ CCI_REG16(0x6f12), 0xf7a0 },
	{ CCI_REG16(0x6f12), 0x8280 },
	{ CCI_REG16(0x6f12), 0xe3d8 },
	{ CCI_REG16(0x6f12), 0x06fe },
	{ CCI_REG16(0x6f12), 0xba97 },
	{ CCI_REG16(0x6f12), 0xf917 },
	{ CCI_REG16(0x6f12), 0xcdb7 },
	{ CCI_REG16(0x6f12), 0xb717 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9387 },
	{ CCI_REG16(0x6f12), 0x07ca },
	{ CCI_REG16(0x6f12), 0xaa97 },
	{ CCI_REG16(0x6f12), 0x3387 },
	{ CCI_REG16(0x6f12), 0xb700 },
	{ CCI_REG16(0x6f12), 0x8d8f },
	{ CCI_REG16(0x6f12), 0x83c5 },
	{ CCI_REG16(0x6f12), 0xd705 },
	{ CCI_REG16(0x6f12), 0xb747 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9386 },
	{ CCI_REG16(0x6f12), 0x078f },
	{ CCI_REG16(0x6f12), 0x83d7 },
	{ CCI_REG16(0x6f12), 0xe670 },
	{ CCI_REG16(0x6f12), 0x0905 },
	{ CCI_REG16(0x6f12), 0x0347 },
	{ CCI_REG16(0x6f12), 0xc705 },
	{ CCI_REG16(0x6f12), 0x630b },
	{ CCI_REG16(0x6f12), 0xf500 },
	{ CCI_REG16(0x6f12), 0x8567 },
	{ CCI_REG16(0x6f12), 0xb697 },
	{ CCI_REG16(0x6f12), 0x03a6 },
	{ CCI_REG16(0x6f12), 0x8794 },
	{ CCI_REG16(0x6f12), 0xb306 },
	{ CCI_REG16(0x6f12), 0xb700 },
	{ CCI_REG16(0x6f12), 0xb296 },
	{ CCI_REG16(0x6f12), 0x23a4 },
	{ CCI_REG16(0x6f12), 0xd794 },
	{ CCI_REG16(0x6f12), 0x2207 },
	{ CCI_REG16(0x6f12), 0x3305 },
	{ CCI_REG16(0x6f12), 0xb700 },
	{ CCI_REG16(0x6f12), 0x4205 },
	{ CCI_REG16(0x6f12), 0x4181 },
	{ CCI_REG16(0x6f12), 0x8280 },
	{ CCI_REG16(0x6f12), 0x5d71 },
	{ CCI_REG16(0x6f12), 0xa2c6 },
	{ CCI_REG16(0x6f12), 0xa6c4 },
	{ CCI_REG16(0x6f12), 0x7324 },
	{ CCI_REG16(0x6f12), 0x2034 },
	{ CCI_REG16(0x6f12), 0xf324 },
	{ CCI_REG16(0x6f12), 0x1034 },
	{ CCI_REG16(0x6f12), 0x7360 },
	{ CCI_REG16(0x6f12), 0x0430 },
	{ CCI_REG16(0x6f12), 0x2ad8 },
	{ CCI_REG16(0x6f12), 0x2ed6 },
	{ CCI_REG16(0x6f12), 0x3545 },
	{ CCI_REG16(0x6f12), 0x9305 },
	{ CCI_REG16(0x6f12), 0x8008 },
	{ CCI_REG16(0x6f12), 0x22da },
	{ CCI_REG16(0x6f12), 0x3ece },
	{ CCI_REG16(0x6f12), 0x86c2 },
	{ CCI_REG16(0x6f12), 0x96c0 },
	{ CCI_REG16(0x6f12), 0x1ade },
	{ CCI_REG16(0x6f12), 0x1edc },
	{ CCI_REG16(0x6f12), 0x32d4 },
	{ CCI_REG16(0x6f12), 0x36d2 },
	{ CCI_REG16(0x6f12), 0x3ad0 },
	{ CCI_REG16(0x6f12), 0x42cc },
	{ CCI_REG16(0x6f12), 0x46ca },
	{ CCI_REG16(0x6f12), 0x72c8 },
	{ CCI_REG16(0x6f12), 0x76c6 },
	{ CCI_REG16(0x6f12), 0x7ac4 },
	{ CCI_REG16(0x6f12), 0x7ec2 },
	{ CCI_REG16(0x6f12), 0x9710 },
	{ CCI_REG16(0x6f12), 0x00fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x40f5 },
	{ CCI_REG16(0x6f12), 0x9377 },
	{ CCI_REG16(0x6f12), 0x8500 },
	{ CCI_REG16(0x6f12), 0x2a84 },
	{ CCI_REG16(0x6f12), 0x85c3 },
	{ CCI_REG16(0x6f12), 0xb737 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9387 },
	{ CCI_REG16(0x6f12), 0x077c },
	{ CCI_REG16(0x6f12), 0x03d7 },
	{ CCI_REG16(0x6f12), 0x6702 },
	{ CCI_REG16(0x6f12), 0x0507 },
	{ CCI_REG16(0x6f12), 0x2393 },
	{ CCI_REG16(0x6f12), 0xe702 },
	{ CCI_REG16(0x6f12), 0x9710 },
	{ CCI_REG16(0x6f12), 0x00fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x00ed },
	{ CCI_REG16(0x6f12), 0x0545 },
	{ CCI_REG16(0x6f12), 0xd535 },
	{ CCI_REG16(0x6f12), 0x1374 },
	{ CCI_REG16(0x6f12), 0x0408 },
	{ CCI_REG16(0x6f12), 0x11cc },
	{ CCI_REG16(0x6f12), 0xb737 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9387 },
	{ CCI_REG16(0x6f12), 0x077c },
	{ CCI_REG16(0x6f12), 0x03d7 },
	{ CCI_REG16(0x6f12), 0x8705 },
	{ CCI_REG16(0x6f12), 0x0507 },
	{ CCI_REG16(0x6f12), 0x239c },
	{ CCI_REG16(0x6f12), 0xe704 },
	{ CCI_REG16(0x6f12), 0x9710 },
	{ CCI_REG16(0x6f12), 0x00fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x6065 },
	{ CCI_REG16(0x6f12), 0x9640 },
	{ CCI_REG16(0x6f12), 0x8642 },
	{ CCI_REG16(0x6f12), 0x7253 },
	{ CCI_REG16(0x6f12), 0xe253 },
	{ CCI_REG16(0x6f12), 0x5254 },
	{ CCI_REG16(0x6f12), 0x4255 },
	{ CCI_REG16(0x6f12), 0xb255 },
	{ CCI_REG16(0x6f12), 0x2256 },
	{ CCI_REG16(0x6f12), 0x9256 },
	{ CCI_REG16(0x6f12), 0x0257 },
	{ CCI_REG16(0x6f12), 0xf247 },
	{ CCI_REG16(0x6f12), 0x6248 },
	{ CCI_REG16(0x6f12), 0xd248 },
	{ CCI_REG16(0x6f12), 0x424e },
	{ CCI_REG16(0x6f12), 0xb24e },
	{ CCI_REG16(0x6f12), 0x224f },
	{ CCI_REG16(0x6f12), 0x924f },
	{ CCI_REG16(0x6f12), 0x7370 },
	{ CCI_REG16(0x6f12), 0x0430 },
	{ CCI_REG16(0x6f12), 0x7390 },
	{ CCI_REG16(0x6f12), 0x1434 },
	{ CCI_REG16(0x6f12), 0x7310 },
	{ CCI_REG16(0x6f12), 0x2434 },
	{ CCI_REG16(0x6f12), 0x3644 },
	{ CCI_REG16(0x6f12), 0xa644 },
	{ CCI_REG16(0x6f12), 0x6161 },
	{ CCI_REG16(0x6f12), 0x7300 },
	{ CCI_REG16(0x6f12), 0x2030 },
	{ CCI_REG16(0x6f12), 0x1743 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0xe702 },
	{ CCI_REG16(0x6f12), 0xc369 },
	{ CCI_REG16(0x6f12), 0xb787 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x83a4 },
	{ CCI_REG16(0x6f12), 0xc700 },
	{ CCI_REG16(0x6f12), 0x2a84 },
	{ CCI_REG16(0x6f12), 0x0146 },
	{ CCI_REG16(0x6f12), 0xa685 },
	{ CCI_REG16(0x6f12), 0x1145 },
	{ CCI_REG16(0x6f12), 0x9790 },
	{ CCI_REG16(0x6f12), 0xfffb },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x60ad },
	{ CCI_REG16(0x6f12), 0x2285 },
	{ CCI_REG16(0x6f12), 0x97e0 },
	{ CCI_REG16(0x6f12), 0xfffb },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x40a9 },
	{ CCI_REG16(0x6f12), 0xb787 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9387 },
	{ CCI_REG16(0x6f12), 0x87f5 },
	{ CCI_REG16(0x6f12), 0x03c7 },
	{ CCI_REG16(0x6f12), 0x0700 },
	{ CCI_REG16(0x6f12), 0x8546 },
	{ CCI_REG16(0x6f12), 0x6315 },
	{ CCI_REG16(0x6f12), 0xd706 },
	{ CCI_REG16(0x6f12), 0xb776 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x03a6 },
	{ CCI_REG16(0x6f12), 0x468d },
	{ CCI_REG16(0x6f12), 0x8945 },
	{ CCI_REG16(0x6f12), 0x9386 },
	{ CCI_REG16(0x6f12), 0xa700 },
	{ CCI_REG16(0x6f12), 0x630f },
	{ CCI_REG16(0x6f12), 0xb600 },
	{ CCI_REG16(0x6f12), 0x9386 },
	{ CCI_REG16(0x6f12), 0x2700 },
	{ CCI_REG16(0x6f12), 0x630b },
	{ CCI_REG16(0x6f12), 0xe600 },
	{ CCI_REG16(0x6f12), 0x3717 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x0356 },
	{ CCI_REG16(0x6f12), 0x6738 },
	{ CCI_REG16(0x6f12), 0x2d47 },
	{ CCI_REG16(0x6f12), 0x6314 },
	{ CCI_REG16(0x6f12), 0xe600 },
	{ CCI_REG16(0x6f12), 0x9386 },
	{ CCI_REG16(0x6f12), 0xa700 },
	{ CCI_REG16(0x6f12), 0xb747 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x83a5 },
	{ CCI_REG16(0x6f12), 0xc786 },
	{ CCI_REG16(0x6f12), 0x2946 },
	{ CCI_REG16(0x6f12), 0x1305 },
	{ CCI_REG16(0x6f12), 0x0404 },
	{ CCI_REG16(0x6f12), 0x83c7 },
	{ CCI_REG16(0x6f12), 0xc52f },
	{ CCI_REG16(0x6f12), 0x2148 },
	{ CCI_REG16(0x6f12), 0x1d8e },
	{ CCI_REG16(0x6f12), 0x8147 },
	{ CCI_REG16(0x6f12), 0x3387 },
	{ CCI_REG16(0x6f12), 0xf500 },
	{ CCI_REG16(0x6f12), 0xb388 },
	{ CCI_REG16(0x6f12), 0xf600 },
	{ CCI_REG16(0x6f12), 0x0317 },
	{ CCI_REG16(0x6f12), 0xe73c },
	{ CCI_REG16(0x6f12), 0x8398 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x8907 },
	{ CCI_REG16(0x6f12), 0x1105 },
	{ CCI_REG16(0x6f12), 0x4697 },
	{ CCI_REG16(0x6f12), 0x3317 },
	{ CCI_REG16(0x6f12), 0xc700 },
	{ CCI_REG16(0x6f12), 0x232e },
	{ CCI_REG16(0x6f12), 0xe5fe },
	{ CCI_REG16(0x6f12), 0xe391 },
	{ CCI_REG16(0x6f12), 0x07ff },
	{ CCI_REG16(0x6f12), 0x0546 },
	{ CCI_REG16(0x6f12), 0xa685 },
	{ CCI_REG16(0x6f12), 0x1145 },
	{ CCI_REG16(0x6f12), 0x9790 },
	{ CCI_REG16(0x6f12), 0xfffb },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x60a4 },
	{ CCI_REG16(0x6f12), 0x1743 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0x6700 },
	{ CCI_REG16(0x6f12), 0x0361 },
	{ CCI_REG16(0x6f12), 0x1743 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0xe702 },
	{ CCI_REG16(0x6f12), 0xa35c },
	{ CCI_REG16(0x6f12), 0xb787 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x83a4 },
	{ CCI_REG16(0x6f12), 0x8700 },
	{ CCI_REG16(0x6f12), 0x4111 },
	{ CCI_REG16(0x6f12), 0xaa89 },
	{ CCI_REG16(0x6f12), 0x2e8a },
	{ CCI_REG16(0x6f12), 0xb28a },
	{ CCI_REG16(0x6f12), 0xa685 },
	{ CCI_REG16(0x6f12), 0x0146 },
	{ CCI_REG16(0x6f12), 0x1145 },
	{ CCI_REG16(0x6f12), 0x36c6 },
	{ CCI_REG16(0x6f12), 0x9790 },
	{ CCI_REG16(0x6f12), 0xfffb },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x60a1 },
	{ CCI_REG16(0x6f12), 0xb787 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x1387 },
	{ CCI_REG16(0x6f12), 0x87f5 },
	{ CCI_REG16(0x6f12), 0x0347 },
	{ CCI_REG16(0x6f12), 0x2701 },
	{ CCI_REG16(0x6f12), 0x1384 },
	{ CCI_REG16(0x6f12), 0x87f5 },
	{ CCI_REG16(0x6f12), 0xb246 },
	{ CCI_REG16(0x6f12), 0x0149 },
	{ CCI_REG16(0x6f12), 0x11cf },
	{ CCI_REG16(0x6f12), 0x3767 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x0357 },
	{ CCI_REG16(0x6f12), 0x2777 },
	{ CCI_REG16(0x6f12), 0xb777 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9387 },
	{ CCI_REG16(0x6f12), 0x07c7 },
	{ CCI_REG16(0x6f12), 0x0e07 },
	{ CCI_REG16(0x6f12), 0x03d9 },
	{ CCI_REG16(0x6f12), 0x871c },
	{ CCI_REG16(0x6f12), 0x2394 },
	{ CCI_REG16(0x6f12), 0xe71c },
	{ CCI_REG16(0x6f12), 0x5686 },
	{ CCI_REG16(0x6f12), 0xd285 },
	{ CCI_REG16(0x6f12), 0x4e85 },
	{ CCI_REG16(0x6f12), 0x97e0 },
	{ CCI_REG16(0x6f12), 0xfffb },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x0088 },
	{ CCI_REG16(0x6f12), 0x8347 },
	{ CCI_REG16(0x6f12), 0x2401 },
	{ CCI_REG16(0x6f12), 0x89c7 },
	{ CCI_REG16(0x6f12), 0xb777 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x239c },
	{ CCI_REG16(0x6f12), 0x27e3 },
	{ CCI_REG16(0x6f12), 0x0546 },
	{ CCI_REG16(0x6f12), 0xa685 },
	{ CCI_REG16(0x6f12), 0x1145 },
	{ CCI_REG16(0x6f12), 0x9790 },
	{ CCI_REG16(0x6f12), 0xfffb },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0xc09b },
	{ CCI_REG16(0x6f12), 0x4101 },
	{ CCI_REG16(0x6f12), 0x1743 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0x6700 },
	{ CCI_REG16(0x6f12), 0xa357 },
	{ CCI_REG16(0x6f12), 0x1743 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0xe702 },
	{ CCI_REG16(0x6f12), 0x0353 },
	{ CCI_REG16(0x6f12), 0x3784 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9307 },
	{ CCI_REG16(0x6f12), 0x04f4 },
	{ CCI_REG16(0x6f12), 0x83c7 },
	{ CCI_REG16(0x6f12), 0x4701 },
	{ CCI_REG16(0x6f12), 0x5d71 },
	{ CCI_REG16(0x6f12), 0x2a89 },
	{ CCI_REG16(0x6f12), 0x8def },
	{ CCI_REG16(0x6f12), 0xb787 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x03a4 },
	{ CCI_REG16(0x6f12), 0x4700 },
	{ CCI_REG16(0x6f12), 0x0146 },
	{ CCI_REG16(0x6f12), 0x1145 },
	{ CCI_REG16(0x6f12), 0xa285 },
	{ CCI_REG16(0x6f12), 0x9790 },
	{ CCI_REG16(0x6f12), 0xfffb },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x2098 },
	{ CCI_REG16(0x6f12), 0x4a85 },
	{ CCI_REG16(0x6f12), 0x97e0 },
	{ CCI_REG16(0x6f12), 0xfffb },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x400f },
	{ CCI_REG16(0x6f12), 0x0546 },
	{ CCI_REG16(0x6f12), 0xa285 },
	{ CCI_REG16(0x6f12), 0x1145 },
	{ CCI_REG16(0x6f12), 0x9790 },
	{ CCI_REG16(0x6f12), 0xfffb },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0xa096 },
	{ CCI_REG16(0x6f12), 0x6161 },
	{ CCI_REG16(0x6f12), 0x1743 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0x6700 },
	{ CCI_REG16(0x6f12), 0xe351 },
	{ CCI_REG16(0x6f12), 0x1304 },
	{ CCI_REG16(0x6f12), 0x04f4 },
	{ CCI_REG16(0x6f12), 0x0347 },
	{ CCI_REG16(0x6f12), 0x5401 },
	{ CCI_REG16(0x6f12), 0x8547 },
	{ CCI_REG16(0x6f12), 0x6310 },
	{ CCI_REG16(0x6f12), 0xf716 },
	{ CCI_REG16(0x6f12), 0xb785 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9384 },
	{ CCI_REG16(0x6f12), 0x05f8 },
	{ CCI_REG16(0x6f12), 0x4146 },
	{ CCI_REG16(0x6f12), 0x9385 },
	{ CCI_REG16(0x6f12), 0x05f8 },
	{ CCI_REG16(0x6f12), 0x0a85 },
	{ CCI_REG16(0x6f12), 0x9740 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x6058 },
	{ CCI_REG16(0x6f12), 0x4146 },
	{ CCI_REG16(0x6f12), 0x9385 },
	{ CCI_REG16(0x6f12), 0x0401 },
	{ CCI_REG16(0x6f12), 0x0808 },
	{ CCI_REG16(0x6f12), 0x9740 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x6057 },
	{ CCI_REG16(0x6f12), 0x4146 },
	{ CCI_REG16(0x6f12), 0x9385 },
	{ CCI_REG16(0x6f12), 0x0402 },
	{ CCI_REG16(0x6f12), 0x0810 },
	{ CCI_REG16(0x6f12), 0x9740 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x6056 },
	{ CCI_REG16(0x6f12), 0x4146 },
	{ CCI_REG16(0x6f12), 0x9385 },
	{ CCI_REG16(0x6f12), 0x0403 },
	{ CCI_REG16(0x6f12), 0x0818 },
	{ CCI_REG16(0x6f12), 0x9740 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x6055 },
	{ CCI_REG16(0x6f12), 0x1c40 },
	{ CCI_REG16(0x6f12), 0xb7db },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x014b },
	{ CCI_REG16(0x6f12), 0xbec0 },
	{ CCI_REG16(0x6f12), 0x5c40 },
	{ CCI_REG16(0x6f12), 0x8149 },
	{ CCI_REG16(0x6f12), 0x854a },
	{ CCI_REG16(0x6f12), 0xbec2 },
	{ CCI_REG16(0x6f12), 0x5c44 },
	{ CCI_REG16(0x6f12), 0x096d },
	{ CCI_REG16(0x6f12), 0x370c },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0xbec4 },
	{ CCI_REG16(0x6f12), 0x1c48 },
	{ CCI_REG16(0x6f12), 0x938b },
	{ CCI_REG16(0x6f12), 0x0b03 },
	{ CCI_REG16(0x6f12), 0x930c },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x6f12), 0xbec6 },
	{ CCI_REG16(0x6f12), 0x0347 },
	{ CCI_REG16(0x6f12), 0x7401 },
	{ CCI_REG16(0x6f12), 0x8967 },
	{ CCI_REG16(0x6f12), 0x631b },
	{ CCI_REG16(0x6f12), 0x5701 },
	{ CCI_REG16(0x6f12), 0x93f7 },
	{ CCI_REG16(0x6f12), 0xf900 },
	{ CCI_REG16(0x6f12), 0x9808 },
	{ CCI_REG16(0x6f12), 0xba97 },
	{ CCI_REG16(0x6f12), 0x83c7 },
	{ CCI_REG16(0x6f12), 0x07fb },
	{ CCI_REG16(0x6f12), 0x8a07 },
	{ CCI_REG16(0x6f12), 0xca97 },
	{ CCI_REG16(0x6f12), 0x9c43 },
	{ CCI_REG16(0x6f12), 0x63de },
	{ CCI_REG16(0x6f12), 0x3a01 },
	{ CCI_REG16(0x6f12), 0x1387 },
	{ CCI_REG16(0x6f12), 0x69fe },
	{ CCI_REG16(0x6f12), 0x63fa },
	{ CCI_REG16(0x6f12), 0xea00 },
	{ CCI_REG16(0x6f12), 0x1387 },
	{ CCI_REG16(0x6f12), 0xa9fd },
	{ CCI_REG16(0x6f12), 0x63f6 },
	{ CCI_REG16(0x6f12), 0xea00 },
	{ CCI_REG16(0x6f12), 0x1387 },
	{ CCI_REG16(0x6f12), 0x49fc },
	{ CCI_REG16(0x6f12), 0x63e3 },
	{ CCI_REG16(0x6f12), 0xea0a },
	{ CCI_REG16(0x6f12), 0x131a },
	{ CCI_REG16(0x6f12), 0x1b00 },
	{ CCI_REG16(0x6f12), 0x9808 },
	{ CCI_REG16(0x6f12), 0x5297 },
	{ CCI_REG16(0x6f12), 0x8354 },
	{ CCI_REG16(0x6f12), 0x07ff },
	{ CCI_REG16(0x6f12), 0x0145 },
	{ CCI_REG16(0x6f12), 0xb384 },
	{ CCI_REG16(0x6f12), 0xf402 },
	{ CCI_REG16(0x6f12), 0xb580 },
	{ CCI_REG16(0x6f12), 0x637a },
	{ CCI_REG16(0x6f12), 0x9d00 },
	{ CCI_REG16(0x6f12), 0x2685 },
	{ CCI_REG16(0x6f12), 0x9780 },
	{ CCI_REG16(0x6f12), 0xfffb },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x6052 },
	{ CCI_REG16(0x6f12), 0x4915 },
	{ CCI_REG16(0x6f12), 0x1375 },
	{ CCI_REG16(0x6f12), 0xf50f },
	{ CCI_REG16(0x6f12), 0x9c08 },
	{ CCI_REG16(0x6f12), 0xd297 },
	{ CCI_REG16(0x6f12), 0x03d7 },
	{ CCI_REG16(0x6f12), 0x07fe },
	{ CCI_REG16(0x6f12), 0xb3d4 },
	{ CCI_REG16(0x6f12), 0xa400 },
	{ CCI_REG16(0x6f12), 0xc204 },
	{ CCI_REG16(0x6f12), 0x6297 },
	{ CCI_REG16(0x6f12), 0xc180 },
	{ CCI_REG16(0x6f12), 0x2310 },
	{ CCI_REG16(0x6f12), 0x9700 },
	{ CCI_REG16(0x6f12), 0x03d7 },
	{ CCI_REG16(0x6f12), 0x07fc },
	{ CCI_REG16(0x6f12), 0x83d7 },
	{ CCI_REG16(0x6f12), 0x07fd },
	{ CCI_REG16(0x6f12), 0x050b },
	{ CCI_REG16(0x6f12), 0x6297 },
	{ CCI_REG16(0x6f12), 0x8356 },
	{ CCI_REG16(0x6f12), 0x0700 },
	{ CCI_REG16(0x6f12), 0x3315 },
	{ CCI_REG16(0x6f12), 0xf500 },
	{ CCI_REG16(0x6f12), 0x558d },
	{ CCI_REG16(0x6f12), 0x4205 },
	{ CCI_REG16(0x6f12), 0x4181 },
	{ CCI_REG16(0x6f12), 0x2310 },
	{ CCI_REG16(0x6f12), 0xa700 },
	{ CCI_REG16(0x6f12), 0x8509 },
	{ CCI_REG16(0x6f12), 0xe395 },
	{ CCI_REG16(0x6f12), 0x99f7 },
	{ CCI_REG16(0x6f12), 0x8d47 },
	{ CCI_REG16(0x6f12), 0x37d4 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x2319 },
	{ CCI_REG16(0x6f12), 0xf40a },
	{ CCI_REG16(0x6f12), 0x2316 },
	{ CCI_REG16(0x6f12), 0x040c },
	{ CCI_REG16(0x6f12), 0x9790 },
	{ CCI_REG16(0x6f12), 0x00fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x6093 },
	{ CCI_REG16(0x6f12), 0xaa84 },
	{ CCI_REG16(0x6f12), 0x9790 },
	{ CCI_REG16(0x6f12), 0x00fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x2092 },
	{ CCI_REG16(0x6f12), 0x9307 },
	{ CCI_REG16(0x6f12), 0x0008 },
	{ CCI_REG16(0x6f12), 0x33d5 },
	{ CCI_REG16(0x6f12), 0xa700 },
	{ CCI_REG16(0x6f12), 0x9307 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x6f12), 0x1205 },
	{ CCI_REG16(0x6f12), 0xb3d7 },
	{ CCI_REG16(0x6f12), 0x9700 },
	{ CCI_REG16(0x6f12), 0x1375 },
	{ CCI_REG16(0x6f12), 0x0503 },
	{ CCI_REG16(0x6f12), 0x8d8b },
	{ CCI_REG16(0x6f12), 0x5d8d },
	{ CCI_REG16(0x6f12), 0x2319 },
	{ CCI_REG16(0x6f12), 0xa40c },
	{ CCI_REG16(0x6f12), 0x45b5 },
	{ CCI_REG16(0x6f12), 0x1397 },
	{ CCI_REG16(0x6f12), 0x1900 },
	{ CCI_REG16(0x6f12), 0x9394 },
	{ CCI_REG16(0x6f12), 0x0701 },
	{ CCI_REG16(0x6f12), 0x5e97 },
	{ CCI_REG16(0x6f12), 0xc180 },
	{ CCI_REG16(0x6f12), 0x2310 },
	{ CCI_REG16(0x6f12), 0x9700 },
	{ CCI_REG16(0x6f12), 0x6db7 },
	{ CCI_REG16(0x6f12), 0x0347 },
	{ CCI_REG16(0x6f12), 0x6401 },
	{ CCI_REG16(0x6f12), 0xe315 },
	{ CCI_REG16(0x6f12), 0xf7fa },
	{ CCI_REG16(0x6f12), 0xb784 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9384 },
	{ CCI_REG16(0x6f12), 0x04f8 },
	{ CCI_REG16(0x6f12), 0x4146 },
	{ CCI_REG16(0x6f12), 0x9385 },
	{ CCI_REG16(0x6f12), 0x0404 },
	{ CCI_REG16(0x6f12), 0x0a85 },
	{ CCI_REG16(0x6f12), 0x9740 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x2042 },
	{ CCI_REG16(0x6f12), 0x4146 },
	{ CCI_REG16(0x6f12), 0x9385 },
	{ CCI_REG16(0x6f12), 0x0405 },
	{ CCI_REG16(0x6f12), 0x0808 },
	{ CCI_REG16(0x6f12), 0x9740 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x2041 },
	{ CCI_REG16(0x6f12), 0x4146 },
	{ CCI_REG16(0x6f12), 0x9385 },
	{ CCI_REG16(0x6f12), 0x0406 },
	{ CCI_REG16(0x6f12), 0x0810 },
	{ CCI_REG16(0x6f12), 0x9740 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x2040 },
	{ CCI_REG16(0x6f12), 0x4146 },
	{ CCI_REG16(0x6f12), 0x9385 },
	{ CCI_REG16(0x6f12), 0x0407 },
	{ CCI_REG16(0x6f12), 0x0818 },
	{ CCI_REG16(0x6f12), 0x9740 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x203f },
	{ CCI_REG16(0x6f12), 0x0357 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x8357 },
	{ CCI_REG16(0x6f12), 0x2400 },
	{ CCI_REG16(0x6f12), 0x37db },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x2310 },
	{ CCI_REG16(0x6f12), 0xe104 },
	{ CCI_REG16(0x6f12), 0x2311 },
	{ CCI_REG16(0x6f12), 0xf104 },
	{ CCI_REG16(0x6f12), 0x2312 },
	{ CCI_REG16(0x6f12), 0xe104 },
	{ CCI_REG16(0x6f12), 0x2313 },
	{ CCI_REG16(0x6f12), 0xf104 },
	{ CCI_REG16(0x6f12), 0x0357 },
	{ CCI_REG16(0x6f12), 0x8400 },
	{ CCI_REG16(0x6f12), 0x8357 },
	{ CCI_REG16(0x6f12), 0xa400 },
	{ CCI_REG16(0x6f12), 0x814a },
	{ CCI_REG16(0x6f12), 0x2314 },
	{ CCI_REG16(0x6f12), 0xe104 },
	{ CCI_REG16(0x6f12), 0x2315 },
	{ CCI_REG16(0x6f12), 0xf104 },
	{ CCI_REG16(0x6f12), 0x2316 },
	{ CCI_REG16(0x6f12), 0xe104 },
	{ CCI_REG16(0x6f12), 0x2317 },
	{ CCI_REG16(0x6f12), 0xf104 },
	{ CCI_REG16(0x6f12), 0x8149 },
	{ CCI_REG16(0x6f12), 0x854b },
	{ CCI_REG16(0x6f12), 0x096d },
	{ CCI_REG16(0x6f12), 0x130b },
	{ CCI_REG16(0x6f12), 0x0b03 },
	{ CCI_REG16(0x6f12), 0x370c },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x930c },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x6f12), 0x0347 },
	{ CCI_REG16(0x6f12), 0x7401 },
	{ CCI_REG16(0x6f12), 0x8967 },
	{ CCI_REG16(0x6f12), 0x631b },
	{ CCI_REG16(0x6f12), 0x7701 },
	{ CCI_REG16(0x6f12), 0x93f7 },
	{ CCI_REG16(0x6f12), 0xf900 },
	{ CCI_REG16(0x6f12), 0x9808 },
	{ CCI_REG16(0x6f12), 0xba97 },
	{ CCI_REG16(0x6f12), 0x83c7 },
	{ CCI_REG16(0x6f12), 0x07fb },
	{ CCI_REG16(0x6f12), 0x8a07 },
	{ CCI_REG16(0x6f12), 0xca97 },
	{ CCI_REG16(0x6f12), 0x9c43 },
	{ CCI_REG16(0x6f12), 0x63c4 },
	{ CCI_REG16(0x6f12), 0x3b07 },
	{ CCI_REG16(0x6f12), 0x139a },
	{ CCI_REG16(0x6f12), 0x1a00 },
	{ CCI_REG16(0x6f12), 0x9808 },
	{ CCI_REG16(0x6f12), 0x5297 },
	{ CCI_REG16(0x6f12), 0x8354 },
	{ CCI_REG16(0x6f12), 0x07ff },
	{ CCI_REG16(0x6f12), 0x0145 },
	{ CCI_REG16(0x6f12), 0xb384 },
	{ CCI_REG16(0x6f12), 0xf402 },
	{ CCI_REG16(0x6f12), 0xb580 },
	{ CCI_REG16(0x6f12), 0x637a },
	{ CCI_REG16(0x6f12), 0x9d00 },
	{ CCI_REG16(0x6f12), 0x2685 },
	{ CCI_REG16(0x6f12), 0x9780 },
	{ CCI_REG16(0x6f12), 0xfffb },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0xa03b },
	{ CCI_REG16(0x6f12), 0x4915 },
	{ CCI_REG16(0x6f12), 0x1375 },
	{ CCI_REG16(0x6f12), 0xf50f },
	{ CCI_REG16(0x6f12), 0x9c08 },
	{ CCI_REG16(0x6f12), 0xd297 },
	{ CCI_REG16(0x6f12), 0x03d7 },
	{ CCI_REG16(0x6f12), 0x07fe },
	{ CCI_REG16(0x6f12), 0xb3d4 },
	{ CCI_REG16(0x6f12), 0xa400 },
	{ CCI_REG16(0x6f12), 0xc204 },
	{ CCI_REG16(0x6f12), 0x6297 },
	{ CCI_REG16(0x6f12), 0xc180 },
	{ CCI_REG16(0x6f12), 0x2310 },
	{ CCI_REG16(0x6f12), 0x9700 },
	{ CCI_REG16(0x6f12), 0x03d7 },
	{ CCI_REG16(0x6f12), 0x07fc },
	{ CCI_REG16(0x6f12), 0x83d7 },
	{ CCI_REG16(0x6f12), 0x07fd },
	{ CCI_REG16(0x6f12), 0x850a },
	{ CCI_REG16(0x6f12), 0x6297 },
	{ CCI_REG16(0x6f12), 0x8356 },
	{ CCI_REG16(0x6f12), 0x0700 },
	{ CCI_REG16(0x6f12), 0x3315 },
	{ CCI_REG16(0x6f12), 0xf500 },
	{ CCI_REG16(0x6f12), 0x558d },
	{ CCI_REG16(0x6f12), 0x4205 },
	{ CCI_REG16(0x6f12), 0x4181 },
	{ CCI_REG16(0x6f12), 0x2310 },
	{ CCI_REG16(0x6f12), 0xa700 },
	{ CCI_REG16(0x6f12), 0x8509 },
	{ CCI_REG16(0x6f12), 0xe391 },
	{ CCI_REG16(0x6f12), 0x99f9 },
	{ CCI_REG16(0x6f12), 0x51bd },
	{ CCI_REG16(0x6f12), 0x1397 },
	{ CCI_REG16(0x6f12), 0x1900 },
	{ CCI_REG16(0x6f12), 0x9394 },
	{ CCI_REG16(0x6f12), 0x0701 },
	{ CCI_REG16(0x6f12), 0x5a97 },
	{ CCI_REG16(0x6f12), 0xc180 },
	{ CCI_REG16(0x6f12), 0x2310 },
	{ CCI_REG16(0x6f12), 0x9700 },
	{ CCI_REG16(0x6f12), 0xe5b7 },
	{ CCI_REG16(0x6f12), 0x1743 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0xe702 },
	{ CCI_REG16(0x6f12), 0xe326 },
	{ CCI_REG16(0x6f12), 0xb737 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x83a7 },
	{ CCI_REG16(0x6f12), 0x0761 },
	{ CCI_REG16(0x6f12), 0xaa84 },
	{ CCI_REG16(0x6f12), 0x2e89 },
	{ CCI_REG16(0x6f12), 0x8297 },
	{ CCI_REG16(0x6f12), 0xb787 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x03a4 },
	{ CCI_REG16(0x6f12), 0x0700 },
	{ CCI_REG16(0x6f12), 0x0146 },
	{ CCI_REG16(0x6f12), 0x1145 },
	{ CCI_REG16(0x6f12), 0xa285 },
	{ CCI_REG16(0x6f12), 0x9780 },
	{ CCI_REG16(0x6f12), 0xfffb },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0xc069 },
	{ CCI_REG16(0x6f12), 0xca85 },
	{ CCI_REG16(0x6f12), 0x2685 },
	{ CCI_REG16(0x6f12), 0x9730 },
	{ CCI_REG16(0x6f12), 0x00fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x20f5 },
	{ CCI_REG16(0x6f12), 0x0546 },
	{ CCI_REG16(0x6f12), 0xa285 },
	{ CCI_REG16(0x6f12), 0x1145 },
	{ CCI_REG16(0x6f12), 0x9780 },
	{ CCI_REG16(0x6f12), 0xfffb },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x2068 },
	{ CCI_REG16(0x6f12), 0x1743 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0x6700 },
	{ CCI_REG16(0x6f12), 0xc324 },
	{ CCI_REG16(0x6f12), 0xb717 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x83c7 },
	{ CCI_REG16(0x6f12), 0x0734 },
	{ CCI_REG16(0x6f12), 0xedcf },
	{ CCI_REG16(0x6f12), 0x1743 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0xe702 },
	{ CCI_REG16(0x6f12), 0x6321 },
	{ CCI_REG16(0x6f12), 0x9780 },
	{ CCI_REG16(0x6f12), 0x00fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x803e },
	{ CCI_REG16(0x6f12), 0xb787 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x1387 },
	{ CCI_REG16(0x6f12), 0x87f5 },
	{ CCI_REG16(0x6f12), 0x0347 },
	{ CCI_REG16(0x6f12), 0xf701 },
	{ CCI_REG16(0x6f12), 0x9387 },
	{ CCI_REG16(0x6f12), 0x87f5 },
	{ CCI_REG16(0x6f12), 0x19eb },
	{ CCI_REG16(0x6f12), 0x37f7 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x8356 },
	{ CCI_REG16(0x6f12), 0x6772 },
	{ CCI_REG16(0x6f12), 0x2391 },
	{ CCI_REG16(0x6f12), 0xd702 },
	{ CCI_REG16(0x6f12), 0x0357 },
	{ CCI_REG16(0x6f12), 0xa772 },
	{ CCI_REG16(0x6f12), 0x2392 },
	{ CCI_REG16(0x6f12), 0xe702 },
	{ CCI_REG16(0x6f12), 0xb776 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x83c6 },
	{ CCI_REG16(0x6f12), 0xb6f1 },
	{ CCI_REG16(0x6f12), 0x0547 },
	{ CCI_REG16(0x6f12), 0xa38f },
	{ CCI_REG16(0x6f12), 0xe700 },
	{ CCI_REG16(0x6f12), 0x99c6 },
	{ CCI_REG16(0x6f12), 0x83d6 },
	{ CCI_REG16(0x6f12), 0x4701 },
	{ CCI_REG16(0x6f12), 0x238f },
	{ CCI_REG16(0x6f12), 0xe700 },
	{ CCI_REG16(0x6f12), 0x2380 },
	{ CCI_REG16(0x6f12), 0xd702 },
	{ CCI_REG16(0x6f12), 0x83c6 },
	{ CCI_REG16(0x6f12), 0x0702 },
	{ CCI_REG16(0x6f12), 0x03c7 },
	{ CCI_REG16(0x6f12), 0xe701 },
	{ CCI_REG16(0x6f12), 0xb9ce },
	{ CCI_REG16(0x6f12), 0x0dc3 },
	{ CCI_REG16(0x6f12), 0x03d7 },
	{ CCI_REG16(0x6f12), 0x6701 },
	{ CCI_REG16(0x6f12), 0x0dcf },
	{ CCI_REG16(0x6f12), 0xb7f6 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x2393 },
	{ CCI_REG16(0x6f12), 0xe672 },
	{ CCI_REG16(0x6f12), 0x03d7 },
	{ CCI_REG16(0x6f12), 0x8701 },
	{ CCI_REG16(0x6f12), 0x0dcf },
	{ CCI_REG16(0x6f12), 0xb7f6 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x2395 },
	{ CCI_REG16(0x6f12), 0xe672 },
	{ CCI_REG16(0x6f12), 0x238f },
	{ CCI_REG16(0x6f12), 0x0700 },
	{ CCI_REG16(0x6f12), 0x03c7 },
	{ CCI_REG16(0x6f12), 0x0702 },
	{ CCI_REG16(0x6f12), 0x7d17 },
	{ CCI_REG16(0x6f12), 0x1377 },
	{ CCI_REG16(0x6f12), 0xf70f },
	{ CCI_REG16(0x6f12), 0x2380 },
	{ CCI_REG16(0x6f12), 0xe702 },
	{ CCI_REG16(0x6f12), 0x01e7 },
	{ CCI_REG16(0x6f12), 0x0547 },
	{ CCI_REG16(0x6f12), 0x238f },
	{ CCI_REG16(0x6f12), 0xe700 },
	{ CCI_REG16(0x6f12), 0x1743 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0x6700 },
	{ CCI_REG16(0x6f12), 0x631a },
	{ CCI_REG16(0x6f12), 0x83d6 },
	{ CCI_REG16(0x6f12), 0x2702 },
	{ CCI_REG16(0x6f12), 0x37f7 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x2313 },
	{ CCI_REG16(0x6f12), 0xd772 },
	{ CCI_REG16(0x6f12), 0xd1b7 },
	{ CCI_REG16(0x6f12), 0x83d6 },
	{ CCI_REG16(0x6f12), 0x4702 },
	{ CCI_REG16(0x6f12), 0x37f7 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x2315 },
	{ CCI_REG16(0x6f12), 0xd772 },
	{ CCI_REG16(0x6f12), 0xd1b7 },
	{ CCI_REG16(0x6f12), 0x71df },
	{ CCI_REG16(0x6f12), 0x03d7 },
	{ CCI_REG16(0x6f12), 0xa701 },
	{ CCI_REG16(0x6f12), 0x19cf },
	{ CCI_REG16(0x6f12), 0xb7f6 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x2393 },
	{ CCI_REG16(0x6f12), 0xe672 },
	{ CCI_REG16(0x6f12), 0x03d7 },
	{ CCI_REG16(0x6f12), 0xc701 },
	{ CCI_REG16(0x6f12), 0x19cf },
	{ CCI_REG16(0x6f12), 0xb7f6 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x2395 },
	{ CCI_REG16(0x6f12), 0xe672 },
	{ CCI_REG16(0x6f12), 0x238f },
	{ CCI_REG16(0x6f12), 0x0700 },
	{ CCI_REG16(0x6f12), 0x6dbf },
	{ CCI_REG16(0x6f12), 0x83d6 },
	{ CCI_REG16(0x6f12), 0x2702 },
	{ CCI_REG16(0x6f12), 0x37f7 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x2313 },
	{ CCI_REG16(0x6f12), 0xd772 },
	{ CCI_REG16(0x6f12), 0xc5b7 },
	{ CCI_REG16(0x6f12), 0x83d6 },
	{ CCI_REG16(0x6f12), 0x4702 },
	{ CCI_REG16(0x6f12), 0x37f7 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x2315 },
	{ CCI_REG16(0x6f12), 0xd772 },
	{ CCI_REG16(0x6f12), 0xc5b7 },
	{ CCI_REG16(0x6f12), 0x8280 },
	{ CCI_REG16(0x6f12), 0x1743 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0xe702 },
	{ CCI_REG16(0x6f12), 0xc311 },
	{ CCI_REG16(0x6f12), 0xb787 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0xa166 },
	{ CCI_REG16(0x6f12), 0xb775 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9386 },
	{ CCI_REG16(0x6f12), 0x76f7 },
	{ CCI_REG16(0x6f12), 0x3777 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9387 },
	{ CCI_REG16(0x6f12), 0xc701 },
	{ CCI_REG16(0x6f12), 0x2946 },
	{ CCI_REG16(0x6f12), 0x9385 },
	{ CCI_REG16(0x6f12), 0xa57f },
	{ CCI_REG16(0x6f12), 0x3545 },
	{ CCI_REG16(0x6f12), 0x2320 },
	{ CCI_REG16(0x6f12), 0xf73c },
	{ CCI_REG16(0x6f12), 0x9710 },
	{ CCI_REG16(0x6f12), 0x00fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0xa01f },
	{ CCI_REG16(0x6f12), 0xb777 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0xb785 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x3755 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x3737 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9387 },
	{ CCI_REG16(0x6f12), 0x4774 },
	{ CCI_REG16(0x6f12), 0x0146 },
	{ CCI_REG16(0x6f12), 0x9385 },
	{ CCI_REG16(0x6f12), 0xa58b },
	{ CCI_REG16(0x6f12), 0x1305 },
	{ CCI_REG16(0x6f12), 0x0537 },
	{ CCI_REG16(0x6f12), 0x232c },
	{ CCI_REG16(0x6f12), 0xf75e },
	{ CCI_REG16(0x6f12), 0x9710 },
	{ CCI_REG16(0x6f12), 0x00fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x4058 },
	{ CCI_REG16(0x6f12), 0xb787 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x23a6 },
	{ CCI_REG16(0x6f12), 0xa700 },
	{ CCI_REG16(0x6f12), 0xb785 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x3765 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x0146 },
	{ CCI_REG16(0x6f12), 0x9385 },
	{ CCI_REG16(0x6f12), 0xe59f },
	{ CCI_REG16(0x6f12), 0x1305 },
	{ CCI_REG16(0x6f12), 0x45b2 },
	{ CCI_REG16(0x6f12), 0x9710 },
	{ CCI_REG16(0x6f12), 0x00fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x2056 },
	{ CCI_REG16(0x6f12), 0xb787 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x23a2 },
	{ CCI_REG16(0x6f12), 0xa700 },
	{ CCI_REG16(0x6f12), 0xb785 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x3755 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x0146 },
	{ CCI_REG16(0x6f12), 0x9385 },
	{ CCI_REG16(0x6f12), 0x2597 },
	{ CCI_REG16(0x6f12), 0x1305 },
	{ CCI_REG16(0x6f12), 0x0525 },
	{ CCI_REG16(0x6f12), 0x9710 },
	{ CCI_REG16(0x6f12), 0x00fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x0054 },
	{ CCI_REG16(0x6f12), 0xb787 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x23a4 },
	{ CCI_REG16(0x6f12), 0xa700 },
	{ CCI_REG16(0x6f12), 0xb775 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x3775 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x0146 },
	{ CCI_REG16(0x6f12), 0x9385 },
	{ CCI_REG16(0x6f12), 0x257b },
	{ CCI_REG16(0x6f12), 0x1305 },
	{ CCI_REG16(0x6f12), 0x85d3 },
	{ CCI_REG16(0x6f12), 0x9710 },
	{ CCI_REG16(0x6f12), 0x00fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0xe051 },
	{ CCI_REG16(0x6f12), 0xb787 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x23a8 },
	{ CCI_REG16(0x6f12), 0xa700 },
	{ CCI_REG16(0x6f12), 0xb785 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x37b5 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x0146 },
	{ CCI_REG16(0x6f12), 0x9385 },
	{ CCI_REG16(0x6f12), 0x85ce },
	{ CCI_REG16(0x6f12), 0x1305 },
	{ CCI_REG16(0x6f12), 0xa5c6 },
	{ CCI_REG16(0x6f12), 0x9710 },
	{ CCI_REG16(0x6f12), 0x00fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0xc04f },
	{ CCI_REG16(0x6f12), 0xb787 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x23a0 },
	{ CCI_REG16(0x6f12), 0xa700 },
	{ CCI_REG16(0x6f12), 0xb787 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x3737 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9387 },
	{ CCI_REG16(0x6f12), 0x67d3 },
	{ CCI_REG16(0x6f12), 0x2320 },
	{ CCI_REG16(0x6f12), 0xf768 },
	{ CCI_REG16(0x6f12), 0x1743 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0x6700 },
	{ CCI_REG16(0x6f12), 0x4304 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0203 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0203 },
	{ CCI_REG16(0x6f12), 0x0405 },
	{ CCI_REG16(0x6f12), 0x0607 },
	{ CCI_REG16(0x6f12), 0x0405 },
	{ CCI_REG16(0x6f12), 0x0607 },
	{ CCI_REG16(0x6f12), 0x10d0 },
	{ CCI_REG16(0x6f12), 0x10d0 },
	{ CCI_REG16(0x6f12), 0x1cd0 },
	{ CCI_REG16(0x6f12), 0x1cd0 },
	{ CCI_REG16(0x6f12), 0x22d0 },
	{ CCI_REG16(0x6f12), 0x22d0 },
	{ CCI_REG16(0x6f12), 0x2ed0 },
	{ CCI_REG16(0x6f12), 0x2ed0 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0c00 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0c00 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x30d0 },
	{ CCI_REG16(0x6f12), 0x32d0 },
	{ CCI_REG16(0x6f12), 0x64d0 },
	{ CCI_REG16(0x6f12), 0x66d0 },
	{ CCI_REG16(0x6f12), 0x7cd0 },
	{ CCI_REG16(0x6f12), 0x7ed0 },
	{ CCI_REG16(0x6f12), 0xa8d0 },
	{ CCI_REG16(0x6f12), 0xaad0 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0405 },
	{ CCI_REG16(0x6f12), 0x0405 },
	{ CCI_REG16(0x6f12), 0x0405 },
	{ CCI_REG16(0x6f12), 0x0405 },
	{ CCI_REG16(0x6f12), 0x10d0 },
	{ CCI_REG16(0x6f12), 0x10d0 },
	{ CCI_REG16(0x6f12), 0x12d0 },
	{ CCI_REG16(0x6f12), 0x12d0 },
	{ CCI_REG16(0x6f12), 0x20d0 },
	{ CCI_REG16(0x6f12), 0x20d0 },
	{ CCI_REG16(0x6f12), 0x22d0 },
	{ CCI_REG16(0x6f12), 0x22d0 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x30d0 },
	{ CCI_REG16(0x6f12), 0x32d0 },
	{ CCI_REG16(0x6f12), 0x38d0 },
	{ CCI_REG16(0x6f12), 0x3ad0 },
	{ CCI_REG16(0x6f12), 0x70d0 },
	{ CCI_REG16(0x6f12), 0x72d0 },
	{ CCI_REG16(0x6f12), 0x78d0 },
	{ CCI_REG16(0x6f12), 0x7ad0 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x1743 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0xe702 },
	{ CCI_REG16(0x6f12), 0xa3f3 },
	{ CCI_REG16(0x6f12), 0xb787 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x1307 },
	{ CCI_REG16(0x6f12), 0xc00e },
	{ CCI_REG16(0x6f12), 0x9387 },
	{ CCI_REG16(0x6f12), 0xc701 },
	{ CCI_REG16(0x6f12), 0xb785 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x3755 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0xba97 },
	{ CCI_REG16(0x6f12), 0x0146 },
	{ CCI_REG16(0x6f12), 0x3777 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9385 },
	{ CCI_REG16(0x6f12), 0x4507 },
	{ CCI_REG16(0x6f12), 0x1305 },
	{ CCI_REG16(0x6f12), 0xc504 },
	{ CCI_REG16(0x6f12), 0x2320 },
	{ CCI_REG16(0x6f12), 0xf73c },
	{ CCI_REG16(0x6f12), 0x9710 },
	{ CCI_REG16(0x6f12), 0x00fc },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0x603c },
	{ CCI_REG16(0x6f12), 0xb787 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x23a0 },
	{ CCI_REG16(0x6f12), 0xa710 },
	{ CCI_REG16(0x6f12), 0x4928 },
	{ CCI_REG16(0x6f12), 0xe177 },
	{ CCI_REG16(0x6f12), 0x3747 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9387 },
	{ CCI_REG16(0x6f12), 0x5776 },
	{ CCI_REG16(0x6f12), 0x2317 },
	{ CCI_REG16(0x6f12), 0xf782 },
	{ CCI_REG16(0x6f12), 0x1743 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0x6700 },
	{ CCI_REG16(0x6f12), 0xe3f0 },
	{ CCI_REG16(0x6f12), 0x1743 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0xe702 },
	{ CCI_REG16(0x6f12), 0x83ec },
	{ CCI_REG16(0x6f12), 0xb787 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x83a4 },
	{ CCI_REG16(0x6f12), 0x0710 },
	{ CCI_REG16(0x6f12), 0xaa89 },
	{ CCI_REG16(0x6f12), 0x2e8a },
	{ CCI_REG16(0x6f12), 0x0146 },
	{ CCI_REG16(0x6f12), 0xa685 },
	{ CCI_REG16(0x6f12), 0x1145 },
	{ CCI_REG16(0x6f12), 0x9780 },
	{ CCI_REG16(0x6f12), 0xfffb },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0xa031 },
	{ CCI_REG16(0x6f12), 0xb787 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x03c7 },
	{ CCI_REG16(0x6f12), 0x4710 },
	{ CCI_REG16(0x6f12), 0x3e84 },
	{ CCI_REG16(0x6f12), 0x0149 },
	{ CCI_REG16(0x6f12), 0x11cf },
	{ CCI_REG16(0x6f12), 0x3767 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x0357 },
	{ CCI_REG16(0x6f12), 0x2777 },
	{ CCI_REG16(0x6f12), 0xb777 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9387 },
	{ CCI_REG16(0x6f12), 0x07c7 },
	{ CCI_REG16(0x6f12), 0x0e07 },
	{ CCI_REG16(0x6f12), 0x03d9 },
	{ CCI_REG16(0x6f12), 0x871c },
	{ CCI_REG16(0x6f12), 0x2394 },
	{ CCI_REG16(0x6f12), 0xe71c },
	{ CCI_REG16(0x6f12), 0xd285 },
	{ CCI_REG16(0x6f12), 0x4e85 },
	{ CCI_REG16(0x6f12), 0x97d0 },
	{ CCI_REG16(0x6f12), 0xfffb },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0xa0f8 },
	{ CCI_REG16(0x6f12), 0x8347 },
	{ CCI_REG16(0x6f12), 0x4410 },
	{ CCI_REG16(0x6f12), 0x89c7 },
	{ CCI_REG16(0x6f12), 0xb777 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x239c },
	{ CCI_REG16(0x6f12), 0x27e3 },
	{ CCI_REG16(0x6f12), 0x0546 },
	{ CCI_REG16(0x6f12), 0xa685 },
	{ CCI_REG16(0x6f12), 0x1145 },
	{ CCI_REG16(0x6f12), 0x9780 },
	{ CCI_REG16(0x6f12), 0xfffb },
	{ CCI_REG16(0x6f12), 0xe780 },
	{ CCI_REG16(0x6f12), 0xa02c },
	{ CCI_REG16(0x6f12), 0x1743 },
	{ CCI_REG16(0x6f12), 0x01fc },
	{ CCI_REG16(0x6f12), 0x6700 },
	{ CCI_REG16(0x6f12), 0xa3e8 },
	{ CCI_REG16(0x6f12), 0xe177 },
	{ CCI_REG16(0x6f12), 0x3747 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6f12), 0x9387 },
	{ CCI_REG16(0x6f12), 0x5776 },
	{ CCI_REG16(0x6f12), 0x2318 },
	{ CCI_REG16(0x6f12), 0xf782 },
	{ CCI_REG16(0x6f12), 0x8280 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x35cc },
	{ CCI_REG16(0x6f12), 0x1c80 },
	{ CCI_REG16(0x6f12), 0x0024 },
	{ CCI_REG16(0x6028), 0x2400 },
	{ CCI_REG16(0x602a), 0x1354 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x7017 },
	{ CCI_REG16(0x602a), 0x13b2 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1236 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1a0a },
	{ CCI_REG16(0x6f12), 0x4c0a },
	{ CCI_REG16(0x602a), 0x2210 },
	{ CCI_REG16(0x6f12), 0x3401 },
	{ CCI_REG16(0x602a), 0x2176 },
	{ CCI_REG16(0x6f12), 0x6400 },
	{ CCI_REG16(0x602a), 0x222e },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x06b6 },
	{ CCI_REG16(0x6f12), 0x0a00 },
	{ CCI_REG16(0x602a), 0x06bc },
	{ CCI_REG16(0x6f12), 0x1001 },
	{ CCI_REG16(0x602a), 0x2140 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x218e },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1a0e },
	{ CCI_REG16(0x6f12), 0x9600 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0xf44e), 0x0011 },
	{ CCI_REG16(0xf44c), 0x0b0b },
	{ CCI_REG16(0xf44a), 0x0006 },
	{ CCI_REG16(0x0118), 0x0002 },
	{ CCI_REG16(0x011a), 0x0001 },
};

static const struct cci_reg_sequence s5kjn1_4080x3072_30fps_mode[] = {
	{ CCI_REG16(0x6028), 0x2400 },
	{ CCI_REG16(0x602a), 0x1a28 },
	{ CCI_REG16(0x6f12), 0x4c00 },
	{ CCI_REG16(0x602a), 0x065a },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x139e },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x139c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x13a0 },
	{ CCI_REG16(0x6f12), 0x0a00 },
	{ CCI_REG16(0x6f12), 0x0120 },
	{ CCI_REG16(0x602a), 0x2072 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1a64 },
	{ CCI_REG16(0x6f12), 0x0301 },
	{ CCI_REG16(0x6f12), 0xff00 },
	{ CCI_REG16(0x602a), 0x19e6 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x1a30 },
	{ CCI_REG16(0x6f12), 0x3401 },
	{ CCI_REG16(0x602a), 0x19fc },
	{ CCI_REG16(0x6f12), 0x0b00 },
	{ CCI_REG16(0x602a), 0x19f4 },
	{ CCI_REG16(0x6f12), 0x0606 },
	{ CCI_REG16(0x602a), 0x19f8 },
	{ CCI_REG16(0x6f12), 0x1010 },
	{ CCI_REG16(0x602a), 0x1b26 },
	{ CCI_REG16(0x6f12), 0x6f80 },
	{ CCI_REG16(0x6f12), 0xa060 },
	{ CCI_REG16(0x602a), 0x1a3c },
	{ CCI_REG16(0x6f12), 0x6207 },
	{ CCI_REG16(0x602a), 0x1a48 },
	{ CCI_REG16(0x6f12), 0x6207 },
	{ CCI_REG16(0x602a), 0x1444 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x602a), 0x144c },
	{ CCI_REG16(0x6f12), 0x3f00 },
	{ CCI_REG16(0x6f12), 0x3f00 },
	{ CCI_REG16(0x602a), 0x7f6c },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x2f00 },
	{ CCI_REG16(0x6f12), 0xfa00 },
	{ CCI_REG16(0x6f12), 0x2400 },
	{ CCI_REG16(0x6f12), 0xe500 },
	{ CCI_REG16(0x602a), 0x0650 },
	{ CCI_REG16(0x6f12), 0x0600 },
	{ CCI_REG16(0x602a), 0x0654 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1a46 },
	{ CCI_REG16(0x6f12), 0xba00 },
	{ CCI_REG16(0x602a), 0x1a52 },
	{ CCI_REG16(0x6f12), 0xbf00 },
	{ CCI_REG16(0x602a), 0x0674 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x602a), 0x0668 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x602a), 0x0684 },
	{ CCI_REG16(0x6f12), 0x4001 },
	{ CCI_REG16(0x602a), 0x0688 },
	{ CCI_REG16(0x6f12), 0x4001 },
	{ CCI_REG16(0x602a), 0x147c },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x1480 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x19f6 },
	{ CCI_REG16(0x6f12), 0x0904 },
	{ CCI_REG16(0x602a), 0x0812 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1a02 },
	{ CCI_REG16(0x6f12), 0x1800 },
	{ CCI_REG16(0x602a), 0x2148 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x2042 },
	{ CCI_REG16(0x6f12), 0x1a00 },
	{ CCI_REG16(0x602a), 0x0874 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x09c0 },
	{ CCI_REG16(0x6f12), 0x2008 },
	{ CCI_REG16(0x602a), 0x09c4 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x602a), 0x19fe },
	{ CCI_REG16(0x6f12), 0x0e1c },
	{ CCI_REG16(0x602a), 0x4d92 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x84c8 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x4d94 },
	{ CCI_REG16(0x6f12), 0x0005 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x0810 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x6f12), 0x0810 },
	{ CCI_REG16(0x6f12), 0x0810 },
	{ CCI_REG16(0x6f12), 0x8002 },
	{ CCI_REG16(0x6f12), 0xfd03 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x1510 },
	{ CCI_REG16(0x602a), 0x3570 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x3574 },
	{ CCI_REG16(0x6f12), 0x1201 },
	{ CCI_REG16(0x602a), 0x21e4 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x602a), 0x21ec },
	{ CCI_REG16(0x6f12), 0x1f04 },
	{ CCI_REG16(0x602a), 0x2080 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x6f12), 0xff00 },
	{ CCI_REG16(0x6f12), 0x7f01 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x8001 },
	{ CCI_REG16(0x6f12), 0xd244 },
	{ CCI_REG16(0x6f12), 0xd244 },
	{ CCI_REG16(0x6f12), 0x14f4 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x20ba },
	{ CCI_REG16(0x6f12), 0x141c },
	{ CCI_REG16(0x6f12), 0x111c },
	{ CCI_REG16(0x6f12), 0x54f4 },
	{ CCI_REG16(0x602a), 0x120e },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x212e },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x13ae },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x0718 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x0710 },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x6f12), 0x0804 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x1b5c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x0786 },
	{ CCI_REG16(0x6f12), 0x7701 },
	{ CCI_REG16(0x602a), 0x2022 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x602a), 0x1360 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x1376 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x6038 },
	{ CCI_REG16(0x6f12), 0x7038 },
	{ CCI_REG16(0x6f12), 0x8038 },
	{ CCI_REG16(0x602a), 0x1386 },
	{ CCI_REG16(0x6f12), 0x0b00 },
	{ CCI_REG16(0x602a), 0x06fa },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x4a94 },
	{ CCI_REG16(0x6f12), 0x0900 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0900 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x0a76 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0aee },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0b66 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0bde },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0be8 },
	{ CCI_REG16(0x6f12), 0x3000 },
	{ CCI_REG16(0x6f12), 0x3000 },
	{ CCI_REG16(0x602a), 0x0c56 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0c60 },
	{ CCI_REG16(0x6f12), 0x3000 },
	{ CCI_REG16(0x6f12), 0x3000 },
	{ CCI_REG16(0x602a), 0x0cb6 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x0cf2 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x0cf0 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x11b8 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x11f6 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x602a), 0x4a74 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0xd8ff },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0xd8ff },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x218e },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x2268 },
	{ CCI_REG16(0x6f12), 0xf279 },
	{ CCI_REG16(0x602a), 0x5006 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x500e },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x4e70 },
	{ CCI_REG16(0x6f12), 0x2062 },
	{ CCI_REG16(0x6f12), 0x5501 },
	{ CCI_REG16(0x602a), 0x06dc },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0xf46a), 0xae80 },
	{ CCI_REG16(0x0344), 0x0000 },
	{ CCI_REG16(0x0346), 0x0000 },
	{ CCI_REG16(0x0348), 0x1fff },
	{ CCI_REG16(0x034a), 0x181f },
	{ CCI_REG16(0x034c), 0x0ff0 },
	{ CCI_REG16(0x034e), 0x0c00 },
	{ CCI_REG16(0x0350), 0x0008 },
	{ CCI_REG16(0x0352), 0x0008 },
	{ CCI_REG16(0x0900), 0x0122 },
	{ CCI_REG16(0x0380), 0x0002 },
	{ CCI_REG16(0x0382), 0x0002 },
	{ CCI_REG16(0x0384), 0x0002 },
	{ CCI_REG16(0x0386), 0x0002 },
	{ CCI_REG16(0x0110), 0x1002 },
	{ CCI_REG16(0x0114), 0x0301 },
	{ CCI_REG16(0x0116), 0x3000 },
	{ CCI_REG16(0x0136), 0x1800 },
	{ CCI_REG16(0x013e), 0x0000 },
	{ CCI_REG16(0x0300), 0x0006 },
	{ CCI_REG16(0x0302), 0x0001 },
	{ CCI_REG16(0x0304), 0x0004 },
	{ CCI_REG16(0x0306), 0x008c },
	{ CCI_REG16(0x0308), 0x0008 },
	{ CCI_REG16(0x030a), 0x0001 },
	{ CCI_REG16(0x030c), 0x0000 },
	{ CCI_REG16(0x030e), 0x0004 },
	{ CCI_REG16(0x0310), 0x0069 },
	{ CCI_REG16(0x0312), 0x0000 },
	{ CCI_REG16(0x080e), 0x0000 },
	{ CCI_REG16(0x0340), 0x0c60 },
	{ CCI_REG16(0x0342), 0x1706 },
	{ CCI_REG16(0x0702), 0x0000 },
	{ CCI_REG16(0x0202), 0x0100 },
	{ CCI_REG16(0x0200), 0x0100 },
	{ CCI_REG16(0x0d00), 0x0101 },
	{ CCI_REG16(0x0d02), 0x0101 },
	{ CCI_REG16(0x0d04), 0x0102 },
	{ CCI_REG16(0x6226), 0x0000 },
};

static const struct cci_reg_sequence s5kjn1_8160x6144_10fps_mode[] = {
	{ CCI_REG16(0x6028), 0x2400 },
	{ CCI_REG16(0x602a), 0x1a28 },
	{ CCI_REG16(0x6f12), 0x4c00 },
	{ CCI_REG16(0x602a), 0x065a },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x139e },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x602a), 0x139c },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x13a0 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x6f12), 0x0120 },
	{ CCI_REG16(0x602a), 0x2072 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x1a64 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x19e6 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x1a30 },
	{ CCI_REG16(0x6f12), 0x3403 },
	{ CCI_REG16(0x602a), 0x19fc },
	{ CCI_REG16(0x6f12), 0x0700 },
	{ CCI_REG16(0x602a), 0x19f4 },
	{ CCI_REG16(0x6f12), 0x0707 },
	{ CCI_REG16(0x602a), 0x19f8 },
	{ CCI_REG16(0x6f12), 0x0b0b },
	{ CCI_REG16(0x602a), 0x1b26 },
	{ CCI_REG16(0x6f12), 0x6f80 },
	{ CCI_REG16(0x6f12), 0xa060 },
	{ CCI_REG16(0x602a), 0x1a3c },
	{ CCI_REG16(0x6f12), 0x8207 },
	{ CCI_REG16(0x602a), 0x1a48 },
	{ CCI_REG16(0x6f12), 0x8207 },
	{ CCI_REG16(0x602a), 0x1444 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x602a), 0x144c },
	{ CCI_REG16(0x6f12), 0x3f00 },
	{ CCI_REG16(0x6f12), 0x3f00 },
	{ CCI_REG16(0x602a), 0x7f6c },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x2f00 },
	{ CCI_REG16(0x6f12), 0xfa00 },
	{ CCI_REG16(0x6f12), 0x2400 },
	{ CCI_REG16(0x6f12), 0xe500 },
	{ CCI_REG16(0x602a), 0x0650 },
	{ CCI_REG16(0x6f12), 0x0600 },
	{ CCI_REG16(0x602a), 0x0654 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1a46 },
	{ CCI_REG16(0x6f12), 0x8500 },
	{ CCI_REG16(0x602a), 0x1a52 },
	{ CCI_REG16(0x6f12), 0x9800 },
	{ CCI_REG16(0x602a), 0x0674 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x602a), 0x0668 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x602a), 0x0684 },
	{ CCI_REG16(0x6f12), 0x4001 },
	{ CCI_REG16(0x602a), 0x0688 },
	{ CCI_REG16(0x6f12), 0x4001 },
	{ CCI_REG16(0x602a), 0x147c },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x602a), 0x1480 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x602a), 0x19f6 },
	{ CCI_REG16(0x6f12), 0x0404 },
	{ CCI_REG16(0x602a), 0x0812 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1a02 },
	{ CCI_REG16(0x6f12), 0x1800 },
	{ CCI_REG16(0x602a), 0x2148 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x2042 },
	{ CCI_REG16(0x6f12), 0x1a00 },
	{ CCI_REG16(0x602a), 0x0874 },
	{ CCI_REG16(0x6f12), 0x0106 },
	{ CCI_REG16(0x602a), 0x09c0 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x602a), 0x09c4 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x602a), 0x19fe },
	{ CCI_REG16(0x6f12), 0x0c1c },
	{ CCI_REG16(0x602a), 0x4d92 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x84c8 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x4d94 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x3570 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x3574 },
	{ CCI_REG16(0x6f12), 0x7306 },
	{ CCI_REG16(0x602a), 0x21e4 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x602a), 0x21ec },
	{ CCI_REG16(0x6f12), 0x6902 },
	{ CCI_REG16(0x602a), 0x2080 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0xff00 },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x6f12), 0xd244 },
	{ CCI_REG16(0x6f12), 0xd244 },
	{ CCI_REG16(0x6f12), 0x14f4 },
	{ CCI_REG16(0x6f12), 0x101c },
	{ CCI_REG16(0x6f12), 0x0d1c },
	{ CCI_REG16(0x6f12), 0x54f4 },
	{ CCI_REG16(0x602a), 0x20ba },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x120e },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x212e },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x13ae },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x0718 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x0710 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x0201 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x602a), 0x1b5c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x0786 },
	{ CCI_REG16(0x6f12), 0x1401 },
	{ CCI_REG16(0x602a), 0x2022 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x6f12), 0x0500 },
	{ CCI_REG16(0x602a), 0x1360 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1376 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x6038 },
	{ CCI_REG16(0x6f12), 0x7038 },
	{ CCI_REG16(0x6f12), 0x8038 },
	{ CCI_REG16(0x602a), 0x1386 },
	{ CCI_REG16(0x6f12), 0x0b00 },
	{ CCI_REG16(0x602a), 0x06fa },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x4a94 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0400 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x602a), 0x0a76 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0aee },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0b66 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0bde },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0be8 },
	{ CCI_REG16(0x6f12), 0x5000 },
	{ CCI_REG16(0x6f12), 0x5000 },
	{ CCI_REG16(0x602a), 0x0c56 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x602a), 0x0c60 },
	{ CCI_REG16(0x6f12), 0x5000 },
	{ CCI_REG16(0x6f12), 0x5000 },
	{ CCI_REG16(0x602a), 0x0cb6 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x0cf2 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x0cf0 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x11b8 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x11f6 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x602a), 0x4a74 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x218e },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x2268 },
	{ CCI_REG16(0x6f12), 0xf279 },
	{ CCI_REG16(0x602a), 0x5006 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x500e },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x4e70 },
	{ CCI_REG16(0x6f12), 0x2062 },
	{ CCI_REG16(0x6f12), 0x5501 },
	{ CCI_REG16(0x602a), 0x06dc },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0xf46a), 0xae80 },
	{ S5KJN1_REG_X_ADDR_START,  0x0000 },
	{ S5KJN1_REG_Y_ADDR_START,  0x0000 },
	{ S5KJN1_REG_X_ADDR_END,    0x1fff },
	{ S5KJN1_REG_Y_ADDR_END,    0x181f },
	{ S5KJN1_REG_X_OUTPUT_SIZE, 0x1fe0 },
	{ S5KJN1_REG_Y_OUTPUT_SIZE, 0x1800 },
	{ CCI_REG16(0x0350), 0x0010 },
	{ CCI_REG16(0x0352), 0x0010 },
	{ CCI_REG16(0x0900), 0x0111 },
	{ CCI_REG16(0x0380), 0x0001 },
	{ CCI_REG16(0x0382), 0x0001 },
	{ CCI_REG16(0x0384), 0x0001 },
	{ CCI_REG16(0x0386), 0x0001 },
	{ CCI_REG16(0x0110), 0x1002 },
	{ CCI_REG16(0x0114), 0x0300 },
	{ CCI_REG16(0x0116), 0x3000 },

	/* Clock settings */
	{ CCI_REG16(0x0136), 0x1800 },
	{ CCI_REG16(0x013e), 0x0000 },
	{ CCI_REG16(0x0300), 0x0006 },
	{ CCI_REG16(0x0302), 0x0001 },
	{ CCI_REG16(0x0304), 0x0004 },
	{ CCI_REG16(0x0306), 0x008c },
	{ CCI_REG16(0x0308), 0x0008 },
	{ CCI_REG16(0x030a), 0x0001 },
	{ CCI_REG16(0x030c), 0x0000 },
	{ CCI_REG16(0x030e), 0x0004 },
	{ CCI_REG16(0x0310), 0x0074 },
	{ CCI_REG16(0x0312), 0x0000 },

	{ CCI_REG16(0x080e), 0x0000 },
	{ S5KJN1_REG_VTS,    0x1900 },
	{ S5KJN1_REG_HTS,    0x21f0 },
	{ CCI_REG16(0x0702), 0x0000 },
	{ S5KJN1_REG_EXPOSURE, 0x0100 },
	{ CCI_REG16(0x0200), 0x0100 },
	{ CCI_REG16(0x0d00), 0x0100 },
	{ CCI_REG16(0x0d02), 0x0001 },
	{ CCI_REG16(0x0d04), 0x0002 },
	{ CCI_REG16(0x6226), 0x0000 },
};

static const struct s5kjn1_mode s5kjn1_supported_modes[] = {
	{
		.width = 4080,
		.height = 3072,
		.hts = 4352,
		.vts = 4288,
		.exposure = 3840,
		.exposure_margin = 22,
		.reg_list = {
			.regs = s5kjn1_4080x3072_30fps_mode,
			.num_regs = ARRAY_SIZE(s5kjn1_4080x3072_30fps_mode),
		},
	},
	{
		.width = 8160,
		.height = 6144,
		.hts = 8688,
		.vts = 6400,
		.exposure = 6144,
		.exposure_margin = 44,
		.reg_list = {
			.regs = s5kjn1_8160x6144_10fps_mode,
			.num_regs = ARRAY_SIZE(s5kjn1_8160x6144_10fps_mode),
		},
	},
};

static int s5kjn1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5kjn1 *s5kjn1 = container_of(ctrl->handler, struct s5kjn1,
					     ctrl_handler);
	const struct s5kjn1_mode *mode = s5kjn1->mode;
	s64 exposure_max;
	int ret;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		exposure_max = mode->height + ctrl->val - mode->exposure_margin;
		__v4l2_ctrl_modify_range(s5kjn1->exposure,
					 s5kjn1->exposure->minimum,
					 exposure_max,
					 s5kjn1->exposure->step,
					 s5kjn1->exposure->default_value);
		break;
	}

	/* V4L2 controls are applied, when sensor is powered up for streaming */
	if (!pm_runtime_get_if_active(s5kjn1->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(s5kjn1->regmap, S5KJN1_REG_AGAIN,
				ctrl->val << S5KJN1_AGAIN_SHIFT, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(s5kjn1->regmap, S5KJN1_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(s5kjn1->regmap, S5KJN1_REG_VTS,
				ctrl->val + mode->height, NULL);
		break;
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		ret = cci_write(s5kjn1->regmap, S5KJN1_REG_ORIENTATION,
				(s5kjn1->vflip->val ? S5KJN1_VFLIP : 0) |
				(s5kjn1->hflip->val ? S5KJN1_HFLIP : 0), NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(s5kjn1->regmap, S5KJN1_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(s5kjn1->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5kjn1_ctrl_ops = {
	.s_ctrl = s5kjn1_set_ctrl,
};

static inline u64 s5kjn1_freq_to_pixel_rate(const u64 freq)
{
	return div_u64(freq * 2 * S5KJN1_DATA_LANES, 10);
}

static int s5kjn1_init_controls(struct s5kjn1 *s5kjn1)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &s5kjn1->ctrl_handler;
	const struct s5kjn1_mode *mode = s5kjn1->mode;
	s64 pixel_rate, hblank, vblank, exposure_max;
	struct v4l2_fwnode_device_properties props;
	int ret;

	v4l2_ctrl_handler_init(ctrl_hdlr, 9);

	s5kjn1->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &s5kjn1_ctrl_ops,
					V4L2_CID_LINK_FREQ,
					ARRAY_SIZE(s5kjn1_link_freq_menu) - 1,
					0, s5kjn1_link_freq_menu);
	if (s5kjn1->link_freq)
		s5kjn1->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pixel_rate = s5kjn1_freq_to_pixel_rate(s5kjn1_link_freq_menu[0]);
	s5kjn1->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &s5kjn1_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       0, pixel_rate, 1, pixel_rate);

	hblank = mode->hts - mode->width;
	s5kjn1->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5kjn1_ctrl_ops,
					   V4L2_CID_HBLANK, hblank,
					   hblank, 1, hblank);
	if (s5kjn1->hblank)
		s5kjn1->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->vts - mode->height;
	s5kjn1->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5kjn1_ctrl_ops,
					   V4L2_CID_VBLANK, vblank,
					   S5KJN1_VTS_MAX - mode->height, 1,
					   vblank);

	v4l2_ctrl_new_std(ctrl_hdlr, &s5kjn1_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  S5KJN1_AGAIN_MIN, S5KJN1_AGAIN_MAX,
			  S5KJN1_AGAIN_STEP, S5KJN1_AGAIN_DEFAULT);

	exposure_max = mode->vts - mode->exposure_margin;
	s5kjn1->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &s5kjn1_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5KJN1_EXPOSURE_MIN,
					     exposure_max,
					     S5KJN1_EXPOSURE_STEP,
					     mode->exposure);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &s5kjn1_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5kjn1_test_pattern_menu) - 1,
				     0, 0, s5kjn1_test_pattern_menu);

	s5kjn1->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5kjn1_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (s5kjn1->hflip)
		s5kjn1->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	s5kjn1->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5kjn1_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (s5kjn1->vflip)
		s5kjn1->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	ret = v4l2_fwnode_device_parse(s5kjn1->dev, &props);
	if (ret)
		goto error_free_hdlr;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &s5kjn1_ctrl_ops,
					      &props);
	if (ret)
		goto error_free_hdlr;

	s5kjn1->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error_free_hdlr:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static int s5kjn1_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct s5kjn1 *s5kjn1 = to_s5kjn1(sd);
	const struct s5kjn1_reg_list *reg_list = &s5kjn1->mode->reg_list;
	int ret;

	ret = pm_runtime_resume_and_get(s5kjn1->dev);
	if (ret)
		return ret;

	/* Page pointer */
	cci_write(s5kjn1->regmap, CCI_REG16(0x6028), 0x4000, &ret);

	/* Set version */
	cci_write(s5kjn1->regmap, CCI_REG16(0x0000), 0x0003, &ret);
	cci_write(s5kjn1->regmap, CCI_REG16(0x0000), S5KJN1_CHIP_ID, &ret);
	cci_write(s5kjn1->regmap, CCI_REG16(0x001e), 0x0007, &ret);
	cci_write(s5kjn1->regmap, CCI_REG16(0x6028), 0x4000, &ret);
	cci_write(s5kjn1->regmap, CCI_REG16(0x6010), 0x0001, &ret);
	if (ret)
		goto error;

	usleep_range(5 * USEC_PER_MSEC, 6 * USEC_PER_MSEC);

	cci_write(s5kjn1->regmap, CCI_REG16(0x6226), 0x0001, &ret);
	if (ret)
		goto error;

	usleep_range(10 * USEC_PER_MSEC, 11 * USEC_PER_MSEC);

	/* Sensor init settings */
	cci_multi_reg_write(s5kjn1->regmap, init_array_setting,
			    ARRAY_SIZE(init_array_setting), &ret);
	cci_multi_reg_write(s5kjn1->regmap, reg_list->regs,
			    reg_list->num_regs, &ret);
	if (ret)
		goto error;

	ret = __v4l2_ctrl_handler_setup(s5kjn1->sd.ctrl_handler);

	cci_write(s5kjn1->regmap, S5KJN1_REG_CTRL_MODE,
		  S5KJN1_MODE_STREAMING, &ret);
	if (ret)
		goto error;

	{
		u64 mode = 0, chip = 0;
		int r2 = 0;

		cci_read(s5kjn1->regmap, S5KJN1_REG_CTRL_MODE, &mode, &r2);
		cci_read(s5kjn1->regmap, CCI_REG16(0x0000), &chip, &r2);
		dev_info(s5kjn1->dev,
			 "streaming on: ctrl_mode=%04x chip=%04x read_ret=%d\n",
			 mode, chip, r2);
	}

	return 0;

error:
	dev_err(s5kjn1->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(s5kjn1->dev);

	return ret;
}

static int s5kjn1_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5kjn1 *s5kjn1 = to_s5kjn1(sd);
	int ret;

	ret = cci_write(s5kjn1->regmap, S5KJN1_REG_CTRL_MODE, 0x0, NULL);
	if (ret)
		dev_err(s5kjn1->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(s5kjn1->dev);

	return ret;
}

static ssize_t s5kjn1_dbg_stream_write(struct file *file,
				       const char __user *ubuf,
				       size_t count, loff_t *ppos)
{
	struct s5kjn1 *s5kjn1 = file->private_data;
	char buf[8];
	int on, ret;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';
	if (kstrtoint(buf, 10, &on))
		return -EINVAL;

	if (on)
		ret = s5kjn1_enable_streams(&s5kjn1->sd, NULL, 0, BIT(0));
	else
		ret = s5kjn1_disable_streams(&s5kjn1->sd, NULL, 0, BIT(0));
	if (ret)
		return ret;
	return count;
}

static const struct file_operations s5kjn1_dbg_stream_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = s5kjn1_dbg_stream_write,
	.llseek = default_llseek,
};

static u32 s5kjn1_get_format_code(struct s5kjn1 *s5kjn1)
{
	unsigned int i;

	i = (s5kjn1->vflip->val ? 2 : 0) | (s5kjn1->hflip->val ? 1 : 0);

	return s5kjn1_mbus_formats[i];
}

static void s5kjn1_update_pad_format(struct s5kjn1 *s5kjn1,
				     const struct s5kjn1_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = s5kjn1_get_format_code(s5kjn1);
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5kjn1_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct s5kjn1 *s5kjn1 = to_s5kjn1(sd);
	s64 hblank, vblank, exposure_max;
	const struct s5kjn1_mode *mode;

	mode = v4l2_find_nearest_size(s5kjn1_supported_modes,
				      ARRAY_SIZE(s5kjn1_supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	s5kjn1_update_pad_format(s5kjn1, mode, &fmt->format);

	/* Format code could be updated with respect to flip controls */
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY || s5kjn1->mode == mode)
		goto set_format;

	/* Update limits and set FPS and exposure to default values */
	hblank = mode->hts - mode->width;
	__v4l2_ctrl_modify_range(s5kjn1->hblank, hblank, hblank, 1, hblank);

	vblank = mode->vts - mode->height;
	__v4l2_ctrl_modify_range(s5kjn1->vblank, vblank,
				 S5KJN1_VTS_MAX - mode->height, 1, vblank);
	__v4l2_ctrl_s_ctrl(s5kjn1->vblank, vblank);

	exposure_max = mode->vts - mode->exposure_margin;
	__v4l2_ctrl_modify_range(s5kjn1->exposure, S5KJN1_EXPOSURE_MIN,
				 exposure_max, S5KJN1_EXPOSURE_STEP,
				 mode->exposure);
	__v4l2_ctrl_s_ctrl(s5kjn1->exposure, mode->exposure);

	if (s5kjn1->sd.ctrl_handler->error)
		return s5kjn1->sd.ctrl_handler->error;

	s5kjn1->mode = mode;

set_format:
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int s5kjn1_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct s5kjn1 *s5kjn1 = to_s5kjn1(sd);

	/* Media bus code index is constant, but code formats are not */
	if (code->index > 0)
		return -EINVAL;

	code->code = s5kjn1_get_format_code(s5kjn1);

	return 0;
}

static int s5kjn1_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct s5kjn1 *s5kjn1 = to_s5kjn1(sd);

	if (fse->index >= ARRAY_SIZE(s5kjn1_supported_modes))
		return -EINVAL;

	if (fse->code != s5kjn1_get_format_code(s5kjn1))
		return -EINVAL;

	fse->min_width = s5kjn1_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5kjn1_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int s5kjn1_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct s5kjn1 *s5kjn1 = to_s5kjn1(sd);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = s5kjn1->mode->width;
		sel->r.height = s5kjn1->mode->width;
		return 0;
	default:
		return -EINVAL;
	}

	return 0;
}

static int s5kjn1_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct s5kjn1 *s5kjn1 = to_s5kjn1(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			/* Media bus code depends on current flip controls */
			.width = s5kjn1->mode->width,
			.height = s5kjn1->mode->height,
		},
	};

	s5kjn1_set_pad_format(sd, state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops s5kjn1_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5kjn1_pad_ops = {
	.set_fmt = s5kjn1_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = s5kjn1_get_selection,
	.enum_mbus_code = s5kjn1_enum_mbus_code,
	.enum_frame_size = s5kjn1_enum_frame_size,
	.enable_streams = s5kjn1_enable_streams,
	.disable_streams = s5kjn1_disable_streams,
};

static const struct v4l2_subdev_ops s5kjn1_subdev_ops = {
	.video = &s5kjn1_video_ops,
	.pad = &s5kjn1_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5kjn1_internal_ops = {
	.init_state = s5kjn1_init_state,
};

static const struct media_entity_operations s5kjn1_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int s5kjn1_identify_sensor(struct s5kjn1 *s5kjn1)
{
	u64 val;
	int ret;

	ret = cci_read(s5kjn1->regmap, S5KJN1_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_err(s5kjn1->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (val != S5KJN1_CHIP_ID) {
		dev_err(s5kjn1->dev, "chip id mismatch: %x!=%llx\n",
			S5KJN1_CHIP_ID, val);
		return -ENODEV;
	}

	return 0;
}

static int s5kjn1_check_hwcfg(struct s5kjn1 *s5kjn1)
{
	struct fwnode_handle *fwnode = dev_fwnode(s5kjn1->dev), *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus = {
			.mipi_csi2 = {
				.num_data_lanes = S5KJN1_DATA_LANES,
			},
		},
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	unsigned long freq_bitmap;
	int ret;

	if (!fwnode)
		return -ENODEV;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != S5KJN1_DATA_LANES) {
		dev_err(s5kjn1->dev, "Invalid number of data lanes: %u\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto endpoint_free;
	}

	ret = v4l2_link_freq_to_bitmap(s5kjn1->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       s5kjn1_link_freq_menu,
				       ARRAY_SIZE(s5kjn1_link_freq_menu),
				       &freq_bitmap);

endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int s5kjn1_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5kjn1 *s5kjn1 = to_s5kjn1(sd);
	int ret;

	if (s5kjn1->vddd) {
		ret = regulator_enable(s5kjn1->vddd);
		if (ret)
			return ret;

		usleep_range(USEC_PER_MSEC, 2 * USEC_PER_MSEC);
	}

	if (s5kjn1->vdda) {
		ret = regulator_enable(s5kjn1->vdda);
		if (ret)
			goto disable_vddd;
	}

	if (s5kjn1->vddio) {
		ret = regulator_enable(s5kjn1->vddio);
		if (ret)
			goto disable_vdda;
	}

	if (s5kjn1->afvdd) {
		ret = regulator_enable(s5kjn1->afvdd);
		if (ret)
			goto disable_vddio;
	}

	ret = clk_prepare_enable(s5kjn1->mclk);
	if (ret)
		goto disable_regulators;

	gpiod_set_value_cansleep(s5kjn1->reset_gpio, 0);
	usleep_range(10 * USEC_PER_MSEC, 15 * USEC_PER_MSEC);

	return 0;

disable_regulators:
	if (s5kjn1->afvdd)
		regulator_disable(s5kjn1->afvdd);

disable_vddio:
	if (s5kjn1->vddio)
		regulator_disable(s5kjn1->vddio);

disable_vdda:
	if (s5kjn1->vdda)
		regulator_disable(s5kjn1->vdda);

disable_vddd:
	if (s5kjn1->vddd) {
		usleep_range(USEC_PER_MSEC, 2 * USEC_PER_MSEC);
		regulator_disable(s5kjn1->vddd);
	}

	return ret;
}

static int s5kjn1_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5kjn1 *s5kjn1 = to_s5kjn1(sd);

	gpiod_set_value_cansleep(s5kjn1->reset_gpio, 1);

	clk_disable_unprepare(s5kjn1->mclk);

	if (s5kjn1->afvdd)
		regulator_disable(s5kjn1->afvdd);

	if (s5kjn1->vddio)
		regulator_disable(s5kjn1->vddio);

	if (s5kjn1->vdda)
		regulator_disable(s5kjn1->vdda);

	if (s5kjn1->vddd) {
		usleep_range(USEC_PER_MSEC, 2 * USEC_PER_MSEC);
		regulator_disable(s5kjn1->vddd);
	}

	return 0;
}

static int s5kjn1_probe(struct i2c_client *client)
{
	struct s5kjn1 *s5kjn1;
	unsigned long freq;
	int ret;

	s5kjn1 = devm_kzalloc(&client->dev, sizeof(*s5kjn1), GFP_KERNEL);
	if (!s5kjn1)
		return -ENOMEM;

	s5kjn1->dev = &client->dev;
	v4l2_i2c_subdev_init(&s5kjn1->sd, client, &s5kjn1_subdev_ops);

	s5kjn1->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5kjn1->regmap))
		return dev_err_probe(s5kjn1->dev, PTR_ERR(s5kjn1->regmap),
				     "failed to init CCI\n");

	s5kjn1->mclk = devm_v4l2_sensor_clk_get(s5kjn1->dev, NULL);
	if (IS_ERR(s5kjn1->mclk))
		return dev_err_probe(s5kjn1->dev, PTR_ERR(s5kjn1->mclk),
				     "failed to get MCLK clock\n");

	freq = clk_get_rate(s5kjn1->mclk);
	if (freq != S5KJN1_MCLK_FREQ_24MHZ)
		return dev_err_probe(s5kjn1->dev, -EINVAL,
				     "MCLK clock frequency %lu is not supported\n",
				     freq);

	ret = s5kjn1_check_hwcfg(s5kjn1);
	if (ret)
		return dev_err_probe(s5kjn1->dev, ret,
				     "failed to check HW configuration\n");

	s5kjn1->reset_gpio = devm_gpiod_get_optional(s5kjn1->dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(s5kjn1->reset_gpio))
		return dev_err_probe(s5kjn1->dev, PTR_ERR(s5kjn1->reset_gpio),
				     "cannot get reset GPIO\n");

	s5kjn1->afvdd = devm_regulator_get_optional(s5kjn1->dev, "afvdd");
	if (IS_ERR(s5kjn1->afvdd)) {
		ret = PTR_ERR(s5kjn1->afvdd);
		if (ret != -ENODEV) {
			return dev_err_probe(s5kjn1->dev, ret,
					"Failed to get 'afvdd' regulator\n");
		}

		s5kjn1->afvdd = NULL;
	}

	s5kjn1->vdda = devm_regulator_get_optional(s5kjn1->dev, "vdda");
	if (IS_ERR(s5kjn1->vdda)) {
		ret = PTR_ERR(s5kjn1->vdda);
		if (ret != -ENODEV) {
			return dev_err_probe(s5kjn1->dev, ret,
					"Failed to get 'vdda' regulator\n");
		}

		s5kjn1->vdda = NULL;
	}

	s5kjn1->vddd = devm_regulator_get_optional(s5kjn1->dev, "vddd");
	if (IS_ERR(s5kjn1->vddd)) {
		ret = PTR_ERR(s5kjn1->vddd);
		if (ret != -ENODEV) {
			return dev_err_probe(s5kjn1->dev, ret,
					"Failed to get 'vddd' regulator\n");
		}

		s5kjn1->vddd = NULL;
	}

	s5kjn1->vddio = devm_regulator_get_optional(s5kjn1->dev, "vddio");
	if (IS_ERR(s5kjn1->vddio)) {
		ret = PTR_ERR(s5kjn1->vddio);
		if (ret != -ENODEV) {
			return dev_err_probe(s5kjn1->dev, ret,
					"Failed to get 'vddio' regulator\n");
		}

		s5kjn1->vddio = NULL;
	}

	/* The sensor must be powered on to read the CHIP_ID register */
	ret = s5kjn1_power_on(s5kjn1->dev);
	if (ret)
		return ret;

	ret = s5kjn1_identify_sensor(s5kjn1);
	if (ret) {
		dev_err_probe(s5kjn1->dev, ret, "failed to find sensor\n");
		goto power_off;
	}

	s5kjn1->mode = &s5kjn1_supported_modes[0];
	ret = s5kjn1_init_controls(s5kjn1);
	if (ret) {
		dev_err_probe(s5kjn1->dev, ret, "failed to init controls\n");
		goto power_off;
	}

	s5kjn1->sd.state_lock = s5kjn1->ctrl_handler.lock;
	s5kjn1->sd.internal_ops = &s5kjn1_internal_ops;
	s5kjn1->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5kjn1->sd.entity.ops = &s5kjn1_subdev_entity_ops;
	s5kjn1->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5kjn1->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&s5kjn1->sd.entity, 1, &s5kjn1->pad);
	if (ret) {
		dev_err_probe(s5kjn1->dev, ret,
			      "failed to init media entity pads\n");
		goto v4l2_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&s5kjn1->sd);
	if (ret < 0) {
		dev_err_probe(s5kjn1->dev, ret,
			      "failed to init media entity pads\n");
		goto media_entity_cleanup;
	}

	pm_runtime_set_active(s5kjn1->dev);
	pm_runtime_enable(s5kjn1->dev);

	ret = v4l2_async_register_subdev_sensor(&s5kjn1->sd);
	if (ret < 0) {
		dev_err_probe(s5kjn1->dev, ret,
			      "failed to register V4L2 subdev\n");
		goto subdev_cleanup;
	}

	pm_runtime_set_autosuspend_delay(s5kjn1->dev, 1000);
	pm_runtime_use_autosuspend(s5kjn1->dev);
	pm_runtime_idle(s5kjn1->dev);

	{
		struct dentry *dir = debugfs_create_dir("s5kjn1", NULL);

		debugfs_create_file("stream", 0600, dir, s5kjn1,
				    &s5kjn1_dbg_stream_fops);
	}

	return 0;

subdev_cleanup:
	v4l2_subdev_cleanup(&s5kjn1->sd);
	pm_runtime_disable(s5kjn1->dev);
	pm_runtime_set_suspended(s5kjn1->dev);

media_entity_cleanup:
	media_entity_cleanup(&s5kjn1->sd.entity);

v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(s5kjn1->sd.ctrl_handler);

power_off:
	s5kjn1_power_off(s5kjn1->dev);

	return ret;
}

static void s5kjn1_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5kjn1 *s5kjn1 = to_s5kjn1(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(s5kjn1->dev);

	if (!pm_runtime_status_suspended(s5kjn1->dev)) {
		s5kjn1_power_off(s5kjn1->dev);
		pm_runtime_set_suspended(s5kjn1->dev);
	}
}

static const struct dev_pm_ops s5kjn1_pm_ops = {
	SET_RUNTIME_PM_OPS(s5kjn1_power_off, s5kjn1_power_on, NULL)
};

static const struct of_device_id s5kjn1_of_match[] = {
	{ .compatible = "samsung,s5kjn1" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5kjn1_of_match);

static struct i2c_driver s5kjn1_i2c_driver = {
	.driver = {
		.name = "s5kjn1",
		.pm = &s5kjn1_pm_ops,
		.of_match_table = s5kjn1_of_match,
	},
	.probe = s5kjn1_probe,
	.remove = s5kjn1_remove,
};

module_i2c_driver(s5kjn1_i2c_driver);

MODULE_AUTHOR("Vladimir Zapolskiy <vladimir.zapolskiy@linaro.org>");
MODULE_DESCRIPTION("Samsung S5KJN1 image sensor driver");
MODULE_LICENSE("GPL");
