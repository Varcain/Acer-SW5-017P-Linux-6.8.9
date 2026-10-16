// SPDX-License-Identifier: GPL-2.0
/*
 * OV5670 ATOMISP sensor driver with OVTI5670 ACPI ID support
 *
 * For Acer 2-in-1 laptops with Cherry Trail ATOMISP and OV5670 sensor.
 * PLL configured for 19.2MHz XVCLK (Cherry Trail PMC clock).
 * Register tables from mainline drivers/media/i2c/ov5670.c.
 */

#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include "../include/linux/atomisp_gmin_platform.h"

#define OV5670_NAME			"ov5670"
#define OV5670_CHIP_ID			0x005670
#define OV5670_REG_CHIP_ID		0x300A
#define OV5670_REG_STREAM_CTRL		0x0100
#define OV5670_REG_SOFTWARE_RST		0x0103

/* Default external clock frequency — Cherry Trail PMC provides 19.2MHz */
#define OV5670_XVCLK_FREQ		19200000

/* Supported link frequencies */
#define OV5670_LINK_FREQ_422MHZ		422400000ULL

/* Register value macros */
#define OV5670_REG_VALUE_08BIT		1
#define OV5670_REG_VALUE_16BIT		2
#define OV5670_REG_VALUE_24BIT		3

struct ov5670_reg {
	u16 address;
	u8 val;
};

struct ov5670_reg_list {
	u32 num_of_regs;
	const struct ov5670_reg *regs;
};

/* Mode configuration */
struct ov5670_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts_def;
	u32 vts_min;
	u32 link_freq_index;
	const struct ov5670_reg_list reg_list;
};

struct ov5670 {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;

	/* Current mode */
	const struct ov5670_mode *cur_mode;

	/* Mutex for controls */
	struct mutex mutex;

	/* Streaming state */
	bool streaming;

	/* Power state (managed via s_power from atomisp) */
	int power_count;

	/* GMIN platform data for PMIC power control */
	struct camera_sensor_platform_data *platform_data;

	/* CSI-2 metadata for ATOMISP hostdata */
	struct camera_mipi_info csi;
};

/*
 * PLL configuration for 19.2MHz XVCLK → 840 Mbps MIPI data rate
 * (422.4 MHz link frequency, 2 lanes)
 * From mainline drivers/media/i2c/ov5670.c
 */
static const struct ov5670_reg mipi_data_rate_840mbps[] = {
	{0x0300, 0x04},
	{0x0301, 0x00},
	{0x0302, 0x84},	/* PLL multiplier = 132 (for 19.2MHz XVCLK) */
	{0x0303, 0x00},
	{0x0304, 0x03},
	{0x0305, 0x01},
	{0x0306, 0x01},
	{0x030a, 0x00},
	{0x030b, 0x00},
	{0x030c, 0x00},
	{0x030d, 0x26},
	{0x030e, 0x00},
	{0x030f, 0x06},
	{0x0312, 0x01},
	{0x3018, 0x32},	/* MIPI 2-lane mode */
	{0x3031, 0x0a},
};

static const struct ov5670_reg_list mipi_data_rate_840mbps_list = {
	.num_of_regs = ARRAY_SIZE(mipi_data_rate_840mbps),
	.regs = mipi_data_rate_840mbps,
};

/*
 * 1936x1456 mode registers (full readout, no binning, ISP outputs 1920x1440)
 * Based on mainline 2592x1944 mode with adjusted output window
 */
static const struct ov5670_reg mode_1936x1456_regs[] = {
	{0x3000, 0x00},
	{0x3002, 0x21},
	{0x3005, 0xf0},
	{0x3007, 0x00},
	{0x3015, 0x0f},
	{0x301a, 0xf0},
	{0x301b, 0xf0},
	{0x301c, 0xf0},
	{0x301d, 0xf0},
	{0x301e, 0xf0},
	{0x3030, 0x00},
	{0x3031, 0x0a},
	{0x303c, 0xff},
	{0x303e, 0xff},
	{0x3040, 0xf0},
	{0x3041, 0x00},
	{0x3042, 0xf0},
	{0x3106, 0x11},
	{0x3500, 0x00},
	{0x3501, 0x80},
	{0x3502, 0x00},
	{0x3503, 0x04},
	{0x3504, 0x03},
	{0x3505, 0x83},
	{0x3508, 0x04},
	{0x3509, 0x00},
	{0x350e, 0x04},
	{0x350f, 0x00},
	{0x3510, 0x00},
	{0x3511, 0x02},
	{0x3512, 0x00},
	{0x3601, 0xc8},
	{0x3610, 0x88},
	{0x3612, 0x48},
	{0x3614, 0x5b},
	{0x3615, 0x96},
	{0x3621, 0xd0},
	{0x3622, 0x00},
	{0x3623, 0x00},
	{0x3633, 0x13},
	{0x3634, 0x13},
	{0x3635, 0x13},
	{0x3636, 0x13},
	{0x3645, 0x13},
	{0x3646, 0x82},
	{0x3650, 0x00},
	{0x3652, 0xff},
	{0x3655, 0x20},
	{0x3656, 0xff},
	{0x365a, 0xff},
	{0x365e, 0xff},
	{0x3668, 0x00},
	{0x366a, 0x07},
	{0x366e, 0x10},
	{0x366d, 0x00},
	{0x366f, 0x80},
	{0x3700, 0x28},
	{0x3701, 0x10},
	{0x3702, 0x3a},
	{0x3703, 0x19},
	{0x3704, 0x10},
	{0x3705, 0x00},
	{0x3706, 0x66},
	{0x3707, 0x08},
	{0x3708, 0x34},
	{0x3709, 0x40},
	{0x370a, 0x01},
	{0x370b, 0x1b},
	{0x3714, 0x24},
	{0x371a, 0x3e},
	{0x3733, 0x00},
	{0x3734, 0x00},
	{0x373a, 0x05},
	{0x373b, 0x06},
	{0x373c, 0x0a},
	{0x373f, 0xa0},
	{0x3755, 0x00},
	{0x3758, 0x00},
	{0x375b, 0x0e},
	{0x3766, 0x5f},
	{0x3768, 0x00},
	{0x3769, 0x22},
	{0x3773, 0x08},
	{0x3774, 0x1f},
	{0x3776, 0x06},
	{0x37a0, 0x88},
	{0x37a1, 0x5c},
	{0x37a7, 0x88},
	{0x37a8, 0x70},
	{0x37aa, 0x88},
	{0x37ab, 0x48},
	{0x37b3, 0x66},
	{0x37c2, 0x04},
	{0x37c5, 0x00},
	{0x37c8, 0x00},
	{0x3800, 0x01},	/* X start H */
	{0x3801, 0x50},	/* X start L = 336 */
	{0x3802, 0x00},	/* Y start H */
	{0x3803, 0xf8},	/* Y start L = 248 */
	{0x3804, 0x08},	/* X end H */
	{0x3805, 0xe3},	/* X end L = 2275 */
	{0x3806, 0x06},	/* Y end H */
	{0x3807, 0xab},	/* Y end L = 1707 */
	{0x3808, 0x07},	/* Output width H */
	{0x3809, 0x90},	/* Output width L = 1936 */
	{0x380a, 0x05},	/* Output height H */
	{0x380b, 0xb0},	/* Output height L = 1456 */
	{0x380c, 0x06},	/* HTS H */
	{0x380d, 0x90},	/* HTS L = 1680 */
	{0x380e, 0x08},	/* VTS H */
	{0x380f, 0x08},	/* VTS L = 2056 */
	{0x3811, 0x04},
	{0x3813, 0x02},
	{0x3814, 0x01},	/* X subsample odd inc = 1 (no binning) */
	{0x3815, 0x01},	/* X subsample even inc = 1 */
	{0x3816, 0x00},
	{0x3817, 0x00},
	{0x3818, 0x00},
	{0x3819, 0x00},
	{0x3820, 0x84},
	{0x3821, 0x46},
	{0x3822, 0x48},
	{0x3826, 0x00},
	{0x3827, 0x08},
	{0x382a, 0x01},	/* Y subsample odd inc = 1 (no binning) */
	{0x382b, 0x01},	/* Y subsample even inc = 1 */
	{0x3830, 0x08},
	{0x3836, 0x02},
	{0x3837, 0x00},
	{0x3838, 0x10},
	{0x3841, 0xff},
	{0x3846, 0x48},
	{0x3861, 0x00},
	{0x3862, 0x04},
	{0x3863, 0x06},
	{0x3a11, 0x01},
	{0x3a12, 0x78},
	{0x3b00, 0x00},
	{0x3b02, 0x00},
	{0x3b03, 0x00},
	{0x3b04, 0x00},
	{0x3b05, 0x00},
	{0x3c00, 0x89},
	{0x3c01, 0xab},
	{0x3c02, 0x01},
	{0x3c03, 0x00},
	{0x3c04, 0x00},
	{0x3c05, 0x03},
	{0x3c06, 0x00},
	{0x3c07, 0x05},
	{0x3c0c, 0x00},
	{0x3c0d, 0x00},
	{0x3c0e, 0x00},
	{0x3c0f, 0x00},
	{0x3c40, 0x00},
	{0x3c41, 0xa3},
	{0x3c43, 0x7d},
	{0x3c45, 0xd7},
	{0x3c47, 0xfc},
	{0x3c50, 0x05},
	{0x3c52, 0xaa},
	{0x3c54, 0x71},
	{0x3c56, 0x80},
	{0x3d85, 0x17},
	{0x3f03, 0x00},
	{0x3f0a, 0x00},
	{0x3f0b, 0x00},
	{0x4001, 0x60},
	{0x4009, 0x0d},
	{0x4020, 0x00},
	{0x4021, 0x00},
	{0x4022, 0x00},
	{0x4023, 0x00},
	{0x4024, 0x00},
	{0x4025, 0x00},
	{0x4026, 0x00},
	{0x4027, 0x00},
	{0x4028, 0x00},
	{0x4029, 0x00},
	{0x402a, 0x00},
	{0x402b, 0x00},
	{0x402c, 0x00},
	{0x402d, 0x00},
	{0x402e, 0x00},
	{0x402f, 0x00},
	{0x4040, 0x00},
	{0x4041, 0x03},
	{0x4042, 0x00},
	{0x4043, 0x7A},
	{0x4044, 0x00},
	{0x4045, 0x7A},
	{0x4046, 0x00},
	{0x4047, 0x7A},
	{0x4048, 0x00},
	{0x4049, 0x7A},
	{0x4307, 0x30},
	{0x4500, 0x58},
	{0x4501, 0x04},
	{0x4502, 0x40},
	{0x4503, 0x10},
	{0x4508, 0xaa},
	{0x4509, 0xaa},
	{0x450a, 0x00},
	{0x450b, 0x00},
	{0x4600, 0x00},	/* MIPI word count for 1936px (was 0x0103 for 2592) */
	{0x4601, 0xc1},	/* 1936/10 = 193 = 0xC1 */
	{0x4700, 0xa4},
	{0x4800, 0x4c},
	{0x4816, 0x53},
	{0x481f, 0x40},
	{0x4837, 0x13},
	{0x5000, 0x56},
	{0x5001, 0x01},
	{0x5002, 0x28},
	{0x5004, 0x0c},
	{0x5006, 0x0c},
	{0x5007, 0xe0},
	{0x5008, 0x01},
	{0x5009, 0xb0},
	{0x5901, 0x00},
	{0x5a01, 0x00},
	{0x5a03, 0x00},
	{0x5a04, 0x0c},
	{0x5a05, 0xe0},
	{0x5a06, 0x09},
	{0x5a07, 0xb0},
	{0x5a08, 0x06},
	{0x5e00, 0x00},
	{0x3734, 0x40},
	{0x5b00, 0x01},
	{0x5b01, 0x10},
	{0x5b02, 0x01},
	{0x5b03, 0xdb},
	{0x3d8c, 0x71},
	{0x3d8d, 0xea},
	{0x4017, 0x08},
	{0x3618, 0x2a},
	{0x5780, 0x3e},
	{0x5781, 0x0f},
	{0x5782, 0x44},
	{0x5783, 0x02},
	{0x5784, 0x01},
	{0x5785, 0x01},
	{0x5786, 0x00},
	{0x5787, 0x04},
	{0x5788, 0x02},
	{0x5789, 0x0f},
	{0x578a, 0xfd},
	{0x578b, 0xf5},
	{0x578c, 0xf5},
	{0x578d, 0x03},
	{0x578e, 0x08},
	{0x578f, 0x0c},
	{0x5790, 0x08},
	{0x5791, 0x06},
	{0x5792, 0x00},
	{0x5793, 0x52},
	{0x5794, 0xa3},
	{0x3503, 0x00},
	{0x5045, 0x05},
	{0x4003, 0x40},
	{0x5048, 0x40},
};

/*
 * 1296x972 mode registers (2x2 binning from full 2592x1944)
 * From mainline drivers/media/i2c/ov5670.c
 */
static const struct ov5670_reg mode_1296x972_regs[] = {
	{0x3000, 0x00},
	{0x3002, 0x21},
	{0x3005, 0xf0},
	{0x3007, 0x00},
	{0x3015, 0x0f},
	{0x301a, 0xf0},
	{0x301b, 0xf0},
	{0x301c, 0xf0},
	{0x301d, 0xf0},
	{0x301e, 0xf0},
	{0x3030, 0x00},
	{0x3031, 0x0a},
	{0x303c, 0xff},
	{0x303e, 0xff},
	{0x3040, 0xf0},
	{0x3041, 0x00},
	{0x3042, 0xf0},
	{0x3106, 0x11},
	{0x3500, 0x00},
	{0x3501, 0x80},
	{0x3502, 0x00},
	{0x3503, 0x04},
	{0x3504, 0x03},
	{0x3505, 0x83},
	{0x3508, 0x07},
	{0x3509, 0x80},
	{0x350e, 0x04},
	{0x350f, 0x00},
	{0x3510, 0x00},
	{0x3511, 0x02},
	{0x3512, 0x00},
	{0x3601, 0xc8},
	{0x3610, 0x88},
	{0x3612, 0x48},
	{0x3614, 0x5b},
	{0x3615, 0x96},
	{0x3621, 0xd0},
	{0x3622, 0x00},
	{0x3623, 0x00},
	{0x3633, 0x13},
	{0x3634, 0x13},
	{0x3635, 0x13},
	{0x3636, 0x13},
	{0x3645, 0x13},
	{0x3646, 0x82},
	{0x3650, 0x00},
	{0x3652, 0xff},
	{0x3655, 0x20},
	{0x3656, 0xff},
	{0x365a, 0xff},
	{0x365e, 0xff},
	{0x3668, 0x00},
	{0x366a, 0x07},
	{0x366e, 0x08},
	{0x366d, 0x00},
	{0x366f, 0x80},
	{0x3700, 0x28},
	{0x3701, 0x10},
	{0x3702, 0x3a},
	{0x3703, 0x19},
	{0x3704, 0x10},
	{0x3705, 0x00},
	{0x3706, 0x66},
	{0x3707, 0x08},
	{0x3708, 0x34},
	{0x3709, 0x40},
	{0x370a, 0x01},
	{0x370b, 0x1b},
	{0x3714, 0x24},
	{0x371a, 0x3e},
	{0x3733, 0x00},
	{0x3734, 0x00},
	{0x373a, 0x05},
	{0x373b, 0x06},
	{0x373c, 0x0a},
	{0x373f, 0xa0},
	{0x3755, 0x00},
	{0x3758, 0x00},
	{0x375b, 0x0e},
	{0x3766, 0x5f},
	{0x3768, 0x00},
	{0x3769, 0x22},
	{0x3773, 0x08},
	{0x3774, 0x1f},
	{0x3776, 0x06},
	{0x37a0, 0x88},
	{0x37a1, 0x5c},
	{0x37a7, 0x88},
	{0x37a8, 0x70},
	{0x37aa, 0x88},
	{0x37ab, 0x48},
	{0x37b3, 0x66},
	{0x37c2, 0x04},
	{0x37c5, 0x00},
	{0x37c8, 0x00},
	{0x3800, 0x00},	/* X start H */
	{0x3801, 0x0c},	/* X start L = 12 */
	{0x3802, 0x00},	/* Y start H */
	{0x3803, 0x04},	/* Y start L = 4 */
	{0x3804, 0x0a},	/* X end H */
	{0x3805, 0x33},	/* X end L = 2611 */
	{0x3806, 0x07},	/* Y end H */
	{0x3807, 0xa3},	/* Y end L = 1955 */
	{0x3808, 0x05},	/* Output width H */
	{0x3809, 0x10},	/* Output width L = 1296 */
	{0x380a, 0x03},	/* Output height H */
	{0x380b, 0xcc},	/* Output height L = 972 */
	{0x380c, 0x06},	/* HTS H */
	{0x380d, 0x90},	/* HTS L = 1680 */
	{0x380e, 0x08},	/* VTS H */
	{0x380f, 0x08},	/* VTS L = 2056 */
	{0x3811, 0x04},
	{0x3813, 0x04},
	{0x3814, 0x03},	/* X subsample odd inc */
	{0x3815, 0x01},	/* X subsample even inc */
	{0x3816, 0x00},
	{0x3817, 0x00},
	{0x3818, 0x00},
	{0x3819, 0x00},
	{0x3820, 0x94},
	{0x3821, 0x47},
	{0x3822, 0x48},
	{0x3826, 0x00},
	{0x3827, 0x08},
	{0x382a, 0x03},
	{0x382b, 0x01},
	{0x3830, 0x08},
	{0x3836, 0x02},
	{0x3837, 0x00},
	{0x3838, 0x10},
	{0x3841, 0xff},
	{0x3846, 0x48},
	{0x3861, 0x00},
	{0x3862, 0x04},
	{0x3863, 0x06},
	{0x3a11, 0x01},
	{0x3a12, 0x78},
	{0x3b00, 0x00},
	{0x3b02, 0x00},
	{0x3b03, 0x00},
	{0x3b04, 0x00},
	{0x3b05, 0x00},
	{0x3c00, 0x89},
	{0x3c01, 0xab},
	{0x3c02, 0x01},
	{0x3c03, 0x00},
	{0x3c04, 0x00},
	{0x3c05, 0x03},
	{0x3c06, 0x00},
	{0x3c07, 0x05},
	{0x3c0c, 0x00},
	{0x3c0d, 0x00},
	{0x3c0e, 0x00},
	{0x3c0f, 0x00},
	{0x3c40, 0x00},
	{0x3c41, 0xa3},
	{0x3c43, 0x7d},
	{0x3c45, 0xd7},
	{0x3c47, 0xfc},
	{0x3c50, 0x05},
	{0x3c52, 0xaa},
	{0x3c54, 0x71},
	{0x3c56, 0x80},
	{0x3d85, 0x17},
	{0x3f03, 0x00},
	{0x3f0a, 0x00},
	{0x3f0b, 0x00},
	{0x4001, 0x60},
	{0x4009, 0x05},
	{0x4020, 0x00},
	{0x4021, 0x00},
	{0x4022, 0x00},
	{0x4023, 0x00},
	{0x4024, 0x00},
	{0x4025, 0x00},
	{0x4026, 0x00},
	{0x4027, 0x00},
	{0x4028, 0x00},
	{0x4029, 0x00},
	{0x402a, 0x00},
	{0x402b, 0x00},
	{0x402c, 0x00},
	{0x402d, 0x00},
	{0x402e, 0x00},
	{0x402f, 0x00},
	{0x4040, 0x00},
	{0x4041, 0x03},
	{0x4042, 0x00},
	{0x4043, 0x7a},
	{0x4044, 0x00},
	{0x4045, 0x7a},
	{0x4046, 0x00},
	{0x4047, 0x7a},
	{0x4048, 0x00},
	{0x4049, 0x7a},
	{0x4307, 0x30},
	{0x4500, 0x58},
	{0x4501, 0x04},
	{0x4502, 0x48},
	{0x4503, 0x10},
	{0x4508, 0x55},
	{0x4509, 0x55},
	{0x450a, 0x00},
	{0x450b, 0x00},
	{0x4600, 0x00},
	{0x4601, 0x81},
	{0x4700, 0xa4},
	{0x4800, 0x4c},
	{0x4816, 0x53},
	{0x481f, 0x40},
	{0x4837, 0x13},
	{0x5000, 0x56},
	{0x5001, 0x01},
	{0x5002, 0x28},
	{0x5004, 0x0c},
	{0x5006, 0x0c},
	{0x5007, 0xe0},
	{0x5008, 0x01},
	{0x5009, 0xb0},
	{0x5901, 0x00},
	{0x5a01, 0x00},
	{0x5a03, 0x00},
	{0x5a04, 0x0c},
	{0x5a05, 0xe0},
	{0x5a06, 0x09},
	{0x5a07, 0xb0},
	{0x5a08, 0x06},
	{0x5e00, 0x00},
	{0x3734, 0x40},
	{0x5b00, 0x01},
	{0x5b01, 0x10},
	{0x5b02, 0x01},
	{0x5b03, 0xdb},
	{0x3d8c, 0x71},
	{0x3d8d, 0xea},
	{0x4017, 0x10},
	{0x3618, 0x2a},
	{0x5780, 0x3e},
	{0x5781, 0x0f},
	{0x5782, 0x44},
	{0x5783, 0x02},
	{0x5784, 0x01},
	{0x5785, 0x01},
	{0x5786, 0x00},
	{0x5787, 0x04},
	{0x5788, 0x02},
	{0x5789, 0x0f},
	{0x578a, 0xfd},
	{0x578b, 0xf5},
	{0x578c, 0xf5},
	{0x578d, 0x03},
	{0x578e, 0x08},
	{0x578f, 0x0c},
	{0x5790, 0x08},
	{0x5791, 0x04},
	{0x5792, 0x00},
	{0x5793, 0x52},
	{0x5794, 0xa3},
	{0x3503, 0x00},
	{0x5045, 0x05},
	{0x4003, 0x40},
	{0x5048, 0x40},
};

/*
 * 640x360 mode registers (4x subsample from full 2592x1944)
 * From mainline drivers/media/i2c/ov5670.c
 */
static const struct ov5670_reg mode_640x360_regs[] = {
	{0x3000, 0x00},
	{0x3002, 0x21},
	{0x3005, 0xf0},
	{0x3007, 0x00},
	{0x3015, 0x0f},
	{0x301a, 0xf0},
	{0x301b, 0xf0},
	{0x301c, 0xf0},
	{0x301d, 0xf0},
	{0x301e, 0xf0},
	{0x3030, 0x00},
	{0x3031, 0x0a},
	{0x303c, 0xff},
	{0x303e, 0xff},
	{0x3040, 0xf0},
	{0x3041, 0x00},
	{0x3042, 0xf0},
	{0x3106, 0x11},
	{0x3500, 0x00},
	{0x3501, 0x80},
	{0x3502, 0x00},
	{0x3503, 0x04},
	{0x3504, 0x03},
	{0x3505, 0x83},
	{0x3508, 0x04},
	{0x3509, 0x00},
	{0x350e, 0x04},
	{0x350f, 0x00},
	{0x3510, 0x00},
	{0x3511, 0x02},
	{0x3512, 0x00},
	{0x3601, 0xc8},
	{0x3610, 0x88},
	{0x3612, 0x48},
	{0x3614, 0x5b},
	{0x3615, 0x96},
	{0x3621, 0xd0},
	{0x3622, 0x00},
	{0x3623, 0x04},
	{0x3633, 0x13},
	{0x3634, 0x13},
	{0x3635, 0x13},
	{0x3636, 0x13},
	{0x3645, 0x13},
	{0x3646, 0x82},
	{0x3650, 0x00},
	{0x3652, 0xff},
	{0x3655, 0x20},
	{0x3656, 0xff},
	{0x365a, 0xff},
	{0x365e, 0xff},
	{0x3668, 0x00},
	{0x366a, 0x07},
	{0x366e, 0x08},
	{0x366d, 0x00},
	{0x366f, 0x80},
	{0x3700, 0x28},
	{0x3701, 0x10},
	{0x3702, 0x3a},
	{0x3703, 0x19},
	{0x3704, 0x10},
	{0x3705, 0x00},
	{0x3706, 0x66},
	{0x3707, 0x08},
	{0x3708, 0x34},
	{0x3709, 0x40},
	{0x370a, 0x01},
	{0x370b, 0x1b},
	{0x3714, 0x24},
	{0x371a, 0x3e},
	{0x3733, 0x00},
	{0x3734, 0x00},
	{0x373a, 0x05},
	{0x373b, 0x06},
	{0x373c, 0x0a},
	{0x373f, 0xa0},
	{0x3755, 0x00},
	{0x3758, 0x00},
	{0x375b, 0x0e},
	{0x3766, 0x5f},
	{0x3768, 0x00},
	{0x3769, 0x22},
	{0x3773, 0x08},
	{0x3774, 0x1f},
	{0x3776, 0x06},
	{0x37a0, 0x88},
	{0x37a1, 0x5c},
	{0x37a7, 0x88},
	{0x37a8, 0x70},
	{0x37aa, 0x88},
	{0x37ab, 0x48},
	{0x37b3, 0x66},
	{0x37c2, 0x04},
	{0x37c5, 0x00},
	{0x37c8, 0x00},
	{0x3800, 0x00},
	{0x3801, 0x0c},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3804, 0x0a},
	{0x3805, 0x33},
	{0x3806, 0x07},
	{0x3807, 0xa3},
	{0x3808, 0x02},	/* Output width = 640 */
	{0x3809, 0x80},
	{0x380a, 0x01},	/* Output height = 360 */
	{0x380b, 0x68},
	{0x380c, 0x06},	/* HTS = 1680 */
	{0x380d, 0x90},
	{0x380e, 0x08},	/* VTS = 2056 */
	{0x380f, 0x08},
	{0x3811, 0x04},
	{0x3813, 0x02},
	{0x3814, 0x07},	/* X subsample (4x) */
	{0x3815, 0x01},
	{0x3816, 0x00},
	{0x3817, 0x00},
	{0x3818, 0x00},
	{0x3819, 0x00},
	{0x3820, 0x94},
	{0x3821, 0xc6},
	{0x3822, 0x48},
	{0x3826, 0x00},
	{0x3827, 0x08},
	{0x382a, 0x07},
	{0x382b, 0x01},
	{0x3830, 0x08},
	{0x3836, 0x02},
	{0x3837, 0x00},
	{0x3838, 0x10},
	{0x3841, 0xff},
	{0x3846, 0x48},
	{0x3861, 0x00},
	{0x3862, 0x04},
	{0x3863, 0x06},
	{0x3a11, 0x01},
	{0x3a12, 0x78},
	{0x3b00, 0x00},
	{0x3b02, 0x00},
	{0x3b03, 0x00},
	{0x3b04, 0x00},
	{0x3b05, 0x00},
	{0x3c00, 0x89},
	{0x3c01, 0xab},
	{0x3c02, 0x01},
	{0x3c03, 0x00},
	{0x3c04, 0x00},
	{0x3c05, 0x03},
	{0x3c06, 0x00},
	{0x3c07, 0x05},
	{0x3c0c, 0x00},
	{0x3c0d, 0x00},
	{0x3c0e, 0x00},
	{0x3c0f, 0x00},
	{0x3c40, 0x00},
	{0x3c41, 0xa3},
	{0x3c43, 0x7d},
	{0x3c45, 0xd7},
	{0x3c47, 0xfc},
	{0x3c50, 0x05},
	{0x3c52, 0xaa},
	{0x3c54, 0x71},
	{0x3c56, 0x80},
	{0x3d85, 0x17},
	{0x3f03, 0x00},
	{0x3f0a, 0x00},
	{0x3f0b, 0x00},
	{0x4001, 0x60},
	{0x4009, 0x05},
	{0x4020, 0x00},
	{0x4021, 0x00},
	{0x4022, 0x00},
	{0x4023, 0x00},
	{0x4024, 0x00},
	{0x4025, 0x00},
	{0x4026, 0x00},
	{0x4027, 0x00},
	{0x4028, 0x00},
	{0x4029, 0x00},
	{0x402a, 0x00},
	{0x402b, 0x00},
	{0x402c, 0x00},
	{0x402d, 0x00},
	{0x402e, 0x00},
	{0x402f, 0x00},
	{0x4040, 0x00},
	{0x4041, 0x03},
	{0x4042, 0x00},
	{0x4043, 0x7a},
	{0x4044, 0x00},
	{0x4045, 0x7a},
	{0x4046, 0x00},
	{0x4047, 0x7a},
	{0x4048, 0x00},
	{0x4049, 0x7a},
	{0x4307, 0x30},
	{0x4500, 0x58},
	{0x4501, 0x04},
	{0x4502, 0x40},
	{0x4503, 0x10},
	{0x4508, 0x55},
	{0x4509, 0x55},
	{0x450a, 0x02},
	{0x450b, 0x00},
	{0x4600, 0x00},
	{0x4601, 0x40},
	{0x4700, 0xa4},
	{0x4800, 0x4c},
	{0x4816, 0x53},
	{0x481f, 0x40},
	{0x4837, 0x13},
	{0x5000, 0x56},
	{0x5001, 0x01},
	{0x5002, 0x28},
	{0x5004, 0x0c},
	{0x5006, 0x0c},
	{0x5007, 0xe0},
	{0x5008, 0x01},
	{0x5009, 0xb0},
	{0x5901, 0x00},
	{0x5a01, 0x00},
	{0x5a03, 0x00},
	{0x5a04, 0x0c},
	{0x5a05, 0xe0},
	{0x5a06, 0x09},
	{0x5a07, 0xb0},
	{0x5a08, 0x06},
	{0x5e00, 0x00},
	{0x3734, 0x40},
	{0x5b00, 0x01},
	{0x5b01, 0x10},
	{0x5b02, 0x01},
	{0x5b03, 0xdb},
	{0x3d8c, 0x71},
	{0x3d8d, 0xea},
	{0x4017, 0x10},
	{0x3618, 0x2a},
	{0x5780, 0x3e},
	{0x5781, 0x0f},
	{0x5782, 0x44},
	{0x5783, 0x02},
	{0x5784, 0x01},
	{0x5785, 0x01},
	{0x5786, 0x00},
	{0x5787, 0x04},
	{0x5788, 0x02},
	{0x5789, 0x0f},
	{0x578a, 0xfd},
	{0x578b, 0xf5},
	{0x578c, 0xf5},
	{0x578d, 0x03},
	{0x578e, 0x08},
	{0x578f, 0x0c},
	{0x5790, 0x08},
	{0x5791, 0x06},
	{0x5792, 0x00},
	{0x5793, 0x52},
	{0x5794, 0xa3},
	{0x3503, 0x00},
	{0x5045, 0x05},
	{0x4003, 0x40},
	{0x5048, 0x40},
};

/* Modes sorted largest-first */
static const struct ov5670_mode ov5670_modes[] = {
	{
		.width = 1936,
		.height = 1456,
		.hts = 1680,
		.vts_def = 2056,
		.vts_min = 2056,
		.link_freq_index = 0,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1936x1456_regs),
			.regs = mode_1936x1456_regs,
		},
	},
	{
		.width = 1296,
		.height = 972,
		.hts = 1680,
		.vts_def = 2056,
		.vts_min = 2056,
		.link_freq_index = 0,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1296x972_regs),
			.regs = mode_1296x972_regs,
		},
	},
	{
		.width = 640,
		.height = 360,
		.hts = 1680,
		.vts_def = 2056,
		.vts_min = 2056,
		.link_freq_index = 0,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_640x360_regs),
			.regs = mode_640x360_regs,
		},
	},
};

static inline struct ov5670 *to_ov5670(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov5670, sd);
}

/* I2C register read/write functions */
static int ov5670_read_reg(struct ov5670 *ov5670, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov5670->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0 };
	int ret;
	u32 i;

	if (len > sizeof(data_buf))
		return -EINVAL;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(addr_buf);
	msgs[0].buf = addr_buf;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data_buf;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return ret < 0 ? ret : -EIO;

	/* Big-endian: MSB first in data_buf[0] */
	*val = 0;
	for (i = 0; i < len; i++)
		*val = (*val << 8) | data_buf[i];

	return 0;
}

static int ov5670_write_reg(struct ov5670 *ov5670, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov5670->sd);
	u8 buf[6];
	u32 buf_len;
	int ret;
	u32 i;

	if (len > 4)
		return -EINVAL;

	buf_len = len + 2;
	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	/* Big-endian: MSB at buf[2], LSB at buf[2+len-1] */
	for (i = 0; i < len; i++)
		buf[2 + len - 1 - i] = (val >> (i * 8)) & 0xff;

	ret = i2c_master_send(client, buf, buf_len);
	if (ret != buf_len)
		return ret < 0 ? ret : -EIO;

	return 0;
}

static int ov5670_write_regs(struct ov5670 *ov5670,
			     const struct ov5670_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov5670->sd);
	int ret;
	u32 i;

	for (i = 0; i < len; i++) {
		ret = ov5670_write_reg(ov5670, regs[i].address,
				       OV5670_REG_VALUE_08BIT, regs[i].val);
		if (ret) {
			dev_err(&client->dev,
				"Failed to write reg 0x%04x: %d\n",
				regs[i].address, ret);
			return ret;
		}
	}

	return 0;
}

static int ov5670_identify_module(struct ov5670 *ov5670)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov5670->sd);
	u32 val;
	int ret;

	ret = ov5670_read_reg(ov5670, OV5670_REG_CHIP_ID,
			      OV5670_REG_VALUE_24BIT, &val);
	if (ret)
		return ret;

	if (val != OV5670_CHIP_ID) {
		dev_err(&client->dev, "Chip ID mismatch: expected 0x%06x, got 0x%06x\n",
			OV5670_CHIP_ID, val);
		return -ENODEV;
	}

	dev_info(&client->dev, "OV5670 chip identified: 0x%06x\n", val);
	return 0;
}

/*
 * Power management — uses GMIN platform for PMIC-controlled power on
 * Cherry Trail ATOMISP systems.
 */
static int ov5670_power_on(struct v4l2_subdev *sd)
{
	struct ov5670 *ov5670 = to_ov5670(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_sensor_platform_data *pdata = ov5670->platform_data;
	int ret;

	if (!pdata) {
		dev_err(&client->dev, "no platform data for power on\n");
		return -ENODEV;
	}

	/* 1. Enable power rails */
	ret = pdata->v1p8_ctrl(sd, 1);
	if (ret) {
		dev_err(&client->dev, "v1p8 power on failed: %d\n", ret);
		return ret;
	}

	ret = pdata->v2p8_ctrl(sd, 1);
	if (ret) {
		dev_err(&client->dev, "v2p8 power on failed: %d\n", ret);
		pdata->v1p8_ctrl(sd, 0);
		return ret;
	}

	/* 2. Enable clock */
	ret = pdata->flisclk_ctrl(sd, 1);
	if (ret) {
		dev_err(&client->dev, "clock enable failed: %d\n", ret);
		pdata->v2p8_ctrl(sd, 0);
		pdata->v1p8_ctrl(sd, 0);
		return ret;
	}

	/* 3. Release reset/powerdown GPIOs */
	if (pdata->gpio0_ctrl)
		pdata->gpio0_ctrl(sd, 1);
	if (pdata->gpio1_ctrl)
		pdata->gpio1_ctrl(sd, 1);

	/* Wait for sensor to stabilize after power on (OV5670 needs ~20ms) */
	msleep(20);

	return 0;
}

static void ov5670_power_off(struct v4l2_subdev *sd)
{
	struct ov5670 *ov5670 = to_ov5670(sd);
	struct camera_sensor_platform_data *pdata = ov5670->platform_data;

	if (!pdata)
		return;

	/* Reverse of power_on */
	if (pdata->gpio1_ctrl)
		pdata->gpio1_ctrl(sd, 0);
	if (pdata->gpio0_ctrl)
		pdata->gpio0_ctrl(sd, 0);
	pdata->flisclk_ctrl(sd, 0);
	pdata->v2p8_ctrl(sd, 0);
	pdata->v1p8_ctrl(sd, 0);
}

/* V4L2 subdev operations */
static int ov5670_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov5670 *ov5670 = to_ov5670(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&ov5670->mutex);

	if (ov5670->streaming == enable) {
		mutex_unlock(&ov5670->mutex);
		return 0;
	}

	if (enable) {
		/*
		 * Ensure sensor is powered on. s_power() may have been called
		 * by atomisp at S_INPUT time, but not always (e.g. default input).
		 */
		if (ov5670->power_count == 0) {
			ret = ov5670_power_on(sd);
			if (ret) {
				dev_err(&client->dev, "Failed to power on for streaming\n");
				mutex_unlock(&ov5670->mutex);
				return ret;
			}
			ov5670->power_count = 1;
		}

		/* 1. Software reset */
		ret = ov5670_write_reg(ov5670, OV5670_REG_SOFTWARE_RST,
				       OV5670_REG_VALUE_08BIT, 0x01);
		if (ret) {
			dev_err(&client->dev, "Failed to reset sensor\n");
			mutex_unlock(&ov5670->mutex);
			return ret;
		}
		usleep_range(1000, 2000);

		/* 2. Write PLL configuration for 19.2MHz → 840Mbps */
		ret = ov5670_write_regs(ov5670,
					mipi_data_rate_840mbps_list.regs,
					mipi_data_rate_840mbps_list.num_of_regs);
		if (ret) {
			dev_err(&client->dev, "Failed to write PLL config\n");
			mutex_unlock(&ov5670->mutex);
			return ret;
		}

		/* 3. Write mode-specific registers */
		ret = ov5670_write_regs(ov5670, ov5670->cur_mode->reg_list.regs,
					ov5670->cur_mode->reg_list.num_of_regs);
		if (ret) {
			dev_err(&client->dev, "Failed to apply mode %dx%d\n",
				ov5670->cur_mode->width, ov5670->cur_mode->height);
			mutex_unlock(&ov5670->mutex);
			return ret;
		}

		dev_info(&client->dev, "Configured mode %dx%d, starting stream\n",
			 ov5670->cur_mode->width, ov5670->cur_mode->height);

		/* 4. Force manual AEC/AGC so V4L2 controls take effect */
		ov5670_write_reg(ov5670, 0x3503, OV5670_REG_VALUE_08BIT, 0x03);

		/* 5. Apply V4L2 control values (exposure, gain) */
		ov5670->streaming = true;
		ret = __v4l2_ctrl_handler_setup(&ov5670->ctrl_handler);
		if (ret) {
			dev_err(&client->dev, "Failed to apply controls\n");
			ov5670->streaming = false;
			mutex_unlock(&ov5670->mutex);
			return ret;
		}

		/* 6. Start streaming */
		ret = ov5670_write_reg(ov5670, OV5670_REG_STREAM_CTRL,
				       OV5670_REG_VALUE_08BIT, 0x01);
		if (ret) {
			dev_err(&client->dev, "Failed to start streaming\n");
			ov5670->streaming = false;
			mutex_unlock(&ov5670->mutex);
			return ret;
		}

	} else {
		ret = ov5670_write_reg(ov5670, OV5670_REG_STREAM_CTRL,
				       OV5670_REG_VALUE_08BIT, 0x00);
		ov5670->streaming = false;
	}
	mutex_unlock(&ov5670->mutex);

	return ret;
}

static int ov5670_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	return 0;
}

static int ov5670_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(ov5670_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SGRBG10_1X10)
		return -EINVAL;

	fse->min_width = ov5670_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = ov5670_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int ov5670_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct ov5670 *ov5670 = to_ov5670(sd);
	struct v4l2_mbus_framefmt *mbus_fmt = &fmt->format;

	mutex_lock(&ov5670->mutex);

	mbus_fmt->width = ov5670->cur_mode->width;
	mbus_fmt->height = ov5670->cur_mode->height;
	mbus_fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	mbus_fmt->field = V4L2_FIELD_NONE;
	mbus_fmt->colorspace = V4L2_COLORSPACE_RAW;
	mbus_fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	mbus_fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	mbus_fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	mutex_unlock(&ov5670->mutex);

	return 0;
}

static int ov5670_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct ov5670 *ov5670 = to_ov5670(sd);
	const struct ov5670_mode *mode;
	struct v4l2_mbus_framefmt *mbus_fmt = &fmt->format;
	int i;

	mutex_lock(&ov5670->mutex);

	/*
	 * Find the smallest mode that fits the request.
	 * Modes are sorted largest-first, search backwards (smallest-first).
	 * Fall back to the largest (first) mode if nothing fits.
	 */
	mode = &ov5670_modes[0];
	for (i = ARRAY_SIZE(ov5670_modes) - 1; i >= 0; i--) {
		if (ov5670_modes[i].width >= mbus_fmt->width &&
		    ov5670_modes[i].height >= mbus_fmt->height) {
			mode = &ov5670_modes[i];
			break;
		}
	}

	mbus_fmt->width = mode->width;
	mbus_fmt->height = mode->height;
	mbus_fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	mbus_fmt->field = V4L2_FIELD_NONE;
	mbus_fmt->colorspace = V4L2_COLORSPACE_RAW;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_state_get_format(sd_state, 0) = *mbus_fmt;
	} else {
		ov5670->cur_mode = mode;
	}

	mutex_unlock(&ov5670->mutex);

	return 0;
}

static int ov5670_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov5670 *ov5670 = to_ov5670(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&ov5670->mutex);

	if (on) {
		if (ov5670->power_count++ > 0)
			goto out;

		ret = ov5670_power_on(sd);
		if (ret) {
			ov5670->power_count--;
			dev_err(&client->dev, "s_power(on) failed: %d\n", ret);
		}
	} else {
		if (--ov5670->power_count > 0)
			goto out;

		ov5670->power_count = 0;
		ov5670_power_off(sd);
	}

out:
	mutex_unlock(&ov5670->mutex);
	return ret;
}

static int ov5670_get_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *sd_state,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_info(&client->dev, "get_frame_interval called\n");
	fi->interval.numerator = 1;
	fi->interval.denominator = 30;

	return 0;
}

static const struct v4l2_subdev_core_ops ov5670_core_ops = {
	.s_power = ov5670_s_power,
};

static const struct v4l2_subdev_video_ops ov5670_video_ops = {
	.s_stream = ov5670_set_stream,
};

static const struct v4l2_subdev_pad_ops ov5670_pad_ops = {
	.enum_mbus_code = ov5670_enum_mbus_code,
	.enum_frame_size = ov5670_enum_frame_size,
	.get_fmt = ov5670_get_fmt,
	.set_fmt = ov5670_set_fmt,
	.get_frame_interval = ov5670_get_frame_interval,
};

static const struct v4l2_subdev_ops ov5670_subdev_ops = {
	.core = &ov5670_core_ops,
	.video = &ov5670_video_ops,
	.pad = &ov5670_pad_ops,
};

/* Control operations */
static int ov5670_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov5670 *ov5670 = container_of(ctrl->handler,
					     struct ov5670, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&ov5670->sd);
	int ret = 0;

	/* Controls are only applied when streaming */
	if (!ov5670->streaming)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = ov5670_write_reg(ov5670, 0x3500,
				       OV5670_REG_VALUE_08BIT,
				       (ctrl->val >> 12) & 0x0f);
		ret |= ov5670_write_reg(ov5670, 0x3501,
					OV5670_REG_VALUE_08BIT,
					(ctrl->val >> 4) & 0xff);
		ret |= ov5670_write_reg(ov5670, 0x3502,
					OV5670_REG_VALUE_08BIT,
					(ctrl->val & 0x0f) << 4);
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		ret = ov5670_write_reg(ov5670, 0x3508,
				       OV5670_REG_VALUE_08BIT,
				       (ctrl->val >> 8) & 0x1f);
		ret |= ov5670_write_reg(ov5670, 0x3509,
					OV5670_REG_VALUE_08BIT,
					ctrl->val & 0xff);
		break;

	default:
		ret = -EINVAL;
		dev_warn(&client->dev, "Unhandled control: %d\n", ctrl->id);
		break;
	}

	return ret;
}

static int ov5670_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_LINK_FREQ:
		/*
		 * ATOMISP reads V4L2_CID_LINK_FREQ via v4l2_g_ctrl() which
		 * returns the integer menu INDEX (0), not the actual frequency.
		 * Override here to return the real frequency (422400000 Hz)
		 * so CSI-2 receiver timing is calculated correctly.
		 */
		ctrl->val = OV5670_LINK_FREQ_422MHZ;
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops ov5670_ctrl_ops = {
	.s_ctrl = ov5670_set_ctrl,
	.g_volatile_ctrl = ov5670_g_volatile_ctrl,
};

static int ov5670_init_controls(struct ov5670 *ov5670)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov5670->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr = &ov5670->ctrl_handler;
	int ret;

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 6);
	if (ret)
		return ret;

	/* Link frequency (read-only) */
	ov5670->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr,
						   &ov5670_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   0, 0,
						   (const s64[]){OV5670_LINK_FREQ_422MHZ});
	if (ov5670->link_freq)
		ov5670->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY |
					    V4L2_CTRL_FLAG_VOLATILE;

	/* Pixel rate (read-only) */
	ov5670->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &ov5670_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       0, 168960000, 1, 168960000);
	if (ov5670->pixel_rate)
		ov5670->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Exposure */
	ov5670->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &ov5670_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     4, 65535, 1, 2048);

	/* Analogue gain: value/128 = real gain. 128=1x, 256=2x, 512=4x */
	v4l2_ctrl_new_std(ctrl_hdlr, &ov5670_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, 128, 8191, 1, 1024);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "Control init failed: %d\n", ret);
		goto error;
	}

	ov5670->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	return ret;
}

/* Probe and remove */
static int ov5670_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ov5670 *ov5670;
	int ret;

	ov5670 = devm_kzalloc(dev, sizeof(*ov5670), GFP_KERNEL);
	if (!ov5670)
		return -ENOMEM;

	mutex_init(&ov5670->mutex);

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&ov5670->sd, client, &ov5670_subdev_ops);

	/*
	 * Get GMIN platform data for PMIC power control.
	 * This sets up the power rails, clocks, and GPIOs through the
	 * Cherry Trail GMIN platform layer.
	 */
	ov5670->platform_data = gmin_camera_platform_data(&ov5670->sd,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_grbg);
	if (!ov5670->platform_data) {
		dev_warn(dev, "No GMIN platform data, power management may not work\n");
	}

	/*
	 * Set up CSI-2 metadata as v4l2 subdev hostdata for ATOMISP.
	 * Without this, atomisp_set_sensor_mipi_to_isp() uses an
	 * incomplete fallback path, and CSI-2 configuration may fail.
	 */
	ov5670->csi.port = ATOMISP_CAMERA_PORT_SECONDARY;	/* Port 1 */
	ov5670->csi.num_lanes = 2;
	ov5670->csi.input_format = ATOMISP_INPUT_FORMAT_RAW_10;
	ov5670->csi.raw_bayer_order = atomisp_bayer_order_grbg;
	ov5670->csi.metadata_format = ATOMISP_INPUT_FORMAT_EMBEDDED;
	v4l2_set_subdev_hostdata(&ov5670->sd, &ov5670->csi);

	dev_info(dev, "OV5670 sensor matched via ACPI (skipping I2C ID check)\n");

	/* Set default mode */
	ov5670->cur_mode = &ov5670_modes[0];

	/* Initialize controls */
	ret = ov5670_init_controls(ov5670);
	if (ret) {
		dev_err(dev, "Failed to init controls: %d\n", ret);
		return ret;
	}

	/* Initialize media entity */
	ov5670->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov5670->pad.flags = MEDIA_PAD_FL_SOURCE;
	ov5670->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&ov5670->sd.entity, 1, &ov5670->pad);
	if (ret) {
		dev_err(dev, "Failed to init media entity: %d\n", ret);
		goto ctrl_handler_free;
	}

	/*
	 * Register via v4l2 async framework for atomisp discovery.
	 * The atomisp bridge code adds an async connection for our ACPI HID,
	 * and this registration triggers the match, creating media entity links.
	 */
	ret = v4l2_async_register_subdev(&ov5670->sd);
	if (ret) {
		dev_err(dev, "Failed to register async subdev: %d\n", ret);
		goto entity_cleanup;
	}

	dev_info(dev, "OV5670 camera sensor probed successfully\n");

	return 0;

entity_cleanup:
	media_entity_cleanup(&ov5670->sd.entity);
ctrl_handler_free:
	v4l2_ctrl_handler_free(&ov5670->ctrl_handler);
	return ret;
}

static void ov5670_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5670 *ov5670 = to_ov5670(sd);

	v4l2_async_unregister_subdev(&ov5670->sd);
	media_entity_cleanup(&ov5670->sd.entity);
	v4l2_ctrl_handler_free(&ov5670->ctrl_handler);

	if (ov5670->power_count > 0)
		ov5670_power_off(&ov5670->sd);

	atomisp_gmin_remove_subdev(&ov5670->sd);

	mutex_destroy(&ov5670->mutex);
}

/*
 * ACPI device ID table - includes both standard INT3479 and
 * Acer-specific OVTI5670 identifiers
 */
static const struct acpi_device_id ov5670_acpi_ids[] = {
	{ "INT3479", 0 },   /* Standard Intel ACPI ID */
	{ "OVTI5670", 0 },  /* Acer 2-in-1 and other OEM variants */
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(acpi, ov5670_acpi_ids);

static const struct i2c_device_id ov5670_id_table[] = {
	{ "ov5670", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, ov5670_id_table);

static const struct of_device_id ov5670_of_match[] = {
	{ .compatible = "ovti,ov5670" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov5670_of_match);

static struct i2c_driver ov5670_i2c_driver = {
	.driver = {
		.name = OV5670_NAME,
		/* Power managed via s_power from atomisp, no pm_runtime */
		.acpi_match_table = ACPI_PTR(ov5670_acpi_ids),
		.of_match_table = ov5670_of_match,
	},
	.probe = ov5670_probe,
	.remove = ov5670_remove,
	.id_table = ov5670_id_table,
};

module_i2c_driver(ov5670_i2c_driver);

MODULE_DESCRIPTION("OV5670 CMOS sensor driver with OVTI5670 ACPI support");
MODULE_AUTHOR("Linux Media Driver Authors");
MODULE_LICENSE("GPL");
MODULE_SOFTDEP("pre: atomisp");
