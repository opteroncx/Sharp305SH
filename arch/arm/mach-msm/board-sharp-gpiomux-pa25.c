/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <mach/socinfo.h>

#define WLAN_CLK	44
#define WLAN_SET	43
#define WLAN_DATA0	42
#define WLAN_DATA1	41
#define WLAN_DATA2	40

#if 0
static struct gpiomux_setting smsc_hub_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting smsc_hub_susp_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct msm_gpiomux_config smsc_hub_configs[] = {
	{
		.gpio = 114, /* reset_n */
		.settings = {
			[GPIOMUX_ACTIVE] = &smsc_hub_act_cfg,
			[GPIOMUX_SUSPENDED] = &smsc_hub_susp_cfg,
		},
	},
	{
		.gpio = 8, /* clk_en */
		.settings = {
			[GPIOMUX_ACTIVE] = &smsc_hub_act_cfg,
			[GPIOMUX_SUSPENDED] = &smsc_hub_susp_cfg,
		},
	},
	{
		.gpio = 9, /* int_n */
		.settings = {
			[GPIOMUX_ACTIVE] = &smsc_hub_act_cfg,
			[GPIOMUX_SUSPENDED] = &smsc_hub_susp_cfg,
		},
	},
};

static struct gpiomux_setting gpio_keys_active = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting gpio_keys_suspend = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gpio_spi_act_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gpio_spi_susp_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
#endif

static struct gpiomux_setting wcnss_5wire_suspend_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv  = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting wcnss_5wire_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv  = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting wcnss_5gpio_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting wcnss_5gpio_active_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting wcnss_5wire_suspend_cfg_np = {
	.func = GPIOMUX_FUNC_1,
	.drv  = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting wcnss_5wire_active_cfg_np = {
	.func = GPIOMUX_FUNC_1,
	.drv  = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};

#if 0
static struct msm_gpiomux_config msm_keypad_configs[] __initdata = {
	{
		.gpio = 106,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_keys_active,
			[GPIOMUX_SUSPENDED] = &gpio_keys_suspend,
		},
	},
	{
		.gpio = 107,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_keys_active,
			[GPIOMUX_SUSPENDED] = &gpio_keys_suspend,
		},
	},
};

static struct gpiomux_setting lcd_rst_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting lcd_rst_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct msm_gpiomux_config msm_lcd_configs[] __initdata = {
	{
		.gpio = 25,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_rst_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_rst_sus_cfg,
		},
	}
};

static struct msm_gpiomux_config msm_blsp_configs[] __initdata = {
	{
		.gpio      = 0,		/* BLSP1 QUP1 SPI_DATA_MOSI */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_spi_act_config,
			[GPIOMUX_SUSPENDED] = &gpio_spi_susp_config,
		},
	},
	{
		.gpio      = 1,		/* BLSP1 QUP1 SPI_DATA_MISO */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_spi_act_config,
			[GPIOMUX_SUSPENDED] = &gpio_spi_susp_config,
		},
	},
};

static struct msm_gpiomux_config msm_blsp_spi_cs_config[] __initdata = {
	{
		.gpio      = 2,		/* BLSP1 QUP1 SPI_CS1 */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_spi_cs_act_config,
			[GPIOMUX_SUSPENDED] = &gpio_spi_susp_config,
		},
	},
};

static struct gpiomux_setting sd_card_det_active_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting sd_card_det_sleep_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config sd_card_det __initdata = {
	.gpio = 38,
	.settings = {
		[GPIOMUX_ACTIVE]    = &sd_card_det_active_config,
		[GPIOMUX_SUSPENDED] = &sd_card_det_sleep_config,
	},
};
#endif

static struct msm_gpiomux_config wcnss_5wire_interface[] = {
	{
		.gpio = 40,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 41,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 42,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 43,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg_np,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg_np,
		},
	},
	{
		.gpio = 44,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg_np,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg_np,
		},
	},
};

static struct msm_gpiomux_config wcnss_5gpio_interface[] = {
	{
		.gpio = 40,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5gpio_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5gpio_suspend_cfg,
		},
	},
	{
		.gpio = 41,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5gpio_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5gpio_suspend_cfg,
		},
	},
	{
		.gpio = 42,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5gpio_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5gpio_suspend_cfg,
		},
	},
	{
		.gpio = 43,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5gpio_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5gpio_suspend_cfg,
		},
	},
	{
		.gpio = 44,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5gpio_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5gpio_suspend_cfg,
		},
	},
};
#if 0
static struct gpiomux_setting gpio_suspend_config[] = {
	{
		.func = GPIOMUX_FUNC_GPIO,  /* IN-NP */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},
	{
		.func = GPIOMUX_FUNC_GPIO,  /* O-LOW */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_LOW,
	},
};

static struct gpiomux_setting cam_settings[] = {
	{
		.func = GPIOMUX_FUNC_1, /*active 1*/ /* 0 */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_1, /*suspend*/ /* 1 */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
	},

	{
		.func = GPIOMUX_FUNC_1, /*i2c suspend*/ /* 2 */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_KEEPER,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*active 0*/ /* 3 */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*suspend 0*/ /* 4 */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
	},
};


static struct msm_gpiomux_config msm_sensor_configs[] __initdata = {
	{
		.gpio = 26, /* CAM_MCLK0 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &cam_settings[1],
		},
	},
	{
		.gpio = 27, /* CAM_MCLK1 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &cam_settings[1],
		},

	},
	{
		.gpio = 29, /* CCI_I2C_SDA0 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},
	{
		.gpio = 30, /* CCI_I2C_SCL0 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},
	{
		.gpio = 37, /* CAM1_RST_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[4],
		},
	},
	{
		.gpio = 28, /* CAM2_RST_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[4],
		},
	},

};

static struct gpiomux_setting auxpcm_act_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting auxpcm_sus_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct msm_gpiomux_config msm_auxpcm_configs[] __initdata = {
	{
		.gpio = 63,
		.settings = {
			[GPIOMUX_SUSPENDED] = &auxpcm_sus_cfg,
			[GPIOMUX_ACTIVE] = &auxpcm_act_cfg,
		},
	},
	{
		.gpio = 64,
		.settings = {
			[GPIOMUX_SUSPENDED] = &auxpcm_sus_cfg,
			[GPIOMUX_ACTIVE] = &auxpcm_act_cfg,
		},
	},
};
#endif


/* suspended */
static struct gpiomux_setting sh_sus_func1_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_func1_np_4ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_func1_np_8ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_func1_pd_2ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting sh_sus_func3_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_func3_np_6ma_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_sus_gpio_np_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sh_sus_gpio_np_2ma_out_high_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};
static struct gpiomux_setting sh_sus_gpio_np_2ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting sh_sus_gpio_np_4ma_out_low_cfg = {
        .func = GPIOMUX_FUNC_GPIO,
        .drv = GPIOMUX_DRV_4MA,
        .pull = GPIOMUX_PULL_NONE,
        .dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting sh_sus_gpio_pd_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sh_sus_gpio_pu_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

/* active */
static struct gpiomux_setting sh_act_func1_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_func1_np_4ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_func1_np_8ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_func1_pd_2ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting sh_act_func3_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_func3_np_6ma_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sh_act_gpio_np_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sh_act_gpio_np_2ma_out_high_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};
static struct gpiomux_setting sh_act_gpio_np_4ma_out_high_cfg = {
        .func = GPIOMUX_FUNC_GPIO,
        .drv = GPIOMUX_DRV_4MA,
        .pull = GPIOMUX_PULL_NONE,
        .dir = GPIOMUX_OUT_HIGH,
};
static struct gpiomux_setting sh_act_gpio_pd_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sh_act_gpio_pu_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config sh_msm8926_gpio_configs[] __initdata = {
	{
		.gpio = 0,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 1,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 2,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func3_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func3_np_6ma_cfg,
		},
	},
	{
		.gpio = 3,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func3_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func3_np_6ma_cfg,
		},
	},
	{
		.gpio = 4,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 5,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 6,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func3_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func3_np_2ma_cfg,
		},
	},
	{
		.gpio = 7,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func3_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func3_np_2ma_cfg,
		},
	},
	{
		.gpio = 8,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 9,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 10,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 11,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 12,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 13,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 14,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 15,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 16,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_4ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_4ma_cfg,
		},
	},
	{
		.gpio = 17,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_pd_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_pd_2ma_cfg,
		},
	},
	{
		.gpio = 18,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_4ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_4ma_cfg,
		},
	},
	{
		.gpio = 19,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_4ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_4ma_cfg,
		},
	},
	{
		.gpio = 20,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_low_cfg,
		},
	},
	{
		.gpio = 21,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 22,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_low_cfg,
		},
	},
	{
		.gpio = 23,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 24,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
#if 0
	{
		.gpio = 25,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_high_cfg,
		},
	},
#endif
	{
		.gpio = 26,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_4ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_4ma_cfg,
		},
	},
	{
		.gpio = 27,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_8ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_8ma_cfg,
		},
	},
	{
		.gpio = 28,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_low_cfg,
		},
	},
	{
		.gpio = 29,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 30,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 31,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 32,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 33,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 34,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 35,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 36,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 37,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_low_cfg,
		},
	},
	{
		.gpio = 38,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 45,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 46,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 49,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 50,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 51,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 52,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 53,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 54,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 55,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 56,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 60,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 62,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 63,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 64,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 65,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 66,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 67,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 69,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 75,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 76,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 77,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 78,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 79,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 80,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 81,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 82,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 83,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 86,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 88,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 89,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 90,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 91,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 92,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 93,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 94,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 97,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 98,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 103,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 104,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 106,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 107,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 108,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 109,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 110,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_low_cfg,
		},
	},
	{
		.gpio = 111,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_4ma_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_4ma_out_low_cfg,
		},
	},
	{
		.gpio = 112,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 113,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 114,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 115,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_low_cfg,
		},
	},
	{
		.gpio = 116,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_low_cfg,
		},
	},
	{
		.gpio = 117,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 118,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 119,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 120,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sh_act_gpio_np_2ma_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &sh_sus_gpio_np_2ma_out_high_cfg,
		},
	},
};

void __init msm8226_init_gpiomux(void)
{
	int rc;

	rc = msm_gpiomux_init_dt();
	if (rc) {
		pr_err("%s failed %d\n", __func__, rc);
		return;
	}

#if 0
	msm_gpiomux_install(msm_keypad_configs,
			ARRAY_SIZE(msm_keypad_configs));

	msm_gpiomux_install(msm_blsp_configs,
			ARRAY_SIZE(msm_blsp_configs));
		if (machine_is_msm8226())
			msm_gpiomux_install(msm_blsp_spi_cs_config,
				ARRAY_SIZE(msm_blsp_spi_cs_config));
#endif

	msm_gpiomux_install(wcnss_5wire_interface,
				ARRAY_SIZE(wcnss_5wire_interface));

#if 0
	msm_gpiomux_install(&sd_card_det, 1);

	msm_gpiomux_install_nowrite(msm_lcd_configs,
			ARRAY_SIZE(msm_lcd_configs));

	msm_gpiomux_install(msm_sensor_configs, ARRAY_SIZE(msm_sensor_configs));

	msm_gpiomux_install(msm_auxpcm_configs,
			ARRAY_SIZE(msm_auxpcm_configs));
	if (machine_is_msm8926() && of_board_is_mtp())
		msm_gpiomux_install(smsc_hub_configs,
			ARRAY_SIZE(smsc_hub_configs));
#endif

	sh_msm_gpiomux_install(sh_msm8926_gpio_configs,
			ARRAY_SIZE(sh_msm8926_gpio_configs));
}

static void wcnss_switch_to_gpio(void)
{
	/* Switch MUX to GPIO */
	msm_gpiomux_install(wcnss_5gpio_interface,
			ARRAY_SIZE(wcnss_5gpio_interface));

	/* Ensure GPIO config */
	gpio_direction_input(WLAN_DATA2);
	gpio_direction_input(WLAN_DATA1);
	gpio_direction_input(WLAN_DATA0);
	gpio_direction_output(WLAN_SET, 0);
	gpio_direction_output(WLAN_CLK, 0);
}

static void wcnss_switch_to_5wire(void)
{
	msm_gpiomux_install(wcnss_5wire_interface,
			ARRAY_SIZE(wcnss_5wire_interface));
}

u32 wcnss_rf_read_reg(u32 rf_reg_addr)
{
	int count = 0;
	u32 rf_cmd_and_addr = 0;
	u32 rf_data_received = 0;
	u32 rf_bit = 0;

	wcnss_switch_to_gpio();

	/* Reset the signal if it is already being used. */
	gpio_set_value(WLAN_SET, 0);
	gpio_set_value(WLAN_CLK, 0);

	/* We start with cmd_set high WLAN_SET = 1. */
	gpio_set_value(WLAN_SET, 1);

	gpio_direction_output(WLAN_DATA0, 1);
	gpio_direction_output(WLAN_DATA1, 1);
	gpio_direction_output(WLAN_DATA2, 1);

	gpio_set_value(WLAN_DATA0, 0);
	gpio_set_value(WLAN_DATA1, 0);
	gpio_set_value(WLAN_DATA2, 0);

	/* Prepare command and RF register address that need to sent out.
	 * Make sure that we send only 14 bits from LSB.
	 */
	rf_cmd_and_addr  = (((WLAN_RF_READ_REG_CMD) |
		(rf_reg_addr << WLAN_RF_REG_ADDR_START_OFFSET)) &
		WLAN_RF_READ_CMD_MASK);

	for (count = 0; count < 5; count++) {
		gpio_set_value(WLAN_CLK, 0);

		rf_bit = (rf_cmd_and_addr & 0x1);
		gpio_set_value(WLAN_DATA0, rf_bit ? 1 : 0);
		rf_cmd_and_addr = (rf_cmd_and_addr >> 1);

		rf_bit = (rf_cmd_and_addr & 0x1);
		gpio_set_value(WLAN_DATA1, rf_bit ? 1 : 0);
		rf_cmd_and_addr = (rf_cmd_and_addr >> 1);

		rf_bit = (rf_cmd_and_addr & 0x1);
		gpio_set_value(WLAN_DATA2, rf_bit ? 1 : 0);
		rf_cmd_and_addr = (rf_cmd_and_addr >> 1);

		/* Send the data out WLAN_CLK = 1 */
		gpio_set_value(WLAN_CLK, 1);
	}

	/* Pull down the clock signal */
	gpio_set_value(WLAN_CLK, 0);

	/* Configure data pins to input IO pins */
	gpio_direction_input(WLAN_DATA0);
	gpio_direction_input(WLAN_DATA1);
	gpio_direction_input(WLAN_DATA2);

	for (count = 0; count < 2; count++) {
		gpio_set_value(WLAN_CLK, 1);
		gpio_set_value(WLAN_CLK, 0);
	}

	rf_bit = 0;
	for (count = 0; count < 6; count++) {
		gpio_set_value(WLAN_CLK, 1);
		gpio_set_value(WLAN_CLK, 0);

		rf_bit = gpio_get_value(WLAN_DATA0);
		rf_data_received |= (rf_bit << (count * 3 + 0));

		if (count != 5) {
			rf_bit = gpio_get_value(WLAN_DATA1);
			rf_data_received |= (rf_bit << (count * 3 + 1));

			rf_bit = gpio_get_value(WLAN_DATA2);
			rf_data_received |= (rf_bit << (count * 3 + 2));
		}
	}

	gpio_set_value(WLAN_SET, 0);
	wcnss_switch_to_5wire();

	return rf_data_received;
}
