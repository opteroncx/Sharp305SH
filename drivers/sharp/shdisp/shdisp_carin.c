/* drivers/sharp/shdisp/shdisp_carin.c  (Display Driver)
 *
 * Copyright (C) 2011-2014 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/idr.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/wakelock.h>
#include <linux/qpnp/qpnp-api.h>
#include <sharp/shdisp_kerl.h>
#include "shdisp_kerl_priv.h"
#include "shdisp_pm.h"
#include "shdisp_panel.h"
#include "shdisp_carin.h"
#include "shdisp_system.h"
#include "shdisp_bdic.h"
#include "shdisp_type.h"
#include "shdisp_dbg.h"

#include "data/shdisp_carin_data.h"



/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_CARIN_VCOM_MIN                0x0048
#define SHDISP_CARIN_VCOM_MAX                0x01C8
#define SHDISP_CARIN_ALPHA_DEFAULT           0x013F

#define SHDISP_CARIN_GAMMA_SETTING                      0xC7

#define SHDISP_CARIN_POWER_SETTING                          0xD0
#define SHDISP_CARIN_POWER_SETTING_FOR_INTERNAL_POWER       0xD3
#define SHDISP_CARIN_GAMMA_SETTING_SIZE                     30
#define SHDISP_CARIN_POWER_SETTING_SIZE                     10
#define SHDISP_CARIN_POWER_SETTING_FOR_INTERNAL_POWER_SIZE  25
#define SHDISP_CARIN_GAMMA_LEVEL_MIN                        1
#define SHDISP_CARIN_GAMMA_LEVEL_MAX                        15
#define SHDISP_CARIN_GAMMA_NEGATIVE_OFFSET                  15
#define SHDISP_CARIN_VLM1       3
#define SHDISP_CARIN_VLM2       4
#define SHDISP_CARIN_VLM3       5
#define SHDISP_CARIN_VC1        12
#define SHDISP_CARIN_VC2        13
#define SHDISP_CARIN_VC3        15
#define SHDISP_CARIN_VPL        16
#define SHDISP_CARIN_VNL        17
#define SHDISP_CARIN_SVSS_SVDD  25

#define SHDISP_CARIN_INVALID_DEVICE_CODE    (0x00)
#define SHDISP_CARIN_VALID_DEVICE_CODE      (0x01)
#define SHDISP_CARIN_INVALID_INTERFACE_ID   (0x00)


/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_carin_API_init_io(void);
static int shdisp_carin_API_exit_io(void);
static int shdisp_carin_API_power_on(int mode);
static int shdisp_carin_API_power_off(int mode);
static int shdisp_carin_API_disp_on(void);
static int shdisp_carin_API_disp_off(void);
static int shdisp_carin_API_start_display(void);
static int shdisp_carin_API_post_video_start(void);
static int shdisp_carin_API_check_flicker_param(unsigned short alpha_in, unsigned short *alpha_out);
static int shdisp_carin_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size);
static int shdisp_carin_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size);
static int shdisp_carin_API_diag_set_flicker_param(struct shdisp_diag_flicker_param alpha);
static int shdisp_carin_API_diag_get_flicker_param(struct shdisp_diag_flicker_param *alpha);
static int shdisp_carin_API_diag_get_flicker_low_param(struct shdisp_diag_flicker_param *alpha);
static int shdisp_carin_API_check_recovery(void);
static int shdisp_carin_API_diag_set_gamma_info(struct shdisp_diag_gamma_info *gamma_info);
static int shdisp_carin_API_diag_get_gamma_info(struct shdisp_diag_gamma_info *gamma_info);
static int shdisp_carin_API_diag_set_gamma(struct shdisp_diag_gamma *gamma);
static void shdisp_carin_API_dump(int type);


static int shdisp_carin_mipi_cmd_lcd_on(void);
static int shdisp_carin_mipi_cmd_lcd_off(void);
static int shdisp_carin_API_mipi_start_display(void);
static char shdisp_carin_mipi_manufacture_id(void);
static int shdisp_carin_panel_clk_ctl(int enable );
int shdisp_panel_carin_API_TestImageGen(int onoff);
int shdisp_carin_init_phy_gamma(struct shdisp_lcddr_phy_gamma_reg *phy_gamma);

static int shdisp_carin_check_mipi_error(void);
static char shdisp_carin_mipi_interface_id_setting(void);
static int shdisp_carin_mipi_checksum_and_ecc_error_count(char *out);
static int shdisp_carin_request_irq(void);
static irqreturn_t shdisp_carin_int_isr(int irq_num, void *data);
static int shdisp_carin_set_irq(int enable);
static int shdisp_carin_register_driver(void);
static void shdisp_workqueue_handler_carin(struct work_struct *work);

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
#ifndef SHDISP_NOT_SUPPORT_FLICKER
static struct shdisp_panel_param_str carin_param_str;
#endif /* SHDISP_NOT_SUPPORT_FLICKER */

#ifndef SHDISP_NOT_SUPPORT_FLICKER
static unsigned char carin_wdata[8];
static unsigned char carin_rdata[8];
#endif /* SHDISP_NOT_SUPPORT_FLICKER */

static char TIG_on_payloads[7]   = {0xDE,
     0x01, 0xff, 0x02, 0x00, 0x00, 0xFF};
static struct shdisp_dsi_cmd_desc TIG_on_cmds[] = {
    {SHDISP_DTYPE_GEN_LWRITE, sizeof(TIG_on_payloads),TIG_on_payloads, 0}
};

static char TIG_off_payloads[7]  = {0xDE,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static struct shdisp_dsi_cmd_desc TIG_off_cmds[] = {
    {SHDISP_DTYPE_GEN_LWRITE, sizeof(TIG_off_payloads),TIG_off_payloads, 0}
};

static char gammmaSettingCommonSet_payloads[31];
static char chargePumpSetting_payloads[11];
static char internalPower_payloads[26];

static unsigned char carin_device_codes[4] = {0x01, 0x22, 0x33, 0x15};

static struct shdisp_panel_operations shdisp_carin_fops = {
    shdisp_carin_API_init_io,
    shdisp_carin_API_exit_io,
    NULL,
    shdisp_carin_API_power_on,
    shdisp_carin_API_power_off,
    shdisp_carin_API_disp_on,
    shdisp_carin_API_disp_off,
    shdisp_carin_API_start_display,
    shdisp_carin_API_post_video_start,
    shdisp_carin_API_check_flicker_param,
    shdisp_carin_API_diag_write_reg,
    shdisp_carin_API_diag_read_reg,
    shdisp_carin_API_diag_set_flicker_param,
    shdisp_carin_API_diag_get_flicker_param,
    shdisp_carin_API_diag_get_flicker_low_param,
    shdisp_carin_API_check_recovery,
    shdisp_carin_API_diag_set_gamma_info,
    shdisp_carin_API_diag_get_gamma_info,
    shdisp_carin_API_diag_set_gamma,
    NULL,
    shdisp_carin_API_dump
};

static struct workqueue_struct    *shdisp_wq_carin = NULL;
static struct work_struct         shdisp_wq_carin_wk;
static int shdisp_carin_irq = 0;
static spinlock_t shdisp_carin_spin_lock;
static struct wake_lock shdisp_carin_wake_lock;
static struct platform_device *shdisp_carin_int_irq_port_pdev = NULL;
static int shdisp_carin_int_irq_port_status = SHDISP_IRQ_DISABLE;
static spinlock_t shdisp_carin_set_irq_spinlock;


/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define MIPI_DSI_COMMAND_TX(x)         (shdisp_panel_API_mipi_dsi_cmds_tx(0,x, ARRAY_SIZE(x)))
#define MIPI_DSI_COMMAND_TX_COMMIT(x)  (shdisp_panel_API_mipi_dsi_cmds_tx(1,x, ARRAY_SIZE(x)))

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_create                                                   */
/* ------------------------------------------------------------------------- */
struct shdisp_panel_operations *shdisp_carin_API_create(void)
{
    SHDISP_TRACE("\n");
    return &shdisp_carin_fops;
}

/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_init_io                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_API_init_io(void)
{
    int rc = SHDISP_RESULT_SUCCESS;
#ifndef SHDISP_NOT_SUPPORT_NO_OS
    unsigned short tmp_alpha;
#endif /* SHDISP_NOT_SUPPORT_NO_OS */
    int ret = 0;
    struct shdisp_lcddr_phy_gamma_reg* phy_gamma;

    SHDISP_TRACE("in\n");

#ifndef SHDISP_NOT_SUPPORT_NO_OS
    tmp_alpha = shdisp_api_get_alpha();
    SHDISP_DEBUG("alpha=0x%04x\n", tmp_alpha);
    power_setting_payloads[4] = ((tmp_alpha >> 8) & 0x01);
    power_setting_payloads[5] = (tmp_alpha & 0xFF);
    power_setting_payloads[6] = ((tmp_alpha >> 8) & 0x01);
    power_setting_payloads[7] = (tmp_alpha & 0xFF);
    SHDISP_DEBUG("bit[8]:VCOMDC=0x%02x bit[7:0]:VCOMDC=0x%02x\n",
        power_setting_payloads[4], power_setting_payloads[5]);
#endif /* SHDISP_NOT_SUPPORT_NO_OS */

    phy_gamma = shdisp_api_get_lcddr_phy_gamma();
    ret = shdisp_carin_init_phy_gamma(phy_gamma);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_carin_init_phy_gamma.\n");
    }

    shdisp_wq_carin = create_singlethread_workqueue("shdisp_carin_queue");
    if (!shdisp_wq_carin) {
        SHDISP_ERR("shdisp_carin_workqueue create failed.\n");
        rc = SHDISP_RESULT_FAILURE;
        goto workq_create_error;
    }
    INIT_WORK(&shdisp_wq_carin_wk, shdisp_workqueue_handler_carin);

    spin_lock_init(&shdisp_carin_spin_lock);
    spin_lock_init(&shdisp_carin_set_irq_spinlock);
    wake_lock_init(&shdisp_carin_wake_lock, WAKE_LOCK_SUSPEND, "carin_wake_lock");

    shdisp_carin_register_driver();

    rc = shdisp_carin_request_irq();
    if (rc != 0) {
        SHDISP_ERR("shdisp_carin_request_irq() failed. rc = %d\n", rc);
        rc = SHDISP_RESULT_FAILURE;
        goto request_irq_error;
    }

    SHDISP_TRACE("out\n");
    return rc;

request_irq_error:
    destroy_workqueue(shdisp_wq_carin);
    shdisp_wq_carin = NULL;

workq_create_error:

    SHDISP_TRACE("out\n");
    return rc;
}


/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_exit_io                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_API_exit_io(void)
{
    SHDISP_TRACE("in\n");

    if (shdisp_wq_carin) {
        flush_workqueue(shdisp_wq_carin);
        destroy_workqueue(shdisp_wq_carin);
        shdisp_wq_carin = NULL;
    }

    free_irq(shdisp_carin_irq, 0);

    SHDISP_TRACE("out\n");

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_power_on                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_API_power_on(int mode)
{
    SHDISP_TRACE("in\n");

    shdisp_bdic_API_LCD_power_on();
    shdisp_bdic_API_LCD_m_power_on();
    shdisp_SYS_delay_us(1000);

    shdisp_bdic_API_LCD_release_hw_reset();
    shdisp_SYS_delay_us(1000);
    shdisp_carin_panel_clk_ctl(1);
    shdisp_SYS_delay_us(3000);

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_power_off                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_API_power_off(int mode)
{
    SHDISP_TRACE("in\n");

    if(mode != SHDISP_PANEL_POWER_RECOVERY_OFF){
    }

    shdisp_carin_panel_clk_ctl(0);

    shdisp_bdic_API_LCD_m_power_off();
    shdisp_bdic_API_LCD_power_off();
    shdisp_bdic_API_LCD_set_hw_reset();

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_disp_on                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_API_disp_on(void)
{
    int ret = 0;

    SHDISP_TRACE("in\n");

    ret = shdisp_carin_mipi_cmd_lcd_on();

    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_disp_off                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_API_disp_off(void)
{
    SHDISP_TRACE("in\n");

    (void)shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_STATE_OFF);

    shdisp_carin_set_irq(SHDISP_IRQ_DISABLE);

    shdisp_carin_mipi_cmd_lcd_off();

    SHDISP_TRACE("out\n");
    return 0;
}

/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_start_display                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_API_start_display(void)
{
    SHDISP_TRACE("in\n");
    shdisp_carin_API_mipi_start_display();
    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_post_video_start                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_API_post_video_start(void)
{
    struct timespec ts, ts2;
    unsigned long long wtime = 0;

    SHDISP_TRACE("in\n");

    getnstimeofday(&ts);

    (void)shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_STATE_INIT);
    (void)shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_STATE_ON);

    getnstimeofday(&ts2);

    wtime = (ts2.tv_sec - ts.tv_sec) * 1000000;
    wtime += (ts2.tv_nsec - ts.tv_nsec) / 1000;
    SHDISP_PERFORMANCE("rest of wait=%lld, wtime=%llu\n", ((4 * WAIT_1FRAME_US) - wtime), wtime);

    if (wtime < (4 * WAIT_1FRAME_US)) {
        shdisp_SYS_delay_us((WAIT_1FRAME_US * 4) - wtime);
    }

    shdisp_carin_set_irq(SHDISP_IRQ_ENABLE);

    SHDISP_TRACE("out\n");

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_check_flicker_param                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_API_check_flicker_param(unsigned short alpha_in, unsigned short *alpha_out)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    unsigned short tmp_alpha = alpha_in;

    SHDISP_TRACE("in\n");
    if (alpha_out == NULL){
        SHDISP_ERR("<NULL_POINTER> alpha_out.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if ((tmp_alpha & 0xF000) != 0x9000) {
        *alpha_out = SHDISP_CARIN_ALPHA_DEFAULT;
        SHDISP_DEBUG("out1\n");
        return SHDISP_RESULT_SUCCESS;
    }

    tmp_alpha = tmp_alpha & 0x01FF;
    if ((tmp_alpha < SHDISP_CARIN_VCOM_MIN) || (tmp_alpha > SHDISP_CARIN_VCOM_MAX)) {
        *alpha_out = SHDISP_CARIN_ALPHA_DEFAULT;
        SHDISP_DEBUG("out2\n");
        return SHDISP_RESULT_SUCCESS;
    }

    *alpha_out = tmp_alpha;

    SHDISP_TRACE("out\n");
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_diag_write_reg                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size)
{
    int ret = 0;
    char dtype;

    SHDISP_TRACE("in\n");

    if ((addr >= 0xB0) && (addr != 0xDA) && (addr != 0xDB) && (addr != 0xDC)) {
        if (size == 0) {
            dtype = SHDISP_DTYPE_GEN_WRITE;
        } else if(size == 1) {
            dtype = SHDISP_DTYPE_GEN_WRITE1;
        } else {
            dtype = SHDISP_DTYPE_GEN_LWRITE;
        }
    }
    else {
        if (size == 0) {
            dtype = SHDISP_DTYPE_DCS_WRITE;
        } else if(size == 1) {
            dtype = SHDISP_DTYPE_DCS_WRITE1;
        } else {
            dtype = SHDISP_DTYPE_DCS_LWRITE;
        }
    }

    ret = shdisp_panel_API_mipi_diag_write_reg(dtype,addr, write_data, size);
    if(ret) {
        SHDISP_DEBUG("out1\n");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_diag_read_reg                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size)
{
    int ret = 0;
    char dtype;


    SHDISP_TRACE("in\n");

    if((addr >= 0xB0) && (addr != 0xDA) && (addr != 0xDB) && (addr != 0xDC)) {
        dtype = SHDISP_DTYPE_GEN_READ2;
    } else {
        dtype = SHDISP_DTYPE_DCS_READ;
    }


    ret = shdisp_panel_API_mipi_diag_read_reg(dtype,addr, read_data,size);

    if(ret) {
        SHDISP_DEBUG("out1\n");
        return SHDISP_RESULT_FAILURE;
    }
    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_diag_set_flicker_param                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_API_diag_set_flicker_param(struct shdisp_diag_flicker_param flicker_param)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int i;
    int ret = 0;

    SHDISP_TRACE("in\n");
    if ((flicker_param.master_alpha < SHDISP_CARIN_VCOM_MIN) || (flicker_param.master_alpha > SHDISP_CARIN_VCOM_MAX)) {
        SHDISP_ERR("<INVALID_VALUE> alpha(0x%04X).\n", flicker_param.master_alpha);
        return SHDISP_RESULT_FAILURE;
    }

    for (i=1; i<=7; i++) {
        carin_rdata[i] = 0;
    }

    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_GEN_READ2,0xD5, carin_rdata, 7);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.\n");
        return SHDISP_RESULT_FAILURE;
    }

    carin_wdata[0] = carin_rdata[0];
    carin_wdata[1] = carin_rdata[1];
    carin_wdata[2] = carin_rdata[2];
    carin_wdata[3] = (unsigned char) ((flicker_param.master_alpha >> 8) & 0x01);
    carin_wdata[4] = (unsigned char) (flicker_param.master_alpha & 0xFF);
    carin_wdata[5] = (unsigned char) ((flicker_param.master_alpha >> 8) & 0x01);
    carin_wdata[6] = (unsigned char) (flicker_param.master_alpha & 0xFF);

    ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_GEN_LWRITE,0xD5, carin_wdata, 7);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
        return SHDISP_RESULT_FAILURE;
    }

    carin_param_str.vcom_alpha = flicker_param.master_alpha;
    SHDISP_TRACE("out\n");
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_diag_get_flicker_param                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_API_diag_get_flicker_param(struct shdisp_diag_flicker_param *flicker_param)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int i;
    int ret = 0;

    SHDISP_TRACE("in\n");
    if (flicker_param == NULL){
        SHDISP_ERR("<NULL_POINTER> alpha.\n");
        return SHDISP_RESULT_FAILURE;
    }

    for (i=1; i<=7; i++) {
        carin_rdata[i] = 0;
    }

    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_GEN_READ2,0xD5, carin_rdata, 7);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.\n");
        return SHDISP_RESULT_FAILURE;
    }

    flicker_param->master_alpha = ((carin_rdata[3] & 0x01) << 8) | carin_rdata[4];
    SHDISP_TRACE("out\n");
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_diag_get_flicker_low_param                               */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_API_diag_get_flicker_low_param(struct shdisp_diag_flicker_param *flicker_param)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    SHDISP_TRACE("\n");
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_check_recovery                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_API_check_recovery(void)
{
    int ret;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_API_RECOVERY_check_restoration();

#if defined(CONFIG_ANDROID_ENGINEERING)
    if (shdisp_dbg_get_recovery_check_error() == SHDISP_DBG_RECOVERY_ERROR_DETLOW) {
        shdisp_dbg_update_recovery_check_error(SHDISP_DBG_RECOVERY_ERROR_NONE);
        SHDISP_DEBUG("force lcd det low.\n");
        ret = SHDISP_RESULT_FAILURE;
    }
#endif /* CONFIG_ANDROID_ENGINEERING */

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_RECOVERY_check_restoration.\n");
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PANEL;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_DET_LOW;
        shdisp_dbg_api_err_output(&err_code, 0);
        shdisp_dbg_set_subcode(SHDISP_DBG_SUBCODE_DET_LOW);
#endif /* SHDISP_RESET_LOG */
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_diag_set_gamma_info                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_API_diag_set_gamma_info(struct shdisp_diag_gamma_info *gamma_info)
{
    int ret = 0;
    unsigned char carin_rdata[SHDISP_CARIN_GAMMA_SETTING_SIZE];
    unsigned char carin_wdata[SHDISP_CARIN_GAMMA_SETTING_SIZE];

    SHDISP_TRACE("in\n");

    ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_GEN_LWRITE,SHDISP_CARIN_GAMMA_SETTING,
                                               gamma_info->gamma,
                                               SHDISP_CARIN_GAMMA_SETTING_SIZE);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
        return SHDISP_RESULT_FAILURE;
    }

    memset(carin_wdata, 0, SHDISP_CARIN_POWER_SETTING_SIZE);
    memset(carin_rdata, 0, SHDISP_CARIN_POWER_SETTING_SIZE);
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_GEN_READ2,SHDISP_CARIN_POWER_SETTING,
                                              carin_rdata,
                                              SHDISP_CARIN_POWER_SETTING_SIZE);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.\n");
        return SHDISP_RESULT_FAILURE;
    }

    memcpy(carin_wdata, carin_rdata, SHDISP_CARIN_POWER_SETTING_SIZE);
    carin_wdata[SHDISP_CARIN_VLM1 - 1] = gamma_info->vlm1;
    carin_wdata[SHDISP_CARIN_VLM2 - 1] = gamma_info->vlm2;
    carin_wdata[SHDISP_CARIN_VLM3 - 1] = gamma_info->vlm3;

    ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_GEN_LWRITE,SHDISP_CARIN_POWER_SETTING,
                                               carin_wdata,
                                               SHDISP_CARIN_POWER_SETTING_SIZE);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
        return SHDISP_RESULT_FAILURE;
    }

    memset(carin_wdata, 0, SHDISP_CARIN_POWER_SETTING_FOR_INTERNAL_POWER_SIZE);
    memset(carin_rdata, 0, SHDISP_CARIN_POWER_SETTING_FOR_INTERNAL_POWER_SIZE);
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_GEN_READ2,SHDISP_CARIN_POWER_SETTING_FOR_INTERNAL_POWER,
                                              carin_rdata,
                                              SHDISP_CARIN_POWER_SETTING_FOR_INTERNAL_POWER_SIZE);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.\n");
        return SHDISP_RESULT_FAILURE;
    }

    memcpy(carin_wdata, carin_rdata, SHDISP_CARIN_POWER_SETTING_FOR_INTERNAL_POWER_SIZE);
    carin_wdata[SHDISP_CARIN_VC1 - 1]       = gamma_info->vc1;
    carin_wdata[SHDISP_CARIN_VC2 - 1]       = gamma_info->vc2;
    carin_wdata[SHDISP_CARIN_VC3 - 1]       = gamma_info->vc3;
    carin_wdata[SHDISP_CARIN_VPL - 1]       = gamma_info->vpl;
    carin_wdata[SHDISP_CARIN_VNL - 1]       = gamma_info->vnl;
    carin_wdata[SHDISP_CARIN_SVSS_SVDD - 1] = gamma_info->svss_svdd;

    ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_GEN_LWRITE,SHDISP_CARIN_POWER_SETTING_FOR_INTERNAL_POWER,
                                               carin_wdata,
                                               SHDISP_CARIN_POWER_SETTING_FOR_INTERNAL_POWER_SIZE);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_diag_get_gamma_info                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_API_diag_get_gamma_info(struct shdisp_diag_gamma_info *gamma_info)
{
    int ret = 0;
    unsigned char carin_rdata[SHDISP_CARIN_GAMMA_SETTING_SIZE];

    SHDISP_TRACE("in\n");
    if (gamma_info == NULL){
        SHDISP_ERR("<NULL_POINTER> gamma_info.\n");
        return SHDISP_RESULT_FAILURE;
    }

    memset(carin_rdata, 0, SHDISP_CARIN_GAMMA_SETTING_SIZE);
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_GEN_READ2,SHDISP_CARIN_GAMMA_SETTING,
                                              carin_rdata,
                                              SHDISP_CARIN_GAMMA_SETTING_SIZE);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.\n");
        return SHDISP_RESULT_FAILURE;
    }
    memcpy(gamma_info->gamma, carin_rdata, SHDISP_CARIN_GAMMA_SETTING_SIZE);

    memset(carin_rdata, 0, SHDISP_CARIN_POWER_SETTING_SIZE);
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_GEN_READ2,SHDISP_CARIN_POWER_SETTING,
                                              carin_rdata,
                                              SHDISP_CARIN_POWER_SETTING_SIZE);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.\n");
        return SHDISP_RESULT_FAILURE;
    }
    gamma_info->vlm1 = carin_rdata[SHDISP_CARIN_VLM1 - 1];
    gamma_info->vlm2 = carin_rdata[SHDISP_CARIN_VLM2 - 1];
    gamma_info->vlm3 = carin_rdata[SHDISP_CARIN_VLM3 - 1];

    memset(carin_rdata, 0, SHDISP_CARIN_POWER_SETTING_FOR_INTERNAL_POWER_SIZE);
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_GEN_READ2,SHDISP_CARIN_POWER_SETTING_FOR_INTERNAL_POWER,
                                              carin_rdata,
                                              SHDISP_CARIN_POWER_SETTING_FOR_INTERNAL_POWER_SIZE);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.\n");
        return SHDISP_RESULT_FAILURE;
    }
    gamma_info->vc1       = carin_rdata[SHDISP_CARIN_VC1 - 1];
    gamma_info->vc2       = carin_rdata[SHDISP_CARIN_VC2 - 1];
    gamma_info->vc3       = carin_rdata[SHDISP_CARIN_VC3 - 1];
    gamma_info->vpl       = carin_rdata[SHDISP_CARIN_VPL - 1];
    gamma_info->vnl       = carin_rdata[SHDISP_CARIN_VNL - 1];
    gamma_info->svss_svdd = carin_rdata[SHDISP_CARIN_SVSS_SVDD - 1];

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_diag_set_gamma                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_API_diag_set_gamma(struct shdisp_diag_gamma *gamma)
{
    int ret = 0;
    unsigned char carin_rwdata[SHDISP_CARIN_GAMMA_SETTING_SIZE];

    SHDISP_TRACE("in\n");
    if ((gamma->level < SHDISP_CARIN_GAMMA_LEVEL_MIN) || (gamma->level > SHDISP_CARIN_GAMMA_LEVEL_MAX)) {
        SHDISP_ERR("<INVALID_VALUE> gamma->level(%d).\n", gamma->level);
        return SHDISP_RESULT_FAILURE;
    }

    memset(carin_rwdata, 0, SHDISP_CARIN_GAMMA_SETTING_SIZE);
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_GEN_READ2,SHDISP_CARIN_GAMMA_SETTING,
                                              carin_rwdata,
                                              SHDISP_CARIN_GAMMA_SETTING_SIZE);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.\n");
        return SHDISP_RESULT_FAILURE;
    }
    carin_rwdata[gamma->level - 1]                                      = gamma->gamma_p;
    carin_rwdata[gamma->level + SHDISP_CARIN_GAMMA_NEGATIVE_OFFSET - 1] = gamma->gamma_n;
    ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_GEN_LWRITE,SHDISP_CARIN_GAMMA_SETTING,
                                               carin_rwdata,
                                               SHDISP_CARIN_GAMMA_SETTING_SIZE);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_dump                                                     */
/* ------------------------------------------------------------------------- */
static void shdisp_carin_API_dump(int type)
{
#define PANEL_DUMP_RECORD(a) {a, sizeof(a)/sizeof(*a)}
    static struct {
        struct shdisp_dsi_cmd_desc * cmds;
        int                          len;
    } const dumpary[] = {
        PANEL_DUMP_RECORD(initial_setting_1_2_cmds),
        PANEL_DUMP_RECORD(timing_setting_cmds),
        PANEL_DUMP_RECORD(gamma_setting_pic_adj_cmds),
        PANEL_DUMP_RECORD(dig_gamma_setting_cmds),
        PANEL_DUMP_RECORD(pin_config_cmds),
        PANEL_DUMP_RECORD(power_setting_chargepump_cmds),
        PANEL_DUMP_RECORD(power_setting_internalpower_cmds),
        PANEL_DUMP_RECORD(power_setting_cmds),
        PANEL_DUMP_RECORD(sync_signal_setting_cmds),
    };

    char readbuf[64];
    const int reclen = sizeof(dumpary)/sizeof(*dumpary);
    int reccnt;
    int cmdcnt;
    int dumpparams = 0;
    struct shdisp_dsi_cmd_desc *cmdpos;
    struct shdisp_dsi_cmd_desc readcmd;
    char payload[2];
    char valstr[(6*64)+1];
    char *pvalstr;
    int  len;
    int res;

    SHDISP_TRACE("in\n");

    readcmd.payload = payload;
    readcmd.dtype = SHDISP_DTYPE_GEN_READ2;
    readcmd.dlen   = 2;
    readcmd.wait  = 0;
    payload[1] = 0;

    for(reccnt = 0; reccnt < reclen; reccnt++) {
        cmdpos = dumpary[reccnt].cmds;

        for(cmdcnt = 0; cmdcnt < dumpary[reccnt].len; ++cmdcnt, ++cmdpos){
            payload[0] = cmdpos->payload[0];
            memset(readbuf, 0, sizeof(readbuf));
            res = shdisp_panel_API_mipi_dsi_cmds_rx(readbuf, &readcmd, cmdpos->dlen-1);
            if (res != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("carin dump[%02d][%02d](addr=0x%02x) read failure.\n", reccnt, cmdcnt, payload[0]);
                continue;
            }
            pvalstr = valstr;
            len = sizeof(valstr)/sizeof(*valstr);
            for(dumpparams = 0; dumpparams < cmdpos->dlen-1; dumpparams++){
                pvalstr += snprintf(pvalstr, len, " 0x%02x,", readbuf[dumpparams]);
                len = (sizeof(valstr)/sizeof(*valstr)) - (valstr - pvalstr);
            }
            SHDISP_DEBUG("carin dump[%02d][%02d](addr=0x%02x) = %s\n", reccnt, cmdcnt, payload[0], valstr);

            if( memcmp(readbuf, &cmdpos->payload[1], cmdpos->dlen-1) ){
                pvalstr = valstr;
                len = sizeof(valstr)/sizeof(*valstr);
                for(dumpparams = 0; dumpparams < cmdpos->dlen-1; dumpparams++){
                    pvalstr += snprintf(pvalstr, len, " 0x%02x,", cmdpos->payload[1+dumpparams]);
                    len = (sizeof(valstr)/sizeof(*valstr)) - (valstr - pvalstr);
                }
                SHDISP_DEBUG("error cmds[%02d][%02d](addr=0x%02x) = %s", reccnt, cmdcnt, payload[0], valstr);
            }
        }
    }

    SHDISP_TRACE("out\n");
}


/* ------------------------------------------------------------------------- */
/* shdisp_carin_mipi_manufacture_id                                          */
/* ------------------------------------------------------------------------- */
static char shdisp_carin_mipi_manufacture_id(void)
{
    int res, ret;
    unsigned char rbuf[5];
    struct shdisp_dsi_cmd_desc *cmd;

    memset(rbuf, 0, sizeof(rbuf));
    cmd = devcode_read_cmds;
    res = shdisp_panel_API_mipi_dsi_cmds_rx(rbuf, cmd, sizeof(rbuf));
    if (res == SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("addr:BFh data: %02X %02X %02X %02X %02X\n", rbuf[0], rbuf[1], rbuf[2], rbuf[3], rbuf[4]);
        if (!memcmp(carin_device_codes, rbuf, sizeof(carin_device_codes))) {
            ret = SHDISP_CARIN_VALID_DEVICE_CODE;
        } else {
            ret = SHDISP_CARIN_INVALID_DEVICE_CODE;
        }
    } else {
        SHDISP_ERR("device code read failure.\n");
        ret = SHDISP_CARIN_INVALID_DEVICE_CODE;
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_carin_mipi_cmd_start_display                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_mipi_cmd_start_display(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in\n");
    /* Preparation display ON */
    ret = MIPI_DSI_COMMAND_TX_COMMIT(disp_on_exit_sleep_cmds);
    SHDISP_TRACE("out ret=%d\n", ret);
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_carin_mipi_cmd_lcd_off                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_mipi_cmd_lcd_off(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in\n");
    ret = MIPI_DSI_COMMAND_TX(disp_off_enter_sleep_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("disp_off_enter_sleep error!!\n");
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX_COMMIT(deep_standby_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("deep_stanby error!!\n");
    }
    SHDISP_TRACE("out\n");
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_carin_mipi_cmd_lcd_on                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_mipi_cmd_lcd_on(void)
{
    char device_code = SHDISP_CARIN_INVALID_DEVICE_CODE;
    int ret = SHDISP_RESULT_SUCCESS;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    SHDISP_TRACE("in\n");

    ret = MIPI_DSI_COMMAND_TX_COMMIT(initial_setting_1_1_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out1 ret=%d\n", ret);
        return ret;
    }

    device_code = shdisp_carin_mipi_manufacture_id();
#ifdef SHDISP_RESET_LOG
    if (device_code != SHDISP_CARIN_VALID_DEVICE_CODE){
        SHDISP_ERR("device_code not match!!\n");
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PANEL;
        err_code.code = SHDISP_DBG_CODE_READ_ERROR;
        err_code.subcode = SHDISP_DBG_SUBCODE_DEVCODE;
        shdisp_dbg_api_err_output(&err_code, 0);
    }
#endif /* SHDISP_RESET_LOG */
    SHDISP_DEBUG("device_code=%02X\n", device_code);
    ret = MIPI_DSI_COMMAND_TX(initial_setting_1_2_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out2 ret=%d\n", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX(timing_setting_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out3 ret=%d\n", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX(gamma_setting_pic_adj_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out4 ret=%d\n", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX(dig_gamma_setting_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out5 ret=%d\n", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX(pin_config_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out6 ret=%d\n", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX(power_setting_chargepump_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out7 ret=%d\n", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX(power_setting_internalpower_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out8 ret=%d\n", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX(power_setting_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out9 ret=%d\n", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX_COMMIT(sync_signal_setting_cmds);

    SHDISP_TRACE("out ret=%d\n", ret);
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_carin_API_mipi_start_display                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_API_mipi_start_display(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in\n");

    ret = shdisp_carin_mipi_cmd_start_display();

    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_carin_init_phy_gamma                                               */
/* ------------------------------------------------------------------------- */
int shdisp_carin_init_phy_gamma(struct shdisp_lcddr_phy_gamma_reg *phy_gamma)
{
    int ret = 0;
    int cnt;
    int checksum;

    SHDISP_TRACE("in\n");

    memcpy(gammmaSettingCommonSet_payloads, gamma_setting_pic_adj_payloads, sizeof(gammmaSettingCommonSet_payloads));
    gamma_setting_pic_adj_cmds[0].payload = gammmaSettingCommonSet_payloads;
    memcpy(chargePumpSetting_payloads, power_setting_chargepump_payloads, sizeof(chargePumpSetting_payloads));
    power_setting_chargepump_cmds[0].payload = chargePumpSetting_payloads;
    memcpy(internalPower_payloads, power_setting_internalpower_payloads, sizeof(internalPower_payloads));
    power_setting_internalpower_cmds[0].payload = internalPower_payloads;

    if(phy_gamma == NULL) {
        SHDISP_ERR("phy_gamma is NULL.\n");
        ret = -1;
    }
    else if(phy_gamma->status != SHDISP_LCDDR_GAMMA_STATUS_OK) {
        SHDISP_ERR("gammg status invalid. status=%02x\n", phy_gamma->status);
        ret = -1;
    }
    else {
        checksum = phy_gamma->status;
        for(cnt = 0; cnt < SHDISP_LCDDR_PHY_GAMMA_BUF_MAX; cnt++) {
            checksum = checksum + phy_gamma->buf[cnt];
        }
        for(cnt = 0; cnt < SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE; cnt++) {
            checksum = checksum + phy_gamma->applied_voltage[cnt];
        }
        if((checksum & 0xFFFF) != phy_gamma->chksum) {
            pr_err("%s: gammg chksum NG. chksum=%04x calc_chksum=%04x\n", __func__, phy_gamma->chksum, (checksum & 0xFFFF));
            ret = -1;
        }
        else {
            for(cnt = 0; cnt < SHDISP_CARIN_GAMMA_NEGATIVE_OFFSET; cnt++) {
                gammmaSettingCommonSet_payloads[cnt + 1]                                      = phy_gamma->buf[cnt];
                gammmaSettingCommonSet_payloads[cnt + SHDISP_CARIN_GAMMA_NEGATIVE_OFFSET + 1] = phy_gamma->buf[cnt + SHDISP_CARIN_GAMMA_NEGATIVE_OFFSET];
            }
            cnt = 0;
            chargePumpSetting_payloads[SHDISP_CARIN_VLM1]  = phy_gamma->applied_voltage[cnt++];
            chargePumpSetting_payloads[SHDISP_CARIN_VLM2]  = phy_gamma->applied_voltage[cnt++];
            chargePumpSetting_payloads[SHDISP_CARIN_VLM3]  = phy_gamma->applied_voltage[cnt++];
            internalPower_payloads[SHDISP_CARIN_VC1]       = phy_gamma->applied_voltage[cnt++];
            internalPower_payloads[SHDISP_CARIN_VC2]       = phy_gamma->applied_voltage[cnt++];
            internalPower_payloads[SHDISP_CARIN_VC3]       = phy_gamma->applied_voltage[cnt++];
            internalPower_payloads[SHDISP_CARIN_VPL]       = phy_gamma->applied_voltage[cnt++];
            internalPower_payloads[SHDISP_CARIN_VNL]       = phy_gamma->applied_voltage[cnt++];
            internalPower_payloads[SHDISP_CARIN_SVSS_SVDD] = phy_gamma->applied_voltage[cnt++];
        }
    }

    SHDISP_TRACE("out ret=%04x\n", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_carin_panel_clk_ctl                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_panel_clk_ctl(int enable)
{
    int ret=0;

    SHDISP_TRACE("in enable=%d\n", enable);
    if(enable) {
        ret = qpnp_bbclk2_control_enable(true);
    } else {
        ret = qpnp_bbclk2_control_enable(false);
    }
    SHDISP_DEBUG("out ret=%04x\n", ret);

    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> ret=%d\n", ret);
        ret = SHDISP_RESULT_FAILURE;
    }
    else {
        ret = SHDISP_RESULT_SUCCESS;
    }
    SHDISP_TRACE("out ret=%04x\n", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_carin_API_TestImageGen                                       */
/* ------------------------------------------------------------------------- */
int shdisp_panel_carin_API_TestImageGen(int onoff) /* Test Image Generator ON */
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in\n");
    if (onoff) {
        ret = MIPI_DSI_COMMAND_TX_COMMIT(TIG_on_cmds);
    }
    else {
        ret = MIPI_DSI_COMMAND_TX_COMMIT(TIG_off_cmds);
    }
    SHDISP_TRACE("out ret=%d\n", ret);
    return ret;
}

/*---------------------------------------------------------------------------*/
/*      shdisp_carin_request_irq                                             */
/*---------------------------------------------------------------------------*/
static int shdisp_carin_request_irq(void)
{
    int rc;

    SHDISP_TRACE("in\n");

    rc = devm_request_irq(&shdisp_carin_int_irq_port_pdev->dev, shdisp_carin_irq, shdisp_carin_int_isr,
                                                                IRQF_TRIGGER_RISING | IRQF_DISABLED,
                                                                "shdisp_carin", NULL);
    if (rc) {
        SHDISP_ERR("request_irq() failed. irq = 0x%x\n", shdisp_carin_irq);
    } else {
        shdisp_carin_int_irq_port_status = SHDISP_IRQ_ENABLE;
        if (shdisp_api_get_boot_disp_status() == SHDISP_MAIN_DISP_ON) {
            shdisp_carin_set_irq(SHDISP_IRQ_ENABLE);
        } else {
            shdisp_carin_set_irq(SHDISP_IRQ_DISABLE);
        }
    }

    SHDISP_TRACE("out rc = %d\n", rc);

    return rc;
}

/* ------------------------------------------------------------------------- */
/* shdisp_carin_check_mipi_error                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_check_mipi_error(void)
{
    int ret = SHDISP_RESULT_FAILURE;
    int rc;
    int retry;
    int i;
    char interface_id;
    char rbuf[3];

    SHDISP_TRACE("in\n");
    interface_id = shdisp_carin_mipi_interface_id_setting();

    if (interface_id == 0x0C) {
        retry = 3;
        for (i = 0; i < retry; i++) {
            memset(rbuf, 0, sizeof(rbuf));
            rc = shdisp_carin_mipi_checksum_and_ecc_error_count(rbuf);
            if ((rc == SHDISP_RESULT_SUCCESS) && (rbuf[0] == 0x00 && rbuf[1] == 0x00 && rbuf[2] == 0x00)) {
                ret = SHDISP_RESULT_SUCCESS;
                break;
            }
            SHDISP_DEBUG("retry=%d\n", i);
        }
        if (i == retry) {
            SHDISP_DEBUG("retry over\n");
        }
    }

    SHDISP_TRACE("out ret=%d\n", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_carin_mipi_interface_id_setting                                    */
/* ------------------------------------------------------------------------- */
static char shdisp_carin_mipi_interface_id_setting(void)
{
    int res, ret;
    unsigned char rbuf[1];
    struct shdisp_dsi_cmd_desc *cmd;

    memset(rbuf, 0, sizeof(rbuf));
    cmd = interface_id_read_cmds;
    res = shdisp_panel_API_mipi_dsi_cmds_rx(rbuf, cmd, sizeof(rbuf));
    if (res == SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("addr:B4h data: %02X\n", rbuf[0]);
        ret = rbuf[0];
    } else {
        SHDISP_ERR("interface id setting read failure.\n");
        ret = SHDISP_CARIN_INVALID_INTERFACE_ID;
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_carin_mipi_checksum_and_ecc_error_count                            */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_mipi_checksum_and_ecc_error_count(char *out)
{
    int ret;
    unsigned char rbuf[3];
    struct shdisp_dsi_cmd_desc *cmd;

    memset(rbuf, 0, sizeof(rbuf));
    cmd = read_checksum_and_ecc_error_count_cmds;
    ret = shdisp_panel_API_mipi_dsi_cmds_rx(rbuf, cmd, sizeof(rbuf));
    if (ret == SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("addr:B5h data: %02X %02X %02X\n", rbuf[0], rbuf[1], rbuf[2]);
        if (out) {
            memcpy(out, rbuf, sizeof(rbuf));
        }
    } else {
        SHDISP_ERR("read checksum and ecc error count failure.\n");
    }

    return ret;
}

/*---------------------------------------------------------------------------*/
/* shdisp_workqueue_handler_carin                                            */
/*---------------------------------------------------------------------------*/
static void shdisp_workqueue_handler_carin(struct work_struct *work)
{
    int ret;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    SHDISP_TRACE("in\n");

    shdisp_semaphore_start();
    ret = shdisp_carin_check_mipi_error();
    if (ret == SHDISP_RESULT_FAILURE) {
        SHDISP_ERR("MIPI Error\n");
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PANEL;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_ESD_MIPI;
        shdisp_dbg_api_err_output(&err_code, 0);
        shdisp_dbg_set_subcode(SHDISP_DBG_SUBCODE_ESD_MIPI);
#endif /* SHDISP_RESET_LOG */
        if (shdisp_api_do_lcd_det_recovery() != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("recovery request error!!\n");
        }
    } else {
        shdisp_carin_set_irq(SHDISP_IRQ_ENABLE);
    }
    shdisp_semaphore_end(__func__);

    SHDISP_TRACE("out\n");

    wake_unlock(&shdisp_carin_wake_lock);
}

/*---------------------------------------------------------------------------*/
/* shdisp_carin_int_isr                                                      */
/*---------------------------------------------------------------------------*/
static irqreturn_t shdisp_carin_int_isr(int irq_num, void *data)
{
    unsigned long flags;
    int ret;

    SHDISP_TRACE("in\n");

    shdisp_carin_set_irq(SHDISP_IRQ_DISABLE);

    spin_lock_irqsave(&shdisp_carin_spin_lock, flags);
    if (shdisp_wq_carin) {
        wake_lock(&shdisp_carin_wake_lock);
        ret = queue_work(shdisp_wq_carin, &shdisp_wq_carin_wk);
        if (ret == 0) {
            wake_unlock(&shdisp_carin_wake_lock);
            SHDISP_DEBUG("queue_work failed.\n");
        }
    }
    spin_unlock_irqrestore(&shdisp_carin_spin_lock, flags);

    SHDISP_TRACE("out\n");

    return IRQ_HANDLED;
}

/*---------------------------------------------------------------------------*/
/*      shdisp_carin_set_irq                                                 */
/*---------------------------------------------------------------------------*/
static int shdisp_carin_set_irq(int enable)
{
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned long flags = 0;
    SHDISP_TRACE("in enable=%d.\n", enable);

    spin_lock_irqsave(&shdisp_carin_set_irq_spinlock, flags);

    if (enable == shdisp_carin_int_irq_port_status) {
        spin_unlock_irqrestore(&shdisp_carin_set_irq_spinlock, flags);
        return SHDISP_RESULT_SUCCESS;
    }

    if (enable == SHDISP_IRQ_ENABLE) {
        enable_irq(shdisp_carin_irq);
        shdisp_carin_int_irq_port_status = enable;
    } else if (enable == SHDISP_IRQ_DISABLE) {
        disable_irq_nosync(shdisp_carin_irq);
        shdisp_carin_int_irq_port_status = enable;
    } else {
        SHDISP_ERR("<INVALID_VALUE> enable=%d.\n", enable);
        ret = SHDISP_RESULT_FAILURE;
    }
    spin_unlock_irqrestore(&shdisp_carin_set_irq_spinlock, flags);
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/*      shdisp_carin_probe                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_probe(struct platform_device *pdev)
{
#ifdef CONFIG_OF
    struct resource *res;
    int rc = 0;

    SHDISP_TRACE("in pdev = 0x%p\n", pdev );

    if (pdev) {
        res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
        if (!res) {
            SHDISP_ERR("irq resouce err!!\n");
            rc = 0;
            goto probe_done;
        } else {
            shdisp_carin_irq = res->start;
            shdisp_carin_int_irq_port_pdev = pdev;
        }
    }

probe_done:
    SHDISP_TRACE("out rc = %d\n", rc);

    return rc;
#else
    return SHDISP_RESULT_SUCCESS;
#endif /* CONFIG_OF */
}


/* ------------------------------------------------------------------------- */
/*      shdisp_carin_remove                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_remove(struct platform_device *pdev)
{
    return SHDISP_RESULT_SUCCESS;
}

#ifdef CONFIG_OF
static const struct of_device_id shdisp_carin_dt_match[] = {
    { .compatible = "sharp,shdisp_carin", },
    {}
};
#else
#define shdisp_carin_dt_match NULL;
#endif /* CONFIG_OF */

static struct platform_driver shdisp_carin_driver = {
    .probe = shdisp_carin_probe,
    .remove = shdisp_carin_remove,
    .shutdown = NULL,
    .driver = {
        /*
         * Driver name must match the device name added in
         * platform.c.
         */
        .name = "shdisp_carin",
        .of_match_table = shdisp_carin_dt_match,
    },
};

/* ------------------------------------------------------------------------- */
/*      shdisp_carin_register_driver                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_carin_register_driver(void)
{
    SHDISP_TRACE("\n");
    return platform_driver_register(&shdisp_carin_driver);
}


MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
