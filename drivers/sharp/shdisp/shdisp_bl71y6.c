/* drivers/sharp/shdisp/shdisp_bl71y6.c  (Display Driver)
 *
 * Copyright (C) 2013-2014 SHARP CORPORATION
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
#include <sharp/shdisp_kerl.h>
#include "shdisp_system.h"
#include "shdisp_type.h"
#include "shdisp_bdic.h"
#include "shdisp_dbg.h"

#if defined(CONFIG_MACH_PA25) || defined(CONFIG_MACH_PB25) || defined(CONFIG_MACH_KF10)
#include "./data/shdisp_bl71y6_data_pa25_pb25.h"
#elif defined(CONFIG_MACH_PA28)
#include "./data/shdisp_bl71y6_data_pa28.h"
#elif defined(CONFIG_MACH_TAC)
#include "./data/shdisp_bl71y6_data_pa27.h"
#include "shdisp_kerl_priv.h"
#else
#include "./data/shdisp_bl71y6_data_default.h"
#endif

#include "shdisp_pm.h"
#include <sharp/sh_boot_manager.h>

#include "data/shdisp_bl71y6_ctrl.h"

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#ifdef SHDISP_NOT_SUPPORT_NO_OS
 #define SHDISP_BDIC_HW_INIT
#endif /* SHDISP_NOT_SUPPORT_NO_OS */

#define SHDISP_BDIC_BKL_MODE_OFF                (0)
#define SHDISP_BDIC_BKL_MODE_FIX                (1)
#define SHDISP_BDIC_BKL_MODE_AUTO               (2)

#define SHDISP_BDIC_BKL_DBC_OFF                 (0)
#define SHDISP_BDIC_BKL_DBC_ON                  (1)
#define SHDISP_BDIC_BKL_DTV_OFF                 (0)
#define SHDISP_BDIC_BKL_DTV_ON                  (1)

#define SHDISP_BDIC_BKL_EMG_OFF                 (0)
#define SHDISP_BDIC_BKL_EMG_ON                  (1)

#define SHDISP_BDIC_BKL_ECO_OFF                 (0)
#define SHDISP_BDIC_BKL_ECO_ON                  (1)

#define SHDISP_BDIC_BKL_CHG_OFF                 (0)
#define SHDISP_BDIC_BKL_CHG_ON                  (1)

#define SHDISP_BDIC_BKL_AUTO_FIX_PARAM_MIN_ADO  (0)

#define SHDISP_BDIC_TRI_LED_MODE_OFF           (-1)
#define SHDISP_BDIC_TRI_LED_MODE_NORMAL         (0)
#define SHDISP_BDIC_TRI_LED_MODE_BLINK          (1)
#define SHDISP_BDIC_TRI_LED_MODE_FIREFLY        (2)
#define SHDISP_BDIC_TRI_LED_MODE_HISPEED        (3)
#define SHDISP_BDIC_TRI_LED_MODE_STANDARD       (4)
#define SHDISP_BDIC_TRI_LED_MODE_BREATH         (5)
#define SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH    (6)
#define SHDISP_BDIC_TRI_LED_MODE_WAVE           (7)
#define SHDISP_BDIC_TRI_LED_MODE_FLASH          (8)
#define SHDISP_BDIC_TRI_LED_MODE_AURORA         (9)
#define SHDISP_BDIC_TRI_LED_MODE_RAINBOW       (10)

#define SHDISP_BDIC_GET_ADO_RETRY_TIMES         (3)
#define SHDISP_BDIC_MAX_ADO_VALUE          (0xFFFF)
#define SHDISP_BDIC_RATIO_OF_ALS0            (64/4)
#define SHDISP_BDIC_RATIO_OF_ALS1            (60/4)
#define SHDISP_BDIC_AB_MAG_X1              (0x8000)

#define SHDISP_BDIC_LUX_TABLE_ARRAY_SIZE    (ARRAY_SIZE(shdisp_bdic_bkl_ado_tbl))
#define SHDISP_BDIC_LUX_DIVIDE_COFF         (100)

#define SHDISP_BDIC_REGSET(x)               (shdisp_bdic_seq_regset(x, ARRAY_SIZE(x)))
#define SHDISP_BDIC_BACKUP_REGS_BDIC(x)     (shdisp_bdic_seq_backup_bdic_regs(x, ARRAY_SIZE(x)))
#define SHDISP_BDIC_BACKUP_REGS_ALS(x)      (shdisp_bdic_seq_backup_als_regs(x, ARRAY_SIZE(x)))
#define SHDISP_BDIC_RESTORE_REGS(x)         (shdisp_bdic_seq_regset(x, ARRAY_SIZE(x)))

#define SHDISP_BDIC_AVE_ADO_READ_TIMES          (5)
/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

#ifdef SHDISP_BDIC_HW_INIT
static int  shdisp_bdic_LD_version_check(unsigned char version);
static int  shdisp_bdic_LD_hw_init(void);
#endif /* SHDISP_BDIC_HW_INIT */
static void shdisp_bdic_LD_GPIO_control(unsigned char symbol, unsigned char status);
static void shdisp_bdic_seq_backlight_off(void);
static void shdisp_bdic_seq_backlight_fix_on(int param);
static void shdisp_bdic_seq_backlight_auto_on(int param);
static int  shdisp_bdic_seq_led_off(void);
static int  shdisp_bdic_seq_led_normal_on(unsigned char color);
static void shdisp_bdic_seq_led_blink_on(unsigned char color, int ontime, int interval, int count);
static void shdisp_bdic_seq_led_firefly_on(unsigned char color, int ontime, int interval, int count);
static void shdisp_bdic_LD_set_led_fix_on_table(int clr_vari, int color);
#if defined(CONFIG_MACH_TAC)
static void shdisp_bdic_seq_key_led_ctl(unsigned char dim, unsigned char index, int ontime, int interval);
#endif
static int  shdisp_bdic_LD_PHOTO_SENSOR_get_lux(unsigned short *value, unsigned long *lux);
static int  shdisp_bdic_LD_PHOTO_SENSOR_get_raw_als(unsigned short *clear, unsigned short *ir);
static void shdisp_bdic_seq_psals_active(unsigned long dev_type);
static void shdisp_bdic_seq_psals_standby(unsigned long dev_type);
static void shdisp_bdic_LD_LCD_BKL_dtv_on(void);
static void shdisp_bdic_LD_LCD_BKL_dtv_off(void);
static void shdisp_bdic_LD_LCD_BKL_emg_on(void);
static void shdisp_bdic_LD_LCD_BKL_emg_off(void);
static void shdisp_bdic_LD_LCD_BKL_get_mode(int *mode);
static void shdisp_bdic_LD_LCD_BKL_get_fix_param(int mode, int level, unsigned char *value);
static void shdisp_bdic_LD_LCD_BKL_get_pwm_param(int mode, int level, unsigned char *opt_val);
static void shdisp_bdic_LD_LCD_BKL_eco_on(void);
static void shdisp_bdic_LD_LCD_BKL_eco_off(void);
static void shdisp_bdic_LD_LCD_BKL_chg_on(void);
static void shdisp_bdic_LD_LCD_BKL_chg_off(void);

#ifdef SHDISP_BDIC_HW_INIT
static void shdisp_bdic_PD_hw_reset(void);
#endif /* SHDISP_BDIC_HW_INIT */
static void shdisp_bdic_PD_LCD_POS_PWR_on(void);
static void shdisp_bdic_PD_LCD_POS_PWR_off(void);
static void shdisp_bdic_PD_LCD_NEG_PWR_on(void);
static void shdisp_bdic_PD_LCD_NEG_PWR_off(void);
static int  shdisp_bdic_PD_slave_transfer(struct shdisp_bdic_i2c_msg *msg);
static void shdisp_bdic_seq_bdic_active_for_led(int);
static void shdisp_bdic_seq_bdic_standby_for_led(int);
static int  shdisp_bdic_seq_regset(const shdisp_bdicRegSetting_t* regtable, int size);
static int  shdisp_bdic_seq_backup_bdic_regs(shdisp_bdicRegSetting_t *regs, int size);
static int  shdisp_bdic_seq_backup_als_regs(shdisp_bdicRegSetting_t *regs, int size);
static int  shdisp_bdic_PD_set_active(int power_status);
static int  shdisp_bdic_PD_set_standby(void);
static void shdisp_bdic_PD_BKL_control(unsigned char request, int param);
static void shdisp_bdic_PD_GPIO_control(unsigned char port, unsigned char status);
static unsigned char shdisp_bdic_PD_opt_th_shift(int index);
static void shdisp_bdic_PD_BKL_set_led_value(void);
static void shdisp_bdic_PD_BKL_set_opt_value(void);
static void shdisp_bdic_PD_TRI_LED_control(unsigned char request, int param);
static void shdisp_bdic_PD_TRI_LED_set_anime(void);
static void shdisp_bdic_PD_TRI_LED_set_chdig(void);
#if defined(CONFIG_MACH_TAC)
static void shdisp_bdic_PD_KEY_LED_control(unsigned char dim, unsigned char index, int ontime, int interval);
static void shdisp_bdic_PD_KEY_LED_set_chdig(unsigned char dim, unsigned char index);
#endif
static int shdisp_bdic_PD_get_sensor_state(void);
static int shdisp_bdic_PD_wait4i2ctimer_stop(void);
static int  shdisp_bdic_PD_psals_power_on(void);
static int  shdisp_bdic_PD_psals_power_off(void);
static int  shdisp_bdic_PD_psals_ps_init_als_off(void);
static int  shdisp_bdic_PD_psals_ps_init_als_on(void);
static int  shdisp_bdic_PD_psals_ps_deinit_als_off(void);
static int  shdisp_bdic_PD_psals_ps_deinit_als_on(void);
static int  shdisp_bdic_PD_psals_als_init_ps_off(void);
static int  shdisp_bdic_PD_psals_als_init_ps_on(void);
static int  shdisp_bdic_PD_psals_als_deinit_ps_off(void);
static int  shdisp_bdic_PD_psals_als_deinit_ps_on(void);
static int  shdisp_bdic_PD_get_ave_ado(struct shdisp_ave_ado *ave_ado);

static int  shdisp_bdic_PD_REG_ADO_get_opt(unsigned short *value);
static void shdisp_bdic_PD_REG_RAW_DATA_get_opt(unsigned short *clear, unsigned short *ir);

static int shdisp_bdic_IO_write_reg(unsigned char reg, unsigned char val);
static int shdisp_bdic_IO_multi_write_reg(unsigned char reg, unsigned char *wval, unsigned char size);
static int shdisp_bdic_IO_read_reg(unsigned char reg, unsigned char *val);
static int shdisp_bdic_IO_read_no_check_reg(unsigned char reg, unsigned char *val);
static int shdisp_bdic_IO_read_check_reg(unsigned char reg, unsigned char *val);
static int shdisp_bdic_IO_multi_read_reg(unsigned char reg, unsigned char *val, int size);
static int shdisp_bdic_IO_set_bit_reg(unsigned char reg, unsigned char val);
static int shdisp_bdic_IO_clr_bit_reg(unsigned char reg, unsigned char val);
static int shdisp_bdic_IO_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char msk);
static int shdisp_bdic_IO_bank_set(unsigned char val);
static int shdisp_photo_sensor_IO_write_reg(unsigned char reg, unsigned char val);
static int shdisp_phote_sensor_IO_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char mask);
static int shdisp_photo_sensor_IO_read_reg(unsigned char reg, unsigned char *val);
static int shdisp_photo_sensor_IO_burst_write_reg(unsigned char *wval, unsigned char dataNum);
static int shdisp_photo_sensor_IO_burst_read_reg(unsigned char reg, unsigned char *rval, unsigned char dataNum);

static int shdisp_bdic_PD_psals_write_threshold(struct shdisp_prox_params *prox_params);
static int shdisp_bdic_register_driver(void);

static int shdisp_bdic_PD_i2c_throughmode_ctrl(bool ctrl);

static void shdisp_bdic_set_default_sensor_param(struct shdisp_photo_sensor_adj *tmp_adj);

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static struct shdisp_bdic_state_str s_state_str;

static int shdisp_bdic_bkl_mode;
static int shdisp_bdic_tri_led_before_mode;
static int shdisp_bdic_bkl_param;
static int shdisp_bdic_bkl_param_auto;
static int shdisp_bdic_bkl_before_mode;

static int shdisp_bdic_dbc_mode;
static int shdisp_bdic_dbc_before_mode;

static int shdisp_bdic_dtv;

static int shdisp_bdic_emg;

static int shdisp_bdic_eco;

static int shdisp_bdic_chg;

static unsigned char shdisp_bdic_tri_led_color;
static int shdisp_bdic_tri_led_mode;
static int shdisp_bdic_tri_led_ontime;
static int shdisp_bdic_tri_led_interval;
static int shdisp_bdic_tri_led_count;

#if defined(CONFIG_MACH_TAC)
static unsigned char shdisp_bdic_key_led_index;
static unsigned char shdisp_bdic_key_led_before_index;

static int shdisp_bdic_key_led_ontime;
static int shdisp_bdic_key_led_interval;
#endif

static struct shdisp_main_bkl_ctl shdisp_bkl_priority_table[NUM_SHDISP_MAIN_BKL_DEV_TYPE] = {
    { SHDISP_MAIN_BKL_MODE_OFF      , SHDISP_MAIN_BKL_PARAM_OFF },
    { SHDISP_MAIN_BKL_MODE_AUTO     , SHDISP_MAIN_BKL_PARAM_OFF }
};


static unsigned int shdisp_bdic_irq_fac = 0;
static unsigned int shdisp_bdic_irq_fac_exe = 0;

static int  shdisp_bdic_irq_prioriy[SHDISP_IRQ_MAX_KIND];

static unsigned char shdisp_backup_irq_photo_req[3];

static int shdisp_bdic_irq_det_flag = 0;

static int psals_recovery_flag = SHDISP_BDIC_PSALS_RECOVERY_NONE;

static int mled_delay_ms1 = 200;
static int mled_delay_ms2 = 0;
static int slope_fast = 0x00;

#if defined(CONFIG_ANDROID_ENGINEERING)
module_param(mled_delay_ms1, int, 0600);
module_param(mled_delay_ms2, int, 0600);
module_param(slope_fast, int, 0600);
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_boot_init                                                 */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_boot_init( void )
{
#ifdef SHDISP_BDIC_HW_INIT
    int ret;
#endif /* SHDISP_BDIC_HW_INIT */

    SHDISP_TRACE("in\n")
    shdisp_bdic_bkl_mode        = SHDISP_BDIC_BKL_MODE_OFF;
    shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
    shdisp_bdic_bkl_param       = SHDISP_MAIN_BKL_PARAM_OFF;
    shdisp_bdic_bkl_param_auto  = SHDISP_MAIN_BKL_PARAM_OFF;

    shdisp_bdic_dtv             = SHDISP_BDIC_BKL_DTV_OFF;

    shdisp_bdic_emg             = SHDISP_BDIC_BKL_EMG_OFF;

    shdisp_bdic_eco             = SHDISP_BDIC_BKL_ECO_OFF;

    shdisp_bdic_chg             = SHDISP_BDIC_BKL_CHG_OFF;

    shdisp_bdic_tri_led_color    = 0;
    shdisp_bdic_tri_led_mode     = SHDISP_BDIC_TRI_LED_MODE_OFF;
    shdisp_bdic_tri_led_before_mode = SHDISP_BDIC_TRI_LED_MODE_OFF;
    shdisp_bdic_tri_led_ontime   = 0;
    shdisp_bdic_tri_led_interval = 0;
    shdisp_bdic_tri_led_count    = 0;

    shdisp_bdic_register_driver();

#ifdef SHDISP_BDIC_HW_INIT
    s_state_str.bdic_chipver = 0x00;
    ret = shdisp_bdic_LD_hw_init();
    if (ret != SHDISP_RESULT_SUCCESS) {
        shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_BL_RST_N, SHDISP_GPIO_CTL_LOW);
        return SHDISP_BDIC_IS_NOT_EXIST;
    }
#endif /* SHDISP_BDIC_HW_INIT */

    SHDISP_TRACE("out\n")
    return SHDISP_BDIC_IS_EXIST;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_get_bdic_chipver                                          */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_get_bdic_chipver(int* chipver)
{
    *chipver = s_state_str.bdic_chipver;
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_initialize                                                */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_initialize(struct shdisp_bdic_state_str* state_str)
{
    s_state_str.bdic_is_exist                   = state_str->bdic_is_exist;
    s_state_str.bdic_main_bkl_opt_mode_output   = SHDISP_BDIC_MAIN_BKL_OPT_LOW;
    s_state_str.bdic_main_bkl_opt_mode_ado      = SHDISP_BDIC_MAIN_BKL_OPT_LOW;
    s_state_str.shdisp_lux_change_level1        = SHDISP_LUX_CHANGE_LEVEL1;
    s_state_str.shdisp_lux_change_level2        = SHDISP_LUX_CHANGE_LEVEL2;

#ifdef SHDISP_BDIC_HW_INIT
    if (s_state_str.bdic_is_exist == SHDISP_BDIC_IS_EXIST) {
        (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_INIT, SHDISP_DEV_STATE_ON);
        (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_INIT, SHDISP_DEV_STATE_OFF);
    }
#else  /* SHDISP_BDIC_HW_INIT */
    s_state_str.bdic_chipver                    = state_str->bdic_chipver;
#endif /* SHDISP_BDIC_HW_INIT */
    s_state_str.bdic_clrvari_index              = state_str->bdic_clrvari_index;
    memcpy(&(s_state_str.photo_sensor_adj),
                                &(state_str->photo_sensor_adj), sizeof(struct shdisp_photo_sensor_adj));
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_set_prox_sensor_param                                     */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_set_prox_sensor_param(struct shdisp_prox_params *prox_params)
{
    shdisp_bdic_PD_psals_write_threshold(prox_params);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_check_sensor_param                                        */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_check_sensor_param(struct shdisp_photo_sensor_adj *adj_in, struct shdisp_photo_sensor_adj *adj_out)
{
    struct shdisp_photo_sensor_adj tmp_adj;
    int err_flg = 0;
    unsigned long chksum;

    SHDISP_TRACE("in\n")
    memcpy(&tmp_adj, adj_in, sizeof(struct shdisp_photo_sensor_adj));

    SHDISP_INFO(" ---before---\n");
    SHDISP_INFO(" chksum        = 0x%08x\n", (unsigned int)tmp_adj.chksum);
    SHDISP_INFO(" status        = 0x%02x\n", tmp_adj.status);
    SHDISP_INFO(" als_adj0      = 0x%04x\n", (unsigned short)tmp_adj.als_adjust[0].als_adj0);
    SHDISP_INFO(" als_adj1      = 0x%04x\n", (unsigned short)tmp_adj.als_adjust[0].als_adj1);
    SHDISP_INFO(" als_shift     = 0x%02x\n", tmp_adj.als_adjust[0].als_shift);
    SHDISP_INFO(" clear_offset  = 0x%02x\n", tmp_adj.als_adjust[0].clear_offset);
    SHDISP_INFO(" ir_offset     = 0x%02x\n", tmp_adj.als_adjust[0].ir_offset);
    SHDISP_INFO(" als_adj0      = 0x%04x\n", (unsigned short)tmp_adj.als_adjust[1].als_adj0);
    SHDISP_INFO(" als_adj1      = 0x%04x\n", (unsigned short)tmp_adj.als_adjust[1].als_adj1);
    SHDISP_INFO(" als_shift     = 0x%02x\n", tmp_adj.als_adjust[1].als_shift);
    SHDISP_INFO(" clear_offset  = 0x%02x\n", tmp_adj.als_adjust[1].clear_offset);
    SHDISP_INFO(" ir_offset     = 0x%02x\n", tmp_adj.als_adjust[1].ir_offset);
    SHDISP_INFO(" key_backlight = 0x%02x\n", tmp_adj.key_backlight);

    if (tmp_adj.status != SHDISP_ALS_SENSOR_ADJUST_STATUS_COMPLETED) {
        err_flg = 1;
        SHDISP_DEBUG(": status check error.\n");
    } else if (tmp_adj.als_adjust[0].als_shift > 0x1F) {
        err_flg = 2;
        SHDISP_ERR(": als_shift check error.\n");
    } else if (tmp_adj.als_adjust[1].als_shift > 0x1F) {
        err_flg = 3;
        SHDISP_ERR(": als_shift check error.\n");
    } else {
        chksum = (unsigned long)tmp_adj.status
                  + (unsigned long)tmp_adj.key_backlight
                  + tmp_adj.als_adjust[0].als_adj0
                  + tmp_adj.als_adjust[0].als_adj1
                  + (unsigned long)tmp_adj.als_adjust[0].als_shift
                  + (unsigned long)tmp_adj.als_adjust[0].clear_offset
                  + (unsigned long)tmp_adj.als_adjust[0].ir_offset
                  + tmp_adj.als_adjust[1].als_adj0
                  + tmp_adj.als_adjust[1].als_adj1
                  + (unsigned long)tmp_adj.als_adjust[1].als_shift
                  + (unsigned long)tmp_adj.als_adjust[1].clear_offset
                  + (unsigned long)tmp_adj.als_adjust[1].ir_offset;
        if (tmp_adj.chksum != chksum) {
            err_flg = 9;
            SHDISP_ERR(": chksum check error.\n");
            SHDISP_ERR(" chksum = 0x%08x\n", (unsigned int)tmp_adj.chksum);
            SHDISP_ERR(" result = 0x%08x\n", (unsigned int)chksum);
        }
    }
    if (err_flg != 0) {
        shdisp_bdic_set_default_sensor_param(&tmp_adj);
        tmp_adj.status = (unsigned char)err_flg;
    }
    memcpy(adj_out, &tmp_adj, sizeof(struct shdisp_photo_sensor_adj));

    SHDISP_INFO("---after---\n");
    SHDISP_INFO(" chksum        = 0x%08x\n", (unsigned int)tmp_adj.chksum);
    SHDISP_INFO(" status        = 0x%02x\n", tmp_adj.status);
    SHDISP_INFO(" als_adj0      = 0x%04x\n", (unsigned short)tmp_adj.als_adjust[0].als_adj0);
    SHDISP_INFO(" als_adj1      = 0x%04x\n", (unsigned short)tmp_adj.als_adjust[0].als_adj1);
    SHDISP_INFO(" als_shift     = 0x%02x\n", tmp_adj.als_adjust[0].als_shift);
    SHDISP_INFO(" clear_offset  = 0x%02x\n", tmp_adj.als_adjust[0].clear_offset);
    SHDISP_INFO(" ir_offset     = 0x%02x\n", tmp_adj.als_adjust[0].ir_offset);
    SHDISP_INFO(" als_adj0      = 0x%04x\n", (unsigned short)tmp_adj.als_adjust[1].als_adj0);
    SHDISP_INFO(" als_adj1      = 0x%04x\n", (unsigned short)tmp_adj.als_adjust[1].als_adj1);
    SHDISP_INFO(" als_shift     = 0x%02x\n", tmp_adj.als_adjust[1].als_shift);
    SHDISP_INFO(" clear_offset  = 0x%02x\n", tmp_adj.als_adjust[1].clear_offset);
    SHDISP_INFO(" ir_offset     = 0x%02x\n", tmp_adj.als_adjust[1].ir_offset);
    SHDISP_INFO(" key_backlight = 0x%02x\n", tmp_adj.key_backlight);
    SHDISP_TRACE("out\n")

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_set_default_sensor_param                                      */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_set_default_sensor_param(struct shdisp_photo_sensor_adj *tmp_adj)
{
    unsigned short tmp_param1;
    unsigned short tmp_param2;

    tmp_param1 = ((unsigned short)BDIC_REG_ALS_ADJ0_H_DEFAULT_A << 8);
    tmp_param2 = (unsigned short)BDIC_REG_ALS_ADJ0_L_DEFAULT_A;
    tmp_adj->als_adjust[0].als_adj0     = tmp_param1 | tmp_param2;
    tmp_param1 = ((unsigned short)BDIC_REG_ALS_ADJ1_H_DEFAULT_A << 8);
    tmp_param2 = (unsigned short)BDIC_REG_ALS_ADJ1_L_DEFAULT_A;
    tmp_adj->als_adjust[0].als_adj1     = tmp_param1 | tmp_param2;
    tmp_adj->als_adjust[0].als_shift    = BDIC_REG_ALS_SHIFT_DEFAULT_A;
    tmp_adj->als_adjust[0].clear_offset = BDIC_REG_CLEAR_OFFSET_DEFAULT_A;
    tmp_adj->als_adjust[0].ir_offset    = BDIC_REG_IR_OFFSET_DEFAULT_A;

    tmp_param1 = ((unsigned short)BDIC_REG_ALS_ADJ0_H_DEFAULT_B << 8);
    tmp_param2 = (unsigned short)BDIC_REG_ALS_ADJ0_L_DEFAULT_B;
    tmp_adj->als_adjust[1].als_adj0     = tmp_param1 | tmp_param2;
    tmp_param1 = ((unsigned short)BDIC_REG_ALS_ADJ1_H_DEFAULT_B << 8);
    tmp_param2 = (unsigned short)BDIC_REG_ALS_ADJ1_L_DEFAULT_B;
    tmp_adj->als_adjust[1].als_adj1     = tmp_param1 | tmp_param2;
    tmp_adj->als_adjust[1].als_shift    = BDIC_REG_ALS_SHIFT_DEFAULT_B;
    tmp_adj->als_adjust[1].clear_offset = BDIC_REG_CLEAR_OFFSET_DEFAULT_B;
    tmp_adj->als_adjust[1].ir_offset    = BDIC_REG_IR_OFFSET_DEFAULT_B;

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_als_sensor_adjust                                         */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_als_sensor_adjust(struct shdisp_photo_sensor_adj *adj)
{
    shdisp_bdic_als_sensor_adjust[0].data = (unsigned char)(adj->als_adjust[1].als_adj0 & 0x00FF);
    shdisp_bdic_als_sensor_adjust[1].data = (unsigned char)(adj->als_adjust[1].als_adj0 >> 8);
    shdisp_bdic_als_sensor_adjust[2].data = (unsigned char)(adj->als_adjust[1].als_adj1 & 0x00FF);
    shdisp_bdic_als_sensor_adjust[3].data = (unsigned char)(adj->als_adjust[1].als_adj1 >> 8);
    shdisp_bdic_als_sensor_adjust[4].data = (adj->als_adjust[1].als_shift & 0x1F);
    shdisp_bdic_als_sensor_adjust[5].data = (unsigned char)(adj->als_adjust[1].als_adj0 & 0x00FF);
    shdisp_bdic_als_sensor_adjust[6].data = (unsigned char)(adj->als_adjust[1].als_adj0 >> 8);
    shdisp_bdic_als_sensor_adjust[7].data = (unsigned char)(adj->als_adjust[1].als_adj1 & 0x00FF);
    shdisp_bdic_als_sensor_adjust[8].data = (unsigned char)(adj->als_adjust[1].als_adj1 >> 8);
    shdisp_bdic_als_sensor_adjust[9].data = (adj->als_adjust[1].als_shift & 0x1F);
    shdisp_bdic_als_sensor_adjust[10].data = (unsigned char)(adj->als_adjust[1].als_adj0 & 0x00FF);
    shdisp_bdic_als_sensor_adjust[11].data = (unsigned char)(adj->als_adjust[1].als_adj0 >> 8);
    shdisp_bdic_als_sensor_adjust[12].data = (unsigned char)(adj->als_adjust[1].als_adj1 & 0x00FF);
    shdisp_bdic_als_sensor_adjust[13].data = (unsigned char)(adj->als_adjust[1].als_adj1 >> 8);
    shdisp_bdic_als_sensor_adjust[14].data = (adj->als_adjust[1].als_shift & 0x1F);

    shdisp_bdic_als_sensor_adjust[15].data = (unsigned char)(adj->als_adjust[0].als_adj0 & 0x00FF);
    shdisp_bdic_als_sensor_adjust[16].data = (unsigned char)(adj->als_adjust[0].als_adj0 >> 8);
    shdisp_bdic_als_sensor_adjust[17].data = (unsigned char)(adj->als_adjust[0].als_adj1 & 0x00FF);
    shdisp_bdic_als_sensor_adjust[18].data = (unsigned char)(adj->als_adjust[0].als_adj1 >> 8);
    shdisp_bdic_als_sensor_adjust[19].data = (adj->als_adjust[0].als_shift & 0x1F);

    SHDISP_BDIC_REGSET(shdisp_bdic_set_bank1);
    shdisp_SYS_delay_us(10*1000);
    SHDISP_BDIC_REGSET(shdisp_bdic_als_sensor_adjust);
    SHDISP_BDIC_REGSET(shdisp_bdic_set_bank0);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_release_hw_reset                                      */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_release_hw_reset(void)
{
    shdisp_bdic_LD_GPIO_control(SHDISP_BDIC_GPIO_COG_RESET, SHDISP_BDIC_GPIO_HIGH);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_set_hw_reset                                          */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_set_hw_reset(void)
{
    shdisp_bdic_LD_GPIO_control(SHDISP_BDIC_GPIO_COG_RESET, SHDISP_BDIC_GPIO_LOW);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_power_on                                              */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_power_on(void)
{
    SHDISP_TRACE("in\n")
    shdisp_bdic_PD_LCD_POS_PWR_on();
    SHDISP_TRACE("out\n")

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_power_off                                             */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_power_off(void)
{
    SHDISP_TRACE("in\n")
    shdisp_bdic_PD_LCD_POS_PWR_off();
    SHDISP_TRACE("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_m_power_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_m_power_on(void)
{
    SHDISP_TRACE("in\n")
    shdisp_bdic_PD_LCD_NEG_PWR_on();
    SHDISP_TRACE("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_m_power_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_m_power_off(void)
{
    SHDISP_TRACE("in\n")
    shdisp_bdic_PD_LCD_NEG_PWR_off();
    SHDISP_TRACE("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_off                                               */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_off(void)
{
    SHDISP_TRACE("in\n")
    shdisp_bdic_seq_backlight_off();
    SHDISP_TRACE("out\n")
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_set_active                                                */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_set_active(int power_status)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret =  shdisp_bdic_PD_set_active(power_status);
    SHDISP_TRACE("out\n");

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_set_standby                                               */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_set_standby(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_set_standby();
    SHDISP_TRACE("out\n");

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_fix_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_fix_on(int param)
{
    SHDISP_TRACE("in\n")
    shdisp_bdic_seq_backlight_fix_on(param);
    SHDISP_TRACE("out\n")
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_auto_on                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_auto_on(int param)
{
    SHDISP_TRACE("in\n")
    shdisp_bdic_seq_backlight_auto_on(param);
    SHDISP_TRACE("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_get_param                                         */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_get_param(unsigned long int* param)
{
    int mode = 0;
    unsigned char value;

    switch (shdisp_bdic_bkl_mode) {
    case SHDISP_BDIC_BKL_MODE_FIX:
        shdisp_bdic_LD_LCD_BKL_get_mode(&mode);
        shdisp_bdic_LD_LCD_BKL_get_fix_param(mode, shdisp_bdic_bkl_param, &value);
        *param = value;
        break;

    case SHDISP_BDIC_BKL_MODE_AUTO:
        *param = 0x100;
        break;

    default:
        *param = 0;
        break;
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_set_request                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_set_request(int type, struct shdisp_main_bkl_ctl *tmp)
{
    shdisp_bkl_priority_table[type].mode  = tmp->mode;
    shdisp_bkl_priority_table[type].param = tmp->param;

    shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
    shdisp_bdic_bkl_mode   = tmp->mode;
    shdisp_bdic_bkl_param  = tmp->param;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_set_request                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_TRI_LED_set_request(struct shdisp_tri_led *tmp)
{
    int color = 0x00;

    color = (tmp->blue << 2) | (tmp->green << 1) | tmp->red;

    shdisp_bdic_tri_led_mode        = tmp->led_mode;
    shdisp_bdic_tri_led_before_mode = tmp->led_mode;
    shdisp_bdic_tri_led_color       = color;
    shdisp_bdic_tri_led_ontime      = tmp->ontime;
    shdisp_bdic_tri_led_interval    = tmp->interval;
    shdisp_bdic_tri_led_count       = tmp->count;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_get_request                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_get_request(int type, struct shdisp_main_bkl_ctl *tmp, struct shdisp_main_bkl_ctl *req)
{
    shdisp_bkl_priority_table[type].mode  = tmp->mode;
    shdisp_bkl_priority_table[type].param = tmp->param;


    SHDISP_DEBUG("tmp->mode %d, tmp->param %d \n", tmp->mode, tmp->param)

    if (shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode == SHDISP_MAIN_BKL_MODE_OFF) {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param;
    } else if ((shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode == SHDISP_MAIN_BKL_MODE_FIX) &&
               (shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param == SHDISP_MAIN_BKL_PARAM_WEAK)) {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param;
    } else if (shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO].param != SHDISP_MAIN_BKL_PARAM_OFF) {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO].param;
    } else {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_dtv_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_dtv_on(void)
{
    shdisp_bdic_LD_LCD_BKL_dtv_on();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_dtv_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_dtv_off(void)
{
    shdisp_bdic_LD_LCD_BKL_dtv_off();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_emg_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_emg_on(void)
{
    shdisp_bdic_LD_LCD_BKL_emg_on();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_emg_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_emg_off(void)
{
    shdisp_bdic_LD_LCD_BKL_emg_off();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_eco_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_eco_on(void)
{
    shdisp_bdic_LD_LCD_BKL_eco_on();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_eco_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_eco_off(void)
{
    shdisp_bdic_LD_LCD_BKL_eco_off();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_chg_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_chg_on(void)
{
    shdisp_bdic_LD_LCD_BKL_chg_on();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_chg_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_chg_off(void)
{
    shdisp_bdic_LD_LCD_BKL_chg_off();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_off                                               */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_TRI_LED_off(void)
{
    int ret;
    ret = shdisp_bdic_seq_led_off();
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_get_color_index_and_reedit                        */
/* ------------------------------------------------------------------------- */

unsigned char shdisp_bdic_API_TRI_LED_get_color_index_and_reedit(struct shdisp_tri_led *tri_led )
{
    int i;
    unsigned char color = 0xFF;

    for (i = 0; i < ARRAY_SIZE(shdisp_triple_led_color_index_tbl); i++) {
        if (shdisp_triple_led_color_index_tbl[i].red   == tri_led->red     &&
            shdisp_triple_led_color_index_tbl[i].green == tri_led->green   &&
            shdisp_triple_led_color_index_tbl[i].blue  == tri_led->blue){
            color = shdisp_triple_led_color_index_tbl[i].color;
            break;
        }
    }

    if (color == 0xFF) {
        if (tri_led->red > 1) {
            tri_led->red = 1;
        }
        if (tri_led->green > 1) {
            tri_led->green = 1;
        }
        if (tri_led->blue > 1) {
            tri_led->blue = 1;
        }
        for (i = 0; i < ARRAY_SIZE(shdisp_triple_led_color_index_tbl); i++) {
            if (shdisp_triple_led_color_index_tbl[i].red   == tri_led->red     &&
                shdisp_triple_led_color_index_tbl[i].green == tri_led->green   &&
                shdisp_triple_led_color_index_tbl[i].blue  == tri_led->blue){
                color = shdisp_triple_led_color_index_tbl[i].color;
                break;
            }
        }
        if (color == 0xFF) {
            color = 0;
        }
    }
    return color;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_KEY_LED_get_color_index_and_reedit                        */
/* ------------------------------------------------------------------------- */
#if defined(CONFIG_MACH_TAC)
unsigned char shdisp_bdic_API_KEY_LED_get_color_index_and_reedit(struct shdisp_key_bkl_ctl *key_bkl_ctl )
{
    int i;
    unsigned char index = 0xFF;
    struct shdisp_key_bkl_ctl key_bkl;

    key_bkl = *key_bkl_ctl;

    if (key_bkl.key_left == SHDISP_KEY_BKL_DIM) {
        key_bkl.key_left = SHDISP_KEY_BKL_NORMAL;
    }
    if (key_bkl.key_center == SHDISP_KEY_BKL_DIM) {
        key_bkl.key_center = SHDISP_KEY_BKL_NORMAL;
    }
    if (key_bkl.key_right == SHDISP_KEY_BKL_DIM) {
       key_bkl.key_right = SHDISP_KEY_BKL_NORMAL;
    }

    for (i = 0; i < ARRAY_SIZE(shdisp_key_led_index_tbl); i++) {
        if ((shdisp_key_led_index_tbl[i].key_left== key_bkl.key_left) &&
            (shdisp_key_led_index_tbl[i].key_center== key_bkl.key_center) &&
            (shdisp_key_led_index_tbl[i].key_right== key_bkl.key_right)) {
            index = shdisp_key_led_index_tbl[i].index;
            break;
        }
    }

    if (index == 0xFF) {
        if (key_bkl.key_left > 1) {
            key_bkl.key_left = 1;
        }
        if (key_bkl.key_center > 1) {
            key_bkl.key_center = 1;
        }
        if (key_bkl.key_right > 1) {
            key_bkl.key_right = 1;
        }
        for (i = 0; i < ARRAY_SIZE(shdisp_key_led_index_tbl); i++) {
            if ((shdisp_key_led_index_tbl[i].key_left == key_bkl.key_left) &&
                (shdisp_key_led_index_tbl[i].key_center == key_bkl.key_center) &&
                (shdisp_key_led_index_tbl[i].key_right == key_bkl.key_right)) {
                index = shdisp_key_led_index_tbl[i].index;
                break;
            }
        }
        if (index == 0xFF) {
            index = 0;
        }
    }
    return index;
}
#else
unsigned char shdisp_bdic_API_KEY_LED_get_color_index_and_reedit(struct shdisp_key_bkl_ctl *key_bkl_ctl )
{
    return SHDISP_RESULT_SUCCESS;
}
#endif

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_normal_on                                         */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_TRI_LED_normal_on(unsigned char color)
{
    int ret;
    ret = shdisp_bdic_seq_led_normal_on(color);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_blink_on                                          */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_blink_on(unsigned char color, int ontime, int interval, int count)
{
    shdisp_bdic_seq_led_blink_on(color, ontime, interval, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_firefly_on                                        */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_firefly_on(unsigned char color, int ontime, int interval, int count)
{
    shdisp_bdic_seq_led_firefly_on(color, ontime, interval, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_KEY_LED_ctl                                               */
/* ------------------------------------------------------------------------- */
#if defined(CONFIG_MACH_TAC)
void shdisp_bdic_API_KEY_LED_ctl(unsigned char dim, unsigned char index, int ontime, int interval)
{
    shdisp_bdic_seq_key_led_ctl(dim, index, ontime, interval);
    return;
}
#else
void shdisp_bdic_API_KEY_LED_ctl(unsigned char dim, unsigned char index, int ontime, int interval)
{
    return;
}
#endif
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_get_clrvari_index                                 */
/* ------------------------------------------------------------------------- */

int  shdisp_bdic_API_TRI_LED_get_clrvari_index( int clrvari )
{
    int i = 0;

    for (i = 0; i < SHDISP_COL_VARI_KIND; i++) {
        if ((int)shdisp_clrvari_index[i] == clrvari) {
            break;
        }
    }
    if (i >= SHDISP_COL_VARI_KIND) {
        i = 0;
    }
    return i;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PHOTO_SENSOR_get_lux                                      */
/* ------------------------------------------------------------------------- */

int  shdisp_bdic_API_PHOTO_SENSOR_get_lux(unsigned short *value, unsigned long *lux)
{
    int ret;

    ret = shdisp_bdic_LD_PHOTO_SENSOR_get_lux(value, lux);

    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PHOTO_SENSOR_get_raw_als                                      */
/* ------------------------------------------------------------------------- */

int  shdisp_bdic_API_PHOTO_SENSOR_get_raw_als(unsigned short *clear, unsigned short *ir)
{
    int ret;

    ret = shdisp_bdic_LD_PHOTO_SENSOR_get_raw_als(clear, ir);

    return ret;
}
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PHOTO_SENSOR_lux_change_ind                               */
/* ------------------------------------------------------------------------- */

int  shdisp_bdic_API_PHOTO_SENSOR_lux_change_ind(int *mode)
{
    if (s_state_str.bdic_main_bkl_opt_mode_ado == SHDISP_BDIC_MAIN_BKL_OPT_LOW) {
        *mode = SHDISP_LUX_MODE_LOW;
    } else {
        *mode = SHDISP_LUX_MODE_HIGH;
    }

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_i2c_transfer                                              */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_i2c_transfer(struct shdisp_bdic_i2c_msg *msg)
{
    int ret = SHDISP_RESULT_SUCCESS;

    if (msg == NULL) {
        SHDISP_ERR("<NULL_POINTER> msg.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if (msg->mode >= NUM_SHDISP_BDIC_I2C_M) {
        SHDISP_ERR("<INVALID_VALUE> msg->mode(%d).\n", msg->mode);
        return SHDISP_RESULT_FAILURE;
    }


    shdisp_bdic_PD_i2c_throughmode_ctrl(true);
    ret = shdisp_bdic_PD_slave_transfer(msg);
    shdisp_bdic_PD_i2c_throughmode_ctrl(false);

    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DIAG_write_reg                                            */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_DIAG_write_reg(unsigned char reg, unsigned char val)
{
    int ret = 0;

    ret = shdisp_bdic_IO_write_reg(reg, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DIAG_read_reg                                             */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_DIAG_read_reg(unsigned char reg, unsigned char *val)
{
    int ret = 0;

    ret = shdisp_bdic_IO_read_reg(reg, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DIAG_multi_read_reg                                       */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_DIAG_multi_read_reg(unsigned char reg, unsigned char *val, int size)
{
    int ret = 0;

    ret = shdisp_bdic_IO_multi_read_reg(reg, val, size);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_RECOVERY_check_restoration                                */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_RECOVERY_check_restoration(void)
{
    unsigned char dummy=0;

    shdisp_bdic_IO_read_reg(BDIC_REG_GINF4, &dummy);

    if (dummy & 0x04) {
        return SHDISP_RESULT_SUCCESS;
    } else {
        return SHDISP_RESULT_FAILURE;
    }
}

#if defined (CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DBG_INFO_output                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_DBG_INFO_output(void)
{
    int idx;
    unsigned char   *p;
    unsigned char   *pbuf;
    unsigned char   shdisp_log_lv_bk;

    (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_DEBUG, SHDISP_DEV_STATE_ON);
    pbuf = kzalloc(256*2, GFP_KERNEL);
    if (!pbuf) {
        SHDISP_ERR("kzalloc failed. size=%d\n", 256*2);
        return;
    }
    SHDISP_BDIC_BACKUP_REGS_BDIC(shdisp_bdic_restore_regs_for_bank1_dump);

    shdisp_log_lv_bk = shdisp_log_lv;
    shdisp_log_lv = SHDISP_LOG_LV_ERR;

    shdisp_bdic_PD_wait4i2ctimer_stop();

    p = pbuf;
    shdisp_bdic_IO_bank_set(0x00);
    for (idx = 0x00; idx <= 0xFF; idx++ ) {
        *p = 0x00;
        shdisp_bdic_API_DIAG_read_reg(idx, p);
        p++;
    }
    shdisp_bdic_IO_bank_set(0x01);
    for (idx = 0x00; idx <= 0xFF; idx++ ) {
        *p = 0x00;
        shdisp_bdic_API_DIAG_read_reg(idx, p);
        p++;
    }
    shdisp_bdic_IO_bank_set(0x00);
    shdisp_log_lv = shdisp_log_lv_bk;

    SHDISP_BDIC_RESTORE_REGS(shdisp_bdic_restore_regs_for_bank1_dump);
    (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_DEBUG, SHDISP_DEV_STATE_OFF);

    printk("[SHDISP] BDIC INFO ->>\n");
    printk("[SHDISP] s_state_str.bdic_is_exist              = %d.\n", s_state_str.bdic_is_exist);
    printk("[SHDISP] s_state_str.bdic_chipver               = %d.\n", s_state_str.bdic_chipver);
    printk("[SHDISP] s_state_str.bdic_main_bkl_opt_mode_output = %d. (0:low/1:high)\n",
                                                                      s_state_str.bdic_main_bkl_opt_mode_output);
    printk("[SHDISP] s_state_str.bdic_main_bkl_opt_mode_ado = %d. (0:low/1:high)\n",
                                                                      s_state_str.bdic_main_bkl_opt_mode_ado);
    printk("[SHDISP] s_state_str.shdisp_lux_change_level1   = %d.\n", s_state_str.shdisp_lux_change_level1);
    printk("[SHDISP] s_state_str.shdisp_lux_change_level2   = %d.\n", s_state_str.shdisp_lux_change_level2);
    printk("[SHDISP] shdisp_bdic_bkl_mode                   = %d.\n", shdisp_bdic_bkl_mode);
    printk("[SHDISP] shdisp_bdic_bkl_param                  = %d.\n", shdisp_bdic_bkl_param);
    printk("[SHDISP] shdisp_bdic_bkl_param_auto             = %d.\n", shdisp_bdic_bkl_param_auto);
    printk("[SHDISP] shdisp_bdic_dtv                        = %d.\n", shdisp_bdic_dtv);
    printk("[SHDISP] shdisp_bdic_emg                        = %d.\n", shdisp_bdic_emg);
    printk("[SHDISP] shdisp_bdic_eco                        = %d.\n", shdisp_bdic_eco);
    printk("[SHDISP] shdisp_bdic_chg                        = %d.\n", shdisp_bdic_chg);
    printk("[SHDISP] shdisp_bdic_tri_led_color              = %d.\n", (int)shdisp_bdic_tri_led_color);
    printk("[SHDISP] shdisp_bdic_tri_led_mode               = %d.\n", shdisp_bdic_tri_led_mode);
    printk("[SHDISP] shdisp_bdic_tri_led_before_mode        = %d.\n", shdisp_bdic_tri_led_before_mode);
    printk("[SHDISP] shdisp_bdic_tri_led_ontime             = %d.\n", shdisp_bdic_tri_led_ontime);
    printk("[SHDISP] shdisp_bdic_tri_led_interval           = %d.\n", shdisp_bdic_tri_led_interval);
    printk("[SHDISP] shdisp_bdic_tri_led_count              = %d.\n", shdisp_bdic_tri_led_count);
    printk("[SHDISP] bdic_clrvari_index                     = %d.\n", s_state_str.bdic_clrvari_index );

    for ( idx = 0; idx < NUM_SHDISP_MAIN_BKL_DEV_TYPE; idx++ ) {
        printk("[SHDISP] shdisp_bkl_priority_table[%d]       = (mode:%d, param:%d).\n",
                                    idx, shdisp_bkl_priority_table[idx].mode, shdisp_bkl_priority_table[idx].param);
    }

    p = pbuf;
    for (idx = 0x00; idx < 0xFF; idx += 8) {
        printk("[SHDISP] BDIC_REG_BANK0 0x%2X: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                                    idx, *p, *(p+1), *(p+2), *(p+3), *(p+4), *(p+5), *(p+6), *(p+7));
        p += 8;
    }
    for (idx = 0x00; idx < 0xFF; idx += 8) {
        printk("[SHDISP] BDIC_REG_BANK1 0x%2X: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                                    idx, *p, *(p+1), *(p+2), *(p+3), *(p+4), *(p+5), *(p+6), *(p+7));
        p += 8;
    }
    printk("[SHDISP] BDIC INFO <<-\n");
    kfree(pbuf);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_OPT_INFO_output                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_OPT_INFO_output(void)
{
    int idx;
    unsigned char   *p;
    unsigned char   *pbuf;
    unsigned char   shdisp_log_lv_bk;

    (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_DEBUG, SHDISP_DEV_STATE_ON);
    pbuf = kzalloc((BDIC_REG_OPT23 - BDIC_REG_OPT0 + 1), GFP_KERNEL);
    if (!pbuf) {
        SHDISP_ERR("kzalloc failed. size=%d\n", (BDIC_REG_OPT23 - BDIC_REG_OPT0 + 1));
        return;
    }
    SHDISP_BDIC_BACKUP_REGS_BDIC(shdisp_bdic_restore_regs_for_bank1_dump);

    shdisp_log_lv_bk = shdisp_log_lv;
    shdisp_log_lv = SHDISP_LOG_LV_ERR;

    shdisp_bdic_PD_wait4i2ctimer_stop();

    p = pbuf;
    shdisp_bdic_IO_bank_set(0x01);
    for (idx = BDIC_REG_OPT0; idx <= BDIC_REG_OPT23; idx++) {
        *p = 0x00;
        shdisp_bdic_API_DIAG_read_reg(idx, p);
        p++;
    }
    shdisp_bdic_IO_bank_set(0x00);
    shdisp_log_lv = shdisp_log_lv_bk;

    SHDISP_BDIC_RESTORE_REGS(shdisp_bdic_restore_regs_for_bank1_dump);
    (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_DEBUG, SHDISP_DEV_STATE_OFF);

    printk("[SHDISP] BDIC_REG_OPT INFO ->>\n");
    printk("[SHDISP] shdisp_bdic_bkl_mode                   = %d.\n", shdisp_bdic_bkl_mode);
    printk("[SHDISP] shdisp_bdic_bkl_param                  = %d.\n", shdisp_bdic_bkl_param);
    printk("[SHDISP] shdisp_bdic_bkl_param_auto             = %d.\n", shdisp_bdic_bkl_param_auto);
    printk("[SHDISP] shdisp_bdic_emg                        = %d.\n", shdisp_bdic_emg);
    printk("[SHDISP] shdisp_bdic_chg                        = %d.\n", shdisp_bdic_chg);

    p = pbuf;
    printk("[SHDISP] BDIC_REG_OPT0  0x%2X: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                                BDIC_REG_OPT0, *p, *(p+1), *(p+2), *(p+3), *(p+4), *(p+5), *(p+6), *(p+7));
    p += 8;
    printk("[SHDISP] BDIC_REG_OPT8  0x%2X: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                                BDIC_REG_OPT8, *p, *(p+1), *(p+2), *(p+3), *(p+4), *(p+5), *(p+6), *(p+7));
    p += 8;
    printk("[SHDISP] BDIC_REG_OPT16 0x%2X: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                                BDIC_REG_OPT16, *p, *(p+1), *(p+2), *(p+3), *(p+4), *(p+5), *(p+6), *(p+7));

    printk("[SHDISP] BDIC_REG_OPT INFO <<-\n");
    kfree(pbuf);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_INFO_output                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_INFO_output(void)
{
    int idx;
    unsigned char   *p;
    unsigned char   *pbuf;

    pbuf = (unsigned char*)kzalloc((BDIC_REG_CH2_C - BDIC_REG_SEQ_ANIME + 1), GFP_KERNEL);
    if (!pbuf) {
        SHDISP_ERR("kzalloc failed. size=%d\n", (BDIC_REG_CH2_C - BDIC_REG_SEQ_ANIME + 1));
        return;
    }
    p = pbuf;
    for (idx = BDIC_REG_SEQ_ANIME; idx <= BDIC_REG_CH2_C; idx++ ) {
        *p = 0x00;
        shdisp_bdic_API_DIAG_read_reg(idx, p);
        p++;
    }

    printk("[SHDISP] TRI-LED INFO ->>\n");
    printk("[SHDISP] s_state_str.bdic_is_exist      = %d.\n", s_state_str.bdic_is_exist);
    printk("[SHDISP] shdisp_bdic_tri_led_color      = %d.\n", (int)shdisp_bdic_tri_led_color);
    printk("[SHDISP] shdisp_bdic_tri_led_mode       = %d.\n", shdisp_bdic_tri_led_mode);
    printk("[SHDISP] shdisp_bdic_tri_led_ontime     = %d.\n", shdisp_bdic_tri_led_ontime);
    printk("[SHDISP] shdisp_bdic_tri_led_interval   = %d.\n", shdisp_bdic_tri_led_interval);
    printk("[SHDISP] shdisp_bdic_tri_led_count      = %d.\n", shdisp_bdic_tri_led_count);
    printk("[SHDISP] shdisp_bdic_clrvari_index      = %d.\n", s_state_str.bdic_clrvari_index);

    p = pbuf;
    printk("[SHDISP] BDIC_REG_TIMER_SETTING 0x%2X: %02x %02x %02x\n", BDIC_REG_SEQ_ANIME, *p, *(p+1), *(p+2));
    p += 3;
    printk("[SHDISP] BDIC_REG_LED_SETTING   0x%2X: %02x %02x %02x %02x %02x %02x %02x\n",
                            BDIC_REG_CH0_SET1, *p, *(p+1), *(p+2), *(p+3), *(p+4), *(p+5), *(p+6));
    p += 7;
    printk("[SHDISP] BDIC_REG_LED_CURRENT   0x%2X: %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                            BDIC_REG_CH0_A, *p, *(p+1), *(p+2), *(p+3), *(p+4), *(p+5), *(p+6), *(p+7), *(p+8));

    kfree(pbuf);

    printk("[SHDISP] TRI-LED INFO <<-\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_psals_API_DBG_INFO_output                                          */
/* ------------------------------------------------------------------------- */

void shdisp_psals_API_DBG_INFO_output(void)
{
    int idx;
    unsigned char   *p;
    unsigned char   *pbuf;

    printk("[SHDISP] in PSALS SENSOR INFO ->>\n");
    (void)shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_DEBUG, SHDISP_DEV_STATE_ON);
    pbuf = kzalloc((((SENSOR_REG_D2_MSB+7)/8)*8), GFP_KERNEL);
    if (!pbuf) {
        SHDISP_ERR("kzalloc failed. size=%d\n", (((SENSOR_REG_D2_MSB+7)/8)*8));
        return;
    }

    shdisp_SYS_delay_us(1000*1000);

    shdisp_bdic_PD_i2c_throughmode_ctrl(true);

    p = pbuf;
    for (idx = SENSOR_REG_COMMAND1; idx <= SENSOR_REG_D2_MSB; idx++ ) {
        *p = 0x00;
        shdisp_photo_sensor_IO_read_reg(idx, p);
        p++;
    }
    p = pbuf;
    printk("[SHDISP] SENSOR_REG_DUMP 0x00: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                                                *p, *(p+1), *(p+2), *(p+3), *(p+4), *(p+5), *(p+6), *(p+7));
    p += 8;
    printk("[SHDISP] SENSOR_REG_DUMP 0x08: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                                                *p, *(p+1), *(p+2), *(p+3), *(p+4), *(p+5), *(p+6), *(p+7));
    p += 8;
    printk("[SHDISP] SENSOR_REG_DUMP 0x10: %02x %02x                              \n", *p, *(p+1));
    kfree(pbuf);

    shdisp_bdic_PD_i2c_throughmode_ctrl(false);

    (void)shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_DEBUG, SHDISP_DEV_STATE_OFF);

    printk("[SHDISP] out PSALS SENSOR INFO <<-\n");
    return;
}
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_type                                            */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_check_type(int irq_type)
{
    if ((irq_type < SHDISP_IRQ_TYPE_ALS) || (irq_type >= NUM_SHDISP_IRQ_TYPE)) {
        return SHDISP_RESULT_FAILURE;
    }
    if (irq_type == SHDISP_IRQ_TYPE_DET) {
        if (!(SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET)) {
           return SHDISP_RESULT_FAILURE;
        }
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_save_fac                                              */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_save_fac(void)
{
    unsigned char value1=0, value2=0, value3=0;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    shdisp_bdic_IO_read_reg(BDIC_REG_GFAC1, &value1);
    value1 &= 0x7F;
    shdisp_bdic_IO_read_reg(BDIC_REG_GFAC3, &value2);
    shdisp_bdic_IO_read_reg(BDIC_REG_GFAC4, &value3);
    SHDISP_DEBUG("GFAC4=%02x GFAC3=%02x GFAC1=%02x\n", value3, value2, value1);

    shdisp_bdic_irq_fac = (unsigned int)value1 | ((unsigned int)value2 << 8 ) | ((unsigned int)value3 << 16);

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMF4, 0x04);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMR1, 0x08);
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMF1, 0x08);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS2) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMF3, 0x02);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_I2C_ERR) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMR4, 0x08);
        SHDISP_ERR("[SHDISP] ps_als error : INT_I2C_ERR_REQ(GFAC4[3]) detect\n" );
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PSALS;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_I2C_ERROR;
        shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
    }

    if (shdisp_bdic_irq_fac & (SHDISP_BDIC_INT_GFAC_DET | SHDISP_BDIC_INT_GFAC_PS2 | SHDISP_BDIC_INT_GFAC_I2C_ERR)) {
        shdisp_bdic_IO_read_reg(BDIC_REG_GIMR3, &shdisp_backup_irq_photo_req[0]);
        shdisp_bdic_IO_read_reg(BDIC_REG_GIMF3, &shdisp_backup_irq_photo_req[1]);
        shdisp_bdic_IO_read_reg(BDIC_REG_GIMR4, &shdisp_backup_irq_photo_req[2]);
    }

    if ((shdisp_bdic_irq_fac & (SHDISP_BDIC_INT_GFAC_ALS | SHDISP_BDIC_INT_GFAC_OPTSEL)) != 0 ) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMR3, 0x01);
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMF3, 0x01);
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMR4, 0x20);
    }

    if (shdisp_bdic_irq_det_flag == 1) {
        shdisp_bdic_irq_fac |= SHDISP_BDIC_INT_GFAC_DET;
        shdisp_bdic_irq_det_flag = 2;
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_DET                                             */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_check_DET(void)
{
    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET) {
        return SHDISP_BDIC_IRQ_TYPE_DET;
    } else {
        return SHDISP_BDIC_IRQ_TYPE_NONE;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_I2C_ERR                                         */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_check_I2C_ERR(void)
{
    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_I2C_ERR) {
        return SHDISP_BDIC_IRQ_TYPE_I2C_ERR;
    } else {
        return SHDISP_BDIC_IRQ_TYPE_NONE;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_fac                                             */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_check_fac(void)
{
    int i;

    if (shdisp_bdic_irq_fac == 0) {
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_bdic_irq_fac_exe = (shdisp_bdic_irq_fac & SHDISP_INT_ENABLE_GFAC);
    if (shdisp_bdic_irq_fac_exe == 0) {
        return SHDISP_RESULT_FAILURE;
    }

    for (i = 0; i < SHDISP_IRQ_MAX_KIND; i++) {
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_NONE;
    }

    i = 0;
    if ((shdisp_bdic_irq_fac_exe &
        (SHDISP_BDIC_INT_GFAC_PS | SHDISP_BDIC_INT_GFAC_I2C_ERR | SHDISP_BDIC_INT_GFAC_PS2))
        == SHDISP_BDIC_INT_GFAC_PS) {
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_PS;
        i++;
    }

    if ((shdisp_bdic_irq_fac_exe & SHDISP_BDIC_INT_GFAC_DET) != 0) {
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_DET;
        i++;
    } else if (((shdisp_bdic_irq_fac_exe & SHDISP_BDIC_INT_GFAC_I2C_ERR) != 0)) {
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_I2C_ERR;
        i++;
    } else if ((shdisp_bdic_irq_fac_exe & (SHDISP_BDIC_INT_GFAC_ALS | SHDISP_BDIC_INT_GFAC_OPTSEL)) != 0) {
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_ALS;
        i++;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_get_fac                                               */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_get_fac(int iQueFac)
{
    if (iQueFac >= SHDISP_IRQ_MAX_KIND) {
        return SHDISP_BDIC_IRQ_TYPE_NONE;
    }
    return shdisp_bdic_irq_prioriy[iQueFac];
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_Clear                                                 */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_Clear(void)
{
    unsigned char out1,out2,out3;

    if (shdisp_bdic_irq_fac == 0) {
        return;
    }

    out1 = (unsigned char)(shdisp_bdic_irq_fac & 0x000000FF);
    out2 = (unsigned char)((shdisp_bdic_irq_fac >> 8 ) & 0x000000FF);
    out3 = (unsigned char)((shdisp_bdic_irq_fac >> 16 ) & 0x000000FF);

    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR1, out1);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, out2);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, out3);

    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR1, 0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, 0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, 0x00);

    if ((shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS) && (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS)) {
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_GIMR1, 0x08);
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_GIMF1, 0x08);
    }

    if ((shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS2) && (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2)) {
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_GIMF3, 0x02);
    }

    if (((shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET) &&
         (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET)) ||
        ((shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS2) &&
         (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2)) ||
        ((shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_I2C_ERR) &&
         (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_I2C_ERR))) {
        if (shdisp_backup_irq_photo_req[0] & 0x01) {
            shdisp_bdic_IO_set_bit_reg(BDIC_REG_GIMR3, 0x01);
        }
        if (shdisp_backup_irq_photo_req[1] & 0x01) {
            shdisp_bdic_IO_set_bit_reg(BDIC_REG_GIMF3, 0x01);
        }
        if (shdisp_backup_irq_photo_req[2] & 0x20) {
            shdisp_bdic_IO_set_bit_reg(BDIC_REG_GIMR4, 0x20);
        }
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_i2c_error_Clear                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_i2c_error_Clear(void)
{
    unsigned char out2=0,out3=0;

    if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_I2C_ERR) {
        out3 = 0x08;
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, out3);
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, 0x00);
    }

    if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) {
        out2 = 0x02;
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, out2);
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, 0x00);
    }
#ifndef SHDISP_NOT_SUPPORT_DET_I2CERR
    psals_recovery_flag = SHDISP_BDIC_PSALS_RECOVERY_DURING;
#endif /* SHDISP_NOT_SUPPORT_DET_I2CERR */
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_det_fac_Clear                                         */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_det_fac_Clear(void)
{
    unsigned char out3 = 0;

    if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET) {
        out3 = 0x04;
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, out3);
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, 0x00);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_det_irq_ctrl                                          */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_IRQ_det_irq_ctrl(int ctrl)
{
    shdisp_bdic_IO_bank_set(0x00);
    if (ctrl) {
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_GIMF4, 0x04);
    } else {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMF4, 0x04);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_dbg_Clear_All                                         */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_dbg_Clear_All(void)
{
    unsigned char out1, out2, out3;

    out1 = (unsigned char)(SHDISP_INT_ENABLE_GFAC & 0x000000FF);
    out2 = (unsigned char)((SHDISP_INT_ENABLE_GFAC >> 8 ) & 0x000000FF);
    out3 = (unsigned char)((SHDISP_INT_ENABLE_GFAC >> 16 ) & 0x000000FF);

    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR1, out1);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, out2);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, out3);

    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR1, 0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, 0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, 0x00);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_dbg_set_fac                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_dbg_set_fac(unsigned int nGFAC)
{
    shdisp_bdic_irq_fac = nGFAC;

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMF4, 0x04);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMR1, 0x08);
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMF1, 0x08);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_ALS) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMR3, 0x01);
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMF3, 0x01);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS2) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMF3, 0x02);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_OPTSEL) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMR4, 0x20);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_dbg_photo_param                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_dbg_photo_param(int level1, int level2)
{
    s_state_str.shdisp_lux_change_level1 = (unsigned char)(level1 & 0x00FF);
    s_state_str.shdisp_lux_change_level2 = (unsigned char)(level2 & 0x00FF);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_i2c_throughmode_ctrl                                      */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_i2c_throughmode_ctrl(bool ctrl)
{
    int ret;
    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_i2c_throughmode_ctrl(ctrl);
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_als_sensor_pow_ctl                                        */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_als_sensor_pow_ctl(int dev_type, int power_mode)
{
    unsigned long type = 0;
    int param_chk = 0;

    switch (dev_type) {
    case SHDISP_PHOTO_SENSOR_TYPE_APP:
        type = SHDISP_DEV_TYPE_ALS_APP;
        break;
    case SHDISP_PHOTO_SENSOR_TYPE_CAMERA:
        type = SHDISP_DEV_TYPE_ALS_CAMERA;
        break;
    case SHDISP_PHOTO_SENSOR_TYPE_KEYLED:
        type = SHDISP_DEV_TYPE_ALS_KEYLED;
        break;
    case SHDISP_PHOTO_SENSOR_TYPE_DIAG:
        type = SHDISP_DEV_TYPE_ALS_DIAG;
        break;
    default:
        param_chk = 1;
        SHDISP_ERR("<INVALID_VALUE> ctl->type(%d).\n", dev_type);
        break;
    }

    switch (power_mode) {
    case SHDISP_PHOTO_SENSOR_DISABLE:
        shdisp_bdic_seq_psals_standby(type);
        break;
    case SHDISP_PHOTO_SENSOR_ENABLE:
        shdisp_bdic_seq_psals_active(type);
        break;
    default:
        param_chk = 1;
        SHDISP_ERR("<INVALID_VALUE> ctl->power(%d).\n", power_mode);
        break;
    }

    if (param_chk == 1) {
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_power_on                                            */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_power_on(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_power_on();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_power_off                                           */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_power_off(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_power_off();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_active                                              */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_psals_active(unsigned long dev_type)
{
    SHDISP_TRACE("in\n");
    shdisp_bdic_seq_psals_active(dev_type);
    SHDISP_TRACE("out\n");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_standby                                             */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_psals_standby(unsigned long dev_type)
{
    SHDISP_TRACE("in\n");
    shdisp_bdic_seq_psals_standby(dev_type);
    SHDISP_TRACE("out\n");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_ps_init_als_off                                     */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_ps_init_als_off(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_ps_init_als_off();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_ps_init_als_on                                      */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_ps_init_als_on(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_ps_init_als_on();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_ps_deinit_als_off                                   */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_ps_deinit_als_off(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_ps_deinit_als_off();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_ps_deinit_als_on                                    */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_ps_deinit_als_on(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_ps_deinit_als_on();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_als_init_ps_off                                     */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_als_init_ps_off(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_als_init_ps_off();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_als_init_ps_on                                      */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_als_init_ps_on(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_als_init_ps_on();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_als_deinit_ps_off                                   */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_als_deinit_ps_off(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_als_deinit_ps_off();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_als_deinit_ps_on                                    */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_als_deinit_ps_on(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_psals_als_deinit_ps_on();
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_is_recovery_successful                              */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_is_recovery_successful(void)
{
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */
    if (psals_recovery_flag == SHDISP_BDIC_PSALS_RECOVERY_RETRY_OVER) {
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_GIMR4, 0x08);
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, 0x08);
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR4, 0x00);
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PSALS;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_RECOVERY_NG;
        shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
        return SHDISP_RESULT_FAILURE;
    }

    psals_recovery_flag = SHDISP_BDIC_PSALS_RECOVERY_NONE;
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_get_ave_ado                                               */
/* ------------------------------------------------------------------------- */
int  shdisp_bdic_API_get_ave_ado(struct shdisp_ave_ado *ave_ado)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_get_ave_ado(ave_ado);
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* Logical Driver                                                            */
/* ------------------------------------------------------------------------- */
#ifdef SHDISP_BDIC_HW_INIT
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_version_check                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_LD_version_check(unsigned char version)
{
    if ((version & SHDISP_BDIC_VERSION71) != SHDISP_BDIC_VERSION71) {
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_hw_init                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_LD_hw_init(void)
{
    int ret;
    unsigned char version;

    shdisp_bdic_PD_hw_reset();

    ret = shdisp_bdic_IO_read_check_reg(BDIC_REG_VERSION, &version);
    if (ret != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_bdic_LD_version_check(version);
    if (ret != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_BDIC_HW_INIT */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_GPIO_control                                               */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_GPIO_control(unsigned char symbol, unsigned char status)
{
    unsigned char port;

    switch (symbol) {
    case SHDISP_BDIC_GPIO_COG_RESET:
        port = SHDISP_BDIC_GPIO_GPOD4;
        break;

    default:
        return;
    }

    shdisp_bdic_PD_GPIO_control(port, status);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_backlight_off                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_backlight_off(void)
{

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_MODE_OFF, 0);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_STOP, 0);

    (void)shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_ALS_BKL, SHDISP_DEV_STATE_OFF);
    (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_BKL, SHDISP_DEV_STATE_OFF);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_backlight_fix_on                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_backlight_fix_on(int param)
{
    SHDISP_TRACE("in param:%d\n", param);

    (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_BKL, SHDISP_DEV_STATE_ON);

    SHDISP_PERFORMANCE("RESUME BDIC TURN-ON BACKLIGHT 0020 START\n");

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_MODE_FIX, param);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_LED_VALUE, 0);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_ON, 0);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_FIX_START, 0);

    SHDISP_PERFORMANCE("RESUME BDIC TURN-ON BACKLIGHT 0020 END\n");

    (void)shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_ALS_BKL, SHDISP_DEV_STATE_ON);

    SHDISP_TRACE("out\n");
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_backlight_auto_on                                         */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_backlight_auto_on(int param)
{
    (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_BKL, SHDISP_DEV_STATE_ON);

    SHDISP_PERFORMANCE("RESUME BDIC TURN-ON BACKLIGHT 0010 START\n");

    (void)shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_ALS_BKL, SHDISP_DEV_STATE_ON);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_MODE_AUTO, param);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_LED_VALUE, 0);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_ON, 0);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_OPT_VALUE, 0);
    shdisp_SYS_delay_us(13*1000);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_AUTO_START, 0);

    SHDISP_PERFORMANCE("RESUME BDIC TURN-ON BACKLIGHT 0010 END\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_off                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_seq_led_off(void)
{
    SHDISP_TRACE("in\n");

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_STOP, 0);
    shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_normal_on                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_seq_led_normal_on(unsigned char color)
{
    SHDISP_TRACE("in color:%d\n", color);

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_blink_on                                              */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_led_blink_on(unsigned char color, int ontime, int interval, int count)
{
    SHDISP_TRACE("in color:%d\n", color);

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BLINK,   color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME,       ontime);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);

    if (s_state_str.bdic_chipver >= SHDISP_BDIC_CHIPVER_1) {
        shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);
    }
    SHDISP_TRACE("out\n");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_firefly_on                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_led_firefly_on(unsigned char color, int ontime, int interval, int count)
{
    SHDISP_TRACE("in color:%d\n", color);

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FIREFLY, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME,       ontime);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);

    if (s_state_str.bdic_chipver >= SHDISP_BDIC_CHIPVER_1) {
        shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);
    }
    SHDISP_TRACE("out\n");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_set_led_fix_on_table                                       */
/* ------------------------------------------------------------------------- */
#if defined(CONFIG_MACH_PA25) || defined(CONFIG_MACH_PB25) || defined(CONFIG_MACH_KF10) || defined(CONFIG_MACH_TAC) || defined(CONFIG_MACH_PA28)
static void shdisp_bdic_LD_set_led_fix_on_table(int clr_vari, int color)
{
    unsigned char *pTriLed;

    pTriLed = (unsigned char*)(&(shdisp_triple_led_tbl[clr_vari][color]));

    shdisp_bdic_led_fix_on[0].data = *(pTriLed + 0);
    shdisp_bdic_led_fix_on[1].data = *(pTriLed + 1);
    shdisp_bdic_led_fix_on[2].data = *(pTriLed + 2);
}
#endif


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_key_led_ctl                                               */
/* ------------------------------------------------------------------------- */
#if defined(CONFIG_MACH_TAC)
static void shdisp_bdic_seq_key_led_ctl(unsigned char dim, unsigned char index, int ontime, int interval)
{
    SHDISP_TRACE("in index:%d\n", index);

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_KEYLED);
    shdisp_bdic_PD_KEY_LED_control(dim, index, ontime, interval);

    if (s_state_str.bdic_chipver == SHDISP_BDIC_CHIPVER_0) {
        if (index == 0) {
            shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_KEYLED);
        }
    } else if (s_state_str.bdic_chipver >= SHDISP_BDIC_CHIPVER_1) {
        if (index == 0 || ((8 <= index) && (index <= 14))) {
            shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_KEYLED);
        }
    } else {
    }

    SHDISP_TRACE("out\n");

}
#endif

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_PHOTO_SENSOR_get_lux                                       */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_PHOTO_SENSOR_get_lux(unsigned short *value, unsigned long *lux)
{
    int i;
    int ret;
    unsigned long ret_lux;
    unsigned long ado;

    SHDISP_TRACE("in\n");

    ret = shdisp_pm_is_als_active();
    if (ret == SHDISP_DEV_STATE_OFF) {
        SHDISP_ERR("<OTHER> photo sensor user none.\n");
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_bdic_PD_REG_ADO_get_opt(value);
    if (ret != SHDISP_RESULT_SUCCESS) {
        return ret;
    }

    ado = (unsigned long)(*value);
    ret_lux = 0;
    if (ado != 0) {
        for (i = 0; i < SHDISP_BDIC_LUX_TABLE_ARRAY_SIZE; i++) {
            if ((ado >= shdisp_bdic_bkl_ado_tbl[i].range_low) &&
                (ado < shdisp_bdic_bkl_ado_tbl[i].range_high)) {
                ret_lux  = (unsigned long)((unsigned short)ado * (unsigned short)shdisp_bdic_bkl_ado_tbl[i].param_a);
                ret_lux += shdisp_bdic_bkl_ado_tbl[i].param_b;
                ret_lux += (SHDISP_BDIC_LUX_DIVIDE_COFF / 2);
                ret_lux /= SHDISP_BDIC_LUX_DIVIDE_COFF;
                break;
            }
        }
    }

   *lux = ret_lux;

    SHDISP_TRACE("out ado=0x%04X, lux=%lu\n", (unsigned int)ado, ret_lux);
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_PHOTO_SENSOR_get_raw_als                                       */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_PHOTO_SENSOR_get_raw_als(unsigned short *clear, unsigned short *ir)
{
    int ret;

    SHDISP_TRACE("in\n");

    ret = shdisp_pm_is_als_active();
    if (ret == SHDISP_DEV_STATE_OFF) {
        SHDISP_ERR("<OTHER> photo sensor user none.\n");
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_bdic_PD_REG_RAW_DATA_get_opt(clear,ir);

    SHDISP_TRACE("out clear=0x%04X, ir=0x%04X\n", (unsigned int)*clear, (unsigned int)*ir);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_psals_active                                              */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_psals_active(unsigned long dev_type)
{
    (void)shdisp_pm_bdic_power_manager(dev_type, SHDISP_DEV_STATE_ON);
    (void)shdisp_pm_psals_power_manager(dev_type, SHDISP_DEV_STATE_ON);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_psals_standby                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_psals_standby(unsigned long dev_type)
{
    (void)shdisp_pm_psals_power_manager(dev_type, SHDISP_DEV_STATE_OFF);
    (void)shdisp_pm_bdic_power_manager(dev_type, SHDISP_DEV_STATE_OFF);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_dtv_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_dtv_on(void)
{
    if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_ON) {
        return;
    }

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_DTV_ON, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_dtv_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_dtv_off(void)
{
    if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_OFF) {
        return;
    }

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_DTV_OFF, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_emg_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_emg_on(void)
{
    if (shdisp_bdic_emg == SHDISP_BDIC_BKL_EMG_ON) {
        return;
    }

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_EMG_ON, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_emg_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_emg_off(void)
{
    if (shdisp_bdic_emg == SHDISP_BDIC_BKL_EMG_OFF) {
        return;
    }

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_EMG_OFF, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_get_mode                                           */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_get_mode(int *mode)
{

    if (shdisp_bdic_emg == SHDISP_BDIC_BKL_EMG_ON) {
        *mode = SHDISP_BKL_TBL_MODE_EMERGENCY;
    } else if ((shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_FIX) && (shdisp_bdic_chg == SHDISP_BDIC_BKL_CHG_ON)) {
        *mode = SHDISP_BKL_TBL_MODE_CHARGE;
    } else if (shdisp_bdic_eco == SHDISP_BDIC_BKL_ECO_ON) {
        *mode = SHDISP_BKL_TBL_MODE_ECO;
    } else {
        *mode = SHDISP_BKL_TBL_MODE_NORMAL;
    }

    if (*mode >= NUM_SHDISP_BKL_TBL_MODE) {
        *mode = SHDISP_BKL_TBL_MODE_NORMAL;
    }

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_get_fix_param                                      */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_get_fix_param(int mode, int level, unsigned char *param)
{
    unsigned char value;

    if (param == NULL) {
        return;
    }

    value = shdisp_main_bkl_tbl[level];

    switch (mode) {
    case SHDISP_BKL_TBL_MODE_NORMAL:
    case SHDISP_BKL_TBL_MODE_ECO:
        break;
    case SHDISP_BKL_TBL_MODE_EMERGENCY:
        if (value > SHDISP_BKL_EMERGENCY_LIMIT_FIX) {
            value = SHDISP_BKL_EMERGENCY_LIMIT_FIX;
        }
        break;
    case SHDISP_BKL_TBL_MODE_CHARGE:
    default:
        break;
    }

    *param = value;
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_eco_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_eco_on(void)
{
    if (shdisp_bdic_eco == SHDISP_BDIC_BKL_ECO_ON) {
        return;
    }

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_ECO_ON, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_eco_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_eco_off(void)
{
    if (shdisp_bdic_eco == SHDISP_BDIC_BKL_ECO_OFF) {
        return;
    }

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_ECO_OFF, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_chg_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_chg_on(void)
{
    if (shdisp_bdic_chg == SHDISP_BDIC_BKL_CHG_ON) {
        return;
    }

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_CHG_ON, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_chg_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_chg_off(void)
{
    if (shdisp_bdic_chg == SHDISP_BDIC_BKL_CHG_OFF) {
        return;
    }

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_CHG_OFF, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* Phygical Driver                                                           */
/* ------------------------------------------------------------------------- */
#ifdef SHDISP_BDIC_HW_INIT
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_hw_reset                                                   */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_hw_reset(void)
{
    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_BL_RST_N, SHDISP_GPIO_CTL_LOW);
    shdisp_SYS_delay_us(15000);
    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_BL_RST_N, SHDISP_GPIO_CTL_HIGH);
    shdisp_SYS_delay_us(1000);
}
#endif /* SHDISP_BDIC_HW_INIT */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_POS_PWR_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_LCD_POS_PWR_on(void)
{
    SHDISP_TRACE("in\n")
    SHDISP_BDIC_REGSET(shdisp_bdic_vsp_on);
    SHDISP_TRACE("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_POS_PWR_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_LCD_POS_PWR_off(void)
{
    SHDISP_TRACE("in\n")
    SHDISP_BDIC_REGSET(shdisp_bdic_vsp_off);
    SHDISP_TRACE("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_NEG_PWR_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_LCD_NEG_PWR_on(void)
{
    SHDISP_TRACE("in\n")
    if (s_state_str.bdic_chipver >= SHDISP_BDIC_CHIPVER_1) {
        SHDISP_DEBUG("VSN ON(TS2) chipver=%d\n", s_state_str.bdic_chipver);
        SHDISP_BDIC_REGSET(shdisp_bdic_vsn_on_ts2);
    } else {
        SHDISP_DEBUG("VSN ON(TS1) chipver=%d\n", s_state_str.bdic_chipver);
        SHDISP_BDIC_REGSET(shdisp_bdic_vsn_on_ts1);
    }
    SHDISP_TRACE("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_NEG_PWR_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_LCD_NEG_PWR_off(void)
{
    SHDISP_TRACE("in\n")
    SHDISP_BDIC_REGSET(shdisp_bdic_vsn_off);
    SHDISP_TRACE("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_bdic_active_for_led                                       */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_bdic_active_for_led(int dev_type)
{
    SHDISP_TRACE("in dev_type:%d\n", dev_type);
    (void)shdisp_pm_bdic_power_manager(dev_type, SHDISP_DEV_STATE_ON);
    SHDISP_TRACE("out\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_bdic_standby_for_led                                      */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_bdic_standby_for_led(int dev_type)
{
    (void)shdisp_pm_bdic_power_manager(dev_type, SHDISP_DEV_STATE_OFF);
    return;
}

/*---------------------------------------------------------------------------*/
/* shdisp_bdic_seq_regset                                                    */
/*---------------------------------------------------------------------------*/
static int shdisp_bdic_seq_regset(const shdisp_bdicRegSetting_t* regtable, int size)
{
    int i, cnt_bdic, cnt_als;
    int ret = SHDISP_RESULT_SUCCESS;
    shdisp_bdicRegSetting_t* tbl;
    unsigned char top_addr_bdic;
    unsigned char bBuf_bdic[16];
    unsigned char bBuf_als[16];

    cnt_bdic = 0;
    cnt_als  = 0;
    top_addr_bdic = 0x00;

    tbl = (shdisp_bdicRegSetting_t*)regtable;
    for (i = 0; i < size; i++) {
        if (((cnt_bdic > 0) && (tbl->flg != SHDISP_BDIC_STRM)) || (cnt_bdic == sizeof(bBuf_bdic))) {
            ret = shdisp_bdic_IO_multi_write_reg(top_addr_bdic, bBuf_bdic, cnt_bdic);
            cnt_bdic = 0;
            top_addr_bdic = 0x00;
        }
        if (((cnt_als > 0) && (tbl->flg != SHDISP_ALS_STRM)) || (cnt_als == sizeof(bBuf_als))) {
            ret = shdisp_photo_sensor_IO_burst_write_reg(bBuf_als, (unsigned char)cnt_als);
            cnt_als = 0;
        }
        switch(tbl->flg) {
        case SHDISP_BDIC_STR:
            ret = shdisp_bdic_IO_write_reg(tbl->addr, tbl->data);
            break;
        case SHDISP_BDIC_SET:
            ret = shdisp_bdic_IO_set_bit_reg(tbl->addr, tbl->data);
            break;
        case SHDISP_BDIC_CLR:
            ret = shdisp_bdic_IO_clr_bit_reg(tbl->addr, tbl->mask);
            break;
        case SHDISP_BDIC_RMW:
            ret = shdisp_bdic_IO_msk_bit_reg(tbl->addr, tbl->data, tbl->mask);
            break;
        case SHDISP_BDIC_STRM:
            if (cnt_bdic == 0) {
                top_addr_bdic = tbl->addr;
            }
            bBuf_bdic[cnt_bdic] = tbl->data;
            cnt_bdic++;
            if ((i + 1) == size) {
                ret = shdisp_bdic_IO_multi_write_reg(top_addr_bdic, bBuf_bdic, cnt_bdic);
                cnt_bdic = 0;
                top_addr_bdic = 0x00;
            }
            break;
        case SHDISP_BDIC_BANK:
            ret = shdisp_bdic_IO_bank_set(tbl->data);
            break;
        case SHDISP_BDIC_WAIT:
            shdisp_SYS_delay_us(tbl->wait);
            ret = SHDISP_RESULT_SUCCESS;
            break;
        case SHDISP_ALS_STR:
            ret = shdisp_photo_sensor_IO_write_reg(tbl->addr, tbl->data);
            break;
        case SHDISP_ALS_STRM:
        case SHDISP_ALS_STRMS:
            if (cnt_als == 0) {
                bBuf_als[cnt_als] = tbl->addr;
                cnt_als++;
            }
            bBuf_als[cnt_als] = tbl->data;
            cnt_als++;
            if ((i + 1) == size) {
                ret = shdisp_photo_sensor_IO_burst_write_reg(bBuf_als, (unsigned char)cnt_als);
            }
            break;
        case SHDISP_ALS_RMW:
            ret = shdisp_phote_sensor_IO_msk_bit_reg(tbl->addr, tbl->data, tbl->mask);
            break;
        default:
            break;
        }
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("bdic R/W Error addr=%02X, data=%02X, mask=%02X\n", tbl->addr, tbl->data, tbl->mask);
            continue;
        }
        if (tbl->wait > 0) {
            if ((cnt_bdic > 0) && (tbl->flg == SHDISP_BDIC_STRM)) {
                ret = shdisp_bdic_IO_multi_write_reg(top_addr_bdic, bBuf_bdic, cnt_bdic);
                cnt_bdic = 0;
                top_addr_bdic = 0x00;
            }
            if ((cnt_als > 0) && (tbl->flg == SHDISP_ALS_STRM)) {
                ret = shdisp_photo_sensor_IO_burst_write_reg(bBuf_als, (unsigned char)cnt_als);
                cnt_als = 0;
            }
            shdisp_SYS_delay_us(tbl->wait);
        }
        tbl++;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_backup_bdic_regs                                          */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_seq_backup_bdic_regs(shdisp_bdicRegSetting_t *regs, int size)
{
    int i;
    int ret = SHDISP_RESULT_SUCCESS;
    shdisp_bdicRegSetting_t* tbl;

    tbl = (shdisp_bdicRegSetting_t*)regs;
    for (i = 0; i < size; i++) {
        switch(tbl->flg) {
        case SHDISP_BDIC_STR:
        case SHDISP_BDIC_STRM:
            ret = shdisp_bdic_IO_read_reg(tbl->addr, &(tbl->data));
            break;
        case SHDISP_BDIC_SET:
        case SHDISP_BDIC_CLR:
        case SHDISP_BDIC_RMW:
            break;
        case SHDISP_BDIC_BANK:
            ret = shdisp_bdic_IO_bank_set(tbl->data);
            break;
        case SHDISP_BDIC_WAIT:
            shdisp_SYS_delay_us(tbl->wait);
            ret = SHDISP_RESULT_SUCCESS;
            break;
        case SHDISP_ALS_STR:
        case SHDISP_ALS_STRM:
        case SHDISP_ALS_STRMS:
        case SHDISP_ALS_RMW:
            break;
        default:
            break;
        }
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("bdic Read Error addr=%02X, data=%02X, mask=%02X\n", tbl->addr, tbl->data, tbl->mask);
            continue;
        }
        tbl++;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_backup_als_regs                                           */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_seq_backup_als_regs(shdisp_bdicRegSetting_t *regs, int size)
{
    int i;
    int ret = SHDISP_RESULT_SUCCESS;
    shdisp_bdicRegSetting_t* tbl;

    tbl = (shdisp_bdicRegSetting_t*)regs;
    for (i = 0; i < size; i++) {
        switch(tbl->flg) {
        case SHDISP_BDIC_STR:
        case SHDISP_BDIC_STRM:
            ret = shdisp_bdic_IO_write_reg(tbl->addr, tbl->data);
            break;
        case SHDISP_BDIC_SET:
        case SHDISP_BDIC_CLR:
        case SHDISP_BDIC_RMW:
            break;
        case SHDISP_BDIC_BANK:
            ret = shdisp_bdic_IO_bank_set(tbl->data);
            break;
        case SHDISP_BDIC_WAIT:
            shdisp_SYS_delay_us(tbl->wait);
            ret = SHDISP_RESULT_SUCCESS;
            break;
        case SHDISP_ALS_STR:
        case SHDISP_ALS_STRM:
        case SHDISP_ALS_STRMS:
            ret = shdisp_photo_sensor_IO_read_reg(tbl->addr, &(tbl->data));
            break;
        case SHDISP_ALS_RMW:
            break;
        default:
            break;
        }
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("bdic Read Error addr=%02X, data=%02X, mask=%02X\n", tbl->addr, tbl->data, tbl->mask);
            continue;
        }
        tbl++;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_set_active                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_set_active(int power_status)
{
    int ret;
    unsigned char version;
#if defined(CONFIG_MACH_TAC)
    unsigned short hw_handset;
    unsigned short hw_revision;
    unsigned short hw_revision_bit;

    hw_handset = shdisp_api_get_hw_handset();
    hw_revision_bit = shdisp_api_get_hw_revision();
    hw_revision = shdisp_sys_api_hw_revision_bit_conv(hw_revision_bit);
#endif
    SHDISP_TRACE("in\n");

    if (power_status == SHDISP_DEV_STATE_NOINIT) {
        SHDISP_BDIC_REGSET(shdisp_bdic_init1);
        ret = shdisp_bdic_IO_read_check_reg(BDIC_REG_VERSION, &version);
        if (ret != SHDISP_RESULT_SUCCESS) {
            return SHDISP_RESULT_FAILURE;
        }
        s_state_str.bdic_chipver = (int)SHDISP_BDIC_GET_CHIPVER(version);

        SHDISP_BDIC_REGSET(shdisp_bdic_init2);
#if defined(CONFIG_MACH_TAC)
        if (hw_handset == 0) {
            SHDISP_BDIC_REGSET(shdisp_bdic_init_no_led);
        }
        else if (hw_handset == 1) {
            if (hw_revision >= 2) {
                SHDISP_BDIC_REGSET(shdisp_bdic_init_3_led);
            } else {
                SHDISP_BDIC_REGSET(shdisp_bdic_init_1_led);
            }
        }
#endif
        SHDISP_BDIC_REGSET(shdisp_bdic_init3);
        shdisp_bdic_tri_led_before_mode = SHDISP_BDIC_TRI_LED_MODE_OFF;
    }
    SHDISP_BDIC_REGSET(shdisp_bdic_active);
    SHDISP_TRACE("out\n");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_set_standby                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_set_standby(void)
{
    SHDISP_TRACE("in\n");
    SHDISP_BDIC_REGSET(shdisp_bdic_standby);
    SHDISP_TRACE("out\n");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_BKL_control                                                */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_BKL_control(unsigned char request, int param)
{
    int ret;
    unsigned char val = 0x00;

    switch (request) {
    case SHDISP_BDIC_REQ_ACTIVE:
        break;

    case SHDISP_BDIC_REQ_STANDBY:
        break;

    case SHDISP_BDIC_REQ_BKL_ON:
        if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_OFF) {
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_on);
            ret = shdisp_bdic_IO_read_reg(BDIC_REG_GINF3, &val);
            SHDISP_DEBUG("DCDC1 Err Chk. ret=%d. val=0x%02x\n", ret, val);
            if (ret == SHDISP_RESULT_SUCCESS) {
                if ((val & SHDISP_BDIC_GINF3_DCDC1_OVD) == SHDISP_BDIC_GINF3_DCDC1_OVD) {
                    SHDISP_ERR("DCDC1_OVD bit ON.\n");
                    SHDISP_BDIC_REGSET(shdisp_bdic_dcdc1_err);
                }
            } else {
                SHDISP_ERR("BDIC GINF3 read error.\n");
            }
        }
        break;

    case SHDISP_BDIC_REQ_BKL_FIX_START:
        if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_OFF) {

        } else if (shdisp_bdic_bkl_before_mode != SHDISP_BDIC_BKL_MODE_FIX) {
            if (shdisp_bdic_dbc_mode == SHDISP_BDIC_BKL_DBC_OFF) {
                SHDISP_BDIC_REGSET(shdisp_bdic_bkl_fix);
            } else {
                SHDISP_BDIC_REGSET(shdisp_bdic_bkl_fix_dbc);
            }

        } else {
            if ((shdisp_bdic_dbc_before_mode == SHDISP_BDIC_BKL_DBC_ON) && (shdisp_bdic_dbc_mode == SHDISP_BDIC_BKL_DBC_OFF)) {
                SHDISP_BDIC_REGSET(shdisp_bdic_bkl_fix);
            } else if ((shdisp_bdic_dbc_before_mode == SHDISP_BDIC_BKL_DBC_OFF) && (shdisp_bdic_dbc_mode == SHDISP_BDIC_BKL_DBC_ON)) {
                SHDISP_BDIC_REGSET(shdisp_bdic_bkl_fix_dbc);
            }

        }
        break;

    case SHDISP_BDIC_REQ_BKL_AUTO_START:
        if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_OFF) {
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_auto_on);

        } else if (shdisp_bdic_bkl_before_mode != SHDISP_BDIC_BKL_MODE_AUTO) {
            if (shdisp_bdic_dbc_mode == SHDISP_BDIC_BKL_DBC_OFF) {
                SHDISP_BDIC_REGSET(shdisp_bdic_bkl_auto);
            } else {
                SHDISP_BDIC_REGSET(shdisp_bdic_bkl_auto_dbc);
            }

        } else {
            if ((shdisp_bdic_dbc_before_mode == SHDISP_BDIC_BKL_DBC_ON) && (shdisp_bdic_dbc_mode == SHDISP_BDIC_BKL_DBC_OFF)) {
                SHDISP_BDIC_REGSET(shdisp_bdic_bkl_auto);
            } else if ((shdisp_bdic_dbc_before_mode == SHDISP_BDIC_BKL_DBC_OFF) && (shdisp_bdic_dbc_mode == SHDISP_BDIC_BKL_DBC_ON)) {
                SHDISP_BDIC_REGSET(shdisp_bdic_bkl_auto_dbc);
            }

        }

        break;

    case SHDISP_BDIC_REQ_BKL_SET_LED_VALUE:
        if (shdisp_bdic_bkl_mode  == SHDISP_BDIC_BKL_MODE_FIX) {
            shdisp_bdic_PD_BKL_set_led_value();
        } else if (shdisp_bdic_bkl_mode  == SHDISP_BDIC_BKL_MODE_AUTO) {
            if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_OFF) {
                shdisp_SYS_delay_us(100*1000);
                shdisp_bdic_PD_BKL_set_led_value();
            }
        }
        break;

    case SHDISP_BDIC_REQ_BKL_SET_OPT_VALUE:
        shdisp_bdic_PD_BKL_set_opt_value();
        break;

    case SHDISP_BDIC_REQ_START:
        if (shdisp_bdic_bkl_mode  == SHDISP_BDIC_BKL_MODE_FIX) {
            shdisp_bdic_PD_BKL_set_led_value();
        } else if (shdisp_bdic_bkl_mode  == SHDISP_BDIC_BKL_MODE_AUTO) {
            shdisp_bdic_PD_BKL_set_opt_value();
        }
        break;

    case SHDISP_BDIC_REQ_STOP:
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_OFF;
        shdisp_bdic_bkl_param = SHDISP_MAIN_BKL_PARAM_OFF;
        SHDISP_BDIC_REGSET(shdisp_bdic_bkl_off);
        break;

    case SHDISP_BDIC_REQ_BKL_SET_MODE_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_OFF;
        shdisp_bdic_bkl_param = param;
        break;

    case SHDISP_BDIC_REQ_BKL_SET_MODE_FIX:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_FIX;
        shdisp_bdic_bkl_param = param;
        break;

    case SHDISP_BDIC_REQ_BKL_SET_MODE_AUTO:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_AUTO;
        shdisp_bdic_bkl_param_auto = param;
        break;

    case SHDISP_BDIC_REQ_BKL_DTV_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_dtv = SHDISP_BDIC_BKL_DTV_OFF;
        break;

    case SHDISP_BDIC_REQ_BKL_DTV_ON:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_dtv = SHDISP_BDIC_BKL_DTV_ON;
        break;

    case SHDISP_BDIC_REQ_BKL_EMG_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_emg = SHDISP_BDIC_BKL_EMG_OFF;
        break;

    case SHDISP_BDIC_REQ_BKL_EMG_ON:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_emg = SHDISP_BDIC_BKL_EMG_ON;
        break;

    case SHDISP_BDIC_REQ_BKL_ECO_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_eco = SHDISP_BDIC_BKL_ECO_OFF;
        break;

    case SHDISP_BDIC_REQ_BKL_ECO_ON:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_eco = SHDISP_BDIC_BKL_ECO_ON;
        break;

    case SHDISP_BDIC_REQ_BKL_CHG_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_chg = SHDISP_BDIC_BKL_CHG_OFF;
        break;

    case SHDISP_BDIC_REQ_BKL_CHG_ON:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_chg = SHDISP_BDIC_BKL_CHG_ON;
        break;

    default:
        break;
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_GPIO_control                                               */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_GPIO_control(unsigned char port, unsigned char status)
{
    unsigned char    reg;
    unsigned char    bit;

    switch (port) {
    case SHDISP_BDIC_GPIO_GPOD0:
        reg = BDIC_REG_GPIO_0;
        bit = 0x01;
        break;
    case SHDISP_BDIC_GPIO_GPOD1:
        reg = BDIC_REG_GPIO_1;
        bit = 0x01;
        break;
    case SHDISP_BDIC_GPIO_GPOD2:
        reg = BDIC_REG_GPIO_2;
        bit = 0x01;
        break;
    case SHDISP_BDIC_GPIO_GPOD3:
        reg = BDIC_REG_GPIO_3;
        bit = 0x01;
        break;
    case SHDISP_BDIC_GPIO_GPOD4:
        reg = BDIC_REG_GPIO_4;
        bit = 0x01;
        break;
    case SHDISP_BDIC_GPIO_GPOD5:
        reg = BDIC_REG_GPIO_5;
        bit = 0x01;
        break;
    default:
        return;
    }
    if (status == SHDISP_BDIC_GPIO_HIGH) {
        shdisp_bdic_IO_set_bit_reg(reg, bit);
    } else {
        shdisp_bdic_IO_clr_bit_reg(reg, bit);
    }

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_opt_th_shift                                               */
/* ------------------------------------------------------------------------- */
static unsigned char shdisp_bdic_PD_opt_th_shift(int index)
{
    unsigned char opt_th_shift[SHDISP_BKL_AUTO_OPT_TBL_NUM];
    opt_th_shift[0] = (0x07 & BDIC_REG_OPT_TH_SHIFT_1_0_VAL);
    opt_th_shift[1] = (0x70 & BDIC_REG_OPT_TH_SHIFT_1_0_VAL) >> 4;
    opt_th_shift[2] = (0x07 & BDIC_REG_OPT_TH_SHIFT_3_2_VAL);
    opt_th_shift[3] = (0x70 & BDIC_REG_OPT_TH_SHIFT_3_2_VAL) >> 4;
    opt_th_shift[4] = (0x07 & BDIC_REG_OPT_TH_SHIFT_4_5_VAL);
    opt_th_shift[5] = (0x70 & BDIC_REG_OPT_TH_SHIFT_4_5_VAL) >> 4;
    opt_th_shift[6] = (0x07 & BDIC_REG_OPT_TH_SHIFT_6_7_VAL);
    opt_th_shift[7] = (0x70 & BDIC_REG_OPT_TH_SHIFT_6_7_VAL) >> 4;
    opt_th_shift[8] = (0x07 & BDIC_REG_OPT_TH_SHIFT_8_9_VAL);
    opt_th_shift[9] = (0x70 & BDIC_REG_OPT_TH_SHIFT_8_9_VAL) >> 4;
    opt_th_shift[10] = (0x07 & BDIC_REG_OPT_TH_SHIFT_11_10_VAL);
    opt_th_shift[11] = (0x70 & BDIC_REG_OPT_TH_SHIFT_11_10_VAL) >> 4;
    opt_th_shift[12] = (0x07 & BDIC_REG_OPT_TH_SHIFT_13_12_VAL);
    opt_th_shift[13] = (0x70 & BDIC_REG_OPT_TH_SHIFT_13_12_VAL) >> 4;
    opt_th_shift[14] = (0x07 & BDIC_REG_OPT_TH_SHIFT_15_14_VAL);
    opt_th_shift[15] = (0x70 & BDIC_REG_OPT_TH_SHIFT_15_14_VAL) >> 4;
    opt_th_shift[16] = (0x07 & BDIC_REG_OPT_TH_SHIFT_17_16_VAL);
    opt_th_shift[17] = (0x70 & BDIC_REG_OPT_TH_SHIFT_17_16_VAL) >> 4;
    opt_th_shift[18] = (0x07 & BDIC_REG_OPT_TH_SHIFT_19_18_VAL);
    opt_th_shift[19] = (0x70 & BDIC_REG_OPT_TH_SHIFT_19_18_VAL) >> 4;
    opt_th_shift[20] = (0x07 & BDIC_REG_OPT_TH_SHIFT_21_20_VAL);
    opt_th_shift[21] = (0x70 & BDIC_REG_OPT_TH_SHIFT_21_20_VAL) >> 4;
    opt_th_shift[22] = (0x07 & BDIC_REG_OPT_TH_SHIFT_23_22_VAL);
    opt_th_shift[23] = (0x70 & BDIC_REG_OPT_TH_SHIFT_23_22_VAL) >> 4;
    if (index >= SHDISP_BKL_AUTO_OPT_TBL_NUM || index < 0) {
        SHDISP_ERR("invalid index\n");
    }
    return opt_th_shift[index];
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_BKL_set_led_value                                          */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_BKL_set_led_value()
{
    int i, mode = 0;
    unsigned short ado_tmp;
    unsigned long ado;
    unsigned char data;

    if (shdisp_bdic_bkl_mode == SHDISP_BDIC_BKL_MODE_OFF) {
        return;
    }

    shdisp_bdic_LD_LCD_BKL_get_mode(&mode);

    switch (shdisp_bdic_bkl_mode) {
    case SHDISP_BDIC_BKL_MODE_FIX:
        shdisp_bdic_LD_LCD_BKL_get_fix_param(mode, shdisp_bdic_bkl_param, &data );
        shdisp_bdic_bkl_led_value[0].data = data;
#ifdef CONFIG_SHDISP_USE_M2LED
        shdisp_bdic_bkl_led_value[1].data = data;
#endif
        break;
    case SHDISP_BDIC_BKL_MODE_AUTO:
        shdisp_bdic_PD_REG_ADO_get_opt(&ado_tmp);
        ado = (unsigned long)ado_tmp << 4;
        if (ado <= SHDISP_BDIC_BKL_AUTO_FIX_PARAM_MIN_ADO) {
            shdisp_bdic_LD_LCD_BKL_get_pwm_param(mode, 0, &data);
        } else {
            for (i = 0; i < SHDISP_BKL_AUTO_OPT_TBL_NUM; i++) {
                if ((ado >> shdisp_bdic_PD_opt_th_shift(i)) <= shdisp_bdic_bkl_ado_index[i]) {
                    shdisp_bdic_LD_LCD_BKL_get_pwm_param(mode, i, &data);
                    break;
                }
            }
        }
        shdisp_bdic_bkl_led_value[0].data = data;
#ifdef CONFIG_SHDISP_USE_M2LED
        shdisp_bdic_bkl_led_value[1].data = data;
#endif
        SHDISP_DEBUG("ado=0x%04X, current=%02X\n", (unsigned int)ado, shdisp_bdic_bkl_led_value[0].data);
        break;
    default:

        return;
    }

    SHDISP_BDIC_REGSET(shdisp_bdic_bkl_led_value);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_BKL_set_opt_value                                          */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_BKL_set_opt_value(void)
{
    int i;
    int mode = 0;

    unsigned char reg = BDIC_REG_ADO_INDEX;
    unsigned char ado_idx = 0x00;

    shdisp_bdic_LD_LCD_BKL_get_mode(&mode);

    switch (shdisp_bdic_bkl_mode) {
    case SHDISP_BDIC_BKL_MODE_OFF:
        break;

    case SHDISP_BDIC_BKL_MODE_FIX:
        break;

    case SHDISP_BDIC_BKL_MODE_AUTO:
        for (i = 0; i < SHDISP_BKL_AUTO_OPT_TBL_NUM; i++) {
            shdisp_bdic_LD_LCD_BKL_get_pwm_param(mode, i, &(shdisp_bdic_bkl_opt_value[i].data));
        }

        shdisp_bdic_PD_wait4i2ctimer_stop();

        if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_AUTO){
            shdisp_bdic_IO_read_reg(reg, &ado_idx);
            ado_idx = ado_idx & 0x1F;
            shdisp_bdic_bkl_led_value[0].data=shdisp_bdic_bkl_opt_value[ado_idx].data;
#ifdef CONFIG_SHDISP_USE_M2LED
            shdisp_bdic_bkl_led_value[1].data=shdisp_bdic_bkl_opt_value[ado_idx].data;
#endif
            if(ado_idx >= SHDISP_BKL_AUTO_OPT_TBL_NUM) ado_idx = SHDISP_BKL_AUTO_OPT_TBL_NUM - 1;
            shdisp_bdic_bkl_slope_fast[0].data = 0xFF & (unsigned char)slope_fast;
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_slope_fast);
        }

        SHDISP_BDIC_REGSET(shdisp_bdic_set_bank1);
        SHDISP_BDIC_REGSET(shdisp_bdic_bkl_opt_value);
        SHDISP_BDIC_REGSET(shdisp_bdic_set_bank0);

        if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_AUTO){
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_opt_mode_off);
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_led_value);
            shdisp_SYS_delay_us(1000*mled_delay_ms1);
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_opt_mode_on);
        }

        SHDISP_BDIC_REGSET(shdisp_bdic_i2ctimer_start);

        if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_AUTO){
            shdisp_SYS_delay_us(1000*mled_delay_ms2);
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_slope_slow);

        }
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_get_pwm_param                                      */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_get_pwm_param(int mode, int level, unsigned char *opt_val)
{
    unsigned long  pwm_val;
    unsigned short min_val;
    unsigned short max_val;

    min_val = shdisp_main_bkl_min_tbl[level];
    max_val = shdisp_main_bkl_max_tbl[level];

    switch (mode) {
    case SHDISP_BKL_TBL_MODE_NORMAL:
    case SHDISP_BKL_TBL_MODE_ECO:
    case SHDISP_BKL_TBL_MODE_EMERGENCY:
        if (shdisp_bdic_bkl_param_auto <= SHDISP_MAIN_BKL_PARAM_MIN_AUTO) {
            pwm_val = (unsigned long)min_val;
        } else if (shdisp_bdic_bkl_param_auto >= SHDISP_MAIN_BKL_PARAM_MAX_AUTO) {
            pwm_val = (unsigned long)max_val;
        } else {
            pwm_val = (unsigned long)((max_val - min_val) * (shdisp_bdic_bkl_param_auto - 2));
            pwm_val /= (unsigned char)SHDISP_BKL_AUTO_STEP_NUM;
            pwm_val += (unsigned long)min_val;
        }
        if ((mode == SHDISP_BKL_TBL_MODE_EMERGENCY) && (pwm_val > SHDISP_BKL_EMERGENCY_LIMIT_AUTO)) {
            pwm_val = SHDISP_BKL_EMERGENCY_LIMIT_AUTO;
        }
        break;
    case SHDISP_BKL_TBL_MODE_CHARGE:
        pwm_val = (unsigned long)max_val;
        break;
    default:
        pwm_val = (unsigned long)SHDISP_BKL_PWM_LOWER_LIMIT;
        break;
    }

    if (level == 0 && pwm_val < (unsigned long)SHDISP_BKL_PWM_OPT0_LOWER_LIMIT) {
        pwm_val = (unsigned long)SHDISP_BKL_PWM_OPT0_LOWER_LIMIT;
    } else if (level != 0 && pwm_val < (unsigned long)SHDISP_BKL_PWM_OPT1_23_LOWER_LIMIT) {
        pwm_val = (unsigned long)SHDISP_BKL_PWM_OPT1_23_LOWER_LIMIT;
    } else if (pwm_val > (unsigned long)SHDISP_BKL_PWM_UPPER_LIMIT) {
        pwm_val = (unsigned long)SHDISP_BKL_PWM_UPPER_LIMIT;
    } else {
        ;
    }

    pwm_val *= (unsigned char)SHDISP_BKL_CURRENT_UPPER_LIMIT;
    pwm_val /= (unsigned short)SHDISP_BKL_PWM_UPPER_LIMIT;
    *opt_val = (unsigned char)pwm_val;
    return;
}

#if defined(CONFIG_MACH_PA25) || defined(CONFIG_MACH_PB25) || defined(CONFIG_MACH_KF10) || defined(CONFIG_MACH_PA28)
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_control                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_TRI_LED_control(unsigned char request, int param)
{
    switch (request) {
    case SHDISP_BDIC_REQ_ACTIVE:
        break;

    case SHDISP_BDIC_REQ_STANDBY:
        break;

    case SHDISP_BDIC_REQ_START:
        SHDISP_DEBUG("SHDISP_BDIC_REQ_START.tri_led_mode=%d.led_before_mode=%d.\n"
                       ,shdisp_bdic_tri_led_mode, shdisp_bdic_tri_led_before_mode);
        SHDISP_BDIC_REGSET(shdisp_bdic_set_bank0);
        switch (shdisp_bdic_tri_led_before_mode) {
        case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
            if (shdisp_bdic_tri_led_mode != SHDISP_BDIC_TRI_LED_MODE_NORMAL) {
                SHDISP_BDIC_REGSET(shdisp_bdic_led_off_fix);
            } else {
                SHDISP_BDIC_REGSET(shdisp_bdic_led_off);
            }
            break;
        case SHDISP_BDIC_TRI_LED_MODE_OFF:
            break;
        case SHDISP_BDIC_TRI_LED_MODE_BLINK:
        case SHDISP_BDIC_TRI_LED_MODE_FIREFLY:
        case SHDISP_BDIC_TRI_LED_MODE_HISPEED:
        case SHDISP_BDIC_TRI_LED_MODE_STANDARD:
        case SHDISP_BDIC_TRI_LED_MODE_BREATH:
        case SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH:
        case SHDISP_BDIC_TRI_LED_MODE_WAVE:
        case SHDISP_BDIC_TRI_LED_MODE_FLASH:
        case SHDISP_BDIC_TRI_LED_MODE_AURORA:
        case SHDISP_BDIC_TRI_LED_MODE_RAINBOW:
        default:
            SHDISP_BDIC_REGSET(shdisp_bdic_led_off);
            break;
        }

        if (shdisp_bdic_tri_led_mode == SHDISP_BDIC_TRI_LED_MODE_NORMAL) {
            shdisp_bdic_LD_set_led_fix_on_table(s_state_str.bdic_clrvari_index, shdisp_bdic_tri_led_color);
            SHDISP_BDIC_REGSET(shdisp_bdic_led_fix_on);
        } else {
            SHDISP_BDIC_REGSET(shdisp_bdic_led_lposc_enable);
            shdisp_bdic_PD_TRI_LED_set_chdig();
            shdisp_bdic_PD_TRI_LED_set_anime();
            SHDISP_BDIC_REGSET(shdisp_bdic_led_ani_on);
        }
        shdisp_bdic_tri_led_before_mode = shdisp_bdic_tri_led_mode;
        break;

    case SHDISP_BDIC_REQ_STOP:
        SHDISP_BDIC_REGSET(shdisp_bdic_set_bank0);
        SHDISP_BDIC_REGSET(shdisp_bdic_led_off);
        shdisp_bdic_tri_led_mode        = SHDISP_BDIC_TRI_LED_MODE_NORMAL;
        shdisp_bdic_tri_led_before_mode = SHDISP_BDIC_TRI_LED_MODE_OFF;
        shdisp_bdic_tri_led_color       = 0;
        shdisp_bdic_tri_led_ontime      = 0;
        shdisp_bdic_tri_led_interval    = 0;
        shdisp_bdic_tri_led_count       = 0;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_NORMAL;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BLINK:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_BLINK;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FIREFLY:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_FIREFLY;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME:
        shdisp_bdic_tri_led_ontime = param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL:
        shdisp_bdic_tri_led_interval = param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_COUNT:
        shdisp_bdic_tri_led_count = param;
        break;

    default:
        break;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_set_anime                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_TRI_LED_set_anime(void)
{
    unsigned char timeer1_val;
    unsigned char ch_set1_val;
    unsigned char ch_set2_val;

    timeer1_val  = (unsigned char)(shdisp_bdic_tri_led_interval << 4);
    timeer1_val |= (unsigned char)(shdisp_bdic_tri_led_count & 0x07);

    switch (shdisp_bdic_tri_led_mode) {
    case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BLINK:
        ch_set1_val = 0x46;
        ch_set2_val = (unsigned char)(shdisp_bdic_tri_led_ontime);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH0_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH0_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH1_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH1_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH2_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH2_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_TIMEER, (unsigned char)timeer1_val, 0xF7);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_FIREFLY:
        ch_set1_val = 0x06;
        ch_set2_val = (unsigned char)(shdisp_bdic_tri_led_ontime);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH0_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH0_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH1_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH1_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH2_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH2_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_TIMEER, (unsigned char)timeer1_val, 0xF7);
        break;

    default:
        break;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_set_chdig                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_TRI_LED_set_chdig(void)
{
    int clrvari = s_state_str.bdic_clrvari_index;
    unsigned char wBuf[9];

    memset( wBuf, 0, sizeof( wBuf ));

    switch (shdisp_bdic_tri_led_mode) {
    case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BLINK:
    case SHDISP_BDIC_TRI_LED_MODE_FIREFLY:
        wBuf[0] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][0];
        wBuf[1] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][1];
        wBuf[2] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][2];
        wBuf[3] = shdisp_triple_led_anime_tbl[clrvari][1][shdisp_bdic_tri_led_color][0];
        wBuf[4] = shdisp_triple_led_anime_tbl[clrvari][1][shdisp_bdic_tri_led_color][1];
        wBuf[5] = shdisp_triple_led_anime_tbl[clrvari][1][shdisp_bdic_tri_led_color][2];
        wBuf[6] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][0];
        wBuf[7] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][1];
        wBuf[8] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][2];
        shdisp_bdic_IO_multi_write_reg(BDIC_REG_CH0_A, wBuf, 9);
        break;

    default:
        break;
    }

    return;
}
#endif

#if defined(CONFIG_MACH_TAC)
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_control                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_TRI_LED_control(unsigned char request, int param)
{
    switch (request) {
    case SHDISP_BDIC_REQ_ACTIVE:
        break;

    case SHDISP_BDIC_REQ_STANDBY:
        break;

    case SHDISP_BDIC_REQ_START:
        SHDISP_DEBUG("SHDISP_BDIC_REQ_START.tri_led_mode=%d.led_before_mode=%d.\n"
                       ,shdisp_bdic_tri_led_mode, shdisp_bdic_tri_led_before_mode);
        SHDISP_BDIC_REGSET(shdisp_bdic_set_bank0);
        switch (shdisp_bdic_tri_led_before_mode) {
        case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
            if (shdisp_bdic_tri_led_mode != SHDISP_BDIC_TRI_LED_MODE_NORMAL) {
                SHDISP_BDIC_REGSET(shdisp_bdic_led_off_fix);
            } else {
                SHDISP_BDIC_REGSET(shdisp_bdic_led_off);
                if(shdisp_pm_is_keyled_active() && shdisp_bdic_key_led_index <= 7) {
                    SHDISP_BDIC_REGSET(shdisp_bdic_led_lposc_disable);
                }
            }
            break;
        case SHDISP_BDIC_TRI_LED_MODE_OFF:
            break;
        case SHDISP_BDIC_TRI_LED_MODE_BLINK:
        default:
            SHDISP_BDIC_REGSET(shdisp_bdic_led_off);
            if(shdisp_pm_is_keyled_active() && shdisp_bdic_key_led_index <= 7) {
                    SHDISP_BDIC_REGSET(shdisp_bdic_led_lposc_disable);
            }
            break;
        }

        if (shdisp_bdic_tri_led_mode == SHDISP_BDIC_TRI_LED_MODE_NORMAL) {
            shdisp_bdic_LD_set_led_fix_on_table(s_state_str.bdic_clrvari_index, shdisp_bdic_tri_led_color);
            SHDISP_BDIC_REGSET(shdisp_bdic_led_fix_on);
        } else {
            SHDISP_BDIC_REGSET(shdisp_bdic_led_lposc_enable);
            shdisp_bdic_PD_TRI_LED_set_chdig();
            shdisp_bdic_PD_TRI_LED_set_anime();
            SHDISP_BDIC_REGSET(shdisp_bdic_led_ani_on);
        }
        shdisp_bdic_tri_led_before_mode = shdisp_bdic_tri_led_mode;
        break;

    case SHDISP_BDIC_REQ_STOP:
        SHDISP_BDIC_REGSET(shdisp_bdic_set_bank0);
        SHDISP_BDIC_REGSET(shdisp_bdic_led_off);
        if(shdisp_pm_is_keyled_active() && shdisp_bdic_key_led_index <= 7) {
            SHDISP_BDIC_REGSET(shdisp_bdic_led_lposc_disable);
        }
        shdisp_bdic_tri_led_mode        = SHDISP_BDIC_TRI_LED_MODE_NORMAL;
        shdisp_bdic_tri_led_before_mode = SHDISP_BDIC_TRI_LED_MODE_OFF;
        shdisp_bdic_tri_led_color       = 0;
        shdisp_bdic_tri_led_ontime      = 0;
        shdisp_bdic_tri_led_interval    = 0;
        shdisp_bdic_tri_led_count       = 0;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_NORMAL;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BLINK:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_BLINK;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME:
        shdisp_bdic_tri_led_ontime = param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL:
        shdisp_bdic_tri_led_interval = param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_COUNT:
        shdisp_bdic_tri_led_count = param;
        break;

    default:
        break;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_set_anime                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_TRI_LED_set_anime(void)
{
    unsigned char timeer1_val;
    unsigned char ch_set1_val;
    unsigned char ch_set2_val;

    timeer1_val  = (unsigned char)(shdisp_bdic_tri_led_interval << 4);
    timeer1_val |= (unsigned char)(shdisp_bdic_tri_led_count & 0x07);

    switch (shdisp_bdic_tri_led_mode) {
    case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BLINK:
        ch_set1_val = 0x46;
        ch_set2_val = (unsigned char)(shdisp_bdic_tri_led_ontime);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH3_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH3_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH4_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH4_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH5_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH5_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_TIMER2, (unsigned char)timeer1_val, 0xF7);
        break;

    default:
        break;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_set_chdig                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_TRI_LED_set_chdig(void)
{
    int clrvari = s_state_str.bdic_clrvari_index;
    unsigned char wBuf[9];

    memset( wBuf, 0, sizeof( wBuf ));

    switch (shdisp_bdic_tri_led_mode) {
    case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BLINK:
        wBuf[0] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][0];
        wBuf[0] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][0];
        wBuf[1] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][1];
        wBuf[2] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][2];
        wBuf[3] = shdisp_triple_led_anime_tbl[clrvari][1][shdisp_bdic_tri_led_color][0];
        wBuf[4] = shdisp_triple_led_anime_tbl[clrvari][1][shdisp_bdic_tri_led_color][1];
        wBuf[5] = shdisp_triple_led_anime_tbl[clrvari][1][shdisp_bdic_tri_led_color][2];
        wBuf[6] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][0];
        wBuf[7] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][1];
        wBuf[8] = shdisp_triple_led_anime_tbl[clrvari][0][shdisp_bdic_tri_led_color][2];
        shdisp_bdic_IO_multi_write_reg(BDIC_REG_CH3_A, wBuf, 9);
        break;

    default:
        break;
    }

    return;
}
#endif

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_KEY_LED_control                                            */
/* ------------------------------------------------------------------------- */
#if defined(CONFIG_MACH_TAC)
static void shdisp_bdic_PD_KEY_LED_control(unsigned char dim, unsigned char index, int ontime, int interval)
{
    shdisp_bdic_key_led_index = index;

    SHDISP_DEBUG("in before_index:%d, index:%d\n", shdisp_bdic_key_led_before_index, shdisp_bdic_key_led_index);

    if(shdisp_bdic_key_led_index > 7) {
        shdisp_bdic_key_led_ontime   = ontime;
        shdisp_bdic_key_led_interval = interval;
    } else {
        shdisp_bdic_key_led_ontime   = 0;
        shdisp_bdic_key_led_interval = 0;
    }

    if(shdisp_bdic_key_led_before_index == shdisp_bdic_key_led_index) {
    return;
    }

    if(shdisp_bdic_key_led_index == 0) {
        SHDISP_BDIC_REGSET(shdisp_bdic_set_bank0);
        SHDISP_BDIC_REGSET(shdisp_bdic_key_led_off);
        if(shdisp_pm_is_led_active() && shdisp_bdic_tri_led_mode != SHDISP_BDIC_TRI_LED_MODE_BLINK) {
            SHDISP_BDIC_REGSET(shdisp_bdic_led_lposc_disable);
        }
    } else {

        SHDISP_BDIC_REGSET(shdisp_bdic_set_bank0);

        if(shdisp_bdic_key_led_before_index == 0) {
            //nothing
        } else if (shdisp_bdic_key_led_before_index <=7 && shdisp_bdic_key_led_index > 7) {
            SHDISP_BDIC_REGSET(shdisp_bdic_key_led_off_fix);
        } else {
            SHDISP_BDIC_REGSET(shdisp_bdic_key_led_off);
            if(shdisp_pm_is_led_active() && shdisp_bdic_tri_led_mode != SHDISP_BDIC_TRI_LED_MODE_BLINK) {
                SHDISP_BDIC_REGSET(shdisp_bdic_led_lposc_disable);
            }
        }

        if(shdisp_bdic_key_led_index <= 7) {
            shdisp_bdic_PD_KEY_LED_set_chdig(dim, shdisp_bdic_key_led_index);
            SHDISP_BDIC_REGSET(shdisp_bdic_key_led_fix_on);
        } else {
            SHDISP_BDIC_REGSET(shdisp_bdic_led_lposc_enable);
            shdisp_bdic_PD_KEY_LED_set_chdig(dim, shdisp_bdic_key_led_index);
            SHDISP_BDIC_REGSET(shdisp_bdic_key_led_ani_on);
        }
    }

    shdisp_bdic_key_led_before_index = shdisp_bdic_key_led_index;

    return;
}
#endif

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_KEY_LED_set_chdig                                          */
/* ------------------------------------------------------------------------- */
#if defined(CONFIG_MACH_TAC)
static void shdisp_bdic_PD_KEY_LED_set_chdig(unsigned char dim, unsigned char index)
{
    unsigned char key_left_mode = shdisp_key_led_index_tbl[index].key_left;
    unsigned char key_center_mode = shdisp_key_led_index_tbl[index].key_center;
    unsigned char key_right_mode = shdisp_key_led_index_tbl[index].key_right;
    unsigned char wBuf[9];
    unsigned char timeer1_val = 0;
    unsigned char ch_set1_val = 0;
    unsigned char ch_set2_val = 0;

    if (dim & (1<<SHDISP_KEY_BKL_LEFT)) {
        key_left_mode = SHDISP_KEY_BKL_DIM;
    }
    if (dim & (1<<SHDISP_KEY_BKL_CENTER)) {
        key_center_mode = SHDISP_KEY_BKL_DIM;
    }
    if (dim & (1<<SHDISP_KEY_BKL_RIGHT)) {
        key_right_mode = SHDISP_KEY_BKL_DIM;
    }

    memset( wBuf, 0, sizeof( wBuf ));

    if(key_left_mode == SHDISP_KEY_BKL_OFF) {
        wBuf[0] = 0;
        wBuf[3] = 0;
        wBuf[6] = 0;
    } else if (key_left_mode == SHDISP_KEY_BKL_NORMAL) {
        wBuf[0] = shdisp_key_led_tbl[SHDISP_KEY_BKL_LEFT][SHDISP_KEY_BKL_NORMAL];
        wBuf[3] = 0;
        wBuf[6] = 0;
    } else if (key_left_mode == SHDISP_KEY_BKL_BLINK) {
        wBuf[0] = 0;
        wBuf[3] = shdisp_key_led_tbl[SHDISP_KEY_BKL_LEFT][SHDISP_KEY_BKL_BLINK];
        wBuf[6] = 0;
    } else { /* key_left_mode = SHDISP_KEY_BKL_DIM */
        wBuf[0] = shdisp_key_led_tbl[SHDISP_KEY_BKL_LEFT][SHDISP_KEY_BKL_DIM];
        wBuf[3] = 0;
        wBuf[6] = 0;
    }

    if(key_center_mode == SHDISP_KEY_BKL_OFF) {
        wBuf[1] = 0;
        wBuf[4] = 0;
        wBuf[7] = 0;
    } else if (key_center_mode == SHDISP_KEY_BKL_NORMAL) {
        wBuf[1] = shdisp_key_led_tbl[SHDISP_KEY_BKL_CENTER][SHDISP_KEY_BKL_NORMAL];
        wBuf[4] = 0;
        wBuf[7] = 0;
    } else if (key_center_mode == SHDISP_KEY_BKL_BLINK) {
        wBuf[1] = 0;
        wBuf[4] = shdisp_key_led_tbl[SHDISP_KEY_BKL_CENTER][SHDISP_KEY_BKL_BLINK];
        wBuf[7] = 0;
    } else { /* key_center_mode = SHDISP_KEY_BKL_DIM */
        wBuf[1] = shdisp_key_led_tbl[SHDISP_KEY_BKL_CENTER][SHDISP_KEY_BKL_DIM];
        wBuf[4] = 0;
        wBuf[7] = 0;
    }

    if(key_right_mode == SHDISP_KEY_BKL_OFF) {
        wBuf[2] = 0;
        wBuf[5] = 0;
        wBuf[8] = 0;
    } else if (key_right_mode == SHDISP_KEY_BKL_NORMAL) {
        wBuf[2] = shdisp_key_led_tbl[SHDISP_KEY_BKL_RIGHT][SHDISP_KEY_BKL_NORMAL];
        wBuf[5] = 0;
        wBuf[8] = 0;
    } else if (key_right_mode == SHDISP_KEY_BKL_BLINK) {
        wBuf[2] = 0;
        wBuf[5] = shdisp_key_led_tbl[SHDISP_KEY_BKL_RIGHT][SHDISP_KEY_BKL_BLINK];
        wBuf[8] = 0;
    } else { /* key_right_mode = SHDISP_KEY_BKL_DIM */
        wBuf[2] = shdisp_key_led_tbl[SHDISP_KEY_BKL_RIGHT][SHDISP_KEY_BKL_DIM];
        wBuf[5] = 0;
        wBuf[8] = 0;
    }

    shdisp_bdic_IO_multi_write_reg(BDIC_REG_CH0_A, wBuf, 9);

    if(index > 7) {
        timeer1_val  = (unsigned char)(shdisp_bdic_key_led_interval << 4);
        timeer1_val |= (unsigned char)(0x00 & 0x07);
        ch_set1_val = 0x46;
        ch_set2_val = (unsigned char)(shdisp_bdic_key_led_ontime);
    }

    if(key_left_mode == SHDISP_KEY_BKL_BLINK) {
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH0_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH0_SET2, ch_set2_val);
    }  else {
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH0_SET1, 0x20, 0x20);
    }

    if(key_center_mode == SHDISP_KEY_BKL_BLINK) {
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH1_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH1_SET2, ch_set2_val);
    } else {
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH1_SET1, 0x20, 0x20);
    }

    if(key_right_mode == SHDISP_KEY_BKL_BLINK) {
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH2_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH2_SET2, ch_set2_val);
    } else {
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH2_SET1, 0x20, 0x20);
    }

    if(index > 7) {
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_TIMEER, (unsigned char)timeer1_val, 0xF7);
    }

    return;
}
#endif

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_get_sensor_state                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_get_sensor_state(void)
{
    int ret;
    unsigned char reg = SENSOR_REG_COMMAND1;
    unsigned char val = 0x00;
    SHDISP_TRACE("in\n");

    ret = shdisp_photo_sensor_IO_read_reg(reg, &val);

    if (ret != SHDISP_RESULT_SUCCESS) {
        val = 0x00;
    }

#if defined(CONFIG_ANDROID_ENGINEERING)
    if (shdisp_dbg_get_recovery_check_error() == SHDISP_DBG_RECOVERY_ERROR_PSALS) {
        shdisp_dbg_update_recovery_check_error(SHDISP_DBG_RECOVERY_ERROR_NONE);
        SHDISP_DEBUG("force sensor state error.\n");
        val = 0x00;
    }
#endif /* CONFIG_ANDROID_ENGINEERING */

    SHDISP_TRACE("out\n");
    return val;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_wait4i2ctimer_stop                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_wait4i2ctimer_stop(void)
{
    int waitcnt = 3;
    int ret;
    unsigned char reg = BDIC_REG_SYSTEM8;
    unsigned char val = 0x00;

    SHDISP_BDIC_REGSET(shdisp_bdic_i2ctimer_stop);

    do {
        shdisp_SYS_delay_us(BDIC_WAIT_TIMER_STOP);
        ret = shdisp_bdic_IO_read_reg(reg, &val);
        SHDISP_DEBUG("retry(%d)!! SYSTEM8 = 0x%02x\n", waitcnt, val);
        if (ret != SHDISP_RESULT_SUCCESS) {
            continue;
        }

        if (val != 0xA0) {
            continue;
        }

        return SHDISP_RESULT_SUCCESS;
    } while (0 < --waitcnt);

    SHDISP_ERR("i2ctimer wait failed.\n");
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_i2c_throughmode_ctrl                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_i2c_throughmode_ctrl(bool ctrl)
{

    if (ctrl) {
        shdisp_bdic_PD_wait4i2ctimer_stop();
        SHDISP_BDIC_REGSET(shdisp_bdic_i2c_throughmode_on);
    } else {
        SHDISP_BDIC_REGSET(shdisp_bdic_i2c_throughmode_off);
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_slave_transfer                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_slave_transfer(struct shdisp_bdic_i2c_msg *msg)
{
    int ret;
    switch (msg->mode) {
    case SHDISP_BDIC_I2C_M_W:
        ret = shdisp_SYS_Host_i2c_send(msg->addr, (unsigned char*)msg->wbuf, msg->wlen);
        break;
    case SHDISP_BDIC_I2C_M_R:
        ret = shdisp_SYS_Host_i2c_recv(msg->addr, (unsigned char*)msg->wbuf, msg->wlen, msg->rbuf, msg->rlen);
        break;
    case SHDISP_BDIC_I2C_M_R_MODE1:
    case SHDISP_BDIC_I2C_M_R_MODE2:
    case SHDISP_BDIC_I2C_M_R_MODE3:
        ret = SHDISP_RESULT_SUCCESS;
        break;
    default:
        SHDISP_ERR("<INVALID_VALUE> msg->mode(%d).\n", msg->mode);
        ret = SHDISP_RESULT_FAILURE;
        break;
    }
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_power_on                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_power_on(void)
{
    int sensor_state;
    int i;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    SHDISP_TRACE("in\n");

    SHDISP_BDIC_REGSET(shdisp_bdic_sensor_power_on);
    SHDISP_BDIC_REGSET(shdisp_bdic_psals_init);

    sensor_state = shdisp_bdic_PD_get_sensor_state();
    if (sensor_state == 0x00) {
        SHDISP_ERR("psals poweron(first) failed!! sensor_state = %02x\n",sensor_state);
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PSALS;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_PSALS_ON_NG;
        shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */

        for (i = 0; i < 3; i++) {
            SHDISP_BDIC_REGSET(shdisp_bdic_psals_init);

            sensor_state = shdisp_bdic_PD_get_sensor_state();
            if (sensor_state != 0x00) {
                break;
            }
            SHDISP_WARN("try psals poweron failed(%d)!! sensor_state = %02x\n", i+1,sensor_state);
        }

        if (i == 3) {
            SHDISP_ERR("psals poweron retry over!!\n");
            if (psals_recovery_flag == SHDISP_BDIC_PSALS_RECOVERY_DURING) {
                psals_recovery_flag = SHDISP_BDIC_PSALS_RECOVERY_RETRY_OVER;
            }
#ifdef SHDISP_RESET_LOG
            err_code.mode = SHDISP_DBG_MODE_LINUX;
            err_code.type = SHDISP_DBG_TYPE_PSALS;
            err_code.code = SHDISP_DBG_CODE_RETRY_OVER;
            err_code.subcode = SHDISP_DBG_SUBCODE_POWER_ON_NG;
            shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
        }
    }

    SHDISP_BDIC_REGSET(shdisp_bdic_psals_set);

    shdisp_bdic_PD_wait4i2ctimer_stop();

    SHDISP_BDIC_REGSET(shdisp_bdic_psals_standby);

    SHDISP_TRACE("out\n");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_power_off                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_power_off(void)
{
    SHDISP_TRACE("in\n");
    SHDISP_BDIC_REGSET(shdisp_bdic_sensor_power_off);
    psals_recovery_flag = SHDISP_BDIC_PSALS_RECOVERY_NONE;
    SHDISP_TRACE("out\n");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_ps_init_als_off                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_ps_init_als_off(void)
{
    int ret;

    SHDISP_TRACE("in\n");

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_init_als_off1);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1\n");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_init_als_off2);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2\n");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_init_als_off3);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out3\n");
        return ret;
    }

    SHDISP_TRACE("out\n");
    return ret;
}
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_ps_init_als_on                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_ps_init_als_on(void)
{
    int ret;

    SHDISP_TRACE("in\n");

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_init_als_on1);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1\n");
        return ret;
    }

    ret = shdisp_bdic_PD_wait4i2ctimer_stop();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2\n");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_init_als_on2);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out3\n");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_init_als_on3);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out4\n");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_init_als_on4);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out5\n");
        return ret;
    }

    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_ps_deinit_als_off                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_ps_deinit_als_off(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_deinit_als_off1);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1");
        return ret;
    }

    ret = shdisp_bdic_PD_wait4i2ctimer_stop();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_deinit_als_off2);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out3");
        return ret;
    }
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_ps_deinit_als_on                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_ps_deinit_als_on(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_deinit_als_on1);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1\n");
        return ret;
    }

    ret = shdisp_bdic_PD_wait4i2ctimer_stop();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_deinit_als_on2);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out3\n");
        return ret;
    }
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_als_init_ps_off                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_als_init_ps_off(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = SHDISP_BDIC_REGSET(shdisp_bdic_als_init_ps_off);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out\n");
        return ret;
    }
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_als_init_ps_on                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_als_init_ps_on(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = SHDISP_BDIC_REGSET(shdisp_bdic_als_init_ps_on1);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1");
        return ret;
    }

    ret = shdisp_bdic_PD_wait4i2ctimer_stop();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_als_init_ps_on2);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out3");
        return ret;
    }
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_als_deinit_ps_off                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_als_deinit_ps_off(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = SHDISP_BDIC_REGSET(shdisp_bdic_als_deinit_ps_off1);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1");
        return ret;
    }

    ret = shdisp_bdic_PD_wait4i2ctimer_stop();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_als_deinit_ps_off2);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out3");
        return ret;
    }
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_als_deinit_ps_on                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_als_deinit_ps_on(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = SHDISP_BDIC_REGSET(shdisp_bdic_als_deinit_ps_on1);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1");
        return ret;
    }

    ret = shdisp_bdic_PD_wait4i2ctimer_stop();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_als_deinit_ps_on2);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out3");
        return ret;
    }
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_write_threshold                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_write_threshold(struct shdisp_prox_params *prox_params)
{
    if (!prox_params) {
        return SHDISP_RESULT_FAILURE;
    }
    shdisp_bdic_ps_init_set_threshold[0].data = (unsigned char)(prox_params->threshold_low & 0x00FF);
    shdisp_bdic_ps_init_set_threshold[1].data = (unsigned char)((prox_params->threshold_low >> 8) & 0x00FF);
    shdisp_bdic_ps_init_set_threshold[2].data = (unsigned char)(prox_params->threshold_high & 0x00FF);
    shdisp_bdic_ps_init_set_threshold[3].data = (unsigned char)((prox_params->threshold_high >> 8) & 0x00FF);
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_REG_ADO_get_opt                                            */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_PD_REG_ADO_get_opt(unsigned short *value)
{
    int i, ret,shift_tmp;
    unsigned long ado0, ado1;
    unsigned short als0, als1;
    unsigned short alpha, beta, gamma;
    unsigned char rval[(SENSOR_REG_D1_MSB+1)-SENSOR_REG_D0_LSB];
    signed char range0, range1,res;

    SHDISP_TRACE("in\n");

    shdisp_bdic_PD_i2c_throughmode_ctrl(true);
    ret = shdisp_photo_sensor_IO_read_reg(SENSOR_REG_COMMAND2, rval);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2");
        goto i2c_err;
    }
    range0 = rval[0] & 0x07;

    for (i = 0; i < SHDISP_BDIC_GET_ADO_RETRY_TIMES; i++) {
        ret = shdisp_photo_sensor_IO_burst_read_reg(SENSOR_REG_D0_LSB, rval, sizeof(rval));
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("out3");
            goto i2c_err;
        }
        als0 = (rval[1] << 8 | rval[0]);
        als1 = (rval[3] << 8 | rval[2]);

        ret = shdisp_photo_sensor_IO_read_reg(SENSOR_REG_COMMAND2, rval);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("out4");
            goto i2c_err;
        }
        range1 = rval[0] & 0x07;
        res = (rval[0] & 0x38) >> 3;
        SHDISP_DEBUG("i=%d, range0=%d, range1=%d\n", i, range0, range1);
        if (range0 == range1) {
            break;
        }
        range0 = range1;
    }
    SHDISP_BDIC_REGSET(shdisp_bdic_reg_ar_ctrl);
    shdisp_bdic_PD_i2c_throughmode_ctrl(false);
    shdisp_SYS_delay_us(1000);

    SHDISP_DEBUG("als1*16=%d, als0*15=%d\n", als1 * SHDISP_BDIC_RATIO_OF_ALS0, als0 * SHDISP_BDIC_RATIO_OF_ALS1);

    if ((als1 * SHDISP_BDIC_RATIO_OF_ALS0) > (als0 * SHDISP_BDIC_RATIO_OF_ALS1)) {
        alpha = s_state_str.photo_sensor_adj.als_adjust[1].als_adj0;
        beta  = s_state_str.photo_sensor_adj.als_adjust[1].als_adj1;
        gamma = s_state_str.photo_sensor_adj.als_adjust[1].als_shift;
        if (gamma < 16) {
            ado0 = ((als0 * alpha - als1 * beta) << gamma) >> 15;
        } else {
            ado0 = ((als0 * alpha - als1 * beta) << (32-gamma)) >> 15;
        }
        SHDISP_DEBUG("ROUTE-1 als0=%04X, als1=%04X, alpha=%04X, gamma=%d\n", als0, als1, alpha, gamma);
    } else {
        alpha = s_state_str.photo_sensor_adj.als_adjust[0].als_adj0;
        beta  = s_state_str.photo_sensor_adj.als_adjust[0].als_adj1;
        gamma = s_state_str.photo_sensor_adj.als_adjust[0].als_shift;
        if (gamma < 16) {
            ado0 = ((als0 * alpha - als1 * beta) << gamma) >> 15;
        } else {
            ado0 = ((als0 * alpha - als1 * beta) << (32-gamma)) >> 15;
        }
        SHDISP_DEBUG("ROUTE-2 als0=%04X, als1=%04X, alpha=%04X, beta=%04X, gamma=%d\n", als0, als1, alpha, beta, gamma);
    }

    if (res < 3){
        shift_tmp = res - 3 + range0 -4;
    }else{
        shift_tmp = (res - 3)*2 + range0 -4;
    }

    if(shift_tmp < 0){
        shift_tmp = -1*shift_tmp;
        ado1 = ado0 >> shift_tmp;
    }else{
        ado1 = ado0 << shift_tmp;
    }

    if (ado1 > SHDISP_BDIC_MAX_ADO_VALUE) {
        ado1 = SHDISP_BDIC_MAX_ADO_VALUE;
    }
    *value = (unsigned short)ado1;

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;

i2c_err:
    shdisp_bdic_PD_i2c_throughmode_ctrl(false);
    return SHDISP_RESULT_FAILURE;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_REG_RAW_DATA_get_opt                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_REG_RAW_DATA_get_opt(unsigned short *clear, unsigned short *ir)
{
    int i, ret;
    unsigned short als0, als1;
    unsigned char rval[(SENSOR_REG_D1_MSB+1)-SENSOR_REG_D0_LSB];
    signed char range0, range1;

    SHDISP_TRACE("in\n");

    ret = shdisp_bdic_PD_wait4i2ctimer_stop();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1");
        return;
    }

    shdisp_bdic_PD_i2c_throughmode_ctrl(true);
    ret = shdisp_photo_sensor_IO_read_reg(SENSOR_REG_COMMAND2, rval);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2");
        return;
    }
    range0 = rval[0] & 0x07;

    for (i = 0; i < SHDISP_BDIC_GET_ADO_RETRY_TIMES; i++) {
        ret = shdisp_photo_sensor_IO_burst_read_reg(SENSOR_REG_D0_LSB, rval, sizeof(rval));
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("out3");
            return;
        }
        als0 = (rval[1] << 8 | rval[0]);
        als1 = (rval[3] << 8 | rval[2]);

        ret = shdisp_photo_sensor_IO_read_reg(SENSOR_REG_COMMAND2, rval);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("out4");
            return;
        }
        range1 = rval[0] & 0x07;
        SHDISP_DEBUG("i=%d, range0=%d, range1=%d\n", i, range0, range1);
        if (range0 == range1) {
            break;
        }
        range0 = range1;
    }

    shdisp_bdic_PD_i2c_throughmode_ctrl(false);
    shdisp_SYS_delay_us(1000);
    *clear = als0;
    *ir = als1;

    SHDISP_DEBUG("als0=0x%04X, als1=0x%04X\n", (unsigned int)als0, (unsigned int)als1);

    SHDISP_TRACE("out\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_get_ave_ado                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_get_ave_ado(struct shdisp_ave_ado *ave_ado)
{
    int i, ret;
    unsigned long als0 = 0;
    unsigned long als1 = 0;
    unsigned long ado  = 0;
    unsigned char rvalL, rvalH;

    SHDISP_TRACE("in\n");

    ret = shdisp_bdic_PD_wait4i2ctimer_stop();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1");
        return ret;
    }

    SHDISP_BDIC_BACKUP_REGS_BDIC(shdisp_bdic_restore_regs_for_ave_ado_bdic);
    SHDISP_BDIC_BACKUP_REGS_ALS(shdisp_bdic_restore_regs_for_ave_ado_als);

    SHDISP_BDIC_REGSET(shdisp_bdic_ave_ado_param1);
    shdisp_bdic_ave_ado_param2[2].data = (0x18 | (ave_ado->als_range & 0x07));
    if (shdisp_bdic_restore_regs_for_ave_ado_als[1].data != shdisp_bdic_ave_ado_param2[2].data) {
        SHDISP_BDIC_REGSET(shdisp_bdic_ave_ado_param2);
    }
    SHDISP_BDIC_REGSET(shdisp_bdic_i2ctimer_start_ave_ado);

    for (i = 0; i < SHDISP_BDIC_AVE_ADO_READ_TIMES; i++) {
        SHDISP_BDIC_REGSET(shdisp_bdic_i2ctimer_stop_ave_ado);

        shdisp_bdic_IO_read_reg(BDIC_REG_I2C_RDATA0, &rvalL);
        shdisp_bdic_IO_read_reg(BDIC_REG_I2C_RDATA1, &rvalH);
        als0 += (unsigned long)((rvalH << 8) | rvalL);

        shdisp_bdic_IO_read_reg(BDIC_REG_I2C_RDATA2, &rvalL);
        shdisp_bdic_IO_read_reg(BDIC_REG_I2C_RDATA3, &rvalH);
        als1 += (unsigned long)((rvalH << 8) | rvalL);

        shdisp_bdic_IO_read_reg(BDIC_REG_ADOL, &rvalL);
        shdisp_bdic_IO_read_reg(BDIC_REG_ADOH, &rvalH);
        ado  += (unsigned long)((rvalH << 8) | rvalL);

        SHDISP_BDIC_REGSET(shdisp_bdic_i2ctimer_start_ave_ado);
    }

    ret = shdisp_bdic_PD_wait4i2ctimer_stop();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1");
        return ret;
    }

    SHDISP_BDIC_RESTORE_REGS(shdisp_bdic_restore_regs_for_ave_ado_bdic);
    SHDISP_BDIC_RESTORE_REGS(shdisp_bdic_restore_regs_for_ave_ado_als);
    SHDISP_BDIC_REGSET(shdisp_bdic_i2ctimer_start_ave_ado);

    ave_ado->ave_als0 = (unsigned short)(als0 / SHDISP_BDIC_AVE_ADO_READ_TIMES);
    ave_ado->ave_als1 = (unsigned short)(als1 / SHDISP_BDIC_AVE_ADO_READ_TIMES);
    ave_ado->ave_ado  = (unsigned short)(ado  / SHDISP_BDIC_AVE_ADO_READ_TIMES);
    SHDISP_DEBUG("ave_als0:0x%04X ave_als1:0x%04X ave_ado:0x%04X\n", ave_ado->ave_als0, ave_ado->ave_als1, ave_ado->ave_ado);

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* Input/Output                                                              */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_write_reg                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_write_reg(unsigned char reg, unsigned char val)
{
    int ret;

    if (s_state_str.bdic_is_exist != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("<ACCESS_ERR> \n");
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_SYS_bdic_i2c_write(reg, val);

    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    } else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_write.\n");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_write.\n");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_multi_write_reg                                            */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_multi_write_reg(unsigned char reg, unsigned char *wval, unsigned char size)
{
    int ret;

    if (s_state_str.bdic_is_exist != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("<ACCESS_ERR> \n");
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_SYS_bdic_i2c_multi_write(reg, wval, size);

    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    } else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_multi_write.\n");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_multi_write.\n");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_read_reg                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_read_reg(unsigned char reg, unsigned char *val)
{
    int ret;

    if (val == NULL) {
        return SHDISP_RESULT_FAILURE;
    }

    if (s_state_str.bdic_is_exist != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("<ACCESS_ERR> \n");
        return SHDISP_RESULT_SUCCESS;
    }

    if ((reg == BDIC_REG_TEST_B3)
    ||  (reg == BDIC_REG_SYSTEM8)) {
        ret = shdisp_bdic_IO_read_no_check_reg(reg, val);
    } else {
        ret = shdisp_bdic_IO_read_check_reg(reg, val);
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_read_no_check_reg                                          */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_read_no_check_reg(unsigned char reg, unsigned char *val)
{
    int ret;

    ret = shdisp_SYS_bdic_i2c_read(reg, val);
    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    } else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_read_check_reg                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_read_check_reg(unsigned char reg, unsigned char *val)
{
    int ret;
    int retry = 0;
    unsigned char try_1st, try_2nd;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    try_1st = 0;
    try_2nd = 0;

    for (retry = 0; retry < 3; retry++) {
        ret = shdisp_SYS_bdic_i2c_read(reg, &try_1st);

        if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
            SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE_I2C_TMO;
        } else if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE;
        }

        ret = shdisp_SYS_bdic_i2c_read(reg, &try_2nd);

        if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
            SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE_I2C_TMO;
        } else if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE;
        }

        if (try_1st == try_2nd) {
            *val = try_1st;
            return SHDISP_RESULT_SUCCESS;
        } else if (retry == 2) {
            SHDISP_ERR("<OTHER> i2c read retry over! addr:0x%02X val:(1st:0x%02X, 2nd:0x%02X).\n",
                                                                            reg, try_1st, try_2nd);
#ifdef SHDISP_RESET_LOG
            err_code.mode = SHDISP_DBG_MODE_LINUX;
            err_code.type = SHDISP_DBG_TYPE_BDIC;
            err_code.code = SHDISP_DBG_CODE_RETRY_OVER;
            err_code.subcode = SHDISP_DBG_SUBCODE_I2C_READ;
            shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
            *val = try_1st;
            return SHDISP_RESULT_SUCCESS;
        } else {
            SHDISP_WARN("<OTHER> i2c read retry (%d)! addr:0x%02X val:(1st:0x%02X, 2nd:0x%02X).\n",
                                                                            retry, reg, try_1st, try_2nd);
        }
    }

    SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_read.\n");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_multi_read_reg                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_multi_read_reg(unsigned char reg, unsigned char *val, int size)
{
    int ret;
    int maxreg;

    if (val == NULL) {
        SHDISP_ERR("<NULL_POINTER> val.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if ((size < 1) || (size > 8)) {
        SHDISP_ERR("<INVALID_VALUE> size(%d).\n", size);
        return SHDISP_RESULT_FAILURE;
    }

    maxreg = (int)reg + (size - 1);
    if (maxreg > BDIC_REG_BANKSEL) {
        SHDISP_ERR("<OTHER> register address overflow.\n");
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_SYS_bdic_i2c_multi_read(reg, val, size);

    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    } else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_multi_read.\n");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_multi_read.\n");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_set_bit_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_set_bit_reg(unsigned char reg, unsigned char val)
{
    int ret;

    ret = shdisp_SYS_bdic_i2c_mask_write(reg, val, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_clr_bit_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_clr_bit_reg(unsigned char reg, unsigned char val)
{
    int ret;

    ret = shdisp_SYS_bdic_i2c_mask_write(reg, 0x00, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_bank_set                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_IO_bank_set(unsigned char val)
{
    int ret;

    ret = shdisp_bdic_IO_write_reg(BDIC_REG_BANKSEL, val);
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_msk_bit_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char msk)
{
    int ret;

    ret = shdisp_SYS_bdic_i2c_mask_write(reg, val, msk);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_photo_sensor_IO_write_reg                                          */
/* ------------------------------------------------------------------------- */

static int shdisp_photo_sensor_IO_write_reg(unsigned char reg, unsigned char val)
{
    int ret = SHDISP_RESULT_SUCCESS;
    struct shdisp_bdic_i2c_msg msg;
    unsigned char wbuf[2];

    wbuf[0] = reg;
    wbuf[1] = val;

    msg.addr = SHDISP_BDIC_SENSOR_SLAVE_ADDR;
    msg.mode = SHDISP_BDIC_I2C_M_W;
    msg.wlen = 2;
    msg.rlen = 0;
    msg.wbuf = &wbuf[0];
    msg.rbuf = NULL;

    ret = shdisp_bdic_PD_slave_transfer(&msg);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_photo_sensor_IO_read_reg                                           */
/* ------------------------------------------------------------------------- */

static int shdisp_photo_sensor_IO_read_reg(unsigned char reg, unsigned char *val)
{
    int ret = SHDISP_RESULT_SUCCESS;
    struct shdisp_bdic_i2c_msg msg;
    unsigned char wbuf[1];
    unsigned char rbuf[1];

    wbuf[0] = reg;
    rbuf[0] = 0x00;

    msg.addr = SHDISP_BDIC_SENSOR_SLAVE_ADDR;
    msg.mode = SHDISP_BDIC_I2C_M_R;
    msg.wlen = 1;
    msg.rlen = 1;
    msg.wbuf = &wbuf[0];
    msg.rbuf = &rbuf[0];

    ret = shdisp_bdic_PD_slave_transfer(&msg);
    if (ret == SHDISP_RESULT_SUCCESS) {
        *val = rbuf[0];
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_phote_sensor_IO_msk_bit_reg                                        */
/* ------------------------------------------------------------------------- */

static int shdisp_phote_sensor_IO_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char mask)
{
    int ret;
    unsigned char rval;
    unsigned char wval;

    ret = shdisp_photo_sensor_IO_read_reg(reg, &rval);
    if (ret != SHDISP_RESULT_SUCCESS) {
        return ret;
    }

    wval = (rval & ~mask) | (val & mask);

    if (rval == wval) {
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_photo_sensor_IO_write_reg(reg, wval);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_photo_sensor_IO_burst_write_reg                                    */
/* ------------------------------------------------------------------------- */

static int shdisp_photo_sensor_IO_burst_write_reg(unsigned char *wval, unsigned char dataNum)
{
    int ret = SHDISP_RESULT_SUCCESS;
    struct shdisp_bdic_i2c_msg msg;

    msg.addr = SHDISP_BDIC_SENSOR_SLAVE_ADDR;
    msg.mode = SHDISP_BDIC_I2C_M_W;
    msg.wlen = dataNum;
    msg.rlen = 0;
    msg.wbuf = wval;
    msg.rbuf = NULL;
    ret = shdisp_bdic_PD_slave_transfer(&msg);

    return ret;
}
/* ------------------------------------------------------------------------- */
/* shdisp_photo_sensor_IO_burst_read_reg                                     */
/* ------------------------------------------------------------------------- */

static int shdisp_photo_sensor_IO_burst_read_reg(unsigned char reg, unsigned char *rval, unsigned char dataNum)
{
    int ret = SHDISP_RESULT_SUCCESS;
    struct shdisp_bdic_i2c_msg msg;
    unsigned char wbuf[1];

    wbuf[0] = reg;
    msg.addr = SHDISP_BDIC_SENSOR_SLAVE_ADDR;
    msg.mode = SHDISP_BDIC_I2C_M_R;
    msg.wlen = 1;
    msg.rlen = dataNum;
    msg.wbuf = &wbuf[0];
    msg.rbuf = rval;

    ret = shdisp_bdic_PD_slave_transfer(&msg);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_probe                                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_probe(struct platform_device *pdev)
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
            shdisp_SYS_set_irq_port(res->start, pdev);
        }
    }

probe_done:
    SHDISP_TRACE("out rc = %d\n", rc );

    return rc;
#else
    return 0;
#endif /* CONFIG_OF */
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_remove                                                        */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_remove(struct platform_device *pdev)
{
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id shdisp_bdic_dt_match[] = {
    { .compatible = "sharp,shdisp_bdic",},
    {}
};
#else
#define shdisp_bdic_dt_match NULL
#endif /* CONFIG_OF */

static struct platform_driver shdisp_bdic_driver = {
    .probe = shdisp_bdic_probe,
    .remove = shdisp_bdic_remove,
    .shutdown = NULL,
    .driver = {
        /*
         * Driver name must match the device name added in
         * platform.c.
         */
        .name = "shdisp_bdic",
        .of_match_table = shdisp_bdic_dt_match,
    },
};

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_register_driver                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_register_driver(void)
{
    return platform_driver_register(&shdisp_bdic_driver);
}

MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
