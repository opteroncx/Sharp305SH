/* drivers/sharp/shdisp/shdisp_pm.c  (Display Driver)
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
#include <linux/spi/spi.h>
#include <linux/pwm.h>
#include <linux/mfd/pm8xxx/pwm.h>
#include <mach/gpio.h>
#include <mach/msm_iomap.h>
#include <asm/io.h>


#include <sharp/shdisp_kerl.h>

#include "shdisp_system.h"
#include "shdisp_bdic.h"
#include "shdisp_pm.h"
#include "shdisp_dbg.h"
/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_NAME "shdisp"
/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
struct shdisp_psals_status {
    int power_status;
    int als_mode;
    unsigned long users;
};
struct shdisp_pm_context {
    struct shdisp_bdic_status bdic_status;
    struct shdisp_psals_status psals_status;
};
static struct shdisp_pm_context shdisp_pm_ctx;
static struct shdisp_pm_context shdisp_pm_ctx_recovery;

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static int  shdisp_pm_bdic_set_active(void);
static int  shdisp_pm_bdic_set_standby(void);
static int  shdisp_pm_psals_power_on(void);
static int  shdisp_pm_psals_power_off(void);
static int  shdisp_pm_psals_ps_init(void);
static int  shdisp_pm_psals_ps_deinit(void);
static int  shdisp_pm_psals_als_init(void);
static int  shdisp_pm_psals_als_deinit(void);
extern void shdisp_psals_recovery_subscribe( void );
extern void shdisp_psals_recovery_unsubscribe(void);

/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_pm_init                                                            */
/* ------------------------------------------------------------------------- */
void shdisp_pm_init(struct shdisp_boot_context *shdisp_boot_ctx)
{
    memcpy(&(shdisp_pm_ctx.bdic_status), &(shdisp_boot_ctx->bdic_status), sizeof(struct shdisp_bdic_status));

    shdisp_pm_ctx.psals_status.power_status = SHDISP_SENSOR_STATE_POWER_OFF;
    shdisp_pm_ctx.psals_status.als_mode     = SHDISP_BDIC_MAIN_BKL_OPT_LOW;
    shdisp_pm_ctx.psals_status.users        = SHDISP_DEV_TYPE_NONE;

    shdisp_pm_ctx_recovery.psals_status.power_status = SHDISP_SENSOR_STATE_POWER_OFF;
    shdisp_pm_ctx_recovery.psals_status.users        = SHDISP_DEV_TYPE_NONE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_bdic_power_manager                                              */
/* ------------------------------------------------------------------------- */
int shdisp_pm_bdic_power_manager(int type, int state)
{
    int ret;
    unsigned long users_wk;

    if (shdisp_pm_ctx.bdic_status.bdic_is_exist != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("bdic does not exist.\n");
        return SHDISP_RESULT_SUCCESS;
    }

    SHDISP_TRACE("in type:0x%08X, state:%s, bdic_status:%s, users:0x%08X\n",
            type, ((state == SHDISP_DEV_STATE_OFF) ? "off":"on"),
            (shdisp_pm_ctx.bdic_status.power_status ?
            ((shdisp_pm_ctx.bdic_status.power_status == SHDISP_DEV_STATE_OFF) ? "standby":"active") : "noinit"),
            (int)shdisp_pm_ctx.bdic_status.users
        );

    if (shdisp_pm_ctx.bdic_status.power_status != state) {
        if (state == SHDISP_DEV_STATE_ON) {
            ret = shdisp_pm_bdic_set_active();
            if (ret != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_bdic_set_active.\n");
                return SHDISP_RESULT_FAILURE;
            }
            shdisp_pm_ctx.bdic_status.power_status = state;
        } else {
            users_wk = shdisp_pm_ctx.bdic_status.users;
            users_wk &= (unsigned long)(~(type & SHDISP_DEV_TYPE_MASK));
            if (users_wk == SHDISP_DEV_TYPE_NONE) {
                ret = shdisp_pm_bdic_set_standby();
                shdisp_pm_ctx.bdic_status.power_status = state;
                if (ret != SHDISP_RESULT_SUCCESS) {
                    SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_bdic_set_standby.\n");
                    return SHDISP_RESULT_FAILURE;
                }
            }
        }
    }
    if (state == SHDISP_DEV_STATE_ON) {
        shdisp_pm_ctx.bdic_status.users |= (unsigned long)(type & SHDISP_DEV_TYPE_MASK);
    } else {
        shdisp_pm_ctx.bdic_status.users &= (unsigned long)(~(type & SHDISP_DEV_TYPE_MASK));
    }

    SHDISP_TRACE("out bdic_status:%s, users:0x%08X\n",
            (shdisp_pm_ctx.bdic_status.power_status ?
            ((shdisp_pm_ctx.bdic_status.power_status == SHDISP_DEV_STATE_OFF) ? "standby":"active") : "noinit"),
            (int)shdisp_pm_ctx.bdic_status.users
        );

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_power_manager                                             */
/* ------------------------------------------------------------------------- */
int shdisp_pm_psals_power_manager(int type, int state)
{
    int ret;
    int before_state;
    unsigned long als_users_wk, users_wk;

    SHDISP_TRACE("in type:0x%08X, state:%s, psals_status:%s, users:0x%08X\n",
            type, ((state == SHDISP_DEV_STATE_OFF) ? "off":"on"),
            (shdisp_pm_ctx.psals_status.power_status ?
            ((shdisp_pm_ctx.psals_status.power_status == SHDISP_DEV_STATE_OFF) ? "standby":"active") : "noinit"),
            (int)shdisp_pm_ctx.psals_status.users
        );

    if (shdisp_pm_ctx.bdic_status.bdic_is_exist != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("bdic does not exist.\n");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_pm_ctx.bdic_status.power_status == SHDISP_DEV_STATE_OFF) {
        SHDISP_ERR("cannot sensor %s because bdic power off\n", ((state == SHDISP_DEV_STATE_OFF) ? "off":"on"));
        return SHDISP_RESULT_FAILURE;
    }

    if ((type != SHDISP_DEV_TYPE_PS) &&
        ((type & SHDISP_DEV_TYPE_ALS_MASK) == SHDISP_DEV_TYPE_NONE)) {
        return SHDISP_RESULT_FAILURE;
    }

    als_users_wk = (unsigned long)(shdisp_pm_ctx.psals_status.users & SHDISP_DEV_TYPE_ALS_MASK);
    als_users_wk &= (unsigned long)(~(type & SHDISP_DEV_TYPE_ALS_MASK));

    switch (shdisp_pm_ctx.psals_status.power_status) {
    case SHDISP_SENSOR_STATE_POWER_OFF:
        before_state = SHDISP_DEV_STATE_OFF;
        break;
    case SHDISP_SENSOR_STATE_POWER_ON:
        before_state = SHDISP_DEV_STATE_INIT;
        break;
    default:
        before_state = SHDISP_DEV_STATE_ON;
        break;
    }
    if (before_state != state) {
        if ((state == SHDISP_DEV_STATE_ON) && (before_state == SHDISP_DEV_STATE_INIT)) {
            ;
        } else if ((state == SHDISP_DEV_STATE_INIT) && (before_state == SHDISP_DEV_STATE_ON)) {
            ;
        } else if ((state == SHDISP_DEV_STATE_ON) || (state == SHDISP_DEV_STATE_INIT)) {
            ret = shdisp_pm_psals_power_on();
            if (ret != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_psals_power_on.\n");
                return SHDISP_RESULT_FAILURE;
            }
            shdisp_pm_ctx.psals_status.power_status = SHDISP_SENSOR_STATE_POWER_ON;
#ifndef SHDISP_USE_LEDC
            if (shdisp_pm_ctx_recovery.psals_status.users == SHDISP_DEV_TYPE_NONE) {
                shdisp_psals_recovery_subscribe();
            }
#endif /* SHDISP_USE_LEDC */
        } else {
            if (type == SHDISP_DEV_TYPE_PS) {
                shdisp_pm_psals_ps_deinit();
            } else {
                if (als_users_wk == SHDISP_DEV_TYPE_NONE) {
                    shdisp_pm_psals_als_deinit();
                }
            }
            users_wk = shdisp_pm_ctx.psals_status.users;
            users_wk &= (unsigned long)(~(type & SHDISP_DEV_TYPE_MASK));
            if (users_wk == SHDISP_DEV_TYPE_NONE) {
                ret = shdisp_pm_psals_power_off();
                if (ret != SHDISP_RESULT_SUCCESS) {
                    SHDISP_ERR("<RESULT_FAILURE> shdisp_pm_psals_power_off.\n");
                    return SHDISP_RESULT_FAILURE;
                }
                shdisp_pm_ctx.psals_status.power_status = SHDISP_SENSOR_STATE_POWER_OFF;
#ifndef SHDISP_USE_LEDC
                if (shdisp_pm_ctx_recovery.psals_status.users == SHDISP_DEV_TYPE_NONE) {
                    shdisp_psals_recovery_unsubscribe();
                }
#endif /* SHDISP_USE_LEDC */
            }
        }
    }

    if ((state == SHDISP_DEV_STATE_INIT) || (state == SHDISP_DEV_STATE_ON)) {
        shdisp_pm_ctx.psals_status.users |= (unsigned long)(type & SHDISP_DEV_TYPE_MASK);
        if (state == SHDISP_DEV_STATE_ON) {
            if (type == SHDISP_DEV_TYPE_PS) {
                shdisp_pm_psals_ps_init();
            } else {
                shdisp_pm_psals_als_init();
            }
        }
    } else {
        shdisp_pm_ctx.psals_status.users &= (unsigned long)(~(type & SHDISP_DEV_TYPE_MASK));
    }

    SHDISP_TRACE("out psals_status:%s, users:0x%08X\n",
            ((shdisp_pm_ctx.psals_status.power_status) ? "active":"standby"),
            (int)shdisp_pm_ctx.psals_status.users
        );

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_bdic_set_active                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_bdic_set_active(void)
{
    int ret;

    SHDISP_TRACE("in bdic_status:%d\n", shdisp_pm_ctx.bdic_status.power_status);
    ret = shdisp_bdic_API_set_active(shdisp_pm_ctx.bdic_status.power_status);
    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pm_bdic_set_standby                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_bdic_set_standby(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_API_set_standby();
    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_power_on                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_power_on(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_API_psals_power_on();
    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_power_off                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_power_off(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_API_psals_power_off();
    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_ps_init                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_ps_init(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int state = shdisp_pm_ctx.psals_status.power_status;

    SHDISP_TRACE("in\n");

    switch (state) {
    case SHDISP_SENSOR_STATE_POWER_ON:
        ret = shdisp_bdic_API_psals_ps_init_als_off();
        if (ret == SHDISP_RESULT_SUCCESS) {
            shdisp_pm_ctx.psals_status.power_status= SHDISP_SENSOR_STATE_PROX_ON_ALS_OFF;
        }
        break;
    case SHDISP_SENSOR_STATE_PROX_ON_ALS_OFF:
        break;
    case SHDISP_SENSOR_STATE_PROX_OFF_ALS_ON:
        ret = shdisp_bdic_API_psals_ps_init_als_on();
        if (ret == SHDISP_RESULT_SUCCESS) {
            shdisp_pm_ctx.psals_status.power_status = SHDISP_SENSOR_STATE_PROX_ON_ALS_ON;
        }
        break;
    case SHDISP_SENSOR_STATE_PROX_ON_ALS_ON:
        break;
    default:
        SHDISP_ERR("<RESULT_FAILURE> invalid state %d.\n", state);
        ret = SHDISP_RESULT_FAILURE;
        break;
    }

    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_ps_deinit                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_ps_deinit(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int state = shdisp_pm_ctx.psals_status.power_status;

    SHDISP_TRACE("in\n");

    switch (state) {
    case SHDISP_SENSOR_STATE_POWER_ON:
        break;
    case SHDISP_SENSOR_STATE_PROX_ON_ALS_OFF:
        ret = shdisp_bdic_API_psals_ps_deinit_als_off();
        if (ret == SHDISP_RESULT_SUCCESS) {
            shdisp_pm_ctx.psals_status.power_status = SHDISP_SENSOR_STATE_POWER_ON;
        }
        break;
    case SHDISP_SENSOR_STATE_PROX_OFF_ALS_ON:
        break;
    case SHDISP_SENSOR_STATE_PROX_ON_ALS_ON:
        ret = shdisp_bdic_API_psals_ps_deinit_als_on();
        if (ret == SHDISP_RESULT_SUCCESS) {
            shdisp_pm_ctx.psals_status.power_status = SHDISP_SENSOR_STATE_PROX_OFF_ALS_ON;
        }
        break;
    default:
        SHDISP_ERR("<RESULT_FAILURE> invalid state %d.\n", state);
        ret = SHDISP_RESULT_FAILURE;
        break;
    }

    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_als_init                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_als_init(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int state = shdisp_pm_ctx.psals_status.power_status;

    SHDISP_TRACE("in\n");

    switch (state) {
    case SHDISP_SENSOR_STATE_POWER_ON:
        ret = shdisp_bdic_API_psals_als_init_ps_off();
        if (ret == SHDISP_RESULT_SUCCESS) {
            shdisp_pm_ctx.psals_status.power_status = SHDISP_SENSOR_STATE_PROX_OFF_ALS_ON;
        }
        break;
    case SHDISP_SENSOR_STATE_PROX_ON_ALS_OFF:
        ret = shdisp_bdic_API_psals_als_init_ps_on();
        if (ret == SHDISP_RESULT_SUCCESS) {
            shdisp_pm_ctx.psals_status.power_status = SHDISP_SENSOR_STATE_PROX_ON_ALS_ON;
        }
        break;
    case SHDISP_SENSOR_STATE_PROX_OFF_ALS_ON:
    case SHDISP_SENSOR_STATE_PROX_ON_ALS_ON:
        break;
    default:
        SHDISP_ERR("<RESULT_FAILURE> invalid state %d.\n", state);
        ret = SHDISP_RESULT_FAILURE;
        break;
    }

    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;

}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_als_deinit                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_pm_psals_als_deinit(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int state = shdisp_pm_ctx.psals_status.power_status;

    SHDISP_TRACE("in\n");

    switch (state) {
    case SHDISP_SENSOR_STATE_POWER_ON:
    case SHDISP_SENSOR_STATE_PROX_ON_ALS_OFF:
        break;
    case SHDISP_SENSOR_STATE_PROX_OFF_ALS_ON:
        ret = shdisp_bdic_API_psals_als_deinit_ps_off();
        if (ret == SHDISP_RESULT_SUCCESS) {
            shdisp_pm_ctx.psals_status.power_status = SHDISP_SENSOR_STATE_POWER_ON;
        }
        break;
    case SHDISP_SENSOR_STATE_PROX_ON_ALS_ON:
        ret = shdisp_bdic_API_psals_als_deinit_ps_on();
        if (ret == SHDISP_RESULT_SUCCESS) {
            shdisp_pm_ctx.psals_status.power_status = SHDISP_SENSOR_STATE_PROX_ON_ALS_OFF;
        }
        break;
    default:
        SHDISP_ERR("<RESULT_FAILURE> invalid state %d.\n", state);
        ret = SHDISP_RESULT_FAILURE;
        break;
    }

    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_bdic_shutdown                                                   */
/* ------------------------------------------------------------------------- */

void shdisp_pm_bdic_shutdown(void)
{
    shdisp_pm_ctx_recovery.bdic_status.power_status = shdisp_pm_ctx.bdic_status.power_status;
    shdisp_pm_ctx_recovery.bdic_status.users        = shdisp_pm_ctx.bdic_status.users;

    shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_MASK, SHDISP_DEV_STATE_OFF);

    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_BL_RST_N, SHDISP_GPIO_CTL_LOW);
    shdisp_SYS_delay_us(15000);

    shdisp_pm_ctx.bdic_status.power_status = SHDISP_DEV_STATE_NOINIT;
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_bdic_resume                                                     */
/* ------------------------------------------------------------------------- */
int  shdisp_pm_bdic_resume(void)
{

    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_BL_RST_N, SHDISP_GPIO_CTL_HIGH);
    shdisp_SYS_delay_us(1000);

    shdisp_pm_bdic_power_manager(shdisp_pm_ctx_recovery.bdic_status.users, SHDISP_DEV_STATE_ON);

    shdisp_pm_ctx_recovery.bdic_status.power_status = SHDISP_DEV_STATE_NOINIT;
    shdisp_pm_ctx_recovery.bdic_status.users        = SHDISP_DEV_TYPE_NONE;

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_error_power_off                                           */
/* ------------------------------------------------------------------------- */
void shdisp_pm_psals_error_power_off(void)
{
    SHDISP_TRACE("in\n");
    shdisp_pm_ctx_recovery.psals_status.power_status = shdisp_pm_ctx.psals_status.power_status;
    shdisp_pm_ctx_recovery.psals_status.users        = shdisp_pm_ctx.psals_status.users;

    shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_ALS_MASK, SHDISP_DEV_STATE_OFF);
    shdisp_pm_psals_power_manager(SHDISP_DEV_TYPE_PS, SHDISP_DEV_STATE_OFF);

    shdisp_pm_ctx.psals_status.power_status = SHDISP_SENSOR_STATE_POWER_OFF;
    SHDISP_TRACE("out\n");
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_psals_error_power_recovery                                      */
/* ------------------------------------------------------------------------- */
int shdisp_pm_psals_error_power_recovery(void)
{
    int ret;
    unsigned long user;

    SHDISP_TRACE("in\n");
    user = shdisp_pm_ctx_recovery.psals_status.users & SHDISP_DEV_TYPE_PS;
    if (user == SHDISP_DEV_TYPE_PS) {
        ret = shdisp_pm_psals_power_manager(user, SHDISP_DEV_STATE_ON);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG("out ret = SHDISP_RESULT_FAILURE\n");
            return SHDISP_RESULT_FAILURE;
        }
    }

    user = shdisp_pm_ctx_recovery.psals_status.users & SHDISP_DEV_TYPE_ALS_MASK;
    if (user != SHDISP_DEV_TYPE_NONE) {
        ret = shdisp_pm_psals_power_manager(user, SHDISP_DEV_STATE_ON);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG("out ret = SHDISP_RESULT_FAILURE\n");
            return SHDISP_RESULT_FAILURE;
        }
    }

    shdisp_pm_ctx_recovery.psals_status.power_status = SHDISP_SENSOR_STATE_POWER_OFF;
    shdisp_pm_ctx_recovery.psals_status.users        = SHDISP_DEV_TYPE_NONE;

    SHDISP_TRACE("out ret = SHDISP_RESULT_SUCCESS\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_is_psals_active                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_pm_is_psals_active(void)
{
    int dev_state;

    switch (shdisp_pm_ctx.psals_status.power_status) {
    case SHDISP_SENSOR_STATE_POWER_OFF:
        dev_state = SHDISP_DEV_STATE_OFF;
        break;
    default:
        dev_state = SHDISP_DEV_STATE_ON;
        break;
    }
    return dev_state;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_is_als_active                                                   */
/* ------------------------------------------------------------------------- */
int shdisp_pm_is_als_active(void)
{
    int dev_state;

    switch (shdisp_pm_ctx.psals_status.power_status) {
    case SHDISP_SENSOR_STATE_PROX_OFF_ALS_ON:
    case SHDISP_SENSOR_STATE_PROX_ON_ALS_ON:
        dev_state = SHDISP_DEV_STATE_ON;
        break;
    default:
        dev_state = SHDISP_DEV_STATE_OFF;
        break;
    }
    return dev_state;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_is_ps_active                                                    */
/* ------------------------------------------------------------------------- */
int shdisp_pm_is_ps_active(void)
{
    int dev_state;

    switch (shdisp_pm_ctx.psals_status.power_status) {
    case SHDISP_SENSOR_STATE_PROX_ON_ALS_OFF:
    case SHDISP_SENSOR_STATE_PROX_ON_ALS_ON:
        dev_state = SHDISP_DEV_STATE_ON;
        break;
    default:
        dev_state = SHDISP_DEV_STATE_OFF;
        break;
    }
    return dev_state;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_is_led_active                                                   */
/* ------------------------------------------------------------------------- */
int shdisp_pm_is_led_active(void)
{
    unsigned long user;

    user = shdisp_pm_ctx.bdic_status.users & SHDISP_DEV_TYPE_LED;
    if (user == SHDISP_DEV_TYPE_NONE) {
        return SHDISP_DEV_STATE_OFF;
    }
    return SHDISP_DEV_STATE_ON;
}

/* ------------------------------------------------------------------------- */
/* shdisp_pm_is_keyled_active                                                */
/* ------------------------------------------------------------------------- */
int shdisp_pm_is_keyled_active(void)
{
    unsigned long user;

    user = shdisp_pm_ctx.bdic_status.users & SHDISP_DEV_TYPE_KEYLED;
    if (user == SHDISP_DEV_TYPE_NONE) {
        return SHDISP_DEV_STATE_OFF;
    }
    return SHDISP_DEV_STATE_ON;
}

#if defined (CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_pm_power_manager_users_dump                                        */
/* ------------------------------------------------------------------------- */
void shdisp_pm_power_manager_users_dump(void)
{
    printk("[SHDISP] psals_status:%s, als_status:%s, ps_status:%s, users:0x%08X, status:S%d\n",
            ((shdisp_pm_ctx.psals_status.power_status) ? "active":"standby"),
            ((shdisp_pm_is_als_active() == SHDISP_DEV_STATE_ON) ? "active":"standby"),
            ((shdisp_pm_is_ps_active() == SHDISP_DEV_STATE_ON) ? "active":"standby"),
            (int)shdisp_pm_ctx.psals_status.users,
            (shdisp_pm_ctx.psals_status.power_status + 1)
        );
    printk("[SHDISP] bdic_status:%s, users:0x%08X\n",
            (shdisp_pm_ctx.bdic_status.power_status ?
            ((shdisp_pm_ctx.bdic_status.power_status == SHDISP_DEV_STATE_OFF) ? "standby":"active") : "noinit"),
            (int)shdisp_pm_ctx.bdic_status.users
        );
}
#endif /* CONFIG_ANDROID_ENGINEERING */

MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");
/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
