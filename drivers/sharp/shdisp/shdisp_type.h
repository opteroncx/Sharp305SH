/* drivers/sharp/shdisp/shdisp_type.h  (Display Driver)
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

#ifndef SHDISP_TYPE_H
#define SHDISP_TYPE_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */



/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* STRUCTURE                                                                 */
/* ------------------------------------------------------------------------- */

struct shdisp_queue_data_t {
    int                 irq_GFAC;
    struct list_head    list;
};


/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

enum {
    SHDISP_MAIN_DISP_OFF,
    SHDISP_MAIN_DISP_ON,
    NUM_SHDISP_MAIN_DISP_STATUS
};

enum {
    SHDISP_DRIVER_IS_NOT_INITIALIZED,
    SHDISP_DRIVER_IS_INITIALIZED,
    NUM_SHDISP_DRIVER_STATUS
};

enum {
    SHDISP_START_DISPLAY_ON,
    SHDISP_START_DISPLAY_OFF
};

enum {
    SHDISP_THREAD_IS_NOT_ALIVE,
    SHDISP_THREAD_IS_ALIVE,
    NUM_SHDISP_THREAD_STATUS
};

enum {
    SHDISP_DTV_OFF,
    SHDISP_DTV_1SEG_ON,
    NUM_SHDISP_DTV_STATUS
};

enum {
    SHDISP_SUBSCRIBE_TYPE_INT,
    SHDISP_SUBSCRIBE_TYPE_POL,
    NUM_SHDISP_SUBSCRIBE_TYPE
};

enum {
    SHDISP_DEBUG_PROCESS_STATE_OUTPUT,
    SHDISP_DEBUG_TRACE_LOG_SWITCH,
    SHDISP_DEBUG_BDIC_I2C_WRITE,
    SHDISP_DEBUG_BDIC_I2C_READ,
    SHDISP_DEBUG_PROX_SENSOR_CTL,
    SHDISP_DEBUG_BKL_CTL,
    SHDISP_DEBUG_IRQ_LOGIC_CHK = 10,
    SHDISP_DEBUG_BDIC_IRQ_ALL_CLEAR = 11,
    SHDISP_DEBUG_BDIC_IRQ_CLEAR = 12,
    SHDISP_DEBUG_DUMMY_SUBSCRIBE = 13,
    SHDISP_DEBUG_DUMMY_UNSUBSCRIBE_PS = 14,
    SHDISP_DEBUG_DUMMY_UNSUBSCRIBE_ALS = 15,
    SHDISP_DEBUG_LUX_REGISTER_CHANGE = 16,
    SHDISP_DEBUG_XEN_SENSOR_CLEAR_WAIT_TIME = 17,
    SHDISP_DEBUG_ADO_REG_WRITE_PROTECT = 18,
    SHDISP_DEBUG_BDIC_WRITE = 20,
    SHDISP_DEBUG_BDIC_READ = 21,
    SHDISP_DEBUG_RGB_LED = 25,
    SHDISP_DEBUG_LED_REG_DUMP = 26,
    SHDISP_DEBUG_BDIC_RESTART = 27,
    SHDISP_DEBUG_MIPI_TX_FREQ_CHG = 40,
    SHDISP_DEBUG_DISPLAYLOG_ERROR_LOG_TEST = 45,
    SHDISP_DEBUG_DISPLAYLOG_SUMMARY_TEST = 46,
    SHDISP_DEBUG_CHARGE_BLK_MODE = 51,
    SHDISP_DEBUG_ECO_BLK_MODE = 52,
    SHDISP_DEBUG_EMG_BLK_MODE = 53,
    SHDISP_DEBUG_DBC_ACC = 55,
    SHDISP_DEBUG_RECOVERY_NG = 60,
    SHDISP_DEBUG_DSI_WRITE = 90,
    SHDISP_DEBUG_DSI_READ = 91,
};

enum {
    SHDISP_DEBUG_INFO_TYPE_BOOT,
    SHDISP_DEBUG_INFO_TYPE_KERNEL,
    SHDISP_DEBUG_INFO_TYPE_BDIC,
    SHDISP_DEBUG_INFO_TYPE_SENSOR,
    SHDISP_DEBUG_INFO_TYPE_POWERON,
    SHDISP_DEBUG_INFO_TYPE_PANEL,
    SHDISP_DEBUG_INFO_TYPE_PM,
    SHDISP_DEBUG_INFO_TYPE_BDIC_OPT,
    NUM_SHDISP_DEBUG_INFO_TYPE
};

enum {
    SHDISP_LUX_CHANGE_STATE_INIT,
    SHDISP_LUX_CHANGE_STATE_WAIT,
    SHDISP_LUX_CHANGE_STATE_WAKEUP,
    SHDISP_LUX_CHANGE_STATE_EXIT,
    NUM_SHDISP_LUX_CHANGE_STATE
};

enum {
    SHDISP_BKL_MODE_OFF,
    SHDISP_BKL_MODE_ON,
    SHDISP_BKL_MODE_AUTO,
    NUM_SHDISP_BKL_MODE
};

enum {
    SHDISP_ALS_IRQ_SUBSCRIBE_TYPE_BKL_CTRL,
    SHDISP_ALS_IRQ_SUBSCRIBE_TYPE_DBC_IOCTL,
    NUM_SHDISP_ALS_IRQ_SUBSCRIBE_TYPE
};

#endif /* SHDISP_TYPE_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
