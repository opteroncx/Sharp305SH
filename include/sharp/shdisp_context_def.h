/* include/sharp/shdisp_context_def.h  (Display Driver)
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

#define SHDISP_LCDC_EWB_TBL_SIZE            256
#define SHDISP_NOOS_RESET_NUM 3

#if defined(CONFIG_SHDISP_PANEL_CARIN) || defined(USER_CONFIG_SHDISP_PANEL_CARIN)
    #define SHDISP_PANEL_GAMMA_TBL_SIZE             30
#elif defined(CONFIG_SHDISP_PANEL_KITE) || defined(USER_CONFIG_SHDISP_PANEL_KITE)
    #define SHDISP_PANEL_GAMMA_TBL_SIZE             24
#else
    #define SHDISP_PANEL_GAMMA_TBL_SIZE             24
#endif

#define SHDISP_ARGC_MBR_NUM                 3
#define SHDISP_ARGC_STAGE_NUM               16
#define SHDISP_IGC_LUT_ENTRIES              256

struct shdisp_dbg_error_code {
    unsigned char mode;
    unsigned char type;
    unsigned char code;
    unsigned char subcode;
};

struct shdisp_main_bkl_ctl {
    int mode;
    int param;
};

struct shdisp_main_bkl_auto {
    int mode;
    int param;
};

struct shdisp_tri_led {
    unsigned long red;
    unsigned long green;
    unsigned long blue;
    int ext_mode;
    int led_mode;
    int ontime;
    int interval;
    int count;
};

struct shdisp_key_bkl_ctl {
    unsigned char key_left;
    unsigned char key_center;
    unsigned char key_right;
    int ontime;
    int interval;
};

struct shdisp_als_adjust {
    unsigned short als_adj0;
    unsigned short als_adj1;
    unsigned char als_shift;
    unsigned char clear_offset;
    unsigned char ir_offset;
};

struct shdisp_photo_sensor_adj {
    unsigned char status;
    unsigned char key_backlight;
    unsigned long chksum;
    struct shdisp_als_adjust als_adjust[2];
};

struct shdisp_ledc_req {
    unsigned long red[2];
    unsigned long green[2];
    unsigned long blue[2];
    int led_mode;
    int on_count;
};

struct shdisp_ledc_status {
    int ledc_is_exist;
    int power_status;
    struct shdisp_ledc_req ledc_req;
};

struct shdisp_bdic_status {
    int bdic_is_exist;
    int bdic_chipver;
    int power_status;
    unsigned long users;
};

#if defined(CONFIG_SHDISP_PANEL_CARIN) || defined(USER_CONFIG_SHDISP_PANEL_CARIN)
    #define SHDISP_LCDDR_PHY_GAMMA_BUF_MAX          SHDISP_PANEL_GAMMA_TBL_SIZE
    #define SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE       9
#elif defined(CONFIG_SHDISP_PANEL_KITE) || defined(USER_CONFIG_SHDISP_PANEL_KITE)
    #define SHDISP_LCDDR_PHY_GAMMA_BUF_MAX          (SHDISP_PANEL_GAMMA_TBL_SIZE * 3)
    #define SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE       8
#else
    #define SHDISP_LCDDR_PHY_GAMMA_BUF_MAX          (SHDISP_PANEL_GAMMA_TBL_SIZE * 3)
    #define SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE       9
#endif

struct shdisp_lcddr_phy_gamma_reg {
#if defined(CONFIG_SHDISP_PANEL_CARIN) || defined(USER_CONFIG_SHDISP_PANEL_CARIN)
    unsigned char  status;
    unsigned char  buf[SHDISP_LCDDR_PHY_GAMMA_BUF_MAX];
    unsigned char  applied_voltage[SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE];
    unsigned short chksum;
#elif defined(CONFIG_SHDISP_PANEL_KITE) || defined(USER_CONFIG_SHDISP_PANEL_KITE)
    unsigned char  status;
    unsigned char  buf[SHDISP_LCDDR_PHY_GAMMA_BUF_MAX];
    unsigned char  applied_voltage[SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE];
    unsigned short chksum;
#else
    unsigned char  status;
    unsigned char  buf[SHDISP_LCDDR_PHY_GAMMA_BUF_MAX];
    unsigned char  applied_voltage[SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE];
    unsigned short chksum;
#endif
};

struct dma_abl_color {
    unsigned char blue;
    unsigned char green;
    unsigned char red;
};

struct shdisp_argc_lut {
    unsigned short  red[SHDISP_ARGC_STAGE_NUM][SHDISP_ARGC_MBR_NUM];
    unsigned short  green[SHDISP_ARGC_STAGE_NUM][SHDISP_ARGC_MBR_NUM];
    unsigned short  blue[SHDISP_ARGC_STAGE_NUM][SHDISP_ARGC_MBR_NUM];
};

struct shdisp_igc_lut {
    unsigned char   r_data[SHDISP_IGC_LUT_ENTRIES];
    unsigned char   g_data[SHDISP_IGC_LUT_ENTRIES];
    unsigned char   b_data[SHDISP_IGC_LUT_ENTRIES];
};

enum {
    SHDISP_MAIN_DISP_DRIVE_FREQ_DEFAULT,
    SHDISP_MAIN_DISP_DRIVE_FREQ_TYPE_A,
    SHDISP_MAIN_DISP_DRIVE_FREQ_TYPE_B,
    SHDISP_MAIN_DISP_DRIVE_FREQ_TYPE_C,
    NUM_SHDISP_MAIN_DISP_DRIVE_FREQ_TYPE
};

enum {
    SHDISP_UPPER_UNIT_IS_NOT_CONNECTED,
    SHDISP_UPPER_UNIT_IS_CONNECTED,
    NUM_UPPER_UNIT_STATUS
};

enum {
    SHDISP_BDIC_IS_NOT_EXIST,
    SHDISP_BDIC_IS_EXIST,
    NUM_BDIC_EXIST_STATUS
};

enum {
    SHDISP_MAIN_BKL_MODE_OFF,
    SHDISP_MAIN_BKL_MODE_FIX,
    SHDISP_MAIN_BKL_MODE_AUTO,
    SHDISP_MAIN_BKL_MODE_AUTO_ECO,
    SHDISP_MAIN_BKL_MODE_DTV_OFF,
    SHDISP_MAIN_BKL_MODE_DTV_FIX,
    SHDISP_MAIN_BKL_MODE_DTV_AUTO,
    NUM_SHDISP_MAIN_BKL_MODE
};

enum {
    SHDISP_MAIN_BKL_PARAM_0 = 0,
    SHDISP_MAIN_BKL_PARAM_1 = 1,
    SHDISP_MAIN_BKL_PARAM_2 = 2,
    SHDISP_MAIN_BKL_PARAM_3 = 14,
    SHDISP_MAIN_BKL_PARAM_4 = 26,
    SHDISP_MAIN_BKL_PARAM_5 = 39,
    SHDISP_MAIN_BKL_PARAM_6 = 52,
    SHDISP_MAIN_BKL_PARAM_7 = 65,
    SHDISP_MAIN_BKL_PARAM_8 = 77,
    SHDISP_MAIN_BKL_PARAM_9 = 90,
    SHDISP_MAIN_BKL_PARAM_10 = 103,
    SHDISP_MAIN_BKL_PARAM_11 = 115,
    SHDISP_MAIN_BKL_PARAM_12 = 128,
    SHDISP_MAIN_BKL_PARAM_13 = 141,
    SHDISP_MAIN_BKL_PARAM_14 = 153,
    SHDISP_MAIN_BKL_PARAM_15 = 166,
    SHDISP_MAIN_BKL_PARAM_16 = 179,
    SHDISP_MAIN_BKL_PARAM_17 = 192,
    SHDISP_MAIN_BKL_PARAM_18 = 204,
    SHDISP_MAIN_BKL_PARAM_19 = 217,
    SHDISP_MAIN_BKL_PARAM_20 = 230,
    SHDISP_MAIN_BKL_PARAM_21 = 242,
    SHDISP_MAIN_BKL_PARAM_22 = 255,
    NUM_SHDISP_MAIN_BKL_PARAM
};

#define SHDISP_MAIN_BKL_PARAM_OFF           (0)
#define SHDISP_MAIN_BKL_PARAM_WEAK          (1)
#define SHDISP_MAIN_BKL_PARAM_MIN           (0)
#define SHDISP_MAIN_BKL_PARAM_MAX           (255)
#define SHDISP_MAIN_BKL_PARAM_MIN_AUTO      (2)
#define SHDISP_MAIN_BKL_PARAM_MAX_AUTO      (255)

enum {
    SHDISP_MAIN_BKL_AUTO_OFF,
    SHDISP_MAIN_BKL_AUTO_ON,
    SHDISP_MAIN_BKL_AUTO_ECO_ON,
    NUM_SHDISP_MAIN_BKL_AUTO
};

enum {
    SHDISP_MAIN_BKL_DTV_OFF,
    SHDISP_MAIN_BKL_DTV_ON,
    NUM_SHDISP_MAIN_BKL_DTV
};

enum {
    SHDISP_MAIN_BKL_EMG_OFF,
    SHDISP_MAIN_BKL_EMG_ON,
    NUM_SHDISP_MAIN_BKL_EMG
};

enum {
    SHDISP_MAIN_BKL_ECO_OFF,
    SHDISP_MAIN_BKL_ECO_ON,
    NUM_SHDISP_MAIN_BKL_ECO
};

enum {
    SHDISP_MAIN_DISP_ALS_RANGE_001 = 0,
    SHDISP_MAIN_DISP_ALS_RANGE_002,
    SHDISP_MAIN_DISP_ALS_RANGE_004,
    SHDISP_MAIN_DISP_ALS_RANGE_008,
    SHDISP_MAIN_DISP_ALS_RANGE_016,
    SHDISP_MAIN_DISP_ALS_RANGE_032,
    SHDISP_MAIN_DISP_ALS_RANGE_064,
    SHDISP_MAIN_DISP_ALS_RANGE_128,
    NUM_SHDISP_MAIN_DISP_ALS_RANGE
};

enum {
    SHDISP_PHOTO_SENSOR_DISABLE,
    SHDISP_PHOTO_SENSOR_ENABLE,
    NUM_SHDISP_PHOTO_SENSOR
};

enum {
    SHDISP_TRI_LED_EXT_MODE_DISABLE,
    SHDISP_TRI_LED_EXT_MODE_ENABLE,
    NUM_SHDISP_TRI_LED_EXT_MODE
};

enum {
    SHDISP_KEY_BKL_OFF,
    SHDISP_KEY_BKL_NORMAL,
    SHDISP_KEY_BKL_BLINK,
    SHDISP_KEY_BKL_DIM,
    NUM_SHDISP_KEY_BKL_MODE
};

enum {
    SHDISP_KEY_BKL_LEFT,
    SHDISP_KEY_BKL_CENTER,
    SHDISP_KEY_BKL_RIGHT,
    NUM_SHDISP_KEY_SELECT
};

enum {
    SHDISP_TRI_LED_MODE_NORMAL,
    SHDISP_TRI_LED_MODE_BLINK,
    SHDISP_TRI_LED_MODE_FIREFLY,
    SHDISP_TRI_LED_MODE_HISPEED,
    SHDISP_TRI_LED_MODE_STANDARD,
    SHDISP_TRI_LED_MODE_BREATH,
    SHDISP_TRI_LED_MODE_LONG_BREATH,
    SHDISP_TRI_LED_MODE_WAVE,
    SHDISP_TRI_LED_MODE_FLASH,
    SHDISP_TRI_LED_MODE_AURORA,
    SHDISP_TRI_LED_MODE_RAINBOW,
    NUM_SHDISP_TRI_LED_MODE
};

enum {
    SHDISP_TRI_LED_ONTIME_TYPE0,
    SHDISP_TRI_LED_ONTIME_TYPE1,
    SHDISP_TRI_LED_ONTIME_TYPE2,
    SHDISP_TRI_LED_ONTIME_TYPE3,
    SHDISP_TRI_LED_ONTIME_TYPE4,
    SHDISP_TRI_LED_ONTIME_TYPE5,
    SHDISP_TRI_LED_ONTIME_TYPE6,
    SHDISP_TRI_LED_ONTIME_TYPE7,
    NUM_SHDISP_TRI_LED_ONTIME
};

enum {
    SHDISP_KEY_BKL_ONTIME_TYPE0,
    SHDISP_KEY_BKL_ONTIME_TYPE1,
    SHDISP_KEY_BKL_ONTIME_TYPE2,
    SHDISP_KEY_BKL_ONTIME_TYPE3,
    SHDISP_KEY_BKL_ONTIME_TYPE4,
    SHDISP_KEY_BKL_ONTIME_TYPE5,
    SHDISP_KEY_BKL_ONTIME_TYPE6,
    SHDISP_KEY_BKL_ONTIME_TYPE7,
    NUM_SHDISP_KEY_BKL_ONTIME
};

enum {
    SHDISP_TRI_LED_COUNT_NONE,
    SHDISP_TRI_LED_COUNT_1,
    SHDISP_TRI_LED_COUNT_2,
    SHDISP_TRI_LED_COUNT_3,
    SHDISP_TRI_LED_COUNT_4,
    SHDISP_TRI_LED_COUNT_5,
    SHDISP_TRI_LED_COUNT_6,
    SHDISP_TRI_LED_COUNT_7,
    NUM_SHDISP_TRI_LED_COUNT
};

enum {
    SHDISP_PHOTO_SENSOR_TYPE_APP,
    SHDISP_PHOTO_SENSOR_TYPE_LUX,
    SHDISP_PHOTO_SENSOR_TYPE_CAMERA,
    SHDISP_PHOTO_SENSOR_TYPE_KEYLED,
    SHDISP_PHOTO_SENSOR_TYPE_DIAG,
    NUM_SHDISP_PHOTO_SENSOR_TYPE
};

enum {
    SHDISP_LUX_MODE_LOW,
    SHDISP_LUX_MODE_HIGH,
    NUM_SHDISP_LUX_MODE
};

enum {
    SHDISP_LEDC_ONCOUNT_REPEAT,
    SHDISP_LEDC_ONCOUNT_1SHOT,
    NUM_SHDISP_LEDC_ONCOUNT
};

enum {
    SHDISP_LEDC_IS_NOT_EXIST,
    SHDISP_LEDC_IS_EXIST,
    NUM_LEDC_EXIST_STATUS
};

enum {
    SHDISP_DIAG_COG_ID_NONE,
    SHDISP_DIAG_COG_ID_MASTER,
    SHDISP_DIAG_COG_ID_SLAVE,
    SHDISP_DIAG_COG_ID_BOTH,
    NUM_SHDISP_DIAG_COG_ID
};

struct shdisp_host_gpio {
    int num;
    int value;
};

struct shdisp_diag_bdic_reg {
    unsigned char reg;
    unsigned char val;
};

struct shdisp_diag_psals_reg {
    unsigned char reg;
    unsigned char val;
};

struct shdisp_diag_bdic_reg_multi {
    unsigned char reg;
    unsigned char val[8];
    unsigned char size;
};

#define SHDISP_LCDDR_BUF_MAX    64
struct shdisp_lcddr_reg {
    unsigned char address;
    unsigned char size;
    unsigned char buf[SHDISP_LCDDR_BUF_MAX];
    int      cog;
};

struct shdisp_photo_sensor_val {
    unsigned short value;
    unsigned long  lux;
    int mode;
    int result;
};

struct shdisp_photo_sensor_raw_val {
    unsigned short clear;
    unsigned short ir;
    int result;
};

struct shdisp_photo_sensor_power_ctl {
    int type;
    int power;
};

struct shdisp_ledc_rgb {
    unsigned long mode;
    unsigned long red[2];
    unsigned long green[2];
    unsigned long blue[2];
};

struct shdisp_diag_ledc_reg {
    unsigned char reg;
    unsigned char val;
};

struct shdisp_ledc_mono {
    unsigned long led;
    int led_mode;
    int on_count;
};

struct shdisp_diag_gamma_info {
#if defined(CONFIG_SHDISP_PANEL_CARIN) || defined(USER_CONFIG_SHDISP_PANEL_CARIN)
    unsigned char   gamma[SHDISP_PANEL_GAMMA_TBL_SIZE];
    unsigned char   vlm1;
    unsigned char   vlm2;
    unsigned char   vlm3;
    unsigned char   vc1;
    unsigned char   vc2;
    unsigned char   vc3;
    unsigned char   vpl;
    unsigned char   vnl;
    unsigned char   svss_svdd;
#elif defined(CONFIG_SHDISP_PANEL_KITE) || defined(USER_CONFIG_SHDISP_PANEL_KITE)
    unsigned char   gammaR[SHDISP_PANEL_GAMMA_TBL_SIZE];
    unsigned char   gammaG[SHDISP_PANEL_GAMMA_TBL_SIZE];
    unsigned char   gammaB[SHDISP_PANEL_GAMMA_TBL_SIZE];
    unsigned char   vlm1;
    unsigned char   vlm2;
    unsigned char   vlm3;
    unsigned char   vc1;
    unsigned char   vc2;
    unsigned char   vc3;
    unsigned char   vpl;
    unsigned char   vnl;
#else
    unsigned char   gammaR[SHDISP_PANEL_GAMMA_TBL_SIZE];
    unsigned char   gammaG[SHDISP_PANEL_GAMMA_TBL_SIZE];
    unsigned char   gammaB[SHDISP_PANEL_GAMMA_TBL_SIZE];
    unsigned char   vlm1;
    unsigned char   vlm2;
    unsigned char   vlm3;
    unsigned char   vc1;
    unsigned char   vc2;
    unsigned char   vc3;
    unsigned char   vpl;
    unsigned char   vnl;
    unsigned char   svss_svdd;
#endif
};

struct shdisp_diag_gamma {
#if defined(CONFIG_SHDISP_PANEL_CARIN) || defined(USER_CONFIG_SHDISP_PANEL_CARIN)
    unsigned char   level;
    unsigned char   gamma_p;
    unsigned char   gamma_n;
#elif defined(CONFIG_SHDISP_PANEL_KITE) || defined(USER_CONFIG_SHDISP_PANEL_KITE)
    unsigned char   level;
    unsigned char   gammaR_p;
    unsigned char   gammaR_n;
    unsigned char   gammaG_p;
    unsigned char   gammaG_n;
    unsigned char   gammaB_p;
    unsigned char   gammaB_n;
#else
    unsigned char   level;
    unsigned char   gammaR_p;
    unsigned char   gammaR_n;
    unsigned char   gammaG_p;
    unsigned char   gammaG_n;
    unsigned char   gammaB_p;
    unsigned char   gammaB_n;
#endif
};

struct shdisp_diag_flicker_param {
    unsigned short  master_alpha;
    unsigned short  slave_alpha;
};

struct shdisp_ave_ado {
    unsigned char  als_range;
    unsigned short ave_als0;
    unsigned short ave_als1;
    unsigned short ave_ado;
};

