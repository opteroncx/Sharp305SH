/* drivers/sharp/shdisp/data/shdisp_carin_data.h  (Display Driver)
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



static char initial_setting_1_1_payloads[] = {
    0xB0,0x04
};

static struct shdisp_dsi_cmd_desc initial_setting_1_1_cmds[] = {
    { SHDISP_DTYPE_GEN_WRITE2, 2, &initial_setting_1_1_payloads[0],0}
};


static char devcode_read_payloads[] = {
    0xBF,
};

static struct shdisp_dsi_cmd_desc devcode_read_cmds[] = {
    { SHDISP_DTYPE_GEN_READ1, 1, &devcode_read_payloads[0],0}
};


static char initial_setting_1_2_payloads[] = {
    0xB3,0x14,0x00,0x00,0x00,0x00,0x00,
    0xB4,0x0C,0x00,
    0xB6,0x39,0xA3,
    0xBE,0x00,0x07,
    0xC0,0x00
};

static struct shdisp_dsi_cmd_desc initial_setting_1_2_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, 7, &initial_setting_1_2_payloads[0],0},
    { SHDISP_DTYPE_GEN_LWRITE, 3, &initial_setting_1_2_payloads[7],0},
    { SHDISP_DTYPE_GEN_LWRITE, 3, &initial_setting_1_2_payloads[10],0},
    { SHDISP_DTYPE_GEN_LWRITE, 3, &initial_setting_1_2_payloads[13],0},
    { SHDISP_DTYPE_GEN_WRITE2, 2, &initial_setting_1_2_payloads[16],0}
};


static char timing_setting_payloads[] = {
    0xC1,0x44,0x66,0x00,0x5F,0x90,0xFF,0xFF,0x07,0xFF,0xFF,0xFF,0x5D,0x63,0xAC,0xB9,0x0F,0xDF,0x57,0xE1,0xFF,0xFF,0xCB,0xF8,0x42,0x42,0x42,0x42,0x00,0x22,0x00,0x00,0x62,0x03,0xD1,
    0xC2,0x31,0xF5,0x00,0x0C,0x08,0x00,0x00,
    0xC3,0x00,0x00,0x00,
    0xC4,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x0C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x0C,
    0xC6,0xC8,0x02,0xF2,0x0A,0xE2,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0x2E,0x20,0xC8,0x02,0xF2,0x0A,0xE2,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0x2E,0x20
};

static struct shdisp_dsi_cmd_desc timing_setting_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, 35, &timing_setting_payloads[0],0},
    { SHDISP_DTYPE_GEN_LWRITE, 8, &timing_setting_payloads[35],0},
    { SHDISP_DTYPE_GEN_LWRITE, 4, &timing_setting_payloads[43],0},
    { SHDISP_DTYPE_GEN_LWRITE, 23, &timing_setting_payloads[47],0},
    { SHDISP_DTYPE_GEN_LWRITE, 41, &timing_setting_payloads[70],0}
};


static char gamma_setting_pic_adj_payloads[] = {
    0xC7,0x00,0x13,0x18,0x1F,0x2F,0x3F,0x4A,0x5A,0x3E,0x46,0x51,0x5F,0x68,0x6F,0x72,0x00,0x14,0x19,0x20,0x2F,0x3D,0x46,0x56,0x3B,0x46,0x53,0x61,0x6B,0x72,0x76
};

static struct shdisp_dsi_cmd_desc gamma_setting_pic_adj_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, 31, &gamma_setting_pic_adj_payloads[0],0}
};


static char dig_gamma_setting_payloads[] = {
    0xC8,0x00
};

static struct shdisp_dsi_cmd_desc dig_gamma_setting_cmds[] = {
    { SHDISP_DTYPE_GEN_WRITE2, 2, &dig_gamma_setting_payloads[0],0}
};


static char pin_config_payloads[] = {
    0xCB,0x86,0xE0,0xB7,0x61,0x00,0x00,0x00,0x00,0xC0,
    0xCC,0x06,
    0xCF,0x00,0x00,0xC1,0x05,0x3F
};

static struct shdisp_dsi_cmd_desc pin_config_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, 10, &pin_config_payloads[0],0},
    { SHDISP_DTYPE_GEN_LWRITE, 2, &pin_config_payloads[10],0},
    { SHDISP_DTYPE_GEN_LWRITE, 6, &pin_config_payloads[12],0}
};


static char power_setting_chargepump_payloads[] = {
    0xD0,0x44,0x81,0xBB,0x58,0xCD,0x4C,0x19,0x19,0x04,0x00
};

static struct shdisp_dsi_cmd_desc power_setting_chargepump_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, 11, &power_setting_chargepump_payloads[0],0}
};


static char power_setting_internalpower_payloads[] = {
    0xD3,0x1B,0x33,0xBB,0xBB,0xB3,0x33,0x33,0x33,0x00,0x01,0x00,0xA0,0xD8,0xA0,0x02,0x46,0x3E,0x33,0x3B,0x37,0x72,0x07,0x3D,0xBF,0x00
};

static struct shdisp_dsi_cmd_desc power_setting_internalpower_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, 26, &power_setting_internalpower_payloads[0],0}
};


static char power_setting_payloads[] = {
    0xD5,0x06,0x00,0x00,0x01,0x3F,0x01,0x3F,
    0xD9,0x20,0x00,0x00
};

static struct shdisp_dsi_cmd_desc power_setting_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, 8, &power_setting_payloads[0],0},
    { SHDISP_DTYPE_GEN_LWRITE, 4, &power_setting_payloads[8],0}
};


static char sync_signal_setting_payloads[] = {
    0xEC,0x50,0x10,
    0xED,0x00,0x00,0x00,
    0xEE,0x00,0x32,
    0xEF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0xD6,0x01,
    0xD7,0x84,0xE0,0x7F,0xA8,0xCE,0x38,0xFC,0xC1,0x18,0xE7,0x83,0x1F,0x3C,0x10,0xFA,0xC3,0x0F,0x04,0x41,0x20,0x00,0x00
};

static struct shdisp_dsi_cmd_desc sync_signal_setting_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, 3, &sync_signal_setting_payloads[0],0},
    { SHDISP_DTYPE_GEN_LWRITE, 4, &sync_signal_setting_payloads[3],0},
    { SHDISP_DTYPE_GEN_LWRITE, 3, &sync_signal_setting_payloads[7],0},
    { SHDISP_DTYPE_GEN_LWRITE, 13, &sync_signal_setting_payloads[10],0},
    { SHDISP_DTYPE_GEN_WRITE2, 2, &sync_signal_setting_payloads[23],0},
    { SHDISP_DTYPE_GEN_LWRITE, 23, &sync_signal_setting_payloads[25],0}
};


static char disp_on_exit_sleep_payloads[] = {
    0x29,
    0x11,
};

static struct shdisp_dsi_cmd_desc disp_on_exit_sleep_cmds[] = {
    { SHDISP_DTYPE_DCS_WRITE, 1, &disp_on_exit_sleep_payloads[0],0},
    { SHDISP_DTYPE_DCS_WRITE, 1, &disp_on_exit_sleep_payloads[1],0}
};


static char disp_off_enter_sleep_payloads[] = {
    0x28,
    0x10,
};

static struct shdisp_dsi_cmd_desc disp_off_enter_sleep_cmds[] = {
    { SHDISP_DTYPE_DCS_WRITE, 1, &disp_off_enter_sleep_payloads[0],16666},
    { SHDISP_DTYPE_DCS_WRITE, 1, &disp_off_enter_sleep_payloads[1],66664}
};


static char deep_standby_payloads[] = {
    0xB1,0x01
};

static struct shdisp_dsi_cmd_desc deep_standby_cmds[] = {
    { SHDISP_DTYPE_GEN_WRITE2, 2, &deep_standby_payloads[0],0}
};

static char interface_id_setting[] = {
    0xB4,
};

static struct shdisp_dsi_cmd_desc interface_id_read_cmds[] = {
    { SHDISP_DTYPE_GEN_READ1, 1, &interface_id_setting[0],0}
};


static char read_checksum_and_ecc_error_count[] = {
    0xB5,
};

static struct shdisp_dsi_cmd_desc read_checksum_and_ecc_error_count_cmds[] = {
    { SHDISP_DTYPE_GEN_READ1, 1, &read_checksum_and_ecc_error_count[0],0}
};
