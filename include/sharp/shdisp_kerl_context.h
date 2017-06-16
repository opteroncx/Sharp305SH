/* include/sharp/shdisp_kerl_context.h  (Display Driver)
 *
 * Copyright (C) 2011-2012 SHARP CORPORATION
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

struct shdisp_kernel_context {
    int driver_is_initialized;
    unsigned short hw_revision;
#if defined(CONFIG_MACH_TAC)
	unsigned short hw_handset;
#endif
    int handset_color;
    int upper_unit_is_connected;
    int bdic_is_exist;
    int main_disp_status;
    struct shdisp_main_bkl_ctl main_bkl;
    struct shdisp_tri_led tri_led;
    struct shdisp_key_bkl_ctl key_bkl_ctl;
    unsigned short alpha;
    unsigned short alpha_low;
    struct shdisp_photo_sensor_adj photo_sensor_adj;
    struct shdisp_lcddr_phy_gamma_reg lcddr_phy_gamma;
    struct shdisp_lcddr_phy_gamma_reg lcddr_rom_gamma;
    int dtv_status;
    int thermal_status;
    int eco_bkl_status;
    int usb_chg_status;
    struct shdisp_ledc_status ledc_status;
    unsigned int shdisp_lcd;
    unsigned char dbgTraceF;
    struct shdisp_igc_lut igc_lut;
};
