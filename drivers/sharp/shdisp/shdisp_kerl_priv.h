/* include/sharp/shdisp_kerl_priv.h  (Display Driver)
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

#ifndef SHDISP_KERL_PRIV_H
#define SHDISP_KERL_PRIV_H

void shdisp_api_get_boot_context(void);
int shdisp_api_get_boot_disp_status(void);
int shdisp_api_get_upper_unit(void);
int shdisp_api_set_upper_unit(int mode);
unsigned short shdisp_api_get_hw_revision(void);
unsigned short shdisp_api_get_hw_handset(void);
int shdisp_api_is_open(void);
void shdisp_semaphore_start(void);
void shdisp_semaphore_end(const char *func);



/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define SHDISP_LCDDR_GAMMA_STATUS_OK            0x96



enum {
     SHDISP_PANEL_POWER_NORMAL_ON,
     SHDISP_PANEL_POWER_RECOVERY_ON
};

enum {
     SHDISP_PANEL_POWER_NORMAL_OFF,
     SHDISP_PANEL_POWER_RECOVERY_OFF,
     SHDISP_PANEL_POWER_SHUTDOWN_OFF
};

#endif /* SHDISP_KERL_PRIV_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
