/* include/sharp/shsmartamp_ssm4329.h - SHARP smart amplifier driver
 *
 * Copyright (C) 2011 SHARP CORPORATION
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
/* CONFIG_SH_AUDIO_DRIVER newly created */

#ifndef __ASM_ARM_ARCH_SHSMARTAMP_SSM4329_H
#define __ASM_ARM_ARCH_SHSMARTAMP_SSM4329_H

#ifdef CONFIG_SH_AUDIO_SMARTAMP
#define SHSMARTAMP_I2C_NAME "shsmartamp_i2c"

extern void shsmartamp_control(int on);
#endif /* CONFIG_SHSMARTAMP_SSM4329 */
#endif
