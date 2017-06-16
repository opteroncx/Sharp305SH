/* drivers/sharp/shdisp/data/shdisp_bl71y6_ctrl.h  (Display Driver)
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

#ifndef SHDISP_BL71Y6_CTRL_H
#define SHDISP_BL71Y6_CTRL_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include "../shdisp_bl71y6.h"

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static const shdisp_bdicRegSetting_t shdisp_bdic_set_bank0[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_set_bank1[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x01,                       0x00,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_init1[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
    ,{BDIC_REG_SYSTEM1,             SHDISP_BDIC_STR,    0x80,                       0xFF,  10600}
    ,{BDIC_REG_SYSTEM3,             SHDISP_BDIC_CLR,    0x00,                       0x02,      0}
    ,{BDIC_REG_SYSTEM1,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_init2[] = {
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_SYSTEM2,             SHDISP_BDIC_STR,    0x20,                       0xFF,      0}
    ,{BDIC_REG_SYSTEM3,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_SYSTEM4,             SHDISP_BDIC_STR,    0x31,                       0xFF,      0}
    ,{BDIC_REG_SYSTEM5,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_SYSTEM6,             SHDISP_BDIC_STR,    0x28,                       0xFF,      0}
    ,{BDIC_REG_I2C_TIMER,           SHDISP_BDIC_STR,    0xFF,                       0xFF,      0}
    ,{BDIC_REG_I2C_SYS,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_I2C_DATA0,           SHDISP_BDIC_STR,    0x72,                       0xFF,      0}
    ,{BDIC_REG_XEN_TSD,             SHDISP_BDIC_STR,    0x01,                       0xFF,      0}

    ,{BDIC_REG_OPT_MODE,            SHDISP_BDIC_STR,    0x01,                       0xFF,      0}
    ,{BDIC_REG_LDO,                 SHDISP_BDIC_STR,    0x01,                       0xFF,      0}
    ,{BDIC_REG_ALS_DATA0_SET,       SHDISP_BDIC_STR,    0x80,                       0xFF,      0}
    ,{BDIC_REG_ALS_DATA1_SET,       SHDISP_BDIC_STR,    0x21,                       0xFF,      0}

    ,{BDIC_REG_MODE_M1,             SHDISP_BDIC_STR,    0x03,                       0xFF,      0}
    ,{BDIC_REG_PSDATA_SET,          SHDISP_BDIC_STR,    0x02,                       0xFF,      0}
    ,{BDIC_REG_PS_HT_LSB,           SHDISP_BDIC_STR,    0x10,                       0xFF,      0}
    ,{BDIC_REG_PS_HT_MSB,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_PS_LT_LSB,           SHDISP_BDIC_STR,    0x0F,                       0xFF,      0}
    ,{BDIC_REG_PS_LT_MSB,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0}

    ,{BDIC_REG_CPM1,                SHDISP_BDIC_STR,    0x05,                       0xFF,      0}
    ,{BDIC_REG_CPM2,                SHDISP_BDIC_STR,    0x31,                       0xFF,      0}
    ,{BDIC_REG_DCDC2_ERES2_1,       SHDISP_BDIC_STR,    0x33,                       0xFF,      0}
    ,{BDIC_REG_DCDC2_ERES2_2,       SHDISP_BDIC_STR,    0x05,                       0xFF,      0}
    ,{BDIC_REG_DCDC2_CLIMIT_1,      SHDISP_BDIC_STR,    0x06,                       0xFF,      0}
    ,{BDIC_REG_DCDC2_CLIMIT_2,      SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_DCDC2_GM,            SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_DCDC2_BF,            SHDISP_BDIC_STR,    0xC3,                       0xFF,      0}
    ,{BDIC_REG_DCDC2_SW,            SHDISP_BDIC_STR,    0xC3,                       0xFF,      0}
    ,{BDIC_REG_DCDC2_OSC_1,         SHDISP_BDIC_STR,    0xB1,                       0xFF,      0}
    ,{BDIC_REG_DCDC2_OSC_2,         SHDISP_BDIC_STR,    0x01,                       0xFF,      0}
    ,{BDIC_REG_VO2_PFML,            SHDISP_BDIC_STR,    BDIC_REG_VO2_PFML_VAL,      0xFF,      0}
    ,{BDIC_REG_VO2_PWML,            SHDISP_BDIC_STR,    BDIC_REG_VO2_PWML_VAL,      0xFF,      0}
    ,{BDIC_REG_VO2_PWMH,            SHDISP_BDIC_STR,    BDIC_REG_VO2_PWMH_VAL,      0xFF,      0}
    ,{BDIC_REG_DCCD2_MODE_SEL,      SHDISP_BDIC_STR,    0x85,                       0xFF,      0}
    ,{BDIC_REG_DCDC1_CUR_SEL_DRV,   SHDISP_BDIC_STR,    0x80,                       0xFF,      0}
    ,{BDIC_REG_DCDC2_TEST_57,       SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_DCDC1_CLIMT_1,       SHDISP_BDIC_STR,    0xF5,                       0xFF,      0}
    ,{BDIC_REG_DCDC1_CLIMT_2,       SHDISP_BDIC_STR,    BDIC_REG_DCDC1_CLIMT_2_VAL, 0xFF,      0}
    ,{BDIC_REG_DCDC1_GM,            SHDISP_BDIC_STR,    BDIC_REG_DCDC1_GM_VAL,      0xFF,      0}
    ,{BDIC_REG_DCDC1_BF,            SHDISP_BDIC_STR,    0xF3,                       0xFF,      0}
    ,{BDIC_REG_DCDC1_SW,            SHDISP_BDIC_STR,    0xF3,                       0xFF,      0}
    ,{BDIC_REG_DCDC1_OSC_1,         SHDISP_BDIC_STR,    0x31,                       0xFF,      0}
    ,{BDIC_REG_DCDC1_OSC_2,         SHDISP_BDIC_STR,    BDIC_REG_DCDC1_OSC_2_VAL,   0xFF,      0}
    ,{BDIC_REG_DCDC1_OVDETREF,      SHDISP_BDIC_STR,    0x84,                       0xFF,      0}
    ,{BDIC_REG_DCDC1_TIMER,         SHDISP_BDIC_STR,    0x04,                       0xFF,      0}
    ,{BDIC_REG_DCDC1_TH_LCUR,       SHDISP_BDIC_STR,    0x38,                       0xFF,      0}
    ,{BDIC_REG_DCDC1_TH_HCUR,       SHDISP_BDIC_STR,    0x3C,                       0xFF,      0}
    ,{BDIC_REG_DCDC1_CUR_SOFT,      SHDISP_BDIC_STR,    0x04,                       0xFF,      0}
    ,{BDIC_REG_DCDC1_REF1_SOFT,     SHDISP_BDIC_STR,    0x22,                       0xFF,      0}
    ,{BDIC_REG_DCDC1_EAMPREF2,      SHDISP_BDIC_STR,    0x32,                       0xFF,      0}
    ,{BDIC_REG_DCDC1_PFMREF1,       SHDISP_BDIC_STR,    0x32,                       0xFF,      0}
    ,{BDIC_REG_DCDC1_PFMREF2,       SHDISP_BDIC_STR,    0x32,                       0xFF,      0}

    ,{BDIC_REG_M1LED,               SHDISP_BDIC_STR,    0xFF,                       0xFF,      0}
    ,{BDIC_REG_M2LED,               SHDISP_BDIC_STR,    0xFF,                       0xFF,      0}
    ,{BDIC_REG_PWMDC1,              SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_PWMDC2,              SHDISP_BDIC_STR,    0x00,                       0xFF,      0}

    ,{BDIC_REG_GPIO_0,              SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GPIO_1,              SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
#if defined(CONFIG_MACH_PA25) || defined(CONFIG_MACH_PB25) || defined(CONFIG_MACH_KF10) || defined(CONFIG_MACH_PA28)
    ,{BDIC_REG_GPIO_2,              SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GPIO_3,              SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GPIO_4,              SHDISP_BDIC_STR,    0x02,                       0xFF,      0}
#endif
    ,{BDIC_REG_GPIO_5,              SHDISP_BDIC_STR,    0x02,                       0xFF,      0}
};
static const shdisp_bdicRegSetting_t shdisp_bdic_init3[] = {
#ifndef CONFIG_MACH_TAC
     {BDIC_REG_GPIO_6,              SHDISP_BDIC_STR,    BDIC_REG_GPIO_6_VAL,        0xFF,      0}
    ,{BDIC_REG_GPIO_ANSW,           SHDISP_BDIC_STR,    BDIC_REG_GPIO_ANSW_VAL,     0xFF,      0}
#endif
    ,{BDIC_REG_GPIMSK0,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
#ifndef CONFIG_MACH_TAC
    ,{BDIC_REG_GPIMSK1,             SHDISP_BDIC_STR,    BDIC_REG_GPIMSK1_VAL,       0xFF,      0}
    ,{BDIC_REG_GPIMSK2,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
#endif
    ,{BDIC_REG_INT_CTRL,            SHDISP_BDIC_STR,    0x02,                       0xFF,      0}
    ,{BDIC_REG_GPIO_OPD_PUSEL,      SHDISP_BDIC_STR,    0x00,                       0xFF,      0}

    ,{BDIC_REG_DETECTOR,            SHDISP_BDIC_STR,    0x02,                       0xFF,      0}
    ,{BDIC_REG_GIMR1,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GIMF1,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GIMR2,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GIMR3,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GIMR4,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GIMF3,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GIMF4,               SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_INT1,            SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_INT2,            SHDISP_BDIC_STR,    0x00,                       0xFF,      0}

    ,{BDIC_REG_AR_HI_TH_L_L,        SHDISP_BDIC_STR,    BDIC_REG_AR_HI_TH_L_L_VAL,  0xFF,      0}
    ,{BDIC_REG_AR_HI_TH_L_H,        SHDISP_BDIC_STR,    BDIC_REG_AR_HI_TH_L_H_VAL,  0xFF,      0}
    ,{BDIC_REG_AR_LO_TH_M_L,        SHDISP_BDIC_STR,    BDIC_REG_AR_LO_TH_M_L_VAL,  0xFF,      0}
    ,{BDIC_REG_AR_LO_TH_M_H,        SHDISP_BDIC_STR,    BDIC_REG_AR_LO_TH_M_H_VAL,  0xFF,      0}
    ,{BDIC_REG_AR_HI_TH_M_L,        SHDISP_BDIC_STR,    BDIC_REG_AR_HI_TH_M_L_VAL,  0xFF,      0}
    ,{BDIC_REG_AR_HI_TH_M_H,        SHDISP_BDIC_STR,    BDIC_REG_AR_HI_TH_M_H_VAL,  0xFF,      0}
    ,{BDIC_REG_AR_LO_TH_H_L,        SHDISP_BDIC_STR,    BDIC_REG_AR_LO_TH_H_L_VAL,  0xFF,      0}
    ,{BDIC_REG_AR_LO_TH_H_H,        SHDISP_BDIC_STR,    BDIC_REG_AR_LO_TH_H_H_VAL,  0xFF,      0}
    ,{BDIC_REG_AR_RANGE_L,          SHDISP_BDIC_STR,    BDIC_REG_AR_RANGE_L_VAL,    0xFF,      0}
    ,{BDIC_REG_AR_RANGE_M,          SHDISP_BDIC_STR,    BDIC_REG_AR_RANGE_M_VAL,    0xFF,      0}
    ,{BDIC_REG_AR_RANGE_H,          SHDISP_BDIC_STR,    BDIC_REG_AR_RANGE_H_VAL,    0xFF,      0}
    ,{BDIC_REG_DATA_SHIFT_ML,       SHDISP_BDIC_STR,    BDIC_REG_DATA_SHIFT_M_VAL,  0xFF,      0}
    ,{BDIC_REG_DATA_SHIFT_H,        SHDISP_BDIC_STR,    BDIC_REG_DATA_SHIFT_H_VAL,  0xFF,      0}

    ,{BDIC_REG_CLR_IR_CMP_TH1,      SHDISP_BDIC_STR,    0x3E,                       0xFF,      0}
    ,{BDIC_REG_CLR_IR_CMP_TH2,      SHDISP_BDIC_STR,    0x3D,                       0xFF,      0}
    ,{BDIC_REG_CLR_IR_CMP_TH3,      SHDISP_BDIC_STR,    0x3C,                       0xFF,      0}
    ,{BDIC_REG_AR_DATA0,            SHDISP_BDIC_STR,    0x72,                       0xFF,      0}
    ,{BDIC_REG_AR_DATA1,            SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_AR_DATA2,            SHDISP_BDIC_STR,    0x5E,                       0xFF,      0}
    ,{BDIC_REG_AR_DATA3,            SHDISP_BDIC_STR,    0x01,                       0xFF,      0}
    ,{BDIC_REG_AR_DATA5,            SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_AR_DATA6,            SHDISP_BDIC_STR,    0xDE,                       0xFF,      0}
    ,{BDIC_REG_AR_RANGE_SEL,        SHDISP_BDIC_STR,    0x10,                       0xFF,      0}
    ,{BDIC_REG_AR_CTRL,             SHDISP_BDIC_STR,    BDIC_REG_AR_CTRL_VAL,       0xFF,      0}
    ,{BDIC_REG_SLOPE,               SHDISP_BDIC_STR,    0xFF,                       0xFF,      0}
    ,{BDIC_REG_TIMER_DIV,           SHDISP_BDIC_STR,    0x91,                       0xFF,      0}
    ,{BDIC_REG_I2C_TIMER,           SHDISP_BDIC_STR,    0xD0,                       0xFF,      0}

    ,{BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x01,                       0x00,      0}
    ,{BDIC_REG_PS_DATA_L,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_PS_DATA_H,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_CLR_DATA_L,          SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_CLR_DATA_H,          SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_IR_DATA_L,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_IR_DATA_H,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0}

    ,{BDIC_REG_ALS_ADJ0_0_L,        SHDISP_BDIC_STR,    BDIC_REG_ALS_ADJ0_L_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ0_0_H,        SHDISP_BDIC_STR,    BDIC_REG_ALS_ADJ0_H_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ1_0_L,        SHDISP_BDIC_STR,    BDIC_REG_ALS_ADJ1_L_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ1_0_H,        SHDISP_BDIC_STR,    BDIC_REG_ALS_ADJ1_H_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_SHIFT_0,         SHDISP_BDIC_STR,    BDIC_REG_ALS_SHIFT_DEFAULT_B,  0xFF,   0}
    ,{BDIC_REG_ALS_ADJ0_1_L,        SHDISP_BDIC_STR,    BDIC_REG_ALS_ADJ0_L_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ0_1_H,        SHDISP_BDIC_STR,    BDIC_REG_ALS_ADJ0_H_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ1_1_L,        SHDISP_BDIC_STR,    BDIC_REG_ALS_ADJ1_L_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ1_1_H,        SHDISP_BDIC_STR,    BDIC_REG_ALS_ADJ1_H_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_SHIFT_1,         SHDISP_BDIC_STR,    BDIC_REG_ALS_SHIFT_DEFAULT_B,  0xFF,   0}
    ,{BDIC_REG_ALS_ADJ0_2_L,        SHDISP_BDIC_STR,    BDIC_REG_ALS_ADJ0_L_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ0_2_H,        SHDISP_BDIC_STR,    BDIC_REG_ALS_ADJ0_H_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ1_2_L,        SHDISP_BDIC_STR,    BDIC_REG_ALS_ADJ1_L_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ1_2_H,        SHDISP_BDIC_STR,    BDIC_REG_ALS_ADJ1_H_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_SHIFT_2,         SHDISP_BDIC_STR,    BDIC_REG_ALS_SHIFT_DEFAULT_B,  0xFF,   0}
    ,{BDIC_REG_ALS_ADJ0_3_L,        SHDISP_BDIC_STR,    BDIC_REG_ALS_ADJ0_L_DEFAULT_A, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ0_3_H,        SHDISP_BDIC_STR,    BDIC_REG_ALS_ADJ0_H_DEFAULT_A, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ1_3_L,        SHDISP_BDIC_STR,    BDIC_REG_ALS_ADJ1_L_DEFAULT_A, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ1_3_H,        SHDISP_BDIC_STR,    BDIC_REG_ALS_ADJ1_H_DEFAULT_A, 0xFF,   0}
    ,{BDIC_REG_ALS_SHIFT_3,         SHDISP_BDIC_STR,    BDIC_REG_ALS_SHIFT_DEFAULT_A,  0xFF,   0}

    ,{BDIC_REG_OPT_TH_SHIFT_1_0,    SHDISP_BDIC_STR,    BDIC_REG_OPT_TH_SHIFT_1_0_VAL,   0xFF, 0}
    ,{BDIC_REG_OPT_TH_SHIFT_3_2,    SHDISP_BDIC_STR,    BDIC_REG_OPT_TH_SHIFT_3_2_VAL,   0xFF, 0}
    ,{BDIC_REG_OPT_TH_SHIFT_4_5,    SHDISP_BDIC_STR,    BDIC_REG_OPT_TH_SHIFT_4_5_VAL,   0xFF, 0}
    ,{BDIC_REG_OPT_TH_SHIFT_6_7,    SHDISP_BDIC_STR,    BDIC_REG_OPT_TH_SHIFT_6_7_VAL,   0xFF, 0}
    ,{BDIC_REG_OPT_TH_SHIFT_8_9,    SHDISP_BDIC_STR,    BDIC_REG_OPT_TH_SHIFT_8_9_VAL,   0xFF, 0}
    ,{BDIC_REG_OPT_TH_SHIFT_11_10,  SHDISP_BDIC_STR,    BDIC_REG_OPT_TH_SHIFT_11_10_VAL, 0xFF, 0}
    ,{BDIC_REG_OPT_TH_SHIFT_13_12,  SHDISP_BDIC_STR,    BDIC_REG_OPT_TH_SHIFT_13_12_VAL, 0xFF, 0}
    ,{BDIC_REG_OPT_TH_SHIFT_15_14,  SHDISP_BDIC_STR,    BDIC_REG_OPT_TH_SHIFT_15_14_VAL, 0xFF, 0}
    ,{BDIC_REG_OPT_TH_SHIFT_17_16,  SHDISP_BDIC_STR,    BDIC_REG_OPT_TH_SHIFT_17_16_VAL, 0xFF, 0}
    ,{BDIC_REG_OPT_TH_SHIFT_19_18,  SHDISP_BDIC_STR,    BDIC_REG_OPT_TH_SHIFT_19_18_VAL, 0xFF, 0}
    ,{BDIC_REG_OPT_TH_SHIFT_21_20,  SHDISP_BDIC_STR,    BDIC_REG_OPT_TH_SHIFT_21_20_VAL, 0xFF, 0}
    ,{BDIC_REG_OPT_TH_SHIFT_23_22,  SHDISP_BDIC_STR,    BDIC_REG_OPT_TH_SHIFT_23_22_VAL, 0xFF, 0}

    ,{BDIC_REG_OPT0_LT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT0_LT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT0_LT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT0_LT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT0_HT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT0_HT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT0_HT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT0_HT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT1_LT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT1_LT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT1_LT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT1_LT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT1_HT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT1_HT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT1_HT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT1_HT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT2_LT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT2_LT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT2_LT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT2_LT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT2_HT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT2_HT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT2_HT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT2_HT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT3_LT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT3_LT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT3_LT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT3_LT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT3_HT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT3_HT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT3_HT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT3_HT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT4_LT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT4_LT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT4_LT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT4_LT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT4_HT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT4_HT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT4_HT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT4_HT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT5_LT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT5_LT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT5_LT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT5_LT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT5_HT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT5_HT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT5_HT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT5_HT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT6_LT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT6_LT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT6_LT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT6_LT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT6_HT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT6_HT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT6_HT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT6_HT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT7_LT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT7_LT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT7_LT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT7_LT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT7_HT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT7_HT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT7_HT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT7_HT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT8_LT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT8_LT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT8_LT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT8_LT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT8_HT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT8_HT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT8_HT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT8_HT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT9_LT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT9_LT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT9_LT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT9_LT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT9_HT_L,           SHDISP_BDIC_STR,    BDIC_REG_OPT9_HT_L_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT9_HT_H,           SHDISP_BDIC_STR,    BDIC_REG_OPT9_HT_H_VAL,     0xFF,      0}
    ,{BDIC_REG_OPT10_LT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT10_LT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT10_LT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT10_LT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT10_HT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT10_HT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT10_HT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT10_HT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT11_LT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT11_LT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT11_LT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT11_LT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT11_HT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT11_HT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT11_HT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT11_HT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT12_LT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT12_LT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT12_LT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT12_LT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT12_HT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT12_HT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT12_HT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT12_HT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT13_LT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT13_LT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT13_LT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT13_LT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT13_HT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT13_HT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT13_HT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT13_HT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT14_LT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT14_LT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT14_LT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT14_LT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT14_HT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT14_HT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT14_HT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT14_HT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT15_LT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT15_LT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT15_LT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT15_LT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT15_HT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT15_HT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT15_HT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT15_HT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT16_LT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT16_LT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT16_LT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT16_LT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT16_HT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT16_HT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT16_HT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT16_HT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT17_LT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT17_LT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT17_LT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT17_LT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT17_HT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT17_HT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT17_HT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT17_HT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT18_LT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT18_LT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT18_LT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT18_LT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT18_HT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT18_HT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT18_HT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT18_HT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT19_LT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT19_LT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT19_LT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT19_LT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT19_HT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT19_HT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT19_HT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT19_HT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT20_LT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT20_LT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT20_LT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT20_LT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT20_HT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT20_HT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT20_HT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT20_HT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT21_LT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT21_LT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT21_LT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT21_LT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT21_HT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT21_HT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT21_HT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT21_HT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT22_LT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT22_LT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT22_LT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT22_LT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT22_HT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT22_HT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT22_HT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT22_HT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT23_LT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT23_LT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT23_LT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT23_LT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT23_HT_L,          SHDISP_BDIC_STR,    BDIC_REG_OPT23_HT_L_VAL,    0xFF,      0}
    ,{BDIC_REG_OPT23_HT_H,          SHDISP_BDIC_STR,    BDIC_REG_OPT23_HT_H_VAL,    0xFF,      0}
    ,{BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
};
#if defined(CONFIG_MACH_TAC)
static const shdisp_bdicRegSetting_t shdisp_bdic_init_no_led[] = {
     {BDIC_REG_GPIO_2,              SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GPIO_3,              SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GPIO_4,              SHDISP_BDIC_STR,    0x02,                       0xFF,      0}
    ,{BDIC_REG_GPIO_6,              SHDISP_BDIC_STR,    BDIC_REG_GPIO_6_VAL,        0xFF,      0}
    ,{BDIC_REG_GPIO_ANSW,           SHDISP_BDIC_STR,    BDIC_REG_GPIO_ANSW_VAL,     0xFF,      0}
    ,{BDIC_REG_GPIMSK1,             SHDISP_BDIC_STR,    BDIC_REG_GPIMSK1_VAL,       0xFF,      0}
    ,{BDIC_REG_GPIMSK2,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
};
static const shdisp_bdicRegSetting_t shdisp_bdic_init_1_led[] = {
     {BDIC_REG_GPIO_3,              SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GPIO_4,              SHDISP_BDIC_STR,    0x02,                       0xFF,      0}
    ,{BDIC_REG_GPIO_6,              SHDISP_BDIC_STR,    BDIC_REG_GPIO_6_VAL,        0xFF,      0}
    ,{BDIC_REG_GPIO_ANSW,           SHDISP_BDIC_STR,    BDIC_REG_GPIO_ANSW_VAL,     0xFF,      0}
    ,{BDIC_REG_GPIMSK1,             SHDISP_BDIC_STR,    BDIC_REG_GPIMSK1_VAL,       0xFF,      0}
    ,{BDIC_REG_GPIMSK2,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
};
static const shdisp_bdicRegSetting_t shdisp_bdic_init_3_led[] = {
     {BDIC_REG_GPIO_6,              SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GPIO_ANSW,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GPIMSK1,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_GPIMSK2,             SHDISP_BDIC_STR,    0x20,                       0xFF,      0}
};
#endif

static const shdisp_bdicRegSetting_t shdisp_bdic_active[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
    ,{BDIC_REG_SYSTEM1,             SHDISP_BDIC_SET,    0x80,                       0x80,   1000}
    ,{BDIC_REG_SYSTEM3,             SHDISP_BDIC_SET,    0x02,                       0x02,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_standby[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
    ,{BDIC_REG_SYSTEM3,             SHDISP_BDIC_CLR,    0x00,                       0x02,      0}
    ,{BDIC_REG_SYSTEM1,             SHDISP_BDIC_CLR,    0x00,                       0x80,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_vsn_on_ts1[] = {
     {BDIC_REG_SYSTEM2,             SHDISP_BDIC_CLR,    0x00,                       0x20,      0}
    ,{BDIC_REG_SYSTEM2,             SHDISP_BDIC_RMW,    0x10,                       0x50, 16*1000}
    ,{BDIC_REG_SYSTEM2,             SHDISP_BDIC_SET,    0x20,                       0x20,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_vsn_on_ts2[] = {
     {BDIC_REG_SYSTEM2,             SHDISP_BDIC_RMW,    0x10,                       0x50, 20*1000}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_vsn_off[] = {
     {BDIC_REG_SYSTEM2,             SHDISP_BDIC_RMW,    0x40,                       0x50,   1000}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_vsp_on[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
    ,{BDIC_REG_DCDC2_TEST_57,       SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_SYSTEM2,             SHDISP_BDIC_SET,    0x01,                       0x01,   5000}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_vsp_off[] = {
     {BDIC_REG_SYSTEM2,             SHDISP_BDIC_CLR,    0x00,                       0x01,      0}
    ,{BDIC_REG_DCDC2_TEST_57,       SHDISP_BDIC_STR,    0x80,                       0xFF,   2000}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_bkl_on[] = {
     {BDIC_REG_SYSTEM6,             SHDISP_BDIC_RMW,    0x01,                       0x55,      0}
    ,{BDIC_REG_OPT_MODE,            SHDISP_BDIC_SET,    0x01,                       0x01,      0}
#ifdef CONFIG_SHDISP_USE_M2LED
    ,{BDIC_REG_SYSTEM1,             SHDISP_BDIC_RMW,    BDIC_REG_SYSTEM1_BKL,       0x03,   5000}
#else
    ,{BDIC_REG_SYSTEM1,             SHDISP_BDIC_RMW,    BDIC_REG_SYSTEM1_BKL,       0x01,   5000}
#endif
};

static const shdisp_bdicRegSetting_t shdisp_bdic_bkl_auto_on[] = {
     {BDIC_REG_SYSTEM6,             SHDISP_BDIC_CLR,    0x00,                       0x01,      0}
    ,{BDIC_REG_OPT_MODE,            SHDISP_BDIC_CLR,    0x00,                       0x01,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_bkl_fix[] = {
     {BDIC_REG_OPT_MODE,            SHDISP_BDIC_SET,    0x01,                       0x01,      0}
    ,{BDIC_REG_SYSTEM6,             SHDISP_BDIC_RMW,    0x01,                       0x05,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_bkl_auto[] = {
     {BDIC_REG_OPT_MODE,            SHDISP_BDIC_CLR,    0x00,                       0x01,      0}
    ,{BDIC_REG_SYSTEM6,             SHDISP_BDIC_CLR,    0x00,                       0x05,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_bkl_fix_dbc[] = {
     {BDIC_REG_OPT_MODE,            SHDISP_BDIC_SET,    0x01,                       0x01,      0}
    ,{BDIC_REG_SYSTEM6,             SHDISP_BDIC_SET,    0x05,                       0x05,      0}
};


static shdisp_bdicRegSetting_t shdisp_bdic_bkl_slope_fast[] = {
     {BDIC_REG_SLOPE,               SHDISP_BDIC_STR,    0x11,                       0xFF,      0}
};
static const shdisp_bdicRegSetting_t shdisp_bdic_bkl_slope_slow[] = {
     {BDIC_REG_SLOPE,               SHDISP_BDIC_STR,    0xFF,                       0xFF,      0}
};


static const shdisp_bdicRegSetting_t shdisp_bdic_bkl_opt_mode_off[] = {
     {BDIC_REG_OPT_MODE,            SHDISP_BDIC_SET,    0x01,                       0x01,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_bkl_opt_mode_on[] = {
     {BDIC_REG_OPT_MODE,            SHDISP_BDIC_CLR,    0x00,                       0x01,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_bkl_auto_dbc[] = {
     {BDIC_REG_OPT_MODE,            SHDISP_BDIC_CLR,    0x00,                       0x01,      0}
    ,{BDIC_REG_SYSTEM6,             SHDISP_BDIC_RMW,    0x04,                       0x05,      0}
};

static shdisp_bdicRegSetting_t shdisp_bdic_bkl_led_value[] = {
     {BDIC_REG_M1LED,               SHDISP_BDIC_STR,    BDIC_REG_M1LED_VAL,         0xFF,      0}
#ifdef CONFIG_SHDISP_USE_M2LED
    ,{BDIC_REG_M2LED,               SHDISP_BDIC_STR,    BDIC_REG_M2LED_VAL,         0xFF,      0}
#endif
};

static shdisp_bdicRegSetting_t shdisp_bdic_bkl_opt_value[] = {
     {BDIC_REG_OPT0,                SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT1,                SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT2,                SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT3,                SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT4,                SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT5,                SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT6,                SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT7,                SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT8,                SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT9,                SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT10,               SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT11,               SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT12,               SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT13,               SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT14,               SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT15,               SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT16,               SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT17,               SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT18,               SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT19,               SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT20,               SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT21,               SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT22,               SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
    ,{BDIC_REG_OPT23,               SHDISP_BDIC_STRM,    0x00,         0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_bkl_post_start[] = {
     {BDIC_REG_SYSTEM6,             SHDISP_BDIC_RMW,    0x01,                       0x05,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_bkl_off[] = {
#ifdef CONFIG_SHDISP_USE_M2LED
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_CLR,    0x00,                       0x03,  15000}
#else
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_CLR,    0x00,                       0x01,  15000}
#endif
    ,{BDIC_REG_SYSTEM6,             SHDISP_BDIC_RMW,    0x01,                       0x05,      0}
};

static shdisp_bdicRegSetting_t shdisp_bdic_als_sensor_adjust[] = {
     {BDIC_REG_ALS_ADJ0_0_L,        SHDISP_BDIC_STR,   BDIC_REG_ALS_ADJ0_L_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ0_0_H,        SHDISP_BDIC_STR,   BDIC_REG_ALS_ADJ0_H_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ1_0_L,        SHDISP_BDIC_STR,   BDIC_REG_ALS_ADJ1_L_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ1_0_H,        SHDISP_BDIC_STR,   BDIC_REG_ALS_ADJ1_H_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_SHIFT_0,         SHDISP_BDIC_STR,   BDIC_REG_ALS_SHIFT_DEFAULT_B,  0xFF,   0}
    ,{BDIC_REG_ALS_ADJ0_1_L,        SHDISP_BDIC_STR,   BDIC_REG_ALS_ADJ0_L_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ0_1_H,        SHDISP_BDIC_STR,   BDIC_REG_ALS_ADJ0_H_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ1_1_L,        SHDISP_BDIC_STR,   BDIC_REG_ALS_ADJ1_L_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ1_1_H,        SHDISP_BDIC_STR,   BDIC_REG_ALS_ADJ1_H_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_SHIFT_1,         SHDISP_BDIC_STR,   BDIC_REG_ALS_SHIFT_DEFAULT_B,  0xFF,   0}
    ,{BDIC_REG_ALS_ADJ0_2_L,        SHDISP_BDIC_STR,   BDIC_REG_ALS_ADJ0_L_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ0_2_H,        SHDISP_BDIC_STR,   BDIC_REG_ALS_ADJ0_H_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ1_2_L,        SHDISP_BDIC_STR,   BDIC_REG_ALS_ADJ1_L_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ1_2_H,        SHDISP_BDIC_STR,   BDIC_REG_ALS_ADJ1_H_DEFAULT_B, 0xFF,   0}
    ,{BDIC_REG_ALS_SHIFT_2,         SHDISP_BDIC_STR,   BDIC_REG_ALS_SHIFT_DEFAULT_B,  0xFF,   0}
    ,{BDIC_REG_ALS_ADJ0_3_L,        SHDISP_BDIC_STR,   BDIC_REG_ALS_ADJ0_L_DEFAULT_A, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ0_3_H,        SHDISP_BDIC_STR,   BDIC_REG_ALS_ADJ0_H_DEFAULT_A, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ1_3_L,        SHDISP_BDIC_STR,   BDIC_REG_ALS_ADJ1_L_DEFAULT_A, 0xFF,   0}
    ,{BDIC_REG_ALS_ADJ1_3_H,        SHDISP_BDIC_STR,   BDIC_REG_ALS_ADJ1_H_DEFAULT_A, 0xFF,   0}
    ,{BDIC_REG_ALS_SHIFT_3,         SHDISP_BDIC_STR,   BDIC_REG_ALS_SHIFT_DEFAULT_A,  0xFF,   0}
};

#if defined(CONFIG_MACH_PA25) || defined(CONFIG_MACH_PB25) || defined(CONFIG_MACH_KF10) || defined(CONFIG_MACH_PA28)
static shdisp_bdicRegSetting_t shdisp_bdic_led_fix_on[] = {
     {BDIC_REG_CH0_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH1_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH2_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH0_B,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH1_B,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH2_B,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH0_C,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH1_C,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH2_C,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET1,            SHDISP_BDIC_SET,    0x20,                       0x20,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_SET,    0x20,                       0x20,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_SET,    0x20,                       0x20,      0}
    ,{BDIC_REG_SYSTEM5,             SHDISP_BDIC_SET,    0x07,                       0x07,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_ani_on[] = {
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_SET,    0x07,                       0x07,      0}
    ,{BDIC_REG_SYSTEM7,             SHDISP_BDIC_RMW,    0x01,                       0xF3,   6000}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_lposc_enable[] = {
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_SET,    0x10,                       0x10,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_off[] = {
     {BDIC_REG_SYSTEM7,             SHDISP_BDIC_CLR,    0x00,                       0xF3,      0}
    ,{BDIC_REG_SYSTEM5,             SHDISP_BDIC_CLR,    0x00,                       0x07,   5500}
    ,{BDIC_REG_SYSTEM1,             SHDISP_BDIC_CLR,    0x00,                       0x10,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_off_fix[] = {
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_CLR,    0x00,                       0x07,   5500}
};
#endif

#if defined(CONFIG_MACH_TAC)
static shdisp_bdicRegSetting_t shdisp_bdic_led_fix_on[] = {
     {BDIC_REG_CH3_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH4_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH5_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH3_B,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH4_B,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH5_B,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH3_C,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH4_C,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH5_C,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH3_SET1,            SHDISP_BDIC_SET,    0x20,                       0x20,      0}
    ,{BDIC_REG_CH4_SET1,            SHDISP_BDIC_SET,    0x20,                       0x20,      0}
    ,{BDIC_REG_CH5_SET1,            SHDISP_BDIC_SET,    0x20,                       0x20,      0}
    ,{BDIC_REG_SYSTEM5,             SHDISP_BDIC_SET,    0x38,                       0x38,      0}
};


static const shdisp_bdicRegSetting_t shdisp_bdic_led_ani_on[] = {
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_SET,    0x38,                       0x38,      0}
    ,{BDIC_REG_SYSTEM7,             SHDISP_BDIC_RMW,    0x04,                       0xFC,   6000}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_lposc_enable[] = {
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_SET,    0x10,                       0x10,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_lposc_disable[] = {
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_CLR,    0x00,                       0x10,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_off[] = {
     {BDIC_REG_SYSTEM7,             SHDISP_BDIC_CLR,    0x00,                       0xFC,      0}
    ,{BDIC_REG_SYSTEM5,             SHDISP_BDIC_CLR,    0x00,                       0x38,   5500}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_off_fix[] = {
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_CLR,    0x00,                       0x38,   5500}
};
#endif

#if defined(CONFIG_MACH_TAC)
static shdisp_bdicRegSetting_t shdisp_bdic_key_led_fix_on[] = {
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_SET,    0x07,                       0x07,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_key_led_ani_on[] = {
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_SET,    0x07,                       0x07,      0}
    ,{BDIC_REG_SYSTEM7,             SHDISP_BDIC_RMW,    0x01,                       0xF3,   6000}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_key_led_off[] = {
     {BDIC_REG_SYSTEM7,             SHDISP_BDIC_CLR,    0x00,                       0xF3,      0}
    ,{BDIC_REG_SYSTEM5,             SHDISP_BDIC_CLR,    0x00,                       0x07,   5500}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_key_led_off_fix[] = {
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_CLR,    0x00,                       0x07,   5500}
};
#endif

static const shdisp_bdicRegSetting_t shdisp_bdic_sensor_power_on[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
    ,{BDIC_REG_SYSTEM4,             SHDISP_BDIC_SET,    0x02,                       0x02,   5000}
    ,{BDIC_REG_GIMR4,               SHDISP_BDIC_SET,    0x08,                       0x08,      0}
    ,{BDIC_REG_I2C_DATA6,           SHDISP_BDIC_STR,    0x0C,                       0xFF,      0}
    ,{BDIC_REG_I2C_SET,             SHDISP_BDIC_RMW,    0x50,                       0x70,      0}
    ,{BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x10,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_sensor_power_off[] = {
     {BDIC_REG_SYSTEM4,             SHDISP_BDIC_CLR,    0x00,                       0x02,      0}
};

static shdisp_bdicRegSetting_t shdisp_bdic_ps_init_set_threshold[] = {
     {SENSOR_REG_PS_LT_LSB,         SHDISP_ALS_STRMS,   SENSOR_REG_PS_LT_LSB_VAL,   0xFF,       0}
    ,{SENSOR_REG_PS_LT_MSB,         SHDISP_ALS_STRM,    SENSOR_REG_PS_LT_MSB_VAL,   0xFF,       0}
    ,{SENSOR_REG_PS_HT_LSB,         SHDISP_ALS_STRM,    SENSOR_REG_PS_HT_LSB_VAL,   0xFF,       0}
    ,{SENSOR_REG_PS_HT_MSB,         SHDISP_ALS_STRM,    SENSOR_REG_PS_HT_MSB_VAL,   0xFF,       0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_psals_init[] = {
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STRMS,   0x00,                       0xFF,      0}
    ,{SENSOR_REG_COMMAND2,          SHDISP_ALS_STRM,    SENSOR_REG_COMMAND2_INI_VAL,0xFF,      0}
    ,{SENSOR_REG_COMMAND3,          SHDISP_ALS_STRM,    0x10,                       0xFF,      0}
    ,{SENSOR_REG_COMMAND4,          SHDISP_ALS_STRM,    0x00,                       0xFF,      0}

    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xD0,                       0xFF,      0}

};

static const shdisp_bdicRegSetting_t shdisp_bdic_psals_set[] = {
     {BDIC_REG_AR_DATA6,          SHDISP_BDIC_STR,    0xD0,                       0xFF,      0}
    ,{BDIC_REG_SYSTEM8,           SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_AR_CTRL,           SHDISP_BDIC_SET,    0x40,                       0x40, (10*1000)}
    ,{BDIC_REG_SYSTEM8,           SHDISP_BDIC_STR,    0x04,                       0xFF,  (1*1000)}
};
static const shdisp_bdicRegSetting_t shdisp_bdic_psals_standby[] = {
     {BDIC_REG_AR_CTRL,           SHDISP_BDIC_CLR,    0x00,                       0x40,      0}
    ,{BDIC_REG_SYSTEM8,           SHDISP_BDIC_STR,    0x10,                       0xFF,      0}
    ,{SENSOR_REG_COMMAND1,        SHDISP_ALS_STR,     0x00,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_ps_init_als_off1[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
    ,{BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x10,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STRMS,   0x00,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND2,          SHDISP_ALS_STRM,    0x00,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND3,          SHDISP_ALS_STRM,    0x10,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND4,          SHDISP_ALS_STRM,    0xEC,                       0xFF,       0}
};

#define shdisp_bdic_ps_init_als_off2    (shdisp_bdic_ps_init_set_threshold)

static const shdisp_bdicRegSetting_t shdisp_bdic_ps_init_als_off3[] = {
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xEC,                       0xFF,       0}
    ,{BDIC_REG_AR_DATA6,            SHDISP_BDIC_STR,    0xEC,                       0xFF,       0}
    ,{BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x00,                       0xFF,       0}
    ,{BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x04,                       0xFF,       (35*1000)}
    ,{BDIC_REG_GIMR1,               SHDISP_BDIC_STR,    0x08,                       0xFF,       0}
    ,{BDIC_REG_GIMF1,               SHDISP_BDIC_STR,    0x08,                       0xFF,       0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_ps_deinit_als_off1[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
    ,{BDIC_REG_GIMR1,               SHDISP_BDIC_STR,    0x00,                       0xFF,       0}
    ,{BDIC_REG_GIMF1,               SHDISP_BDIC_STR,    0x00,                       0xFF,       0}
    ,{BDIC_REG_GIMR4,               SHDISP_BDIC_CLR,    0x00,                       0x08,       0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_ps_deinit_als_off2[] = {
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x10,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0x80,                       0xFF,    1000}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0x00,                       0xFF,       0}
    ,{BDIC_REG_AR_DATA6,            SHDISP_BDIC_STR,    0x00,                       0xFF,       0}
    ,{BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x00,                       0xFF,       0}
};

static shdisp_bdicRegSetting_t shdisp_bdic_ps_init_als_on1[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_ps_init_als_on2[] = {
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x10,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0x00,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND2,          SHDISP_ALS_RMW,     0x20,                       0xF8,       0}
    ,{SENSOR_REG_COMMAND3,          SHDISP_ALS_STRMS,   0x10,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND4,          SHDISP_ALS_STRM,    0xEC,                       0xFF,       0}
};

#define shdisp_bdic_ps_init_als_on3     (shdisp_bdic_ps_init_set_threshold)

static const shdisp_bdicRegSetting_t shdisp_bdic_ps_init_als_on4[] = {
     {SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xCC,                           0xFF,       0}
    ,{BDIC_REG_AR_DATA6,            SHDISP_BDIC_STR,    0xCC,                           0xFF,       0}
    ,{BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x00,                           0xFF,       0}
    ,{BDIC_REG_AR_CTRL,             SHDISP_BDIC_STR,    0x9E,                           0xFF,       0}
    ,{BDIC_REG_AR_RANGE_L,          SHDISP_BDIC_STRM,   BDIC_REG_AR_RANGE_L_VAL_PS_ON,  0xFF,       0}
    ,{BDIC_REG_AR_RANGE_M,          SHDISP_BDIC_STRM,   BDIC_REG_AR_RANGE_M_VAL_PS_ON,  0xFF,       0}
    ,{BDIC_REG_AR_RANGE_H,          SHDISP_BDIC_STRM,   BDIC_REG_AR_RANGE_H_VAL_PS_ON,  0xFF,       0}
    ,{BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x04,                           0xFF,       (35*1000)}
    ,{BDIC_REG_GIMR1,               SHDISP_BDIC_STR,    0x08,                           0xFF,       0}
    ,{BDIC_REG_GIMF1,               SHDISP_BDIC_STR,    0x08,                           0xFF,       0}
};


static const shdisp_bdicRegSetting_t shdisp_bdic_ps_deinit_als_on1[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_ps_deinit_als_on2[] = {
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x10,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0x00,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND2,          SHDISP_ALS_RMW,     0x18,                       0xF8,       0}
    ,{SENSOR_REG_COMMAND3,          SHDISP_ALS_STRMS,   0x10,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND4,          SHDISP_ALS_STRM,    0x00,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xD0,                       0xFF,       0}

    ,{BDIC_REG_AR_DATA6,            SHDISP_BDIC_STR,    0xD0,                       0xFF,       0}
    ,{BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x00,                       0xFF,       0}
    ,{BDIC_REG_AR_CTRL,             SHDISP_BDIC_STR,    0x9E,                       0xFF,       0}
    ,{BDIC_REG_AR_RANGE_L,          SHDISP_BDIC_STRM,   BDIC_REG_AR_RANGE_L_VAL,    0xFF,       0}
    ,{BDIC_REG_AR_RANGE_M,          SHDISP_BDIC_STRM,   BDIC_REG_AR_RANGE_M_VAL,    0xFF,       0}
    ,{BDIC_REG_AR_RANGE_H,          SHDISP_BDIC_STRM,   BDIC_REG_AR_RANGE_H_VAL,    0xFF,       0}
    ,{BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x04,                       0xFF,       0}
    ,{BDIC_REG_GIMR1,               SHDISP_BDIC_STR,    0x00,                       0xFF,       0}
    ,{BDIC_REG_GIMF1,               SHDISP_BDIC_STR,    0x00,                       0xFF,       0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_als_init_ps_off[] = {
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x10,                       0xFF,       0}

    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0x00,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND2,          SHDISP_ALS_STR,     0x18,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND3,          SHDISP_ALS_STRMS,   0x10,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND4,          SHDISP_ALS_STRM,    0x00,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xD0,                       0xFF,       0}

    ,{BDIC_REG_AR_DATA6,            SHDISP_BDIC_STR,    0xD0,                       0xFF,       0}
    ,{BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x00,                       0xFF,       0}
    ,{BDIC_REG_AR_CTRL,             SHDISP_BDIC_STR,    0x9E,                       0xFF,       0}
    ,{BDIC_REG_AR_RANGE_L,          SHDISP_BDIC_STRM,   BDIC_REG_AR_RANGE_L_VAL,    0xFF,       0}
    ,{BDIC_REG_AR_RANGE_M,          SHDISP_BDIC_STRM,   BDIC_REG_AR_RANGE_M_VAL,    0xFF,       0}
    ,{BDIC_REG_AR_RANGE_H,          SHDISP_BDIC_STRM,   BDIC_REG_AR_RANGE_H_VAL,    0xFF,       0}
    ,{BDIC_REG_SYSTEM8,             SHDISP_BDIC_SET,    0x04,                       0x04,       0}
    ,{BDIC_REG_GIMR1,               SHDISP_BDIC_STR,    0x00,                       0xFF,       0}
    ,{BDIC_REG_GIMF1,               SHDISP_BDIC_STR,    0x00,                       0xFF,       0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_als_init_ps_on1[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_als_init_ps_on2[] = {
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x10,                           0xFF,       0}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0x0C,                           0xFF,       0}
    ,{SENSOR_REG_COMMAND2,          SHDISP_ALS_RMW,     0x20,                           0xF8,       0}
    ,{SENSOR_REG_COMMAND3,          SHDISP_ALS_STR,     0x10,                           0xFF,       0}
    ,{SENSOR_REG_COMMAND4,          SHDISP_ALS_RMW,     0xCC,                           0xCF,       0}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xCC,                           0xFF,       0}

    ,{BDIC_REG_AR_DATA6,            SHDISP_BDIC_STR,    0xCC,                           0xFF,       0}
    ,{BDIC_REG_SYSTEM8  ,           SHDISP_BDIC_STR,    0x00,                           0xFF,       0}
    ,{BDIC_REG_AR_CTRL,             SHDISP_BDIC_STR,    0x9E,                           0xFF,       0}
    ,{BDIC_REG_AR_RANGE_L,          SHDISP_BDIC_STRM,   BDIC_REG_AR_RANGE_L_VAL_PS_ON,  0xFF,       0}
    ,{BDIC_REG_AR_RANGE_M,          SHDISP_BDIC_STRM,   BDIC_REG_AR_RANGE_M_VAL_PS_ON,  0xFF,       0}
    ,{BDIC_REG_AR_RANGE_H,          SHDISP_BDIC_STRM,   BDIC_REG_AR_RANGE_H_VAL_PS_ON,  0xFF,       0}
    ,{BDIC_REG_SYSTEM8  ,           SHDISP_BDIC_STR,    0x04,                           0xFF,       0}
    ,{BDIC_REG_GIMR1,               SHDISP_BDIC_STR,    0x08,                           0xFF,       0}
    ,{BDIC_REG_GIMF1,               SHDISP_BDIC_STR,    0x08,                           0xFF,       0}
};

static shdisp_bdicRegSetting_t const shdisp_bdic_als_auto_light_bdic[] = {
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_SET,    0x04,                       0x04,       0}
};

static shdisp_bdicRegSetting_t const shdisp_bdic_als_auto_light_bdic_wait[] = {
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_SET,    0x04,                       0x04,       0}
    ,{BDIC_REG_NONE,                SHDISP_BDIC_WAIT,   0x00,                       0x00,   35000}
};

#define shdisp_bdic_als_deinit_ps_off1   (shdisp_bdic_ps_deinit_als_off1)
#define shdisp_bdic_als_deinit_ps_off2   (shdisp_bdic_ps_deinit_als_off2)

static const shdisp_bdicRegSetting_t shdisp_bdic_als_deinit_ps_on1[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_als_deinit_ps_on2[] = {
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x10,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STRMS,   0x0C,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND2,          SHDISP_ALS_STRM,    0x00,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND3,          SHDISP_ALS_STRM,    0x10,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND4,          SHDISP_ALS_STRM,    0xEC,                       0xFF,       0}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xEC,                       0xFF,       0}
    ,{BDIC_REG_AR_DATA6,            SHDISP_BDIC_STR,    0xEC,                       0xFF,       0}
    ,{BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x00,                       0xFF,       0}
    ,{BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x04,                       0xFF,       0}
    ,{BDIC_REG_GIMR1,               SHDISP_BDIC_STR,    0x08,                       0xFF,       0}
    ,{BDIC_REG_GIMF1,               SHDISP_BDIC_STR,    0x08,                       0xFF,       0}
};


static const shdisp_bdicRegSetting_t shdisp_bdic_i2ctimer_stop[] = {
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x08,                       0xFF,       0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_i2ctimer_start[] = {
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x04,                       0xFF,       0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_dcdc1_err[] = {
#ifdef CONFIG_SHDISP_USE_M2LED
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_CLR,    0x00,                       0x03,       0}
#else
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_CLR,    0x00,                       0x01,       0}
#endif
};

static shdisp_bdicRegSetting_t const shdisp_bdic_reg_ar_ctrl[] = {
    {BDIC_REG_AR_CTRL,             SHDISP_BDIC_STR,    0x9A,                       0xFF,      0}
};

static shdisp_bdicRegSetting_t const shdisp_bdic_i2c_throughmode_on[] = {
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x10,                       0xFF,       0}
};

static shdisp_bdicRegSetting_t const shdisp_bdic_i2c_throughmode_off[] = {
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x00,                       0xFF,       0}
    ,{BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x04,                       0xFF,       0}
};

static shdisp_bdicRegSetting_t shdisp_bdic_restore_regs_for_ave_ado_bdic[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
    ,{BDIC_REG_AR_CTRL,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_DATA_SHIFT_ML,       SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x01,                       0x00,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_1_0,    SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_3_2,    SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_4_5,    SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_6_7,    SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_8_9,    SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_11_10,  SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_13_12,  SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_15_14,  SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_17_16,  SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_19_18,  SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_21_20,  SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_23_22,  SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
};

static shdisp_bdicRegSetting_t shdisp_bdic_restore_regs_for_ave_ado_als[] = {
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x10,                       0xFF,      0}
    ,{SENSOR_REG_COMMAND2,          SHDISP_ALS_STR,     0x00,                       0xFF,      0}
    ,{BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
};

static shdisp_bdicRegSetting_t const shdisp_bdic_ave_ado_param1[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
    ,{BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x08,                       0xFF,   5000}
    ,{BDIC_REG_AR_CTRL,             SHDISP_BDIC_STR,    0x5E,                       0xFF,      0}
    ,{BDIC_REG_DATA_SHIFT_ML,       SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x01,                       0x00,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_1_0,    SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_3_2,    SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_4_5,    SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_6_7,    SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_8_9,    SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_11_10,  SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_13_12,  SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_15_14,  SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_17_16,  SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_19_18,  SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_21_20,  SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_OPT_TH_SHIFT_23_22,  SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
    ,{BDIC_REG_AR_CTRL,             SHDISP_BDIC_STR,    0xDA,                       0xFF,      0}
};

static shdisp_bdicRegSetting_t shdisp_bdic_ave_ado_param2[] = {
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x10,                       0xFF,      0}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0x50,                       0xFF,      0}
    ,{SENSOR_REG_COMMAND2,          SHDISP_ALS_STR,     0x18,                       0xFF,      0}
    ,{SENSOR_REG_COMMAND1,          SHDISP_ALS_STR,     0xD0,                       0xFF,      0}
    ,{BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0}
    ,{BDIC_REG_NONE,                SHDISP_BDIC_WAIT,   0x00,                       0x00, 100000}
};

static shdisp_bdicRegSetting_t const shdisp_bdic_i2ctimer_stop_ave_ado[] = {
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x08,                       0xFF,   5000}
};

static shdisp_bdicRegSetting_t const shdisp_bdic_i2ctimer_start_ave_ado[] = {
     {BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x04,                       0xFF,   1000}
};

#if defined (CONFIG_ANDROID_ENGINEERING)
static shdisp_bdicRegSetting_t shdisp_bdic_restore_regs_for_bank1_dump[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0}
    ,{BDIC_REG_SYSTEM8,             SHDISP_BDIC_STR,    0x04,                       0xFF,   1000}
};
#endif /* CONFIG_ANDROID_ENGINEERING */

#endif /* SHDISP_BL71Y6_CTRL_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
