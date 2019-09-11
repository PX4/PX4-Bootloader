/* this file lists all board types for boards that use the bootloader
 * protocol implemented in this bootloader.
 * /

#pragma once

/* NAME, TYPE, ID, BOOTLOADER_START, APP_LOAD_SECTOR, RESERVED_SECTORS_START, RESERVED_SECTORS_END */
#define BOARD_TABLE \
BOARD(px4_fmu-v1,            5,      5,  0, 2, 0, 0, 1024) \
BOARD(px4_flow-v1,           6,      6,  0, 2, 0, 0, 1024) \
BOARD(px4_pio-v1,            7,      7,  0, 2, 0, 0,   64) \
BOARD(px4_fmu-v2,            9,      9,  0, 2, 0, 0, 1024) \
BOARD(px4_fmu-v3,            9,      9,  0, 2, 0, 0, 2048) \
BOARD(px4_pio-v2,           10,     10,  0, 2, 0, 0,   64) \
BOARD(px4_fmu-v4,           11,     11,  0, 2, 0, 0, 2048) \
BOARD(bitcraze_crazyflie,   12,     12,  0, 2, 0, 0, 2048) \
BOARD(px4_fmu-v4pro,        13,     13,  0, 2, 0, 0, 2048) \
BOARD(gumstiz_aerocore-v1,  98,     19,  0, 2, 0, 0, 2048) \
BOARD(AP_HW_F4BY-v1,        20,     20,  0, 2, 0, 0, 2048) \
BOARD(px4_cannode-v1,       22,     22,  0, 2, 0, 0, 2048) \
BOARD(s2740vc-v1,           23,     23,  0, 2, 0, 0, 2048) \
BOARD(px4_flow-v2,          24,     24,  0, 2, 0, 0, 2048) \
BOARD(px4_esc-v1,           25,     25,  0, 2, 0, 0, 2048) \
BOARD(zubax_gnss-v1,        26,     26,  0, 2, 0, 0, 2048) \
BOARD(esc35-v1,             27,     27,  0, 2, 0, 0, 2048) \
BOARD(nxp_hlite-v1,         28,     28,  0, 2, 0, 0, 2048) \
BOARD(nxp_hlite-v3,         29,     29,  0, 2, 0, 0, 2048) \
BOARD(smartap_pro,          32,     32,  0, 2, 0, 0, 2048) \
BOARD(auav_x21,             33,     33,  0, 2, 0, 0, 2048) \
BOARD(px4_fmu-v5,           50,     50,  0, 2, 0, 0, 2048) \
BOARD(px4_fmu-v5x,          51,     51,  0, 2, 0, 0, 2048) \
BOARD(px4_fmu-v6,           52,     52,  0, 2, 0, 0, 2048) \
BOARD(px4_fmu-v6x,          53,     53,  0, 2, 0, 0, 2048) \
BOARD(tap_v1,               64,     64,  0, 2, 0, 0, 2048) \
BOARD(intel_aerofc-v1,      65,     65,  0, 2, 0, 0, 1024) \
BOARD(WIP_66,               66,     66,  0, 2, 0, 0, 2048) \
BOARD(WIP_67,               67,     67,  0, 2, 0, 0, 2048) \
BOARD(airmind_mindpx-v2,    88,     88,  0, 2, 0, 0, 2048) \
BOARD(WIP_91                91,     91,  0, 2, 0, 0, 2048) \
BOARD(WIP_96,               96,     96,  0, 2, 0, 0, 2048) \
BOARD(gumstiz_aerocore-v2,  98,     98,  0, 2, 0, 0, 2048) \
BOARD(stm32discoveryf4,     99,     99,  0, 2, 0, 0, 2048) \
BOARD(same70xplained-v1,   110,    110,  0, 2, 0, 0, 2048) \
BOARD(cubepilot_yellow,    120,    120,  0, 4, 2, 0, 2048) \
BOARD(omnibus-f7v2,        121,    121,  0, 4, 2, 0, 2048) \
BOARD(holybro_kakutef4,    122,    122,  0, 2, 0, 0, 2048) \
BOARD(holybro_kakutef7,    123,    123,  0, 2, 0, 0, 2048) \
BOARD(revolution,          124,    124,  0, 2, 0, 0, 2048) \
BOARD(matek-f405,          125,    125,  0, 2, 0, 0, 2048) \
BOARD(nucleo-f767zi,       126,    126,  0, 2, 0, 0, 2048) \
BOARD(matek-f405wing,      127,    127,  0, 2, 0, 0, 2048) \
BOARD(airbot-f4,           128,    128,  0, 2, 0, 0, 2048) \
BOARD(sparky-v2,           130,    130,  0, 2, 0, 0, 2048) \
BOARD(omnibus-f4pro,       131,    131,  0, 2, 0, 0, 2048) \
BOARD(anyfcf7,             132,    132,  0, 2, 0, 0, 2048) \
BOARD(omnibus_nano-v6,     133,    133,  0, 2, 0, 0, 2048) \
BOARD(speedybee-f4,        134,    134,  0, 2, 0, 0, 2048) \
BOARD(f35lightning,        135,    135,  0, 2, 0, 0, 2048) \
BOARD(mro_x21-777,         136,    136,  0, 2, 0, 0, 2048) \
BOARD(omnibus_f4-v6,       137,    137,  0, 2, 0, 0, 2048) \
BOARD(heliospring,         138,    138,  0, 2, 0, 0, 2048) \
BOARD(holybro_durandal,    139,    139,  0, 2, 0, 0, 2048) \
BOARD(cubepilot_orange,    140,    140,  0, 2, 0, 0, 2048) \
BOARD(mro_ctrl-zero,       141,    141,  0, 2, 0, 0, 2048) \
BOARD(mro_ctrl-zero-oem,   142,    142,  0, 2, 0, 0, 2048) \
BOARD(matek_f765-wing,     143,    143,  0, 2, 0, 0, 2048) \
BOARD(jdmini-f405,         144,    144,  0, 2, 0, 0, 2048) \
BOARD(kakute-f7-mini,      145,    145,  0, 2, 0, 0, 2048) \
BOARD(f103_periph,        1000,   1000,  0, 2, 0, 0,   64) \
BOARD(cuav_gps-v1,        1001,   1001,  0, 2, 0, 0,   64) \
BOARD(omnibus_f4periph,   1002,   1002,  0, 2, 0, 0,   64) \
BOARD(cubepilot_black+,   1004,   1004,  0, 2, 0, 0,   64) \
BOARD(vrbrain_v51,        1151,   1151,  0, 2, 0, 0, 2048) \
BOARD(vrbrain_v52,        1152,   1152,  0, 2, 0, 0, 2048) \
BOARD(vrbrain_v54,        1154,   1154,  0, 2, 0, 0, 2048) \
BOARD(vrubrain_v51,       1351,   1351,  0, 2, 0, 0, 2048) \
BOARD(vrcore-v10,         1910,   1910,  0, 2, 0, 0, 2048) \

