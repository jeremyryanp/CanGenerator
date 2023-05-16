
#include "Arduino.h"

const uint16_t sineTab[] = {
    2048, 2060, 2073, 2085, 2098, 2110, 2123, 2135, 2148, 2161, 2173, 2186, 2198, 2211, 2223, 2236,
    2248, 2261, 2273, 2286, 2298, 2311, 2323, 2335, 2348, 2360, 2373, 2385, 2398, 2410, 2422, 2435,
    2447, 2459, 2472, 2484, 2496, 2508, 2521, 2533, 2545, 2557, 2569, 2581, 2594, 2606, 2618, 2630,
    2642, 2654, 2666, 2678, 2690, 2702, 2714, 2725, 2737, 2749, 2761, 2773, 2784, 2796, 2808, 2819,
    2831, 2843, 2854, 2866, 2877, 2889, 2900, 2912, 2923, 2934, 2946, 2957, 2968, 2979, 2990, 3002,
    3013, 3024, 3035, 3046, 3057, 3068, 3078, 3089, 3100, 3111, 3122, 3132, 3143, 3154, 3164, 3175,
    3185, 3195, 3206, 3216, 3226, 3237, 3247, 3257, 3267, 3277, 3287, 3297, 3307, 3317, 3327, 3337,
    3346, 3356, 3366, 3375, 3385, 3394, 3404, 3413, 3423, 3432, 3441, 3450, 3459, 3468, 3477, 3486,
    3495, 3504, 3513, 3522, 3530, 3539, 3548, 3556, 3565, 3573, 3581, 3590, 3598, 3606, 3614, 3622,
    3630, 3638, 3646, 3654, 3662, 3669, 3677, 3685, 3692, 3700, 3707, 3714, 3722, 3729, 3736, 3743,
    3750, 3757, 3764, 3771, 3777, 3784, 3791, 3797, 3804, 3810, 3816, 3823, 3829, 3835, 3841, 3847,
    3853, 3859, 3865, 3871, 3876, 3882, 3888, 3893, 3898, 3904, 3909, 3914, 3919, 3924, 3929, 3934,
    3939, 3944, 3949, 3953, 3958, 3962, 3967, 3971, 3975, 3980, 3984, 3988, 3992, 3996, 3999, 4003,
    4007, 4010, 4014, 4017, 4021, 4024, 4027, 4031, 4034, 4037, 4040, 4042, 4045, 4048, 4051, 4053,
    4056, 4058, 4060, 4063, 4065, 4067, 4069, 4071, 4073, 4075, 4076, 4078, 4080, 4081, 4083, 4084,
    4085, 4086, 4087, 4088, 4089, 4090, 4091, 4092, 4093, 4093, 4094, 4094, 4094, 4095, 4095, 4095,
    4095, 4095, 4095, 4095, 4094, 4094, 4094, 4093, 4093, 4092, 4091, 4090, 4089, 4088, 4087, 4086,
    4085, 4084, 4083, 4081, 4080, 4078, 4076, 4075, 4073, 4071, 4069, 4067, 4065, 4063, 4060, 4058,
    4056, 4053, 4051, 4048, 4045, 4042, 4040, 4037, 4034, 4031, 4027, 4024, 4021, 4017, 4014, 4010,
    4007, 4003, 3999, 3996, 3992, 3988, 3984, 3980, 3975, 3971, 3967, 3962, 3958, 3953, 3949, 3944,
    3939, 3934, 3929, 3924, 3919, 3914, 3909, 3904, 3898, 3893, 3888, 3882, 3876, 3871, 3865, 3859,
    3853, 3847, 3841, 3835, 3829, 3823, 3816, 3810, 3804, 3797, 3791, 3784, 3777, 3771, 3764, 3757,
    3750, 3743, 3736, 3729, 3722, 3714, 3707, 3700, 3692, 3685, 3677, 3669, 3662, 3654, 3646, 3638,
    3630, 3622, 3614, 3606, 3598, 3590, 3581, 3573, 3565, 3556, 3548, 3539, 3530, 3522, 3513, 3504,
    3495, 3486, 3477, 3468, 3459, 3450, 3441, 3432, 3423, 3413, 3404, 3394, 3385, 3375, 3366, 3356,
    3346, 3337, 3327, 3317, 3307, 3297, 3287, 3277, 3267, 3257, 3247, 3237, 3226, 3216, 3206, 3195,
    3185, 3175, 3164, 3154, 3143, 3132, 3122, 3111, 3100, 3089, 3078, 3068, 3057, 3046, 3035, 3024,
    3013, 3002, 2990, 2979, 2968, 2957, 2946, 2934, 2923, 2912, 2900, 2889, 2877, 2866, 2854, 2843,
    2831, 2819, 2808, 2796, 2784, 2773, 2761, 2749, 2737, 2725, 2714, 2702, 2690, 2678, 2666, 2654,
    2642, 2630, 2618, 2606, 2594, 2581, 2569, 2557, 2545, 2533, 2521, 2508, 2496, 2484, 2472, 2459,
    2447, 2435, 2422, 2410, 2398, 2385, 2373, 2360, 2348, 2335, 2323, 2311, 2298, 2286, 2273, 2261,
    2248, 2236, 2223, 2211, 2198, 2186, 2173, 2161, 2148, 2135, 2123, 2110, 2098, 2085, 2073, 2060,
    2048, 2035, 2022, 2010, 1997, 1985, 1972, 1960, 1947, 1934, 1922, 1909, 1897, 1884, 1872, 1859,
    1847, 1834, 1822, 1809, 1797, 1784, 1772, 1760, 1747, 1735, 1722, 1710, 1697, 1685, 1673, 1660,
    1648, 1636, 1623, 1611, 1599, 1587, 1574, 1562, 1550, 1538, 1526, 1514, 1501, 1489, 1477, 1465,
    1453, 1441, 1429, 1417, 1405, 1393, 1381, 1370, 1358, 1346, 1334, 1322, 1311, 1299, 1287, 1276,
    1264, 1252, 1241, 1229, 1218, 1206, 1195, 1183, 1172, 1161, 1149, 1138, 1127, 1116, 1105, 1093,
    1082, 1071, 1060, 1049, 1038, 1027, 1017, 1006, 995, 984, 973, 963, 952, 941, 931, 920,
    910, 900, 889, 879, 869, 858, 848, 838, 828, 818, 808, 798, 788, 778, 768, 758,
    749, 739, 729, 720, 710, 701, 691, 682, 672, 663, 654, 645, 636, 627, 618, 609,
    600, 591, 582, 573, 565, 556, 547, 539, 530, 522, 514, 505, 497, 489, 481, 473,
    465, 457, 449, 441, 433, 426, 418, 410, 403, 395, 388, 381, 373, 366, 359, 352,
    345, 338, 331, 324, 318, 311, 304, 298, 291, 285, 279, 272, 266, 260, 254, 248,
    242, 236, 230, 224, 219, 213, 207, 202, 197, 191, 186, 181, 176, 171, 166, 161,
    156, 151, 146, 142, 137, 133, 128, 124, 120, 115, 111, 107, 103, 99, 96, 92,
    88, 85, 81, 78, 74, 71, 68, 64, 61, 58, 55, 53, 50, 47, 44, 42,
    39, 37, 35, 32, 30, 28, 26, 24, 22, 20, 19, 17, 15, 14, 12, 11,
    10, 9, 8, 7, 6, 5, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0,
    0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 5, 6, 7, 8, 9,
    10, 11, 12, 14, 15, 17, 19, 20, 22, 24, 26, 28, 30, 32, 35, 37,
    39, 42, 44, 47, 50, 53, 55, 58, 61, 64, 68, 71, 74, 78, 81, 85,
    88, 92, 96, 99, 103, 107, 111, 115, 120, 124, 128, 133, 137, 142, 146, 151,
    156, 161, 166, 171, 176, 181, 186, 191, 197, 202, 207, 213, 219, 224, 230, 236,
    242, 248, 254, 260, 266, 272, 279, 285, 291, 298, 304, 311, 318, 324, 331, 338,
    345, 352, 359, 366, 373, 381, 388, 395, 403, 410, 418, 426, 433, 441, 449, 457,
    465, 473, 481, 489, 497, 505, 514, 522, 530, 539, 547, 556, 565, 573, 582, 591,
    600, 609, 618, 627, 636, 645, 654, 663, 672, 682, 691, 701, 710, 720, 729, 739,
    749, 758, 768, 778, 788, 798, 808, 818, 828, 838, 848, 858, 869, 879, 889, 900,
    910, 920, 931, 941, 952, 963, 973, 984, 995, 1006, 1017, 1027, 1038, 1049, 1060, 1071,
    1082, 1093, 1105, 1116, 1127, 1138, 1149, 1161, 1172, 1183, 1195, 1206, 1218, 1229, 1241, 1252,
    1264, 1276, 1287, 1299, 1311, 1322, 1334, 1346, 1358, 1370, 1381, 1393, 1405, 1417, 1429, 1441,
    1453, 1465, 1477, 1489, 1501, 1514, 1526, 1538, 1550, 1562, 1574, 1587, 1599, 1611, 1623, 1636,
    1648, 1660, 1673, 1685, 1697, 1710, 1722, 1735, 1747, 1760, 1772, 1784, 1797, 1809, 1822, 1834,
    1847, 1859, 1872, 1884, 1897, 1909, 1922, 1934, 1947, 1960, 1972, 1985, 1997, 2010, 2022, 2035};