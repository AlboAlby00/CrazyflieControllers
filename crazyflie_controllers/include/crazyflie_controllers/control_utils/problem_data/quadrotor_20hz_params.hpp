#pragma once

#include <tinympc/types.hpp>

tinytype rho_value = 5.0;

tinytype Adyn_data[NSTATES * NSTATES] = {
    1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0245250, 0.0000000, 0.0500000, 0.0000000, 0.0000000, 0.0000000, 0.0002044, 0.0000000,
    0.0000000, 1.0000000, 0.0000000, -0.0245250, 0.0000000, 0.0000000, 0.0000000, 0.0500000, 0.0000000, -0.0002044, 0.0000000, 0.0000000,
    0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0500000, 0.0000000, 0.0000000, 0.0000000,
    0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0250000, 0.0000000, 0.0000000,
    0.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0250000, 0.0000000,
    0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0250000,
    0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.9810000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0122625, 0.0000000,
    0.0000000, 0.0000000, 0.0000000, -0.9810000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, -0.0122625, 0.0000000, 0.0000000,
    0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000,
    0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000,
    0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000,
    0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000};

tinytype Bdyn_data[NSTATES * NINPUTS] = {
    -0.0007069, 0.0007773, 0.0007091, -0.0007795,
    0.0007034, 0.0007747, -0.0007042, -0.0007739,
    0.0052554, 0.0052554, 0.0052554, 0.0052554,
    -0.1720966, -0.1895213, 0.1722891, 0.1893288,
    -0.1729419, 0.1901740, 0.1734809, -0.1907131,
    0.0123423, -0.0045148, -0.0174024, 0.0095748,
    -0.0565520, 0.0621869, 0.0567283, -0.0623632,
    0.0562756, 0.0619735, -0.0563386, -0.0619105,
    0.2102143, 0.2102143, 0.2102143, 0.2102143,
    -13.7677303, -15.1617018, 13.7831318, 15.1463003,
    -13.8353509, 15.2139209, 13.8784751, -15.2570451,
    0.9873856, -0.3611820, -1.3921880, 0.7659845};

tinytype Kinf_data[NINPUTS * NSTATES] = {
    -0.1131651, 0.0804349, 1.2891591, -0.3933755, -0.5924539, -2.5576673, -0.0885719, 0.0612491, 0.5438439, -0.0355682, -0.0570154, -0.5522377,
    0.1079494, 0.0311034, 1.2891591, -0.1027003, 0.5690238, 2.5574092, 0.0847269, 0.0206449, 0.5438439, -0.0049179, 0.0550856, 0.5519773,
    0.0028796, -0.0363306, 1.2891591, 0.1261700, -0.0756569, -2.5561006, -0.0033242, -0.0244975, 0.5438439, 0.0068503, -0.0147863, -0.5512313,
    0.0023362, -0.0752077, 1.2891591, 0.3699059, 0.0990870, 2.5563587, 0.0071693, -0.0573965, 0.5438439, 0.0336359, 0.0167160, 0.5514916};

tinytype Pinf_data[NSTATES * NSTATES] = {
    1542.3620564, -0.2580837, -0.0000000, 1.1105663, 1299.4507594, 17.2503454, 437.8595030, -0.1904147, -0.0000000, 0.0771823, 14.7132685, 2.6899147,
    -0.2580837, 1541.8513994, -0.0000000, -1297.2441085, -1.1105778, -6.8996289, -0.1904156, 437.4818451, 0.0000000, -14.5599331, -0.0771835, -1.0758960,
    -0.0000000, -0.0000000, 885.9046714, 0.0000000, -0.0000000, -0.0000000, -0.0000000, -0.0000000, 74.7159753, -0.0000000, -0.0000000, -0.0000000,
    1.1105663, -1297.2441085, 0.0000000, 4887.6852669, 5.6086128, 37.3859869, 0.8718046, -888.4130773, -0.0000000, 57.7207548, 0.4493830, 6.4604041,
    1299.4507594, -1.1105778, -0.0000000, 5.6086128, 4898.9334870, 93.4688410, 890.1530523, -0.8718090, -0.0000000, 0.4493817, 58.6258449, 16.1514679,
    17.2503454, -6.8996289, -0.0000000, 37.3859869, 93.4688410, 3494.0354062, 13.8693079, -5.5473978, -0.0000000, 3.2600215, 8.1502590, 153.1645212,
    437.8595030, -0.1904156, -0.0000000, 0.8718046, 890.1530523, 13.8693079, 248.9416157, -0.1440668, -0.0000000, 0.0633741, 10.3204552, 2.2347043,
    -0.1904147, 437.4818451, -0.0000000, -888.4130773, -0.8718090, -5.5473978, -0.1440668, 248.6549585, 0.0000000, -10.1937790, -0.0633746, -0.8938386,
    0.0000000, -0.0000000, 74.7159753, 0.0000000, -0.0000000, -0.0000000, -0.0000000, -0.0000000, 34.1517347, -0.0000000, -0.0000000, -0.0000000,
    0.0771823, -14.5599331, -0.0000000, 57.7207548, 0.4493817, 3.2600215, 0.0633741, -10.1937790, -0.0000000, 7.7498781, 0.0461173, 0.6846635,
    14.7132685, -0.0771835, -0.0000000, 0.4493830, 58.6258449, 8.1502590, 10.3204552, -0.0633746, -0.0000000, 0.0461173, 7.8436250, 1.7116687,
    2.6899147, -1.0758960, -0.0000000, 6.4604041, 16.1514679, 153.1645212, 2.2347043, -0.8938386, -0.0000000, 0.6846635, 1.7116687, 35.6329664};

tinytype Quu_inv_data[NINPUTS * NINPUTS] = {
    0.0321638, -0.0004152, 0.0319393, -0.0003762,
    -0.0004152, 0.0320971, -0.0003206, 0.0319504,
    0.0319393, -0.0003206, 0.0319730, -0.0002801,
    -0.0003762, 0.0319504, -0.0002801, 0.0320176};

tinytype AmBKt_data[NSTATES * NSTATES] = {
    0.9998359, -0.0000002, -0.0000000, 0.0000449, -0.0401542, 0.0019118, -0.0131304, -0.0000147, -0.0000000, 0.0035944, -3.2123337, 0.1529465,
    -0.0000002, 0.9998355, -0.0000000, 0.0402357, -0.0000449, -0.0007645, -0.0000147, -0.0131571, -0.0000000, 3.2188558, -0.0035941, -0.0611574,
    0.0000000, -0.0000000, 0.9729000, 0.0000000, 0.0000000, -0.0000000, 0.0000000, -0.0000000, -1.0839986, 0.0000000, 0.0000000, -0.0000000,
    0.0000006, -0.0237936, 0.0000000, 0.8210660, 0.0001576, 0.0030454, 0.0000515, -0.9224886, 0.0000000, -14.3147219, 0.0126089, 0.2436300,
    0.0237948, -0.0000006, -0.0000000, 0.0001576, 0.8213486, 0.0076159, 0.9225810, -0.0000515, -0.0000000, 0.0126107, -14.2921134, 0.6092739,
    0.0000093, -0.0000037, -0.0000000, 0.0009137, 0.0022852, 0.9741549, 0.0007472, -0.0002988, -0.0000000, 0.0730969, 0.1828126, -2.0676097,
    0.0498795, -0.0000001, -0.0000000, 0.0000300, -0.0294867, 0.0013492, 0.9903579, -0.0000098, -0.0000000, 0.0024000, -2.3589348, 0.1079370,
    -0.0000001, 0.0498793, -0.0000000, 0.0295409, -0.0000300, -0.0005395, -0.0000098, 0.9903401, -0.0000000, 2.3632693, -0.0023998, -0.0431602,
    -0.0000000, -0.0000000, 0.0385676, -0.0000000, -0.0000000, -0.0000000, -0.0000000, 0.0000000, 0.5427050, -0.0000000, 0.0000000, -0.0000000,
    0.0000000, -0.0001447, -0.0000000, 0.0103983, 0.0000104, 0.0002139, 0.0000034, -0.0074877, -0.0000000, -0.1681348, 0.0008353, 0.0171156,
    0.0001448, -0.0000000, -0.0000000, 0.0000104, 0.0104169, 0.0005350, 0.0074938, -0.0000034, -0.0000000, 0.0008354, -0.1666483, 0.0428026,
    0.0000013, -0.0000005, -0.0000000, 0.0001312, 0.0003280, 0.0194348, 0.0001073, -0.0000429, -0.0000000, 0.0104925, 0.0262414, 0.5547841};

tinytype coeff_d2p_data[NSTATES * NINPUTS] = {
    0.0000000, -0.0000000, -0.0000000, 0.0000000,
    -0.0000000, -0.0000000, 0.0000000, 0.0000000,
    0.0000000, 0.0000000, -0.0000000, 0.0000000,
    0.0000000, 0.0000000, -0.0000000, -0.0000000,
    0.0000000, -0.0000000, -0.0000000, 0.0000000,
    -0.0000000, -0.0000000, 0.0000000, 0.0000000,
    -0.0000000, 0.0000000, 0.0000000, 0.0000000,
    -0.0000000, -0.0000000, 0.0000000, 0.0000000,
    -0.0000000, 0.0000000, -0.0000000, 0.0000000,
    -0.0000000, 0.0000000, 0.0000000, -0.0000000,
    -0.0000000, -0.0000000, 0.0000000, 0.0000000,
    0.0000000, 0.0000000, -0.0000000, 0.0000000};

tinytype Q_data[NSTATES] = {100.0000000, 100.0000000, 100.0000000, 4.0000000, 4.0000000, 400.0000000, 4.0000000, 4.0000000, 4.0000000, 2.0408163, 2.0408163, 4.0000000};

tinytype R_data[NINPUTS] = {4.0000000, 4.0000000, 4.0000000, 4.0000000};