// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>
//         Basile Bougenot <bbougenot@student.ethz.ch>
//         Matteo Perotti <mperotti@iis.ee.ethz.ch>

#include "float_macros.h"
#include "vector_macros.h"

// Simple random test with similar values
void TEST_CASE1(void) {
  VSET(16, e16, m1);
  //              -56.5312,  95.3750,  86.3750, -33.4375,  4.7656,  58.8438,
  //              -80.4375, -96.4375,  74.3750, -92.7500, -57.2812, -90.0625,
  //              -93.2500,  40.6875, -32.2812, -36.8125
  VLOAD_16(v2, 0xd311, 0x55f6, 0x5566, 0xd02e, 0x44c4, 0x535b, 0xd507, 0xd607,
           0x54a6, 0xd5cc, 0xd329, 0xd5a1, 0xd5d4, 0x5116, 0xd009, 0xd09a);
  //               96.4375, -98.8125, -49.1250, -78.8750,
  //               -5.9180,  32.8750,  32.8750, -74.8125,
  //               -10.3750,  39.5938,  43.2812,  15.0547, -31.9062,
  //               -11.2500,  16.3594,  28.6094
  VLOAD_16(v6, 0x5607, 0xd62d, 0xd224, 0xd4ee, 0xc5eb, 0x501c, 0x501c, 0xd4ad,
           0xc930, 0x50f3, 0x5169, 0x4b87, 0xcffa, 0xc9a0, 0x4c17, 0x4f27);
  asm volatile("vfwmul.vv v4, v2, v6");
  //              -5451.73242188, -9424.24218750, -4243.17187500, 2637.38281250,
  //              -28.20281982,  1934.48828125, -2644.38281250,  7214.73046875,
  //              -771.64062500, -3672.32031250, -2479.20410156, -1355.86279297,
  //              2975.25781250, -457.73437500, -528.10107422, -1053.18261719
  VCMP_U32(1, v4, 0xc5aa5ddc, 0xc61340f8, 0xc5849960, 0x4524d620, 0xc1e19f60,
           0x44f1cfa0, 0xc5254620, 0x45e175d8, 0xc440e900, 0xc5658520,
           0xc51af344, 0xc4a97b9c, 0x4539f420, 0xc3e4de00, 0xc4040678,
           0xc483a5d8);

  VSET(16, e32, m1);
  //              -89875.40625000,  87678.49218750, -37342.58593750,
  //              -47507.81640625, -80717.72656250,  2230.02978516,
  //              -68805.99218750,  79032.60156250, -43338.95703125,
  //              42250.94531250, -6447.03955078,  25544.21679688,
  //              5945.30566406, -47409.30468750, -43415.17187500,
  //              92669.35156250
  VLOAD_32(v2, 0xc7af89b4, 0x47ab3f3f, 0xc711de96, 0xc73993d1, 0xc79da6dd,
           0x450b607a, 0xc78662ff, 0x479a5c4d, 0xc7294af5, 0x47250af2,
           0xc5c97851, 0x46c7906f, 0x45b9ca72, 0xc739314e, 0xc729972c,
           0x47b4fead);
  //               99630.39843750,  37076.73437500, -66118.01562500,
  //               -99829.85156250, -78879.75000000,  75633.85937500,
  //               -90564.15625000, -84653.48437500,  34630.80859375,
  //               85817.48437500, -23627.74023438, -79522.11718750,
  //               51590.63671875,  7574.55957031, -93117.57812500,
  //               28056.31054688
  VLOAD_32(v6, 0x47c29733, 0x4710d4bc, 0xc7812302, 0xc7c2faed, 0xc79a0fe0,
           0x4793b8ee, 0xc7b0e214, 0xc7a556be, 0x470746cf, 0x47a79cbe,
           0xc6b8977b, 0xc79b510f, 0x474986a3, 0x45ecb47a, 0xc7b5deca,
           0x46db309f);
  asm volatile("vfwmul.vv v4, v2, v6");
  //              -8954322534.4196777343750000,  3250832165.2364501953125000,
  //              2469017680.4935302734375000,  4742698259.8944396972656250,
  //              6366994091.8183593750000000,  168665759.1725692749023438,
  //              6231356627.4050292968750000, -6690385101.4866943359375000,
  //              -1500863125.6019744873046875,  3625869839.1844482421875000,
  //              152328975.7866010665893555, -2031330201.5839996337890625,
  //              306722104.6965751647949219, -359104602.5425643920898438,
  //              4042715658.8806152343750000,  2599960105.6150360107421875
  VCMP_U64(2, v4, 0xc200adc0f3335b80, 0x41e8387864a79100, 0x41e265470a0fcb00,
           0x41f1aafd513e4fa0, 0x41f7b809eabd1800, 0x41a41b453e585b00,
           0x41f736af4d367b00, 0xc1f8ec72ccd7c980, 0xc1d65d56a56686c0,
           0x41eb03cc41e5e700, 0x41a228b61f92bd60, 0xc1de44e8e6656040,
           0x41b2483538b252c0, 0xc1b567805a8ae580, 0x41ee1ede415c2e00,
           0x41e35f07c533ae60);
};

// Simple random test with similar values (masked)
// The numbers are the same of TEST_CASE1
void TEST_CASE2(void) {
  VSET(16, e16, m1);
  //              -56.5312,  95.3750,  86.3750, -33.4375,  4.7656,  58.8438,
  //              -80.4375, -96.4375,  74.3750, -92.7500, -57.2812, -90.0625,
  //              -93.2500,  40.6875, -32.2812, -36.8125
  VLOAD_16(v2, 0xd311, 0x55f6, 0x5566, 0xd02e, 0x44c4, 0x535b, 0xd507, 0xd607,
           0x54a6, 0xd5cc, 0xd329, 0xd5a1, 0xd5d4, 0x5116, 0xd009, 0xd09a);
  //               96.4375, -98.8125, -49.1250, -78.8750,
  //               -5.9180,  32.8750,  32.8750, -74.8125,
  //               -10.3750,  39.5938,  43.2812,  15.0547, -31.9062,
  //               -11.2500,  16.3594,  28.6094
  VLOAD_16(v6, 0x5607, 0xd62d, 0xd224, 0xd4ee, 0xc5eb, 0x501c, 0x501c, 0xd4ad,
           0xc930, 0x50f3, 0x5169, 0x4b87, 0xcffa, 0xc9a0, 0x4c17, 0x4f27);
  VLOAD_8(v0, 0xAA, 0xAA);
  VCLEAR(v4);
  asm volatile("vfwmul.vv v4, v2, v6, v0.t");
  //               0.00000000, -9424.24218750,  0.00000000,  2637.38281250,
  //               0.00000000,  1934.48828125,  0.00000000,  7214.73046875,
  //               0.00000000, -3672.32031250,  0.00000000, -1355.86279297,
  //               0.00000000, -457.73437500,  0.00000000, -1053.18261719
  VCMP_U32(3, v4, 0x0, 0xc61340f8, 0x0, 0x4524d620, 0x0, 0x44f1cfa0, 0x0,
           0x45e175d8, 0x0, 0xc5658520, 0x0, 0xc4a97b9c, 0x0, 0xc3e4de00, 0x0,
           0xc483a5d8);

  VSET(16, e32, m1);
  //              -89875.40625000,  87678.49218750, -37342.58593750,
  //              -47507.81640625, -80717.72656250,  2230.02978516,
  //              -68805.99218750,  79032.60156250, -43338.95703125,
  //              42250.94531250, -6447.03955078,  25544.21679688,
  //              5945.30566406, -47409.30468750, -43415.17187500,
  //              92669.35156250
  VLOAD_32(v2, 0xc7af89b4, 0x47ab3f3f, 0xc711de96, 0xc73993d1, 0xc79da6dd,
           0x450b607a, 0xc78662ff, 0x479a5c4d, 0xc7294af5, 0x47250af2,
           0xc5c97851, 0x46c7906f, 0x45b9ca72, 0xc739314e, 0xc729972c,
           0x47b4fead);
  //               99630.39843750,  37076.73437500, -66118.01562500,
  //               -99829.85156250, -78879.75000000,  75633.85937500,
  //               -90564.15625000, -84653.48437500,  34630.80859375,
  //               85817.48437500, -23627.74023438, -79522.11718750,
  //               51590.63671875,  7574.55957031, -93117.57812500,
  //               28056.31054688
  VLOAD_32(v6, 0x47c29733, 0x4710d4bc, 0xc7812302, 0xc7c2faed, 0xc79a0fe0,
           0x4793b8ee, 0xc7b0e214, 0xc7a556be, 0x470746cf, 0x47a79cbe,
           0xc6b8977b, 0xc79b510f, 0x474986a3, 0x45ecb47a, 0xc7b5deca,
           0x46db309f);
  VLOAD_8(v0, 0xAA, 0xAA);
  VCLEAR(v4);
  asm volatile("vfwmul.vv v4, v2, v6, v0.t");
  //               0.0000000000000000,  3250832165.2364501953125000,
  //               0.0000000000000000,  4742698259.8944396972656250,
  //               0.0000000000000000,  168665759.1725692749023438,
  //               0.0000000000000000, -6690385101.4866943359375000,
  //               0.0000000000000000,  3625869839.1844482421875000,
  //               0.0000000000000000, -2031330201.5839996337890625,
  //               0.0000000000000000, -359104602.5425643920898438,
  //               0.0000000000000000,  2599960105.6150360107421875
  VCMP_U64(4, v4, 0x0, 0x41e8387864a79100, 0x0, 0x41f1aafd513e4fa0, 0x0,
           0x41a41b453e585b00, 0x0, 0xc1f8ec72ccd7c980, 0x0, 0x41eb03cc41e5e700,
           0x0, 0xc1de44e8e6656040, 0x0, 0xc1b567805a8ae580, 0x0,
           0x41e35f07c533ae60);
};

// Simple random test with similar values (vector-scalar)
void TEST_CASE3(void) {
  VSET(16, e16, m1);
  double dscalar_16;
  //              -44.4062, -27.0781, -21.6562,  75.5625, -84.5000,
  //              -1.0713,  72.5625, -84.6250,  83.9375, -52.3438,
  //              -40.5625,  1.6523,  79.6875, -36.2812,  33.5938, -72.4375
  VLOAD_16(v2, 0xd18d, 0xcec5, 0xcd6a, 0x54b9, 0xd548, 0xbc49, 0x5489, 0xd54a,
           0x553f, 0xd28b, 0xd112, 0x3e9c, 0x54fb, 0xd089, 0x5033, 0xd487);
  //                             -58.9688
  BOX_HALF_IN_DOUBLE(dscalar_16, 0xd35f);
  asm volatile("vfwmul.vf v4, v2, %[A]" ::[A] "f"(dscalar_16));
  //               2618.58105469,  1596.76318359,  1277.04199219,
  //               -4455.82617188,  4982.85937500,  63.17257690, -4278.91992188,
  //               4990.23046875, -4949.68945312,  3086.64550781, 2391.91992188,
  //               -97.43664551, -4699.07226562,  2139.45996094, -1980.98144531,
  //               4271.54882812
  VCMP_U32(5, v4, 0x4523a94c, 0x44c7986c, 0x449fa158, 0xc58b3e9c, 0x459bb6e0,
           0x427cb0b8, 0xc585b75c, 0x459bf1d8, 0xc59aad84, 0x4540ea54,
           0x45157eb8, 0xc2c2df90, 0xc592d894, 0x4505b75c, 0xc4f79f68,
           0x45857c64);

  VSET(16, e32, m1);
  double dscalar_32;
  //              -187018.20312500, -714032.18750000, -891429.25000000,
  //              -378265.00000000,  211566.90625000,  231934.78125000,
  //              947047.75000000, -241945.03125000, -489658.75000000,
  //              -788001.68750000, -817411.37500000, -522168.21875000,
  //              -668021.56250000,  744069.12500000, -620354.68750000,
  //              913454.68750000
  VLOAD_32(v2, 0xc836a28d, 0xc92e5303, 0xc959a254, 0xc8b8b320, 0x484e9bba,
           0x48627fb2, 0x4967367c, 0xc86c4642, 0xc8ef1758, 0xc940621b,
           0xc9479036, 0xc8fef707, 0xc9231759, 0x4935a852, 0xc917742b,
           0x495f02eb);
  //                              -50557.21484375
  BOX_FLOAT_IN_DOUBLE(dscalar_32, 0xc7457d37);
  asm volatile("vfwmul.vf v4, v2, %[A]" ::[A] "f"(dscalar_32));
  //               9455119475.0827026367187500,  36099478708.7902832031250000,
  //               45068180110.2529296875000000,  19124024872.8710937500000000,
  //               -10696233533.1087646484375000, -11725976565.3944091796875000,
  //               -47880096564.0400390625000000,  12232066925.2840576171875000,
  //               24755782623.8720703125000000,  39839170612.1750488281250000,
  //               41326042501.6000976562500000,  26399370819.9219970703125000,
  //               33773309655.5700683593750000, -37618062611.2260742187500000,
  //               31363405215.2648925781250000, -46181724885.9680175781250000
  VCMP_U64(6, v4, 0x42019c8d6398a960, 0x4220cf64a96994a0, 0x4224fc8bb51c8180,
           0x4211cf85d8a37c00, 0xc203ec5c91e8dec0, 0xc205d7619fab27c0,
           0xc2264bc096681480, 0x4206c8b43b6a45c0, 0x42170e3d0f7f7d00,
           0x42228d33006859a0, 0x42233e72bb0b3340, 0x42189619b90fb020,
           0x421f742f3b5e47c0, 0xc221846c2a2673c0, 0x421d359e567d0f40,
           0xc225814a65abefa0);
};

// Simple random test with similar values (vector-scalar) (masked)
void TEST_CASE4(void) {
  VSET(16, e16, m1);
  double dscalar_16;
  //               -44.4062, -27.0781, -21.6562,  75.5625, -84.5000,
  //               -1.0713,  72.5625, -84.6250,  83.9375, -52.3438,
  //               -40.5625,  1.6523,  79.6875, -36.2812,  33.5938, -72.4375
  VLOAD_16(v2, 0xd18d, 0xcec5, 0xcd6a, 0x54b9, 0xd548, 0xbc49, 0x5489, 0xd54a,
           0x553f, 0xd28b, 0xd112, 0x3e9c, 0x54fb, 0xd089, 0x5033, 0xd487);
  //                             -58.9688
  BOX_HALF_IN_DOUBLE(dscalar_16, 0xd35f);
  VLOAD_8(v0, 0xAA, 0xAA);
  VCLEAR(v4);
  asm volatile("vfwmul.vf v4, v2, %[A], v0.t" ::[A] "f"(dscalar_16));
  //                0.00000000,  1596.76318359,  0.00000000, -4455.82617188,
  //                0.00000000,  63.17257690,  0.00000000,  4990.23046875,
  //                0.00000000,  3086.64550781,  0.00000000, -97.43664551,
  //                0.00000000,  2139.45996094,  0.00000000,  4271.54882812
  VCMP_U32(7, v4, 0x0, 0x44c7986c, 0x0, 0xc58b3e9c, 0x0, 0x427cb0b8, 0x0,
           0x459bf1d8, 0x0, 0x4540ea54, 0x0, 0xc2c2df90, 0x0, 0x4505b75c, 0x0,
           0x45857c64);

  VSET(16, e32, m1);
  double dscalar_32;
  //               -187018.20312500, -714032.18750000, -891429.25000000,
  //               -378265.00000000,  211566.90625000,  231934.78125000,
  //               947047.75000000, -241945.03125000, -489658.75000000,
  //               -788001.68750000, -817411.37500000, -522168.21875000,
  //               -668021.56250000,  744069.12500000, -620354.68750000,
  //               913454.68750000
  VLOAD_32(v2, 0xc836a28d, 0xc92e5303, 0xc959a254, 0xc8b8b320, 0x484e9bba,
           0x48627fb2, 0x4967367c, 0xc86c4642, 0xc8ef1758, 0xc940621b,
           0xc9479036, 0xc8fef707, 0xc9231759, 0x4935a852, 0xc917742b,
           0x495f02eb);
  //                              -50557.21484375
  BOX_FLOAT_IN_DOUBLE(dscalar_32, 0xc7457d37);
  VLOAD_8(v0, 0xAA, 0xAA);
  VCLEAR(v4);
  asm volatile("vfwmul.vf v4, v2, %[A], v0.t" ::[A] "f"(dscalar_32));
  //                0.0000000000000000,  36099478708.7902832031250000,
  //                0.0000000000000000,  19124024872.8710937500000000,
  //                0.0000000000000000, -11725976565.3944091796875000,
  //                0.0000000000000000,  12232066925.2840576171875000,
  //                0.0000000000000000,  39839170612.1750488281250000,
  //                0.0000000000000000,  26399370819.9219970703125000,
  //                0.0000000000000000, -37618062611.2260742187500000,
  //                0.0000000000000000, -46181724885.9680175781250000
  VCMP_U64(8, v4, 0x0, 0x4220cf64a96994a0, 0x0, 0x4211cf85d8a37c00, 0x0,
           0xc205d7619fab27c0, 0x0, 0x4206c8b43b6a45c0, 0x0, 0x42228d33006859a0,
           0x0, 0x42189619b90fb020, 0x0, 0xc221846c2a2673c0, 0x0,
           0xc225814a65abefa0);
};

int main(void) {
  enable_vec();
  enable_fp();

  TEST_CASE1();
  TEST_CASE2();
  TEST_CASE3();
  TEST_CASE4();

  EXIT_CHECK();
}
