/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Laurens Mackay <mackayl@student.ethz.ch>
 *   		 Dominik Honegger <dominik.honegger@inf.ethz.ch>
 *   		 Petri Tanskanen <tpetri@inf.ethz.ch>
 *   		 Samuel Zihlmann <samuezih@ee.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ov2640.h"

I2C_HandleTypeDef I2C2_Handle;

const uint8_t ov2640_sxga_init_reg_tbl[][2]=
{
	0xff, 0x00,
	0x2c, 0xff,
	0x2e, 0xdf,
	0xff, 0x01,
	0x3c, 0x32,
	//
	0x11, 0x00,
	0x09, 0x02,
	0x04, 0xD8,//Ë®Æ½¾µÏñ,´¹Ö±·­×ª
	0x13, 0xe5,
	0x14, 0x48,
	0x2c, 0x0c,
	0x33, 0x78,
	0x3a, 0x33,
	0x3b, 0xfB,
	//
	0x3e, 0x00,
	0x43, 0x11,
	0x16, 0x10,
	//
	0x39, 0x92,
	//
	0x35, 0xda,
	0x22, 0x1a,
	0x37, 0xc3,
	0x23, 0x00,
	0x34, 0xc0,
	0x36, 0x1a,
	0x06, 0x88,
	0x07, 0xc0,
	0x0d, 0x87,
	0x0e, 0x41,
	0x4c, 0x00,

	0x48, 0x00,
	0x5B, 0x00,
	0x42, 0x03,
	//
	0x4a, 0x81,
	0x21, 0x99,
	//
	0x24, 0x40,
	0x25, 0x38,
	0x26, 0x82,
	0x5c, 0x00,
	0x63, 0x00,
	0x46, 0x00,
	0x0c, 0x3c,
	//
	0x61, 0x70,
	0x62, 0x80,
	0x7c, 0x05,
	//
	0x20, 0x80,
	0x28, 0x30,
	0x6c, 0x00,
	0x6d, 0x80,
	0x6e, 0x00,
	0x70, 0x02,
	0x71, 0x94,
	0x73, 0xc1,
	0x3d, 0x34,
	0x5a, 0x57,
	//
	0x12, 0x00,//UXGA 1600*1200

	0x17, 0x11,
	0x18, 0x75,
	0x19, 0x01,
	0x1a, 0x97,
	0x32, 0x36,
	0x03, 0x0f,
	0x37, 0x40,
	//
	0x4f, 0xca,
	0x50, 0xa8,
	0x5a, 0x23,
	0x6d, 0x00,
	0x6d, 0x38,
	//
	0xff, 0x00,
	0xe5, 0x7f,
	0xf9, 0xc0,
	0x41, 0x24,
	0xe0, 0x14,
	0x76, 0xff,
	0x33, 0xa0,
	0x42, 0x20,
	0x43, 0x18,
	0x4c, 0x00,
	0x87, 0xd5,
	0x88, 0x3f,
	0xd7, 0x03,
	0xd9, 0x10,
	0xd3, 0x82,
	//
	0xc8, 0x08,
	0xc9, 0x80,
	//
	0x7c, 0x00,
	0x7d, 0x00,
	0x7c, 0x03,
	0x7d, 0x48,
	0x7d, 0x48,
	0x7c, 0x08,
	0x7d, 0x20,
	0x7d, 0x10,
	0x7d, 0x0e,
	//
	0x90, 0x00,
	0x91, 0x0e,
	0x91, 0x1a,
	0x91, 0x31,
	0x91, 0x5a,
	0x91, 0x69,
	0x91, 0x75,
	0x91, 0x7e,
	0x91, 0x88,
	0x91, 0x8f,
	0x91, 0x96,
	0x91, 0xa3,
	0x91, 0xaf,
	0x91, 0xc4,
	0x91, 0xd7,
	0x91, 0xe8,
	0x91, 0x20,
	//
	0x92, 0x00,
	0x93, 0x06,
	0x93, 0xe3,
	0x93, 0x05,
	0x93, 0x05,
	0x93, 0x00,
	0x93, 0x04,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	//
	0x96, 0x00,
	0x97, 0x08,
	0x97, 0x19,
	0x97, 0x02,
	0x97, 0x0c,
	0x97, 0x24,
	0x97, 0x30,
	0x97, 0x28,
	0x97, 0x26,
	0x97, 0x02,
	0x97, 0x98,
	0x97, 0x80,
	0x97, 0x00,
	0x97, 0x00,
	//
	0xc3, 0xef,

	0xa4, 0x00,
	0xa8, 0x00,
	0xc5, 0x11,
	0xc6, 0x51,
	0xbf, 0x80,
	0xc7, 0x10,
	0xb6, 0x66,
	0xb8, 0xA5,
	0xb7, 0x64,
	0xb9, 0x7C,
	0xb3, 0xaf,
	0xb4, 0x97,
	0xb5, 0xFF,
	0xb0, 0xC5,
	0xb1, 0x94,
	0xb2, 0x0f,
	0xc4, 0x5c,
	//
	0xc0, 0xc8,
	0xc1, 0x96,
	0x8c, 0x00,
	0x86, 0x3d,
	0x50, 0x00,
	0x51, 0x90,
	0x52, 0x2c,
	0x53, 0x00,
	0x54, 0x00,
	0x55, 0x88,

	0x5a, 0x90,
	0x5b, 0x2C,
	0x5c, 0x05,

	0xd3, 0x02,//autoÉèÖÃÒªÐ¡ÐÄ
	//
	0xc3, 0xed,
	0x7f, 0x00,

	0xda, 0x09,

	0xe5, 0x1f,
	0xe1, 0x67,
	0xe0, 0x00,
	0xdd, 0x7f,
	0x05, 0x00,
};

/* QQVGA 160x120 */
const uint8_t OV2640_QQVGA[][2]=
{
  0xff, 0x00,
  0x2c, 0xff,
  0x2e, 0xdf,
  0xff, 0x01,

  0xc2, 0x01,

  0x3c, 0x32,
  0x11, 0x00,
  0x09, 0x02,
  0x03, 0xcf,
  0x04, 0x08,
  0x13, 0xe5,
  0x14, 0x48,
  0x2c, 0x0c,
  0x33, 0x78,
  0x3a, 0x33,
  0x3b, 0xfb,
  0x3e, 0x00,
  0x43, 0x11,
  0x16, 0x10,
  0x39, 0x02,
  0x35, 0x88,
  0x22, 0x0a,
  0x37, 0x40,
  0x23, 0x00,
  0x34, 0xa0,
  0x36, 0x1a,
  0x06, 0x02,
  0x07, 0xc0,
  0x0d, 0xb7,
  0x0e, 0x01,
  0x4c, 0x00,
  0x4a, 0x81,
  0x21, 0x99,
  0x24, 0x3a,
  0x25, 0x32,
  0x26, 0x82,
  0x5c, 0x00,
  0x63, 0x00,
  0x5d, 0x55,
  0x5e, 0x7d,
  0x5f, 0x7d,
  0x60, 0x55,
  0x61, 0x70,
  0x62, 0x80,
  0x7c, 0x05,
  0x20, 0x80,
  0x28, 0x30,
  0x6c, 0x00,
  0x6d, 0x80,
  0x6e, 0x00,
  0x70, 0x02,
  0x71, 0x96,
  0x73, 0xe1,
  0x3d, 0x34,
  0x5a, 0x57,
  0x4f, 0xbb,
  0x50, 0x9c,
  0x0f, 0x43,
  0xff, 0x00,
  0xe5, 0x7f,
  0xf9, 0xc0,
  0x41, 0x24,
  0xe0, 0x14,
  0x76, 0xff,
  0x33, 0xa0,
  0x42, 0x20,
  0x43, 0x18,
  0x4c, 0x00,
  0x87, 0xd0,
  0x88, 0x3f,
  0xd7, 0x03,
  0xd9, 0x10,
  0xd3, 0x82,
  0xc8, 0x08,
  0xc9, 0x80,
  0x7c, 0x00,
  0x7d, 0x02,
  0x7c, 0x03,
  0x7d, 0x48,
  0x7d, 0x48,
  0x7c, 0x08,
  0x7d, 0x20,
  0x7d, 0x10,
  0x7d, 0x0e,
  0x90, 0x00,
  0x91, 0x0e,
  0x91, 0x1a,
  0x91, 0x31,
  0x91, 0x5a,
  0x91, 0x69,
  0x91, 0x75,
  0x91, 0x7e,
  0x91, 0x88,
  0x91, 0x8f,
  0x91, 0x96,
  0x91, 0xa3,
  0x91, 0xaf,
  0x91, 0xc4,
  0x91, 0xd7,
  0x91, 0xe8,
  0x91, 0x20,
  0x92, 0x00,
  0x93, 0x06,
  0x93, 0xe3,
  0x93, 0x05,
  0x93, 0x05,
  0x93, 0x00,
  0x93, 0x00,
  0x93, 0x00,
  0x93, 0x00,
  0x93, 0x00,
  0x93, 0x00,
  0x93, 0x00,
  0x93, 0x00,
  0x93, 0x00,
  0x96, 0x00,
  0x97, 0x08,
  0x97, 0x19,
  0x97, 0x02,
  0x97, 0x0c,
  0x97, 0x24,
  0x97, 0x30,
  0x97, 0x28,
  0x97, 0x26,
  0x97, 0x02,
  0x97, 0x98,
  0x97, 0x80,
  0x97, 0x00,
  0x97, 0x00,
  0xc3, 0xed,
  0xa4, 0x00,
  0xa8, 0x00,
  0xbf, 0x00,
  0xba, 0xf0,
  0xbc, 0x64,
  0xbb, 0x02,
  0xb6, 0x3d,
  0xb8, 0x57,
  0xb7, 0x38,
  0xb9, 0x4e,
  0xb3, 0xe8,
  0xb4, 0xe1,
  0xb5, 0x66,
  0xb0, 0x67,
  0xb1, 0x5e,
  0xb2, 0x04,
  0xc7, 0x00,
  0xc6, 0x51,
  0xc5, 0x11,
  0xc4, 0x9c,
  0xcf, 0x02,
  0xa6, 0x00,
  0xa7, 0xe0,
  0xa7, 0x10,
  0xa7, 0x1e,
  0xa7, 0x21,
  0xa7, 0x00,
  0xa7, 0x28,
  0xa7, 0xd0,
  0xa7, 0x10,
  0xa7, 0x16,
  0xa7, 0x21,
  0xa7, 0x00,
  0xa7, 0x28,
  0xa7, 0xd0,
  0xa7, 0x10,
  0xa7, 0x17,
  0xa7, 0x21,
  0xa7, 0x00,
  0xa7, 0x28,
  0xc0, 0xc8,
  0xc1, 0x96,
  0x86, 0x1d,
  0x50, 0x00,
  0x51, 0x90,
  0x52, 0x18,
  0x53, 0x00,
  0x54, 0x00,
  0x55, 0x88,
  0x57, 0x00,
  0x5a, 0x90,
  0x5b, 0x18,
  0x5c, 0x05,
  0xc3, 0xef,
  0x7f, 0x00,
  0xda, 0x00,
  0xe5, 0x1f,
  0xe1, 0x67,
  0xe0, 0x00,
  0xdd, 0xff,
  0x05, 0x00,
  0xff, 0x01,
  0xff, 0x01,
  0x12, 0x00,
  0x17, 0x11,
  0x18, 0x75,
  0x19, 0x01,
  0x1a, 0x97,
  0x32, 0x36,
  0x4f, 0xbb,
  0x6d, 0x80,
  0x3d, 0x34,
  0x39, 0x02,
  0x35, 0x88,
  0x22, 0x0a,
  0x37, 0x40,
  0x23, 0x00,
  0x34, 0xa0,
  0x36, 0x1a,
  0x06, 0x02,
  0x07, 0xc0,
  0x0d, 0xb7,
  0x0e, 0x01,
  0x4c, 0x00,
  0xff, 0x00,
  0xe0, 0x04,
  0x8c, 0x00,
  0x87, 0xd0,
  0xe0, 0x00,
  0xff, 0x00,
  0xe0, 0x14,
  0xe1, 0x77,
  0xe5, 0x1f,
  0xd7, 0x03,
  0xda, 0x10,
  0xe0, 0x00,
  0xff, 0x00,
  0xe0, 0x04,
  0xc0, 0xc8,
  0xc1, 0x96,
  0x86, 0x1d,
  0x50, 0x00,
  0x51, 0x90,
  0x52, 0x2c,
  0x53, 0x00,
  0x54, 0x00,
  0x55, 0x88,
  0x57, 0x00,
  0x5a, 0x90,
  0x5b, 0x2c,
  0x5c, 0x05,
  0xe0, 0x00,
  0xd3, 0x04,
  0xff, 0x00,
  0xc3, 0xef,
  0xa6, 0x00,
  0xa7, 0xdd,
  0xa7, 0x78,
  0xa7, 0x7e,
  0xa7, 0x24,
  0xa7, 0x00,
  0xa7, 0x25,
  0xa6, 0x06,
  0xa7, 0x20,
  0xa7, 0x58,
  0xa7, 0x73,
  0xa7, 0x34,
  0xa7, 0x00,
  0xa7, 0x25,
  0xa6, 0x0c,
  0xa7, 0x28,
  0xa7, 0x58,
  0xa7, 0x6d,
  0xa7, 0x34,
  0xa7, 0x00,
  0xa7, 0x25,
  0xff, 0x00,
  0xe0, 0x04,
  0xe1, 0x67,
  0xe5, 0x1f,
  0xd7, 0x01,
  0xda, 0x08,
  0xda, 0x04,//0x09
  0xe0, 0x00,
  0x98, 0x00,
  0x99, 0x00,
  0xff, 0x01,
  0x04, 0x28,
  0xff, 0x01,
#if CLR_BAR_EN	> 0
  0x12, 0x42,	//0x42虏脢脤玫虏芒脢脭脢鹿脛脺
#else
  0x12, 0x40,
#endif
  0x17, 0x11,
  0x18, 0x43,
  0x19, 0x00,
  0x1a, 0x4b,
  0x32, 0x09,
  0x4f, 0xca,
  0x50, 0xa8,
  0x5a, 0x23,
  0x6d, 0x00,
  0x39, 0x12,
  0x35, 0xda,
  0x22, 0x1a,
  0x37, 0xc3,
  0x23, 0x00,
  0x34, 0xc0,
  0x36, 0x1a,
  0x06, 0x88,
  0x07, 0xc0,
  0x0d, 0x87,
  0x0e, 0x41,
  0x4c, 0x00,
  0xff, 0x00,
  0xe0, 0x04,
  0xc0, 0x64,
  0xc1, 0x4b,
  0x86, 0x35,
  0x50, 0x92,
  0x51, 0xc8,
  0x52, 0x96,
  0x53, 0x00,
  0x54, 0x00,
  0x55, 0x00,
  0x57, 0x00,
  0x5a, 0x28,
  0x5b, 0x1e,
  0x5c, 0x00,
  0xe0, 0x00,
  0xff, 0x01,
  0x11, 0x00,//脢卤脰脫路脰脝碌
  0x3d, 0x38,
  0x2d, 0x00,
  0x50, 0x65,
  0xff, 0x00,
  0xd3, 0x04,
  0x7c, 0x00,
  0x7d, 0x04,
  0x7c, 0x09,
  0x7d, 0x28,
  0x7d, 0x00,
};

/* QVGA 320x240 */
const unsigned char OV2640_QVGA[][2]=
{
  0xff, 0x00,
  0x2c, 0xff,
  0x2e, 0xdf,
  0xff, 0x01,
  0x3c, 0x32,
  0x11, 0x00,
  0x09, 0x02,
  0x04, 0x28,
  0x13, 0xe5,
  0x14, 0x48,
  0x2c, 0x0c,
  0x33, 0x78,
  0x3a, 0x33,
  0x3b, 0xfB,
  0x3e, 0x00,
  0x43, 0x11,
  0x16, 0x10,
  0x4a, 0x81,
  0x21, 0x99,
  0x24, 0x40,
  0x25, 0x38,
  0x26, 0x82,
  0x5c, 0x00,
  0x63, 0x00,
  0x46, 0x3f,
  0x0c, 0x3c,
  0x61, 0x70,
  0x62, 0x80,
  0x7c, 0x05,
  0x20, 0x80,
  0x28, 0x30,
  0x6c, 0x00,
  0x6d, 0x80,
  0x6e, 0x00,
  0x70, 0x02,
  0x71, 0x94,
  0x73, 0xc1,
  0x3d, 0x34,
  0x5a, 0x57,
#if CLR_BAR_EN	> 0
  0x12, 0x02,	//bit1虏脢脤玫虏芒脢脭脢鹿脛脺
#else
  0x12, 0x00,
#endif
  0x11, 0x00,
  0x17, 0x11,
  0x18, 0x75,
  0x19, 0x01,
  0x1a, 0x97,
  0x32, 0x36,
  0x03, 0x0f,
  0x37, 0x40,
  0x4f, 0xbb,
  0x50, 0x9c,
  0x5a, 0x57,
  0x6d, 0x80,
  0x6d, 0x38,
  0x39, 0x02,
  0x35, 0x88,
  0x22, 0x0a,
  0x37, 0x40,
  0x23, 0x00,
  0x34, 0xa0,
  0x36, 0x1a,
  0x06, 0x02,
  0x07, 0xc0,
  0x0d, 0xb7,
  0x0e, 0x01,
  0x4c, 0x00,
  0xff, 0x00,
  0xe5, 0x7f,
  0xf9, 0xc0,
  0x41, 0x24,
  0xe0, 0x14,
  0x76, 0xff,
  0x33, 0xa0,
  0x42, 0x20,
  0x43, 0x18,
  0x4c, 0x00,
  0x87, 0xd0,
  0x88, 0x3f,
  0xd7, 0x03,
  0xd9, 0x10,
  0xd3, 0x82,
  0xc8, 0x08,
  0xc9, 0x80,
  0x7d, 0x00,
  0x7c, 0x03,
  0x7d, 0x48,
  0x7c, 0x08,
  0x7d, 0x20,
  0x7d, 0x10,
  0x7d, 0x0e,
  0x90, 0x00,
  0x91, 0x0e,
  0x91, 0x1a,
  0x91, 0x31,
  0x91, 0x5a,
  0x91, 0x69,
  0x91, 0x75,
  0x91, 0x7e,
  0x91, 0x88,
  0x91, 0x8f,
  0x91, 0x96,
  0x91, 0xa3,
  0x91, 0xaf,
  0x91, 0xc4,
  0x91, 0xd7,
  0x91, 0xe8,
  0x91, 0x20,
  0x92, 0x00,
  0x93, 0x06,
  0x93, 0xe3,
  0x93, 0x02,
  0x93, 0x02,
  0x93, 0x00,
  0x93, 0x04,
  0x93, 0x00,
  0x93, 0x03,
  0x93, 0x00,
  0x93, 0x00,
  0x93, 0x00,
  0x93, 0x00,
  0x93, 0x00,
  0x93, 0x00,
  0x93, 0x00,
  0x96, 0x00,
  0x97, 0x08,
  0x97, 0x19,
  0x97, 0x02,
  0x97, 0x0c,
  0x97, 0x24,
  0x97, 0x30,
  0x97, 0x28,
  0x97, 0x26,
  0x97, 0x02,
  0x97, 0x98,
  0x97, 0x80,
  0x97, 0x00,
  0x97, 0x00,
  0xc3, 0xeF,	//AWB碌脠脢鹿脛脺
  0xff, 0x00,
  0xba, 0xdc,
  0xbb, 0x08,
  0xb6, 0x24,
  0xb8, 0x33,
  0xb7, 0x20,
  0xb9, 0x30,
  0xb3, 0xb4,
  0xb4, 0xca,
  0xb5, 0x43,
  0xb0, 0x5c,
  0xb1, 0x4f,
  0xb2, 0x06,
  0xc7, 0x00,
  0xc6, 0x51,
  0xc5, 0x11,
  0xc4, 0x9c,
  0xbf, 0x00,
  0xbc, 0x64,
  0xa6, 0x00,
  0xa7, 0x1e,
  0xa7, 0x6b,
  0xa7, 0x47,
  0xa7, 0x33,
  0xa7, 0x00,
  0xa7, 0x23,
  0xa7, 0x2e,
  0xa7, 0x85,
  0xa7, 0x42,
  0xa7, 0x33,
  0xa7, 0x00,
  0xa7, 0x23,
  0xa7, 0x1b,
  0xa7, 0x74,
  0xa7, 0x42,
  0xa7, 0x33,
  0xa7, 0x00,
  0xa7, 0x23,
  0xc0, 0xc8,
  0xc1, 0x96,
  0x8c, 0x00,
  0x86, 0x3d,
  0x50, 0x92,
  0x51, 0x90,
  0x52, 0x2c,
  0x53, 0x00,
  0x54, 0x00,
  0x55, 0x88,
  0x5a, 0x50,
  0x5b, 0x3c,
  0x5c, 0x00,
  0xd3, 0x04,
  0x7f, 0x00,
  0xda, 0x00,
  0xe5, 0x1f,
  0xe1, 0x67,
  0xe0, 0x00,
  0xdd, 0x7f,
  0x05, 0x00,
  0xff, 0x00,
  0xe0, 0x04,
  0xc0, 0xc8,
  0xc1, 0x96,
  0x86, 0x3d,
  0x50, 0x92,
  0x51, 0x90,
  0x52, 0x2c,
  0x53, 0x00,
  0x54, 0x00,
  0x55, 0x88,
  0x57, 0x00,
  0x5a, 0x50,
  0x5b, 0x3c,
  0x5c, 0x00,
  0xd3, 0x04,
  0xe0, 0x00,
  0xFF, 0x00,
  0x05, 0x00,
  0xDA, 0x08,
  0xda, 0x09,
  0x98, 0x00,
  0x99, 0x00,
};

const unsigned char OV2640_JPEG_INIT[][2]=
{
  0xff, 0x00,
  0x2c, 0xff,
  0x2e, 0xdf,
  0xff, 0x01,
  0x3c, 0x32,
  0x11, 0x00,
  0x09, 0x02,
  0x04, 0x28,
  0x13, 0xe5,
  0x14, 0x48,
  0x2c, 0x0c,
  0x33, 0x78,
  0x3a, 0x33,
  0x3b, 0xfB,
  0x3e, 0x00,
  0x43, 0x11,
  0x16, 0x10,
  0x39, 0x92,
  0x35, 0xda,
  0x22, 0x1a,
  0x37, 0xc3,
  0x23, 0x00,
  0x34, 0xc0,
  0x36, 0x1a,
  0x06, 0x88,
  0x07, 0xc0,
  0x0d, 0x87,
  0x0e, 0x41,
  0x4c, 0x00,
  0x48, 0x00,
  0x5B, 0x00,
  0x42, 0x03,
  0x4a, 0x81,
  0x21, 0x99,
  0x24, 0x40,
  0x25, 0x38,
  0x26, 0x82,
  0x5c, 0x00,
  0x63, 0x00,
  0x61, 0x70,
  0x62, 0x80,
  0x7c, 0x05,
  0x20, 0x80,
  0x28, 0x30,
  0x6c, 0x00,
  0x6d, 0x80,
  0x6e, 0x00,
  0x70, 0x02,
  0x71, 0x94,
  0x73, 0xc1,
  0x12, 0x40,//0x40
  0x17, 0x11,
  0x18, 0x43,
  0x19, 0x00,
  0x1a, 0x4b,
  0x32, 0x09,
  0x37, 0xc0,
  0x4f, 0x60,
  0x50, 0xa8,
  0x6d, 0x00,
  0x3d, 0x38,
  0x46, 0x3f,
  0x4f, 0x60,
  0x0c, 0x3c,
  0xff, 0x00,
  0xe5, 0x7f,
  0xf9, 0xc0,
  0x41, 0x24,
  0xe0, 0x14,
  0x76, 0xff,
  0x33, 0xa0,
  0x42, 0x20,
  0x43, 0x18,
  0x4c, 0x00,
  0x87, 0xd5,
  0x88, 0x3f,
  0xd7, 0x03,
  0xd9, 0x10,
  0xd3, 0x82,
  0xc8, 0x08,
  0xc9, 0x80,
  0x7c, 0x00,
  0x7d, 0x00,
  0x7c, 0x03,
  0x7d, 0x48,
  0x7d, 0x48,
  0x7c, 0x08,
  0x7d, 0x20,
  0x7d, 0x10,
  0x7d, 0x0e,
  0x90, 0x00,
  0x91, 0x0e,
  0x91, 0x1a,
  0x91, 0x31,
  0x91, 0x5a,
  0x91, 0x69,
  0x91, 0x75,
  0x91, 0x7e,
  0x91, 0x88,
  0x91, 0x8f,
  0x91, 0x96,
  0x91, 0xa3,
  0x91, 0xaf,
  0x91, 0xc4,
  0x91, 0xd7,
  0x91, 0xe8,
  0x91, 0x20,
  0x92, 0x00,
  0x93, 0x06,
  0x93, 0xe3,
  0x93, 0x05,
  0x93, 0x05,
  0x93, 0x00,
  0x93, 0x04,
  0x93, 0x00,
  0x93, 0x00,
  0x93, 0x00,
  0x93, 0x00,
  0x93, 0x00,
  0x93, 0x00,
  0x93, 0x00,
  0x96, 0x00,
  0x97, 0x08,
  0x97, 0x19,
  0x97, 0x02,
  0x97, 0x0c,
  0x97, 0x24,
  0x97, 0x30,
  0x97, 0x28,
  0x97, 0x26,
  0x97, 0x02,
  0x97, 0x98,
  0x97, 0x80,
  0x97, 0x00,
  0x97, 0x00,
  0xc3, 0xed,
  0xa4, 0x00,
  0xa8, 0x00,
  0xc5, 0x11,
  0xc6, 0x51,
  0xbf, 0x80,
  0xc7, 0x10,
  0xb6, 0x66,
  0xb8, 0xA5,
  0xb7, 0x64,
  0xb9, 0x7C,
  0xb3, 0xaf,
  0xb4, 0x97,
  0xb5, 0xFF,
  0xb0, 0xC5,
  0xb1, 0x94,
  0xb2, 0x0f,
  0xc4, 0x5c,
  0xc0, 0x64,
  0xc1, 0x4B,
  0x8c, 0x00,
  0x86, 0x3D,
  0x50, 0x00,
  0x51, 0xC8,
  0x52, 0x96,
  0x53, 0x00,
  0x54, 0x00,
  0x55, 0x00,
  0x5a, 0xC8,
  0x5b, 0x96,
  0x5c, 0x00,
  0xd3, 0x7f,
  0xc3, 0xed,
  0x7f, 0x00,
  0xda, 0x00,
  0xe5, 0x1f,
  0xe1, 0x67,
  0xe0, 0x00,
  0xdd, 0x7f,
  0x05, 0x00,

  0x12, 0x40,//0x40
  0xd3, 0x7f,
  0xc0, 0x16,
  0xC1, 0x12,
  0x8c, 0x00,
  0x86, 0x3d,
  0x50, 0x00,
  0x51, 0x2C,
  0x52, 0x24,
  0x53, 0x00,
  0x54, 0x00,
  0x55, 0x00,
  0x5A, 0x2c,
  0x5b, 0x24,
  0x5c, 0x00,
};

const unsigned char OV2640_YUV422[][2]=
{
  0xFF, 0x00,
  0x05, 0x00,
  0xDA, 0x10,
  0xD7, 0x03,
  0xDF, 0x00,
  0x33, 0x80,
  0x3C, 0x40,
  0xe1, 0x77,
  0x00, 0x00,
};

const unsigned char OV2640_JPEG[][2]=
{
  0xe0, 0x14,
  0xe1, 0x77,
  0xe5, 0x1f,
  0xd7, 0x03,
  0xda, 0x10,
  0xe0, 0x00,
  0xFF, 0x01,
  0x04, 0x08,
};

/* JPG 160x120 */
const unsigned char OV2640_160x120_JPEG[][2]=
{
  0xff, 0x01,
  0x12, 0x40,
  0x17, 0x11,
  0x18, 0x43,
  0x19, 0x00,
  0x1a, 0x4b,
  0x32, 0x09,
  0x4f, 0xca,
  0x50, 0xa8,
  0x5a, 0x23,
  0x6d, 0x00,
  0x39, 0x12,
  0x35, 0xda,
  0x22, 0x1a,
  0x37, 0xc3,
  0x23, 0x00,
  0x34, 0xc0,
  0x36, 0x1a,
  0x06, 0x88,
  0x07, 0xc0,
  0x0d, 0x87,
  0x0e, 0x41,
  0x4c, 0x00,
  0xff, 0x00,
  0xe0, 0x04,
  0xc0, 0x64,
  0xc1, 0x4b,
  0x86, 0x35,
  0x50, 0x92,
  0x51, 0xc8,
  0x52, 0x96,
  0x53, 0x00,
  0x54, 0x00,
  0x55, 0x00,
  0x57, 0x00,
  0x5a, 0x28,
  0x5b, 0x1e,
  0x5c, 0x00,
  0xe0, 0x00,
};

/* JPG, 0x176x144 */
const unsigned char OV2640_176x144_JPEG[][2]=
{
  0xff, 0x01,
  0x12, 0x40,
  0x17, 0x11,
  0x18, 0x43,
  0x19, 0x00,
  0x1a, 0x4b,
  0x32, 0x09,
  0x4f, 0xca,
  0x50, 0xa8,
  0x5a, 0x23,
  0x6d, 0x00,
  0x39, 0x12,
  0x35, 0xda,
  0x22, 0x1a,
  0x37, 0xc3,
  0x23, 0x00,
  0x34, 0xc0,
  0x36, 0x1a,
  0x06, 0x88,
  0x07, 0xc0,
  0x0d, 0x87,
  0x0e, 0x41,
  0x4c, 0x00,
  0xff, 0x00,
  0xe0, 0x04,
  0xc0, 0x64,
  0xc1, 0x4b,
  0x86, 0x35,
  0x50, 0x92,
  0x51, 0xc8,
  0x52, 0x96,
  0x53, 0x00,
  0x54, 0x00,
  0x55, 0x00,
  0x57, 0x00,
  0x5a, 0x2c,
  0x5b, 0x24,
  0x5c, 0x00,
  0xe0, 0x00,
};

/* JPG 320x240 */
const unsigned char OV2640_320x240_JPEG[][2]=
{
{0xff, 0x01},
  {0x11, 0x01},
  {0x12, 0x00}, // Bit[6:4]: Resolution selection//0x02涓哄僵鏉�
  {0x17, 0x11}, // HREFST[10:3]
  {0x18, 0x75}, // HREFEND[10:3]
  {0x32, 0x36}, // Bit[5:3]: HREFEND[2:0]; Bit[2:0]: HREFST[2:0]
  {0x19, 0x01}, // VSTRT[9:2]
  {0x1a, 0x97}, // VEND[9:2]
  {0x03, 0x0f}, // Bit[3:2]: VEND[1:0]; Bit[1:0]: VSTRT[1:0]
  {0x37, 0x40},
  {0x4f, 0xbb},
  {0x50, 0x9c},
  {0x5a, 0x57},
  {0x6d, 0x80},
  {0x3d, 0x34},
  {0x39, 0x02},
  {0x35, 0x88},
  {0x22, 0x0a},
  {0x37, 0x40},
  {0x34, 0xa0},
  {0x06, 0x02},
  {0x0d, 0xb7},
  {0x0e, 0x01},

  ////////////////
  /*
  //176*144
   0xff,      0x00,
      0xc0,      0xC8,
      0xc1,      0x96,
      0x8c,      0x00,
      0x86,      0x3D,
      0x50,      0x9B,
      0x51,      0x90,
      0x52,      0x2C,
      0x53,      0x00,
      0x54,      0x00,
      0x55,      0x88,
      0x5a,      0x2C,
      0x5b,      0x24,
      0x5c,      0x00,
      0xd3,      0x7F,
	  ////////////
	  */

	 ////////////////
	 //320*240
	  0xff,      0x00,
      0xe0,      0x04,
      0xc0,      0xc8,
      0xc1,      0x96,
      0x86,      0x3d,
      0x50,      0x92,
      0x51,      0x90,
      0x52,      0x2c,
      0x53,      0x00,
      0x54,      0x00,
      0x55,      0x88,
      0x57,      0x00,
      0x5a,      0x50,
      0x5b,      0x3c,
      0x5c,      0x00,
      0xd3,      0x7F,
      0xe0,      0x00,
	  ///////////////////

  /*
0xff,      0x00,
      0xe0,      0x04,
      0xc0,      0xc8,
      0xc1,      0x96,
      0x86,      0x35,
      0x50,      0x92,
      0x51,      0x90,
      0x52,      0x2c,
      0x53,      0x00,
      0x54,      0x00,
      0x55,      0x88,
      0x57,      0x00,
      0x5a,      0x58,
      0x5b,      0x48,
      0x5c,      0x00,
      0xd3,      0x08,
      0xe0,      0x00
*/
/*
//640*480
 	  0xff,      0x00,
      0xe0,      0x04,
      0xc0,      0xc8,
      0xc1,      0x96,
      0x86,      0x3d,
      0x50,      0x89,
      0x51,      0x90,
      0x52,      0x2c,
      0x53,      0x00,
      0x54,      0x00,
      0x55,      0x88,
      0x57,      0x00,
      0x5a,      0xa0,
      0x5b,      0x78,
      0x5c,      0x00,
      0xd3,      0x04,
      0xe0,      0x00
	  */
	  /////////////////////
	  /*
	  //800*600
	  0xff,      0x00,
      0xe0,      0x04,
      0xc0,      0xc8,
      0xc1,      0x96,
      0x86,      0x35,
      0x50,      0x89,
      0x51,      0x90,
      0x52,      0x2c,
      0x53,      0x00,
      0x54,      0x00,
      0x55,      0x88,
      0x57,      0x00,
      0x5a,      0xc8,
      0x5b,      0x96,
      0x5c,      0x00,
      0xd3,      0x02,
      0xe0,      0x00
	  */
	  /*
	  //1280*1024

	  0xff,      0x00,
      0xe0,      0x04,
      0xc0,      0xc8,
      0xc1,      0x96,
      0x86,      0x3d,
      0x50,      0x00,
      0x51,      0x90,
      0x52,      0x2c,
      0x53,      0x00,
      0x54,      0x00,
      0x55,      0x88,
      0x57,      0x00,
      0x5a,      0x40,
      0x5b,      0xf0,
      0x5c,      0x01,
      0xd3,      0x02,
      0xe0,      0x00
	  */
	  /*
	  /////////////////////
	  //1600*1200

	  0xff,      0x00,
      0xe0,      0x04,
      0xc0,      0xc8,
      0xc1,      0x96,
      0x86,      0x3d,
      0x50,      0x00,
      0x51,      0x90,
      0x52,      0x2c,
      0x53,      0x00,
      0x54,      0x00,
      0x55,      0x88,
      0x57,      0x00,
      0x5a,      0x90,
      0x5b,      0x2C,
      0x5c,      0x05,//bit2->1;bit[1:0]->1
      0xd3,      0x02,
      0xe0,      0x00
	  /////////////////////
	  */
	  /*
	  //1024*768
	   0xff,      0x00,
      0xc0,      0xC8,
      0xc1,      0x96,
      0x8c,      0x00,
      0x86,      0x3D,
      0x50,      0x00,
      0x51,      0x90,
      0x52,      0x2C,
      0x53,      0x00,
      0x54,      0x00,
      0x55,      0x88,
      0x5a,      0x00,
      0x5b,      0xC0,
      0x5c,      0x01,
      0xd3,      0x02
	  */
};

/* JPG 352x288 */
const unsigned char OV2640_352x288_JPEG[][2]=
{
  0xff, 0x01,
  0x12, 0x40,
  0x17, 0x11,
  0x18, 0x43,
  0x19, 0x00,
  0x1a, 0x4b,
  0x32, 0x09,
  0x4f, 0xca,
  0x50, 0xa8,
  0x5a, 0x23,
  0x6d, 0x00,
  0x39, 0x12,
  0x35, 0xda,
  0x22, 0x1a,
  0x37, 0xc3,
  0x23, 0x00,
  0x34, 0xc0,
  0x36, 0x1a,
  0x06, 0x88,
  0x07, 0xc0,
  0x0d, 0x87,
  0x0e, 0x41,
  0x4c, 0x00,

  0xff, 0x00,
  0xe0, 0x04,
  0xc0, 0x64,
  0xc1, 0x4b,
  0x86, 0x35,
  0x50, 0x89,
  0x51, 0xc8,
  0x52, 0x96,
  0x53, 0x00,
  0x54, 0x00,
  0x55, 0x00,
  0x57, 0x00,
  0x5a, 0x58,
  0x5b, 0x48,
  0x5c, 0x00,
  0xe0, 0x00,
};

/**
  * @brief  Writes a byte at a specific Camera register
  * @param  Addr: ov2640 register address.
  * @param  Data: Data to be written to the specific register
  * @retval 0x00 if write operation is OK.
  *       0xFF if timeout condition occured (device not connected or bus error).
  */
uint8_t OV2640_WriteReg(uint8_t Addr, uint8_t Data)
{
	uint8_t sta = 0;
	uint8_t data[2] = {Addr, Data};
	sta = HAL_I2C_Master_Transmit(&I2C2_Handle, ov2640_DEVICE_WRITE_ADDRESS, data, 2, OV_TIMEOUT_MAX);
	if(sta)
		return sta;
	/* If operation is OK, return 0 */
	return 0;
}

/**
  * @brief  Writes to a specific Camera register
  */
uint8_t OV2640_WriteReg16(uint16_t address, uint16_t Data)
{
	uint8_t result = OV2640_WriteReg(address, (uint8_t)( Data >> 8)); // write upper byte
	result |= OV2640_WriteReg(0xF0, (uint8_t) Data); // write lower byte
	return result;
}

/**
  * @brief  Reads a byte from a specific Camera register
  * @param  Addr: ov2640 register address.
  * @retval data read from the specific register or 0xFF if timeout condition
  *         occured.
  */
uint8_t OV2640_ReadReg(uint8_t Addr)
{
  uint8_t Data = 0;
  if( HAL_I2C_Master_Transmit(&I2C2_Handle, ov2640_DEVICE_WRITE_ADDRESS, &Addr, 1, OV_TIMEOUT_MAX))
	  return 0xff;

  if( HAL_I2C_Master_Receive(&I2C2_Handle, ov2640_DEVICE_WRITE_ADDRESS, &Data, 1, OV_TIMEOUT_MAX) )
	  return 0xff;

  /* return the read data */
  return Data;
}

/**
  * @brief  Reads from a specific Camera register
  */
uint16_t OV2640_ReadReg16(uint8_t address)
{
	uint16_t result = OV2640_ReadReg(address) << 8; // read upper byte
	result |= OV2640_ReadReg(0xF0); // read lower byte
	return result;
}

/**
  * @brief  Resets the OV2640 camera.
  * @param  None
  * @retval None
  */
void OV2640_HW_Init(void)
{
	/* I2C2 clock enable */
	__HAL_RCC_I2C2_CLK_ENABLE();

	/* GPIOB clock enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure I2C2 GPIOs */
	GPIO_InitStructure.Pin = GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Set the I2C structure parameters */
	I2C2_Handle.Instance = I2C2;

	/* I2C DeInit */
	HAL_I2C_DeInit(&I2C2_Handle);

	I2C2_Handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
	I2C2_Handle.Init.OwnAddress1 = 0xFE;
	I2C2_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	I2C2_Handle.Init.ClockSpeed = 100000;
	HAL_I2C_Init(&I2C2_Handle);
}

/**
  * @brief  Resets the OV2640 camera.
  * @param  None
  * @retval None
  */
void OV2640_Reset(void)
{
  OV2640_WriteReg(OV2640_DSP_RA_DLMT, 0x01);
  OV2640_WriteReg(OV2640_SENSOR_COM7, 0x80);
}

/**
  * @brief  Reads the OV2640 Manufacturer identifier.
  * @param  OV2640ID: Pointer to the OV2640 Manufacturer identifier
  * @retval None
  */
void OV2640_ReadID(OV2640_IDTypeDef *OV2640ID)
{
  OV2640_WriteReg(OV2640_DSP_RA_DLMT, 0x01);
  OV2640ID->Manufacturer_ID1 = OV2640_ReadReg(OV2640_SENSOR_MIDH);
  OV2640ID->Manufacturer_ID2 = OV2640_ReadReg(OV2640_SENSOR_MIDL);
  OV2640ID->PIDH = OV2640_ReadReg(OV2640_SENSOR_PIDH);
  OV2640ID->PIDL = OV2640_ReadReg(OV2640_SENSOR_PIDL);
}

/**
  * @brief  Reads the OV2640 QVGA CONFIG.
  * @param  None
  * @retval None
  */
void OV2640_QQVGAConfig(void)
{
  uint32_t i;

  OV2640_Reset();
  HAL_Delay(200);

  /* Initialize OV2640
  for(i=0; i<(sizeof(OV2640_QQVGA)/2); i++)
  {
    OV2640_WriteReg(OV2640_QQVGA[i][0], OV2640_QQVGA[i][1]);
    HAL_Delay(2);
  }*/
  /* Initialize OV2640 */
  for(i=0; i<(sizeof(ov2640_sxga_init_reg_tbl)/2); i++)
  {
    OV2640_WriteReg(ov2640_sxga_init_reg_tbl[i][0], ov2640_sxga_init_reg_tbl[i][1]);
    HAL_Delay(2);
  }
}

/**
  * @brief  Configures the OV2640 camera in JPEG mode.
  * @param  JPEGImageSize: JPEG image size
  * @retval None
  */
void OV2640_JPEGConfig(ImageFormat_TypeDef ImageFormat)
{
  uint32_t i;

  OV2640_Reset();
  HAL_Delay(200);

  /* Initialize OV2640 */
  for(i=0; i<(sizeof(OV2640_JPEG_INIT)/2); i++)
  {
    OV2640_WriteReg(OV2640_JPEG_INIT[i][0], OV2640_JPEG_INIT[i][1]);
    HAL_Delay(1);
  }

  /* Set to output YUV422 */
  for(i=0; i<(sizeof(OV2640_YUV422)/2); i++)
  {
    OV2640_WriteReg(OV2640_YUV422[i][0], OV2640_YUV422[i][1]);
    HAL_Delay(1);
  }

  OV2640_WriteReg(0xff, 0x01);
  OV2640_WriteReg(0x15, 0x00);

  /* Set to output JPEG */
  for(i=0; i<(sizeof(OV2640_JPEG)/2); i++)
  {
    OV2640_WriteReg(OV2640_JPEG[i][0], OV2640_JPEG[i][1]);
    HAL_Delay(1);
  }

  HAL_Delay(100);

  switch(ImageFormat)
  {
    case JPEG_160x120:
    {
      for(i=0; i<(sizeof(OV2640_160x120_JPEG)/2); i++)
      {
        OV2640_WriteReg(OV2640_160x120_JPEG[i][0], OV2640_160x120_JPEG[i][1]);
        HAL_Delay(1);
      }
      break;
    }
    case JPEG_176x144:
    {
      for(i=0; i<(sizeof(OV2640_176x144_JPEG)/2); i++)
      {
        OV2640_WriteReg(OV2640_176x144_JPEG[i][0], OV2640_176x144_JPEG[i][1]);
      }
      break;
    }
    case JPEG_320x240:
    {
       for(i=0; i<(sizeof(OV2640_320x240_JPEG)/2); i++)
      {
        OV2640_WriteReg(OV2640_320x240_JPEG[i][0], OV2640_320x240_JPEG[i][1]);
        HAL_Delay(1);
      }
      break;
    }
    case JPEG_352x288:
    {
      for(i=0; i<(sizeof(OV2640_352x288_JPEG)/2); i++)
      {
        OV2640_WriteReg(OV2640_352x288_JPEG[i][0], OV2640_352x288_JPEG[i][1]);
      }
      break;
    }
    default:
    {
      for(i=0; i<(sizeof(OV2640_160x120_JPEG)/2); i++)
      {
        OV2640_WriteReg(OV2640_160x120_JPEG[i][0], OV2640_160x120_JPEG[i][1]);
      }
      break;
    }
  }
}

/**
  * @brief  Configures the OV2640 camera brightness.
  * @param  Brightness: Brightness value, where Brightness can be:
  *         0x40 for Brightness +2,
  *         0x30 for Brightness +1,
  *         0x20 for Brightness 0,
  *         0x10 for Brightness -1,
  *         0x00 for Brightness -2,
  * @retval None
  */
void OV2640_BrightnessConfig(uint8_t Brightness)
{
  OV2640_WriteReg(0xff, 0x00);
  OV2640_WriteReg(0x7c, 0x00);
  OV2640_WriteReg(0x7d, 0x04);
  OV2640_WriteReg(0x7c, 0x09);
  OV2640_WriteReg(0x7d, Brightness);
  OV2640_WriteReg(0x7d, 0x00);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const uint8_t OV2640_AUTOEXPOSURE_LEVEL0[]=
{
	0xFF,	0x01,	0xff,
	0x24,	0x20,	0xff,
	0x25,	0x18,	0xff,
	0x26,	0x60,	0xff,
	0x00,	0x00,	0x00
};

const uint8_t OV2640_AUTOEXPOSURE_LEVEL1[]=
{
	0xFF,	0x01,	0xff,
	0x24,	0x34,	0xff,
	0x25,	0x1c,	0xff,
	0x26,	0x70,	0xff,
	0x00,	0x00,	0x00
};

const uint8_t OV2640_AUTOEXPOSURE_LEVEL2[]=
{
	0xFF,	0x01,	0xff,
	0x24,	0x3e,	0xff,
	0x25,	0x38,	0xff,
	0x26,	0x81,	0xff,
	0x00,	0x00,	0x00
};
const uint8_t OV2640_AUTOEXPOSURE_LEVEL3[]=
{
	0xFF,	0x01,	0xff,
	0x24,	0x48,	0xff,
	0x25,	0x40,	0xff,
	0x26,	0x81,	0xff,
	0x00,	0x00,	0x00
};
const uint8_t OV2640_AUTOEXPOSURE_LEVEL4[]=
{
	0xFF,	0x01,	0xff,
	0x24,	0x58,	0xff,
	0x25,	0x50,	0xff,
	0x26,	0x92,	0xff,
	0x00,	0x00,	0x00
};

void SCCB_WriteRegs(const uint8_t *pbuf)
{
	OV2640_WriteReg(pbuf[0], pbuf[1]);
	OV2640_WriteReg(pbuf[2], pbuf[3]);
	OV2640_WriteReg(pbuf[4], pbuf[5]);
	OV2640_WriteReg(pbuf[6], pbuf[7]);
	OV2640_WriteReg(pbuf[8], pbuf[9]);
	OV2640_WriteReg(pbuf[10], pbuf[11]);
}

void OV2640_AutoExposure(uint8_t level)
{
	switch(level)
	{
		case 0:
			SCCB_WriteRegs(OV2640_AUTOEXPOSURE_LEVEL0);
			break;
		case 1:
			SCCB_WriteRegs(OV2640_AUTOEXPOSURE_LEVEL1);
			break;
		case 2:
			SCCB_WriteRegs(OV2640_AUTOEXPOSURE_LEVEL2);
			break;
		case 3:
			SCCB_WriteRegs(OV2640_AUTOEXPOSURE_LEVEL3);
			break;
		case 4:
			SCCB_WriteRegs(OV2640_AUTOEXPOSURE_LEVEL4);
			break;
		default:
			SCCB_WriteRegs(OV2640_AUTOEXPOSURE_LEVEL0);
			break;
	}

}
/**
  * @brief  Configures the OV2640 camera Black and white mode.
  * @param  BlackWhite: BlackWhite value, where BlackWhite can be:
  *         0x18 for B&W,
  *         0x40 for Negative,
  *         0x58 for B&W negative,
  *         0x00 for Normal,
  * @retval None
  */
void OV2640_BandWConfig(uint8_t BlackWhite)
{
  OV2640_WriteReg(0xff, 0x00);
  OV2640_WriteReg(0x7c, 0x00);
  OV2640_WriteReg(0x7d, BlackWhite);
  OV2640_WriteReg(0x7c, 0x05);
  OV2640_WriteReg(0x7d, 0x80);
  OV2640_WriteReg(0x7d, 0x80);
}

/**
  * @brief  Configures the OV2640 camera color effects.
  * @param  value1: Color effects value1
  * @param  value2: Color effects value2
  *         where value1 and value2 can be:
  *         value1 = 0x40, value2 = 0xa6 for Antique,
  *         value1 = 0xa0, value2 = 0x40 for Bluish,
  *         value1 = 0x40, value2 = 0x40 for Greenish,
  *         value1 = 0x40, value2 = 0xc0 for Reddish,
  * @retval None
  */
void OV2640_ColorEffectsConfig(uint8_t value1, uint8_t value2)
{
  OV2640_WriteReg(0xff, 0x00);
  OV2640_WriteReg(0x7c, 0x00);
  OV2640_WriteReg(0x7d, 0x18);
  OV2640_WriteReg(0x7c, 0x05);
  OV2640_WriteReg(0x7d, value1);
  OV2640_WriteReg(0x7d, value2);
}

/**
  * @brief  Configures the OV2640 camera contrast.
  * @param  value1: Contrast value1
  * @param  value2: Contrast value2
  *         where value1 and value2 can be:
  *         value1 = 0x28, value2 = 0x0c for Contrast +2,
  *         value1 = 0x24, value2 = 0x16 for Contrast +1,
  *         value1 = 0x20, value2 = 0x20 for Contrast 0,
  *         value1 = 0x1c, value2 = 0x2a for Contrast -1,
  *         value1 = 0x18, value2 = 0x34 for Contrast -2,
  * @retval None
  */
void OV2640_ContrastConfig(uint8_t value1, uint8_t value2)
{
  OV2640_WriteReg(0xff, 0x00);
  OV2640_WriteReg(0x7c, 0x00);
  OV2640_WriteReg(0x7d, 0x04);
  OV2640_WriteReg(0x7c, 0x07);
  OV2640_WriteReg(0x7d, 0x20);
  OV2640_WriteReg(0x7d, value1);
  OV2640_WriteReg(0x7d, value2);
  OV2640_WriteReg(0x7d, 0x06);
}

/**
  * @brief  Changes sensor context based on settings
  */
void ov2640_set_context()
{
	uint16_t new_control;
	if (global_data.param[PARAM_VIDEO_ONLY])
		new_control = 0x8188; // Context B
	else
		new_control = 0x0188; // Context A

	//ov2640_WriteReg16(MTV_CHIP_CONTROL_REG, new_control);
}
