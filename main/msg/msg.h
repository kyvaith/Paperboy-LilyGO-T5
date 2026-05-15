//
// MSG
// Copyright 2026 Wenting Zhang
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
#pragma once

#include <stdint.h>
#include <stdbool.h>

// Platform definition
#define PLAT_M5PAPERS3

#if defined(PLAT_M5PAPERS3)
#define EPD_WIDTH           960
#define EPD_HEIGHT          540
#define EPD_LINE_PAD        16
#define EPD_XCK             20000000
#define EPD_BUSW            8

#define EPD_IMAGE_FIELDS    8

#define EPD_PWR_EN_PIN      45
#define EPD_BST_EN_PIN      46
#define EPD_SDCE_PIN        13
#define EPD_SDLE_PIN        15
#define EPD_SDCK_PIN        16
#define EPD_GDSP_PIN        17
#define EPD_GDCK_PIN        18

#define EPD_D0_PIN          6
#define EPD_D1_PIN          14
#define EPD_D2_PIN          7
#define EPD_D3_PIN          12
#define EPD_D4_PIN          9
#define EPD_D5_PIN          11
#define EPD_D6_PIN          8
#define EPD_D7_PIN          10
#endif

#define EPD_VIDEO_WIDTH     (144*3)
#define EPD_VIDEO_HEIGHT    (160)

#define EPD_FB_SIZE         (EPD_WIDTH * EPD_HEIGHT / 8)
#define EPD_VIDEO_FB_SIZE   (EPD_VIDEO_WIDTH * EPD_VIDEO_HEIGHT / 8)

#define MSG_TASK_PRIORITY   1

void msg_init(void);
void msg_start(void);
void msg_display_image(uint8_t *img, bool to_white);
void msg_enable_video(bool en);
uint8_t *msg_flip(void);
