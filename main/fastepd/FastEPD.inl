//
// C core functions for the FastEPD library
// Written by Larry Bank (bitbank@pobox.com)
// Copyright (C) 2024-2026 BitBank Software, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//    http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//===========================================================================
//
#include "FastEPD.h"
#ifndef __LINUX__
#ifdef CONFIG_IDF_TARGET_ESP32C5
#include "driver/parlio_tx.h"
#else
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#endif
#include <esp_log.h>
#if PSRAM != enabled && !defined(CONFIG_ESP32_SPIRAM_SUPPORT) && !defined(CONFIG_ESP32S3_SPIRAM_SUPPORT) && !defined(CONFIG_ESP32C5_SPIRAM_SUPPORT)
#error "Please enable PSRAM support"
#endif
#endif // !__LINUX__

#ifndef __BB_EP__
#define __BB_EP__
#pragma GCC optimize("O2")

// For measuring the performance of each stage of updates
//#define SHOW_TIME

const uint8_t u8M5Matrix[] = {
    /* 0 */  1, 1, 1, 1, 1, 1, 1, 1,
    /* 1 */  2, 2, 1, 1, 2, 1, 1, 1,
    /* 2 */  2, 2, 1, 1, 1, 1, 2, 1,
    /* 3 */  2, 2, 1, 1, 2, 2, 1, 1,
    /* 4 */  2, 2, 2, 2, 1, 1, 2, 1,
    /* 5 */  2, 2, 1, 1, 1, 2, 2, 1,
    /* 6 */  2, 2, 1, 1, 2, 1, 1, 2,
    /* 7 */  2, 2, 2, 1, 2, 1, 1, 2,
    /* 8 */  2, 2, 2, 2, 2, 1, 2, 1,
    /* 9 */  1, 1, 1, 1, 1, 1, 2, 2,
    /* 10 */  2, 2, 1, 1, 1, 1, 2, 2,
    /* 11 */  1, 1, 1, 1, 2, 1, 2, 2,
    /* 12 */  2, 2, 1, 1, 2, 1, 2, 2,
    /* 13 */  2, 1, 1, 2, 2, 1, 2, 2,
    /* 14 */  2, 2, 1, 2, 2, 1, 2, 2,
    /* 15 */  2, 2, 2, 2, 2, 2, 2, 2,
    };

// Forward references
int bbepSetPixel2Clr(void *pb, int x, int y, unsigned char ucColor);
void bbepSetPixelFast2Clr(void *pb, int x, int y, unsigned char ucColor);
int bbepSetPanelSize(FASTEPDSTATE *pState, int width, int height, int flags, int iVCOM);
int bbepSetCustomMatrix(FASTEPDSTATE *pState, const uint8_t *pMatrix, size_t matrix_size);
//
// Pre-defined panels for popular products and boards
//
// width, height, bus_speed, flags, data[8], bus_width, ioPWR, ioSPV, ioCKV, ioSPH, ioOE, ioLE,
// ioCL, ioPWR_Good, ioSDA, ioSCL, ioShiftSTR/Wakeup, ioShiftMask/vcom, ioDCDummy, graymatrix, sizeof(graymatrix), iLinePadding
const BBPANELDEF panelDefs[] = {
    {0,0,0,0,{0},0,0,0,0,0,0,0,0,0,0,0,0,0,0,NULL,0,0,0}, // BB_PANEL_NONE
    {960, 540, 20000000, BB_PANEL_FLAG_NONE, {6,14,7,12,9,11,8,10}, 8, 46, 17, 18, 13, 45, 15,
      16, BB_NOT_USED, BB_NOT_USED, BB_NOT_USED, BB_NOT_USED, BB_NOT_USED, 47, u8M5Matrix, sizeof(u8M5Matrix), 16, -1600}, // BB_PANEL_M5PAPERS3
};

//
// Forward references for panel callback functions
//
// M5Stack PaperS3
int PaperS3EinkPower(void *pBBEP, int bOn);
int PaperS3IOInit(void *pBBEP);
void PaperS3RowControl(void *pBBEP, int iMode);

// List of predefined callback functions for the panels supported by bb_epdiy
// BB_EINK_POWER, BB_IO_INIT, BB_ROW_CONTROL
const BBPANELPROCS panelProcs[] = {
    {NULL,NULL,NULL,NULL,NULL}, // BB_PANEL_NONE
    {PaperS3EinkPower, PaperS3IOInit, PaperS3RowControl, NULL, NULL}, // BB_PANEL_M5PAPERS3
};

uint8_t ioRegs[24]; // MCP23017 copy of I/O register state so that we can just write new bits
static uint16_t LUTW_16[256];
static uint16_t LUTB_16[256];
static uint16_t LUTBW_16[256];
// Lookup tables for grayscale mode
static uint8_t *pGrayLower = NULL, *pGrayUpper = NULL;
volatile bool dma_is_done = true;
static uint8_t u8Cache[1024]; // used also for masking a row of 2-bit codes, needs to handle up to 4096 pixels wide
#ifndef __LINUX__
static gpio_num_t u8CKV, u8SPH;
static uint8_t bSlowSPH = 0;

#ifdef CONFIG_IDF_TARGET_ESP32C5
parlio_tx_unit_config_t parlio_tx_config;
parlio_tx_unit_handle_t parlio_tx_handle;
static bool c5_notify_dma_ready(parlio_tx_unit_handle_t handle, const parlio_tx_done_event_data_t *edata, void *user_ctx)
{
    if (bSlowSPH) {
        gpio_set_level(u8SPH, 1); // CS deactivate
    }
    dma_is_done = true;
    return false;
}
#else
static bool s3_notify_dma_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{           
    if (bSlowSPH) {
        gpio_set_level(u8SPH, 1); // CS deactivate
    }
    dma_is_done = true;
    return false;
}
// Maximum line width = 1024 * 4 = 4096 pixels
#define MAX_TX_SIZE 1024
static esp_lcd_i80_bus_config_t s3_bus_config = {
    .dc_gpio_num = (gpio_num_t)0,
    .wr_gpio_num = (gpio_num_t)0,
    .clk_src = /*LCD_CLK_SRC_DEFAULT,*/ LCD_CLK_SRC_PLL160M,
    .data_gpio_nums = {(gpio_num_t)0},
    .bus_width = 0,
    .max_transfer_bytes = MAX_TX_SIZE, 
#if ESP_IDF_VERSION <= ESP_IDF_VERSION_VAL(5, 0, 0)
    .psram_trans_align = 0, // 0 = use default values
    .sram_trans_align = 0,
#else
    .dma_burst_size = 32,
#endif
};
static esp_lcd_panel_io_i80_config_t s3_io_config = {
        .cs_gpio_num = (gpio_num_t)0,
        .pclk_hz = 12000000,
        .trans_queue_depth = 4,
        .on_color_trans_done = s3_notify_dma_ready,
        .user_ctx = NULL, // debug
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .flags = {
            .cs_active_high = 0,
            .reverse_color_bits = 0,
            .swap_color_bytes = 0, // Swap can be done in software (default) or DMA
            .pclk_active_neg = 0,
            .pclk_idle_low = 0,
        },
    };

static esp_lcd_i80_bus_handle_t i80_bus = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;
#endif // S3/C5
#endif // !__LINUX__


// Control the DC/DC power circuit of the M5Stack PaperS3
//
int PaperS3EinkPower(void *pBBEP, int bOn)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    if (bOn == pState->pwr_on) return BBEP_SUCCESS; // already on
    if (bOn) {
        gpio_set_level((gpio_num_t)pState->panelDef.ioOE, 1);
        delayMicroseconds(100);
        gpio_set_level((gpio_num_t)pState->panelDef.ioPWR, 1);
        delayMicroseconds(100);
        gpio_set_level((gpio_num_t)pState->panelDef.ioSPV, 1);
        gpio_set_level((gpio_num_t)pState->panelDef.ioSPH, 1);
        pState->pwr_on = 1;
    } else { // power off
        gpio_set_level((gpio_num_t)pState->panelDef.ioPWR, 0);
        delay(1); // give a little time to power down
        gpio_set_level((gpio_num_t)pState->panelDef.ioOE, 0);
        delayMicroseconds(100);
        gpio_set_level((gpio_num_t)pState->panelDef.ioSPV, 0);
        pState->pwr_on = 0;
    }
    return BBEP_SUCCESS;
} /* PaperS3EinkPower() */
//
// Initialize the (non parallel data) lines of the M5Stack PaperS3
//
int PaperS3IOInit(void *pBBEP)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    bbepPinMode(pState->panelDef.ioPWR, OUTPUT);
    bbepPinMode(pState->panelDef.ioSPV, OUTPUT);
    bbepPinMode(pState->panelDef.ioCKV, OUTPUT);
    bbepPinMode(pState->panelDef.ioSPH, OUTPUT);
    bbepPinMode(pState->panelDef.ioOE, OUTPUT);
    bbepPinMode(pState->panelDef.ioLE, OUTPUT);
    bbepPinMode(pState->panelDef.ioCL, OUTPUT);
    return BBEP_SUCCESS;
} /* PaperS3IOInit() */
//
// Start or step the current row on the M5Stack PaperS3
//
void PaperS3RowControl(void *pBBEP, int iType)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    gpio_num_t ckv = (gpio_num_t)pState->panelDef.ioCKV;
    gpio_num_t spv = (gpio_num_t)pState->panelDef.ioSPV;
    gpio_num_t le = (gpio_num_t)pState->panelDef.ioLE;

    if (iType == ROW_START) {
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(7);
        gpio_set_level(spv, 0); // SPV off
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(8);                    
        gpio_set_level(spv, 1); // SPV on
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(18);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(18);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); // CKV on
    } else if (iType == ROW_STEP) {
        gpio_set_level(ckv, 0); // CKV off
        gpio_set_level(le, 1); // LE toggle
        gpio_set_level(le, 0);
        delayMicroseconds(0);
    }
} /* PaperS3RowControl() */

void bbepRowControl(FASTEPDSTATE *pState, int iType)
{
    (*(pState->pfnRowControl))(pState, iType);
    return;
} /* bbepRowControl() */

// The data needs to come from a DMA buffer or the Espressif DMA driver
// will allocate (and leak) an internal buffer each time
#ifndef __LINUX__
void bbepWriteRow(FASTEPDSTATE *pState, uint8_t *pData, int iLen, int bRowStep)
{
    esp_err_t err;
    
//    Serial.printf("bbepWriteRow %d bytes\n", iLen);
    
    while (!dma_is_done) {
        delayMicroseconds(1);
    }
    if (bRowStep) {
        bbepRowControl(pState, ROW_STEP);
    }
    if (bSlowSPH) {
        gpio_set_level(u8SPH, 0); // SPH/CS active
//        gpio_set_level(u8CKV, 1); // CKV on
    }
    dma_is_done = false;
    gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 1); // CKV on
#ifdef CONFIG_IDF_TARGET_ESP32C5
    parlio_transmit_config_t tx_cfg;
    memset(&tx_cfg, 0, sizeof(tx_cfg));
    err = parlio_tx_unit_transmit(parlio_tx_handle, pData, (iLen + pState->panelDef.iLinePadding) * 8, &tx_cfg);
#else
    err = esp_lcd_panel_io_tx_color(io_handle, -1, pData, iLen + pState->panelDef.iLinePadding);
#endif // S3/C5
    if (err != ESP_OK) {
       // Serial.printf("Error %d sending row data\n", (int)err);
    }
//    while (!dma_is_done) {
//        delayMicroseconds(1);
//        vTaskDelay(0);
//    }
} /* bbepWriteRow() */
#endif // !__LINUX__

uint8_t TPS65185PowerGood(void)
{
uint8_t ucTemp[4];

    bbepI2CReadRegister(0x68, 0x0f, ucTemp, 1);
    return ucTemp[0];
}
//
// Initialize the board-specific I/O components (IOInit callback)
// and then initialize the ESP32 LCD API to drive the parallel data bus
//
int bbepIOInit(FASTEPDSTATE *pState)
{
#ifndef __LINUX__
        esp_log_level_set("gpio", ESP_LOG_NONE);
#endif
    int rc = (*(pState->pfnIOInit))(pState);
    if (rc != BBEP_SUCCESS) return rc;
    pState->iPartialPasses = 4; // N.B. The default number of passes for partial updates
    pState->iFullPasses = 5; // the default number of passes for smooth and full updates
#ifndef __LINUX__
#ifdef CONFIG_IDF_TARGET_ESP32C5
    memset(&parlio_tx_config, 0, sizeof(parlio_tx_config));
    parlio_tx_config.clk_src = PARLIO_CLK_SRC_DEFAULT;
    parlio_tx_config.clk_in_gpio_num = (gpio_num_t)-1; // external clock disabled
    parlio_tx_config.output_clk_freq_hz = pState->panelDef.bus_speed;
    parlio_tx_config.data_width = pState->panelDef.bus_width;
    for (int i=0; i<pState->panelDef.bus_width; i++) {
        parlio_tx_config.data_gpio_nums[i] = (gpio_num_t)pState->panelDef.data[i];
    }
    if (pState->panelDef.bus_width < 16) {
        for (int i=8; i<16; i++) {
            parlio_tx_config.data_gpio_nums[i] = (gpio_num_t)-1;
        }
    }
    parlio_tx_config.clk_out_gpio_num = (gpio_num_t)pState->panelDef.ioCL;
    parlio_tx_config.valid_gpio_num = (gpio_num_t)pState->panelDef.ioSPH; // CS
    parlio_tx_config.valid_start_delay = 1; // N.B. this cannot be 0
    parlio_tx_config.valid_stop_delay = 1;
    parlio_tx_config.trans_queue_depth = 2;
    parlio_tx_config.max_transfer_size = 1024; // max 4096 pixels
    parlio_tx_config.dma_burst_size = 32;
    parlio_tx_config.sample_edge = PARLIO_SAMPLE_EDGE_POS;
    parlio_tx_config.flags = {
        .invert_valid_out = true, // The valid signal is high by default, inverted to simulate the chip select signal CS in QPI timing
    };
   // parlio_tx_config.clk_gate_en = 0; // disable
    parlio_tx_handle = NULL;
    ESP_ERROR_CHECK(parlio_new_tx_unit(&parlio_tx_config, &parlio_tx_handle));
    parlio_tx_event_callbacks_t tx_callbacks;
    tx_callbacks.on_trans_done = c5_notify_dma_ready;
    ESP_ERROR_CHECK(parlio_tx_unit_register_event_callbacks(parlio_tx_handle, &tx_callbacks, nullptr));
    ESP_ERROR_CHECK(parlio_tx_unit_enable(parlio_tx_handle));
//    bSlowSPH = 1; // no CS signal in PARLIO
//    u8SPH = (gpio_num_t)pState->panelDef.ioSPH;
    u8CKV = (gpio_num_t)pState->panelDef.ioCKV;
#else
    // Initialize the ESP32 LCD API to drive parallel data at high speed
    // The code forces the use of a D/C pin, so we must assign it to an unused GPIO on each device
    s3_bus_config.dc_gpio_num = (gpio_num_t)pState->panelDef.ioDCDummy;
    s3_bus_config.wr_gpio_num = (gpio_num_t)pState->panelDef.ioCL;
    s3_bus_config.bus_width = pState->panelDef.bus_width;
    for (int i=0; i<pState->panelDef.bus_width; i++) {
        s3_bus_config.data_gpio_nums[i] = (gpio_num_t)pState->panelDef.data[i];
    }
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&s3_bus_config, &i80_bus));
    s3_io_config.pclk_hz = pState->panelDef.bus_speed;
    if (pState->panelDef.flags & BB_PANEL_FLAG_SLOW_SPH) {
        bSlowSPH = 1;
        u8SPH = (gpio_num_t)pState->panelDef.ioSPH;
        u8CKV = (gpio_num_t)pState->panelDef.ioCKV;
        s3_io_config.cs_gpio_num = (gpio_num_t)-1; // disable hardware CS
    } else {
        s3_io_config.cs_gpio_num = (gpio_num_t)pState->panelDef.ioSPH;
    }
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &s3_io_config, &io_handle));
#endif // S3/C5
#endif // !__LINUX__
 
    dma_is_done = true;
    //Serial.println("IO init done");
    return BBEP_SUCCESS;
} /* bbepIOInit() */
//
// For displays with pre-defined configurations (size, speed, flags, gray matrix)
//
int bbepSetDefinedPanel(FASTEPDSTATE *pState, int iPanel)
{
    (void)pState;
    (void)iPanel;
    return BBEP_ERROR_NOT_SUPPORTED;
} /* bbepSetDefinedPanel() */
//
// For board definitions without an associated display (e.g. EPDiy V7 PCB)
// Set the display size and flags
//
int bbepSetPanelSize(FASTEPDSTATE *pState, int width, int height, int flags, int iVCOM) {
    int iPasses;
    uint8_t *pMatrix;

    if (pState->pCurrent) return BBEP_ERROR_BAD_PARAMETER; // panel size is already set

    pState->width = pState->native_width = width;
    pState->height = pState->native_height = height;
    pState->iFlags = flags;
    pState->iVCOM = iVCOM;
#ifdef __LINUX__
    pState->pCurrent = (uint8_t *)malloc((pState->width * pState->height)/2);
    if (pState->iPanelType == BB_PANEL_VIRTUAL) { 
        return BBEP_SUCCESS; // for graphics only
    }   
    pState->pTemp = (uint8_t *)malloc((pState->width * pState->height)/4); // LUT data
    pState->pPrevious = &pState->pCurrent[(width/4) * height]; // comparison with previous buffer (only 1-bpp mode)
#else
    pState->pCurrent = (uint8_t *)heap_caps_aligned_alloc(16, (pState->width * pState->height) / 2, MALLOC_CAP_SPIRAM); // current pixels (allocate for 4-bpp size to work in all modes)
    if (!pState->pCurrent) return BBEP_ERROR_NO_MEMORY;
    if (pState->iPanelType == BB_PANEL_VIRTUAL) {
        return BBEP_SUCCESS; // for graphics only
    }
    pState->pPrevious = &pState->pCurrent[(width/4) * height]; // comparison with previous buffer (only 1-bpp and 2-bpp mode)
    pState->pTemp = (uint8_t *)heap_caps_aligned_alloc(16, (pState->width * pState->height) / 4, MALLOC_CAP_SPIRAM); // LUT data
    if (!pState->pTemp) {
        free(pState->pCurrent);
        return BBEP_ERROR_NO_MEMORY;
    }
#endif // !__LINUX__

    // Allocate memory for each line to transmit
#ifndef __LINUX__
    pState->dma_buf = (uint8_t *)heap_caps_aligned_alloc(16, (pState->width / 2) + pState->panelDef.iLinePadding + 16, MALLOC_CAP_DMA);
#else
    pState->dma_buf = (uint8_t *)malloc((pState->width/2) + pState->panelDef.iLinePadding + 16);
#endif
    iPasses = (pState->panelDef.iMatrixSize / 16); // number of passes
    pGrayLower = (uint8_t *)malloc(256 * iPasses);
    if (!pGrayLower) return BBEP_ERROR_NO_MEMORY;
    pGrayUpper = (uint8_t *)malloc(256 * iPasses);
    if (!pGrayUpper) {
        free(pGrayLower);
        return BBEP_ERROR_NO_MEMORY;
    }
    //Serial.printf("passes = %d\n", iPasses);
    // Prepare grayscale lookup tables
    pMatrix = (uint8_t *)pState->panelDef.pGrayMatrix;
    for (int j = 0; j < iPasses; j++) {
        for (int i = 0; i < 256; i++) {
            if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                pGrayLower[j * 256 + i] = (pMatrix[((i & 0xf)*iPasses)+j] << 2) | (pMatrix[((i >> 4)*iPasses)+j]);
                pGrayUpper[j * 256 + i] = ((pMatrix[((i & 0xf)*iPasses)+j] << 2) | (pMatrix[((i >> 4)*iPasses)+j])) << 4;
            } else {
                pGrayLower[j * 256 + i] = (pMatrix[((i >> 4)*iPasses)+j] << 2) | (pMatrix[((i & 0xf)*iPasses)+j]);
                pGrayUpper[j * 256 + i] = ((pMatrix[((i >> 4)*iPasses)+j] << 2) | (pMatrix[((i & 0xf)*iPasses)+j])) << 4;
            }
        } // for i
    } // for j
    // Create the lookup tables for 1-bit mode. Allow for inverted and mirrored
    for (int i=0; i<256; i++) {
        uint16_t b, w, bw, u16W, u16B, u16BW;
        u16W = u16B = u16BW = 0;
        for (int j=0; j<8; j++) {
            // a 1 means do nothing and 0 means move towards black or white (depending on the LUT)
            if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                if (!(i & (1<<(7-j)))) {
                    w = 2; b = 1;
                    bw = 1;
                } else {
                    w = 3; b = 3;
                    bw = 2;
                }
            } else {
                if (!(i & (0x80>>(7-j)))) {
                    w = 2; b = 1;
                    bw = 1;
                } else {
                    w = 3; b = 3;
                    bw = 2;
                }
            }
            u16W |= (w << (j * 2));
            u16B |= (b << (j * 2));
            u16BW |= (bw << (j * 2));
        } // for j
        LUTW_16[i] = __builtin_bswap16(u16W);
        LUTB_16[i] = __builtin_bswap16(u16B);
        LUTBW_16[i] = __builtin_bswap16(u16BW);
    } // for i
return BBEP_SUCCESS;
} /* setPanelSize() */

//
// Set the individual brightness of the 1 or 2 front lights
//
void bbepSetBrightness(FASTEPDSTATE *pState, uint8_t led1, uint8_t led2)
{
    (void)pState; (void)led1; (void)led2;
} /* bbepSetBrightness() */

//
// Initialize the front light(s) if present
//
void bbepInitLights(FASTEPDSTATE *pState, uint8_t led1, uint8_t led2)
{
    pState->u8LED1 = led1;
    pState->u8LED2 = led2;
} /* bbepInitLights() */
int bbepInitIT8951(FASTEPDSTATE *pState, uint8_t u8MOSI, uint8_t u8MISO, uint8_t u8CLK, uint8_t u8CS, uint8_t u8Busy, uint8_t u8RST, uint8_t u8EN, uint8_t u8ITE_EN)
{
    (void)pState;
    (void)u8MOSI;
    (void)u8MISO;
    (void)u8CLK;
    (void)u8CS;
    (void)u8Busy;
    (void)u8RST;
    (void)u8EN;
    (void)u8ITE_EN;
    return BBEP_ERROR_NOT_SUPPORTED;
}

//
// Initialize the panel based on the constant name
// Each name points to a configuration with info about the PCB and possibly a display
// e.g. BB_PANEL_M5PAPERs3 has both PCB and display info in a single configuration
//
int bbepInitPanel(FASTEPDSTATE *pState, int iPanel, uint32_t u32Speed)
{
    int rc;
    if (iPanel == BB_PANEL_VIRTUAL) {
        pState->iPanelType = iPanel;
        pState->pfnExtIO = NULL;
        pState->pfnEinkPower = NULL;
        pState->pfnRowControl = NULL;
        return BBEP_SUCCESS;
    }
    if (iPanel == BB_PANEL_M5PAPERS3) {
        pState->iPanelType = iPanel;
        pState->mode = BB_MODE_1BPP; // start in 1-bit mode
        pState->iFG = BBEP_BLACK;
        pState->iBG = BBEP_TRANSPARENT;
        pState->iVCOM = -1600; // assume VCOM is -1.6V (typical)
        pState->pfnSetPixel = bbepSetPixel2Clr;
        pState->pfnSetPixelFast = bbepSetPixelFast2Clr;
        pState->pCurrent = NULL; // make sure the memory is allocated
        pState->width = pState->native_width = panelDefs[iPanel].width;
        pState->height = pState->native_height = panelDefs[iPanel].height;
        memcpy(&pState->panelDef, &panelDefs[iPanel], sizeof(BBPANELDEF));
        if (u32Speed) pState->panelDef.bus_speed = u32Speed; // custom speed
        pState->iFlags = pState->panelDef.flags; // copy flags to main class structure
        // Get the 5 callback functions
        pState->pfnEinkPower = panelProcs[iPanel].pfnEinkPower;
        pState->pfnIOInit = panelProcs[iPanel].pfnIOInit;
        pState->pfnIODeInit = panelProcs[iPanel].pfnIODeInit;
        pState->pfnRowControl = panelProcs[iPanel].pfnRowControl;
        pState->pfnExtIO = panelProcs[iPanel].pfnExtIO;
        rc = bbepIOInit(pState);
        if (rc == BBEP_SUCCESS) {
            // allocate memory for the buffers if the paneldef contains the size
            if (pState->width) { // if size is defined
                // VCOM is defined for predefined products
                pState->iVCOM = pState->panelDef.iVCOM;
                rc = bbepSetPanelSize(pState, pState->width, pState->height, pState->iFlags, pState->iVCOM);
                if (rc != BBEP_SUCCESS) return rc; // no memory? stop
            }
        }
        return rc;
    }
    return BBEP_ERROR_NOT_SUPPORTED;
} /* bbepInitPanel() */

//
// Allow the user to set up a custom grayscale matrix
// The number of passes is determined by dividing the table size by 16
//
int bbepSetCustomMatrix(FASTEPDSTATE *pState, const uint8_t *pMatrix, size_t matrix_size)
{
int iPasses;

    if (pState == NULL || pMatrix == NULL) return BBEP_ERROR_BAD_PARAMETER;
    if ((matrix_size & 15) != 0) return BBEP_ERROR_BAD_PARAMETER; // must be divisible by 16

    if (pState->iPanelType == BB_PANEL_VIRTUAL) return BBEP_SUCCESS;

    if (pGrayLower) free(pGrayLower);
    if (pGrayUpper) free(pGrayUpper);
    iPasses = (int)matrix_size / 16; // number of passes
    pState->panelDef.pGrayMatrix = (uint8_t *)pMatrix;
    pState->panelDef.iMatrixSize = matrix_size;
    pGrayLower = (uint8_t *)malloc(256 * iPasses);
    if (!pGrayLower) return BBEP_ERROR_NO_MEMORY;
    pGrayUpper = (uint8_t *)malloc(256 * iPasses);
        if (!pGrayUpper) {
            free(pGrayLower);
            return BBEP_ERROR_NO_MEMORY;
        }
    // Prepare grayscale lookup tables
    for (int j = 0; j < iPasses; j++) {
        for (int i = 0; i < 256; i++) {
            if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                pGrayLower[j * 256 + i] = (pMatrix[((i & 0xf)*iPasses)+j] << 2) | (pMatrix[((i >> 4)*iPasses)+j]);
                pGrayUpper[j * 256 + i] = ((pMatrix[((i & 0xf)*iPasses)+j] << 2) | (pMatrix[((i >> 4)*iPasses)+j])) << 4;
            } else {
                pGrayLower[j * 256 + i] = (pMatrix[((i >> 4)*iPasses)+j] << 2) | (pMatrix[((i & 0xf)*iPasses)+j]);
                pGrayUpper[j * 256 + i] = ((pMatrix[((i >> 4)*iPasses)+j] << 2) | (pMatrix[((i & 0xf)*iPasses)+j])) << 4;
            }
        }
    }
    return BBEP_SUCCESS;
} /* bbepSetCustomMatrix() */
//
// Turn the DC/DC boost circuit on or off
//
int bbepEinkPower(FASTEPDSTATE *pState, int bOn)
{
    if (!pState->pfnEinkPower) return BBEP_ERROR_BAD_PARAMETER;

    return (*(pState->pfnEinkPower))(pState, bOn);
} /* bbepEinkPower() */
//
// Fix a rectangle's coordinates for the current rotation and mirroring flags
// returns FALSE (0) if okay, TRUE (1) if invalid
//
int bbepFixRect(FASTEPDSTATE *pState, BB_RECT *pRect, int *iStartCol, int *iEndCol, int *iStartRow, int *iEndRow)
{
    int i;
        *iStartCol = pRect->x;
        *iEndCol = *iStartCol + pRect->w - 1;
        *iStartRow = pRect->y;
        *iEndRow = *iStartRow + pRect->h - 1;
        if (*iStartCol >= *iEndCol || *iStartRow >= *iEndRow) return 1; // invalid area

        if (*iStartCol < 0) *iStartCol = 0;
        if (*iStartRow < 0) *iStartRow = 0;
        if (*iEndCol >= pState->width) *iEndCol = pState->width - 1;
        if (*iEndRow >= pState->height) *iEndRow = pState->height - 1;
        switch (pState->rotation) { // rotate to native panel direction
            case 0: // nothing to do
                break;
            case 90:
                i = *iStartCol;
                *iStartCol = *iStartRow;
                *iStartRow = pState->width - 1 - *iEndCol;
                *iEndCol = *iEndRow;
                *iEndRow = pState->width - 1 - i; // iStartCol
                break;
            case 270:
                i = *iStartCol;
                *iStartCol = pState->height - 1 - *iEndRow;
                *iEndRow = *iEndCol;
                *iEndCol = pState->height - 1 - *iStartRow;
                *iStartRow = i; // iStartCol
                break;
            case 180:
                i = *iStartCol;
                *iStartCol = pState->width - 1 - *iEndCol;
                *iEndCol = pState->width - 1 - i;
                i = *iStartRow;
                *iStartRow = pState->height - 1 - *iEndRow;
                *iEndRow = pState->height - 1 - i;
                break;
        }
    return 0;
} /* bbepFixRect() */

//
// Clear the display with the given code for the given number of repetitions
//
void bbepClear(FASTEPDSTATE *pState, uint8_t val, uint8_t count, BB_RECT *pRect)
{
    int i, k, dy, iStartCol, iEndCol, iStartRow, iEndRow; // clipping area
    if (val == BB_CLEAR_LIGHTEN) val = 0xaa;
    else if (val == BB_CLEAR_DARKEN) val = 0x55;
    else if (val == BB_CLEAR_NEUTRAL) val = 0x00;
    else val = 0xff; // skip

    if (pRect) {
        if (bbepFixRect(pState, pRect, &iStartCol, &iEndCol, &iStartRow, &iEndRow)) return;
    } else { // use the whole display
        iStartCol = iStartRow = 0;
        iEndCol = pState->native_width - 1;
        iEndRow = pState->native_height - 1;
    }
    // Prepare masked row
    memset(u8Cache, val, pState->native_width / 4);
    i = iStartCol/4;
    memset(u8Cache, 0, i); // whole bytes on left side
    if ((iStartCol & 3) != 0) { // partial byte
        u8Cache[i] = val;
    }
    i = (iEndCol + 3)/4;
    memset(&u8Cache[i], 0, (pState->native_width / 4) - i); // whole bytes on right side
    if ((iEndCol & 3) != 3) { // partial byte
        u8Cache[i-1] = val;
    }
    for (k = 0; k < count; k++) {
        bbepRowControl(pState, ROW_START);
        for (i = 0; i < pState->native_height; i++)
        {
            dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;
            // Send the data
            if (dy < iStartRow || dy > iEndRow) { // skip this row
                memset(pState->dma_buf, 0, pState->native_width / 4);
            } else { // mask the area we want to change
                memcpy(pState->dma_buf, u8Cache, pState->native_width / 4);
            }
            bbepWriteRow(pState, pState->dma_buf, pState->native_width / 4, (i!=0));
        }
        delayMicroseconds(230);
    }
} /* bbepClear() */
//
// Perform a full update with a single color transition (user selected)
// This allows for a faster and less visually disturbing display that is also faster
// than a normal full update
//
int bbepSmoothUpdate(FASTEPDSTATE *pState, bool bKeepOn, uint8_t u8Color)
{
    int i, n, pass;
    
    if (pState->iPanelType == BB_PANEL_VIRTUAL) return BBEP_ERROR_BAD_PARAMETER;

    if (bbepEinkPower(pState, 1) != BBEP_SUCCESS) return BBEP_IO_ERROR;
    bbepClear(pState, (u8Color == BBEP_WHITE) ? BB_CLEAR_LIGHTEN : BB_CLEAR_DARKEN, 5, NULL);
    // The other update methods transition everything from white. In this case, we
    // need to allow the user to update from black also
    if (pState->mode == BB_MODE_1BPP) {
        // Set the color in multiple passes starting from white or black
        // First create the 2-bit codes per pixel for the changes
        uint8_t *s, *d;
        int dy; // destination Y for flipped displays
        uint8_t u8Invert = (u8Color == BBEP_WHITE) ? 0x00 : 0xff;
        uint16_t u16Invert = (u8Color == BBEP_WHITE) ? 0x00 : 0xffff;
        for (i = 0; i < pState->native_height; i++) {
            dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;
            s = &pState->pCurrent[i * (pState->native_width/8)];
            d = &pState->pTemp[dy * (pState->native_width/4)];
            memcpy(&pState->pPrevious[i * (pState->native_width/8)], s, pState->native_width / 8); // previous = current
            if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                s += (pState->native_width/8) - 1;
                for (n = 0; n < (pState->native_width / 4); n += 4) {
                    uint8_t dram2 = *s--;
                    uint8_t dram1 = *s--;
                    *(uint16_t *)&d[n] = u16Invert ^ LUTB_16[dram2 ^ u8Invert];
                    *(uint16_t *)&d[n+2] = u16Invert ^ LUTB_16[dram1 ^ u8Invert];
                }
            } else {
                for (n = 0; n < (pState->native_width / 4); n += 4) {
                    uint8_t dram1 = *s++;
                    uint8_t dram2 = *s++;
                    *(uint16_t *)&d[n+2] = u16Invert ^ LUTB_16[dram2 ^ u8Invert];
                    *(uint16_t *)&d[n] = u16Invert ^ LUTB_16[dram1 ^ u8Invert];
                }
            }
        } // for i
        // Write N passes of the black data to the whole display
        for (pass = 0; pass < pState->iFullPasses; pass++) {
            bbepRowControl(pState, ROW_START);
            for (i = 0; i < pState->native_height; i++) {
                s = &pState->pTemp[i * (pState->native_width / 4)];
                // Send the data for the row
                memcpy(pState->dma_buf, s, pState->native_width/4);
                bbepWriteRow(pState, pState->dma_buf, (pState->native_width / 4), 0);
                bbepRowControl(pState, ROW_STEP);
            }
            delayMicroseconds(230);
        } // for pass
    } else { // must be 4BPP mode
        int dy, iPasses = (pState->panelDef.iMatrixSize / 16); // number of passes
        uint8_t u8Invert = (u8Color = BBEP_WHITE) ? 0x00 : 0xff;
        for (pass = 0; pass < iPasses; pass++) { // number of passes to make 16 unique gray levels
            uint8_t *s, *d = pState->dma_buf;
            uint8_t *pGrayU, *pGrayL;
            pGrayU = pGrayUpper + (pass * 256);
            pGrayL = pGrayLower + (pass * 256);
            bbepRowControl(pState, ROW_START);
            for (i = 0; i < pState->native_height; i++) {
                dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;
                s = &pState->pCurrent[dy * (pState->native_width / 2)];
                if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                    s += (pState->native_width / 2) - 8;
                    for (n = 0; n < (pState->native_width / 4); n += 4) {
                        d[n + 0] = u8Invert ^ (pGrayU[(s[7] ^ u8Invert)] | pGrayL[(s[6] ^ u8Invert)]);
                        d[n + 1] = u8Invert ^ (pGrayU[(s[5] ^ u8Invert)] | pGrayL[(s[4] ^ u8Invert)]);
                        d[n + 2] = u8Invert ^ (pGrayU[(s[3] ^ u8Invert)] | pGrayL[(s[2] ^ u8Invert)]);
                        d[n + 3] = u8Invert ^ (pGrayU[(s[1] ^ u8Invert)] | pGrayL[(s[0] ^ u8Invert)]);
                        s -= 8;
                    } // for n
                } else {
                    for (n = 0; n < (pState->native_width / 4); n += 4) {
                        d[n + 0] = u8Invert ^ (pGrayU[(s[0] ^ u8Invert)] | pGrayL[(s[1] ^ u8Invert)]);
                        d[n + 1] = u8Invert ^ (pGrayU[(s[2] ^ u8Invert)] | pGrayL[(s[3] ^ u8Invert)]);
                        d[n + 2] = u8Invert ^ (pGrayU[(s[4] ^ u8Invert)] | pGrayL[(s[5] ^ u8Invert)]);
                        d[n + 3] = u8Invert ^ (pGrayU[(s[6] ^ u8Invert)] | pGrayL[(s[7] ^ u8Invert)]);
                        s += 8;
                    } // for n
                    //  vTaskDelay(0);
                }
                bbepWriteRow(pState, pState->dma_buf, (pState->native_width / 4), 0);
                bbepRowControl(pState, ROW_STEP);
            } // for i
            delayMicroseconds(230);
        } // for pass
    } // 4bpp
    // Set the drivers inside epaper panel into discharge state.
    bbepClear(pState, BB_CLEAR_NEUTRAL, 1, NULL);
    if (!bKeepOn) bbepEinkPower(pState, 0);
    return BBEP_SUCCESS;
} /* bbepSmoothUpdate() */
//  
// Perform a fast (single flash) update
// This allows for faster 1-bpp updates without an explicit clear step
//  
int bbepFastUpdate(FASTEPDSTATE *pState, bool bKeepOn)
{
    uint8_t *s, *d;
    int i, n, pass, iDMAOff, dy; // destination Y for flipped displays

    if (pState->iPanelType == BB_PANEL_VIRTUAL || pState->mode != BB_MODE_1BPP) return BBEP_ERROR_BAD_PARAMETER;
    if (bbepEinkPower(pState, 1) != BBEP_SUCCESS) return BBEP_IO_ERROR;
    // Set the color in two steps (inverted, non-inverted)
    // First create the 2-bit codes per pixel for pushing both white and black simultaneously
    for (i = 0; i < pState->native_height; i++) {
        dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;
        s = &pState->pCurrent[i * (pState->native_width/8)];
        d = &pState->pTemp[dy * (pState->native_width/4)];
        memcpy(&pState->pPrevious[i * (pState->native_width/8)], s, pState->native_width / 8); // previous = current
        if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
            s += (pState->native_width/8) - 1;
            for (n = 0; n < (pState->native_width / 4); n += 4) {
                uint8_t dram2 = *s--;
                uint8_t dram1 = *s--;
                *(uint16_t *)&d[n] = LUTBW_16[dram2];
                *(uint16_t *)&d[n+2] = LUTBW_16[dram1];
            }
        } else {
            for (n = 0; n < (pState->native_width / 4); n += 4) {
                uint8_t dram1 = *s++;
                uint8_t dram2 = *s++;
                *(uint16_t *)&d[n+2] = LUTBW_16[dram2];
                *(uint16_t *)&d[n] = LUTBW_16[dram1];
            }
        }
    } // for i
    // Write N passes of the push data, but inverted first
    for (pass = 0; pass < pState->iFullPasses; pass++) {
        uint32_t *s32, *d32;
        int iPitch = pState->native_width / 4; // bytes
        iPitch = (iPitch+3)/4; // 32-bit words
        bbepRowControl(pState, ROW_START);
        iDMAOff = 0;
        for (i = 0; i < pState->native_height; i++) {
            d = &pState->dma_buf[iDMAOff];
            d32 = (uint32_t *)d;
            s32 = (uint32_t *)&pState->pTemp[i * (pState->native_width / 4)];
            // Send the data for the row
            for (n = 0; n < iPitch; n++) {
                *d32++ = ~(*s32++); // inverted
            }
            bbepWriteRow(pState, d, (pState->native_width / 4), (i!=0));
            iDMAOff ^= (pState->native_width/4);
        }
        delayMicroseconds(230);
    } // for inverted passes
    // Write N passes of the push data, but non-inverted
    for (pass = 0; pass < pState->iFullPasses; pass++) {
        int iPitch = pState->native_width / 4; // bytes
        bbepRowControl(pState, ROW_START);
        iDMAOff = 0;
        for (i = 0; i < pState->native_height; i++) {
            d = &pState->dma_buf[iDMAOff];
            s = &pState->pTemp[i * (pState->native_width / 4)];
            // Send the data for the row
            memcpy(d, s, iPitch);
            bbepWriteRow(pState, d, (pState->native_width / 4), (i!=0));
            iDMAOff ^= (pState->native_width/4);
        }
        delayMicroseconds(230);
    } // for non-inverted passes
    // Set the drivers inside epaper panel into discharge state.
    bbepClear(pState, BB_CLEAR_NEUTRAL, 1, NULL);
    if (!bKeepOn) bbepEinkPower(pState, 0);
    return BBEP_SUCCESS;
} /* bbepFastUpdate() */
//
// Perform a full (flashing) update given the current mode and pixels
// The time to perform the update can vary greatly depending on the pixel mode
// and selected options
//
int bbepFullUpdate(FASTEPDSTATE *pState, int iClearMode, bool bKeepOn, BB_RECT *pRect)
{
    int i, n, pass, iDMAOff = 0;
    int iStartCol, iStartRow, iEndCol, iEndRow;
    uint8_t u8;
#ifdef SHOW_TIME
    long l = millis();
#endif
    if (pState->iPanelType == BB_PANEL_VIRTUAL) return BBEP_ERROR_BAD_PARAMETER;

    if (bbepEinkPower(pState, 1) != BBEP_SUCCESS) return BBEP_IO_ERROR;
    switch (iClearMode) {
        case CLEAR_SLOW:
            bbepClear(pState, BB_CLEAR_DARKEN, 8, pRect);
            bbepClear(pState, BB_CLEAR_LIGHTEN, 8, pRect);
            bbepClear(pState, BB_CLEAR_DARKEN, 8, pRect);
            bbepClear(pState, BB_CLEAR_LIGHTEN, 8, pRect);
            break;
        case CLEAR_FAST:
            bbepClear(pState, BB_CLEAR_DARKEN, 8, pRect);
            bbepClear(pState, BB_CLEAR_LIGHTEN, 8, pRect);
            break;
        case CLEAR_WHITE:
            if (pState->panelDef.flags & BB_PANEL_FLAG_DARK) {
                bbepClear(pState, BB_CLEAR_LIGHTEN, 13, pRect); // push more white
            } else {
                bbepClear(pState, BB_CLEAR_LIGHTEN, 8, pRect);
            }
            break;
        case CLEAR_BLACK: // probably a mistake
            bbepClear(pState, BB_CLEAR_DARKEN, 8, pRect);
            break;
        case CLEAR_NONE: // nothing to do
        default:
            break;
    }
    bbepClear(pState, BB_CLEAR_NEUTRAL, 1, pRect);
#ifdef SHOW_TIME
    l = millis() - l;
    printf("clear time = %dms\n", (int)l);
    l = millis();
#endif // SHOW_TIME

    if (pRect) {
        if (bbepFixRect(pState, pRect, &iStartCol, &iEndCol, &iStartRow, &iEndRow)) return BBEP_ERROR_BAD_PARAMETER;
        // Prepare masked row
        memset(u8Cache, 0xff, pState->native_width / 4);
        i = iStartCol/4;
        memset(u8Cache, 0, i); // whole bytes on left side
        if ((iStartCol & 3) != 0) { // partial byte
            u8 = 0xff >> ((iStartCol & 3)*2);
            u8Cache[i] = u8;
        }
        i = (iEndCol + 3)/4;
        memset(&u8Cache[i], 0, (pState->native_width / 4) - i); // whole bytes on right side
        if ((iEndCol & 3) != 3) { // partial byte
            u8 = 0xff << ((3-(iEndCol & 3))*2);
            u8Cache[i-1] = u8;
        }
    } else { // use the whole display
        iStartCol = iStartRow = 0;
        iEndCol = pState->native_width - 1;
        iEndRow = pState->native_height - 1;
    }
    if (pState->mode == BB_MODE_2BPP) {
        // Do the update as either push all (push both black and white)
        // or push black if you know you're starting from white
        // N.B. to maintain a balance of charge, be careful with the 'push all' mode
        uint8_t *s, *d;
        uint8_t *u8Gray2BW, *u8Gray2Gray;
        int dy; // destination Y for flipped displays
        // Create fast lookup tables to convert the pixels directly into pushes
        u8Gray2BW = u8Cache;
        u8Gray2Gray = &u8Cache[256];
        memcpy(pState->pPrevious, pState->pCurrent, (pState->native_width/4) * pState->native_height); // previous = current
        for (i=0; i<256; i++) {
            // black/white table
            uint8_t ucB, ucW, uc = i, ucBW = 0, ucGray = 0;
            if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                ucB = 0x40; ucW = 0x80;
            } else {
                ucB = 0x01; ucW = 0x02;
            }
            for (n=0; n<4; n++) { // for each pixel
                if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                    ucBW >>= 2; ucGray >>= 2;
                } else {
                    ucBW <<= 2; ucGray <<= 2;
                }
                switch (uc & 0xc0) {
                    case 0:
                        ucBW |= ucB; // push black
                        ucGray |= ucB; // also push black
                        break;
                    case 0x40:
                        ucBW |= ucB; // push black
                        ucGray |= ucW; // push white
                        break;
                    case 0x80:
                        ucBW |= ucW; // push white
                        ucGray |= ucB; // push black
                        break;
                    case 0xc0:
                        ucBW |= ucW; // push white
                        ucGray |= ucW; // push white
                        break;
                } // switch
                uc <<= 2;
            } // for n
            if (iClearMode != CLEAR_NONE) {
                // Clearing to white means we don't need to push white in the first passes
                ucBW &= 0x55;
            }
            u8Gray2BW[i] = ucBW;
            u8Gray2Gray[i] = ucGray;
        } // for i
        for (pass = 0; pass < 5/*pState->iFullPasses*/; pass++) { // first N passes push to primary color
            bbepRowControl(pState, ROW_START);
            iDMAOff = 0;
            for (i = 0; i < pState->native_height; i++) {
                dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;
                d = &pState->dma_buf[iDMAOff];
                if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                    s = &pState->pCurrent[(dy * (pState->native_width/4)) + pState->native_width/4 - 1];
                    // push white and black pixels simultaneously
                    for (n = 0; n < (pState->native_width / 4); n++) {
                        *d++ = u8Gray2BW[*s--]; // Turn each byte directly into black or white pushes
                    } // for n
                } else {
                    s = &pState->pCurrent[dy * (pState->native_width/4)];
                    // push white and black pixels simultaneously
                    for (n = 0; n < (pState->native_width / 4); n++) {
                        *d++ = u8Gray2BW[*s++]; // Turn each byte directly into black or white pushes
                    } // for n
                }
                // Send the data for the row
                bbepWriteRow(pState, &pState->dma_buf[iDMAOff], (pState->native_width / 4), (i!=0));
                iDMAOff ^= (pState->native_width/4);
            } // for i
            delayMicroseconds(230);
        } // for pass
        for (pass = 0; pass < 1; pass++) { // final passes push to grays
            bbepRowControl(pState, ROW_START);
            iDMAOff = 0;
            for (i = 0; i < pState->native_height; i++) {
                dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;
                d = &pState->dma_buf[iDMAOff];
                if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                    s = &pState->pCurrent[(dy * (pState->native_width/4)) + pState->native_width/4 - 1];
                    // push white and black pixels simultaneously
                    for (n = 0; n < (pState->native_width / 4); n++) {
                        *d++ = u8Gray2Gray[*s--]; // Turn each byte directly into black or white pushes
                    } // for n
                } else {
                    s = &pState->pCurrent[dy * (pState->native_width/4)];
                    // push light gray and dark gray pixels simultaneously
                    for (n = 0; n < (pState->native_width / 4); n++) {
                        *d++ = u8Gray2Gray[*s++]; // Turn each byte directly into light or dark pushes
                    } // for n
                }
                // Send the data for the row
                bbepWriteRow(pState, &pState->dma_buf[iDMAOff], (pState->native_width / 4), (i!=0));
                iDMAOff ^= (pState->native_width/4);
            } // for i
            delayMicroseconds(230);
        } // for pass
    } else if (pState->mode == BB_MODE_1BPP) {
        // Set the color in multiple passes starting from white
        // First create the 2-bit codes per pixel for the black pixels
        uint8_t *s, *d;
        uint16_t *pLUT;
        int dy; // destination Y for flipped displays
        if (iClearMode == CLEAR_NONE) {
            pLUT = LUTBW_16; // expert user - pushing white and black simultaneously
        } else {
            pLUT = LUTB_16; // push black only since we just cleared to white
        }
        for (i = 0; i < pState->native_height; i++) {
            dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;
            if (i >= iStartRow && i <= iEndRow) {
                s = &pState->pCurrent[i * (pState->native_width/8)];
                d = &pState->pTemp[dy * (pState->native_width/4)];
                memcpy(&pState->pPrevious[i * (pState->native_width/8)], s, pState->native_width / 8); // previous = current
                if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                    s += (pState->native_width/8) - 1;
                    for (n = 0; n < (pState->native_width / 4); n += 4) {
                        uint8_t dram2 = *s--;
                        uint8_t dram1 = *s--;
                        *(uint16_t *)&d[n] = pLUT[dram2];
                        *(uint16_t *)&d[n+2] = pLUT[dram1];
                    }
                } else {
                    for (n = 0; n < (pState->native_width / 4); n += 4) {
                        uint8_t dram1 = *s++;
                        uint8_t dram2 = *s++;
                        *(uint16_t *)&d[n+2] = pLUT[dram2];
                        *(uint16_t *)&d[n] = pLUT[dram1];
                    }
                }
                if (iStartCol > 0 || iEndCol < pState->native_width-1) { // There is a region rectangle defined, clip the output to it
                    uint32_t *src, *dst;
                    src = (uint32_t *)u8Cache;
                    dst = (uint32_t *)d;
                    for (n=0; n<pState->native_width/16; n++) { // mask off non-changing pixels to 0s
                        dst[n] &= src[n];
                    }
                }
            } else { // row is not in update area
                d = &pState->pTemp[dy * (pState->native_width/4)];
                memset(d, 0, pState->native_width/4); // skip all these pixels
            }
            //vTaskDelay(0);
        } // for i
        // Write N passes of the black (and possibly white) data to the whole display
        for (pass = 0; pass < pState->iFullPasses; pass++) {
            bbepRowControl(pState, ROW_START);
            iDMAOff = 0;
            for (i = 0; i < pState->native_height; i++) {
                d = &pState->dma_buf[iDMAOff];
//                s = &pState->pCurrent[i * (pState->native_width/8)];
//                s3_onebit_black(s, d, pState->native_width);
                s = &pState->pTemp[i * (pState->native_width / 4)];
                // Send the data for the row
                memcpy(d, s, pState->native_width/4);
                bbepWriteRow(pState, d, (pState->native_width / 4), (i!=0));
                iDMAOff ^= (pState->native_width/4);
            }
            delayMicroseconds(230);
        } // for pass
    } else { // must be 4BPP mode
        int dy, iPasses = (pState->panelDef.iMatrixSize / 16); // number of passes
        for (pass = 0; pass < iPasses; pass++) { // number of passes to make 16 unique gray levels
            uint8_t *s, *d;
            uint8_t *pGrayU, *pGrayL;
            pGrayU = pGrayUpper + (pass * 256);
            pGrayL = pGrayLower + (pass * 256);
            bbepRowControl(pState, ROW_START);
            iDMAOff = 0;
            for (i = 0; i < pState->native_height; i++) {
                d = &pState->dma_buf[iDMAOff];
                dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;
                if (dy >= iStartRow && dy <= iEndRow) { // within the clip rectangle
                    s = &pState->pCurrent[dy * (pState->native_width / 2)];
                    if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                        s += (pState->native_width / 2) - 8;
                        for (n = 0; n < (pState->native_width / 4); n += 4) {
                            d[n + 0] = (pGrayU[s[7]] | pGrayL[s[6]]);
                            d[n + 1] = (pGrayU[s[5]] | pGrayL[s[4]]);
                            d[n + 2] = (pGrayU[s[3]] | pGrayL[s[2]]);
                            d[n + 3] = (pGrayU[s[1]] | pGrayL[s[0]]);
                            s -= 8;
                        } // for n
                    } else {
                        for (n = 0; n < (pState->native_width / 4); n += 4) {
                            d[n + 0] = (pGrayU[s[0]] | pGrayL[s[1]]);
                            d[n + 1] = (pGrayU[s[2]] | pGrayL[s[3]]);
                            d[n + 2] = (pGrayU[s[4]] | pGrayL[s[5]]);
                            d[n + 3] = (pGrayU[s[6]] | pGrayL[s[7]]);
                            s += 8;
                        } // for n
                      //  vTaskDelay(0);
                    }
                    if (iStartCol > 0 || iEndCol < pState->native_width-1) { // There is a region rectangle defined, clip the output to it
                        uint32_t *src, *dst;
                        src = (uint32_t *)u8Cache;
                        dst = (uint32_t *)pState->dma_buf;
                        for (n=0; n<pState->native_width/16; n++) { // mask off non-changing pixels to 0s
                            dst[n] &= src[n];
                        }
                    }
                } else { // outside the clip rectangle
                    memset(d, 0, pState->native_width/4);
                }
                bbepWriteRow(pState, d, (pState->native_width / 4), (i!=0));
                iDMAOff ^= (pState->native_width / 4); // toggle offset
                //bbepRowControl(pState, ROW_STEP);
            } // for i
            delayMicroseconds(230);
        } // for pass
    } // 4bpp
    // Set the drivers inside epaper panel into discharge state.
    bbepClear(pState, BB_CLEAR_NEUTRAL, 1, pRect);
    if (!bKeepOn) bbepEinkPower(pState, 0);
    
#ifdef SHOW_TIME
    l = millis() - l;
    printf("fullUpdate time: %dms\n", (int)l);
#endif // SHOW_TIME
    // Mark this as able to do a partialUpdate() to the same bit mode
    pState->prev_mode = pState->mode;
    return BBEP_SUCCESS;
} /* bbepFullUpdate() */
//
// Convert the previous image buffer contents to be
// the same bit depth as the current mode (1 or 2-bpp) so that
// it can do a differential (partial/non-flickering) update
//
void bbepConvertPrevBuffer(FASTEPDSTATE *pState)
{
    uint8_t *s, *d, *c;
    int i, n, x, y, iSrcPitch = 0, iDestPitch;
    
    switch (pState->prev_mode) {
        case BB_MODE_1BPP:
            iSrcPitch = pState->native_width / 8;
            break;
        case BB_MODE_2BPP:
            iSrcPitch = pState->native_width / 4;
            break;
        case BB_MODE_4BPP:
            iSrcPitch = pState->native_width / 2;
            break;
        default:
            iSrcPitch = pState->native_width / 8;
            break;
    }
    if (pState->mode == BB_MODE_1BPP) { // convert to 1-bpp
        iDestPitch = pState->native_width / 8;
        for (y=0; y<pState->native_height; y++) {
            s = &pState->pPrevious[iSrcPitch * y];
            d = &pState->pTemp[iDestPitch * y];
            c = &pState->pCurrent[iDestPitch * y];
            if (pState->prev_mode == BB_MODE_2BPP) {
                for (x=0; x<pState->native_width; x+=8) { // work 8 pixels at a time
                    uint8_t ucSrc, ucCurr, ucDest = 0;
                    ucCurr = *c++; // get current pixels in case we need to make a decision
                    for (n=0; n<2; n++) { // work on 2 sets of 4 pixels
                        ucSrc = *s++; // get 4 source pixels
                        for (i=0; i<4; i++) {
                            ucDest <<= 1;
                            switch (ucSrc & 0xc0) {
                                case 0xc0: // white - easy
                                    ucDest |= 1; // compared pixel will be white
                                case 0x00: // black - easy
                                    break;
                                case 0x80: // light or dark gray - difficult, check current pixel
                                case 0x40:
                                    ucDest |= ((ucCurr >> 7) ^ 1); // opposite of current color
                                    break;
                            }
                            ucCurr <<= 1;
                            ucSrc <<= 2;
                        } // for i
                    } // for n
                    *d++ = ucDest;
                } // for x
            } else { // 4bpp => 1bpp
                for (x=0; x<pState->native_width; x+=8) { // work 8 pixels at a time
                    uint8_t ucSrc, ucCurr, ucDest = 0;
                    ucCurr = *c++; // get current pixels in case we need to make a decision
                    for (n=0; n<4; n++) { // work on 4 sets of 2 pixels
                        ucSrc = *s++; // get 2 source pixels
                        for (i=0; i<2; i++) {
                            ucDest <<= 1;
                            switch (ucSrc & 0xf0) {
                                case 0xf0: // white - easy
                                    ucDest |= 1; // compared pixel will be white
                                case 0x00: // black - easy
                                    break;
                                default: // middle grays - difficult, check current pixel
                                    ucDest |= ((ucCurr >> 7) ^ 1); // opposite of current color
                                    break;
                            }
                            ucCurr <<= 1;
                            ucSrc <<= 4;
                        } // for i
                    } // for n
                    *d++ = ucDest;
                } // for x
            } // previous pixels are 4-bpp
        } // for y
    } else { // convert to 2-bpp
        iDestPitch = pState->native_width / 4;
        for (y=0; y<pState->native_height; y++) {
            s = &pState->pPrevious[iSrcPitch * y];
            d = &pState->pTemp[iDestPitch * y];
            c = &pState->pCurrent[iDestPitch * y];
            if (pState->prev_mode == BB_MODE_1BPP) {
                const uint8_t u8Conv1To2[16] = {0, 3, 0xc, 0xf, 0x30, 0x33, 0x3c, 0x3f,
                                                0xc0, 0xc3, 0xcc, 0xcf, 0xf0, 0xf3, 0xfc, 0xff}; // convert each nibble
                // Simple direct convertsion: 0->00, 1->11
                for (x=0; x<pState->native_width; x += 8) { // work 8 pixels at a time
                    uint8_t ucSrc = *s++; // get 8 source pixels
                    *d++ = u8Conv1To2[ucSrc>>4];
                    *d++ = u8Conv1To2[ucSrc & 0xf];
                } // for x
            } else { // 4bpp => 2bpp
                for (x=0; x<pState->native_width; x += 4) { // work 4 pixels at a time
                    uint8_t ucSrc, ucDest;
                    ucSrc = *s++; // get 2 source pixels
                    ucDest = (ucSrc & 0xc0); // left pixel top 2 bits
                    ucDest |= ((ucSrc >> 2) << 4); // right pixel top 2 bits
                    ucSrc = *s++; // another 2 source pixels
                    ucDest |= ((ucSrc >> 6) << 2);
                    ucDest |= (ucSrc >> 2);
                    *d++ = ucDest;
                } // for x
            } // previous pixels are 4-bpp
        } // for y
    }
    // Now we can overwrite the old pixels with the converted ones
    memcpy(pState->pPrevious, pState->pTemp, iDestPitch * pState->native_height);
    pState->prev_mode = pState->mode;
} /* bbepConvertPrevBuffer() */

// Future use
void bbepPrepareDiff(uint8_t *c, uint8_t *p, uint8_t *d, int iWidth)
{
    for (int n = 0; n < iWidth / 16; n++) {
        uint8_t cur, prev, diffw, diffb;
        cur = *c++; prev = *p;
        *p++ = cur; // new->old
        diffw = prev & ~cur;
        diffb = ~prev & cur;
        *(uint16_t *)&d[0] = LUTW_16[diffw] & LUTB_16[diffb];

        cur = *c++; prev = *p;
        *p++ = cur; // new->old
        diffw = prev & ~cur;
        diffb = ~prev & cur;
        *(uint16_t *)&d[2] = LUTW_16[diffw] & LUTB_16[diffb];
        d += 4;
    }
} /* bbepPrepareDiff() */

//
// Non-flickering update in 2-bpp gray mode
//
int bbep2BppPartial(FASTEPDSTATE *pState, bool bKeepOn, int iStartLine, int iEndLine)
{
    uint8_t *s, *pNew, *pOld, *d;
    uint8_t *u8Gray2BW, *u8Gray2Gray;
    int pass, iDMAOff;
    int i, k, n, dy; // destination Y for flipped displays
    // Create fast lookup tables to convert the pixels directly into pushes
    u8Gray2BW = u8Cache;
    u8Gray2Gray = &u8Cache[16];
    // Create a lookup table for the 16 permutations of old + new pixels
    // combine them as new (upper 2 bits) with old (lower 2 bits)
    for (i=0; i<16; i++) {
        // black/white table
        switch (i) {
            case 0x0: // black to black
                u8Gray2BW[i] = 0; // nothing in first pass
                u8Gray2Gray[i] = 0; // nothing in second pass
                break;
            case 0x1: // dark gray to black
                u8Gray2BW[i] = 1; // push black in first pass
                u8Gray2Gray[i] = 0; // nothing in second pass
                break;
            case 0x2: // light gray to black
            case 0x3: // white to black
                u8Gray2BW[i] = 1; // push black in first pass
                u8Gray2Gray[i] = 0; // nothing in second pass
                break;
            case 0x4: // black to dark gray
                u8Gray2BW[i] = 0; // nothing in first pass
                u8Gray2Gray[i] = 2; // push white in second pass
                break;
            case 0x5: // dark gray to dark gray
                u8Gray2BW[i] = 0; // nothing in first pass
                u8Gray2Gray[i] = 0; // nothing in second pass
                break;
            case 0x6: // light gray to dark gray
                u8Gray2BW[i] = 0; // nothing in first pass
                u8Gray2Gray[i] = 1; // push black in second pass
                break;
            case 0x7: // white to dark gray
                u8Gray2BW[i] = 1; // push black in first pass
                u8Gray2Gray[i] = 2; // push white in second pass
                break;
            case 0x8: // black to light gray
                u8Gray2BW[i] = 2; // push white in first pass
                u8Gray2Gray[i] = 1; // push black in second pass
                break;
            case 0x9: // dark gray to light gray
                u8Gray2BW[i] = 0; // nothing in first pass
                u8Gray2Gray[i] = 2; // push white in second pass
                break;
            case 0xa: // light gray to light gray
                u8Gray2BW[i] = 0; // nothing in first pass
                u8Gray2Gray[i] = 0; // nothing in second pass
                break;
            case 0xb: // white to light gray
                u8Gray2BW[i] = 0; // nothing in first pass
                u8Gray2Gray[i] = 1; // push black in second pass
                break;
            case 0xc: // black to white
            case 0xd: // dark gray to white
                u8Gray2BW[i] = 2; // push white in first pass
                u8Gray2Gray[i] = 0; // nothing in second pass
                break;
            case 0xe: // light gray to white
                u8Gray2BW[i] = 2; // push white in first pass
                u8Gray2Gray[i] = 2; // nothing in second pass
                break;
            case 0xf: // white to white
                u8Gray2BW[i] = 0; // nothing in first pass
                u8Gray2Gray[i] = 0; // nothing in second pass
                break;
        } // switch
    } // for i
    for (i = 0; i < pState->native_height; i++) {
        dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;
        d = &pState->pTemp[i * (pState->native_width/4)];
        if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
            pNew = &pState->pCurrent[(dy * (pState->native_width/4)) + pState->native_width/4 - 1];
            pOld = &pState->pPrevious[(dy * (pState->native_width/4)) + pState->native_width/4 - 1];
            // push white and black pixels simultaneously
            for (n = 0; n < (pState->native_width / 4); n++) {
                uint8_t ucNew, ucOld, ucPush = 0;
                ucNew = *pNew--; ucOld = *pOld--;
                for (k = 0; k < 4; k++) { // 4 pixels per byte
                    ucPush <<= 2;
                    // convert new+old pixel into correct color pushes
                    ucPush |= u8Gray2BW[((ucNew << 2) & 0xc) | (ucOld & 3)];
                    ucNew >>= 2; ucOld >>= 2;
                }
                *d++ = ucPush;
            } // for n
        } else {
            pNew = &pState->pCurrent[dy * (pState->native_width/4)];
            pOld = &pState->pPrevious[dy * (pState->native_width/4)];
            // push white and black pixels simultaneously
            for (n = 0; n < (pState->native_width / 4); n++) {
                uint8_t ucNew, ucOld, ucPush = 0;
                ucNew = *pNew++; ucOld = *pOld++;
                for (k = 0; k < 4; k++) { // 4 pixels per byte
                    ucPush >>= 2;
                    // convert new+old pixel into correct color pushes
                    ucPush |= (u8Gray2BW[((ucNew << 2) & 0xc) | (ucOld & 3)]) << 6;
                    ucNew >>= 2; ucOld >>= 2;
                }
                *d++ = ucPush;
            } // for n
        }
    } // for i
    for (pass = 0; pass < 5/*pState->iFullPasses*/; pass++) { // first N passes push to primary color
        bbepRowControl(pState, ROW_START);
        iDMAOff = 0;
        for (i = 0; i < pState->native_height; i++) {
            s = &pState->pTemp[i * (pState->native_width/4)];
            d = &pState->dma_buf[iDMAOff];
            memcpy(d, s, pState->native_width/4);
            // Send the data for the row
            bbepWriteRow(pState, &pState->dma_buf[iDMAOff], (pState->native_width / 4), (i!=0));
            iDMAOff ^= (pState->native_width/4);
        } // for i
        delayMicroseconds(230);
    } // for pass
    for (i = 0; i < pState->native_height; i++) {
        dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;
        d = &pState->pTemp[i * (pState->native_width/4)];
        if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
            pNew = &pState->pCurrent[(dy * (pState->native_width/4)) + pState->native_width/4 - 1];
            pOld = &pState->pPrevious[(dy * (pState->native_width/4)) + pState->native_width/4 - 1];
            // push white and black pixels simultaneously
            for (n = 0; n < (pState->native_width / 4); n++) {
                uint8_t ucNew, ucOld, ucPush = 0;
                ucNew = *pNew--; ucOld = *pOld--;
                for (k = 0; k < 4; k++) { // 4 pixels per byte
                    ucPush <<= 2;
                    // convert new+old pixel into correct color pushes
                    ucPush |= u8Gray2Gray[((ucNew << 2) & 0xc) | (ucOld & 3)];
                    ucNew >>= 2; ucOld >>= 2;
                }
                *d++ = ucPush;
            } // for n
        } else {
            pNew = &pState->pCurrent[dy * (pState->native_width/4)];
            pOld = &pState->pPrevious[dy * (pState->native_width/4)];
            // push white and black pixels simultaneously
            for (n = 0; n < (pState->native_width / 4); n++) {
                uint8_t ucNew, ucOld, ucPush = 0;
                ucNew = *pNew++; ucOld = *pOld++;
                for (k = 0; k < 4; k++) { // 4 pixels per byte
                    ucPush >>= 2;
                    // convert new+old pixel into correct color pushes
                    ucPush |= (u8Gray2Gray[((ucNew << 2) & 0xc) | (ucOld & 3)]) << 6;
                    ucNew >>= 2; ucOld >>= 2;
                }
                *d++ = ucPush;
            } // for n
        }
    } // for i
    for (pass = 0; pass < 1; pass++) { // final passes push to grays
        bbepRowControl(pState, ROW_START);
        iDMAOff = 0;
        for (i = 0; i < pState->native_height; i++) {
            s = &pState->pTemp[i * (pState->native_width/4)];
            d = &pState->dma_buf[iDMAOff];
            memcpy(d, s, pState->native_width/4);
            // Send the data for the row
            bbepWriteRow(pState, &pState->dma_buf[iDMAOff], (pState->native_width / 4), (i!=0));
            iDMAOff ^= (pState->native_width/4);
        } // for i
        delayMicroseconds(230);
    } // for pass
    memcpy(pState->pPrevious, pState->pCurrent, (pState->native_width/4) * pState->native_height); // previous = current
    // This clear to neutral step is necessary; do not remove
    bbepClear(pState, BB_CLEAR_NEUTRAL, 1, NULL);
    if (!bKeepOn) {
        bbepEinkPower(pState, 0);
    }
    return BBEP_SUCCESS;
} /* bbep2BppPartial()*/

int bbepPartialUpdate(FASTEPDSTATE *pState, bool bKeepOn, int iStartLine, int iEndLine)
{
    int i, n, pass, iDMAOff;
#ifdef SHOW_TIME
    long l = millis();
#endif
    if (pState->iPanelType == BB_PANEL_VIRTUAL) return BBEP_ERROR_BAD_PARAMETER;

// Only supported in 1 and 2-bit mode (for now)
    if (pState->mode != BB_MODE_1BPP && pState->mode != BB_MODE_2BPP) return BBEP_ERROR_BAD_PARAMETER;
    if (pState->prev_mode == BB_MODE_NONE) return BBEP_ERROR_BAD_PARAMETER;
                    
    if (pState->prev_mode != pState->mode) { // convert 1/2/4-bpp previous image to make current bit depth 
        bbepConvertPrevBuffer(pState);
    }           
    if (bbepEinkPower(pState, 1) != BBEP_SUCCESS) return BBEP_IO_ERROR;

    if (pState->mode == BB_MODE_2BPP) return bbep2BppPartial(pState, bKeepOn, iStartLine, iEndLine);
    if (iStartLine < 0) iStartLine = 0;
    if (iEndLine >= pState->native_height) iEndLine = pState->native_height-1;
    if (iEndLine < iStartLine) return BBEP_ERROR_BAD_PARAMETER;

    uint8_t *pCur, *pPrev, *d;
    uint8_t diffw, diffb, cur, prev;
    
    for (i = iStartLine; i <= iEndLine; i++) {
        d = &pState->pTemp[i * (pState->native_width/4)]; // LUT temp storage
        pCur = &pState->pCurrent[i * (pState->native_width / 8)];
        pPrev = &pState->pPrevious[i * (pState->native_width / 8)];
        if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
            pCur += (pState->native_width / 8) - 1;
            pPrev += (pState->native_width / 8) - 1;
            for (n = 0; n < pState->native_width / 16; n++) {
                cur = *pCur--; prev = *pPrev;
                *pPrev-- = cur; // new->old
                diffw = prev & ~cur;
                diffb = ~prev & cur;
                *(uint16_t *)&d[0] = LUTW_16[diffw] & LUTB_16[diffb];

                cur = *pCur--; prev = *pPrev;
                *pPrev-- = cur; // new->old
                diffw = prev & ~cur;
                diffb = ~prev & cur;
                *(uint16_t *)&d[2] = LUTW_16[diffw] & LUTB_16[diffb];
                d += 4;
            }
        } else {
            for (n = 0; n < pState->native_width / 16; n++) {
                cur = *pCur++; prev = *pPrev;
                *pPrev++ = cur; // new->old
                diffw = prev & ~cur;
                diffb = ~prev & cur;
                *(uint16_t *)&d[0] = LUTW_16[diffw] & LUTB_16[diffb];

                cur = *pCur++; prev = *pPrev;
                *pPrev++ = cur; // new->old
                diffw = prev & ~cur;
                diffb = ~prev & cur;
                *(uint16_t *)&d[2] = LUTW_16[diffw] & LUTB_16[diffb];
                d += 4;
            } // for n
        }
    }
    if (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) {
        // adjust start/end line to be flipped
        int i;
        iStartLine = pState->native_height - 1 - iStartLine;
        iEndLine = pState->native_height - 1 - iEndLine;
        // now swap them
        i = iStartLine;
        iStartLine = iEndLine;
        iEndLine = i;
    }
    for (pass = 0; pass < pState->iPartialPasses; pass++) { // each pass is about 32ms
        uint8_t *d, *dp = pState->pTemp;
        int iDelta = pState->native_width / 4; // 2 bits per pixel
        int iSkipped = 0;
        if (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) {
            dp = &pState->pTemp[(pState->native_height-1) * iDelta]; // read the memory upside down
            iDelta = -iDelta;
        }
        bbepRowControl(pState, ROW_START);
        iDMAOff = 0;
        for (i = 0; i < pState->native_height; i++) {
            d = &pState->dma_buf[iDMAOff];
            if (i >= iStartLine && i <= iEndLine) {
                // Send the data
                memcpy(d, dp, pState->native_width/4);
                bbepWriteRow(pState, d, (pState->native_width / 4), (i!=0));
                iSkipped = 0;
            } else {
               // write a neutral row
                if (iSkipped == 0) { // new skipped section
                    memset((void *)d, 0, pState->native_width/4);
                }
                bbepWriteRow(pState, d, (pState->native_width / 4), (i!=0));
                iSkipped++;
            }
            dp += iDelta;
            iDMAOff ^= (pState->native_width/4);
        }
    } // for each pass

// This clear to neutral step is necessary; do not remove
    bbepClear(pState, BB_CLEAR_NEUTRAL, 1, NULL);
    if (!bKeepOn) {
        bbepEinkPower(pState, 0);
    }

#ifdef SHOW_TIME
    l = millis() - l;
    printf("partialUpdate time: %dms\n", (int)l);
#endif // SHOW_TIME
    return BBEP_SUCCESS;
} /* bbepPartialUpdate() */
//
// Copy the current pixels to the previous
// This facilitates doing partial updates after the power is lost
//
void bbepBackupPlane(FASTEPDSTATE *pState)
{
    if (pState->mode == BB_MODE_4BPP) return; // not applicable to 4-bpp mode
    int iSize = (pState->native_width/2) * pState->native_height;
    if (!pState->pPrevious || !pState->pCurrent) return;
    memcpy(pState->pPrevious, pState->pCurrent, iSize);
}
#endif // __BB_EP__
