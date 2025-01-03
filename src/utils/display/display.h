/*
 *SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */
#pragma once
#include <M5Unified.hpp>
#include <M5GFX.h>
#include <lgfx/v1/panel/Panel_ST7789.hpp>
#include "../../pin_config.h"

class LGFX_StamPLC : public lgfx::LGFX_Device {
    lgfx::Panel_ST7789 _panel_instance;
    lgfx::Bus_SPI _bus_instance;

public:
    LGFX_StamPLC(void)
    {
        {
            auto cfg       = _bus_instance.config();
            cfg.pin_mosi   = STAMPLC_PIN_LCD_MOSI;
            cfg.pin_miso   = STAMPLC_PIN_LCD_MISO;
            cfg.pin_sclk   = STAMPLC_PIN_LCD_SCLK;
            cfg.pin_dc     = STAMPLC_PIN_LCD_DC;
            cfg.freq_write = 40000000;
            cfg.spi_host   = SPI2_HOST;
            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }
        {
            auto cfg            = _panel_instance.config();
            cfg.invert          = true;
            cfg.pin_cs          = STAMPLC_PIN_LCD_CS;
            cfg.pin_rst         = STAMPLC_PIN_LCD_RST;
            cfg.pin_busy        = STAMPLC_PIN_LCD_BUSY;
            cfg.panel_width     = 135;
            cfg.panel_height    = 240;
            cfg.offset_x        = 52;
            cfg.offset_y        = 40;
            cfg.bus_shared      = true;
            cfg.offset_rotation = 1;
            _panel_instance.config(cfg);
        }
        setPanel(&_panel_instance);
    }
};