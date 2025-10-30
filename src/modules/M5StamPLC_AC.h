/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include <M5GFX.h>
#include <M5Unified.hpp>
#include <utility/PI4IOE5V6408_Class.hpp>
#include <memory>

class M5StamPLC_AC {
public:
    /**
     * @brief Initialize M5StamPLC-AC
     *
     * @return true
     * @return false
     */
    bool begin();

    /**
     * @brief Read Relay state
     *
     * @return true if ON, false if OFF
     */
    bool readRelay();

    /**
     * @brief Write AC Relay state
     *
     * @param state true if ON, false if OFF
     */
    void writeRelay(const bool& state);

    /**
     * @brief Set Status Light
     *
     * @param r
     * @param g
     * @param b
     */
    void setStatusLight(const uint8_t& r, const uint8_t& g, const uint8_t& b);

protected:
    std::unique_ptr<m5::PI4IOE5V6408_Class> _ioe;

    bool is_ioe_valid();
};
