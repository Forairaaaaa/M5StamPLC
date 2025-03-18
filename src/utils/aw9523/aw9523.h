/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

/*!
 *  @file Adafruit_AW9523.h
 *
 * 	I2C Driver for the Adafruit AW9523 GPIO expander
 *
 * 	This is a library for the Adafruit AW9523 breakout:
 * 	https://www.adafruit.com/products/4886
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#pragma once
#include <M5Unified.h>

// https://cdn-shop.adafruit.com/product-files/4886/AW9523+English+Datasheet.pdf
// https://github.com/adafruit/Adafruit_AW9523
class AW9523_Class : public m5::I2C_Device {
public:
    enum PinMode_t {
        AW_OUTPUT = 0,
        AW_INPUT,
    };

public:
    AW9523_Class(std::uint8_t i2c_addr = 0x59, std::uint32_t freq = 400000, m5::I2C_Class* i2c = &m5::In_I2C)
        : I2C_Device(i2c_addr, freq, i2c)
    {
    }

    bool begin();
    bool reset(void);
    bool openDrainPort0(bool od);

    // All 16 pins at once
    bool outputGPIO(uint16_t pins);
    uint16_t inputGPIO(void);
    bool configureDirection(uint16_t pins);
    // bool configureLEDMode(uint16_t pins);
    bool interruptEnableGPIO(uint16_t pins);

    // Individual pin control
    void pinMode(uint8_t pin, uint8_t mode);
    void digitalWrite(uint8_t pin, bool val);
    bool digitalRead(uint8_t pin);
    // void analogWrite(uint8_t pin, uint8_t val);
    void enableInterrupt(uint8_t pin, bool en);

    inline void disableIrq()
    {
        interruptEnableGPIO(0);
    }
    inline void resetIrq()
    {
        inputGPIO();
    }
};
