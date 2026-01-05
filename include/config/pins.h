// Copyright 2024 SAA7706H Attack Project
// SPDX-License-Identifier: MIT
//
// Pin configuration for ESP32

#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2s.h"

namespace saa7706h {
namespace pins {

// =============================================================================
// I2C Configuration (MITM on head unit bus)
// =============================================================================

constexpr gpio_num_t kI2cSda = GPIO_NUM_21;
constexpr gpio_num_t kI2cScl = GPIO_NUM_22;
constexpr i2c_port_t kI2cPort = I2C_NUM_0;
constexpr uint32_t kI2cFreqHz = 400000;  // 400kHz Fast Mode

// =============================================================================
// I2S0 TX Configuration (Test signal to SAA7706H)
// =============================================================================

constexpr gpio_num_t kI2s0Bck = GPIO_NUM_26;   // Bit clock
constexpr gpio_num_t kI2s0Ws = GPIO_NUM_25;    // Word select (LRCK)
constexpr gpio_num_t kI2s0DataOut = GPIO_NUM_27;  // Data out to chip
constexpr i2s_port_t kI2s0Port = I2S_NUM_0;

// =============================================================================
// I2S1 RX Configuration (Detector from SAA7706H output)
// =============================================================================

constexpr gpio_num_t kI2s1Bck = GPIO_NUM_14;   // Bit clock
constexpr gpio_num_t kI2s1Ws = GPIO_NUM_15;    // Word select (LRCK)
constexpr gpio_num_t kI2s1DataIn = GPIO_NUM_32;   // Data in from chip
constexpr i2s_port_t kI2s1Port = I2S_NUM_1;

// =============================================================================
// Audio Configuration
// =============================================================================

constexpr uint32_t kSampleRate = 44100;
constexpr i2s_bits_per_sample_t kBitsPerSample = I2S_BITS_PER_SAMPLE_16BIT;

}  // namespace pins
}  // namespace saa7706h
