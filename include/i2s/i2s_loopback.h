// Copyright 2024 SAA7706H Attack Project
// SPDX-License-Identifier: MIT
//
// I2S Loopback Tester - TX and RX for signal detection

#pragma once

#include <cstdint>
#include <cstddef>
#include "esp_err.h"

namespace saa7706h {

class I2SLoopback {
public:
    I2SLoopback() = default;
    ~I2SLoopback();

    // Initialize both I2S ports
    esp_err_t Init();
    bool IsInitialized() const { return initialized_; }

    // TX operations (I2S0 - to SAA7706H)
    esp_err_t StartTx();
    esp_err_t StopTx();
    esp_err_t WriteSamples(const int16_t* samples, size_t count);

    // Write embedded WAV file
    esp_err_t WriteWavFile(const uint8_t* wav_data, size_t wav_size);

    // Generate test tone
    esp_err_t GenerateTestTone(uint32_t frequency_hz, uint32_t duration_ms);

    // Write test pattern continuously
    esp_err_t WriteTestPattern();

    // RX operations (I2S1 - from SAA7706H)
    esp_err_t StartRx();
    esp_err_t StopRx();
    esp_err_t ReadSamples(int16_t* samples, size_t count, size_t* bytes_read);

    // Check if signal is present
    bool CheckSignalPresent(uint32_t threshold = 100);

    // Compare with test pattern
    bool CompareWithTestPattern();

private:
    bool initialized_ = false;
    bool tx_running_ = false;
    bool rx_running_ = false;
};

// Singleton access
I2SLoopback& GetI2SLoopback();

}  // namespace saa7706h
