// Copyright 2024 SAA7706H Attack Project
// SPDX-License-Identifier: MIT
//
// Register Attack Module - Automated testing of SAA7706H registers

#pragma once

#include <cstdint>
#include "esp_err.h"

namespace saa7706h {

// Result of a single register test
struct TestResult {
    uint32_t sel_value;
    bool signal_detected;
    uint32_t rms_level;
};

class RegisterAttack {
public:
    RegisterAttack() = default;

    // Initialize attack module (init I2C and I2S)
    esp_err_t Init();

    // Capture baseline registers
    void CaptureBaseline();

    // Run full automated attack on SEL register
    void RunSelAttack();

    // Test specific SEL value
    TestResult TestSelValue(uint32_t sel_value);

    // Test all source combinations (0-7)
    void TestAllSources();

    // Test all format combinations for a source
    void TestFormatsForSource(uint8_t source);

    // Test with I2S1 source
    void TestI2S1();

    // Test with I2S2 source
    void TestI2S2();

    // Test with SPDIF source
    void TestSPDIF();

    // Print results
    void PrintResults();

    // Run continuous test with WAV file
    void RunWavTest();

    // Run continuous test with tone
    void RunToneTest(uint32_t frequency_hz);

    // NEW: Targeted I2S attack with correct format parameters
    void RunI2SAttack();

    // NEW: Quick test with optimal SEL value
    void RunQuickI2STest();

private:
    bool initialized_ = false;
    static constexpr size_t kMaxResults = 64;
    TestResult results_[kMaxResults];
    size_t result_count_ = 0;

    void AddResult(const TestResult& result);
};

// Singleton access
RegisterAttack& GetRegisterAttack();

}  // namespace saa7706h
