// Copyright 2024 SAA7706H Attack Project
// SPDX-License-Identifier: MIT

#include "attack/register_attack.h"
#include "saa7706h/saa7706h.h"
#include "i2s/i2s_loopback.h"
#include "config/config.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// External WAV file embedded in firmware
extern const uint8_t wav_start[] asm("_binary_no_wav_start");
extern const uint8_t wav_end[] asm("_binary_no_wav_end");

namespace {
const char* kTag = "ATTACK";
}

namespace saa7706h {

esp_err_t RegisterAttack::Init() {
    if (initialized_) {
        return ESP_OK;
    }

    ESP_LOGI(kTag, "Initializing Register Attack Module...");

    // Initialize SAA7706H I2C driver
    esp_err_t err = GetSAA7706H().Init();
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "SAA7706H init failed");
        return err;
    }

    // Initialize I2S loopback
    err = GetI2SLoopback().Init();
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "I2S loopback init failed");
        return err;
    }

    initialized_ = true;
    result_count_ = 0;

    ESP_LOGI(kTag, "Attack module initialized!");
    return ESP_OK;
}

void RegisterAttack::CaptureBaseline() {
    ESP_LOGI(kTag, "=== BASELINE CAPTURE ===");
    GetSAA7706H().PrintAllRegisters();
}

TestResult RegisterAttack::TestSelValue(uint32_t sel_value) {
    TestResult result = {sel_value, false, 0};

    SAA7706H& chip = GetSAA7706H();
    I2SLoopback& i2s = GetI2SLoopback();

    // Write SEL value
    esp_err_t err = chip.WriteSel(sel_value);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "Failed to write SEL=0x%06lX", sel_value);
        return result;
    }

    // Wait for chip to settle
    vTaskDelay(pdMS_TO_TICKS(test::kDelayAfterWriteMs));

    // Write test pattern
    i2s.WriteTestPattern();

    // Wait a bit
    vTaskDelay(pdMS_TO_TICKS(50));

    // Check for signal
    result.signal_detected = i2s.CheckSignalPresent(100);

    if (result.signal_detected) {
        ESP_LOGW(kTag, "*** SIGNAL DETECTED! SEL=0x%06lX ***", sel_value);
    } else {
        ESP_LOGD(kTag, "No signal: SEL=0x%06lX", sel_value);
    }

    return result;
}

void RegisterAttack::AddResult(const TestResult& result) {
    if (result_count_ < kMaxResults) {
        results_[result_count_++] = result;
    }
}

void RegisterAttack::TestAllSources() {
    ESP_LOGI(kTag, "=== TESTING ALL SOURCES ===");

    SAA7706H& chip = GetSAA7706H();
    I2SLoopback& i2s = GetI2SLoopback();

    // Apply Linux driver init first
    chip.ApplyLinuxDriverInit();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Start I2S TX and RX
    i2s.StartTx();
    i2s.StartRx();

    // Test all source combinations (bits 0-2)
    for (uint8_t src = 0; src <= 7; src++) {
        // Test all format combinations (bits 4-5)
        for (uint8_t fmt = 0; fmt <= 3; fmt++) {
            uint32_t sel = sel::kEnHostIo | src | (fmt << 4);

            TestResult result = TestSelValue(sel);
            AddResult(result);

            // Also test without EN_HOST_IO
            sel = src | (fmt << 4);
            result = TestSelValue(sel);
            AddResult(result);
        }
    }

    i2s.StopTx();
    i2s.StopRx();

    PrintResults();
}

void RegisterAttack::TestFormatsForSource(uint8_t source) {
    ESP_LOGI(kTag, "=== TESTING FORMATS FOR SOURCE %d ===", source);

    SAA7706H& chip = GetSAA7706H();
    I2SLoopback& i2s = GetI2SLoopback();

    chip.ApplyLinuxDriverInit();
    vTaskDelay(pdMS_TO_TICKS(500));

    i2s.StartTx();
    i2s.StartRx();

    for (uint8_t fmt = 0; fmt <= 3; fmt++) {
        // With EN_HOST_IO
        uint32_t sel = sel::kEnHostIo | source | (fmt << 4);
        TestResult result = TestSelValue(sel);
        AddResult(result);

        // Also test SRCB
        sel = sel::kEnHostIo | (source << 6) | (fmt << 9);
        result = TestSelValue(sel);
        AddResult(result);
    }

    i2s.StopTx();
    i2s.StopRx();

    PrintResults();
}

void RegisterAttack::TestI2S1() {
    ESP_LOGI(kTag, "=== TESTING I2S1 SOURCE ===");
    TestFormatsForSource(sel::kSrcI2s1);
}

void RegisterAttack::TestI2S2() {
    ESP_LOGI(kTag, "=== TESTING I2S2 SOURCE ===");
    TestFormatsForSource(sel::kSrcI2s2);
}

void RegisterAttack::TestSPDIF() {
    ESP_LOGI(kTag, "=== TESTING SPDIF SOURCE ===");

    SAA7706H& chip = GetSAA7706H();
    I2SLoopback& i2s = GetI2SLoopback();

    chip.ApplyLinuxDriverInit();
    vTaskDelay(pdMS_TO_TICKS(500));

    i2s.StartTx();
    i2s.StartRx();

    // SPDIF without SPDIF2 bit
    uint32_t sel = sel::kEnHostIo | sel::kSrcSpdif;
    TestResult result = TestSelValue(sel);
    AddResult(result);

    // SPDIF with SPDIF2 bit
    sel = sel::kEnHostIo | sel::kSrcSpdif | sel::kSpdif2;
    result = TestSelValue(sel);
    AddResult(result);

    i2s.StopTx();
    i2s.StopRx();

    PrintResults();
}

void RegisterAttack::RunSelAttack() {
    ESP_LOGI(kTag, "");
    ESP_LOGI(kTag, "╔══════════════════════════════════════════════════════════╗");
    ESP_LOGI(kTag, "║           SAA7706H REGISTER ATTACK - SEL SWEEP           ║");
    ESP_LOGI(kTag, "╚══════════════════════════════════════════════════════════╝");
    ESP_LOGI(kTag, "");

    result_count_ = 0;

    // Capture baseline first
    CaptureBaseline();

    // Test all combinations
    TestAllSources();
}

void RegisterAttack::RunI2SAttack() {
    ESP_LOGI(kTag, "");
    ESP_LOGI(kTag, "╔══════════════════════════════════════════════════════════╗");
    ESP_LOGI(kTag, "║      I2S ATTACK - 44.1kHz 16-bit Stereo WAV              ║");
    ESP_LOGI(kTag, "╚══════════════════════════════════════════════════════════╝");
    ESP_LOGI(kTag, "");
    ESP_LOGI(kTag, "WAV Format: 44100 Hz, 16-bit, 2ch stereo");
    ESP_LOGI(kTag, "ESP32 I2S: Master, I2S Standard (Philips)");
    ESP_LOGI(kTag, "");

    SAA7706H& chip = GetSAA7706H();
    I2SLoopback& i2s = GetI2SLoopback();

    result_count_ = 0;

    // Apply Linux driver init first
    ESP_LOGI(kTag, "Step 1: Applying Linux driver init sequence...");
    chip.ApplyLinuxDriverInit();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Show current registers
    ESP_LOGI(kTag, "Step 2: Current register state:");
    chip.PrintAllRegisters();

    // Start I2S
    ESP_LOGI(kTag, "Step 3: Starting I2S TX/RX...");
    i2s.StartTx();
    i2s.StartRx();

    // Test pre-calculated SEL values
    ESP_LOGI(kTag, "Step 4: Testing %zu SEL configurations...", sel::kSelTestValuesCount);
    ESP_LOGI(kTag, "");

    for (size_t i = 0; i < sel::kSelTestValuesCount; i++) {
        uint32_t sel_val = sel::kSelTestValues[i];

        ESP_LOGI(kTag, "[%zu/%zu] Testing SEL=0x%06lX",
                 i + 1, sel::kSelTestValuesCount, sel_val);

        // Describe what this value means
        uint8_t src = sel_val & 0x07;
        uint8_t fmt = (sel_val >> 4) & 0x03;
        bool host_io = (sel_val & sel::kEnHostIo) != 0;

        const char* src_name[] = {"Internal", "I2S1", "I2S2", "SPDIF", "HostIO", "?", "?", "?"};
        const char* fmt_name[] = {"LeftJust", "I2S_Std", "RightJust", "?"};

        ESP_LOGI(kTag, "        Source=%s, Format=%s, HostIO=%s",
                 src_name[src], fmt_name[fmt], host_io ? "ON" : "OFF");

        TestResult result = TestSelValue(sel_val);
        AddResult(result);

        if (result.signal_detected) {
            ESP_LOGW(kTag, "        >>> SIGNAL DETECTED! <<<");
        }

        ESP_LOGI(kTag, "");
    }

    i2s.StopTx();
    i2s.StopRx();

    PrintResults();
}

void RegisterAttack::RunQuickI2STest() {
    ESP_LOGI(kTag, "=== QUICK I2S TEST (I2S1 + I2S Standard) ===");
    ESP_LOGI(kTag, "SEL = 0x%06lX", sel::kSelI2s1Std);

    SAA7706H& chip = GetSAA7706H();
    I2SLoopback& i2s = GetI2SLoopback();

    // Apply init
    chip.ApplyLinuxDriverInit();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Set optimal SEL: I2S1 + I2S Standard format
    esp_err_t err = chip.WriteSel(sel::kSelI2s1Std);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "Failed to write SEL");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Verify
    uint32_t read_val = 0;
    chip.ReadSel(&read_val);
    ESP_LOGI(kTag, "SEL readback: 0x%06lX", read_val);

    // Start I2S and play WAV
    size_t wav_size = wav_end - wav_start;
    ESP_LOGI(kTag, "Playing WAV file (%zu bytes)...", wav_size);

    i2s.WriteWavFile(wav_start, wav_size);

    ESP_LOGI(kTag, "Quick test complete - check analog output!");
}

void RegisterAttack::PrintResults() {
    ESP_LOGI(kTag, "");
    ESP_LOGI(kTag, "=== ATTACK RESULTS ===");
    ESP_LOGI(kTag, "Tested %zu configurations", result_count_);

    size_t success_count = 0;
    for (size_t i = 0; i < result_count_; i++) {
        if (results_[i].signal_detected) {
            ESP_LOGW(kTag, "  SUCCESS: SEL=0x%06lX", results_[i].sel_value);
            success_count++;
        }
    }

    if (success_count == 0) {
        ESP_LOGI(kTag, "  No working configurations found");
    } else {
        ESP_LOGW(kTag, "  Found %zu working configurations!", success_count);
    }

    ESP_LOGI(kTag, "======================");
}

void RegisterAttack::RunWavTest() {
    ESP_LOGI(kTag, "=== WAV FILE TEST ===");

    SAA7706H& chip = GetSAA7706H();
    I2SLoopback& i2s = GetI2SLoopback();

    // Apply init
    chip.ApplyLinuxDriverInit();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Set to I2S1 source
    chip.SetSource(sel::kSrcI2s1, sel::kFmtI2sStandard);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Play WAV file
    size_t wav_size = wav_end - wav_start;
    ESP_LOGI(kTag, "Playing WAV file (%zu bytes)...", wav_size);

    i2s.WriteWavFile(wav_start, wav_size);

    ESP_LOGI(kTag, "WAV playback complete");
}

void RegisterAttack::RunToneTest(uint32_t frequency_hz) {
    ESP_LOGI(kTag, "=== TONE TEST @ %lu Hz ===", frequency_hz);

    SAA7706H& chip = GetSAA7706H();
    I2SLoopback& i2s = GetI2SLoopback();

    chip.ApplyLinuxDriverInit();
    vTaskDelay(pdMS_TO_TICKS(500));

    chip.SetSource(sel::kSrcI2s1, sel::kFmtI2sStandard);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Generate 1 second tone
    i2s.GenerateTestTone(frequency_hz, 1000);

    ESP_LOGI(kTag, "Tone test complete");
}

// Singleton
RegisterAttack& GetRegisterAttack() {
    static RegisterAttack instance;
    return instance;
}

}  // namespace saa7706h
