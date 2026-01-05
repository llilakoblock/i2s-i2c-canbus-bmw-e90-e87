// Copyright 2024 SAA7706H Attack Project
// SPDX-License-Identifier: MIT

#include "i2s/i2s_loopback.h"
#include "config/config.h"
#include "config/pins.h"

#include "driver/i2s.h"
#include "esp_log.h"
#include <cmath>
#include <cstring>

namespace {
const char* kTag = "I2S";
constexpr size_t kDmaBufferCount = 4;
constexpr size_t kDmaBufferLen = 256;
}

namespace saa7706h {

I2SLoopback::~I2SLoopback() {
    if (tx_running_) StopTx();
    if (rx_running_) StopRx();
    if (initialized_) {
        i2s_driver_uninstall(pins::kI2s0Port);
        i2s_driver_uninstall(pins::kI2s1Port);
    }
}

esp_err_t I2SLoopback::Init() {
    if (initialized_) {
        return ESP_OK;
    }

    esp_err_t err;

    // Configure I2S0 as TX (output to SAA7706H)
    i2s_config_t tx_config = {};
    tx_config.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX);
    tx_config.sample_rate = pins::kSampleRate;
    tx_config.bits_per_sample = pins::kBitsPerSample;
    tx_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    tx_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    tx_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    tx_config.dma_buf_count = kDmaBufferCount;
    tx_config.dma_buf_len = kDmaBufferLen;
    tx_config.use_apll = false;
    tx_config.tx_desc_auto_clear = true;

    err = i2s_driver_install(pins::kI2s0Port, &tx_config, 0, nullptr);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "I2S0 TX driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    i2s_pin_config_t tx_pins = {};
    tx_pins.bck_io_num = pins::kI2s0Bck;
    tx_pins.ws_io_num = pins::kI2s0Ws;
    tx_pins.data_out_num = pins::kI2s0DataOut;
    tx_pins.data_in_num = I2S_PIN_NO_CHANGE;

    err = i2s_set_pin(pins::kI2s0Port, &tx_pins);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "I2S0 TX pin config failed: %s", esp_err_to_name(err));
        return err;
    }

    // Configure I2S1 as RX (input from SAA7706H)
    i2s_config_t rx_config = {};
    rx_config.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX);
    rx_config.sample_rate = pins::kSampleRate;
    rx_config.bits_per_sample = pins::kBitsPerSample;
    rx_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    rx_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    rx_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    rx_config.dma_buf_count = kDmaBufferCount;
    rx_config.dma_buf_len = kDmaBufferLen;
    rx_config.use_apll = false;

    err = i2s_driver_install(pins::kI2s1Port, &rx_config, 0, nullptr);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "I2S1 RX driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    i2s_pin_config_t rx_pins = {};
    rx_pins.bck_io_num = pins::kI2s1Bck;
    rx_pins.ws_io_num = pins::kI2s1Ws;
    rx_pins.data_out_num = I2S_PIN_NO_CHANGE;
    rx_pins.data_in_num = pins::kI2s1DataIn;

    err = i2s_set_pin(pins::kI2s1Port, &rx_pins);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "I2S1 RX pin config failed: %s", esp_err_to_name(err));
        return err;
    }

    initialized_ = true;
    ESP_LOGI(kTag, "I2S initialized: TX on I2S0, RX on I2S1 @ %lu Hz", pins::kSampleRate);

    return ESP_OK;
}

esp_err_t I2SLoopback::StartTx() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    i2s_start(pins::kI2s0Port);
    tx_running_ = true;
    return ESP_OK;
}

esp_err_t I2SLoopback::StopTx() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    i2s_stop(pins::kI2s0Port);
    tx_running_ = false;
    return ESP_OK;
}

esp_err_t I2SLoopback::StartRx() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    i2s_start(pins::kI2s1Port);
    rx_running_ = true;
    return ESP_OK;
}

esp_err_t I2SLoopback::StopRx() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    i2s_stop(pins::kI2s1Port);
    rx_running_ = false;
    return ESP_OK;
}

esp_err_t I2SLoopback::WriteSamples(const int16_t* samples, size_t count) {
    if (!initialized_ || !tx_running_) return ESP_ERR_INVALID_STATE;

    size_t bytes_written = 0;
    return i2s_write(pins::kI2s0Port, samples, count * sizeof(int16_t),
                     &bytes_written, portMAX_DELAY);
}

esp_err_t I2SLoopback::ReadSamples(int16_t* samples, size_t count, size_t* bytes_read) {
    if (!initialized_ || !rx_running_) return ESP_ERR_INVALID_STATE;

    return i2s_read(pins::kI2s1Port, samples, count * sizeof(int16_t),
                    bytes_read, pdMS_TO_TICKS(100));
}

esp_err_t I2SLoopback::WriteWavFile(const uint8_t* wav_data, size_t wav_size) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    // Skip WAV header (44 bytes typically)
    if (wav_size < 44) {
        ESP_LOGE(kTag, "WAV file too small");
        return ESP_ERR_INVALID_SIZE;
    }

    const uint8_t* audio_data = wav_data + 44;
    size_t audio_size = wav_size - 44;

    StartTx();

    size_t bytes_written = 0;
    esp_err_t err = i2s_write(pins::kI2s0Port, audio_data, audio_size,
                               &bytes_written, portMAX_DELAY);

    ESP_LOGI(kTag, "Wrote %zu bytes of WAV audio", bytes_written);
    return err;
}

esp_err_t I2SLoopback::GenerateTestTone(uint32_t frequency_hz, uint32_t duration_ms) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    uint32_t num_samples = (pins::kSampleRate * duration_ms) / 1000;
    int16_t* buffer = new int16_t[num_samples * 2];  // Stereo

    // Generate sine wave
    for (uint32_t i = 0; i < num_samples; i++) {
        float t = static_cast<float>(i) / pins::kSampleRate;
        int16_t sample = static_cast<int16_t>(16000 * sinf(2.0f * M_PI * frequency_hz * t));
        buffer[i * 2] = sample;      // Left
        buffer[i * 2 + 1] = sample;  // Right
    }

    StartTx();

    size_t bytes_written = 0;
    esp_err_t err = i2s_write(pins::kI2s0Port, buffer, num_samples * 4,
                               &bytes_written, portMAX_DELAY);

    delete[] buffer;

    ESP_LOGI(kTag, "Generated %lu ms of %lu Hz tone", duration_ms, frequency_hz);
    return err;
}

esp_err_t I2SLoopback::WriteTestPattern() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    // Create test pattern buffer
    constexpr size_t kPatternSize = 256;
    int16_t buffer[kPatternSize * 2];

    // Fill with recognizable pattern
    for (size_t i = 0; i < kPatternSize; i++) {
        int16_t sample = (i & 1) ? 0x5555 : static_cast<int16_t>(0xAAAA);
        buffer[i * 2] = sample;
        buffer[i * 2 + 1] = sample;
    }

    StartTx();

    size_t bytes_written = 0;
    return i2s_write(pins::kI2s0Port, buffer, sizeof(buffer),
                     &bytes_written, portMAX_DELAY);
}

bool I2SLoopback::CheckSignalPresent(uint32_t threshold) {
    if (!initialized_ || !rx_running_) return false;

    constexpr size_t kSampleCount = 256;
    int16_t buffer[kSampleCount * 2];
    size_t bytes_read = 0;

    esp_err_t err = i2s_read(pins::kI2s1Port, buffer, sizeof(buffer),
                              &bytes_read, pdMS_TO_TICKS(100));

    if (err != ESP_OK || bytes_read == 0) {
        return false;
    }

    // Calculate RMS amplitude
    uint64_t sum_squares = 0;
    size_t sample_count = bytes_read / sizeof(int16_t);

    for (size_t i = 0; i < sample_count; i++) {
        int32_t sample = buffer[i];
        sum_squares += sample * sample;
    }

    uint32_t rms = static_cast<uint32_t>(sqrtf(static_cast<float>(sum_squares) / sample_count));

    ESP_LOGD(kTag, "RX RMS: %lu (threshold: %lu)", rms, threshold);

    return rms > threshold;
}

bool I2SLoopback::CompareWithTestPattern() {
    if (!initialized_ || !rx_running_) return false;

    constexpr size_t kSampleCount = 256;
    int16_t buffer[kSampleCount * 2];
    size_t bytes_read = 0;

    esp_err_t err = i2s_read(pins::kI2s1Port, buffer, sizeof(buffer),
                              &bytes_read, pdMS_TO_TICKS(100));

    if (err != ESP_OK || bytes_read == 0) {
        return false;
    }

    // Check for pattern (simple check - look for alternating values)
    int matches = 0;
    size_t sample_count = bytes_read / sizeof(int16_t);

    for (size_t i = 1; i < sample_count; i++) {
        if ((buffer[i] ^ buffer[i-1]) != 0) {
            matches++;
        }
    }

    float match_ratio = static_cast<float>(matches) / sample_count;
    ESP_LOGD(kTag, "Pattern match ratio: %.2f", match_ratio);

    return match_ratio > 0.3f;  // At least 30% should be changing
}

// Singleton
I2SLoopback& GetI2SLoopback() {
    static I2SLoopback instance;
    return instance;
}

}  // namespace saa7706h
