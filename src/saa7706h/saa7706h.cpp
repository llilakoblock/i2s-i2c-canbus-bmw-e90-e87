// Copyright 2024 SAA7706H Attack Project
// SPDX-License-Identifier: MIT

#include "saa7706h/saa7706h.h"
#include "config/config.h"
#include "config/pins.h"

#include "driver/i2c.h"
#include "esp_log.h"

namespace {
const char* kTag = "SAA7706H";
}

namespace saa7706h {

SAA7706H::~SAA7706H() {
    if (initialized_) {
        i2c_driver_delete(pins::kI2cPort);
    }
}

esp_err_t SAA7706H::Init() {
    if (initialized_) {
        return ESP_OK;
    }

    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = pins::kI2cSda;
    conf.scl_io_num = pins::kI2cScl;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = pins::kI2cFreqHz;

    esp_err_t err = i2c_param_config(pins::kI2cPort, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(pins::kI2cPort, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    initialized_ = true;
    ESP_LOGI(kTag, "I2C initialized on SDA=%d, SCL=%d @ %lu Hz",
             pins::kI2cSda, pins::kI2cScl, pins::kI2cFreqHz);

    return ESP_OK;
}

esp_err_t SAA7706H::ReadReg24(uint16_t reg_addr, uint32_t* value) {
    if (!initialized_ || !value) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t addr_h = (reg_addr >> 8) & 0xFF;
    uint8_t addr_l = reg_addr & 0xFF;
    uint8_t data[3] = {0};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Write register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c::kDeviceAddress << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, addr_h, true);
    i2c_master_write_byte(cmd, addr_l, true);

    // Repeated start and read 3 bytes
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c::kDeviceAddress << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 3, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(pins::kI2cPort, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (err == ESP_OK) {
        *value = (data[0] << 16) | (data[1] << 8) | data[2];
    } else {
        ESP_LOGE(kTag, "ReadReg24(0x%04X) failed: %s", reg_addr, esp_err_to_name(err));
    }

    return err;
}

esp_err_t SAA7706H::WriteReg24(uint16_t reg_addr, uint32_t value) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t addr_h = (reg_addr >> 8) & 0xFF;
    uint8_t addr_l = reg_addr & 0xFF;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c::kDeviceAddress << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, addr_h, true);
    i2c_master_write_byte(cmd, addr_l, true);
    i2c_master_write_byte(cmd, (value >> 16) & 0xFF, true);
    i2c_master_write_byte(cmd, (value >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, value & 0xFF, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(pins::kI2cPort, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(kTag, "WriteReg24(0x%04X, 0x%06lX) failed: %s",
                 reg_addr, value, esp_err_to_name(err));
    }

    return err;
}

esp_err_t SAA7706H::ReadReg16(uint16_t reg_addr, uint16_t* value) {
    if (!initialized_ || !value) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t addr_h = (reg_addr >> 8) & 0xFF;
    uint8_t addr_l = reg_addr & 0xFF;
    uint8_t data[2] = {0};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c::kDeviceAddress << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, addr_h, true);
    i2c_master_write_byte(cmd, addr_l, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c::kDeviceAddress << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(pins::kI2cPort, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (err == ESP_OK) {
        *value = (data[0] << 8) | data[1];
    }

    return err;
}

esp_err_t SAA7706H::WriteReg16(uint16_t reg_addr, uint16_t value) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t addr_h = (reg_addr >> 8) & 0xFF;
    uint8_t addr_l = reg_addr & 0xFF;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c::kDeviceAddress << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, addr_h, true);
    i2c_master_write_byte(cmd, addr_l, true);
    i2c_master_write_byte(cmd, (value >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, value & 0xFF, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(pins::kI2cPort, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return err;
}

esp_err_t SAA7706H::ReadSel(uint32_t* value) {
    return ReadReg24(reg::kSel, value);
}

esp_err_t SAA7706H::WriteSel(uint32_t value) {
    return WriteReg24(reg::kSel, value);
}

void SAA7706H::PrintReg(uint16_t reg_addr, uint32_t value, const char* name) {
    ESP_LOGI(kTag, "| 0x%04X | %-25s | 0x%06lX | %02X %02X %02X |",
             reg_addr, name, value,
             (uint8_t)(value >> 16), (uint8_t)(value >> 8), (uint8_t)value);
}

void SAA7706H::PrintAllRegisters() {
    ESP_LOGI(kTag, "=== SAA7706H Register Dump ===");
    ESP_LOGI(kTag, "| Addr   | Name                      | Value    | Bytes    |");
    ESP_LOGI(kTag, "|--------|---------------------------|----------|----------|");

    uint16_t ctrl;
    if (ReadReg16(reg::kCtrl, &ctrl) == ESP_OK) {
        ESP_LOGI(kTag, "| 0x%04X | %-25s | 0x%04X   | %02X %02X    |",
                 reg::kCtrl, "CTRL (DSP Control)", ctrl,
                 (uint8_t)(ctrl >> 8), (uint8_t)ctrl);
    }

    struct RegInfo {
        uint16_t addr;
        const char* name;
    };

    const RegInfo regs[] = {
        {reg::kEvaluation, "Evaluation"},
        {reg::kUnknown1, "Unknown 1"},
        {reg::kUnknown2, "Unknown 2"},
        {reg::kClGen1, "CL_GEN1"},
        {reg::kClGen2, "CL_GEN2"},
        {reg::kClGen3, "CL_GEN3"},
        {reg::kClGen4, "CL_GEN4"},
        {reg::kSel, "SEL (Source Select)"},
        {reg::kIac, "IAC"},
        {reg::kClkSet, "CLK_SET"},
        {reg::kClkCoeff, "CLK_COEFF"},
        {reg::kInputSens, "INPUT_SENS"},
        {reg::kPhoneNavAudio, "PHONE_NAV_AUDIO"},
        {reg::kIoConfDsp2, "IO_CONF_DSP2"},
        {reg::kStatusDsp2, "STATUS_DSP2"},
        {reg::kPcDsp2, "PC_DSP2"},
    };

    for (const auto& r : regs) {
        uint32_t value;
        if (ReadReg24(r.addr, &value) == ESP_OK) {
            PrintReg(r.addr, value, r.name);
        }
    }

    ESP_LOGI(kTag, "==============================");
}

esp_err_t SAA7706H::ApplyLinuxDriverInit() {
    ESP_LOGI(kTag, "Applying Linux driver initialization sequence...");

    esp_err_t err;

    // 1. Reset PLL
    err = WriteReg16(reg::kCtrl, defaults::kCtrlPllInit);
    if (err != ESP_OK) return err;

    // 2. Configure hardware registers
    err = WriteReg24(reg::kEvaluation, defaults::kEvaluation);
    if (err != ESP_OK) return err;

    err = WriteReg24(reg::kClGen1, defaults::kClGen1);
    if (err != ESP_OK) return err;

    err = WriteReg24(reg::kClGen2, defaults::kClGen2);
    if (err != ESP_OK) return err;

    err = WriteReg24(reg::kClGen4, defaults::kClGen4);
    if (err != ESP_OK) return err;

    err = WriteReg24(reg::kSel, defaults::kSel);
    if (err != ESP_OK) return err;

    err = WriteReg24(reg::kIac, defaults::kIac);
    if (err != ESP_OK) return err;

    err = WriteReg24(reg::kClkSet, defaults::kClkSet);
    if (err != ESP_OK) return err;

    err = WriteReg24(reg::kClkCoeff, defaults::kClkCoeff);
    if (err != ESP_OK) return err;

    err = WriteReg24(reg::kInputSens, defaults::kInputSens);
    if (err != ESP_OK) return err;

    err = WriteReg24(reg::kPhoneNavAudio, defaults::kPhoneNavAudio);
    if (err != ESP_OK) return err;

    err = WriteReg24(reg::kIoConfDsp2, defaults::kIoConfDsp2);
    if (err != ESP_OK) return err;

    err = WriteReg24(reg::kStatusDsp2, defaults::kStatusDsp2);
    if (err != ESP_OK) return err;

    err = WriteReg24(reg::kPcDsp2, defaults::kPcDsp2);
    if (err != ESP_OK) return err;

    // 3. Configure DSP2 XRAM pointers
    err = WriteReg24(reg::kIis1Pntr, defaults::kXramPtr);
    if (err != ESP_OK) return err;

    err = WriteReg24(reg::kFdacPntr, defaults::kXramPtr);
    if (err != ESP_OK) return err;

    // 4. Set volume in YRAM
    err = WriteReg24(reg::kPvga, defaults::kPvgaVal);
    if (err != ESP_OK) return err;

    err = WriteReg24(reg::kPvat1, defaults::kPvatVal);
    if (err != ESP_OK) return err;

    err = WriteReg24(reg::kPvat, defaults::kPvatVal);
    if (err != ESP_OK) return err;

    // 5. Release PLL
    err = WriteReg16(reg::kCtrl, defaults::kCtrl);
    if (err != ESP_OK) return err;

    ESP_LOGI(kTag, "Linux driver init complete!");
    return ESP_OK;
}

esp_err_t SAA7706H::SetSource(uint8_t source, uint8_t format) {
    uint32_t sel_value = sel::kEnHostIo | (source & 0x07) | ((format & 0x03) << 4);
    ESP_LOGI(kTag, "Setting source=%d, format=%d -> SEL=0x%06lX", source, format, sel_value);
    return WriteSel(sel_value);
}

// Singleton
SAA7706H& GetSAA7706H() {
    static SAA7706H instance;
    return instance;
}

}  // namespace saa7706h
