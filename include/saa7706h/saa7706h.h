// Copyright 2024 SAA7706H Attack Project
// SPDX-License-Identifier: MIT
//
// SAA7706H I2C Driver - 24-bit register access

#pragma once

#include <cstdint>
#include "esp_err.h"

namespace saa7706h {

class SAA7706H {
public:
    SAA7706H() = default;
    ~SAA7706H();

    // Initialization
    esp_err_t Init();
    bool IsInitialized() const { return initialized_; }

    // 24-bit register operations (correct for SAA7706H!)
    esp_err_t ReadReg24(uint16_t reg_addr, uint32_t* value);
    esp_err_t WriteReg24(uint16_t reg_addr, uint32_t value);

    // 16-bit register operations (for DSP Control 0x0FFF)
    esp_err_t ReadReg16(uint16_t reg_addr, uint16_t* value);
    esp_err_t WriteReg16(uint16_t reg_addr, uint16_t value);

    // Convenience methods
    esp_err_t ReadSel(uint32_t* value);
    esp_err_t WriteSel(uint32_t value);

    // Print register value
    void PrintReg(uint16_t reg_addr, uint32_t value, const char* name);

    // Read all hardware registers and print them
    void PrintAllRegisters();

    // Apply Linux driver initialization sequence
    esp_err_t ApplyLinuxDriverInit();

    // Set source in SEL register
    esp_err_t SetSource(uint8_t source, uint8_t format = 0);

private:
    bool initialized_ = false;
};

// Singleton access
SAA7706H& GetSAA7706H();

}  // namespace saa7706h
