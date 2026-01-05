// Copyright 2024 SAA7706H Attack Project
// SPDX-License-Identifier: MIT
//
// Main configuration for SAA7706H register attack

#pragma once

#include <cstdint>

namespace saa7706h {

// =============================================================================
// SAA7706H I2C Configuration
// =============================================================================

namespace i2c {

constexpr uint8_t kDeviceAddress = 0x1C;  // 7-bit address
constexpr uint8_t kDeviceAddressWrite = 0x38;  // 8-bit write
constexpr uint8_t kDeviceAddressRead = 0x39;   // 8-bit read

}  // namespace i2c

// =============================================================================
// Register Addresses
// =============================================================================

namespace reg {

// DSP Control
constexpr uint16_t kCtrl = 0x0FFF;

// Hardware Registers (24-bit)
constexpr uint16_t kEvaluation = 0x1FF0;
constexpr uint16_t kUnknown1 = 0x1FF1;
constexpr uint16_t kUnknown2 = 0x1FF2;
constexpr uint16_t kClGen1 = 0x1FF3;
constexpr uint16_t kClGen2 = 0x1FF4;
constexpr uint16_t kClGen3 = 0x1FF5;
constexpr uint16_t kClGen4 = 0x1FF6;
constexpr uint16_t kSel = 0x1FF7;          // Source selector - KEY!
constexpr uint16_t kIac = 0x1FF8;
constexpr uint16_t kClkSet = 0x1FF9;       // Clock settings
constexpr uint16_t kClkCoeff = 0x1FFA;
constexpr uint16_t kInputSens = 0x1FFB;
constexpr uint16_t kPhoneNavAudio = 0x1FFC;  // Audio routing
constexpr uint16_t kIoConfDsp2 = 0x1FFD;   // I/O config
constexpr uint16_t kStatusDsp2 = 0x1FFE;
constexpr uint16_t kPcDsp2 = 0x1FFF;

// DSP2 XRAM
constexpr uint16_t kFdacPntr = 0x11F9;
constexpr uint16_t kIis1Pntr = 0x11FB;

// DSP2 YRAM
constexpr uint16_t kPvga = 0x20CB;
constexpr uint16_t kPvat1 = 0x20CD;
constexpr uint16_t kPvat = 0x20CF;

}  // namespace reg

// =============================================================================
// SEL Register Bit Masks (0x1FF7)
// =============================================================================

namespace sel {

constexpr uint32_t kDsp2SrcaMask = 0x000007;    // Bits 0-2: Source A
constexpr uint32_t kDsp2FmtaMask = 0x000030;    // Bits 4-5: Format A
constexpr uint32_t kDsp2SrcbMask = 0x0001C0;    // Bits 6-8: Source B
constexpr uint32_t kDsp2FmtbMask = 0x000E00;    // Bits 9-11: Format B
constexpr uint32_t kDsp1SrcMask = 0x003000;     // Bits 12-13: DSP1 Source
constexpr uint32_t kDsp1FmtMask = 0x01C000;     // Bits 14-16: DSP1 Format
constexpr uint32_t kSpdif2 = 0x020000;          // Bit 17: SPDIF2
constexpr uint32_t kHostIoFmtMask = 0x1C0000;   // Bits 18-20: Host I/O Format
constexpr uint32_t kEnHostIo = 0x200000;        // Bit 21: Enable Host I/O

// Source values for SRCA/SRCB (bits 0-2)
constexpr uint8_t kSrcInternal = 0;  // FM/AM
constexpr uint8_t kSrcI2s1 = 1;
constexpr uint8_t kSrcI2s2 = 2;
constexpr uint8_t kSrcSpdif = 3;
constexpr uint8_t kSrcHostIo = 4;

// Format values (bits 4-5)
constexpr uint8_t kFmtLeftJustified = 0;
constexpr uint8_t kFmtI2sStandard = 1;
constexpr uint8_t kFmtRightJustified = 2;

// =============================================================================
// Pre-calculated SEL values for I2S attack (44.1kHz, 16-bit, stereo)
// ESP32 sends I2S Standard format, so we need format bits 4-5 = 01
// =============================================================================

// I2S1 source (CD input pins 27-29) + I2S Standard format + EN_HOST_IO
// bits 0-2 = 001 (I2S1), bits 4-5 = 01 (I2S Std), bit 21 = 1
constexpr uint32_t kSelI2s1Std = 0x200011;

// I2S1 source + Left Justified format (fallback)
constexpr uint32_t kSelI2s1Left = 0x200001;

// I2S2 source (co-processor pins 31-32) + I2S Standard format
constexpr uint32_t kSelI2s2Std = 0x200012;

// SPDIF1 source + EN_HOST_IO
constexpr uint32_t kSelSpdif1 = 0x200003;

// SPDIF2 source + SPDIF2 bit + EN_HOST_IO
constexpr uint32_t kSelSpdif2 = 0x220003;

// Test array for sweep attack
constexpr uint32_t kSelTestValues[] = {
    0x200011,  // I2S1 + I2S Std (PRIMARY - should work!)
    0x200001,  // I2S1 + Left Justified
    0x200021,  // I2S1 + Right Justified
    0x200012,  // I2S2 + I2S Std
    0x200002,  // I2S2 + Left Justified
    0x000011,  // I2S1 + I2S Std (no EN_HOST_IO)
    0x000001,  // I2S1 + Left Justified (no EN_HOST_IO)
    0x200080,  // Linux driver default
    0x200081,  // Linux driver + I2S1
    0x200091,  // Linux driver + I2S1 + I2S Std
};
constexpr size_t kSelTestValuesCount = sizeof(kSelTestValues) / sizeof(kSelTestValues[0]);

}  // namespace sel

// =============================================================================
// Default Register Values (from Linux driver)
// =============================================================================

namespace defaults {

constexpr uint32_t kCtrl = 0x0000;
constexpr uint32_t kCtrlPllInit = 0x003E;
constexpr uint32_t kEvaluation = 0x000000;
constexpr uint32_t kClGen1 = 0x040022;
constexpr uint32_t kClGen2 = 0x000001;
constexpr uint32_t kClGen4 = 0x024080;
constexpr uint32_t kSel = 0x200080;
constexpr uint32_t kIac = 0xF4CAED;
constexpr uint32_t kClkSet = 0x124334;
constexpr uint32_t kClkCoeff = 0x004A1A;
constexpr uint32_t kInputSens = 0x0071C7;
constexpr uint32_t kPhoneNavAudio = 0x0E22FF;
constexpr uint32_t kIoConfDsp2 = 0x001FF8;
constexpr uint32_t kStatusDsp2 = 0x080003;
constexpr uint32_t kPcDsp2 = 0x000004;

// XRAM/YRAM
constexpr uint32_t kXramPtr = 0x2000CB;
constexpr uint32_t kPvgaVal = 0x000F80;
constexpr uint32_t kPvatVal = 0x000800;

}  // namespace defaults

// =============================================================================
// Test Configuration
// =============================================================================

namespace test {

constexpr uint32_t kTestPattern = 0xAA55AA55;
constexpr uint32_t kDelayAfterWriteMs = 100;
constexpr uint32_t kI2sBufferSize = 1024;

}  // namespace test

}  // namespace saa7706h
