# SAA7706H - Car Radio Digital Signal Processor (DSP)

## Overview

SAA7706H is an audio DSP from NXP/Philips for car radios.
Used in BMW CD73, CD53 and other head units.

- **I2C Address**: 0x1C (7-bit) / 0x38-0x39 (8-bit R/W)
- **I2C Mode**: Standard and Fast mode (400kHz)
- **Data Format**: 24-bit registers (3 bytes)

## Datasheet

Official datasheet is located in this folder: [SAA7706H_datasheet.pdf](./SAA7706H_datasheet.pdf)

---

## Memory Map

```
Address       | Size          | Description
--------------|---------------|---------------------------
0x0000-0x017F | 384 × 18 bits | XRAM (DSP1)
0x0800-0x097F | 384 × 12 bits | YRAM (DSP1)
0x0FFF        | 1 × 16 bits   | DSP CONTROL
0x1000-0x127F | 640 × 24 bits | XRAM (DSP2)
0x1FF0-0x1FFF | 16 × 24 bits  | Hardware Registers
0x2000-0x21FF | 512 × 12 bits | YRAM (DSP2)
```

---

## Hardware Registers (0x1FF0 - 0x1FFF)

### 0x0FFF - DSP Control Register (16 bits)

| Bit   | Name                  | Description                        |
|-------|-----------------------|------------------------------------|
| 0     | BYP_PLL               | Bypass PLL                         |
| 1-5   | PLL_DIV_MASK          | PLL divider (0x1F = 62.975MHz)     |
| 6     | DSP_TURBO             | DSP turbo mode                     |
| 7     | PC_RESET_DSP1         | Reset DSP1 program counter         |
| 8     | PC_RESET_DSP2         | Reset DSP2 program counter         |
| 15    | DIG_SIL_INTERPOL      | Digital silence interpolation      |

**Values:**
- `0x003E` - PLL 62.975MHz, normal mode
- `0x0000` - Normal operation after init

---

### 0x1FF0 - Evaluation Register (24 bits)

Disable charge pump and DCS.

**Init value:** `0x000000`

---

### 0x1FF3 - CL_GEN1 Register (24 bits)

Loop gain settings for clock generator.

**Init value:** `0x040022`

---

### 0x1FF4 - CL_GEN2 Register (24 bits)

Falling edge detection.

**Init value:** `0x000001`

---

### 0x1FF6 - CL_GEN4 Register (24 bits)

PLL1 and turbo settings.

**Init value:** `0x024080`

---

### 0x1FF7 - SEL Register (Selector) - IMPORTANT! (24 bits)

**This register controls audio source selection (I2S, SPDIF, etc.)**

| Bits  | Mask     | Name           | Description                 |
|-------|----------|----------------|-----------------------------|
| 0-2   | 0x000007 | DSP2_SRCA      | DSP2 Source A selection     |
| 4-5   | 0x000030 | DSP2_FMTA      | DSP2 Format A               |
| 6-8   | 0x0001C0 | DSP2_SRCB      | DSP2 Source B selection     |
| 9-11  | 0x000E00 | DSP2_FMTB      | DSP2 Format B               |
| 12-13 | 0x003000 | DSP1_SRC       | DSP1 Source selection       |
| 14-16 | 0x01C000 | DSP1_FMT       | DSP1 Format                 |
| 17    | 0x020000 | SPDIF2         | SPDIF2 output selection     |
| 18-20 | 0x1C0000 | HOST_IO_FMT    | Host I/O format             |
| 21    | 0x200000 | EN_HOST_IO     | Enable Host I/O interface   |

**Init value:** `0x200080`
- EN_HOST_IO = 1 (host interface enabled)
- DSP2_SRCB bit 7 = 1

#### Possible sources (SRCA/SRCB):
```
000 = Internal (FM/AM)
001 = I2S1
010 = I2S2
011 = SPDIF
100 = Host I/O
...
```

---

### 0x1FF8 - IAC Register (24 bits)

Interference Absorption Circuit settings.

**Init value:** `0xF4CAED`

---

### 0x1FF9 - CLK_SET Register (24 bits)

Clock configuration.

**Init value:** `0x124334`

---

### 0x1FFA - CLK_COEFF Register (24 bits)

Clock coefficients.

**Init value:** `0x004A1A`

---

### 0x1FFB - INPUT_SENS Register (24 bits)

Input sensitivity and filters.

| Bits  | Mask     | Name               | Description           |
|-------|----------|--------------------|-----------------------|
| 0-5   | 0x00003F | RDS_VOL            | RDS volume            |
| 6-11  | 0x000FC0 | FM_VOL             | FM volume             |
| 12    | 0x001000 | FM_MPX             | FM multiplex enable   |
| 13    | 0x002000 | OFF_FILTER_A_EN    | Offset filter A       |
| 14    | 0x004000 | OFF_FILTER_B_EN    | Offset filter B       |

**Init value:** `0x0071C7`

---

### 0x1FFC - PHONE_NAV_AUDIO Register (24 bits)

Phone, navigation and audio routing.

**Init value:** `0x0E22FF`

---

### 0x1FFD - IO_CONF_DSP2 Register (24 bits)

I/O configuration for DSP2.

**Init value:** `0x001FF8`

---

### 0x1FFE - STATUS_DSP2 Register (24 bits)

DSP2 status register.

**Init value:** `0x080003`

---

### 0x1FFF - PC_DSP2 Register (24 bits)

Program counter DSP2.

**Init value:** `0x000004`

---

## DSP2 XRAM Important Addresses

| Address | Name            | Init Value | Description           |
|---------|-----------------|------------|-----------------------|
| 0x11F9  | FDACPNTR        | -          | FDAC pointer          |
| 0x11FB  | IIS1PNTR        | -          | I2S1 interface ptr    |

When writing to XRAM, use address 0x1xxx with data 0x2000CB.

---

## DSP2 YRAM Volume Controls

| Address | Name     | Value  | Description            |
|---------|----------|--------|------------------------|
| 0x20CB  | PVGA     | 0x0F80 | Programmable VGA       |
| 0x20CD  | PVAT1    | 0x0800 | PVA attenuation 1      |
| 0x20CF  | PVAT     | 0x0800 | PVA attenuation        |

---

## I2C Protocol

### Write Sequence

```
START -> 0x38 (write) -> ADDR_H -> ADDR_L -> DATA[0] -> DATA[1] -> DATA[2] -> STOP
```

Example: writing 0x200080 to register 0x1FF7:
```
START -> 0x38 -> 0x1F -> 0xF7 -> 0x20 -> 0x00 -> 0x80 -> STOP
```

### Read Sequence

```
START -> 0x38 (write) -> ADDR_H -> ADDR_L ->
RESTART -> 0x39 (read) -> DATA[0] <- DATA[1] <- DATA[2] <- STOP
```

---

## Initialization Sequence (from Linux driver)

Complete initialization sequence to enable audio:

```c
// 1. Reset PLL
writeReg(0x0FFF, 0x003E);  // PLL div for 62.975MHz

// 2. Configure hardware registers
writeReg(0x1FF0, 0x000000);  // Evaluation - disable charge pump
writeReg(0x1FF3, 0x040022);  // CL_GEN1
writeReg(0x1FF4, 0x000001);  // CL_GEN2
writeReg(0x1FF6, 0x024080);  // CL_GEN4
writeReg(0x1FF7, 0x200080);  // SEL - enable host I/O
writeReg(0x1FF8, 0xF4CAED);  // IAC
writeReg(0x1FF9, 0x124334);  // CLK_SET
writeReg(0x1FFA, 0x004A1A);  // CLK_COEFF
writeReg(0x1FFB, 0x0071C7);  // INPUT_SENS
writeReg(0x1FFC, 0x0E22FF);  // PHONE_NAV_AUDIO
writeReg(0x1FFD, 0x001FF8);  // IO_CONF_DSP2
writeReg(0x1FFE, 0x080003);  // STATUS_DSP2
writeReg(0x1FFF, 0x000004);  // PC_DSP2

// 3. Configure DSP2 XRAM pointers
writeReg(0x11FB, 0x2000CB);  // IIS1PNTR
writeReg(0x11F9, 0x2000CB);  // FDACPNTR

// 4. Set volume in YRAM
writeReg(0x20CB, 0x000F80);  // PVGA
writeReg(0x20CD, 0x000800);  // PVAT1
writeReg(0x20CF, 0x000800);  // PVAT

// 5. Release PLL
writeReg(0x0FFF, 0x0000);    // Normal operation
```

---

## I2S Input Configuration

To receive I2S signal, configure SEL register (0x1FF7):

```c
// Option 1: I2S1 as source for DSP2
// DSP2_SRCA = 001 (I2S1)
// EN_HOST_IO = 1
uint32_t sel_i2s1 = 0x200001;  // or try 0x200081

// Option 2: I2S2 as source
// DSP2_SRCA = 010 (I2S2)
uint32_t sel_i2s2 = 0x200002;
```

---

## SPDIF Input Configuration

```c
// SPDIF as source for DSP2
// DSP2_SRCA = 011 (SPDIF)
uint32_t sel_spdif = 0x200003;

// Also try enabling SPDIF2 bit
uint32_t sel_spdif2 = 0x220003;
```

---

## Resources

- [Linux kernel driver (saa7706h.c)](https://github.com/torvalds/linux/blob/master/drivers/media/radio/saa7706h.c)
- [NXP Product Page](https://www.nxp.com/products/no-longer-manufactured/car-radio-digital-signal-processor-dsp:SAA7706H)

---

## TODO / Unknown Registers

The following registers require additional research:

- 0x1FF1 - Unknown register 1
- 0x1FF2 - Unknown register 2
- 0x1FF5 - CL_GEN3 (not used in Linux driver)

---

## Notes for BMW CD73

CD73 uses SAA7706H for audio processing. The chip receives:
- FM/AM signal from tuner
- CD audio via I2S
- External sources (AUX) can be connected via I2S or SPDIF

### System Architecture (I2C MITM)

```
┌──────────────┐                      ┌───────────────┐
│  Head Unit   │                      │   SAA7706H    │──────▶ MOST Bus ──▶ Amplifier
│  Processor   │                      │    (DSP)      │
└──────┬───────┘                      └───────┬───────┘
       │                                      │
       │              I2C Bus                 │
       └──────────────┬───────────────────────┘
                      │
               ┌──────┴──────┐
               │    ESP32    │  ◀── Monitors I2C, detects "CD mode"
               │   (MITM)    │  ──▶ Reconfigures SAA7706H for I2S input
               └─────────────┘

┌─────────────────┐   I2S    ┌───────────────┐
│  External Audio │─────────▶│   SAA7706H    │
│     Board       │          │   (I2S IN)    │
└─────────────────┘          └───────────────┘
```

- **Head Unit Processor** - normally controls SAA7706H via I2C
- **ESP32 (MITM)** - monitors I2C bus, detects mode changes, injects commands
- **External Audio Board** - provides I2S or SPDIF digital audio signal
- **SAA7706H** - receives digital audio, processes it, outputs to MOST bus

### How It Works:
1. ESP32 monitors I2C traffic between head unit processor and SAA7706H
2. When user selects "CD" mode, ESP32 detects this command on I2C bus
3. ESP32 sends its own I2C commands to reconfigure SAA7706H for I2S input
4. External audio board feeds I2S signal directly to SAA7706H
5. Digital audio goes through DSP → MOST bus → Amplifier
