# SAA7706H Digital Audio Interface & Pinout

## Overview

SAA7706H - NXP/Philips Car Radio Digital Signal Processor (DSP)
- Package: 80-pin QFP (LQFP80)
- Crystal: 11.2896 MHz
- I2C Address: 0x1C (7-bit) / 0x38-0x39 (8-bit R/W)

---

## I2S Interface

### Operating Mode
**SAA7706H работает как SLAVE** - внешний источник должен предоставлять clock!

### Supported Formats
| Format | Description |
|--------|-------------|
| I2S-bus | Standard Philips I2S format |
| LSB-justified 16-bit | Left-aligned, 16-bit samples |
| LSB-justified 18-bit | Left-aligned, 18-bit samples |
| LSB-justified 20-bit | Left-aligned, 20-bit samples |
| LSB-justified 24-bit | Left-aligned, 24-bit samples |

### Supported Sample Rates
- **44.1 kHz** (CD quality)
- **48 kHz** (DAT/DVD quality)

---

## I2S Input Pins (CD/Tuner Input)

Эти пины используются для подключения CD-чейнджера или внешнего I2S источника.

| Pin # | Name | Function | Direction |
|-------|------|----------|-----------|
| 27 | CD_WS | Word Select (LRCK) | Input |
| 28 | CD_DATA | Serial Data | Input |
| 29 | CD_CLK | Bit Clock (BCK) | Input |

**Подключение ESP32:**
```
ESP32 GPIO26 (I2S_BCK)  → Pin 29 (CD_CLK)
ESP32 GPIO25 (I2S_WS)   → Pin 27 (CD_WS)
ESP32 GPIO27 (I2S_DOUT) → Pin 28 (CD_DATA)
```

---

## I2S Output Pins (To External DAC/Amplifier)

| Pin # | Name | Function | Direction |
|-------|------|----------|-----------|
| 30 | IIS_CLK | Bit Clock Output | Output |
| 33 | IIS_WS | Word Select Output | Output |
| 34 | IIS_OUT1 | Serial Data Output 1 (Front) | Output |
| 35 | IIS_OUT2 | Serial Data Output 2 (Rear) | Output |

---

## I2S Co-Processor Input Pins

Входы от внешнего DSP/процессора (Host I/O interface).

| Pin # | Name | Function | Direction |
|-------|------|----------|-----------|
| 31 | IIS_IN1 | Serial Data Input 1 | Input |
| 32 | IIS_IN2 | Serial Data Input 2 | Input |

---

## SPDIF Interface

### Capabilities
- **2 независимых SPDIF входа**
- Только **Consumer Mode** (не Professional)
- Sample rates: **44.1 kHz** и **48 kHz** (32 kHz НЕ поддерживается!)

### SPDIF Input Pins

| Pin # | Name | Function | Direction |
|-------|------|----------|-----------|
| 25 | SPDIF1 | SPDIF Input 1 | Input |
| 24 | SPDIF2 | SPDIF Input 2 | Input |

---

## Complete 80-Pin QFP Pinout

### Pins 1-20 (Analog Section)
| Pin | Name | Description |
|-----|------|-------------|
| 1 | TEF_I2S_CLK | TEF6730 I2S clock |
| 2 | TEF_I2S_WS | TEF6730 I2S word select |
| 3 | TEF_I2S_DI | TEF6730 I2S data in |
| 4 | TEF_I2S_DO | TEF6730 I2S data out |
| 5 | VDDA1 | Analog supply 1 |
| 6 | VSSA1 | Analog ground 1 |
| 7 | RDS_CLK | RDS clock output |
| 8 | RDS_DATA | RDS data input |
| 9 | RFAGC | RF AGC output |
| 10 | IFAGC | IF AGC output |
| 11 | VDDA2 | Analog supply 2 |
| 12 | VSSA2 | Analog ground 2 |
| 13 | FLO | Front Left Output |
| 14 | FRO | Front Right Output |
| 15 | RLO | Rear Left Output |
| 16 | RRO | Rear Right Output |
| 17 | VDDA3 | Analog supply 3 |
| 18 | VSSA3 | Analog ground 3 |
| 19 | SWLO | Subwoofer Left Output |
| 20 | SWRO | Subwoofer Right Output |

### Pins 21-40 (Digital Audio)
| Pin | Name | Description |
|-----|------|-------------|
| 21 | VDDD1 | Digital supply 1 |
| 22 | VSSD1 | Digital ground 1 |
| 23 | XTAL1 | Crystal input (11.2896 MHz) |
| 24 | **SPDIF2** | **SPDIF Input 2** |
| 25 | **SPDIF1** | **SPDIF Input 1** |
| 26 | VDDD2 | Digital supply 2 |
| 27 | **CD_WS** | **CD I2S Word Select** |
| 28 | **CD_DATA** | **CD I2S Data Input** |
| 29 | **CD_CLK** | **CD I2S Bit Clock** |
| 30 | **IIS_CLK** | **I2S Clock Output** |
| 31 | **IIS_IN1** | **I2S Data Input 1** |
| 32 | **IIS_IN2** | **I2S Data Input 2** |
| 33 | **IIS_WS** | **I2S Word Select Output** |
| 34 | **IIS_OUT1** | **I2S Data Output 1** |
| 35 | **IIS_OUT2** | **I2S Data Output 2** |
| 36 | VSSD2 | Digital ground 2 |
| 37 | VDDD3 | Digital supply 3 |
| 38 | SPEED | Speed pulse input |
| 39 | MUTE_OUT | Mute output |
| 40 | DIAG | Diagnostic output |

### Pins 41-60 (Control & I2C)
| Pin | Name | Description |
|-----|------|-------------|
| 41 | VSSD3 | Digital ground 3 |
| 42 | VDDD4 | Digital supply 4 |
| 43 | **SDA** | **I2C Data** |
| 44 | **SCL** | **I2C Clock** |
| 45 | RESET_N | Reset (active low) |
| 46 | TEST | Test mode |
| 47 | VSSD4 | Digital ground 4 |
| 48 | VDDD5 | Digital supply 5 |
| 49 | GPIO0 | General purpose I/O 0 |
| 50 | GPIO1 | General purpose I/O 1 |
| 51 | GPIO2 | General purpose I/O 2 |
| 52 | GPIO3 | General purpose I/O 3 |
| 53 | VSSD5 | Digital ground 5 |
| 54 | VDDD6 | Digital supply 6 |
| 55 | INT_N | Interrupt output |
| 56 | RESERVED | Reserved |
| 57 | RESERVED | Reserved |
| 58 | RESERVED | Reserved |
| 59 | VSSD6 | Digital ground 6 |
| 60 | VDDD7 | Digital supply 7 |

### Pins 61-80 (Audio Input)
| Pin | Name | Description |
|-----|------|-------------|
| 61 | PHONE_L | Phone Left Input |
| 62 | PHONE_R | Phone Right Input |
| 63 | NAV_L | Navigation Left Input |
| 64 | NAV_R | Navigation Right Input |
| 65 | VDDA4 | Analog supply 4 |
| 66 | VSSA4 | Analog ground 4 |
| 67 | AUX1_L | Auxiliary 1 Left |
| 68 | AUX1_R | Auxiliary 1 Right |
| 69 | AUX2_L | Auxiliary 2 Left |
| 70 | AUX2_R | Auxiliary 2 Right |
| 71 | VDDA5 | Analog supply 5 |
| 72 | VSSA5 | Analog ground 5 |
| 73 | FM_L | FM Tuner Left |
| 74 | FM_R | FM Tuner Right |
| 75 | AM | AM Tuner Input |
| 76 | VDDA6 | Analog supply 6 |
| 77 | VSSA6 | Analog ground 6 |
| 78 | WB_L | Wideband Left |
| 79 | WB_R | Wideband Right |
| 80 | VDDA7 | Analog supply 7 |

---

## Key Registers for Digital Audio

### SEL Register (0x1FF7) - Source Selection

| Bits | Field | Values |
|------|-------|--------|
| 2:0 | DSP2_SRCA | 0=Internal, 1=I2S1, 2=I2S2, 3=SPDIF, 4=HostIO |
| 5:4 | DSP2_SRCB | Secondary source |
| 6 | SPDIF2 | Select SPDIF2 instead of SPDIF1 |
| 21 | EN_HOST_IO | Enable Host I/O interface |

### Recommended Values for I2S Input
```c
// I2S1 input (CD_WS/CD_DATA/CD_CLK pins)
SEL = 0x200001  // DSP2_SRCA = I2S1, EN_HOST_IO = 1

// I2S2 input (co-processor)
SEL = 0x200002  // DSP2_SRCA = I2S2, EN_HOST_IO = 1

// SPDIF1 input
SEL = 0x200003  // DSP2_SRCA = SPDIF, EN_HOST_IO = 1

// SPDIF2 input
SEL = 0x200043  // DSP2_SRCA = SPDIF, SPDIF2 = 1, EN_HOST_IO = 1
```

---

## ESP32 Connection Diagram

```
ESP32                          SAA7706H
──────                         ────────
GPIO21 (SDA) ◄──────────────► Pin 43 (SDA)
GPIO22 (SCL) ◄──────────────► Pin 44 (SCL)

GPIO26 (I2S_BCK)  ──────────► Pin 29 (CD_CLK)
GPIO25 (I2S_WS)   ──────────► Pin 27 (CD_WS)
GPIO27 (I2S_DOUT) ──────────► Pin 28 (CD_DATA)

GPIO14 (I2S_BCK)  ◄────────── Pin 30 (IIS_CLK)
GPIO15 (I2S_WS)   ◄────────── Pin 33 (IIS_WS)
GPIO32 (I2S_DIN)  ◄────────── Pin 34 (IIS_OUT1)
```

---

## Important Notes

1. **SLAVE Mode**: SAA7706H не генерирует I2S clock - ESP32 должен быть master
2. **Sample Rate**: Только 44.1kHz или 48kHz, другие частоты не поддерживаются
3. **SPDIF**: Только consumer format, professional mode не поддерживается
4. **I2C**: Fast mode 400kHz поддерживается, адрес 0x1C
5. **Crystal**: 11.2896 MHz (стандартный audio crystal для 44.1kHz кратных частот)

---

## Attack Strategy

Для включения I2S входа на SAA7706H:

1. Инициализировать I2C (400kHz, адрес 0x1C)
2. Записать SEL register (0x1FF7) = 0x200001 для I2S1
3. Опционально: настроить формат в IO_CONF_DSP2 (0x1FFD)
4. Подать I2S сигнал на пины 27-29
5. Проверить выход на аналоговых пинах 13-16 или I2S выходе 34-35
