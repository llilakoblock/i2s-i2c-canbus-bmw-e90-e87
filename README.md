# SAA7706H Attack Tool - BMW CD73 I2S Injection

Reverse engineering the **SAA7706H DSP** chip in BMW CD73 Professional radio to enable **I2S digital audio input** via ESP32 MITM attack.

## Goal

When user selects "CD mode" on head unit, ESP32 intercepts I2C command and reconfigures SAA7706H to accept I2S audio from ESP32 instead of original CD changer.

```
┌─────────────┐     I2C      ┌───────────────┐
│  Head Unit  │ ◄──────────► │    ESP32      │ (MITM)
│  Processor  │              │               │
└─────────────┘              └───────┬───────┘
                                     │
                    ┌────────────────┼────────────────┐
                    │ I2C            │ I2S            │
                    ▼                ▼                │
              ┌───────────────────────────┐          │
              │       SAA7706H DSP        │          │
              │  (reconfigured for I2S)   │          │
              └───────────────────────────┘          │
                    │                                │
                    ▼ Analog Output                  │
              ┌───────────────┐                      │
              │   Amplifier   │ ◄────────────────────┘
              │   Speakers    │      (audio from ESP32)
              └───────────────┘
```

## Hardware

- **ESP32 DevKit** - I2C MITM + I2S Master
- **BMW CD73** - Head unit with SAA7706H DSP
- **Logic Analyzer** - I2C/I2S debugging
- **Oscilloscope** - Analog output verification

## Pin Configuration

### I2C (MITM on head unit bus)
| ESP32 | SAA7706H | Function |
|-------|----------|----------|
| GPIO21 | Pin 43 (SDA) | I2C Data |
| GPIO22 | Pin 44 (SCL) | I2C Clock |

### I2S TX (to SAA7706H CD input)
| ESP32 | SAA7706H | Function |
|-------|----------|----------|
| GPIO26 | Pin 29 (CD_CLK) | Bit Clock |
| GPIO25 | Pin 27 (CD_WS) | Word Select |
| GPIO27 | Pin 28 (CD_DATA) | Audio Data |

### I2S RX (from SAA7706H output - optional)
| ESP32 | SAA7706H | Function |
|-------|----------|----------|
| GPIO14 | Pin 30 (IIS_CLK) | Bit Clock |
| GPIO15 | Pin 33 (IIS_WS) | Word Select |
| GPIO32 | Pin 34 (IIS_OUT1) | Audio Data |

## Audio Format

- **Sample Rate**: 44100 Hz (or 48000 Hz)
- **Bit Depth**: 16-bit
- **Channels**: 2 (stereo)
- **I2S Format**: Philips I2S Standard

## Key Register: SEL (0x1FF7)

The SEL register controls audio source selection:

```
SEL = 0x200011  ← Primary attack value
      │ │││
      │ │││
      │ ││└── bits 0-2 = 001 (Source = I2S1)
      │ │└─── bits 4-5 = 01 (Format = I2S Standard)
      │ └──── bit 21 = 1 (EN_HOST_IO)
      └────── 0x20 = bit 21
```

| Value | Description |
|-------|-------------|
| 0x200011 | I2S1 + I2S Standard format (primary) |
| 0x200001 | I2S1 + Left Justified |
| 0x200012 | I2S2 + I2S Standard |
| 0x200003 | SPDIF1 input |

## Build & Flash (ESP-IDF)

```bash
# Configure
idf.py set-target esp32
idf.py menuconfig

# Build
idf.py build

# Flash
idf.py -p COM3 flash monitor
```

## Serial Menu

```
╔══════════════════════════════════════════════════════════╗
║           SAA7706H ATTACK TOOL - ESP-IDF                 ║
╠══════════════════════════════════════════════════════════╣
║  1. Read all registers                                   ║
║  2. Apply Linux driver init                              ║
║  3. Run I2S Attack (44.1kHz 16-bit sweep)                ║
║  4. Quick I2S test (SEL=0x200011)                        ║
║  5. Run full SEL sweep                                   ║
║  6. Play WAV file                                        ║
║  7. Generate 1kHz tone                                   ║
║  8. Test I2S1 source                                     ║
║  9. Test I2S2 source                                     ║
║  0. Test SPDIF source                                    ║
╚══════════════════════════════════════════════════════════╝
```

## Project Structure

```
├── src/
│   ├── main.cpp              # Entry point with serial menu
│   ├── saa7706h/             # I2C driver (24-bit registers)
│   ├── i2s/                  # I2S TX/RX loopback
│   ├── attack/               # Automated SEL register sweep
│   └── audio/                # Embedded WAV file
├── include/
│   ├── config/               # Pin config, register addresses
│   ├── saa7706h/             # SAA7706H driver header
│   ├── i2s/                  # I2S driver header
│   └── attack/               # Attack module header
├── docs/
│   ├── SAA7706H_datasheet.pdf
│   ├── SAA7706H_Digital_Audio_Pinout.md
│   └── SAA7706H_Register_Map.md
└── CMakeLists.txt            # ESP-IDF build config
```

## Documentation

- [SAA7706H Digital Audio Pinout](docs/SAA7706H_Digital_Audio_Pinout.md) - I2S/SPDIF pins and capabilities
- [SAA7706H Register Map](docs/SAA7706H_Register_Map.md) - Register addresses and values

## References

- [Linux SAA7706H Driver](https://github.com/torvalds/linux/blob/master/sound/soc/codecs/saa7706h.c)
- [NXP SAA7706H Datasheet](docs/SAA7706H_datasheet.pdf)

## License

MIT License
