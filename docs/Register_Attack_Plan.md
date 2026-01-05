# SAA7706H Register Attack Plan

## Goal
Determine which register bits to set for SAA7706H to receive I2S or SPDIF digital audio input.

## Test Environment
- **Head unit**: CD73 running (full power)
- **ESP32**: MITM on I2C bus (can sniff and inject)
- **Test signal**: ESP32 I2S output → SAA7706H I2S input
- **Detection**: Oscilloscope on chip output pins (analog or digital depending on ISTA config) + Logic analyzer

## Critical Bug in Current Code

**Current `readReg16()` reads only 2 bytes, but SAA7706H hardware registers are 24-bit (3 bytes)!**

```cpp
// WRONG - reads only 16 bits
int n = Wire.requestFrom(dev_addr, 2);  // Should be 3!
```

This must be fixed first - we're missing the third byte of every register.

---

## Phase 1: Fix I2C Functions (24-bit)

### 1.1 Create proper 24-bit read/write functions

```cpp
uint32_t readReg24(uint8_t h_addr, uint8_t l_addr, uint8_t dev_addr);
void writeReg24(uint8_t h_addr, uint8_t l_addr, uint8_t dev_addr, uint32_t value);
```

### 1.2 Update print function for 24-bit values

---

## Phase 2: I2C Sniffing (GOLDEN DATA!)

Since head unit processor is running, we can **sniff what IT writes** to SAA7706H!

### 2.1 I2C Sniffer Mode
- Configure ESP32 as I2C slave/sniffer (not master)
- Log ALL I2C traffic on the bus
- Capture: address, register, data bytes

### 2.2 Capture register writes in each mode
```
User action          → Capture I2C traffic
─────────────────────────────────────────
Power on             → Initial config sequence
Switch to FM         → FM mode registers
Switch to AM         → AM mode registers
Switch to CD         → CD MODE REGISTERS ← KEY!
Switch to AUX        → AUX mode registers
Volume change        → Volume registers
```

### 2.3 Analyze CD mode vs FM mode
**The difference between CD and FM register values = what we need for I2S input!**

CD mode likely changes:
- SEL register (0x1FF7) - source selection
- CLK_SET (0x1FF9) - clock for I2S
- Maybe IO_CONF (0x1FFD) - I/O pins

### 2.4 Baseline Read (as backup)
If sniffing is hard, just read registers in each mode:
- ESP32 reads all registers when in FM mode
- User switches to CD
- ESP32 reads all registers again
- Compare and diff

---

## Phase 3: Targeted Register Testing

Based on Linux driver analysis, priority registers for I2S/SPDIF:

### 3.1 SEL Register (0x1FF7) - HIGHEST PRIORITY

This register selects audio source. Test these values:

| Value | Expected Source | Test |
|-------|-----------------|------|
| 0x200001 | I2S1 + Host I/O | Try first |
| 0x200002 | I2S2 + Host I/O | |
| 0x200003 | SPDIF + Host I/O | |
| 0x200041 | I2S1 + SRCB=I2S1 | |
| 0x200081 | I2S1 + bit7 | Linux driver uses this |
| 0x220003 | SPDIF + SPDIF2 bit | |

### 3.2 IO_CONF_DSP2 (0x1FFD) - I/O Configuration

Controls input/output pin configuration.
- Default: 0x001FF8
- Try enabling I2S input pins

### 3.3 CLK_SET (0x1FF9) - Clock Configuration

I2S needs proper clock setup.
- Default: 0x124334
- May need adjustment for external I2S clock

### 3.4 PHONE_NAV_AUDIO (0x1FFC) - Audio Routing

Routes audio between sources and outputs.
- Default: 0x0E22FF
- May need to route I2S to output

---

## Phase 4: Systematic Bit Testing for SEL (0x1FF7)

### 4.1 Test each source bit combination (bits 0-2)

```
000 = 0x200000 - Internal (baseline)
001 = 0x200001 - I2S1
010 = 0x200002 - I2S2
011 = 0x200003 - SPDIF
100 = 0x200004 - Host I/O
101 = 0x200005 - ?
110 = 0x200006 - ?
111 = 0x200007 - ?
```

### 4.2 For each source, test format bits (bits 4-5)

```
00 = Left justified
01 = I2S standard
10 = Right justified
11 = ?
```

### 4.3 Test with/without EN_HOST_IO (bit 21)

---

## Phase 5: Automated I2S Loopback Testing (RECOMMENDED)

Using ESP32 dual I2S peripherals for fully automated testing!

### 5.1 Setup I2S loopback
```cpp
// I2S0 = TX (to SAA7706H input)
i2s_config_t i2s_tx_config = {
    .mode = I2S_MODE_MASTER | I2S_MODE_TX,
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    ...
};

// I2S1 = RX (from SAA7706H output)
i2s_config_t i2s_rx_config = {
    .mode = I2S_MODE_MASTER | I2S_MODE_RX,
    ...
};
```

### 5.2 Test pattern generation
- Generate known pattern: 0xAA55AA55 or sine wave
- Continuously transmit via I2S0

### 5.3 Automated register sweep
```cpp
void autoTestRegisters() {
    // Test all SEL source combinations
    for (uint8_t src = 0; src <= 7; src++) {
        // Test all format combinations
        for (uint8_t fmt = 0; fmt <= 3; fmt++) {
            uint32_t sel = 0x200000 | src | (fmt << 4);
            writeReg24(0x1F, 0xF7, 0x1C, sel);

            delay(100);

            if (checkI2SOutput()) {
                Serial.printf("FOUND: SEL=0x%06X\n", sel);
            }
        }
    }
}
```

### 5.4 Success detection
- Compare received I2S1 data with transmitted I2S0 pattern
- Allow for DSP processing delay
- Log all working configurations

### 5.5 Alternative: Manual with oscilloscope
- Connect oscilloscope to SAA7706H output pins
- For each register combination, visually check signal
- Slower but works if I2S RX has issues

---

## Phase 6: SPDIF Testing

If I2S doesn't work, try SPDIF:

### 6.1 SPDIF source setup
- Use SPDIF transmitter board
- Or ESP32 with SPDIF library

### 6.2 Test SPDIF-specific register values
```cpp
// SPDIF as source
writeReg24(0x1F, 0xF7, 0x1C, 0x200003);  // DSP2_SRCA = SPDIF

// Try with SPDIF2 bit
writeReg24(0x1F, 0xF7, 0x1C, 0x220003);  // + SPDIF2 bit
```

---

## Test Sequence (Step by Step)

### Step 1: Fix code
- [ ] Implement 24-bit read/write functions
- [ ] Test by reading and writing back same values

### Step 2: Setup I2S loopback
- [ ] Configure I2S0 as TX (test signal generator)
- [ ] Configure I2S1 as RX (detector)
- [ ] Verify loopback works with direct wire connection (bypass SAA7706H)

### Step 3: Capture baselines
- [ ] Read all registers at power-on
- [ ] Read all registers in CD mode (sniff or read)
- [ ] Compare FM vs CD mode differences

### Step 4: Apply Linux driver init
- [ ] Write full initialization sequence from Linux driver
- [ ] Verify registers are set correctly

### Step 5: Automated register sweep
- [ ] Run autoTestRegisters() function
- [ ] Test all SEL source combinations (0-7)
- [ ] Test all format combinations (0-3)
- [ ] Log all combinations that pass audio through

### Step 6: Verify and optimize
- [ ] Test found configurations with real audio
- [ ] Try different clock settings if needed
- [ ] Find optimal configuration

### Step 7: Document working configuration
- [ ] Record all register values that produce audio
- [ ] Create initialization function
- [ ] Update SAA7706H_Register_Map.md

---

## Files to Modify

1. **src/main.cpp** - Fix I2C functions, add test sequences
2. **docs/SAA7706H_Register_Map.md** - Document findings

---

## Hardware Setup

### Available
- Logic analyzer (I2C sniffing + output monitoring)
- Oscilloscope (output monitoring)
- ESP32 (I2C MITM + I2S TX/RX for automated testing)
- CD73 head unit (full power)

### ESP32 as I2S Loopback Tester

ESP32 has **two independent I2S peripherals** (I2S0 and I2S1):
- **I2S0** = TX (test signal source → SAA7706H input)
- **I2S1** = RX (detector ← SAA7706H output)

```
┌─────────────────────────────────────────────────────────────┐
│                         ESP32                                │
│  ┌─────────┐                              ┌─────────┐       │
│  │  I2S0   │──── TX ────┐    ┌──── RX ────│  I2S1   │       │
│  │  (OUT)  │            │    │            │  (IN)   │       │
│  └─────────┘            │    │            └─────────┘       │
│                         │    │                               │
│  ┌─────────┐            │    │                               │
│  │  I2C    │────────────┼────┼───────────────────────────   │
│  │ Master  │            │    │                           │   │
│  └─────────┘            │    │                           │   │
└─────────────────────────┼────┼───────────────────────────┼───┘
                          │    │                           │
                          ▼    │                           │
                    ┌──────────┴───────────┐               │
                    │      SAA7706H        │               │
                    │   ┌───────────────┐  │               │
                    │   │    DSP        │  │◀──── I2C ─────┘
                    │   └───────────────┘  │
                    │  I2S IN      I2S OUT │
                    └────┬────────────┬────┘
                         │            │
                         │            └──── to ESP32 I2S1 RX
                         │
                    from ESP32 I2S0 TX
```

### Automated Test Algorithm

```cpp
// 1. Generate test pattern via I2S0 TX
uint32_t test_pattern = 0xAA55AA55;
i2s_write(I2S_NUM_0, &test_pattern, ...);

// 2. Try different SEL register values
for (uint32_t sel = 0x200000; sel <= 0x200007; sel++) {
    writeReg24(0x1F, 0xF7, 0x1C, sel);
    delay(100);

    // 3. Read from I2S1 RX
    uint32_t received;
    i2s_read(I2S_NUM_1, &received, ...);

    // 4. Check if pattern passes through
    if (received == test_pattern) {
        Serial.printf("SUCCESS! SEL=0x%06X works!\n", sel);
    }
}
```

### ESP32 Pin Configuration

| Function | ESP32 Pin | SAA7706H | Notes |
|----------|-----------|----------|-------|
| I2C SDA | GPIO21 | SDA | Shared with head unit |
| I2C SCL | GPIO22 | SCL | Shared with head unit |
| **I2S0 TX** | | | **Test signal OUT** |
| I2S0 BCK | GPIO26 | IIS_BCK (in) | Bit clock |
| I2S0 WS | GPIO25 | IIS_WS (in) | Word select |
| I2S0 DATA | GPIO27 | IIS_DI | Data to chip |
| **I2S1 RX** | | | **Detector IN** |
| I2S1 BCK | GPIO14 | IIS_BCK (out) | Bit clock |
| I2S1 WS | GPIO15 | IIS_WS (out) | Word select |
| I2S1 DATA | GPIO32 | IIS_DO | Data from chip |

**Note:** No pin conflicts between I2C and I2S!

---

## Quick Reference: Key Registers

| Addr | Name | Purpose | Priority |
|------|------|---------|----------|
| 0x1FF7 | SEL | Source selection | HIGHEST |
| 0x1FF9 | CLK_SET | Clock config | HIGH |
| 0x1FFD | IO_CONF | I/O pins | HIGH |
| 0x1FFC | PHONE_NAV | Audio routing | MEDIUM |
| 0x0FFF | CTRL | DSP/PLL | MEDIUM |
