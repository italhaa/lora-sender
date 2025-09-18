# STM32L476 LoRa Transmitter Firmware - ULTRA SIMPLE VERSION

## Flash Files

- **LR1121.v4.hex** - Intel HEX format for most STM32 flash tools  
- **LR1121.v4.bin** - Raw binary format for ST-Link/OpenOCD (13,936 bytes)
- **LR1121.v4.elf** - ELF file with debug symbols for debugging

## 🚀 SIMPLIFIED DESIGN - TRANSMIT ONLY!

This version has been **dramatically simplified** from 800+ lines to ~180 lines:

### ✅ **WHAT IT DOES:**
- **Pure transmitter** - no RX, no responses, no complexity!
- Sends 16-byte test patterns every 2 seconds
- 4 cycling test patterns for comprehensive testing
- Simple power level switching (5 levels: -10, -5, 0, +5, +10 dBm)
- Clear debug output showing exactly what's transmitted

### ❌ **WHAT WAS REMOVED:**
- All RX/response handling (~300 lines)
- Complex state machines and interrupt handling
- Multi-mode (Hub/Node) architecture  
- File transfer concepts and metrics
- CAD (Channel Activity Detection)
- Multi-frequency support (fixed to 2444 MHz)
- Sleep mode management

## Test Patterns

Each transmission cycles through these 16-byte patterns:

1. **Pattern 0**: `AA 55 AA 55 AA 55 AA 55 AA 55 AA 55 AA 55 AA 55`
2. **Pattern 1**: `00 10 20 30 40 50 60 70 80 90 A0 B0 C0 D0 E0 F0`  
3. **Pattern 2**: `01 02 04 08 10 20 40 80 01 02 04 08 10 20 40 80`
4. **Pattern 3**: `[timestamp-based varying pattern]`

## How It Works

```c
// Ultra-simple main loop:
void lora_system_process(void) {
    if (!initialized) simple_lora_init();     // One-time init
    generate_test_pattern(pattern_counter);   // Make test data
    simple_transmit();                        // Send it!
    pattern_counter++;                        // Next pattern
    HAL_Delay(2000);                         // Wait 2 seconds
}
```

## Hardware Requirements

- STM32L476RG microcontroller
- LR1121 LoRa transceiver 
- 2.4 GHz antenna
- Push button for power level cycling
- LEDs for TX status indication

## Build Information

- **Simplified Code**: ~180 lines (vs 800+ original)
- **Flash Size**: 13.9 KB (vs 17.8 KB original)
- **RAM Usage**: Significantly reduced
- **Complexity**: Ultra-simple, easy to understand
- **Compiler**: ARM GCC 14.2.1
- **Built**: September 18, 2025

## Perfect For:

✅ **Testing receiver development**  
✅ **Learning LoRa basics**  
✅ **Quick prototyping**  
✅ **Range testing**  
✅ **Understanding transmission patterns**

This is the **cleanest, simplest LoRa transmitter possible** - perfect for getting started!