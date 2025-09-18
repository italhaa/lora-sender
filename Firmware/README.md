# STM32L476 LoRa Transmitter Firmware

## Flash Files

- **LR1121.v4.hex** - Intel HEX format for most STM32 flash tools
- **LR1121.v4.bin** - Raw binary format for ST-Link/OpenOCD
- **LR1121.v4.elf** - ELF file with debug symbols for debugging

## Test Binary Data Transmission

This firmware sends structured test patterns via LoRa at 2444 MHz:

1. **Pattern 0**: Alternating 0xAA/0x55 (for basic connectivity testing)
2. **Pattern 1**: Counting sequence 0, 16, 32, 48... (for data integrity)
3. **Pattern 2**: Binary bit shift 0x01, 0x02, 0x04, 0x08... (for bit errors)
4. **Pattern 3**: XOR with timestamp (for timing/sync testing)

Each packet is 25 bytes:
- 9-byte header (source ID, file type/version, packet ID, length)
- 16-byte test data payload

## Flashing Instructions

### Using ST-Link Utility:
1. Connect ST-Link to STM32L476
2. Open ST-Link Utility
3. Load `LR1121.v4.hex` 
4. Click "Program & Verify"

### Using STM32CubeProgrammer:
1. Connect via ST-Link
2. Load `LR1121.v4.hex`
3. Download to target

### Using OpenOCD (command line):
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "program LR1121.v4.elf verify reset exit"
```

## Hardware Requirements

- STM32L476RG microcontroller
- LR1121 LoRa transceiver 
- 2.4 GHz antenna
- Push button for power level cycling
- LEDs for TX/RX/SCAN status indication

## Debug Output

Connect UART2 (PA2/PA3) at 115200 baud to see:
- Startup mode identification
- Transmission details with HEX/DEC/BIN data
- Pattern cycle information
- Power level changes

## Build Information

- Compiler: ARM GCC 14.2.1
- Optimization: -Os (size optimized)
- Flash size: ~17KB
- Built: September 18, 2025