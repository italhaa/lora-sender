/*
 * lora_base.c - SIMPLE LoRa Transmitter
 * 
 * Ultra-simplified version for testing - TRANSMIT ONLY!
 * No RX, no responses, no state machines, no complexity.
 *
 *  Created on: Feb 27, 2025
 *  Author: b.jamin (simplified by AI)
 */

#include "lora_base.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* External variables */
extern SPI_HandleTypeDef hspi1;

/* Simple configuration - 2.4GHz only */
#define LORA_FREQ_MHZ 2444
#define LORA_SYNCWORD 0x11
#define PACKET_SIZE 16

/* LoRa Parameters - Fixed and Simple */
static lr11xx_radio_pa_cfg_t pa_config = {
    .pa_sel = LR11XX_RADIO_PA_SEL_HF,
    .pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG,
    .pa_duty_cycle = 0x00,
    .pa_hp_sel = 0x00
};

static lr11xx_radio_mod_params_lora_t lora_params = {
    .sf = LR11XX_RADIO_LORA_SF5,
    .bw = LR11XX_RADIO_LORA_BW_500,
    .cr = LR11XX_RADIO_LORA_CR_4_7,
    .ldro = 0x00
};

static lr11xx_radio_pkt_params_lora_t packet_params = {
    .preamble_len_in_symb = 20,
    .header_type = LR11XX_RADIO_LORA_PKT_EXPLICIT,
    .pld_len_in_bytes = PACKET_SIZE,
    .crc = LR11XX_RADIO_LORA_CRC_ON,
    .iq = LR11XX_RADIO_LORA_IQ_INVERTED,
};

/* Simple power levels (2.4GHz) */
static int8_t power_levels[5] = {-10, -5, 0, 5, 10};
static uint8_t current_power_index = 2; // Start at 0 dBm

/* Transmit buffer and pattern counter */
static uint8_t tx_buffer[PACKET_SIZE];
static uint8_t pattern_counter = 0;
static uint32_t packet_counter = 0;

/* Simple status flags */
static uint8_t is_initialized = 0;

/* UI flags for main.c LED control */
uint8_t enable_ui_scan = 0;
uint8_t enable_ui_tx = 0;
uint8_t enable_ui_rx = 0;
uint8_t enable_ui_power = 0;

/* Function prototypes */
static void simple_lora_init(void);
static void generate_test_pattern(uint8_t pattern_type);
static void simple_transmit(void);
static void simple_print(char* message);

/**
 * @brief Initialize LoRa system - called once from main
 */
void lora_system_init(void) {
    simple_print("\n=== SIMPLE LoRa Transmitter ===\n");
    simple_print("Frequency: 2444 MHz\n");
    simple_print("Packet Size: 16 bytes\n");
    simple_print("Message: Hello World + counter\n");
    simple_print("Press button to change power\n");
    simple_print("================================\n\n");
}

/**
 * @brief Main process loop - called continuously from main
 */
void lora_system_process(void) {
    
    // One-time initialization
    if (!is_initialized) {
        simple_lora_init();
        is_initialized = 1;
    }
    
    // Generate test data and transmit
    generate_test_pattern(pattern_counter % 4);
    simple_transmit();
    
    // Cycle through patterns
    pattern_counter++;
    packet_counter++;
    
    // Wait 2 seconds between transmissions
    HAL_Delay(2000);
}

/**
 * @brief Change power level (called from button press)
 */
uint8_t lora_system_transmit_power(uint8_t boost) {
    if (boost > 0) {
        current_power_index = (current_power_index + 1) % 5;
        
        // Reinitialize with new power
        lr11xx_radio_set_tx_params(NULL, power_levels[current_power_index], LR11XX_RADIO_RAMP_48_US);
        
        char buffer[50];
        sprintf(buffer, "Power changed to: %d dBm\n", power_levels[current_power_index]);
        simple_print(buffer);
    }
    return current_power_index;
}

/**
 * @brief Initialize LR1121 LoRa chip - SIMPLIFIED
 */
static void simple_lora_init(void) {
    simple_print("Initializing LR1121...\n");
    
    // Hardware reset sequence
    HAL_GPIO_WritePin(LR_RESET_GPIO_Port, LR_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LR_NSS_GPIO_Port, LR_NSS_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    
    // Reset the chip
    lr11xx_hal_reset(NULL);
    HAL_Delay(10);
    
    // Basic configuration
    lr11xx_system_set_reg_mode(NULL, LR11XX_SYSTEM_REG_MODE_DCDC);
    lr11xx_system_cfg_lfclk(NULL, LR11XX_SYSTEM_LFCLK_XTAL, true);
    
    // Set LoRa mode
    lr11xx_radio_set_pkt_type(NULL, LR11XX_RADIO_PKT_TYPE_LORA);
    
    // Set frequency (2444 MHz)
    lr11xx_radio_set_rf_freq(NULL, LORA_FREQ_MHZ * 1000000UL);
    
    // Configure power amplifier and power level
    lr11xx_radio_set_pa_cfg(NULL, &pa_config);
    lr11xx_radio_set_tx_params(NULL, power_levels[current_power_index], LR11XX_RADIO_RAMP_48_US);
    
    // Set LoRa parameters
    lr11xx_radio_set_lora_mod_params(NULL, &lora_params);
    lr11xx_radio_set_lora_sync_word(NULL, LORA_SYNCWORD);
    
    // Clear any errors
    lr11xx_system_clear_errors(NULL);
    lr11xx_system_clear_irq_status(NULL, LR11XX_SYSTEM_IRQ_ALL_MASK);
    
    simple_print("LR1121 initialized successfully!\n\n");
}

/**
 * @brief Generate "Hello World" message with packet counter
 */
static void generate_test_pattern(uint8_t pattern_type) {
    // Clear buffer first
    memset(tx_buffer, 0, PACKET_SIZE);
    
    // Create the message with packet counter
    char message[PACKET_SIZE];
    snprintf(message, PACKET_SIZE, "Hello World %d", packet_counter % 100);
    
    // Copy string to tx_buffer (will be null-terminated or truncated to fit)
    strncpy((char*)tx_buffer, message, PACKET_SIZE - 1);
    
    // Ensure the last byte is null-terminated for safety
    tx_buffer[PACKET_SIZE - 1] = '\0';
}

/**
 * @brief Transmit the data - SIMPLE!
 */
static void simple_transmit(void) {
    // Print what we're sending as string
    char buffer[128];
    sprintf(buffer, "TX #%lu Message: \"%s\"\n", packet_counter, (char*)tx_buffer);
    simple_print(buffer);
    
    // Also show data in hex for debugging
    sprintf(buffer, "TX #%lu Binary: ", packet_counter);
    simple_print(buffer);
    for (int i = 0; i < PACKET_SIZE; i++) {
        sprintf(buffer, "%02X ", tx_buffer[i]);
        simple_print(buffer);
    }
    simple_print("\n");
    
    // Clear interrupts and set packet parameters
    lr11xx_system_clear_irq_status(NULL, LR11XX_SYSTEM_IRQ_ALL_MASK);
    lr11xx_radio_set_lora_pkt_params(NULL, &packet_params);
    
    // Load data into transmit buffer
    lr11xx_regmem_write_buffer8(NULL, tx_buffer, PACKET_SIZE);
    
    // Transmit with timeout
    lr11xx_radio_set_tx_with_timeout_in_rtc_step(NULL, 5000);
    
    // Show TX LED
    enable_ui_tx = 1;
    
    // Wait for transmission to complete (simple polling)
    HAL_Delay(100); // Give time for transmission
}

/**
 * @brief Simple print function (uses printf if UART is configured)
 */
static void simple_print(char* message) {
    // This will work if you have printf retargeted to UART
    printf("%s", message);
    
    // Alternative: Use HAL_UART_Transmit if you have UART handle
    // HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);
}


