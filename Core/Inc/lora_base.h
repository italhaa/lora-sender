/*
 * lora_base.h - SIMPLE LoRa Transmitter Header
 *
 * Ultra-simplified version for testing - TRANSMIT ONLY!
 *
 *  Created on: Feb 27, 2025
 *  Author: b.jamin (simplified by AI)
 */

#ifndef LORA_BASE_H
#define LORA_BASE_H

#ifdef __cplusplus
extern "C" {
#endif

/* System includes */
#include "../../Drivers/lr11xx_driver/src/lr11xx_hal.h"
#include "../../Drivers/lr11xx_driver/src/lr11xx_system.h"
#include "../../Drivers/lr11xx_driver/src/lr11xx_radio.h"
#include "../../Drivers/lr11xx_driver/src/lr11xx_regmem.h"

/* Public functions - SIMPLIFIED INTERFACE */

/**
 * @brief Initialize the LoRa system (called once from main)
 */
void lora_system_init(void);

/**
 * @brief Main process loop for transmitting (called continuously from main)
 */
void lora_system_process(void);

/**
 * @brief Change transmit power level (called from button press)
 * @param boost: 0 = no change, >0 = increment power level
 * @return current power level index (0-4)
 */
uint8_t lora_system_transmit_power(uint8_t boost);

#ifdef __cplusplus
}
#endif

#endif  // LORA_BASE_H

