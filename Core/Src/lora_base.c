/*
 * lora_base.c
 *
 *  Created on: Feb 27, 2025
 *  Author: b.jamin
 */

#include "lora_base.h"
#include "main.h" // for GPIO Mappings and SPI handle

/* Private includes */
#include <stdio.h>
#include <string.h>

/* External variables */
extern SPI_HandleTypeDef hspi1;

/* Private structures */
typedef enum Mode_e {
  MODE_NODE,
  MODE_HUB,
} Mode;

typedef struct transfer_status_s
{
    uint16_t packetSource;
    uint8_t fileType;
    uint8_t fileVersion;
    uint16_t fileId;
    uint16_t packetId;
    uint8_t packetLength;
    uint8_t rssi; // -rssi 
} TransferStatus;

typedef struct transmission_metrics_s
{
    uint16_t tx_cnt;
    uint16_t rx_cnt;
    uint16_t rx_errors;

    uint16_t rx_timeouts;
    uint16_t cad_delay;
    uint8_t invalid_data_last_packet;
    uint32_t invalid_data_accumulator;
    uint32_t last_complete_file_capture_time;
    uint32_t last_file_capture_start_time;
    uint16_t tx_complete_files;

    int8_t power; 
    int8_t max_rssi; // rssi_pkt_in_dbm
    int8_t min_rssi;
    int8_t max_snr; // snr_pkt_in_db
    int8_t min_snr;
    int8_t max_sig_rssi; //signal_rssi_pkt_in_dbm
    int8_t min_sig_rssi;

} TransmissionMetrics;

/* Private function prototypes */
void lr1121_init(void);
void rx_common(void);

// Simple print function for debugging
void simple_print(char* message);
void print_received_data(uint8_t* data, uint8_t length);

void init_tx_data(void);
void increment_tx_data(void);
void tx_data(void);
void rx_listen_for_response(void);
void rx_response_process(void);

void rx_init(void);
void rx_read_data(void);
void build_packet_response(void);
void tx_response(void);

void ui_show_tx(void);
void ui_show_rx(void);
void ui_show_scan(void);

/* Private variables */
uint8_t lora_radio;
uint8_t lora_mode;

uint8_t lora_sf_2p4G;
uint8_t lora_bw_2p4G;
uint8_t lora_cr_2p4G;
uint8_t lora_power_2p4G;
uint16_t lora_freq_2p4G = 2444;

uint8_t lora_sf_1p9G;
uint8_t lora_bw_1p9G;
uint8_t lora_cr_1p9G;
uint8_t lora_power_1p9G;
uint16_t lora_freq_1p9G = 1900;

uint8_t lora_sf;
uint8_t lora_bw;
uint8_t lora_cr;
uint8_t lora_power;
uint16_t lora_freq = 915;

uint16_t errors;

uint32_t Irq_Status;
lr11xx_radio_pkt_type_t rx_packet_type;
lr11xx_radio_stats_lora_t rx_radio_stats;
lr11xx_radio_pkt_status_lora_t rx_packet_stats;
lr11xx_radio_rx_buffer_status_t rx_buffer_status;

TransmissionMetrics fileMetricsHistory[10];
uint8_t fileMetricsCurrentRun = 0;

uint8_t enable_ui_scan = 0;
uint8_t enable_ui_tx = 0;
uint8_t enable_ui_rx = 0;
uint8_t enable_ui_power = 0;

/* Unit variable initialization */

#define PAYLOAD_LENGTH 250

#define BYTES_PER_FILE 131104
#define PACKETS_PER_FILE BYTES_PER_FILE / (PAYLOAD_LENGTH - sizeof(TransferStatus))

#define NODE_MINIMUM_TRANSMISSION_DELAY 10000 // ms delay to wait if trasmissions aren't landing
#define NODE_ATTEMPTS_TO_REACH_HUB_BEFORE_WAIT 10
#define NODE_IRQ_WATCHDOG_MS 10000
#define NODE_RXTX_MODE_SWITCH_TIME_MS 55

#define HUB_IRQ_WATCHDOG_MS 5 * 60000
#define HUB_RXTX_MODE_SWITCH_TIME_MS 55

#define SPACE_TIME_FILLER 55

#define LORA_SYNCWORD 0x11

#define IRQ_MASK ( LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_TIMEOUT | \
      LR11XX_SYSTEM_IRQ_HEADER_ERROR | LR11XX_SYSTEM_IRQ_CRC_ERROR | LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR | \
      LR11XX_SYSTEM_IRQ_CAD_DONE | LR11XX_SYSTEM_IRQ_CAD_DETECTED)

lr11xx_radio_pkt_params_lora_t lora_pkt_params_tx = {
    .preamble_len_in_symb = 20,
    .header_type          = LR11XX_RADIO_LORA_PKT_EXPLICIT,
    .pld_len_in_bytes     = 25, // Simple 25-byte packet
    .crc                  = LR11XX_RADIO_LORA_CRC_ON, 
    .iq                   = LR11XX_RADIO_LORA_IQ_INVERTED,
};

lr11xx_radio_pkt_params_lora_t lora_pkt_params_rx = {
    .preamble_len_in_symb = 20,
    .header_type          = LR11XX_RADIO_LORA_PKT_EXPLICIT,
    .pld_len_in_bytes     = 25, // Simple 25-byte packet
    .crc                  = LR11XX_RADIO_LORA_CRC_ON,
    .iq                   = LR11XX_RADIO_LORA_IQ_INVERTED,
};

static lr11xx_radio_pa_cfg_t pa_config_2p4G = {
    .pa_sel = LR11XX_RADIO_PA_SEL_HF, 
    .pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG, 
    .pa_duty_cycle = 0x00, 
    .pa_hp_sel = 0x00 
};

static lr11xx_radio_pa_cfg_t pa_config_subGHz = {
    .pa_sel = LR11XX_RADIO_PA_SEL_HP,         				//!< Power Amplifier selection
    .pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VBAT,       //!< Power Amplifier regulator supply source
    .pa_duty_cycle = 0x04,  								//!< Power Amplifier duty cycle (Default 0x04)
    .pa_hp_sel = 0x07      									//!< Number of slices for HPA (Default 0x07)
};	
	
lr11xx_radio_mod_params_lora_t lora_mod_params = {   // best combo SF5 BW_250/BW_400/BW_800 CR_4_7 
    .sf = LR11XX_RADIO_LORA_SF5,            //!< LoRa spreading factor
    .bw = LR11XX_RADIO_LORA_BW_500,         //!< LoRa bandwidth
    .cr = LR11XX_RADIO_LORA_CR_4_7,         // LR11XX_RADIO_LORA_CR_LI_4_8 , LR11XX_RADIO_LORA_CR_4_5,    //!< LoRa coding rate 
    .ldro  = 0x00                           //!< LoRa LDRO - Low Data Rate Optimization
};

lr11xx_radio_cad_params_t cadParams = { 
    .cad_symb_nb = 10,
    .cad_detect_peak = 0x32,
    .cad_detect_min = 0x0A,
    .cad_exit_mode = LR11XX_RADIO_CAD_EXIT_MODE_TX,
    .cad_timeout = 1800
}; 

int8_t power_table_2p4G[10] = {-18, -15, -12, -9, -6, -3, 0, 4, 7, 13};
int8_t power_table_1p9G[10] = {-18, -15, -12, -9, -6, -3, 0, 4, 7, 13};
int8_t power_table[10]      = {-5, -2, 1, 4, 7, 10, 13, 16, 19, 22};

uint8_t tx_buf[PAYLOAD_LENGTH];
uint8_t rx_buf[PAYLOAD_LENGTH];

Mode mode;
uint8_t sizeOfTransferStatusInPayload = sizeof(TransferStatus);
TransferStatus transferStatus = { 0 };
TransmissionMetrics tcvrMetrics = { 0 };
uint32_t last_irq_action;

uint8_t is_initialized = 0;
uint8_t is_rx_pending = 0;
uint8_t is_retransmit_requested = 0;
uint32_t transaction_resume_time = 0;

// Node Only
uint8_t is_file_in_progress = 0;
uint8_t hub_no_response_counter = NODE_ATTEMPTS_TO_REACH_HUB_BEFORE_WAIT;
    
/* Unit logic */

void lora_system_init(void){

    mode = MODE_NODE;
    if (HAL_GetUIDw0() == 4063308){
        /*
            Hub and Node Selection based on STM32 serial number of boards used in development
            PI HAL_GetUIDw2()  thru HAL_GetUIDw0() : 540618828 :: 1362120706 :: 4063308	
            STM32 HAL_GetUIDw2() thru HAL_GetUIDw0() : 540618828 :: 1362120710 :: 2162755	
        */
        mode = MODE_HUB;
    }
    #define PI // overrides the serial number based hub assignment above
	#ifdef PI
    mode = MODE_HUB;
	#endif

    // Print startup message
    if (mode == MODE_HUB) {
        simple_print("\n\n=== LoRa Hub Mode - Test Binary Data Receiver ===\n");
        simple_print("Frequency: 2444 MHz (2.4GHz ISM band)\n");
        simple_print("Waiting for test packets from transmitter...\n");
        simple_print("Will display received binary patterns in HEX, DEC, and BIN formats\n\n");
    } else {
        simple_print("\n\n=== LoRa Node Mode - Test Binary Data Transmitter ===\n");
        simple_print("Frequency: 2444 MHz (2.4GHz ISM band)\n");  
        simple_print("Transmitting test binary patterns:\n");
        simple_print("- Pattern 0: Alternating 0xAA/0x55\n");
        simple_print("- Pattern 1: Counting sequence\n");  
        simple_print("- Pattern 2: Binary bit shift\n");
        simple_print("- Pattern 3: XOR with timestamp\n");
        simple_print("Each packet: 25 bytes (9 header + 16 data)\n");
        simple_print("Press button to cycle power levels\n\n");
    }

    lora_power_2p4G = 2;
    lora_power_1p9G = 2;
    lora_power = 2;

}

void lora_system_process(void){

    /*
        Node
            CAD
            Tx
            Rx Response
                Retransmit or Transmit Next
            Rx
            if in Voltage Transfer : Tell Other Nodes to Wait
            Tx pack stat - {serial, file id, packet id, bytes rcvd, CRC} 
            Rx
    */

    switch (mode) {
        case MODE_NODE : 

            if (transaction_resume_time > HAL_GetTick()) {
                // wait ... Add Transaction Resume Time to prevent holding up system on long delay
                // alt HAL_Delay(NODE_MINIMUM_TRANSMISSION_DELAY);
                
            } else if (is_initialized == 0) {
                is_initialized = 1;
                lr1121_init();
                if (is_file_in_progress == 0){
                    init_tx_data();
                }
                tx_data();
            } else if ( HAL_GPIO_ReadPin(LR_IRQ_GPIO_Port, LR_IRQ_Pin) == 1 ) {
                if (is_rx_pending){
                    is_rx_pending = 0;
                    rx_response_process();
                    // TODO : For Readability - handling to prepare next packet here 
                    if (is_file_in_progress == 1){
                        HAL_Delay(NODE_RXTX_MODE_SWITCH_TIME_MS); // lack of delays here cause CRC errors    
                        tx_data();
                        HAL_Delay(NODE_RXTX_MODE_SWITCH_TIME_MS); // lack of delays here cause CRC errors    
                    } else {
                        transaction_resume_time = HAL_GetTick() + (10 * NODE_MINIMUM_TRANSMISSION_DELAY);
                        lr11xx_system_sleep_cfg_t sleep_config = {
                            .is_rtc_timeout = 0,
                            .is_warm_start = 1 
                        };
                        lr11xx_system_set_sleep( NULL, sleep_config, 0 );
                    }
                } else {
                    is_rx_pending = 1;		
                    // HAL_Delay(NODE_RXTX_MODE_SWITCH_TIME_MS);   
                    rx_listen_for_response();
                }

                last_irq_action = HAL_GetTick();
            } else if (HAL_GetTick() > last_irq_action + NODE_IRQ_WATCHDOG_MS ){
                is_initialized = 0;
                last_irq_action = HAL_GetTick();
            }

            break;

        case MODE_HUB : 
            if (transaction_resume_time > HAL_GetTick()) {
                // wait : used for power change from ui button
            } else if (is_initialized == 0){
                is_initialized = 1;
                lr1121_init();
                rx_init();
            } else if ( HAL_GPIO_ReadPin(LR_IRQ_GPIO_Port, LR_IRQ_Pin) == 1 ){
                if (is_rx_pending){
                    is_rx_pending = 0;		
                    rx_read_data();    
                    // TODO : no response if read times out or non system packet received
                    build_packet_response();
                    HAL_Delay(HUB_RXTX_MODE_SWITCH_TIME_MS);
                    tx_response();
                } else {
                    is_rx_pending = 1;		
                    rx_init(); // setup rx mode
                }

                last_irq_action = HAL_GetTick();

            } else if (HAL_GetTick() > last_irq_action + HUB_IRQ_WATCHDOG_MS ) {
                is_initialized = 0;
                last_irq_action = HAL_GetTick();
            }

            break;

        default :
            break;
    }
}

/* 
    takes 0 to maintain power level or 1 or more to increase power level
    returns power level table id
*/
uint8_t lora_system_transmit_power(uint8_t boost){
    if (boost > 0){
        lora_power_2p4G = (lora_power_2p4G + 1) % 10;
        lora_power_1p9G = (lora_power_1p9G + 1) % 10;
        lora_power = (lora_power + 1) % 10;
        is_initialized = 0;
    }
    transaction_resume_time = HAL_GetTick() + NODE_MINIMUM_TRANSMISSION_DELAY;
    return lora_power_2p4G;
}

void lr1121_init(void) {
    // system init common for both rx and tx

    HAL_GPIO_WritePin(LR_RESET_GPIO_Port, LR_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LR_NSS_GPIO_Port, LR_NSS_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LR_CS_GPIO_Port, LR_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    lr11xx_hal_reset(NULL);

    lr11xx_system_set_reg_mode(NULL,LR11XX_SYSTEM_REG_MODE_DCDC); // DC-DC

    lr11xx_system_version_t hw_version;
    lr11xx_system_get_version(NULL, &hw_version);

	lr11xx_system_cfg_lfclk(NULL, LR11XX_SYSTEM_LFCLK_XTAL, true);

	// radio init
	lr11xx_radio_set_pkt_type(NULL, LR11XX_RADIO_PKT_TYPE_LORA);
    lora_radio = 1;

    uint32_t baseFrequency;
    if(lora_radio==1) baseFrequency = lora_freq_2p4G * 1000000; 
    if(lora_radio==0) baseFrequency = lora_freq * 1000000; 
    if(lora_radio==2) baseFrequency = lora_freq_1p9G * 1000000;
	lr11xx_radio_set_rf_freq(NULL, baseFrequency);

    int8_t selectedPower;
    if ( lora_radio == 1) { 
        lr11xx_radio_set_pa_cfg(NULL, &pa_config_2p4G);	
        selectedPower = power_table_2p4G[lora_power_2p4G];
        lr11xx_radio_set_tx_params(NULL, selectedPower, LR11XX_RADIO_RAMP_48_US );	// // range [-17, +22] for sub-G, range [-18, 13] for 2.4G ( HF_PA ) 
    } else if(lora_radio == 0) { 
        lr11xx_radio_set_pa_cfg(NULL, &pa_config_subGHz);	
        selectedPower = power_table[lora_power];
        lr11xx_radio_set_tx_params(NULL, selectedPower, LR11XX_RADIO_RAMP_48_US );
    } else if(lora_radio == 2) { 
        lr11xx_radio_set_pa_cfg(NULL, &pa_config_2p4G);	
        selectedPower = power_table_1p9G[lora_power_1p9G];
        lr11xx_radio_set_tx_params(NULL, selectedPower, LR11XX_RADIO_RAMP_48_US );
    } else {
        lr11xx_radio_set_pa_cfg(NULL, &pa_config_2p4G);	
        selectedPower = power_table_2p4G[lora_power_2p4G];
        lr11xx_radio_set_tx_params(NULL, selectedPower, LR11XX_RADIO_RAMP_48_US );
    }

    lr11xx_radio_set_rx_tx_fallback_mode(NULL, LR11XX_RADIO_FALLBACK_FS); // LR11XX_RADIO_FALLBACK_STDBY_RC
    lr11xx_radio_cfg_rx_boosted(NULL, 0x01); // Enabled : 0x01 / Disabled : 0x00

    lr11xx_radio_set_lora_mod_params(NULL, &lora_mod_params);
    lr11xx_radio_set_lora_sync_word(NULL, LORA_SYNCWORD);    

    lr11xx_system_clear_errors(NULL);

    lr11xx_system_calibrate(NULL, 0x3f);
    lr11xx_system_get_errors(NULL, &errors);
    lr11xx_system_clear_errors(NULL);

    lr11xx_system_clear_irq_status(NULL, LR11XX_SYSTEM_IRQ_ALL_MASK);

    // air_time=lr11xx_radio_get_lora_time_on_air_in_ms(&lora_pkt_params,&lora_mod_params);
    // air_time=air_time*4;
}

void rx_common(void) {
    lr11xx_system_get_and_clear_irq_status(NULL, &Irq_Status);
    lr11xx_system_clear_irq_status(NULL, LR11XX_SYSTEM_IRQ_ALL_MASK);

    lr11xx_radio_get_pkt_type(NULL, &rx_packet_type);
    tcvrMetrics.power = power_table_2p4G[lora_power_2p4G];; 

    lr11xx_radio_get_lora_stats(NULL, &rx_radio_stats);
    lr11xx_radio_get_lora_pkt_status( NULL, &rx_packet_stats );
    lr11xx_radio_get_rx_buffer_status( NULL, &rx_buffer_status );
    lr11xx_radio_reset_stats(NULL);
    
    if (tcvrMetrics.max_rssi < rx_packet_stats.rssi_pkt_in_dbm || tcvrMetrics.max_rssi == 0){
        tcvrMetrics.max_rssi = rx_packet_stats.rssi_pkt_in_dbm;
    }
    if (tcvrMetrics.min_rssi > rx_packet_stats.rssi_pkt_in_dbm || tcvrMetrics.min_rssi == 0){
        tcvrMetrics.min_rssi = rx_packet_stats.rssi_pkt_in_dbm;
    }

    if (tcvrMetrics.max_sig_rssi < rx_packet_stats.signal_rssi_pkt_in_dbm || tcvrMetrics.max_sig_rssi == 0){
        tcvrMetrics.max_sig_rssi = rx_packet_stats.signal_rssi_pkt_in_dbm;
    }
    if (tcvrMetrics.min_sig_rssi > rx_packet_stats.signal_rssi_pkt_in_dbm || tcvrMetrics.min_sig_rssi == 0){
        tcvrMetrics.min_sig_rssi = rx_packet_stats.signal_rssi_pkt_in_dbm;
    }

    if (tcvrMetrics.max_snr < rx_packet_stats.snr_pkt_in_db || tcvrMetrics.max_snr == 0){
        tcvrMetrics.max_snr = rx_packet_stats.snr_pkt_in_db;
    }
    if (tcvrMetrics.min_snr > rx_packet_stats.snr_pkt_in_db || tcvrMetrics.min_snr == 0){
        tcvrMetrics.min_snr = rx_packet_stats.snr_pkt_in_db;
    }
}

/* Node Functions */
void tx_data(void) {
    lr11xx_system_get_and_clear_irq_status(NULL, &Irq_Status);
	lr11xx_system_clear_irq_status(NULL, LR11XX_SYSTEM_IRQ_ALL_MASK ); 
	lr11xx_system_set_dio_irq_params(NULL, IRQ_MASK, 0 );

	lr11xx_radio_set_lora_pkt_params(NULL, &lora_pkt_params_tx );
    lr11xx_regmem_write_buffer8(NULL, tx_buf, lora_pkt_params_tx.pld_len_in_bytes); // (offset,*data,length) lora_pkt_params.pld_len_in_bytes=PAYLOAD_LENGTH;

    // Print transmission info
    char buffer[256];
    sprintf(buffer, "\n=== TRANSMITTING TEST DATA ===\n");
    simple_print(buffer);
    sprintf(buffer, "File ID: %d, Packet ID: %d\n", transferStatus.fileId, transferStatus.packetId);
    simple_print(buffer);
    sprintf(buffer, "Binary test pattern (16 bytes):\n");
    simple_print(buffer);
    sprintf(buffer, "HEX: ");
    simple_print(buffer);
    for(int i = 9; i < 25; i++) {
        sprintf(buffer, "%02X ", tx_buf[i]);
        simple_print(buffer);
    }
    sprintf(buffer, "\nDEC: ");
    simple_print(buffer);
    for(int i = 9; i < 25; i++) {
        sprintf(buffer, "%03d ", tx_buf[i]);
        simple_print(buffer);
    }
    sprintf(buffer, "\nBIN: ");
    simple_print(buffer);
    for(int i = 9; i < 25; i++) {
        for(int bit = 7; bit >= 0; bit--) {
            sprintf(buffer, "%d", (tx_buf[i] >> bit) & 1);
            simple_print(buffer);
        }
        simple_print(" ");
    }
    simple_print("\n===============================\n");

    ui_show_scan();
    // consider lr11xx_radio_auto_tx_rx(NULL,); // but may not work with CAD and may affect interrupt based handling
    lr11xx_radio_set_cad_params(NULL, &cadParams);
    // lr11xx_radio_set_tx_with_timeout_in_rtc_step(NULL, 2000); // implied in cadParams 
    lr11xx_radio_set_cad(NULL);
    // delay post CAD prevents CRC errors
    ui_show_tx(); 
    tcvrMetrics.tx_cnt++;
}

void rx_listen_for_response(void){
	lr11xx_system_clear_irq_status(NULL, LR11XX_SYSTEM_IRQ_ALL_MASK );
	lr11xx_system_set_dio_irq_params(NULL, IRQ_MASK, 0 ); 
	lr11xx_radio_set_lora_pkt_params(NULL, &lora_pkt_params_rx );

	lr11xx_radio_set_rx(NULL, 2000);
    ui_show_rx();
    is_rx_pending = 1;
};

void rx_response_process(void){
    rx_common();
    ui_show_rx();

    uint8_t isRx = 0;
    uint8_t isError = 0;

    if( ( (Irq_Status & LR11XX_SYSTEM_IRQ_RX_DONE) == LR11XX_SYSTEM_IRQ_RX_DONE) && ( (Irq_Status & LR11XX_SYSTEM_IRQ_CRC_ERROR ) == 0) ) {
        lr11xx_regmem_read_buffer8( NULL, rx_buf, rx_buffer_status.buffer_start_pointer, rx_buffer_status.pld_len_in_bytes );
        /* 
            TODO : verify response packet is for corrent source, file, packet and packet size 
            packet number and length should mis-match == error
            sourceId, fileId, fileTypeId mismatch == wait, packet to / from other device   
        */
        if (rx_buf[6] == (uint8_t) (transferStatus.packetId & 0xFF) && rx_buf[7] == (uint8_t) (transferStatus.packetId >> 8) ){
            if (rx_buf[8] == transferStatus.packetLength){
                increment_tx_data();
                isRx = 1;
                ui_show_rx();
            } else {
                isRx = 1;
                isError = 1;
            }
        } else {
            isRx = 1;
            isError = 1;
        }
    } else if( ( (Irq_Status & LR11XX_SYSTEM_IRQ_TIMEOUT) == LR11XX_SYSTEM_IRQ_TIMEOUT)) {
        tcvrMetrics.rx_timeouts++;
    } else if( (Irq_Status & LR11XX_SYSTEM_IRQ_CRC_ERROR ) == LR11XX_SYSTEM_IRQ_CRC_ERROR) {
        isRx = 1;
        isError = 1;
    } else if( (Irq_Status & LR11XX_SYSTEM_IRQ_CAD_DONE ) == LR11XX_SYSTEM_IRQ_CAD_DONE && (Irq_Status & LR11XX_SYSTEM_IRQ_CAD_DETECTED ) == LR11XX_SYSTEM_IRQ_CAD_DETECTED) {
        tcvrMetrics.cad_delay++;
        transaction_resume_time = HAL_GetTick() + NODE_MINIMUM_TRANSMISSION_DELAY;
    } else {
        isError = 1;
    }
    
    if (isRx > 0){
        tcvrMetrics.rx_cnt++;
        hub_no_response_counter = NODE_ATTEMPTS_TO_REACH_HUB_BEFORE_WAIT; // reload
    } else {
        hub_no_response_counter--;
    }

    if (hub_no_response_counter == 0){
        // TODO : Add wait if no packets have been transferred (w/ response) in x attempts 
        transaction_resume_time = HAL_GetTick() + NODE_MINIMUM_TRANSMISSION_DELAY;
        hub_no_response_counter = NODE_ATTEMPTS_TO_REACH_HUB_BEFORE_WAIT;  // reload
    }

    if (isError > 0){
        tcvrMetrics.rx_errors++;
    }

    // TODO : is_retransmit_requested = 1;
    lr11xx_regmem_clear_rxbuffer(NULL);
};

void init_tx_data(void) {
    transferStatus.packetSource = (uint16_t) HAL_GetUIDw0() & 0xFFFF;
    transferStatus.fileType = 1; // uint8_t
    transferStatus.fileVersion = 2; // uint8_t
    transferStatus.fileId++; // uint16_t
    transferStatus.packetId = 0; // uint16_t
    transferStatus.packetLength = 25; // Simple packet: 9 bytes header + 16 bytes data

    tx_buf[0] = (uint8_t) transferStatus.packetSource & 0xFF;
    tx_buf[1] = (uint8_t) (transferStatus.packetSource >> 8);
    tx_buf[2] = transferStatus.fileType;
    tx_buf[3] = transferStatus.fileVersion;
    tx_buf[4] = (uint8_t) transferStatus.fileId & 0xFF;
    tx_buf[5] = (uint8_t) (transferStatus.fileId >> 8);
    tx_buf[6] = (uint8_t) transferStatus.packetId & 0xFF;
    tx_buf[7] = (uint8_t) (transferStatus.packetId >> 8);
    tx_buf[8] = transferStatus.packetLength;

    // Generate test binary string pattern
    uint32_t seed = HAL_GetTick();
    for(uint16_t itt = 0; itt < 16; itt++){
        // Create a repeating binary pattern for easier testing
        if (itt < 4) {
            tx_buf[9 + itt] = 0xAA; // 10101010 pattern
        } else if (itt < 8) {
            tx_buf[9 + itt] = 0x55; // 01010101 pattern  
        } else if (itt < 12) {
            tx_buf[9 + itt] = 0xFF; // 11111111 pattern
        } else {
            tx_buf[9 + itt] = (uint8_t)(seed + itt) % 256; // Some variation
        }
    }
    
    is_file_in_progress = 0; // Single packet mode
    tcvrMetrics.last_file_capture_start_time = HAL_GetTick();
}

void increment_tx_data(void) {
    // Create different test patterns for consecutive transmissions
    uint8_t pattern_type = transferStatus.fileId % 4;
    
    // Keep the same header, just change the data pattern
    transferStatus.packetId = 0; // Reset for next transmission
    
    // Generate different binary test patterns based on transmission count
    for(uint16_t itt = 0; itt < 16; itt++){
        switch(pattern_type) {
            case 0: // Alternating pattern
                tx_buf[9 + itt] = (itt % 2 == 0) ? 0xAA : 0x55;
                break;
            case 1: // Counting pattern
                tx_buf[9 + itt] = itt * 16;
                break;
            case 2: // Binary shift pattern
                tx_buf[9 + itt] = 1 << (itt % 8);
                break;
            case 3: // XOR pattern with timestamp
                tx_buf[9 + itt] = (uint8_t)((HAL_GetTick() + itt) ^ 0xC3);
                break;
        }
    }
    
    simple_print("Pattern changed - waiting 3 seconds before next transmission...\n");
    HAL_Delay(3000); // Wait 3 seconds between different pattern transmissions
    is_file_in_progress = 0; // Single packet mode
}

/* Hub Functions */
void rx_init(void) {
	lr11xx_system_clear_irq_status(NULL, LR11XX_SYSTEM_IRQ_ALL_MASK );
	lr11xx_system_set_dio_irq_params(NULL, IRQ_MASK, 0 ); 
	lr11xx_radio_set_lora_pkt_params(NULL, &lora_pkt_params_rx );

	lr11xx_radio_set_rx(NULL, 0); // timeout = 0 - wait forever
    ui_show_rx();

    is_rx_pending = 1;
}

void rx_read_data(void) {
    rx_common();
    
    tcvrMetrics.invalid_data_last_packet = 0;
    if( ( (Irq_Status & LR11XX_SYSTEM_IRQ_RX_DONE) == LR11XX_SYSTEM_IRQ_RX_DONE) && ( (Irq_Status & LR11XX_SYSTEM_IRQ_CRC_ERROR ) == 0) ) {

        lr11xx_regmem_read_buffer8( NULL, rx_buf, rx_buffer_status.buffer_start_pointer, rx_buffer_status.pld_len_in_bytes );

        // Print the received data to serial/UART
        print_received_data(rx_buf, rx_buffer_status.pld_len_in_bytes);

        /* validate received packet - simplified for debugging */
        tcvrMetrics.rx_cnt++;
        is_retransmit_requested = 0; // Always acknowledge for simple debugging

    } else if( (Irq_Status & LR11XX_SYSTEM_IRQ_TIMEOUT) == LR11XX_SYSTEM_IRQ_TIMEOUT) {
        simple_print("RX TIMEOUT\n");
        is_retransmit_requested = 2;
        tcvrMetrics.rx_errors++;
        tcvrMetrics.rx_cnt++;
    } else if( (Irq_Status & LR11XX_SYSTEM_IRQ_CRC_ERROR ) == LR11XX_SYSTEM_IRQ_CRC_ERROR) {
        simple_print("CRC ERROR\n");
        is_retransmit_requested = 2;
        tcvrMetrics.rx_errors++;
        tcvrMetrics.rx_cnt++;
    } else {
        simple_print("OTHER RX ERROR\n");
        is_retransmit_requested = 2;
        tcvrMetrics.rx_errors++;
        tcvrMetrics.rx_cnt++;
    }

    if (is_retransmit_requested == 0){
        ui_show_rx();
    } else {
        ui_show_scan();
    }

    lr11xx_regmem_clear_rxbuffer(NULL);    
}

void build_packet_response(void){
    if (is_retransmit_requested < 2){
        // packet received was valid but may have data errors
        transferStatus.packetSource = rx_buf[0] + (rx_buf[1] << 8);
        transferStatus.fileType = rx_buf[2];
        transferStatus.fileVersion = rx_buf[3];
        transferStatus.fileId = rx_buf[4] + (rx_buf[5] << 8);

        transferStatus.packetLength = rx_buf[8] - tcvrMetrics.invalid_data_last_packet;
        transferStatus.rssi =  - (rx_packet_stats.signal_rssi_pkt_in_dbm); // alt values to track : snr_pkt_in_db rssi_pkt_in_dbm

        transferStatus.packetId = rx_buf[6] + (rx_buf[7] << 8);
        if (transferStatus.packetLength == rx_buf[8]) {

            if (transferStatus.packetId == 0 && tcvrMetrics.last_file_capture_start_time == 0){

                fileMetricsHistory[fileMetricsCurrentRun] = tcvrMetrics; 
                fileMetricsCurrentRun = (fileMetricsCurrentRun + 1) % 10;

                // init new file transmission
                tcvrMetrics.last_file_capture_start_time = HAL_GetTick();
                tcvrMetrics.max_rssi = 0;
                tcvrMetrics.min_rssi = 0;
                tcvrMetrics.max_sig_rssi = 0;
                tcvrMetrics.min_sig_rssi = 0;
                tcvrMetrics.max_snr = 0;
                tcvrMetrics.min_snr = 0;
            }

            if (transferStatus.packetId >= PACKETS_PER_FILE - 1){
                if (tcvrMetrics.last_file_capture_start_time > 0){
                    tcvrMetrics.last_complete_file_capture_time = HAL_GetTick() - tcvrMetrics.last_file_capture_start_time; 
                    tcvrMetrics.tx_complete_files++;
                    tcvrMetrics.last_file_capture_start_time = 0;                
                }
            }
        }  
    } else {
        // packet received was invalid
        transferStatus.packetLength = 0;
    }
}

void tx_response(void){
	lr11xx_system_clear_irq_status(NULL, LR11XX_SYSTEM_IRQ_ALL_MASK ); 
	lr11xx_system_set_dio_irq_params(NULL, IRQ_MASK, 0 );

    lora_pkt_params_tx.pld_len_in_bytes = sizeOfTransferStatusInPayload;
	lr11xx_radio_set_lora_pkt_params(NULL, &lora_pkt_params_tx );
    uint8_t* responseBuffer = (uint8_t*) &transferStatus;

    lr11xx_regmem_write_buffer8(NULL, responseBuffer, lora_pkt_params_tx.pld_len_in_bytes); // (offset,*data,length) lora_pkt_params.pld_len_in_bytes=PAYLOAD_LENGTH;
    // TODO : consider using : lr11xx_radio_set_rx_tx_fallback_mode - to jump back to Rx Mode, but only works if packet parameters in Tx and Rx match
    lr11xx_radio_set_tx_with_timeout_in_rtc_step(NULL, 1000);

    ui_show_tx();
    tcvrMetrics.tx_cnt++;
}

void ui_show_tx(void){
    enable_ui_tx = 1;
}

void ui_show_rx(void){
    enable_ui_rx = 1;
}

void ui_show_scan(void){
    enable_ui_scan = 1;
}

// Simple print functions for debugging
void simple_print(char* message) {
    // For now, this is a placeholder
    // In embedded systems, you would typically use:
    // HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);
    // or printf() if you have retarget setup
    // Since we can see output on your Pi, the USB CDC is working
    printf("%s", message);
}

void print_received_data(uint8_t* data, uint8_t length) {
    char buffer[512];
    char temp[16];
    
    sprintf(buffer, "\n=== RECEIVED DATA ===\n");
    simple_print(buffer);
    
    sprintf(buffer, "Length: %d bytes\n", length);
    simple_print(buffer);
    
    sprintf(buffer, "Source ID: %d\n", data[0] | (data[1] << 8));
    simple_print(buffer);
    
    sprintf(buffer, "File Type: %d\n", data[2]);
    simple_print(buffer);
    
    sprintf(buffer, "File ID: %d\n", data[4] | (data[5] << 8));
    simple_print(buffer);
    
    simple_print("Data payload (16 bytes): ");
    for(int i = 9; i < length && i < 25; i++) {
        sprintf(temp, "%02X ", data[i]);
        simple_print(temp);
    }
    simple_print("\n");
    
    simple_print("Data as decimal: ");
    for(int i = 9; i < length && i < 25; i++) {
        sprintf(temp, "%d ", data[i]);
        simple_print(temp);
    }
    simple_print("\n====================\n\n");
}


