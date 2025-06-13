/*
 * ESP32-C6 Modbus Master - VERSIONE FINALE FUNZIONANTE
 * Con controllo DE/RE manuale e filtro echo
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "sdkconfig.h"

#define MB_PORT_NUM     (1)      // UART1
#define MB_DEV_SPEED    (19200)  // 19200 baud
#define MB_UART_TXD     (4)      // TX pin (GPIO4)
#define MB_UART_RXD     (5)      // RX pin (GPIO5)
#define DE_PIN          (2)      // Data Enable pin

// Slave device address
#define FRIDGE_SLAVE_ADDR   (197)

// Register to read (2013)
#define TARGET_REGISTER     2013     // Registro corretto!

static const char *TAG = "FRIDGE_MASTER";

// Storage for register value
uint16_t evap_pressure_value = 0;
bool data_valid = false;

// Initialize hardware
void init_hardware(void) {
    ESP_LOGI(TAG, "Inizializzazione hardware...");
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = MB_DEV_SPEED,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_param_config(MB_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(MB_PORT_NUM, MB_UART_TXD, MB_UART_RXD, 
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(MB_PORT_NUM, 512, 512, 0, NULL, 0));
    
    // Configure DE pin
    gpio_reset_pin(DE_PIN);
    gpio_set_direction(DE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DE_PIN, 0); // Start in RX mode
    
    ESP_LOGI(TAG, "Hardware inizializzato: %d baud, TX=%d, RX=%d, DE=%d", 
             MB_DEV_SPEED, MB_UART_TXD, MB_UART_RXD, DE_PIN);
}

// Calculate Modbus RTU CRC16
uint16_t calculate_crc16(uint8_t *data, int length) {
    uint16_t crc = 0xFFFF;
    
    for (int i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

// Send Modbus request and receive response
esp_err_t modbus_read_holding_register(uint8_t slave_addr, uint16_t reg_addr, uint16_t *value) {
    ESP_LOGI(TAG, "Lettura registro %d da slave %d...", reg_addr, slave_addr);
    
    // Build Modbus RTU request
    uint8_t request[8];
    request[0] = slave_addr;                    // Slave address
    request[1] = 0x03;                          // Function code (Read Holding Registers)
    request[2] = (reg_addr) >> 8;           // Register address high byte (0-based)
    request[3] = (reg_addr) & 0xFF;         // Register address low byte
    request[4] = 0x00;                          // Number of registers high byte
    request[5] = 0x01;                          // Number of registers low byte (1 register)
    
    // Calculate and add CRC
    uint16_t crc = calculate_crc16(request, 6);
    request[6] = crc & 0xFF;                    // CRC low byte
    request[7] = (crc >> 8) & 0xFF;             // CRC high byte
    
    // Clear UART buffers
    uart_flush(MB_PORT_NUM);
    uart_flush_input(MB_PORT_NUM);
    
    // Switch to TX mode
    gpio_set_level(DE_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(2)); // 2ms stability delay
    
    // Send request
    ESP_LOGI(TAG, "TX: %02X %02X %02X %02X %02X %02X %02X %02X", 
             request[0], request[1], request[2], request[3],
             request[4], request[5], request[6], request[7]);
    
    int sent = uart_write_bytes(MB_PORT_NUM, (char*)request, sizeof(request));
    uart_wait_tx_done(MB_PORT_NUM, pdMS_TO_TICKS(100));
    
    // Switch to RX mode
    vTaskDelay(pdMS_TO_TICKS(5)); // 5ms delay after TX
    gpio_set_level(DE_PIN, 0);
    
    ESP_LOGI(TAG, "Inviati %d bytes, aspettando risposta...", sent);
    
    // Receive response with echo filtering
    uint8_t rx_buffer[32];
    int total_received = 0;
    bool echo_filtered = false;
    
    // Listen for response (up to 1 second)
    for (int attempt = 0; attempt < 10; attempt++) {
        int received = uart_read_bytes(MB_PORT_NUM, rx_buffer + total_received, 
                                     sizeof(rx_buffer) - total_received, pdMS_TO_TICKS(100));
        
        if (received > 0) {
            total_received += received;
            
            // Print all received data for debugging
            printf("RX attempt %d (%d bytes): ", attempt + 1, received);
            for (int i = total_received - received; i < total_received; i++) {
                printf("%02X ", rx_buffer[i]);
            }
            printf("\n");
            
            // Check if we received echo of our request
            if (!echo_filtered && total_received >= 8) {
                bool is_echo = true;
                for (int i = 0; i < 8; i++) {
                    if (rx_buffer[i] != request[i]) {
                        is_echo = false;
                        break;
                    }
                }
                
                if (is_echo) {
                    ESP_LOGI(TAG, "Echo filtrato, aspettando risposta reale...");
                    echo_filtered = true;
                    total_received = 0; // Reset buffer after echo
                    continue;
                }
            }
            
            // Check for valid Modbus response after echo is filtered
            if (echo_filtered && total_received >= 5) {
                if (rx_buffer[0] == slave_addr && rx_buffer[1] == 0x03) {
                    // Valid response header
                    int expected_length = 5 + rx_buffer[2]; // Header + data + CRC
                    
                    if (total_received >= expected_length) {
                        // Complete response received
                        ESP_LOGI(TAG, "Risposta completa ricevuta (%d bytes)", total_received);
                        
                        // Extract value
                        if (rx_buffer[2] == 2) { // 2 bytes of data
                            *value = (rx_buffer[3] << 8) | rx_buffer[4];
                            ESP_LOGI(TAG, "SUCCESS! Valore registro: %u", *value);
                            return ESP_OK;
                        }
                    }
                }
            }
        }
    }
    
    if (total_received == 0) {
        ESP_LOGW(TAG, "Nessuna risposta ricevuta");
    } else {
        ESP_LOGW(TAG, "Risposta incompleta o non valida (%d bytes)", total_received);
    }
    
    return ESP_ERR_TIMEOUT;
}

void app_main(void) {
    ESP_LOGI(TAG, "===============================================================================");
    ESP_LOGI(TAG, "=== ESP32-C6 MODBUS RTU MASTER - VERSIONE FINALE ===");
    ESP_LOGI(TAG, "===============================================================================");
    ESP_LOGI(TAG, "Target: Slave %d, Register %d", FRIDGE_SLAVE_ADDR, TARGET_REGISTER);
    ESP_LOGI(TAG, "Hardware: TX=GPIO%d, RX=GPIO%d, DE=GPIO%d, RE=GND", 
             MB_UART_TXD, MB_UART_RXD, DE_PIN);
    ESP_LOGI(TAG, "===============================================================================");
    
    // Initialize hardware
    init_hardware();
    
    ESP_LOGI(TAG, "Sistema pronto! Avvio polling...");
    
    // Main loop
    uint32_t poll_count = 0;
    uint32_t success_count = 0;
    
    while (1) {
        poll_count++;
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "=== POLLING #%lu ===", poll_count);
        
        esp_err_t result = modbus_read_holding_register(FRIDGE_SLAVE_ADDR, TARGET_REGISTER, &evap_pressure_value);
        
        if (result == ESP_OK) {
            success_count++;
            data_valid = true;
            ESP_LOGI(TAG, "✅ SUCCESS! Apertura valvola: %u%%", evap_pressure_value);
        } else {
            data_valid = false;
            evap_pressure_value = 0;
            ESP_LOGW(TAG, "❌ FAILED! Errore comunicazione");
        }
        
        // Statistics
        float success_rate = (float)success_count / poll_count * 100.0f;
        ESP_LOGI(TAG, "Statistiche: %lu/%lu riusciti (%.1f%%)", success_count, poll_count, success_rate);
        ESP_LOGI(TAG, "Valore attuale: %u%% %s", evap_pressure_value, data_valid ? "(VALIDO)" : "(NON VALIDO)");
        
        // Wait before next poll
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}