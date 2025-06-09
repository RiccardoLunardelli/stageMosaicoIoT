/**
 * ===============================================================================
 * ESP32-C6 MODBUS RTU MASTER + MQTT - 2.0.0_CLEAN_NO_BIT8
 * ===============================================================================
 * 
 * DESCRIZIONE:
 * Sistema completo di acquisizione dati Modbus RTU con trasmissione MQTT.
 * Legge 8 registri holding (1-8) da dispositivo slave e li trasmette via MQTT
 * con gestione intelligente LED e auto-recovery errori.
 * 
 * HARDWARE SETUP:
 * - ESP32-C6 Development Board
 * - Convertitore USB-RS485 con controllo DE/RE
 * - LED giallo status (GPIO18) + LED rosso errori (GPIO19)
 * 
 * CABLAGGIO:
 * - GPIO4 (TX) → TXD RS485, GPIO5 (RX) ← RXD RS485, GPIO6 → DE/RE
 * - GPIO18 → LED Status, GPIO19 → LED Errori (resistori 220Ω)
 * 
 * CONFIGURAZIONE:
 * - Modbus RTU: 9600 baud, 8-N-1, Slave=1, Registri 1-8
 * - Wi-Fi: LAPTOP_Luna / esp12345
 * - MQTT: mqtt://192.168.137.128:1883
 * - Topic: data_static (10s), status (30s), debug_raw
 * 
 * REGISTRI (1-based addressing):
 * - Reg1: Temperatura/10 (°C), Reg2: Setpoint/10 (°C)
 * - Reg3: Umidità/10 (%), Reg4: Pressione/100 (bar)  
 * - Reg5: Status word, Reg6: Runtime (ore)
 * - Reg7: Power/100 (kW), Reg8: Allarmi 
 * 
 * LED LOGICA:
 * - Giallo: Boot/Connecting/Normal/TX/RS485/Error
 * - Rosso: Wi-Fi down OR MQTT down OR Allarmi Modbus OR Errori RS485
 * 
 * AUTORI: Soufian Markouni e Riccardo Lunardelli
 * DATA: Giugno 2025
 * VERSIONE: 2.0.0_CLEAN_NO_BIT8
 * ===============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include <inttypes.h> 

// ===============================================================================
// CONFIGURAZIONE SISTEMA
// ===============================================================================

#define WIFI_SSID      "LAPTOP_Luna"        // Nome rete Wi-Fi per connessione
#define WIFI_PASSWORD  "esp12345"           // Password rete Wi-Fi
#define MQTT_BROKER_URI "mqtt://192.168.137.128:1883"  // Indirizzo broker MQTT
#define MQTT_CLIENT_ID   "ESP32_FRIGO01_V2_CLEAN"      // ID client univoco MQTT

// ===============================================================================
// TOPIC MQTT PER PUBBLICAZIONE DATI
// ===============================================================================

#define TOPIC_DATA_STATIC    "sensori/frigo01/data_static"   // Dati operativi principali
#define TOPIC_SYSTEM_STATUS  "sensori/frigo01/status"        // Status sistema e statistiche
#define TOPIC_DEBUG_RAW      "sensori/frigo01/debug_raw"     // Valori grezzi registri

// ===============================================================================
// CONFIGURAZIONE HARDWARE
// ===============================================================================

#define LED_PIN          GPIO_NUM_18  // LED giallo stato sistema
#define LED_ERROR_PIN    GPIO_NUM_19  // LED rosso errori/allarmi

// ===============================================================================
// CONFIGURAZIONE RS485
// ===============================================================================

#define RS485_UART_NUM   UART_NUM_1   // Porta UART per comunicazione RS485
#define RS485_TX_PIN     GPIO_NUM_4   // Pin TX per trasmissione dati
#define RS485_RX_PIN     GPIO_NUM_5   // Pin RX per ricezione dati
#define RS485_DE_RE_PIN  GPIO_NUM_6   // Pin controllo direzione DE/RE
#define RS485_BAUD_RATE  9600         // Velocità baud comunicazione
#define RS485_BUF_SIZE   1024         // Dimensione buffer UART

// ===============================================================================
// CONFIGURAZIONE TIMING
// ===============================================================================

#define DATA_READ_INTERVAL    10000  // 10 secondi - intervallo lettura dati
#define STATUS_SEND_INTERVAL  30000  // 30 secondi - intervallo invio status
#define LED_UPDATE_INTERVAL   100    // 100ms - frequenza aggiornamento LED

// ===============================================================================
// CONFIGURAZIONE MODBUS RTU
// ===============================================================================

#define MB_SLAVE_ADDR        1       // Indirizzo dispositivo slave Modbus
#define MB_HOLDING_REG_START 1       // Primo registro (1-based addressing!)
#define MB_HOLDING_REG_COUNT 8       // Numero totale registri da leggere
#define MB_TIMEOUT_MS        1000    // Timeout comunicazione in millisecondi

// ===============================================================================
// STRUTTURE DATI
// ===============================================================================

// Struttura per contenere tutti i parametri del frigorifero
typedef struct {
    float paramET_temp;        // Registro 1: Temperatura evaporatore (°C)
    float paramET_setpoint;    // Registro 2: Setpoint temperatura (°C)  
    float paramET_umid;        // Registro 3: Umidità relativa (%)
    float paramET_pressure;    // Registro 4: Pressione (bar)
    uint16_t paramET_status;   // Registro 5: Status word (bit field)
    uint32_t paramET_runtime;  // Registro 6: Runtime compressore (ore)
    float paramET_power;       // Registro 7: Consumo energia (kW)
    uint16_t paramET_alarms;   // Registro 8: Allarmi attivi (bit field)
} modbus_registers_t;

// Stati del sistema per gestione macchina a stati
typedef enum {
    SYS_INIT,              // Inizializzazione sistema
    SYS_WIFI_CONNECTING,   // Connessione Wi-Fi in corso
    SYS_MQTT_CONNECTING,   // Connessione MQTT in corso
    SYS_RUNNING,           // Sistema operativo normale
    SYS_ERROR_RS485,       // Errore comunicazione RS485
    SYS_ERROR_COMM         // Errore comunicazione generale
} system_state_t;

// Stati LED per indicazione visiva stato sistema
typedef enum {
    LED_BOOT,         // Lampeggio boot/inizializzazione
    LED_CONNECTING,   // Lampeggio connessione
    LED_NORMAL,       // Impulso normale operativo
    LED_DATA_TX,      // Lampeggi trasmissione dati
    LED_ERROR,        // Lampeggio errore
    LED_RS485_COMM    // Lampeggi comunicazione RS485
} led_state_t;

// ===============================================================================
// VARIABILI GLOBALI
// ===============================================================================

static const char *TAG = "MODBUS_V2_CLEAN";  // Tag per logging ESP_LOG
static EventGroupHandle_t s_wifi_event_group; // Event group per gestione Wi-Fi
#define WIFI_CONNECTED_BIT BIT0                // Bit per stato connessione Wi-Fi

// Handle e stati connessioni
esp_mqtt_client_handle_t mqtt_client = NULL;  // Handle client MQTT
bool mqtt_connected = false;                  // Flag stato connessione MQTT
bool wifi_connected = false;                  // Flag stato connessione Wi-Fi

// Stati sistema e LED
system_state_t system_state = SYS_INIT;      // Stato corrente sistema
led_state_t led_state = LED_BOOT;            // Stato corrente LED giallo
bool led_physical_state = false;             // Stato fisico LED giallo
TickType_t led_last_toggle = 0;              // Timestamp ultimo toggle LED
int led_blink_count = 0;                     // Contatore lampeggi LED
bool led_error_state = false;                // Stato LED errore rosso

// Contatori statistiche
unsigned long message_counter = 0;           // Contatore messaggi MQTT inviati
unsigned long rs485_read_counter = 0;        // Contatore tentativi lettura RS485
unsigned long error_counter = 0;             // Contatore errori totali
unsigned long successful_reads = 0;          // Contatore letture RS485 riuscite

// Dati Modbus
modbus_registers_t registers = {             // Struttura parametri convertiti
    .paramET_temp = 0.0f, .paramET_setpoint = 0.0f, .paramET_umid = 0.0f,
    .paramET_pressure = 0.0f, .paramET_status = 0x0000, .paramET_runtime = 0,
    .paramET_power = 0.0f, .paramET_alarms = 0x0000
};

bool data_initialized = false;               // Flag validità dati inizializzati
uint16_t raw_modbus_data[MB_HOLDING_REG_COUNT]; // Array dati grezzi registri
TickType_t last_successful_read = 0;         // Timestamp ultima lettura riuscita

// ===============================================================================
// DICHIARAZIONI FUNZIONI
// ===============================================================================

void init_rs485();
bool read_all_modbus_registers();
void convert_raw_to_registers();
void send_data_static_message();
void send_system_status_message();
void send_debug_raw_message();
void update_system_state(system_state_t new_state);
void update_led_state(led_state_t new_state);
void update_error_led();
void decode_status_register(uint16_t status);
void decode_alarms_register(uint16_t alarms);

// ===============================================================================
// FUNZIONI UTILITÀ
// ===============================================================================

// Genera timestamp ISO 8601 per messaggi MQTT
void get_iso_timestamp(char* buffer, size_t buffer_size) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(buffer, buffer_size, "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
}

// ===============================================================================
// FUNZIONI DECODIFICA REGISTRI
// ===============================================================================

// Decodifica e visualizza contenuto registro status (registro 5)
void decode_status_register(uint16_t status) {
    ESP_LOGI(TAG, "🔍 DECODIFICA STATUS REGISTER (0x%04X):", status);
    ESP_LOGI(TAG, "   %s Bit 0: Compressore %s", (status & 0x0001) ? "✅" : "❌", (status & 0x0001) ? "ACCESO" : "SPENTO");
    ESP_LOGI(TAG, "   %s Bit 1: Ventilatore evaporatore %s", (status & 0x0002) ? "✅" : "❌", (status & 0x0002) ? "ACCESO" : "SPENTO");
    ESP_LOGI(TAG, "   %s Bit 2: Ventilatore condensatore %s", (status & 0x0004) ? "✅" : "❌", (status & 0x0004) ? "ACCESO" : "SPENTO");
    ESP_LOGI(TAG, "   %s Bit 3: %s", (status & 0x0008) ? "🧊" : "❄️", (status & 0x0008) ? "Sbrinamento ATTIVO" : "Funzionamento NORMALE");
    ESP_LOGI(TAG, "   %s Bit 4: Modalità %s", (status & 0x0010) ? "🔧" : "🤖", (status & 0x0010) ? "MANUALE" : "AUTOMATICA");
    ESP_LOGI(TAG, "   %s Bit 5: Modalità %s", (status & 0x0020) ? "💤" : "🏃", (status & 0x0020) ? "STANDBY" : "ATTIVA");
    ESP_LOGI(TAG, "   %s Bit 6: Porta %s", (status & 0x0040) ? "🚪" : "🔒", (status & 0x0040) ? "APERTA" : "CHIUSA");
    
    if (status & 0x0080) {
        ESP_LOGI(TAG, "   ✅ Bit 7: Sensore temperatura OK");
    } else {
        ESP_LOGW(TAG, "   ⚠️ Bit 7: Sensore temperatura NON CONFERMATO");
    }
    
    if (status & 0xFF00) {
        ESP_LOGI(TAG, "   📋 Bit 8-15: Altri stati attivi (0x%02X)", (status >> 8));
    }
}

// Decodifica e visualizza contenuto registro allarmi (registro 8)
void decode_alarms_register(uint16_t alarms) {
    if (alarms == 0) {
        ESP_LOGI(TAG, "✅ ALLARMI: Nessun allarme attivo");
        return;
    }
    
    ESP_LOGW(TAG, "🚨 ALLARMI ATTIVI (0x%04X):", alarms);
    
    // Bit 0-7: Allarmi base
    if (alarms & 0x0001) ESP_LOGW(TAG, "     🌡️ Bit 0: ALLARME Temperatura alta");
    if (alarms & 0x0002) ESP_LOGW(TAG, "     📊 Bit 1: ALLARME Pressione alta");
    if (alarms & 0x0004) ESP_LOGW(TAG, "     🔧 Bit 2: ALLARME Sensore difettoso");
    if (alarms & 0x0008) ESP_LOGW(TAG, "     📡 Bit 3: ALLARME Comunicazione persa");
    if (alarms & 0x0010) ESP_LOGW(TAG, "     ⚙️ Bit 4: ALLARME Compressore fault");
    if (alarms & 0x0020) ESP_LOGW(TAG, "     🔧 Bit 5: ALLARME Manutenzione richiesta");
    if (alarms & 0x0040) ESP_LOGW(TAG, "     🚪 Bit 6: ALLARME Porta aperta troppo a lungo");
    if (alarms & 0x0080) ESP_LOGW(TAG, "     🧽 Bit 7: ALLARME Filtro sporco");
    
    // Bit 8-9
    if (alarms & 0x0100) ESP_LOGW(TAG, "     ⚡ Bit 8: ALLARME Tensione anomala");        
    if (alarms & 0x0200) ESP_LOGW(TAG, "     🔥 Bit 9: ALLARME Sovratemperatura motore"); 
    
    // Bit 10-15: Allarmi aggiuntivi per espansioni future
    if (alarms & 0xFC00) ESP_LOGW(TAG, "     📋 Bit 10-15: Altri allarmi attivi (0x%04X)", (alarms >> 10));
}

// ===============================================================================
// GESTIONE LED ERRORI (ROSSO)
// ===============================================================================

// Aggiorna stato LED errore rosso basandosi su condizioni sistema
void update_error_led() {
    bool should_be_on = false;
    
    // Controlla condizioni che richiedono LED errore acceso
    if (!wifi_connected) {
        should_be_on = true;
        if (!led_error_state) ESP_LOGW(TAG, "🔴 LED ERRORE ON: WiFi disconnesso");
    } else if (!mqtt_connected) {
        should_be_on = true;
        if (!led_error_state) ESP_LOGW(TAG, "🔴 LED ERRORE ON: MQTT disconnesso");
    } else if (data_initialized && registers.paramET_alarms != 0) {
        should_be_on = true;
        if (!led_error_state) ESP_LOGW(TAG, "🔴 LED ERRORE ON: Allarmi Modbus attivi (0x%04X)", registers.paramET_alarms);
    } else if (system_state == SYS_ERROR_RS485 || system_state == SYS_ERROR_COMM) {
        should_be_on = true;
        if (!led_error_state) ESP_LOGW(TAG, "🔴 LED ERRORE ON: Errore sistema");
    } else {
        if (led_error_state) ESP_LOGI(TAG, "🔴 LED ERRORE OFF: Errori risolti");
    }
    
    // Aggiorna stato fisico LED solo se cambiato
    if (led_error_state != should_be_on) {
        led_error_state = should_be_on;
        gpio_set_level(LED_ERROR_PIN, should_be_on ? 1 : 0);
    }
}

// ===============================================================================
// GESTIONE LED STATUS (GIALLO)
// ===============================================================================

// Imposta stato fisico LED giallo
void set_led_physical(bool state) {
    led_physical_state = state;
    gpio_set_level(LED_PIN, state ? 1 : 0);
}

// Cambia stato logico LED con reset contatori
void update_led_state(led_state_t new_state) {
    if (led_state != new_state) {
        led_state = new_state;
        led_last_toggle = xTaskGetTickCount();
        led_blink_count = 0;
        const char* state_names[] = {"BOOT", "CONNECTING", "NORMAL", "DATA_TX", "ERROR", "RS485_COMM"};
        ESP_LOGI(TAG, "💡 LED GIALLO: %s", state_names[new_state]);
    }
}

// Task gestione pattern LED intelligenti
void led_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        
        switch (led_state) {
            case LED_BOOT:
                // Lampeggio rapido durante inizializzazione (200ms)
                if ((current_time - led_last_toggle) > pdMS_TO_TICKS(200)) {
                    set_led_physical(!led_physical_state);
                    led_last_toggle = current_time;
                }
                break;
                
            case LED_CONNECTING:
                // Lampeggio medio durante connessioni (500ms)
                if ((current_time - led_last_toggle) > pdMS_TO_TICKS(500)) {
                    set_led_physical(!led_physical_state);
                    led_last_toggle = current_time;
                }
                break;
                
            case LED_NORMAL:
                // Breve impulso ogni 2 secondi (sistema operativo)
                if (!led_physical_state && (current_time - led_last_toggle) > pdMS_TO_TICKS(2000)) {
                    set_led_physical(true);
                    led_last_toggle = current_time;
                } else if (led_physical_state && (current_time - led_last_toggle) > pdMS_TO_TICKS(150)) {
                    set_led_physical(false);
                    led_last_toggle = current_time;
                }
                break;
                
            case LED_DATA_TX:
                // 3 lampeggi rapidi per trasmissione dati (100ms)
                if ((current_time - led_last_toggle) > pdMS_TO_TICKS(100)) {
                    if (led_blink_count < 6) {  // 3 lampeggi = 6 toggle
                        set_led_physical(!led_physical_state);
                        led_blink_count++;
                        led_last_toggle = current_time;
                    } else {
                        update_led_state(LED_NORMAL);
                    }
                }
                break;
                
            case LED_RS485_COMM:
                // 2 lampeggi veloci per comunicazione RS485 (80ms)
                if ((current_time - led_last_toggle) > pdMS_TO_TICKS(80)) {
                    if (led_blink_count < 4) {  // 2 lampeggi = 4 toggle
                        set_led_physical(!led_physical_state);
                        led_blink_count++;
                        led_last_toggle = current_time;
                    } else if ((current_time - led_last_toggle) > pdMS_TO_TICKS(800)) {
                        update_led_state(LED_NORMAL);
                    }
                }
                break;
                
            case LED_ERROR:
                // Lampeggio continuo errore (150ms)
                if ((current_time - led_last_toggle) > pdMS_TO_TICKS(150)) {
                    set_led_physical(!led_physical_state);
                    led_last_toggle = current_time;
                }
                break;
        }
        
        // Aggiorna LED errore ad ogni ciclo
        update_error_led();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LED_UPDATE_INTERVAL));
    }
}

// ===============================================================================
// GESTIONE COMUNICAZIONE RS485
// ===============================================================================

// Inizializza comunicazione RS485 con configurazione UART
void init_rs485() {
    ESP_LOGI(TAG, "🔧 Inizializzazione RS485 Modbus RTU Master...");

    // Configurazione parametri UART (9600-8-N-1)
    uart_config_t uart_config = {
        .baud_rate = RS485_BAUD_RATE,        // 9600 baud
        .data_bits = UART_DATA_8_BITS,       // 8 bit dati
        .parity = UART_PARITY_DISABLE,       // Nessuna parità
        .stop_bits = UART_STOP_BITS_1,       // 1 bit stop
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // Nessun controllo flusso
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Installa driver, configura parametri e pin
    ESP_ERROR_CHECK(uart_driver_install(RS485_UART_NUM, RS485_BUF_SIZE, RS485_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RS485_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART_NUM, RS485_TX_PIN, RS485_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "✅ RS485 configurato: UART%d TX=%d RX=%d", 
             RS485_UART_NUM, RS485_TX_PIN, RS485_RX_PIN);
}

// Calcola CRC16 Modbus per verifica integrità dati
uint16_t modbus_crc16(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

// Legge tutti i registri Modbus in una singola richiesta
bool read_all_modbus_registers() {
    ESP_LOGI(TAG, "📡 Lettura registri %d-%d (1-based)...", MB_HOLDING_REG_START, MB_HOLDING_REG_START + MB_HOLDING_REG_COUNT - 1);
    
    // Costruisce richiesta Modbus: [SlaveAddr][FC][StartAddr][Count][CRC]
    uint8_t request[8];
    request[0] = MB_SLAVE_ADDR;                           // Indirizzo slave
    request[1] = 0x03;                                    // Function code: Read Holding Registers
    request[2] = (MB_HOLDING_REG_START >> 8) & 0xFF;    // Start address high byte
    request[3] = MB_HOLDING_REG_START & 0xFF;            // Start address low byte
    request[4] = (MB_HOLDING_REG_COUNT >> 8) & 0xFF;    // Register count high byte
    request[5] = MB_HOLDING_REG_COUNT & 0xFF;            // Register count low byte
    
    // Calcola e aggiunge CRC
    uint16_t crc = modbus_crc16(request, 6);
    request[6] = crc & 0xFF;        // CRC low byte
    request[7] = (crc >> 8) & 0xFF; // CRC high byte
    
    // Pulisce buffer e prepara trasmissione
    uart_flush(RS485_UART_NUM);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Abilita trasmissione (DE/RE = HIGH)
    gpio_set_level(RS485_DE_RE_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Invia richiesta
    int written = uart_write_bytes(RS485_UART_NUM, request, sizeof(request));
    if (written != sizeof(request)) {
        ESP_LOGE(TAG, "❌ Errore invio: %d/%d bytes", written, sizeof(request));
        gpio_set_level(RS485_DE_RE_PIN, 0);
        return false;
    }
    
    // Attende completamento trasmissione e passa in ricezione
    uart_wait_tx_done(RS485_UART_NUM, pdMS_TO_TICKS(200));
    gpio_set_level(RS485_DE_RE_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    
    // Calcola numero byte attesi: [SlaveAddr][FC][ByteCount][Data][CRC]
    int expected_bytes = 3 + (MB_HOLDING_REG_COUNT * 2) + 2;
    uint8_t response[256];
    int received = uart_read_bytes(RS485_UART_NUM, response, sizeof(response), pdMS_TO_TICKS(MB_TIMEOUT_MS));
    
    // Verifica lunghezza risposta
    if (received < expected_bytes) {
        ESP_LOGW(TAG, "⚠️ Risposta incompleta: %d<%d bytes", received, expected_bytes);
        if (received >= 3 && response[1] == 0x83) {
            ESP_LOGE(TAG, "🚨 ERRORE MODBUS: Exception 0x%02X", response[2]);
        }
        return false;
    }
    
    // Verifica header risposta
    if (response[0] != MB_SLAVE_ADDR || response[1] != 0x03) {
        ESP_LOGW(TAG, "⚠️ Header non valido: Slave=%d, FC=%d", response[0], response[1]);
        return false;
    }
    
    // Verifica CRC risposta
    uint16_t received_crc = (response[received-1] << 8) | response[received-2];
    uint16_t calculated_crc = modbus_crc16(response, received - 2);
    if (received_crc != calculated_crc) {
        ESP_LOGW(TAG, "⚠️ Errore CRC: 0x%04X vs 0x%04X", received_crc, calculated_crc);
        return false;
    }
    
    // Estrae dati registri dalla risposta
    ESP_LOGI(TAG, "✅ Estrazione %d registri:", MB_HOLDING_REG_COUNT);
    for (int i = 0; i < MB_HOLDING_REG_COUNT; i++) {
        int byte_idx = 3 + (i * 2);  // Salta header [SlaveAddr][FC][ByteCount]
        raw_modbus_data[i] = (response[byte_idx] << 8) | response[byte_idx + 1];
        ESP_LOGI(TAG, "   📊 Reg%d: %d (0x%04X)", i + 1, raw_modbus_data[i], raw_modbus_data[i]);
    }
    
    // Aggiorna statistiche
    successful_reads++;
    last_successful_read = xTaskGetTickCount();
    return true;
}

// ===============================================================================
// CONVERSIONE E GESTIONE DATI
// ===============================================================================

// Converte dati grezzi registri in valori fisici con scaling appropriato
void convert_raw_to_registers() {
    ESP_LOGI(TAG, "🔄 Conversione dati grezzi...");
    
    // Applica fattori di scala specifici per ogni parametro
    registers.paramET_temp = raw_modbus_data[0] / 10.0f;      // Temperatura: /10 per °C
    registers.paramET_setpoint = raw_modbus_data[1] / 10.0f;  // Setpoint: /10 per °C
    registers.paramET_umid = raw_modbus_data[2] / 10.0f;      // Umidità: /10 per %
    registers.paramET_pressure = raw_modbus_data[3] / 100.0f; // Pressione: /100 per bar
    registers.paramET_status = raw_modbus_data[4];            // Status: valore diretto
    registers.paramET_runtime = raw_modbus_data[5];           // Runtime: valore diretto (ore)
    registers.paramET_power = raw_modbus_data[6] / 100.0f;    // Power: /100 per kW
    registers.paramET_alarms = raw_modbus_data[7];            // Allarmi: valore diretto
    
    data_initialized = true;  // Marca dati come validi
    
    ESP_LOGI(TAG, "✅ Conversione completata:");
    ESP_LOGI(TAG, "   🌡️ Temp: %.1f°C", registers.paramET_temp);
    ESP_LOGI(TAG, "   🎯 Setpoint: %.1f°C", registers.paramET_setpoint);
    ESP_LOGI(TAG, "   💧 Umidità: %.1f%%", registers.paramET_umid);
    ESP_LOGI(TAG, "   📊 Pressione: %.2f bar", registers.paramET_pressure);
    ESP_LOGI(TAG, "   ⏰ Runtime: %lu ore", registers.paramET_runtime);
    ESP_LOGI(TAG, "   ⚡ Power: %.2f kW", registers.paramET_power);
    
    // Decodifica registri complessi
    decode_status_register(registers.paramET_status);
    decode_alarms_register(registers.paramET_alarms);
}

// Funzione principale lettura Modbus con gestione LED
bool read_modbus_registers_from_serial() {
    update_led_state(LED_RS485_COMM);  // Indica comunicazione in corso
    
    bool success = read_all_modbus_registers();
    if (success) {
        convert_raw_to_registers();
        rs485_read_counter++;
        ESP_LOGI(TAG, "✅ Lettura Modbus completata (#%lu successi)", successful_reads);
        return true;
    } else {
        ESP_LOGE(TAG, "❌ Errore lettura Modbus");
        error_counter++;
        return false;
    }
}

// ===============================================================================
// GESTIONE MESSAGGI MQTT
// ===============================================================================

// Invia dati grezzi registri per debug avanzato
void send_debug_raw_message() {
    if (!mqtt_connected || !data_initialized) return;
    
    char message[512];
    char timestamp_str[32];
    get_iso_timestamp(timestamp_str, sizeof(timestamp_str));
    
    // Costruisce JSON con valori grezzi registri
    int len = snprintf(message, sizeof(message),
        "{\"tipo\":\"DEBUG_RAW\",\"id_nodo\":\"FRIGO01\","
        "\"raw_reg1\":%u,\"raw_reg2\":%u,\"raw_reg3\":%u,\"raw_reg4\":%u,"
        "\"raw_reg5\":%u,\"raw_reg6\":%u,\"raw_reg7\":%u,\"raw_reg8\":%u,"
        "\"timestamp\":\"%s\"}",
        raw_modbus_data[0], raw_modbus_data[1], raw_modbus_data[2], raw_modbus_data[3],
        raw_modbus_data[4], raw_modbus_data[5], raw_modbus_data[6], raw_modbus_data[7],
        timestamp_str);
    
    if (len > 0 && len < sizeof(message)) {
        esp_mqtt_client_publish(mqtt_client, TOPIC_DEBUG_RAW, message, 0, 0, 0);
        ESP_LOGI(TAG, "🔧 Dati RAW debug inviati");
    }
}

// Invia messaggio principale con dati operativi convertiti
void send_data_static_message() {
    if (!mqtt_connected || !data_initialized) {
        ESP_LOGW(TAG, "⚠️ Skip invio: MQTT=%s, Data=%s", mqtt_connected ? "OK" : "NO", data_initialized ? "OK" : "NO");
        return;
    }
    
    update_led_state(LED_DATA_TX);  // Indica trasmissione dati
    
    char message[1024];
    char timestamp_str[32];
    get_iso_timestamp(timestamp_str, sizeof(timestamp_str));
    
    // Costruisce JSON completo con dati convertiti e stati decodificati
    int len = snprintf(message, sizeof(message),
        "{\"tipo\":\"DATA_STATIC\",\"id_nodo\":\"FRIGO01\",\"versione\":\"2.0.0_CLEAN_NO_BIT8\","
        "\"data_valid\":%s,\"paramET_temp\":%.2f,\"paramET_setpoint\":%.2f,"
        "\"paramET_umid\":%.1f,\"paramET_pressure\":%.2f,\"paramET_status\":%u,"
        "\"paramET_runtime\":%lu,\"paramET_power\":%.2f,\"paramET_alarms\":%u,"
        "\"compressore_stato\":\"%s\",\"allarmi_attivi\":%s,\"led_error_active\":%s,"
        "\"comp_acceso\":%s,\"fan_evap_acceso\":%s,\"fan_cond_acceso\":%s,"
        "\"sbrinamento_attivo\":%s,\"modalita_manuale\":%s,\"porta_aperta\":%s,"
        "\"letture_rs485\":%lu,\"letture_ok\":%lu,\"timestamp\":\"%s\",\"message_id\":%lu}",
        data_initialized ? "true" : "false", registers.paramET_temp, registers.paramET_setpoint,
        registers.paramET_umid, registers.paramET_pressure, registers.paramET_status,
        registers.paramET_runtime, registers.paramET_power, registers.paramET_alarms,
        (registers.paramET_status & 0x0001) ? "ACCESO" : "SPENTO",
        (registers.paramET_alarms != 0) ? "true" : "false", led_error_state ? "true" : "false",
        (registers.paramET_status & 0x0001) ? "true" : "false",
        (registers.paramET_status & 0x0002) ? "true" : "false",
        (registers.paramET_status & 0x0004) ? "true" : "false",
        (registers.paramET_status & 0x0008) ? "true" : "false",
        (registers.paramET_status & 0x0010) ? "true" : "false",
        (registers.paramET_status & 0x0040) ? "true" : "false",
        rs485_read_counter, successful_reads, timestamp_str, ++message_counter);
    
    if (len > 0 && len < sizeof(message)) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, TOPIC_DATA_STATIC, message, 0, 0, 0);
        ESP_LOGI(TAG, "📤 Dati statici inviati! MSG_ID: %d (Counter: %lu)", msg_id, message_counter);
        send_debug_raw_message();  // Invia anche dati debug
    } else {
        ESP_LOGE(TAG, "❌ Errore creazione messaggio dati statici");
        error_counter++;
    }
}

// Invia status sistema e statistiche operative
void send_system_status_message() {
    if (!mqtt_connected) return;
    
    char message[512];
    char timestamp_str[32];
    get_iso_timestamp(timestamp_str, sizeof(timestamp_str));
    
    // Calcola tempo dall'ultima lettura riuscita
    TickType_t time_since_read = (xTaskGetTickCount() - last_successful_read) * portTICK_PERIOD_MS / 1000;
    
    // Costruisce JSON con statistiche sistema
    int len = snprintf(message, sizeof(message),
        "{\"tipo\":\"SYSTEM_STATUS\",\"id_nodo\":\"FRIGO01\",\"versione\":\"2.0.0_CLEAN_NO_BIT8\","
        "\"system_state\":\"%s\",\"led_state\":\"%s\",\"led_error_active\":%s,"
        "\"wifi_connected\":%s,\"mqtt_connected\":%s,\"data_initialized\":%s,"
        "\"rs485_reads\":%lu,\"successful_reads\":%lu,\"messages_sent\":%lu,"
        "\"errors\":%lu,\"last_read_sec_ago\":%lu,\"uptime_sec\":%llu,"
        "\"free_heap\":%lu,\"timestamp\":\"%s\"}",
        system_state == SYS_RUNNING ? "RUNNING" :
        system_state == SYS_WIFI_CONNECTING ? "WIFI_CONNECTING" :
        system_state == SYS_MQTT_CONNECTING ? "MQTT_CONNECTING" : "ERROR",
        led_state == LED_NORMAL ? "NORMAL" :
        led_state == LED_DATA_TX ? "DATA_TX" :
        led_state == LED_RS485_COMM ? "RS485_COMM" : "OTHER",
        led_error_state ? "true" : "false", wifi_connected ? "true" : "false",
        mqtt_connected ? "true" : "false", data_initialized ? "true" : "false",
        rs485_read_counter, successful_reads, message_counter, error_counter,
        time_since_read, esp_timer_get_time() / 1000000ULL, esp_get_free_heap_size(), timestamp_str);
    
    if (len > 0 && len < sizeof(message)) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, TOPIC_SYSTEM_STATUS, message, 0, 0, 0);
        ESP_LOGI(TAG, "📊 Status sistema inviato! MSG_ID: %d", msg_id);
    }
}

// ===============================================================================
// GESTIONE EVENTI MQTT
// ===============================================================================

// Handler eventi client MQTT
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "✅ MQTT connesso!");
            mqtt_connected = true;
            update_system_state(SYS_RUNNING);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "⚠️ MQTT disconnesso!");
            mqtt_connected = false;
            update_system_state(SYS_MQTT_CONNECTING);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGD(TAG, "📤 Messaggio MQTT pubblicato");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "❌ Errore MQTT");
            error_counter++;
            break;
        default:
            break;
    }
}

// Inizializza client MQTT con configurazione broker
void init_mqtt() {
    ESP_LOGI(TAG, "🌐 Inizializzazione client MQTT...");
    
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = MQTT_BROKER_URI;        // URI broker
    mqtt_cfg.credentials.client_id = MQTT_CLIENT_ID;      // ID client univoco
    mqtt_cfg.session.keepalive = 60;                      // Keepalive 60 secondi

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
    
    ESP_LOGI(TAG, "✅ Client MQTT inizializzato per %s", MQTT_BROKER_URI);
}

// ===============================================================================
// GESTIONE EVENTI WI-FI
// ===============================================================================

// Handler eventi Wi-Fi per gestione connessione automatica
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "📶 Wi-Fi avviato, connessione in corso...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* disconnected = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGW(TAG, "⚠️ Wi-Fi disconnesso! Motivo: %d", disconnected->reason);
        wifi_connected = false;
        update_system_state(SYS_WIFI_CONNECTING);
        vTaskDelay(pdMS_TO_TICKS(5000));  // Attesa prima di riconnettere
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "✅ Wi-Fi connesso! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        update_system_state(SYS_MQTT_CONNECTING);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Inizializza Wi-Fi in modalità Station
void init_wifi() {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Registra handler eventi
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    // Configura credenziali Wi-Fi
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASSWORD);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "📶 Wi-Fi inizializzato");
}

// ===============================================================================
// GESTIONE STATI SISTEMA
// ===============================================================================

// Aggiorna stato sistema con transizioni LED appropriate
void update_system_state(system_state_t new_state) {
    if (system_state != new_state) {
        system_state = new_state;
        const char* state_names[] = {"INIT", "WIFI_CONNECTING", "MQTT_CONNECTING", "RUNNING", "ERROR_RS485", "ERROR_COMM"};
        ESP_LOGI(TAG, "🔄 Sistema: %s", state_names[new_state]);
        
        // Aggiorna stato LED basandosi su nuovo stato sistema
        switch (new_state) {
            case SYS_INIT: 
                update_led_state(LED_BOOT); 
                break;
            case SYS_WIFI_CONNECTING:
            case SYS_MQTT_CONNECTING: 
                update_led_state(LED_CONNECTING); 
                break;
            case SYS_RUNNING: 
                update_led_state(LED_NORMAL); 
                break;
            case SYS_ERROR_RS485:
            case SYS_ERROR_COMM: 
                update_led_state(LED_ERROR); 
                break;
        }
    }
}

// ===============================================================================
// TASK PRINCIPALI
// ===============================================================================

// Task acquisizione dati Modbus con timing preciso
void data_acquisition_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    ESP_LOGI(TAG, "🔄 Task acquisizione dati avviato");
    
    while (1) {
        if (system_state == SYS_RUNNING) {
            ESP_LOGI(TAG, "🚀 === INIZIO CICLO LETTURA ===");
            if (read_modbus_registers_from_serial()) {
                ESP_LOGI(TAG, "✅ Lettura OK - Invio dati...");
                send_data_static_message();
                ESP_LOGI(TAG, "📤 Dati inviati con successo");
            } else {
                ESP_LOGE(TAG, "❌ Errore lettura registri RS485");
                error_counter++;
                // Cambia stato dopo 3 errori consecutivi
                if (error_counter % 3 == 0) {
                    ESP_LOGW(TAG, "⚠️ Troppi errori consecutivi - cambio stato");
                    update_system_state(SYS_ERROR_RS485);
                }
            }
            ESP_LOGI(TAG, "🏁 === FINE CICLO LETTURA ===");
        } else {
            ESP_LOGW(TAG, "⏳ Sistema non pronto - skip lettura");
        }
        // Attesa precisa per mantenere intervallo costante
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DATA_READ_INTERVAL));
    }
}

// Task monitoraggio sistema e invio statistiche
void system_status_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    ESP_LOGI(TAG, "📊 Task status sistema avviato");
    
    while (1) {
        // Invia status se MQTT connesso
        if (mqtt_connected) {
            send_system_status_message();
        }
        
        // Calcola tasso di successo letture
        float success_rate = (rs485_read_counter > 0) ? (100.0f * successful_reads / rs485_read_counter) : 0.0f;
        
        // Report statistiche dettagliate
        ESP_LOGI(TAG, "📈 STATISTICHE:");
        ESP_LOGI(TAG, "   🎯 Sistema: %s | MQTT: %s | Dati: %s",
                 system_state == SYS_RUNNING ? "OK" : "ERROR",
                 mqtt_connected ? "CONNESSO" : "DISCONNESSO",
                 data_initialized ? "VALIDI" : "NON_INIT");
        ESP_LOGI(TAG, "   📊 RS485: %lu letture, %lu OK (%.1f%% successo)", rs485_read_counter, successful_reads, success_rate);
        ESP_LOGI(TAG, "   📤 MQTT: %lu messaggi inviati | ❌ %lu errori totali", message_counter, error_counter);
        ESP_LOGI(TAG, "   🔴 LED ERRORE: %s", led_error_state ? "ATTIVO" : "SPENTO");
        
        // Report valori attuali se dati disponibili
        if (data_initialized) {
            ESP_LOGI(TAG, "   🌡️ VALORI: T=%.1f°C, P=%.2fbar, Status=0x%04X, Allarmi=0x%04X",
                     registers.paramET_temp, registers.paramET_pressure, registers.paramET_status, registers.paramET_alarms);
            ESP_LOGI(TAG, "   ⚙️ STATI: Comp:%s | FanE:%s | FanC:%s | Defrost:%s | Porta:%s",
                     (registers.paramET_status & 0x0001) ? "ON" : "OFF",
                     (registers.paramET_status & 0x0002) ? "ON" : "OFF",
                     (registers.paramET_status & 0x0004) ? "ON" : "OFF",
                     (registers.paramET_status & 0x0008) ? "ON" : "OFF",
                     (registers.paramET_status & 0x0040) ? "OPEN" : "CLOSE");
            
            // Report allarmi attivi se presenti
            if (registers.paramET_alarms != 0) {
                ESP_LOGW(TAG, "   🚨 ALLARMI ATTIVI: TempHi:%s | Press:%s | Sensor:%s | Comm:%s | CompFault:%s",
                         (registers.paramET_alarms & 0x0001) ? "✗" : "✓",
                         (registers.paramET_alarms & 0x0002) ? "✗" : "✓",
                         (registers.paramET_alarms & 0x0004) ? "✗" : "✓",
                         (registers.paramET_alarms & 0x0008) ? "✗" : "✓",
                         (registers.paramET_alarms & 0x0010) ? "✗" : "✓");
            }
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(STATUS_SEND_INTERVAL));
    }
}

// ===============================================================================
// FUNZIONE PRINCIPALE
// ===============================================================================

// Funzione principale con inizializzazione completa sistema
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "🚀 === ESP32-C6 Modbus RS485 + MQTT - VERSIONE FINALE COMPLETA ===");
    ESP_LOGI(TAG, "📋 Versione: 2.0.0_CLEAN_NO_BIT8 - Codice Completo + LED Errore");
    ESP_LOGI(TAG, "🔧 Hardware: RS485(TX:%d,RX:%d,DE/RE:%d) + LED_GIALLO:%d + LED_ROSSO:%d", 
             RS485_TX_PIN, RS485_RX_PIN, RS485_DE_RE_PIN, LED_PIN, LED_ERROR_PIN);
    ESP_LOGI(TAG, "🎯 FUNZIONALITÀ:");
    ESP_LOGI(TAG, "   ✅ Indirizzamento 1-BASED (registri 1-8)");
    ESP_LOGI(TAG, "   ✅ Decodifica completa registri status e allarmi");
    ESP_LOGI(TAG, "   ✅ LED intelligenti (giallo: sistema, rosso: errori)");
    ESP_LOGI(TAG, "   ✅ Auto-recovery errori RS485");
    ESP_LOGI(TAG, "   ✅ MQTT con 3 topic + debug completo");
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STEP 1: INIZIALIZZAZIONE NVS FLASH
    // ═══════════════════════════════════════════════════════════════════════════
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STEP 2: CONFIGURAZIONE LED HARDWARE
    // ═══════════════════════════════════════════════════════════════════════════
    
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_PIN) | (1ULL << LED_ERROR_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_conf);
    gpio_set_level(LED_PIN, 0);      // LED giallo spento
    gpio_set_level(LED_ERROR_PIN, 0); // LED rosso spento
    
    ESP_LOGI(TAG, "💡 LED configurati: GIALLO=%d, ROSSO=%d", LED_PIN, LED_ERROR_PIN);
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STEP 3: AVVIO TASK LED
    // ═══════════════════════════════════════════════════════════════════════════
    
    xTaskCreate(led_task, "led_task", 2048, NULL, 7, NULL);
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STEP 4: INIZIALIZZAZIONE RS485
    // ═══════════════════════════════════════════════════════════════════════════
    
    init_rs485();
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STEP 5: INIZIALIZZAZIONE CONNETTIVITÀ
    // ═══════════════════════════════════════════════════════════════════════════
    
    update_system_state(SYS_WIFI_CONNECTING);
    init_wifi();
    
    // Attendi connessione Wi-Fi
    ESP_LOGI(TAG, "⏳ Attesa connessione Wi-Fi...");
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STEP 6: CONFIGURAZIONE NTP
    // ═══════════════════════════════════════════════════════════════════════════
    
    ESP_LOGI(TAG, "🕐 Configurazione NTP...");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STEP 7: INIZIALIZZAZIONE MQTT
    // ═══════════════════════════════════════════════════════════════════════════
    
    update_system_state(SYS_MQTT_CONNECTING);
    init_mqtt();
    
    // Attendi connessione MQTT con timeout
    ESP_LOGI(TAG, "⏳ Attesa connessione MQTT...");
    int mqtt_wait_count = 0;
    while (!mqtt_connected && mqtt_wait_count < 30) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        mqtt_wait_count++;
    }
    
    if (mqtt_connected) {
        ESP_LOGI(TAG, "✅ Sistema completamente inizializzato!");
        update_system_state(SYS_RUNNING);
        
        vTaskDelay(pdMS_TO_TICKS(2000));
        send_system_status_message();
        
        // ═══════════════════════════════════════════════════════════════════
        // TEST INIZIALE COMUNICAZIONE MODBUS
        // ═══════════════════════════════════════════════════════════════════
        
        ESP_LOGI(TAG, "🧪 === TEST INIZIALE INDIRIZZAMENTO 1-BASED ===");
        ESP_LOGI(TAG, "🎯 Lettura registri 1-8 (non 0-7)...");
        if (read_modbus_registers_from_serial()) {
            ESP_LOGI(TAG, "🎉 TEST INIZIALE RIUSCITO - Dati validi ricevuti!");
            send_data_static_message();
        } else {
            ESP_LOGW(TAG, "⚠️ Test iniziale fallito - continuerò a provare");
        }
        ESP_LOGI(TAG, "🧪 === FINE TEST INIZIALE ===");
    } else {
        ESP_LOGE(TAG, "❌ Timeout connessione MQTT");
        update_system_state(SYS_ERROR_COMM);
    }
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STEP 8: AVVIO TASK PRINCIPALI
    // ═══════════════════════════════════════════════════════════════════════════
    
    xTaskCreate(data_acquisition_task, "data_task", 4096, NULL, 5, NULL);
    xTaskCreate(system_status_task, "status_task", 3072, NULL, 4, NULL);
    
    // ═══════════════════════════════════════════════════════════════════════════
    // SISTEMA OPERATIVO - INFORMAZIONI FINALI
    // ═══════════════════════════════════════════════════════════════════════════
    
    ESP_LOGI(TAG, "🎯 === SISTEMA OPERATIVO ===");
    ESP_LOGI(TAG, "📡 Topic MQTT: data=%s, status=%s, debug=%s", TOPIC_DATA_STATIC, TOPIC_SYSTEM_STATUS, TOPIC_DEBUG_RAW);
    ESP_LOGI(TAG, "⏰ Intervalli: lettura=%ds, status=%ds", DATA_READ_INTERVAL/1000, STATUS_SEND_INTERVAL/1000);
    
    // ═══════════════════════════════════════════════════════════════════════════
    // LOOP PRINCIPALE CON AUTO-RECOVERY E MONITORAGGIO
    // ═══════════════════════════════════════════════════════════════════════════
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));  // Check ogni 10 secondi
        
        // Auto-recovery errori RS485
        if (system_state == SYS_ERROR_RS485) {
            ESP_LOGW(TAG, "🔄 Tentativo recupero errore RS485...");
            vTaskDelay(pdMS_TO_TICKS(5000));
            if (read_modbus_registers_from_serial()) {
                ESP_LOGI(TAG, "✅ Recupero RS485 riuscito");
                update_system_state(SYS_RUNNING);
                error_counter = 0;  // Reset contatore errori
            }
        }
        
        // Monitoraggio memoria heap per rilevare memory leak
        size_t free_heap = esp_get_free_heap_size();
        if (free_heap < 50000) {
            ESP_LOGW(TAG, "⚠️ Memoria bassa: %zu bytes", free_heap);
        }
    }
}