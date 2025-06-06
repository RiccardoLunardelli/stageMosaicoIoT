/**
 * ===============================================================================
 * ESP32-C6 MODBUS RTU MASTER - 1.0
 * ===============================================================================
 * 
 * DESCRIZIONE:
 * Sistema di lettura sequenziale registri Modbus con controllo LED di stato.
 * Legge i registri uno alla volta in modalità ciclica e controlla un LED
 * basandosi sui valori letti.
 * 
 * HARDWARE SETUP:
 * - ESP32-C6 Development Board
 * - Convertitore USB-RS485 Waveshare
 * - LED di status con resistore 220Ω
 * - Simulatore Modbus su PC
 * 
 * CABLAGGIO:
 * - GPIO4 (TX) → TXD Convertitore
 * - GPIO5 (RX) ← RXD Convertitore  
 * - GND → GND Convertitore
 * - GPIO18 → LED Status (tramite resistore 220Ω)
 * - USB-C → Alimentazione 5V
 * 
 * CONFIGURAZIONE MODBUS:
 * - Protocollo: Modbus RTU
 * - Velocità: 9600 baud, 8-N-1
 * - Slave Address: 1
 * - Registri: 1-2 (holding registers)
 * - Funzione: 0x03 (Read Holding Registers)
 * 
 * LOGICA LED:
 * - LED ACCESO se qualsiasi registro = 1
 * - LED SPENTO per tutti gli altri valori
 * - LED SPENTO in caso di errori di comunicazione
 * 
 * AUTORE: Soufian Markouni e Riccardo Lunardelli
 * DATA: Giugno 2025
 * VERSIONE: 1.0
 * ===============================================================================
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

// Include Modbus Master ESP-IDF
#include "esp_modbus_master.h"
#include "esp_modbus_common.h"

// ===============================================================================
// CONFIGURAZIONE HARDWARE
// ===============================================================================

#define LED_GPIO GPIO_NUM_18        // Pin LED di status (con resistore 220Ω)

// ===============================================================================
// CONFIGURAZIONE MODBUS RTU
// ===============================================================================

#define MB_PORT_NUM     UART_NUM_1  // Porta UART per comunicazione Modbus
#define MB_DEV_SPEED    9600        // Velocità baud (matching simulatore)
#define MB_UART_TXD     4           // GPIO TX → TXD Convertitore
#define MB_UART_RXD     5           // GPIO RX ← RXD Convertitore
#define MB_UART_RTS     UART_PIN_NO_CHANGE  // RTS non utilizzato

// ===============================================================================
// PARAMETRI SLAVE MODBUS
// ===============================================================================

#define MB_SLAVE_ADDR           1   // Indirizzo dispositivo slave
#define MB_HOLDING_REG_START    1   // Primo registro (1-based addressing)
#define MB_HOLDING_REG_COUNT    3  // Numero totale registri disponibili (1 e 2)
#define MB_SINGLE_REG_READ      1   // Legge un registro alla volta

// ===============================================================================
// VARIABILI GLOBALI
// ===============================================================================

static const char *TAG = "MODBUS_LED";

// Buffer per dati lettura sequenziale
uint16_t current_register_value = 0;        // Valore del registro corrente
uint16_t current_register_address = MB_HOLDING_REG_START;  // Indirizzo registro corrente
bool data_valid = false;                    // Flag validità dati

// ===============================================================================
// FUNZIONI LED DI STATUS
// ===============================================================================

// Configura il pin LED come output
void configure_led(void) {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);  // Inizialmente spento
    ESP_LOGI(TAG, "💡 LED configurato su GPIO%d", LED_GPIO);
}

// Accende il LED di status
void led_on(void) {
    gpio_set_level(LED_GPIO, 1);
    ESP_LOGI(TAG, "🔴 LED ACCESO");
}

// Spegne il LED di status
void led_off(void) {
    gpio_set_level(LED_GPIO, 0);
    ESP_LOGI(TAG, "⚫ LED SPENTO");
}

// Fa lampeggiare il LED per test iniziale
// times: numero di lampeggi, delay_ms: durata ON/OFF in millisecondi
void led_blink(int times, int delay_ms) {
    for (int i = 0; i < times; i++) {
        led_on();
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        led_off();
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

// ===============================================================================
// LOGICA CONTROLLO LED
// ===============================================================================

// Aggiorna lo stato del LED basandosi sul valore del registro corrente
// LOGICA: LED ACCESO se valore = 1, SPENTO per altri valori o errori
void update_led_based_on_current_register(void) {
    if (!data_valid) {
        ESP_LOGW(TAG, "⚠️ Dati non validi, LED spento");
        led_off();
        return;
    }
    
    ESP_LOGI(TAG, "📊 Registro %d = %d", current_register_address, current_register_value);
    
    // Controlla il valore del registro corrente
    if (current_register_value == 1) {
        ESP_LOGI(TAG, "💡 Registro %d = 1 → LED ACCESO", current_register_address);
        led_on();
    } else {
        ESP_LOGI(TAG, "💡 Registro %d = %d → LED SPENTO", current_register_address, current_register_value);
        led_off();
    }
}

// ===============================================================================
// TASK COMUNICAZIONE MODBUS
// ===============================================================================

// Task principale per polling Modbus RTU
// Legge un registro alla volta, cicla attraverso tutti i registri (1-2)
// Aggiorna LED basandosi sul valore letto, ripete ogni 1 secondo
void modbus_task(void *pvParameters) {
    esp_err_t err;
    uint32_t poll_count = 0;
    
    ESP_LOGI(TAG, "🚀 Task Modbus avviato - polling ogni 1 secondo");
    
    while (1) {
        ESP_LOGI(TAG, "🔄 Polling #%lu - Leggendo registro %d", ++poll_count, current_register_address);
        
        // Prepara richiesta Modbus per lettura singolo registro
        mb_param_request_t request;
        request.slave_addr = MB_SLAVE_ADDR;      // Indirizzo slave
        request.command = 0x03;                  // Function code: Read Holding Registers
        request.reg_start = current_register_address;  // Registro da leggere
        request.reg_size = MB_SINGLE_REG_READ;   // Leggi solo 1 registro
        
        // Esegui lettura Modbus
        err = mbc_master_send_request(&request, &current_register_value);
        
        if (err == ESP_OK) {
            // Lettura completata con successo
            ESP_LOGI(TAG, "✅ Lettura registro %d completata: valore = %d", 
                     current_register_address, current_register_value);
            data_valid = true;
            update_led_based_on_current_register();
        } else {
            // Errore durante la lettura
            ESP_LOGW(TAG, "❌ Errore lettura registro %d: %s", 
                     current_register_address, esp_err_to_name(err));
            data_valid = false;
            led_off(); // Spegni LED in caso di errore
        }
        
        // Passa al registro successivo (lettura ciclica)
        current_register_address++;
        if (current_register_address >= (MB_HOLDING_REG_START + MB_HOLDING_REG_COUNT)) {
            current_register_address = MB_HOLDING_REG_START; // Ricomincia dal primo
            ESP_LOGI(TAG, "🔄 Ciclo registri completato, ricomincio dal registro %d", current_register_address);
        }
        
        // Attesa prima del prossimo polling (1 secondo)
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ===============================================================================
// FUNZIONE PRINCIPALE
// ===============================================================================

// Funzione principale del programma
// Sequenza: LED test → UART config → Modbus init → Task avvio → Loop monitoraggio
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "===============================================================================");
    ESP_LOGI(TAG, "=== ESP32-C6 MODBUS RTU MASTER - LETTURA SEQUENZIALE REGISTRI ===");
    ESP_LOGI(TAG, "===============================================================================");
    ESP_LOGI(TAG, "Target: Slave %d, Registri %d-%d (lettura uno alla volta)", 
             MB_SLAVE_ADDR, 
             MB_HOLDING_REG_START, 
             MB_HOLDING_REG_START + MB_HOLDING_REG_COUNT - 1);
    ESP_LOGI(TAG, "Hardware: GPIO%d(TX), GPIO%d(RX), GPIO%d(LED)", 
             MB_UART_TXD, MB_UART_RXD, LED_GPIO);
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STEP 1: CONFIGURAZIONE LED DI STATUS
    // ═══════════════════════════════════════════════════════════════════════════
    
    ESP_LOGI(TAG, "📋 STEP 1: Configurazione LED di status...");
    configure_led();
    
    // Test LED iniziale per verificare funzionamento
    ESP_LOGI(TAG, "🧪 Test LED funzionamento...");
    led_blink(3, 300);  // 3 lampeggi da 300ms
    ESP_LOGI(TAG, "✅ STEP 1: LED configurato e testato con successo");
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STEP 2: CONFIGURAZIONE PARAMETRI UART
    // ═══════════════════════════════════════════════════════════════════════════
    
    ESP_LOGI(TAG, "📋 STEP 2: Configurazione parametri UART...");
    
    // Configurazione UART dettagliata (9600-8-N-1)
    uart_config_t uart_config = {
        .baud_rate = MB_DEV_SPEED,          // 9600 baud (matching simulatore)
        .data_bits = UART_DATA_8_BITS,      // 8 bit dati
        .parity = UART_PARITY_DISABLE,      // Nessuna parità (N)
        .stop_bits = UART_STOP_BITS_1,      // 1 bit di stop
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // Nessun controllo flusso
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_LOGI(TAG, "📡 Parametri UART: %d baud, 8-N-1 (matching simulatore)", MB_DEV_SPEED);
    esp_err_t err = uart_param_config(MB_PORT_NUM, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ ERRORE STEP 2 - Configurazione UART: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "✅ STEP 2: Parametri UART configurati con successo");
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STEP 3: INIZIALIZZAZIONE MODBUS MASTER
    // ═══════════════════════════════════════════════════════════════════════════
    
    ESP_LOGI(TAG, "📋 STEP 3: Inizializzazione Modbus Master...");
    
    // Configurazione parametri comunicazione Modbus
    mb_communication_info_t comm_info;
    comm_info.slave_addr = 0;               // Non usato per master
    comm_info.port = MB_PORT_NUM;           // UART1
    comm_info.mode = MB_MODE_RTU;           // Modalità RTU
    comm_info.baudrate = MB_DEV_SPEED;      // 9600 baud
    comm_info.parity = MB_PARITY_NONE;      // Nessuna parità
    comm_info.dummy_port = UART_PIN_NO_CHANGE;
    
    void* master_handler = NULL;
    
    // Inizializza il master Modbus
    err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ ERRORE STEP 3 - Inizializzazione Modbus Master: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "✅ STEP 3: Modbus Master inizializzato con successo");
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STEP 4: SETUP COMUNICAZIONE RTU
    // ═══════════════════════════════════════════════════════════════════════════
    
    ESP_LOGI(TAG, "📋 STEP 4: Setup comunicazione RTU...");
    err = mbc_master_setup((void*)&comm_info);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ ERRORE STEP 4 - Setup comunicazione Modbus: %s", esp_err_to_name(err));
        mbc_master_destroy();
        return;
    }
    ESP_LOGI(TAG, "✅ STEP 4: Comunicazione RTU configurata con successo");
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STEP 5: CONFIGURAZIONE PIN GPIO
    // ═══════════════════════════════════════════════════════════════════════════
    
    ESP_LOGI(TAG, "📋 STEP 5: Configurazione pin UART...");
    err = uart_set_pin(MB_PORT_NUM, MB_UART_TXD, MB_UART_RXD, MB_UART_RTS, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ ERRORE STEP 5 - Configurazione pin GPIO: %s", esp_err_to_name(err));
        mbc_master_destroy();
        return;
    }
    ESP_LOGI(TAG, "✅ STEP 5: Pin GPIO configurati con successo");
    ESP_LOGI(TAG, "   📡 UART%d: TX=GPIO%d, RX=GPIO%d", MB_PORT_NUM, MB_UART_TXD, MB_UART_RXD);
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STEP 6: AVVIO SISTEMA MODBUS
    // ═══════════════════════════════════════════════════════════════════════════
    
    ESP_LOGI(TAG, "📋 STEP 6: Avvio sistema Modbus...");
    err = mbc_master_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ ERRORE STEP 6 - Avvio sistema Modbus: %s", esp_err_to_name(err));
        mbc_master_destroy();
        return;
    }
    ESP_LOGI(TAG, "✅ STEP 6: Sistema Modbus avviato con successo");
    
    // ═══════════════════════════════════════════════════════════════════════════
    // SISTEMA PRONTO - RIEPILOGO CONFIGURAZIONE
    // ═══════════════════════════════════════════════════════════════════════════
    
    ESP_LOGI(TAG, "===============================================================================");
    ESP_LOGI(TAG, "🎉 SISTEMA PRONTO! Configurazione completata con successo");
    ESP_LOGI(TAG, "===============================================================================");
    ESP_LOGI(TAG, "📡 Comunicazione:");
    ESP_LOGI(TAG, "   • Protocollo: Modbus RTU");
    ESP_LOGI(TAG, "   • UART%d: TX=GPIO%d, RX=GPIO%d", MB_PORT_NUM, MB_UART_TXD, MB_UART_RXD);
    ESP_LOGI(TAG, "   • Velocità: %d baud, 8-N-1", MB_DEV_SPEED);
    ESP_LOGI(TAG, "   • Slave Address: %d", MB_SLAVE_ADDR);
    ESP_LOGI(TAG, "💡 Hardware:");
    ESP_LOGI(TAG, "   • LED Status: GPIO%d (resistore 220Ω)", LED_GPIO);
    ESP_LOGI(TAG, "   • Convertitore: USB-RS485 Waveshare");
    ESP_LOGI(TAG, "🔄 Operazioni:");
    ESP_LOGI(TAG, "   • Modalità: Lettura sequenziale registri %d-%d", 
             MB_HOLDING_REG_START, MB_HOLDING_REG_START + MB_HOLDING_REG_COUNT - 1);
    ESP_LOGI(TAG, "   • Intervallo: 1 secondo per registro");
    ESP_LOGI(TAG, "   • Funzione: 0x03 (Read Holding Registers)");
    ESP_LOGI(TAG, "🎮 Controllo LED:");
    ESP_LOGI(TAG, "   • Qualsiasi registro = 1 → LED ACCESO");
    ESP_LOGI(TAG, "   • Qualsiasi registro ≠ 1 → LED SPENTO");
    ESP_LOGI(TAG, "   • Errori comunicazione → LED SPENTO");
    ESP_LOGI(TAG, "===============================================================================");
    
    // Attesa prima di avviare il polling
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // ═══════════════════════════════════════════════════════════════════════════
    // STEP 7: AVVIO TASK POLLING MODBUS
    // ═══════════════════════════════════════════════════════════════════════════
    
    ESP_LOGI(TAG, "📋 STEP 7: Avvio task polling Modbus...");
    xTaskCreate(modbus_task, "modbus_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "✅ STEP 7: Task polling avviato con successo");
    
    // ═══════════════════════════════════════════════════════════════════════════
    // LOOP PRINCIPALE - MONITORAGGIO SISTEMA
    // ═══════════════════════════════════════════════════════════════════════════
    
    ESP_LOGI(TAG, "🔄 Avvio loop principale di monitoraggio...");
    
    while (1) {
        // Attesa 5 secondi tra i report di status
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        // Calcola quale registro è stato letto per ultimo
        uint16_t last_register = (current_register_address == MB_HOLDING_REG_START) ? 
                                 MB_HOLDING_REG_START + MB_HOLDING_REG_COUNT - 1 : 
                                 current_register_address - 1;
        
        // Report periodico dello stato sistema
        ESP_LOGI(TAG, "📊 STATUS SISTEMA:");
        ESP_LOGI(TAG, "   • Ultimo registro letto: %d", last_register);
        ESP_LOGI(TAG, "   • Ultimo valore: %d", current_register_value);
        ESP_LOGI(TAG, "   • Dati validi: %s", data_valid ? "SÌ" : "NO");
        ESP_LOGI(TAG, "   • Stato LED: %s", data_valid && current_register_value == 1 ? "ACCESO" : "SPENTO");
        ESP_LOGI(TAG, "   • Prossimo registro: %d", current_register_address);
    }
}