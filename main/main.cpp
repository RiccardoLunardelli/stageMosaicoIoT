/**
 * ESP32-C6 MODBUS - VERSIONE CHE FUNZIONAVA (3 lampeggi LED)
 * Configurazione testata e funzionante
 */

#include "mbcontroller.h"
#include "driver/gpio.h"
#include "driver/uart.h"

// Configurazioni
#define CONFIG_MB_UART_PORT_NUM 1
#define CONFIG_MB_UART_BAUD_RATE 19200
#define CONFIG_MB_UART_TXD 17   // GPIO 4
#define CONFIG_MB_UART_RXD 16   // GPIO 5  
#define CONFIG_MB_UART_RTS 6    // GPIO 6 - DE
#define CONFIG_MB_UART_RE  7    // GPIO 7 - RE

#define MB_PORT_NUM     ((uart_port_t)CONFIG_MB_UART_PORT_NUM)
#define MB_DEV_SPEED    CONFIG_MB_UART_BAUD_RATE
#define FRIGO_SLAVE_ADDR    197
#define PRESSURE_REG_ADDR   2013
#define STATUS_LED_GPIO 2

// Strutture richieste
typedef struct { uint16_t dummy_data; } holding_reg_params_t;
typedef struct { uint16_t dummy_data; } input_reg_params_t;
typedef struct { uint8_t dummy_data; } coil_reg_params_t;
typedef struct { uint8_t dummy_data; } discrete_reg_params_t;

holding_reg_params_t holding_reg_params = {0};
input_reg_params_t input_reg_params = {0};
coil_reg_params_t coil_reg_params = {0};
discrete_reg_params_t discrete_reg_params = {0};

enum { CID_PRESSURE_EVAP = 0, CID_COUNT };

// LED
static void init_led(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << STATUS_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&cfg);
    gpio_set_level((gpio_num_t)STATUS_LED_GPIO, 0);
}

static void blink(int times) {
    for(int i = 0; i < times; i++) {
        gpio_set_level((gpio_num_t)STATUS_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level((gpio_num_t)STATUS_LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Controllo DE/RE
static void set_de_re_mode(bool tx_mode) {
    if (tx_mode) {
        gpio_set_level((gpio_num_t)CONFIG_MB_UART_RTS, 1);  // DE HIGH
        gpio_set_level((gpio_num_t)CONFIG_MB_UART_RE, 1);   // RE HIGH
    } else {
        gpio_set_level((gpio_num_t)CONFIG_MB_UART_RTS, 0);  // DE LOW
        gpio_set_level((gpio_num_t)CONFIG_MB_UART_RE, 0);   // RE LOW
    }
    vTaskDelay(pdMS_TO_TICKS(2));
}

// Parametri Modbus
const mb_parameter_descriptor_t device_parameters[] = {
    { 
        CID_PRESSURE_EVAP, 
        "Pressure", 
        "bar", 
        FRIGO_SLAVE_ADDR, 
        MB_PARAM_HOLDING,
        PRESSURE_REG_ADDR, 
        (mb_descr_size_t)1,         // Cast esplicito
        (mb_descr_size_t)1,         // Cast esplicito  
        PARAM_TYPE_U16, 
        (mb_descr_size_t)2,         // Cast esplicito
        {0, 1000, 1}, 
        PAR_PERMS_READ_WRITE_TRIGGER 
    }
};

// Task lettura (SENZA LOG per evitare interferenze)
static void read_task(void *arg) {
    uint32_t success = 0, errors = 0, cycles = 0;
    
    while (1) {
        cycles++;
        const mb_parameter_descriptor_t* param = NULL;
        
        if (mbc_master_get_cid_info(CID_PRESSURE_EVAP, &param) == ESP_OK) {
            set_de_re_mode(false);  // RX mode
            
            uint16_t pressure = 0;
            uint8_t type = 0;
            
            esp_err_t err = mbc_master_get_parameter(CID_PRESSURE_EVAP, 
                                                   (char*)"Pressure",  // Cast esplicito
                                                   (uint8_t*)&pressure, 
                                                   &type);
            
            if (err == ESP_OK) {
                success++;
                blink(3); // SUCCESSO!
            } else {
                errors++;
                blink(1); // ERRORE
            }
        } else {
            blink(2); // Errore config
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000)); // 2 secondi
    }
}

// Inizializzazione
static esp_err_t init_modbus(void) {
    mb_communication_info_t comm;
    comm.port = MB_PORT_NUM;
    comm.mode = MB_MODE_RTU;
    comm.baudrate = MB_DEV_SPEED;
    comm.parity = MB_PARITY_NONE;
    
    void* handler = NULL;
    
    if (mbc_master_init(MB_PORT_SERIAL_MASTER, &handler) != ESP_OK) {
        blink(10); return ESP_FAIL;
    }
    
    if (mbc_master_setup((void*)&comm) != ESP_OK) {
        blink(10); return ESP_FAIL;
    }
    
    if (uart_set_pin(MB_PORT_NUM, 
                    CONFIG_MB_UART_TXD, CONFIG_MB_UART_RXD, 
                    CONFIG_MB_UART_RTS, UART_PIN_NO_CHANGE) != ESP_OK) {
        blink(10); return ESP_FAIL;
    }
    
    if (mbc_master_start() != ESP_OK) {
        blink(10); return ESP_FAIL;
    }
    
    // Configura DE/RE separati
    gpio_config_t de_re_cfg = {
        .pin_bit_mask = (1ULL << CONFIG_MB_UART_RTS) | (1ULL << CONFIG_MB_UART_RE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&de_re_cfg);
    
    set_de_re_mode(false); // Modalità RX iniziale
    
    if (mbc_master_set_descriptor(&device_parameters[0], 1) != ESP_OK) {
        blink(10); return ESP_FAIL;
    }
    
    return ESP_OK;
}

extern "C" void app_main(void) {
    // NESSUN LOG! Solo LED
    
    init_led();
    blink(5); // Sistema avviato
    
    if (init_modbus() == ESP_OK) {
        blink(2); // Init OK
        xTaskCreate(read_task, "read_task", 4096, NULL, 5, NULL);
    }
    
    // Loop silenzioso
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}