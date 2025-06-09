# ESP32-C6 Development Environment Setup

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.4.1-blue)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/windows-setup.html)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.4.1-blue)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html)
[![VS Code](https://img.shields.io/badge/VS%20Code-ESP--IDF%20Extension-green)](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension)
[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20Linux-lightgrey)](https://github.com/espressif/esp-idf)

## 📋 Panoramica

Guida completa per configurare l'ambiente di sviluppo ESP32-C6 con VS Code e ESP-IDF in modo semplice e veloce.

## 🎯 Setup in 4 Passi (15-30 minuti)

### ✅ Passo 1: Installa Visual Studio Code

**Windows:**
- Scarica da: https://code.visualstudio.com/
- Esegui installer come **amministratore**
- ⚠️ **Importante**: Seleziona "Add to PATH" durante installazione

**Linux (Ubuntu/Debian):**
```bash
sudo snap install --classic code
```

### ✅ Passo 2: Installa Estensione ESP-IDF

1. Apri VS Code
2. Vai a **Extensions** (`Ctrl+Shift+X`)
3. Cerca: **ESP-IDF**
4. Installa: **ESP-IDF** by Espressif Systems
5. Riavvia VS Code

### ✅ Passo 3: Configurazione Automatica ESP-IDF

1. **Command Palette**: `F1`
2. **Digita**: `ESP-IDF: Configure ESP-IDF Extension`
3. **Seleziona**: `Express Installation`
4. **Configura**:
   - Select download server: Github
   - Select ESP-IDF version: **Find ESP-IDF in your system** 
     - Windows: `Directory inserita durante l'installazione di ESP-IDF (LINK SOPRA INDICATI)`
     - Linux: `Directory inserita durante l'installazione di ESP-IDF (LINK SOPRA INDICATI)`
5. **Click**: `Install` e aspetta **15-30 minuti**

> 🤖 **L'estensione installa automaticamente tutto il necessario:**
> - ESP-IDF Framework completo
> - Toolchain per ESP32-C6  
> - Python + dipendenze
> - OpenOCD, CMake, Ninja
> - Driver USB

### ✅ Passo 4: Test Installazione

1. **Command Palette**: `F1` → `ESP-IDF: Show Examples`
2. **Scegli**: `hello_world`
3. **Crea progetto** in una cartella di test
4. **Verifica target**: `esp32c6` (barra inferiore VS Code)
5. **Build**: `Ctrl+E B`

**🎉 Se il build completa senza errori, sei pronto!**

## 🎛️ Interfaccia VS Code ESP-IDF

### Barra Inferiore (Status Bar)
```
🎯 esp32c6 | 🔌 /dev/ttyUSB0 | 🔨 Build | ⚡ Flash | 📱 Monitor
```

### Comandi Principali (`F1`)
- `ESP-IDF: Set Target` → Seleziona ESP32-C6
- `ESP-IDF: Select Port` → Seleziona porta seriale  
- `ESP-IDF: Build Project` → Compila progetto
- `ESP-IDF: Flash Device` → Flash su ESP32
- `ESP-IDF: Monitor Device` → Monitor seriale
- `ESP-IDF: Show Examples` → Progetti esempio

### Shortcut Essenziali
- **`Ctrl+E B`** → Build
- **`Ctrl+E F`** → Flash
- **`Ctrl+E M`** → Monitor
- **`Ctrl+E D`** → Build + Flash + Monitor

## 🔌 Setup Hardware

### Connessione ESP32-C6
1. **Collega ESP32-C6** al PC via USB
2. **Seleziona porta** nella barra VS Code:
   - Linux: `/dev/ttyUSB0` o `/dev/ttyACM0`
   - Windows: `COM3`, `COM4`, ecc.

### Permessi Linux (se necessario)
```bash
sudo usermod -a -G dialout $USER
# Richiede logout/login
```

## 🧪 Test Completo

### Hello World Test
```
1. F1 → ESP-IDF: Show Examples
2. Cerca: "hello_world"  
3. Crea progetto test
4. Build: Ctrl+E B
5. Flash: Ctrl+E F  
6. Monitor: Ctrl+E M
```

### Output Atteso
```
Hello world!
This is esp32c6 chip with 1 CPU core(s), WiFi 6, BLE
Minimum free heap size: XXXXX bytes
Restarting in 10 seconds...
```

**🚀 Ambiente di sviluppo pronto!**

## 🔌 Schema Cablaggio Infrastruttura Completa


<details>
<summary>Architettura Sistema</summary>

<pre><code>
                 ┌─────────────────┐
                 │   INTERNET      │
                 │                 │
                 └─────────┬───────┘
                           │
                           │ WiFi
                 ┌─────────▼───────┐
                 │    ROUTER       │
                 │   192.168.1.1   │
                 └─────────┬───────┘
                           │ Ethernet/WiFi
                           │
                 ┌─────────▼───────┐
                 │  MQTT BROKER    │
                 │ 192.168.1.100   │
                 │   Port: 1883    │
                 └─────────▲───────┘
                           │ WiFi
                           │
              ┌────────────▼────────────┐
              │      ESP32-C6           │
              │   MODBUS GATEWAY        │
              │                         │
              │  WiFi: 192.168.1.101    │
              │  ID: FRIGO01_V2_CLEAN   │
              └────────────┬────────────┘
                           │ TTL
                           │
              ┌────────────▼────────────┐
              │  USB-RS485 CONVERTER    │
              │    (Waveshare)          │
              └────────────┬────────────┘
                           │ USB
                           │
              ┌────────────▼────────────┐
              │   MODBUS SIMULATOR      │
              │        (PC)             │
              │   Slave Address: 1      │
              │   Registri: 1-8         │
              └─────────────────────────┘
</code></pre>
</details>



<details>
<summary>Schema Elettrico Reale ESP32-C6</summary>

<pre><code>
                    ┌─────────────────────────────┐
                    │         ESP32-C6            │
                    │                             │
┌─────────┐         │  ┌──────┐                   │         ┌─────────────┐
│ Presa   │◄────────┤  │ USB-C│                   │         │    LED      │
│  5V     │ Type-C  │  │ PWR  │                   │         │   Status    │
└─────────┘         │  └──────┘                   │         └──────┬──────┘
                    │                             │                │
                    │                   GPIO18 ───┼────────────────┴────┐
                    │                             │                     │
                    │                    GND ─────┼─────────────────┐   │
                    │                             │                 │   │
                    │                             │         ┌───────▼───▼───┐
                    │  ┌─────────────────────────┐│         │   RESISTOR    │
                    │  │     UART TTL            ││         │     220Ω      │
                    │  │                         ││         └───────┬───────┘
                    │  │   GPIO4 (TX) ──────────►┼┼──┐              │
                    │  │   GPIO5 (RX) ◄──────────┼┼──┼──┐           │ LED
                    │  │   GND ───────────────────┼┼─┼── ┼──┐       │ Cathode
                    │  │                         ││  │   │  │       │
                    │  └─────────────────────────┘│  │   │  │       │
                    └─────────────────────────────┘  │   │  │       │
                                                     │   │  │       │
                    ┌────────────────────────────────┼── ┼──┼───────┘
                    │    USB TO RS485 CONVERTER      │   │  │
                    │       (WAVESHARE)              │   │  │
                    │                                │   │  │
┌─────────┐         │   RXD ◄────────────────────────┘   │  │
│   PC    │◄────USB─┤   TXD ─────────────────────────────┘  │
│(Simul.) │         │   GND ◄───────────────────────────────┘
└─────────┘         │                               │
                    │   RS485 A+ ─────┐             │
                    │   RS485 B- ─────┼──┐          │
                    └─────────────────┼──┼──────────┘
                                      │  │
                           ┌──────────┼──┼──────────┐
                           │          │  │          │
                           │    MODBUS SIMULATOR    │
                           │      (Software)        │
                           │                        │
                           │  A+ ◄─────┘  B- ◄─────┘│
                           │                        │
                           │ Slave Address: 1       │
                           │ Baud Rate: 9600        │
                           │ Data: 8-N-1            │
                           │ Registri: 1-8          │
                           │                        │
                           │ Simula frigorifero     │
                           │ con valori test        │
                           └────────────────────────┘
</code></pre>
</details>



<details>
<summary>Setup Hardware Reale</summary>

<pre><code>
┌─────────────────────────────────────────────────────────────┐
│                    SETUP DI TEST                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  🖥️ PC SVILUPPO                                             │
│  ├─ Software: Modbus Simulator                              │
│  ├─ Porta: USB → Convertitore RS485                         │
│  ├─ Simula: Slave Modbus ID=1                               │
│  └─ Registri: 1-8 con valori test                           │
│                                                             │
│  🔄 CONVERTITORE USB TO RS485                               │
│  ├─ Modello: Waveshare RS232/485/TTL                        │
│  ├─ Input: USB (PC Simulator)                               │
│  ├─ Output TTL: TXD/RXD/GND                                 │
│  └─ Configurazione: 9600-8-N-1                              │
│                                                             │
│  🔧 ESP32-C6 GATEWAY                                        │
│  ├─ Alimentazione: USB-C 5V                                 │
│  ├─ GPIO4: TX → TXD (Convertitore)                          │
│  ├─ GPIO5: RX ← RXD (Convertitore)                          │
│  ├─ GND: Comune con convertitore                            │
│  ├─ GPIO18: LED Status (220Ω resistor)                      │
│  └─ WiFi: Connessione a MQTT Broker                         │
│                                                             │
│  💡 LED STATUS                                              │
│  ├─ GPIO18 → Resistor 220Ω → LED → GND                      │
│  ├─ Indica: Boot, Connecting, Normal, Error                 │
│  └─ Pattern di lampeggio per ogni stato                     │
│                                                             │
└─────────────────────────────────────────────────────────────┘
</code></pre>
</details>



<details>
<summary>Cablaggio Specifico</summary>

<pre><code>
ESP32-C6 Pinout:              Convertitore USB-RS485:
                              
     ┌─────────────┐               ┌─────────────┐
     │   ESP32-C6  │               │ Waveshare   │
     │             │               │ Converter   │
     │   GPIO4 ────┼──────────────►┼─── TXD      │
     │   GPIO5 ◄───┼───────────────┼──► RXD      │
     │   GND ──────┼───────────────┼──── GND     │
     │   GPIO18 ───┼─── LED        │             │
     │   USB-C ────┼─── 5V PWR     │   USB ──────┼─── PC
     └─────────────┘               └─────────────┘
     
LED Connection:
GPIO18 ──► [220Ω] ──► LED+ ──► LED- ──► GND
</code></pre>
</details>


### Note Importanti
- **GPIO4** = TX ESP (trasmette a TXD convertitore)  
- **GPIO5** = RX ESP (riceve da RXD convertitore)
- **Collegamento diretto** TX-TXD e RX-RXD perché il convertitore gestisce la direzione
- **GND comune** essenziale per riferimento segnali
- **LED** con resistore 220Ω
- **Alimentazione separata** USB-C 5V per ESP32-C6
- **Concentratore** Raspberry pi 3.0 --> tramite Mosquitto

## 📚 Risorse Utili

- [Documentazione ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [ESP32-C6 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-c6_datasheet_en.pdf)
- [VS Code ESP-IDF Extension](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension)
- [Esempi ESP-IDF](https://github.com/espressif/esp-idf/tree/master/examples)

## 🤝 Supporto

- **Problemi ESP-IDF**: [GitHub Issues](https://github.com/espressif/esp-idf/issues)
- **Estensione VS Code**: [GitHub Extension](https://github.com/espressif/vscode-esp-idf-extension/issues)
- **Community**: [ESP32 Forum](https://esp32.com/)

# ESP32-C6 RS485 Modbus Master – Versione 1.0

[![Platform](https://img.shields.io/badge/Platform-ESP32--C6-blue)](https://www.espressif.com/en/products/socs/esp32-c6)
[![Modbus](https://img.shields.io/badge/Protocol-Modbus%20RTU-lightgrey)](https://modbus.org/)
[![Version](https://img.shields.io/badge/Version-1.0-green)]()

## 🔖 Versione di riferimento
Questa versione corrisponde al [commit iniziale v1.0](https://github.com/soufian774/stageMosaico/commit/22749744fa6fb87ecd782e3f4cd4104ce1107d3e) del progetto.

## 📋 Panoramica

Questa versione del progetto implementa una semplice connessione Modbus RTU su RS485 tramite ESP32-C6.  
Il codice è sviluppato in C++ usando ESP-IDF e consente la comunicazione seriale tra ESP32 e un dispositivo slave simulato.

## 🚀 Funzionalità Versione 1.0

- Configurazione porta UART1 (TX: GPIO4, RX: GPIO5)
- Baudrate: `9600`, 8N1
- Compatibile con convertitori USB-RS485
- Include configurazione base per driver Modbus Master (`esp_modbus_master`)
- LED di stato su `GPIO18` (da utilizzare per feedback futuri)

## 🔧 Pinout ESP32-C6

| Funzione | GPIO  | Descrizione              |
|----------|-------|--------------------------|
| RS485 TX | GPIO4 | Trasmissione UART        |
| RS485 RX | GPIO5 | Ricezione UART           |
| LED      | GPIO18| LED di stato (opzionale) |

## 🛠️ Requisiti

- ESP32-C6 con ESP-IDF installato (versione >= 5.0)
- Convertitore RS485 (es. Waveshare TTL to RS485)
- Simulatore Modbus su PC 
- Collegamento GND comune tra dispositivi

## ⚙️ Compilazione

```bash
idf.py set-target esp32c6
idf.py build
idf.py flash
idf.py monitor
```

## 📚 Riferimenti

- [ESP-IDF UART Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html)
- [ESP Modbus Master](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/mb_master.html)
- [Modbus Protocol](https://modbus.org/)

# ESP32-C6 MODBUS RTU MASTER + MQTT - Versione 2.0

[![Platform](https://img.shields.io/badge/Platform-ESP32--C6-blue)](https://www.espressif.com/en/products/socs/esp32-c6)
[![Modbus](https://img.shields.io/badge/Protocol-Modbus%20RTU-lightgrey)](https://modbus.org/)
[![MQTT](https://img.shields.io/badge/Protocol-MQTT%20RTU-lightgrey)](https://mqtt.org/)
[![Version](https://img.shields.io/badge/Version-2.0-green)]()


## 📋 Descrizione

Sistema completo di acquisizione dati Modbus RTU con trasmissione MQTT per il monitoraggio di frigoriferi industriali. Il sistema legge 8 registri holding (1-8) da un dispositivo slave Modbus e li trasmette via MQTT con gestione intelligente LED e auto-recovery degli errori.

## 🔧 Hardware Setup

### Componenti Richiesti
- **ESP32-C6 Development Board**
- **Convertitore USB-RS485** 
- **LED giallo** per status sistema (con resistore 220Ω)
- **LED rosso** per segnalazione errori (con resistore 220Ω)

### Schema di Collegamento

```
ESP32-C6          RS485 Converter        
--------          ---------------        
GPIO4 (TX)    →   TXD                    
GPIO5 (RX)    ←   RXD                    
GND           →   GND                    
5V/3.3V       →   VCC                    

GPIO18        →   LED Giallo + Resistore 220Ω → GND
GPIO19        →   LED Rosso + Resistore 220Ω → GND
```

## ⚙️ Configurazione

### Parametri Modbus RTU
- **Velocità**: 9600 baud
- **Formato**: 8-N-1 (8 bit dati, nessuna parità, 1 stop bit)
- **Indirizzo Slave**: 1
- **Registri**: 1-8 (1-based addressing - HOLDING REGISTERS)
- **Timeout**: 1000ms

### Configurazione Wi-Fi
```c
#define WIFI_SSID      ""
#define WIFI_PASSWORD  ""
```

### Configurazione MQTT
```c
#define MQTT_BROKER_URI ""
#define MQTT_CLIENT_ID   ""
```

### Topic MQTT
- **Dati operativi**: `sensori/frigo01/data_static` (ogni 10s)
- **Status sistema**: `sensori/frigo01/status` (ogni 30s)
- **Debug raw**: `sensori/frigo01/debug_raw` (con ogni lettura)

## 📊 Mappatura Registri

| Registro | Descrizione | Unità | Scaling | Range |
|----------|-------------|-------|---------|-------|
| **Reg1** | Temperatura evaporatore | °C | /10 | -40.0 ÷ +85.0 |
| **Reg2** | Setpoint temperatura | °C | /10 | -40.0 ÷ +85.0 |
| **Reg3** | Umidità relativa | % | /10 | 0.0 ÷ 100.0 |
| **Reg4** | Pressione sistema | bar | /100 | 0.00 ÷ 25.00 |
| **Reg5** | Status word | - | 1:1 | Bit field |
| **Reg6** | Runtime compressore | ore | 1:1 | 0 ÷ 65535 |
| **Reg7** | Consumo energia | kW | /100 | 0.00 ÷ 655.35 |
| **Reg8** | Registro allarmi | - | 1:1 | Bit field |

## 🔍 Decodifica Registro Status (Reg5)

| Bit | Descrizione | Valore 0 | Valore 1 |
|-----|-------------|----------|----------|
| 0 | Compressore | SPENTO | ACCESO |
| 1 | Ventilatore evaporatore | SPENTO | ACCESO |
| 2 | Ventilatore condensatore | SPENTO | ACCESO |
| 3 | Modalità operativa | NORMALE | SBRINAMENTO |
| 4 | Controllo | AUTOMATICO | MANUALE |
| 5 | Stato sistema | ATTIVO | STANDBY |
| 6 | Stato porta | CHIUSA | APERTA |
| 7 | Sensore temperatura | NON CONFERMATO | OK |
| 8-15 | Riservati | - | Altri stati |

## 🚨 Decodifica Registro Allarmi (Reg8)

| Bit | Allarme | Descrizione |
|-----|---------|-------------|
| 0 | Temperatura alta | Superato limite massimo |
| 1 | Pressione alta | Pressione oltre soglia |
| 2 | Sensore difettoso | Guasto sensore temperatura |
| 3 | Comunicazione | Perdita comunicazione |
| 4 | Compressore fault | Malfunzionamento compressore |
| 5 | Manutenzione | Richiesta manutenzione |
| 6 | Porta aperta | Porta aperta troppo a lungo |
| 7 | Filtro sporco | Necessaria pulizia filtro |
| 8 | Tensione anomala | Problemi alimentazione |
| 9 | Sovratemperatura motore | Surriscaldamento motore |
| 10-15 | Riservati | Espansioni future |


## 💡 Logica LED

### LED Giallo (GPIO18) - Status Sistema
- **Boot**: Lampeggio rapido (200ms) - Inizializzazione
- **Connecting**: Lampeggio medio (500ms) - Connessione Wi-Fi/MQTT
- **Normal**: Impulso breve ogni 2s - Sistema operativo
- **Data TX**: 3 lampeggi rapidi - Trasmissione dati
- **RS485 Comm**: 2 lampeggi veloci - Comunicazione Modbus
- **Error**: Lampeggio continuo - Errore sistema

### LED Rosso (GPIO19) - Errori
Il LED rosso si accende quando si verifica una delle seguenti condizioni:
- Wi-Fi disconnesso
- MQTT disconnesso  
- Allarmi Modbus attivi (Reg8 ≠ 0)
- Errori comunicazione RS485

## 📡 Formato Messaggi MQTT

### Data Static Message
```json
{
  "tipo": "DATA_STATIC",
  "id_nodo": "FRIGO01",
  "versione": "2.0.0_CLEAN_NO_BIT8",
  "data_valid": true,
  "paramET_temp": 4.5,
  "paramET_setpoint": 5.0,
  "paramET_umid": 75.2,
  "paramET_pressure": 2.45,
  "paramET_status": 7,
  "paramET_runtime": 1250,
  "paramET_power": 1.85,
  "paramET_alarms": 0,
  "compressore_stato": "ACCESO",
  "allarmi_attivi": false,
  "led_error_active": false,
  "comp_acceso": true,
  "fan_evap_acceso": true,
  "fan_cond_acceso": true,
  "sbrinamento_attivo": false,
  "modalita_manuale": false,
  "porta_aperta": false,
  "letture_rs485": 125,
  "letture_ok": 120,
  "timestamp": "2025-06-09T14:30:25Z",
  "message_id": 125
}
```

### System Status Message
```json
{
  "tipo": "SYSTEM_STATUS",
  "id_nodo": "FRIGO01",
  "versione": "2.0.0_CLEAN_NO_BIT8",
  "system_state": "RUNNING",
  "led_state": "NORMAL",
  "led_error_active": false,
  "wifi_connected": true,
  "mqtt_connected": true,
  "data_initialized": true,
  "rs485_reads": 125,
  "successful_reads": 120,
  "messages_sent": 125,
  "errors": 5,
  "last_read_sec_ago": 2,
  "uptime_sec": 3600,
  "free_heap": 180000,
  "timestamp": "2025-06-09T14:30:25Z"
}
```

### Debug Raw Message
```json
{
  "tipo": "DEBUG_RAW",
  "id_nodo": "FRIGO01",
  "raw_reg1": 45,
  "raw_reg2": 50,
  "raw_reg3": 752,
  "raw_reg4": 245,
  "raw_reg5": 7,
  "raw_reg6": 1250,
  "raw_reg7": 185,
  "raw_reg8": 0,
  "timestamp": "2025-06-09T14:30:25Z"
}
```

## 🚀 Installazione e Uso

### 1. Configurazione ESP-IDF
```bash
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh
. ./export.sh
```

### 2. Compilazione
```bash
idf.py set-target esp32c6
idf.py build
```

### 3. Flash
```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

### 4. Personalizzazione
Modifica le seguenti definizioni in base al tuo setup:
```c
#define WIFI_SSID      "YOUR_WIFI_SSID"
#define WIFI_PASSWORD  "YOUR_WIFI_PASSWORD"
#define MQTT_BROKER_URI "mqtt://YOUR_BROKER_IP:1883"
#define MB_SLAVE_ADDR   1  // Indirizzo del tuo dispositivo
```

## 🔧 Test e Debug

### Test Comunicazione Modbus
Il sistema esegue un test iniziale alla connessione per verificare la comunicazione:
```
🧪 === TEST INIZIALE INDIRIZZAMENTO 1-BASED ===
🎯 Lettura registri 1-8 (non 0-7)...
```

### Verifica LED
- **LED Giallo lampeggiante**: Sistema in inizializzazione/connessione
- **LED Giallo impulsi regolari**: Sistema operativo normale
- **LED Rosso acceso**: Presenza errori o allarmi

### Debug via Serial Monitor
Il sistema fornisce log dettagliati via seriale:
```
📈 STATISTICHE:
   🎯 Sistema: OK | MQTT: CONNESSO | Dati: VALIDI  
   📊 RS485: 125 letture, 120 OK (96.0% successo)
   📤 MQTT: 125 messaggi inviati | ❌ 5 errori totali
```

### Test Valori Simulati
Per testare la decodifica allarmi, usa il valore **1791** nei registri di test (rappresenta tutti gli allarmi attivi eccetto il bit 8 batteria che è stato rimosso).

## 🛠️ Auto-Recovery

Il sistema implementa funzionalità di auto-recovery:
- **Riconnessione Wi-Fi automatica** in caso di disconnessione
- **Retry comunicazione Modbus** con cambio stato dopo errori consecutivi
- **Ripristino automatico** dello stato normale dopo recupero errori
- **Monitoraggio memoria heap** per rilevare memory leak

## 📊 Monitoring

### Statistiche Sistema
- Contatore messaggi MQTT inviati
- Tasso di successo letture RS485
- Tempo dall'ultima lettura riuscita
- Uptime sistema
- Memoria heap disponibile

### Stati Sistema
- `SYS_INIT`: Inizializzazione
- `SYS_WIFI_CONNECTING`: Connessione Wi-Fi
- `SYS_MQTT_CONNECTING`: Connessione MQTT  
- `SYS_RUNNING`: Operativo normale
- `SYS_ERROR_RS485`: Errore comunicazione
- `SYS_ERROR_COMM`: Errore comunicazione generale

## 🔒 Note di Sicurezza

- Il sistema non include autenticazione MQTT (configurabile)
- Validazione CRC16 per tutte le comunicazioni Modbus
- Gestione timeout per evitare blocchi
- Monitoraggio continuo dello stato delle connessioni

## 📞 Supporto

Per problemi o domande contattare:
- **Soufian Markouni** 
- **Riccardo Lunardelli**

**Versione documentazione**: 1.0  
**Compatibile con**: ESP32-C6, ESP-IDF v5.x
