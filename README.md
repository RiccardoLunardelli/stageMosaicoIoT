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
