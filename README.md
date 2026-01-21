# ESP32 Modbus RTU Slave – Gas Valve

Firmware Modbus RTU **slave** per ESP32, progettato con organizzazione dati *Siemens-oriented* e compatibile con **Siemens ET 200SP**.

Il dispositivo ESP32 rappresenta una periferica di campo (slave) che:

* riceve **comandi** dal master (PLC)
* espone **stati** e **misure**
* gestisce **parametri** tramite holding register

Il progetto è pensato per:

* sviluppo con **Arduino IDE ver 2.3.6**
* board per compilazione **DOIT ESP32 DEVKIT1** 

## Hardware

### Scheda

* **ESP32 DEVKITC-32E**

### Interfaccia Modbus

* RS485 tramite **MAX14852** (isolamento elettrico)

## Modello dati interno

il firmware ha implementato:
* macchia a stati finita (FSM) per gestione I/O;
* watchdog;
* debounce per gli I/O;
* modbus.

la FSM ha una struttura dati:

```c
typedef enum {
    ST_START = 0,
    ST_READY,
    ST_FAULT,
    ST_RESET
} SystemState;

typedef struct {
    SystemState state;
    uint32_t    t_state;     // timestamp ingresso stato
    uint32_t    fault_code;  // opzionale, 0 = no fault
    uint32_t    t_valve_closed;  // timer monitor chiusura (se valvola resta troppo chiusa per più di un tempo)
} SystemCtx;

SystemCtx sys = {
    .state = ST_START //inizializzo sys sullo stato start
};
```

la struttura dati del modbus è:
```c
struct {
  uint8_t states;     // 1 byte -> discrete inputs (read only)
  uint8_t commands;   // 1 byte -> coils (R/W)
  int16_t setpoint;   // holding register (R/W)
  int16_t measure;    // input register (R only)
} deviceData;
```

con Equivalente Siemens (DB)

```plaintext
STRUCT GasValve
  Commands : BYTE;
  States   : BYTE;
  Setpoint : INT;
  Measure  : INT;
END_STRUCT
```
per il debounce si usa:

```c
typedef struct {
    bool     stable_state;
    bool     last_raw;
    uint32_t t_last_change;
} Debounce_t;
```

## Logica firmware

* Le variabili applicative (`deviceData`) sono indipendenti dalla mappa Modbus
* Nel `loop()` avviene la sincronizzazione:

  * `deviceData → registri Modbus`
* Le callback Modbus aggiornano solo il modello dati

## FSM – Finite State Machine

Il comportamento della Gas Valve PS è modellato tramite una **macchina a stati finiti (FSM)**, basata sulla combinazione di:

* **comandi** ricevuti via Modbus (`Commands`)
* **stati interni** calcolati dal firmware (`States`)
* **misure** e condizioni operative

### Stati principali

| Stato  |  Descrizione        |
| ------ |  ------------------ |
| START  | inizializzazione    |
| READY  |  Dispositivo pronto |
| FAULT  |  stato di fault     |
| RESET  |  tentativo di reset |

**Autore:** Bruno Laterza
