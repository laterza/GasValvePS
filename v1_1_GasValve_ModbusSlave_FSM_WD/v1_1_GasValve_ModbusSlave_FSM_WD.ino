/*  
  Tabella IO scheda GasValve verso Campo:
   morsetto:            I/O ESP32:
   J25      ON1         IO21 <-
   J26      READY       IO18 ->
   J10      DAC         IO34 <-
   J27      I MONITOR   --
   J24      MODBUS      IO16,17,4 (modbus) RX2,TX2,RTS/RE <->
   J19      FAULT       IO19 ->
   J4       RESET       IO22 <-

  Tabella IO scheda GasValve interno scheda:
   I/O ESP32:
   IO35     DAC <-
   IO25     RESET DRIVER IGBT ->
   IO26     FAULT DRIVER IGBT <-
   IO27     READY DRIVER IGBT <-
   IO14     DAC
   IO13     FIRE IGBT
   IO2      DE (modbus)

  tabella variabili modbus:
  ===========================================================
    Modbus RTU Slave - ESP32 Gas Valve
    -----------------------------------------------------------
    Slave Address : 1
    Baudrate      : 9600 8N1
    Interface     : RS485 (MAX485)
    -----------------------------------------------------------

    MODBUS MAP (Siemens-oriented)

    COILS (0xxxx)  - Commands BYTE (R/W)
    ----------------------------------------------------------
    Address   Bit   Siemens DB BOOL      Description
    00001     0     Commands.0           Enable
    00002     1     Commands.1           Open
    00003     2     Commands.2           Close
    00004     3     Commands.3           Reset
    00005     4     Commands.4           Reserved
    00006     5     Commands.5           Reserved
    00007     6     Commands.6           Reserved
    00008     7     Commands.7           Reserved

    DISCRETE INPUTS (1xxxx) - States BYTE (R)
    ----------------------------------------------------------
    Address   Bit   Siemens DB BOOL      Description
    10001     0     States.0             Ready
    10002     1     States.1             Opened
    10003     2     States.2             Closed
    10004     3     States.3             Alarm
    10005     4     States.4             Reserved
    10006     5     States.5             Reserved
    10007     6     States.6             Reserved
    10008     7     States.7             Reserved

    HOLDING REGISTERS (4xxxx) - Parameters (R/W)
    ----------------------------------------------------------
    Address   Siemens Type   Description
    40001     INT            Setpoint

    INPUT REGISTERS (3xxxx) - Measurements (R)
    ----------------------------------------------------------
    Address   Siemens Type   Description
    30001     INT            Measure

    -----------------------------------------------------------
    Siemens DB Equivalent:

    STRUCT GasValve
      Commands : BYTE;   // mapped from Coils 00001..00008
      States   : BYTE;   // mapped from Ists 10001..10008
      Setpoint : INT;    // 40001
      Measure  : INT;    // 30001
    END_STRUCT
  ===========================================================


  TABELLA ERRORI:
  | Codice | Nome              | Descrizione                    | Stato |
  | ------ | ----------------- | ------------------------------ | ----- |
  | 0      | NONE              | Nessun errore                  | READY |
  | 1      | POWER_ON_MISSING  | Consenso power_on assente      | FAULT |
  | 2      | HW_DRIVER         | Fault dal driver ISO5451       | FAULT |
  | 3      | WATCHDOG          | Watchdog CPU scaduto           | FAULT |
  | 4      | VALVE_CLOSED_LONG | Valvola comandata chiusa > 10s | FAULT |
  | 5      | RESET_TIMEOUT     | Reset driver non riuscito      | FAULT |
  | 255    | STATE_INVALID     | Stato FSM non valido           | FAULT |

  TABELLA FSM:
  | ID  | Stato FSM | Evento / Condizione         | Possibile Causa                   | Effetto sul Sistema        | Rilevazione              | Azione FSM               | Note di Progetto              |
  | --- | --------- | --------------------------- | --------------------------------- | -------------------------- | ------------------------ | ------------------------ | ----------------------------- |
  | F1  | START     | `power_on = 0`              | Alimentazione assente o instabile | Sistema non avviabile      | GPIO IO21 + debounce     | Transizione a `ST_FAULT` | Scelta conservativa, PLC-like |
  | F2  | START     | `power_on = 1` ma instabile | Rampa lenta o rimbalzi            | Avvio errato               | TON `power_ok`           | Attesa `power_ok`        | `POWER_OK_DELAY_MS = 50 ms`   |
  | F3  | READY     | `drv_fault = 1`             | Corto IGBT, sovracorrente         | Comando valvola non sicuro | Ingresso IO26            | Transizione a `ST_FAULT` | Fault hardware latched        |
  | F4  | READY     | `!power_ok`                 | Caduta alimentazione              | Stato indeterminato        | TON power_ok             | Transizione a `ST_FAULT` | Evita fault spurii            |
  | F5  | READY     | `watchdog_err = 1`          | Blocco firmware                   | Sistema non affidabile     | Watchdog HW (futuro)     | Transizione a `ST_FAULT` | Da integrare                  |
  | F6  | READY     | `valve_cmd_mon = 1` > 10 s  | Valvola chiusa troppo a lungo     | Rischio di mancato flusso  | Timer `t_valve_closed`   | Transizione a `ST_FAULT` | Monitor solo osservativo      |
  | F7  | FAULT     | Fault persistente           | Errore non risolto                | Sistema bloccato           | Stato latched            | Rimane in `ST_FAULT`     | Richiede reset esterno        |
  | F8  | FAULT     | `reset_cmd = 1`             | Comando PLC / pulsante            | Tentativo ripristino       | GPIO IO22                | Transizione a `ST_RESET` | Reset controllato             |
  | F9  | RESET     | `drv_fault = 1`             | Fault ancora presente             | Reset inefficace           | IO26                     | Ritorno a `ST_FAULT`     | Evita ripartenze forzate      |
  | F10 | RESET     | Timeout OK                  | Sistema stabile                   | Rientro operativo          | Timer `RESET_TIMEOUT_MS` | Transizione a `ST_READY` | Ritardo stile PLC             |
  | F11 | ANY       | Stato FSM invalido          | Corruzione memoria / bug          | Comportamento non noto     | `default:`               | Forza `ST_FAULT`         | Fail-safe                     |

    stato : ST_START
    | Evento / Condizione | Azione                    | Stato Successivo |
    | ------------------- | ------------------------- | ---------------- |
    | `power_ok == true`  | Inizializza contatori     | **ST_READY**     |
    | `power_ok == false` | `fault_code = POWER_FAIL` | **ST_FAULT**     |

    stato : ST_READY
    | Evento / Condizione                                 | Azione                       | Stato Successivo |
    | --------------------------------------------------- | ---------------------------- | ---------------- |
    | `drv_fault == true`                                 | `fault_code = HW_FAULT`      | **ST_FAULT**     |
    | `power_ok == false`                                 | `fault_code = POWER_FAIL`    | **ST_FAULT**     |
    | `watchdog_err == true`                              | `fault_code = WD_FAULT`      | **ST_FAULT**     |
    | `valve_cmd_mon == true` per > `VALVE_CLOSED_MAX_MS` | `fault_code = VALVE_TIMEOUT` | **ST_FAULT**     |
    | *(nessuna condizione)*                              | –                            | **ST_READY**     |

    stato : ST_FAULT
    | Evento / Condizione | Azione                   | Stato Successivo |
    | ------------------- | ------------------------ | ---------------- |
    | `reset_cmd == true` | Avvia reset temporizzato | **ST_RESET**     |
    | *(altrimenti)*      | Fault latched            | **ST_FAULT**     |

    stato : ST_RESET
    | Evento / Condizione                    | Azione         | Stato Successivo |
    | -------------------------------------- | -------------- | ---------------- |
    | `power_ok == false`                    | –              | **ST_FAULT**     |
    | `drv_fault == true`                    | –              | **ST_FAULT**     |
    | `watchdog_err == true`                 | –              | **ST_FAULT**     |
    | Tutte OK **e** `Δt ≥ RESET_TIMEOUT_MS` | Cancella fault | **ST_READY**     |
    | *(timeout non scaduto)*                | Attesa         | **ST_RESET**     |



*/

// WATCHDOG ->
//  _   _   __ _____ ____  _ __   __   __  
// | | | | /  \_   _/ _/ || | _\ /__\ / _] 
// | 'V' || /\ || || \_| >< | v | \/ | [/\ 
// !_/ \_!|_||_||_| \__/_||_|__/ \__/ \__/ 
//

#include "esp_task_wdt.h"
#include "esp_system.h" //watchdog
#define WDT_TIMEOUT_S 3   // timeout watchdog in secondi

static esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true
};
// <- END WATCHDOG

// STATE MACHINE ->
//   __ _____ __ _____ ___   __ __  __   ____ __  _ ___  
// /' _/_   _/  \_   _| __| |  V  |/  \ / _/ |  \| | __| 
// `._`. | || /\ || | | _|  | \_/ | /\ | \_| | | ' | _|  
// |___/ |_||_||_||_| |___| |_| |_|_||_|\__/_|_|\__|___| 
//
#include <stdbool.h>
#include <stdint.h>

#define RESET_TIMEOUT_MS 1000
#define VALVE_CLOSED_MAX_MS 10000

//codice errore
#define FAULT_NONE              0
#define FAULT_POWER_FAIL        1
#define FAULT_HW_DRV            2
#define FAULT_VALVE_TIMEOUT     3
#define FAULT_WATCHDOG          4

//pseudo alias degli ingressi digitali:
bool power_ok;        // alimentazioni ready
bool power_on;        // comando ingresso on --- OK IO21  -> power_on        (accensione)
bool reset_cmd;       // comando reset (PLC o pulsante) --- IO22  -> reset_cmd       (reset esterno)
bool valve_cmd_mon;   // monitor limite tempo apertura valvola --- IO34  -> valve_cmd_mon   (monitor comando valvola – solo osservato)
bool drv_fault;       // IO drv flt --- IO26  -> drv_fault
bool drv_ready;       // IO27
bool watchdog_err = false;  // per ora placeholder --- booleano a servizio della cpu... io aggiungerei il watchdog hardware dell'ESP32.

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

#define POWER_OK_DELAY_MS 50 //per il TON del power on
static uint32_t t_power_on = 0;
// <- END STATE MACHINE

// DEBOUNCE ->
//  __  ___ __  __  _  _ __  _  ______  
// | _\| __|  \/__\| || |  \| |/ _/ __| 
// | v | _|| -< \/ | \/ | | ' | \_| _|  
// |__/|___|__/\__/ \__/|_|\__|\__/___| 
//
#define DEBOUNCE_MS 2

typedef struct {
    bool     stable_state;
    bool     last_raw;
    uint32_t t_last_change;
} Debounce_t;

Debounce_t db_power_on     = {0}; //così inizializzo la struttura a zero...
Debounce_t db_reset_cmd    = {0};
Debounce_t db_valve_cmd    = {0};
Debounce_t db_drv_fault    = {0};
Debounce_t db_drv_ready    = {0};
// <- END DEBOUNCE

#include <ModbusRTU.h>

#define RTS_RE 4 //pin dell'abilitazione lettura/scrittura
#define TXD 17 //modbus trasmissione seriale
#define RXD 16 //modbus ricezione seriale

// ================= PIN MAPPING =================
// Ingressi esterni
#define PIN_POWER_ON        21  // IO21 - accensione
#define PIN_RESET_CMD       22  // IO22 - reset esterno
#define PIN_VALVE_CMD_MON   34  // IO34 - monitor comando valvola (solo input)

// Driver ISO5451
#define PIN_DRV_FAULT       26  // IO26 - fault driver - ingresso
#define PIN_DRV_READY       27  // IO27 - ready driver - ingresso
#define PIN_DRV_RESET       25  // IO25 - reset driver - uscita

// Uscite
#define PIN_READY_OUT       18  // IO18 - pronto
#define PIN_FAULT_OUT       19  // IO19 - fault


// MODBUS ->
//  __ __  __  __  __ _  _   __  
// |  V  |/__\| _\|  \ || |/' _/ 
// | \_/ | \/ | v | -< \/ |`._`. 
// |_| |_|\__/|__/|__/\__/ |___/ 
//
HardwareSerial MySerial(2); // UART2: GPIO16=RX, GPIO17=TX
ModbusRTU mb;

// Struttura dati condivisa
struct {
  uint8_t states;     // 1 byte -> discrete inputs (read only)
  uint8_t commands;   // 1 byte -> coils (R/W)
  int16_t setpoint;   // holding register (R/W)
  int16_t measure;    // input register (R only)
} deviceData;

// Callback per coil write
uint16_t cbCoil(TRegister* reg, uint16_t val) {
  uint8_t bit = reg->address.address; //l'offset del coil
  if (bit < 8) {
    if (val)
      deviceData.commands |=  (1 << bit); // aggiorno il byte comandi
    else
      deviceData.commands &= ~(1 << bit); // aggiorno il byte comandi
  } 
  return val; // restituisce il valore accettato
}

// Callback per holding register write
uint16_t cbHreg(TRegister* reg, uint16_t val) {
  deviceData.setpoint = (int16_t)val; // aggiorno il setpoint
  return val;
}
// <- MODBUS

void setup() {
  // UART2 config: 9600 8N1 modbus
  MySerial.begin(9600, SERIAL_8N1, RXD, TXD);

  // connetto la serial USB
  Serial.begin(115200);
  delay(100);
  Serial.println("Accensione ESP32-Gasvalve v0_0");

//WATCHDOG->
    esp_reset_reason_t reason = esp_reset_reason();
    if (reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT) {
        watchdog_err = true;
        Serial.println("BOOT: Reset da Watchdog");
    } else {
        Serial.println("BOOT: Reset normale");
    }

  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);   // aggiunge il task loop()
//<-WATCHDOG

  // MAX485, DE/RE pin
  mb.begin(&MySerial, RTS_RE); // -1 = nessun pin DE/RE, metti il tuo se serve
  mb.slave(1);             // Indirizzo slave = 1 (imposto l'indirizzo slave nel bus modbus)

  // Inizializzazione variabili
  deviceData.states   = 0x01;
  deviceData.commands = 0x00;
  deviceData.setpoint = 100;
  deviceData.measure  = 25;

  // // Mappatura coils (0xxxx) -> comandi
  // mb.addCoil(0, deviceData.commands);     // indirizzo 00001
  // mb.onSetCoil(0, cbCoil);

  // Mappatura coils (0xxxx) -> comandi
  mb.addCoil(0, false, 8);     // set byte dei coil
  mb.onSetCoil(0, cbCoil);

  // Mappatura discrete inputs (1xxxx) -> stati
  mb.addIsts(0, false, 8);       // indirizzo 10001

  // Mappatura holding registers (4xxxx) -> setpoint
  mb.addHreg(0, deviceData.setpoint);     // indirizzo 40001
  mb.onSetHreg(0, cbHreg);

  // Mappatura input registers (3xxxx) -> misura
  mb.addIreg(0, deviceData.measure);      // indirizzo 30001

  // setup pin I/O
  pinMode(PIN_POWER_ON,      INPUT);
  pinMode(PIN_RESET_CMD,     INPUT);
  pinMode(PIN_VALVE_CMD_MON, INPUT);
  pinMode(PIN_DRV_FAULT,     INPUT); //drv IGBT fault active low
  pinMode(PIN_DRV_READY,     INPUT); //drv IGBT Power-good output, active high 

  pinMode(PIN_READY_OUT, OUTPUT);
  pinMode(PIN_FAULT_OUT, OUTPUT);
  pinMode(PIN_DRV_RESET, OUTPUT);    //drv IGBT Reset input, apply a low pulse to reset fault latch

  digitalWrite(PIN_READY_OUT, LOW);
  digitalWrite(PIN_FAULT_OUT, LOW);
  digitalWrite(PIN_DRV_RESET, LOW);
}

void loop() {
  esp_task_wdt_reset();   // <<< KICK WATCHDOG

  uint32_t now = millis(); //macchina stati

  read_inputs(now);           // 1) GPIO + debounce
  update_power_ok(now);       // 2) TON power_ok
  system_fsm_tick(&sys, now); // 3) FSM
  update_outputs();           // 4) uscite
  

  // Simuliamo aggiornamento misura !!ESEMPIO DA TOGLIERE!!
  deviceData.measure++;

  // Logica esempio: se setpoint pari o dispari cambia il coil !!ESEMPIO DA TOGLIERE!!
  if (deviceData.setpoint  % 2 == 0) {
    deviceData.states |= 0x01;
  } else {
    deviceData.states &= ~0x01;
  }


  // Aggiorna i registri in memoria Modbus
  mb.Ireg(0, deviceData.measure);
  mb.Hreg(0, deviceData.setpoint);
  for (uint8_t i = 0; i < 8; i++) {
    mb.Coil(i, (deviceData.commands >> i) & 0x01);
    mb.Ists(i, (deviceData.states >> i) & 0x01);
  }
  mb.task(); //aggiorna costantemente il dato dal modbus (tiene 'sveglio' il modbus sulla seriale)
  delay(10);
}


// MACCHINA STATI ->
//  __ __  __   ___ ____  _ _ __  _  __     __ _____ __ _____ _  
// |  V  |/  \ / _// _/ || | |  \| |/  \  /' _/_   _/  \_   _| | 
// | \_/ | /\ | \_| \_| >< | | | ' | /\ | `._`. | || /\ || | | | 
// |_| |_|_||_|\__/\__/_||_|_|_|\__|_||_| |___/ |_||_||_||_| |_| 
//
// #define RESET_TIMEOUT_MS 1000
// #define VALVE_CLOSED_MAX_MS 10000

// //pseudo alias degli ingressi digitali:
// bool power_ok;        // alimentazioni ready
// bool power_on;        // comando ingresso on --- OK IO21  -> power_on        (accensione)
// bool reset_cmd;       // comando reset (PLC o pulsante) --- IO22  -> reset_cmd       (reset esterno)
// bool valve_cmd_mon;   // monitor limite tempo apertura valvola --- IO34  -> valve_cmd_mon   (monitor comando valvola – solo osservato)
// bool drv_fault;       // IO drv flt --- IO26  -> drv_fault
// bool drv_ready;       // IO27
// bool watchdog_err = false;  // per ora placeholder --- booleano a servizio della cpu... io aggiungerei il watchdog hardware dell'ESP32.


void system_fsm_tick(SystemCtx *sys, uint32_t now_ms) {
    switch (sys->state)
    {
    case ST_START:
        sys->fault_code = FAULT_NONE;
        //Serial.println("st_start");
        //Serial.println(sys->fault_code);

        if (power_ok)
        {
            sys->state   = ST_READY;
            sys->t_state = now_ms;
        }
        else
        {
            sys->fault_code = FAULT_POWER_FAIL; // POWER_FAIL
            sys->state      = ST_FAULT;
            sys->t_state    = now_ms;
        }
        break;

    case ST_READY:
        //Serial.println("ST_READY");
        //Serial.println(sys->fault_code);
        /* Monitor chiusura valvola */
        if (valve_cmd_mon) // se valvola chiusa
        {
            if (sys->t_valve_closed == 0)
            {
                // fronte di salita: inizio monitor
                sys->t_valve_closed = now_ms;
            }
            else if (now_ms - sys->t_valve_closed >= VALVE_CLOSED_MAX_MS)
            {
                sys->fault_code = FAULT_VALVE_TIMEOUT; // VALVE_CLOSED_TOO_LONG
                sys->state      = ST_FAULT;
                sys->t_state    = now_ms;
                sys->t_valve_closed = 0;
                break;
            }
        }
        else
        {
            // valvola non chiusa → reset timer
            sys->t_valve_closed = 0;
        }

        /* Fault hardware / power / watchdog */
        if (drv_fault) {
            sys->fault_code = FAULT_HW_DRV;
            sys->state      = ST_FAULT;
            sys->t_state    = now_ms;
            sys->t_valve_closed = 0;
        }
        else if (!power_ok) {
            sys->fault_code = FAULT_POWER_FAIL;
            sys->state      = ST_FAULT;
            sys->t_state    = now_ms;
            sys->t_valve_closed = 0;
        }
        else if (watchdog_err) {
            sys->fault_code = FAULT_WATCHDOG;
            sys->state      = ST_FAULT;
            sys->t_state    = now_ms;
            sys->t_valve_closed = 0;
        }
       break;

    case ST_FAULT:
        //Serial.println("ST_FAULT");
        //Serial.println(sys->fault_code);
        // fault latched
        if (reset_cmd)
        {
            sys->state   = ST_RESET;
            sys->t_state = now_ms;
        }
        power_ok = false; // in fault mai power_ok!!!
        break;

case ST_RESET:
    //Serial.println("ST_Reset");
    //Serial.println(sys->fault_code);
    // attesa condizioni stabili
    if (power_ok && !drv_fault && !watchdog_err)
    {
        if (now_ms - sys->t_state >= RESET_TIMEOUT_MS)
        {
            sys->fault_code = FAULT_NONE;
            sys->state      = ST_READY;
            sys->t_state    = now_ms;
        }
    }
    else
    {
        // condizioni non valide → ritorna in FAULT
        if (drv_fault)
            sys->fault_code = FAULT_HW_DRV;
        else if (!power_ok)
            sys->fault_code = FAULT_POWER_FAIL;
        else if (watchdog_err)
            sys->fault_code = FAULT_WATCHDOG;

        sys->state   = ST_FAULT;
        sys->t_state = now_ms;
    }
    break;


    default:
        // sicurezza: stato invalido
        sys->fault_code = 0xFF;
        sys->state      = ST_FAULT;
        sys->t_state    = now_ms;
        break;
    }
}
// <- END MACCHINA STATI

// DEBOUNCE ->
//  __  ___ __  __  _  _ __  _  ______  
// | _\| __|  \/__\| || |  \| |/ _/ __| 
// | v | _|| -< \/ | \/ | | ' | \_| _|  
// |__/|___|__/\__/ \__/|_|\__|\__/___|  
//
bool debounce_input(Debounce_t *db, bool raw, uint32_t now_ms) //variabile associata al pin, pin da leggere, tempo assoluto
{
    if (raw != db->last_raw) {
        db->last_raw = raw;
        db->t_last_change = now_ms;
    }

    if ((now_ms - db->t_last_change) >= DEBOUNCE_MS) {
        db->stable_state = db->last_raw;
    }

    return db->stable_state;
}
// <- END DEBOUNCE

// READ INPUT ->
//  ___ ___  __  __    _ __  _ ___ _  _ _____  
// | _ \ __|/  \| _\  | |  \| | _,\ || |_   _| 
// | v / _|| /\ | v | | | | ' | v_/ \/ | | |   
// |_|_\___|_||_|__/  |_|_|\__|_|  \__/  |_|   
//
void read_inputs(uint32_t now_ms)
{
    power_on = debounce_input(&db_power_on,
                              digitalRead(PIN_POWER_ON),
                              now_ms);

    reset_cmd = debounce_input(&db_reset_cmd,
                               digitalRead(PIN_RESET_CMD),
                               now_ms);

    valve_cmd_mon = debounce_input(&db_valve_cmd,
                                   !digitalRead(PIN_VALVE_CMD_MON),
                                   now_ms);

    drv_fault = debounce_input(&db_drv_fault,
                               !digitalRead(PIN_DRV_FAULT),
                               now_ms); // la logica FLT è negata

    drv_ready = debounce_input(&db_drv_ready,
                               digitalRead(PIN_DRV_READY),
                               now_ms);
}
// <- END READ INPUT

// OUTPUT ->
//   __  _  _ _____ ___ _  _ _____  
//  /__\| || |_   _| _,\ || |_   _| 
// | \/ | \/ | | | | v_/ \/ | | |   
//  \__/ \__/  |_| |_|  \__/  |_|   
//
void update_outputs(void)
{
    digitalWrite(PIN_READY_OUT, (sys.state == ST_READY));
    digitalWrite(PIN_FAULT_OUT, (sys.state == ST_FAULT));

    // reset driver solo nello stato RESET
    digitalWrite(PIN_DRV_RESET, !(sys.state == ST_RESET));
}
// <- END OUTPUT

// POWER OK ->
//  ___  __  _   _  ___ ___    __  _  __ 
// | _,\/__\| | | || __| _ \  /__\| |/ / 
// | v_/ \/ | 'V' || _|| v / | \/ |   <  
// |_|  \__/!_/ \_!|___|_|_\  \__/|_|\_\ 
//
void update_power_ok(uint32_t now_ms)
{
    if (power_on)
    {
        if (t_power_on == 0)
            t_power_on = now_ms;

        if (now_ms - t_power_on >= POWER_OK_DELAY_MS)
            power_ok = true;
    }
    else
    {
        power_ok = false;
        t_power_on = 0;
    }
}
// <- END POWER OK