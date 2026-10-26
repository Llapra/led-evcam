/*
 * =====================================================================================
 *  Generador de estímulos ópticos para caracterización de cámaras de eventos (DVS)
 *  FIRMWARE v2  —  reemplaza a firmware_esp32_control_led_dvs.cpp (v1, se conserva)
 * -------------------------------------------------------------------------------------
 *  Placa   : ESP32 DevKit V3 (ESP32-WROOM-32)
 *  Salida  : GPIO 26 = DAC2 interno de 8 bits (0..255), sin PWM   (configurable: DAC_GPIO)
 *  Circuito: DAC -> base 2N2222A (seguidor de emisor), R_E = 150 ohm, LED rojo en colector
 *  Serie   : 115200 bps, 8N1, líneas terminadas en '\n'
 *
 *  Cambios respecto a v1
 *   1. STOP deja SIEMPRE el LED en el nivel base (v1 lo dejaba en 0) y responde
 *      en < 1 ms aunque el semiperiodo sea de segundos (v1 revisaba 1 vez por ciclo).
 *   2. Flancos programados en instantes ABSOLUTOS (t_siguiente = t_anterior + semiperiodo)
 *      con el reloj de 1 µs de la ESP32: el error no se acumula ciclo a ciclo.
 *   3. Escritura directa al registro del DAC (< 1 µs) en vez de dacWrite() en cada flanco.
 *   4. Cola de "segmentos": la secuencia completa (pausa inicial, niveles, pausas entre
 *      niveles) se ejecuta dentro de la ESP32, sin depender de la latencia del PC.
 *   5. Modo continuo (ciclos = 0 -> infinito, hasta STOP).
 *   6. Eventos con marca de tiempo en µs (relativa al inicio) para el registro CSV.
 *
 *  Protocolo (ver README.md para el detalle completo)
 *   PING                                   -> PONG
 *   INFO                                   -> INFO LED_DVS_v2 fw=2.2 dac=26 maxseg=512 base=<b> out=<o>
 *   SET <v>                                -> ACK SET <v>          (nivel fijo manual)
 *   BASE <v>                               -> ACK BASE <v>         (define base y la aplica)
 *   CLEAR                                  -> ACK CLEAR            (vacía la cola)
 *   Q <base> <alto> <t_alto_us> <t_bajo_us> <ciclos> <pausa_us>
 *                                          -> (silencio si OK) / ERR Q <motivo> <idx>
 *   RUN <pausa_inicial_us>                 -> ACK RUN <n_segmentos> y luego eventos:
 *        EV <t_us> START <n>
 *        EV <t_us> SEG <i> <base> <alto> <t_alto_us> <t_bajo_us> <ciclos>
 *        EV <t_us> SEGEND <i> <ciclos_hechos>
 *        EV <t_us> DONE            |   EV <t_us> ABORTED <i> <ciclos_hechos>
 *   STOP                                   -> ACK STOP  (LED queda en el nivel base)
 *   MEAS                                   -> MEAS out=<n> d34=<mV> d35=<mV>
 *        (diagnóstico: lee GPIO34 y GPIO35 con el ADC; ver README, sección problemas)
 *
 *  v2.1: SET, BASE y STOP usan dacWrite() (función oficial de Arduino). La escritura
 *        directa al registro se usa solo para los flancos de las ondas (RUN).
 *   Durante una ejecución solo se aceptan STOP y PING (el resto -> ERR BUSY).
 * =====================================================================================
 */

#include <Arduino.h>
#include <stdarg.h>
#include "esp_timer.h"
#include "soc/soc.h"
#include "soc/rtc_io_reg.h"

// --------------------------------------------------------------------------------------
// Configuración
// --------------------------------------------------------------------------------------
#define FW_ID            "LED_DVS_v2"
#define FW_VER           "2.2"

// ---- Pin del DAC: 26 (DAC2, por defecto) o 25 (DAC1). Cambiar solo esta línea. ----
// (En el montaje del laboratorio el GPIO 25 resultó dañado: se usa el 26.)
#define DAC_GPIO         26
#if DAC_GPIO == 25
  #define DAC_PAD_REG    RTC_IO_PAD_DAC1_REG
#elif DAC_GPIO == 26
  #define DAC_PAD_REG    RTC_IO_PAD_DAC2_REG
#else
  #error "DAC_GPIO debe ser 25 o 26"
#endif
#define USE_FAST_DAC     1        // 1 = registro directo (rápido). 0 = dacWrite() (lento, respaldo)

const uint8_t  PIN_DAC     = DAC_GPIO;
const uint8_t  PIN_ADC_A   = 34;  // diagnóstico: ADC1 (solo entrada)
const uint8_t  PIN_ADC_B   = 35;  // diagnóstico: ADC1 (solo entrada)
const uint32_t BAUDRATE    = 115200;
const uint16_t MAX_SEG     = 512; // máximo de segmentos en cola
const uint32_t MIN_T_US    = 5;   // semiperiodo mínimo aceptado (µs)

// Umbrales del bucle de espera (µs)
const int64_t  SLEEP_MARGIN_US = 3000; // si falta más que esto, se cede la CPU 1 ms
const int64_t  POLL_MARGIN_US  = 25;   // si falta más que esto, se revisa el puerto serie
const int64_t  PRINT_MARGIN_US = 80;   // si falta más que esto, se envían eventos pendientes
const uint32_t FORCED_POLL_EVERY = 2048; // ciclos: revisión forzada si los semiperiodos son < POLL_MARGIN

// --------------------------------------------------------------------------------------
// Estado
// --------------------------------------------------------------------------------------
struct Segment {
  uint8_t  base;
  uint8_t  high;
  uint32_t tHigh;   // µs en nivel alto
  uint32_t tLow;    // µs en nivel base dentro del ciclo
  uint32_t cycles;  // 0 = infinito
  uint32_t pause;   // µs en base después del segmento
};

Segment  segQueue[MAX_SEG];
uint16_t nSeg = 0;

uint8_t  baseLevel  = 0;   // nivel al que vuelve STOP
uint8_t  currentOut = 0;   // último valor escrito en el DAC

char     rxBuf[160];
uint16_t rxLen = 0;

bool     runAbort = false;

// Buffer de eventos pendientes (se envía solo en tiempos muertos)
char     pend[1024];
uint16_t pendLen = 0;
uint16_t pendSent = 0;

// --------------------------------------------------------------------------------------
// DAC
// --------------------------------------------------------------------------------------
static inline void dacOut(uint8_t v) {
#if USE_FAST_DAC
  // Mismo campo que escribe dacWrite(), pero sin la sobrecarga del driver.
  // campo "dac" del registro del pad: bits 26..19 (igual para DAC1 y DAC2)
  SET_PERI_REG_BITS(DAC_PAD_REG, 0xFF, v, 19);
#else
  dacWrite(PIN_DAC, v);
#endif
  currentOut = v;
}

// Escritura con la función oficial de Arduino (para niveles fijos: no es crítica en tiempo)
static void dacOutStd(uint8_t v) {
  dacWrite(PIN_DAC, v);
  currentOut = v;
}

static uint32_t readMilliVoltsAvg(uint8_t pin) {
  uint32_t sum = 0;
  for (int i = 0; i < 64; i++) sum += analogReadMilliVolts(pin);
  return sum / 64;
}

// --------------------------------------------------------------------------------------
// Eventos pendientes
// --------------------------------------------------------------------------------------
static void queueEvent(const char* fmt, ...) {
  char line[128];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n > (int)sizeof(line) - 1) n = sizeof(line) - 1;
  // compactar si hace falta
  if (pendSent > 0 && pendLen + n > sizeof(pend)) {
    memmove(pend, pend + pendSent, pendLen - pendSent);
    pendLen -= pendSent;
    pendSent = 0;
  }
  if (pendLen + n <= sizeof(pend)) {
    memcpy(pend + pendLen, line, n);
    pendLen += n;
  }
}

// Envía lo que quepa en el buffer TX sin bloquear.
static void flushPendingNonBlocking() {
  if (pendSent >= pendLen) { pendSent = pendLen = 0; return; }
  int room = Serial.availableForWrite();
  if (room <= 0) return;
  int n = pendLen - pendSent;
  if (n > room) n = room;
  Serial.write((const uint8_t*)pend + pendSent, n);
  pendSent += n;
  if (pendSent >= pendLen) pendSent = pendLen = 0;
}

static void flushPendingBlocking() {
  if (pendLen > pendSent) Serial.write((const uint8_t*)pend + pendSent, pendLen - pendSent);
  pendSent = pendLen = 0;
}

// --------------------------------------------------------------------------------------
// Lectura serie durante una ejecución: solo STOP y PING
// --------------------------------------------------------------------------------------
static bool pollDuringRun() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) {
        rxBuf[rxLen] = 0;
        String cmd(rxBuf);
        cmd.trim();
        rxLen = 0;
        if (cmd.equalsIgnoreCase("STOP")) {
          runAbort = true;
          return true;
        } else if (cmd.equalsIgnoreCase("PING")) {
          queueEvent("PONG\n");
        } else {
          queueEvent("ERR BUSY\n");
        }
      }
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    }
  }
  return false;
}

// Espera hasta el instante absoluto 'target' (µs de esp_timer). false = abortado.
static bool waitUntil(int64_t target) {
  for (;;) {
    int64_t rem = target - esp_timer_get_time();
    if (rem <= 0) return true;
    if (rem > PRINT_MARGIN_US) flushPendingNonBlocking();
    if (rem > POLL_MARGIN_US) {
      if (pollDuringRun()) return false;
    }
    if (rem > SLEEP_MARGIN_US) {
      vTaskDelay(1);  // cede la CPU ~1 ms (tick de 1 kHz); el margen evita pasarse
    }
    // si falta poco: espera activa
  }
}

// --------------------------------------------------------------------------------------
// Ejecución de la cola
// --------------------------------------------------------------------------------------
static void runQueue(uint32_t preUs) {
  runAbort = false;
  pendLen = pendSent = 0;
  rxLen = 0;   // el buffer aún contiene "RUN ..." (lo procesó loop()); se vacía para leer STOP

  Serial.print("ACK RUN ");
  Serial.println(nSeg);
  Serial.flush();

  baseLevel = segQueue[0].base;
  dacOutStd(baseLevel);               // asegura el canal DAC habilitado antes de los flancos rápidos
  const int64_t t0 = esp_timer_get_time();
  queueEvent("EV 0 START %u\n", (unsigned)nSeg);

  int64_t  t = t0 + (int64_t)preUs;  // instante programado del próximo flanco
  uint16_t i = 0;
  uint32_t c = 0;
  bool ok = true;

  for (i = 0; i < nSeg && ok; i++) {
    const Segment s = segQueue[i];
    baseLevel = s.base;
    c = 0;

    queueEvent("EV %lld SEG %u %u %u %lu %lu %lu\n", (long long)(t - t0), (unsigned)i,
               (unsigned)s.base, (unsigned)s.high, (unsigned long)s.tHigh,
               (unsigned long)s.tLow, (unsigned long)s.cycles);

    if (!waitUntil(t)) { ok = false; break; }
    if (currentOut != s.base) dacOut(s.base);  // por si la base cambió entre segmentos

    const bool tight = (s.tHigh < (uint32_t)POLL_MARGIN_US) || (s.tLow < (uint32_t)POLL_MARGIN_US);

    while (s.cycles == 0 || c < s.cycles) {
      if (!waitUntil(t)) { ok = false; break; }
      dacOut(s.high);
      t += s.tHigh;
      if (!waitUntil(t)) { ok = false; break; }
      dacOut(s.base);
      t += s.tLow;
      c++;
      // Con semiperiodos muy cortos waitUntil no alcanza a revisar el puerto:
      // revisión forzada cada FORCED_POLL_EVERY ciclos (introduce unos µs de jitter).
      if (tight && (c % FORCED_POLL_EVERY) == 0) {
        if (pollDuringRun()) { ok = false; break; }
        flushPendingNonBlocking();
      }
    }
    if (!ok) break;

    if (!waitUntil(t)) { ok = false; break; }  // termina el último tramo en base
    queueEvent("EV %lld SEGEND %u %lu\n", (long long)(t - t0), (unsigned)i, (unsigned long)c);
    t += s.pause;
    if (!waitUntil(t)) { ok = false; break; }   // pausa entre niveles (LED en base)
  }

  dacOutStd(baseLevel);  // estado final garantizado: base

  if (!ok || runAbort) {
    uint16_t idx = (i < nSeg) ? i : (nSeg - 1);
    queueEvent("EV %lld ABORTED %u %lu\n", (long long)(esp_timer_get_time() - t0),
               (unsigned)idx, (unsigned long)c);
    flushPendingBlocking();
    Serial.println("ACK STOP");
  } else {
    queueEvent("EV %lld DONE\n", (long long)(t - t0));
    flushPendingBlocking();
  }
  rxLen = 0;
}

// --------------------------------------------------------------------------------------
// Comandos en reposo
// --------------------------------------------------------------------------------------
static bool parseU32(const char* s, uint32_t& out) {
  if (!s || !*s) return false;
  char* end;
  unsigned long v = strtoul(s, &end, 10);
  if (*end != 0) return false;
  out = (uint32_t)v;
  return true;
}

static void processCommand(char* line) {
  // separar en tokens
  char* tok[10];
  int n = 0;
  char* p = strtok(line, " \t");
  while (p && n < 10) { tok[n++] = p; p = strtok(nullptr, " \t"); }
  if (n == 0) return;
  String cmd(tok[0]);
  cmd.toUpperCase();

  if (cmd == "PING") { Serial.println("PONG"); return; }

  if (cmd == "INFO") {
    Serial.printf("INFO %s fw=%s dac=%u maxseg=%u base=%u out=%u nseg=%u\n", FW_ID, FW_VER,
                  (unsigned)PIN_DAC, (unsigned)MAX_SEG, (unsigned)baseLevel,
                  (unsigned)currentOut, (unsigned)nSeg);
    return;
  }

  if (cmd == "SET" || cmd == "BASE") {
    uint32_t v;
    if (n < 2) { Serial.println("ERR PARAM_MISSING"); return; }
    if (!parseU32(tok[1], v) || v > 255) { Serial.println("ERR VAL_OUT_OF_RANGE"); return; }
    if (cmd == "BASE") baseLevel = (uint8_t)v;
    dacOutStd((uint8_t)v);
    Serial.printf("ACK %s %u\n", cmd.c_str(), (unsigned)v);
    return;
  }

  if (cmd == "CLEAR") { nSeg = 0; Serial.println("ACK CLEAR"); return; }

  if (cmd == "Q") {
    uint32_t v[6];
    if (n != 7) { Serial.printf("ERR Q BAD_PARAMS %u\n", (unsigned)nSeg); return; }
    for (int k = 0; k < 6; k++) {
      if (!parseU32(tok[k + 1], v[k])) { Serial.printf("ERR Q BAD_PARAMS %u\n", (unsigned)nSeg); return; }
    }
    if (v[0] > 255 || v[1] > 255)      { Serial.printf("ERR Q INTENSITY_RANGE %u\n", (unsigned)nSeg); return; }
    if (v[2] < MIN_T_US || v[3] < MIN_T_US) { Serial.printf("ERR Q MIN_TIME %u\n", (unsigned)nSeg); return; }
    if (nSeg >= MAX_SEG)               { Serial.printf("ERR Q QUEUE_FULL %u\n", (unsigned)nSeg); return; }
    segQueue[nSeg++] = { (uint8_t)v[0], (uint8_t)v[1], v[2], v[3], v[4], v[5] };
    return;  // silencio: la confirmación es "ACK RUN <n>"
  }

  if (cmd == "RUN") {
    uint32_t pre = 0;
    if (n >= 2 && !parseU32(tok[1], pre)) { Serial.println("ERR BAD_PARAMS"); return; }
    if (nSeg == 0) { Serial.println("ERR QUEUE_EMPTY"); return; }
    runQueue(pre);
    return;
  }

  if (cmd == "STOP") {
    dacOutStd(baseLevel);
    Serial.println("ACK STOP");
    return;
  }

  if (cmd == "MEAS") {
    delay(20);
    Serial.printf("MEAS out=%u d34=%lu d35=%lu\n", (unsigned)currentOut,
                  (unsigned long)readMilliVoltsAvg(PIN_ADC_A),
                  (unsigned long)readMilliVoltsAvg(PIN_ADC_B));
    return;
  }

  Serial.println("ERR UNKNOWN_COMMAND");
}

// --------------------------------------------------------------------------------------
void setup() {
  Serial.setRxBufferSize(8192);   // permite recibir la cola completa sin pérdidas
  Serial.setTxBufferSize(1024);   // los eventos no bloquean la temporización
  Serial.begin(BAUDRATE);
  delay(50);

  dacOutStd(0);                   // habilita el canal DAC1 y deja el LED apagado
  baseLevel = 0;
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ADC_A, ADC_11db);   // rango ~0 - 3.1 V
  analogSetPinAttenuation(PIN_ADC_B, ADC_11db);

  Serial.println();
  Serial.println("READY");
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) {
        rxBuf[rxLen] = 0;
        processCommand(rxBuf);
        rxLen = 0;
      }
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    }
  }
}
