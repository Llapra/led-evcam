/*
 * =====================================================================================
 *  DIAGNOSTICO COMPLETO  —  DAC de la ESP32 (D26) + circuito del LED      (version 3)
 * -------------------------------------------------------------------------------------
 *  Responde dos preguntas por separado:
 *   1) ¿El CODIGO escribe bien el nivel?  -> se leen los REGISTROS internos del DAC
 *      (el valor escrito, si el DAC esta encendido, si hay funciones extra activas).
 *   2) ¿El HARDWARE responde?             -> se miden base (D34) y emisor (D35) con el ADC,
 *      128 lecturas por nivel, con minimo y maximo para detectar contactos malos o ruido.
 *
 *  Solo usa funciones estandar de Arduino (dacWrite, analogReadMilliVolts) y lectura
 *  de registros. Se maneja desde el Monitor Serie: 115200 baudios, "Nueva linea".
 *
 *  CONEXIONES (montaje actual):
 *     D26  -> base del transistor (pata del centro del PN2222)   [salida del DAC]
 *     D34  -> D26 (o la base, es el mismo punto)                  [mide la base]
 *     D35  -> emisor (pata izquierda del PN2222, donde llegan las resistencias)
 *     GND  -> extremo de las resistencias que va al negativo de la fuente
 *     Fuente 5 V: + al LED, - al negativo comun. PUEDE ESTAR CONECTADA (es lo normal).
 *
 *  COMANDOS:
 *     0 ... 255   aplica ese nivel y muestra una linea completa de diagnostico
 *     t           barrido automatico 0,16,32...255 y veredicto final
 *     d           muestra los registros del DAC en bruto
 *     p25 / p26   cambia el pin del DAC (por defecto 26). El otro queda deshabilitado.
 *     ?           ayuda
 * =====================================================================================
 */

#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_io_reg.h"
#include "soc/sens_reg.h"

uint8_t dacPin = 26;                 // D26 = DAC2 (por defecto)
const uint8_t PIN_VB = 34;           // ADC: base
const uint8_t PIN_VE = 35;           // ADC: emisor
const float   R_E = 141.3;           // ohm
const int     MUESTRAS = 128;

int nivel = 0;
String linea;

// Tipos definidos ANTES de cualquier función (el Arduino IDE genera las declaraciones
// de las funciones antes de la primera función del archivo).
struct Lect { uint32_t avg, mn, mx; };
struct RegDac {
  uint32_t pad, c1, c2;
  int dac, xpd, force, cw, inv, dc, scale, tone, digforce, rue, rde;
};

struct Fila { int n; bool cod; uint32_t vb, ve; uint32_t sb, se; };


Lect leer(uint8_t pin) {
  uint32_t s = 0, mn = 100000, mx = 0;
  for (int i = 0; i < MUESTRAS; i++) {
    uint32_t v = analogReadMilliVolts(pin);
    s += v; if (v < mn) mn = v; if (v > mx) mx = v;
  }
  Lect r = { s / MUESTRAS, mn, mx };
  return r;
}

// ---------- registros del DAC del pin elegido ----------

RegDac leerRegistros() {
  RegDac r;
  r.pad = READ_PERI_REG(dacPin == 25 ? RTC_IO_PAD_DAC1_REG : RTC_IO_PAD_DAC2_REG);
  r.c1  = READ_PERI_REG(SENS_SAR_DAC_CTRL1_REG);
  r.c2  = READ_PERI_REG(SENS_SAR_DAC_CTRL2_REG);
  r.dac   = (r.pad >> 19) & 0xFF;   // valor que convierte el DAC
  r.xpd   = (r.pad >> 18) & 1;      // DAC encendido
  r.force = (r.pad >> 10) & 1;
  r.rue   = (r.pad >> 27) & 1;      // pull-up del pin
  r.rde   = (r.pad >> 28) & 1;      // pull-down del pin
  bool ch1 = (dacPin == 25);
  r.dc    = ch1 ? (r.c2 & 0xFF)        : ((r.c2 >> 8) & 0xFF);
  r.scale = ch1 ? ((r.c2 >> 16) & 3)   : ((r.c2 >> 18) & 3);
  r.inv   = ch1 ? ((r.c2 >> 20) & 3)   : ((r.c2 >> 22) & 3);
  r.cw    = ch1 ? ((r.c2 >> 24) & 1)   : ((r.c2 >> 25) & 1);   // generador coseno
  r.tone  = (r.c1 >> 16) & 1;
  r.digforce = (r.c1 >> 22) & 1;
  return r;
}

bool codigoOk(const RegDac& r, int n) {
  return r.dac == n && r.xpd == 1 && r.cw == 0 && r.tone == 0 && r.digforce == 0 &&
         r.rue == 0 && r.rde == 0;
}

void dumpRegistros() {
  RegDac r = leerRegistros();
  Serial.printf("REG pin=D%u PAD=0x%08lX CTRL1=0x%08lX CTRL2=0x%08lX\n", dacPin,
                (unsigned long)r.pad, (unsigned long)r.c1, (unsigned long)r.c2);
  Serial.printf("    dac=%d encendido=%d force=%d pullup=%d pulldown=%d | coseno=%d tono=%d "
                "dig_force=%d inv=%d dc=%d scale=%d\n",
                r.dac, r.xpd, r.force, r.rue, r.rde, r.cw, r.tone, r.digforce, r.inv, r.dc, r.scale);
}

// Apaga un DAC limpiando sus bits de encendido (xpd_dac bit 18, force bit 10)
void apagarDac(uint8_t pin) {
  CLEAR_PERI_REG_MASK(pin == 25 ? RTC_IO_PAD_DAC1_REG : RTC_IO_PAD_DAC2_REG, (1UL << 18) | (1UL << 10));
}

// ---------- aplicar y medir ----------
void aplicar(int n) {
  nivel = constrain(n, 0, 255);
  dacWrite(dacPin, nivel);
}


Fila medirLinea() {
  delay(150);                          // estabiliza
  RegDac r = leerRegistros();
  Lect b = leer(PIN_VB);
  Lect e = leer(PIN_VE);
  bool cod = codigoOk(r, nivel);
  float esperado = nivel * 3300.0f / 255.0f;
  float iled = e.avg / R_E;
  Serial.printf("N=%3d | reg=%3d %s | base %4lu mV (esp ~%4.0f) [%4lu-%4lu] | emisor %4lu mV [%4lu-%4lu]"
                " | I_LED~%5.2f mA | Vbe=%5ld",
                nivel, r.dac, cod ? "CODIGO-OK " : "CODIGO-MAL", (unsigned long)b.avg, esperado,
                (unsigned long)b.mn, (unsigned long)b.mx, (unsigned long)e.avg,
                (unsigned long)e.mn, (unsigned long)e.mx, iled, (long)b.avg - (long)e.avg);
  if (b.mx - b.mn > 250 || e.mx - e.mn > 250) Serial.print(" | INESTABLE (contacto o ruido)");
  if (!cod) {
    Serial.printf(" | registros: dac=%d encendido=%d coseno=%d tono=%d pull=%d/%d",
                  r.dac, r.xpd, r.cw, r.tone, r.rue, r.rde);
  }
  Serial.println();
  Fila f = { nivel, cod, b.avg, e.avg, b.mx - b.mn, e.mx - e.mn };
  return f;
}

void barrido() {
  Serial.printf("\n=== BARRIDO en D%u: 0,16,32...255 (ver el LED mientras corre) ===\n", dacPin);
  Fila f[18];
  int k = 0;
  for (int n = 0; n <= 256; n += 16) {
    aplicar(n > 255 ? 255 : n);
    f[k++] = medirLinea();
    delay(600);
  }
  aplicar(0);
  medirLinea();

  // ---- veredicto ----
  int codMal = 0, bajadas = 0, lejos = 0, inest = 0;
  for (int i = 0; i < k; i++) {
    if (!f[i].cod) codMal++;
    if (f[i].sb > 250 || f[i].se > 250) inest++;
    float esp = f[i].n * 3300.0f / 255.0f;
    float dif = (float)f[i].vb - esp;
    if (dif > 350 || dif < -350) lejos++;
    if (i > 0 && f[i].vb + 80 < f[i - 1].vb) bajadas++;
  }
  Serial.println("\n=== VEREDICTO ===");
  Serial.printf("1) CODIGO: %s (%d de %d niveles con el registro distinto o una funcion extra activa)\n",
                codMal == 0 ? "CORRECTO - el DAC recibe exactamente el nivel pedido" : "PROBLEMA",
                codMal, k);
  Serial.printf("2) SALIDA EN LA BASE: %s (%d bajadas al subir el nivel, %d niveles lejos de lo esperado)\n",
                (bajadas == 0 && lejos <= 2) ? "SUBE PAREJO" : "NO SIGUE AL NIVEL",
                bajadas, lejos);
  if (inest) Serial.printf("   %d niveles con lectura INESTABLE: revisar contactos de D34/D35\n", inest);
  Serial.println("Copie TODO este texto y envielo.\n");
}

void ayuda() {
  Serial.println();
  Serial.println("=== DIAGNOSTICO DAC + LED (v3) ===");
  Serial.printf("Salida del DAC: D%u   |  D34 = base   |  D35 = emisor\n", dacPin);
  Serial.println("Comandos: numero 0-255 = aplicar y medir | t = barrido + veredicto");
  Serial.println("          d = registros del DAC | p25 / p26 = cambiar pin | ? = ayuda");
  Serial.println("Con la fuente CONECTADA y todo bien deberia verse:");
  Serial.println("  N=0   -> base ~0 mV (el ADC marca ~140 como cero), emisor ~0, LED apagado");
  Serial.println("  N=128 -> base ~1650 mV, emisor ~1000 mV, I_LED ~7 mA");
  Serial.println("  N=255 -> base ~3100 mV, emisor ~2400 mV, I_LED ~17 mA");
  Serial.println();
}

void cambiarPin(uint8_t p) {
  if (p == dacPin) return;
  dacWrite(dacPin, 0);
  apagarDac(dacPin);
  dacPin = p;
  aplicar(nivel);
  Serial.printf("Pin del DAC cambiado a D%u\n", dacPin);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_VB, ADC_11db);
  analogSetPinAttenuation(PIN_VE, ADC_11db);
  aplicar(0);                            // D25 no se toca: queda como entrada (DAC apagado)
  Serial.printf("\nChip rev %d | SDK %s\n", ESP.getChipRevision(), ESP.getSdkVersion());
  ayuda();
  dumpRegistros();
  medirLinea();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      linea.trim();
      if (linea.length() > 0) {
        if (linea == "t" || linea == "T")       barrido();
        else if (linea == "d" || linea == "D")  dumpRegistros();
        else if (linea == "?")                  ayuda();
        else if (linea == "p25")                cambiarPin(25);
        else if (linea == "p26")                cambiarPin(26);
        else {
          bool num = true;
          for (unsigned i = 0; i < linea.length(); i++) if (!isDigit(linea[i])) num = false;
          if (num) { aplicar(linea.toInt()); medirLinea(); }
          else Serial.println("Comando no reconocido (? = ayuda)");
        }
      }
      linea = "";
    } else {
      linea += c;
    }
  }
}
