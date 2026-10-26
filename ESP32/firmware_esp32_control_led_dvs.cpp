/*
 * =====================================================================================
 * Proyecto: Generador de Estímulos Ópticos para Caracterización de Cámaras de Eventos (DVS)
 * Dispositivo: ESP32 DevKit (V1 / V3)
 * Salida Analógica: GPIO 25 (Canal DAC1 interno)
 * Baudrate: 115200 bps
 * 
 * Descripción:
 * Firmware determinista para control continuo y ráfagas de alta velocidad de un LED
 * mediante transistor en configuración seguidor de emisor. Diseñado para ser controlado
 * de forma remota desde un host (Python).
 * =====================================================================================
 */

#include <Arduino.h>

// Definición de pines y hardware
const uint8_t PIN_DAC = 25; // DAC1 interno del ESP32

// Buffer de comunicación serie
String rxBuffer = "";
bool burstAborted = false;

// Prototipos de funciones
void processCommand(const String& cmd);
void executeBurst(uint8_t baseVal, uint8_t targetVal, uint32_t tHighUs, uint32_t tLowUs, uint32_t cycles);
void delayMicrosPrecise(uint32_t us);

void setup() {
  // Inicialización del puerto serie a alta velocidad
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {
    // Espera hasta 2 segundos para conexión USB nativa
  }

  // Inicializar DAC en nivel 0 (apagado seguro)
  dacWrite(PIN_DAC, 0);

  // Mensaje de sincronización inicial para el script de Python
  Serial.println("READY");
}

void loop() {
  // Lectura no bloqueante del puerto serie
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();

    if (inChar == '\n' || inChar == '\r') {
      if (rxBuffer.length() > 0) {
        rxBuffer.trim();
        processCommand(rxBuffer);
        rxBuffer = "";
      }
    } else {
      // Acumular caracteres legibles en el buffer
      if (rxBuffer.length() < 128) {
        rxBuffer += inChar;
      }
    }
  }
}

/**
 * Procesa e interpreta las cadenas de texto recibidas por el puerto serie.
 */
void processCommand(const String& cmd) {
  // 1. Comando PING: Verificación de comunicación y sincronización
  if (cmd.equalsIgnoreCase("PING")) {
    Serial.println("PONG");
    return;
  }

  // 2. Comando SET: Ajuste de nivel DC continuo estático
  // Formato: SET <valor_0_a_255>
  if (cmd.startsWith("SET") || cmd.startsWith("set")) {
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx == -1) {
      Serial.println("ERR PARAM_MISSING");
      return;
    }

    int val = cmd.substring(spaceIdx + 1).toInt();
    if (val < 0 || val > 255) {
      Serial.println("ERR VAL_OUT_OF_RANGE");
      return;
    }

    dacWrite(PIN_DAC, (uint8_t)val);
    Serial.print("ACK SET ");
    Serial.println(val);
    return;
  }

  // 3. Comando BURST: Ejecución de ráfaga de pulsos u ondas cuadradas
  // Formato: BURST <val_base> <val_target> <t_alto_us> <t_bajo_us> <ciclos>
  if (cmd.startsWith("BURST") || cmd.startsWith("burst")) {
    char cmdStr[128];
    cmd.toCharArray(cmdStr, sizeof(cmdStr));

    char tag[10];
    int baseVal, targetVal;
    unsigned long tHighUs, tLowUs, cycles;

    int parsed = sscanf(cmdStr, "%s %d %d %lu %lu %lu", tag, &baseVal, &targetVal, &tHighUs, &tLowUs, &cycles);

    if (parsed != 6) {
      Serial.println("ERR BAD_BURST_PARAMS");
      return;
    }

    // Validación de rangos de intensidad
    if (baseVal < 0 || baseVal > 255 || targetVal < 0 || targetVal > 255) {
      Serial.println("ERR INTENSITY_RANGE");
      return;
    }

    // Validación de ciclos y temporizaciones mínimas
    if (cycles == 0) {
      Serial.println("ERR ZERO_CYCLES");
      return;
    }

    if (tHighUs < 2 || tLowUs < 2) {
      Serial.println("ERR MIN_TIME_VIOLATION"); // Mínimo 2 microsegundos garantizados
      return;
    }

    // Respuesta inmediata de aceptación
    Serial.println("ACK BURST");

    // Ejecución de la ráfaga
    executeBurst((uint8_t)baseVal, (uint8_t)targetVal, (uint32_t)tHighUs, (uint32_t)tLowUs, (uint32_t)cycles);
    return;
  }

  // 4. Comando STOP: Apagado y retorno a reposo
  if (cmd.equalsIgnoreCase("STOP")) {
    burstAborted = true;
    dacWrite(PIN_DAC, 0);
    Serial.println("ACK STOP");
    return;
  }

  // Comando desconocido
  Serial.println("ERR UNKNOWN_COMMAND");
}

/**
 * Ejecuta la ráfaga de pulsos con temporización en microsegundos.
 * Al finalizar o ante una interrupción, deja el LED en el valor base.
 */
void executeBurst(uint8_t baseVal, uint8_t targetVal, uint32_t tHighUs, uint32_t tLowUs, uint32_t cycles) {
  burstAborted = false;

  // Asegurar nivel inicial en reposo
  dacWrite(PIN_DAC, baseVal);

  for (uint32_t i = 0; i < cycles; i++) {
    // Escaneo rápido no bloqueante de aborto vía serie
    if (Serial.available() > 0) {
      String interruptCmd = Serial.readStringUntil('\n');
      interruptCmd.trim();
      if (interruptCmd.equalsIgnoreCase("STOP")) {
        burstAborted = true;
        break;
      }
    }

    // 1. Transición al nivel objetivo (Target)
    dacWrite(PIN_DAC, targetVal);
    delayMicrosPrecise(tHighUs);

    // 2. Retorno al nivel base (Base)
    dacWrite(PIN_DAC, baseVal);
    delayMicrosPrecise(tLowUs);
  }

  // Estado final garantizado: nivel base
  dacWrite(PIN_DAC, baseVal);

  if (burstAborted) {
    dacWrite(PIN_DAC, 0);
    Serial.println("ABORTED");
  } else {
    // Notificación a Python de fin de ráfaga exitosa
    Serial.println("DONE");
  }
}

/**
 * Función auxiliar de retardo determinista que maneja tanto
 * microsegundos cortos (>= 2 us) como periodos largos de segundos sin desbordes.
 */
void delayMicrosPrecise(uint32_t us) {
  if (us == 0) return;

  // Si el tiempo es mayor a 10 milisegundos, usar delay() fraccionado
  // para permitir que las tareas del sistema del ESP32 respiren
  if (us >= 10000) {
    uint32_t ms = us / 1000;
    uint32_t remUs = us % 1000;
    delay(ms);
    if (remUs > 0) {
      delayMicroseconds(remUs);
    }
  } else {
    // Retardo directo a nivel de microsegundos para tiempos ultracortos
    delayMicroseconds(us);
  }
}