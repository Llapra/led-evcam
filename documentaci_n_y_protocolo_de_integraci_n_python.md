# Guía de Integración Firmware ESP32 - Control de LED para Cámara de Eventos (DVS)

Este documento describe la arquitectura, conexionado eléctrico, protocolo serie y pautas de integración necesarias para controlar la placa ESP32 desde un programa en **Python**. El sistema está diseñado para generar estímulos ópticos continuos y calibrados para la caracterización de sensores de visión dinámica (DVS / neuromorphic event cameras) sin introducir modulaciones parásitas como PWM.

---

## 1. Arquitectura del Sistema

El sistema implementa una separación estricta de responsabilidades:

```
+--------------------------+               +--------------------------+               +--------------------------+
|       Host (PC)          |  USB / UART   |       ESP32 DevKit       |  DAC (0-3.3V) |    Circuito Analógico    |
|         Python           | ------------> |    Firmware Autónomo     | ------------> | Transistor 2N2222A + LED |
| (Barridos, UI, Metadatos)| <------------ | (Temporización us pura)  |               |  (Emisión Fotónica Pura) |
+--------------------------+    115200 bps +--------------------------+    GPIO 25    +--------------------------+
```

* **Python:** Actúa como coordinador de alto nivel. Define los niveles de contraste relativo, tiempos de pausa para segmentación en los logs de eventos de la cámara, conteo de ciclos y registro de marcas de tiempo (*timestamps*).
* **ESP32:** Actúa como sintetizador de hardware determinista. Ejecuta las conmutaciones a nivel de microsegundos en su CPU sin sufrir el *jitter* ni las latencias del sistema operativo del ordenador.
* **Circuito Lineal (Seguidor de Emisor):** El DAC del ESP32 modula la base de un transistor bipolar NPN operando en su zona activa. La corriente por el LED varía de forma continua y libre de conmutaciones digitales.

---

## 2. Resumen del Circuito Eléctrico

* **Salida de control:** Pin `GPIO 25` del ESP32 (DAC1 analógico interno).
* **Alimentación del LED:** Fuente DC externa o baterías (4.8 V a 6.0 V).
* **LED:** LED rojo monocromático (5 mm, $\lambda \approx 625\text{ nm}$). Se desaconsejan LEDs blancos para pruebas dinámicas por la persistencia del fósforo.
* **Transistor:** NPN de propósito general (2N2222A o PN2222 en encapsulado TO-92).
  * **Colector:** Al cátodo (-) del LED.
  * **Base:** Directo a `GPIO 25` del ESP32.
  * **Emisor:** A la resistencia $R_E$.
* **Resistencia de sensado y protección ($R_E$):** $120\ \Omega$ (óptima) a $150\ \Omega$ (1/4 W).
* **Línea de masa (GND):** **Masa común obligatoria**. El pin `GND` del ESP32, el polo negativo de la fuente de alimentación externa y el extremo inferior de $R_E$ deben estar unidos en el mismo nodo.

---

## 3. Especificación del Protocolo Serie (UART)

* **Baudrate:** `115200`
* **Bits de datos:** 8
* **Paridad:** Ninguna (None)
* **Bits de parada:** 1
* **Terminador de línea:** `\n` (salto de línea LF) o `\r\n`.
* **Codificación:** ASCII estándar.

### Mensaje de inicio
Al arrancar o reiniciar el ESP32, la placa envía automáticamente:
```text
READY
```
El script de Python debe esperar esta cadena para confirmar que el puerto está listo.

---

## 4. Diccionario de Comandos

Todos los comandos se envían en texto plano seguidos de `\n`.

### 1. `PING`
Verifica la comunicación activa con la placa.
* **Argumentos:** Ninguno.
* **Respuesta del ESP32:**
  ```text
  PONG
  ```

---

### 2. `SET <valor>`
Establece un nivel de luminosidad continua permanente. El DAC mantiene este valor indefinidamente sin entrar en bucles de temporización. Útil para iluminación de fondo (DC offset), enfoque de cámara y calibración de fotómetro.
* **Argumentos:**
  * `<valor>`: Entero de 8 bits entre `0` y `255`.
    * `0`: LED completamente apagado.
    * `1` a `~50`: Transistor en corte ($V_{DAC} < 0.7\text{ V}$).
    * `55` a `60`: Encendido mínimo continuo ($\approx 0.1\text{ mA}$, $\approx 0.5\%$ de la potencia del LED).
    * `255`: Máxima intensidad segura ($\approx 21.7\text{ mA}$ con $R_E = 120\ \Omega$).
* **Respuesta del ESP32:**
  ```text
  ACK SET <valor>
  ```
* **Ejemplo:**
  ```text
  SET 120\n  --> Respuesta: ACK SET 120
  ```

---

### 3. `BURST <val_base> <val_target> <t_alto_us> <t_bajo_us> <ciclos>`
Inicia una ráfaga de oscilación determinista entre un nivel de reposo y un nivel objetivo.
* **Argumentos:**
  * `<val_base>`: Entero (`0` a `255`). Nivel continuo de reposo.
  * `<val_target>`: Entero (`0` a `255`). Nivel pico alcanzado durante la oscilación.
  * `<t_alto_us>`: Entero positivo (microsegundos). Tiempo que permanece en `<val_target>` ($\ge 2\ \mu\text{s}$).
  * `<t_bajo_us>`: Entero positivo (microsegundos). Tiempo que permanece en `<val_base>` dentro de cada ciclo ($\ge 2\ \mu\text{s}$).
  * `<ciclos>`: Entero positivo ($\ge 1$). Cantidad de repeticiones completas.
* **Flujo de Respuestas:**
  1. Al recibir y validar los parámetros, el ESP32 responde de inmediato:
     ```text
     ACK BURST
     ```
  2. Al concluir el último ciclo, el DAC regresa automáticamente a `<val_base>` y envía:
     ```text
     DONE
     ```
  3. Si la ráfaga es interrumpida con un comando `STOP`, responde:
     ```text
     ABORTED
     ```
* **Ejemplo (Onda cuadrada simétrica a 5 Hz entre nivel 60 y nivel 85 durante 20 ciclos):**
  * Semiperiodo: $t_{semi} = \frac{10^6}{2 \times 5} = 100\,000\ \mu\text{s}$.
  ```text
  BURST 60 85 100000 100000 20\n
  ```

---

### 4. `STOP`
Interrumpe de forma inmediata cualquier ráfaga en ejecución, apaga el LED (`SET 0`) y regresa el sistema a reposo seguro.
* **Argumentos:** Ninguno.
* **Respuesta del ESP32:**
  ```text
  ACK STOP
  ```

---

### Mensajes de Error (`ERR`)
Si se envía una sintaxis errónea o valores fuera de rango, el firmware responderá:
* `ERR PARAM_MISSING`
* `ERR VAL_OUT_OF_RANGE`
* `ERR BAD_BURST_PARAMS`
* `ERR INTENSITY_RANGE`
* `ERR ZERO_CYCLES`
* `ERR MIN_TIME_VIOLATION`
* `ERR UNKNOWN_COMMAND`

---

## 5. Dinámica Temporal y Buenas Prácticas para Cámaras de Eventos

### Frentes de onda ($t_{subida} \le 5\ \mu\text{s}$)
El seguidor de emisor con el DAC interno del ESP32 completa su excursión de voltaje en menos de **$2.5\ \mu\text{s}$**, garantizando que el flanco óptico suba o baje en menos de **$5\ \mu\text{s}$**. Frente al tiempo de integración del fotorreceptor de una DVS (que suele ser de decenas a cientos de microsegundos), este estímulo equivale a un **escalón ideal**.

### Pausas entre ráfagas como separadores en el flujo de eventos
Para aislar cada nivel de contraste dentro de la grabación de la cámara:
1. Python envía un `BURST`.
2. El ESP32 oscila y genera eventos DVS activos.
3. El ESP32 termina y notifica `DONE`, dejando el LED en `val_base`.
4. Python recibe `DONE` y ejecuta un retardo (`time.sleep(pausa)`).
5. Durante esa pausa, la iluminación es completamente plana; la cámara no genera eventos de señal (solo ruido de fondo). Esto crea una brecha limpia y fácilmente identificable en el archivo de eventos (`.aedat` o HDF5).

---

## 6. Esquema de Integración de Referencia en Python

El siguiente pseudocódigo / patrón en Python muestra cómo estructurar la comunicación orientada a eventos para que el generador de estímulos pueda ser integrado en scripts o interfaces gráficas:

```python
import time
import serial

class DVSLightController:
    def __init__(self, port, baudrate=115200, timeout=2.0):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(1.5) # Espera a que el ESP32 reinicie tras abrir puerto
        self._flush_and_sync()

    def _flush_and_sync(self):
        self.ser.reset_input_buffer()
        # Verificar estado
        self.ser.write(b"PING\n")
        res = self.ser.readline().decode().strip()
        if "PONG" not in res and "READY" not in res:
            # Reintento de sincronización
            pass

    def send_command(self, cmd_str):
        self.ser.write(f"{cmd_str}\n".encode())
        return self.ser.readline().decode().strip()

    def set_dc_level(self, val):
        """Fija un nivel DC continuo estático (0 a 255)."""
        return self.send_command(f"SET {int(val)}")

    def run_burst(self, val_base, val_target, t_high_us, t_low_us, cycles):
        """
        Ejecuta una ráfaga bloqueando hasta que el ESP32 finaliza y envía DONE.
        """
        cmd = f"BURST {int(val_base)} {int(val_target)} {int(t_high_us)} {int(t_low_us)} {int(cycles)}"
        ack = self.send_command(cmd)
        if ack != "ACK BURST":
            raise RuntimeError(f"Error iniciando ráfaga: {ack}")

        # Esperar la confirmación de finalización del hardware
        status = self.ser.readline().decode().strip()
        return status == "DONE"

    def run_sweep(self, base_val, target_levels, freq_hz, cycles_per_level, pause_sec):
        """
        Ejecuta un barrido incremental con pausas de silencio entre niveles.
        """
        t_semi_us = int((1.0 / (2.0 * freq_hz)) * 1_000_000)

        for target in target_levels:
            print(f"Probando salto: Base={base_val} -> Target={target}...")
            # 1. Ejecutar oscilación
            success = self.run_burst(base_val, target, t_semi_us, t_semi_us, cycles_per_level)
            if not success:
                print("Ráfaga interrumpida.")
                break

            # 2. Pausa en el nivel base (silencio de eventos para segmentación)
            print(f"Pausa de segmentación: {pause_sec} segundos...")
            time.sleep(pause_sec)

        # Dejar apagado al terminar
        self.set_dc_level(0)

    def stop(self):
        """Interrupción de emergencia."""
        self.ser.write(b"STOP\n")
        return self.ser.readline().decode().strip()

    def close(self):
        self.set_dc_level(0)
        self.ser.close()
```