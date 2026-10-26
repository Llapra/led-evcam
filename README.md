# Generador de intensidades LED para caracterizar los BIAS de la EvCam

Sistema para controlar la intensidad de un LED rojo con una ESP32, pensado para relacionar
los **BIAS de la cámara de eventos** con la **potencia de luz (mW)**. Permite:

- fijar un nivel continuo (para ver si la cámara satura y medir mW con el fotodetector),
- hacer **ondas cuadradas** entre un **nivel base** y un nivel alto, con frecuencia y ciclos a elección,
- hacer **barridos de niveles** (cada nivel alterna con la base, separados por pausas en la base),
- hacer **barridos de frecuencia** con un nivel alto fijo,
- dejar una oscilación **continua** hasta presionar STOP,
- ver siempre el **tiempo estimado** y, si se quiere, **grabar un registro CSV** con tiempos en µs.

La interfaz **solo controla el LED**. La cámara de eventos se maneja con su propio programa.

---

## 1. Cómo funciona

```
   PC (Python, PyQt6)              ESP32 DevKit V3                 Circuito
 ┌──────────────────────┐  USB  ┌──────────────────────┐ GPIO26 ┌──────────────────────┐
 │ Interfaz gráfica     │──────►│ Firmware v2          │──────► │ 2N2222A (seguidor de │
 │ - arma la secuencia  │ serie │ - ejecuta la         │  DAC   │ emisor) + LED rojo   │
 │ - estima el tiempo   │◄──────│   secuencia completa │ 0–255  │ R_E = 150 Ω          │
 │ - registro CSV       │eventos│ - temporiza en µs    │        │                      │
 └──────────────────────┘       └──────────────────────┘        └──────────────────────┘
```

1. En la interfaz se configura lo que se quiere (niveles, frecuencia, ciclos, pausas).
2. Al presionar **Iniciar**, el programa envía a la ESP32 la secuencia completa como una lista de
   *tramos*. Un tramo es "alternar base ↔ alto N ciclos y luego quedar en la base X segundos".
3. La **ESP32 ejecuta toda la secuencia por su cuenta**, con su reloj de 1 µs. El PC no interviene
   en la temporización, así que Windows/Linux, el USB o una interfaz ocupada no afectan las ondas.
4. La ESP32 avisa el inicio y fin de cada tramo con marcas de tiempo en µs. La interfaz las usa
   para mostrar el progreso y para el registro CSV.

### El circuito

| ESP32 | Conexión |
|---|---|
| **GPIO 26 (DAC2, pin D26)** | Base del transistor NPN (PN2222 / 2N2222A / BC548) |
| GND | Masa común (negativo de la batería y extremo inferior de R_E) |

El LED va del +5 V (fuente) al colector. El emisor va a R_E (en el montaje actual **141.3 Ω**,
varias resistencias en paralelo) y de ahí a masa.
> **Pin del DAC: D26.** En el montaje del laboratorio la salida GPIO 25 (D25) quedó dañada (con nivel 0 entregaba ~3.1 V), así que desde el firmware 2.2 se usa **GPIO 26 (D26)**. Para volver a D25 basta cambiar `#define DAC_GPIO 26` a `25` en el firmware.

**La masa común es obligatoria**: sin ella la base queda flotante y aparece ruido de 50 Hz.

**Orden de las patas (cara plana hacia uno, patas hacia abajo, de izquierda a derecha):**

| Transistor | Izquierda | Centro | Derecha |
|---|---|---|---|
| PN2222 / PN2222A / 2N2222A | Emisor → R_E | Base → D26 | Colector → LED (−) |
| BC548 / BC547 | Colector → LED (−) | Base → D26 | Emisor → R_E |
| P2N2222A (con la "P" adelante) | Colector | Base | Emisor |

Siempre confirmar con la prueba de diodo del multímetro: la base es la pata común a las dos
lecturas de ~0.6–0.7 V; entre colector y emisor no debe haber lectura en ningún sentido.

**Color del LED y voltaje de la fuente:** el circuito necesita
`V_fuente > V_LED + V_E(máx) + 0.3 V ≈ V_LED + 2.8 V`. Con 5 V sirven LEDs **rojos, naranjos,
amarillos y verdes "antiguos"** (~2 V). Los **azules, blancos y verdes puros** (~3–3.2 V)
saturan el transistor desde el nivel ~180: el brillo deja de subir y luego **baja**. Para esos
colores, usar una fuente de 6–9 V o no pasar del nivel ~180.

**Prueba rápida del montaje:** con la fuente conectada y la ESP32 desconectada, uniendo base y
emisor, el LED debe quedar **completamente apagado** (mirándolo a oscuras). Si se ve encendido,
hay una fuga o un corto (transistor dañado o mal conectado).

La corriente del LED es aproximadamente `I ≈ (V_DAC − 0.7 V) / R_E`, con
`V_DAC = nivel × 3.3 V / 255` (≈ 12.9 mV por nivel):

| Nivel DAC | V_DAC | Corriente aprox. |
|---|---|---|
| 0 | 0 V | 0 (LED apagado) |
| 1 – ~54 | < 0.7 V | casi 0: microamperes; en oscuridad puede verse un brillo muy tenue |
| 60 | 0.78 V | ≈ 0.6 mA |
| 100 | 1.29 V | ≈ 4.2 mA |
| 150 | 1.94 V | ≈ 8.8 mA |
| 255 | 3.30 V | ≈ 18 mA (máximo) |

(Valores con R_E = 141.3 Ω.)

Estos valores son **teóricos**. El valor real en mW se obtiene midiendo con el fotodetector
en la pestaña *Nivel fijo (SET)*. La interfaz muestra estas aproximaciones al lado de cada nivel.

### Resolución

El DAC interno de la ESP32 es de **8 bits: 256 niveles** (0–255). Como bajo ~54 el LED está
apagado, quedan unos **200 niveles útiles**. Cada nivel sube la corriente en ≈ 86 µA, siempre igual.

---

## 2. Archivos

```
Scripts/
├── README.md                          ← este documento
├── documentaci_n_y_protocolo_...md    ← documentación del firmware v1 (histórica)
├── ESP32/
│   ├── firmware_esp32_control_led_dvs.cpp                 ← v1 (original, se conserva)
│   └── firmware_esp32_control_led_dvs_v2/
│   │   └── firmware_esp32_control_led_dvs_v2.ino          ← v2.1 (el que se usa ahora)
│   └── prueba_dac_esp32/
│       └── prueba_dac_esp32.ino       ← prueba de diagnóstico (sin Python, ver sección 8)
└── Python/
    ├── led_dvs_gui.py      ← programa principal (interfaz PyQt6)
    ├── led_core.py         ← lógica: series, tiempos, protocolo, registro, ESP32 simulada
    ├── requirements.txt    ← dependencias (PyQt6, pyserial)
    └── registros/          ← se crea al grabar el primer registro CSV
```

El firmware v2 está en una carpeta con su mismo nombre porque el Arduino IDE lo exige.

---

## 3. Instalación

### 3.1 Cargar el firmware v2 en la ESP32

1. Instale el **Arduino IDE 2** y el soporte para ESP32: *Herramientas → Placa → Gestor de
   placas*, busque **esp32 (Espressif Systems)** e instálelo. Sirven las versiones 2.x y 3.x.
2. Abra `ESP32/firmware_esp32_control_led_dvs_v2/firmware_esp32_control_led_dvs_v2.ino`.
3. Placa: **ESP32 Dev Module**. Puerto: el de la DevKit (en Linux, `/dev/ttyUSB0`).
4. Presione **Subir**. Si se queda en "Connecting…", mantenga presionado el botón **BOOT** de
   la placa hasta que empiece a escribir.
5. (Opcional) Abra el Monitor Serie a 115200 baudios con fin de línea "Nueva línea". Al reiniciar
   debe aparecer `READY`, y al escribir `INFO` debe responder `INFO LED_DVS_v2 ...`.
   **Cierre el Monitor Serie antes de usar la interfaz**: el puerto no se puede abrir dos veces.

> Si la compilación falla en la línea `SET_PERI_REG_BITS` (escritura directa al DAC), cambie
> `#define USE_FAST_DAC 1` por `0`. Así usa `dacWrite()`: funciona igual, pero cada cambio de
> nivel tarda unos µs más.

### 3.2 Instalar el programa en Python

Requiere Python 3.9 o superior.

```bash
cd Scripts/Python
python3 -m pip install -r requirements.txt
```

En Linux, el usuario debe tener permiso para usar el puerto serie (una sola vez, luego cierre
sesión y vuelva a entrar):

```bash
sudo usermod -a -G dialout $USER
```

### 3.3 Ejecutar

```bash
cd Scripts/Python
python3 led_dvs_gui.py
```

---

## 4. Uso de la interfaz

### 4.1 Conexión (barra superior)

- Elija el puerto (`/dev/ttyUSB0`, normalmente) y presione **Conectar**. El programa abre el puerto en modo exclusivo: si otro programa lo tiene abierto (por ejemplo el Monitor Serie del Arduino IDE), avisa en vez de conectarse a medias. La placa **se reinicia
  al conectar** (el LED queda en 0) y el programa comprueba que tenga el firmware v2.
- **SIMULADOR (sin hardware)**: es una ESP32 simulada para probar la interfaz sin la placa. La
  temporización es la del PC, así que no sirve para medir.
- **⟳** actualiza la lista de puertos.
- Cada `SET` y `BASE` se verifica: la ESP32 confirma el valor recibido (`ACK SET 150`) y, si no coincide con el enviado, la interfaz muestra un aviso de **error de comunicación**.

### 4.2 Panel izquierdo (común a todas las pestañas)

| Elemento | Qué hace |
|---|---|
| **Nivel base** | Nivel de reposo. Todos los loops y barridos alternan contra este nivel, y STOP vuelve a él. **Aplicar base ahora** lo deja puesto en el LED. |
| **Pausa inicial en base** | Tiempo en la base antes de la primera oscilación (por defecto **3 s**). Separa en el video el encendido del primer nivel. |
| **Grabar registro CSV** | Desactivado por defecto. Ver sección 5. |
| **Estado** | Salida actual del LED, fase (pausa inicial / oscilando / pausa entre niveles), tramo actual, tiempo transcurrido y restante, ciclos y barra de progreso. |
| **STOP** (o tecla **Esc**) | Detiene lo que esté corriendo en menos de 1 ms y deja el LED **en el nivel base**. Para apagar el LED, ponga la base en 0 o use SET 0. |

Mientras corre una secuencia no se puede cambiar la base, la pausa inicial ni el registro, ni
lanzar otra secuencia. STOP siempre está disponible.

### 4.3 Temporización: onda cuadrada o tiempos separados

Las pestañas *Loop simple* y *Continuo* tienen un bloque **Temporización** con un botón que
cambia entre dos modos:

- **Onda cuadrada 50 % (por defecto)**: se ingresa la **frecuencia**. El tiempo en el nivel alto
  y en la base son iguales (semiperiodo = 1 / 2f).
- **Tiempos separados**: se ingresan por separado el **tiempo en nivel alto** y el **tiempo en
  nivel base**, en ms con resolución de µs (0.001 ms).

Debajo se muestra lo que realmente se enviará: t_alto, t_base, periodo, **frecuencia real**
(los tiempos se redondean a µs enteros) y ciclo de trabajo. Sobre 20 kHz aparece un aviso
porque la precisión relativa baja. El semiperiodo mínimo es 5 µs (100 kHz).

Las pestañas de barrido de niveles usan el mismo bloque. El barrido de frecuencia usa
frecuencias y **ciclo de trabajo %**.

### 4.4 Pestaña «Nivel fijo (SET)»

Deja un nivel **continuo**, sin oscilar. Sirve para:
- ver en la cámara si un nivel se ve bien o satura,
- medir la potencia (mW) de cada nivel con el fotodetector, para la curva nivel → mW.

Se elige el nivel con el número o la barra y se presiona **Aplicar nivel (SET)**. Con
*Aplicar automáticamente al mover*, el LED sigue a la barra en tiempo real.

**Diagnóstico del circuito (firmware 2.1):** con dos cables puente, **D34 → base** del
transistor y **D35 → emisor** (pata de las resistencias), la ESP32 mide esos voltajes con su
ADC. *Medir nivel actual* agrega una fila a la tabla. *Prueba automática 0 → 255* recorre los
niveles 0, 25, 50 … 255 (≈ 1.5 s cada uno) y llena la tabla con la base esperada y medida,
el emisor, la corriente del LED (emisor / R_E) y una lectura automática:
- **"base NO sigue al nivel"**: problema en la ESP32 o en el cable de la base.
- **"pasa corriente con nivel 0"**: problema en el circuito (fuga, corto o transistor dañado).
- **OK** en todas las filas, pero el brillo no sube: el transistor está saturado (ver color del
  LED en la sección 1).
D34 y D35 son pines solo de entrada, así que conectarlos no altera el circuito. Sin los puentes,
sus lecturas no significan nada.

### 4.5 Pestaña «Loop simple»

Alterna base ↔ nivel alto la cantidad de ciclos indicada. Se elige: nivel alto, temporización
y ciclos. Abajo aparece el **tiempo estimado** (incluye la pausa inicial).

Secuencia: `[pausa inicial en base] → (alto, base) × ciclos → queda en base`.

### 4.6 Pestaña «Barrido de niveles»

Recorre una serie de niveles altos. **Cada uno alterna con la base**, y entre niveles el LED
queda en la base durante la **pausa entre niveles** (por defecto **2 s**). Así cada nivel queda
separado en el video.

Ejemplo: base 60, desde 70 hasta 80, paso 5:

```
3 s en 60 → loop(60↔70) → 2 s en 60 → loop(60↔75) → 2 s en 60 → loop(60↔80) → queda en 60
```

- **Desde / hasta / paso**: los niveles van de `min(desde, hasta)` a `max(desde, hasta)` con ese
  paso. El último solo se incluye si cae justo en la serie (70–82 paso 5 da 70, 75, 80).
- **Dirección**: el botón cambia entre **▲ Ascendente** (por defecto) y **▼ Descendente**.
  En descendente se recorre **la misma lista al revés**, así ambos sentidos usan los mismos niveles.
- **Ciclos por nivel**, **temporización** y **pausa entre niveles** son iguales para toda la serie.
- La **tabla** muestra la serie completa antes de empezar: #, base, alto, frecuencia, tiempos,
  ciclos, duración y la hora de inicio y fin de cada nivel (contada desde que se presiona
  Iniciar). Durante la ejecución se marca la fila que está corriendo.

### 4.7 Pestaña «Continuo»

Alterna base ↔ nivel alto **indefinidamente** hasta presionar STOP. El panel de estado muestra
el tiempo transcurrido y los ciclos hechos.

### 4.8 Pestaña «Barrido de frecuencia»

Nivel alto fijo y la **frecuencia** recorre una serie lineal (desde, hasta, paso en Hz), con
dirección ascendente o descendente, **ciclo de trabajo** (50 % = onda cuadrada), ciclos por
frecuencia y pausa entre frecuencias (2 s). Tiene la misma tabla de vista previa. Como máximo
admite 512 frecuencias.

### 4.9 Mensajes de la ESP32

El recuadro inferior muestra lo que se envía (`> ...`) y lo que responde la placa. La línea de
abajo permite escribir comandos a mano (`PING`, `INFO`, `SET 120`…), útil para diagnóstico.

### 4.10 Tiempo estimado

Se calcula así:

```
total = pausa_inicial + Σ (ciclos × (t_alto + t_base))  +  (N − 1) × pausa_entre_niveles
```

Es exacto porque la ESP32 usa la **misma** línea de tiempo: cada flanco ocurre en un instante
calculado desde el inicio. Antes de empezar se agregan algunos ms, el tiempo que tarda en
cargarse la secuencia por USB (menos de 1 s incluso con cientos de niveles).

---

## 5. Registro CSV (opcional)

Si **Grabar registro CSV** está marcado, cada ejecución crea un archivo en la carpeta elegida
(por defecto `Python/registros/`):

```
LED_log_2026-10-12_14h23.csv        ← fecha, hora y minuto de inicio
LED_log_2026-10-12_14h23_2.csv      ← si hubo dos en el mismo minuto
```

Contenido: unas líneas de cabecera `#` con los parámetros, y luego una fila por evento:

```
t_us,evento,tramo,base,alto,t_alto_us,t_bajo_us,frecuencia_hz,ciclos_programados,ciclos_hechos
0,INICIO_PAUSA_INICIAL,,60,70,100000,100000,5,100,
3000000,NIVEL_INICIO,1,60,70,100000,100000,5,100,
23000000,NIVEL_FIN_INICIO_PAUSA,1,60,70,100000,100000,5,100,100
25000000,NIVEL_INICIO,2,60,75,100000,100000,5,100,
...
89000000,NIVEL_FIN,4,60,85,100000,100000,5,100,100
89000000,FIN,,,,,,,,
```

- `t_us`: **microsegundos desde 0** (el inicio de la ejecución), medidos con el **reloj de la ESP32**.
  Son los instantes programados de los flancos. Los reales difieren en pocos µs.
- `evento`: `INICIO_PAUSA_INICIAL`, `NIVEL_INICIO`, `NIVEL_FIN_INICIO_PAUSA`, `NIVEL_FIN`
  (último nivel), `FIN`, o `ABORTADO_STOP` (con el tramo y los ciclos hechos al detener).
- Se registra un evento por **nivel**, no por flanco. Dentro de un nivel, los flancos son
  deterministas: el flanco k está en `t_inicio + k × periodo`.

El reloj de la ESP32 **no está sincronizado** con el de la cámara. Para alinear ambos
registros, use las pausas en la base: en el archivo de eventos aparecen como intervalos sin
eventos de señal.

---

## 6. Relación con el paper (McReynolds, Graca & Delbruck, 2022)

| Medición del paper | Cómo hacerla aquí |
|---|---|
| **Umbral de contraste θ** (Tabla 2): onda cuadrada de 5 Hz, grabar ≥ 20 s por amplitud y bajar la amplitud de a pasos pequeños | *Barrido de niveles*, 5 Hz, **100 ciclos** (= 20 s), dirección **▼ Descendente** si se quiere bajar como en el paper, pausa de 2 s |
| **Frecuencia de corte f₃dB** (Tabla 3): amplitud fija, frecuencia creciente en pasos de 5 % de la inicial, 1000 ciclos por frecuencia | *Barrido de frecuencia* con 1000 ciclos. **Ojo:** el paper usa estímulo **sinusoidal** y aquí es **cuadrado**; el paper indica que el cuadrado se puede modelar igual con la respuesta de primer orden, pero no es la misma medición directa |
| **Periodo refractario** (Tabla 4): onda cuadrada de gran amplitud a 1 Hz, 100 ciclos | *Loop simple*, 1 Hz, 100 ciclos, nivel alto grande |

En el paper, la amplitud de un estímulo se expresa como `ln(I_max / I_min)`. Aquí eso es
`ln(P(alto) / P(base))`, con las potencias que mida el fotodetector para cada nivel.

---

## 7. Detalles del firmware v2

### Diferencias con v1

| | v1 | v2 |
|---|---|---|
| STOP | dejaba el LED en **0** | deja el LED en el **nivel base** |
| Reacción a STOP | hasta 1 ciclo completo (1 s a 1 Hz) | < 1 ms en cualquier condición |
| Temporización | relativa ("esperar t"); los retardos se acumulan | **absoluta**: cada flanco en `t₀ + Σ tiempos`; el error no se acumula |
| Escritura del DAC | `dacWrite()` en cada flanco | registro directo (< 1 µs) |
| Barridos | un comando por nivel desde el PC (huecos de ms entre niveles) | **toda la secuencia en la ESP32** |
| Modo continuo | no | sí (ciclos = 0) |
| Marcas de tiempo | no | sí, en µs, para el registro |

### Cambios de la versión 2.1

- `SET`, `BASE`, `STOP` y el inicio y el final de cada ejecución usan **`dacWrite()`**, la
  función oficial de Arduino. La escritura directa al registro del DAC (más rápida) se usa
  solo para los flancos de las ondas, que es donde importa la velocidad.
- Nuevo comando `MEAS` para el diagnóstico con el ADC (D34 y D35).
- `INFO` incluye `fw=2.1`.

### Flancos y la meta de 10 µs

Hay dos cosas distintas:

1. **Duración del flanco** (cuánto tarda la luz en cambiar de nivel). Depende del DAC, el
   transistor y el LED, y debería estar bajo 10 µs. **No ha sido medido.** Para verificarlo, ponga
   un **osciloscopio sobre la resistencia de 150 Ω** (entre emisor y GND). Ese voltaje es
   proporcional a la corriente del LED, así que muestra directamente la forma de la onda de luz.
2. **Precisión del instante de cada flanco**. Escribir el DAC toma menos de 1 µs. Las interrupciones
   del sistema (por ejemplo, la recepción serie) pueden retrasar un flanco en algunos µs, pero el
   siguiente vuelve al instante correcto porque los tiempos son absolutos. Si un flanco llega muy
   tarde, el firmware se pone al día sin cambiar la duración total.

Con semiperiodos menores a 25 µs (sobre ~20 kHz), el firmware revisa el puerto serie cada 2048
ciclos, lo que agrega unos µs de jitter en esos ciclos. Para la cámara esto no importa: según el
paper, el periodo refractario mínimo es ~330 µs y el fotorreceptor responde en escalas de ms.

### Protocolo serie (115200 8N1, líneas con `\n`)

| Comando | Respuesta | Descripción |
|---|---|---|
| `PING` | `PONG` | Comprobar comunicación |
| `INFO` | `INFO LED_DVS_v2 fw=2.2 dac=26 maxseg=512 base=<b> out=<o> nseg=<n>` | Versión y estado |
| `SET <v>` | `ACK SET <v>` | Nivel fijo 0–255 (no cambia la base) |
| `BASE <v>` | `ACK BASE <v>` | Define la base y la aplica |
| `CLEAR` | `ACK CLEAR` | Vacía la cola de tramos |
| `Q <base> <alto> <t_alto_us> <t_bajo_us> <ciclos> <pausa_us>` | (nada si está bien) / `ERR Q <motivo> <idx>` | Agrega un tramo. `ciclos = 0`: infinito |
| `RUN <pausa_inicial_us>` | `ACK RUN <n>` + eventos | Ejecuta la cola |
| `STOP` | `ACK STOP` | Detiene y deja la base |
| `MEAS` | `MEAS out=<n> d34=<mV> d35=<mV>` | Diagnóstico: lee GPIO34 y GPIO35 con el ADC (v2.1) |

Eventos durante `RUN` (t en µs desde el inicio):

```
EV <t> START <n>
EV <t> SEG <i> <base> <alto> <t_alto_us> <t_bajo_us> <ciclos>
EV <t> SEGEND <i> <ciclos_hechos>
EV <t> DONE
EV <t> ABORTED <i> <ciclos_hechos>      (seguido de ACK STOP)
```

Durante una ejecución solo se aceptan `STOP` y `PING`; cualquier otro comando responde
`ERR BUSY`. Errores posibles: `ERR PARAM_MISSING`, `ERR VAL_OUT_OF_RANGE`,
`ERR Q BAD_PARAMS|INTENSITY_RANGE|MIN_TIME|QUEUE_FULL <idx>`, `ERR QUEUE_EMPTY`,
`ERR UNKNOWN_COMMAND`. Límites: 512 tramos y semiperiodo mínimo de 5 µs.

---

## 8. Problemas frecuentes

| Síntoma | Solución |
|---|---|
| `Permission denied: '/dev/ttyUSB0'` | Agregar el usuario al grupo `dialout` (sección 3.2) y volver a iniciar sesión |
| "Puerto ocupado" / no conecta | Cerrar el Monitor Serie del Arduino IDE u otro programa que use el puerto |
| "Firmware NO v2" | Cargar `firmware_esp32_control_led_dvs_v2.ino` |
| "Comunicación inestable" o "⚠ Error de comunicación: se envió SET X pero la ESP32 aplicó SET Y" | Se están perdiendo o dañando datos en el puerto serie: cerrar el Monitor Serie del Arduino IDE, usar un cable USB corto de datos directo al PC (sin hub), y revisar `sudo dmesg -w` por desconexiones o `brltty` |
| Los niveles no corresponden a lo que se pide, pero no hay avisos de comunicación | Revisar el circuito: voltaje de la batería (con carga), masa común firme, contactos de la protoboard. Medir con multímetro el voltaje en R_E (SET 150 → ≈ 1.2 V) |
| La placa no responde | Revisar que el cable USB sea de datos, presionar EN en la placa y volver a conectar |
| El LED no enciende con niveles bajos | Normal bajo ~54 (transistor casi en corte). Probar SET 150 |
| El LED no se apaga con SET 0 | Problema del circuito, no del código: con SET 0 la base queda en ~0 V. Hacer la prueba rápida del montaje (sección 1) y la prueba de diagnóstico (abajo) |
| El brillo sube y luego **baja** en los niveles altos | Saturación: el LED necesita más voltaje del que deja la fuente (LED azul, blanco o verde puro) o la fuente está baja. Ver "Color del LED" en la sección 1 |

### Prueba de diagnóstico: ¿es el código o el circuito?

`ESP32/prueba_dac_esp32/prueba_dac_esp32.ino` (versión 3) es un programa independiente del
firmware principal. Usa solo `dacWrite` y `analogReadMilliVolts` de Arduino, y se maneja desde el
**Monitor Serie** del Arduino IDE (115200 baudios, "Nueva línea"), sin el programa de Python.

**Conexiones:** D26 → base · puente D34 → D26 (mide la base) · puente D35 → emisor ·
GND → negativo de las resistencias · fuente de 5 V conectada (funcionamiento normal).

**Comandos:** un número `0`–`255` aplica ese nivel y mide · `t` hace un barrido completo con
veredicto · `d` muestra los registros del DAC · `p25` / `p26` cambia el pin del DAC.

Cada línea entrega:
- **reg:** el valor que quedó escrito en el registro interno del DAC, leído directamente del chip.
  `CODIGO-OK` significa que es exactamente el nivel pedido, que el DAC está encendido y que no hay
  ninguna función extra activa (generador de coseno, pull-up o pull-down del pin). Esto separa
  el **código** del **hardware**.
- **base / emisor:** promedio de 128 lecturas, con el mínimo y el máximo entre corchetes. Una
  diferencia grande se marca como `INESTABLE` (contacto malo o ruido).
- **I_LED:** emisor / R_E.

Al final del barrido (`t`) se imprime un **veredicto**: (1) si el código escribe bien todos los
niveles y (2) si la base sube de forma pareja. El ADC de la ESP32 marca ~140 mV cuando el
voltaje real es 0 V.

Después de la prueba, volver a cargar `firmware_esp32_control_led_dvs_v2.ino`.
| Ruido de 50 Hz en la cámara | Revisar la masa común (GND de la ESP32 unido al negativo de la batería) |

---

## 9. Estado de las pruebas

- El **firmware v2** se compiló y ejecutó en un PC con una capa que imita la API de
  Arduino/ESP32. Se verificó que los comandos y errores respondan bien, que STOP funcione durante
  oscilación y durante pausas (dejando la base), los modos continuo y barrido, y la línea de
  tiempo: los flancos caen en los instantes programados y el periodo medio es exacto, sin deriva.
  **Falta compilarlo y probarlo en la DevKit real**, y medir los flancos con osciloscopio.
- La **lógica de Python** (series, tiempos estimados, protocolo, registro CSV, simulador) se
  probó contra ese firmware emulado y contra el simulador.
- La **interfaz PyQt6** no se pudo abrir en el entorno donde se escribió (no había PyQt6). La
  primera prueba recomendada es abrirla con **SIMULADOR (sin hardware)**.
