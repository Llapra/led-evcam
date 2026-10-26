"""
led_core.py — Lógica del generador de estímulos LED para la EvCam (sin interfaz gráfica).

Contiene todo lo que no depende de Qt, para poder probarlo por separado:
  * Segment ............ un tramo "base <-> alto" con sus tiempos, ciclos y pausa posterior.
  * build_* ............ constructores de secuencias (loop simple, barrido de niveles,
                         barrido de frecuencias, continuo).
  * schedule / duración  línea de tiempo planificada y tiempo total estimado.
  * protocolo .......... comandos que entiende el firmware v2 y lectura de sus eventos.
  * RunTracker ......... sigue una ejecución a partir de las líneas que llegan de la ESP32.
  * RunLogger .......... registro CSV (t_us desde 0) de una ejecución.
  * FakeESP32 .......... ESP32 simulada (misma interfaz que pyserial) para probar sin hardware.

Protocolo del firmware v2 (resumen; detalle en README.md):
  SET v | BASE v | CLEAR | Q base alto t_alto_us t_bajo_us ciclos pausa_us | RUN pausa_ini_us | STOP
  Eventos: EV t START n | EV t SEG i base alto th tl ciclos | EV t SEGEND i c | EV t DONE |
           EV t ABORTED i c
"""

from __future__ import annotations

import csv
import math
import os
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Iterable, List, Optional, Tuple

# ----------------------------------------------------------------------------------------
# Constantes del sistema
# ----------------------------------------------------------------------------------------
DAC_MIN = 0
DAC_MAX = 255
MIN_T_US = 5            # semiperiodo mínimo aceptado por el firmware (µs)
MAX_SEGMENTS = 512      # tamaño de la cola del firmware
PRECISION_WARN_HZ = 20_000.0  # sobre esto la temporización pierde precisión relativa
FW_ID = "LED_DVS_v2"
BAUDRATE = 115200

# Parámetros eléctricos (solo para mostrar valores aproximados)
V_REF = 3.3
V_BE = 0.7
R_E = 141.3       # resistencia de emisor medida en el montaje (ohm)


def dac_to_volts(n: int) -> float:
    return n * V_REF / DAC_MAX


def dac_to_ma(n: int) -> float:
    """Corriente aproximada del LED (mA) según I = (V_DAC - 0.7 V) / R_E. Solo orientativa."""
    return max(0.0, (dac_to_volts(n) - V_BE) / R_E * 1000.0)


# ----------------------------------------------------------------------------------------
# Segmentos y secuencias
# ----------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Segment:
    base: int
    high: int
    t_high_us: int
    t_low_us: int
    cycles: int          # 0 = infinito (modo continuo)
    pause_us: int = 0    # tiempo en base después del segmento

    @property
    def period_us(self) -> int:
        return self.t_high_us + self.t_low_us

    @property
    def freq_hz(self) -> float:
        return 1e6 / self.period_us

    @property
    def duty(self) -> float:
        return self.t_high_us / self.period_us

    @property
    def active_us(self) -> Optional[int]:
        """Duración de la oscilación (sin la pausa). None si es infinita."""
        if self.cycles == 0:
            return None
        return self.cycles * self.period_us

    def command(self) -> str:
        return (f"Q {self.base} {self.high} {self.t_high_us} {self.t_low_us} "
                f"{self.cycles} {self.pause_us}")


def clamp_level(v: int) -> int:
    return max(DAC_MIN, min(DAC_MAX, int(v)))


def times_from_freq(freq_hz: float, duty: float = 0.5) -> Tuple[int, int]:
    """Convierte frecuencia (Hz) y ciclo de trabajo (0..1) en (t_alto_us, t_bajo_us) enteros."""
    if freq_hz <= 0:
        raise ValueError("La frecuencia debe ser positiva")
    period = 1e6 / freq_hz
    th = int(round(period * duty))
    tl = int(round(period)) - th
    return max(MIN_T_US, th), max(MIN_T_US, tl)


def level_series(start: int, stop: int, step: int, descending: bool = False) -> List[int]:
    """Niveles desde min(start,stop) hasta max(start,stop) con paso 'step' (inclusive si cae justo).
    En descendente se recorre la MISMA lista al revés, para comparar ambos sentidos."""
    step = abs(int(step))
    if step < 1:
        raise ValueError("El paso debe ser >= 1")
    lo, hi = sorted((clamp_level(start), clamp_level(stop)))
    vals = list(range(lo, hi + 1, step))
    return vals[::-1] if descending else vals


def freq_series(f_start: float, f_stop: float, step: float, descending: bool = False) -> List[float]:
    """Frecuencias desde min hasta max con paso lineal 'step' (Hz)."""
    step = abs(float(step))
    if step <= 0:
        raise ValueError("El paso debe ser > 0")
    lo, hi = sorted((float(f_start), float(f_stop)))
    if lo <= 0:
        raise ValueError("Las frecuencias deben ser > 0")
    n = int(math.floor((hi - lo) / step + 1e-9)) + 1
    vals = [round(lo + k * step, 6) for k in range(n)]
    return vals[::-1] if descending else vals


def build_loop(base: int, high: int, t_high_us: int, t_low_us: int, cycles: int) -> List[Segment]:
    return [Segment(base, high, t_high_us, t_low_us, cycles, 0)]


def build_continuous(base: int, high: int, t_high_us: int, t_low_us: int) -> List[Segment]:
    return [Segment(base, high, t_high_us, t_low_us, 0, 0)]


def build_level_sweep(base: int, levels: Iterable[int], t_high_us: int, t_low_us: int,
                      cycles: int, pause_us: int) -> List[Segment]:
    levels = list(levels)
    segs = []
    for k, lv in enumerate(levels):
        last = (k == len(levels) - 1)
        segs.append(Segment(base, lv, t_high_us, t_low_us, cycles, 0 if last else pause_us))
    return segs


def build_freq_sweep(base: int, high: int, freqs: Iterable[float], duty: float,
                     cycles: int, pause_us: int) -> List[Segment]:
    freqs = list(freqs)
    segs = []
    for k, f in enumerate(freqs):
        th, tl = times_from_freq(f, duty)
        last = (k == len(freqs) - 1)
        segs.append(Segment(base, high, th, tl, cycles, 0 if last else pause_us))
    return segs


def validate(segments: List[Segment], pre_us: int) -> List[str]:
    """Devuelve una lista de problemas (vacía si todo está bien)."""
    errs = []
    if not segments:
        errs.append("La secuencia está vacía.")
    if len(segments) > MAX_SEGMENTS:
        errs.append(f"La secuencia tiene {len(segments)} tramos; el máximo es {MAX_SEGMENTS}.")
    if pre_us < 0:
        errs.append("La pausa inicial no puede ser negativa.")
    for k, s in enumerate(segments):
        if not (DAC_MIN <= s.base <= DAC_MAX and DAC_MIN <= s.high <= DAC_MAX):
            errs.append(f"Tramo {k + 1}: nivel fuera de 0..255.")
        if s.t_high_us < MIN_T_US or s.t_low_us < MIN_T_US:
            errs.append(f"Tramo {k + 1}: tiempos menores a {MIN_T_US} µs.")
        if s.cycles < 0:
            errs.append(f"Tramo {k + 1}: ciclos negativos.")
        if s.cycles == 0 and len(segments) > 1:
            errs.append("El modo infinito (ciclos = 0) solo se permite con un único tramo.")
            break
        for name, v in (("t_alto", s.t_high_us), ("t_bajo", s.t_low_us), ("pausa", s.pause_us),
                        ("ciclos", s.cycles)):
            if v > 0xFFFFFFFF:
                errs.append(f"Tramo {k + 1}: {name} demasiado grande.")
    return errs


@dataclass(frozen=True)
class ScheduledSegment:
    index: int
    segment: Segment
    start_us: int              # inicio de la oscilación
    end_us: Optional[int]      # fin de la oscilación (None si infinita)
    pause_end_us: Optional[int]


def schedule(segments: List[Segment], pre_us: int) -> List[ScheduledSegment]:
    """Línea de tiempo planificada (misma aritmética que el firmware)."""
    out = []
    t = pre_us
    for k, s in enumerate(segments):
        act = s.active_us
        if act is None:
            out.append(ScheduledSegment(k, s, t, None, None))
            break
        end = t + act
        out.append(ScheduledSegment(k, s, t, end, end + s.pause_us))
        t = end + s.pause_us
    return out


def sequence_duration_us(segments: List[Segment], pre_us: int) -> Optional[int]:
    """Duración total (µs) incluyendo pausa inicial y pausas. None si es infinita."""
    sch = schedule(segments, pre_us)
    if not sch:
        return pre_us
    last = sch[-1]
    return last.pause_end_us


def segment_at(sched: List[ScheduledSegment], t_us: float) -> Tuple[str, int]:
    """Qué está pasando en el instante t_us de la ejecución: ('pre'|'seg'|'pause'|'end', índice)."""
    if not sched or t_us < sched[0].start_us:
        return ("pre", -1)
    for s in sched:
        if s.end_us is None or t_us < s.end_us:
            if t_us >= s.start_us:
                return ("seg", s.index)
        elif s.pause_end_us is not None and t_us < s.pause_end_us:
            return ("pause", s.index)
    return ("end", len(sched) - 1)


def fmt_duration(us: Optional[float]) -> str:
    """Formatea µs en texto legible."""
    if us is None:
        return "∞ (hasta STOP)"
    us = float(us)
    if us < 1000:
        return f"{us:.0f} µs"
    if us < 1e6:
        return f"{us / 1000:.3f} ms"
    s = us / 1e6
    if s < 60:
        return f"{s:.3f} s"
    m, s = divmod(s, 60)
    if m < 60:
        return f"{int(m)} min {s:06.3f} s"
    h, m = divmod(int(m), 60)
    return f"{h} h {m:02d} min {s:06.3f} s"


def fmt_clock(us: float) -> str:
    """Formato reloj m:ss.s para la tabla de vista previa."""
    s = max(0.0, us / 1e6)
    m, s = divmod(s, 60)
    if m >= 60:
        h, m = divmod(int(m), 60)
        return f"{h}:{m:02d}:{s:04.1f}"
    return f"{int(m)}:{s:04.1f}"


def fmt_freq(f: float) -> str:
    if f >= 1000:
        return f"{f / 1000:.4g} kHz"
    return f"{f:.4g} Hz"


# ----------------------------------------------------------------------------------------
# Protocolo
# ----------------------------------------------------------------------------------------
def commands_for_run(segments: List[Segment], pre_us: int) -> List[str]:
    return ["CLEAR"] + [s.command() for s in segments] + [f"RUN {int(pre_us)}"]


@dataclass(frozen=True)
class Event:
    t_us: int
    kind: str
    args: Tuple[int, ...]


def parse_event(line: str) -> Optional[Event]:
    """'EV <t> <KIND> <args...>' -> Event. None si la línea no es un evento."""
    parts = line.split()
    if len(parts) < 3 or parts[0] != "EV":
        return None
    try:
        t = int(parts[1])
        args = tuple(int(x) for x in parts[3:])
    except ValueError:
        return None
    return Event(t, parts[2], args)


# ----------------------------------------------------------------------------------------
# Registro CSV
# ----------------------------------------------------------------------------------------
LOG_COLUMNS = ["t_us", "evento", "tramo", "base", "alto", "t_alto_us", "t_bajo_us",
               "frecuencia_hz", "ciclos_programados", "ciclos_hechos"]


class RunLogger:
    """Archivo LED_log_AAAA-MM-DD_HHhMM.csv; t_us = µs desde el inicio de la ejecución,
    medidos con el reloj de la ESP32 (instantes programados de los flancos)."""

    def __init__(self) -> None:
        self._f = None
        self._w = None
        self.path: Optional[str] = None

    @staticmethod
    def make_path(folder: str, when: datetime) -> str:
        base = f"LED_log_{when:%Y-%m-%d_%Hh%M}"
        path = os.path.join(folder, base + ".csv")
        k = 2
        while os.path.exists(path):
            path = os.path.join(folder, f"{base}_{k}.csv")
            k += 1
        return path

    def open(self, folder: str, when: datetime, meta: dict) -> str:
        os.makedirs(folder, exist_ok=True)
        self.path = self.make_path(folder, when)
        self._f = open(self.path, "w", newline="", encoding="utf-8")
        self._f.write(f"# Registro generador LED EvCam - inicio {when:%Y-%m-%d %H:%M:%S}\n")
        for k, v in meta.items():
            self._f.write(f"# {k}: {v}\n")
        self._f.write("# t_us: microsegundos desde el inicio (reloj de la ESP32)\n")
        self._w = csv.writer(self._f)
        self._w.writerow(LOG_COLUMNS)
        return self.path

    def row(self, t_us: int, evento: str, seg: Optional[Segment] = None, idx: Optional[int] = None,
            done_cycles: Optional[int] = None) -> None:
        if not self._w:
            return
        if seg is None:
            self._w.writerow([t_us, evento, "" if idx is None else idx + 1, "", "", "", "", "", "",
                              "" if done_cycles is None else done_cycles])
        else:
            self._w.writerow([t_us, evento, "" if idx is None else idx + 1, seg.base, seg.high,
                              seg.t_high_us, seg.t_low_us, f"{seg.freq_hz:.6g}",
                              seg.cycles if seg.cycles else "inf",
                              "" if done_cycles is None else done_cycles])

    def close(self) -> None:
        if self._f:
            self._f.close()
        self._f = None
        self._w = None

    @property
    def is_open(self) -> bool:
        return self._f is not None


# ----------------------------------------------------------------------------------------
# Seguimiento de una ejecución
# ----------------------------------------------------------------------------------------
class RunTracker:
    """Máquina de estados de una ejecución. Se alimenta con las líneas recibidas.

    feed(line) devuelve una lista de notificaciones:
      ('started', n)  ('segment', i)  ('segment_end', i, ciclos)  ('done',)
      ('aborted', i, ciclos)  ('error', texto)
    """

    IDLE, UPLOADING, RUNNING = "idle", "uploading", "running"

    def __init__(self) -> None:
        self.state = self.IDLE
        self.segments: List[Segment] = []
        self.pre_us = 0
        self.logger: Optional[RunLogger] = None
        self.last_t_us = 0

    def begin(self, segments: List[Segment], pre_us: int,
              logger: Optional[RunLogger] = None) -> List[str]:
        self.segments = list(segments)
        self.pre_us = int(pre_us)
        self.logger = logger
        self.state = self.UPLOADING
        self.last_t_us = 0
        return commands_for_run(self.segments, self.pre_us)

    @property
    def busy(self) -> bool:
        return self.state != self.IDLE

    def _finish(self) -> None:
        self.state = self.IDLE
        if self.logger:
            self.logger.close()

    def reset(self) -> None:
        self._finish()

    def feed(self, line: str) -> list:
        out = []
        if self.state == self.IDLE:
            return out
        if self.state == self.UPLOADING:
            if line.startswith("ACK RUN"):
                try:
                    n = int(line.split()[2])
                except (IndexError, ValueError):
                    n = -1
                if n != len(self.segments):
                    out.append(("error", f"La ESP32 recibió {n} tramos de {len(self.segments)}."))
                    out.append(("need_stop",))
                    self._finish()
                    return out
                self.state = self.RUNNING
                return out
            if line.startswith("ERR"):
                out.append(("error", f"La ESP32 rechazó la secuencia: {line}"))
                out.append(("need_stop",))
                self._finish()
                return out
            if line.startswith("ACK CLEAR"):
                return out
            return out

        # RUNNING
        ev = parse_event(line)
        if ev is None:
            return out
        self.last_t_us = ev.t_us
        lg = self.logger if (self.logger and self.logger.is_open) else None
        if ev.kind == "START":
            if lg:
                lg.row(0, "INICIO_PAUSA_INICIAL", self.segments[0] if self.segments else None,
                       None)
            out.append(("started", len(self.segments)))
        elif ev.kind == "SEG" and ev.args:
            i = ev.args[0]
            if lg and i < len(self.segments):
                lg.row(ev.t_us, "NIVEL_INICIO", self.segments[i], i)
            out.append(("segment", i))
        elif ev.kind == "SEGEND" and len(ev.args) >= 2:
            i, c = ev.args[0], ev.args[1]
            if lg and i < len(self.segments):
                last = (i == len(self.segments) - 1)
                lg.row(ev.t_us, "NIVEL_FIN" if last else "NIVEL_FIN_INICIO_PAUSA",
                       self.segments[i], i, c)
            out.append(("segment_end", i, c))
        elif ev.kind == "DONE":
            if lg:
                lg.row(ev.t_us, "FIN")
            out.append(("done",))
            self._finish()
        elif ev.kind == "ABORTED":
            i = ev.args[0] if ev.args else -1
            c = ev.args[1] if len(ev.args) > 1 else 0
            if lg:
                lg.row(ev.t_us, "ABORTADO_STOP", None, i, c)
            out.append(("aborted", i, c))
            self._finish()
        return out


# ----------------------------------------------------------------------------------------
# ESP32 simulada (para probar la interfaz sin hardware)
# ----------------------------------------------------------------------------------------
class FakeESP32:
    """Imita el firmware v2 con la interfaz mínima de serial.Serial que usa el programa:
    write(bytes), read(n), in_waiting, close(), is_open. La temporización es la del PC
    (precisión de ms): sirve para probar la interfaz, no para medir."""

    def __init__(self) -> None:
        self._rx = bytearray()           # lo que la "ESP32" envía al PC
        self._lock = threading.Lock()
        self._inbuf = ""
        self._queue: List[Segment] = []
        self.base = 0
        self.out = 0
        self.is_open = True
        self._abort = threading.Event()
        self._run_thread: Optional[threading.Thread] = None
        self.port = "SIMULADOR"
        self._emit("READY")

    # --- lado PC ---
    @property
    def in_waiting(self) -> int:
        with self._lock:
            return len(self._rx)

    def read(self, n: int = 1) -> bytes:
        with self._lock:
            data = bytes(self._rx[:n])
            del self._rx[:n]
            return data

    def write(self, data: bytes) -> int:
        self._inbuf += data.decode("ascii", "ignore")
        while "\n" in self._inbuf:
            line, self._inbuf = self._inbuf.split("\n", 1)
            line = line.strip()
            if line:
                self._handle(line)
        return len(data)

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        with self._lock:
            self._rx.clear()

    def close(self) -> None:
        self._abort.set()
        self.is_open = False

    # --- lado ESP32 ---
    def _emit(self, s: str) -> None:
        with self._lock:
            self._rx += (s + "\r\n").encode()

    def _running(self) -> bool:
        return self._run_thread is not None and self._run_thread.is_alive()

    def _handle(self, line: str) -> None:
        tok = line.split()
        cmd = tok[0].upper()
        if self._running():
            if cmd == "STOP":
                self._abort.set()
            elif cmd == "PING":
                self._emit("PONG")
            else:
                self._emit("ERR BUSY")
            return
        if cmd == "PING":
            self._emit("PONG")
        elif cmd == "INFO":
            self._emit(f"INFO {FW_ID} fw=2.1 dac=25 maxseg={MAX_SEGMENTS} base={self.base} "
                       f"out={self.out} nseg={len(self._queue)} SIMULADO")
        elif cmd in ("SET", "BASE"):
            if len(tok) < 2:
                self._emit("ERR PARAM_MISSING")
                return
            try:
                v = int(tok[1])
            except ValueError:
                v = -1
            if not 0 <= v <= 255:
                self._emit("ERR VAL_OUT_OF_RANGE")
                return
            if cmd == "BASE":
                self.base = v
            self.out = v
            self._emit(f"ACK {cmd} {v}")
        elif cmd == "MEAS":
            vb = int(round(dac_to_volts(self.out) * 1000))
            self._emit(f"MEAS out={self.out} d34={vb} d35={max(0, vb - 650)}")
        elif cmd == "CLEAR":
            self._queue.clear()
            self._emit("ACK CLEAR")
        elif cmd == "Q":
            try:
                v = [int(x) for x in tok[1:]]
                assert len(v) == 6 and min(v) >= 0
            except (ValueError, AssertionError):
                self._emit(f"ERR Q BAD_PARAMS {len(self._queue)}")
                return
            if v[0] > 255 or v[1] > 255:
                self._emit(f"ERR Q INTENSITY_RANGE {len(self._queue)}")
            elif v[2] < MIN_T_US or v[3] < MIN_T_US:
                self._emit(f"ERR Q MIN_TIME {len(self._queue)}")
            elif len(self._queue) >= MAX_SEGMENTS:
                self._emit(f"ERR Q QUEUE_FULL {len(self._queue)}")
            else:
                self._queue.append(Segment(*v))
        elif cmd == "RUN":
            if not self._queue:
                self._emit("ERR QUEUE_EMPTY")
                return
            pre = int(tok[1]) if len(tok) > 1 else 0
            self._emit(f"ACK RUN {len(self._queue)}")
            self._abort.clear()
            self._run_thread = threading.Thread(target=self._run, args=(pre,), daemon=True)
            self._run_thread.start()
        elif cmd == "STOP":
            self.out = self.base
            self._emit("ACK STOP")
        else:
            self._emit("ERR UNKNOWN_COMMAND")

    def _run(self, pre_us: int) -> None:
        t0 = time.perf_counter()
        segs = list(self._queue)
        self.base = segs[0].base
        self.out = self.base
        self._emit(f"EV 0 START {len(segs)}")

        def wait_until(t_us: int) -> bool:
            while True:
                rem = t_us / 1e6 - (time.perf_counter() - t0)
                if rem <= 0:
                    return True
                if self._abort.wait(min(rem, 0.02)):
                    return False

        t = pre_us
        i, c, ok = 0, 0, True
        for i, s in enumerate(segs):
            self.base = s.base
            c = 0
            if not wait_until(t):
                ok = False
                break
            self._emit(f"EV {t} SEG {i} {s.base} {s.high} {s.t_high_us} {s.t_low_us} {s.cycles}")
            if s.cycles == 0:
                # infinito: avanzar por bloques de tiempo hasta STOP
                while not self._abort.wait(0.05):
                    pass
                c = int(((time.perf_counter() - t0) * 1e6 - t) // s.period_us)
                ok = False
                break
            t_end = t + s.cycles * s.period_us
            if not wait_until(t_end):
                c = max(0, int(((time.perf_counter() - t0) * 1e6 - t) // s.period_us))
                ok = False
                break
            c = s.cycles
            t = t_end
            self._emit(f"EV {t} SEGEND {i} {c}")
            t += s.pause_us
            if not wait_until(t):
                ok = False
                break
        self.out = self.base
        if ok:
            self._emit(f"EV {t} DONE")
        else:
            now = int((time.perf_counter() - t0) * 1e6)
            self._emit(f"EV {now} ABORTED {i} {c}")
            self._emit("ACK STOP")
