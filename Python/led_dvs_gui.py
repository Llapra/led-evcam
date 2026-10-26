#!/usr/bin/env python3
"""
led_dvs_gui.py — Interfaz para controlar la intensidad del LED (ESP32 + DAC + 2N2222A)
usada en la caracterización de BIAS de la cámara de eventos.

Ejecutar:   python3 led_dvs_gui.py
Requiere:   PyQt6, pyserial  (pip install -r requirements.txt)
Firmware:   Scripts/ESP32/firmware_esp32_control_led_dvs_v2/firmware_esp32_control_led_dvs_v2.ino

Diseño para que la interfaz nunca se trabe:
  * Toda la comunicación serie vive en un hilo propio (SerialWorker en un QThread).
  * La temporización de las ondas la hace la ESP32; el PC solo envía la secuencia y escucha.
  * Las tablas y tiempos estimados se recalculan con un pequeño retardo (debounce).
"""

from __future__ import annotations

import os
import re
import sys
from collections import deque
import time
from datetime import datetime
from typing import List, Optional

from PyQt6.QtCore import (QMetaObject, QObject, QSettings, Qt, QThread, QTimer, pyqtSignal,
                          pyqtSlot)
from PyQt6.QtGui import QColor, QFont, QKeySequence, QPalette, QShortcut
from PyQt6.QtWidgets import (QAbstractItemView, QApplication, QCheckBox, QComboBox,
                             QDoubleSpinBox, QFileDialog, QFormLayout, QGridLayout,
                             QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit,
                             QMainWindow, QMessageBox, QPlainTextEdit, QProgressBar,
                             QPushButton, QSlider, QSpinBox, QSplitter, QTableWidget,
                             QTableWidgetItem, QTabWidget, QVBoxLayout, QWidget)

import led_core as core

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # permite usar el simulador aunque falte pyserial
    serial = None

SIM_PORT = "SIMULADOR (sin hardware)"
APP_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_LOG_DIR = os.path.join(APP_DIR, "registros")


# =========================================================================================
# Hilo de comunicación serie
# =========================================================================================
class SerialWorker(QObject):
    lineReceived = pyqtSignal(str)
    connected = pyqtSignal(str, str, bool)   # puerto, línea INFO, es_v2
    disconnected = pyqtSignal(str)           # motivo ("" = desconexión pedida)

    def __init__(self) -> None:
        super().__init__()
        self.ser = None
        self.buf = bytearray()
        self.timer: Optional[QTimer] = None

    @pyqtSlot()
    def start(self) -> None:
        self.timer = QTimer()
        self.timer.setInterval(4)
        self.timer.timeout.connect(self._poll)
        self.timer.start()

    # ---- utilidades internas ----
    def _readlines(self) -> List[str]:
        out = []
        n = self.ser.in_waiting
        if n:
            self.buf += self.ser.read(n)
        while b"\n" in self.buf:
            raw, _, rest = self.buf.partition(b"\n")
            self.buf = bytearray(rest)
            s = raw.decode("ascii", "replace").strip()
            if s:
                out.append(s)
        return out

    def _wait_for(self, pred, timeout_s: float) -> Optional[str]:
        t_end = time.perf_counter() + timeout_s
        while time.perf_counter() < t_end:
            for ln in self._readlines():
                if pred(ln):
                    return ln
            time.sleep(0.01)
        return None

    def _write(self, text: str) -> None:
        self.ser.write(text.encode("ascii"))

    # ---- slots llamados desde la interfaz ----
    @pyqtSlot(str)
    def open_port(self, port: str) -> None:
        self._close_silent()
        try:
            if port == SIM_PORT:
                self.ser = core.FakeESP32()
            else:
                if serial is None:
                    raise RuntimeError("Falta pyserial (pip install pyserial)")
                s = serial.Serial()
                s.port = port
                s.baudrate = core.BAUDRATE
                s.timeout = 0
                s.write_timeout = 5
                s.dtr = False      # no dejar IO0 en bajo (modo programación)
                s.rts = False
                s.exclusive = True  # falla si otro programa (p. ej. Monitor Serie) usa el puerto
                try:
                    s.open()
                except serial.SerialException as e:
                    raise RuntimeError(
                        f"{e}\n¿Está abierto el Monitor Serie del Arduino IDE u otro programa "
                        "usando el puerto? Ciérrelo e intente de nuevo.") from e
                # Reinicio limpio de la DevKit (igual que esptool): EN bajo 120 ms
                s.rts = True
                time.sleep(0.12)
                s.rts = False
                self.ser = s
            self.buf.clear()
            self._wait_for(lambda l: l == "READY", 3.0)
            time.sleep(0.1)
            info = None
            for _ in range(3):   # reintentos: una respuesta perdida no debe marcar "firmware antiguo"
                self._write("INFO\n")
                info = self._wait_for(lambda l: l.startswith("INFO") or l.startswith("ERR"), 1.0)
                if info is not None and (info.startswith("INFO") or "UNKNOWN" in info):
                    break
            if info is None:
                self._write("PING\n")
                if self._wait_for(lambda l: l == "PONG", 1.0) is None:
                    raise RuntimeError("La placa no responde (¿firmware cargado? ¿puerto correcto?)")
                info = "(sin INFO)"
            is_v2 = info.startswith("INFO") and core.FW_ID in info
            self.connected.emit(port, info, is_v2)
        except Exception as e:  # noqa: BLE001
            self._close_silent()
            self.disconnected.emit(f"No se pudo conectar a {port}: {e}")

    @pyqtSlot()
    def close_port(self) -> None:
        self._close_silent(send_stop=True)
        self.disconnected.emit("")

    @pyqtSlot(list)
    def send_lines(self, lines: list) -> None:
        if not self.ser:
            return
        try:
            self._write("".join(f"{ln}\n" for ln in lines))
        except Exception as e:  # noqa: BLE001
            self._close_silent()
            self.disconnected.emit(f"Error de escritura: {e}")

    @pyqtSlot()
    def shutdown(self) -> None:
        if self.timer:
            self.timer.stop()
        self._close_silent(send_stop=True)

    def _close_silent(self, send_stop: bool = False) -> None:
        if self.ser:
            try:
                if send_stop:
                    self._write("STOP\n")
                    time.sleep(0.05)
                self.ser.close()
            except Exception:  # noqa: BLE001
                pass
        self.ser = None

    @pyqtSlot()
    def _poll(self) -> None:
        if not self.ser:
            return
        try:
            for ln in self._readlines():
                self.lineReceived.emit(ln)
        except Exception as e:  # noqa: BLE001
            self._close_silent()
            self.disconnected.emit(f"Se perdió la conexión: {e}")


# =========================================================================================
# Widgets reutilizables
# =========================================================================================
def level_spin(default: int) -> QSpinBox:
    sb = QSpinBox()
    sb.setRange(core.DAC_MIN, core.DAC_MAX)
    sb.setValue(default)
    sb.setAccelerated(True)
    return sb


def dspin(lo: float, hi: float, val: float, dec: int, suffix: str) -> QDoubleSpinBox:
    sb = QDoubleSpinBox()
    sb.setRange(lo, hi)
    sb.setDecimals(dec)
    sb.setValue(val)
    sb.setSuffix(suffix)
    sb.setAccelerated(True)
    return sb


def level_hint(n: int) -> str:
    i = core.dac_to_ma(n)
    txt = f"≈ {core.dac_to_volts(n):.3f} V en base · ≈ {i:.2f} mA (teórico)"
    if i == 0:
        txt += (" · casi en corte: queda una corriente de microamperes que en oscuridad puede "
                "verse como un brillo muy tenue (para apagar del todo use 0)")
    return txt


class DirectionButton(QPushButton):
    """Botón que alterna Ascendente / Descendente (inicia en ascendente)."""
    changed = pyqtSignal()

    def __init__(self) -> None:
        super().__init__()
        self.setCheckable(True)
        self.toggled.connect(self._upd)
        self._upd()

    def _upd(self) -> None:
        self.setText("▼  Descendente" if self.isChecked() else "▲  Ascendente")
        self.changed.emit()

    @property
    def descending(self) -> bool:
        return self.isChecked()


class TimingWidget(QGroupBox):
    """Tiempos de la onda: modo onda cuadrada (frecuencia, 50 %) o tiempos separados."""
    changed = pyqtSignal()

    def __init__(self, default_freq: float = 5.0) -> None:
        super().__init__("Temporización")
        self.btn = QPushButton()
        self.btn.setCheckable(True)
        self.btn.toggled.connect(self._mode_changed)

        self.freq = dspin(0.001, 100_000.0, default_freq, 3, " Hz")
        half_ms = 1000.0 / (2 * default_freq)
        self.t_high = dspin(0.005, 3_600_000.0, half_ms, 3, " ms")
        self.t_low = dspin(0.005, 3_600_000.0, half_ms, 3, " ms")
        for w in (self.freq, self.t_high, self.t_low):
            w.valueChanged.connect(self._emit)

        self.info = QLabel()
        self.info.setWordWrap(True)

        lay = QGridLayout(self)
        lay.addWidget(self.btn, 0, 0, 1, 2)
        self.lbl_f = QLabel("Frecuencia:")
        self.lbl_th = QLabel("Tiempo en nivel alto:")
        self.lbl_tl = QLabel("Tiempo en nivel base:")
        lay.addWidget(self.lbl_f, 1, 0)
        lay.addWidget(self.freq, 1, 1)
        lay.addWidget(self.lbl_th, 2, 0)
        lay.addWidget(self.t_high, 2, 1)
        lay.addWidget(self.lbl_tl, 3, 0)
        lay.addWidget(self.t_low, 3, 1)
        lay.addWidget(self.info, 4, 0, 1, 2)
        self._mode_changed(False)

    @property
    def separate(self) -> bool:
        return self.btn.isChecked()

    def _mode_changed(self, sep: bool) -> None:
        # traspasar valores entre modos para no perder lo configurado
        for w in (self.freq, self.t_high, self.t_low):
            w.blockSignals(True)
        if sep:
            th, tl = core.times_from_freq(self.freq.value())
            self.t_high.setValue(th / 1000.0)
            self.t_low.setValue(tl / 1000.0)
            self.btn.setText("Tiempos SEPARADOS (alto ≠ base) — clic para onda cuadrada")
        else:
            period_ms = self.t_high.value() + self.t_low.value()
            if period_ms > 0:
                self.freq.setValue(1000.0 / period_ms)
            self.btn.setText("Onda cuadrada 50 % (alto = base) — clic para separar tiempos")
        for w in (self.freq, self.t_high, self.t_low):
            w.blockSignals(False)
        self.lbl_f.setVisible(not sep)
        self.freq.setVisible(not sep)
        for w in (self.lbl_th, self.t_high, self.lbl_tl, self.t_low):
            w.setVisible(sep)
        self._emit()

    def times_us(self):
        if self.separate:
            th = max(core.MIN_T_US, int(round(self.t_high.value() * 1000)))
            tl = max(core.MIN_T_US, int(round(self.t_low.value() * 1000)))
            return th, tl
        return core.times_from_freq(self.freq.value(), 0.5)

    def _emit(self) -> None:
        th, tl = self.times_us()
        per = th + tl
        f = 1e6 / per
        txt = (f"t_alto = {core.fmt_duration(th)} · t_base = {core.fmt_duration(tl)} · "
               f"periodo = {core.fmt_duration(per)} · f real = {core.fmt_freq(f)} · "
               f"ciclo de trabajo = {100 * th / per:.1f} %")
        if f > core.PRECISION_WARN_HZ:
            txt += "  ⚠ sobre 20 kHz la temporización pierde precisión relativa"
            self.info.setStyleSheet("color:#e0a050")
        else:
            self.info.setStyleSheet("color:#7f8eaa")
        self.info.setText(txt)
        self.changed.emit()


class SeriesTable(QTableWidget):
    COLS = ["#", "Base", "Alto", "Frecuencia", "t_alto / t_base", "Ciclos", "Duración",
            "Inicio", "Fin"]

    def __init__(self) -> None:
        super().__init__(0, len(self.COLS))
        self.setHorizontalHeaderLabels(self.COLS)
        self.verticalHeader().setVisible(False)
        self.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.setAlternatingRowColors(True)

    def fill(self, sched: List[core.ScheduledSegment]) -> None:
        self.setUpdatesEnabled(False)
        self.setRowCount(len(sched))
        for r, s in enumerate(sched):
            g = s.segment
            vals = [str(r + 1), str(g.base), str(g.high), core.fmt_freq(g.freq_hz),
                    f"{core.fmt_duration(g.t_high_us)} / {core.fmt_duration(g.t_low_us)}",
                    str(g.cycles) if g.cycles else "∞",
                    core.fmt_duration(g.active_us), core.fmt_clock(s.start_us),
                    core.fmt_clock(s.end_us) if s.end_us is not None else "—"]
            for c, v in enumerate(vals):
                it = QTableWidgetItem(v)
                it.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.setItem(r, c, it)
        self.setUpdatesEnabled(True)

    def mark(self, row: int) -> None:
        if 0 <= row < self.rowCount():
            self.selectRow(row)
            self.scrollToItem(self.item(row, 0), QAbstractItemView.ScrollHint.EnsureVisible)
        else:
            self.clearSelection()


# =========================================================================================
# Pestañas
# =========================================================================================
class ModeTab(QWidget):
    """Base de las pestañas que ejecutan una secuencia."""
    startRequested = pyqtSignal(object)   # self
    paramsChanged = pyqtSignal()

    title = ""
    has_table = False

    def __init__(self, main: "MainWindow") -> None:
        super().__init__()
        self.main = main
        self.table: Optional[SeriesTable] = SeriesTable() if self.has_table else None
        self.lbl_total = QLabel()
        f = QFont()
        f.setPointSize(f.pointSize() + 3)
        f.setBold(True)
        self.lbl_total.setFont(f)
        self.btn_start = QPushButton("▶  Iniciar")
        self.btn_start.setMinimumHeight(38)
        self.btn_start.setStyleSheet("font-weight:bold; font-size:14px;")
        self.btn_start.clicked.connect(lambda: self.startRequested.emit(self))
        self._debounce = QTimer(self)
        self._debounce.setSingleShot(True)
        self._debounce.setInterval(120)
        self._debounce.timeout.connect(self.refresh)
        self.paramsChanged.connect(self._debounce.start)

    def watch(self, *widgets) -> None:
        for w in widgets:
            if isinstance(w, (QSpinBox, QDoubleSpinBox)):
                w.valueChanged.connect(lambda *_: self.paramsChanged.emit())
            elif isinstance(w, (TimingWidget, DirectionButton)):
                w.changed.connect(lambda *_: self.paramsChanged.emit())

    def finish_layout(self, form_widget: QWidget) -> None:
        lay = QVBoxLayout(self)
        lay.addWidget(form_widget)
        if self.table is not None:
            lay.addWidget(QLabel("Serie que se ejecutará:"))
            lay.addWidget(self.table, 1)
        else:
            lay.addStretch(1)
        bottom = QHBoxLayout()
        bottom.addWidget(self.lbl_total, 1)
        bottom.addWidget(self.btn_start)
        lay.addLayout(bottom)

    # a implementar por cada pestaña
    def build(self) -> List[core.Segment]:
        raise NotImplementedError

    def describe(self) -> dict:
        return {}

    def refresh(self) -> None:
        pre = self.main.pre_us()
        try:
            segs = self.build()
            errs = core.validate(segs, pre)
        except ValueError as e:
            segs, errs = [], [str(e)]
        if errs:
            self.lbl_total.setText("⚠ " + errs[0])
            self.lbl_total.setStyleSheet("color:#ff7a7a")
            if self.table is not None:
                self.table.setRowCount(0)
            return
        self.lbl_total.setStyleSheet("")
        sched = core.schedule(segs, pre)
        total = core.sequence_duration_us(segs, pre)
        n = len(segs)
        extra = f" · {n} niveles" if n > 1 else ""
        self.lbl_total.setText(f"Tiempo estimado: {core.fmt_duration(total)}{extra} "
                               f"(incluye {core.fmt_duration(pre)} de pausa inicial)")
        if self.table is not None:
            self.table.fill(sched)


class LoopTab(ModeTab):
    title = "Loop simple"

    def __init__(self, main: "MainWindow") -> None:
        super().__init__(main)
        self.high = level_spin(80)
        self.hint = QLabel()
        self.timing = TimingWidget(5.0)
        self.cycles = QSpinBox()
        self.cycles.setRange(1, 2_000_000_000)
        self.cycles.setValue(100)
        self.high.valueChanged.connect(lambda v: self.hint.setText(level_hint(v)))
        self.hint.setText(level_hint(self.high.value()))
        w = QWidget()
        f = QFormLayout(w)
        f.addRow(QLabel("Alterna entre el <b>nivel base</b> (panel izquierdo) y un nivel alto, "
                        "como onda cuadrada, la cantidad de ciclos indicada."))
        f.addRow("Nivel alto (0–255):", self.high)
        f.addRow("", self.hint)
        f.addRow(self.timing)
        f.addRow("Ciclos:", self.cycles)
        self.finish_layout(w)
        self.watch(self.high, self.timing, self.cycles)

    def build(self):
        th, tl = self.timing.times_us()
        return core.build_loop(self.main.base(), self.high.value(), th, tl, self.cycles.value())

    def describe(self):
        return {"modo": "Loop simple"}


class LevelSweepTab(ModeTab):
    title = "Barrido de niveles"
    has_table = True

    def __init__(self, main: "MainWindow") -> None:
        super().__init__(main)
        self.l_from = level_spin(65)
        self.l_to = level_spin(120)
        self.step = QSpinBox()
        self.step.setRange(1, 255)
        self.step.setValue(5)
        self.dir = DirectionButton()
        self.timing = TimingWidget(5.0)
        self.cycles = QSpinBox()
        self.cycles.setRange(1, 2_000_000_000)
        self.cycles.setValue(100)
        self.pause = dspin(0.0, 3600.0, 2.0, 3, " s")
        w = QWidget()
        f = QFormLayout(w)
        f.addRow(QLabel("Cada nivel de la serie alterna con el <b>nivel base</b> la cantidad de "
                        "ciclos indicada; entre niveles el LED queda en la base durante la pausa."))
        rng = QHBoxLayout()
        rng.addWidget(QLabel("Desde"))
        rng.addWidget(self.l_from)
        rng.addWidget(QLabel("hasta"))
        rng.addWidget(self.l_to)
        rng.addWidget(QLabel("paso"))
        rng.addWidget(self.step)
        f.addRow("Niveles altos:", rng)
        f.addRow("Dirección:", self.dir)
        f.addRow(self.timing)
        f.addRow("Ciclos por nivel:", self.cycles)
        f.addRow("Pausa entre niveles (en base):", self.pause)
        self.finish_layout(w)
        self.watch(self.l_from, self.l_to, self.step, self.dir, self.timing, self.cycles,
                   self.pause)

    def build(self):
        th, tl = self.timing.times_us()
        lv = core.level_series(self.l_from.value(), self.l_to.value(), self.step.value(),
                               self.dir.descending)
        return core.build_level_sweep(self.main.base(), lv, th, tl, self.cycles.value(),
                                      int(round(self.pause.value() * 1e6)))

    def describe(self):
        return {"modo": "Barrido de niveles",
                "niveles": f"{self.l_from.value()}..{self.l_to.value()} paso {self.step.value()} "
                           f"({'desc' if self.dir.descending else 'asc'})",
                "pausa_entre_niveles_s": self.pause.value()}


class ContinuousTab(ModeTab):
    title = "Continuo"

    def __init__(self, main: "MainWindow") -> None:
        super().__init__(main)
        self.high = level_spin(80)
        self.hint = QLabel(level_hint(80))
        self.high.valueChanged.connect(lambda v: self.hint.setText(level_hint(v)))
        self.timing = TimingWidget(5.0)
        w = QWidget()
        f = QFormLayout(w)
        f.addRow(QLabel("Alterna entre el <b>nivel base</b> y el nivel alto <b>indefinidamente</b>"
                        " hasta presionar STOP. El panel izquierdo muestra tiempo y ciclos."))
        f.addRow("Nivel alto (0–255):", self.high)
        f.addRow("", self.hint)
        f.addRow(self.timing)
        self.finish_layout(w)
        self.watch(self.high, self.timing)

    def build(self):
        th, tl = self.timing.times_us()
        return core.build_continuous(self.main.base(), self.high.value(), th, tl)

    def describe(self):
        return {"modo": "Continuo"}


class FreqSweepTab(ModeTab):
    title = "Barrido de frecuencia"
    has_table = True

    def __init__(self, main: "MainWindow") -> None:
        super().__init__(main)
        self.high = level_spin(80)
        self.f_from = dspin(0.001, 100_000.0, 5.0, 3, " Hz")
        self.f_to = dspin(0.001, 100_000.0, 100.0, 3, " Hz")
        self.f_step = dspin(0.001, 100_000.0, 5.0, 3, " Hz")
        self.dir = DirectionButton()
        self.duty = dspin(1.0, 99.0, 50.0, 1, " %")
        self.cycles = QSpinBox()
        self.cycles.setRange(1, 2_000_000_000)
        self.cycles.setValue(100)
        self.pause = dspin(0.0, 3600.0, 2.0, 3, " s")
        w = QWidget()
        f = QFormLayout(w)
        f.addRow(QLabel("Nivel alto fijo; la frecuencia recorre la serie. Entre frecuencias el LED "
                        "queda en la base durante la pausa."))
        f.addRow("Nivel alto (0–255):", self.high)
        rng = QHBoxLayout()
        rng.addWidget(QLabel("Desde"))
        rng.addWidget(self.f_from)
        rng.addWidget(QLabel("hasta"))
        rng.addWidget(self.f_to)
        rng.addWidget(QLabel("paso"))
        rng.addWidget(self.f_step)
        f.addRow("Frecuencias:", rng)
        f.addRow("Dirección:", self.dir)
        f.addRow("Ciclo de trabajo (tiempo en alto):", self.duty)
        f.addRow("Ciclos por frecuencia:", self.cycles)
        f.addRow("Pausa entre frecuencias (en base):", self.pause)
        self.finish_layout(w)
        self.watch(self.high, self.f_from, self.f_to, self.f_step, self.dir, self.duty,
                   self.cycles, self.pause)

    def build(self):
        fr = core.freq_series(self.f_from.value(), self.f_to.value(), self.f_step.value(),
                              self.dir.descending)
        if len(fr) > core.MAX_SEGMENTS:
            raise ValueError(f"La serie tiene {len(fr)} frecuencias; el máximo es "
                             f"{core.MAX_SEGMENTS}. Aumente el paso.")
        return core.build_freq_sweep(self.main.base(), self.high.value(), fr,
                                     self.duty.value() / 100.0, self.cycles.value(),
                                     int(round(self.pause.value() * 1e6)))

    def describe(self):
        return {"modo": "Barrido de frecuencia",
                "frecuencias": f"{self.f_from.value()}..{self.f_to.value()} Hz paso "
                               f"{self.f_step.value()} ({'desc' if self.dir.descending else 'asc'})",
                "ciclo_trabajo_pct": self.duty.value(),
                "pausa_entre_frecuencias_s": self.pause.value()}


class SetTab(QWidget):
    """Nivel fijo manual: para ver saturación en la cámara y medir mW con el fotodetector."""

    def __init__(self, main: "MainWindow") -> None:
        super().__init__()
        self.main = main
        self.spin = level_spin(100)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(core.DAC_MIN, core.DAC_MAX)
        self.slider.setValue(100)
        self.hint = QLabel(level_hint(100))
        self.live = QCheckBox("Aplicar automáticamente al mover")
        self.btn = QPushButton("Aplicar nivel (SET)")
        self.btn.setMinimumHeight(38)
        self.btn.setStyleSheet("font-weight:bold; font-size:14px;")
        self.spin.valueChanged.connect(self.slider.setValue)
        self.slider.valueChanged.connect(self.spin.setValue)
        self.spin.valueChanged.connect(self._changed)
        self.btn.clicked.connect(lambda: self.apply())
        self._throttle = QTimer(self)
        self._throttle.setSingleShot(True)
        self._throttle.setInterval(40)
        self._throttle.timeout.connect(self.apply)

        lay = QVBoxLayout(self)
        lay.addWidget(QLabel("Fija un nivel <b>continuo</b> en el LED (sin oscilar). Úselo para "
                             "verificar en la cámara si el nivel se ve bien o satura, y para medir "
                             "la potencia (mW) con el fotodetector.<br>Para apagar el LED aplique 0."))
        row = QHBoxLayout()
        row.addWidget(QLabel("Nivel (0–255):"))
        row.addWidget(self.spin)
        row.addWidget(self.slider, 1)
        lay.addLayout(row)
        self.hint.setWordWrap(True)
        lay.addWidget(self.hint)
        lay.addWidget(self.live)
        lay.addWidget(self._build_diag())
        lay.addWidget(self.btn)

    # ---------------- diagnóstico con el ADC de la ESP32 (comando MEAS, firmware 2.1) ----------
    DIAG_LEVELS = [0, 25, 50, 75, 100, 125, 150, 175, 200, 225, 255]

    def _build_diag(self) -> QGroupBox:
        g = QGroupBox("Diagnóstico del circuito (opcional, firmware 2.1)")
        v = QVBoxLayout(g)
        info = QLabel(
            "Con dos cables puente: <b>D34 → base</b> del transistor y <b>D35 → emisor</b> "
            "(pata de las resistencias). La ESP32 mide esos voltajes y calcula la corriente del LED. "
            "Sin los puentes, las lecturas de D34/D35 no significan nada.")
        info.setWordWrap(True)
        v.addWidget(info)
        row = QHBoxLayout()
        self.btn_meas = QPushButton("Medir nivel actual")
        self.btn_meas.clicked.connect(lambda: self.main.send(["MEAS"]))
        self.btn_ramp = QPushButton("Prueba automática 0 → 255")
        self.btn_ramp.clicked.connect(lambda: self.start_ramp())
        row.addWidget(self.btn_meas)
        row.addWidget(self.btn_ramp)
        v.addLayout(row)
        self.diag_table = QTableWidget(0, 6)
        self.diag_table.setHorizontalHeaderLabels(
            ["Nivel", "Base esperada", "Base medida (D34)", "Emisor (D35)", "I_LED ≈", "Lectura"])
        self.diag_table.verticalHeader().setVisible(False)
        self.diag_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.diag_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.diag_table.horizontalHeader().setStretchLastSection(True)
        v.addWidget(self.diag_table, 1)
        self._ramp_idx = -1
        self._ramp_timer = QTimer(self)
        self._ramp_timer.setSingleShot(True)
        self._ramp_timer.timeout.connect(self._ramp_measure)
        return g

    def start_ramp(self) -> None:
        if not self.main.is_connected or getattr(self.main, "_running_ui", False):
            return
        self.diag_table.setRowCount(0)
        self._ramp_idx = 0
        self.btn_ramp.setEnabled(False)
        self._ramp_step()

    def _ramp_step(self) -> None:
        if self._ramp_idx < 0 or self._ramp_idx >= len(self.DIAG_LEVELS):
            self._ramp_idx = -1
            self.btn_ramp.setEnabled(True)
            self.main.send(["SET 0"])
            return
        lv = self.DIAG_LEVELS[self._ramp_idx]
        self.spin.blockSignals(True)
        self.spin.setValue(lv)
        self.slider.setValue(lv)
        self.spin.blockSignals(False)
        self.main.send([f"SET {lv}"])
        self._ramp_timer.start(1200)      # deja ver el LED y estabilizar antes de medir

    def abort_ramp(self) -> None:
        self._ramp_timer.stop()
        self._ramp_idx = -1
        self.btn_ramp.setEnabled(True)

    def _ramp_measure(self) -> None:
        self.main.send(["MEAS"])

    def on_meas(self, out: int, d34: int, d35: int) -> None:
        exp = core.dac_to_volts(out) * 1000
        i_ma = d35 / core.R_E
        notes = []
        if abs(d34 - exp) > 300:
            notes.append("base NO sigue al nivel")
        if out == 0 and d35 > 50:
            notes.append("pasa corriente con nivel 0 → revisar circuito")
        if out > 60 and d34 - d35 > 1000:
            notes.append("Vbe muy alto: el emisor no sigue a la base")
        if not notes:
            notes.append("OK")
        r = self.diag_table.rowCount()
        self.diag_table.insertRow(r)
        for c, val in enumerate([str(out), f"{exp:.0f} mV", f"{d34} mV", f"{d35} mV",
                                 f"{i_ma:.2f} mA", " · ".join(notes)]):
            it = QTableWidgetItem(val)
            it.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.diag_table.setItem(r, c, it)
        self.diag_table.scrollToBottom()
        if self._ramp_idx >= 0:
            self._ramp_idx += 1
            QTimer.singleShot(300, self._ramp_step)

    def _changed(self, v: int) -> None:
        self.hint.setText(level_hint(v))
        if self.live.isChecked():
            self._throttle.start()

    def apply(self) -> None:
        if getattr(self.main, "_running_ui", False):
            return   # durante una secuencia la ESP32 solo acepta STOP
        self.main.send([f"SET {self.spin.value()}"])


# =========================================================================================
# Ventana principal
# =========================================================================================
class MainWindow(QMainWindow):
    reqOpen = pyqtSignal(str)
    reqClose = pyqtSignal()
    reqSend = pyqtSignal(list)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Generador LED · Caracterización BIAS EvCam (ESP32 v2)")
        self.settings = QSettings("LED_C", "LED_DVS_GUI")
        self.is_connected = False
        self.is_v2 = False
        self.tracker = core.RunTracker()
        self.run_tab: Optional[ModeTab] = None
        self.run_sched: List[core.ScheduledSegment] = []
        self.run_total: Optional[int] = None
        self.run_t0: Optional[float] = None
        self.run_row = -2
        self.fw_base = 0          # nivel base que tiene la ESP32 (al que vuelve STOP)
        self.expected_acks = deque()   # (SET|BASE, valor) enviados y aún no confirmados
        self.comm_errors = 0
        self._manual_pending = False
        self._meas_pending = False

        # ---------- hilo serie ----------
        self.thread = QThread(self)
        self.worker = SerialWorker()
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.start)
        self.reqOpen.connect(self.worker.open_port)
        self.reqClose.connect(self.worker.close_port)
        self.reqSend.connect(self.worker.send_lines)
        self.worker.lineReceived.connect(self.on_line)
        self.worker.connected.connect(self.on_connected)
        self.worker.disconnected.connect(self.on_disconnected)
        self.thread.start()

        self._build_ui()
        self.refresh_ports()
        self._set_running(False)
        self._set_connected(False)

        self.ui_timer = QTimer(self)
        self.ui_timer.setInterval(100)
        self.ui_timer.timeout.connect(self._tick)

        self.sc_stop = QShortcut(QKeySequence(Qt.Key.Key_Escape), self)
        self.sc_stop.activated.connect(self.stop)
        for t in self.mode_tabs:
            t.refresh()

    # ------------------------------------------------------------------ UI
    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # conexión
        con = QHBoxLayout()
        self.cmb_port = QComboBox()
        self.cmb_port.setMinimumWidth(260)
        self.btn_refresh = QPushButton("⟳")
        self.btn_refresh.setToolTip("Actualizar lista de puertos")
        self.btn_refresh.setFixedWidth(36)
        self.btn_refresh.clicked.connect(lambda: self.refresh_ports())
        self.btn_connect = QPushButton("Conectar")
        self.btn_connect.clicked.connect(lambda: self.toggle_connection())
        self.lbl_conn = QLabel()
        con.addWidget(QLabel("Puerto:"))
        con.addWidget(self.cmb_port)
        con.addWidget(self.btn_refresh)
        con.addWidget(self.btn_connect)
        con.addWidget(self.lbl_conn, 1)
        root.addLayout(con)

        split = QSplitter(Qt.Orientation.Horizontal)
        root.addWidget(split, 1)

        # ---------- panel izquierdo ----------
        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 6, 0)

        g_base = QGroupBox("Nivel base")
        fb = QFormLayout(g_base)
        self.spin_base = level_spin(int(self.settings.value("base", 60)))
        self.lbl_base_hint = QLabel(level_hint(self.spin_base.value()))
        self.lbl_base_hint.setWordWrap(True)
        self.btn_base = QPushButton("Aplicar base ahora")
        self.btn_base.clicked.connect(lambda: self.send([f"BASE {self.base()}"]))
        fb.addRow("Base (0–255):", self.spin_base)
        fb.addRow(self.lbl_base_hint)
        fb.addRow(self.btn_base)
        lv.addWidget(g_base)

        g_pre = QGroupBox("Inicio")
        fp = QFormLayout(g_pre)
        self.spin_pre = dspin(0.0, 3600.0, float(self.settings.value("pre_s", 3.0)), 3, " s")
        fp.addRow("Pausa inicial en base:", self.spin_pre)
        lv.addWidget(g_pre)

        g_log = QGroupBox("Registro")
        fl = QVBoxLayout(g_log)
        self.chk_log = QCheckBox("Grabar registro CSV")
        self.chk_log.setChecked(False)
        self.log_dir = str(self.settings.value("log_dir", DEFAULT_LOG_DIR))
        self.lbl_logdir = QLabel()
        self.lbl_logdir.setWordWrap(True)
        self.lbl_logdir.setStyleSheet("color:#7f8eaa")
        btn_dir = QPushButton("Carpeta…")
        btn_dir.clicked.connect(lambda: self.choose_log_dir())
        fl.addWidget(self.chk_log)
        fl.addWidget(self.lbl_logdir)
        fl.addWidget(btn_dir)
        self._upd_logdir()
        lv.addWidget(g_log)

        g_st = QGroupBox("Estado")
        fs = QFormLayout(g_st)
        self.lbl_out = QLabel("—")
        fo = QFont()
        fo.setPointSize(fo.pointSize() + 4)
        fo.setBold(True)
        self.lbl_out.setFont(fo)
        self.lbl_state = QLabel("Desconectado")
        self.lbl_state.setWordWrap(True)
        self.lbl_seg = QLabel("—")
        self.lbl_seg.setWordWrap(True)
        self.lbl_elapsed = QLabel("—")
        self.lbl_remain = QLabel("—")
        self.lbl_cycles = QLabel("—")
        self.progress = QProgressBar()
        self.progress.setRange(0, 1000)
        self.progress.setTextVisible(False)
        fs.addRow("Salida LED:", self.lbl_out)
        fs.addRow("Estado:", self.lbl_state)
        fs.addRow("Tramo:", self.lbl_seg)
        fs.addRow("Transcurrido:", self.lbl_elapsed)
        fs.addRow("Restante:", self.lbl_remain)
        fs.addRow("Ciclos:", self.lbl_cycles)
        fs.addRow(self.progress)
        lv.addWidget(g_st)

        self.btn_stop = QPushButton("■  STOP  (Esc)\nvuelve al nivel base")
        self.btn_stop.setMinimumHeight(64)
        self.btn_stop.setStyleSheet(
            "QPushButton{background:#8e1f24;color:#f0dede;font-weight:bold;font-size:16px;"
            "border:1px solid #b0343a;border-radius:6px;}"
            "QPushButton:hover{background:#a3262c;}"
            "QPushButton:disabled{background:#2a1f2b;color:#6b5a66;border-color:#3a2a36;}")
        self.btn_stop.clicked.connect(lambda: self.stop())
        lv.addWidget(self.btn_stop)
        lv.addStretch(1)
        split.addWidget(left)

        # ---------- pestañas ----------
        right = QSplitter(Qt.Orientation.Vertical)
        self.tabs = QTabWidget()
        self.set_tab = SetTab(self)
        self.tabs.addTab(self.set_tab, "Nivel fijo (SET)")
        self.mode_tabs: List[ModeTab] = [LoopTab(self), LevelSweepTab(self), ContinuousTab(self),
                                         FreqSweepTab(self)]
        for t in self.mode_tabs:
            self.tabs.addTab(t, t.title)
            t.startRequested.connect(self.start_tab)
        right.addWidget(self.tabs)

        g_con = QGroupBox("Mensajes de la ESP32")
        cl = QVBoxLayout(g_con)
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(400)
        self.console.setFont(QFont("Monospace", 9))
        row = QHBoxLayout()
        self.cmd_edit = QLineEdit()
        self.cmd_edit.setPlaceholderText("Comando manual (ej: PING, INFO, SET 120)…")
        self.cmd_edit.returnPressed.connect(self._manual_cmd)
        row.addWidget(self.cmd_edit)
        cl.addWidget(self.console)
        cl.addLayout(row)
        right.addWidget(g_con)
        right.setStretchFactor(0, 4)
        right.setStretchFactor(1, 1)
        split.addWidget(right)
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        split.setSizes([320, 900])

        # recalcular estimaciones al cambiar base o pausa inicial
        self.spin_base.valueChanged.connect(self._globals_changed)
        self.spin_pre.valueChanged.connect(self._globals_changed)
        self.resize(1280, 860)

    def _globals_changed(self) -> None:
        self.lbl_base_hint.setText(level_hint(self.spin_base.value()))
        for t in self.mode_tabs:
            t.paramsChanged.emit()

    def _out_base(self) -> None:
        self.lbl_out.setText(f"{self.fw_base}  (base)")

    def _upd_logdir(self) -> None:
        self.lbl_logdir.setText(f"Carpeta: {self.log_dir}")

    def choose_log_dir(self) -> None:
        d = QFileDialog.getExistingDirectory(self, "Carpeta de registros", self.log_dir)
        if d:
            self.log_dir = d
            self._upd_logdir()

    # ------------------------------------------------------------------ accesores
    def base(self) -> int:
        return self.spin_base.value()

    def pre_us(self) -> int:
        return int(round(self.spin_pre.value() * 1e6))

    # ------------------------------------------------------------------ conexión
    def refresh_ports(self) -> None:
        cur = self.cmb_port.currentText() or str(self.settings.value("port", ""))
        self.cmb_port.clear()
        ports = []
        if serial is not None:
            for p in serial.tools.list_ports.comports():
                ports.append(p.device)
                self.cmb_port.addItem(p.device)
                self.cmb_port.setItemData(self.cmb_port.count() - 1,
                                          f"{p.description} {p.hwid}",
                                          Qt.ItemDataRole.ToolTipRole)
        self.cmb_port.addItem(SIM_PORT)
        idx = self.cmb_port.findText(cur)
        if idx >= 0:
            self.cmb_port.setCurrentIndex(idx)
        else:
            # preferir puertos USB típicos de la DevKit
            for k, p in enumerate(ports):
                if "USB" in p or "ACM" in p:
                    self.cmb_port.setCurrentIndex(k)
                    break

    def toggle_connection(self) -> None:
        if self.is_connected:
            self.reqClose.emit()
        else:
            port = self.cmb_port.currentText()
            if not port:
                return
            self.btn_connect.setEnabled(False)
            self.lbl_conn.setText(f"Conectando a {port} (reiniciando la placa)…")
            self.reqOpen.emit(port)

    @pyqtSlot(str, str, bool)
    def on_connected(self, port: str, info: str, is_v2: bool) -> None:
        self.is_v2 = is_v2
        self.settings.setValue("port", port)
        self._set_connected(True)
        self._log(f"[conectado a {port}] {info}")
        m = re.search(r"base=(\d+)", info)
        self.fw_base = int(m.group(1)) if m else 0
        self.expected_acks.clear()
        self.comm_errors = 0
        self.lbl_state.setStyleSheet("")
        if is_v2:
            self.lbl_conn.setText(f"<span style='color:#5fbf7a'>● Conectado</span> · {port} · {info}")
            self.lbl_out.setText("0")
            self.lbl_state.setText("Reposo")
        elif info == "(sin INFO)":
            # Respondió PING pero se perdieron 3 respuestas a INFO: no es el firmware, es la línea serie
            self.lbl_conn.setText(f"<span style='color:#e0a050'>● Conectado, comunicación "
                                  f"inestable</span> · {port}")
            QMessageBox.warning(
                self, "Comunicación inestable",
                "La placa responde a PING, pero se perdieron las respuestas a INFO.\n\n"
                "Eso indica pérdida de datos en el puerto serie, no un firmware antiguo:\n"
                "  • cierre el Monitor Serie del Arduino IDE (y cualquier otro programa\n"
                "    que use el puerto),\n"
                "  • use un cable USB corto y de datos, conectado directo al PC.\n\n"
                "Después presione Desconectar y Conectar de nuevo.\n"
                "(También puede escribir INFO en la línea de comandos: si responde\n"
                "LED_DVS_v2, se habilitan los loops y barridos.)")
        else:
            self.lbl_conn.setText(f"<span style='color:#e0a050'>● Conectado, firmware NO v2</span>"
                                  f" · {port} · {info}")
            QMessageBox.warning(self, "Firmware antiguo",
                                "La placa responde, pero no tiene el firmware v2.\n"
                                "Cargue firmware_esp32_control_led_dvs_v2.ino para usar los "
                                "loops y barridos. El modo SET puede funcionar igual.")

    @pyqtSlot(str)
    def on_disconnected(self, reason: str) -> None:
        was_running = self.tracker.busy
        self.tracker.reset()
        self.set_tab.abort_ramp()
        self._meas_pending = False
        self._set_connected(False)
        self._set_running(False)
        self.lbl_conn.setText("<span style='color:#6f7f9c'>● Desconectado</span>"
                              + (f" · {reason}" if reason else ""))
        self.lbl_state.setText("Desconectado")
        self.lbl_out.setText("—")
        if reason:
            self._log(f"[!] {reason}")
            if was_running or "conectar" in reason:
                QMessageBox.warning(self, "Conexión", reason)

    def _set_connected(self, on: bool) -> None:
        self.is_connected = on
        self.btn_connect.setEnabled(True)
        self.btn_connect.setText("Desconectar" if on else "Conectar")
        self.cmb_port.setEnabled(not on)
        self.btn_refresh.setEnabled(not on)
        self.btn_stop.setEnabled(on)
        self.cmd_edit.setEnabled(on)
        self._update_enables()

    def _set_running(self, on: bool) -> None:
        self._running_ui = on
        self._update_enables()

    def _update_enables(self) -> None:
        running = getattr(self, "_running_ui", False)
        ok = self.is_connected and not running
        for t in self.mode_tabs:
            t.btn_start.setEnabled(ok and self.is_v2)
        self.set_tab.btn.setEnabled(ok)
        self.set_tab.btn_meas.setEnabled(ok)
        self.set_tab.btn_ramp.setEnabled(ok and self.set_tab._ramp_idx < 0)
        self.btn_base.setEnabled(ok)
        self.spin_base.setEnabled(not running)
        self.spin_pre.setEnabled(not running)
        self.chk_log.setEnabled(not running)

    # ------------------------------------------------------------------ envío / recepción
    def send(self, lines: List[str]) -> None:
        if not self.is_connected:
            self.lbl_state.setText("No conectado")
            return
        for ln in lines:
            if not ln.startswith("Q "):
                self._log(f"> {ln}")
            parts = ln.split()
            if ln.strip().upper() == "MEAS":
                self._meas_pending = True
            if len(parts) == 2 and parts[0].upper() in ("SET", "BASE"):
                self.expected_acks.append((parts[0].upper(), parts[1]))
        self.reqSend.emit(list(lines))

    def _check_ack(self, kind: str, value: str) -> None:
        """Compara el valor que confirmó la ESP32 con el que se envió.
        Una diferencia indica errores en la comunicación serie (cable, otro programa en el puerto)."""
        if (kind, value) in self.expected_acks:
            while self.expected_acks:
                if self.expected_acks.popleft() == (kind, value):
                    break
            return
        self.comm_errors += 1
        sent = self.expected_acks[0][1] if self.expected_acks else "?"
        self.expected_acks.clear()
        self._comm_warning(f"Se envió {kind} {sent} pero la ESP32 aplicó {kind} {value}")

    def _comm_warning(self, detail: str) -> None:
        msg = (f"⚠ Error de comunicación ({self.comm_errors}): {detail}. "
               "Revise el cable USB y que ningún otro programa use el puerto.")
        self._log(msg)
        self.lbl_state.setText(msg)
        self.lbl_state.setStyleSheet("color:#ff7a7a")

    def _manual_cmd(self) -> None:
        txt = self.cmd_edit.text().strip()
        if txt:
            self._manual_pending = True   # un error aquí puede ser un comando mal escrito
            self.send([txt])
            self.cmd_edit.clear()

    def _log(self, s: str) -> None:
        self.console.appendPlainText(s)

    @pyqtSlot(str)
    def on_line(self, line: str) -> None:
        self._log(line)
        if line.startswith("INFO") and core.FW_ID in line and not self.is_v2 and self.is_connected:
            # respuesta a INFO que llegó tarde (o INFO manual): habilitar el firmware v2
            self.is_v2 = True
            self._update_enables()
            self.lbl_conn.setText(f"<span style='color:#5fbf7a'>● Conectado</span> · {line}")
        if line.startswith("MEAS "):
            self._meas_pending = False
            m = re.match(r"MEAS out=(\d+) d34=(\d+) d35=(\d+)", line)
            if m:
                self.set_tab.on_meas(int(m.group(1)), int(m.group(2)), int(m.group(3)))
        if line.startswith("ACK SET") or line.startswith("ACK BASE"):
            v = line.split()[-1]
            kind = "BASE" if "BASE" in line else "SET"
            self.lbl_state.setStyleSheet("")
            self._check_ack(kind, v)
            if "BASE" in line:
                self.fw_base = int(v)
            self.lbl_out.setText(v + ("  (base)" if "BASE" in line else "  (fijo)"))
            self.lbl_state.setText("Reposo")
        elif line == "ACK STOP" and not self.tracker.busy:
            self._out_base()
            if not self.lbl_state.text().startswith("Detenido"):
                self.lbl_state.setText("Reposo (STOP → base)")
        elif line == "ERR UNKNOWN_COMMAND" and self._meas_pending:
            self._meas_pending = False
            self.set_tab.abort_ramp()
            self.lbl_state.setText("La placa no reconoce MEAS: cargue el firmware 2.1 para el diagnóstico")
        elif line.startswith("ERR") and not self.tracker.busy:
            if line in ("ERR UNKNOWN_COMMAND", "ERR VAL_OUT_OF_RANGE", "ERR PARAM_MISSING") \
                    and not self._manual_pending:
                self.comm_errors += 1
                self.expected_acks.clear()
                self._comm_warning(f"la ESP32 recibió un comando dañado ({line})")
            else:
                self.lbl_state.setText(f"Error: {line}")
        self._manual_pending = False

        if self.tracker.busy:
            for note in self.tracker.feed(line):
                self._on_note(note)

    def _on_note(self, note: tuple) -> None:
        kind = note[0]
        if kind == "started":
            self.run_t0 = time.perf_counter()
            self.ui_timer.start()
            self.lbl_state.setText("Pausa inicial en base")
        elif kind == "done":
            self._end_run("Completado ✔")
        elif kind == "aborted":
            self._end_run(f"Detenido con STOP (tramo {note[1] + 1}) → base")
        elif kind == "error":
            self._end_run("Error")
            QMessageBox.warning(self, "Error en la secuencia", note[1])
        elif kind == "need_stop":
            self.send(["STOP"])

    # ------------------------------------------------------------------ ejecución
    def start_tab(self, tab: ModeTab) -> None:
        if not self.is_connected or self.tracker.busy:
            return
        if not self.is_v2:
            QMessageBox.warning(self, "Firmware", "Se necesita el firmware v2 en la ESP32.")
            return
        pre = self.pre_us()
        try:
            segs = tab.build()
        except ValueError as e:
            QMessageBox.warning(self, "Parámetros", str(e))
            return
        errs = core.validate(segs, pre)
        if errs:
            QMessageBox.warning(self, "Parámetros", "\n".join(errs))
            return

        logger = None
        if self.chk_log.isChecked():
            logger = core.RunLogger()
            meta = {"base": self.base(), "pausa_inicial_s": self.spin_pre.value(),
                    "tramos": len(segs),
                    "duracion_estimada": core.fmt_duration(core.sequence_duration_us(segs, pre))}
            meta.update(tab.describe())
            try:
                path = logger.open(self.log_dir, datetime.now(), meta)
                self._log(f"[registro] {path}")
            except OSError as e:
                QMessageBox.warning(self, "Registro", f"No se pudo crear el archivo:\n{e}")
                return

        self.run_tab = tab
        self.run_sched = core.schedule(segs, pre)
        self.run_total = core.sequence_duration_us(segs, pre)
        self.run_t0 = None
        self.run_row = -2
        cmds = self.tracker.begin(segs, pre, logger)
        self.fw_base = segs[0].base
        self._set_running(True)
        self.lbl_state.setText(f"Cargando {len(segs)} tramo(s) en la ESP32…")
        self.progress.setValue(0)
        self.send(cmds)

    def stop(self) -> None:
        if self.is_connected:
            self.send(["STOP"])

    def _end_run(self, msg: str) -> None:
        self.ui_timer.stop()
        self._tick()
        self.tracker.reset()
        self._set_running(False)
        self.lbl_state.setText(msg)
        self._out_base()
        self.progress.setRange(0, 1000)
        if self.run_tab and self.run_tab.table is not None:
            self.run_tab.table.mark(-1)
        if msg.startswith("Completado"):
            self.progress.setValue(1000)

    def _tick(self) -> None:
        if self.run_t0 is None or not self.run_sched:
            return
        el = (time.perf_counter() - self.run_t0) * 1e6
        self.lbl_elapsed.setText(core.fmt_duration(el))
        if self.run_total is None:
            self.lbl_remain.setText("∞ (hasta STOP)")
            self.progress.setRange(0, 0)   # barra "ocupada"
        else:
            self.progress.setRange(0, 1000)
            self.lbl_remain.setText(core.fmt_duration(max(0.0, self.run_total - el)))
            self.progress.setValue(int(1000 * min(1.0, el / max(1, self.run_total))))
        if not self.tracker.busy:
            return
        phase, idx = core.segment_at(self.run_sched, el)
        n = len(self.run_sched)
        if phase == "pre":
            self.lbl_state.setText("Pausa inicial en base")
            self.lbl_seg.setText(f"— (comienza en {core.fmt_duration(self.run_sched[0].start_us - el)})")
            self._out_base()
            self.lbl_cycles.setText("—")
        elif phase == "seg":
            s = self.run_sched[idx]
            g = s.segment
            self.lbl_state.setText("Oscilando")
            self.lbl_seg.setText(f"{idx + 1}/{n}: {g.base} ↔ {g.high} @ {core.fmt_freq(g.freq_hz)}")
            self.lbl_out.setText(f"{g.base} ↔ {g.high}")
            done = int((el - s.start_us) // g.period_us)
            tot = "∞" if g.cycles == 0 else str(g.cycles)
            self.lbl_cycles.setText(f"{min(done, g.cycles) if g.cycles else done} / {tot}")
        elif phase == "pause":
            s = self.run_sched[idx]
            self.lbl_state.setText("Pausa entre niveles (base)")
            self.lbl_seg.setText(f"después de {idx + 1}/{n} · siguiente en "
                                 f"{core.fmt_duration(s.pause_end_us - el)}")
            self.lbl_out.setText(f"{s.segment.base}  (base)")
        else:
            self.lbl_state.setText("Terminando…")
        row = idx if phase in ("seg",) else -1
        if self.run_tab and self.run_tab.table is not None and row != self.run_row:
            self.run_row = row
            self.run_tab.table.mark(row)

    # ------------------------------------------------------------------ cierre
    def closeEvent(self, ev) -> None:
        self.settings.setValue("base", self.spin_base.value())
        self.settings.setValue("pre_s", self.spin_pre.value())
        self.settings.setValue("log_dir", self.log_dir)
        QMetaObject.invokeMethod(self.worker, "shutdown",
                                 Qt.ConnectionType.BlockingQueuedConnection)
        self.tracker.reset()
        self.thread.quit()
        self.thread.wait(2000)
        super().closeEvent(ev)


# =========================================================================================
# Tema oscuro "medianoche" (para trabajar con la luz del laboratorio apagada)
# =========================================================================================
MIDNIGHT = {
    "window": "#0d1526",      # fondo general
    "base": "#091020",        # campos, tablas, consola
    "alt": "#101b30",         # filas alternas
    "button": "#16233d",
    "text": "#b9c6dc",        # texto atenuado (no blanco puro, para no encandilar)
    "bright": "#d5dfef",
    "dim": "#4f5d78",         # texto deshabilitado
    "border": "#22324f",
    "highlight": "#2a4677",
}


def apply_midnight_theme(app: QApplication) -> None:
    c = MIDNIGHT
    pal = QPalette()
    R, G = QPalette.ColorRole, QPalette.ColorGroup
    pal.setColor(R.Window, QColor(c["window"]))
    pal.setColor(R.WindowText, QColor(c["text"]))
    pal.setColor(R.Base, QColor(c["base"]))
    pal.setColor(R.AlternateBase, QColor(c["alt"]))
    pal.setColor(R.Text, QColor(c["text"]))
    pal.setColor(R.Button, QColor(c["button"]))
    pal.setColor(R.ButtonText, QColor(c["text"]))
    pal.setColor(R.BrightText, QColor("#ff7a7a"))
    pal.setColor(R.Highlight, QColor(c["highlight"]))
    pal.setColor(R.HighlightedText, QColor(c["bright"]))
    pal.setColor(R.ToolTipBase, QColor(c["button"]))
    pal.setColor(R.ToolTipText, QColor(c["text"]))
    pal.setColor(R.PlaceholderText, QColor("#5f6f8c"))
    pal.setColor(R.Link, QColor("#7fa6e0"))
    pal.setColor(R.Light, QColor("#22324f"))
    pal.setColor(R.Midlight, QColor("#1a2843"))
    pal.setColor(R.Mid, QColor("#14203a"))
    pal.setColor(R.Dark, QColor("#070c18"))
    pal.setColor(R.Shadow, QColor("#03060d"))
    for role in (R.WindowText, R.Text, R.ButtonText):
        pal.setColor(G.Disabled, role, QColor(c["dim"]))
    pal.setColor(G.Disabled, R.Button, QColor("#111a2d"))
    pal.setColor(G.Disabled, R.Base, QColor("#0b1222"))
    app.setPalette(pal)
    app.setStyleSheet(f"""
        QGroupBox {{ border:1px solid {c['border']}; border-radius:6px; margin-top:10px;
                     padding-top:6px; }}
        QGroupBox::title {{ subcontrol-origin: margin; left:8px; padding:0 4px;
                            color:{c['bright']}; }}
        QTabWidget::pane {{ border:1px solid {c['border']}; border-radius:4px; }}
        QTabBar::tab {{ background:{c['window']}; color:{c['text']}; padding:6px 14px;
                        border:1px solid {c['border']}; border-bottom:none;
                        border-top-left-radius:4px; border-top-right-radius:4px; }}
        QTabBar::tab:selected {{ background:{c['button']}; color:{c['bright']}; }}
        QHeaderView::section {{ background:{c['button']}; color:{c['text']};
                                border:1px solid {c['border']}; padding:3px; }}
        QTableWidget {{ gridline-color:{c['border']}; }}
        QProgressBar {{ border:1px solid {c['border']}; border-radius:3px;
                        background:{c['base']}; }}
        QProgressBar::chunk {{ background:#2f5a96; }}
        QToolTip {{ background:{c['button']}; color:{c['text']};
                    border:1px solid {c['border']}; }}
    """)


def main() -> None:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    apply_midnight_theme(app)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
