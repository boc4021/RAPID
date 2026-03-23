"""
gui.py - Real-time UART polar-coordinate visualizer.

Launches RAPID.exe as a child process, parses its stdout for ACK /
CRC / FPGA log lines, and plots acknowledged points on an XY scatter chart
(converted from polar coordinates).

Usage:
    python gui.py
"""

import sys
import signal
import json
import math
from pathlib import Path

from PySide6.QtCore import QProcess, QTimer
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLineEdit, QLabel, QFileDialog, QPlainTextEdit,
)
import pyqtgraph as pg

# ---------------------------------------------------------------------------
# Structured event parsing
# ---------------------------------------------------------------------------
# RAPID.exe emits lines beginning with ">> " carrying a JSON event object.
# Human-readable lines are always printed alongside them for terminal use.
# Event types emitted by main.c:
#   {"type":"ack",       "r_um":<int>,  "theta_deg":<float>}
#   {"type":"crc_error"}
#   {"type":"done",      "points_sent":<int>, "acks_received":<int>}
# See src/main.c for the full list.

# Number of segments used to draw the reference circle on the plot
_CIRCLE_PTS = 256


# ---------------------------------------------------------------------------
# Protocol parser  (S-SRP-04 / S-OCP-03)
# ---------------------------------------------------------------------------

class ProtocolParser:
    """Parse RAPID.exe structured output lines and dispatch to registered handlers.

    Lines must begin with ">> " followed by a JSON object carrying a "type" field.
    Call register() once per event type; call feed_line() for each output line.
    Adding a new event type requires only a register() call — this class never
    needs to change.  (S-OCP-03)
    """

    def __init__(self):
        self._handlers: dict = {}

    def register(self, event_type: str, fn):
        """Register *fn(evt: dict)* as the handler for *event_type*."""
        self._handlers[event_type] = fn

    def feed_line(self, line: str):
        """Parse *line*; call the matching handler if registered.  Silent on errors."""
        if not line.startswith(">> "):
            return
        try:
            evt = json.loads(line[3:])
        except json.JSONDecodeError:
            return
        t = evt.get("type")
        if t in self._handlers:
            self._handlers[t](evt)


def polar_to_xy(r_list, theta_deg_list):
    """Convert parallel lists of (r, theta°) into (x, y) cartesian lists."""
    xs, ys = [], []
    for r, th in zip(r_list, theta_deg_list):
        rad = math.radians(th)
        xs.append(r * math.cos(rad))
        ys.append(r * math.sin(rad))
    return xs, ys


# ---------------------------------------------------------------------------
# Main GUI widget
# ---------------------------------------------------------------------------
class GUI(QWidget):
    """Single-window GUI: controls, live XY plot, and scrolling log."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("UART Polar GUI")

        # ---- subprocess state — normal run ----
        self.proc = QProcess(self)
        self.proc.setProcessChannelMode(QProcess.MergedChannels)
        self.proc.readyReadStandardOutput.connect(self._on_ready_read)
        self.proc.finished.connect(self._on_finished)
        self.proc.errorOccurred.connect(self._on_error)

        # ---- subprocess state — E2E test ----
        self.test_proc = QProcess(self)
        self.test_proc.setProcessChannelMode(QProcess.MergedChannels)
        self.test_proc.readyReadStandardOutput.connect(self._on_test_ready_read)
        self.test_proc.finished.connect(self._on_test_finished)
        self.test_proc.errorOccurred.connect(self._on_test_error)
        self._test_read_buf = ""

        # ---- data state ----
        self.ack_count = 0
        self.crc_count = 0
        self.ack_r: list[float] = []          # radii of ACK'd points (µm)
        self.ack_theta_deg: list[float] = []   # angles of ACK'd points (degrees)
        self._read_buf = ""                    # partial-line accumulation buffer
        self._plot_dirty = False               # flag: new data since last repaint

        # ---- protocol parser ----
        self._parser = ProtocolParser()
        self._parser.register("ack",       self._on_ack)
        self._parser.register("crc_error", self._on_crc_error)

        # ---- build UI ----
        root = QVBoxLayout(self)
        self._build_exe_row(root)
        self._build_port_file_row(root)
        self._build_button_row(root)
        self._build_test_row(root)
        self._build_status_row(root)
        self._build_plot(root)
        self._build_log(root)

        # Repaint the plot at a fixed interval rather than on every ACK
        self._plot_timer = QTimer(self)
        self._plot_timer.setInterval(100)  # 10 Hz refresh
        self._plot_timer.timeout.connect(self._refresh_plot)
        self._plot_timer.start()

    # ---- UI construction helpers ----

    def _build_exe_row(self, parent):
        row = QHBoxLayout()
        self.exe_path = QLineEdit(str(Path(__file__).parent.parent / "build" / "RAPID.exe"))
        btn = QPushButton("Browse EXE")
        btn.clicked.connect(self._pick_exe)
        row.addWidget(QLabel("Sender EXE:"))
        row.addWidget(self.exe_path, 1)
        row.addWidget(btn)
        parent.addLayout(row)

    def _build_port_file_row(self, parent):
        row = QHBoxLayout()
        self.port = QLineEdit("COM25")
        self.gds_file = QLineEdit("input.gds")
        btn = QPushButton("Browse File")
        btn.clicked.connect(self._pick_file)
        row.addWidget(QLabel("Port:"))
        row.addWidget(self.port)
        row.addWidget(QLabel("File:"))
        row.addWidget(self.gds_file, 1)
        row.addWidget(btn)
        parent.addLayout(row)

    def _build_button_row(self, parent):
        row = QHBoxLayout()
        self.btn_start = QPushButton("Start")
        self.btn_stop  = QPushButton("Stop")
        self.btn_stop.setEnabled(False)
        self.btn_clear = QPushButton("Clear")
        self.btn_start.clicked.connect(self.start)
        self.btn_stop.clicked.connect(self.stop)
        self.btn_clear.clicked.connect(self.clear)
        row.addWidget(self.btn_start)
        row.addWidget(self.btn_stop)
        row.addWidget(self.btn_clear)
        row.addStretch(1)
        parent.addLayout(row)

    def _build_test_row(self, parent):
        row = QHBoxLayout()
        self.sim_port = QLineEdit("COM4")
        self.sim_port.setFixedWidth(70)
        self.btn_run_test  = QPushButton("Run E2E Test")
        self.btn_stop_test = QPushButton("Stop Test")
        self.btn_stop_test.setEnabled(False)
        self.btn_run_test.clicked.connect(self.run_test)
        self.btn_stop_test.clicked.connect(self.stop_test)
        row.addWidget(QLabel("Sim Port:"))
        row.addWidget(self.sim_port)
        row.addWidget(self.btn_run_test)
        row.addWidget(self.btn_stop_test)
        row.addStretch(1)
        parent.addLayout(row)

    def _build_status_row(self, parent):
        row = QHBoxLayout()
        self.lbl_ack    = QLabel("ACK: 0")
        self.lbl_crc    = QLabel("CRC: 0")
        self.lbl_status = QLabel("Status: idle")
        row.addWidget(self.lbl_ack)
        row.addWidget(self.lbl_crc)
        row.addStretch(1)
        row.addWidget(self.lbl_status)
        parent.addLayout(row)

    def _build_plot(self, parent):
        """Create the pyqtgraph scatter plot with a reference circle."""
        self.plot = pg.PlotWidget()
        self.plot.setLabel("bottom", "X (µm)")
        self.plot.setLabel("left",   "Y (µm)")
        self.plot.showGrid(x=True, y=True, alpha=0.2)
        self.plot.setAspectLocked(True)
        self.curve_ack   = self.plot.plot([], [], pen=None, symbol="o", symbolSize=6)
        self.ref_circle  = self.plot.plot([], [], pen=pg.mkPen(width=1))
        parent.addWidget(self.plot, 1)

    def _build_log(self, parent):
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(3000)
        parent.addWidget(self.log, 1)

    # ---- file dialogs ----

    def _pick_exe(self):
        p, _ = QFileDialog.getOpenFileName(
            self, "Select sender exe", "", "Executable (*.exe);;All files (*)")
        if p:
            self.exe_path.setText(p)

    def _pick_file(self):
        p, _ = QFileDialog.getOpenFileName(
            self, "Select input file", "", "All files (*)")
        if p:
            self.gds_file.setText(p)

    # ---- helpers ----

    def _append_log(self, text: str):
        """Add one line to the scrolling log pane."""
        self.log.appendPlainText(text.rstrip())

    def _set_running(self, running: bool):
        """Toggle button enable state based on process running/stopped."""
        self.btn_start.setEnabled(not running)
        self.btn_stop.setEnabled(running)

    # ---- public actions ----

    def clear(self):
        """Reset all counters, data buffers, and the plot."""
        self.ack_count = 0
        self.crc_count = 0
        self.ack_r.clear()
        self.ack_theta_deg.clear()
        self._read_buf = ""
        self._plot_dirty = True
        self.lbl_ack.setText("ACK: 0")
        self.lbl_crc.setText("CRC: 0")
        self.log.clear()
        self._refresh_plot()

    def start(self):
        """Validate inputs and launch RAPID.exe as a child process."""
        exe  = self.exe_path.text().strip()
        port = self.port.text().strip()
        gds  = self.gds_file.text().strip()

        if not exe or not Path(exe).exists():
            self._append_log("[GUI] EXE path invalid.")
            return

        args = [port]
        if gds:
            args.append(gds)

        self._read_buf = ""
        self._append_log(f"[GUI] Starting: {exe} {' '.join(args)}")
        self.lbl_status.setText("Status: running")
        self._set_running(True)

        # keep working directory at the project root so input.gds resolves correctly
        self.proc.setWorkingDirectory(str(Path(exe).parent.parent))
        self.proc.setProgram(exe)
        self.proc.setArguments(args)
        self.proc.start()

    def stop(self):
        """Gracefully terminate, then kill if the process doesn't exit."""
        if self.proc.state() != QProcess.NotRunning:
            self._append_log("[GUI] Stopping process...")
            self.proc.terminate()
            if not self.proc.waitForFinished(1000):
                self.proc.kill()
        self.lbl_status.setText("Status: idle")
        self._set_running(False)

    def run_test(self):
        """Launch e2e_test.py connecting fpga_sim.py to RAPID.exe via a COM port pair."""
        if self.test_proc.state() != QProcess.NotRunning:
            return

        sim_port = self.sim_port.text().strip()
        pc_port  = self.port.text().strip()
        gds      = self.gds_file.text().strip()
        exe      = self.exe_path.text().strip()
        root     = str(Path(exe).parent.parent) if exe else "."

        script = str(Path(root) / "tests" / "e2e_test.py")
        args   = [
            script,
            "--sim-port", sim_port,
            "--pc-port",  pc_port,
            "--gds",      gds,
            "--rapid",    exe,
        ]

        self._append_log(f"[TEST] Starting E2E test  sim={sim_port}  pc={pc_port}  gds={gds}")
        self.lbl_status.setText("Status: testing")
        self.btn_run_test.setEnabled(False)
        self.btn_stop_test.setEnabled(True)

        self.test_proc.setWorkingDirectory(root)
        self.test_proc.setProgram(sys.executable)
        self.test_proc.setArguments(args)
        self.test_proc.start()

    def stop_test(self):
        if self.test_proc.state() != QProcess.NotRunning:
            self.test_proc.terminate()
            if not self.test_proc.waitForFinished(1000):
                self.test_proc.kill()

    # ---- QProcess signal handlers ----

    def _on_error(self, _err):
        self._append_log(f"[GUI] Process error: {self.proc.errorString()}")
        self.lbl_status.setText("Status: error")
        self._set_running(False)

    def _on_finished(self):
        if self._read_buf.strip():
            self._process_line(self._read_buf)
        self._read_buf = ""
        self._append_log("[GUI] Process finished.")
        self.lbl_status.setText("Status: idle")
        self._set_running(False)

    def _on_test_ready_read(self):
        self._test_read_buf += bytes(
            self.test_proc.readAllStandardOutput()).decode(errors="replace")
        while "\n" in self._test_read_buf:
            line, self._test_read_buf = self._test_read_buf.split("\n", 1)
            self._append_log(f"[TEST] {line.rstrip()}")

    def _on_test_finished(self, exit_code, _exit_status):
        if self._test_read_buf.strip():
            self._append_log(f"[TEST] {self._test_read_buf.rstrip()}")
        self._test_read_buf = ""
        result = "PASS" if exit_code == 0 else "FAIL"
        self._append_log(f"[TEST] {result} (exit code {exit_code})")
        self.lbl_status.setText(f"Status: test {result.lower()}")
        self.btn_run_test.setEnabled(True)
        self.btn_stop_test.setEnabled(False)

    def _on_test_error(self, _err):
        self._append_log(f"[TEST] Error: {self.test_proc.errorString()}")
        self.lbl_status.setText("Status: test error")
        self.btn_run_test.setEnabled(True)
        self.btn_stop_test.setEnabled(False)

    def _on_ready_read(self):
        """Accumulate stdout data; dispatch only complete newline-terminated lines."""
        self._read_buf += bytes(self.proc.readAllStandardOutput()).decode(errors="replace")

        while "\n" in self._read_buf:
            line, self._read_buf = self._read_buf.split("\n", 1)
            self._process_line(line)

    # ---- line parser ----

    def _process_line(self, line: str):
        """Log *line* then hand it to the protocol parser for event dispatch."""
        self._append_log(line)
        self._parser.feed_line(line)

    # ---- protocol event handlers ----

    def _on_ack(self, evt: dict):
        r_um      = evt.get("r_um")
        theta_deg = evt.get("theta_deg")
        if r_um is None or theta_deg is None:
            return
        self.ack_r.append(r_um)
        self.ack_theta_deg.append(theta_deg)
        self.ack_count += 1
        self.lbl_ack.setText(f"ACK: {self.ack_count}")
        self._plot_dirty = True

    def _on_crc_error(self, _evt: dict):
        self.crc_count += 1
        self.lbl_crc.setText(f"CRC: {self.crc_count}")

    # ---- plot updates ----

    def _refresh_plot(self):
        """Redraw the scatter plot and reference circle (called by timer)."""
        if not self._plot_dirty:
            return
        self._plot_dirty = False

        # update ACK scatter points
        if self.ack_theta_deg:
            ax, ay = polar_to_xy(self.ack_r, self.ack_theta_deg)
            self.curve_ack.setData(ax, ay)
        else:
            self.curve_ack.setData([], [])

        # draw a reference circle at the maximum radius
        rmax = max(self.ack_r) if self.ack_r else 0
        if rmax > 0:
            angles = [2 * math.pi * i / _CIRCLE_PTS for i in range(_CIRCLE_PTS + 1)]
            xs = [rmax * math.cos(a) for a in angles]
            ys = [rmax * math.sin(a) for a in angles]
            self.ref_circle.setData(xs, ys)
        else:
            self.ref_circle.setData([], [])


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = GUI()
    w.resize(1100, 700)
    w.show()

    # Allow Ctrl+C to reach Python's signal handler (Qt blocks it otherwise)
    signal.signal(signal.SIGINT, lambda *_: w.close())
    _sigint_timer = QTimer()
    _sigint_timer.setInterval(200)
    _sigint_timer.timeout.connect(lambda: None)  # wake the event loop periodically
    _sigint_timer.start()

    sys.exit(app.exec())