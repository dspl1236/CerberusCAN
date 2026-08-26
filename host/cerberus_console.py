#!/usr/bin/env python3
"""
CerberusCAN Console
===================
A plug-and-go app for the CerberusCAN VCI — no Simos-Suite needed.

    pip install pyserial          # tkinter ships with Python
    python cerberus_console.py    # pick the COM port, hit Connect

Organised by Cerberus's heads:
  * CANBUS  (Heads 1-2, CAN) — Sniff (passive Head-2 logger) + Diagnostics (Head-1 UDS:
            VIN/part, read/clear DTCs with SAE J2012 text).
  * K-Line  (Head 3, KWP2000/KW1281) — init, ECU-ID, read/clear faults (VAG DIDB text).
            Needs the K-line transceiver wired (Serial2 7/8 -> OBD 7).
  * Firmware — running vs bundled version, one-click flash.

Switching views sets the firmware MODE automatically; version + board show in the title bar.
"""
import sys, os, time, threading, queue, csv, subprocess, shutil
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

CONSOLE_VERSION = "0.9.26"
BUNDLED_FW = "0.9.18"                      # bump in lockstep when the bundled hexes change


def _base():
    if getattr(sys, "frozen", False):
        return getattr(sys, "_MEIPASS", os.path.dirname(sys.executable))
    return os.path.dirname(os.path.abspath(__file__))


BASE = _base()
sys.path.insert(0, BASE)   # so didb/ + sibling modules import when frozen or run from elsewhere


def _firstfile(*cands):
    for c in cands:
        if c and os.path.isfile(c):
            return c
    return None


def _hex(name):
    return _firstfile(os.path.join(BASE, "firmware", name),
                      os.path.join(BASE, "..", "firmware", name),
                      os.path.join(BASE, name))


HEX = {"T4.1": _hex("cerberus-can-teensy41.hex"), "T4.0": _hex("cerberus-can-teensy40.hex")}
MCU = {"T4.1": "TEENSY41", "T4.0": "TEENSY40"}


PIO_ENV = {"T4.1": "teensy41", "T4.0": "teensy40"}


def find_loader():
    for n in ("teensy_loader_cli.exe", "teensy_loader_cli"):
        f = _firstfile(os.path.join(BASE, "tools", n), os.path.join(BASE, n),
                       os.path.join(BASE, "..", "tools", n), os.path.join(BASE, "..", n)) or shutil.which(n)
        if f:
            return f
    return None


def find_pio():
    """PlatformIO executable — the fallback flasher when running from source (no teensy_loader_cli)."""
    home = os.path.expanduser("~")
    cands = [shutil.which("platformio"), shutil.which("pio"),
             os.path.join(home, "AppData", "Roaming", "Python", "Python314", "Scripts", "platformio.exe"),
             os.path.join(home, ".platformio", "penv", "Scripts", "platformio.exe")]
    return _firstfile(*[c for c in cands if c]) or shutil.which("platformio")


def find_repo_root():
    """Dir holding platformio.ini (so the pio fallback can build/upload from the right place)."""
    for d in (os.path.join(BASE, ".."), BASE, os.path.join(BASE, "..", "..")):
        if os.path.isfile(os.path.join(d, "platformio.ini")):
            return os.path.abspath(d)
    return None


try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("This app needs pyserial:  pip install pyserial")
    sys.exit(1)

# Decoders (local modules; degrade gracefully if data is absent)
try:
    import sae_decode            # CAN/UDS SAE J2012 text  (saedb/)
except Exception:
    sae_decode = None
try:
    import kline_decode          # K-line VAG fault-location text (didb/)
except Exception:
    kline_decode = None
try:
    import formula               # K-line measuring-block scaling
except Exception:
    formula = None

# Sniff-grid CAN-ID labels
LABELS = {
    0x700: "Broadcast/TP", 0x710: "Gateway req", 0x77A: "Gateway resp",
    0x7E0: "Engine req", 0x7E8: "Engine resp", 0x7E1: "TCU req", 0x7E9: "TCU resp",
    0x712: "Airbag req", 0x77C: "Airbag resp", 0x713: "Cluster req", 0x77D: "Cluster resp",
    0x714: "ABS req", 0x77E: "ABS resp", 0x715: "Climatronic req", 0x77F: "Climatronic resp",
    0x70E: "MMI req", 0x778: "MMI resp",
}
# CAN diagnostics module picker: (label, request id, response id)
MODULES = [
    ("Engine / ECM", "7E0", "7E8"), ("Transmission / TCM", "7E1", "7E9"),
    ("Gateway / J533", "710", "77A"), ("ABS / brakes", "714", "77E"),
    ("Airbag", "712", "77C"), ("Instrument cluster", "713", "77D"),
    ("Climatronic / HVAC", "715", "77F"), ("MMI / infotainment", "70E", "778"),
]
# K-line module picker: (label, address byte hex)
KL_MODULES = [
    ("Engine — 01", "01"), ("Transmission — 02", "02"), ("ABS — 03", "03"),
    ("Central electrics — 09", "09"), ("Airbag — 15", "15"), ("Instruments — 17", "17"),
    ("Immobiliser — 25", "25"),
]
NRC = {0x11: "service not supported", 0x12: "sub-function not supported",
       0x22: "conditions not correct", 0x31: "request out of range",
       0x33: "security access denied", 0x78: "response pending",
       0x7E: "service not supported in session", 0x7F: "service not supported in session"}


def label_for(idhex):
    try:
        return LABELS.get(int(idhex, 16), "")
    except ValueError:
        return ""


def decode_dtc(b0, b1, b2):
    """3-byte UDS DTC -> SAE J2012 code + failure-type byte, e.g. P0420-00."""
    cat = "PCBU"[(b0 >> 6) & 0x3]
    return f"{cat}{(b0 >> 4) & 0x3}{b0 & 0xF:X}{(b1 >> 4) & 0xF:X}{b1 & 0xF:X}-{b2:02X}"


def ascii_of(b):
    return "".join(chr(c) if 32 <= c < 127 else "." for c in b)


class Console:
    def __init__(self, root):
        self.root = root
        self.fw_ver = "?"
        self.fw_board = "?"
        root.title("CerberusCAN Console  v" + CONSOLE_VERSION)
        self.ser = None
        self.running = False
        self.mon_q = queue.Queue()
        self.resp_q = queue.Queue()
        self.result_q = queue.Queue()
        self.cmd_lock = threading.Lock()
        self._busy_btns = []
        self.rec_fh = None
        self.rec_writer = None
        self._link = "connected"
        self.agg = {}
        self.total = 0
        self.dropped = 0
        self._t_rate = time.time()
        self._n_rate = 0
        self.sniffing = False
        self._build()
        self.root.after(50, self._poll)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------- UI ----------------
    def _build(self):
        bar = ttk.Frame(self.root, padding=6)
        bar.pack(fill="x")
        ttk.Label(bar, text="Port:").pack(side="left")
        self.port = tk.StringVar(value=self._default_port())
        self.port_cb = ttk.Combobox(bar, textvariable=self.port, width=10, values=self._ports())
        self.port_cb.pack(side="left", padx=4)
        ttk.Button(bar, text="Rescan", width=7, command=self._refresh_ports).pack(side="left")  # re-scan COM ports
        self.btn_conn = ttk.Button(bar, text="Connect", command=self._toggle_conn)
        self.btn_conn.pack(side="left", padx=4)
        # INFO: show the popup AND refresh the fw/board fields from the reply
        self._action(bar, "INFO", lambda: self._diag("INFO", lambda r: (self._on_info(r), self._popup("INFO", r))))
        self._action(bar, "SELFTEST", lambda: self._diag("SELFTEST", self._selftest_done, 6))
        self.mode_lbl = tk.StringVar(value="—")
        ttk.Label(bar, textvariable=self.mode_lbl, foreground="#0a0",
                  font=("TkDefaultFont", 9, "bold")).pack(side="right")
        ttk.Label(bar, text="mode:").pack(side="right")

        self.nb = ttk.Notebook(self.root)               # top-level: CANBUS / K-Line / Firmware
        self.nb.pack(fill="both", expand=True, padx=6, pady=4)
        self._build_canbus()
        self._build_kline()
        self._build_raw()
        self._build_fw()
        self._refresh_fw_tab()   # enable the blank-board picker up front (a blank board can't connect)
        self.nb.bind("<<NotebookTabChanged>>", self._on_tab)

        self.status = tk.StringVar(value="Disconnected.")
        ttk.Label(self.root, textvariable=self.status, relief="sunken",
                  anchor="w", padding=4).pack(fill="x", side="bottom")

    # ----- CANBUS page (Heads 1-2): Sniff + Diagnostics -----
    def _build_canbus(self):
        page = ttk.Frame(self.nb)
        self.nb.add(page, text="CANBUS")
        self.canbus_nb = ttk.Notebook(page)
        self.canbus_nb.pack(fill="both", expand=True)
        self._build_sniff()
        self._build_diag()
        self.canbus_nb.bind("<<NotebookTabChanged>>", self._on_tab)

    def _build_sniff(self):
        f = ttk.Frame(self.canbus_nb)
        self.canbus_nb.add(f, text="Sniff")
        btns = ttk.Frame(f, padding=4)
        btns.pack(fill="x")
        self.btn_pause = ttk.Button(btns, text="Pause", command=self._toggle_pause)
        self.btn_pause.pack(side="left", padx=2)
        ttk.Button(btns, text="Clear", command=self._clear).pack(side="left", padx=2)
        ttk.Button(btns, text="Save grid", command=self._save).pack(side="left", padx=2)
        self.btn_rec = ttk.Button(btns, text="Record session", command=self._toggle_record)
        self.btn_rec.pack(side="left", padx=2)
        self.paused = False
        cols = ("id", "label", "count", "period", "data")
        self.tree = ttk.Treeview(f, columns=cols, show="headings", height=16)
        for c, w, t, a in (("id", 60, "ID", "center"), ("label", 140, "Label", "w"),
                           ("count", 70, "Count", "e"), ("period", 80, "Period ms", "e"),
                           ("data", 280, "Last Data", "w")):
            self.tree.heading(c, text=t)
            self.tree.column(c, width=w, anchor=a)
        self.tree.pack(fill="both", expand=True, padx=4, pady=4)

    def _build_diag(self):
        f = ttk.Frame(self.canbus_nb)
        self.canbus_nb.add(f, text="Diagnostics")
        top = ttk.Frame(f, padding=4)
        top.pack(fill="x")
        ttk.Label(top, text="Module:").pack(side="left")
        self.module = tk.StringVar(value=MODULES[0][0])
        ttk.Combobox(top, textvariable=self.module, width=20, state="readonly",
                     values=[m[0] for m in MODULES]).pack(side="left", padx=4)
        self._action(top, "Read VIN", lambda: self._read_did("F190"))
        self._action(top, "Read Part #", lambda: self._read_did("F187"))
        self._action(top, "Read DTCs", self._read_dtcs)
        self._action(top, "Clear DTCs", self._clear_dtcs)
        ttk.Button(top, text="Clear log", command=lambda: self.out.delete("1.0", "end")).pack(side="right", padx=2)
        self.out = tk.Text(f, height=18, wrap="word")
        self.out.pack(fill="both", expand=True, padx=4, pady=4)

    # ----- K-Line page (Head 3): KWP2000 / KW1281 -----
    def _build_kline(self):
        f = ttk.Frame(self.nb)
        self.nb.add(f, text="K-Line")
        note = ("Head 3 — K-line / KWP2000 (pre-CAN VAG). Needs the K-line transceiver wired "
                "(Serial2: pin 8→TXD, pin 7←RXD; line→OBD 7). Bench-untested.  •  Dumb-cable for "
                "NefMoto/VCDS-dumb is ALWAYS transparent on the board's 2nd COM port (shown at right) "
                "— no trigger needed; smart KWP here just yields the K-line while that app is open.")
        ttk.Label(f, text=note, foreground="#a60", wraplength=720, padding=(6, 4)).pack(fill="x")
        top = ttk.Frame(f, padding=4)
        top.pack(fill="x")
        ttk.Label(top, text="Module/addr:").pack(side="left")
        self.kl_mod = tk.StringVar(value=KL_MODULES[0][0])
        ttk.Combobox(top, textvariable=self.kl_mod, width=20, state="readonly",
                     values=[m[0] for m in KL_MODULES]).pack(side="left", padx=4)
        self._action(top, "Fast init", self._kwp_fast)
        self._action(top, "5-baud init", self._kwp_slow)
        self._action(top, "Session 10 89", lambda: self._kwp_send("1089", "session"))
        self._action(top, "Read ECU-ID", self._kwp_ecuid)
        self._action(top, "Read Faults", self._kwp_faults)
        self._action(top, "Clear Faults", self._kwp_clear)
        ttk.Button(top, text="Clear log", command=lambda: self.kl_out.delete("1.0", "end")).pack(side="right", padx=2)
        top2 = ttk.Frame(f, padding=(4, 0))
        top2.pack(fill="x")
        self.kl_proto = "kwp"           # which protocol last inited -> how Read-MB frames the request
        ttk.Label(top2, text="Measuring group:").pack(side="left")
        self.kl_group = tk.StringVar(value="1")
        ttk.Spinbox(top2, from_=0, to=255, width=5, textvariable=self.kl_group).pack(side="left", padx=4)
        self._action(top2, "Read MB", self._read_mb)
        ttk.Label(top2, text="      KW1281:").pack(side="left")
        self._action(top2, "Init", self._k81_init)
        self._action(top2, "Read block", self._k81_read)
        self.kl_rawport = tk.StringVar(value="")     # auto-filled with the always-dumb K-line COM port
        ttk.Label(top2, textvariable=self.kl_rawport, foreground="#06c",
                  font=("TkDefaultFont", 9, "bold")).pack(side="right", padx=6)
        self.kl_out = tk.Text(f, height=16, wrap="word")
        self.kl_out.pack(fill="both", expand=True, padx=6, pady=4)

    def _action(self, parent, text, cmd):
        b = ttk.Button(parent, text=text, command=cmd)
        b.pack(side="left", padx=2)
        self._busy_btns.append(b)
        return b

    def _set_busy(self, busy):
        st = "disabled" if busy else "normal"
        for b in self._busy_btns:
            try:
                b.config(state=st)
            except Exception:
                pass

    def _ports(self):
        try:
            return [p.device for p in serial.tools.list_ports.comports()]
        except Exception:
            return []

    def _default_port(self):
        cands = self._candidate_ports()
        return cands[0] if cands else "COM12"

    def _candidate_ports(self):
        """COM devices to try on connect — Teensy (USB VID 0x16C0) first, then the rest. Probing
        each for an INFO reply is the source of truth; this just orders the attempts and avoids
        relying on serial_number (which a frozen PyInstaller exe often leaves blank)."""
        try:
            ports = list(serial.tools.list_ports.comports())
        except Exception:
            return []
        teensy = sorted(p.device for p in ports if p.vid == 0x16C0)
        others = sorted(p.device for p in ports if p.vid != 0x16C0)
        return teensy + others

    def _refresh_ports(self):
        """Re-scan COM ports (for a board plugged in after the app started) and update the dropdown.
        Keeps the current selection if still present, else picks the best Teensy candidate."""
        ports = self._ports()
        self.port_cb["values"] = ports
        if self.port.get() not in ports:
            self.port.set(self._default_port())
        self.status.set("Ports: " + (", ".join(ports) if ports else "none found"))

    # ---------------- connection ----------------
    def _toggle_conn(self):
        self._disconnect() if self.running else self._connect_smart()

    def _probe_port(self, dev, timeout=1.6):
        """Synchronously test whether dev is the Cerberus COMMAND port: open, send INFO, read
        directly for a 'CERBERUS:' reply. Returns (open_serial, info_line) on success — the port is
        left OPEN and handed to _connect (no close/reopen race) — or (None, None). Direct blocking
        read, no async/queue timing, so it's reliable in the frozen exe. Raw K-line CDC -> (None,None).

        Special case: a bare 0x07 (BEL) reply is SLCAN's NACK for an unknown command, so the board
        IS a Cerberus but is parked in Lawicel mode and no longer answers INFO. That returns
        (None, "slcan") so the caller can say so instead of "no board found"."""
        try:
            s = serial.Serial(dev, 115200, timeout=0.2)
        except Exception:
            return None, None
        try:
            time.sleep(0.25)
            s.reset_input_buffer()
            s.write(b"INFO\n")
            deadline = time.time() + timeout
            buf = b""
            while time.time() < deadline:
                chunk = s.read(256)
                if chunk:
                    buf += chunk
                    if b"\x07" in buf and b"CERBERUS:" not in buf:
                        s.close()             # SLCAN NACK: it's a Cerberus, wrong mode
                        return None, "slcan"
                    if b"CERBERUS:" in buf:
                        info = ""
                        for line in buf.replace(b"\r", b"\n").split(b"\n"):
                            if b"CERBERUS:" in line:
                                info = line.decode("ascii", "replace").strip()
                                break
                        return s, info        # leave OPEN, hand off to _connect
            s.close()
            return None, None
        except Exception:
            try:
                s.close()
            except Exception:
                pass
            return None, None

    def _connect_smart(self):
        """Find the port that actually answers INFO (the smart command port) and connect to THAT.
        Probes the user's selection first, then the other Teensy (VID 0x16C0) ports — never relies
        on serial_number (blank in a frozen exe), and skips the silent raw K-line CDC."""
        sel = self.port.get()
        order = [sel] + [d for d in self._candidate_ports() if d != sel]
        stuck = []                              # ports that NACKed -> Cerberus in SLCAN mode
        for dev in order:
            self.status.set(f"Probing {dev} for the Cerberus command port…")
            self.root.update_idletasks()
            ser, info = self._probe_port(dev)
            if ser:
                self.port.set(dev)
                self._connect(ser=ser, info=info)
                return
            if info == "slcan":
                stuck.append(dev)
        if stuck:
            messagebox.showerror("Cerberus stuck in SLCAN mode",
                f"{', '.join(stuck)} answered 0x07 (BEL) to INFO — the Lawicel SLCAN NACK.\n\n"
                "So the board IS there and the firmware is fine; it is just parked in SLCAN "
                "mode, where it no longer speaks the native protocol (no INFO, SNIFF, MODE, "
                "SCAN or MON).\n\n"
                "SLCAN mode is auto-entered by any Lawicel-looking line — O/C/L/V/F/N, S<n>, "
                "or t/T+hex — so any SavvyCAN, python-can or slcand session flips it, and it "
                "can ONLY be left by a board reset.\n\n"
                "• Power-cycle the Teensy: pull USB AT THE BOARD END. A powered hub or USB "
                "extender keeps 5 V on the board if you only unplug the PC end, and then the "
                "sketch never restarts.\n"
                "• Do NOT reflash — there is nothing wrong with the firmware.\n\n"
                "Then try Connect again.")
            self.status.set(f"{stuck[0]}: stuck in SLCAN mode — power-cycle the board (not a reflash).")
            return
        messagebox.showerror("No Cerberus found",
            "No COM port answered INFO.\n\n"
            "• Check the board is plugged in and flashed (the K-line port is silent by design).\n"
            "• Close any other app holding the port (incl. the Python console).\n"
            "Then try Connect again.")
        self.status.set("No Cerberus command port found — see the dialog.")

    def _connect(self, ser=None, info=None):
        if ser is None:                         # manual path (no pre-probed handle)
            try:
                ser = serial.Serial(self.port.get(), 115200, timeout=0.2)
            except Exception as e:
                messagebox.showerror("Connect failed", str(e))
                return
            time.sleep(0.4)
        self.ser = ser
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self.running = True
        threading.Thread(target=self._read_loop, daemon=True).start()
        self.btn_conn.config(text="Disconnect")
        self.status.set(f"Connected {self.port.get()}")
        if info:                                # set fw/board straight from the probe's INFO —
            self._on_info(info)                 # detection no longer depends on the async read path
        self._on_tab()
        self._diag("INFO", self._on_info, 6.0)  # re-confirm via the live path (won't clobber on timeout)

    def _disconnect(self):
        self.running = False
        self.sniffing = False
        self._stop_record()
        try:
            if self.ser:
                self.ser.write(b"MON:off\n")
                time.sleep(0.1)
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.fw_ver = self.fw_board = "?"
        self.root.title("CerberusCAN Console  v" + CONSOLE_VERSION)
        self.fw_running.set("— (connect first)")
        self.fw_status.set("")
        self.kl_rawport.set("")
        self.btn_conn.config(text="Connect")
        self.mode_lbl.set("—")
        self.status.set("Disconnected.")

    def _sibling_kline_port(self):
        """The board's OTHER CDC port = the always-dumb raw K-line cable. Prefer matching by USB
        serial number; fall back to 'the other Teensy (VID 0x16C0) port' when serial_number is
        blank — which it often is in a frozen PyInstaller exe. Returns its device, or None."""
        try:
            ports = list(serial.tools.list_ports.comports())
        except Exception:
            return None
        me = self.port.get()
        my_sn = next((p.serial_number for p in ports if p.device == me), None)
        if my_sn:
            sibs = [p.device for p in ports if p.serial_number == my_sn and p.device != me]
            if sibs:
                return sorted(sibs)[0]
        teensy = sorted(p.device for p in ports if p.vid == 0x16C0 and p.device != me)   # fallback
        return teensy[0] if teensy else None

    def _refresh_rawport(self):
        """Passively show WHICH COM is the always-dumb raw K-line cable (the board's 2nd CDC port).
        No trigger needed — that port is transparent from power-on; this just labels it so you know
        where to point NefMoto. Smart KWP here yields it automatically while that app is open."""
        sib = self._sibling_kline_port() if self.running else None
        self.kl_rawport.set(f"Raw K-line cable (NefMoto) → {sib}" if sib else "")

    def _on_close(self):
        self._disconnect()
        self.root.destroy()

    def _current_view(self):
        top = self.nb.tab(self.nb.select(), "text")
        if top == "CANBUS":
            try:
                return self.canbus_nb.tab(self.canbus_nb.select(), "text")  # "Sniff" | "Diagnostics"
            except Exception:
                return "Diagnostics"
        return top  # "K-Line" | "Firmware"

    def _on_tab(self, _evt=None):
        if not self.running:
            return
        view = self._current_view()
        if view == "Sniff":
            self._send_raw("MODE:sniff")
            self._send_raw("MON:on")
            self.mode_lbl.set("SNIFF")
            self.sniffing = True
        else:
            self.sniffing = False
            self._send_raw("MON:off")
            self._send_raw("MODE:vci")
            self.mode_lbl.set("KWP" if view == "K-Line" else "VCI")

    # ---------------- serial ----------------
    def _send_raw(self, cmd):
        try:
            if self.ser:
                self.ser.write((cmd + "\n").encode())
        except Exception:
            pass

    def _read_loop(self):
        buf = b""
        while self.running and self.ser:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception:
                if not self.running:
                    break
                buf = b""
                if not self._reconnect():
                    break
                continue
            if not chunk:
                continue
            try:
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    ln = raw.decode(errors="replace").strip()
                    if not ln:
                        continue
                    if ln.startswith("M2:"):
                        if self.sniffing and not self.paused:
                            self.mon_q.put(ln)
                    else:
                        self.resp_q.put(ln)
            except Exception:
                import traceback
                traceback.print_exc()      # to the log; never let the read thread die silently
                buf = b""

    def _reconnect(self):
        self._link = "reconnecting"
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        while self.running:
            try:
                self.ser = serial.Serial(self.port.get(), 115200, timeout=0.2)
            except Exception:
                time.sleep(1.0)
                continue
            time.sleep(0.4)
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
            if self.sniffing:
                self._send_raw("MODE:sniff")
                self._send_raw("MON:on")
            else:
                self._send_raw("MODE:vci")
            self._link = "connected"
            return True
        return False

    def _cmd(self, cmd, timeout=8.0, expect=None):
        with self.cmd_lock:
            try:
                while True:
                    self.resp_q.get_nowait()
            except queue.Empty:
                pass
            self._send_raw(cmd)
            t0 = time.time()
            while time.time() - t0 < timeout:
                try:
                    ln = self.resp_q.get(timeout=0.2)
                except queue.Empty:
                    continue
                if ln.startswith("OK:"):
                    if expect:
                        p = ln[3:].upper()
                        if not (p.startswith(expect) or p.startswith("7F")):
                            continue
                    return ln
                if ln.startswith(("ERR", "DONE", "STATS:", "MODE:", "M2STAT",
                                  "PONG", "CERBERUS:", "H2TEST", "SELFTEST")):
                    return ln
            return "(timeout)"

    def _diag(self, cmd, parser, timeout=8.0, expect=None):
        if not self.running:
            messagebox.showinfo("Not connected", "Connect first.")
            return
        self._set_busy(True)

        def work():
            r = self._cmd(cmd, timeout, expect)
            self.result_q.put((parser, r))
        threading.Thread(target=work, daemon=True).start()

    # ---------------- CAN diagnostics ----------------
    def _mod_ids(self):
        name = self.module.get()
        for m in MODULES:
            if m[0] == name:
                return m[1], m[2]
        return "7E0", "7E8"

    def _read_did(self, did):
        tx, rx = self._mod_ids()
        self._diag(f"UDS:1:{tx}:{rx}:22{did}", lambda r: self._show(self._fmt_did(r, did)), expect="62")

    def _read_dtcs(self):
        tx, rx = self._mod_ids()
        self._diag(f"UDS:1:{tx}:{rx}:1902FF", lambda r: self._show(self._fmt_dtcs(r)), expect="59")

    def _clear_dtcs(self):
        tx, rx = self._mod_ids()
        if not messagebox.askyesno("Clear DTCs", f"Clear all DTCs on {self.module.get()}?"):
            return
        self._diag(f"UDS:1:{tx}:{rx}:14FFFFFF", lambda r: self._show(self._fmt_clear(r)), expect="54")

    def _fmt_did(self, r, did):
        if r.startswith("OK:"):
            try:
                b = bytes.fromhex(r[3:])
                if len(b) >= 3 and b[0] == 0x62:
                    return f"{self.module.get()}  DID {did}: {ascii_of(b[3:])}"
            except ValueError:
                pass
        return self._fmt_neg(r) or f"{self.module.get()}  DID {did}: {r}"

    def _fmt_dtcs(self, r):
        head = f"{self.module.get()}  DTCs:"
        if r.startswith("OK:"):
            try:
                b = bytes.fromhex(r[3:])
            except ValueError:
                return f"{head} {r}"
            if len(b) >= 2 and b[0] == 0x59 and b[1] == 0x02:
                recs = b[3:]
                n = len(recs) // 4
                if n == 0:
                    return f"{head} none stored."
                lines = [head]
                for i in range(0, n * 4, 4):
                    b0, b1, b2, st = recs[i], recs[i + 1], recs[i + 2], recs[i + 3]
                    code = decode_dtc(b0, b1, b2)
                    desc = sae_decode.describe(code.split("-")[0], b2) if sae_decode else ""
                    flag = " [confirmed]" if st & 0x08 else (" [pending]" if st & 0x04 else "")
                    txt = f"   {code}   {desc}".rstrip() if desc else f"   {code}"
                    lines.append(f"{txt}   status=0x{st:02X}{flag}")
                return "\n".join(lines)
        return self._fmt_neg(r) or f"{head} {r}"

    def _fmt_clear(self, r):
        if r.startswith("OK:") and r[3:].upper().startswith("54"):
            return f"{self.module.get()}  DTCs cleared (positive 54)."
        return self._fmt_neg(r) or f"{self.module.get()}  clear: {r}"

    def _fmt_neg(self, r):
        if r.startswith("OK:7F") or (r.startswith("OK:") and r[3:5].upper() == "7F"):
            try:
                b = bytes.fromhex(r[3:])
                nrc = b[2] if len(b) >= 3 else 0
                return f"{self.module.get()}  negative response 7F: {NRC.get(nrc, f'NRC 0x{nrc:02X}')}"
            except ValueError:
                pass
        if r.startswith("ERR"):
            return f"{self.module.get()}  {r}"
        return None

    def _show(self, text):
        self.out.insert("end", text + "\n")
        self.out.see("end")

    # ---------------- K-line (KWP2000) actions ----------------
    def _kl_addr(self):
        name = self.kl_mod.get()
        for label, addr in KL_MODULES:
            if label == name:
                return addr
        return "01"

    def _kl_show(self, text):
        self.kl_out.insert("end", text + "\n")
        self.kl_out.see("end")

    def _kwp_fast(self):
        self.kl_proto = "kwp"
        self._diag(f"KWP:fast:{self._kl_addr()}", self._kl_show, 3.0)

    def _kwp_slow(self):
        self.kl_proto = "kwp"
        self._diag(f"KWP:slow:{self._kl_addr()}", self._kl_show, 5.0)

    def _k81_init(self):
        self.kl_proto = "kw1281"
        self._diag(f"K81:init:{self._kl_addr()}", self._kl_show, 6.0)

    def _k81_read(self):
        self._diag("K81:read", lambda r: self._kl_show(self._fmt_block(r)), 4.0)

    def _read_mb(self):
        try:
            g = int(self.kl_group.get()) & 0xFF
        except ValueError:
            g = 1
        if self.kl_proto == "kw1281":
            self._diag(f"K81:block:29:{g:02X}", lambda r: self._kl_show(self._fmt_block(r, g)), 5.0)
        else:
            self._diag(f"KWP:21{g:02X}", lambda r: self._kl_show(self._fmt_mb(r, g)), 5.0)

    def _cells(self, payload):
        """Decode a measuring block's 3-byte [formula][A][B] cells via formula.py."""
        out = []
        for i in range(0, (len(payload) // 3) * 3, 3):
            f, a, b = payload[i], payload[i + 1], payload[i + 2]
            if formula:
                _, _, disp = formula.decode_cell(f, a, b)
            else:
                disp = f"raw {a*256+b} (fmt=0x{f:02X})"
            out.append(f"      cell {i//3+1}: {disp}")
        return out or ["      (no cells)"]

    def _fmt_mb(self, r, g):
        """KWP2000 0x21 measuring block: response payload [0x61][localid][cells...]."""
        if r.startswith("OK:"):
            try:
                b = bytes.fromhex(r[3:])
                if len(b) >= 2 and b[0] == 0x61:
                    return f"Measuring group {g}:\n" + "\n".join(self._cells(b[2:]))
            except ValueError:
                pass
        return f"Measuring group {g}: {r}"

    def _fmt_block(self, r, g=None):
        """KW1281 block dump: 'OK:title=XX data=...' — for measuring groups (0xE7), decode cells."""
        if r.startswith("OK:title="):
            body = r[3:]
            title = body.split()[0].split("=")[1] if "=" in body else "?"
            data_hex = body.split("data=")[1] if "data=" in body else ""
            try:
                data = bytes.fromhex(data_hex)
            except ValueError:
                data = b""
            if title.upper() == "E7" and data:        # measuring-value response
                head = f"Measuring group {g}:" if g is not None else "Measuring values:"
                return head + "\n" + "\n".join(self._cells(data))
            return f"block title=0x{title} data={data_hex}"
        return f"block: {r}"

    def _kwp_send(self, hexreq, what):
        self._diag(f"KWP:{hexreq}", lambda r: self._kl_show(f"{what}: {r}"), 4.0)

    def _kwp_ecuid(self):
        self._diag("KWP:1A9B", lambda r: self._kl_show(self._fmt_kwp_id(r)), 4.0)

    def _kwp_faults(self):
        self._diag("KWP:1882FFFFFF", lambda r: self._kl_show(self._fmt_kwp_faults(r)), 5.0)

    def _kwp_clear(self):
        if not messagebox.askyesno("Clear K-line faults", "Clear all faults on the K-line ECU?"):
            return
        self._diag("KWP:14FF00", lambda r: self._kl_show(f"clear: {r}"), 4.0)

    def _fmt_kwp_id(self, r):
        if r.startswith("OK:"):
            try:
                b = bytes.fromhex(r[3:])
                if len(b) >= 2 and b[0] == 0x5A:
                    return f"ECU-ID: {ascii_of(b[2:])}"
            except ValueError:
                pass
        return f"ECU-ID: {r}"

    def _fmt_kwp_faults(self, r):
        if not r.startswith("OK:"):
            return f"K-line faults: {r}"
        try:
            b = bytes.fromhex(r[3:])
        except ValueError:
            return f"K-line faults: {r}"
        if len(b) >= 2 and b[0] == 0x58:           # readDTCByStatus positive
            count = b[1]
            recs = b[2:]
            if count == 0 or len(recs) < 3:
                return "K-line faults: none stored."
            lines = [f"K-line faults ({count}):"]
            for i in range(0, (len(recs) // 3) * 3, 3):
                hi, lo, st = recs[i], recs[i + 1], recs[i + 2]
                if (hi << 8 | lo) == 0xFFFF:
                    continue
                if kline_decode:
                    lines.append("   " + kline_decode.fault_line(hi, lo, st))
                else:
                    lines.append(f"   {(hi<<8|lo):05d}   status=0x{st:02X}")
            return "\n".join(lines)
        return f"K-line faults: {r}"

    def _popup(self, title, r):
        messagebox.showinfo(title, r)

    def _selftest_done(self, r):
        messagebox.showinfo("SELFTEST", r)
        self._on_tab()

    # ---------------- main-thread poll ----------------
    def _poll(self):
        try:
            while True:
                cb, r = self.result_q.get_nowait()
                cb(r)
                self._set_busy(False)
        except queue.Empty:
            pass
        n = 0
        try:
            while True:
                self._ingest(self.mon_q.get_nowait())
                n += 1
        except queue.Empty:
            pass
        if n:
            self._refresh()
        now = time.time()
        if now - self._t_rate >= 1.0:
            rate = self._n_rate / (now - self._t_rate)
            self._n_rate = 0
            self._t_rate = now
            if self.running:
                link = "  [RECONNECTING…]" if self._link == "reconnecting" else ""
                rec = "  ●REC" if self.rec_writer else ""
                self.status.set(
                    f"{self.port.get()}  |  {self.mode_lbl.get()}  |  frames {self.total}"
                    f"  dropped {self.dropped}  ids {len(self.agg)}  {rate:.0f}/s{rec}{link}")
        self.root.after(50, self._poll)

    def _ingest(self, ln):
        parts = ln.split(":")
        if len(parts) < 4:
            return
        idh = parts[2].upper()
        data = parts[3]
        ovr = len(parts) >= 5 and parts[4] == "OVR"
        if ovr:
            self.dropped += 1
        try:
            ts = int(parts[1])
        except ValueError:
            ts = 0
        if self.rec_writer:
            self.rec_writer.writerow([f"{time.time():.3f}", ts, idh, label_for(idh), data, "OVR" if ovr else ""])
        a = self.agg.get(idh)
        if a is None:
            self.agg[idh] = {"count": 1, "last": data, "ts": ts, "period": 0}
        else:
            if ts and a["ts"]:
                a["period"] = ts - a["ts"]
            a["ts"] = ts
            a["count"] += 1
            a["last"] = data
        self.total += 1
        self._n_rate += 1

    def _refresh(self):
        for idh in sorted(self.agg):
            a = self.agg[idh]
            vals = (idh, label_for(idh), a["count"], a["period"], a["last"])
            if self.tree.exists(idh):
                self.tree.item(idh, values=vals)
            else:
                self.tree.insert("", "end", iid=idh, values=vals)

    def _toggle_pause(self):
        self.paused = not self.paused
        self.btn_pause.config(text="Resume" if self.paused else "Pause")

    def _clear(self):
        self.agg.clear()
        self.total = 0
        self.dropped = 0
        for i in self.tree.get_children():
            self.tree.delete(i)

    def _save(self):
        f = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")])
        if not f:
            return
        with open(f, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["id", "label", "count", "period_ms", "last_data"])
            for idh in sorted(self.agg):
                a = self.agg[idh]
                w.writerow([idh, label_for(idh), a["count"], a["period"], a["last"]])
        messagebox.showinfo("Saved", f)

    def _toggle_record(self):
        if self.rec_writer:
            self._stop_record()
            return
        f = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")],
                                         title="Record session to…")
        if not f:
            return
        self.rec_fh = open(f, "w", newline="")
        self.rec_writer = csv.writer(self.rec_fh)
        self.rec_writer.writerow(["host_time", "bus_ms", "id", "label", "data_hex", "flags"])
        self.btn_rec.config(text="Stop recording")
        self.status.set("Recording session -> " + f)

    def _stop_record(self):
        if self.rec_writer:
            try:
                self.rec_fh.close()
            except Exception:
                pass
        self.rec_fh = self.rec_writer = None
        try:
            self.btn_rec.config(text="Record session")
        except Exception:
            pass

    # ---------------- firmware page ----------------
    # ----- Raw TX page: type any firmware command, see the raw reply (custom TX per head) -----
    def _build_raw(self):
        f = ttk.Frame(self.nb)
        self.nb.add(f, text="Raw TX")
        ttk.Label(f, foreground="#06c", wraplength=760, padding=(6, 4),
                  text="Send any firmware command verbatim — the prefix picks the head. "
                       "Single request/response (for live streams use the Sniff tab).").pack(fill="x")
        cheat = (
            "CAN   <TX>:<RX>:<HEX>             UDS on Head 1 (shorthand)      710:77A:2200BE\n"
            "      UDS:<bus>:<TX>:<RX>:<HEX>   full ISO-TP UDS (bus 1|2)      UDS:1:7E0:7E8:22F190\n"
            "      RAW:<bus>:<ID>:<HEX>        one classic frame (no ISO-TP)  RAW:1:7DF:0201050000000000\n"
            "      CANX:<bus>:<ID>:<HEX>[:ms]  send one frame, then listen ms CANX:1:7DF:0201:300\n"
            "KWP   KWP:<hex> | KWP:raw:<hex>   K-line framed / raw bytes      KWP:1A9B   KWP:raw:81\n"
            "K81   K81:read:<title>            KW1281 read block\n"
            "DoIP  (future head)"
        )
        ttk.Label(f, text=cheat, font=("Consolas", 9), justify="left",
                  foreground="#555", padding=(10, 2)).pack(fill="x", anchor="w")
        row = ttk.Frame(f, padding=6)
        row.pack(fill="x")
        ttk.Label(row, text="Command:").pack(side="left")
        self.raw_in = ttk.Entry(row)
        self.raw_in.pack(side="left", fill="x", expand=True, padx=4)
        self.raw_in.bind("<Return>", lambda e: self._raw_send())
        ttk.Button(row, text="Send", command=self._raw_send).pack(side="left")
        ttk.Button(row, text="Clear", command=lambda: self.raw_out.delete("1.0", "end")).pack(side="left", padx=4)
        self.raw_out = tk.Text(f, height=15, wrap="word")
        self.raw_out.pack(fill="both", expand=True, padx=6, pady=4)

    def _raw_log(self, t):
        self.raw_out.insert("end", t + "\n")
        self.raw_out.see("end")

    def _raw_send(self):
        cmd = self.raw_in.get().strip()
        if not cmd:
            return
        if not self.running:
            messagebox.showinfo("Not connected", "Connect first.")
            return
        self._raw_log("> " + cmd)
        self._diag(cmd, lambda r: self._raw_log(r))   # routed through the cmd machinery -> raw reply

    def _build_fw(self):
        f = ttk.Frame(self.nb)
        self.nb.add(f, text="Firmware")
        g = ttk.Frame(f, padding=8)
        g.pack(fill="x", anchor="w")
        self.fw_running = tk.StringVar(value="— (connect first)")
        self.fw_status = tk.StringVar(value="")
        ttk.Label(g, text="Console version:").grid(row=0, column=0, sticky="w")
        ttk.Label(g, text=CONSOLE_VERSION).grid(row=0, column=1, sticky="w", padx=8)
        ttk.Label(g, text="Running firmware:").grid(row=1, column=0, sticky="w")
        ttk.Label(g, textvariable=self.fw_running).grid(row=1, column=1, sticky="w", padx=8)
        ttk.Label(g, text="Bundled firmware:").grid(row=2, column=0, sticky="w")
        ttk.Label(g, text=BUNDLED_FW).grid(row=2, column=1, sticky="w", padx=8)
        # Blank-board picker: only enabled when INFO can't self-identify the board
        # (a never-flashed Teensy reports board=?; 4.0 and 4.1 share MCU + bootloader,
        #  so the model can't be auto-detected until our firmware is running on it).
        ttk.Label(g, text="Blank board:").grid(row=3, column=0, sticky="w")
        self.board_choice = tk.StringVar(value="")
        self.cmb_board = ttk.Combobox(g, textvariable=self.board_choice, state="disabled",
                                      width=22, values=("Teensy 4.0 (Orthrus)", "Teensy 4.1 (Cerberus)"))
        self.cmb_board.grid(row=3, column=1, sticky="w", padx=8)
        ttk.Label(g, textvariable=self.fw_status, foreground="#0a0",
                  font=("TkDefaultFont", 9, "bold")).grid(row=4, column=0, columnspan=2, sticky="w", pady=6)
        btns = ttk.Frame(f)
        btns.pack(anchor="w", padx=8)
        self.btn_flash = ttk.Button(btns, text="Flash / Update firmware", command=self._flash_firmware)
        self.btn_flash.pack(side="left")
        ttk.Button(btns, text="Detect new / blank board",
                   command=self._detect_blank_board).pack(side="left", padx=6)
        self.fw_out = tk.Text(f, height=12, wrap="word")
        self.fw_out.pack(fill="both", expand=True, padx=8, pady=6)

    def _on_info(self, r):
        # Only act on a real INFO reply. A timed-out/other reply (e.g. "(timeout)") must NOT clobber
        # a good detection — that bug reset fw/board to "?" right after the probe set them correctly.
        if not r or not r.startswith("CERBERUS:"):
            return
        self.fw_ver, self.fw_board, self.product = "?", "?", "CerberusCAN"
        toks = r.split()
        head = toks[0].split(":", 1)
        if len(head) == 2:
            self.fw_ver = head[1]
        for t in toks:
            if t.startswith("board="):
                self.fw_board = t.split("=", 1)[1]
            elif t.startswith("product="):
                self.product = t.split("=", 1)[1]       # Cerberus (4.1) | Orthrus (4.0)
        self.root.title(f"{self.product} Console  v{CONSOLE_VERSION}  —  {self.fw_board} fw {self.fw_ver}")
        self._refresh_fw_tab()
        self._refresh_rawport()              # we're on the smart port now → label the raw sibling

    def _refresh_fw_tab(self):
        self.fw_running.set(f"{self.fw_board}  —  {self.fw_ver}")
        # The picker is only live when the board can't self-identify (blank/never-flashed).
        if self.fw_board in HEX:
            self.cmb_board.set("")
            self.cmb_board.config(state="disabled")
        else:
            self.cmb_board.config(state="readonly")
        if self.fw_ver == BUNDLED_FW:
            self.fw_status.set("Up to date.")
        elif self.fw_board not in HEX:
            self.fw_status.set("Board not auto-detected — pick 4.0/4.1 above, then Flash (first flash only).")
        elif self.fw_ver == "?":
            self.fw_status.set("")
        else:
            self.fw_status.set(f"Bundled {BUNDLED_FW} differs from running {self.fw_ver} — Flash to update.")

    def _fw_append(self, t):
        self.fw_out.insert("end", t + "\n")
        self.fw_out.see("end")

    def _post_fw(self, text):
        self.result_q.put((lambda r, t=text: self._fw_append(t), None))

    def _scan_pjrc_pids(self):
        """Connected PJRC (VID 16C0) USB product-ids — including HID / bootloader boards that do NOT
        show up as COM ports. Windows-only (Get-PnpDevice); empty set elsewhere. CREATE_NO_WINDOW so
        no console window flashes when the frozen GUI exe spawns PowerShell."""
        if not sys.platform.startswith("win"):
            return set()
        ps = (r"Get-PnpDevice | Where-Object { $_.InstanceId -match 'VID_16C0' -and $_.Status -eq 'OK' }"
              r" | Select-Object -ExpandProperty InstanceId")
        try:
            out = subprocess.run(["powershell", "-NoProfile", "-Command", ps],
                                 capture_output=True, text=True, timeout=20,
                                 creationflags=0x08000000).stdout      # CREATE_NO_WINDOW
        except Exception:
            return set()
        import re
        return {int(m.group(1), 16) for ln in out.splitlines()
                for m in [re.search(r"PID_([0-9A-Fa-f]{4})", ln)] if m}

    def _detect_blank_board(self):
        """Find a Teensy in ANY state — factory HID, bootloader, or already running our firmware —
        even when it isn't a COM port, and guide the user to flash it."""
        self.fw_status.set("Scanning USB for a Teensy…")
        self.root.update_idletasks()
        pids = self._scan_pjrc_pids()
        if 0x048B in pids:
            self.fw_status.set("Cerberus board present (Dual Serial) — click Connect at the top.")
        elif 0x0478 in pids:
            self.cmb_board.config(state="readonly")
            self.fw_status.set("Teensy in BOOTLOADER (flash mode) — pick 4.0/4.1 above, then Flash.")
        elif pids:
            self.cmb_board.config(state="readonly")
            self.fw_status.set("Blank/factory Teensy detected — pick 4.0/4.1 above, click Flash, "
                               "then press the white PROGRAM button when prompted.")
        else:
            self.fw_status.set("No Teensy found — plug it in with a DATA usb cable, then Detect again.")

    def _flash_firmware(self):
        # Two paths: (1) auto — a connected board self-identified via INFO; we reboot it
        # over serial into the bootloader. (2) manual — board not detected (blank/never-flashed,
        # may not even be connected); use the picker and let the loader wait for a button press.
        board = self.fw_board
        manual = board not in HEX
        if manual:
            pick = self.board_choice.get()
            if "4.0" in pick:
                board = "T4.0"
            elif "4.1" in pick:
                board = "T4.1"
            else:
                messagebox.showinfo("Pick a board",
                                    "Board couldn't be auto-detected (blank/never-flashed Teensy).\n"
                                    "Choose Teensy 4.0 or 4.1 in the 'Blank board' box, then Flash.")
                return
        elif not self.running:
            messagebox.showinfo("Not connected", "Connect first so the board model can be detected.")
            return
        hexf = HEX.get(board)
        mcu = MCU.get(board)
        if not hexf or not mcu:
            messagebox.showerror("No firmware", f"No bundled hex found for board '{board}'.")
            return
        # Flash backend: teensy_loader_cli if present (bundled in the .exe), else fall back to
        # PlatformIO upload (available when running from source — you built the firmware with it).
        loader = find_loader()
        pio = repo = None
        if not loader:
            pio, repo = find_pio(), find_repo_root()
            if not (pio and repo):
                messagebox.showerror("No flasher found",
                                     "Couldn't find teensy_loader_cli OR PlatformIO.\n\n"
                                     "Either drop teensy_loader_cli.exe next to the Console, or install "
                                     "PlatformIO (pip install platformio) so the source-build flash path works.")
                return
        if manual:
            prompt = (f"Flash {os.path.basename(hexf)}  ({mcu})  to a {board} board?\n\n"
                      "Board not auto-detected (blank/never-flashed). After you click Yes, the loader "
                      "waits — press the white PROGRAM button on the Teensy to enter the bootloader.\n"
                      "(Safe: the bootloader is in ROM — a bad flash can't brick it.)")
        else:
            prompt = (f"Flash {os.path.basename(hexf)}  ({mcu})  to the connected board?\n\n"
                      "The board reboots into the bootloader, flashes, then restarts.\n"
                      "(Safe: the bootloader is in ROM — a bad flash can't brick it.)")
        if not messagebox.askyesno("Flash firmware", prompt):
            return
        self.btn_flash.config(state="disabled")
        self.fw_out.delete("1.0", "end")

        def work():
            try:
                if manual:
                    self._post_fw(f"Manual flash ({board}) — press the PROGRAM button on the Teensy…")
                else:
                    self._post_fw("Rebooting board to bootloader…")
                    self._send_raw("REBOOT")
                    time.sleep(0.3)
                    self.running = False
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    time.sleep(1.0)
                if loader:
                    cmd, cwd = [loader, f"--mcu={mcu}", "-w", "-v", hexf], None
                else:                                   # PlatformIO fallback (source build)
                    cmd, cwd = [pio, "run", "-e", PIO_ENV[board], "-t", "upload"], repo
                    self._post_fw(f"(teensy_loader_cli not found — using PlatformIO upload from {repo})")
                self._post_fw("$ " + " ".join(cmd))
                if manual:
                    self._post_fw(">>> Now PRESS the white PROGRAM button on the Teensy to flash <<<")
                    self.result_q.put((lambda r: self.status.set(
                        "Waiting for the board — press the white PROGRAM button on the Teensy now…"), None))
                cnw = 0x08000000 if sys.platform.startswith("win") else 0   # CREATE_NO_WINDOW: no popup console
                p = subprocess.run(cmd, capture_output=True, text=True, cwd=cwd, creationflags=cnw,
                                   timeout=300 if not loader else (180 if manual else 90))
                if p.stdout:
                    self._post_fw(p.stdout.strip())
                if p.stderr:
                    self._post_fw(p.stderr.strip())
                self._post_fw("Flash " + ("OK — board restarting." if p.returncode == 0
                                          else f"FAILED (exit {p.returncode})"))
            except Exception as e:
                self._post_fw("ERROR: " + str(e))
            finally:
                self.result_q.put((lambda r: self._flash_done(), None))
        threading.Thread(target=work, daemon=True).start()

    def _flash_done(self):
        self.btn_flash.config(state="normal")
        self.btn_conn.config(text="Connect")
        self.mode_lbl.set("—")
        self.status.set("Flash complete — click Connect to reconnect and verify.")


def _setup_frozen_logging():
    """A --windowed PyInstaller exe has sys.stdout/stderr == None. Any print or uncaught
    traceback then raises inside a background thread and silently kills it (the read loop!),
    which is exactly why the frozen exe connected but never saw INFO. Redirect both to a log
    file so that can't happen — and so errors are visible. Returns the log path (or None)."""
    if sys.stdout is not None and sys.stderr is not None:
        return None
    import tempfile
    try:
        path = os.path.join(tempfile.gettempdir(), "CerberusConsole.log")
        f = open(path, "a", buffering=1, encoding="utf-8", errors="replace")
        sys.stdout = sys.stderr = f
        f.write("\n--- CerberusConsole start ---\n")
        return path
    except Exception:
        class _Null:
            def write(self, *a): pass
            def flush(self): pass
        sys.stdout = sys.stderr = _Null()
        return None


if __name__ == "__main__":
    _setup_frozen_logging()
    root = tk.Tk()
    root.geometry("780x620")
    Console(root)
    root.mainloop()
