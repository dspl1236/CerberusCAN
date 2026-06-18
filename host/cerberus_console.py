#!/usr/bin/env python3
"""
CerberusCAN Console
===================
A simple plug-and-go app for the CerberusCAN VCI — no Simos-Suite needed.

    pip install pyserial          # tkinter ships with Python
    python cerberus_console.py    # pick the COM port, hit Connect

Tabs:
  * Sniff       — live passive CAN trace (Head 2). Selecting it puts the board in
                  MODE:sniff (BOTH heads listen-only = zero bus footprint), safe to
                  log a live ODIS/dealer session.
  * Diagnostics — basic active VCI: read VIN / part number, Read DTCs, Clear DTCs,
                  per module. Selecting it puts the board in MODE:vci (Head 1 active).

Switching tabs sets the firmware MODE automatically, so you never think about it.
Top bar: Connect, live mode label, INFO, SELFTEST.
"""
import sys, os, time, threading, queue, csv, subprocess, shutil
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

CONSOLE_VERSION = "0.9.6"
BUNDLED_FW = "0.9.6"                       # bump in lockstep when the bundled hexes change


def _base():
    # PyInstaller-frozen: bundled data lives in sys._MEIPASS; dev: this script's folder.
    if getattr(sys, "frozen", False):
        return getattr(sys, "_MEIPASS", os.path.dirname(sys.executable))
    return os.path.dirname(os.path.abspath(__file__))


BASE = _base()


def _firstfile(*cands):
    for c in cands:
        if c and os.path.isfile(c):
            return c
    return None


def _hex(name):
    return _firstfile(os.path.join(BASE, "firmware", name),        # frozen bundle / shipped layout
                      os.path.join(BASE, "..", "firmware", name),   # dev repo layout
                      os.path.join(BASE, name))


HEX = {"T4.1": _hex("cerberus-can-teensy41.hex"), "T4.0": _hex("cerberus-can-teensy40.hex")}
MCU = {"T4.1": "TEENSY41", "T4.0": "TEENSY40"}


def find_loader():
    """teensy_loader_cli.exe — bundled in tools/ (frozen exe or repo), else next to the app, else PATH."""
    for n in ("teensy_loader_cli.exe", "teensy_loader_cli"):
        f = _firstfile(os.path.join(BASE, "tools", n), os.path.join(BASE, n),
                       os.path.join(BASE, "..", "tools", n), os.path.join(BASE, "..", n)) or shutil.which(n)
        if f:
            return f
    return None

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("This app needs pyserial:  pip install pyserial")
    sys.exit(1)

# Known VAG diagnostic-CAN IDs (500k OBD diag bus) -> label, for the sniff grid.
LABELS = {
    0x700: "Broadcast/TP",
    0x710: "Gateway req",   0x77A: "Gateway resp",
    0x7E0: "Engine req",    0x7E8: "Engine resp",
    0x7E1: "TCU req",       0x7E9: "TCU resp",
    0x712: "Airbag req",    0x77C: "Airbag resp",
    0x713: "Cluster req",   0x77D: "Cluster resp",
    0x714: "ABS req",       0x77E: "ABS resp",
    0x715: "Climatronic req", 0x77F: "Climatronic resp",
    0x70E: "MMI req",       0x778: "MMI resp",
}
# Diagnostics module picker: (label, request id, response id)
MODULES = [
    ("Engine / ECM",        "7E0", "7E8"),
    ("Transmission / TCM",  "7E1", "7E9"),
    ("Gateway / J533",      "710", "77A"),
    ("ABS / brakes",        "714", "77E"),
    ("Airbag",              "712", "77C"),
    ("Instrument cluster",  "713", "77D"),
    ("Climatronic / HVAC",  "715", "77F"),
    ("MMI / infotainment",  "70E", "778"),
]
NRC = {0x11: "service not supported", 0x12: "sub-function not supported",
       0x22: "conditions not correct", 0x31: "request out of range",
       0x33: "security access denied", 0x7E: "service not supported in session",
       0x7F: "service not supported in session"}


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
        self.mon_q = queue.Queue()        # M2 sniff frames
        self.resp_q = queue.Queue()       # command replies
        self.result_q = queue.Queue()     # (callback, text) -> main thread
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
        ttk.Combobox(bar, textvariable=self.port, width=10,
                     values=self._ports()).pack(side="left", padx=4)
        self.btn_conn = ttk.Button(bar, text="Connect", command=self._toggle_conn)
        self.btn_conn.pack(side="left", padx=4)
        self._action(bar, "INFO", lambda: self._diag("INFO", lambda r: self._popup("INFO", r)))
        self._action(bar, "SELFTEST", lambda: self._diag("SELFTEST", self._selftest_done, 6))
        self.mode_lbl = tk.StringVar(value="—")
        ttk.Label(bar, textvariable=self.mode_lbl, foreground="#0a0",
                  font=("TkDefaultFont", 9, "bold")).pack(side="right")
        ttk.Label(bar, text="mode:").pack(side="right")

        self.nb = ttk.Notebook(self.root)
        self.nb.pack(fill="both", expand=True, padx=6, pady=4)
        self._build_sniff()
        self._build_diag()
        self._build_fw()
        self.nb.bind("<<NotebookTabChanged>>", self._on_tab)

        self.status = tk.StringVar(value="Disconnected.")
        ttk.Label(self.root, textvariable=self.status, relief="sunken",
                  anchor="w", padding=4).pack(fill="x", side="bottom")

    def _build_sniff(self):
        f = ttk.Frame(self.nb)
        self.nb.add(f, text="Sniff")
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
        self.tree = ttk.Treeview(f, columns=cols, show="headings", height=18)
        for c, w, t, a in (("id", 60, "ID", "center"), ("label", 140, "Label", "w"),
                           ("count", 70, "Count", "e"), ("period", 80, "Period ms", "e"),
                           ("data", 280, "Last Data", "w")):
            self.tree.heading(c, text=t)
            self.tree.column(c, width=w, anchor=a)
        self.tree.pack(fill="both", expand=True, padx=4, pady=4)

    def _build_diag(self):
        f = ttk.Frame(self.nb)
        self.nb.add(f, text="Diagnostics")
        top = ttk.Frame(f, padding=4)
        top.pack(fill="x")
        ttk.Label(top, text="Module:").pack(side="left")
        self.module = tk.StringVar(value=MODULES[0][0])
        ttk.Combobox(top, textvariable=self.module, width=22, state="readonly",
                     values=[m[0] for m in MODULES]).pack(side="left", padx=4)
        self._action(top, "Read VIN", lambda: self._read_did("F190"))
        self._action(top, "Read Part #", lambda: self._read_did("F187"))
        self._action(top, "Read DTCs", self._read_dtcs)
        self._action(top, "Clear DTCs", self._clear_dtcs)
        ttk.Button(top, text="Clear log", command=lambda: self.out.delete("1.0", "end")).pack(side="right", padx=2)
        self.out = tk.Text(f, height=20, wrap="word")
        self.out.pack(fill="both", expand=True, padx=4, pady=4)

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
        ps = self._ports()
        return ps[0] if ps else "COM12"

    # ---------------- connection ----------------
    def _toggle_conn(self):
        self._disconnect() if self.running else self._connect()

    def _connect(self):
        try:
            self.ser = serial.Serial(self.port.get(), 115200, timeout=0.2)
        except Exception as e:
            messagebox.showerror("Connect failed", str(e))
            return
        time.sleep(0.4)
        self.ser.reset_input_buffer()
        self.running = True
        threading.Thread(target=self._read_loop, daemon=True).start()
        self.btn_conn.config(text="Disconnect")
        self.status.set(f"Connected {self.port.get()}")
        self._on_tab()  # apply the current tab's mode
        self._diag("INFO", self._on_info, 2.0)   # read version + board -> title bar + Firmware tab

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
        self.btn_conn.config(text="Connect")
        self.mode_lbl.set("—")
        self.status.set("Disconnected.")

    def _on_close(self):
        self._disconnect()
        self.root.destroy()

    def _on_tab(self, _evt=None):
        if not self.running:
            return
        tab = self.nb.tab(self.nb.select(), "text")
        if tab == "Sniff":
            self._send_raw("MODE:sniff")     # both heads LOM = zero footprint
            self._send_raw("MON:on")         # explicit too, so it streams even on pre-MODE firmware
            self.mode_lbl.set("SNIFF")
            self.sniffing = True
        else:
            self.sniffing = False
            self._send_raw("MON:off")
            self._send_raw("MODE:vci")       # Head 1 active VCI
            self.mode_lbl.set("VCI")

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
                if not self._reconnect():   # USB dropped (flaky extender) -> keep retrying
                    break
                continue
            if not chunk:
                continue
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

    def _reconnect(self):
        """USB link dropped (common on long runs over flaky extenders). Keep retrying to reopen
        the port, then re-apply the current mode and resume. The board may have reset (USB-powered),
        so MON/mode are re-asserted. Returns False if the user disconnected meanwhile."""
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
        """Synchronous send + wait for a terminal reply. Worker-thread only.
        `expect` (e.g. '59','62','54') guards against stale/cross-talk replies: an OK:<hex>
        whose payload doesn't start with the expected response SID (or 7F) is skipped, so a
        leftover reply from a prior read can never be mistaken for this one."""
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
                            continue   # not the reply we asked for -> keep waiting
                    return ln
                if ln.startswith(("ERR", "DONE", "STATS:", "MODE:", "M2STAT",
                                  "PONG", "CERBERUS:", "H2TEST", "SELFTEST")):
                    return ln
            return "(timeout)"

    def _diag(self, cmd, parser, timeout=8.0, expect=None):
        if not self.running:
            messagebox.showinfo("Not connected", "Connect first.")
            return
        self._set_busy(True)   # block click-ahead until this command's reply lands

        def work():
            r = self._cmd(cmd, timeout, expect)
            self.result_q.put((parser, r))
        threading.Thread(target=work, daemon=True).start()

    # ---------------- diagnostics actions ----------------
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
                recs = b[3:]                          # skip 59 02 <availability-mask>
                n = len(recs) // 4
                if n == 0:
                    return f"{head} none stored."
                lines = [head]
                for i in range(0, n * 4, 4):
                    code = decode_dtc(recs[i], recs[i + 1], recs[i + 2])
                    st = recs[i + 3]
                    flag = " [confirmed]" if st & 0x08 else (" [pending]" if st & 0x04 else "")
                    lines.append(f"   {code}   status=0x{st:02X}{flag}")
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

    def _popup(self, title, r):
        # top-bar actions (INFO/SELFTEST) show here so they're visible from ANY tab,
        # not buried in the Diagnostics log.
        messagebox.showinfo(title, r)

    def _selftest_done(self, r):
        messagebox.showinfo("SELFTEST", r)
        self._on_tab()   # SELFTEST reconfigures the heads + turns MON off -> restore current mode

    # ---------------- main-thread poll ----------------
    def _poll(self):
        # diagnostics results
        try:
            while True:
                cb, r = self.result_q.get_nowait()
                cb(r)
                self._set_busy(False)   # command done -> re-enable the action buttons
        except queue.Empty:
            pass
        # sniff frames
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
            self.rec_writer.writerow([f"{time.time():.3f}", ts, idh, label_for(idh),
                                      data, "OVR" if ovr else ""])
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

    # ---------------- firmware tab ----------------
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
        ttk.Label(g, textvariable=self.fw_status, foreground="#0a0",
                  font=("TkDefaultFont", 9, "bold")).grid(row=3, column=0, columnspan=2, sticky="w", pady=6)
        self.btn_flash = ttk.Button(f, text="Flash / Update firmware", command=self._flash_firmware)
        self.btn_flash.pack(anchor="w", padx=8)
        self.fw_out = tk.Text(f, height=14, wrap="word")
        self.fw_out.pack(fill="both", expand=True, padx=8, pady=6)

    def _on_info(self, r):
        self.fw_ver, self.fw_board = "?", "?"
        if r.startswith("CERBERUS:"):
            toks = r.split()
            head = toks[0].split(":", 1)
            if len(head) == 2:
                self.fw_ver = head[1]
            for t in toks:
                if t.startswith("board="):
                    self.fw_board = t.split("=", 1)[1]
        self.root.title(f"CerberusCAN Console  v{CONSOLE_VERSION}  —  {self.fw_board} fw {self.fw_ver}")
        self._refresh_fw_tab()

    def _refresh_fw_tab(self):
        self.fw_running.set(f"{self.fw_board}  —  {self.fw_ver}")
        if self.fw_ver == BUNDLED_FW:
            self.fw_status.set("Up to date.")
        elif self.fw_ver == "?":
            self.fw_status.set("")
        else:
            self.fw_status.set(f"Bundled {BUNDLED_FW} differs from running {self.fw_ver} — Flash to update.")

    def _fw_append(self, t):
        self.fw_out.insert("end", t + "\n")
        self.fw_out.see("end")

    def _post_fw(self, text):
        self.result_q.put((lambda r, t=text: self._fw_append(t), None))

    def _flash_firmware(self):
        if not self.running:
            messagebox.showinfo("Not connected", "Connect first so the board model can be detected.")
            return
        board = self.fw_board
        hexf = HEX.get(board)
        mcu = MCU.get(board)
        if not hexf or not mcu:
            messagebox.showerror("No firmware", f"No bundled hex found for board '{board}'.")
            return
        loader = find_loader()
        if not loader:
            messagebox.showerror("teensy_loader_cli not found",
                                 "Put teensy_loader_cli.exe next to the Console (or install Teensyduino "
                                 "so it's on PATH), then try again.")
            return
        if not messagebox.askyesno("Flash firmware",
                                   f"Flash {os.path.basename(hexf)}  ({mcu})  to the connected board?\n\n"
                                   "The board reboots into the bootloader, flashes, then restarts.\n"
                                   "(Safe: the HalfKay bootloader is in ROM — a bad flash can't brick it.)"):
            return
        self.btn_flash.config(state="disabled")
        self.fw_out.delete("1.0", "end")

        def work():
            try:
                self._post_fw("Rebooting board to bootloader…")
                self._send_raw("REBOOT")
                time.sleep(0.3)
                self.running = False
                try:
                    self.ser.close()
                except Exception:
                    pass
                time.sleep(1.0)
                cmd = [loader, f"--mcu={mcu}", "-w", "-v", hexf]
                self._post_fw("$ " + " ".join(cmd))
                p = subprocess.run(cmd, capture_output=True, text=True, timeout=90)
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

    def _toggle_record(self):
        """Stream EVERY frame, in order, to a CSV (host_time, bus_ms, id, hex, flags) — the full
        chronological session timeline for replay/RE, as opposed to Save-grid's aggregate."""
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


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("760x600")
    Console(root)
    root.mainloop()
