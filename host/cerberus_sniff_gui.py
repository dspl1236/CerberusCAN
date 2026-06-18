#!/usr/bin/env python3
"""
CerberusCAN — Live Sniff GUI
============================
Watch Head 2 (the listen-only MON logger) in real time: a live, aggregated
CAN trace grouped by ID, like a mini SavvyCAN grid.

    pip install pyserial          # tkinter ships with Python
    python cerberus_sniff_gui.py  # pick the COM port, hit Connect

On Connect it sends `MON:on` to the board and streams the `M2:<ms>:<id>:<hex>[:OVR]`
frames Head 2 captures off the bus (it ACKs nothing — pure passive sniff). On
Disconnect it sends `MON:off`. Command replies are ignored; only M2 frames shown.
"""
import sys, time, threading, queue, csv
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("This GUI needs pyserial:  pip install pyserial")
    sys.exit(1)

# Known VAG diagnostic-CAN IDs (the 500k OBD diag bus) -> friendly label.
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


def label_for(idhex):
    try:
        return LABELS.get(int(idhex, 16), "")
    except ValueError:
        return ""


class Sniffer:
    def __init__(self, root):
        self.root = root
        root.title("CerberusCAN — Live Sniff (Head 2 / MON)")
        self.ser = None
        self.running = False
        self.paused = False
        self.q = queue.Queue()
        self.agg = {}                       # id -> {count, last, ts, period}
        self.total = 0
        self.dropped = 0
        self._t_rate = time.time()
        self._n_rate = 0
        self.rate = 0.0
        self._build()
        self.root.after(50, self._poll)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------- UI ----------
    def _build(self):
        top = ttk.Frame(self.root, padding=6)
        top.pack(fill="x")
        ttk.Label(top, text="Port:").pack(side="left")
        self.port = tk.StringVar(value=self._default_port())
        self.port_cb = ttk.Combobox(top, textvariable=self.port, width=12, values=self._ports())
        self.port_cb.pack(side="left", padx=4)
        self.btn_conn = ttk.Button(top, text="Connect", command=self._toggle_conn)
        self.btn_conn.pack(side="left", padx=4)
        self.btn_pause = ttk.Button(top, text="Pause", command=self._toggle_pause, state="disabled")
        self.btn_pause.pack(side="left", padx=4)
        ttk.Button(top, text="Clear", command=self._clear).pack(side="left", padx=4)
        ttk.Button(top, text="Save CSV", command=self._save).pack(side="left", padx=4)

        cols = ("id", "label", "count", "period", "data")
        self.tree = ttk.Treeview(self.root, columns=cols, show="headings", height=22)
        for c, w, t, anchor in (
            ("id", 70, "ID", "center"),
            ("label", 150, "Label", "w"),
            ("count", 80, "Count", "e"),
            ("period", 90, "Period ms", "e"),
            ("data", 280, "Last Data", "w"),
        ):
            self.tree.heading(c, text=t)
            self.tree.column(c, width=w, anchor=anchor)
        self.tree.pack(fill="both", expand=True, padx=6)

        self.status = tk.StringVar(value="Disconnected.")
        ttk.Label(self.root, textvariable=self.status, relief="sunken",
                  anchor="w", padding=4).pack(fill="x", side="bottom")

    def _ports(self):
        try:
            return [p.device for p in serial.tools.list_ports.comports()]
        except Exception:
            return []

    def _default_port(self):
        ps = self._ports()
        return ps[0] if ps else "COM12"

    # ---------- connection ----------
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
        self.ser.write(b"MON:on\n")
        self.running = True
        threading.Thread(target=self._read_loop, daemon=True).start()
        self.btn_conn.config(text="Disconnect")
        self.btn_pause.config(state="normal")
        self.status.set(f"Connected {self.port.get()} — MON on")

    def _disconnect(self):
        self.running = False
        try:
            if self.ser:
                self.ser.write(b"MON:off\n")
                time.sleep(0.1)
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.btn_conn.config(text="Connect")
        self.btn_pause.config(state="disabled", text="Pause")
        self.paused = False
        self.status.set("Disconnected.")

    def _toggle_pause(self):
        self.paused = not self.paused
        self.btn_pause.config(text="Resume" if self.paused else "Pause")

    def _on_close(self):
        self._disconnect()
        self.root.destroy()

    # ---------- serial reader (background thread) ----------
    def _read_loop(self):
        buf = b""
        while self.running and self.ser:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                ln = raw.decode(errors="replace").strip()
                if ln.startswith("M2:") and not self.paused:
                    self.q.put(ln)

    # ---------- main-thread poll ----------
    def _poll(self):
        n = 0
        try:
            while True:
                self._ingest(self.q.get_nowait())
                n += 1
        except queue.Empty:
            pass
        if n:
            self._refresh()
        now = time.time()
        if now - self._t_rate >= 1.0:
            self.rate = self._n_rate / (now - self._t_rate)
            self._n_rate = 0
            self._t_rate = now
            if self.running:
                self.status.set(
                    f"{self.port.get()}  |  frames {self.total}  dropped {self.dropped}"
                    f"  ids {len(self.agg)}  {self.rate:.0f}/s")
        self.root.after(50, self._poll)

    def _ingest(self, ln):
        # M2:<ms>:<id>:<hex>[:OVR]
        parts = ln.split(":")
        if len(parts) < 4:
            return
        idh = parts[2].upper()
        data = parts[3]
        if len(parts) >= 5 and parts[4] == "OVR":
            self.dropped += 1
        try:
            ts = int(parts[1])
        except ValueError:
            ts = 0
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

    def _clear(self):
        self.agg.clear()
        self.total = 0
        self.dropped = 0
        for i in self.tree.get_children():
            self.tree.delete(i)

    def _save(self):
        f = filedialog.asksaveasfilename(defaultextension=".csv",
                                         filetypes=[("CSV", "*.csv")])
        if not f:
            return
        with open(f, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["id", "label", "count", "period_ms", "last_data"])
            for idh in sorted(self.agg):
                a = self.agg[idh]
                w.writerow([idh, label_for(idh), a["count"], a["period"], a["last"]])
        messagebox.showinfo("Saved", f)


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("740x560")
    Sniffer(root)
    root.mainloop()
