#!/usr/bin/env python3
"""
Simple, robust GPS navigator GUI (Tkinter) — minimal deps (only pyserial).
- Reads NMEA RMC/GGA from a serial port (Adafruit, GT-U7, or Nucleo pass-through)
- Shows: Lat/Lon, HDOP, Sats, Speed, Distance, Bearing, Heading Err, Cross-Track Err
- Lets you set Target (supports placeholders like 51.4707A), Vars (A=23,B=07), Shelter,
  Arrival radius, and a Storm deadline (seconds) for auto-divert.
- Clean shutdown, background serial reader thread, 10 Hz UI update.

Later ROS2: add rclpy publisher/subscribers inside TODO stubs.

Usage:
  python3 -m pip install pyserial
  python3 gps_nav_gui.py
"""
import sys, math, time, threading, queue
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial, serial.tools.list_ports
except Exception as e:
    print("pyserial required. Install with: python3 -m pip install pyserial", file=sys.stderr)
    raise

R_EARTH_M = 6371000.0

# ----------------------- NMEA + math helpers -------------------------------

def nmea_deg(val, hemi):
    if not val or '.' not in val:
        return None
    dot = val.find('.')
    deg_len = 3 if len(val[:dot]) >= 5 else 2  # lon usually 3 deg digits
    try:
        deg = float(val[:deg_len])
        minutes = float(val[deg_len:])
    except ValueError:
        return None
    dec = deg + minutes/60.0
    if hemi in ('S','W'):
        dec = -dec
    return dec

def haversine_m(lat1, lon1, lat2, lon2):
    ph1, ph2 = math.radians(lat1), math.radians(lat2)
    dph = ph2 - ph1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dph/2)**2 + math.cos(ph1)*math.cos(ph2)*math.sin(dl/2)**2
    return 2*R_EARTH_M*math.asin(math.sqrt(a))

def initial_bearing_deg(lat1, lon1, lat2, lon2):
    ph1, ph2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(ph2)
    x = math.cos(ph1)*math.cos(ph2) - math.sin(ph1)*math.sin(ph2)*math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

def cross_track_error_m(lat_start, lon_start, lat_end, lon_end, lat_cur, lon_cur):
    d13 = haversine_m(lat_start, lon_start, lat_cur, lon_cur) / R_EARTH_M
    th13 = math.radians(initial_bearing_deg(lat_start, lon_start, lat_cur, lon_cur))
    th12 = math.radians(initial_bearing_deg(lat_start, lon_start, lat_end, lon_end))
    return math.asin(math.sin(d13) * math.sin(th13 - th12)) * R_EARTH_M

def parse_rmc(f):
    if len(f) < 12 or f[2] != 'A':
        return None
    lat = nmea_deg(f[3], f[4]); lon = nmea_deg(f[5], f[6])
    cog = float(f[8]) if f[8] else None
    spd = float(f[7])*0.514444 if f[7] else None  # knots->m/s
    return lat, lon, cog, spd

def parse_gga(f):
    if len(f) < 10:
        return None
    fix = f[6]
    if fix in ('0',''):
        return None
    lat = nmea_deg(f[2], f[3]); lon = nmea_deg(f[4], f[5])
    sats = int(f[7]) if f[7].isdigit() else None
    try:
        hdop = float(f[8]) if f[8] else None
    except:
        hdop = None
    return lat, lon, sats, hdop

def fill_placeholders(coord_str, mapping):
    out = []
    for ch in coord_str.strip():
        if ch.isalpha() and ch.upper() in mapping:
            out.append(str(mapping[ch.upper()]))
        else:
            out.append(ch)
    return float(''.join(out))

def parse_coord(s, varmap):
    if s is None:
        return None
    s = s.strip()
    if any(ch.isalpha() for ch in s):
        return fill_placeholders(s, varmap)
    return float(s)

# -------------------------- Serial reader thread ---------------------------

class SerialReader(threading.Thread):
    def __init__(self, port, baud, out_queue, stop_event):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_q = out_queue
        self.stop = stop_event
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
        except Exception as e:
            self.out_q.put({"type":"err","msg":f"Serial open failed: {e}"})
            return
        buf = bytearray()
        last_rmc = None
        last_gga = None
        while not self.stop.is_set():
            try:
                chunk = self.ser.read(512)
                if chunk:
                    buf.extend(chunk)
                    while True:
                        nl = buf.find(b'\n')
                        if nl < 0:
                            break
                        line = buf[:nl].strip()
                        del buf[:nl+1]
                        if not line.startswith(b'$'):
                            continue
                        s = line.decode('utf-8','ignore')
                        f = s.split(',')
                        typ = f[0][3:6] if len(f[0]) >= 6 else ''
                        tnow = time.monotonic()
                        if typ == 'RMC':
                            r = parse_rmc(f)
                            if r:
                                last_rmc = r  # (lat,lon,cog,spd)
                        elif typ == 'GGA':
                            g = parse_gga(f)
                            if g:
                                last_gga = g  # (lat,lon,sats,hdop)
                        # merge best-known into a single fix
                        lat = lon = cog = spd = sats = hdop = None
                        if last_rmc:
                            lat, lon, cog, spd = last_rmc
                        if last_gga:
                            lat, lon, sats, hdop = last_gga
                        if lat is not None and lon is not None:
                            self.out_q.put({
                                "type":"fix",
                                "lat":lat, "lon":lon,
                                "cog":cog, "spd":spd,
                                "sats":sats, "hdop":hdop,
                                "ts":tnow
                            })
            except Exception as e:
                self.out_q.put({"type":"err","msg":str(e)})
                time.sleep(0.1)
        try:
            if self.ser:
                self.ser.close()
        except:
            pass

# ------------------------------ Main GUI -----------------------------------

class GPSNavGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("GPS Navigator — minimal GUI")
        self.geometry("920x360")
        self.minsize(860, 320)

        # State
        self.varmap = {}
        self.target = [None, None]
        self.shelter = [None, None]
        self.arrive_m = tk.DoubleVar(value=2.0)
        self.deadline_t = None

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.status_var = tk.StringVar(value="Disconnected")

        self.fix = None          # (lat,lon,cog,spd,sats,hdop,ts)
        self.start_fix = None

        self.q = queue.Queue()
        self.stop_evt = threading.Event()
        self.reader = None

        self._build_ui()
        self._refresh_ports()
        self.after(50, self._ui_tick)

    # ---- UI ----
    def _build_ui(self):
        pad = {"padx": 6, "pady": 4}
        top = ttk.Frame(self)
        top.pack(fill="x", **pad)

        ttk.Label(top, text="Port:").pack(side="left")
        self.port_menu = ttk.Combobox(top, textvariable=self.port_var, width=28, state="readonly")
        self.port_menu.pack(side="left", padx=4)
        ttk.Button(top, text="↻", width=3, command=self._refresh_ports).pack(side="left")

        ttk.Label(top, text="Baud:").pack(side="left", padx=(12,4))
        ttk.Entry(top, textvariable=self.baud_var, width=8).pack(side="left")
        self.btn = ttk.Button(top, text="Connect", command=self._toggle)
        self.btn.pack(side="left", padx=8)
        ttk.Label(top, textvariable=self.status_var).pack(side="left", padx=8)

        # Target / Vars
        frm = ttk.LabelFrame(self, text="Targets & Settings")
        frm.pack(fill="x", **pad)

        # Target
        ttk.Label(frm, text="Target lat:").grid(row=0, column=0, sticky="e")
        self.tlat = ttk.Entry(frm, width=18)
        self.tlat.grid(row=0, column=1, sticky="w")
        ttk.Label(frm, text="lon:").grid(row=0, column=2, sticky="e")
        self.tlon = ttk.Entry(frm, width=18)
        self.tlon.grid(row=0, column=3, sticky="w")
        ttk.Button(frm, text="Set Target", command=self._set_target).grid(row=0, column=4, padx=6)

        # Vars
        ttk.Label(frm, text="Vars (A=23,B=07):").grid(row=1, column=0, sticky="e")
        self.vars_entry = ttk.Entry(frm, width=28)
        self.vars_entry.grid(row=1, column=1, columnspan=3, sticky="we")
        ttk.Button(frm, text="Apply Vars", command=self._apply_vars).grid(row=1, column=4, padx=6)

        # Shelter
        ttk.Label(frm, text="Shelter lat:").grid(row=2, column=0, sticky="e")
        self.sh_lat = ttk.Entry(frm, width=18)
        self.sh_lat.grid(row=2, column=1, sticky="w")
        ttk.Label(frm, text="lon:").grid(row=2, column=2, sticky="e")
        self.sh_lon = ttk.Entry(frm, width=18)
        self.sh_lon.grid(row=2, column=3, sticky="w")
        ttk.Button(frm, text="Set Shelter", command=self._set_shelter).grid(row=2, column=4, padx=6)

        # Arrival + Deadline
        ttk.Label(frm, text="Arrive (m):").grid(row=3, column=0, sticky="e")
        ttk.Entry(frm, textvariable=self.arrive_m, width=10).grid(row=3, column=1, sticky="w")
        ttk.Label(frm, text="Deadline (s):").grid(row=3, column=2, sticky="e")
        self.deadline_entry = ttk.Entry(frm, width=10)
        self.deadline_entry.grid(row=3, column=3, sticky="w")
        ttk.Button(frm, text="Start Timer", command=self._start_deadline).grid(row=3, column=4, padx=6)

        for i in range(5):
            frm.grid_columnconfigure(i, weight=1)

        # Live metrics
        live = ttk.LabelFrame(self, text="Live")
        live.pack(fill="both", expand=True, **pad)

        self.lbl_fix = ttk.Label(live, text="Lat: –  Lon: –   HDOP: –  Sats: –  Speed: – m/s")
        self.lbl_fix.pack(anchor="w", padx=6, pady=2)

        self.lbl_tgt = ttk.Label(live, text="Target: unset, unset    Shelter: unset, unset    Deadline: none")
        self.lbl_tgt.pack(anchor="w", padx=6, pady=2)

        self.lbl_nav = ttk.Label(live, font=("TkDefaultFont", 12, "bold"),
                                 text="D= – m   BRG= – °   Head= – °   Herr= – °   XTE= – m   GO")
        self.lbl_nav.pack(anchor="w", padx=6, pady=4)

        self.lbl_arrow = ttk.Label(live, font=("Helvetica", 32))
        self.lbl_arrow.pack(anchor="center", pady=4)

        self.lbl_hint = ttk.Label(self, text="Tip: Targets accept placeholders like 51.4707A; set Vars as A=23,B=07")
        self.lbl_hint.pack(anchor="w", padx=8, pady=(0,6))

    # ---- actions ----
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        ports.sort()
        if sys.platform.startswith('darwin'):
            # prefer tty.* on macOS
            ports = [p for p in ports if '/tty.' in p] + [p for p in ports if '/tty.' not in p]
        self.port_menu['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle(self):
        if self.reader and self.reader.is_alive():
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Baud", "Invalid baud rate")
            return
        if not port:
            messagebox.showinfo("Port", "Select a serial port")
            return
        self.stop_evt.clear()
        self.reader = SerialReader(port, baud, self.q, self.stop_evt)
        self.reader.start()
        self.status_var.set(f"Connecting to {port}@{baud}…")
        self.btn.config(text="Disconnect")

    def _disconnect(self):
        self.status_var.set("Disconnecting…")
        self.stop_evt.set()
        if self.reader:
            self.reader.join(timeout=1.0)
        self.reader = None
        self.status_var.set("Disconnected")
        self.btn.config(text="Connect")

    def _apply_vars(self):
        text = self.vars_entry.get().strip()
        vm = {}
        if text:
            try:
                for pair in text.replace(' ', '').split(','):
                    if '=' in pair:
                        k, v = pair.split('=', 1)
                        vm[k.strip().upper()] = v.strip()
                self.varmap.update(vm)
            except Exception as e:
                messagebox.showerror("Vars", f"Parse error: {e}")
                return
        messagebox.showinfo("Vars", f"Vars now: {self.varmap}")

    def _set_target(self):
        la = self.tlat.get().strip(); lo = self.tlon.get().strip()
        try:
            self.target[0] = parse_coord(la, self.varmap) if la else None
            self.target[1] = parse_coord(lo, self.varmap) if lo else None
            self.start_fix = None  # reset XTE origin on new leg
        except Exception as e:
            messagebox.showerror("Target", f"Parse error: {e}")

    def _set_shelter(self):
        la = self.sh_lat.get().strip(); lo = self.sh_lon.get().strip()
        try:
            self.shelter[0] = parse_coord(la, self.varmap) if la else None
            self.shelter[1] = parse_coord(lo, self.varmap) if lo else None
        except Exception as e:
            messagebox.showerror("Shelter", f"Parse error: {e}")

    def _start_deadline(self):
        txt = self.deadline_entry.get().strip()
        if not txt:
            self.deadline_t = None
            return
        try:
            secs = float(txt)
            self.deadline_t = time.monotonic() + secs
        except Exception as e:
            messagebox.showerror("Deadline", f"Parse error: {e}")

    # ---- periodic UI tick ----
    def _ui_tick(self):
        # drain queue
        while True:
            try:
                msg = self.q.get_nowait()
            except queue.Empty:
                break
            if msg.get('type') == 'err':
                self.status_var.set(msg.get('msg','ERR'))
            elif msg.get('type') == 'fix':
                lat = msg['lat']; lon = msg['lon']
                cog = msg['cog']; spd = msg['spd']
                sats = msg['sats']; hdop = msg['hdop']
                ts = msg['ts']
                self.fix = (lat,lon,cog,spd,sats,hdop,ts)
                if self.start_fix is None:
                    self.start_fix = self.fix
                self.status_var.set("Connected")
        # update labels
        if self.fix:
            lat,lon,cog,spd,sats,hdop,ts = self.fix
            self.lbl_fix.config(text=f"Lat: {lat:.6f}   Lon: {lon:.6f}   HDOP: {hdop if hdop is not None else 'na'}   Sats: {sats if sats is not None else 'na'}   Speed: {spd:.2f} m/s" if spd is not None else f"Lat: {lat:.6f}   Lon: {lon:.6f}   HDOP: {hdop if hdop is not None else 'na'}   Sats: {sats if sats is not None else 'na'}   Speed: na")
            tgt = f"{self.target[0] if self.target[0] is not None else 'unset'}, {self.target[1] if self.target[1] is not None else 'unset'}"
            sh  = f"{self.shelter[0] if self.shelter[0] is not None else 'unset'}, {self.shelter[1] if self.shelter[1] is not None else 'unset'}"
            dl  = "set" if self.deadline_t else "none"
            self.lbl_tgt.config(text=f"Target: {tgt}    Shelter: {sh}    Deadline: {dl}")

            # choose active target (divert if needed)
            active = (self.target[0], self.target[1])
            divert = False
            now = time.monotonic()
            if self.deadline_t and self.shelter[0] is not None and active[0] is not None:
                dist_to_tgt = haversine_m(lat,lon,active[0],active[1])
                v = spd if (spd and spd > 0.1) else 1.0
                eta = dist_to_tgt / v if v > 0 else None
                time_left = self.deadline_t - now
                if eta and time_left < max(60.0, 1.5*eta):
                    active = (self.shelter[0], self.shelter[1])
                    divert = True

            if active[0] is not None and self.start_fix is not None:
                dist = haversine_m(lat, lon, active[0], active[1])
                brg  = initial_bearing_deg(lat, lon, active[0], active[1])
                xte  = cross_track_error_m(self.start_fix[0], self.start_fix[1], active[0], active[1], lat, lon)
                heading = cog if (cog is not None and cog >= 0.0) else None
                herr = None if heading is None else ((brg - heading + 540.0) % 360.0) - 180.0
                arrived = dist <= float(self.arrive_m.get())
                arrtxt = "   ARRIVED" if arrived else ""
                self.lbl_nav.config(text=f"D={dist:6.1f} m   BRG={brg:6.1f}°   Head={heading if heading is not None else 'na'}°   Herr={herr if herr is not None else 'na'}°   XTE={xte:5.1f} m   {'DIVERT' if divert else 'GO'}{arrtxt}")
                # arrow
                arrow = '·'
                if herr is not None:
                    a = ((herr + 360) % 360)
                    idx = int((a + 22.5) // 45) % 8
                    arrows = ['↑','↗','→','↘','↓','↙','←','↖']
                    arrow = arrows[idx]
                self.lbl_arrow.config(text=arrow)
            else:
                self.lbl_nav.config(text="No target set. Use Set Target.")
                self.lbl_arrow.config(text='·')
        else:
            self.lbl_fix.config(text="Waiting for GPS fix (RMC/GGA)…")

        self.after(50, self._ui_tick)

    # ---------------- ROS2 placeholders (add later) -------------------------
    # def _ros2_start(self):
    #     pass

    # def _ros2_publish_status(self):
    #     pass


if __name__ == '__main__':
    app = GPSNavGUI()
    app.mainloop()
