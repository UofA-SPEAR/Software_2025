#!/usr/bin/env python3
"""
GPS Navigator — ROS2 + Tkinter GUI (Base Station)

- Subscribes to rover topics (over network):
    * /fix                : sensor_msgs/NavSatFix (required)
    * /vel (optional)     : geometry_msgs/TwistStamped (linear.x, linear.y in ENU)
    * /hdop (optional)    : std_msgs/Float32 (HDOP)

- Displays: Lat/Lon, HDOP, Sats, Speed, Distance, Bearing, Heading Error, Cross-Track Error
- Lets operator set Target (supports placeholders like 51.4707A), Vars (A=23,B=07),
  Shelter, Arrival radius, and a Storm deadline (seconds) that triggers auto-divert to Shelter.
- All nav math is local to the GUI; rover just publishes its state.

Run (after sourcing ROS2 environment):
  python3 gps_nav_gui_ros2.py

Notes:
- The GUI does NOT open serial; it only uses ROS topics from the rover.
- Provide /fix via nmea_navsat_driver or your own GNSS node on the rover.
- Provide /vel if you want heading/speed (e.g., from wheel odom or GNSS velocity).

"""
import math, time, threading, queue, sys
import tkinter as tk
from tkinter import ttk, messagebox

# ROS2
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import NavSatFix
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import Float32
except Exception as e:
    print("[ROS2] Missing rclpy or message packages. Source your ROS2 setup.bash.", file=sys.stderr)
    raise

R_EARTH_M = 6371000.0

# ---------------- NAV HELPERS -----------------

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

# -------------- ROS NODE (SUBSCRIBER) --------------

class GPSSubNode(Node):
    def __init__(self, out_queue):
        super().__init__('gps_nav_gui')
        self.out_q = out_queue
        self._last_fix = None  # (lat, lon, sats)
        self._last_hdop = None
        self._last_vel_enu = (None, None)  # (vx, vy) m/s in ENU
        # QoS: use best-effort, keep-last
        self.create_subscription(NavSatFix, '/fix', self.fix_cb, 10)
        self.create_subscription(TwistStamped, '/vel', self.vel_cb, 10)
        self.create_subscription(Float32, '/hdop', self.hdop_cb, 10)

    def fix_cb(self, msg: NavSatFix):
        lat, lon = float(msg.latitude), float(msg.longitude)
        sats = None  # NavSatFix doesn't carry sats; left None unless provided elsewhere
        self._last_fix = (lat, lon, sats)
        # attempt HDOP fallback from covariance if driver encodes it (rare)
        hdop = None
        if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            # Very rough: sigma ≈ sqrt(var), HDOP ~ sigma_xy / nominal? Too driver-dependent; omit.
            pass
        # publish combined fix to GUI
        self._emit()

    def vel_cb(self, msg: TwistStamped):
        try:
            vx = float(msg.twist.linear.x)
            vy = float(msg.twist.linear.y)
        except Exception:
            vx = vy = None
        self._last_vel_enu = (vx, vy)
        self._emit()

    def hdop_cb(self, msg: Float32):
        self._last_hdop = float(msg.data)
        self._emit()

    def _emit(self):
        lat = lon = sats = None
        if self._last_fix:
            lat, lon, sats = self._last_fix
        vx, vy = self._last_vel_enu
        hdop = self._last_hdop
        self.out_q.put({
            'type': 'state',
            'lat': lat, 'lon': lon,
            'vx': vx, 'vy': vy,
            'hdop': hdop, 'sats': sats,
            'ts': time.monotonic()
        })

# ---------------------- GUI ----------------------

class GPSNavGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('GPS Navigator — ROS2 Base Station')
        self.geometry('920x360')
        self.minsize(860, 320)

        self.varmap = {}
        self.target = [None, None]
        self.shelter = [None, None]
        self.arrive_m = tk.DoubleVar(value=2.0)
        self.deadline_t = None

        self.state = None  # latest dict from node
        self.start_fix = None

        self.q = queue.Queue()
        self.executor = None
        self.node = None
        self.spin_thread = None

        self._build_ui()
        self.after(50, self._ui_tick)

    def _build_ui(self):
        pad = {'padx': 6, 'pady': 4}
        top = ttk.Frame(self)
        top.pack(fill='x', **pad)
        self.status_var = tk.StringVar(value='Disconnected — click Connect after sourcing ROS2')
        ttk.Button(top, text='Connect to ROS2', command=self._ros_connect).pack(side='left')
        ttk.Label(top, textvariable=self.status_var).pack(side='left', padx=8)

        frm = ttk.LabelFrame(self, text='Targets & Settings')
        frm.pack(fill='x', **pad)

        ttk.Label(frm, text='Target lat:').grid(row=0, column=0, sticky='e')
        self.tlat = ttk.Entry(frm, width=18); self.tlat.grid(row=0, column=1, sticky='w')
        ttk.Label(frm, text='lon:').grid(row=0, column=2, sticky='e')
        self.tlon = ttk.Entry(frm, width=18); self.tlon.grid(row=0, column=3, sticky='w')
        ttk.Button(frm, text='Set Target', command=self._set_target).grid(row=0, column=4, padx=6)

        ttk.Label(frm, text='Vars (A=23,B=07):').grid(row=1, column=0, sticky='e')
        self.vars_entry = ttk.Entry(frm, width=28); self.vars_entry.grid(row=1, column=1, columnspan=3, sticky='we')
        ttk.Button(frm, text='Apply Vars', command=self._apply_vars).grid(row=1, column=4, padx=6)

        ttk.Label(frm, text='Shelter lat:').grid(row=2, column=0, sticky='e')
        self.sh_lat = ttk.Entry(frm, width=18); self.sh_lat.grid(row=2, column=1, sticky='w')
        ttk.Label(frm, text='lon:').grid(row=2, column=2, sticky='e')
        self.sh_lon = ttk.Entry(frm, width=18); self.sh_lon.grid(row=2, column=3, sticky='w')
        ttk.Button(frm, text='Set Shelter', command=self._set_shelter).grid(row=2, column=4, padx=6)

        ttk.Label(frm, text='Arrive (m):').grid(row=3, column=0, sticky='e')
        ttk.Entry(frm, textvariable=self.arrive_m, width=10).grid(row=3, column=1, sticky='w')
        ttk.Label(frm, text='Deadline (s):').grid(row=3, column=2, sticky='e')
        self.deadline_entry = ttk.Entry(frm, width=10); self.deadline_entry.grid(row=3, column=3, sticky='w')
        ttk.Button(frm, text='Start Timer', command=self._start_deadline).grid(row=3, column=4, padx=6)
        for i in range(5):
            frm.grid_columnconfigure(i, weight=1)

        live = ttk.LabelFrame(self, text='Live')
        live.pack(fill='both', expand=True, **pad)
        self.lbl_fix = ttk.Label(live, text='Lat: –  Lon: –   HDOP: –  Sats: –  Speed: – m/s')
        self.lbl_fix.pack(anchor='w', padx=6, pady=2)
        self.lbl_tgt = ttk.Label(live, text='Target: unset, unset    Shelter: unset, unset    Deadline: none')
        self.lbl_tgt.pack(anchor='w', padx=6, pady=2)
        self.lbl_nav = ttk.Label(live, font=('TkDefaultFont', 12, 'bold'),
                                 text='D= – m   BRG= – °   Head= – °   Herr= – °   XTE= – m   GO')
        self.lbl_nav.pack(anchor='w', padx=6, pady=4)
        self.lbl_arrow = ttk.Label(live, font=('Helvetica', 32))
        self.lbl_arrow.pack(anchor='center', pady=4)
        self.lbl_hint = ttk.Label(self, text='Tip: set Vars like A=23,B=07; targets accept 51.4707A placeholders')
        self.lbl_hint.pack(anchor='w', padx=8, pady=(0,6))

        self.protocol('WM_DELETE_WINDOW', self._on_close)

    # ---------- actions ----------
    def _ros_connect(self):
        if self.node:
            self.status_var.set('Already connected')
            return
        try:
            rclpy.init()
            self.node = GPSSubNode(self.q)
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.spin_thread.start()
            self.status_var.set('Connected to ROS2 — waiting for /fix')
        except Exception as e:
            messagebox.showerror('ROS2', f'Failed to init ROS2: {e}')

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
                messagebox.showerror('Vars', f'Parse error: {e}')
                return
        messagebox.showinfo('Vars', f'Vars now: {self.varmap}')

    def _set_target(self):
        la = self.tlat.get().strip(); lo = self.tlon.get().strip()
        try:
            self.target[0] = self._parse_coord(la)
            self.target[1] = self._parse_coord(lo)
            self.start_fix = None  # reset XTE origin
        except Exception as e:
            messagebox.showerror('Target', f'Parse error: {e}')

    def _set_shelter(self):
        la = self.sh_lat.get().strip(); lo = self.sh_lon.get().strip()
        try:
            self.shelter[0] = self._parse_coord(la)
            self.shelter[1] = self._parse_coord(lo)
        except Exception as e:
            messagebox.showerror('Shelter', f'Parse error: {e}')

    def _parse_coord(self, s):
        if not s:
            return None
        s = s.strip()
        if any(ch.isalpha() for ch in s):
            # substitute placeholders
            out = []
            for ch in s:
                if ch.isalpha() and ch.upper() in self.varmap:
                    out.append(str(self.varmap[ch.upper()]))
                else:
                    out.append(ch)
            return float(''.join(out))
        return float(s)

    def _start_deadline(self):
        txt = self.deadline_entry.get().strip()
        if not txt:
            self.deadline_t = None
            return
        try:
            secs = float(txt)
            self.deadline_t = time.monotonic() + secs
        except Exception as e:
            messagebox.showerror('Deadline', f'Parse error: {e}')

    # ---------- periodic tick ----------
    def _ui_tick(self):
        # drain queue
        while True:
            try:
                msg = self.q.get_nowait()
            except queue.Empty:
                break
            if msg.get('type') == 'state':
                self.state = msg
        # update labels
        if self.state:
            lat = self.state.get('lat'); lon = self.state.get('lon')
            vx = self.state.get('vx'); vy = self.state.get('vy')
            hdop = self.state.get('hdop'); sats = self.state.get('sats')
            spd = None
            if vx is not None and vy is not None:
                spd = (vx**2 + vy**2) ** 0.5
                # ENU -> course from North, clockwise: atan2(East, North)
                cog = (math.degrees(math.atan2(vx, vy)) + 360.0) % 360.0
            else:
                cog = None
            if lat is not None and lon is not None:
                self.lbl_fix.config(text=f"Lat: {lat:.6f}   Lon: {lon:.6f}   HDOP: {hdop if hdop is not None else 'na'}   Sats: {sats if sats is not None else 'na'}   Speed: {spd:.2f} m/s" if spd is not None else f"Lat: {lat:.6f}   Lon: {lon:.6f}   HDOP: {hdop if hdop is not None else 'na'}   Sats: {sats if sats is not None else 'na'}   Speed: na")
            else:
                self.lbl_fix.config(text='Waiting for /fix…')

            tgt_txt = f"{self.target[0] if self.target[0] is not None else 'unset'}, {self.target[1] if self.target[1] is not None else 'unset'}"
            sh_txt  = f"{self.shelter[0] if self.shelter[0] is not None else 'unset'}, {self.shelter[1] if self.shelter[1] is not None else 'unset'}"
            dl_txt  = 'set' if self.deadline_t else 'none'
            self.lbl_tgt.config(text=f"Target: {tgt_txt}    Shelter: {sh_txt}    Deadline: {dl_txt}")

            # nav calc
            if lat is not None and lon is not None and self.target[0] is not None:
                if self.start_fix is None:
                    self.start_fix = (lat, lon)
                active = (self.target[0], self.target[1])
                divert = False
                now = time.monotonic()
                if self.deadline_t and self.shelter[0] is not None:
                    dist_to_tgt = haversine_m(lat, lon, active[0], active[1])
                    v = spd if (spd and spd > 0.1) else 1.0
                    eta = dist_to_tgt / v if v > 0 else None
                    time_left = self.deadline_t - now
                    if eta and time_left < max(60.0, 1.5*eta):
                        active = (self.shelter[0], self.shelter[1])
                        divert = True
                dist = haversine_m(lat, lon, active[0], active[1])
                brg  = initial_bearing_deg(lat, lon, active[0], active[1])
                xte  = cross_track_error_m(self.start_fix[0], self.start_fix[1], active[0], active[1], lat, lon)
                heading = cog
                herr = None if heading is None else ((brg - heading + 540.0) % 360.0) - 180.0
                arrived = dist <= float(self.arrive_m.get())
                arrtxt = '   ARRIVED' if arrived else ''
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
                self.lbl_nav.config(text='No target set. Use Set Target.')
                self.lbl_arrow.config(text='·')
        else:
            self.lbl_fix.config(text='Waiting for ROS2 connection…')
        self.after(50, self._ui_tick)

    def _on_close(self):
        try:
            if self.executor:
                self.executor.shutdown(timeout_sec=0.2)
            if self.node:
                self.node.destroy_node()
            rclpy.shutdown(context=None)
        except Exception:
            pass
        self.destroy()

# ---------------------- main ----------------------
if __name__ == '__main__':
    app = GPSNavGUI()
    app.mainloop()
