# telemetry_reader.py
import time, re
import serial
from serial.tools import list_ports
from collections import deque

BAUD = 115200
PERIOD = 2.0  # seconds between prints

# ----- helpers (same GPS parsing as before) -----
def auto_port():
    for p in list_ports.comports():
        name = (p.manufacturer or "") + " " + (p.description or "")
        if "ST" in name or "Nucleo" in name or "STM" in name:
            return p.device
    for p in list_ports.comports():
        if any(k in p.device for k in ("ACM", "USB", "COM")):
            return p.device
    raise RuntimeError("No serial ports found")

def dm_to_deg(dm, hemi):
    if not dm or not hemi: return None
    v = float(dm); deg = int(v // 100); minutes = v - deg * 100
    out = deg + minutes / 60.0
    return -out if hemi in ("S","W") else out

def hhmmss_to_str(h):
    if not h: return None
    hh = int(h[0:2]); mm = int(h[2:4]); ss = int(float(h[4:])) if len(h)>4 else 0
    return f"{hh:02d}:{mm:02d}:{ss:02d}"

FIXQ = {"0":"No fix","1":"GPS","2":"DGPS","4":"RTK fixed","5":"RTK float","6":"DR"}

class GPS:
    def __init__(self): self.utc=self.lat=self.lon=self.alt=self.sats=self.fixq=None; self.spd=self.cog=None
    def pretty(self):
        lat = f"{self.lat:.6f}" if self.lat is not None else "?"
        lon = f"{self.lon:.6f}" if self.lon is not None else "?"
        alt = f"{self.alt:.1f} m" if self.alt is not None else "?"
        sats = f"{self.sats}" if self.sats is not None else "?"
        fix = FIXQ.get(self.fixq or "", self.fixq or "?")
        spd = f"{self.spd:.1f} km/h" if self.spd is not None else "-"
        cog = f"{self.cog}°" if self.cog is not None else "-"
        t = self.utc or "--:--:--"
        return f"[UTC {t}] Lat {lat}, Lon {lon} | Alt {alt} | Sats {sats} | Fix {fix} | Speed {spd} | Course {cog}"

def handle_nmea(line, g: GPS):
    f = line.split(",")
    if line.startswith(("$GPGGA","$GNGGA","$GAGGA")) or ",GGA," in line:
        g.utc  = hhmmss_to_str(f[1]) if len(f)>1 else g.utc
        g.lat  = dm_to_deg(f[2], f[3]) if len(f)>5 else g.lat
        g.lon  = dm_to_deg(f[4], f[5]) if len(f)>7 else g.lon
        g.fixq = f[6] if len(f)>6 else g.fixq
        g.sats = int(f[7]) if len(f)>7 and f[7].isdigit() else g.sats
        try: g.alt = float(f[9]) if len(f)>9 and f[9] else g.alt
        except: pass
    elif line.startswith(("$GPRMC","$GNRMC","$GARMC")) or ",RMC," in line:
        g.utc  = hhmmss_to_str(f[1]) if len(f)>1 else g.utc
        g.lat  = dm_to_deg(f[3], f[4]) if len(f)>6 else g.lat
        g.lon  = dm_to_deg(f[5], f[6]) if len(f)>8 else g.lon
        try: g.spd = float(f[7])*1.852 if len(f)>7 and f[7] else g.spd
        except: pass
        g.cog  = f[8] if len(f)>8 and f[8] else g.cog

# first float in a line (for O3/H2 convenience)
_FLOAT = re.compile(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?")
def first_float(s):
    m = _FLOAT.search(s or "")
    return float(m.group()) if m else None

def main():
    port = auto_port()
    print(f"Opening {port} @ {BAUD} … (Ctrl+C to quit)")
    g = GPS()
    geiger = deque()  # (timestamp, value_uSvph)
    last_o = {"raw": None, "val": None}  # O3 (or O2)
    last_h = {"raw": None, "val": None}  # H2
    last_print = 0.0

    with serial.Serial(port, BAUD, timeout=1) as ser:
        ser.reset_input_buffer()
        while True:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            if line.startswith("G:"):
                payload = line[2:].strip()
                if payload.startswith("$"):
                    handle_nmea(payload, g)
            elif line.startswith("R:"):  # Geiger µSv/h, one float per line
                try:
                    v = float(line[2:].strip())
                    now = time.time()
                    geiger.append((now, v))
                    # keep last ~3 s
                    while geiger and now - geiger[0][0] > 3.0:
                        geiger.popleft()
                except: pass
            elif line.startswith("O:"):   # O3 (or O2) line from DGS2
                last_o["raw"] = line[2:].strip()
                last_o["val"] = first_float(last_o["raw"])
            elif line.startswith("H:"):   # H2 line from DGS2
                last_h["raw"] = line[2:].strip()
                last_h["val"] = first_float(last_h["raw"])

            now = time.time()
            if now - last_print >= PERIOD:
                # Geiger average over last 2 s
                vals = [v for t,v in geiger if now - t <= 2.0]
                g_avg = sum(vals)/len(vals) if vals else None
                g_txt = f"{g_avg:.3f} µSv/h (2s avg)" if g_avg is not None else "—"
                o_txt = f"{last_o['val']}" if last_o["val"] is not None else (last_o["raw"] or "—")
                h_txt = f"{last_h['val']}" if last_h["val"] is not None else (last_h["raw"] or "—")

                print(g.pretty())
                print(f"Geiger: {g_txt}")
                print(f"O3/O2: {o_txt}")
                print(f"H2:     {h_txt}")
                print("-"*70)
                last_print = now

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nBye.")
