#!/usr/bin/env python3
"""
Jetson ROS2 bridge for a USB GPS that outputs NMEA (RMC/GGA).

Publishes:
  /fix   : sensor_msgs/msg/NavSatFix         (lat/lon/alt + approx covariance)
  /vel   : geometry_msgs/msg/TwistStamped    (linear.x=East m/s, linear.y=North m/s) [if RMC speed/course present]
  /hdop  : std_msgs/msg/Float32              (from GGA)

Parameters (ROS2):
  port     (string, default '/dev/ttyUSB0')
  baud     (int,    default 9600)
  frame_id (string, default 'gps')
  uere_m   (float,  default 5.0)   # assumed user equivalent range error for covariance (~5 m for consumer GPS)

Run:
  source /opt/ros/<distro>/setup.bash       # e.g., humble, jazzy
  python3 gps_rover_serial.py --ros-args -p port:=/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_7_-_GPS_GNSS_Receiver-if00  -p baud:=9600 -p frame_id:=gps

Dependencies:
  pip3 install pyserial

Notes:
  - Works with talkers GP/GN/GA/GL, etc.
  - If port is busy on Ubuntu/Jetson, consider:  sudo systemctl disable --now ModemManager
"""
import math, time, threading, queue, sys
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32

try:
    import serial
except Exception as e:
    print("pyserial required. Install with: python3 -m pip install pyserial", file=sys.stderr)
    raise

R_EARTH_M = 6371000.0

# ------------------- NMEA helpers -------------------

def nmea_deg(val: str, hemi: str):
    if not val or '.' not in val:
        return None
    dot = val.find('.')
    deg_len = 3 if len(val[:dot]) >= 5 else 2  # lon typically 3 deg digits
    try:
        deg = float(val[:deg_len])
        minutes = float(val[deg_len:])
    except ValueError:
        return None
    dec = deg + minutes/60.0
    if hemi in ('S','W'):
        dec = -dec
    return dec

def parse_rmc(fields):
    # $--RMC,hhmmss,A,llll.ll,a,yyyyy.yy,a,sog,cog,ddmmyy,...
    if len(fields) < 12 or fields[2] != 'A':
        return None
    lat = nmea_deg(fields[3], fields[4])
    lon = nmea_deg(fields[5], fields[6])
    sog = float(fields[7])*0.514444 if fields[7] else None  # knots -> m/s
    cog = float(fields[8]) if fields[8] else None           # deg true
    return lat, lon, sog, cog

def parse_gga(fields):
    # $--GGA,hhmmss,llll.ll,a,yyyyy.yy,a,fix,numsv,hdop,alt,M,geoid,...
    if len(fields) < 11:
        return None
    fixq = fields[6]
    if fixq in ('0',''):
        return None
    lat = nmea_deg(fields[2], fields[3])
    lon = nmea_deg(fields[4], fields[5])
    sats = int(fields[7]) if fields[7].isdigit() else None
    try:
        hdop = float(fields[8]) if fields[8] else None
    except Exception:
        hdop = None
    try:
        alt = float(fields[9]) if fields[9] else None
    except Exception:
        alt = None
    return lat, lon, sats, hdop, alt

# -------------------- Serial reader thread --------------------

class SerialReader(threading.Thread):
    def __init__(self, port, baud, out_q, stop_evt):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.q = out_q
        self.stop_evt = stop_evt
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
        except Exception as e:
            self.q.put({'type':'err','msg':f'Serial open failed: {e}'})
            return
        buf = bytearray()
        last_rmc = None
        last_gga = None
        while not self.stop_evt.is_set():
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
                        s = line.decode('utf-8', 'ignore')
                        f = s.split(',')
                        if len(f[0]) < 6:
                            continue
                        typ = f[0][3:6]  # RMC/GGA/...
                        tnow = time.monotonic()
                        if typ == 'RMC':
                            r = parse_rmc(f)
                            if r:
                                last_rmc = r  # (lat,lon,sog,cog)
                        elif typ == 'GGA':
                            g = parse_gga(f)
                            if g:
                                last_gga = g  # (lat,lon,sats,hdop,alt)
                        # merge best known into a state sample
                        lat = lon = sog = cog = sats = hdop = alt = None
                        if last_rmc:
                            lat, lon, sog, cog = last_rmc
                        if last_gga:
                            lat, lon, sats, hdop, alt = last_gga
                        if lat is not None and lon is not None:
                            self.q.put({
                                'type':'fix',
                                'lat':lat, 'lon':lon,
                                'sog':sog, 'cog':cog, 'alt':alt,
                                'sats':sats, 'hdop':hdop,
                                't': tnow
                            })
            except Exception as e:
                self.q.put({'type':'err','msg':str(e)})
                time.sleep(0.1)
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass

# -------------------- ROS2 Node --------------------

class GpsNmeaBridge(Node):
    def __init__(self):
        super().__init__('gps_nmea_bridge')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('frame_id', 'gps')
        self.declare_parameter('uere_m', 5.0)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = int(self.get_parameter('baud').get_parameter_value().integer_value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.uere_m = float(self.get_parameter('uere_m').get_parameter_value().double_value)

        self.fix_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.vel_pub = self.create_publisher(TwistStamped, 'vel', 10)
        self.hdop_pub = self.create_publisher(Float32, 'hdop', 10)

        self.q = queue.Queue()
        self.stop_evt = threading.Event()
        self.reader = SerialReader(self.port, self.baud, self.q, self.stop_evt)
        self.reader.start()

        self.timer = self.create_timer(0.02, self._pump)  # 50 Hz pump
        self.last_sample = None
        self.get_logger().info(f"Reading NMEA from {self.port}@{self.baud}, frame_id='{self.frame_id}'")

    def destroy_node(self):
        try:
            self.stop_evt.set()
            if self.reader.is_alive():
                self.reader.join(timeout=1.0)
        except Exception:
            pass
        super().destroy_node()

    def _ros_now(self) -> Time:
        return self.get_clock().now()

    def _pump(self):
        # Drain queue fully each tick
        while True:
            try:
                msg = self.q.get_nowait()
            except queue.Empty:
                break
            if msg['type'] == 'err':
                self.get_logger().warn(msg['msg'])
            elif msg['type'] == 'fix':
                self.last_sample = msg
                self._publish(msg)

    def _publish(self, s):
        lat = s.get('lat'); lon = s.get('lon'); alt = s.get('alt')
        sog = s.get('sog'); cog = s.get('cog')
        sats = s.get('sats'); hdop = s.get('hdop')
        now = self._ros_now().to_msg()

        m = NavSatFix()
        m.header.stamp = now
        m.header.frame_id = self.frame_id
        m.status.service = NavSatStatus.SERVICE_GPS
        m.status.status = NavSatStatus.STATUS_NO_FIX
        if lat is not None and lon is not None:
            m.latitude = float(lat)
            m.longitude = float(lon)
            m.status.status = NavSatStatus.STATUS_FIX
        m.altitude = float(alt) if (alt is not None) else float('nan')

        # covariance estimate from HDOP and UERE
        if hdop is not None and hdop > 0.0:
            sigma_h = hdop * max(1.0, float(self.uere_m))  # meters
            var_h = sigma_h * sigma_h
            m.position_covariance = [var_h, 0.0, 0.0,
                                     0.0, var_h, 0.0,
                                     0.0, 0.0, 4.0*var_h]  # Z variance ~ 2x sigma
            m.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            m.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.fix_pub.publish(m)

        # hdop topic
        if hdop is not None:
            h = Float32(); h.data = float(hdop)
            self.hdop_pub.publish(h)

        # velocity from RMC: convert to ENU (East, North)
        if sog is not None and cog is not None:
            cr = math.radians(cog)
            vx = sog * math.sin(cr)  # East
            vy = sog * math.cos(cr)  # North
            t = TwistStamped()
            t.header.stamp = now
            t.header.frame_id = self.frame_id
            t.twist.linear.x = float(vx)
            t.twist.linear.y = float(vy)
            self.vel_pub.publish(t)


def main():
    rclpy.init()
    node = GpsNmeaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
