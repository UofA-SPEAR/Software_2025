import serial
import pynmea2
import time
from datetime import datetime

def read_gps_data():
    try:
        ser = serial.Serial('/dev/ttyTHS1', 9600, timeout=1)
        print(f"✓ GPS module connected on /dev/ttyTHS1")
        print("📍 GPS Status Monitor")
        print("=" * 50)
        print("Move to outdoor location with clear sky view!")
        print("First fix can take 5-15 minutes...")
        print("Press Ctrl+C to stop\n")
    except Exception as e:
        print(f"❌ Error connecting to GPS: {e}")
        return
    
    satellites_gps = 0
    satellites_glonass = 0
    last_status_time = time.time()
    
    while True:
        try:
            line = ser.readline().decode('ascii', errors='replace').strip()
            
            if line.startswith('$GNGGA'):
                try:
                    msg = pynmea2.parse(line)
                    fix_quality = int(msg.gps_qual) if msg.gps_qual else 0
                    num_sats = int(msg.num_sats) if msg.num_sats else 0
                    
                    if msg.latitude and msg.longitude:
                        print(f"🎯 GPS FIX ACQUIRED!")
                        print(f"📍 Lat: {msg.latitude:.6f}, Lon: {msg.longitude:.6f}")
                        print(f"⛰️  Altitude: {msg.altitude} {msg.altitude_units}")
                        print(f"🛰️  Satellites: {num_sats}")
                        print(f"📶 Fix Quality: {fix_quality}")
                        print(f"⏰ Time: {datetime.now().strftime('%H:%M:%S')}")
                        print("-" * 40)
                    else:
                        # Show status every 10 seconds
                        if time.time() - last_status_time > 10:
                            print(f"⏳ Searching... Sats: GPS({satellites_gps}) GLONASS({satellites_glonass}) Total({num_sats}) | {datetime.now().strftime('%H:%M:%S')}")
                            last_status_time = time.time()
                            
                except pynmea2.ParseError:
                    pass
                    
            elif line.startswith('$GPGSV'):
                # GPS satellite info
                try:
                    msg = pynmea2.parse(line)
                    if msg.msg_num == 1:  # First message has total count
                        satellites_gps = int(msg.num_sv_in_view) if msg.num_sv_in_view else 0
                except:
                    pass
                    
            elif line.startswith('$GLGSV'):
                # GLONASS satellite info  
                try:
                    msg = pynmea2.parse(line)
                    if msg.msg_num == 1:
                        satellites_glonass = int(msg.num_sv_in_view) if msg.num_sv_in_view else 0
                except:
                    pass
                    
            elif line.startswith('$GNRMC'):
                try:
                    msg = pynmea2.parse(line)
                    if msg.status == 'A':  # Active/Valid
                        print(f"🚀 Speed: {msg.spd_over_grnd} knots")
                        print(f"🧭 Course: {msg.true_course}°")
                except:
                    pass
                    
        except KeyboardInterrupt:
            print("\n🛑 Stopping GPS monitoring...")
            break
        except Exception as e:
            print(f"❌ Error: {e}")
            time.sleep(1)
    
    ser.close()

if __name__ == "__main__":
    read_gps_data()