#!/usr/bin/env python3

import cv2
import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import threading
import time

class GStreamerArUcoDetector:
    def __init__(self, port=5065):
        Gst.init(None)
        
        self.port = port
        self.frame = None
        self.frame_lock = threading.Lock()
        self.running = False
        
        # ArUco detector setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        self.setup_pipeline()
    
    def setup_pipeline(self):
        pipeline_str = f"""
        udpsrc port={self.port} caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96"
        ! rtph264depay
        ! avdec_h264
        ! videoconvert
        ! video/x-raw,format=BGR
        ! appsink name=sink emit-signals=true sync=false max-buffers=2 drop=true
        """
        
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name('sink')
        self.appsink.connect('new-sample', self.on_new_sample)
    
    def on_new_sample(self, appsink):
        sample = appsink.emit('pull-sample')
        if sample:
            buffer = sample.get_buffer()
            caps = sample.get_caps()
            
            # Get video info
            structure = caps.get_structure(0)
            width = structure.get_value('width')
            height = structure.get_value('height')
            
            # Convert to numpy array
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if success:
                frame_data = np.frombuffer(map_info.data, dtype=np.uint8)
                frame = frame_data.reshape((height, width, 3))
                
                with self.frame_lock:
                    self.frame = frame.copy()
                
                buffer.unmap(map_info)
        
        return Gst.FlowReturn.OK
    
    def detect_aruco(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None:
            for i, corner in enumerate(corners):
                # Get corner points
                pts = corner[0].astype(int)
                
                # Draw green rectangle
                cv2.polylines(frame, [pts], True, (0, 255, 0), 3)
                
                # Put ID text
                center = pts.mean(axis=0).astype(int)
                cv2.putText(frame, f"ID: {ids[i][0]}", 
                           (center[0] - 30, center[1] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        return frame
    
    def start(self):
        self.running = True
        self.pipeline.set_state(Gst.State.PLAYING)
        
        print(f"Starting ArUco detection on port {self.port}")
        
        while self.running:
            with self.frame_lock:
                if self.frame is not None:
                    display_frame = self.frame.copy()
                else:
                    display_frame = None
            
            if display_frame is not None:
                processed_frame = self.detect_aruco(display_frame)
                cv2.imshow('ArUco Detection', processed_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            
            time.sleep(0.01)
        
        self.stop()
    
    def stop(self):
        self.running = False
        self.pipeline.set_state(Gst.State.NULL)
        cv2.destroyAllWindows()

def main():
    detector = GStreamerArUcoDetector()  # Arm camera port
    
    try:
        detector.start()
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        detector.stop()

if __name__ == '__main__':
    main()