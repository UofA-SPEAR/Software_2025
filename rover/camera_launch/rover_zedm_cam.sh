#!/bin/bash

# Replace with the actual USB identifier for the Arducam
# You can find it with: ls -l /dev/v4l/by-id/
SERIAL_NUMBER="_ZED-M"  

# Find the primary video device (index0) for this camera
DEVICE_PATH=$(find /dev/v4l/by-id/ -name "*$SERIAL_NUMBER*-video-index0" | head -n 1)

if [ -z "$DEVICE_PATH" ]; then
    echo "No device found for: $SERIAL_NUMBER"
    echo "Run 'ls -l /dev/v4l/by-id/' to check the exact name"
    exit 1
fi

echo "Using camera at: $DEVICE_PATH"

# Stream over UDP to the base station
gst-launch-1.0 v4l2src device="$DEVICE_PATH" ! \
    videoconvert ! \
    x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! \
    rtph264pay config-interval=1 pt=96 ! \
    udpsink host=192.168.1.10 port=5065