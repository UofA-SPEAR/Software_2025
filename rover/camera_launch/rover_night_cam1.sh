#!/bin/bash

DEVICE_PATH="/dev/video0"

if [ ! -e "$DEVICE_PATH" ]; then
    echo "No device found at: $DEVICE_PATH"
    echo "Run 'ls -l /dev/video*' to check available devices"
    exit 1
fi

echo "Using camera at: $DEVICE_PATH"

gst-launch-1.0 v4l2src device="$DEVICE_PATH" ! \
    videoconvert ! \
    x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! \
    rtph264pay config-interval=1 pt=96 ! \
    udpsink host=192.168.1.10 port=5070