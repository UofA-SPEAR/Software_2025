#!/bin/bash

mkdir -p /tmp/gst_frames

while true; do
  gst-launch-1.0 zedsrc camera-sn=31826195 ! timeoverlay ! tee name=split has-chain=true ! \
  queue ! autovideoconvert ! fpsdisplaysink \
  split. ! queue max-size-time=0 max-size-bytes=0 max-size-buffers=0 ! autovideoconvert ! \
  x264enc byte-stream=true tune=zerolatency speed-preset=ultrafast bitrate=3000 ! \
  h264parse ! rtph264pay config-interval=-1 pt=96 ! queue ! \
  udpsink clients=192.168.1.10:5000 max-bitrate=3000000 sync=false async=false

  echo "Pipeline exited. Restarting in 5 seconds..."
  sleep 5
done