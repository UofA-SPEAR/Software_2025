#!/bin/bash

# Define the port where the stream will be received
PORT=5040

# Run the GStreamer pipeline to receive the stream
gst-launch-1.0 udpsrc port=$PORT caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink