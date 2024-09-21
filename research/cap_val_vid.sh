#!/bin/bash

TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
VIDEO_FILE="$TIMESTAMP-validation.h264"
TIME=30

while getopts t: flag
do
    case "${flag}" in
    	t) TIME=${OPTARG};;
        *) echo "Usage: $0 -t <video_time>"
           exit 1;;
    esac
done

# Capture video
rpicam-vid -t $TIME --framerate 30 --width 1280 --height 720 --inline --listen -o $VIDEO_FILE
