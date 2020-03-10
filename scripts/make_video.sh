#!/usr/bin/env bash

ffmpeg -r 30 -f image2 -i "$1/output_%d.png" -vcodec libx264 -crf 18 "$2"
