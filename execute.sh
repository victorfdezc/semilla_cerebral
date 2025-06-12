#!/bin/bash
export PYTHONPATH=/usr/local/lib/python3/dist-packages:$PYTHONPATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

python3 detect_person_kinect_yolo.py