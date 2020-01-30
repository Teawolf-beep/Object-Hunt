#!/bin/bash

source source /path/to/your/virtual/env/bin/activate
cd ../object_hunt_camera
python server.py -mW 1 -mH 1

read -p "All done, press any key to close the window" x