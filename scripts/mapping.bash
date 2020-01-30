#!/bin/bash

cd ../object_hunt_mapping
rm *.log > /dev/null 2>&1
rm *.png > /dev/null 2>&1
java -jar ams_mapping.jar 10.0.0.1

read -p "All done, press any key to close the window" x