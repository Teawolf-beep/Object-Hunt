#!/bin/bash

st -e bash -c './mapping.bash' &
st -e bash -c './camera.bash' &

ssh pi@10.0.0.1

