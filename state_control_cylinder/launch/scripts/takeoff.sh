#!/bin/bash

# Record
echo "Pressing Record..."
rosrun korg_nanokontrol2 kontrol_sim.py 28

# Play
echo "Pressing Play..."
rosrun korg_nanokontrol2 kontrol_sim.py 27

# Set (hover)
echo "Pressing Set (to hover)..."
rosrun korg_nanokontrol2 kontrol_sim.py 31

# Line Tracker to {0, 0, 1}
echo "Line Tracker to {0, 0, 1}"
rosrun korg_nanokontrol2 kontrol_sim.py 29 0 0 1
