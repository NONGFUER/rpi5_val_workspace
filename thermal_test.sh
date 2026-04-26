#!/bin/bash
echo "Time, Freq(MHz), Temp(C)"
while true; do
    TEMP=$(vcgencmd measure_temp | egrep -o '[0-9]*\.[0-9]*')
    FREQ=$(vcgencmd measure_clock arm | awk -F= '{printf "%d", $2/1000000}')
    echo "$(date +%H:%M:%S), $FREQ, $TEMP"
    sleep 2
done