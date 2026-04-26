#!/bin/bash

LOG_FILE="thermal_data.csv"

# 如果文件不存在，写入表头
if [ ! -f "$LOG_FILE" ]; then
    echo "Timestamp,Freq_MHz,Temp_C" > "$LOG_FILE"
fi

echo "开始记录温度日志到 $LOG_FILE (按 Ctrl+C 停止)..."

while true; do
    # 获取数据
    TIMESTAMP=$(date +"%Y-%m-%d %H:%M:%S")
    TEMP=$(vcgencmd measure_temp | egrep -o '[0-9]*\.[0-9]*')
    FREQ=$(vcgencmd measure_clock arm | awk -F= '{printf "%d", $2/1000000}')
    
    # 写入 CSV
    echo "$TIMESTAMP,$FREQ,$TEMP" >> "$LOG_FILE"
    
    # 同时在控制台打印方便实时查看
    echo "[$TIMESTAMP] Freq: ${FREQ}MHz, Temp: ${TEMP}C"
    
    sleep 2
done