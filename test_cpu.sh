#!/bin/bash
LOG_FILE="pi5_stress_report.log"

echo "=== 树莓派 5 性能与温度压测开始 ===" | tee $LOG_FILE
echo "测试时间: $(date)" | tee -a $LOG_FILE
echo "------------------------------------" | tee -a $LOG_FILE

# 1. 记录空载温度和占有率
echo "【阶段 1：空载状态读取】" | tee -a $LOG_FILE
sleep 3
# 通过 vcgencmd 和 top 抓取初始状态
IDLE_TEMP=$(vcgencmd measure_temp | grep -o '[0-9.]*')
IDLE_CPU=$(top -bn1 | grep "Cpu(s)" | sed "s/.*, *\([0-9.]*\)%* id.*/\1/" | awk '{print 100 - $1}')
echo "等待时 CPU 占有率: $IDLE_CPU %" | tee -a $LOG_FILE
echo "等待时 CPU 温度: $IDLE_TEMP °C" | tee -a $LOG_FILE
echo "------------------------------------" | tee -a $LOG_FILE

# 2. 满载压力测试 (5分钟) 并实时监听
echo "【阶段 2：正在进行 5 分钟全核满载极限高压测试 (stress-ng)...】" | tee -a $LOG_FILE
echo "期间将每隔 10 秒记录一次 CPU 的实时占有率和核心温度..." | tee -a $LOG_FILE

# 启动跑满 4 核的压测进程，挂在后台
stress-ng --cpu 4 --cpu-method all --timeout 5m &
STRESS_PID=$!

MAX_TEMP=0
AVERAGE_CPU=0
RECORD_COUNT=0

# 在压力测试进程活着的时候，持续死循环抓取数据
while kill -0 $STRESS_PID 2>/dev/null; do
    # 抓取当前主板温度
    CURRENT_TEMP=$(vcgencmd measure_temp | grep -o '[0-9.]*')
    # 抓取此刻的 CPU 占有率 (100 - idle%)
    CURRENT_CPU=$(top -bn1 | grep "Cpu(s)" | sed "s/.*, *\([0-9.]*\)%* id.*/\1/" | awk '{print 100 - $1}')
    
    echo "[实时监测] CPU占用: ${CURRENT_CPU}% | 核心温度: ${CURRENT_TEMP}°C" | tee -a $LOG_FILE

    # 找最高温度
    if (( $(echo "$CURRENT_TEMP > $MAX_TEMP" | bc -l) )); then
        MAX_TEMP=$CURRENT_TEMP
    fi
    
    # 累加 CPU 数据用来算平均负载
    AVERAGE_CPU=$(echo "$AVERAGE_CPU + $CURRENT_CPU" | bc -l)
    RECORD_COUNT=$((RECORD_COUNT + 1))
    
    sleep 10
done

# 如果有拿到数据，算出整个5分钟内的平均占有率
if [ $RECORD_COUNT -gt 0 ]; then
    FINAL_AVG_CPU=$(echo "scale=2; $AVERAGE_CPU / $RECORD_COUNT" | bc -l)
else
    FINAL_AVG_CPU="100.00"
fi

echo "------------------------------------" | tee -a $LOG_FILE
echo "【阶段 3：压测结果统计汇总】" | tee -a $LOG_FILE
echo "满载期间平均 CPU 占有率: $FINAL_AVG_CPU %" | tee -a $LOG_FILE
echo "满载 5分钟内峰值温度: $MAX_TEMP °C" | tee -a $LOG_FILE
echo "------------------------------------" | tee -a $LOG_FILE

# 检测是否发生过热硬降频
if (( $(echo "$MAX_TEMP >= 82.0" | bc -l) )); then
    echo "⚠️ 警告：峰值温度越过了 82°C 的软温控墙！处理器已被迫物理降频(Throttling)，影响 AI 收银速度，必须升级散热！" | tee -a $LOG_FILE
else
    echo "✅ 恭喜：测试通过！峰值温度完美处于 82°C 以下，当前散热模组压制有力，可以部署到生鲜店面。" | tee -a $LOG_FILE
fi
echo "====================================" | tee -a $LOG_FILE
echo "测试结束，该测试小票已自动保存为: $LOG_FILE"
