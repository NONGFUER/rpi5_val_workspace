电子秤 Modbus RTU 通讯协议规范
版本：V1.1
文档类型：技术规格书
1. 概述
本协议定义了称重终端（从机）与应用主控（主机）之间基于标准 Modbus RTU 协议的通讯逻辑，用于实现称重数据读取、状态监控及系统校准等功能。
2. 物理层参数
表格
参数	配置	说明
接口类型	UART（TTL 逻辑电平）	3.3V / 5V 兼容
波特率	9600 bps	标准通讯速率
数据格式	8-N-1	8 位数据，无校验，1 位停止位
通讯模式	Modbus RTU	二进制模式
从机地址	0x01（默认）	范围：1–247
3. Modbus RTU 指令格式
标准 Modbus RTU 报文由四部分组成：
表格
地址码（1 Byte）	功能码（1 Byte）	数据区（N Bytes）	校验码（2 Bytes）
从机地址	操作类型（如 03、06）	寄存器地址、长度或具体数据	CRC16（低位在前）
常用功能码
03（0x03）：读取保持寄存器（Read Holding Registers）
06（0x06）：预置单个寄存器（Preset Single Register）
16（0x10）：预置多个寄存器（Preset Multiple Registers）
4. 寄存器映射表
所有寄存器均映射至 Holding Register 地址空间。
表格
地址（Hex）	变量名	读写权限	数据类型	说明
0x0000	Net_Weight_H	R（只读）	Int16	净重值（32 位补码，单位：g）
0x0001	Net_Weight_L	R（只读）	Uint16	-
0x0002	Status_Word	R（只读）	Uint16	设备状态字
0x0003	ADC_Raw_H	R（只读）	Int16	传感器原始 ADC 采样值
0x0004	ADC_Raw_L	R（只读）	Uint16	-
0x0010	System_Cmd	RW（读写）	Uint16	命令寄存器（1：去皮；2：校准）
0x0020	Cal_Factor_H	RW（读写）	Int16	校准系数 K（32 位定点数）
0x0021	Cal_Factor_L	RW（读写）	Uint16	-
5. 状态字 Bit 详解
Bit 0 稳定（Stability）：1 = 读数稳定；0 = 正在波动
Bit 1 过载（Overload）：1 = 超过额定量程；0 = 正常
Bit 2 负重（Negative）：1 = 当前净重为负；0 = 正数或零
Bit 3 去皮（Tared）：1 = 当前已应用偏移量（皮重）
6. 通讯报文示例
6.1 读取重量及状态（功能码 03）
主机查询（读取 3 个寄存器）：
01 03 00 00 00 03 05 CB
从机响应：
01 03 06 [Net_H] [Net_L] [Status] [CRC_L] [CRC_H]
6.2 执行去皮操作（功能码 06）
主机下发（向 0x0010 写入 1）：
01 06 00 10 00 01 88 09
从机确认：
01 06 00 10 00 01 88 09
称重系统通讯协议 | 技术机密