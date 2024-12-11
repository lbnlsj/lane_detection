# 车道识别串口通信协议规范

## 1. 通信基本参数

- 波特率：115200
- 数据位：8
- 停止位：1
- 校验位：无
- 流控制：无

## 2. 数据帧格式

### 2.1 帧结构

```
帧头(2字节) + 数据长度(1字节) + 数据(N字节) + 校验和(1字节) + 帧尾(2字节)
```

- 帧头: 0xAA 0x55
- 帧尾: 0x0D 0x0A
- 数据长度: 不包括帧头帧尾和校验和的数据字节数
- 校验和: 数据区所有字节的异或值

### 2.2 数据区格式

数据区包含4个浮点数，按以下顺序排列：
1. 左车道线距离 (4字节)
2. 右车道线距离 (4字节)
3. 左车道线角度 (4字节)
4. 右车道线角度 (4字节)

总数据长度：16字节

## 3. Python实现代码

```python
import serial
import struct

class LaneDataTransmitter:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        
    def pack_lane_data(self, left_distance: float, right_distance: float, 
                       left_angle: float, right_angle: float) -> bytes:
        """打包车道线数据"""
        # 帧头
        frame_header = bytes([0xAA, 0x55])
        
        # 数据区
        data = struct.pack('<ffff', 
                          left_distance,
                          right_distance,
                          left_angle,
                          right_angle)
        
        # 数据长度
        length = len(data)
        length_byte = bytes([length])
        
        # 计算校验和
        checksum = 0
        for byte in data:
            checksum ^= byte
        checksum_byte = bytes([checksum])
        
        # 帧尾
        frame_tail = bytes([0x0D, 0x0A])
        
        # 组合完整帧
        frame = frame_header + length_byte + data + checksum_byte + frame_tail
        
        return frame
    
    def send_lane_data(self, left_distance: float, right_distance: float,
                       left_angle: float, right_angle: float):
        """发送车道线数据"""
        frame = self.pack_lane_data(left_distance, right_distance,
                                  left_angle, right_angle)
        self.serial.write(frame)
        
    def close(self):
        """关闭串口"""
        if self.serial.is_open:
            self.serial.close()
```

## 4. 使用示例

```python
# 在LaneDetector类中添加数据发送功能
class LaneDetector:
    def __init__(self):
        # 原有初始化代码保持不变
        self.transmitter = LaneDataTransmitter()
        
    def process_detected_lines(self, frame, left_lines, right_lines):
        # 获取原有处理结果
        detected_lines, result_frame = super().process_detected_lines(
            frame, left_lines, right_lines)
        
        # 提取距离和角度数据
        left_distance = 0.0
        right_distance = 0.0
        left_angle = 0.0
        right_angle = 0.0
        
        # 处理左车道线数据
        if len(detected_lines) > 0:
            left_line = detected_lines[0]
            left_distance = self.calculate_distance(left_line)
            left_angle = self.calculate_lane_angle(left_line)
        
        # 处理右车道线数据
        if len(detected_lines) > 1:
            right_line = detected_lines[1]
            right_distance = self.calculate_distance(right_line)
            right_angle = self.calculate_lane_angle(right_line)
        
        # 发送数据
        self.transmitter.send_lane_data(
            left_distance, right_distance,
            left_angle, right_angle
        )
        
        return detected_lines, result_frame
```

## 5. 数据接收示例（接收端参考代码）

```python
def receive_lane_data(serial_port):
    """接收车道线数据"""
    # 等待帧头
    while True:
        if serial_port.read() == b'\xAA' and serial_port.read() == b'\x55':
            break
    
    # 读取数据长度
    length = ord(serial_port.read())
    
    # 读取数据
    data = serial_port.read(length)
    
    # 读取校验和
    checksum_received = ord(serial_port.read())
    
    # 读取帧尾
    frame_tail = serial_port.read(2)
    if frame_tail != b'\x0D\x0A':
        return None
    
    # 验证校验和
    checksum_calculated = 0
    for byte in data:
        checksum_calculated ^= byte
    
    if checksum_calculated != checksum_received:
        return None
    
    # 解析数据
    left_distance, right_distance, left_angle, right_angle = struct.unpack(
        '<ffff', data)
    
    return {
        'left_distance': left_distance,
        'right_distance': right_distance,
        'left_angle': left_angle,
        'right_angle': right_angle
    }
```

## 6. 注意事项

1. 确保发送端和接收端使用相同的串口参数配置
2. 数据发送频率建议不超过50Hz，以确保串口通信稳定
3. 接收端应该实现超时机制，避免数据帧丢失导致的阻塞
4. 建议在实际应用中增加数据有效性检查机制
5. 地平线X3平台的串口设备名可能需要根据实际情况修改