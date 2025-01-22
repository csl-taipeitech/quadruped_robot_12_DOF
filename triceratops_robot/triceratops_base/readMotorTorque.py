import dynamixel_sdk as dxl
import struct

# 控制參數
DEVICENAME = '/dev/ttyUSB0'  # 修改為你的串口名稱，例如 Windows 上的 'COM3'
BAUDRATE = 57600             # 修改為馬達的波特率
PROTOCOL_VERSION = 2.0       # Dynamixel 通信協議版本

# 馬達參數
DXL_ID = 1                   # 修改為你的馬達 ID
ADDR_PRESENT_LOAD = 126      # Load Address (適用於 Protocol 2.0)

# 初始化 PortHandler 和 PacketHandler
port_handler = dxl.PortHandler(DEVICENAME)
packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)

# 打開串口
if port_handler.openPort():
    print("成功打開串口")
else:
    print("無法打開串口")
    exit()

# 設置波特率
if port_handler.setBaudRate(BAUDRATE):
    print("成功設置波特率")
else:
    print("無法設置波特率")
    exit()

try:
    while True:
        # 讀取馬達的扭矩（Load）
        dxl_comm_result, dxl_error, dxl_data = packet_handler.read2ByteTxRx(port_handler, DXL_ID, ADDR_PRESENT_LOAD)
        
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print(f"通信錯誤: {packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"錯誤: {packet_handler.getRxPacketError(dxl_error)}")
        else:
            # 解析讀取到的數據
            # PRESENT_LOAD 是一個有符號的 16 位整數
            torque_value = dxl_data if dxl_data < 0x4000 else dxl_data - 0x8000
            # 將數值轉換為百分比
            torque_percent = (torque_value / 4095.0) * 100  # 根據馬達的扭矩範圍調整
            print(f"當前扭矩值: {torque_percent:.2f}%")

except KeyboardInterrupt:
    print("\n結束程式")

# 關閉串口
port_handler.closePort()
