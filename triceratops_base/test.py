import dynamixel_sdk as dxl
import time

# Control table addresses
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_PWM = 100
ADDR_PRESENT_POSITION = 132

# Operating modes
PWM_CONTROL_MODE = 16  # PWM control mode

# Protocol version
PROTOCOL_VERSION = 2.0

# Default settings
DXL_ID = 1                  # Dynamixel ID
BAUDRATE = 57600            # Baudrate of Dynamixel
DEVICENAME = '/dev/ttyUSB0'  # Port connected to Dynamixel (adjust for your system)
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
PWM_LIMIT = 885             # Dynamixel X-series max PWM value
POSITION_TOLERANCE = 40     # Tolerance for position error

# PID gains (need tuning based on your system)
KP = 0.8   # Proportional gain
KI = 0.0   # Integral gain
KD = 0.0   # Derivative gain

# Initialize PortHandler and PacketHandler instance
portHandler = dxl.PortHandler(DEVICENAME)
packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port!")
else:
    print("Failed to open the port!")
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate!")
else:
    print("Failed to change the baudrate!")
    quit()

# Set operating mode to PWM Control Mode
def set_pwm_control_mode():
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, PWM_CONTROL_MODE)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("PWM Control Mode enabled")

# Enable Dynamixel Torque
def enable_torque():
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Torque enabled")

# Disable Dynamixel Torque
def disable_torque():
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Torque disabled")

# Set PWM value
def set_pwm_value(pwm_value):
    pwm_value = max(min(pwm_value, PWM_LIMIT), -PWM_LIMIT)  # Clamp the PWM value within the limits
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_PWM, pwm_value)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print(f"PWM set to {pwm_value}")

# Get current position
def get_present_position():
    present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    return present_position

# PID controller
def pid_control(target_position):
    integral = 0
    last_error = 0
    while True:
        current_position = get_present_position()
        error = target_position - current_position
        integral += error
        derivative = error - last_error
        last_error = error

        # PID formula
        pwm_output = KP * error + KI * integral + KD * derivative
        set_pwm_value(int(pwm_output))

        print(f"Target: {target_position}, Current: {current_position}, PWM: {pwm_output}")
        
        if abs(error) <= POSITION_TOLERANCE:
            print("Reached target position!")
            break
        
        time.sleep(0.1)

# Main function with while loop
def main():
    set_pwm_control_mode()  # Set motor to PWM Control Mode
    enable_torque()         # Enable torque
    
    while True:
        # Prompt user for a target position
        target_position = int(input("Enter target position (0-4095, or -1 to quit): "))
        
        if target_position == -1:
            print("Exiting...")
            break  # Exit the loop if the user enters -1
        
        if 0 <= target_position <= 4095:
            pid_control(target_position)  # Run PID control to reach the target position
        else:
            print("Invalid position! Please enter a value between 0 and 4095.")

    disable_torque()  # Disable torque when done

    # Close port
    portHandler.closePort()

if __name__ == "__main__":
    main()
