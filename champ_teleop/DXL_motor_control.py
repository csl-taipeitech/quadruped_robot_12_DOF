#332Hz at 3000000 BAURATE at tx2
import dynamixel_sdk as dxlSDK
import sys, math, time
import log as log

POSITION_RATIO = 2*math.pi/4096
VELOCITY_RATIO = 0.229*2*math.pi/60
CURRENT_RATIO  = 3.6                    #mA
VELOCITY_MODE  = 1
POSITION_MODE  = 3
PWM_MODE       = 16
CURRENT_BASED_POSITION_MODE = 5

TORQUE_ADDR_LEN = (64,1)
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

LED_ADDR_LEN = (65,1)
LED_ON = 1
LED_OFF = 0

OPERATE_MODE_ADD_LEN = (11, 1)

class DXL_Communication(object):
    def __init__(self, device_name = '/dev/ttyUSB0', b_rate=3000000, log_level="info", log_file_level="debug"):
        self.log_level = log_level
        self.log_file_level = log_file_level
        self.BAUDRATE                    = b_rate             # Dynamixel default baudrate : 57600
        DEVICENAME                  = device_name       # Check which port is being used on your controller
        # DEVICENAME                  = '/dev/tty.usbserial-FT2N0CMQ'
                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel
        self.log = log.LogHandler(self.__class__.__name__, __name__, self.log_level, self.log_file_level)
        self.port_handler = dxlSDK.PortHandler(DEVICENAME)
        self.packet_handler = dxlSDK.PacketHandler(PROTOCOL_VERSION)
        self.groupBulkWrite = dxlSDK.GroupBulkWrite(self.port_handler, self.packet_handler)
        self.groupBulkRead = MyGroupBucketRead(self.port_handler, self.packet_handler)

        self.__communicate_error_count = 0

        self.portHandler_Check_Pass = False

    def activateDXLConnection(self):
        try:
            if self.port_handler.openPort():
                self.log.info("Succeeded to open the port", self.activateDXLConnection)
                self.portHandler_Check_Pass = True
            else:
                self.log.warning("Failed to open the port", self.activateDXLConnection)
                self.log.warning("{ System will run WITHOUT Real Mortor", self.activateDXLConnection)
                self.portHandler_Check_Pass = False
                self.closeHandler()
        except Exception:
            self.log.exception("", self.activateDXLConnection)
            self.log.warning("Failed to open the port", self.activateDXLConnection)
            self.log.warning("System will run WITHOUT Real Mortor", self.activateDXLConnection)
            self.portHandler_Check_Pass = False
            return
        
        # Set port baudrate
        if self.portHandler_Check_Pass:
            if self.port_handler.setBaudRate(self.BAUDRATE):
                self.log.info("Succeeded to change the baudrate", self.activateDXLConnection)
                self.portHandler_Check_Pass = True
            else:
                self.log.error("Failed to change the baudrate", self.activateDXLConnection)
                self.log.warning("System will run WITHOUT Real Mortor", self.activateDXLConnection)
                self.portHandler_Check_Pass = False
                self.closeHandler()
        
        self.motors = []
        self.parm = []

    def addAllBuckPrarmeter(self):
        self.groupBulkRead.clearParam()
        for motor in self.motors:
            addr_list = list()
            len_list = list()
            for addr_info in motor.indirect_read_addr_info.values() if motor.indirect_mode else motor.read_addr_info.values():
                addr_list.append(addr_info['ADDR'])
                len_list.append(addr_info['LEN'])
            motor.start_addr = min(addr_list)
            motor.all_data_len = max(addr_list) - min(addr_list) + len_list[addr_list.index(max(addr_list))]
            _ = self.groupBulkRead.addParam(motor.DXL_ID, motor.start_addr, motor.all_data_len)

    def updateMotorData(self, update_all = True, num = 1,  delay=-1):
        dxl_comm_result = self.groupBulkRead.txRxPacket()
        if dxl_comm_result == dxlSDK.COMM_SUCCESS:
            if update_all:
                for motor in self.motors:
                    if self.groupBulkRead.isAvailable(motor.DXL_ID, motor.start_addr, motor.all_data_len):
                        motor.data = self.groupBulkRead.getData(motor.DXL_ID, motor.start_addr, motor.all_data_len)
                        motor.updateValue()
                    else:
                        self.log.warning(f"Motro {motor.DXL_ID} return data error", self.updateMotorData)
                        motor.readHardwareError()

            else:
                motor = self.motors[num - 1]
                if self.groupBulkRead.isAvailable(motor.DXL_ID, motor.start_addr, motor.all_data_len):
                    motor.data = self.groupBulkRead.getData(motor.DXL_ID, motor.start_addr, motor.all_data_len)
                    motor.updateValue()
                else:
                    self.log.warning(f"Motro {motor.DXL_ID} return data error", self.updateMotorData)

        elif dxl_comm_result == dxlSDK.COMM_RX_TIMEOUT:
            # self.log.info("Read data timeout", self.updateMotorData)
            pass
        else:
            self.log.warning(f"Failed: {self.packet_handler.getTxRxResult(dxl_comm_result)}", self.updateMotorData)
            self.__communicate_error_count += 1
        if delay != -1:
            o_time = time.monotonic()
            while time.monotonic() - o_time <= delay/1000: pass

    def createMotor(self, name, motor_number = 1):
        # print("self.motors", self.motors)
        if motor_number not in [motor.DXL_ID for motor in self.motors]:
            motor = DXL_motor(
                self.port_handler,
                self.packet_handler,
                motor_number,
                self.log_level,
                self.log_file_level
                )
            motor.pingMotor()
            
            if motor.connected:
                motor.name = name
                self.motors.append(motor)
                self.addAllBuckPrarmeter()
                # motor.writePosition(initial_position)
                # self.sentAllCmd()
                return motor
            else:
                self.log.warning(f"Motor {motor_number} connect error", self.createMotor)
                return None
        else:
            self.log.warning(f"Motor {motor_number} already exist", self.createMotor)
            for motor in self.motors:
                if motor.DXL_ID == motor_number:
                    return motor


    def activateIndirectMode(self):
        for motor in self.motors:
            motor.activateIndirectMode()
        self.addAllBuckPrarmeter()
        self.log.info("Indirect Mode activated.", self.activateIndirectMode)

    def sentAllCmd(self):
        for motor in self.motors:
            for msg in motor.msg_sent:
                _ = self.groupBulkWrite.addParam(*msg)
            motor.msg_sent = list()
        dxl_comm_result = self.groupBulkWrite.txPacket()
        if dxl_comm_result != dxlSDK.COMM_SUCCESS:
            self.log.error(f"{self.packet_handler.getTxRxResult(dxl_comm_result)}", self.sentAllCmd)
        self.groupBulkWrite.clearParam()

    def setLogLevel(self, log_level="info", log_file_level="debug"):
        '''
            Set current log level to target level
        '''
        self.log.setLogLevel(log_level, log_file_level)
        self.log_level = log_level
        self.log_file_level = log_file_level
        for motor in self.motors:
            motor.log.setLogLevel(self.log_level, self.log_file_level)


    def disableAllMotor(self):
        self.groupBulkWrite.clearParam()
        for motor in self.motors:
            self.groupBulkWrite.addParam(motor.DXL_ID, 64, 1, [0])
        result = self.groupBulkWrite.txPacket()
        if result == dxlSDK.COMM_SUCCESS:
            self.groupBulkWrite.clearParam()
        else:
            self.log.info(f"{self.packet_handler.getTxRxResult(result)}", self.disableAllMotor)
            self.sentAllCmd()

    def closeHandler(self):
        time.sleep(0.1)
        for motor in self.motors:
            motor.closeLog()
        self.motors = list()
        self.port_handler.closePort()
        self.log.info("Close port", self.closeHandler)

    def sentCommand(self):
        self.groupBulkWrite.txPacket()
        self.groupBulkWrite.clearParam()

    def readHardwareError(self):
        for motor in self.motors:
            motor.readHardwareError()

    def rebootAllMotor(self):
        for motor in self.motors:
            motor.rebootMotor()
        time.sleep(1)
        self.__communicate_error_count = 0

    def checkErrorCount(self):
        return self.__communicate_error_count


class DXL_motor(object):

    def __init__(self, port_h, package_h, Motor_number=1, log_level="info", file_log_level="debug"):

        # Control table address
        self.log = log.LogHandler(
            DXL_Communication.__name__ + " ." + self.__class__.__name__ + f"_{Motor_number}",
            __name__,
            log_level,
            file_log_level
            )
        self.name = None
        self.indirect_mode = False
        self.connected = False
        self.write_addr_info = {
            'GOAL_POSITION': {'ADDR': 116, 'LEN': 4},
            'GOAL_VELOCITY': {'ADDR': 104, 'LEN': 4},
            # "GOAL_CURRENT" : {"ADDR": 102, "LEN": 2}
        }
        self.read_addr_info = {
            'TORQUE_ENABLE':      {'ADDR':  64 ,'LEN': 1},
            'HARDWARE_ERR':       {'ADDR':  70 ,'LEN': 1},
            'PRESENT_CURRENT':    {'ADDR': 126 ,'LEN': 2},
            'PRESENT_VELOCITY':   {'ADDR': 128 ,'LEN': 4},
            'PRESENT_POSITION':   {'ADDR': 132 ,'LEN': 4},
            'PRESENT_TEMPERTURE': {'ADDR': 146 ,'LEN': 1},
            'KD':                 {'ADDR': 80 ,'LEN': 2},
            'KI':                 {'ADDR': 82 ,'LEN': 2},
            'KP':                 {'ADDR': 84 ,'LEN': 2},
        }
        self.indirect_read_addr_info = dict()
        self.packet_h = package_h
        self.port_h = port_h
        # Default setting
        self.DXL_ID                      = Motor_number      # Dynamixel ID : 1

        self.MortorStatus                = 0
        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        self.DXL_MINIMUM_POSITION_VALUE  = 10                # Dynamixel will rotate between this value
        self.DXL_MAXIMUM_POSITION_VALUE  = 4000              # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        self.DXL_MAXIMUM_VELOCITY_VALUE  = 210
        self.DXL_MINIMUM_VELOCITY_VALUE  = -210


        self.index = 0
        self.dxl_goal_position = [self.DXL_MINIMUM_POSITION_VALUE, self.DXL_MAXIMUM_POSITION_VALUE]
        self.all_data_len      = 0
        self.start_addr        = 0
        self.data              = list()
        self.acc_profile       = 0

        self.OPERATING_MODE             = None
        self.TORQUE_ENABLE_value        = 0
        self.PRESENT_CURRENT_value      = 0
        self.PRESENT_VELOCITY_value     = 0
        self.PRESENT_POSITION_value     = 0
        self.PRESENT_TEMPERTURE_value   = 0
        self.HARDWARE_ERR_value         = 0
        self.KP                         = 0
        self.KI                         = 0
        self.KD                         = 0

        self.msg_sent = list()
        self.checkOperatingMode()

        #check Handler is working

    def checkOperatingMode(self):
        mode_number, read_success = self.directReadData(*OPERATE_MODE_ADD_LEN)
        if read_success:
            self.OPERATING_MODE = mode_number

    def closeLog(self):
        self.log.debug(f"Remove handler from DXL_Motor {self.DXL_ID}")
        self.log.removeHandler()

    def switchMode(self, mode = 'position'):
        if not self.torqueEnabled():
            if mode == 'position':
                mode = POSITION_MODE
            elif mode == 'velocity':
                mode = VELOCITY_MODE
            elif mode == 'pwm':
                mode = PWM_MODE
            elif mode == "current": 
                mode = CURRENT_BASED_POSITION_MODE
            switch_success = self.directWriteData(mode, *OPERATE_MODE_ADD_LEN)
            if switch_success:
                self.checkOperatingMode()
                if self.OPERATING_MODE == mode:
                    self.log.info(f"Motor {self.DXL_ID} OP Mode chenge to {mode} : True", self.switchMode)
                    return True
                else:
                    self.log.info(f"Motor {self.DXL_ID} OP Mode chenge to {mode} : False", self.switchMode)
                    return False
        else:
            self.log.warning("Mode Not Changed", self.switchMode)
            if self.TORQUE_ENABLE_value == 1:
                self.log.warning("Disable Motor {self.DXL_ID} first", self.switchMode)
                return False

    def torqueEnabled(self):
        torque_enable, read_success = self.directReadData(*TORQUE_ADDR_LEN)
        if read_success:
            self.TORQUE_ENABLE_value = torque_enable
            self.log.debug(f"{self.TORQUE_ENABLE_value}", self.torqueEnabled)
            return True if torque_enable == 1 else False
        else:
            self.log.warning(f"Read fail", self.torqueEnabled)
            return None

    def readHardwareError(self):
        HARDWARE_ERROR_ADD_LEN = (70 ,1)
        value, _ = self.directReadData(*HARDWARE_ERROR_ADD_LEN,True)
        if value != 0:
            self.log.debug(f" Motor {self.DXL_ID} with hardware error code: {value}", self.readHardwareError)

    def enableMotor(self):
        tqe_on = self.directWriteData(TORQUE_ENABLE, *TORQUE_ADDR_LEN)
        # led_on = self.directWriteData(LED_OFF, *LED_ADDR_LEN)
        if tqe_on:
            self.log.info("Motor{0} is successfully armed".format(self.DXL_ID), self.enableMotor)
        elif not tqe_on:
            if self.torqueEnabled():
                self.log.warning("Motor{0} armed with error".format(self.DXL_ID), self.enableMotor)
            else:
                self.log.info("Motor{0} not armed".format(self.DXL_ID), self.enableMotor)

    def activateIndirectMode(self):
        self.indirect_mode = True
        INDIRECT_START = 168
        INDIRECT_DATA_START = 224
        addr_prob = INDIRECT_START
        indirect_addr = INDIRECT_DATA_START
        for data_name, addr_info in self.read_addr_info.items():
            #Create Indirect ADDR information
            # future_to_read, dxl_comm_result, dxl_error = self.packet_h.read2ByteTxRx(
            #         self.port_h, self.DXL_ID, addr_prob
            #         )
            self.indirect_read_addr_info[data_name] = {
                'ADDR': indirect_addr, 'LEN': addr_info['LEN']
            }
            indirect_w_success = None
            for addr_shift in range(addr_info['LEN']):
                indirect_w_success = self.directWriteData(
                    addr_info['ADDR'] + addr_shift, addr_prob, 2
                )
                # dxl_comm_result, dxl_error = self.packet_h.write2ByteTxRx(
                #     self.port_h, self.DXL_ID, addr_prob, addr_info['ADDR'] + addr_shift
                #     )

                if indirect_w_success:
                    self.log.info("data [{0}] bit[{1}] of motor {2}, is set to {3} indirect address".format(
                        data_name, addr_shift+1, self.DXL_ID, indirect_addr
                    ))
                    addr_prob += 2
                    indirect_addr += 1
                else:
                    self.log.warning("Indirect Address Faild in Motor{0}".format(self.DXL_ID))
                    self.indirect_mode = False
                    return


    def disableMotor(self):
        tqe_off = self.directWriteData(TORQUE_DISABLE, *TORQUE_ADDR_LEN)
        if tqe_off:
            self.log.info("Motor{0} disarmed SUCCESSFULLY".format(self.DXL_ID), self.disableMotor)
        else:
            if not tqe_off:
                self.directWriteData(0, *TORQUE_ADDR_LEN, True)
                self.log.info("Motor{0} disarmed UNSUCCESSFULLY".format(self.DXL_ID), self.disableMotor)

    def writeVelocity(self, value):
        if self.OPERATING_MODE == VELOCITY_MODE:
            ADDR = 104
            LEN = 4
            data = [
                dxlSDK.DXL_LOBYTE(dxlSDK.DXL_LOWORD(value)),
                dxlSDK.DXL_HIBYTE(dxlSDK.DXL_LOWORD(value)),
                dxlSDK.DXL_LOBYTE(dxlSDK.DXL_HIWORD(value)),
                dxlSDK.DXL_HIBYTE(dxlSDK.DXL_HIWORD(value))
            ]
            # self.msg_sent.append((self.DXL_ID, ADDR, LEN, data))
            if value >= self.DXL_MINIMUM_VELOCITY_VALUE and value <= self.DXL_MAXIMUM_VELOCITY_VALUE:
                self.msg_sent.append((self.DXL_ID, ADDR, LEN, data))
                # _ = DXL_Conmunication.groupBulkWrite.addParam(self.DXL_ID, ADDR, LEN, data)
            else:
                self.log.warning("Commond exceed maximum range", self.writeVelocity)
        else:
            self.log.error("Operating Mode mismatch while setting velocity", self.writeVelocity)

    def writePosition(self, value):
        if self.OPERATING_MODE in [POSITION_MODE, CURRENT_BASED_POSITION_MODE]:
            ADDR = 116
            LEN = 4
            data = [
                dxlSDK.DXL_LOBYTE(dxlSDK.DXL_LOWORD(value)),
                dxlSDK.DXL_HIBYTE(dxlSDK.DXL_LOWORD(value)),
                dxlSDK.DXL_LOBYTE(dxlSDK.DXL_HIWORD(value)),
                dxlSDK.DXL_HIBYTE(dxlSDK.DXL_HIWORD(value))
            ]
            # self.msg_sent.append((self.DXL_ID, ADDR, LEN, data))
            if value >= self.DXL_MINIMUM_POSITION_VALUE and value <= self.DXL_MAXIMUM_POSITION_VALUE:
                self.msg_sent.append((self.DXL_ID, ADDR, LEN, data))
                # _ = DXL_Conmunication.groupBulkWrite.addParam(self.DXL_ID, ADDR, LEN, data)
            else:
                self.log.warning("Commond exceed maximum range", self.writePosition)
        else:
            self.log.error("Operating Mode mismatch while setting position", self.writePosition)

    def MotorCorrection(self):
        pass

    def infoParam(self,name):
        ADDR, LEN = None, None
        if name == 'torque':
            ADDR, LEN = self.read_addr_info['TORQUE_ENABLE']['ADDR'], self.read_addr_info['TORQUE_ENABLE']['LEN']
        if name == 'current':
            ADDR, LEN = self.read_addr_info['PRESENT_CURRENT']['ADDR'], self.read_addr_info['PRESENT_CURRENT']['LEN']
        if name == 'velocity':
            ADDR, LEN = self.read_addr_info['PRESENT_VELOCITY']['ADDR'], self.read_addr_info['PRESENT_VELOCITY']['LEN']
        if name == 'position':
            ADDR, LEN = self.read_addr_info['PRESENT_POSITION']['ADDR'], self.read_addr_info['PRESENT_POSITION']['LEN']
        if name == 'temperture':
            ADDR, LEN = self.read_addr_info['PRESENT_TEMPERTURE']['ADDR'], self.read_addr_info['PRESENT_TEMPERTURE']['LEN']
        return self.DXL_ID, ADDR, LEN

    def addRequestValue(self, name, addr, dlen):
        self.read_addr_info[name] = {'ADDR': addr, 'LEN': dlen}
        setattr(self, name + "_value", 0)

    def updateValue(self):
        for name, info in self.indirect_read_addr_info.items() if self.indirect_mode else self.read_addr_info.items():
            shifted_address = info['ADDR'] - self.start_addr
            byte_data = self.data[shifted_address:shifted_address+info['LEN']]
            if info['LEN'] == 1:
                value = byte_data[0]
            elif info['LEN'] == 2:
                value = dxlSDK.DXL_MAKEWORD(byte_data[0],byte_data[1])
            elif info['LEN'] == 4:
                value = dxlSDK.DXL_MAKEDWORD(
                    dxlSDK.DXL_MAKEWORD(byte_data[0],byte_data[1]),
                    dxlSDK.DXL_MAKEWORD(byte_data[2],byte_data[3])
                )
            else:
                value = byte_data
            setattr(self, name + "_value", value)
            if self.PRESENT_CURRENT_value >= 32768:
                self.PRESENT_CURRENT_value = self.PRESENT_CURRENT_value-65535
            if self.PRESENT_VELOCITY_value >= (2**32)/2:
                self.PRESENT_VELOCITY_value = self.PRESENT_VELOCITY_value - (2**32-1)
            if self.PRESENT_POSITION_value >= (2**32)/2:
                self.PRESENT_POSITION_value = self.PRESENT_POSITION_value - (2**32-1)
        if self.HARDWARE_ERR_value == 8:
            self.PRESENT_CURRENT_value = None
            self.PRESENT_POSITION_value = None
            self.PRESENT_VELOCITY_value = None

    def directReadData(self, add, len, print_msg=True)->tuple([int,bool]):
        value, com_err_msg, dxl_err_msg = None, None, None
        func_name = "read{0}ByteTxRx".format(len)
        func_ = getattr(self.packet_h,func_name)
        value, dxl_comm_result, dxl_error = func_(self.port_h, self.DXL_ID, add)
        if dxl_comm_result != dxlSDK.COMM_SUCCESS:
            com_err_msg = self.packet_h.getTxRxResult(dxl_comm_result)
        elif dxl_error != 0:
            dxl_err_msg = self.packet_h.getRxPacketError(dxl_error)
        else:
            return value, True
        if com_err_msg is not None or dxl_err_msg is not None:
            if com_err_msg and print_msg: self.log.info("com_err_msg : {0} at ID: {1}".format(com_err_msg, self.DXL_ID), self.directReadData)
            if dxl_err_msg and print_msg: self.log.info("dxl_err_msg : {0} at ID: {1}".format(dxl_err_msg, self.DXL_ID), self.directReadData)
            return value, False

    def directWriteData(self, data, add, len, print_msg=True):
        com_err_msg, dxl_err_msg = None, None
        func_name = "write{0}ByteTxRx".format(len)
        func_ = getattr(self.packet_h, func_name)
        dxl_comm_result, dxl_error = func_(self.port_h, self.DXL_ID, add, data)
        if dxl_comm_result != dxlSDK.COMM_SUCCESS:
            com_err_msg = self.packet_h.getTxRxResult(dxl_comm_result)
        elif dxl_error != 0:
            dxl_err_msg = self.packet_h.getRxPacketError(dxl_error)
        else:
            return True
        if com_err_msg is not None or dxl_err_msg is not None:
            if com_err_msg and print_msg: print("DXL: directWriteData Error: {0} at ID: {1}".format(com_err_msg, self.DXL_ID))
            if dxl_err_msg and print_msg: print("DXL: directWriteData Error: {0} at ID: {1}".format(dxl_err_msg, self.DXL_ID))
            return False
            
    def setVelocity(self, v_cmd):
        if self.OPERATING_MODE == VELOCITY_MODE:
            addr_len = self.write_addr_info['GOAL_VELOCITY']
            if v_cmd <= self.DXL_MAXIMUM_VELOCITY_VALUE and v_cmd >= self.DXL_MINIMUM_VELOCITY_VALUE:
                self.directWriteData(v_cmd, addr_len['ADDR'], addr_len['LEN'],True)
            else:
                print("Command out off range")
        else:
            print("Mode Error while write velocity in {0} mode".format(self.OPERATING_MODE))
    
    def setPosition(self, p_cmd):
        if self.OPERATING_MODE == POSITION_MODE:
            addr_len = self.write_addr_info['GOAL_POSITION']
            if p_cmd <= self.DXL_MAXIMUM_POSITION_VALUE and p_cmd >= self.DXL_MINIMUM_POSITION_VALUE:
                self.directWriteData(p_cmd, addr_len['ADDR'], addr_len['LEN'], True)
            else:
                print("Command out off range")
        else:
            print("Mode Error while write position in {0} mode".format(self.OPERATING_MODE))

    def setPID(self, p, i, d):
        ADDR = 84
        LEN = 2
        self.directWriteData(int(p), ADDR, LEN)
        ADDR = 82
        self.directWriteData(int(i), ADDR, LEN)
        ADDR = 80
        self.directWriteData(int(d), ADDR, LEN)

    def setAccelerationProfile(self, profile: int)->None:
        self.acc_profile = profile
        ADDR = 108
        LEN  = 4
        self.directWriteData(profile, add=ADDR, len=LEN)

    def pingMotor(self):
        dxl_model_number, dxl_comm_result, dxl_error = self.packet_h.ping(self.port_h, self.DXL_ID)
        if dxl_comm_result != dxlSDK.COMM_SUCCESS:
            self.log.info("DXL: Ping Error: {0} at ID:{1}".format(self.packet_h.getTxRxResult(dxl_comm_result), self.DXL_ID), self.pingMotor)
        elif dxl_error != 0:
            self.log.info("DXL: Ping Error: {0} at ID:{1}".format(self.packet_h.getRxPacketError(dxl_error), self.DXL_ID), self.pingMotor)
        else:
            self.log.info("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (self.DXL_ID, dxl_model_number), self.pingMotor)
            self.connected = True

    def rebootMotor(self):
        dxl_comm_result, dxl_error = self.packet_h.reboot(self.port_h, self.DXL_ID)
        if dxl_comm_result != dxlSDK.COMM_SUCCESS:
            self.log.error("ID:{0} reboot Error: {1}".format(self.DXL_ID,self.packet_h.getTxRxResult(dxl_comm_result)), self.rebootMotor)
        elif dxl_error != 0:
            self.log.error("ID:{0} reboot Error: {1}".format(self.DXL_ID,self.packet_h.getRxPacketError(dxl_error)), self.rebootMotor)

        self.log.info("[ID:{0}] reboot Succeeded".format(self.DXL_ID), self.rebootMotor)

class MyGroupBucketRead(dxlSDK.GroupBulkRead):

    def __init__(self,port_handler, packet_handler):
        super(MyGroupBucketRead, self).__init__(port_handler,packet_handler)

    def getData(self, dxl_id, address, data_length):
        PARAM_NUM_DATA = 0
        PARAM_NUM_ADDRESS = 1
        if not self.isAvailable(dxl_id, address, data_length):
            return 0

        start_addr = self.data_dict[dxl_id][PARAM_NUM_ADDRESS]

        if data_length == 1:
            return self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr]
        elif data_length == 2:
            return dxlSDK.DXL_MAKEWORD(
                self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr],
                self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 1]
            )
        elif data_length == 4:
            return dxlSDK.DXL_MAKEDWORD(
                dxlSDK.DXL_MAKEWORD(
                    self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 0],
                    self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 1]),
                dxlSDK.DXL_MAKEWORD(
                    self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 2],
                    self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 3])
                )
        else:
            return self.data_dict[dxl_id][PARAM_NUM_DATA]

def readError():
    import time
    dynamixel = DXL_Communication("/dev/tty.usbserial-FT2N0CMQ")
    # dynamixel = DXL_Conmunication()
    upper_motor = dynamixel.createMotor("upper_motor",1)
    upper_motor.readHardwareError()
    # middl_motor = dynamixel.createMotor("middl_moter",2)
    # lower_moter = dynamixel.createMotor("lower_moter",3)

def torqueTest():
    #Show the current reading at pos 2120 when twisting the motor
    
    dynamixel = DXL_Communication(DEVICE_NAME, B_RATE)
    test_motor = dynamixel.createMotor('test_motor', motor_number=1)
    time.sleep(3)
    test_motor.switchMode('position')
    test_motor.enableMotor()
    test_motor.writePosition(2120)
    dynamixel.sentAllCmd()
    try:
        while True:
            dynamixel.updateMotorData()
            print("     ", end = '\r')
            print(test_motor.PRESENT_CURRENT_value, end = '\r')
    except KeyboardInterrupt:
        dynamixel.disableAllMotor()
        dynamixel.closeHandler()

# def FootMotorSpeedTest():
#     #Test motor control as this package implanted in robot_leg_model
#     #Showing all motor communication speed
#     import robot_leg_model as RF

#     Foot = RF.Leg14_14_7_Real()
#     Foot.activateRealMotor(DEVICE_NAME)
#     Foot.switchMotorMode('velocity')
#     Foot.armAllMotor()

#     counter = 0
#     t1 = time.time()
#     for _ in range(2):
#         for i in range(480):
#             vel = i/100
#             Foot.excu_velocity_cmd([vel,vel,vel],0.1,needLog=False)
#             Foot.updateRealMotorsInfo()
#             counter += 1
#         for i in range(480):
#             vel = (480 - i)/100
#             Foot.excu_velocity_cmd([vel,vel,vel],0.1,needLog=False)
#             Foot.updateRealMotorsInfo()
#             counter += 1
#     t_du = time.time() - t1
#     print("Foot run at {0:.2f}".format(counter/t_du))
#     Foot.excu_velocity_cmd([0,0,0],dt = 0.1, needLog=False)
#     Foot.disarmAllMotor()
#     Foot.dxl_conmunicator.closeHandler()

def threeMotorSeedTest():
    import numpy
    #Three motors communication test, shows frequency and each process duration 

    dynamixel = DXL_Communication(DEVICE_NAME, B_RATE)


    motor0 = dynamixel.createMotor('motor0',1)
    motor1 = dynamixel.createMotor('motor1',2)
    motor2 = dynamixel.createMotor('motor2',3)
    dynamixel.activateIndirectMode()
    # dynamixel.addAllBuckPrarmeter()
    motor_list = [
        motor0, motor1, motor2
    ]

    for motor in motor_list:
        motor.switchMode('velocity')
        motor.enableMotor()


    count = 0
    cmd_number = 1000
    p1 = numpy.zeros(cmd_number)
    p2 = numpy.zeros(cmd_number)

    t0 = time.time()
    while count != cmd_number:
        p0 = time.time()
        for motor in motor_list:
            motor.writeVelocity(0)
        dynamixel.sentAllCmd()
        p1[count] = time.time() - p0
        dynamixel.updateMotorData()
        p2[count] = time.time() - p0 - p1[count]
        count += 1
    t1 = time.time()
    print("average freq is {0}Hz".format(cmd_number/(t1-t0)))
    print("average priod is {0:.4f}ms".format((t1-t0)/cmd_number*1000))
    print("time of process  cmd: {0:.4f}ms".format(numpy.average(p1)*1000))
    print("time of process info: {0:.4f}ms".format(numpy.average(p2)*1000))
    for motor in motor_list:
        motor.disableMotor()
    dynamixel.closeHandler()

def posSwipeTest():
    #Show actual speed when three motors is following position cmd

    dynamixel = DXL_Communication(DEVICE_NAME,B_RATE)

    middl_motor = dynamixel.createMotor("middl_moter",2)

    # test_motor = dynamixel.createMotor("test_motor",4)

    dynamixel.activateIndirectMode()
    dynamixel.updateMotorData()
    ref_motor = middl_motor
    print("torque enable: {0}".format(ref_motor.TORQUE_ENABLE_value))
    print("present current: {0}".format(ref_motor.PRESENT_CURRENT_value))
    print("present position: {0}".format(ref_motor.PRESENT_POSITION_value))
    print("present temperture: {0}".format(ref_motor.PRESENT_TEMPERTURE_value))
    print("present velocity: {0}".format(ref_motor.PRESENT_VELOCITY_value))

    for motor in dynamixel.motors:
        motor.switchMode('position')
        motor.enableMotor()
        motor.writePosition(10)
    dynamixel.sentAllCmd()

    for index in range(5,0,-1): #count down
        print(index)
        time.sleep(1)

    dynamixel.updateMotorData()
    stamp = time.time()
    count = 0
    while ref_motor.PRESENT_POSITION_value <= 3990:
        for motor in dynamixel.motors:
            motor.writePosition(4000)
        dynamixel.sentAllCmd()
        dynamixel.updateMotorData()
        count += 1
    print((time.time()-stamp)/count)
    freq = 1/((time.time()-stamp)/count)
    print("run in %d Hz" %freq)

    for motor in dynamixel.motors:
        motor.disableMotor()
    dynamixel.closeHandler()

def velSwipeToTarget():
    #Track position cmd by velocity mode

    ID     = 1
    dynamixel = DXL_Communication(DEVICE_NAME, B_RATE)
    test_motor = dynamixel.createMotor('test_motor' ,motor_number=ID)
    dynamixel.activateIndirectMode()
    dynamixel.addAllBuckPrarmeter()

    test_motor.disableMotor()
    test_motor.switchMode('position')
    test_motor.enableMotor()
    test_motor.writePosition(4000)
    dynamixel.sentAllCmd()
    
    for i in range(5):
        time.sleep(1)
        print(i)

    test_motor.disableMotor()
    test_motor.switchMode('velocity')
    test_motor.enableMotor()
    dynamixel.updateMotorData()
    try:
        while test_motor.PRESENT_POSITION_value >= 10:
            test_motor.writeVelocity(-10)
            dynamixel.sentAllCmd()
            dynamixel.updateMotorData()
        test_motor.writeVelocity(0)
        dynamixel.sentAllCmd()
    except:
        time.sleep(1)
        test_motor.disableMotor()
    msg = f"Targt = 10, Current reading: {test_motor.PRESENT_POSITION_value}"
    test_motor.disableMotor()
    print(msg)

def motorVelocityErrorTest():
    #Show the error of constant velocity cmd, Error increase as absolute V_cmd decrease
    ID = 1
    dynamixel = DXL_Communication(DEVICE_NAME, B_RATE)
    motor = dynamixel.createMotor('motor',ID)
    dynamixel.activateIndirectMode()

    desir_speed = 2 #rad/s
    speed_cmd = int(desir_speed / 0.023973)

    motor.disableMotor()
    motor.switchMode('position')
    motor.enableMotor()
    motor.setPosition(100)
    time.sleep(2)
    dynamixel.updateMotorData()

    motor.disableMotor()
    motor.switchMode('velocity')
    motor.enableMotor()

    print(f"desire velocity: {desir_speed} <=> velocity cmd: {speed_cmd}")
    motor.setVelocity(speed_cmd)
    pass_2048 = None
    pass_6144 = None
    while motor.PRESENT_POSITION_value <= 4096*2:
        dynamixel.updateMotorData()
        if pass_2048 is None and motor.PRESENT_POSITION_value >= 2048:
            pass_2048 = time.time()
        if pass_6144 is None and motor.PRESENT_POSITION_value >= 6144:
            pass_6144 = time.time()
    motor.setVelocity(0)
    dynamixel.closeHandler()

    duration = pass_6144 - pass_2048
    tested_speed = 2*math.pi / duration
    print("Cmd speed: {0} rad/s, Tested speed: {1:.6f} rad/s {2:2.2f}% off".format(
        desir_speed, tested_speed, (tested_speed-desir_speed)/desir_speed*100
    ))

def SingleMotorTest():
    #show difference between optimized SDK and original SDK

    ID = 1
    dynamixel = DXL_Communication(DEVICE_NAME, B_RATE)
    dynamixel.activateDXLConnection()
    motor = dynamixel.createMotor('test', motor_number=ID)
    # dynamixel.activateIndirectMode()
    dynamixel.addAllBuckPrarmeter()
    dynamixel.updateMotorData()

    print("torque enable: {0}".format(motor.TORQUE_ENABLE_value))
    print("present current: {0}".format(motor.PRESENT_CURRENT_value))
    print("present position: {0}".format(motor.PRESENT_POSITION_value))
    print("present temperture: {0}".format(motor.PRESENT_TEMPERTURE_value))
    print("present velocity: {0}".format(motor.PRESENT_VELOCITY_value))

    motor.switchMode('position')
    motor.enableMotor()
    motor.writePosition(10)
    dynamixel.sentAllCmd()

    for index in range(5,0,-1):
        print(index)
        time.sleep(1)
    dynamixel.updateMotorData()

    stamp = time.time()
    count = 0
    while motor.PRESENT_POSITION_value <= 3990:
        motor.writePosition(4000),
        dynamixel.sentAllCmd()
        dynamixel.updateMotorData()
        count += 1
    print((time.time()-stamp)/count)
    opt_freq = 1/((time.time()-stamp)/count)
    print("run in %d Hz" %opt_freq)

    #SDK method
    time.sleep(2)
    packetHandler = dynamixel.packet_handler
    portHandler = dynamixel.port_handler
    _, _ = packetHandler.write4ByteTxRx(portHandler, ID, 116, 10)
    for index in range(5,0,-1):
        print(index)
        time.sleep(1)

    position, _, _ = packetHandler.read4ByteTxRx(portHandler, ID, 132)
    stamp = time.time()
    count = 0
    while position <= 3900:
        _, _ = packetHandler.write4ByteTxRx(portHandler, ID, 116, 4000)
        torque, _, _ = packetHandler.read1ByteTxRx(portHandler, ID, 64)
        current, _, _ = packetHandler.read2ByteTxRx(portHandler, ID, 126)
        vel, _, _ = packetHandler.read4ByteTxRx(portHandler, ID, 128)
        position, _, _ = packetHandler.read4ByteTxRx(portHandler, ID,  132)
        temp, _, _ = packetHandler.read1ByteTxRx(portHandler, ID, 146)
        count += 1
    print((time.time()-stamp)/count)
    ori_freq = 1/((time.time()-stamp)/count)
    print("run in %d Hz" %ori_freq)
    motor.disableMotor()
    print(f"Optimized SDK: {opt_freq:.2f}Hz, original SDK: {ori_freq:.2f}Hz, with one motor")


def MotorReadTest():
    #show difference between optimized SDK and original SDK

    ID = 1
    dynamixel = DXL_Communication(DEVICE_NAME, B_RATE)
    dynamixel.activateDXLConnection()
    motor = dynamixel.createMotor('test', motor_number=ID)
    # dynamixel.activateIndirectMode()
    dynamixel.addAllBuckPrarmeter()
    dynamixel.updateMotorData()

    print("torque enable: {0}".format(motor.TORQUE_ENABLE_value))
    print("present current: {0}".format(motor.PRESENT_CURRENT_value))
    print("present position: {0}".format(motor.PRESENT_POSITION_value))
    print("present temperture: {0}".format(motor.PRESENT_TEMPERTURE_value))
    print("present velocity: {0}".format(motor.PRESENT_VELOCITY_value))

    motor.switchMode('pwm')
    # motor.enableMotor()
    # motor.writePosition(10)
    # dynamixel.sentAllCmd()

    # for index in range(5,0,-1):
    #     print(index)
    #     time.sleep(1)
    dynamixel.updateMotorData()

    # stamp = time.time()
    # count = 0
    # while motor.PRESENT_POSITION_value <= 3990:
    #     motor.writePosition(4000),
    #     dynamixel.sentAllCmd()
    #     dynamixel.updateMotorData()
    #     count += 1
    # print((time.time()-stamp)/count)
    # opt_freq = 1/((time.time()-stamp)/count)
    # print("run in %d Hz" %opt_freq)

    #SDK method
    time.sleep(2)
    packetHandler = dynamixel.packet_handler
    portHandler = dynamixel.port_handler
    # _, _ = packetHandler.write4ByteTxRx(portHandler, ID, 116, 10)
    # for index in range(5,0,-1):
    #     print(index)
    #     time.sleep(1)

    position, _, _ = packetHandler.read4ByteTxRx(portHandler, ID, 132)
    stamp = time.time()
    count = 0
    # while position <= 3900:
    while True:
        # _, _ = packetHandler.write4ByteTxRx(portHandler, ID, 116, 4000)
        torque, _, _ = packetHandler.read1ByteTxRx(portHandler, ID, 64)
        current, _, _ = packetHandler.read2ByteTxRx(portHandler, ID, 126)
        vel, _, _ = packetHandler.read4ByteTxRx(portHandler, ID, 128)
        position, _, _ = packetHandler.read4ByteTxRx(portHandler, ID,  132)
        # pwm, _, _ = packetHandler.read2ByteTxRx(portHandler, ID,  124)
        temp, _, _ = packetHandler.read1ByteTxRx(portHandler, ID, 146)
        count += 1
        print(position)
    
    
    # print((time.time()-stamp)/count)
    # ori_freq = 1/((time.time()-stamp)/count)
    # print("run in %d Hz" %ori_freq)
    motor.disableMotor()
    # print(f"Optimized SDK: {opt_freq:.2f}Hz, original SDK: {ori_freq:.2f}Hz, with one motor")



def testGround():
    pass

global DEVICE_NAME, B_RATE
if __name__ == "__main__":

    # DEVICE_NAME = "COM3"
    # DEVICE_NAME = "/dev/tty.usbserial-FT2N0CMQ"
    DEVICE_NAME = "/dev/ttyUSB0"
    B_RATE      = 57600

    # testGround()
    # readError()

    # torqueTest()
    # motorVelocityErrorTest()
    # FootMotorSpeedTest()  #Not tested, lack of motors
    # threeMotorSeedTest()  #Not tested, lack of motors
    # posSwipeTest()        #Not tested, lack of motors
    # velSwipeToTarget()
    # SingleMotorTest()
    MotorReadTest()