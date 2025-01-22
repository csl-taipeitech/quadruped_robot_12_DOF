import numpy as np
from math import pi, sqrt, acos, atan2
from Triceratops_Config import PuppyConfiguration, PuppyState, state

class RobotIK:
    def __init__(self):
        config = PuppyConfiguration()
        self.upper_leg_length = config.upper_leg_length
        self.lower_leg_length = config.lower_leg_length
        self.leg_center_position = np.array([
            [2048, 2048, 2048, 2048],  # 肩關節中心位置
            [2822, 2260, 2209, 2929],  # 上腿關節中心位置
            [2200, 2942, 2920, 2257]   # 下腿關節中心位置
        ])
        self.center_angles = np.array([
            [0, 0, 0, 0],  # 肩關節中心角度（設為0）
            [135.80, 135.80, 135.80, 135.80],
            [95.36, 95.36, 95.36, 95.36]
        ]) * pi / 180  # 轉換為弧度

    def leg_inverse_kinematics(self, foot_locations):
        joint_angles = np.zeros((3, 4))
        
        for i in range(4):
            x, y = foot_locations[0, i], foot_locations[1, i]
            
            # 計算腿長
            leg_length = sqrt(x**2 + y**2)
            
            # 檢查是否超出可達範圍
            max_length = self.upper_leg_length + self.lower_leg_length
            if leg_length > max_length:
                # 如果超出範圍，將腳的位置調整到最大可達範圍
                scale = max_length / leg_length
                x *= scale
                y *= scale
                leg_length = max_length

            # 計算下腿角度
            cos_lower = (self.upper_leg_length**2 + self.lower_leg_length**2 - leg_length**2) / (2 * self.upper_leg_length * self.lower_leg_length)
            lower_leg = acos(np.clip(cos_lower, -1.0, 1.0))
            
            # 計算上腿角度
            cos_upper = (self.upper_leg_length**2 + leg_length**2 - self.lower_leg_length**2) / (2 * self.upper_leg_length * leg_length)
            upper_leg = acos(np.clip(cos_upper, -1.0, 1.0))
            
            # 計算腿的總體角度
            leg_angle = atan2(y, x)
            
            # 根據腿的位置調整角度
            if i in [0, 3]:  # Legs 1 and 4
                upper_leg = leg_angle + upper_leg
                lower_leg = pi - lower_leg
            else:  # Legs 2 and 3
                upper_leg = leg_angle - upper_leg
                lower_leg = lower_leg - pi
            
            joint_angles[0, i] = 0  # 肩關節角度設為0
            joint_angles[1, i] = upper_leg
            joint_angles[2, i] = lower_leg

        state.joint_angles = joint_angles
        return joint_angles

    def map_angles_to_servo(self, joint_angles):
        servo_values = np.zeros_like(self.leg_center_position)
        for i in range(4):
            for j in range(3):  # 處理所有三個關節
                angle_diff = joint_angles[j, i] - self.center_angles[j, i]
                
                if j > 0:  # 上腿和下腿
                    if i in [0, 3]:  # 腿 1 和腿 4
                        angle_diff = -angle_diff  # 反轉角度差
                
                servo_value = int(angle_diff * 4095 / (2*pi) + self.leg_center_position[j, i])
                servo_values[j, i] = np.clip(servo_value, 0, 4095)  # 確保值在0-4095範圍內
        return servo_values

if __name__ == "__main__":
    puppy_ik = RobotIK()
    foot_locations = np.array([[10, 10, -10, -10],
                               [-90, -90, -90, -90]])
    
    joint_angles = puppy_ik.leg_inverse_kinematics(foot_locations)
    servo_values = puppy_ik.map_angles_to_servo(joint_angles)
    
    print("Servo values:")
    print(servo_values)
    print("\nDetailed joint information:")
    for i in range(4):
        print(f"Leg {i+1}:")
        print(f"  Shoulder: Angle = {joint_angles[0, i]*180/pi:.2f}°, Servo = {servo_values[0, i]:.0f}")
        print(f"  Upper: Angle = {joint_angles[1, i]*180/pi:.2f}°, Servo = {servo_values[1, i]:.0f}")
        print(f"  Lower: Angle = {joint_angles[2, i]*180/pi:.2f}°, Servo = {servo_values[2, i]:.0f}")