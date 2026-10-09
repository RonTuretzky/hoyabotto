# Verbatim definitions extracted from joycon-robotics/hidapi_for_windows/joyconrobotic_hidapi.py
# Commit 3e37ebfdf1db88fe4ecaa7018ae9c8c49efcde0e. See PROVENANCE.md and bundled licenses.
import numpy as np
import math

class JoyConController:
    """JoyCon控制器（hidapi版本）"""
    
    def __init__(self, reader, init_gpos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], gripper_state=0):
        """初始化控制器
        
        Args:
            reader: JoyConHIDReaderHidapi实例
        """
        self.reader = reader
        self.position = list(init_gpos[0:3])
        self.position_speed = 0.003  # m/step
        
        # Pitch增益（基于实际人体工学：手腕舒适摆动60度 → 机械臂达到83度）
        self.pitch_gain = 1.5  # 适度增益，提升操作舒适度
        
        # 按钮边缘检测（防止反复触发）
        self.last_buttons = {
            'ZR': False,
            'R': False,
            'STICK': False,
            'HOME': False,
        }
        
        # 姿态初始化
        self.roll_offset = 0.0
        self.last_roll = 0.0
        
        # 夹爪状态
        self.gripper_state = gripper_state
        self.gripper_open = 0.5
        self.gripper_close = -0.15
        
        # 初始姿态（用于复位）
        self.init_position = self.position.copy()
        self.init_roll_offset = 0.0
    
    def get_control(self):
        """获取控制指令
        
        Returns:
            pose: [x, y, z, roll, pitch, yaw] (弧度)
            gripper_state: 夹爪状态
            button_control: 按钮控制字典
        """
        state = self.reader.get_state()
        
        # 按钮控制
        button_control = {}
        
        # 位置控制（摇杆 - 第一人称视角）
        stick_x = state['stick_x']
        stick_y = state['stick_y']
        
        # 死区处理（防止漂移）
        deadzone = 0.1
        if abs(stick_x) < deadzone:
            stick_x = 0.0
        if abs(stick_y) < deadzone:
            stick_y = 0.0
        
        # 获取当前姿态（用于计算方向向量）
        roll = state['roll']
        pitch = state['pitch']
        yaw = state['yaw']
        
        # 计算前向方向向量（基于末端姿态）
        # direction_vector = (cos(pitch) * cos(yaw), cos(pitch) * sin(yaw), sin(pitch))
        direction_vector_x = math.cos(pitch) * math.cos(yaw)
        direction_vector_y = math.cos(pitch) * math.sin(yaw)
        direction_vector_z = math.sin(pitch)
        
        # 计算右向方向向量（基于末端姿态）
        # direction_vector_right = (cos(roll) * sin(-yaw), cos(roll) * cos(-yaw), sin(-roll))
        direction_right_x = math.cos(roll) * math.sin(-yaw)
        direction_right_y = math.cos(roll) * math.cos(-yaw)
        direction_right_z = math.sin(-roll)
        
        # 前后移动 - 沿着末端指向的方向（第一人称视角）
        self.position[0] += stick_y * self.position_speed * direction_vector_x
        self.position[1] += stick_y * self.position_speed * direction_vector_y
        self.position[2] += stick_y * self.position_speed * direction_vector_z
        
        # 左右移动 - 沿着末端的横向方向
        self.position[0] -= stick_x * self.position_speed * direction_right_x
        self.position[1] -= stick_x * self.position_speed * direction_right_y
        self.position[2] -= stick_x * self.position_speed * direction_right_z
        
        # 上下（Z轴）- 按键
        if state['buttons'].get('R', False):
            self.position[2] += self.position_speed  # R键上升
        if state['buttons'].get('STICK', False):
            self.position[2] -= self.position_speed  # 摇杆按压下降
        
        # 世界坐标系X轴移动 - 按键（参考说明书）
        if state['buttons'].get('X', False):
            self.position[0] += self.position_speed  # X键向前（世界坐标系X+）
        if state['buttons'].get('B', False):
            self.position[0] -= self.position_speed  # B键向后（世界坐标系X-）
        
        # 夹爪控制（ZR键）- 边缘检测（按下瞬间触发一次）
        zr_pressed = state['buttons'].get('ZR', False)
        if zr_pressed and not self.last_buttons['ZR']:
            # 按钮从未按下变为按下（上升沿）
            self.gripper_state = self.gripper_close if self.gripper_state == self.gripper_open else self.gripper_open
        self.last_buttons['ZR'] = zr_pressed
        
        # Home键复位 - 边缘检测
        home_pressed = state['buttons'].get('HOME', False)
        if home_pressed and not self.last_buttons['HOME']:
            self.position = self.init_position.copy()
            self.roll_offset = self.init_roll_offset
        self.last_buttons['HOME'] = home_pressed
        
        # 姿态控制（陀螺仪）
        # joycon_hid_reader_hidapi已经应用了所有必要的处理：
        # - 加速度计*π
        # - 互补滤波器
        # - 低通滤波器
        # - lerobot模式的Roll缩放
        # 所以这里直接使用，无需额外处理
        roll = state['roll']
        pitch = state['pitch']
        yaw = state['yaw']
        
        # 应用Linux版本的-90度Roll偏移（与lerobot末端坐标系对齐）
        # 参考lerobot_plus_joycon_gpos.py line 82
        roll = roll - np.pi / 2
        
        # Pitch取负并应用增益（基于人体工学优化）
        # 参考lerobot_plus_joycon_gpos.py line 81
        pitch = -pitch * self.pitch_gain  # 手腕摆动60度 → 机械臂83度
        
        # 返回pose
        pose = self.position + [roll, pitch, yaw]
        return pose, self.gripper_state, button_control
    
    def set_position(self, position):
        """设置位置"""
        self.position = list(position)
    
    def disconnect(self):
        """断开连接"""
        self.reader.disconnect()

