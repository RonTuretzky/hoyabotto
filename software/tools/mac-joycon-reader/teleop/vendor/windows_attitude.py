# Verbatim definitions extracted from joycon-robotics/hidapi_for_windows/joycon_hidapi_reader.py
# Commit 3e37ebfdf1db88fe4ecaa7018ae9c8c49efcde0e. See PROVENANCE.md and bundled licenses.
import numpy as np
import threading

class JoyConHIDAPIReader:
    def __init__(self):
        """初始化HID读取器"""
        self.device = None
        self.running = False
        self.thread = None
        
        # IMU原始数据（6轴）
        self.gyro = np.array([0.0, 0.0, 0.0])      # 陀螺仪 (rad/s)
        self.accel = np.array([0.0, 0.0, 0.0])     # 加速度计 (g)
        
        # 按钮和摇杆状态
        self.buttons = {}
        self.stick_x = 0.0  # -1.0 到 1.0
        self.stick_y = 0.0  # -1.0 到 1.0
        
        # 姿态估计（弧度）
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # 校准数据
        self.gyro_offset = np.array([0.0, 0.0, 0.0])
        self.roll_offset = 0.0
        
        # 互补滤波器参数（参考JoyconRobotics）
        self.alpha = 0.55  # 陀螺仪权重（与Linux版本一致）
        self.dt = 0.01  # 固定时间步长（与Linux版本一致）
        
        # 低通滤波器参数（参考JoyconRobotics）
        self.lpf_alpha = 0.08  # lerobot模式
        self.lpf_roll_prev = 0.0
        self.lpf_pitch_prev = 0.0
        
        # Yaw方向向量（用于四元数旋转，简化版）
        self.yaw_integrated = 0.0
        
        # 包计数器（用于发送子命令）
        self.packet_number = 0
        
        # 数据锁
        self.lock = threading.Lock()


    def _update_attitude(self):
        """更新姿态估计（严格参考JoyconRobotics的AttitudeEstimator）"""
        # 重置pitch和roll（将从头计算）
        pitch_gyro = 0.0
        roll_gyro = 0.0
        
        # 加速度计数据处理（关键：乘以π，与Linux版本一致）
        ax = self.accel[0] * np.pi
        ay = self.accel[1] * np.pi
        az = self.accel[2] * np.pi
        
        # 陀螺仪数据
        gx, gy, gz = self.gyro[0], self.gyro[1], self.gyro[2]
        
        # 从加速度计计算Roll和Pitch（与Linux版本一致）
        # 注意：roll_acc使用-az（负号很重要！）
        roll_acc = np.arctan2(ay, -az)
        pitch_acc = np.arctan2(ax, np.sqrt(ay**2 + az**2))
        
        # 陀螺仪积分（注意：Roll是减号！）
        pitch_gyro += gy * self.dt
        roll_gyro -= gx * self.dt  # 关键：减号！
        
        # 互补滤波器（与Linux版本一致：alpha=0.55）
        self.pitch = self.alpha * pitch_gyro + (1 - self.alpha) * pitch_acc
        self.roll = self.alpha * roll_gyro + (1 - self.alpha) * roll_acc
        
        # 低通滤波器（与Linux版本一致）
        self.pitch = self.lpf_alpha * self.pitch + (1 - self.lpf_alpha) * self.lpf_pitch_prev
        self.roll = self.lpf_alpha * self.roll + (1 - self.lpf_alpha) * self.lpf_roll_prev
        
        self.lpf_pitch_prev = self.pitch
        self.lpf_roll_prev = self.roll
        
        # Yaw积分（简化版，不使用四元数）
        self.yaw_integrated += gz * self.dt
        self.yaw = -self.yaw_integrated  # 注意：负号
        
        # lerobot模式的Roll缩放（与Linux版本一致）
        self.roll = self.roll * np.pi / 2


    def get_state(self):
        """获取当前状态"""
        with self.lock:
            return {
                'gyro': self.gyro.copy(),
                'accel': self.accel.copy(),
                'roll': self.roll,
                'pitch': self.pitch,
                'yaw': self.yaw,
                'stick_x': self.stick_x,
                'stick_y': self.stick_y,
                'buttons': self.buttons.copy()
            }

