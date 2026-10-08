# Verbatim definitions extracted from XLeRobot/software/examples/7_xlerobot_2wheels_teleop_joycon.py
# Commit b017b5e6354bd9f61f4247a920c72622ca0aade0. See PROVENANCE.md and bundled licenses.
import numpy as np
# Route upstream per-frame debug output away from the HID JSON stream.
import logging
def print(*args, **kwargs):
    logging.getLogger(__name__).debug(" ".join(map(str, args)))

LEFT_JOINT_MAP = {
    "shoulder_pan": "left_arm_shoulder_pan",
    "shoulder_lift": "left_arm_shoulder_lift",
    "elbow_flex": "left_arm_elbow_flex",
    "wrist_flex": "left_arm_wrist_flex",
    "wrist_roll": "left_arm_wrist_roll",
    "gripper": "left_arm_gripper",
}


RIGHT_JOINT_MAP = {
    "shoulder_pan": "right_arm_shoulder_pan",
    "shoulder_lift": "right_arm_shoulder_lift",
    "elbow_flex": "right_arm_elbow_flex",
    "wrist_flex": "right_arm_wrist_flex",
    "wrist_roll": "right_arm_wrist_roll",
    "gripper": "right_arm_gripper",
}


HEAD_MOTOR_MAP = {
    "head_motor_1": "head_motor_1",
    "head_motor_2": "head_motor_2",
}


class SimpleTeleopArm:
    def __init__(self, joint_map, initial_obs, kinematics, prefix="right", kp=1):
        self.joint_map = joint_map
        self.prefix = prefix
        self.kp = kp
        self.kinematics = kinematics
        
        # Initial joint positions
        self.joint_positions = {
            "shoulder_pan": initial_obs[f"{prefix}_arm_shoulder_pan.pos"],
            "shoulder_lift": initial_obs[f"{prefix}_arm_shoulder_lift.pos"],
            "elbow_flex": initial_obs[f"{prefix}_arm_elbow_flex.pos"],
            "wrist_flex": initial_obs[f"{prefix}_arm_wrist_flex.pos"],
            "wrist_roll": initial_obs[f"{prefix}_arm_wrist_roll.pos"],
            "gripper": initial_obs[f"{prefix}_arm_gripper.pos"],
        }
        
        # Set initial x/y to fixed values
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        
        # Set step size
        self.degree_step = 2
        self.xy_step = 0.005
        
        # P control target positions, set to zero position
        self.target_positions = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        }
        self.zero_pos = {
            'shoulder_pan': 0.0,
            'shoulder_lift': 0.0,
            'elbow_flex': 0.0,
            'wrist_flex': 0.0,
            'wrist_roll': 0.0,
            'gripper': 0.0
        }


    def handle_joycon_input(self, joycon_pose, gripper_state):
        """Handle Joy-Con input, update arm control - based on 6_so100_joycon_ee_control.py"""
        x, y, z, roll_, pitch_, yaw = joycon_pose
        
        # Calculate pitch control - consistent with 6_so100_joycon_ee_control.py
        pitch = -pitch_ * 60 + 10
        
        # Set coordinates - consistent with 6_so100_joycon_ee_control.py
        current_x = 0.1629 + x
        current_y = 0.1131 + z
        
        # Calculate roll - consistent with 6_so100_joycon_ee_control.py
        roll = roll_ * 45
        
        print(f"[{self.prefix}] pitch: {pitch}")
        
        # Add y value to control shoulder_pan joint - consistent with 6_so100_joycon_ee_control.py
        y_scale = 250.0  # Scaling factor, can be adjusted as needed
        self.target_positions["shoulder_pan"] = y * y_scale
        
        # Use inverse kinematics to calculate joint angles - consistent with 6_so100_joycon_ee_control.py
        try:
            joint2_target, joint3_target = self.kinematics.inverse_kinematics(current_x, current_y)
            self.target_positions["shoulder_lift"] = joint2_target
            self.target_positions["elbow_flex"] = joint3_target
        except Exception as e:
            print(f"[{self.prefix}] IK failed: {e}")
        
        # Set wrist_flex - consistent with 6_so100_joycon_ee_control.py
        self.target_positions["wrist_flex"] = -self.target_positions["shoulder_lift"] - self.target_positions["elbow_flex"] + pitch
        
        # Set wrist_roll - consistent with 6_so100_joycon_ee_control.py
        self.target_positions["wrist_roll"] = roll
        
        # Gripper control - now set directly in main loop, no need to handle here
        pass


    def p_control_action(self, robot):
        obs = robot.get_observation()
        current = {j: obs[f"{self.prefix}_arm_{j}.pos"] for j in self.joint_map}
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - current[j]
            control = self.kp * error
            action[f"{self.joint_map[j]}.pos"] = current[j] + control
        return action


class SimpleHeadControl:
    def __init__(self, initial_obs, kp=1):
        self.kp = kp
        self.degree_step = 2  # Move 2 degrees each time
        # Initialize head motor positions
        self.target_positions = {
            "head_motor_1": initial_obs.get("head_motor_1.pos", 0.0),
            "head_motor_2": initial_obs.get("head_motor_2.pos", 0.0),
        }
        self.zero_pos = {"head_motor_1": 0.0, "head_motor_2": 0.0}


    def handle_joycon_input(self, joycon):
        """Handle left Joy-Con directional pad input to control head motors"""
        # Get left Joy-Con directional pad state
        button_up = joycon.joycon.get_button_up()      # Up: head_motor_1+
        button_down = joycon.joycon.get_button_down()  # Down: head_motor_1-
        button_left = joycon.joycon.get_button_left()  # Left: head_motor_2+
        button_right = joycon.joycon.get_button_right() # Right: head_motor_2-
        
        if button_up == 1:
            self.target_positions["head_motor_2"] += self.degree_step
            print(f"[HEAD] head_motor_2: {self.target_positions['head_motor_2']}")
        if button_down == 1:
            self.target_positions["head_motor_2"] -= self.degree_step
            print(f"[HEAD] head_motor_2: {self.target_positions['head_motor_2']}")
        if button_left == 1:
            self.target_positions["head_motor_1"] += self.degree_step
            print(f"[HEAD] head_motor_1: {self.target_positions['head_motor_1']}")
        if button_right == 1:
            self.target_positions["head_motor_1"] -= self.degree_step
            print(f"[HEAD] head_motor_1: {self.target_positions['head_motor_1']}")


    def p_control_action(self, robot):
        obs = robot.get_observation()
        action = {}
        for motor in self.target_positions:
            current = obs.get(f"{HEAD_MOTOR_MAP[motor]}.pos", 0.0)
            error = self.target_positions[motor] - current
            control = self.kp * error
            action[f"{HEAD_MOTOR_MAP[motor]}.pos"] = current + control
        return action


def get_joycon_base_action(joycon, robot):
    """
    Get base control commands from Joy-Con for differential drive
    X: left turn, B: right turn, Y: forward, A: backward
    """
    # Get button states
    button_x = joycon.joycon.get_button_x()  # left turn
    button_b = joycon.joycon.get_button_b()  # right turn
    button_y = joycon.joycon.get_button_y()  # forward
    button_a = joycon.joycon.get_button_a()  # backward
    
    # Build key set (simulate keyboard input)
    pressed_keys = set()
    
    if button_x == 1:
        pressed_keys.add(robot.teleop_keys['rotate_left'])  # left turn
        print("[BASE] Left turn")
    if button_b == 1:
        pressed_keys.add(robot.teleop_keys['rotate_right'])  # right turn
        print("[BASE] Right turn")
    if button_y == 1:
        pressed_keys.add(robot.teleop_keys['forward'])  # forward
        print("[BASE] Forward")
    if button_a == 1:
        pressed_keys.add(robot.teleop_keys['backward'])  # backward
        print("[BASE] Backward")
    
    # Convert to numpy array and get base action
    keyboard_keys = np.array(list(pressed_keys))
    base_action = robot._from_keyboard_to_base_action(keyboard_keys) or {}
    
    return base_action

