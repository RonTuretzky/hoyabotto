# Verbatim definitions extracted from XLeRobot/software/src/model/SO101Robot.py
# Commit b017b5e6354bd9f61f4247a920c72622ca0aade0. See PROVENANCE.md and bundled licenses.
import math

class SO101Kinematics:
    def __init__(self, l1=0.1159, l2=0.1350):
        self.l1 = l1  # Length of the first link (upper arm)
        self.l2 = l2  # Length of the second link (lower arm)


    def inverse_kinematics(self, x, y, l1=None, l2=None):
        """
        Calculate inverse kinematics for a 2-link robotic arm, considering joint offsets
        
        Parameters:
            x: End effector x coordinate
            y: End effector y coordinate
            l1: Upper arm length (default uses instance value)
            l2: Lower arm length (default uses instance value)
            
        Returns:
            joint2_deg, joint3_deg: Joint angles in degrees (shoulder_lift, elbow_flex)
        """
        # Use instance values if not provided
        if l1 is None:
            l1 = self.l1
        if l2 is None:
            l2 = self.l2
            
        # Calculate joint2 and joint3 offsets in theta1 and theta2
        theta1_offset = math.atan2(0.028, 0.11257)  # theta1 offset when joint2=0
        theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset  # theta2 offset when joint3=0
        
        # Calculate distance from origin to target point
        r = math.sqrt(x**2 + y**2)
        r_max = l1 + l2  # Maximum reachable distance
        
        # If target point is beyond maximum workspace, scale it to the boundary
        if r > r_max:
            scale_factor = r_max / r
            x *= scale_factor
            y *= scale_factor
            r = r_max
        
        # If target point is less than minimum workspace (|l1-l2|), scale it
        r_min = abs(l1 - l2)
        if r < r_min and r > 0:
            scale_factor = r_min / r
            x *= scale_factor
            y *= scale_factor
            r = r_min
        
        # Use law of cosines to calculate theta2
        cos_theta2 = -(r**2 - l1**2 - l2**2) / (2 * l1 * l2)
        
        # Clamp cos_theta2 to valid range [-1, 1] to avoid domain errors
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))
        
        # Calculate theta2 (elbow angle)
        theta2 = math.pi - math.acos(cos_theta2)
        
        # Calculate theta1 (shoulder angle)
        beta = math.atan2(y, x)
        gamma = math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
        theta1 = beta + gamma
        
        # Convert theta1 and theta2 to joint2 and joint3 angles
        joint2 = theta1 + theta1_offset
        joint3 = theta2 + theta2_offset
        
        # Ensure angles are within URDF limits
        joint2 = max(-0.1, min(3.45, joint2))
        joint3 = max(-0.2, min(math.pi, joint3))
        
        # Convert from radians to degrees
        joint2_deg = math.degrees(joint2)
        joint3_deg = math.degrees(joint3)

        # Apply coordinate system transformation
        joint2_deg = 90 - joint2_deg
        joint3_deg = joint3_deg - 90
        
        return joint2_deg, joint3_deg


    def forward_kinematics(self, joint2_deg, joint3_deg, l1=None, l2=None):
        """
        Calculate forward kinematics for a 2-link robotic arm
        
        Parameters:
            joint2_deg: Shoulder lift joint angle in degrees
            joint3_deg: Elbow flex joint angle in degrees
            l1: Upper arm length (default uses instance value)
            l2: Lower arm length (default uses instance value)
            
        Returns:
            x, y: End effector coordinates
        """
        # Use instance values if not provided
        if l1 is None:
            l1 = self.l1
        if l2 is None:
            l2 = self.l2
            
        # Convert degrees to radians and apply inverse transformation
        joint2_rad = math.radians(90 - joint2_deg)
        joint3_rad = math.radians(joint3_deg + 90)
        
        # Calculate joint2 and joint3 offsets
        theta1_offset = math.atan2(0.028, 0.11257)
        theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
        
        # Convert joint angles back to theta1 and theta2
        theta1 = joint2_rad - theta1_offset
        theta2 = joint3_rad - theta2_offset
        
        # Forward kinematics calculations
        x = l1 * math.cos(theta1) + l2 * math.cos(theta1 + theta2 - math.pi)
        y = l1 * math.sin(theta1) + l2 * math.sin(theta1 + theta2 - math.pi)
        
        return x, y

