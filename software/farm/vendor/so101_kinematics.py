# Vendored from Vector-Wangel/XLeRobot software/src/model/SO101Robot.py (Apache-2.0); kinematics only.
import math
from typing import List, Tuple, Union
import numpy as np

class SO101Kinematics:
    """
    A class to represent the kinematics of a SO101 robot arm.
    All public methods use degrees for input/output.
    """

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
        
        # Reject impossible targets rather than silently commanding a different
        # point. This is geometry only; motor-unit conversion is a separate step.
        if not all(math.isfinite(v) for v in (x, y, l1, l2)) or min(l1, l2) <= 0:
            raise ValueError("Finite coordinates and positive link lengths required")
        r = math.sqrt(x**2 + y**2)
        r_max = l1 + l2  # Maximum reachable distance
        r_min = abs(l1 - l2)
        if r < r_min - 1e-10 or r > r_max + 1e-10 or r == 0:
            raise ValueError("Target outside the two-link workspace")
        cos_delta = (r**2 - l1**2 - l2**2) / (2 * l1 * l2)
        delta = math.acos(max(-1.0, min(1.0, cos_delta)))
        # FK uses theta1 + theta2 - pi for the forearm direction. Therefore
        # theta2 = pi - delta, not delta. The old inverse mixed these angles.
        theta2 = math.pi - delta
        beta = math.atan2(y, x)
        gamma = math.atan2(l2 * math.sin(delta), l1 + l2 * math.cos(delta))
        theta1 = beta + gamma
        
        # Convert theta1 and theta2 to joint2 and joint3 angles
        joint2 = theta1 + theta1_offset
        joint3 = theta2 + theta2_offset
        
        # Do not conceal an unreachable joint configuration by clamping it.
        if not -0.1 <= joint2 <= 3.45 or not -0.2 <= joint3 <= math.pi:
            raise ValueError("Target exceeds model joint limits")
        
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

    
    def generate_sinusoidal_velocity_trajectory(
        self,
        start_point: Union[List[float], np.ndarray],
        end_point: Union[List[float], np.ndarray],
        control_freq: float = 100.0,  # Hz
        total_time: float = 5.0,      # seconds
        velocity_amplitude: float = 1.0,  # m/s
        velocity_period: float = 2.0,     # seconds
        phase_offset: float = 0.0         # radians
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate a straight-line trajectory with sinusoidal velocity profile.
        
        Parameters:
        -----------
        start_point : array-like
            3D coordinates of starting point [x, y, z]
        end_point : array-like  
            3D coordinates of ending point [x, y, z]
        control_freq : float
            Control frequency in Hz
        total_time : float
            Total trajectory time in seconds
        velocity_amplitude : float
            Amplitude of velocity oscillation in m/s
        velocity_period : float
            Period of velocity oscillation in seconds
        phase_offset : float
            Phase offset in radians
            
        Returns:
        --------
        trajectory : np.ndarray
            Array of 3D positions (n_points, 3)
        velocities : np.ndarray
            Array of velocity magnitudes (n_points,)
        time_array : np.ndarray
            Time array (n_points,)
        """
        
        # Convert to numpy arrays
        start = np.array(start_point, dtype=float)
        end = np.array(end_point, dtype=float)
        
        # Calculate direction and distance
        direction_vector = end - start
        total_distance = np.linalg.norm(direction_vector)
        direction_unit = direction_vector / total_distance if total_distance > 0 else np.zeros(3)
        
        # Generate time array
        dt = 1.0 / control_freq
        n_points = int(total_time * control_freq) + 1
        time_array = np.linspace(0, total_time, n_points)
        
        # Calculate angular frequency
        omega = 2 * np.pi / velocity_period
        
        # Generate sinusoidal velocity profile
        base_velocity = total_distance / total_time  # Average velocity needed
        velocities = base_velocity + velocity_amplitude * np.sin(omega * time_array + phase_offset)
        
        # Ensure non-negative velocities (optional - remove if negative velocities are desired)
        velocities = np.maximum(velocities, 0.1 * base_velocity)
        
        # Integrate velocity to get position along the path
        positions_1d = np.zeros(n_points)
        for i in range(1, n_points):
            positions_1d[i] = positions_1d[i-1] + velocities[i-1] * dt
        
        # Scale positions to fit exactly between start and end points
        if positions_1d[-1] > 0:
            positions_1d = positions_1d * (total_distance / positions_1d[-1])
        
        # Convert 1D positions to 3D trajectory
        trajectory = np.zeros((n_points, 3))
        for i in range(n_points):
            progress = positions_1d[i] / total_distance if total_distance > 0 else 0
            trajectory[i] = start + progress * direction_vector
        
        return trajectory, velocities, time_array
    # Example usage
    # if __name__ == "__main__":
    #     # Define start and end points
    #     start = [0, 0, 0]
    #     end = [5, 3, 2]
        
    #     # Generate trajectory
    #     trajectory, velocities, time_array = generate_sinusoidal_velocity_trajectory(
    #         start_point=start,
    #         end_point=end,
    #         control_freq=100.0,
    #         total_time=6.0,
    #         velocity_amplitude=0.8,
    #         velocity_period=1.5,
    #         phase_offset=0
    #     )
        
    #     print(f"Generated {len(trajectory)} trajectory points")
    #     print(f"Total distance: {np.linalg.norm(np.array(end) - np.array(start)):.3f}")
    #     print(f"Time duration: {time_array[-1]:.2f} seconds")
    #     print(f"Average velocity: {velocities.mean():.3f} m/s")
    #     print(f"Velocity range: {velocities.min():.3f} to {velocities.max():.3f} m/s")
        
    #     print("\nFirst few trajectory points:")
    #     for i in range(0, min(10, len(trajectory)), 2):
    #         print(f"t={time_array[i]:.2f}s: pos=[{trajectory[i,0]:.3f}, {trajectory[i,1]:.3f}, {trajectory[i,2]:.3f}], vel={velocities[i]:.3f} m/s")
