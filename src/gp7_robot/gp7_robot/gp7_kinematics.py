import numpy as np
from scipy.optimize import fsolve

class GP7Kinematics:
    def __init__(self):
        # DH Parameters [a, alpha, d, theta_offset]
        self.dh_params = [
            # Joint 1: Base to Link1
            [0.0, 0.0, 0.1829, 0.0],
            # Joint 2: Link1 to Link2  
            [0.04, np.pi/2, 0.1355, np.pi],
            # Joint 3: Link2 to Link3
            [0.445, 0.0, 0.0, -np.pi/2],
            # Joint 4: Link3 to Link4
            [-0.04, -np.pi/2, 0.0, 0.0],
            # Joint 5: Link4 to Link5
            [0.026, np.pi/2, 0.44, 0.0],
            # Joint 6: Link5 to EE
            [0.0, -np.pi/2, 0.0, 0.0]
        ]
        
        # Joint limits [lower, upper] in radians
        self.joint_limits = [
            [-2.96706, 2.96706],
            [-1.13446, 2.53073],
            [-1.22173, 3.31613],
            [-3.31613, 3.31613],
            [-2.35619, 2.35619],
            [-6.28318, 6.28318]
        ]
    
    def dh_transform(self, a, alpha, d, theta):
        """Create DH transformation matrix"""
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
    
    def forward_kinematics(self, joint_angles):
        """
        Compute forward kinematics
        Args:
            joint_angles: list of 6 joint angles in radians
        Returns:
            4x4 transformation matrix from base to end-effector
        """
        T = np.eye(4)
        
        for i, (a, alpha, d, offset) in enumerate(self.dh_params):
            theta = joint_angles[i] + offset
            T = T @ self.dh_transform(a, alpha, d, theta)
        
        return T
    
    def get_position_orientation(self, T):
        """Extract position and orientation from transformation matrix"""
        position = T[:3, 3]
        # ZYZ Euler angles
        R = T[:3, :3]
        return position, R
    
    def inverse_kinematics(self, target_pos, target_orient=None, initial_guess=None):
        """
        Compute inverse kinematics using numerical optimization
        Args:
            target_pos: [x, y, z] target position
            target_orient: 3x3 rotation matrix (optional)
            initial_guess: initial joint angles (optional)
        Returns:
            joint_angles: list of 6 joint angles
        """
        if initial_guess is None:
            initial_guess = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        def objective(q):
            T = self.forward_kinematics(q)
            pos_error = T[:3, 3] - target_pos
            
            if target_orient is not None:
                # Orientation error using rotation matrix difference
                R_error = T[:3, :3] @ target_orient.T - np.eye(3)
                orient_error = R_error.flatten()[:3]  # Use first row for simplicity
                return np.concatenate([pos_error, orient_error * 0.1])
            
            return pos_error
        
        # Solve using numerical optimization
        result = fsolve(objective, initial_guess, full_output=True)
        joint_angles = result[0]
        
        # Check joint limits
        for i, (lower, upper) in enumerate(self.joint_limits):
            joint_angles[i] = np.clip(joint_angles[i], lower, upper)
        
        return joint_angles
    
    def inverse_kinematics_upright(self, target_pos, initial_guess=None):
        """
        IK maintaining upright orientation (Z-axis pointing up)
        Useful for pick-and-place without rotation
        """
        # Target orientation: Z-axis of gripper points down
        target_orient = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])
        
        return self.inverse_kinematics(target_pos, target_orient, initial_guess)
