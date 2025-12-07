#!/usr/bin/env python3
import numpy as np
from scipy.optimize import least_squares

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
            [0.04, np.pi/2, 0.0, 0.0],
            # Joint 5: Link4 to Link5
            [0.026, -np.pi/2, 0.441, 0.0],
            # Joint 6: Link5 to EE
            [0.0, -np.pi/2, 0.0, 0.0]  # Include EE offset in d6
        ]
        
        # Joint limits [lower, upper] in radians (from URDF)
        self.joint_limits = [
            [-2.96706, 2.96706],   # ±170°
            [-1.13446, 2.53073],   # +145° / -65°
            [-1.22173, 3.31613],   # +190° / -70°
            [-3.31613, 3.31613],   # ±190°
            [-2.35619, 2.35619],   # ±135°
            [-6.28318, 6.28318]    # ±360°
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
        R = T[:3, :3]
        return position, R
    
    def clip_to_limits(self, joint_angles):
        """Clip joint angles to valid limits"""
        clipped = np.zeros(6)
        for i in range(6):
            clipped[i] = np.clip(joint_angles[i], 
                                self.joint_limits[i][0], 
                                self.joint_limits[i][1])
        return clipped
    
    def check_singularity(self, joint_angles, threshold=0.01):
        """
        Check if configuration is near singularity by checking Jacobian determinant
        Returns: (is_singular, manipulability)
        """
        J = self.get_jacobian(joint_angles)
        # Use position Jacobian only (first 3 rows)
        J_pos = J[:3, :]
        
        # Calculate manipulability measure (Yoshikawa's)
        manipulability = np.sqrt(np.linalg.det(J_pos @ J_pos.T))
        
        is_singular = manipulability < threshold
        return is_singular, manipulability
    
    def inverse_kinematics(self, target_pos, target_orient=None, initial_guess=None, 
                          prefer_minimal_motion=True):
        """
        Compute inverse kinematics using numerical optimization
        Args:
            target_pos: [x, y, z] target position
            target_orient: 3x3 rotation matrix (optional)
            initial_guess: initial joint angles (optional)
            prefer_minimal_motion: if True, penalize large changes from initial guess
        Returns:
            joint_angles: list of 6 joint angles
        """
        if initial_guess is None:
            # Better default initial guess (avoids singularities)
            initial_guess = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        else:
            initial_guess = np.array(initial_guess)
        
        target_pos = np.array(target_pos)
        
        def objective(q):
            T = self.forward_kinematics(q)
            pos_error = T[:3, 3] - target_pos
            
            # Add singularity avoidance penalty
            is_singular, manipulability = self.check_singularity(q)
            singularity_penalty = 0
            if manipulability < 0.05:
                singularity_penalty = (0.05 - manipulability) * 10
            
            # Add minimal motion penalty to prefer solutions close to initial guess
            motion_penalty = np.zeros(6)
            if prefer_minimal_motion:
                # Penalize large joint movements, with extra weight on base joint (joint 1)
                joint_weights = np.array([2.0, 1.0, 1.0, 0.5, 0.5, 0.3])
                motion_penalty = (q - initial_guess) * joint_weights * 0.1
            
            if target_orient is not None:
                # Orientation error using rotation matrix difference
                R_current = T[:3, :3]
                R_error = R_current @ target_orient.T - np.eye(3)
                # Use scaled orientation error
                orient_error = R_error.flatten()[:3] * 0.3
                return np.concatenate([pos_error, orient_error, [singularity_penalty], motion_penalty])
            
            return np.concatenate([pos_error, [singularity_penalty], motion_penalty])
        
        # Use least_squares for better convergence
        result = least_squares(
            objective, 
            initial_guess,
            bounds=(
                [limit[0] for limit in self.joint_limits],
                [limit[1] for limit in self.joint_limits]
            ),
            ftol=1e-6,
            xtol=1e-6,
            max_nfev=2000,
            verbose=0
        )
        
        joint_angles = result.x
        
        # Verify solution
        T_final = self.forward_kinematics(joint_angles)
        error = np.linalg.norm(T_final[:3, 3] - target_pos)
        
        # Check singularity
        is_singular, manipulability = self.check_singularity(joint_angles)
        
        if error > 0.01:  # 1cm threshold
            print(f"Warning: IK solution has high error: {error:.4f}m")
        
        if is_singular:
            print(f"Warning: IK solution near singularity (manipulability: {manipulability:.4f})")
        
        return joint_angles.tolist()
    
    def inverse_kinematics_upright(self, target_pos, initial_guess=None):
        """
        IK maintaining upright orientation (gripper pointing down)
        Useful for pick-and-place without rotation
        """
        # Target orientation: gripper Z-axis points down
        target_orient = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])
        
        return self.inverse_kinematics(target_pos, target_orient, initial_guess)
    
    def inverse_kinematics_vertical_motion(self, target_pos, current_joints):
        """
        IK for vertical motion - locks base joint (joint1) to avoid unnecessary rotation
        Use this for straight up/down movements where base shouldn't rotate
        """
        # For vertical motion, we want to lock joint 1
        target_orient = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])
        
        current_joints = np.array(current_joints)
        
        def objective(q):
            T = self.forward_kinematics(q)
            pos_error = T[:3, 3] - target_pos
            
            # STRONG penalty for joint 1 movement (base rotation)
            # When moving vertically, joint 1 should NOT change
            base_rotation_penalty = (q[0] - current_joints[0]) * 5.0  # Heavy weight
            
            # Moderate penalty for other joint movements
            other_motion_penalty = (q[1:] - current_joints[1:]) * 0.5
            
            # Singularity avoidance
            is_singular, manipulability = self.check_singularity(q)
            singularity_penalty = 0
            if manipulability < 0.05:
                singularity_penalty = (0.05 - manipulability) * 10
            
            # Orientation error
            R_current = T[:3, :3]
            R_error = R_current @ target_orient.T - np.eye(3)
            orient_error = R_error.flatten()[:3] * 0.3
            
            return np.concatenate([
                pos_error, 
                orient_error, 
                [singularity_penalty],
                [base_rotation_penalty],
                other_motion_penalty
            ])
        
        result = least_squares(
            objective,
            current_joints,
            bounds=(
                [limit[0] for limit in self.joint_limits],
                [limit[1] for limit in self.joint_limits]
            ),
            ftol=1e-6,
            xtol=1e-6,
            max_nfev=2000,
            verbose=0
        )
        
        joint_angles = result.x
        
        # Verify solution
        T_final = self.forward_kinematics(joint_angles)
        error = np.linalg.norm(T_final[:3, 3] - target_pos)
        
        # Check if joint 1 stayed relatively fixed
        base_change = abs(joint_angles[0] - current_joints[0])
        if base_change > 0.1:  # More than ~5 degrees
            print(f"Warning: Base joint changed by {np.degrees(base_change):.2f}° during vertical motion")
        
        if error > 0.01:
            print(f"Warning: IK solution has high error: {error:.4f}m")
        
        return joint_angles.tolist()
    
    def get_jacobian(self, joint_angles):
        """
        Compute geometric Jacobian matrix
        Returns 6xN Jacobian (3 for position, 3 for orientation)
        """
        epsilon = 1e-6
        J = np.zeros((6, 6))
        
        # Get current end-effector pose
        T0 = self.forward_kinematics(joint_angles)
        p0 = T0[:3, 3]
        R0 = T0[:3, :3]
        
        for i in range(6):
            # Perturb joint i
            q_plus = list(joint_angles)
            q_plus[i] += epsilon
            
            T_plus = self.forward_kinematics(q_plus)
            p_plus = T_plus[:3, 3]
            R_plus = T_plus[:3, :3]
            
            # Position derivative
            J[:3, i] = (p_plus - p0) / epsilon
            
            # Orientation derivative (simplified)
            R_diff = (R_plus - R0) / epsilon
            J[3:, i] = [R_diff[2, 1], R_diff[0, 2], R_diff[1, 0]]
        
        return J
