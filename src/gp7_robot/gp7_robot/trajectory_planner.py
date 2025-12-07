#!/usr/bin/env python3
import numpy as np

class TrajectoryPlanner:
    def __init__(self):
        pass
    
    def trapezoidal_velocity(self, t, t_total, accel_time=0.3):
        """
        Generate trapezoidal velocity profile
        Args:
            t: current time
            t_total: total trajectory time
            accel_time: acceleration/deceleration time (fraction of total)
        Returns:
            s: normalized position [0, 1]
            s_dot: normalized velocity
        """
        t_accel = t_total * accel_time
        t_cruise = t_total - 2 * t_accel
        
        if t <= t_accel:
            # Acceleration phase
            s = 0.5 * (t / t_accel) ** 2 * accel_time
            s_dot = (t / t_accel) * accel_time / t_total
        elif t <= t_accel + t_cruise:
            # Cruise phase
            s = accel_time / 2 + (t - t_accel) / t_total
            s_dot = 1.0 / t_total
        elif t <= t_total:
            # Deceleration phase
            t_decel = t - (t_accel + t_cruise)
            s = 1.0 - 0.5 * ((t_total - t) / t_accel) ** 2 * accel_time
            s_dot = ((t_total - t) / t_accel) * accel_time / t_total
        else:
            s = 1.0
            s_dot = 0.0
        
        return s, s_dot
    
    def quintic_polynomial(self, t, t_total):
        """
        Quintic (5th order) polynomial for smooth trajectory
        Better smoothness than trapezoidal
        """
        if t >= t_total:
            return 1.0, 0.0, 0.0
        
        tau = t / t_total
        
        # Quintic: s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        s_dot = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / t_total
        s_ddot = (60 * tau - 180 * tau**2 + 120 * tau**3) / (t_total**2)
        
        return s, s_dot, s_ddot
    
    def interpolate_joints(self, q_start, q_end, num_points, trajectory_time):
        """
        Generate waypoints between start and end joint configurations
        Args:
            q_start: starting joint angles [6]
            q_end: ending joint angles [6]
            num_points: number of waypoints
            trajectory_time: total time for trajectory
        Returns:
            waypoints: list of (joint_positions, time_from_start)
        """
        q_start = np.array(q_start)
        q_end = np.array(q_end)
        waypoints = []
        
        for i in range(num_points):
            t = (i / (num_points - 1)) * trajectory_time
            s, _, _ = self.quintic_polynomial(t, trajectory_time)
            
            q = q_start + s * (q_end - q_start)
            waypoints.append((q.tolist(), t))
        
        return waypoints
    
    def plan_cartesian_path(self, kinematics, start_pos, end_pos, 
                           num_waypoints=10, initial_guess=None):
        """
        Plan a straight-line path in Cartesian space
        Args:
            kinematics: GP7Kinematics instance
            start_pos: [x, y, z] starting position
            end_pos: [x, y, z] ending position
            num_waypoints: number of intermediate waypoints
            initial_guess: initial joint configuration
        Returns:
            joint_waypoints: list of joint configurations
        """
        joint_waypoints = []
        
        if initial_guess is None:
            initial_guess = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        start_pos = np.array(start_pos)
        end_pos = np.array(end_pos)
        
        for i in range(num_waypoints):
            alpha = i / (num_waypoints - 1)
            target = start_pos + alpha * (end_pos - start_pos)
            
            # Use previous solution as initial guess for continuity
            q = kinematics.inverse_kinematics_upright(target.tolist(), initial_guess)
            joint_waypoints.append(q)
            initial_guess = q  # Update for next iteration
        
        return joint_waypoints
