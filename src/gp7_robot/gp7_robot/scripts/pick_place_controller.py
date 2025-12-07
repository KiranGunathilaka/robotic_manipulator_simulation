#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import time

from gp7_robot.gp7_kinematics import GP7Kinematics
from gp7_robot.trajectory_planner import TrajectoryPlanner


class PickPlaceController(Node):
    def __init__(self):
        super().__init__('pick_place_controller')
        
        # Initialize kinematics and planner
        self.kinematics = GP7Kinematics()
        self.planner = TrajectoryPlanner()
        
        # Publishers for joint trajectory (EXACTLY as in test code)
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )
        
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.gripper_joint_names = ['finger1_joint', 'finger2_joint']
        
        # Task parameters (from world file)
        # Box A: center at (0.30, 0.40, 0.10), size (0.30, 0.30, 0.20)
        # Box B: center at (0.70, 0.30, 0.025), size (0.05, 0.05, 0.05)
        
        self.box_b_pos = np.array([0.70, 0.30, 0.025])  # Box B on ground
        self.box_a_top = np.array([0.30, 0.40, 0.20 + 0.025])  # Top of Box A + half Box B height
        
        # Safety clearance height for movements
        self.clearance_height = 0.15
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('Pick and Place Controller initialized')
        self.get_logger().info('=' * 70)
    
    def print_joint_angles(self, joint_angles, label="Joint Angles"):
        """Print joint angles in a formatted way"""
        self.get_logger().info(f'\n  {label}:')
        for i, angle in enumerate(joint_angles):
            self.get_logger().info(f'    Joint {i+1}: {angle:8.4f} rad ({np.degrees(angle):7.2f}°)')
    
    def verify_fk(self, joint_angles, expected_pos=None):
        """Verify forward kinematics and print result"""
        T = self.kinematics.forward_kinematics(joint_angles)
        actual_pos = T[:3, 3]
        
        self.get_logger().info(f'  Actual end-effector position:')
        self.get_logger().info(f'    X: {actual_pos[0]:.4f} m')
        self.get_logger().info(f'    Y: {actual_pos[1]:.4f} m')
        self.get_logger().info(f'    Z: {actual_pos[2]:.4f} m')
        
        # Check singularity
        is_singular, manipulability = self.kinematics.check_singularity(joint_angles)
        self.get_logger().info(f'  Manipulability: {manipulability:.4f}')
        
        if is_singular:
            self.get_logger().warn(f'  ⚠ WARNING: Configuration near singularity!')
        
        if expected_pos is not None:
            error = np.linalg.norm(actual_pos - expected_pos)
            self.get_logger().info(f'  Position error: {error:.4f} m ({error*1000:.2f} mm)')
            
            if error > 0.01:
                self.get_logger().warn(f'  ⚠ Warning: Position error exceeds 10mm!')
            else:
                self.get_logger().info(f'  ✓ Position within acceptable tolerance')
    
    def move_to_joint_config(self, target_joints, duration=3.0):
        """Execute movement to target joint configuration"""
        # Create message EXACTLY like test code
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = target_joints
        # Use the EXACT format from test code
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        msg.points.append(point)
        
        self.get_logger().info('-' * 70)
        self.print_joint_angles(target_joints, "Target Joint Configuration")
        self.verify_fk(target_joints)
        
        # Publish
        self.traj_pub.publish(msg)
        self.get_logger().info(f'  ✓ Published to /arm_controller/joint_trajectory')
        self.get_logger().info('-' * 70)
    
    def move_to_cartesian(self, target_pos, duration=3.0, current_joints=None):
        """Move to Cartesian position maintaining upright orientation"""
        if current_joints is None:
            # Safe default starting configuration (avoid singularities)
            current_joints = [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]
        
        self.get_logger().info('-' * 70)
        self.get_logger().info(f'Target Cartesian position:')
        self.get_logger().info(f'  X: {target_pos[0]:.4f} m')
        self.get_logger().info(f'  Y: {target_pos[1]:.4f} m')
        self.get_logger().info(f'  Z: {target_pos[2]:.4f} m')
        
        # Compute inverse kinematics
        target_joints = self.kinematics.inverse_kinematics_upright(target_pos, current_joints)
        
        self.print_joint_angles(target_joints, "Computed IK Solution")
        self.verify_fk(target_joints, target_pos)
        
        # Create message EXACTLY like test code
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        msg.points.append(point)
        
        # Publish
        self.traj_pub.publish(msg)
        self.get_logger().info(f'  ✓ Published to /arm_controller/joint_trajectory')
        self.get_logger().info('-' * 70)
        
        return target_joints
    
    def control_gripper(self, close=True, duration=1.0):
        """Control gripper (close or open)"""
        msg = JointTrajectory()
        msg.joint_names = self.gripper_joint_names
        
        point = JointTrajectoryPoint()
        if close:
            # Close gripper (both fingers move inward)
            point.positions = [-0.3, -0.3]  # Adjust based on actual limits
            self.get_logger().info('  🤏 Closing gripper...')
        else:
            # Open gripper
            point.positions = [0.3, 0.3]
            self.get_logger().info('  ✋ Opening gripper...')
        
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        msg.points.append(point)
        
        self.gripper_pub.publish(msg)
    
    def execute_pick_and_place(self):
        """Execute complete pick and place sequence"""
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info('STARTING PICK AND PLACE SEQUENCE')
        self.get_logger().info('=' * 70 + '\n')
        
        # Start from SAFE home position (NOT all zeros - that's a singularity!)
        home_joints = [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]
        current_joints = home_joints
        
        # Step 1: Move to home position
        self.get_logger().info('\n### STEP 1: Moving to home position ###')
        self.move_to_joint_config(home_joints, duration=3.0)
        time.sleep(4.0)
        
        # Step 2: Move above Box B
        above_box_b = self.box_b_pos + np.array([0, 0, self.clearance_height])
        self.get_logger().info('\n### STEP 2: Moving above Box B ###')
        current_joints = self.move_to_cartesian(above_box_b, duration=4.0, current_joints=current_joints)
        time.sleep(5.0)
        
        # Step 3: Lower to grasp Box B
        grasp_offset = 0.02  # Account for gripper size
        grasp_pos = self.box_b_pos + np.array([0, 0, 0.05 + grasp_offset])
        self.get_logger().info('\n### STEP 3: Lowering to grasp Box B ###')
        current_joints = self.move_to_cartesian(grasp_pos, duration=3.0, current_joints=current_joints)
        time.sleep(4.0)
        
        # Step 4: Close gripper (simulate - actual gripper control would go here)
        self.get_logger().info('\n### STEP 4: Closing gripper ###')
        self.control_gripper(close=True, duration=1.0)
        time.sleep(2.0)
        
        # Step 5: Lift Box B
        self.get_logger().info('\n### STEP 5: Lifting Box B ###')
        current_joints = self.move_to_cartesian(above_box_b, duration=3.0, current_joints=current_joints)
        time.sleep(4.0)
        
        # Step 6: Move above Box A
        above_box_a = self.box_a_top + np.array([0, 0, self.clearance_height])
        self.get_logger().info('\n### STEP 6: Moving above Box A ###')
        current_joints = self.move_to_cartesian(above_box_a, duration=4.0, current_joints=current_joints)
        time.sleep(5.0)
        
        # Step 7: Lower to place on Box A
        place_pos = self.box_a_top + np.array([0, 0, grasp_offset])
        self.get_logger().info('\n### STEP 7: Lowering to place on Box A ###')
        current_joints = self.move_to_cartesian(place_pos, duration=3.0, current_joints=current_joints)
        time.sleep(4.0)
        
        # Step 8: Open gripper
        self.get_logger().info('\n### STEP 8: Opening gripper ###')
        self.control_gripper(close=False, duration=1.0)
        time.sleep(2.0)
        
        # Step 9: Retract
        self.get_logger().info('\n### STEP 9: Retracting ###')
        current_joints = self.move_to_cartesian(above_box_a, duration=3.0, current_joints=current_joints)
        time.sleep(4.0)
        
        # Step 10: Return to home
        self.get_logger().info('\n### STEP 10: Returning to home ###')
        self.move_to_joint_config(home_joints, duration=4.0)
        time.sleep(5.0)
        
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info('✓ PICK AND PLACE SEQUENCE COMPLETE!')
        self.get_logger().info('=' * 70 + '\n')


def main():
    rclpy.init()
    controller = PickPlaceController()
    
    # Wait for ROS systems to initialize
    controller.get_logger().info('Waiting for ROS systems to initialize...')
    time.sleep(3.0)
    
    # Execute pick and place
    try:
        controller.execute_pick_and_place()
    except Exception as e:
        controller.get_logger().error(f'Error during execution: {e}')
        import traceback
        traceback.print_exc()
    
    # Keep node alive
    controller.get_logger().info('Task complete. Node will remain active. Press Ctrl+C to exit.')
    rclpy.spin(controller)
    
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
