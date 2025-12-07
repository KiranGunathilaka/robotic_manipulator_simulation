ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /controller_manager
ros2 run controller_manager spawner arm_controller --controller-manager /controller_manager

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