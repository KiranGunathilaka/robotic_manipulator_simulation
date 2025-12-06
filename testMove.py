import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TestMove(Node):
    def __init__(self):
        super().__init__("test_move")
        self.pub = self.create_publisher(JointTrajectory,
                                         "/arm_controller/joint_trajectory",
                                         10)
        self.timer = self.create_timer(2.0, self.send_goal)

    def send_goal(self):
        msg = JointTrajectory()
        msg.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        p = JointTrajectoryPoint()
        p.positions = [0.0, -0.5, 0.5, 0.0, 0.0, 0.0]
        p.time_from_start.sec = 1
        msg.points.append(p)

        self.pub.publish(msg)
        self.get_logger().info("Sent simple trajectory")
        self.timer.cancel()

def main():
    rclpy.init()
    node = TestMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
