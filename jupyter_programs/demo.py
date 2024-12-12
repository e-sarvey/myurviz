import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Int32MultiArray
import time

'''
This demo program executes a simple pick and place series of joint angle actions and gripper publish messages. Client nodes are initialized and interact with the arm vizualization node with the name space "/simulated_arm_server_1/" 
'''

class GripperControlNode(Node):
    """Node to handle gripper control."""
    def __init__(self):
        super().__init__('gripper_control_node')
        self.gripper_pub = self.create_publisher(Int32MultiArray, '/simulated_arm_server1/gripper/control', 10)
        self.get_logger().info("GripperControlNode initialized.")

    def control_gripper(self, position, speed):
        """Publish a gripper control message."""
        msg = Int32MultiArray(data=[position, speed, speed])
        self.get_logger().info(f"Publishing gripper control: Position={position}, Speed={speed}")
        self.gripper_pub.publish(msg)
        time.sleep(1)  # Allow time for the action to take effect


class TrajectoryControlNode(Node):
    """Node to handle trajectory control."""
    def __init__(self):
        super().__init__('trajectory_control_node')
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/simulated_arm_server_1/follow_joint_trajectory')
        self.get_logger().info("TrajectoryControlNode initialized.")

    def send_trajectory_goal(self, joint_names, points):
        """Send a trajectory goal to the action server."""
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Trajectory action server not available.")
            return False

        self.get_logger().info("Sending trajectory goal:")
        for point in points:
            self.get_logger().info(f"Positions: {point['positions']}, Time: {point['time_from_start_sec']}s")

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = [
            JointTrajectoryPoint(positions=point['positions'],
                                 time_from_start=rclpy.time.Duration(seconds=point['time_from_start_sec']).to_msg())
            for point in points
        ]

        future = self.trajectory_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected.")
            return False

        self.get_logger().info("Trajectory goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Trajectory execution completed successfully.")
            return True
        else:
            self.get_logger().error(f"Trajectory execution failed with error code: {result.result.error_code}")
            return False


def run_demo(gripper_node, trajectory_node):
    """Run a demo sequence of gripper and trajectory control."""

    # Move to a starting position
    trajectory_node.get_logger().info("Moving to starting position...")
    starting_position = {
        "joint_names": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
        "points": [{"positions": [0, -3.1416, 1.5708, -1.5708, 1.0647, 0], "time_from_start_sec": 5}],
    }
    trajectory_node.send_trajectory_goal(**starting_position)

    # Open gripper
    gripper_node.get_logger().info("Opening gripper...")
    gripper_node.control_gripper(position=0, speed=127)

    # Move to pickup position
    trajectory_node.get_logger().info("Moving to pickup location...")
    pickup_trajectory = {
        "joint_names": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
        "points": [
            {"positions": [-1.0996, -2.0595, -0.76794, -1.8675, 1.6057, 0.29671], "time_from_start_sec": 5},
            {"positions": [-1.1519, -2.0944, -1.7802, -0.80285, 1.5882, 1.885], "time_from_start_sec": 10}
        ],
    }
    trajectory_node.send_trajectory_goal(**pickup_trajectory)

    # Close gripper
    gripper_node.get_logger().info("Closing gripper...")
    gripper_node.control_gripper(position=200, speed=127)

    # Move to drop-off position
    trajectory_node.get_logger().info("Moving to drop-off location...")
    dropoff_trajectory = {
        "joint_names": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
        "points": [
            {"positions": [-1.1519, -1.7453, -0.57596, -2.2689, 1.5882, 1.885], "time_from_start_sec": 4},
            {"positions": [-0.89012, -1.8326, 1.1868, 0.66323, 2.1991, 1.9199], "time_from_start_sec": 8},
            {"positions": [-1.6406, -1.0821, 0.97738, 0.17453, 1.4137, 3.194], "time_from_start_sec": 12}
        ],
    }
    trajectory_node.send_trajectory_goal(**dropoff_trajectory)

    # Open gripper
    gripper_node.get_logger().info("Opening gripper...")
    gripper_node.control_gripper(position=0, speed=127)


def main(args=None):
    rclpy.init(args=args)

    gripper_node = GripperControlNode()
    trajectory_node = TrajectoryControlNode()

    try:

        run_demo(gripper_node, trajectory_node)
    finally:
        gripper_node.destroy_node()
        trajectory_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()