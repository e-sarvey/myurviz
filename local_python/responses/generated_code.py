import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from math import pi

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('draw_circle_node')

    client = ActionClient(node, FollowJointTrajectory, '/simulated_arm_server_1/follow_joint_trajectory')

    def send_goal():
        joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]

        radius = 0.1  # 10 cm
        num_points = 50
        time_from_start = 1.0  # 1 second for each point

        points = []
        for i in range(num_points + 1):
            theta = 2 * pi * i / num_points
            x = radius * np.cos(theta)
            y = radius * np.sin(theta)

            # Assuming handling for calculating actual joint states is defined in primitives
            joint_positions = calculate_inverse_kinematics(x, y)

            point = JointTrajectoryPoint(
                positions=joint_positions,
                time_from_start=rclpy.duration.Duration(seconds=i * time_from_start).to_msg()
            )
            points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory(
            joint_names=joint_names,
            points=points
        )

        return client.send_goal_async(goal_msg)

    send_goal_future = send_goal()

    def goal_response_callback(future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            node.get_logger().info('Goal rejected')
            return

        node.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(get_result_callback)

    def get_result_callback(future):
        result = future.result().result
        node.get_logger().info(f'Result: {result.error_code}')
        rclpy.shutdown()

    send_goal_future.add_done_callback(goal_response_callback)
    
    rclpy.spin(node)

def calculate_inverse_kinematics(x, y):
    # Placeholder for real inverse kinematics calculation
    # Return some dummy joint values
    return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

if __name__ == '__main__':
    main()