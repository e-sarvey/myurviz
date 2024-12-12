from myur.ik_solver.ur_kinematics import URKinematics
from myur.trajectory_planner import TrajectoryPlanner
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time

# Required dependencies for kinematics
kinematics = URKinematics()
trajectory_planner = TrajectoryPlanner()

# ---- PRIMITIVE FUNCTIONS ----

def set_joint_angles(joint_names: list[str], joint_angles: list[float], time: float, trajectory_client: ActionClient) -> str:
    """
    Purpose:
        Set the joint angles of the robot arm.
    Dependencies:
        - ROS2 Action Client for FollowJointTrajectory.
    Args:
        joint_names (list[str]): Names of the joints.
        joint_angles (list[float]): A list of joint angles in radians.
        time (float): Time in seconds to complete the trajectory.
        trajectory_client (ActionClient): ROS2 Action Client instance.
    Returns:
        str: Status message indicating the action performed.
    """
    if not trajectory_client.wait_for_server(timeout_sec=5.0):
        return "Trajectory action server not available."

    goal_msg = FollowJointTrajectory.Goal()
    goal_msg.trajectory.joint_names = joint_names
    goal_msg.trajectory.points = [
        JointTrajectoryPoint(
            positions=joint_angles,
            time_from_start=rclpy.time.Duration(seconds=time).to_msg(),
        )
    ]

    future = trajectory_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(Node("joint_angle_setter"), future)

    goal_handle = future.result()
    if not goal_handle.accepted:
        return "Joint angle set request rejected."

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(Node("joint_angle_setter"), result_future)

    result = result_future.result()
    return "Joint angles set successfully." if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL else "Failed to set joint angles."

def set_gripper_state(position: float, speed: float, gripper_publisher: Node) -> str:
    """
    Purpose:
        Control the robot gripper.
    Dependencies:
        - ROS2 Publisher for the gripper control topic.
    Args:
        position (float): Gripper state as a percentage open (0 for fully closed, 100 for fully open).
        speed (float): Speed of the gripper movement (0-255).
        gripper_publisher (Publisher): ROS2 Publisher instance for the gripper control topic.
    Returns:
        str: Status message indicating the gripper state.
    """
    msg = Int32MultiArray(data=[int(position * 2.55), int(speed), int(speed)])  # Convert percentage to 0-255 range
    gripper_publisher.publish(msg)
    return f"Gripper state set to: {position}% open."

def get_pose(pose_client: Node) -> dict:
    """
    Purpose:
        Retrieve the current joint angles and gripper state of the robot arm.
    Dependencies:
        - ROS2 Service Client for querying the pose.
    Args:
        pose_client (Node): ROS2 Service Client for the pose request service.
    Returns:
        dict: A dictionary containing:
            - 'success' (bool): Whether the query was successful.
            - 'joint_angles' (list[float]): Current joint angles in radians.
            - 'gripper_state' (float): Current gripper state as a percentage open (0 for fully closed, 100 for fully open).
    """
    if not pose_client.wait_for_service(timeout_sec=5.0):
        return {"success": False, "error": "Pose service unavailable."}

    request = Trigger.Request()
    future = pose_client.call_async(request)
    rclpy.spin_until_future_complete(Node("pose_query_node"), future)

    response = future.result()
    if response.success:
        pose_data = response.message.split(",")  # Assuming pose data is a comma-separated string
        joint_angles = [float(angle) for angle in pose_data[:-1]]  # All but last entry are joint angles
        gripper_state = float(pose_data[-1])  # Last entry is the gripper state
        return {"success": True, "joint_angles": joint_angles, "gripper_state": gripper_state}
    return {"success": False, "error": "Failed to retrieve pose."}

from myur.ik_solver.ur_kinematics import URKinematics

def inverse_kinematics(target_position: list[float], ik_solver: URKinematics) -> dict:
    """
    Purpose:
        Perform inverse kinematics to compute joint angles for a given target position.
    Dependencies:
        - `URKinematics` from `myur.ik_solver` for IK computations.
    Args:
        target_position (list[float]): Target position as [x, y, z, rx, ry, rz] or [x, y, z, qw, qx, qy, qz].
        ik_solver (URKinematics): Instance of the URKinematics solver.
    Returns:
        dict: Result of the inverse kinematics computation containing:
            - 'success' (bool): Whether the computation was successful.
            - 'joint_angles' (list[float]): A list of computed joint angles in radians (if successful).
    """
    try:
        joint_angles = ik_solver.solve_ik(target_position)
        return {'success': True, 'joint_angles': joint_angles}
    except Exception as e:
        return {'success': False, 'error': str(e)}
    
from myur.ik_solver.ur_kinematics import URKinematics

def forward_kinematics(joint_angles: list[float], fk_solver: URKinematics) -> dict:
    """
    Purpose:
        Perform forward kinematics to compute the end-effector position from joint angles.
    Dependencies:
        - `URKinematics` from `myur.ik_solver` for FK computations.
    Args:
        joint_angles (list[float]): A list of joint angles in radians.
        fk_solver (URKinematics): Instance of the URKinematics solver.
    Returns:
        dict: Result of the forward kinematics computation containing:
            - 'success' (bool): Whether the computation was successful.
            - 'position' (list[float]): The computed position as [x, y, z, rx, ry, rz].
    """
    try:
        position = fk_solver.solve_fk(joint_angles)
        return {'success': True, 'position': position}
    except Exception as e:
        return {'success': False, 'error': str(e)}