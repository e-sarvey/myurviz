from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Number of groups (N)
    N = 1
    nodes = []

    # Loop to create N nodes
    for i in range(1, N + 1):
        nodes.append(
            Node(
                package='my_ur_viz_pkg',
                executable='viz_node',
                name=f'arm_visualization_{i}',
                parameters=[{
                    'mqtt_client_name': f'Group{i}ActionServer',
                    'angles_topic': f'group{i}_joint_angles',
                    'status_topic': f'group{i}_status',
                    'gripper_topic': f'group{i}_gripper',
                    'action_name': f'simulated_arm_server_{i}/follow_joint_trajectory',
                    'gripper_topic_name': f'simulated_arm_server{i}/gripper/control',
                    'get_pose_topic': f'group{i}_getpose',
                    'pose_topic': f'group{i}_pose',
                    'service_name': f'simulated_arm_server{i}/get_current_pose'
                    
                }]
            )
        )

    return LaunchDescription(nodes)