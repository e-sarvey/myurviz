import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from std_msgs.msg import Int32MultiArray
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import paho.mqtt.client as mqtt
import json
import time

class SimulatedArmActionServer(Node):
    """
    A ROS2 action server that interacts with a simulated robot arm through MQTT.
    MQTT client name, angles topic, status topic, and action name can be set as ROS parameters.
    """

    def __init__(self):
        super().__init__('simulated_arm_server')

        # Declare parameters for MQTT client name, topics, and action name
        self.declare_parameter('mqtt_client_name', 'SimulatedArmClient')
        self.declare_parameter('angles_topic', 'robot_joint_angles')
        self.declare_parameter('status_topic', 'robot_status')
        self.declare_parameter('gripper_topic', 'robot_gripper')
        self.declare_parameter('action_name', 'follow_joint_trajectory') 
        self.declare_parameter('gripper_topic_name', '/gripper/control')
        self.declare_parameter('get_pose_topic', 'robot_get_pose')
        self.declare_parameter('pose_topic', 'robot_pose')
        self.declare_parameter('service_name', 'get_current_pose')
        
        # Fetch parameters for MQTT client and topics
        mqtt_client_name = self.get_parameter('mqtt_client_name').get_parameter_value().string_value
        self.angles_topic = self.get_parameter('angles_topic').get_parameter_value().string_value
        self.status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        self.gripper_topic = self.get_parameter('gripper_topic').get_parameter_value().string_value
        self.gripper_topic_name = self.get_parameter('gripper_topic_name').get_parameter_value().string_value
        action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self.get_pose_topic = self.get_parameter('get_pose_topic').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.service_name = self.get_parameter('service_name').get_parameter_value().string_value
        
        self.get_logger().info(f'MQTT Client: {mqtt_client_name}')
        self.get_logger().info(f'Angles Topic: {self.angles_topic}')
        self.get_logger().info(f'Status Topic: {self.status_topic}')
        self.get_logger().info(f'Gripper MQTT Topic: {self.gripper_topic}')
        self.get_logger().info(f'Gripper Subscriber Topic: {self.gripper_topic_name}')
        self.get_logger().info(f'Action Name: {action_name}')
        self.get_logger().info(f'Get Pose Topic: {self.get_pose_topic}')
        self.get_logger().info(f'Pose Topic: {self.pose_topic}')
        self.get_logger().info(f'Service Name: {self.service_name}')
        
        # Initialize MQTT client and set up callbacks
        self.mqtt_client = mqtt.Client(mqtt_client_name)
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.mqtt_client.on_connect = self.on_mqtt_connect

        # Variables to store action state
        self.current_goal_handle = None  # Keep track of the current goal handle
        self.action_done = False         # Flag to indicate action completion
        self.reconnecting = False        # Reconnection flag

        # Create an action server with the specified action name
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,  # Use the dynamic action name
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback
        )


        self.create_subscription(
            Int32MultiArray,
            self.gripper_topic_name,
            self.gripper_control_callback,
            10
        )

        self.pose_service = self.create_service(Trigger, self.service_name, self.get_pose_callback)
        self.current_pose = None  # Store the latest received pose
        self.mqtt_client.subscribe(self.pose_topic)  # Subscribe to pose MQTT topic
        self.get_logger().info(f'Subscribed to topic: {self.gripper_topic_name}')
        # Initial connection attempt
        self.connect_to_broker()
        self.get_logger().info("Simulated Arm Action Server initialized.")
        
        
    def connect_to_broker(self):
        """Attempt to connect to the MQTT broker and subscribe to topics."""
        try:
            self.mqtt_client.connect("10.243.82.33", 1883, 60)
            self.mqtt_client.loop_start()  # Start MQTT loop in a background thread
            self.get_logger().info("Connected to MQTT broker.")
            self.mqtt_client.subscribe(self.pose_topic)  # Subscribe after connection
            self.mqtt_client.subscribe(self.status_topic)
            self.get_logger().info(f"Subscribed to MQTT topics: {self.pose_topic}, {self.status_topic}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")
            self.reconnecting = True

    def gripper_control_callback(self, msg):
        """
        Callback for /gripper/control topic.
        Processes position and speed values and forwards a message to MQTT.
        """
        try:
            position = msg.data[0]
            speed = msg.data[1]
    
            # Reverse the position (0-255) to a percentage (0-100)
            disp = int((255 - position) * 100 / 255)
    
            # Convert speed (0-255) to time (0.5 to 10 seconds, inversely proportional)
            time_val = 10 - ((speed * 9.5 / 255) + 0.5)
    
            # Construct MQTT message
            mqtt_message = {"disp": disp, "time": round(time_val, 2)}
            json_message = json.dumps(mqtt_message)
    
            # Publish MQTT message
            self.mqtt_client.publish(self.gripper_topic, json_message)
            self.get_logger().info(f"Gripper control message [MQTT]: {json_message}")
    
        except Exception as e:
            self.get_logger().error(f"Failed to process gripper control message: {e}")

    
    def goal_callback(self, goal_request):
        """Callback to decide whether to accept or reject a goal."""
        self.get_logger().info('Received a goal request.')
        
        if self.reconnecting:
            self.get_logger().warn('Currently reconnecting to MQTT broker, rejecting the goal.')
            return GoalResponse.REJECT
        else:
            self.get_logger().info('Accepting the goal request.')
            return GoalResponse.ACCEPT
            
    def execute_callback(self, goal_handle):
        """
        Handles the execution of an accepted goal. Sends the entire trajectory via MQTT and waits for "done" status.
        """
        self.get_logger().info('Executing goal...')
    
        self.current_goal_handle = goal_handle
        self.action_done = False  # Reset action done flag
    
        # Extract trajectory points
        trajectory_points = goal_handle.request.trajectory.points
        if not trajectory_points:
            self.get_logger().error("Received an empty trajectory.")
            result = FollowJointTrajectory.Result()
            goal_handle.abort()
            self.current_goal_handle = None
            return result
    
        # Build trajectory JSON for MQTT
        trajectory_list = []
        total_time = 0
    
        for point in trajectory_points:
            joint_angles = point.positions
            time_from_start_sec = point.time_from_start.sec
            self.get_logger().info(f"Parsed joint angles from action request: {joint_angles}")
            self.get_logger().info(f"Parsed goal time from action request: {time_from_start_sec}")
    
            trajectory_list.append({
                "angles": list(joint_angles),  # Ensure it is a list
                "time": time_from_start_sec
            })
            total_time += time_from_start_sec
    
        trajectory_json = json.dumps({"trajectory": trajectory_list})
        self.get_logger().info(f"Built trajectory JSON: {trajectory_json}")
    
        # Send trajectory JSON to MQTT
        sent = self.send_trajectory_to_arm(trajectory_json)
        if not sent:
            self.get_logger().error("Failed to send trajectory to MQTT")
            result = FollowJointTrajectory.Result()
            goal_handle.abort()
            self.current_goal_handle = None
            return result
    
        # Subscribe to the status topic
        self.mqtt_client.subscribe(self.status_topic)
    
        # Log message and wait for "done" status
        self.get_logger().info('Waiting for "done" message from the arm...')
    
        timeout_duration = 5 * total_time
        self.get_logger().info(f"Action timeout set to {timeout_duration} seconds.")
        
        # Simple loop to wait for the action to be completed
        start_time = time.time()
        
        while not self.action_done:
            elapsed_time = time.time() - start_time
    
            if elapsed_time > timeout_duration:
                self.get_logger().error(
                    "Timeout exceeded while waiting for 'done' status. Failing the goal."
                )
                result = FollowJointTrajectory.Result()
                result.error_code = result.PATH_TOLERANCE_VIOLATED  # Indicate failure
                goal_handle.abort()
                self.current_goal_handle = None
                return result
                
            time.sleep(0.1)  # Sleep briefly to avoid busy waiting
    
        # When action is done
        self.get_logger().info('Action completed successfully.')
        result = FollowJointTrajectory.Result()
        result.error_code = 0  # Indicate success
        goal_handle.succeed()
    
        self.current_goal_handle = None
        return result

    def get_pose_callback(self, request, response):
        """Handle requests for the current pose."""
        self.get_logger().info('Received pose request. Sending "pose_requested"...')
    
        # Send pose request message
        self.mqtt_client.publish(self.get_pose_topic, "pose_requested")
        self.get_logger().info(f'Sent "pose_requested" to topic: {self.get_pose_topic}')
    
        # Wait for the response (simple wait with timeout)
        start_time = time.time()
        timeout = 10.0  # seconds
        while self.current_pose is None:
            if time.time() - start_time > timeout:
                self.get_logger().error("Pose request timed out.")
                response.success = False
                response.message = "Pose request timed out."
                return response
    
            time.sleep(0.1)
    
        # Return the response with the received pose, treating the gripper as another joint
        self.get_logger().info(f"Pose received: {self.current_pose}")
        response.success = True
        response.message = "Pose retrieved successfully."
        return response
        
    def send_trajectory_to_arm(self, trajectory_json):
        """Sends the trajectory JSON to the configured MQTT topic."""
        if self.reconnecting:
            self.get_logger().warn('Unable to send trajectory while reconnecting to MQTT broker.')
            return False
    
        try:
            self.mqtt_client.publish(self.angles_topic, trajectory_json)
            self.get_logger().info(f"Sent trajectory to {self.angles_topic}: {trajectory_json}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish trajectory: {e}")
            self.reconnecting = True
            return False

    
    def on_mqtt_message(self, client, userdata, msg):
        """Callback for MQTT messages."""
        self.get_logger().info(f"MQTT message received on topic: {msg.topic}")
    
        if msg.topic == self.pose_topic:
            self.get_logger().info("Pose message received. Processing...")
            try:
                pose_data = json.loads(msg.payload.decode())
                if "joint_angles" in pose_data and "gripper" in pose_data:
                    # Treat the gripper as another joint
                    joint_names = [f"joint_{i+1}" for i in range(len(pose_data["joint_angles"]))]
                    joint_positions = pose_data["joint_angles"]
    
                    # Add the gripper
                    joint_names.append("gripper_percent")
                    joint_positions.append(pose_data["gripper"])
    
                    # Create JointState message
                    self.current_pose = JointState(
                        name=joint_names,
                        position=joint_positions
                    )
                    self.get_logger().info(f"Parsed pose: {self.current_pose}")
                else:
                    self.get_logger().warn("Pose message missing required fields. Ignoring.")
            except Exception as e:
                self.get_logger().error(f"Error parsing pose message: {e}")
    
        elif msg.topic == self.status_topic:
            status = msg.payload.decode()
            self.get_logger().info(f"Status message received: {status}")
    
            if status == "done":
                self.get_logger().info("Received 'done' status from the arm.")
                self.action_done = True
            
    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback for handling MQTT disconnections."""
        self.get_logger().warn(f"Disconnected from MQTT broker (code {rc}). Reconnecting...")
        self.reconnecting = True
        self.reconnect_to_broker()

    def reconnect_to_broker(self):
        """Attempts to reconnect to the MQTT broker."""
        while self.reconnecting:
            try:
                self.get_logger().info("Attempting to reconnect to MQTT broker...")
                self.mqtt_client.reconnect()
                self.get_logger().info("Reconnected to MQTT broker.")
                self.reconnecting = False
            except Exception as e:
                self.get_logger().error(f"Reconnection failed: {e}")
                time.sleep(5)  # Wait before retrying

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback for handling successful MQTT connections."""
        self.get_logger().info(f"Successfully connected to MQTT broker (code {rc}).")
        if rc == 0:  # Connection successful
            if self.reconnecting:
                self.get_logger().info("Re-subscribing to topics after reconnecting...")
                self.mqtt_client.subscribe(self.pose_topic)
                self.mqtt_client.subscribe(self.status_topic)
                self.reconnecting = False
        else:
            self.get_logger().error(f"Unexpected connection result: {rc}")
        
    def shutdown(self):
        """Shutdown the MQTT client when the node is terminated."""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()


def main(args=None):
    rclpy.init(args=args)
    simulated_arm_action_server = SimulatedArmActionServer()
    try:
        simulated_arm_action_server.get_logger().info("Preparing to spin...")
        rclpy.spin(simulated_arm_action_server)
    except KeyboardInterrupt:
        simulated_arm_action_server.get_logger().info("Shutting down Simulated Arm Action Server.")
    finally:
        simulated_arm_action_server.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()