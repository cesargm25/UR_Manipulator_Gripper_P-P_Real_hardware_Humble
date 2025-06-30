import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionFK
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration

class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(5.0, self.publish_trajectory)

        # Creating client for IK
        #self.ik_client = self.create_client(GetPositionIK, 'compute_ik') 
        #self.ik_client.wait_for_service()
        #self.get_logger().info('Creating service for compute_IK...')
        
        # Creating client for fk
        self.fk_client = self.create_client(GetPositionFK, 'compute_fk')
        self.fk_client.wait_for_service()
        self.get_logger().info('Creating service for compute_FK...')

        self.ee_pose(None, None)
        # while not self.fk_client.wait_for_service(timeout_sec=2.0):
        #     self.get_logger().info('Esperando al servicio /compute_fk...')

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        joint_positions = [1.57, -1.57, 1.57, 0.0, 1.0, 2.0]

        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(sec=3, nanosec=0)

        msg.points.append(point)

        self.publisher_.publish(msg)
        self.get_logger().info('Trajectory published!')

        while not self.fk_client.wait_for_service(timeout_sec=2.0):
           self.get_logger().info('Esperando al servicio /compute_fk...')

        # F
        self.ee_pose(msg.joint_names, joint_positions)

    def ee_pose(self, joint_names, joint_positions):
        request = GetPositionFK.Request()
        request.header.stamp = self.get_clock().now().to_msg()

        request.header.frame_id = 'base_link'  #base frame
        request.fk_link_names = ['tool0']      

        # self.get_logger().info(str(request.robot_state.joint_state))
        # joint_state = JointState()
        # joint_state.name = joint_names
        # joint_state.position = joint_positions
        # request.robot_state.joint_state = joint_state
        # request.robot_state.joint_state.position = [0.1]
        # self.get_logger().info(str(request.robot_state.joint_state))

        future = self.fk_client.call_async(request)
        future.add_done_callback(self.fk_service_callback)

        
        self.get_logger().info('Called function!')

        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        self.get_logger().info('Executor nodes after spin_until_future_complete: {}'.format(self.executor.get_nodes()))
        
        self.get_logger().info('Got results!')

        if future.result() is not None:
            self.get_logger().info(str(future.result()))
            
            if future.result().pose_stamped:
                pose = future.result().pose_stamped[0].pose
                self.get_logger().info(f"End-effector pose:\n"
                                       f"Position: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}\n"
                                       f"Orientation: x={pose.orientation.x:.3f}, y={pose.orientation.y:.3f}, z={pose.orientation.z:.3f}, w={pose.orientation.w:.3f}")
            else:
                self.get_logger().warn("No se recibió pose del end-effector.")
        else:
            self.get_logger().error("Falló la llamada al servicio de FK.")
            
        self.get_logger().info('Ended!')
    
    def fk_service_callback(self, future):
        try:
            response = future.result()
            if response.error_code.val == 1:  # SUCCESS
                # Extract the end effector pose from the response
                end_effector_pose = response.pose_stamped[0]  # Pose of the first link (typically end effector)
                self.get_logger().info(f"End Effector Pose: {end_effector_pose}")
            else:
                self.get_logger().error(f"Failed to compute FK. Error code: {response.error_code.val}")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryPublisher()

    for _ in range(10):
        rclpy.spin_once(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
