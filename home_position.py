import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from ros2_robotiqgripper.srv import RobotiqGripper

class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        timer_period = 1.0  # segundos
        self.timer = self.create_timer(timer_period, self.publish_trajectory)

    
        self.cli = self.create_client(RobotiqGripper, '/Robotiq_Gripper')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Ha llegado hasta aqui...')


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

        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        point.time_from_start = Duration(sec=3, nanosec=0)

        msg.points.append(point)

        self.publisher_.publish(msg)
        self.get_logger().info('Trajectory published!')

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryPublisher()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
###
#import rclpy
#from rclpy.node import Node
#from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from builtin_interfaces.msg import Duration
#from ros2_robotiqgripper.srv import RobotiqGripper
#
#class JointTrajectoryPublisher(Node):
#    def __init__(self):
#        super().__init__('joint_trajectory_publisher')
#
#        # Publisher for Arm movement
#        self.publisher_ = self.create_publisher(
#            JointTrajectory,
#            '/scaled_joint_trajectory_controller/joint_trajectory',
#            10
#        )
#        
#        #Gripper Client
#        self.cli = self.create_client(RobotiqGripper, '/Robotiq_Gripper')
#        while not self.cli.wait_for_service(timeout_sec=1.0):
#            self.get_logger().info('Esperando al servicio del gripper...')
#
#        #self.get_logger().info('Starting Moving...?')
#        self.publish_trajectory()
#
#        #gripper closes and opens
#        self.timer_close = self.create_timer(4.0, self.close_gripper)
#        self.timer_open = self.create_timer(10.0, self.open_gripper)
#        self.already_closed = False
#        self.already_opened = False
#
#    def publish_trajectory(self):
#        msg = JointTrajectory()
#        msg.joint_names = [
#            'shoulder_pan_joint',
#            'shoulder_lift_joint',
#            'elbow_joint',
#            'wrist_1_joint',
#            'wrist_2_joint',
#            'wrist_3_joint'
#        ]
#
#        point = JointTrajectoryPoint()
#        point.positions = [0.57, -0.57, 0.0, -1.57, -1.0, 0.57]
#
#    
#        point.time_from_start = Duration(sec=3, nanosec=0)
#
#        msg.points.append(point)
#        self.publisher_.publish(msg)
#        self.get_logger().info('it seems like it sent the trajectory')
#
#    def send_gripper_command(self, action: str):
#        request = RobotiqGripper.Request()
#        request.action = action
#        future = self.cli.call_async(request)
#
#        def callback(future):
#            result = future.result()
#            if result:
#                self.get_logger().info(f'Gripper {action} → {result.message}')
#            else:
#                self.get_logger().error(f'Fallo al enviar comando {action} al gripper.')
#
#        future.add_done_callback(callback)
#
#    def close_gripper(self):
#        if not self.already_closed:
#            self.send_gripper_command('CLOSE')
#            self.already_closed = True
#
#    def open_gripper(self):
#        if not self.already_opened:
#            self.send_gripper_command('OPEN')
#            self.already_opened = True
#
#def main(args=None):
#    rclpy.init(args=args)
#    node = JointTrajectoryPublisher()
#    rclpy.spin(node)
#    node.destroy_node()
#    rclpy.shutdown()
#
#if __name__ == '__main__':
#   main()
#