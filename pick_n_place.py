#import rclpy
#import time
#from rclpy.node import Node
#from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from builtin_interfaces.msg import Duration
#from ros2_robotiqgripper.srv import RobotiqGripper
#from sensor_msgs.msg import JointState
#from moveit_msgs.srv import GetPositionFK, GetPositionIK
#from geometry_msgs.msg import PoseStamped
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
#            self.get_logger().info('Waiting for gripper service...')
#
#        #self.get_logger().info('Starting Moving...?')
#        self.publish_trajectory()
#        self.publish_trajectory_2()
#        
#        #gripper closes and opens (Test by time, No obeject) detected yet
#        self.timer_close = self.create_timer(4.0, self.close_gripper)
#        self.timer_open = self.create_timer(5.0, self.open_gripper)
#        self.already_closed = False
#        self.already_opened = False
#
#    def publish_trajectory(self):
#
#
#        #Added to make sure the controller is available
#        while self.publisher_.get_subscription_count() == 0: 
#            self.get_logger().info('Waiting fpr the controller to be ready...')
#            time.sleep(0.5)
#
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
#        point.positions = [0.77, -0.93, 0.79, -1.76, -1.14, 0.80]
#        point.time_from_start = Duration(sec=3, nanosec=0)
#
#        #positionof the object in joint angles 
#        #- -0.9353276652148743
#        #- 0.7918184439288538
#        #- -1.7651006184019984
#        #- -1.1390169302569788
#        #- 0.8020051121711731
#        #- 0.7697243094444275
#
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
#
#  def publish_trajectory_2(self):
#
#        #Added to make sure the controller is available
#        while self.publisher_.get_subscription_count() == 0: 
#            self.get_logger().info('Waiting fpr the controller to be ready...')
#            time.sleep(0.5)
#
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
#        point.positions = [1.62, -0.87, 0.49, -1.71, -1.52, 0.08]
#        point.time_from_start = Duration(sec=3, nanosec=0)
#
#        #positionof the object in joint angles 
#        #- -0.8759711545756836
#        #- 0.4975793997394007
#        #- -1.716412206689352
#        #- -1.5208357016192835
#        #- 0.08253264427185059
#        #- 1.6182715892791748
#
#
#        msg.points.append(point)
#        self.publisher_.publish(msg)
#        self.get_logger().info('it seems like it sent the trajectory')
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


import rclpy
import time
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from ros2_robotiqgripper.srv import RobotiqGripper

class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')

        # Publisher for Arm movement
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Gripper Client
        self.cli = self.create_client(RobotiqGripper, '/Robotiq_Gripper')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for gripper service...')

        # Start the sequence
        self.execute_sequence()

    def execute_sequence(self):
        # Step 1: Move to first position
        self.publish_trajectory()
        self.get_logger().info('Waiting for motion to finish...')
        time.sleep(4.0)

        # Step 2: Close gripper
        self.send_gripper_command('CLOSE')
        time.sleep(2.0)

        # Step 3: Move to second position
        self.publish_trajectory_2()
        self.get_logger().info('Waiting for motion to finish...')
        time.sleep(4.0)

        # Step 4: Open gripper
        self.send_gripper_command('OPEN')
        self.get_logger().info('Sequence completed.')

    def publish_trajectory(self):

        #Added to make sure the controller is available
        while self.publisher_.get_subscription_count() == 0: 
           self.get_logger().info('Waiting fpr the controller to be ready...')
           time.sleep(0.5)

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
        point.positions = [0.77, -0.93, 0.79, -1.76, -1.14, 0.80]
        point.time_from_start = Duration(sec=3)

        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info('Trajectory 1 sent.')

    def publish_trajectory_2(self):
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
        point.positions = [1.62, -0.87, 0.49, -1.71, -1.52, 0.08]
        point.time_from_start = Duration(sec=3)

        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info('Trajectory 2 sent.')

    def send_gripper_command(self, action: str):
        request = RobotiqGripper.Request()
        request.action = action
        future = self.cli.call_async(request)

        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result:
            self.get_logger().info(f'Gripper {action} → {result.message}')
        else:
            self.get_logger().error(f'Failed to send {action} to gripper.')

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryPublisher()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
