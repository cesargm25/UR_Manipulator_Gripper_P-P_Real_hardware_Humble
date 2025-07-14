import rclpy
import time
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from ros2_robotiqgripper.srv import RobotiqGripper

class MoveURWithIK(Node):
    def __init__(self):
        super().__init__('move_ur_with_ik')

        # UR3e pub to scaled_joint (the acutal robot)
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.ik_client = self.create_client(GetPositionIK, 'compute_ik') 
        self.ik_client.wait_for_service() 

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Esperando el servicio /compute_ik...')

        # gripper service
        self.gripper_client = self.create_client(RobotiqGripper, '/Robotiq_Gripper')
        while not self.gripper_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Esperando el servicio del gripper...')

        self.demo_sequence()

    def demo_sequence(self):
        # Pose 1: Pre-grasp
        pose1 = PoseStamped()
        pose1.header.frame_id = "base_link"
        pose1.pose.position.x = 0.15
        pose1.pose.position.y = 0.17
        pose1.pose.position.z = 0.3
        pose1.pose.orientation.x = 0.32
        pose1.pose.orientation.y = -0.9
        pose1.pose.orientation.z = 0.09
        pose1.pose.orientation.w = 0.13

        # Pose 2: Drop location
        pose2 = PoseStamped()
        pose2.header.frame_id = "base_link"
        pose2.pose.position.x = 0.14
        pose2.pose.position.y = 0.20
        pose2.pose.position.z = 0.3
        pose2.pose.orientation.x = 0.55
        pose2.pose.orientation.y = -0.83
        pose2.pose.orientation.z = 0.027
        pose2.pose.orientation.w = 0.043

        #x: 0.0781895622422665
        #y: -0.46382258453013725
        #z: 0.1737647679558
        #rotation:
        #x: 0.552080667940534
        #y: -0.832174259108141
        #z: 0.027679852727421836
        #w: 0.043894923591901426

        self.get_logger().info('Moviendo a pose 1...')
        self.move_to_pose(pose1)
        time.sleep(2.5)

        self.get_logger().info('Cerrando gripper...')
        self.control_gripper("CLOSE")
        time.sleep(2.5)

        self.get_logger().info('Moviendo a pose 2...')
        self.move_to_pose(pose2)
        time.sleep(2.5)

        self.get_logger().info('Abriendo gripper...')
        self.control_gripper("OPEN")
        time.sleep(1.5)

    def move_to_pose(self, target_pose: PoseStamped):
        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = "ur_manipulator"
        ik_req.ik_request.pose_stamped = target_pose
        ik_req.ik_request.ik_link_name = "tool0"  # Puedes cambiarlo si tu herramienta tiene otro nombre
        ik_req.ik_request.timeout.sec = 2

        future = self.ik_client.call_async(ik_req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.error_code.val == 1:
            joint_names = response.solution.joint_state.name
            joint_positions = response.solution.joint_state.position

            msg = JointTrajectory()
            msg.joint_names = joint_names
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start = Duration(sec=3)
            msg.points.append(point)

            self.trajectory_pub.publish(msg)
            self.get_logger().info('Trayectoria publicada desde IK.')
        else:
            self.get_logger().error(f"IK fallida. Código de error: {response.error_code.val}")

    def control_gripper(self, action: str):
        req = RobotiqGripper.Request()
        req.action = action
        future = self.gripper_client.call_async(req)

        def callback(fut):
            result = fut.result()
            if result:
                self.get_logger().info(f'Gripper {action}: {result.message}')
            else:
                self.get_logger().error(f'Error al enviar comando {action} al gripper.')

        future.add_done_callback(callback)

def main(args=None):
    rclpy.init(args=args)
    node = MoveURWithIK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
