#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time


class MoveURRobot(Node):
    def __init__(self):
        super().__init__('move_ur_robot_node')
        
        # IK y FK clients
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik') 
        self.ik_client.wait_for_service()
        
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk') 
        self.fk_client.wait_for_service()

        # Publisher de trayectorias
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscriber al joint_state
        self.joint_state = None
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info("Nodo MoveURRobot inicializado")

    def joint_state_callback(self, msg):
        self.joint_state = msg

    def wait_for_joint_state(self, timeout=10):
        start_time = time.time()
        while self.joint_state is None:
            rclpy.spin_once(self)
            if time.time() - start_time > timeout:
                raise TimeoutError("No se recibió /joint_states a tiempo")

    def compute_ik(self, pose_stamped):
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'ur_manipulator'  # <- mismo que en ur_moveit_config
        req.ik_request.robot_state.joint_state = self.joint_state
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout.sec = 2

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def move_to_pose(self, pose_stamped, duration=5):
        self.wait_for_joint_state()

        result = self.compute_ik(pose_stamped)
        if result.error_code.val != result.error_code.SUCCESS:
            self.get_logger().error("No se encontró solución de IK")
            return

        joint_names = result.solution.joint_state.name
        joint_positions = result.solution.joint_state.position

        traj = JointTrajectory()
        traj.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = duration
        traj.points.append(point)

        self.trajectory_pub.publish(traj)
        self.get_logger().info("Trayectoria publicada")



def main():
    rclpy.init()

    node = MoveURRobot()

    # Pose objetivo (frente al UR3e)
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'base_link'
    target_pose.pose.position.x = 0.3
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.15

    # Orientación apuntando hacia abajo
    target_pose.pose.orientation.w = 1.0
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 0.0

    node.move_to_pose(target_pose)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
