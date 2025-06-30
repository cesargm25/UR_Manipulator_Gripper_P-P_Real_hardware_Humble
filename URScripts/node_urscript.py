#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.task import Future
from rclpy.executors import SingleThreadedExecutor

class URScriptTester(Node):
    def __init__(self):
        super().__init__('urscript_tester')
        self.cli = self.create_client(String, '/ur_hardware_interface/script_command')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /ur_hardware_interface/script_command service...')

    def send_script_command(self, script_code):
        req = String()
        req.data = script_code
        future = self.cli.call_async(req)
        return future


def main(args=None):
    rclpy.init(args=args)
    node = URScriptTester()

    scripts = {
        'popup': 'popup("Mensaje desde ROS2")',
        'open_gripper': 'rq_open()',
        'close_gripper': 'rq_close()'
    }

    for name, code in scripts.items():
        node.get_logger().info(f'Sending script: {name} → {code}')
        future = node.send_script_command(code)
        rclpy.spin_until_future_complete(node, future)

        if future.done():
            node.get_logger().info(f" Sent: {name}")
        else:
            node.get_logger().error(f" Failed: {name}")

        node.get_logger().info("Waiting 3 seconds before next command...\n")
        node.get_clock().sleep_for(rclpy.duration.Duration(seconds=3))

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
