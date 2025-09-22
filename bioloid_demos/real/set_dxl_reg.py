#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dynamixel_interfaces.srv import SetRegister

def main():
    rclpy.init()
    node = Node("test_client")
    client = node.create_client(SetRegister, "set_register")

    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Service not available")
        return

    req = SetRegister.Request()
    req.id = 1
    req.register_name = "Goal_Position"  # example register
    req.value = 531

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        print("Success:", future.result().success, "Message:", future.result().message)
    else:
        print("Service call failed")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
