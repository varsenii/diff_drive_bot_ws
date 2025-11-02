#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, ListParameters


class UseSimTimeChecker(Node):
    def __init__(self):
        super().__init__('use_sim_time_checker')
        self.get_logger().info("UseSimTimeChecker node started.")

        # Get all nodes
        node_info = self.get_node_names_and_namespaces()
        self.node_names = [f"{ns}{name}" for name, ns in node_info]
        self.get_logger().info(f"Found {len(self.node_names)} nodes.")

        # Check each node
        for node_name in self.node_names:
            self.check_use_sim_time(node_name)

    def check_use_sim_time(self, node_name):
        try:
            # Create a temporary node to query parameters
            temp_node = Node(f'temp_client_for_{node_name.replace("/", "_")}')
            
            # List parameters
            list_client = temp_node.create_client(ListParameters, f'{node_name}/list_parameters')
            if not list_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f"Service {node_name}/list_parameters not available")
                temp_node.destroy_node()
                return

            list_req = ListParameters.Request()
            future = list_client.call_async(list_req)
            rclpy.spin_until_future_complete(temp_node, future, timeout_sec=1.0)
            result = future.result()

            param_names = result.result.names if result and hasattr(result, 'result') else []
            if 'use_sim_time' in param_names:
                # Get parameter value
                get_client = temp_node.create_client(GetParameters, f'{node_name}/get_parameters')
                if not get_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().warn(f"Service {node_name}/get_parameters not available")
                    temp_node.destroy_node()
                    return

                get_req = GetParameters.Request()
                get_req.names = ['use_sim_time']
                future_val = get_client.call_async(get_req)
                rclpy.spin_until_future_complete(temp_node, future_val, timeout_sec=1.0)
                val_result = future_val.result()
                use_sim_time = val_result.values[0].bool_value if val_result.values else None
                self.get_logger().info(f"Node '{node_name}': use_sim_time={use_sim_time}")
            else:
                self.get_logger().info(f"Node '{node_name}' has no use_sim_time parameter.")

            temp_node.destroy_node()

        except Exception as e:
            self.get_logger().error(f"Failed to check node '{node_name}': {e}")


def main(args=None):
    rclpy.init(args=args)
    checker = UseSimTimeChecker()
    rclpy.spin_once(checker)
    checker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
