import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import psutil


class MemoryPublisher(Node):
    def __init__(self):
        super().__init__("memory_monitor_publisher")

        # Declare the parameter `use_single_topic` with a default value of `True`
        self.declare_parameter("use_single_topic", True)

        # Initialize all publishers
        self.single_publisher = self.create_publisher(String, "memory_info", 10)
        self.total_memory_publisher = self.create_publisher(Float32, "total_memory", 10)
        self.used_memory_publisher = self.create_publisher(Float32, "used_memory", 10)
        self.memory_percent_publisher = self.create_publisher(
            Float32, "memory_percent", 10
        )

        # Timer to publish memory info every second
        self.timer = self.create_timer(1.0, self.publish_memory_info)
        
    # Timer callback function
    def publish_memory_info(self):
        # Retrieve the current value of `use_single_topic` parameter
        use_single_topic = (
            self.get_parameter("use_single_topic").get_parameter_value().bool_value
        )

        # Collect memory information
        mem = psutil.virtual_memory()
        total_memory = mem.total / (1024**3)  # Total in GB
        used_memory = mem.used / (1024**3)  # Used in GB
        memory_percent = mem.percent  # Usage percentage

        if use_single_topic:
            # Publish a single string message with all data
            message = String()
            message.data = (
                f"Total Memory: {total_memory:.2f} GB, "
                f"Used Memory: {used_memory:.2f} GB, "
                f"Usage Percentage: {memory_percent:.2f}%"
            )
            self.single_publisher.publish(message)
            self.get_logger().info(f'Published: "{message.data}"')
        else:
            # Publish each piece of data separately on different topics
            total_msg = Float32()
            total_msg.data = float(total_memory)
            self.total_memory_publisher.publish(total_msg)
            self.get_logger().info(f"Published Total Memory: {total_memory:.2f} GB")

            used_msg = Float32()
            used_msg.data = float(used_memory)
            self.used_memory_publisher.publish(used_msg)
            self.get_logger().info(f"Published Used Memory: {used_memory:.2f} GB")

            percent_msg = Float32()
            percent_msg.data = float(memory_percent)
            self.memory_percent_publisher.publish(percent_msg)
            self.get_logger().info(f"Published Memory Percent: {memory_percent:.2f}%")


def main(args=None):
    rclpy.init(args=args)
    memory_monitor_publisher = MemoryPublisher()
    rclpy.spin(memory_monitor_publisher)

    # Clean up when done
    memory_monitor_publisher.destroy_node()
    rclpy.shutdown()
