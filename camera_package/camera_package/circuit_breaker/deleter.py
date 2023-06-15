import random

from msg_srv_action_interface_example.msg import ArithmeticArgument, Buffer
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from collections import deque

class Deleter(Node):

    def __init__(self):
        super().__init__('deleter')
        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value
        self.declare_parameter('memory_size', 5)
        self.memory_size = self.get_parameter('memory_size').value
        self.buffer = deque(maxlen=self.memory_size)

        QOS_RKL10V = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = qos_depth,
            durability = QoSDurabilityPolicy.VOLATILE
        )

        self.deleter_subscriber = self.create_subscription(
            Buffer,
            'buffer_msg',
            self.subscribe_topic_message,
            QOS_RKL10V
        )

    def subscribe_topic_message(self, msg:Buffer):
        self.buffer.append(msg.id_list[-1])
        self.get_logger().info(f'Received message: {str(tuple(self.buffer))}')

    def update_parameter(self, params):
        for param in params:
            if param.name == 'memory_size' and param.type_ == Parameter.Type.INTEGER_ARRAY:
                self.memory_size = param.value


def main_00(args = None):
    rclpy.init(args=args)
    try:
        deleter = Deleter()
        try:
            rclpy.spin(deleter)
        except KeyboardInterrupt:
            deleter.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            deleter.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main_00()


