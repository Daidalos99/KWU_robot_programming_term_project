import random

from msg_srv_action_interface_example.msg import ArithmeticArgument, Buffer
from msg_srv_action_interface_example.srv import ArithmeticOperator
from msg_srv_action_interface_example.action import ArithmeticChecker

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from collections import deque



class Argument(Node):

    def __init__(self):
        #! Node 이름을 Blocker라고 짓자
        super().__init__('argument') # Node name
        self.declare_parameter('qos_depth', 10) #! parameter를 변경하면서 크기 조절 (buffer size를 변경할 수 있도록 조정)
        qos_depth = self.get_parameter('qos_depth').value
        self.declare_parameter('min_random_num', 0)
        self.min_random_num = self.get_parameter('min_random_num').value
        self.declare_parameter('max_random_num', 9)
        self.max_random_num = self.get_parameter('max_random_num').value
        self.add_on_set_parameters_callback(self.update_parameter)

        QOS_RKL10V = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = qos_depth,
            durability = QoSDurabilityPolicy.VOLATILE
        )

        self.arithmetic_argument_publisher = self.create_publisher(
            ArithmeticArgument,
            'arithmetic_argument',
            QOS_RKL10V
        )

        self.timer = self.create_timer(1.0, self.publish_random_arithmetic_arguments)

    def publish_random_arithmetic_arguments(self):
        msg = ArithmeticArgument()
        msg.stamp = self.get_clock().now().to_msg()
        msg.argument_a = float(random.randint(self.min_random_num, self.max_random_num))
        msg.argument_b = float(random.randint(self.min_random_num, self.max_random_num))
        self.arithmetic_argument_publisher.publish(msg)
        self.get_logger().info('Published argument a : {0}'.format(msg.argument_a))
        self.get_logger().info('Published argument b : {0}\n'.format(msg.argument_b))

    def update_parameter(self, params):
        for param in params:
            if param.name == 'min_random_num' and param.type_ == Parameter.Type.INTEGER:
                self.min_random_num = param.value
            elif param.name == 'max_random_num' and param.type_ == Parameter.Type.INTEGER:
                self.max_random_num = param.value
        return SetParametersResult(successful=True)


class Register(Node):

    def __init__(self):
        super().__init__('register')
        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value
        self.declare_parameter('memory_size', 10)
        self.memory_size = self.get_parameter('memory_size').value
        self.buffer = deque(maxlen = self.memory_size)

        QOS_RKL10V = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = qos_depth,
            durability = QoSDurabilityPolicy.VOLATILE
        )

        self.register_publisher = self.create_publisher(
            Buffer,
            'buffer_msg',
            QOS_RKL10V
        )

        self.timer = self.create_timer(1.0, self.publish_buffer_information)

    def publish_buffer_information(self):
        msg = Buffer()
        msg.stamp = self.get_clock().now().to_msg()
        msg.id_list = tuple(self.buffer)
        self.register_publisher.publish(msg)
        self.get_logger().info(f'Pulished id_list {str(tuple(self.buffer))}\n')

        # 하나씩  random하게 채워보기
        self.buffer.append(int(random.randint(0,9)))

    def update_parameter(self, params):
        for param in params:
            if param.name == 'memory_size' and param.type_ == Parameter.Type.INTEGER_ARRAY:
                self.memory_size = param.value


class Operator(Node):

    def __init__(self):
        super().__init__('operator')

        self.arithmetic_service_client = self.create_client(
            ArithmeticOperator,
            'arithmetic_operator'
        )

        while not self.arithmetic_service_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('The arithmetic_operator service not available.')

    def send_request(self):
        service_request = ArithmeticOperator.Request()
        service_request.arithmetic_operator = random.randint(1,4)
        futures = self.arithmetic_service_client.call_async(service_request)
        return futures

def main_operator(args=None):
    rclpy.init(args=args)
    operator = Operator()
    future = operator.send_request()
    user_trigger = True
    try:
        while rclpy.ok():
            if user_trigger is True:
                #! 서비스 응답값 확인 코드
                if future.done():
                    try:
                        service_response = future.result()
                    except Exception as e:
                        operator.get_logger().warn('Service call failed: {}'.format(str(e)))
                    else:
                      operator.get_logger().info(
                          'Result: {}'.format(service_response.arithmetic_result)
                      )
                      user_trigger = False
            else:
                input('Press Enter for next service call.')
                future = operator.send_request()
                user_trigger = True
    except KeyboardInterrupt:
        operator.get_logger().info('Keyboard Interrupt (SIGINT)')

    operator.destroy_node()
    rclpy.shutdown()

def main_00(args = None):
    rclpy.init(args=args)
    try:
        argument = Argument()
        try:
            rclpy.spin(argument)
        except KeyboardInterrupt:
            argument.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            argument.destroy_node()
    finally:
        rclpy.shutdown()

def main_01(args = None):
    rclpy.init(args = args)
    try:
        register = Register()
        try:
            rclpy.spin(register)
        except KeyboardInterrupt:
            register.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            register.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main_00()


