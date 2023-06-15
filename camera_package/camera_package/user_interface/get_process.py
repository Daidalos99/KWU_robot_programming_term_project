from msg_srv_action_interface_example.srv import Process
import rclpy
from rclpy.node import Node

class UserProcess(Node):

    def __init__(self):
        super().__init__('user_process')

        self.user_service_client = self.create_client(
            Process,
            'process_information'
        )

        while not self.user_service_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('The user process interface service not available.')

    def send_request(self):
        service_request = Process.Request()
        futures = self.user_service_client.call_async(service_request)
        return futures


def main(args=None):
    rclpy.init(args=args)
    user_process = UserProcess()
    future = user_process.send_request()
    user_trigger = True
    try:
        while rclpy.ok():
            if user_trigger is True:
                rclpy.spin_once(user_process)
                if future.done():
                    try:
                        service_response = future.result()
                    except Exception as e:
                        user_process.get_logger().warn(f'Service call failed: {str(e)}')
                    else:
                        if service_response.out_flag:

                            strings = [f'{car} : {time:1f}s\n' for car, time in zip(service_response.exited_cars, service_response.car_times)]
                            words = ""
                            for string in strings: words += string

                            user_process.get_logger().info(
                                f'\n<TOTAL CAR STATE>\n' \
                                f'\n<LEFT CAR LIST>\n {list(service_response.left_cars)}' \
                                f'\n\n<EXITED CAR LIST>\n{words}'
                            )
                        else:
                            user_process.get_logger().warn(
                                f'The (OUTPUT) car buffer is empty'
                            )
                        user_trigger = False
            else:
                input('Press Any Number(car id) for next service call.')
                future = user_process.send_request()
                user_trigger = True
    except KeyboardInterrupt:
        user_process.get_logger().info('Keyboard Interrupt (SIGINT).')

    user_process.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
