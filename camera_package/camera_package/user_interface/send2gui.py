from msg_srv_action_interface_example.srv import SendToGUI
import rclpy
from rclpy.node import Node

class UserInput(Node):

    def __init__(self):
        super().__init__('processed_car_node')

        self.user_service_client = self.create_client(
            SendToGUI,
            'processed_cars',
        )

        while not self.user_service_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('The user input interface service not available.')

    def send_request(self, car_id):
        service_request = SendToGUI.Request()
        futures = self.user_service_client.call_async(service_request)
        return futures



def main(args=None):
    # parse = argparse.ArgumentParser()
    # parse.add_argument('--car_id', type=int, default=0, help='Input the car id.')
    # opt = parse.parse_args()

    rclpy.init(args=args)
    # start rclpy
    user_in_interface = UserInput()
    future = user_in_interface.send_request(-1)
    user_trigger = True
    try:
        while rclpy.ok():
            if user_trigger is True:
                rclpy.spin_once(user_in_interface)
                if future.done():
                    try:
                        service_response = future.result()
                    except Exception as e:
                        user_in_interface.get_logger().warn(f'Service call failed: {str(e)}')
                    else:
                        user_in_interface.get_logger().info(f'(INPUT) The current buffer {list(service_response.in_cars)}')
                        user_in_interface.get_logger().info(f'(INPUT) is illegal? {list(service_response.in_fake)}')
                        user_in_interface.get_logger().info(f'(OUTPUT) The current buffer {list(service_response.out_cars)}')
                        user_in_interface.get_logger().info(f'(OUTPUT) is illegal? {list(service_response.out_fake)}')

                        user_trigger = False
            else:
                car_id = input('Press Any Number(car id) for next service call.')
                future = user_in_interface.send_request(-1)
                user_trigger = True
    except KeyboardInterrupt:
        user_in_interface.get_logger().info('Keyboard Interrupt (SIGINT).')

    user_in_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


