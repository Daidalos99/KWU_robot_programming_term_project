from msg_srv_action_interface_example.srv import Append, SendToGUI

from rclpy.node import Node


class Database(Node):
    #! 그냥 모든 것을 서버로 진행
    def __init__(self):
        super().__init__('database')

        self.entered_car_buffer = {}
        self.exited_car_buffer = {}
        self.identified_car_buffer = {}
        self.illegal_entered_car_list = []
        self.illegal_exited_car_list = []
        self.maximum_time = 1e6

        self.declare_parameter('size', 10)
        self.size = self.get_parameter('size').value

        self.entered_car_server = self.create_service(
            Append,
            'user_in_information',
            self.execute_in_buffer
        )

        self.exited_car_server = self.create_service(
            Append,
            'user_out_information',
            self.execute_out_buffer
        )

        self.camera_car_server = self.create_service(
            Append,
            'user_camera_information',
            self.execute_camera_buffer
        )

        self.gui_server = self.create_service(
            SendToGUI,
            'processed_cars',
            self.send_to_gui
        )

        self.timer = self.create_timer(0.1, self.increment_cnt)
        self.cnt = 0.0

        # while not self.camera_client.wait_for_service(timeout_sec=0.1):
        #     self.get_logger().warning('Camera Client is NOT avaliable.')


    def increment_cnt(self):
        self.cnt += 0.1

    def execute_in_buffer(self, request, response):
        car_id = request.car_id
        car_time = self.cnt
        response.is_overflow = False
        response.is_duplicated = False
        response.is_exist_input_buffer = False


        if (car_id == -1):
            self.get_logger().info(f"(INPUT) The current buffer state is: {list(self.entered_car_buffer.keys())}")
        elif (car_id in self.entered_car_buffer.keys()):
            response.is_duplicated = True
            self.get_logger().warn(f"(INPUT) the car_id is duplicated!")
        elif (len(self.entered_car_buffer) >= self.size):
            response.is_overflow = True
            self.get_logger().warn(f"(INPUT) the buffer is overflowed! / the maximum size of buffer is {self.size}.")
        else:
            self.entered_car_buffer[car_id] = car_time
            self.get_logger().info(f"(INPUT) car_id: {car_id} / car_time: {self.entered_car_buffer[car_id]:.1f}")

        response.car_ids = list(self.entered_car_buffer.keys())
        response.car_times = list(self.entered_car_buffer.values())

        return response

    def execute_out_buffer(self, request, response):
        car_id = request.car_id
        car_time = self.cnt
        response.is_overflow = False
        response.is_duplicated = False
        response.is_exist_input_buffer = False
        flag = False

        if (car_id == -1):
            self.get_logger().info(f"(OUTPUT) The current buffer state is: {list(self.exited_car_buffer.keys())}")
        elif(car_id not in self.entered_car_buffer.keys()):
            self.get_logger().warn(f"(OUTPUT) This car_id does NOT exist in INPUT buffer!")
            response.is_exist_input_buffer = True
            flag = True
        elif (car_id in self.exited_car_buffer.keys()):
            response.is_duplicated = True
            self.get_logger().warn(f"(OUTPUT) the car_id is duplicated!")
        elif (len(self.exited_car_buffer) >= self.size):
            response.is_overflow = True
            self.get_logger().warn(f"(OUTPUT) the buffer is overflowed! / the maximum size of buffer is {self.size}.")
        else:
            self.exited_car_buffer[car_id] = car_time
            self.get_logger().info(f"(OUTPUT) car_id: {car_id} / car_time: {self.exited_car_buffer[car_id]:.1f}")

        response.car_ids = list(self.exited_car_buffer.keys())
        response.car_times = list(self.exited_car_buffer.values())

        if flag:
           response.car_ids = list(self.exited_car_buffer.keys())

        return response

    def execute_camera_buffer(self, request, response):
        car_id = request.car_id
        car_time = self.cnt
        response.is_overflow = False
        response.is_duplicated = False
        response.is_exist_input_buffer = False
        flag = False

        if (car_id == -1):
            self.get_logger().info(f"(CAMERA) The current buffer state is: {list(self.exited_car_buffer.keys())}")
        elif (car_id in self.identified_car_buffer.keys()):
            response.is_duplicated = True
            self.get_logger().warn(f"(CAMERA) the car_id is duplicated!")
        elif (len(self.identified_car_buffer) >= self.size):
            response.is_overflow = True
            self.get_logger().warn(f"(CAMERA) the buffer is overflowed! / the maximum size of buffer is {self.size}.")
        else:
            self.identified_car_buffer[car_id] = car_time
            self.get_logger().info(f"(CAMERA) car_id: {car_id} / car_time: {self.exited_car_buffer[car_id]:.1f}")

        response.car_ids = list(self.identified_car_buffer.keys())
        response.car_times = list(self.identified_car_buffer.values())

        if flag:
           response.car_ids = list(self.identified_car_buffer.keys())

        return response


    def processing_car(self):
        exited_cars = []
        exited_cars_time = []

        illegal_entered_cars = []
        is_illegal_entered_cars = []
        illegal_exited_cars = []
        is_illegal_exited_cars = []


        #* 정상적으로 나간 차량들에 대한 처리
        for exited_car in self.exited_car_buffer.keys():
            if exited_car in self.entered_car_buffer.keys():
                exited_cars.append(exited_car)
                exited_cars_time.append(self.exited_car_buffer[exited_car] - self.entered_car_buffer[exited_car])
                del self.exited_car_buffer[exited_car]
                del self.entered_car_buffer[exited_car]

        #* Robot으로 들어올 때 꼬리잡기한 자동차들이 있는 지 확인
        for identified_car in self.identified_car_buffer.keys():
            if identified_car not in self.entered_car_buffer.keys():
                illegal_entered_cars.append(identified_car)

        #* Robot으로 나갈 떄 꼬리잡기한 자동차들이 있는 지 확인
        for entered_car in self.entered_car_buffer.keys():
            if entered_car not in self.identified_car_buffer.keys():
                illegal_exited_cars.append(entered_car)

        #* 꼬리잡기로 들어온 차량들에 대한 처리
        for _ in self.entered_car_buffer.keys():
            is_illegal_entered_cars.append(False)
        if illegal_entered_cars:
            for illegal_entered_car in illegal_entered_cars:
                self.entered_car_buffer[illegal_entered_car]
                is_illegal_entered_cars.append(True)
                self.illegal_entered_car_list.append(illegal_entered_car)

        #* 꼬리잡기로 나간 차량들에 대한 처리
        for _ in exited_cars:
            is_illegal_exited_cars.append(False)
        if illegal_exited_cars:
            for illegal_exited_car in illegal_exited_cars:
                exited_cars.append(illegal_exited_car)
                exited_cars_time.append(self.maximum_time)
                is_illegal_exited_cars.append(True)
                self.illegal_exited_car_list.append(illegal_exited_car)


        #* 로봇 안에 있는 identified_car_buffer는 비우기
        for identified_car in self.identified_car_buffer:
            del self.identified_car_buffer[identified_car]

        #! 아래 있는 것들을 리턴
        return [self.entered_car_buffer.keys(), is_illegal_entered_cars,
                exited_cars, exited_cars_time, is_illegal_exited_cars,
                self.illegal_entered_car_list, self.illegal_exited_car_list]

    def send_to_gui(self, request, response):
        entered_cars, is_illegal_entered_cars, exited_cars, exited_cars_time,\
        is_illegal_exited_cars, illegal_entered_cars_list, illegal_exited_cars_list = self.processing_car()

        response.in_cars = entered_cars
        response.in_fake = is_illegal_entered_cars
        response.out_cars = exited_cars
        response.out_fake = is_illegal_exited_cars

        return response


'''
class CameraClient(Node):

    def __init__(self):
        super().__init__("camera_client")
        self.identified_car_buffer = {}
        self.camera_client = self.create_client(
            Enrollment,
            'user_camera_information'
        )
        while not self.camera_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('Camera Client is NOT avaliable.')

    def camera_car_request(self):
        service_request = Enrollment.Request()
        futures = self.camera_client.call_async(service_request)
        return futures
'''
