from msg_srv_action_interface_example.srv import Append, Process, CoreToSlave, Enrollment, SendToGUI


from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

import time

#TODO Database 노드에서 3개의 client를 만들어서 받아오는 거 구현
class RobotCore(Node):

  def __init__(self):
    super().__init__("robot_core")

    self.state_in_cars = {}
    self.state_out_cars = {}
    self.state_camera_cars = [] # 사진을 찍어서 확인하는 노드

    self.declare_parameter('maxlen', 10)
    self.maxlen = self.get_parameter('maxlen').value

    self.callback_group_in = ReentrantCallbackGroup()
    self.callback_group_out = ReentrantCallbackGroup()

    self.declare_parameter('qos_depth', 10)
    qos_depth = self.get_parameter('qos_depth').value

    # QOS_RKL10V = QoSProfile(
    #     reliability=QoSReliabilityPolicy.RELIABLE,
    #     history=QoSHistoryPolicy.KEEP_LAST,
    #     depth=qos_depth,
    #     durability=QoSDurabilityPolicy.VOLATILE)

    self.state_in_server = self.create_service(
      Append,
      'user_in_information',
      self.execute_in_buffer,
      # callback_group=self.callback_group_in
    )

    #! callback_group를 여러개로 나누어야 할 듯
    self.state_out_server = self.create_service(
      Append,
      'user_out_information',
      self.execute_out_buffer,
      # callback_group=self.callback_group_out
    )

    self.process_server = self.create_service(
      Process,
      'process_information',
      self.execute_inner_states,
    )

    #TODO 미리 client에서 처리하는 함수를 만들어서 보낼 수 있도록
    self.camera_server = self.create_service(
      Append,
      'user_camera_information',
      self.execute_camera_buffer,
    )

    self.timer = self.create_timer(0.1, self.timer_func)
    self.cnt = 0.0

  def timer_func(self):
    self.cnt += 0.1

  #! dictionary 특성상 같은 키가 두 번 입력되어도 중복 입력되지 않는다.
  def execute_in_buffer(self, request, response):
    car_id = request.car_id
    car_time = self.cnt# time.time() #! request.start_time
    response.is_overflow = False
    response.is_duplicated = False
    response.is_exist_input_buffer = False


    if (car_id == -1):
      self.get_logger().info(f"(INPUT) The current buffer state is: {list(self.state_in_cars.keys())}")
    elif (car_id in self.state_in_cars.keys()):
      response.is_duplicated = True
      self.get_logger().warn(f"(INPUT) the car_id is duplicated!")
    elif (len(self.state_in_cars) >= self.maxlen):
      response.is_overflow = True
      self.get_logger().warn(f"(INPUT) the buffer is overflowed! / the maximum size of buffer is {self.maxlen}.")
    else:
      self.state_in_cars[car_id] = car_time
      self.get_logger().info(f"(INPUT) car_id: {car_id} / car_time: {self.state_in_cars[car_id]:.1f}")

    response.car_ids = list(self.state_in_cars.keys())
    response.car_times = list(self.state_in_cars.values())

    return response

  def execute_out_buffer(self, request, response):
    car_id = request.car_id
    car_time = self.cnt
    response.is_overflow = False
    response.is_duplicated = False
    response.is_exist_input_buffer = False
    flag = False

    if (car_id == -1):
      self.get_logger().info(f"(OUTPUT) The current buffer state is: {list(self.state_out_cars.keys())}")
    elif(car_id not in self.state_in_cars.keys()):
      self.get_logger().warn(f"(OUTPUT) This car_id does NOT exist in INPUT buffer!")
      response.is_exist_input_buffer = True
      flag = True
    elif (car_id in self.state_out_cars.keys()):
      response.is_duplicated = True
      self.get_logger().warn(f"(OUTPUT) the car_id is duplicated!")
    elif (len(self.state_out_cars) >= self.maxlen):
      response.is_overflow = True
      self.get_logger().warn(f"(OUTPUT) the buffer is overflowed! / the maximum size of buffer is {self.maxlen}.")
    else:
      self.state_out_cars[car_id] = car_time
      self.get_logger().info(f"(OUTPUT) car_id: {car_id} / car_time: {self.state_out_cars[car_id]:.1f}")

    response.car_ids = list(self.state_out_cars.keys())
    response.car_times = list(self.state_out_cars.values())

    if flag:
      response.car_ids = list(self.state_in_cars.keys())


    return response

  def execute_inner_states(self, request, response):

    #! response or requset는 write으로써만 사용하자
    if len(self.state_out_cars) == 0:
      self.get_logger().info("(OUTPUT) the state buffer is empty!")
      response.out_flag = False
      return response

    cars_to_be_deleted = []
    car_times = []
    for output_car in self.state_out_cars.keys():
      if output_car in self.state_in_cars.keys():
        cars_to_be_deleted.append(output_car)
        car_times.append(self.state_out_cars[output_car] - self.state_in_cars[output_car])

    for car in cars_to_be_deleted:
      del self.state_in_cars[car]
      del self.state_out_cars[car]

    self.get_logger().info(f"The state buffer has been renewed.")
    self.get_logger().info(f"(INPUT) the left car_id is : {list(self.state_in_cars)}")

    response.out_flag = True
    response.left_cars = list(self.state_in_cars.keys())
    response.car_times = list(car_times)
    response.exited_cars = list(cars_to_be_deleted)
    return response

  def execute_camera_buffer(self, request, response):
    car_id = request.car_id
    response.is_overflow = False
    response.is_duplicated = False
    response.is_exist_input_buffer = False

    if (car_id == -1):
      self.get_logger().warn(f'Multiple cars are received')
    elif (len(self.state_camera_cars) >= self.maxlen):
      response.is_overflow = True
      self.get_logger().warn(f"(CAMERA) the buffer is overflowed! / the maximum size of buffer is {self.maxlen}.")
    elif (car_id in self.state_camera_cars): #! 같은 차를 다시 인식해도 pass
      self.get_logger().warn(f"(CAMERA) the car_id has been already enrollmented!")
      response.is_duplicated = True
    else:
      self.state_camera_cars.append(car_id)

    response.car_ids = list(self.state_camera_cars)
    self.get_logger().info(f"(CAMERA) car_ids: {list(self.state_camera_cars)}")
    return response




#TODO Database 노드에서 3개의 client를 만들어서 받아오는 거 구현
class DataBase(Node):

  def __init__(self):
    super().__init__("database")

    self.entered_car_buffer = {}
    self.exited_car_buffer = {}
    self.identified_car_buffer = {}
    self.illegal_entered_car_list = []
    self.illegal_exited_car_list = []
    self.maximum_time = 1e6

    self.declare_parameter('size', 10)
    self.size = self.get_parameter('size').value

    self.callback_group = ReentrantCallbackGroup()

    self.camera_client = self.create_client(
      Enrollment,
      'camera_to_capture_car_id'
    )

    self.to_gui_client = self.create_service(
       SendToGUI,
      'processed_cars',
      self.send_to_gui,
    )

    self.timer = self.create_timer(0.1, self.increment_cnt)
    self.cnt = 0.0

    while not self.camera_client.wait_for_service(timeout_sec=0.1):
      self.get_logger().warning('Camera Client Service NOT available')
    while not self.send_to_gui_client.wait_for_service(timeout_sec = 0.1):
      self.get_logger().warning('SendToGUI Client Service NOT available')


  def increment_cnt(self):
    self.cnt += 0.1

  def user_interface(self, key):
    if key == 1:
      self.entered_car_buffer[self.send_car_request()] = self.cnt
      self.get_logger().info(f"[ENTER] buffer state: {list(self.entered_car_buffer.keys())}")
    elif key == 2:
      self.exited_car_buffer[self.send_car_request()] = self.cnt
      self.get_logger().info(f"[EXIT] buffer state: {list(self.exited_car_buffer.keys())}")
    elif key == 3:
      self.identified_car_buffer[self.send_car_request()] = self.cnt
      self.get_logger().info(f"[ROBOT] buffer state: {list(self.identified_car_buffer.keys())}")
    elif key == 4:
      self.send_to_gui()
    elif key == 0:
      self.show_current_state()
    else:
        raise KeyError(f'The value to be received is 1,2,3,4 and 0 / but received: {key}')


  def send_car_request(self):
    service_request = Enrollment.Request()
    futures = self.camera_client.call_async(service_request)
    car_id = None
    if futures.done():
      try:
        service_response = futures.result()
      except Exception as e:
        self.get_logger().warn(f'Service call failed {str(e)}')
      else:
        car_id = service_response.car_id
    else:
      self.get_logger().warn(f'futures is NOT done.')

    return car_id

  def show_current_state(self):
    self.get_logger().info(
                f'<ENTERED CAR BUFFER>\n{list(self.entered_car_buffer)}\n\n'\
                f'<EXITED CAR BUFFER>\n{list(self.exited_car_buffer)}\n\n'\
                f'<IDENTIFIED CAR BUFFER>\b{list(self.identified_car_buffer)}\n\n')

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

  def send_to_gui(self):
    entered_cars, is_illegal_entered_cars, exited_cars, exited_cars_time,\
        is_illegal_exited_cars, illegal_entered_cars_list, illegal_exited_cars_list = self.processing_car()
    pass

  # def send_to_gui(self):

  #   service_request = SendToGUI.Request()
  #   service_request.entered_cars = entered_cars
  #   service_request.is_illegal_entered_cars = is_illegal_entered_cars
  #   service_request.exited_cars = exited_cars
  #   service_request.exited_cars_time = exited_cars_time
  #   service_request.is_illegal_exited_cars = is_illegal_exited_cars
  #   futures = self.send_to_gui_client.call_async(service_request)

  #   if futures.done():
  #     try:
  #       service_response = futures.result()
  #     except Exception as e:
  #       self.get_logger().warn(f"Service call failed: {str(e)}")
  #     else:
  #       if service_response.does_work_out:
  #         self.get_logger().info(f'The values are sent to GUI successfully!')
  #       else:
  #         self.get_logger().warn(f'There some errors when sending to GUI.')
  #   else:
  #     self.get_logger().warn(f'futures is NOT done.')















#  self.execute_inner_states,






