from msg_srv_action_interface_example.srv import Append, CoreToSlave
import rclpy
from rclpy.node import Node
import numpy as np
import roboidai as ai
import cv2
import time


class UserCamera(Node):

    def __init__(self):
        super().__init__('user_camera')

        self.camera_service_client = self.create_client(
            Append,
            'user_camera_information'
        )

        self.declare_parameter('capturing_time', 10.0)
        self.capturing_time = self.get_parameter('capturing_time').value

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
        self.parameters =  cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        self.mtx = np.array([[408.92439747   ,0.           ,315.34337658],
                             [  0.           ,409.02032845 ,143.63927478],
                             [  0.           ,0.           ,1.          ]])
        self.dist = np.array([[-4.06191363e-01,  2.80515124e-01, -2.25598493e-04,  7.67790538e-04, -1.71941377e-01]])
        self.cam = ai.Camera('ip0')

        while not self.camera_service_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('The camera server is not available.')

        self.timer = self.create_timer(0.1, self.increment_cnt)
        self.cnt = 0.0

    def increment_cnt(self):
        self.cnt += 0.1

    def send_request(self):
        start_time = self.cnt
        while (self.cnt - start_time <= self.capturing_time):
            img = self.cam.read()

            key = self.cam.check_key()
            if key == 'esc':
                break
            elif key == '[':
                self.cam.set_flip('h')
            elif key == ']':
                self.cam.set_flip(None)

            if img is not None:
                h, w = img.shape[:2]
                new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w,h))
                img = cv2.undistort(img, self.mtx, self.dist, None, new_camera_mtx)
                x, y, w, h = roi
                img = img[y:y+h, x:x+w]
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(gray)
                # print(markerIds)
                if len(markerCorners) > 0:
                    cv2.aruco.drawDetectedMarkers(img, markerCorners, markerIds)
                    if len(markerIds) == 1:
                        break
                cv2.imshow("Camera", img)

        #! self.cam.dispose()
        try:
            cv2.destroyWindow("Camera")
        except cv2.error as e:
            time.sleep(1)
            print('cv.DestoryWindow Error (When camera is stopped, the error occurs.)')
            return self.send_request()

        if(markerIds.shape[0] > 1):
            car_id = -1
        else:
            car_id = int(markerIds[0].astype(np.int32))
        service_request = Append.Request()
        service_request.car_id = car_id
        futures = self.camera_service_client.call_async(service_request)
        return futures

def main(args=None):
    rclpy.init(args=args)
    camera_interface = UserCamera()
    future = camera_interface.send_request()
    user_trigger = True
    try:
        while rclpy.ok():
            if user_trigger is True:
                rclpy.spin_once(camera_interface)
                if future.done():
                    try:
                        service_response = future.result()
                    except Exception as e:
                        camera_interface.get_logger().warn(f'Service call failed: {str(e)}')
                    else:
                        if service_response.is_overflow:
                            camera_interface.get_logger().warn(
                                f'(CAMERA) the memory is overflowed (the memory state is {list(service_response.car_ids)})'
                            )
                        elif service_response.is_duplicated:
                            camera_interface.get_logger().warn(
                                f'(CAMERA) the car_id has been already in the current camera buffer!'
                            )
                        else:
                            camera_interface.get_logger().info(
                                f'(CAMERA)Current Car State: {list(service_response.car_ids)}'
                            )
                    user_trigger = False
            else:
                input('Press Enter for next service call.')
                future = camera_interface.send_request()
                user_trigger = True
    except KeyboardInterrupt:
        camera_interface.get_logger().info('Keyboard Interrupt (SIGINT).')

    camera_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


