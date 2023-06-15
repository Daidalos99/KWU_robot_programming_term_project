from msg_srv_action_interface_example.srv import Enrollment
import rclpy
from rclpy.node import Node
import numpy as np
import roboidai as ai
import cv2
import time


class CaemraServer(Node):

    def __init__(self):
        super().__init__('camera_server')
        self.camera_server = self.create_service(
            Enrollment,
            'user_camera_information',
            self.execute_camera
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

    def execute_camera(self, request, response):
        flag = False
        start_time = time.time()
        while (time.time() - start_time <= self.capturing_time):
            key = self.cam.check_key()
            if key == 'esc':
                break
            elif key == '[':
                self.cam.set_flip('h')
            elif key == ']':
                self.cam.set_flip(None)

            img = self.cam.read()

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
                flag = True

        #! self.cam.dispose()는 main에서 실행
        try:
            if flag:
                cv2.destroyWindow("Camera")
        except cv2.error as e:
            time.sleep(1)
            print('cv.DestoryWindow Error (When camera is stopped, the error occurs.)')
            return self.execute_camera(request, response)

        if(markerIds.shape[0] > 1):
            car_id = -1
        else:
            car_id = int(markerIds[0].astype(np.int32))

        response.car_id = car_id
        flag = False
        return response


def main(args=None):
    rclpy.init(args=args)
    camera_server = CaemraServer()

    try:
        while rclpy.ok():
            rclpy.spin(camera_server)
    except KeyboardInterrupt:
            camera_server.get_logger.info('Keyboard Interrupt (SIGINT)')

    camera_server.cam.dispose()
    camera_server.camera_server.destroy()
    camera_server.destroy_node()
    rclpy.shutdown()
