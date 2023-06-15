import roboidai as ai
import numpy as np
import cv2

# Load the ArUco marker dictionary.
# arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

mtx = np.array([[408.92439747   ,0.           ,315.34337658],
                [  0.           ,409.02032845 ,143.63927478],
                [  0.           ,0.           ,1.          ]])
dist = np.array([[-4.06191363e-01,  2.80515124e-01, -2.25598493e-04,  7.67790538e-04, -1.71941377e-01]])

cam = ai.Camera('ip0')
flag = False
cnt = 0

while True:
    image = cam.read()
    # cam.show(image)

    key = cam.check_key()
    if key == 'esc':
        break
    elif key == '[':
        cam.set_flip('h')
    elif key == ']':
        cam.set_flip(None)
    elif key == ' ':
        flag = True

    print(flag)
    #! 초반에 아무것도 안들어가짐 -> loading중이라 그런듯
    if image is not None:
        h,  w = image.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        image = cv2.undistort(image, mtx, dist, None, newcameramtx)
        x, y, w, h = roi
        image = image[y:y+h, x:x+w]
        # Convert the image to grayscale.
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        if flag:
          cv2.imwrite(f'./camera_images/{cnt}.jpg', gray)
          cnt += 1
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray)
        if len(markerCorners) > 0:
            cv2.aruco.drawDetectedMarkers(image, markerCorners, markerIds)

        cv2.imshow("Image", image)

    flag = False



cam.dispose()
cv2.destroyAllWindows()


#! 가져온 checker board는 개당 54cm

'''
import cv2
import cv2.aruco as aruco

# 마커 사전 정의
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# 카메라 초기화
cap = cv2.VideoCapture(0)

while True:
    # 카메라 프레임 읽기
    ret, frame = cap.read()

    # 그레이스케일로 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 마커 감지
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        # 마커 인식 시 동작
        aruco.drawDetectedMarkers(frame, corners, ids)

    # 영상 출력
    cv2.imshow('ArUco Marker Detection', frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 리소스 해제
cap.release()
cv2.destroyAllWindows()
'''


