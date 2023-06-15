import numpy as np
import cv2 as cv

view = 0

mtx = np.array([[408.92439747   ,0.           ,315.34337658],
                [  0.           ,409.02032845 ,143.63927478],
                [  0.           ,0.           ,1.          ]])

dist = np.array([[-4.06191363e-01,  2.80515124e-01, -2.25598493e-04,  7.67790538e-04, -1.71941377e-01]])

img = cv.imread(f'./camera_images/{view}.jpg')
h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

# undistort
dst = cv.undistort(img, mtx, dist, None, newcameramtx)
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]

cv.imwrite(f'./calibrated_camera_images/{view}_01.jpg', dst)

# undistort
mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)

# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imwrite(f'./calibrated_camera_images/{view}_02.jpg', dst)
