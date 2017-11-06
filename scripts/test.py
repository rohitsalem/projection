import cv2

img = cv2.imread('1342657000000.JPEG')

x_c = 343
y_c = 397
X = 68
Y = 110

x_min = x_c - X/2
x_max = x_c + X/2
y_min = y_c - Y/2
y_max = y_c + Y/2
cv2.rectangle(img,(x_min, y_min),(x_max, y_max),(0,255,0),3)

cv2.imshow("image",img)
cv2.waitKey(0)
