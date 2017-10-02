import cv2


img = cv2.imread('picc.png')
img =cv2.resize(img,(512,512))
# img =cv2.flip(img,0)
# img [235,88:181]=[0,0,0]
# img [35, 88:181]= [0,0,0]
# img [35:235,88] = [0,0,0]
# img [35:235,181] = [0,0,0]
# img [89:184,242]=[0,0,0]
# img [89:184,68]= [0,0,0]
# img [89,68:242] = [0,0,0]
# img [184, 68:242] = [0,0,0]
img [235,214:297]=[0,0,0]
img [35, 214:297]= [0,0,0]
img [35:235,214] = [0,0,0]
img [35:235,297] = [0,0,0]
cv2.imshow("image", img)
cv2.imwrite("box.png",img)
cv2.waitKey(0)
cv2.destroyAllWindows()
