from sixdrepnet import SixDRepNet
import cv2

# Create model: Weights are automatically downloaded
model = SixDRepNet()
                                                                                                                                                                                                                                                                                                                                                                                                                
img = cv2.imread('/home/javad/Pictures/image-2.png')


# Load the cascade
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_profileface.xml')

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
faces = face_cascade.detectMultiScale(gray, 1.1, 4)
for (x, y, w, h) in faces:
    cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
    print(x, y, w, h)
    face_part = img[y:y+h, x:x+w]

    pitch, yaw, roll = model.predict(face_part)
    model.draw_axis(face_part, yaw, pitch, roll)
cv2.imshow("test_window", img)
cv2.waitKey(0)


