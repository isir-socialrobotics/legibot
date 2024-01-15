# Import SixDRepNet
from sixdrepnet import SixDRepNet
import cv2


# Create model
# Weights are automatically downloaded
model = SixDRepNet()

# img = cv2.imread('/path/to/image.jpg')
cap = cv2.VideoCapture("/home/javad/Downloads/Half Japanese in Japan.mp4")


while True:
    ret, img = cap.read()
    if not ret:
        break

    pitch, yaw, roll = model.predict(img)

    model.draw_axis(img, yaw, pitch, roll)

    cv2.imshow("test_window", img)
    cv2.waitKey(3)

cv2.waitKey(0)
cv2.destroyAllWindows()
cap.release()
