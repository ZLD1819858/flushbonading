import cv2
import numpy as np

net = cv2.dnn.readNetFromTensorflow('fruit_detection.pb', 'fruit_detection.pbtxt')

cap = cv2.VideoCapture(6)
_, first_img = cap.read()
# first_img = cv2.imread("chair_0217.jpg")
score_thresh = 0.6
class_names = ['apple', 'banana', 'chair', 'eraser', 'grape', 'ink_bottle', 'ladder', 'table', 'writing_case']
rows, cols, _ = first_img.shape
print(rows, cols)

while True:
    ret, img = cap.read()
    # img = cv2.imread("chair_0217.jpg")
    if ret is None:
        break
    im_width, im_height, img_channel = img.shape
    net.setInput(cv2.dnn.blobFromImage(img, size=(300, 300), swapRB=True, crop=False))
    cvOut = net.forward()
    rects = []

    index = 0
    for detection in cvOut[0, 0, :, :]:
        score = float(detection[2])
        if score > 0.4:
            label = int(detection[1])
            index += 1
            left = int(detection[3] * im_height)
            top = int(detection[4] * im_width)
            right = int(detection[5] * im_height)
            bottom = int(detection[6] * im_width)
            _rect = img[left: right, top: bottom]
            cv2.rectangle(img, (int(left), int(top)), (int(right), int(bottom)), (0, 0, 255), thickness=2)
            cv2.putText(img, class_names[label - 1] + ': {:.2f}'.format(score), (int(left), int(top)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255), thickness=2)

    cv2.imshow('img', img)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
