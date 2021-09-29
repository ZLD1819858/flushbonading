import cv2
import time
from rknn_client_class import rknn_client

CLASSES = ("right", 'left', 'red', 'straight')


if __name__ == '__main__':
    rknn = rknn_client(8004)

    font = cv2.FONT_HERSHEY_SIMPLEX
    # capture = cv2.VideoCapture("pill_20200715.avi")
    capture = cv2.VideoCapture(0)
    accum_time = 0
    curr_fps = 0
    prev_time = time.time()
    fps = "FPS: ??"
    point_res = []
    while True:
        ret, frame = capture.read()
        # frame = cv2.imread(_img)
        # ret = True
        image_shape = frame.shape[:2]
        if ret:
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = cv2.resize(image, (300, 300))
            # image = cv2.resize(frame, (608, 608))

            testtime = time.time()
            # boxes, classes, scores = rknn.inference(inputs=[image])
            box_class_score = rknn.inference(inputs=[image])

            for i in range(0, len(box_class_score)):
                x1 = box_class_score[i][0] * image_shape[1]
                y1 = box_class_score[i][1] * image_shape[0]
                x2 = box_class_score[i][2] * image_shape[1]
                y2 = box_class_score[i][3] * image_shape[0]

                label = int(box_class_score[i][5])
                score = box_class_score[i][4]

                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), thickness=2)
                cv2.putText(frame, CLASSES[label - 1] + ': {:.2f}'.format(score), (int(x1), int(y1)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), thickness=2)

            cv2.imshow("results", frame)
            c = cv2.waitKey(0) & 0xff
            if c == 27:
                cv2.destroyAllWindows()
                capture.release()
                break
