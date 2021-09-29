#!/usr/bin/env python
# -*- encoding: utf-8 -*-

# here put the import lib
import cv2
import numpy as np
import os

dir_f = os.path.abspath(os.path.dirname(__file__))
pd_path = os.path.join(dir_f,'model','traffic_detection.pb')
graph_path = os.path.join(dir_f,'model','traffic_detection.pbtxt')

net = cv2.dnn.readNetFromTensorflow(pd_path, graph_path)  # 加载模型
class_names = ['left', 'right', 'stop', 'straight', 'red', 'green']
def trafficdetect(img):
    im_width, im_height, img_channel = img.shape
    net.setInput(cv2.dnn.blobFromImage(img, size=(300, 300), swapRB=True, crop=False))
    cvOut = net.forward()

    rets = []
    types = []
    pp = []

    for detection in cvOut[0, 0, :, :]:
        score = float(detection[2])
        if score > 0.8:
            label = int(detection[1])
            left = int(detection[3] * im_height)
            top = int(detection[4] * im_width)
            right = int(detection[5] * im_height)
            bottom = int(detection[6] * im_width)
            #_rect = img[left: right, top: bottom]
            _rect = [left, top, right-left, bottom-top]
            cv2.rectangle(img, (int(left), int(top)), (int(right), int(bottom)), (0, 0, 255), thickness=2)
            cv2.putText(img, class_names[label - 1] + ': {:.2f}'.format(score), (int(left), int(top)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255), thickness=2)
            rets.append(_rect)
            types.append(class_names[label-1])
            pp.append(score)
    return (img, rets, types, pp)
    
if __name__ == '__main__':
    import sys
    cam = 0
    if len(sys.argv) > 1:
        cam = int(sys.argv[1])
    cap = cv2.VideoCapture(cam)

    while True:

        ret, image = cap.read()
        net.setInput(cv2.dnn.blobFromImage(image, size=(300, 300), swapRB=True, crop=False))
        im_height, im_width, img_channel = image.shape

        print("image shape", image.shape)
        output = net.forward()
        for detection in output[0, 0, :, :]:
            """
            detection输出结果为7维数据表示[0, 对应lable id, 置信度, 最左, 最上, 最右, 最下]
            最左与最上组成左上坐标(x1,y1)  最右与最下组成右下角坐标[x2,y2]
            """
            score = float(detection[2])
            if score > 0.85:
                left = int(detection[3] * im_width)
                top = int(detection[4] * im_height)
                right = int(detection[5] * im_width)
                bottom = int(detection[6] * im_height)
                cv2.rectangle(image, (int(left), int(top)), (int(right), int(bottom)),
                                (0, 0, 255), thickness=2)
                labels = ['left', 'right', 'stop', 'straight', 'red', 'green']
                return_result = labels[int(detection[1] - 1)]
                # result['msg'] = return_result
                print("预测结果为:", return_result)
                cv2.putText(image, return_result, (left, top), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), thickness=2)
        cv2.imshow("demo", image)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
