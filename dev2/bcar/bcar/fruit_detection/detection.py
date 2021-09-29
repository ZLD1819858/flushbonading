import cv2
import numpy as np
import os
from copy import deepcopy

c_dir = os.path.split(os.path.realpath(__file__))[0]
pill_net = cv2.dnn.readNetFromTensorflow(c_dir+'/fruit_detection.pb', c_dir+'/fruit_detection.pbtxt')
pill_class_names = ['apple', 'banana', 'chair', 'eraser', 'grape', 'ink_bottle', 'ladder', 'table', 'writing_case']
'''
traffic_net = cv2.dnn.readNetFromTensorflow(c_dir+'/traffic_20200706.pb', c_dir+'/traffic_20200706.pbtxt')
traffic_class_names = ['red', 'green', 'right', 'stop', 'left', 'straight', 'pedestrain', 'nolight']
'''
def __check(img, net, class_name):
    #img = deepcopy(frame)
    im_width, im_height, img_channel = img.shape
    net.setInput(cv2.dnn.blobFromImage(img, size=(300, 300), swapRB=True, crop=False))
    cvOut = net.forward()
    
    rets = []
    types = []
    pp = []
    
    for detection in cvOut[0, 0, :, :]:
        score = float(detection[2])
        if score > 0.5:
            label = int(detection[1])
   
            left = int(detection[3] * im_height)
            top = int(detection[4] * im_width)
            right = int(detection[5] * im_height)
            bottom = int(detection[6] * im_width)
            
            _rect = img[left: right, top: bottom]
            cv2.rectangle(img, (int(left), int(top)), (int(right), int(bottom)), (0, 0, 255), thickness=2)
            cv2.putText(img, class_name[label - 1] + ': {:.2f}'.format(score), (int(left), int(top)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255), thickness=2)
            rets.append((left,top,right-left,bottom-top))
            types.append(class_name[label - 1])
            pp.append(score)
    return img, rets, types, pp

def pilldetect(frame):
    img, rect, types, pp = __check(frame, pill_net, pill_class_names)
    maxidx = 0
    maxval = 0
    ret = []
    type = ""
    for i in range(len(pp)):
        if pp[i] > maxval:
            maxidx = i
            maxval = pp[i]
            ret = rect[i]
            type = types[i]
    return img, ret, type
'''
def trafficdetect(fram):
    img, rect, types, pp = __check(frame, traffic_net, traffic_class_names)
    return img, rect, types, pp
   ''' 
    
if __name__ == '__main__':
    cap = cv2.VideoCapture(6)
    # cap = cv2.VideoCapture('output.avi')
    _, first_img = cap.read()
    rows, cols, _ = first_img.shape
    print(rows, cols)
    while True:
        ret, img = cap.read()
        if ret is None:
            break
        img, rets   = pilldetect(img)
        print rets
         
        cv2.imshow('img', img)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
