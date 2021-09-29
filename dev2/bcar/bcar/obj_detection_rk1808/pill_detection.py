import cv2
import time
import numpy as np
from rknn_client_class import rknn_client
from PIL import Image, ImageFont, ImageDraw
import sys
import os
CLASSES = ("box1", "box2", "box3", "box4")

this = sys.modules[__name__]
this.rknn = None
def __init():
    os.system("sudo ifconfig enx10dcb69f100e 192.168.180.100")
    while this.rknn == None:
        this.rknn = rknn_client(8004)
        if this.rknn == None:
            time.sleep(5)
    print 'rknn init down'

import threading
_t = threading.Thread(target=__init)
_t.setDaemon(True)
_t.start()

def get_center(outputs, _shape):
    _results = []
    for _i in range(0, len(outputs)):
        x1 = outputs[_i][0] * _shape[1]
        y1 = outputs[_i][1] * _shape[0]
        x2 = outputs[_i][2] * _shape[1]
        y2 = outputs[_i][3] * _shape[0]
        pred = CLASSES[int(outputs[_i][5]) - 1]
        res = (((x1 + x2) / 2, (y1 + y2) / 2), pred, outputs[_i][4])
        _results.append(res)

    return _results


def __check(frame):
    image_shape = frame.shape[:2]
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image = cv2.resize(image, (300, 300))
    box_class_score = rknn.inference(inputs=[image])
    img_pil = Image.fromarray(frame)
    draw = ImageDraw.Draw(img_pil)
    font = ImageFont.load_default()
    results = get_center(box_class_score, image_shape)

    rets = []
    for i in range(0, len(box_class_score)):
        x1 = box_class_score[i][0] * image_shape[1]
        y1 = box_class_score[i][1] * image_shape[0]
        x2 = box_class_score[i][2] * image_shape[1]
        y2 = box_class_score[i][3] * image_shape[0]
        # draw rect
        color = (0, int(box_class_score[i][5] / 20.0 * 255), 255)
        draw.line([(x1, y1), (x1, y2), (x2, y2),
                   (x2, y1), (x1, y1)], width=2, fill=color)

        rets.append((int(x1+0.5),int(y1+0.5),int(x2-x1+0.5), int(y2-y1+0.5)))

        display_str = CLASSES[int(box_class_score[i][5]) - 1] + ":" + str('%.2f' % box_class_score[i][4])
        display_str_height = np.ceil((1 + 2 * 0.05) * font.getsize(display_str)[1]) + 1
        if y1 > display_str_height:
            text_bottom = y1
        else:
            text_bottom = y1 + display_str_height
        text_width, text_height = font.getsize(display_str)
        margin = np.ceil(0.05 * text_height)
        draw.rectangle([(x1, text_bottom - text_height - 2 * margin), (x1 + text_width, text_bottom)],
                       fill=color)
        draw.text((x1 + margin, text_bottom - text_height - margin), display_str, fill='black', font=font)
    np.copyto(frame, np.array(img_pil))
    return frame, rets

    #cv2.imshow("results", frame)
    #c = cv2.waitKey(1) & 0xff
    #if c == 27:
    #    cv2.destroyAllWindows()
    #    capture.release()
    #    break

def check(frame):
    ret = []
    img, rets = __check(frame)
    if len(rets) > 0:
        ret = rets[0]
    return img, ret

if __name__ == '__main__':


    font = cv2.FONT_HERSHEY_SIMPLEX
    # capture = cv2.VideoCapture("data/3.mp4")
    capture = cv2.VideoCapture(4)
    accum_time = 0
    curr_fps = 0
    prev_time = time.time()
    fps = "FPS: ??"
    point_res = []
    while (True):
        ret, frame = capture.read()
        image_shape = frame.shape[:2]
        if ret == True:
            frame, rets = check(frame)
            cv2.imshow("results", frame)
            c = cv2.waitKey(1) & 0xff
            if c == 27:
                cv2.destroyAllWindows()
                capture.release()
                break
