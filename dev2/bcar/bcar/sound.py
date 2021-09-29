#!/usr/bin/python
# -*- coding: utf-8 -*-
import pyttsx3
import threading
import os
import sys

from Queue import Queue
import time
this  = sys.modules[__name__]


this.msgque = Queue(maxsize=8)


def run():
    en = pyttsx3.init()
    en.setProperty('voice', 'zh')
    en.setProperty('volume', 1.0)
    while True:
        msg, evt =  this.msgque.get()
        en.say(msg)
        en.runAndWait()
        if evt:
            evt.set()

t = threading.Thread(target=run)
t.setDaemon(True)
t.start()


def say(str, wait=False):
    evt = None
    if wait:
        evt = threading.Event()
    this.msgque.put((str, evt))
    if wait:
        evt.wait()
    
#dir_f = os.path.abspath(os.path.dirname(__file__))
#def bb():
#    os.system("aplay "+dir_f+"/bb.wav")