#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
from ws4py.client.threadedclient import WebSocketClient
import sys
import time
import threading

class AIXCarClient(WebSocketClient):
    def __init__(self, mac="AI:XCar:ad"):
        url = "ws://127.0.0.1:6001"
        WebSocketClient.__init__(self, url)

        self.mac = mac
        self.connectStatus = 0
    
        

    def send_message(self, msg):
        m = {
            "method":"message",
            "addr":self.mac,
            "data":msg
        }
        #print 'send', msg
        self.send(json.dumps(m))
        
    def opened(self):
        #print 'opend'
        self.connectStatus = 1
        #self.send_message('aaa')
    
    def closed(self, code, reason=None):
        #print 'closed'
        self.connectStatus = 2
        #self.__onClosedCall()

    def received_message(self, msg):
        print 'msg', msg
        jo = json.loads(str(msg))
        if jo['method'] == 'control':
            if jo['addr'] == self.mac:
                _onRecvMessage(jo['data'])


        
this = sys.modules[__name__]
this.client = None
this.__onRecvMessageCb = None
def setOnRecvMessageCb(cb):
    this.__onRecvMessageCb = cb
    
def sendMessage(msg):
    if this.client != None and this.client.connectStatus == 1:
        this.client.send_message(msg)
        
def _onRecvMessage(msg):
    if this.__onRecvMessageCb != None:
        this.__onRecvMessageCb(msg)
        
def __run():
    while True:
        if this.client == None or this.client.connectStatus == 2:
            this.client  = AIXCarClient()
            try:
                this.client.connect()
            except:
                print 'connect zxbeegw error!'
                this.client = None
        time.sleep(10)
            
__t = threading.Thread(target=__run)
__t.setDaemon(True)
__t.start()
        
if __name__ == '__main__':
    import time
    time.sleep(60)
    