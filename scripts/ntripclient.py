#!/usr/bin/python

import rospy

#from nmea_msgs.msg import Sentence
from mavros_msgs.msg import RTCM

import datetime
from httplib import HTTPConnection
from base64 import b64encode
from threading import Thread

class ntripconnect(Thread):
    def __init__(self, ntc):
        super(ntripconnect, self).__init__()
        self.ntc = ntc
        self.stop = False

    def run(self):
        headers = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP ntrip_ros',
            'Connection': 'close',
            'Authorization': 'Basic ' + b64encode(self.ntc.ntrip_user + ':' + self.ntc.ntrip_pass)
        }
        connection = HTTPConnection(self.ntc.ntrip_server)
        now = datetime.datetime.utcnow()
        connection.request('GET', '/'+self.ntc.ntrip_stream, self.ntc.nmea_gga % (now.hour, now.minute, now.second), headers)
        
        response = connection.getresponse()
        if response.status != 200: raise Exception("blah")
        buf = ""
        rmsg = RTCM()
        while not self.stop:
            data = response.read(1)
            if data!=chr(211):
                continue
            l1 = ord(response.read(1))
            l2 = ord(response.read(1))
            pkt_len = ((l1&0x3)<<8)+l2
    
            pkt = response.read(pkt_len)
            parity = response.read(3)
            if len(pkt) != pkt_len:
                rospy.logerr("Length error: {} {}".format(len(pkt), pkt_len))
                continue
            rmsg.header.seq += 1
            rmsg.header.stamp = rospy.get_rostime()
            rmsg.data = data + chr(l1) + chr(l2) + pkt + parity
            self.ntc.pub.publish(rmsg)

        connection.close()

class ntripclient:
    def __init__(self):
        rospy.init_node('ntripclient', anonymous=True)

        self.rtcm_topic = rospy.get_param('~rtcm_topic')
        self.nmea_topic = rospy.get_param('~nmea_topic', 'nmea')

        self.ntrip_server = rospy.get_param('~ntrip_server')
        self.ntrip_user = rospy.get_param('~ntrip_user')
        self.ntrip_pass = rospy.get_param('~ntrip_pass')
        self.ntrip_stream = rospy.get_param('~ntrip_stream')
        self.nmea_gga = rospy.get_param('~nmea_gga')

        self.pub = rospy.Publisher(self.rtcm_topic, RTCM, queue_size=10)

        self.connection = None
        self.connection = ntripconnect(self)
        self.connection.start()

    def run(self):
        rospy.spin()
        if self.connection is not None:
            self.connection.stop = True

if __name__ == '__main__':
    c = ntripclient()
    c.run()

