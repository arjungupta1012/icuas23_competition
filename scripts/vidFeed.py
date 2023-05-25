#!/usr/bin/env python

import rospy
import time
import ros_numpy
import numpy as np
import cv2
from sensor_msgs.msg import Image
import os


global frame

def callback(data):
    global frame
    img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
    frame = img


rospy.init_node('vid_listener_for_dl', anonymous=False)

rospy.Subscriber("/red/camera/color/image_raw", Image, callback)
time.sleep(2)

i = 123

if __name__ == '__main__':
    global frame
    input()
    while True:
        i+=1
        #print('gekk')
        #cv2.imshow('cam feed', frame)
        #key = cv2.waitKey(1) and 0xFF == ord('q')
        filename = str(i) + ".jpg"
        print(filename)
        cv2.imwrite(os.path.join("/root/uav_ws/src/icuas23_competition/frames_yolo_detection", filename), frame)
        time.sleep(0.5)
    rospy.spin()



