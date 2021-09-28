#! /usr/bin/env python

import rospy
from time import sleep
import roslib
roslib.load_manifest('enph353_ros_lab')
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import os

import sys

class line_following:


    i = 0
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.follow_line)
        self.previous = 0
        sleep(2)

    def stop(self):
        car = Twist()
        car.linear.x = 0
        car.angular.z = 0

        self.pub.publish(car)

    def follow_line(self, img):
        """
        given an image from the robots camera changes the velocity of the robot
        to follow the on screen line
        """
        line_following.i += 1
        cv_img = self.convert_img(img)
        car = Twist()
        error = self.get_line_position(cv_img)
        if error == None:
            car.linear.x = 0
            car.angular.z = self.previous / 2.0
        elif abs(error) < 50 :
            car.linear.x = 0.2
            car.angular.z = error / 75.0
            try:
                self.previous = error/abs(error)
            except:
                pass
        else:
            car.linear.x = 0.1
            car.angular.z = error / 75.0
            self.previous = error/abs(error)
            print(self.previous)

        if error is None:
            print("no line found")
        elif error > 0:
            print("turning right", error)
        else:
            print("turning left", error)

        self.pub.publish(car)


    def get_line_position(self, img):
        height, width = img.shape[:2]
        #print(img.shape[:2])
        bottom = img[(height-5):height, 0:width]
        #print(bottom.shape[:2])

        color_bottom = cv.cvtColor(bottom, cv.COLOR_RGB2GRAY)
        gray_bottom = cv.cvtColor(color_bottom, cv.COLOR_GRAY2BGR)
        allX = []
        allY = []
        if line_following.i % 1000 == 1:
            cv.imwrite('img' + str(line_following.i) + ".png", color_bottom)
        for y in range(len(img)):
            for x in range(len(img[0])):
                if img[y,x,0] < 100:
                    allX.append(x)
                    allY.append(y)
        try:
            return len(img[0])/2 - int(np.average(allX))
        except:
            return None

    def convert_img(self, img):
        """
        converts a rospy image to a cv image format
        """
        try:
            i = self.bridge.imgmsg_to_cv2(img, "bgr8")
            height, width = i.shape[:2]
            return cv.resize(i,(width / 5, height / 5), interpolation = cv.INTER_CUBIC)
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    os.chdir("/home/fizzer/Documents/ros_driving")
    lf = line_following()
    rospy.init_node('line_following', anonymous=True)
    lf.stop()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Stopping line_following")
