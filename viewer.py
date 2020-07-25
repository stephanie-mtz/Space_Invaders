#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
#check world debug
import sys
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import pygame
from pygame.locals import *
import cv2
import gym
import random

video_size = 700, 500
velocity_publisher = rospy.Publisher('space_invader/move', String, queue_size=10)

def key_action():
    numberList = ["1","2","5"]
    f = open("/home/stephanie/catkin_ws/src/space_invader/archivo", "a")
    vel_msg = Twist()
    keys=pygame.key.get_pressed()
    # if keys[K_LEFT]:
        # return "5"
    # if keys[K_UP]:
        # return "1"
    # if keys[K_RIGHT]:
        # return "2"
    rand=random.choice(numberList)
    f.write(rand+'\n')
    f.close()
    return rand



def callback(ros_data):
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    screen = pygame.display.set_mode(video_size)
    surf = pygame.surfarray.make_surface(image_np)
    screen.blit(surf, (0, 0))
    pygame.display.update()
    vel_msg = key_action()
    velocity_publisher.publish(vel_msg)

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('agent', anonymous=True)
    subscriber = rospy.Subscriber('space_invader/image_raw', CompressedImage, callback)
    try:
        screen = pygame.display.set_mode(video_size)
        vel_msg = key_action()
        velocity_publisher.publish(vel_msg)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Gym Image Viewer module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
