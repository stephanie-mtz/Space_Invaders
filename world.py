#!/usr/bin/env python
# license removed for brevity
import rospy
import gym
import cv2
import numpy as np
import imutils
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
#check world debug

env = gym.make('SpaceInvaders-v0')
image_pub = rospy.Publisher('space_invader/image_raw', CompressedImage, queue_size=10)

def pub_image(env):
    #GYM RENDER AS IMAGE
    img = env.render(mode='rgb_array')
    # ROTATE THE IMAGE THE MATRIX IS 90 grates and mirror
    img = np.flipud(np.rot90(img))
    image_np = imutils.resize(img, width=500)
    # Publish new image
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    compressed_images = cv2.imencode('.jpg', image_np)
    msg.data = np.array(compressed_images[1]).tostring()
    image_pub.publish(msg)

def open_world(vel_msg):
    action = int(vel_msg.data)
    obs, rew, done, info = env.step(action)
    pub_image(env)

if __name__ == '__main__':
    rospy.init_node('space_invader_world', anonymous=True)
    try:
        rospy.Subscriber("space_invader/move", String, open_world)
        env.reset()
        pub_image(env)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

