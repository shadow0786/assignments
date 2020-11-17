#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    self.yellow_img1_pos = np.array([0,0])
    self.blue_img1_pos = np.array([0,0])
    self.green_img1_pos = np.array([0,0])
    self.red_img1_pos = np.array([0,0])
    self.targetS_img1_pos = np.array([0,0])
    self.targetR_img1_pos = np.array([0,0])
    self.img1_object_data = rospy.Subscriber("/image1/objects", Float64MultiArray, self.callbackIMG1)
    self.robot_Sinjoint2 = rospy.Publisher("/robot/Sinjoint2_position_controller/command", Float64, queue_size = 10)
    self.robot_Sinjoint3 = rospy.Publisher("/robot/Sinjoint3_position_controller/command", Float64, queue_size = 10)
    self.robot_Sinjoint4 = rospy.Publisher("/robot/Sinjoint4_position_controller/command", Float64, queue_size = 10)
    self.time = np.array([rospy.get_time()], dtype='float64')
    
   def callback1(self,data):
    circles_pos1 = np.array(data.data)
    self.yellow_proj_pos1 =  np.array([circles_pos1[0],circles_pos1[1]])
    self.blue_proj_pos1 = np.array([circles_pos1[2],circles_pos1[3]])
    self.green_proj_pos1 = np.array([circles_pos1[4],circles_pos1[5]])
    self.red_proj_pos1 = np.array([circles_pos1[6],circles_pos1[7]])
    self.target_proj_pos1 = np.array([circles_pos1[8],circles_pos1[9]]) 


  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


