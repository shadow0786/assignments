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
    
   def callbackIMG1(self,data):
    self.objects_coordinates = np.array(data.data)
    self.yellow_img1_pos =  np.array([objects_coordinates[0],objects_coordinates[1]])
    self.blue_img1_pos = np.array([objects_coordinates[2],objects_coordinates[3]])
    self.green_img1_pos = np.array([objects_coordinates[4],objects_coordinates[5]])
    self.red_img1_pos = np.array([objects_coordinates[6],objects_coordinates[7]])
    self.targetS_img1_pos = np.array([objects_coordinates[8],objects_coordinates[9]])
    self.targetR_img1_pos = np.array([objects_coordinates[10],objects_coordinates[11]]) 


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
      
  def locate_target_sphere(self, image1, template):
    matching = cv2.matchTemplate(image1, template, 0)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(matching)
    x = min_loc[0] 
    y = min_loc[1]
    return np.array([x,y])

  def locate_target_rectangle(self, image1, template):
    matching = cv2.matchTemplate(image1, template, 0)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(matching)
    x = min_loc[0] 
    y = min_loc[1]
    return np.array([x,y])

  def locate_targets(self, image1, templateS , templateR):
    maskOrange = self.find_orange(image1)
    templateSphere = cv2.imread(templateS, 0)
    templateRectangle = cv2.imread(templateR, 0)
    targetSphere = self.locate_target_sphere(maskOrange, templateSphere)
    targetRectangle = self.locate_target_rectangle(maskOrange, templateRectangle)
    return np.array([targetSphere , targetRectangle])

  def find_yellow(self , image):
    mask = cv2.inRange(image , (0,100,100) , (0,255,255))
    kernel = np.ones((5,5) , np.unit8)
    NewMask = cv2.dilate(mask , kernel , iteration = 3)
    Mo = cv2.moments(NewMask)
    if Mo['m00' == 0]:
      cx = -10
      cy = -10
      return np.array([cx,cy])
    cx = int(Mo['m10'] / Mo['m00'])
    cy = int(Mo['m01'] / Mo['m00'])
    return np.array([cx,cy])

  def find_red(self , image):
    mask = cv2.inRange(image , (0,0,100) , (0,0,255))
    kernel = np.ones((5,5) , np.unit8)
    NewMask = cv2.dilate(mask , kernel , iteration = 3)
    Mo = cv2.moments(NewMask)
    if Mo['m00' == 0]:
      cx = -10
      cy = -10
      return np.array([cx,cy])
    cx = int(Mo['m10'] / Mo['m00'])
    cy = int(Mo['m01'] / Mo['m00'])
    return np.array([cx,cy])

  def find_green(self , image):
    mask = cv2.inRange(image , (0,100,0) , (0,255,0))
    kernel = np.ones((5,5) , np.unit8)
    NewMask = cv2.dilate(mask , kernel , iteration = 3)
    Mo = cv2.moments(NewMask)
    if Mo['m00' == 0]:
      cx = -10
      cy = -10
      return np.array([cx,cy])
    cx = int(Mo['m10'] / Mo['m00'])
    cy = int(Mo['m01'] / Mo['m00'])
    return np.array([cx,cy])

  def find_blue(self , image):
    mask = cv2.inRange(image , (100,0,0) , (255,0,0))
    kernel = np.ones((5,5) , np.unit8)
    NewMask = cv2.dilate(mask , kernel , iteration = 3)
    Mo = cv2.moments(NewMask)
    if Mo['m00' == 0]:
      cx = -10
      cy = -10
      return np.array([cx,cy])
    cx = int(Mo['m10'] / Mo['m00'])
    cy = int(Mo['m01'] / Mo['m00'])
    return np.array([cx,cy])

  def find_orange(self , image):
    mask  = cv2.inRange(image  ,(50,100,110) , (90,185,220))
    return mask

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


