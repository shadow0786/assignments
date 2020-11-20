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
        self.img1_object_data = rospy.Subscriber("/cv_image1/objects", Float64MultiArray, self.callbackIMG1)
        self.target_pos_x = rospy.Publisher("/target/pos_x",Float64,queue_size = 10)
        self.target_pos_y = rospy.Publisher("/target/pos_y",Float64,queue_size = 10)
        self.target_pos_z = rospy.Publisher("/target/pos_z",Float64,queue_size = 10)
        self.robot_Sinjoint2 = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size = 10)
        self.robot_Sinjoint3 = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size = 10)
        self.robot_Sinjoint4 = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size = 10)
        self.robot_joint2 = rospy.Publisher("/robot/joint2_angle_controller/command", Float64, queue_size = 10)
        self.robot_joint3 = rospy.Publisher("/robot/joint3_angle_controller/command", Float64, queue_size = 10)
        self.robot_joint4 = rospy.Publisher("/robot/joint4_angle_controller/command", Float64, queue_size = 10)

        self.time = rospy.get_time()
        
    def callbackIMG1(self,data):
        objects_coordinates = np.array(data.data)
        self.yellow_img1_pos =  np.array([objects_coordinates[0],objects_coordinates[1]])
        self.blue_img1_pos = np.array([objects_coordinates[2],objects_coordinates[3]])
        self.green_img1_pos = np.array([objects_coordinates[4],objects_coordinates[5]])
        self.red_img1_pos = np.array([objects_coordinates[6],objects_coordinates[7]])
        self.targetS_img1_pos = np.array([objects_coordinates[8],objects_coordinates[9]])
    #     self.targetR_img1_pos = np.array([objects_coordinates[10],objects_coordinates[11]]) 


    # Recieve data, process it, and publish
    def callback2(self,data):
        # Recieve the image
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self.yellow_pos_IMG2 = self.find_yellow(self.cv_image2)
        self.blue_pos_IMG2 = self.find_blue(self.cv_image2)
        self.green_pos_IMG2 = self.find_green(self.cv_image2)
        self.red_pos_IMG2 = self.find_red(self.cv_image2)
        templateS = cv2.imread("/home/amami/Project/catkin_ws/src/ivr_assignment/src/image_crop.png", 0 )
        self.orange_target_ = self.target_3D_location(self.cv_image2, templateS)
        self.combined_posIMG12 = self.pos_3D_plane()
        
        self.Sinjoint2=Float64()
        self.Sinjoint2.data= self.getSinJoints()[0]
        self.Sinjoint3=Float64()
        self.Sinjoint3.data= self.getSinJoints()[1]
        self.Sinjoint4=Float64()
        self.Sinjoint4.data= self.getSinJoints()[2]
        self.joint2=Float64()
        self.joint2.data= self.get_joint_angles(self.pos_3D_plane)[0]
        self.joint3=Float64()
        self.joint3.data= self.get_joint_angles(self.pos_3D_plane)[1]
        self.joint4=Float64()
        self.joint4.data= self.get_joint_angles(self.pos_3D_plane)[2]
        self.target = Float64()
        self.target= self.orange_target_
        self.target_x = Float64() 
        self.target_y = Float64()
        self.target_z = Float64()
        self.target_x= self.orange_target_[0]  
        self.target_y= self.orange_target_[1]
        self.target_z= self.orange_target_[2]


        # Uncomment if you want to save the image
        #cv2.imwrite('image_copy.png', cv_image)
        im2=cv2.imshow('window2', self.cv_image2)
        cv2.waitKey(1)

        # Publish the results
        try: 
            self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
            self.robot_Sinjoint2.publish(self.Sinjoint2)
            self.robot_Sinjoint3.publish(self.Sinjoint3)
            self.robot_Sinjoint4.publish(self.Sinjoint4)
            self.robot_joint2.publish(self.joint2)
            self.robot_joint3.publish(self.joint3)
            self.robot_joint4.publish(self.joint4)
            self.target_pos_x.publish(self.target_x)
            self.target_pos_x.publish(self.target_y)
            self.target_pos_x.publish(self.target_z)

        except CvBridgeError as e:
            print(e)
        
    def pos_3D_plane(self):

        meter = self.pixTometer() 
        yellow_pos_3D = np.array([(self.yellow_pos_IMG2[0] - self.yellow_pos_IMG2[0]) * meter,(self.yellow_img1_pos[0] -self.yellow_img1_pos[0]) *meter,((self.yellow_img1_pos[1]  - self.yellow_img1_pos[1]) *-1) *meter])
        blue_pos_3D = np.array([(self.blue_pos_IMG2[0] - self.yellow_pos_IMG2[0]) * meter,(self.blue_img1_pos[0] -self.yellow_img1_pos[0]) *meter,((self.blue_img1_pos[1]  - self.yellow_img1_pos[1]) *-1) *meter])
        green_pos_3D = np.array([(self.green_pos_IMG2[0] - self.yellow_pos_IMG2[0]) * meter,(self.green_img1_pos[0] -self.yellow_img1_pos[0]) *meter,((self.green_img1_pos[1]  - self.yellow_img1_pos[1]) *-1) *meter])
        red_pos_3D = np.array([(self.red_pos_IMG2[0] -self.yellow_pos_IMG2[0]) * meter,(self.red_img1_pos[0] -self.yellow_img1_pos[0]) *meter,((self.red_img1_pos[1]  - self.yellow_img1_pos[1]) *-1) *meter])
        
        if (self.green_pos_IMG2[0] == -10):
            if (self.blue_pos_IMG2[0] == -10):
                green_pos_3D = np.array([0, (self.green_img1_pos[0] - self.yellow_img1_pos[0])  * meter, ((self.green_img1_pos[1] - self.yellow_img1_pos[1])*-1) * meter])
            if (self.blue_pos_IMG2[0] != -10):
                if (self.red_pos_IMG2[0] == -10):
                    green_pos_3D = np.array([0, (self.green_img1_pos[0] - self.yellow_img1_pos[0])  * meter, ((self.green_img1_pos[1] - self.yellow_img1_pos[1])*-1) * meter])
                if (self.red_pos_IMG2[0] != -10):
                    if (self.red_img1_pos[1] - self.yellow_img1_pos[1] > 0):
                        green_pos_3D = np.array([0, (self.green_img1_pos[0] - self.yellow_img1_pos[0])  * meter, ((self.green_img1_pos[1] - self.yellow_img1_pos[1])*-1) * meter])
                    if (self.red_img1_pos[1] - self.yellow_img1_pos[1] < 0):
                        green_pos_3D = np.array([0 , (self.red_img1_pos[0] - self.yellow_img1_pos[0]) *meter , ((self.red_img1_pos[1] - self.yellow_img1_pos[1])*-1) * meter])
        
        if (self.red_pos_IMG2[0] == -10):  
            red_pos_3D = np.array([(self.green_img1_pos[0] - self.yellow_img1_pos[0]) * meter, (self.red_img1_pos[0] - self.yellow_img1_pos[0]) *meter , ((self.red_img1_pos[1] - self.yellow_img1_pos[1])*-1) * meter])
        
        if (self.green_img1_pos[0] == -10):
            green_pos_3D = np.array([(self.green_pos_IMG2[0] - self.yellow_pos_IMG2[0]) * meter, 0 , ((self.green_pos_IMG2[1] - self.yellow_pos_IMG2[1])*-1) * meter])
        if (self.red_img1_pos[0] == -10):
            red_pos_3D = np.array([(self.red_pos_IMG2[0] - self.yellow_pos_IMG2[0]) * meter, 0 , ((self.red_pos_IMG2[1] - self.yellow_pos_IMG2[1])*-1) * meter])
        
        return np.array([yellow_pos_3D , blue_pos_3D , green_pos_3D , red_pos_3D])
    
    def rotation_matrix_y(self, angle):
        R_y = np.array([[np.cos(angle),0,-np.sin(angle)],
                            [0,1,0],
                            [np.sin(angle), 0, np.cos(angle)]])
        return R_y

    def rotation_matrix_x(self, angle):
        R_x = np.array([ [1, 0, 0],
                        [0, np.cos(angle), -np.sin(angle)],
                        [0, np.sin(angle), np.cos(angle)]])
        return R_x
    
    def get_joint_angles(self, pos_3D_plane):
        [yellow, blue, green, red] = pos_3D_plane
        link2 = green - blue

        ### start with joint 2 since joint 1 does not rotate
        angle2 = np.arctan2(green[2] - blue[2], green[1] - blue[1])     

        ####### transform the coordinates into the rotated space
        rotation_matrix_2 = self.rotation_matrix_x(-angle2 + np.pi/2)
        yellow2 = np.dot(rotation_matrix_2,yellow)
        blue2 = np.dot(rotation_matrix_2, blue)
        green2 = np.dot(rotation_matrix_2, green)
        red2 = np.dot(rotation_matrix_2, red)

        ########## calculate joint angle 3 in the new rotated space
        angle3 = np.arctan2(green2[2] - blue2[2], green2[0] - blue2[0])  - np.pi/2   
        
        angle3_temp = np.arctan2(green[2] - blue[2], green[0] - blue[0])
        rotation_matrix = self.rotation_matrix_y(-angle3_temp + np.pi/2)
        yellow1 = np.dot(rotation_matrix,yellow)
        blue1 = np.dot(rotation_matrix, blue)
        green1 = np.dot(rotation_matrix, green)
        red1 = np.dot(rotation_matrix, red)
        angle2 = np.arctan2(green1[2] - blue1[2], green1[1] - blue1[1]) - np.pi/4

        ####### transform the coordinates into the rotated space
        rotation_matrix_3 = self.rotation_matrix_y(-angle3)
        yellow3 = np.dot(rotation_matrix_3,yellow)
        blue3 = np.dot(rotation_matrix_3, blue)
        green3 = np.dot(rotation_matrix_3, green)
        red3 = np.dot(rotation_matrix_3, red)
        
        rotation_matrix_4 = self.rotation_matrix_x(-angle2)  # 
        yellow4 = np.dot(rotation_matrix_4,yellow3)
        blue4 = np.dot(rotation_matrix_4, blue3)
        green4 = np.dot(rotation_matrix_4, green3)
        red4 = np.dot(rotation_matrix_4, red3)

        ########## calculate joint angle 3 in the new rotated space
        angle4 = np.arctan2(red4[2] - green4[2], red4[1] - green4[1])    


        return [angle2, -angle3 , angle4]
        # Solve using trigonometry
    # 
        
    def getSinJoints(self):
        cur_time = np.array([rospy.get_time()]) - self.time
        self.joint2 = float(np.pi/2 * np.sin(np.pi/15 * cur_time)) 
        self.joint3 = float(np.pi/2 * np.sin(np.pi/18 * cur_time))
        self.joint4 = float(np.pi/2 * np.sin(np.pi/20 * cur_time))
        return np.array([self.joint2, self.joint3,self.joint4])  
    
    
    def locate_target_sphere(self, image1, templateS):
        maskOrange = self.find_orange(image1)
        templateSphere = cv2.imread(templateS, 0)
        matching = cv2.matchTemplate(maskOrange, templateSphere, 0)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(matching)
        if (min_loc[1] == 0):
            return np.array([-10,-10])
        x = min_loc[0] 
        y = min_loc[1]
        return np.array([x,y])
    
    def target_3D_location(self , image1 , template):

        cam2 = self.locate_target_sphere(image1, template)

        sphere_location = np.array([cam2[0] - self.yellow_pos_IMG2[0], self.targetS_img1_pos[0] - self.yellow_img1_pos[0], ( cam2[1] - self.yellow_img1_pos[1])*-1]) * self.pixTometer()
    
        if (cam2[1] == -10):
            target_z = self.targetS_img1_pos[1]
        if (self.targetS_img1_pos[1] == -10):
            target_z =cam2[1]

        
        if target_z == cam2[1] : 
            sphere_location = np.array([cam2[0] - self.yellow_pos_IMG2[0], self.targetS_img1_pos[0] - self.yellow_img1_pos[0], (target_z  - self.yellow_pos_IMG2[1])*-1]) * self.pixTometer()
        if target_z == self.targetS_img1_pos[1] :
            sphere_location = np.array([cam2[0] - self.yellow_pos_IMG2[0], self.targetS_img1_pos[0] - self.yellow_img1_pos[0], (target_z  - self.yellow_img1_pos[1])*-1]) * self.pixTometer()
        
        return sphere_location
        
        
        

    #   def locate_target_rectangle(self, image1, template):
    #     matching = cv2.matchTemplate(image1, template, 0)
    #     min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(matching)
    #     x = min_loc[0] 
    #     y = min_loc[1]
    #     return np.array([x,y])

    #   def locate_targets(self, image1, templateS , templateR):
    #     maskOrange = self.find_orange(image1)
    #     templateSphere = cv2.imread(templateS, 0)
    #     templateRectangle = cv2.imread(templateR, 0)
    #     targetSphere = self.locate_target_sphere(maskOrange, templateSphere)
    #     targetRectangle = self.locate_target_rectangle(maskOrange, templateRectangle)
    #     return np.array([targetSphere , targetRectangle])

    def find_yellow(self , image):
        mask = cv2.inRange(image , (0,100,100) , (0,255,255))
        kernel = np.ones((5,5) , np.uint8)
        NewMask = cv2.dilate(mask , kernel , iterations = 3)
        Mo = cv2.moments(NewMask)
        if (Mo['m00'] == 0):
            cx = -10
            cy = -10
            return np.array([cx,cy])
        cx = int(Mo['m10'] / Mo['m00'])
        cy = int(Mo['m01'] / Mo['m00'])
        return np.array([cx,cy])

    def find_red(self , image):
        mask = cv2.inRange(image , (0,0,100) , (0,0,255))
        kernel = np.ones((5,5) , np.uint8)
        NewMask = cv2.dilate(mask , kernel , iterations = 3)
        Mo = cv2.moments(NewMask)
        if (Mo['m00'] == 0):
            cx = -10
            cy = -10
            return np.array([cx,cy])
        cx = int(Mo['m10'] / Mo['m00'])
        cy = int(Mo['m01'] / Mo['m00'])
        return np.array([cx,cy])

    def find_green(self , image):
        mask = cv2.inRange(image , (0,100,0) , (0,255,0))
        kernel = np.ones((5,5) , np.uint8)
        NewMask = cv2.dilate(mask , kernel , iterations = 3)
        Mo = cv2.moments(NewMask)
        if (Mo['m00'] == 0):
            cx = -10
            cy = -10
            return np.array([cx,cy])
        cx = int(Mo['m10'] / Mo['m00'])
        cy = int(Mo['m01'] / Mo['m00'])
        return np.array([cx,cy])

    def find_blue(self , image):
        mask = cv2.inRange(image , (100,0,0) , (255,0,0))
        kernel = np.ones((5,5) , np.uint8)
        NewMask = cv2.dilate(mask , kernel , iterations = 3)
        Mo = cv2.moments(NewMask)
        if (Mo['m00'] == 0):
            cx = -10
            cy = -10
            return np.array([cx,cy])
        cx = int(Mo['m10'] / Mo['m00'])
        cy = int(Mo['m01'] / Mo['m00'])
        return np.array([cx,cy])

    def find_orange(self , image):
        image_hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        mask  = cv2.inRange(image_hsv  ,(9,100,100) , (29,255,255))
        return mask
    
    def pixTometer(self):
        distance = 2.5/(self.blue_pos_IMG2[1] - self.yellow_pos_IMG2[1])
        return np.abs(distance)

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


