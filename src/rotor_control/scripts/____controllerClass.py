#!/usr/bin/env python


import rospy
import sys
#sys.path.insert(0, './ML/')
from gazebo_msgs.srv import SetModelState, GetModelState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from ML.test import model_test
from os import walk
import os
import cv2
import math

class controlQuadrotor():
    def __init__(self, mode='teleport', lowHight=2, highHight=7):
        self.mode = mode
        self.lowHight = lowHight
        self.highHight = highHight
        self.x, self.y, self.z = 14, -16, 0
        rospy.init_node('controller', anonymous=True)
        # TAKEOFF Command - Ask The Other Dvir How To Do It...

    def moveDroneToNewLocation(self, newLocation):
        x, y, z = newLocation
        if self.mode == "teleport":
            self.setPosition(x, y, z)
        else:
            self.flyToPos(x, y, z)
        self.x = x
        self.y = y
        self.z = z

    def moveDroneLower(self):
        # move random image from PreTesting to InTesting
        path_to_PreTesting = "/home/dvir/catkin_new/src/rotor_control/scripts/photos_taken_by_quadrotor/PreTesting"
        path_to_InTesting = "/home/dvir/catkin_new/src/rotor_control/scripts/photos_taken_by_quadrotor/InTesting/tmp"
        image_path, image_new_path = "", ""
	rospy.loginfo("Path to Pretesting = " + path_to_PreTesting)
        for (root, _, filename) in walk(path_to_PreTesting):
	    for f in filename:
                if f.endswith('.png'):
                    image_path = os.path.join(root, f)
                    image_new_path = os.path.join(path_to_InTesting, f)
                    os.rename(image_path, image_new_path)
                    rospy.loginfo("Moved Image -> " + f)
                    break
	    if image_path != "":
                break
        if image_path == "":
            rospy.loginfo("Couldn't get the image from the folder")

        if self.mode == "teleport":
            self.setPosition(self.x, self.y, self.lowHight)
        else:
            self.flyToPos(self.x, self.y, self.lowHight)
        self.z = self.lowHight

    def evaluateImage(self):
	rospy.loginfo("evalutate image")
       # evaluate image in InTesting folder
        image_class,img_name = model_test()
        path_to_InTesting = "/home/dvir/catkin_new/src/rotor_control/scripts/photos_taken_by_quadrotor/InTesting/tmp/"
	path_to_img = path_to_InTesting+img_name
        rospy.loginfo(path_to_img)
	 
        if image_class == "EMPTY" :
            rospy.loginfo("Couldn't evaluate the image - Problem")
        else:
            rospy.loginfo("The image has classified as: {image_class}")
	    img = cv2.imread(path_to_img)
            cv2.imshow(image_class,img)
            cv2.waitKey(5000)
            cv2.destroyAllWindows()

    def moveDroneHigher(self):
         # move image from InTesting to Done
        path_to_InTesting = "/home/dvir/catkin_new/src/rotor_control/scripts/photos_taken_by_quadrotor/InTesting/tmp"
        path_to_Done = "/home/dvir/catkin_new/src/rotor_control/scripts/photos_taken_by_quadrotor/Done"
        image_path, image_new_path = "", ""
        for (root, _, filename) in walk(path_to_InTesting):
            for f in filename:
                if f.endswith('.png'):
                    image_path = os.path.join(root, f)
                    image_new_path = os.path.join(path_to_Done, f)
                    os.rename(image_path, image_new_path)
                    rospy.loginfo("Moved Image -> " + f)
		    break
	    if image_path != "":
                break
        if image_path == "":
            rospy.loginfo("Don't have any image inside the InTesting")

        if self.mode == "teleport":
            self.setPosition(self.x, self.y, self.highHight)
        else:
            self.flyToPos(self.x, self.y, self.highHight)
        self.z = self.highHight

    def speed(self, dst, curr,vec):
        if abs(dst-curr) < 1:
            return 0

        return 2*((dst-curr)/vec)

    def init_twist(self, x, y, z):
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.loginfo('Service is ready')
        res = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        curr_pos = res("quadrotor", "")
        rospy.loginfo('dest_x is %f and curr_x %f',
                      x, curr_pos.pose.position.x)
        rospy.loginfo('dest_y is %f and curr_y %f',
                      y, curr_pos.pose.position.y)
        rospy.loginfo('dest_z is %f and curr_z %f',
                      z, curr_pos.pose.position.z)

        twist = Twist()
	vec_speed = math.sqrt(pow((x-curr_pos.pose.position.x),2) + pow((y-curr_pos.pose.position.y),2) +pow((z-curr_pos.pose.position.z),2))
        twist.linear.x = self.speed(x, curr_pos.pose.position.x,vec_speed)
        twist.linear.y = self.speed(y, curr_pos.pose.position.y,vec_speed)
        twist.linear.z = self.speed(z, curr_pos.pose.position.z,vec_speed)

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        return twist

    def init_pose(self, x, y, z):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        return pose

    def init_model_state(self, x, y, z):
        model_name = "quadrotor"
        pose = self.init_pose(x, y, z)

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        ref_frame = ""

        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = pose
        model_state.twist = twist
        model_state.reference_frame = ref_frame

        return model_state

    def setPosition(self, x, y, z):
        rospy.loginfo('A request for position set has been received ')
        # # Create handle to the reset service
        rospy.loginfo('Waiting for set_model_state service')
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.loginfo('Service is ready')
        try:
            res = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            model_state = self.init_model_state(x, y, z)
            res(model_state)
            rospy.loginfo('Quadrotor position changed successfully')

        except rospy.ServiceException:
            rospy.loginfo('position set failed')

    def flyToPos(self, x, y, z):
        rospy.loginfo('A request for position set has been received ')
        rospy.loginfo('quadrotor taking off ')
        # take_off
        # wait
        rospy.loginfo('quadrotor is headed for destination')
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        while not rospy.is_shutdown():
            twist = self.init_twist(x, y, z)
            rospy.sleep(0.5)
            pub.publish(twist)

            if twist.linear.x == 0 and twist.linear.y == 0 and twist.linear.z == 0:
                rospy.loginfo('quadrotor has reached the destination')
                break
            # break

        # land
