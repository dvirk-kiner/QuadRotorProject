#!/usr/bin/env python


import rospy
import sys
# sys.path.insert(0, './ML/')
from gazebo_msgs.srv import SetModelState, GetModelState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from ML.test import model_test
from os import walk
import os
import multiprocessing
import cv2
from std_msgs.msg import Empty
import time

currFolder = os.path.dirname(os.path.realpath(__file__))
parentFolder = os.path.abspath(os.path.join(currFolder, os.pardir))

class controlQuadrotor():
    def __init__(self, mode='teleport', lowHight=2, highHight=7):
        self.mode = mode
        self.lowHight = lowHight
        self.highHight = highHight
        self.x, self.y, self.z = 14, -16, 5
        self.all_img_processes = []
        rospy.init_node('controller', anonymous=True)
        # TAKEOFF Command - Ask The Other Dvir How To Do It...

    def takeOff(self):
        pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)
	while pub.get_num_connections() == 0:
	    rospy.loginfo("Waiting for subscriber to connect")
	    rospy.sleep(1)
	rospy.loginfo("taking off")
        pub.publish(Empty())


    def land(self):
        pub = rospy.Publisher('/ardrone/land', Empty, queue_size=10)
	while pub.get_num_connections() == 0:
	    rospy.loginfo("Waiting for subscriber to connect")
	    rospy.sleep(1)
	rospy.loginfo("landing")
        pub.publish(Empty())

    def moveDroneToNewLocation(self, newLocation,batteryLevel):
	rospy.loginfo("Battery Level  = " + str(batteryLevel))
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
        path_to_PreTesting = currFolder+"/photos_taken_by_quadrotor/PreTesting"
        path_to_InTesting = currFolder+"/photos_taken_by_quadrotor/InTesting/tmp"
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
    # img_path includes the file name as well

    def img_show(self, img_path, img_class, battery):
        rospy.loginfo("img_class = "+img_class + " img_path = " + img_path)
        time.sleep(1)
        img = cv2.imread(img_path)
        cv2.namedWindow(img_class,cv2.WINDOW_NORMAL)  # Create a named window
        #cv2.resizeWindow(img_class, 400, 400)
        cv2.moveWindow(img_class, 40, 80)  # Move it to (40,30)
        # Write some Text

        font = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (5, 210)
        fontScale = 0.6
        fontColor = (255, 255, 255)
        lineType = 2

        cv2.putText(img, img_class,
                    bottomLeftCornerOfText,
                    font,
                    fontScale,
                    fontColor,
                    lineType)
	cv2.putText(img, "battery - " + str(battery)+"%",
                    (5,240),
                    font,
                    fontScale,
                    fontColor,
                    lineType)


        cv2.imshow(img_class, img)
        cv2.waitKey(0)


    def evaluateImage(self,battery):
        for process in self.all_img_processes:
            process.terminate()
        self.all_img_processes = []
        rospy.loginfo("evalutate image")
       # evaluate image in InTesting folder
        image_class,img_name,full_path_to_img = model_test(list_of_classes=[
                                 "With Helmet", "Without Helmet", "No Worker"])
        img_path_with_img_name = ""

        if image_class == "EMPTY":
            rospy.loginfo("Couldn't evaluate the image - Problem")
        else:
            process = multiprocessing.Process(target=self.img_show,\
                                              args=(currFolder+"/photos_taken_by_quadrotor/Done/"+img_name,image_class,battery))
            process.start()
            self.all_img_processes.append(process)
            rospy.loginfo("The image has classified as: " + image_class)

    def moveDroneHigher(self):
         # move image from InTesting to Done
        path_to_InTesting = currFolder+"/photos_taken_by_quadrotor/InTesting/tmp"
        path_to_Done = currFolder+"/photos_taken_by_quadrotor/Done"
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

    def moveDroneToChargeAndBack(self, battery_location,batteryLevel):
        init_location = (self.x, self.y, self.z)
	battery_location_high = (battery_location[0],battery_location[1],5)
	self.moveDroneToNewLocation(battery_location_high,batteryLevel)
        self.moveDroneToNewLocation(battery_location,batteryLevel)
        rospy.loginfo("charge to 100%")
	self.moveDroneToNewLocation(battery_location_high,batteryLevel)
        self.moveDroneToNewLocation(init_location,100)

    def speed(self, dst, curr):
        if abs(dst-curr) < 0.4:
            return 0
        elif dst > curr:
            return 0.4
        else:
            return -0.4

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
        twist.linear.x = self.speed(x, curr_pos.pose.position.x)
        twist.linear.y = self.speed(y, curr_pos.pose.position.y)
        twist.linear.z = self.speed(z, curr_pos.pose.position.z)

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
