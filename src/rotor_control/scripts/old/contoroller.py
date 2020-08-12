#! /usr/bin/env python


import rospy
import sys
from gazebo_msgs.srv import SetModelState, GetModelState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState

def speed(dst,curr):
    if abs(dst-curr) < 0.1:
        return 0
    elif dst > curr:
        return 0.2
    else:
        return -0.2


def init_twist(x,y,z):
    rospy.wait_for_service('/gazebo/set_model_state')
    rospy.loginfo('Service is ready')
    res = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    curr_pos = res("quadrotor","")
    rospy.loginfo('dest_x is %f and curr_x %f',x, curr_pos.pose.position.x)
    rospy.loginfo('dest_y is %f and curr_y %f',y, curr_pos.pose.position.y)
    rospy.loginfo('dest_z is %f and curr_z %f',z, curr_pos.pose.position.z)

    twist = Twist()
    twist.linear.x = speed (x,curr_pos.pose.position.x)
    twist.linear.y = speed (y,curr_pos.pose.position.y)
    twist.linear.z = speed (z,curr_pos.pose.position.z)


    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

    return twist

def init_pose(x,y,z):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0
    return pose

def init_model_state(x,y,z):
    model_name = "quadrotor"
    pose = init_pose(x,y,z)

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


def setPosition(x,y,z):
    rospy.loginfo('A request for position set has been received ')
    # # Create handle to the reset service
    rospy.loginfo('Waiting for set_model_state service')
    rospy.wait_for_service('/gazebo/set_model_state')
    rospy.loginfo('Service is ready')
    try:
        res = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = init_model_state(x,y,z)
        res(model_state)
        rospy.loginfo('Quadrotor position changed successfully')

    except rospy.ServiceException , e:
        rospy.loginfo('position set failed')

def flyToPos(x,y,z):
    rospy.loginfo('A request for position set has been received ')
    rospy.loginfo('quadrotor taking off ')
    # take_off
    # wait
    rospy.loginfo('quadrotor is headed for destination')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    while not rospy.is_shutdown() :
        twist = init_twist(x,y,z)
        rospy.sleep(0.5)
        pub.publish(twist)

        if twist.linear.x == 0 and twist.linear.y == 0 and twist.linear.z == 0 :
            rospy.loginfo('quadrotor has reached the destination')
            break
        #break

    # land


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    if len(sys.argv) == 5 and (sys.argv[1] == "teleport" or sys.argv[1] == "fly"):
        mode = sys.argv[1]
        x = int(sys.argv[2])
        y = int(sys.argv[3])
        z = int(sys.argv[4])

    else:
        rospy.loginfo('\none or more parameters are invalid. service call failed \n'
                      'The desierd parameters are:\n'
                      'Mode("teleport"/"fly"),position_x(float32),position_y(float32),position_z(float32)')
        sys.exit (1)

    if mode == "teleport":
        setPosition(x,y,z)
    else:
        flyToPos(x,y,z)
