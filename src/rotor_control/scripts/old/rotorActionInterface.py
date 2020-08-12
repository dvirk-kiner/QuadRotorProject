#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys
from gazebo_msgs.srv import SetModelState, GetModelState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState


def callback(data):
    # a message was heard
    doAction()

def moveAction(data):
    #do something
    rospy.loginfo("recieved action moveAction")
    rospy.sleep(1) 
    pub_move.publish("done moveAction")
    rospy.loginfo("send done job")
def lowerDroneAction(data):
    #do something
    rospy.loginfo("recieved action lowerDroneAction")
    rospy.sleep(1) 
    pub_lower.publish("done lowerDroneAction")
    rospy.loginfo("send done job")
def higherDroneAction(data):
    #do something
    rospy.loginfo("recieved action higherDroneAction")
    rospy.sleep(1) 
    pub_higher.publish("done higherDroneAction")
    rospy.loginfo("send done job")
def takepicAction(data):
    #do something
    rospy.loginfo("recieved action takepicAction")
    rospy.sleep(1) 
    pub_takepic.publish("done takepicAction")
    rospy.loginfo("send done job")


if __name__ == '__main__':
	rospy.init_node('rotorActionInterface', anonymous=True)
	pub_move = rospy.Publisher('rosplan_interface_movebetweensquares/jobDone', String, queue_size=10)
	pub_lower = rospy.Publisher('rosplan_interface_lowerdrone/jobDone', String, queue_size=10)
	pub_higher = rospy.Publisher('rosplan_interface_higherdrone/jobDone', String, queue_size=10)
	pub_takepic = rospy.Publisher('rosplan_interface_takepic/jobDone', String, queue_size=10)
	rospy.Subscriber("rosplan_interface_movebetweensquares/jobReq", String, moveAction)
	rospy.Subscriber("rosplan_interface_lowerdrone/jobReq", String, lowerDroneAction)
	rospy.Subscriber("rosplan_interface_higherdrone/jobReq", String, higherDroneAction)
	rospy.Subscriber("rosplan_interface_takepic/jobReq", String, takepicAction)
	rospy.loginfo("initiating rotorActionInterface"); 
	rospy.spin()
	rospy.loginfo("System shut down"); 






