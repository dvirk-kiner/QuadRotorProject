#! /usr/bin/env python

import rospy

import actionlib

import rotor_control.msg

class qrotorMoveAction(object):
    # create messages that are used to publish feedback/result
    _feedback = rotor_control.msg.qrotorMoveFeedback()
    _result = rotor_control.msg.qrotorMoveResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rotor_control.msg.qrotorMoveAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # append the seeds for the fibonacci sequence
        self._feedback.dest="start"

        # publish info to the console for the user
        rospy.loginfo('action %s has recieved and handled with goal %s' % (self._action_name, goal.dest))
        
	

        # start executing the action
        for i in range(5):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self._feedback.dest = "feedback_" + str(i)
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            rospy.loginfo('A feedback with value %s was published' % self._feedback.dest )
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            rospy.sleep(3)
          
        if success:
            self._result.dest = self._feedback.dest
            rospy.loginfo('%s: Succeeded python' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('move')
    server = qrotorMoveAction(rospy.get_name())
    rospy.spin()
