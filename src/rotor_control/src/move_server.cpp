#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tut/moveAction.h>

class moveAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_tut::moveAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  actionlib_tut::moveFeedback feedback_;
  actionlib_tut::moveResult result_;

public:

  moveAction(std::string name) :
    as_(nh_, name, boost::bind(&moveAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~moveAction(void)
  {
  }

  void executeCB(const actionlib_tut::moveGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.feedback_num = 0;
    // publish info to the console for the user
    ROS_INFO("%s: Executing, loop with goal %i, from inital feedback %i", action_name_.c_str(), goal->goal_num, feedback_.feedback_num);

    // start executing the action
    for(int i=1; i<=goal->goal_num; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
       ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.feedback_num++;
      // publish the feedback
      as_.publishFeedback(feedback_);
      ROS_INFO("A feedback with value %d was published", feedback_.feedback_num);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      ros::Duration(5).sleep();
    }

    if(success)
    {
      result_.res_num = feedback_.feedback_num;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move");

  moveAction move("move");
  ros::spin();

  return 0;
}
