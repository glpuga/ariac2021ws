/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// ros
#include "actionlib_msgs/GoalID.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "ros/ros.h"

// standard library
#include <iostream>
#include <string>

class InfiniteToleranceNode
{
public:
  InfiniteToleranceNode(const ros::NodeHandle nh, const std::string& controller_facing_server,
                        const std::string& client_facing_server)
    : nh_{ nh }
  {
    const int queue_len = 10;

    goal_sub_ = nh_.subscribe(client_facing_server + "/goal", queue_len,
                              &InfiniteToleranceNode::goalCallback, this);
    status_sub_ = nh_.subscribe(controller_facing_server + "/status", queue_len,
                                &InfiniteToleranceNode::statusCallback, this);
    feedback_sub_ = nh_.subscribe(controller_facing_server + "/feedback", queue_len,
                                  &InfiniteToleranceNode::feedbackCallback, this);
    result_sub_ = nh_.subscribe(controller_facing_server + "/result", queue_len,
                                &InfiniteToleranceNode::resultCallback, this);
    cancel_sub_ = nh_.subscribe(client_facing_server + "/cancel", queue_len,
                                &InfiniteToleranceNode::cancelCallback, this);

    goal_pub_ = nh_.advertise<control_msgs::FollowJointTrajectoryActionGoal>(
        controller_facing_server + "/goal", queue_len);
    status_pub_ =
        nh_.advertise<actionlib_msgs::GoalStatusArray>(client_facing_server + "/status", queue_len);
    feedback_pub_ = nh_.advertise<control_msgs::FollowJointTrajectoryActionFeedback>(
        client_facing_server + "/feedback", queue_len);
    result_pub_ = nh_.advertise<control_msgs::FollowJointTrajectoryActionResult>(
        client_facing_server + "/result", queue_len);
    cancel_pub_ =
        nh_.advertise<actionlib_msgs::GoalID>(controller_facing_server + "/cancel", queue_len);
  }

  void run()
  {
    ros::spin();
  }

private:
  ros::NodeHandle nh_;

  ros::Publisher goal_pub_;
  ros::Publisher status_pub_;
  ros::Publisher cancel_pub_;
  ros::Publisher feedback_pub_;
  ros::Publisher result_pub_;

  ros::Subscriber goal_sub_;
  ros::Subscriber status_sub_;
  ros::Subscriber cancel_sub_;
  ros::Subscriber feedback_sub_;
  ros::Subscriber result_sub_;

  void goalCallback(control_msgs::FollowJointTrajectoryActionGoal::ConstPtr input_msg)
  {
    auto output_msg = *input_msg;

    auto& goal = output_msg.goal;

    for (const auto& name : goal.trajectory.joint_names)
    {
      {
        control_msgs::JointTolerance joint_tolerance;
        joint_tolerance.name = name;
        joint_tolerance.position = -1.0;
        joint_tolerance.velocity = -1.0;
        joint_tolerance.acceleration = -1.0;
        goal.path_tolerance.push_back(joint_tolerance);
      }
      {
        control_msgs::JointTolerance joint_tolerance;
        joint_tolerance.name = name;
        joint_tolerance.position = -1.0;
        joint_tolerance.velocity = -1.0;
        joint_tolerance.acceleration = -1.0;
        goal.goal_tolerance.push_back(joint_tolerance);
      }
    }

    goal.goal_time_tolerance = ros::Duration(5.0);

    goal_pub_.publish(output_msg);
  }

  void statusCallback(actionlib_msgs::GoalStatusArray::ConstPtr msg)
  {
    status_pub_.publish(*msg);
  }

  void feedbackCallback(control_msgs::FollowJointTrajectoryActionFeedback::ConstPtr msg)
  {
    feedback_pub_.publish(*msg);
  }

  void resultCallback(control_msgs::FollowJointTrajectoryActionResult::ConstPtr msg)
  {
    result_pub_.publish(*msg);
  }

  void cancelCallback(actionlib_msgs::GoalID::ConstPtr msg)
  {
    cancel_pub_.publish(*msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "infinite_tolerance_node");
  ros::NodeHandle nh{ "~" };

  std::string controller_facing_server;
  std::string client_facing_server;

  ros::param::param<std::string>("~controller_facing_server", controller_facing_server,
                                 "controller_server");
  ros::param::param<std::string>("~client_facing_server", client_facing_server, "client_client");
  InfiniteToleranceNode node{ nh, controller_facing_server, client_facing_server };
  node.run();
  return 0;
}
