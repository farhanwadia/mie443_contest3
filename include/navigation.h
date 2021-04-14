#pragma once
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
//#include <chrono>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

extern int emotionDetected;
extern bool allFrontiersRed;

void feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback);

class Navigation {
	public:
		static bool moveToGoal(float xGoal, float yGoal, float phiGoal, float timeout);
};