#include <navigation.h>

//bool cancelGoals = false;
void feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback){
    //std::cout << "In feedback callback \n";
    ros::spinOnce();
    if(emotionDetected != -1){
        std::cout << "Emotion detected while in move_base! \n";
        //cancelGoals = true;
    }
}

bool Navigation::moveToGoal(float xGoal, float yGoal, float phiGoal, float timeout){
	// Set up and wait for actionClient.
    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
	// Set goal.
    geometry_msgs::Quaternion phi = tf::createQuaternionMsgFromYaw(phiGoal);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x =  xGoal;
    goal.target_pose.pose.position.y =  yGoal;
    goal.target_pose.pose.position.z =  0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = phi.z;
    goal.target_pose.pose.orientation.w = phi.w;
    ROS_INFO("Sending goal location ...");
	// Send goal and wait for response.
    ac.sendGoal(goal, MoveBaseClient::SimpleDoneCallback(), MoveBaseClient::SimpleActiveCallback(), &feedbackCb);
    //ac.sendGoal(goal);
    ac.waitForResult(ros::Duration(timeout));
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("You have reached the destination");
        return true;
    } else {
        ROS_INFO("The robot failed to reach the destination");
        return false;
    }
}