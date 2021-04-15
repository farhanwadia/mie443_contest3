#ifndef CONTEST3
#define CONTEST3

#include <ros/ros.h>
#include <ros/package.h>
#include "explore.h"
//
// If you cannot find the sound play library try the following command.
// sudo apt install ros-kinetic-sound-play
#include <sound_play/sound_play.h>
#include <ros/console.h>
#include <kobuki_msgs/Led.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/GetPlan.h>
#include <robot_pose.h>
#include <navigation.h>
#include <std_msgs/Int32.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180./M_PI)
#define DEG2RAD(deg) ((deg)*M_PI/180.)

float dist(float x1, float y1, float x2, float y2);

void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);

geometry_msgs::Point getCentroid(std::vector<geometry_msgs::Point> points);

geometry_msgs::Point getClosestPoint(std::vector<geometry_msgs::Point> points, RobotPose robotPose);

geometry_msgs::Point getFurthestPoint(std::vector<geometry_msgs::Point> points, RobotPose robotPose);

std::vector<int> orderCentroidIndices(std::vector<std::vector<geometry_msgs::Point>> frontiers, RobotPose robotPose);

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

void emotionCallback(const std_msgs::Int32::ConstPtr& msg);

void update(geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub, uint64_t* pSecondsElapsed,
            const std::chrono::time_point<std::chrono::system_clock> start);

template <class T>
T randBetween(T a, T b);

bool anyBumperPressed();

void moveThruDistance(float desired_dist, float move_speed, float startX, float startY, geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub,
                    uint64_t* pSecondsElapsed, const std::chrono::time_point<std::chrono::system_clock> start);

void rotateThruAngle(float angleRAD, float angleSpeed, float yawStart, float set_linear, geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub, 
                     uint64_t* pSecondsElapsed, const std::chrono::time_point<std::chrono::system_clock> start);

float chooseAngular(float laserSideSumThreshold, float probSpinToLarger);

void bumperPressedAction(geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub, uint64_t* pSecondsElapsed, 
                         const std::chrono::time_point<std::chrono::system_clock> start);

bool checkPlan(ros::NodeHandle& nh, float xStart, float yStart, float phiStart, float xGoal, float yGoal, float phiGoal);

bool navigateNearby(geometry_msgs::Point startPoint, std::vector<float> radii, std::vector<float> angles, ros::NodeHandle& n, RobotPose robotPose);

void robotReaction();

int main(int argc, char** argv);

#endif