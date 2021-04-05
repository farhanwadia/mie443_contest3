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

bool redFrontier = false;
void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg){
    visualization_msgs::Marker m;
    std_msgs::ColorRGBA color;

    color = msg->markers[0].color;
    if(color.r == 1.0){
        redFrontier = true;
    }
    else{
        redFrontier = false;
    }
}

float posX = 0.0, posY  = 0.0, yaw = 0.0, angular = 0.0, linear = 0.0;
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
}

void update(geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub){    
    // Sets linear and angular velocities, updates main loop timer
    (*pVel).angular.z = angular;
    (*pVel).linear.x = linear;
    (*pVel_pub).publish(*pVel);
}

float dist(float x1, float y1, float x2, float y2){
    //Calculates the Euclidean distance between two points (x1, y1), (x2, y2)
    return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}

void rotateThruAngle(float angleRAD, float angleSpeed, float yawStart, float set_linear, geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub){
    // Rotates turtlebot angleRAD rad CW(-) or CCW(+) depending on angleRAD's sign at angleSpeed rad/s. 
    // Make sure angleRAD is between +/- pi
    // Use set_linear = 0 to rotate in place
    int i = 0;
    while (fabs(yaw - yawStart) <= fabs(angleRAD) && i < 250){
        ros::spinOnce();
        angular = copysign(angleSpeed, angleRAD); //turn angleSpeed rad/s in direction of angleRAD
        linear = set_linear;
        update(pVel, pVel_pub); // publish linear and angular      
        i +=1;
    }
}

int main(int argc, char** argv) {
    //
    // Setup ROS.
    ros::init(argc, argv, "contest3");
    ros::NodeHandle n;
    //
    // Frontier exploration algorithm.
    explore::Explore explore;
    //
    // Class to handle sounds.
    //sound_play::SoundClient sc;
    //
    // The code below shows how to play a sound.
    //std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
    //sc.playWave(path_to_sounds + "sound.wav");
    
    //Publishers to programmable LEDs
    ros::Publisher led1_pub = n.advertise<kobuki_msgs::Led>("mobile_base/commands/led1", 1);
    ros::Publisher led2_pub = n.advertise<kobuki_msgs::Led>("mobile_base/commands/led2", 1);
    kobuki_msgs::Led colour1, colour2;
    const uint8_t BLACK = 0, GREEN = 1, ORANGE = 2, RED = 3;

    //Subscriber to frontier (to check color)
    ros::Subscriber frontier_sub = n.subscribe("contest3/frontiers", 10, &markerCallback);
    
    //Subscribe to odometry
    ros::Subscriber odom = n.subscribe("odom", 1, &odomCallback);

    //Velocity publisher
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel; 

    // The code below shows how to start and stop frontier exploration.
    //explore.stop();
    //explore.start();
    bool exploring = true;
    float oldX = posX, oldY = posY;
    int checkStuckCount = 0;
    while(ros::ok()) {
        // Your code here.
       
        explore.start();

        colour1.value = RED;
        //ROS_INFO("Colour: %d", colour1.value);
        led1_pub.publish(colour1);
        
        
        colour2.value = GREEN;
        //ROS_INFO("Colour: %d", colour2.value);
        led2_pub.publish(colour2);

        
        if(redFrontier && exploring){
            ROS_INFO("FRONTIER RED! SPIN 360!");
            rotateThruAngle(M_PI - 0.01, M_PI/2, yaw, 0.0, &vel, &vel_pub);
            rotateThruAngle(M_PI - 0.01, M_PI/2, yaw, 0.0, &vel, &vel_pub);
        }

        if (dist(oldX, oldY, posX, posY) < 0.5 && exploring){
            checkStuckCount ++;
        }
        else{
            ROS_INFO("Distance: %.1f. Resetting count", dist(oldX, oldY, posX, posY));
            checkStuckCount = 0;
            oldX = posX, oldY = posY;
        }

        if (checkStuckCount >= 750 && exploring){
            rotateThruAngle(M_PI - 0.01, M_PI/2, yaw, 0.0, &vel, &vel_pub);
            rotateThruAngle(M_PI - 0.01, M_PI/2, yaw, 0.0, &vel, &vel_pub);
            checkStuckCount = 0;
            oldX = posX, oldY = posY;
        }
        ROS_INFO("Stuck count: %d", checkStuckCount);



        
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    return 0;
}
