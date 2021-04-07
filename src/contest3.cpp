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

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180./M_PI)
#define DEG2RAD(deg) ((deg)*M_PI/180.)

bool redFrontier = false;
void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg){
    visualization_msgs::Marker m;
    std_msgs::ColorRGBA color;
    int numMarkers;

    color = msg->markers[0].color;
    numMarkers = msg->markers.size();
    if(color.r == 1.0){
        redFrontier = true;
    }
    else{
        redFrontier = false;
    }
   
    std::cout << numMarkers << " markers in markerArray" << "\n";
}

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
const uint8_t LEFT = 0, CENTER = 1, RIGHT = 2;
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    bumper[msg->bumper] = msg->state;
}

float posX = 0.0, posY  = 0.0, yaw = 0.0, angular = 0.0, linear = 0.0;
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
}

float omega = 0.0, accX = 0.0, accY = 0.0, yaw_imu = 0.0;
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    omega = msg->angular_velocity.z;
    accX = msg->linear_acceleration.x;
    accY = msg->linear_acceleration.y;
    yaw_imu = tf::getYaw(msg->orientation);
    //ROS_INFO("Acceleration: (%f, %f) \n IMU Yaw: %f deg Omega %f", accX, accY, RAD2DEG(yaw_imu), omega);
}

void update(geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub, uint64_t* pSecondsElapsed,
            const std::chrono::time_point<std::chrono::system_clock> start){    
    // Sets linear and angular velocities, updates main loop timer
    (*pVel).angular.z = angular;
    (*pVel).linear.x = linear;
    (*pVel_pub).publish(*pVel);

    ros::spinOnce();

    // Update the timer.
    *pSecondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
    ros::Duration(0.01).sleep();
}

float dist(float x1, float y1, float x2, float y2){
    //Calculates the Euclidean distance between two points (x1, y1), (x2, y2)
    return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}

template <class T>
T randBetween(T a, T b){
    // Returns a random number between a and b inclusive. Assumes a<b.
    if (std::is_same<T, int>::value){
        float x = float(rand())/float(RAND_MAX);
        return int(round(float(b-a)*x + float(a)));
    }
    else{
        T x = T(float(rand())/float(RAND_MAX));
        return (b-a)*x + a;
    }
}

bool anyBumperPressed(){
    // Returns true if any bumper is pressed, false otherwise
    bool any_bumper_pressed = false;
    for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
        any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
    }
    return any_bumper_pressed;
}

void moveThruDistance(float desired_dist, float move_speed, float startX, float startY, geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub,
                    uint64_t* pSecondsElapsed, const std::chrono::time_point<std::chrono::system_clock> start){
    // Moves turtlebot desired_dist at move_speed. Negative desired_dist moves backwards. Only magnitude of move_speed is used.
    int i = 0;
    float current_dist = dist(startX, startY, posX, posY);
    while (current_dist < fabs(desired_dist) && i < 100 && *pSecondsElapsed < 1200){
        ros::spinOnce();
        angular = 0;
        linear = copysign(fabs(move_speed), desired_dist); //move move_speed m/s in direction of desired_dist
        update(pVel, pVel_pub, pSecondsElapsed, start); // publish linear and angular
        current_dist = dist(startX, startY, posX, posY);
        
        if (anyBumperPressed()){
            break;
        }
        i+=1;
    }
}

void rotateThruAngle(float angleRAD, float angleSpeed, float yawStart, float set_linear, geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub, 
                     uint64_t* pSecondsElapsed, const std::chrono::time_point<std::chrono::system_clock> start){
    // Rotates turtlebot angleRAD rad CW(-) or CCW(+) depending on angleRAD's sign at angleSpeed rad/s. 
    // Make sure angleRAD is between +/- pi
    // Use set_linear = 0 to rotate in place
    int i = 0;
    while (fabs(yaw - yawStart) <= fabs(angleRAD) && i < 500 && *pSecondsElapsed < 1200){
        ros::spinOnce();
        angular = copysign(angleSpeed, angleRAD); //turn angleSpeed rad/s in direction of angleRAD
        linear = set_linear;
        update(pVel, pVel_pub, pSecondsElapsed, start); // publish linear and angular      
        
        if (anyBumperPressed()){
            ROS_INFO("Breaking out of rotate due to bumper press \n Left: %d \n Center: %d \n Right: %d \n", bumper[LEFT], bumper[CENTER], bumper[RIGHT]);
            break;
        }
        /*if(fabs(omega) < 0.035 && i > 250){
            ROS_INFO("Moving forward and braking out. Likely stuck.");
            moveThruDistance(0.1, 0.1, posX, posY, pVel, pVel_pub, pSecondsElapsed, start);
            break;
        }*/
    
        i +=1;
    }
}

void bumperPressedAction(geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub, uint64_t* pSecondsElapsed, 
                         const std::chrono::time_point<std::chrono::system_clock> start){
    bool any_bumper_pressed = true;
    any_bumper_pressed = anyBumperPressed();
    
    if (any_bumper_pressed && *pSecondsElapsed < 900){
        ROS_INFO("Bumper pressed \n Left: %d \n Center: %d \n Right: %d \n", bumper[LEFT], bumper[CENTER], bumper[RIGHT]);
       
        if (bumper[LEFT]){
            ROS_INFO("Left hit. Move back and spin 90 CW");
            moveThruDistance(-0.8, 0.2, posX, posY, pVel, pVel_pub, pSecondsElapsed, start);
            rotateThruAngle(DEG2RAD(-90), M_PI/6, yaw, 0, pVel, pVel_pub, pSecondsElapsed, start);
        }
        else if (bumper[RIGHT]){
            ROS_INFO("Right hit. Move back and spin 90 CCW");
            moveThruDistance(-0.8, 0.2, posX, posY, pVel, pVel_pub, pSecondsElapsed, start);
            rotateThruAngle(DEG2RAD(90), M_PI/6, yaw, 0, pVel, pVel_pub, pSecondsElapsed, start);
        }
        else if (bumper[CENTER]){
            ROS_INFO("Center hit. Move back and spin");
            moveThruDistance(-0.8, 0.2, posX, posY, pVel, pVel_pub, pSecondsElapsed, start);
            rotateThruAngle(M_PI - 0.01, M_PI/6, yaw, 0, pVel, pVel_pub, pSecondsElapsed, start);
        }
    }
}

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest3");
    ros::NodeHandle n;
    
    // Frontier exploration algorithm.
    explore::Explore explore;
    
    // Class to handle sounds.
    //sound_play::SoundClient sc;
    
    // The code below shows how to play a sound.
    //std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
    //sc.playWave(path_to_sounds + "sound.wav");
    
    // Publishers
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel; 

    ros::Publisher led1_pub = n.advertise<kobuki_msgs::Led>("mobile_base/commands/led1", 1);
    ros::Publisher led2_pub = n.advertise<kobuki_msgs::Led>("mobile_base/commands/led2", 1);
    kobuki_msgs::Led colour1, colour2;
    const uint8_t BLACK = 0, GREEN = 1, ORANGE = 2, RED = 3;

    // Subscribers
    ros::Subscriber bumper_sub = n.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    //ros::Subscriber odom = n.subscribe("odom", 1, &odomCallback);
    //ros::Subscriber imu_sub = n.subscribe("mobile_base/sensors/imu_data", 1, &imuCallback);
    ros::Subscriber frontier_sub = n.subscribe("contest3/frontiers", 10, &markerCallback);
    
    bool exploring = true;
    float oldX = posX, oldY = posY;
    int checkStuckCount = 0;

    // Contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    
    while(ros::ok() && secondsElapsed <= 1200) {      
        explore.start();
        //colour1.value = RED;
        //ROS_INFO("Colour: %d", colour1.value);
        //led1_pub.publish(colour1);
        //colour2.value = GREEN;
        //ROS_INFO("Colour: %d", colour2.value);
        //led2_pub.publish(colour2);

        if(anyBumperPressed()){
            bumperPressedAction(&vel, &vel_pub, &secondsElapsed, start); 
        }

        if(exploring && secondsElapsed % 10 == 0){
            moveThruDistance(0.6, 0.2, posX, posY, &vel, &vel_pub, &secondsElapsed, start);
        }

        /*if(exploring && redFrontier){
            ROS_INFO("FRONTIER RED! SPIN 360!");
            rotateThruAngle(M_PI - 0.01, M_PI/2, yaw, 0.0, &vel, &vel_pub, &secondsElapsed, start);
            rotateThruAngle(M_PI - 0.01, M_PI/2, yaw, 0.0, &vel, &vel_pub, &secondsElapsed, start);
            //moveThruDistance(0.6, 0.2, posX, posY, &vel, &vel_pub, &secondsElapsed, start);
        }*/

        /*if (exploring && secondsElapsed % 5 == 0){
            ROS_INFO("RANDOM 45 TURN!");
            explore.stop();
            rotateThruAngle(randBetween(-M_PI/4, M_PI/4), M_PI/6, yaw, randBetween(0.0, 0.2), &vel, &vel_pub, &secondsElapsed, start);
            //moveThruDistance(0.6, 0.2, posX, posY, &vel, &vel_pub, &secondsElapsed, start);
            explore.start();
        }*/


       /* if (dist(oldX, oldY, posX, posY) < 0.5 && exploring){
            checkStuckCount ++;
        }
        else{
            ROS_INFO("Distance: %.1f. Resetting count", dist(oldX, oldY, posX, posY));
            checkStuckCount = 0;
            
        }

        if (checkStuckCount >= 750 && exploring){
            rotateThruAngle(M_PI - 0.01, M_PI/6, yaw, 0.0, &vel, &vel_pub, &secondsElapsed, start);
            rotateThruAngle(M_PI - 0.01, M_PI/6, yaw, 0.0, &vel, &vel_pub, &secondsElapsed, start);
            checkStuckCount = 0;
            
        }
        ROS_INFO("Stuck count: %d", checkStuckCount);

        oldX = posX, oldY = posY;*/

        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    return 0;
}