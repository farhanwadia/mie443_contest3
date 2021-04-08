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

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180./M_PI)
#define DEG2RAD(deg) ((deg)*M_PI/180.)

bool redFrontier = false;
void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg){
    visualization_msgs::Marker m;
    std_msgs::ColorRGBA color;
    int numMarkers;
    //color = msg->markers[0].color;
    numMarkers = msg->markers.size();
    
    redFrontier = false;
    for(int i = 0; i < numMarkers; i++){
        color = msg->markers[i].color;
        if (color.r == 1.0){
            redFrontier = true;
            break;
        }
    }   
    std::cout << numMarkers << " markers in markerArray \n";
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
    //ROS_INFO("Odom: (%.2f, %.2f, %.2f)", posX, posY, yaw);
}

float minLaserDist = std::numeric_limits<float>::infinity(), minLSLaserDist = std::numeric_limits<float>::infinity(), minRSLaserDist = std::numeric_limits<float>::infinity();
float LSLaserSum = 0, RSLaserSum = 0;
int32_t nLasers=0, desiredNLasers=0, desiredAngle=22.5; 
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	// Calculates minLaserDist overall, per side as minLSLaserDist and minRSLaserDist, 
    // and the sum of laser measurements on each side LSLaserSum and RSLaserSum
    
    minLaserDist = std::numeric_limits<float>::infinity();
    minLSLaserDist = std::numeric_limits<float>::infinity();
    minRSLaserDist = std::numeric_limits<float>::infinity();
    LSLaserSum = 0, RSLaserSum = 0;
    float maxLaserThreshold = 7;
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;

    int start = 0, end = nLasers;
    if (DEG2RAD(desiredAngle) < msg->angle_max && DEG2RAD(-desiredAngle) > msg->angle_min){
        start = nLasers / 2 - desiredNLasers;
        end = nLasers / 2 + desiredNLasers;
    }
    for (uint32_t laser_idx = start; laser_idx < end; ++laser_idx){
        minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        if (laser_idx <= nLasers / 2){
            minRSLaserDist = std::min(minRSLaserDist, msg->ranges[laser_idx]);
            if (msg->ranges[laser_idx] < maxLaserThreshold){
                RSLaserSum += msg->ranges[laser_idx];
            } 
        }
        else{
            minLSLaserDist = std::min(minLSLaserDist, msg->ranges[laser_idx]);
            if (msg->ranges[laser_idx] < maxLaserThreshold){
                LSLaserSum += msg->ranges[laser_idx];
            } 
        }
    }
    ROS_INFO("Min Laser Distance: %f \n Left: %f \n Right: %f \n LSum: %f \n RSum: %f", minLaserDist, minLSLaserDist, minRSLaserDist, LSLaserSum, RSLaserSum);
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

float chooseAngular(float laserSideSumThreshold, float probSpinToLarger){
    // Chooses the angular velocity and direction
    // Can also call this in second argument of copysign() to only extract a direction (e.g. for a rotation)
    float prob = randBetween(0.0, 1.0), angular_vel = M_PI/8, maxLaserThreshold = 7;
    ros::spinOnce();
    if(fabs(fabs(LSLaserSum) - fabs(RSLaserSum)) > laserSideSumThreshold){
        //If one side's laser distance > other side by more than laserSideSumThreshold, go to that side
        if(fabs(LSLaserSum) - fabs(RSLaserSum) > laserSideSumThreshold){
            ROS_INFO("LS >> RS. Spin CCW");
            angular_vel = randBetween(M_PI/16, M_PI/8); //improves gmapping resolution compared to always using constant value
        }
        else{
            ROS_INFO("RS >> LS. Spin CW");
            angular_vel = -randBetween(M_PI/16, M_PI/8);
        }
    }
    else{
        // Laser distances approx. equal. Go to larger at probSpinToLarger probability
        ROS_INFO("LS ~ RS. Spinning to larger at %.2f chance", probSpinToLarger);
        if ((fabs(LSLaserSum) > fabs(RSLaserSum) || fabs(minLSLaserDist) > fabs(minRSLaserDist)) && prob < probSpinToLarger){
            angular_vel = randBetween(M_PI/16, M_PI/8); 
        }
        else{
            angular_vel = -randBetween(M_PI/16, M_PI/8);
        }
    }  
    return angular_vel;
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
    ros::Subscriber odom = n.subscribe("odom", 1, &odomCallback);
    ros::Subscriber imu_sub = n.subscribe("mobile_base/sensors/imu_data", 1, &imuCallback);
    ros::Subscriber laser_sub = n.subscribe("scan", 10, &laserCallback);
    ros::Subscriber frontier_sub = n.subscribe("contest3/frontiers", 10, &markerCallback);
    
    bool exploring = true;
    float oldX = posX, oldY = posY, oldYaw = yaw, prob = 1;
    int checkStuckCount = 0, clearPathIters = 0;
    float maxLaserThreshold = 7, clearPathThreshold = 0.75, slowThreshold = 0.6, stopThreshold = 0;

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

        prob = randBetween(0.0, 1.0);

        if(anyBumperPressed()){
            bumperPressedAction(&vel, &vel_pub, &secondsElapsed, start); 
        }

        if (exploring && !anyBumperPressed() && minLaserDist < maxLaserThreshold){
            if (minLaserDist > clearPathThreshold){
                ROS_INFO("Clear path. Iter: %d", clearPathIters);
                linear = 0.2;
                angular = 0;
                clearPathIters ++;
                if (clearPathIters > 300){
                    // Random movement 25% of the time if clear path
                    if (randBetween(0.0, 1.0) < 0.25){
                        rotateThruAngle(copysign(randBetween(-M_PI/6, M_PI/6), chooseAngular(200, 0.55)), M_PI/3, yaw, 0.2, &vel, &vel_pub, &secondsElapsed, start);
                    }
                    clearPathIters = 0;
                }
            }
        }
        if(exploring && checkStuckCount == 500){
            std::cout << "Distance moved: " << dist(oldX, oldY, posX, posY) << "\n";
            std::cout << "Angle moved: " << fabs(yaw - oldYaw) << "\n";
            if(dist(oldX, oldY, posX, posY) < 0.5 && fabs(yaw - oldYaw) < 0.05){
                std::cout << "Stuck! \n";
                moveThruDistance(-0.55, 0.1, posX, posY, &vel, &vel_pub, &secondsElapsed, start);
                rotateThruAngle(2*M_PI - 0.01, M_PI/2, yaw, 0.0, &vel, &vel_pub, &secondsElapsed, start);
                //rotateThruAngle(randBetween(-M_PI/4, M_PI/4), M_PI/8, yaw, 0.0, &vel, &vel_pub, &secondsElapsed, start);
            }
            oldX = posX;
            oldY = posY;
            oldYaw = yaw;
            checkStuckCount = 0;
        }

        if(exploring && redFrontier){
            explore.start();
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
        //std::cout << "Stuck count: " << checkStuckCount << "\n";
        checkStuckCount ++;
        ros::spinOnce();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        ros::Duration(0.01).sleep();
    }
    return 0;
}