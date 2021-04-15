#include <contest3.h>

float dist(float x1, float y1, float x2, float y2){
    //Calculates the Euclidean distance between two points (x1, y1), (x2, y2)
    return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}

bool allFrontiersRed = false; // return true if ALL frontiers are red
std::vector<std::vector<geometry_msgs::Point>> redFrontiers;
void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg){
    // Checks if all frontiers are red
    // Also updates a vector of vectors corresponding to current red frontiers
    visualization_msgs::Marker m;
    int numMarkers;
    numMarkers = msg->markers.size();
    
    // Reset redFrontiers vector
    if(!redFrontiers.empty()){
        redFrontiers.clear();
    }

    allFrontiersRed = true;
    std::cout << numMarkers/2 << " frontiers found \n";
    for(int i = 0; i < numMarkers; i+=2){
        m = msg->markers[i];
        if (m.color.r < 1.0){
            allFrontiersRed = false;
        }
        else{
            // Update redFrontiers with points
            redFrontiers.push_back(m.points);
        }
        //std::cout << m.points.size() << " points in frontier " << i/2 << ". Frontier Red: " << (m.color.r == 1.0) << "\n";
    } 
}

geometry_msgs::Point getCentroid(std::vector<geometry_msgs::Point> points){
    // Calculates the centroid point (average in each dimension) given an array of points
    geometry_msgs::Point centroidPoint;
    float xAvg = 0.0;
    float yAvg = 0.0;

    if (points.size() > 0){
        for(int i = 0; i < points.size(); i++){
            xAvg += points[i].x;
            yAvg += points[i].y;
        }
        xAvg = xAvg / points.size();
        yAvg = yAvg / points.size();
    }
    centroidPoint.x = xAvg;
    centroidPoint.y = yAvg;
    centroidPoint.z = 0.0;
    return centroidPoint;
}

geometry_msgs::Point getClosestPoint(std::vector<geometry_msgs::Point> points, RobotPose robotPose){
    // Takes in a vector of points corresponding to a frontier and current robotPose
    // Returns the closest point within the frontier
    geometry_msgs::Point closestPoint;
    float distance, minD = std::numeric_limits<float>::infinity();

    closestPoint.x = 0.0;
    closestPoint.y = 0.0;
    closestPoint.z = 0.0;

    if (points.size() > 0){
        for(int i = 0; i < points.size(); i++){
            distance = dist(robotPose.x, robotPose.y, points[i].x, points[i].y);
            if(distance < minD){
                minD = distance;
                closestPoint = points[i];
            }
        }
    }
    return closestPoint;
}

geometry_msgs::Point getFurthestPoint(std::vector<geometry_msgs::Point> points, RobotPose robotPose){
    // Takes in a vector of points corresponding to a frontier and current robotPose
    // Returns the furthest point within the frontier
    geometry_msgs::Point furthestPoint;
    float distance, maxD = 0;

    furthestPoint.x = 0.0;
    furthestPoint.y = 0.0;
    furthestPoint.z = 0.0;

    if (points.size() > 0){
        for(int i = 0; i < points.size(); i++){
            distance = dist(robotPose.x, robotPose.y, points[i].x, points[i].y);
            if(distance > maxD){
                maxD = distance;
                furthestPoint = points[i];
            }
        }
    }
    return furthestPoint;
}

std::vector<int> orderIndices(std::vector<std::vector<geometry_msgs::Point>> frontiers, RobotPose robotPose){
    // Takes in a vector of red frontier vectors and current robotPose
    // Returns a vector of indices corresponding to the frontiers sorted from closest to furthest
    geometry_msgs::Point point;
    float d;
    std::vector<float> distances;
    std::vector<int> indices;
    std::vector<std::pair<float, int>> distances_and_indices;

    for(int i = 0; i < frontiers.size(); i++){
        point = getClosestPoint(frontiers[i], robotPose);
        d = dist(robotPose.x, robotPose.y, point.x, point.y);
        distances_and_indices.push_back(std::make_pair(d, i));
    }
    
    std::sort(distances_and_indices.begin(), distances_and_indices.end());

    for(int i = 0; i < distances_and_indices.size(); i++){
        indices.push_back(distances_and_indices[i].second);
    }

    return indices;
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
    //ROS_INFO("Min Laser Distance: %f \n Left: %f \n Right: %f \n LSum: %f \n RSum: %f", minLaserDist, minLSLaserDist, minRSLaserDist, LSLaserSum, RSLaserSum);
}

float omega = 0.0, accX = 0.0, accY = 0.0, yaw_imu = 0.0;
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    omega = msg->angular_velocity.z;
    accX = msg->linear_acceleration.x;
    accY = msg->linear_acceleration.y;
    yaw_imu = tf::getYaw(msg->orientation);
    //ROS_INFO("Acceleration: (%f, %f) \n IMU Yaw: %f deg Omega %f", accX, accY, RAD2DEG(yaw_imu), omega);
}

int emotionDetected = -1; //Use -1 to indicate exploring is occuring, 0-6 for emotions
void emotionCallback(const std_msgs::Int32::ConstPtr& msg){
    emotionDetected = msg->data;
    std::cout << "Emotion detected in its callback: " << emotionDetected << " \n";
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
            bumperPressedAction(pVel, pVel_pub,pSecondsElapsed, start);
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
            bumperPressedAction(pVel, pVel_pub,pSecondsElapsed, start);
            break;
        }
        i += 1;
    }
}

float chooseAngular(float laserSideSumThreshold, float probSpinToLarger){
    // Chooses the angular velocity and direction based on clearer laser side and probability inputted
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
    // Actions to implement if any bumpers get pressed
    bool any_bumper_pressed = true;
    any_bumper_pressed = anyBumperPressed();
    
    if (any_bumper_pressed && *pSecondsElapsed < 900){
        ROS_INFO("Bumper pressed \n Left: %d \n Center: %d \n Right: %d \n", bumper[LEFT], bumper[CENTER], bumper[RIGHT]);
       
        if (bumper[LEFT]){
            ROS_INFO("Left hit. Move back and spin 90 CW");
            moveThruDistance(-1.2, 0.2, posX, posY, pVel, pVel_pub, pSecondsElapsed, start);
            rotateThruAngle(DEG2RAD(-90), M_PI/4, yaw, 0, pVel, pVel_pub, pSecondsElapsed, start);
        }
        else if (bumper[RIGHT]){
            ROS_INFO("Right hit. Move back and spin 90 CCW");
            moveThruDistance(-1.2, 0.2, posX, posY, pVel, pVel_pub, pSecondsElapsed, start);
            rotateThruAngle(DEG2RAD(90), M_PI/4, yaw, 0, pVel, pVel_pub, pSecondsElapsed, start);
        }
        else if (bumper[CENTER]){
            ROS_INFO("Center hit. Move back and spin");
            moveThruDistance(-1.2, 0.2, posX, posY, pVel, pVel_pub, pSecondsElapsed, start);
            rotateThruAngle(copysign(2*M_PI/3, chooseAngular(10, 0.9)), M_PI/4, yaw, 0, pVel, pVel_pub, pSecondsElapsed, start);
        }
    }
}

bool checkPlan(ros::NodeHandle& nh, float xStart, float yStart, float phiStart, float xGoal, float yGoal, float phiGoal){
	//Returns true if there is a valid path from (xStart, yStart, phiStart) to (xGoal, yGoal, phiGoal)
    //Adapted from https://answers.ros.org/question/264369/move_base-make_plan-service-is-returning-an-empty-path/
    
    bool callExecuted, validPlan;

    //Set start position
    geometry_msgs::PoseStamped start;
    geometry_msgs::Quaternion phi1 = tf::createQuaternionMsgFromYaw(phiStart);
    start.header.seq = 0;
    start.header.stamp = ros::Time::now();
    start.header.frame_id = "map";
    start.pose.position.x = xStart;
    start.pose.position.y = yStart;
    start.pose.position.z = 0.0;
    start.pose.orientation.x = 0.0;
    start.pose.orientation.y = 0.0;
    start.pose.orientation.z = phi1.z;
    start.pose.orientation.w = phi1.w;

    //Set goal position
    geometry_msgs::PoseStamped goal;
    geometry_msgs::Quaternion phi2 = tf::createQuaternionMsgFromYaw(phiGoal);
    goal.header.seq = 0;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.pose.position.x = xGoal;
    goal.pose.position.y = yGoal;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = phi2.z;
    goal.pose.orientation.w = phi2.w;
    
    //Set up the service and call it
    ros::ServiceClient check_path = nh.serviceClient<nav_msgs::GetPlan>("move_base/NavfnROS/make_plan");
    nav_msgs::GetPlan srv;
    srv.request.start = start;
    srv.request.goal = goal;
    srv.request.tolerance = 0.0;
    callExecuted = check_path.call(srv);
    
    if(callExecuted){
        ROS_INFO("Call to check plan sent");
    }
    else{
        ROS_INFO("Call to check plan NOT sent");
    }

    if(srv.response.plan.poses.size() > 0){
        validPlan = true;
        ROS_INFO("Successful plan of size %ld", srv.response.plan.poses.size());
    }
    else{
        validPlan = false;
        ROS_INFO("Unsuccessful plan");
    }
    return validPlan;
}

bool navigateNearby(geometry_msgs::Point startPoint, std::vector<float> radii, std::vector<float> angles, ros::NodeHandle& n, RobotPose robotPose){
    // Navigates to startPoint, and if not successful, to adjacent points defined by radii and angles away
    bool validPlan, navSuccess;
    float xx, yy;

    // Check if valid navigation plan to the startPoint exists
    xx = startPoint.x;
    yy = startPoint.y;
    validPlan = checkPlan(n, robotPose.x, robotPose.y, robotPose.phi, xx, yy, atan2f(yy - robotPose.y, xx - robotPose.x));

    //Try points at different radii and angles from the centroid     
    for(int rCount = 0; rCount < radii.size(); rCount ++){
        for(int aCount = 0; aCount < angles.size(); aCount ++){
            if(!validPlan && allFrontiersRed){
                //ROS_INFO("Testing at offset r=%.2f phi=%.2f", radii[rCount], angles[aCount]);
                std::cout << "Testing at offset r=" << radii[rCount] << " phi=" << angles[aCount] << "\n";
                xx = startPoint.x + radii[rCount]*cosf(angles[aCount]);
                yy = startPoint.y + radii[rCount]*sinf(angles[aCount]);
                validPlan = checkPlan(n, robotPose.x, robotPose.y, robotPose.phi, xx, yy, atan2f(yy - robotPose.y, xx - robotPose.x));
            }
            else {
                break;
            }
        }
        if(validPlan || !allFrontiersRed){
            break;
        }
    }

    if(validPlan){
        navSuccess = Navigation::moveToGoal(xx, yy, atan2f(yy - robotPose.y, xx - robotPose.x), 10);
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    if(!allFrontiersRed){
        std::cout << "At least 1 frontier blue \n";
        return false;
    }
    if(!validPlan || !navSuccess){
        std::cout << "Could not navigate to point. \n";
        return false;
    }
    return true;
}

void robotReaction(sound_play::SoundClient& sc){  
    // Displays image and plays sound based on emotionDetected
    using namespace cv;
    using namespace std;

    string humanEmotions[7] = {"angry", "disgust", "fear", "happy", "sad", "surprise", "neutral"};
    string robotEmotions[7] = {"embarrasment", "disgust", "sadness", "resentment", "positive excitement", "surprise", "pride"};
        
    string imagePaths[7] = {"/home/turtlebot/catkin_ws/src/mie443_contest3/images/embarrasment.jpeg",
                            "/home/turtlebot/catkin_ws/src/mie443_contest3/images/disgust.jpeg",
                            "/home/turtlebot/catkin_ws/src/mie443_contest3/images/sad.jpeg",
                            "/home/turtlebot/catkin_ws/src/mie443_contest3/images/resentment.jpeg",
                            "/home/turtlebot/catkin_ws/src/mie443_contest3/images/excited.jpeg",
                            "/home/turtlebot/catkin_ws/src/mie443_contest3/images/surprise.jpeg",
                            "/home/turtlebot/catkin_ws/src/mie443_contest3/images/proud.jpeg"};
    
    string soundFiles[7] = {"embarrasment.wav",
                            "disgust.wav",
                            "sad.wav",
                            "resentment.wav",
                            "happy.wav",
                            "surprise.wav",
                            "proud.wav"};
    
    if(emotionDetected >= 0 && emotionDetected <= 6){
        cout << "Detected " << humanEmotions[emotionDetected] << ". Responding with " << robotEmotions[emotionDetected] << "\n";
        
        Mat Image = imread(imagePaths[emotionDetected], CV_LOAD_IMAGE_UNCHANGED);
        if (Image.empty()){cout << "Error loading image" << endl;}

        namedWindow("robotEmotion", CV_WINDOW_NORMAL);
        imshow("robotEmotion", Image);
        waitKey(5000);
        destroyWindow("robotEmotion");

        std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
        sc.playWave(path_to_sounds + soundFiles[emotionDetected]);   
    }
}

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest3");
    ros::NodeHandle n;
    
    // Frontier exploration algorithm.
    explore::Explore explore;
    
    // Class to handle sounds.
    sound_play::SoundClient sc;
    ros::Duration(0.5).sleep();
    
    // Publishers
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel; 

    /*ros::Publisher led1_pub = n.advertise<kobuki_msgs::Led>("mobile_base/commands/led1", 1);
    ros::Publisher led2_pub = n.advertise<kobuki_msgs::Led>("mobile_base/commands/led2", 1);
    kobuki_msgs::Led colour1, colour2;
    const uint8_t BLACK = 0, GREEN = 1, ORANGE = 2, RED = 3;*/

    // Subscribers
    ros::Subscriber bumper_sub = n.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber odom = n.subscribe("odom", 1, &odomCallback);
    ros::Subscriber imu_sub = n.subscribe("mobile_base/sensors/imu_data", 1, &imuCallback);
    ros::Subscriber laser_sub = n.subscribe("scan", 10, &laserCallback);
    ros::Subscriber frontier_sub = n.subscribe("contest3/frontiers", 10, &markerCallback);
    ros::Subscriber emotion_sub = n.subscribe("/detected_emotion", 1, &emotionCallback);
   
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    
    bool validPlan = true, navSuccess = false;
    float oldX = posX, oldY = posY, rotationSpeed = M_PI/4;
    int idx = 0, prevNumRed = 0;
    float xx, yy;
    geometry_msgs::Point point;
    
    std::vector<int> redFrontiersSortedIndices;
    std::vector<float> radii = {1.5, 1.0, 2.0, 2.5, 3.0, 0.5};
    std::vector<float> angles = {0.0, M_PI/3, -M_PI/3, 2*M_PI/3, -2*M_PI/3, M_PI-0.01};

    // Contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    std::cout << "Spin 1 started \n";
    rotateThruAngle(copysign(2*M_PI - 0.01, chooseAngular(50, 0.6)), rotationSpeed, yaw, 0.1, &vel, &vel_pub, &secondsElapsed, start);
    std::cout << "Spin 1 complete \n";
    while(ros::ok() && secondsElapsed <= 1200) {      
        explore.start();
        //For publishing colours to LEDs
        //colour1.value = RED;
        //ROS_INFO("Colour: %d", colour1.value);
        //led1_pub.publish(colour1);
        //colour2.value = GREEN;
        //ROS_INFO("Colour: %d", colour2.value);
        //led2_pub.publish(colour2);

        if(emotionDetected >=0){
            explore.stop();
            ROS_INFO("Emotion detected %d", emotionDetected);
            //Do actions here
            robotReaction(sc);
            //Emotion responses finished
            emotionDetected = -1;
            explore.start();
        }
        
        if(anyBumperPressed()){
            bumperPressedAction(&vel, &vel_pub, &secondsElapsed, start); 
        }

        if(emotionDetected == -1){
            if(redFrontiers.size() > prevNumRed){
                std::cout << "New red frontier!!";
                /*explore.stop();
                moveThruDistance(-0.6, 0.25, posX, posY, &vel, &vel_pub, &secondsElapsed, start);
                rotateThruAngle(copysign(2*M_PI - 0.01, chooseAngular(50, 0.6)), rotationSpeed, yaw, 0.2, &vel, &vel_pub, &secondsElapsed, start);
                explore.start();*/               
            }
            prevNumRed = redFrontiers.size();
        }

        if(emotionDetected == -1 && allFrontiersRed){
            explore.start();
            ros::spinOnce();
            //Navigate to closest point of closest frontier(s)
            if(redFrontiers.size() == 1){
                // Single red frontier
                point = getClosestPoint(redFrontiers[0], robotPose);
                navSuccess = navigateNearby(point, radii, angles, n, robotPose);
                ros::spinOnce();
                // Spin 15-60 deg in place if frontiers still red
                if (emotionDetected == -1 && allFrontiersRed){
                    std::cout << "Spin in 1 frontier red started \n";
                    rotateThruAngle(copysign(randBetween(M_PI/12, M_PI/3), chooseAngular(50, 0.6)), rotationSpeed, yaw, 0.0, &vel, &vel_pub, &secondsElapsed, start);
                    std::cout << "Spin in 1 frontier red ended \n";
                }
                // Go to furthest point in frontier if still red
                if(emotionDetected == -1 && allFrontiersRed){
                    ros::spinOnce();
                    point = getFurthestPoint(redFrontiers[0], robotPose); 
                    navSuccess = navigateNearby(point, radii, angles, n, robotPose);
                }
            }
            else{
                // Multiple red frontiers
                // Sort the red frontiers by distance closest to the turtlebot
                // Try navigating to closest point of closest frontier
                redFrontiersSortedIndices = orderIndices(redFrontiers, robotPose);
                for(int i = 0; i < redFrontiersSortedIndices.size(); i++){
                    idx = redFrontiersSortedIndices[i];
                    //point = getCentroid(redFrontiers[idx]);
                    ros::spinOnce();
                    point = getClosestPoint(redFrontiers[idx], robotPose);
                    navSuccess = navigateNearby(point, radii, angles, n, robotPose);
                    ros::spinOnce();
                    if(emotionDetected != -1){break;}
                    // Spin 15-60 deg in place if all frontiers still red
                    if (emotionDetected == -1 && allFrontiersRed){
                        std::cout << "Spin in 2+ frontier red started \n";
                        rotateThruAngle(copysign(randBetween(M_PI/12, M_PI/3), chooseAngular(50, 0.6)), rotationSpeed, yaw, 0.0, &vel, &vel_pub, &secondsElapsed, start);
                        std::cout << "Spin in 2+ frontier red ended \n";
                    }
                    // Go to furthest point in frontier if all still red
                    if(emotionDetected == -1 && allFrontiersRed){
                        ros::spinOnce();
                        point = getFurthestPoint(redFrontiers[idx], robotPose);
                        navSuccess = navigateNearby(point, radii, angles, n, robotPose);
                        ros::spinOnce();
                    }
                }
            }
            // Do below if none of the above worked
            if(!navSuccess && emotionDetected == -1 && allFrontiersRed){
                std::cout << "ALL NAV ATTEMPTS UNSUCCESSFUL! \n";
                explore.stop();
                explore.start();
                // Force movement
                linear = 0.25;
                angular = 0;
                ros::spinOnce();

                if(emotionDetected == -1 && allFrontiersRed){
                    moveThruDistance(0.85, 0.15, posX, posY, &vel, &vel_pub, &secondsElapsed, start);
                }
                //Rotate 10 to 45 in clearer direction 60% of the time
                if(emotionDetected == -1 && allFrontiersRed){
                    std::cout << "Spin in all nav failed started \n";
                    rotateThruAngle(copysign(randBetween(M_PI/12, M_PI/4), chooseAngular(50, 0.6)), rotationSpeed, yaw, 0.0, &vel, &vel_pub, &secondsElapsed, start);
                    std::cout << "Spin in all nav failed ended \n";
                }
            }
        }

        ros::spinOnce();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        ros::Duration(0.01).sleep();
    }
    explore.stop();
    std::cout << "20 MINUTES ELAPSED";
    return 0;
}