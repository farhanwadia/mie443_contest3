#include <ros/ros.h>
#include <ros/package.h>
#include "explore.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
//
// If you cannot find the sound play library try the following command.
// sudo apt install ros-kinetic-sound-play
#include <sound_play/sound_play.h>
#include <ros/console.h>



void robotReaction(int max_idx){
    int humanEmotion = max_idx;

    sound_play::SoundClient sc;
    
    switch(humanEmotion){
        using namespace cv;
        using namespace std;
        //0=Angry --> embarrasment
        case '0':{
            Mat Image = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/images/embarrasment.jpeg", CV_LOAD_IMAGE_UNCHANGED);

            if (Image.empty()){
                cout << "Error loading image" << endl;
            }

            namedWindow("robotEmotion", CV_WINDOW_NORMAL);
            imshow("robotEmotion", Image);

            waitKey(5000);

            destroyWindow("robotEmotion");

            std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
            sc.playWave(path_to_sounds + "embarrasment.wav");
            break;
        }
        // 1=Disgust --> disgust
        case '1':{

            Mat Image = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/images/disgust.jpeg", CV_LOAD_IMAGE_UNCHANGED);

            if (Image.empty()){
                cout << "Error loading image" << endl;
            }

            namedWindow("robotEmotion", CV_WINDOW_NORMAL);
            imshow("robotEmotion", Image);

            waitKey(5000);

            destroyWindow("robotEmotion");

            std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
            sc.playWave(path_to_sounds + "disgust.wav");
            break;
        }
        // 2=Fear --> sad
        case '2':{

            Mat Image = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/images/sad.jpeg", CV_LOAD_IMAGE_UNCHANGED);

            if (Image.empty()){
                cout << "Error loading image" << endl;
            }

            namedWindow("robotEmotion", CV_WINDOW_NORMAL);
            imshow("robotEmotion", Image);

            waitKey(5000);

            destroyWindow("robotEmotion");

            std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
            sc.playWave(path_to_sounds + "sad.wav");
            break;
        }
        // 3=Happy --> resentment 
        case '3':{

            Mat Image = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/images/resentment.jpeg", CV_LOAD_IMAGE_UNCHANGED);

            if (Image.empty()){
                cout << "Error loading image" << endl;
            }

            namedWindow("robotEmotion", CV_WINDOW_NORMAL);
            imshow("robotEmotion", Image);

            waitKey(5000);

            destroyWindow("robotEmotion");

            std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
            sc.playWave(path_to_sounds + "resentment.wav");
            break;
        }
        // 4=Sad --> positively excited
        case '4':{

            Mat Image = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/images/excited.jpeg", CV_LOAD_IMAGE_UNCHANGED);

            if (Image.empty()){
                cout << "Error loading image" << endl;
            }

            namedWindow("robotEmotion", CV_WINDOW_NORMAL);
            imshow("robotEmotion", Image);

            waitKey(5000);

            destroyWindow("robotEmotion");

            std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
            sc.playWave(path_to_sounds + "happy.wav");
            break;
        }
        // 5=Surprise --> surprise
        case '5':{

            Mat Image = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/images/surprise.jpeg", CV_LOAD_IMAGE_UNCHANGED);

            if (Image.empty()){
                cout << "Error loading image" << endl;
            }

            namedWindow("robotEmotion", CV_WINDOW_NORMAL);
            imshow("robotEmotion", Image);

            waitKey(5000);

            destroyWindow("robotEmotion");

            std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
            sc.playWave(path_to_sounds + "surprise.wav");
            break;
        }
        // 6=Neutral --> pride
        case '6':{

            Mat Image = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/images/proud.jpeg", CV_LOAD_IMAGE_UNCHANGED);

            if (Image.empty()){
                cout << "Error loading image" << endl;
            }

            namedWindow("robotEmotion", CV_WINDOW_NORMAL);
            imshow("robotEmotion", Image);

            waitKey(5000);

            destroyWindow("robotEmotion");

            std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
            sc.playWave(path_to_sounds + "proud.wav");
            break;
        }
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
    sound_play::SoundClient sc;

    // 0=Angry --> embarrased
    // 1=Disgust --> disgust
    // 2=Fear --> sad
    // 3=Happy --> resentment 
    // 4=Sad --> positively excited
    // 5=Surprise --> surprise
    // 6=Neutral --> pride
    //
    // The code below shows how to play a sound.
    std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
    sc.playWave(path_to_sounds + "sound.wav");
    //
    // The code below shows how to start and stop frontier exploration.
    explore.stop();
    explore.start();
    while(ros::ok()) {
        // Your code here.
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    return 0;
}
