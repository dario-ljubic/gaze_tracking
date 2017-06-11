#include <iostream>
#include <queue>

// Ros
#include "ros/ros.h"
#include <std_msgs/Float64.h>

// user packages
#include "gazetool/GazeHyps.h"

class gazeTrackingAlgorithm {

public:
    gazeTrackingAlgorithm();
    void callback(const gazetool::GazeHyps& msg);
    ~gazeTrackingAlgorithm();
    void initializePosition();
    void initializeBuffer();
    void checkMutualGaze();
    
    void trackGaze();
    //bool detectMutualGaze(bool mutGaze);
    void run();
    
private:
    // Node handle, publishers and subscriber
    ros::NodeHandle n;
    ros::Publisher pub_arm_1;
    ros::Publisher pub_arm_2;
    ros::Publisher pub_arm_3;
    ros::Publisher pub_arm_4;
    ros::Publisher pub_arm_5;
    ros::Publisher pub_arm_6;
    ros::Subscriber sub;
    
    //buffer
    std::queue<int> buffer;
    int bufferSum;
    int bufferSize = 10; // change if strickter conditions want to be met
    float upperThreshold = 0.70*bufferSize;
    float lowerThreshold = 0.40*bufferSize;
    bool isMutGaze = false;
    
    // gazetool data
    float verGaze;
    float horGaze;
    bool mutGaze;
    
};