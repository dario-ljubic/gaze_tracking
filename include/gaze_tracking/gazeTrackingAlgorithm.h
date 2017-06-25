#include <iostream>
#include <queue>
#include <string>
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <chrono>

// ROS
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>

// user defined
#include "gazetool/GazeHyps.h"
#include "gazetool/GazeInfo.h"
#include "schunk_lwa4p_trajectory/WaypointArray.h"
#include <schunk_lwa4p_kinematics/lwa4p_kinematics.h>

class measureTime
{
public:
    measureTime();
    ~measureTime();
    void start();
    void stop(); 
    
private:
    std::chrono::high_resolution_clock::time_point t1;
    std::chrono::high_resolution_clock::time_point t2;
};

class gazeTrackingAlgorithm {

public:
    gazeTrackingAlgorithm();
    ~gazeTrackingAlgorithm();
    void run();
    
private:
    void gazeCallback(const gazetool::GazeHyps& msg);
    void additionalGazetoolInformationCallback(const gazetool::GazeInfo& msg);
    void jointStatesCallback(const sensor_msgs::JointState &msg);
    
    void initializeBuffer();
    void initializeKinematics();
    void initializePosition();
    
    void checkMutualGaze();
    void trackGaze();
    
    // Node handle, publishers and subscriber
    ros::NodeHandle n;
    ros::Publisher pub_arm_1;
    ros::Publisher pub_arm_2;
    ros::Publisher pub_arm_3;
    ros::Publisher pub_arm_4;
    ros::Publisher pub_arm_5;
    ros::Publisher pub_arm_6;
    ros::Subscriber gazeSub;
    ros::Subscriber gazeInfoSub;
    ros::Subscriber jointStatesSub;
    
    //buffer
    std::queue<int> buffer;
    int bufferSum;
    int bufferSize = 10; // change if strickter conditions want to be met
    float upperThreshold = 0.75 * bufferSize;
    float lowerThreshold = 0.50 * bufferSize;
    
    bool isMutGaze = false;
    bool firstMutGazeDetected = false;
    
    // gazetool data
    double verGaze;
    double horGaze;
    bool mutGaze;
    double horGazeTolerance; // information from gazetool GUI
    double verGazeTolerance;
    
    // algorithm parameters
    double d = 500; // assumend distance from face to camera WARNING: distance is in mm, because all the kinematics is done in mm!
    double x0 = 500;
    double y0 = 0; // or 150
    double z0 = 1000;
    
    // lwa4p_kinematics
    lwa4p_kinematics kinematic;
    int robot_id = 0; // to choose blue robot (important when loading kinematic parameters)
    Eigen::MatrixXd lwa4p_temp_q;
    
    // time measuring
    measureTime timer;

};