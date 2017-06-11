#include <gaze_tracking/gazeTrackingAlgorithm.h> 

gazeTrackingAlgorithm::gazeTrackingAlgorithm(){
    
    // subscribe to a topic which publishes filtered gazetool data
    sub = n.subscribe("gazeHyps_filtered", 100, &gazeTrackingAlgorithm::callback, this);
    // publish commands to the schunk position controllers
    pub_arm_1 = n.advertise<std_msgs::Float64>("lwa4p_blue/arm_1_joint_pos_controller/command", 100);
    pub_arm_2 = n.advertise<std_msgs::Float64>("lwa4p_blue/arm_2_joint_pos_controller/command", 100);
    pub_arm_3 = n.advertise<std_msgs::Float64>("lwa4p_blue/arm_3_joint_pos_controller/command", 100);
    pub_arm_4 = n.advertise<std_msgs::Float64>("lwa4p_blue/arm_4_joint_pos_controller/command", 100);
    pub_arm_5 = n.advertise<std_msgs::Float64>("lwa4p_blue/arm_5_joint_pos_controller/command", 100);
    pub_arm_6 = n.advertise<std_msgs::Float64>("lwa4p_blue/arm_6_joint_pos_controller/command", 100);
    
}

gazeTrackingAlgorithm::~gazeTrackingAlgorithm(){
}

void gazeTrackingAlgorithm::callback(const gazetool::GazeHyps& msg){
    
    // pull out filtered horizontal, vertical and mutual gaze 
    verGaze = msg.verGaze;
    horGaze = msg.horGaze;
    mutGaze = msg.mutGaze;
    
    buffer.push(mutGaze);
    bufferSum = bufferSum - buffer.front() + mutGaze;
    buffer.pop();
    
}

void gazeTrackingAlgorithm::initializePosition(){
    // set the Schunk in the starting position so that camera is facing the user
    
    // ROS needs some time to register the core and to establish all subscriber connections.
    // Since only one message is sent in the beginning, it is lost. Therefore, loop until connection
    // is established. 
    ros::Rate poll_rate(100);
    while(pub_arm_5.getNumSubscribers() == 0)
        poll_rate.sleep();
    
    //WARNING: starting position depends on the outside world and the setting of the robot!!
    std_msgs::Float64 arm_5_0;
    arm_5_0.data = -1.57079;
    
    pub_arm_5.publish(arm_5_0);

}

void gazeTrackingAlgorithm::initializeBuffer(){
    // Mutual Gaze value is calculated based on bufferSize values
    
    // initialize buffer to hold past values of mutual gaze
    for (int i = 0; i < bufferSize; i = i + 1){
        buffer.push(0);
    }
    bufferSum = 0;
}

void gazeTrackingAlgorithm::checkMutualGaze(){
    
    if (!isMutGaze && (bufferSum > upperThreshold)) isMutGaze = true;
    else if (isMutGaze && (bufferSum < lowerThreshold)) isMutGaze = false;
}


void gazeTrackingAlgorithm::run(){
    
    ros::Rate r(20); 
    
    while(ros::ok()){
        
        ros::spinOnce();
        
        checkMutualGaze();


        if (isMutGaze) {
            //waitGazeChange    
            std::cout << "I am looking at you!" << std::endl;
        }
        // while there is no mutGaze from gazeHyps_filtered wait. Then...
        //ros::spinOnce();
        else {
            std::cout << "I am NOT looking at you!" << std::endl;
        
        }
        // if mutgaze, stop, else, based on the angles, calculate the new position
        
        r.sleep();
    }
    
}

//bool gazeTrackingAlgorithm::waitGazeChange()
