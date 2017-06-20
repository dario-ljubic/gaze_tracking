#include <gaze_tracking/gazeTrackingAlgorithm.h> 

gazeTrackingAlgorithm::gazeTrackingAlgorithm(){
    
    // subscribe to a topic which publishes filtered gazetool data
    gazeSub = n.subscribe("gazeHyps_filtered", 1, &gazeTrackingAlgorithm::gazeCallback, this);
    
    // subscribe to joint states
    jointStatesSub = n.subscribe("/lwa4p_blue/joint_states", 1, &gazeTrackingAlgorithm::jointStatesCallback, this);
    
    // publish commands to the schunk position controllers
    pub_arm_1 = n.advertise<std_msgs::Float64>("lwa4p_blue/arm_1_joint_pos_controller/command", 1);
    pub_arm_2 = n.advertise<std_msgs::Float64>("lwa4p_blue/arm_2_joint_pos_controller/command", 1);
    pub_arm_3 = n.advertise<std_msgs::Float64>("lwa4p_blue/arm_3_joint_pos_controller/command", 1);
    pub_arm_4 = n.advertise<std_msgs::Float64>("lwa4p_blue/arm_4_joint_pos_controller/command", 1);
    pub_arm_5 = n.advertise<std_msgs::Float64>("lwa4p_blue/arm_5_joint_pos_controller/command", 1);
    pub_arm_6 = n.advertise<std_msgs::Float64>("lwa4p_blue/arm_6_joint_pos_controller/command", 1);
}

gazeTrackingAlgorithm::~gazeTrackingAlgorithm(){
    
}

void gazeTrackingAlgorithm::gazeCallback(const gazetool::GazeHyps& msg){
    
    // pull out filtered horizontal, vertical and mutual gaze 
    verGaze = msg.verGaze;
    horGaze = msg.horGaze;
    mutGaze = msg.mutGaze;
    
    buffer.push(mutGaze);
    bufferSum = bufferSum - buffer.front() + mutGaze;
    buffer.pop();
    
}

void gazeTrackingAlgorithm::jointStatesCallback(const sensor_msgs::JointState &msg){
    
    lwa4p_temp_q = Eigen::MatrixXd::Zero(6, 1);

    for (int i = 0; i < 6; i = i + 1){
        if (std::abs(msg.position[i]) < 0.0001)
            lwa4p_temp_q(i,0) = 0;
        else
            lwa4p_temp_q(i,0) = msg.position[i];
    }
}

void gazeTrackingAlgorithm::initializePosition(){
    // set the Schunk in the starting position so that camera is facing the user
    //WARNING: starting position depends on the outside world and the setting of the robot!!
    
    // ROS needs some time to register the core and to establish all subscriber connections.
    // Since only one message is sent in the beginning, it is lost. Therefore, loop until connection
    // is established. 
    ros::Rate poll_rate(100);
    while(pub_arm_1.getNumSubscribers() == 0 || pub_arm_2.getNumSubscribers() == 0 || pub_arm_3.getNumSubscribers() == 0 || pub_arm_4.getNumSubscribers() == 0 || pub_arm_5.getNumSubscribers() == 0 || pub_arm_6.getNumSubscribers() == 0)
        poll_rate.sleep();
    
    Eigen::MatrixXd goal_w = Eigen::MatrixXd::Zero(9, 1);
    Eigen::MatrixXd goal_q, q0;
    bool skip = false;
    
    goal_w(0,0) = x0;
    goal_w(1,0) = y0;
    goal_w(2,0) = z0;
    goal_w(3,0) = 0;
    goal_w(4,0) = 0;
    goal_w(5,0) = 1;
    goal_w(6,0) = 1;
    goal_w(7,0) = 0;
    goal_w(8,0) = 0;
    
    goal_q = kinematic.inverseKinematics(goal_w);
    
    q0 = Eigen::MatrixXd::Zero(6, 1);
    goal_q = kinematic.inverseKinematics_closestQ(goal_w, q0); // returns closest solution
    
    // check if nan appears in the solution, if it appears, ignore this result and repeat the procedure
    for (int i = 0; i < 6; i = i + 1){
        if (std::isnan(goal_q(i,0))) {
            skip = true;
            break;
        }
    }
    
    if (!skip) {
        pub_arm_1.publish(goal_q(0,0));
        pub_arm_2.publish(goal_q(1,0));
        pub_arm_3.publish(goal_q(2,0));
        pub_arm_4.publish(goal_q(3,0));
        pub_arm_5.publish(goal_q(4,0));
        pub_arm_6.publish(goal_q(5,0));
        std::cout << "In starting position!" << std::endl;
        ros::Duration(5).sleep();
        std::cout << "Ready!" << std::endl;
        
    } 
}

void gazeTrackingAlgorithm::initializeBuffer(){
    // Mutual Gaze value is calculated based on bufferSize values
    
    // initialize buffer to hold past values of mutual gaze
    for (int i = 0; i < bufferSize; i = i + 1){
        buffer.push(0);
    }
    bufferSum = 0;
}

void gazeTrackingAlgorithm::initializeKinematics(){
    
    std::string configFile;
    std::string path = ros::package::getPath("schunk_lwa4p_kinematics");
    
    ros::NodeHandle private_node_handle_("~");

    //getting ros params
    private_node_handle_.param("config_file", configFile, path + std::string("/config/lwa4p_configuration.yaml"));
    
    kinematic.loadParameters(robot_id, configFile);
}

void gazeTrackingAlgorithm::checkMutualGaze(){
    
    if (!isMutGaze && (bufferSum > upperThreshold)) {
        firstMutGazeDetected = true;
        isMutGaze = true;
        //timer.stop();
    }
    else if (isMutGaze && (bufferSum < lowerThreshold)) isMutGaze = false;
}

void gazeTrackingAlgorithm::trackGaze(){
    
    //TODO: way to get the verGazeTolerance and horGazeTolerance from gazetool gui... 
    // if (abs(horGaze) < horGazeTolerance && abs(verGaze) < verGazeTolerance) {
    
    if (std::abs(horGaze) < 5 && std::abs(verGaze) < 5) {
        // false detection of non mutual gaze, mutual gaze is lost for a brief moment
        std::cout << "Lost you for a moment there!" << std::endl;
    }
    else {
        std::cout << "Move robot in gaze direction!" << std::endl;
        movingToPoint = true;
    }
    
}

void gazeTrackingAlgorithm::run(){
    
    //TODO: check if using initializePosition() slows down initial face detection
    
    // load kinematics parameters
    initializeKinematics();
    
    initializePosition();
    //timer.start();
    initializeBuffer();
    
    ros::Rate r(10);
    
    while(ros::ok()){
        
        ros::spinOnce();
        
        checkMutualGaze();
        
        if (firstMutGazeDetected){
            if (isMutGaze) {
                //waitGazeChange    
                std::cout << "Mutual gaze detected!" << std::endl;
                movingToPoint = false;
            }
            else if (movingToPoint) {
                std::cout << "I'm on my way, from misery to happines today uh-huh uh-huh!" << std::endl;
            }
            else {
                std::cout << "I am NOT looking at you!" << std::endl;
                trackGaze();
            }
        }
        
        // if first mutual gaze is still not detected, do not move robot
        else {
            std::cout << "Hold the mutual gaze!" << std::endl;
        }
        
        r.sleep();
    }   
}

measureTime::measureTime(){
}

measureTime::~measureTime(){
}

void measureTime::start(){
    t1 = std::chrono::high_resolution_clock::now();   
}

void measureTime::stop(){
    t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;
    std::cout << "Time elapsed is: " << fp_ms.count() << " ms"<< std::endl;
}