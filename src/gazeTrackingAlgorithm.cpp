#include <gaze_tracking/gazeTrackingAlgorithm.h> 

gazeTrackingAlgorithm::gazeTrackingAlgorithm(){
    
    // subscribe to a topic which publishes filtered gazetool data
    gazeSub = n.subscribe("gazeHyps_filtered", 1, &gazeTrackingAlgorithm::gazeCallback, this);
    
    //subscribe to additional gazetool information
    gazeInfoSub = n.subscribe("additionalGazetoolInformation", 1, &gazeTrackingAlgorithm::additionalGazetoolInformationCallback, this);
    
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

void gazeTrackingAlgorithm::additionalGazetoolInformationCallback(const gazetool::GazeInfo& msg){
    horGazeTolerance = msg.horizontalGazeTolerance;
    verGazeTolerance = msg.verticalGazeTolerance;
    
}

void gazeTrackingAlgorithm::gazeCallback(const gazetool::GazeHyps& msg){
    
    // pull out filtered horizontal, vertical and mutual gaze 
    verGaze = -msg.verGaze;
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

    if (!mutGaze && (bufferSum > upperThreshold)) {
        firstMutGazeDetected = true;
    }
    isMutGaze = mutGaze;
    
}

void gazeTrackingAlgorithm::trackGaze(){
    
    if (std::abs(horGaze) < horGazeTolerance && std::abs(verGaze) < verGazeTolerance) {
        // false detection of non mutual gaze, mutual gaze is lost for a brief moment
        std::cout << "Lost you for a moment there!" << std::endl;
    }
    else {
        std::cout << "Move robot in gaze direction: " horGaze << " " << verGaze << std::endl;
        bool skip = false;
        
        // proportional controller
        verGaze = 0.05 * verGaze;
        horGaze = 0.05 * horGaze;
            
        // gaze angles are in degrees, so a transformation to radians is needed
        horGaze = horGaze * M_PI/180;
        verGaze = verGaze * M_PI/180;
        
        // transformation from face to point on a sphere
        Eigen::MatrixXd T_fs;
        Eigen::MatrixXd R_yf = Eigen::MatrixXd::Zero(4,4);
        Eigen::MatrixXd R_xf = Eigen::MatrixXd::Zero(4,4);
        Eigen::MatrixXd T_d = Eigen::MatrixXd::Identity(4,4);
        
        R_yf(0,0) = cos(horGaze);
        R_yf(2,0) = -sin(horGaze);
        R_yf(1,1) = 1;
        R_yf(0,2) = sin(horGaze);
        R_yf(2,2) = cos(horGaze);
        R_yf(3,3) = 1;
        
        R_xf(0,0) = 1;
        R_xf(1,1) = cos(verGaze);
        R_xf(2,1) = sin(verGaze);
        R_xf(1,2) = -sin(verGaze);
        R_xf(2,2) = cos(verGaze);
        R_xf(3,3) = 1;
        
        T_d(2,3) = d;
        
        T_fs = R_yf * R_xf * T_d;
        
        // transformation from base to the center of the sphere
        Eigen::MatrixXd T_bf = Eigen::MatrixXd::Zero(4,4);
        
        // direct kinematics transformation matrix
        Eigen::MatrixXd T_06;
        T_06 = kinematic.directKinematics(lwa4p_temp_q, 6);
        
        // transformation to match the orientations
        Eigen::MatrixXd T_orient = Eigen::MatrixXd::Zero(4,4);
        
        T_orient(1,0) = 1;
        T_orient(0,1) = 1;
        T_orient(2,2) = -1;
        T_orient(3,3) = 1;
        
        T_bf = T_06 * T_d * T_orient;
        
        // transformation matrix from the coordinate system of the base to the coordinate system on the sphere
        Eigen::MatrixXd T;
        T = T_bf * T_fs;
        
        // final coordinate system orientation, z pointing in the direction opposite of the sphere normal
        T = T * T_orient;
        
        // create input vector for inverse kinematics
        Eigen::MatrixXd goal_w = Eigen::MatrixXd::Zero(9,1);
        
        goal_w(0,0) = T(0,3);
        goal_w(1,0) = T(1,3);
        goal_w(2,0) = T(2,3);
        goal_w(3,0) = T(0,0);
        goal_w(4,0) = T(1,0);
        goal_w(5,0) = T(2,0);
        goal_w(6,0) = T(0,2);
        goal_w(7,0) = T(1,2);
        goal_w(8,0) = T(2,2);
        
        Eigen::MatrixXd goal_q;
        
//         std::cout << "<----------Tool configuration vector (in mm)---------->" << std::endl;
//         std::cout << goal_w << std::endl;
        
        goal_q = kinematic.inverseKinematics(goal_w); // returns all possible solutions
    //         std::cout << goal_q << std::endl;
        
        goal_q = kinematic.inverseKinematics_closestQ(goal_w, lwa4p_temp_q); // returns closest solution
    //         std::cout << goal_q << std::endl;
        
        // check if nan appears in the solution, if it appears, ignore this result and repeat the procedure
        for (int i = 0; i < 6; i = i + 1){
            if (std::isnan(goal_q(i,0))) {
                skip = true;
                std::cout << "Inverse kinematics solution not feasible!" << std::endl;
                std::cout << "Check wanted position..." << std::endl;
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
        }       
    }   
}

void gazeTrackingAlgorithm::run(){
    
    // load kinematics parameters
    initializeKinematics();
    
    initializePosition();

    initializeBuffer();
    
    ros::Rate r(20);
    
    while(ros::ok()){
        
        ros::spinOnce();
        
        checkMutualGaze();
        
        if (firstMutGazeDetected){
            if (isMutGaze) {
                //waitGazeChange    
                std::cout << "Mutual gaze detected!" << std::endl;

            }
            else {
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

// -----------------------------------------------------------------------------------//

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