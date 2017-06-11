#include <gaze_tracking/gazeTrackingAlgorithm.h> 

int main (int argc, char **argv) {
    
    //TODO: add a parser, figure out which options would be good to have
    
    ros::init(argc, argv, "gaze_tracking");
    
    gazeTrackingAlgorithm tracker;
    tracker.initializePosition();
    
    tracker.initializeBuffer();
    
    tracker.run();
    
    return 0;
}