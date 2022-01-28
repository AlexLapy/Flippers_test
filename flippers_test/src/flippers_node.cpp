#include "flippers_test/robot_hardware_interface.h"


// Main spin
int main(int argc, char** argv)
{
    ros::init(argc, argv, "flippers_node");
    ros::NodeHandle nh;
    
    //ros::AsyncSpinner spinner(4);  
    ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    ROBOTHardwareInterface ROBOT(nh);
    //spinner.start();
    spinner.spin();
    //ros::spin();
    return 0;
}
