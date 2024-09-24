
#include <ros/ros.h>
#include <ros/console.h>
#include "aliengo_dynamics_computer/force_transformer.hpp"

/**
 * \mainpage
 */

/**
 * @brief Main Function that starts the code by creating an instace of the class forceTransform()
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_transformation_node");
    
    forceTransformer force_transform; //creating class member to start computation

    ros::spin();
    return 0;
}