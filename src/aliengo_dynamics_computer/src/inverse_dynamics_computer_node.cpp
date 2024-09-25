
#include <ros/ros.h>
#include <ros/console.h>
#include "aliengo_dynamics_computer/inverse_dynamics_computer.hpp"

/**
 * \mainpage
 */

/**
 * @brief Main Function that starts the code by creating an instance of the class forceTransform()
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "aliengo_inverse_dynamics_computation_node");

    ros::NodeHandle nh_ ("~");

    std::string urdf_;
    nh_.getParam("robot_urdf", urdf_);
    computeInverseDynamics aliengo_inv_dyn(urdf_); //creating class member to start computation

    ros::spin();
    return 0;
}