#ifndef INVERSE_DYNAMICS_COMPUTER_HPP
#define INVERSE_DYNAMICS_COMPUTER_HPP


//pinocchio includes (include pinnochio first to avoid compilation erros)
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/joint/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include <pinocchio/algorithm/frames.hpp>

#include <iostream>
#include <boost/circular_buffer.hpp>
//ros includes
#include <ros/ros.h>
#include <control_msgs/JointTrajectoryControllerState.h> //delete later
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include "aliengo_dynamics_computer/utils.hpp"
//custom message
#include <aliengo_dynamics_computer/ReactionForce.h>

using namespace pinocchio;

class computeInverseDynamics
{
public:

    /**!
     * Constructor
     */
    computeInverseDynamics(std::string robot_model_path);

    /**!
     * Destructor
     */
    ~computeInverseDynamics();

private:
    //create variables

    utilFunction util_func_;

    ros::NodeHandle nh_;

    Model robot_model_; //!< robot model created using the URDF file

    Data robot_data_; //!< data created using the model

    Eigen::VectorXd q_; //!< joint position vector

    Eigen::VectorXd q_dot_; //!< joint velocity vector

    Eigen::VectorXd torque_; //!< joint torque vector

    std::vector<Data::Matrix6x> frame_jacobian_; //!< storing frame jacobians seperately.

    std::vector<std::string> contact_points_; //!< name of all the contact points for which we want to calculate force
    
    std::vector<FrameIndex> contact_pt_ids_; //!< Id of the contact frames if they exist.

    const int MAX_DATA_BUFFER_SIZE = 5;

    boost::circular_buffer<Data> robot_data_buffer_; //!< Variable to store previous data values for later use

    bool odom_available_ = false;
    bool joint_data_available_ = false;
    bool robot_model_created_ = false;

    struct robotDynamicsData{

        bool size_initialized_ = false;

        bool dynamics_data_updated_ = false;

        Eigen::VectorXd joint_q_; //!< joint position vector excluding base

        Eigen::VectorXd joint_q_dot_; //!< joint velocity vector excluding base

        Eigen::VectorXd joint_torque_; //!< joint torque vector excluding base

        void update(sensor_msgs::JointState joint_data, Model robot_model, int joint_vector_size);

    }robot_dynamic_data_;

    ros::Subscriber joint_data_sub_;

    ros::Subscriber odom_sub_;

    //Function definitions

    /**!
     * Function to initialize the model and create data for pinocchio
     * 
     * @param urdf_path Path to the URDF of the robot model
     */
    void createModelAndData(std::string urdf_path);

    void jointDataCallback(const sensor_msgs::JointState::ConstPtr& joint_data);

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_data);

    Eigen::Vector4d computeForcePerLeg(std::vector<std::string> contact_points, Eigen::VectorXd forces);

    void computeFootForce();

    void computeFrameJacobians(std::vector<Data::Matrix6x>& frame_jacobian);

    void createFullModelJacobian(std::vector<Data::Matrix6x> frame_jacobian, Eigen::MatrixXd& full_jacobian);

}; //end computeInverseDynamics

#endif //INVERSE_DYNAMICS_COMPUTER_HPP