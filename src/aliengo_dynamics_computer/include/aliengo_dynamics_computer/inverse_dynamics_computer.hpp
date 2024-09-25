#ifndef INVERSE_DYNAMICS_COMPUTER_HPP
#define INVERSE_DYNAMICS_COMPUTER_HPP


//pinocchio includes (include pinocchio first to avoid compilation errors)
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
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include "aliengo_dynamics_computer/utils.hpp"
//custom message
#include <aliengo_dynamics_computer/ReactionForce.h>
#include <aliengo_dynamics_computer/FootForces.h>

using namespace pinocchio;

class computeInverseDynamics
{
public:

    /*!
     * Constructor
     */
    computeInverseDynamics(std::string robot_model_path);

    /*!
     * Destructor
     */
    ~computeInverseDynamics();

private:
    //create variables

    utilFunction util_func_; //!< object to access some utility functions

    ros::NodeHandle nh_; //!< global ros node handle

    Model robot_model_; //!< robot model created using the URDF file

    Data robot_data_; //!< data created using the model

    Eigen::VectorXd q_; //!< joint position vector

    Eigen::VectorXd q_dot_; //!< joint velocity vector

    Eigen::VectorXd torque_; //!< joint torque vector

    std::vector<Data::Matrix6x> frame_jacobian_; //!< storing frame Jacobian separately.

    std::vector<std::string> contact_points_; //!< name of all the contact points for which we want to calculate force
    
    std::vector<FrameIndex> contact_pt_ids_; //!< Id of the contact frames if they exist.

    const int MAX_DATA_BUFFER_SIZE = 5; //!< buffer size

    boost::circular_buffer<Data> robot_data_buffer_; //!< Variable to store previous data values for later use

    bool odom_available_ = false; //!< Variable to indicate if the odom data has be received
    
    bool joint_data_available_ = false; //!< Variable to indicate if the joint states has be received
    
    bool robot_model_created_ = false; //!< Variable to indicate if the pinocchio model has been created

    /**
     * \struct robotDynamicsData
     * struct to hold dynamics data required by pinocchio for computation
     */
    struct robotDynamicsData{

        bool size_initialized_ = false;

        bool dynamics_data_updated_ = false;

        Eigen::VectorXd joint_q_; //!< joint position vector excluding base

        Eigen::VectorXd joint_q_dot_; //!< joint velocity vector excluding base

        Eigen::VectorXd joint_torque_; //!< joint torque vector excluding base

        /**
         * \brief Function to the joint data to the vectors
         * 
         * @param joint_data - joint angle, velocity and torque for each joint
         * @param robot_model - pinocchio robot model
         * 
         * @return none.
         */
        void update(sensor_msgs::JointState joint_data, Model robot_model);

    };

    robotDynamicsData robot_dynamic_data_; //!< object to use struct

    ros::Subscriber joint_data_sub_; //!< subscriber to joint topic

    ros::Subscriber odom_sub_; //!< subscriber to odom topic

    ros::Publisher force_pub_; //!< publish the cumulative force per leg

    ros::Publisher reaction_force_pub_; //!< publish the force as x,y,z components per leg


    //Function definitions

    /*!
     * @brief to initialize the model and create data for pinocchio
     * 
     * @param urdf_path Path to the URDF of the robot model
     * 
     * @return none
     */
    void createModelAndData(std::string urdf_path);

    /*!
     * @brief Callback to process the joint topic data
     * 
     * @param joint_data - data from /joint_state topic
     * 
     * @return none
     */
    void jointDataCallback(const sensor_msgs::JointState::ConstPtr& joint_data);

    /*!
     * @brief Callback to process the odom topic data
     * 
     * @param odom_data - data from /odom topic
     * 
     * @return none
     */
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_data);

    /*!
     * @brief Function to compute force using inverse dynamics
     */
    void computeFootForce();

    /*!
     * @brief Function to compute jacobian for every #contact_points_
     * 
     * @param frame_jacobian - reference variable to return the jacobian for the frame
     * 
     * @return none
     */
    void computeFrameJacobians(std::vector<Data::Matrix6x>& frame_jacobian);

    /*!
     * @brief Function to concatenate jacobian from computeFrameJacobians() into a jacobian for the model
     * 
     * @param frame_jacobian - jacobian matrix for a frame
     * @param full_jacobian - reference variable to store the final model jacobian
     * 
     * @return none
     */
    void createFullModelJacobian(std::vector<Data::Matrix6x> frame_jacobian, Eigen::MatrixXd& full_jacobian);

    /*!
     * @brief Function to publish the forces to a ros topic
     * 
     * @param foot_forces - vector with force magnitude for each leg
     */
    void publishFootForce(Eigen::Vector4d foot_forces);

}; //end computeInverseDynamics

#endif //INVERSE_DYNAMICS_COMPUTER_HPP