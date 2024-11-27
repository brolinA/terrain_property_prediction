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
#include <vector>
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
#include <aliengo_dynamics_computer/PinocchioDebug.h>

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

    bool odom_available_ = false; //!< Variable to indicate if the odom data has be received
    
    bool joint_data_available_ = false; //!< Variable to indicate if the joint states has be received
    
    bool robot_model_created_ = false; //!< Variable to indicate if the pinocchio model has been created

    /**
     * \struct robotDynamicsData
     * struct to hold dynamics data required by pinocchio for computation
     */
    struct robotDynamicsData{

        const int MAX_DATA_BUFFER_SIZE = 3; //!< buffer size

        bool dynamics_data_updated_ = false;

        int data_counter_ = 0; //<! variable to track if the buffer is filled with proper data

        Eigen::VectorXd joint_q_; //!< joint position vector excluding base

        Eigen::VectorXd joint_q_dot_; //!< joint velocity vector excluding base

        Eigen::VectorXd joint_torque_; //!< joint torque vector excluding base

        boost::circular_buffer<Eigen::VectorXd> joint_q_buffer_; //!< buffer to store joint positions

        boost::circular_buffer<Eigen::VectorXd> joint_q_dot_buffer_; //!< buffer to store joint velocity

        boost::circular_buffer<Eigen::VectorXd> joint_torque_buffer_; //!< buffer to store joint torque

        /**
         * @brief Constructor
         * 
        */
       robotDynamicsData()
       {
        //initialize the variable size. Hardcoding the size for now
		//3 DOF per leg * 4 leg = 12 DOF total
        //initialize to zero for easy computation of average value
		joint_q_ = Eigen::VectorXd::Zero(12);
		joint_q_dot_ = Eigen::VectorXd::Zero(12);
		joint_torque_ = Eigen::VectorXd::Zero(12);
        //resize the ring buffer
        joint_q_buffer_.resize(MAX_DATA_BUFFER_SIZE);
        joint_q_dot_buffer_.resize(MAX_DATA_BUFFER_SIZE);
        joint_torque_buffer_.resize(MAX_DATA_BUFFER_SIZE);
       }
        
        /**
         * \brief Function to the joint data to the vectors
         * 
         * @param joint_data - joint angle, velocity and torque for each joint
         * @param robot_model - pinocchio robot model
         * 
         * @return none.
         */
        void update(sensor_msgs::JointState joint_data, Model robot_model);

        /**
         * @brief Function to average the values from #joint_q_buffer_, #joint_q_dot_buffer_, #joint_torque_buffer_
         * 
        */
        void averageData();

    };

    robotDynamicsData robot_dynamic_data_; //!< object to use struct

    ros::Subscriber joint_data_sub_; //!< subscriber to joint topic

    ros::Subscriber odom_sub_; //!< subscriber to odom topic

    ros::Publisher force_pub_; //!< publish the cumulative force per leg

    ros::Publisher reaction_force_pub_; //!< publish the force as x,y,z components per leg

    ros::Publisher pinocchio_debug_pub_; //!< publish the force as x,y,z components per leg


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