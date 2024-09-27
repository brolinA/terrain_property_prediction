#include "aliengo_dynamics_computer/inverse_dynamics_computer.hpp"
#include "aliengo_dynamics_computer/utils.hpp"

computeInverseDynamics::computeInverseDynamics(std::string robot_model_path)
{
  //Check if the path is not empty
  if(robot_model_path.empty()){
    ROS_ERROR("URDF path is empty. Cannot create model");
    ros::shutdown();
    return;
  }

  //Initialize publisher and subscribers
  joint_data_sub_ = nh_.subscribe("/joint_states", 10, &computeInverseDynamics::jointDataCallback, this);
  odom_sub_ = nh_.subscribe("/odom", 10, &computeInverseDynamics::odometryCallback, this);
  force_pub_ = nh_.advertise<aliengo_dynamics_computer::FootForces>("pinocchio_leg_forces_magnitude", 10);
  reaction_force_pub_ = nh_.advertise<aliengo_dynamics_computer::ReactionForce>("pinocchio_leg_forces_components", 10);
  pinocchio_debug_pub_ = nh_.advertise<aliengo_dynamics_computer::PinocchioDebug>("pinocchio_debug", 10);

  createModelAndData(robot_model_path); //create the robot mode for pinocchio from the URDF file
  ROS_INFO("Successfully initialized");
}

computeInverseDynamics::~computeInverseDynamics()
{

}

void computeInverseDynamics::createModelAndData(std::string urdf_path)
{
  pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), robot_model_);
  
  robot_data_ = Data(robot_model_);
	
	//initialize the vector size
	q_ = Eigen::VectorXd(robot_model_.nq);
	q_dot_ = Eigen::VectorXd(robot_model_.nv);
	torque_ = Eigen::VectorXd(robot_model_.nv);

	//Define the contact points and get the ID to compute jacobians
	//you can get the list of frame using the robot_model_.frames variable. Refer below in the debug print for more info
	// contact_points_ = {"FL_foot_fixed", "FR_foot_fixed", "RL_foot_fixed", "RR_foot_fixed"}; //using suffix fixed here to indicate joint type
	
	//If using this vector then change the type in th getFrameId function below from FIXED_JOINT to BODY
	contact_points_ = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
	
	for(auto contact_pt:contact_points_){
    // contact_pt_ids_.push_back(robot_model_.getFrameId(contact_pt, FIXED_JOINT));
    contact_pt_ids_.push_back(robot_model_.getFrameId(contact_pt, BODY));
    // std::cout << contact_pt << ": " << contact_pt_ids_.back() << std::endl;
  }

	
	//Debug prints to help understand the matrices
	// ROS_INFO_ONCE("Matrix sizes:");
	// ROS_INFO_ONCE("M: %ldx%ld", robot_data_.M.rows(), robot_data_.M.cols());
	// ROS_INFO_ONCE("C: %ldx%ld", robot_data_.C.rows(), robot_data_.C.cols());
	// ROS_INFO_ONCE("g: %ldx%ld", robot_data_.g.rows(), robot_data_.g.cols());
	// ROS_INFO_ONCE("J: %ldx%ld", jointJacobian.rows(), jointJacobian.cols());
	// ROS_INFO_ONCE("Pinv J: %ldx%ld", invJointJacobian.rows(), invJointJacobian.cols());
	// ROS_INFO_ONCE("q: %ldx%ld", q_.rows(), q_.cols());
	// ROS_INFO_ONCE("q_dot: %ldx%ld", q_dot_.rows(), q_dot_.cols());
	// ROS_INFO_ONCE("torque: %ldx%ld", torque_.rows(), torque_.cols());
	// ROS_INFO_ONCE("Force: %ldx%ld", force_.rows(), force_.cols());
	// std::cout << "robot mass: " << robot_data_.mass[0] * 9.81 << std::endl;
	
	//list of all joints and their DOF
	// int i=0;
	// for (auto joint:robot_model_.names)
	// 	std::cout<< "DOF of " << joint << " --> " << *std::next(robot_model_.nqs.begin(), i++) << std::endl;

	//list of all the frames in the model
	//reference: https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1FrameTpl.html
	// for (auto frame:robot_model_.frames)
	// 	std::cout << frame.name << " --> " << frame.type << std::endl;

	robot_model_created_ = true;
}

void computeInverseDynamics::jointDataCallback(const sensor_msgs::JointState::ConstPtr& joint_data)
{
	//test if the vector is not empty. Sometimes the joint_state topic returns empty arrays
	if(robot_model_created_ && 
		 !joint_data->position.empty() &&
		 !joint_data->velocity.empty() &&
		 !joint_data->effort.empty())
		robot_dynamic_data_.update(*joint_data, robot_model_);
}

void computeInverseDynamics::robotDynamicsData::update(sensor_msgs::JointState joint_data, Model robot_model)
{
	std::vector<std::string> ros_joint_names = joint_data.name; //name of the joints available in 
	int row_number=0; //row number to insert the value
	Eigen::VectorXd q(12), q_dot(12), torque(12);

	//use the joint name from the pinocchio model to order the joint vector since the order follows the URDF tree
	for(auto model_joint_name:robot_model.names)
	{
		//find the index of the ros_joint 
		auto it = std::find(ros_joint_names.begin(), ros_joint_names.end(), model_joint_name);

		//if joint is present, then process the data
		if(it != ros_joint_names.end())
		{
			int idx = it - ros_joint_names.begin(); //index value of the array
			q(row_number,0) = joint_data.position[idx];
			q_dot(row_number,0) = joint_data.velocity[idx];
			torque(row_number,0) = joint_data.effort[idx];
			row_number++;
		}
	}
	joint_q_buffer_.push_back(q);
	joint_q_dot_buffer_.push_back(q_dot);
	joint_torque_buffer_.push_back(torque);
	data_counter_++;

	if(data_counter_ >= MAX_DATA_BUFFER_SIZE)
		averageData();

}

void computeInverseDynamics::robotDynamicsData::averageData()
{
	//cycle through the variable and sum them
	for (int i=0; i<MAX_DATA_BUFFER_SIZE; i++)
	{
		joint_q_ += joint_q_buffer_[i];
		joint_q_dot_ += joint_q_dot_buffer_[i];
		joint_torque_ += joint_torque_buffer_[i];
	}

	joint_q_ /= MAX_DATA_BUFFER_SIZE;
	joint_q_dot_ /= MAX_DATA_BUFFER_SIZE;
	joint_torque_ /= MAX_DATA_BUFFER_SIZE;

	dynamics_data_updated_ = true;
}

void computeInverseDynamics::odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_data)
{
	//create the joint angle and joint velocity vectors only when the data is available 
	if(robot_dynamic_data_.dynamics_data_updated_)
	{
		q_ << util_func_.poseToVector(odom_data->pose.pose), robot_dynamic_data_.joint_q_;
		q_dot_ << util_func_.twistToVector(odom_data->twist.twist), robot_dynamic_data_.joint_q_dot_;
		torque_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, robot_dynamic_data_.joint_torque_; //using 0 for torque on body

		//Debug prints
		// std::cout << "q: " << q_.transpose() << std::endl;
		// std::cout << "q_dot: " << q_dot_.transpose() << std::endl;
		// std::cout << "torque: " << torque_.transpose() << std::endl;

		//Debug publisher
		aliengo_dynamics_computer::PinocchioDebug pin_debug;
		pin_debug.joint_angle = util_func_.eigenToStlVector(q_);
		pin_debug.joint_velocity = util_func_.eigenToStlVector(q_dot_);
		pin_debug.joint_torque = util_func_.eigenToStlVector(torque_);
		pinocchio_debug_pub_.publish(pin_debug);

		computeFootForce(); //computer force on each foot using inverse dynamics
	}
}

void computeInverseDynamics::computeFootForce()
{
	//computing all the dynamics terms. result is stored in the robot_model_variable
  pinocchio::computeAllTerms(robot_model_, robot_data_, q_, q_dot_);

	//compute coriolis matrix
  pinocchio::computeCoriolisMatrix(robot_model_, robot_data_, q_, q_dot_);
  
	//updating the robot model in pinocchio
	forwardKinematics(robot_model_, robot_data_, q_); //updates joint placement according to current joint configuration
  updateFramePlacements(robot_model_, robot_data_);

	//Compute Jacobian frame-by-frame for all contact points
	std::vector<Data::Matrix6x> frame_jacobians;
	computeFrameJacobians(frame_jacobians);
  
	//create jacobian matrix for the robot model
	Eigen::MatrixXd full_joint_jacobian(12, robot_model_.nv);
	full_joint_jacobian.setZero(); //required to avoid numerical errors
	createFullModelJacobian(frame_jacobians, full_joint_jacobian);
	
	//pseudo inverse computation
	Eigen::MatrixXd inv_joint_jacobian = full_joint_jacobian.transpose().completeOrthogonalDecomposition().pseudoInverse();

	//Force computation
	//robot_data_.nle represent coriolis+gravity vector
	//Original equ: mass_matrix*joint_velocity + coriolis + gravity = torque + joint_jacobian*force
	Eigen::VectorXd force_;
	force_ = inv_joint_jacobian * ( robot_data_.M * q_dot_ + robot_data_.nle - torque_);

	// std::cout << "Force (F): " << force_.transpose() << std::endl;

	Eigen::Vector4d leg_force = util_func_.computeForcePerLeg(contact_points_, force_, false);
	publishFootForce(leg_force);

	//publish the force components as ROS message
	aliengo_dynamics_computer::ReactionForce leg_reaction_forces;
	util_func_.vectorToForceMsg(contact_points_, force_, leg_reaction_forces);
	leg_reaction_forces.header.stamp = ros::Time::now();
	reaction_force_pub_.publish(leg_reaction_forces);
}

void computeInverseDynamics::computeFrameJacobians(std::vector<Data::Matrix6x>& frame_jacobian)
{
	//compute jacobian for every contact point
	for (auto contact_id:contact_pt_ids_){
		Data::Matrix6x J(6, robot_model_.nv);
		J.setZero();

		computeFrameJacobian(robot_model_, robot_data_, q_, contact_id, LOCAL_WORLD_ALIGNED, J);
		frame_jacobian.push_back(J);

		//print Jacobian
		// std::cout << "Jacobian for contact id " << contact_id << " : \n" << J.block(0,0, 3, 18 ) << std::endl;
	}
}

void computeInverseDynamics::createFullModelJacobian(std::vector<Data::Matrix6x> frame_jacobian, Eigen::MatrixXd& full_jacobian)
{
	//reference for Eigen::block function - https://eigen.tuxfamily.org/dox/group__TutorialSlicingIndexing.html
	int row_number = 0;
	for(auto jacobian:frame_jacobian)
	{
		full_jacobian.block(row_number, 0, 3, robot_model_.nv ) << jacobian.block(0, 0, 3, robot_model_.nv);
		row_number += 3;
	}

	// print Jacobian
	// std::cout << "Full Jacobian " << full_jacobian << std::endl;
}

void computeInverseDynamics::publishFootForce(Eigen::Vector4d foot_forces)
{
	//the order of force is the same as defined in contact_points_ variable
	//For reference:
	//contact_points_ = {"FL_foot_fixed", "FR_foot_fixed", "RL_foot_fixed", "RR_foot_fixed"}
	
	aliengo_dynamics_computer::FootForces forces;
	forces.header.stamp = ros::Time::now();
	forces.FL_foot = foot_forces[0];
	forces.FR_foot = foot_forces[1];
	forces.RL_foot = foot_forces[2];
	forces.RR_foot = foot_forces[3];

	force_pub_.publish(forces);
}
