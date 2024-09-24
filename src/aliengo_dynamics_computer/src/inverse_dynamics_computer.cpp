#include "aliengo_dynamics_computer/inverse_dynamics_computer.hpp"
#include "aliengo_dynamics_computer/utils.hpp"

computeInverseDynamics::computeInverseDynamics(std::string robot_model_path)
{
    if(robot_model_path.empty()){
        ROS_ERROR("URDF path is empty. Cannot create model");
        ros::shutdown();
		return;
    }

		joint_data_sub_ = nh_.subscribe("/joint_states", 10, &computeInverseDynamics::jointDataCallback, this);
		odom_sub_ = nh_.subscribe("/odom", 10, &computeInverseDynamics::odometryCallback, this);
		force_pub_ = nh_.advertise<aliengo_dynamics_computer::FootForces>("pinocchio_leg_forces_magnitude", 10);
		reaction_force_pub_ = nh_.advertise<aliengo_dynamics_computer::ReactionForce>("pinocchio_leg_forces_components", 10);
    createModelAndData(robot_model_path);
    ROS_INFO("Successfully initialized");
}

computeInverseDynamics::~computeInverseDynamics()
{

}

void computeInverseDynamics::createModelAndData(std::string urdf_path)
{
  pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), robot_model_);
  
  robot_data_ = Data(robot_model_);
	robot_data_buffer_.resize(MAX_DATA_BUFFER_SIZE); //setting the size of the buffer

	//initialize the vector size
	q_ = Eigen::VectorXd(robot_model_.nq);
	q_dot_ = Eigen::VectorXd(robot_model_.nv);
	torque_ = Eigen::VectorXd(robot_model_.nv);

	//Define the contact points and get the ID to compute jacobians
	contact_points_ = {"FL_foot_fixed", "FR_foot_fixed", "RL_foot_fixed", "RR_foot_fixed"}; //using suffix fixed here to indicate joint type
	
	for(auto contact_pt:contact_points_){
    contact_pt_ids_.push_back(robot_model_.getFrameId(contact_pt, FIXED_JOINT));
    // std::cout << contact_pt << ": " << contact_pt_ids_.back() << std::endl;
  }

	/*
	//Debug prints
	// ROS_INFO("Matrix sizes:");
	// ROS_INFO("M: %ldx%ld", robot_data_.M.rows(), robot_data_.M.cols());
	// ROS_INFO("C: %ldx%ld", robot_data_.C.rows(), robot_data_.C.cols());
	// ROS_INFO("g: %ldx%ld", robot_data_.g.rows(), robot_data_.g.cols());
	// ROS_INFO("J: %ldx%ld", jointJacobian.rows(), jointJacobian.cols());
	// ROS_INFO("Pinv J: %ldx%ld", invJointJacobian.rows(), invJointJacobian.cols());
	// ROS_INFO("q: %ldx%ld", q_.rows(), q_.cols());
	// ROS_INFO("q_dot: %ldx%ld", q_dot_.rows(), q_dot_.cols());
	// ROS_INFO("Force: %ldx%ld", force_.rows(), force_.cols());
	std::cout << "robot mass: " << robot_data_.mass[0] * 9.81 << std::endl;

	//list of all joints and their DOF
	
	int i=0;
	for (auto joint:robot_model_.names)
		std::cout<< "DOF of " << joint << " --> " << *std::next(robot_model_.nqs.begin(), i++) << std::endl;
	std::cout << std::endl;
	*/
	robot_model_created_ = true;
}

void computeInverseDynamics::jointDataCallback(const sensor_msgs::JointState::ConstPtr& joint_data)
{
	if(!robot_dynamic_data_.size_initialized_)
	{
		//initialize the variable size. Hardcoding the size for now
		robot_dynamic_data_.joint_q_ = Eigen::VectorXd(12);
		robot_dynamic_data_.joint_q_dot_ = Eigen::VectorXd(12);
		robot_dynamic_data_.joint_torque_ = Eigen::VectorXd(12);
		robot_dynamic_data_.size_initialized_ = true;
	}

	if(robot_model_created_ && 
		 !joint_data->position.empty() &&
		 !joint_data->velocity.empty() &&
		 !joint_data->effort.empty())
		robot_dynamic_data_.update(*joint_data, robot_model_, 12);
}

void computeInverseDynamics::robotDynamicsData::update(sensor_msgs::JointState joint_data, Model robot_model, int joint_vector_size)
{
	std::vector<std::string> ros_joint_names = joint_data.name;
	int i=0; //position to insert the values since

	//use the joint name from the pinocchio model to order the joint vector
	for(auto model_joint_name:robot_model.names)
	{
		//check if the joint name exsist in the ros topic
		auto it = std::find(ros_joint_names.begin(), ros_joint_names.end(), model_joint_name);

		//if joint is present, then process the data
		if(it != ros_joint_names.end())
		{
			int idx = it - ros_joint_names.begin(); //index value of the array
			joint_q_(i,0) = joint_data.position[idx];
			joint_q_dot_(i,0) = joint_data.velocity[idx];
			joint_torque_(i,0) = joint_data.effort[idx];
			i++;
		}
	}

	dynamics_data_updated_ = true;
}

void computeInverseDynamics::odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_data)
{
	if(robot_dynamic_data_.dynamics_data_updated_)
	{
		q_ << util_func_.poseToVector(odom_data->pose.pose), robot_dynamic_data_.joint_q_;
		q_dot_ << util_func_.twistToVector(odom_data->twist.twist), robot_dynamic_data_.joint_q_dot_;
		torque_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, robot_dynamic_data_.joint_torque_; //using 0 for torque on body

		odom_available_ = true;
		
		//Debug prints
		// std::cout << "q: " << q_.transpose() << std::endl;
		// std::cout << "q_dot: " << q_dot_.transpose() << std::endl;
		// std::cout << "torque: " << torque_.transpose() << std::endl;
	}
	computeFootForce();
}

void computeInverseDynamics::computeFootForce()
{
	//computing all the terms. result is stored in the robot_model_variable
  pinocchio::computeAllTerms(robot_model_, robot_data_, q_, q_dot_);

	//compute corriollis matrix
  pinocchio::computeCoriolisMatrix(robot_model_, robot_data_, q_, q_dot_);
  
	//updating the robot model in pinocchio
	forwardKinematics(robot_model_, robot_data_, q_); //updates joint placement according to current joint configuration
  updateFramePlacements(robot_model_, robot_data_);

	//Compute Jacobian frame-by-frame for all contact points
	std::vector<Data::Matrix6x> frame_jacobians;
	computeFrameJacobians(frame_jacobians);
  
	//create full model jacobian matrix
	Eigen::MatrixXd full_joint_jacobian(12, robot_model_.nv);
	full_joint_jacobian.setZero();
	createFullModelJacobian(frame_jacobians, full_joint_jacobian);
	
	//pseudo inverse computation
	Eigen::MatrixXd inv_joint_jacobian = full_joint_jacobian.transpose().completeOrthogonalDecomposition().pseudoInverse();

	//Force computation
	//robot_data_.nle represent corriolis+gravity vector
	Eigen::VectorXd force_;
	force_ = inv_joint_jacobian * ( robot_data_.M * q_dot_ + robot_data_.nle - torque_);

	std::cout << "Force (F): " << force_.transpose() << std::endl;

	Eigen::Vector4d leg_force = util_func_.computeForcePerLeg(contact_points_, force_, true);
	publishFootForce(leg_force);

	aliengo_dynamics_computer::ReactionForce leg_reaction_forces;
	util_func_.vectorToForceMsg(contact_points_, force_, leg_reaction_forces);
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
	int row_number = 0;
	for(auto jacobian:frame_jacobian)
	{
		full_jacobian.block(row_number, 0, 3, robot_model_.nv ) << jacobian.block(0,0, 3, robot_model_.nv);
		row_number += 3;
	}

		//print Jacobian
		// std::cout << "Full Jacobian " << full_jacobian << std::endl;
}

void computeInverseDynamics::publishFootForce(Eigen::Vector4d foot_forces)
{
	//the order of force is the same as defined in contact_points_ variable
	//For reference:
	//contact_points_ = {"FL_foot_fixed", "FR_foot_fixed", "RL_foot_fixed", "RR_foot_fixed"}
	
	aliengo_dynamics_computer::FootForces forces;
	forces.FL_foot = foot_forces[0];
	forces.FR_foot = foot_forces[1];
	forces.RL_foot = foot_forces[2];
	forces.RR_foot = foot_forces[3];

	force_pub_.publish(forces);
}
