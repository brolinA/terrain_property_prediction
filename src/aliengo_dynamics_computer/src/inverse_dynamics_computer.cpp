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

	//initializing joint position and velocity to zeros
	// q_ = randomConfiguration(robot_model_); //to test q
  // q_[0] = 1.0; q_[1] = 1.0; q_[2] = 1.0; //to test q

	 
	// Eigen::RowVectorXd q_test_(19);
	// q_test_ << 0.0, 0.0, 0.35, 0.0, 0.0, 0.0, 1.0,
	// 		 -0.0031345892050191893, 0.966314043646169, -1.6204728852253254, 
	// 		 0.006543744565955301, 0.9828929825293384, -1.6231658767648796, 
	// 		 -0.0019896509839556487, 0.917828881092067, -1.6323097227361814, 
	// 		 0.0023186819676359605, 0.9159782798843361, -1.6311252805254828;
	// q_ = q_test_.transpose();
	
	// Eigen::RowVectorXd test_torque_(18);
	// test_torque_ << 0, 0, 0, 0, 0, 0,
	// 								6.936555667870072, -1.5789749897447336, -4.392563649343796, 
	// 								5.020491710991956, 0.6820409727028703, -10.327255721996368, 
	// 								9.238395972928496, -1.2266879684590393, 2.8549442232364406, 
	// 								10.251147044489567, 1.3488240309888766, 5.169799373093317;

	// // q_ = Eigen::VectorXd::Zero(robot_model_.nq);
	// // q_dot_ = Eigen::VectorXd::Zero(robot_model_.nv);

	// Eigen::RowVectorXd test_vel_(18);
	// test_vel_ << 0, 0, 0, 0, 0, 0,
	// 						-0.0005167541598068751, -0.1485678763279267, 0.24284998193821486, 
	// 						-0.0009379085241691798, -0.18031369453816873, 0.2890807213366897, 
	// 						0.00016128987056339032, -0.10699159931474307, 0.18132861039808412, 
	// 						-0.000202119745340932, -0.10206431087830797, 0.1731220393803563;
	// q_dot_ = test_vel_.transpose();
	

  // std::cout << "Joint Dimension: " << robot_model_.nq <<std::endl;
  // std::cout << "Velocity Dimension: " << robot_model_.nv <<std::endl;
  // std::cout << "q: " << q_.transpose() << std::endl;
  // std::cout << "q_dots: " << q_dot_.transpose() << std::endl;

	//Define the contact points and get the ID to compute jacobians
	contact_points_ = {"FL_foot_fixed", "FR_foot_fixed", "RL_foot_fixed", "RR_foot_fixed"}; //using suffix fixed here to indicate joint type
	
	for(auto contact_pt:contact_points_){
    contact_pt_ids_.push_back(robot_model_.getFrameId(contact_pt, FIXED_JOINT));
    // std::cout << contact_pt << ": " << contact_pt_ids_.back() << std::endl;
  }

	//-------------------------------TEST--------------------------
	//computing all the terms. result is stored in the robot_model_variable
  /*pinocchio::computeAllTerms(robot_model_, robot_data_, q_, q_dot_);

	//compute corriollis matrix
  pinocchio::computeCoriolisMatrix(robot_model_, robot_data_, q_, q_dot_);
  
	//updating the robot model in pinocchio
	forwardKinematics(robot_model_, robot_data_, q_); //updates joint placement according to current joint configuration
  updateFramePlacements(robot_model_, robot_data_);

	//compute Jacobian
	//Method 1: Compute Jacobian frame-by-frame for all contact points

	std::vector<Data::Matrix6x> frameJacobians;
  for (auto contact_id:contact_pt_ids_){
    Data::Matrix6x J(6, robot_model_.nv);
    J.setZero();

    computeFrameJacobian(robot_model_, robot_data_, q_, contact_id, LOCAL_WORLD_ALIGNED, J);
    frameJacobians.push_back(J);

		//print Jacobian
		// std::cout << "Jacobian for contact id " << contact_id << " : \n" << J.block(0,0, 3, 18 ) << std::endl;
  }

	Eigen::MatrixXd jointJacobian(12, robot_model_.nv);
	// Eigen::Matrix<double, 12, 18> jointJacobian; //(12, robot_model_.nv);
	jointJacobian.setZero();
	int row_number = 0;
	for(auto frameJacobian:frameJacobians)
	{
		jointJacobian.block(row_number, 0, 3, robot_model_.nv ) << frameJacobian.block(0,0, 3, robot_model_.nv);
		row_number += 3;
	}

	// std::cout << "Full Jacobian ("<< jointJacobian.rows() << " x " << jointJacobian.cols()<< "): \n" 
						// << jointJacobian << std::endl;

	//pseudo inverse computation
	Eigen::MatrixXd invJointJacobian = jointJacobian.transpose().completeOrthogonalDecomposition().pseudoInverse();

	//Force computation
	Eigen::VectorXd force_;
	force_ = invJointJacobian * ( robot_data_.M * q_dot_ + robot_data_.nle - test_torque_.transpose());

	std::cout << "Force (F): " << force_.transpose() << std::endl;
	std::cout << "robot mass: " << robot_data_.mass[0] * 9.81 << std::endl;

	Eigen::Vector4d lf = util_func_.computeForcePerLeg(contact_points_, force_, true);
	
	//Method 2: compute jacobian for the entire model 
	//This computes in the world frame. So we are not using this.
	// Data::Matrix6x full_jacobian(6, robot_model_.nv);
  // full_jacobian = computeJointJacobians(robot_model_, robot_data_, q_);

	robot_data_buffer_.push_back(robot_data_); //store the data for later use

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

	Eigen::Vector4d lf = util_func_.computeForcePerLeg(contact_points_, force_, true); //just for printing

	aliengo_dynamics_computer::ReactionForce leg_reaction_forces;
	util_func_.vectorToForceMsg(contact_points_, force_, leg_reaction_forces);
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

/*{
		//setup the q_ and q_dot_ vectors using the data
	// q_ = ;
	// q_dot_ = ;

	//computing all the terms. result is stored in the robot_model_variable
  pinocchio::computeAllTerms(robot_model_, robot_data_, q_, q_dot_);

	//compute corriollis matrix
  pinocchio::computeCoriolisMatrix(robot_model_, robot_data_, q_, q_dot_);
  
	//updating the robot model in pinocchio
	forwardKinematics(robot_model_, robot_data_, q_); //updates joint placement according to current joint configuration
  updateFramePlacements(robot_model_, robot_data_);

	//compute Jacobian
	//Method 1: Compute Jacobian frame-by-frame for all contact points

	std::vector<Data::Matrix6x> frameJacobian;
  for (auto contact_id:contact_pt_ids_){
    Data::Matrix6x J(6, robot_model_.nv);
    J.setZero();

    computeFrameJacobian(robot_model_, robot_data_, q_, contact_id, LOCAL_WORLD_ALIGNED, J);
    frameJacobian.push_back(J);
  }

	//Method 2: compute jacobian for the entire model
	Data::Matrix6x full_jacobian(6, robot_model_.nv);

  full_jacobian = computeJointJacobians(robot_model_, robot_data_, q_);

	robot_data_buffer_.push_back(robot_data_); //store the data for later use
}*/