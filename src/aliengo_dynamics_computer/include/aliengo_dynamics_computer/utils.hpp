#ifndef UTILS_HPP
#define UTILS_HPP

//custom message
#include <eigen3/Eigen/Dense>
#include <aliengo_dynamics_computer/ReactionForce.h>
#include <aliengo_dynamics_computer/FootForces.h>
#include <aliengo_dynamics_computer/data_normalizer.hpp>

class utilFunction
{
public:

/**
	* @brief Function to extract the force values for each leg
	* 
	* @param contact_points - name of the contact_points to be used for header frames
	* @param forces - vector with force in x,y,z direction for each leg
	* @param print_debug - enable print statements. Turned off by default
	*
	* @return Eigen::Vector4d - Vector of shape 4x1 having the magnitude of force in each leg
*/
Eigen::Vector4d computeForcePerLeg(std::vector<std::string> contact_points, Eigen::VectorXd forces, bool print_debug=false)
{
	int vector_pos = 0; //vector position
	int i=0;
	Eigen::Vector4d leg_force;

	for(auto contact_pt:contact_points)
	{
		//compute the magnitude for each leg
		leg_force[i] = sqrt(forces.block(vector_pos, 0, 3, 1).array().square().sum());
		if(print_debug) ROS_INFO("Force on frame %s is %lf", contact_pt.c_str(), leg_force[i]);
		
		vector_pos +=3;
		i++;

	}
	if(print_debug) ROS_INFO("Total Force: %f", leg_force.array().sum());

	return leg_force;
}

/**
 * @brief Function to convert from geometry_msgs::Twist to Eigen::VectorXd
 * 
 * @param msg - input twist message
 * 
 * @return Eigen::VectorXd - converted message
 */
Eigen::VectorXd twistToVector(geometry_msgs::Twist msg)
{
	Eigen::VectorXd vel_vec(6);
	vel_vec << msg.linear.x, msg.linear.y, msg.linear.z,
				msg.angular.x, msg.angular.y, msg.angular.z;

	return vel_vec;
}

/**
 * @brief Function to convert from geometry_msgs::Pose to Eigen::VectorXd
 * 
 * @param msg - input pose message
 * 
 * @return Eigen::VectorXd - converted message
 */
Eigen::VectorXd poseToVector(geometry_msgs::Pose msg)
{
	Eigen::VectorXd pos_vec(7);
	pos_vec << msg.position.x, msg.position.y, msg.position.z,
				msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w;

	return pos_vec;
}

/**
 * @brief Function to convert from geometry_msgs::Pose to Eigen::VectorXd
 * 
 * @param vec - input vector message
 * 
 * @return std::vector<float> - converted stl vector message message
 */
std::vector<float> eigenToStlVector(Eigen::VectorXd vec)
{
	std::vector<float> stl_vec(vec.data(), vec.data() + vec.rows() * vec.cols());
	return stl_vec;
}

/**
 * @brief Function to extract component forces in x,y,z direction from force vector
 * 
 * @param contact_points - Name of the contact points of force
 * @param force_vector - vector containing the component force values for all leg
 * @param leg_forces - Reference variable to store the extracted force values
 * 
 * @return none
 */
void vectorToForceMsg(std::vector<std::string> contact_points, Eigen::VectorXd force_vector, aliengo_dynamics_computer::ReactionForce& leg_forces)
{
	geometry_msgs::WrenchStamped single_leg_force;
	single_leg_force.header.stamp = ros::Time().now();

	int vector_pos = 0; //vector position
	for(auto contact_pt:contact_points)
	{
		//this frame doesn't exist in ROS. But refers to the frame in the pinocchio model
		single_leg_force.header.frame_id = contact_pt;

		Eigen::Vector3d force_component = force_vector.block(vector_pos, 0, 3, 1);

		single_leg_force.wrench.force.x = force_component(0,0);
		single_leg_force.wrench.force.y = force_component(1,0);
		single_leg_force.wrench.force.z = force_component(2,0);

		leg_forces.reaction_forces.push_back(single_leg_force);
	}
}

bool normalizeComponentData(std::vector<DataNormalizer>& data_normalizer, aliengo_dynamics_computer::ReactionForce component_forces, 
		aliengo_dynamics_computer::ReactionForce& normalized_component_forces)
{
	if(data_normalizer.empty())
	{
		ROS_WARN("Data normalizer is empty. Please initialize the data normalizer.");
		return false;
	}

	//check if data is initialized
	bool is_data_ready = true;
	for(int i=0; i<12; i++)
	{
		if(!data_normalizer[i].isDataReady())
		{
			is_data_ready = false;
			break; //exit the loop if any of the data is not ready
		}
	}

	if(!is_data_ready)
	{
		//add the data to the vector
		int idx = 0;
		for (auto component:component_forces.reaction_forces)
		{
			data_normalizer[idx].addData(component.wrench.force.x);
			data_normalizer[idx+1].addData(component.wrench.force.y);
			data_normalizer[idx+2].addData(component.wrench.force.z);
			idx += 3;
		}
		return false;
	}
	else
	{
		//normalize the data
		int idx = 0;
		for (auto component:component_forces.reaction_forces)
		{
			geometry_msgs::WrenchStamped normalized_component;
			normalized_component.header.frame_id = component.header.frame_id;

			normalized_component.wrench.force.x = data_normalizer[idx].normalizeData(component.wrench.force.x);
			normalized_component.wrench.force.y = data_normalizer[idx+1].normalizeData(component.wrench.force.y);
			normalized_component.wrench.force.z = data_normalizer[idx+2].normalizeData(component.wrench.force.z);
			normalized_component_forces.reaction_forces.push_back(normalized_component);
			idx += 3;
		}
		return true;
	}
}

bool normalizeMagnitudeData(std::vector<DataNormalizer>& data_normalizer, aliengo_dynamics_computer::FootForces force, 
							aliengo_dynamics_computer::FootForces& normalized_force)
{

	if(data_normalizer.empty())
	{
		ROS_WARN("Data normalizer is empty. Please initialize the data normalizer.");
		return false;
	}

	enum FootNumber	{
		FL = 0, FR = 1, RL = 2, RR = 3
	};

	if(data_normalizer.size() == 0){
		
	}

	//check if data is initialized
	if(data_normalizer[FootNumber::FL].isDataReady() 
	&& data_normalizer[FootNumber::FR].isDataReady() 
	&& data_normalizer[FootNumber::RL].isDataReady() 
	&& data_normalizer[FootNumber::RR].isDataReady())
	{
		//normalize the data
		normalized_force.FL_foot = data_normalizer[FootNumber::FL].normalizeData(force.FL_foot);
		normalized_force.FR_foot = data_normalizer[FootNumber::FR].normalizeData(force.FR_foot);
		normalized_force.RL_foot = data_normalizer[FootNumber::RL].normalizeData(force.RL_foot);
		normalized_force.RR_foot = data_normalizer[FootNumber::RR].normalizeData(force.RR_foot);
		return true;
	}
	else
	{
		data_normalizer[FootNumber::FL].addData(force.FL_foot);
		data_normalizer[FootNumber::FR].addData(force.FR_foot);
		data_normalizer[FootNumber::RL].addData(force.RL_foot);
		data_normalizer[FootNumber::RR].addData(force.RR_foot);
		return false;
	}
}
};
#endif //UTILS_HPP