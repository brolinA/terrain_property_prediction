#ifndef UTILS_HPP
#define UTILS_HPP

//custom message
#include <aliengo_dynamics_computer/ReactionForce.h>

class utilFunction
{
public:

Eigen::Vector4d computeForcePerLeg(std::vector<std::string> contact_points, Eigen::VectorXd forces, bool print_debug=false)
{
	int vector_pos = 0; //vector position
	int i=0;
	Eigen::Vector4d leg_force;
	for(auto contact_pt:contact_points)
	{
		leg_force[i] = sqrt(forces.block(vector_pos, 0, 3, 1).array().square().sum());
		if(print_debug) ROS_INFO("Force on frame %s is %lf", contact_pt.c_str(), leg_force[i]);
		
		vector_pos +=3;
		i++;

	}
	if(print_debug) ROS_INFO("Total Force: %f", leg_force.array().sum());

	return leg_force;
}

Eigen::VectorXd twistToVector(geometry_msgs::Twist msg)
{
	Eigen::VectorXd vel_vec(6);
	vel_vec << msg.linear.x, msg.linear.y, msg.linear.z,
				msg.angular.x, msg.angular.y, msg.angular.z;

	return vel_vec;
}

Eigen::VectorXd poseToVector(geometry_msgs::Pose msg)
{
	Eigen::VectorXd pos_vec(7);
	pos_vec << msg.position.x, msg.position.y, msg.position.z,
				msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w;

	return pos_vec;
}

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

};
#endif //UTILS_HPP