
#include "aliengo_dynamics_computer/force_transformer.hpp"

forceTransformer::forceTransformer(/* args */)
{
  //load all the variables
  loadParams();

  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
  //initialize the synchronizer
  foot_1_ = new message_filters::Subscriber<geometry_msgs::WrenchStamped>(nh_, foot_topics_[0], 1);
  foot_2_ = new message_filters::Subscriber<geometry_msgs::WrenchStamped>(nh_, foot_topics_[1], 1);
  foot_3_ = new message_filters::Subscriber<geometry_msgs::WrenchStamped>(nh_, foot_topics_[2], 1);
  foot_4_ = new message_filters::Subscriber<geometry_msgs::WrenchStamped>(nh_, foot_topics_[3], 1);
  footSynchronizer = new message_filters::Synchronizer<forceTransformer::footSyncPolicy>(forceTransformer::footSyncPolicy(10),
                         *foot_1_, *foot_2_, *foot_3_, *foot_4_);

  footSynchronizer->registerCallback(boost::bind(&forceTransformer::footSynchronizerCallback, this, _1, _2, _3, _4));
  reaction_force_pub_ = nh_.advertise<aliengo_dynamics_computer::ReactionForce>("gazebo_leg_forces_components", 2);
  foot_force_pub_ = nh_.advertise<aliengo_dynamics_computer::FootForces>("gazebo_leg_forces_magnitude", 2);

  ROS_INFO("Done initializing %s", ros::this_node::getName().c_str());

}

forceTransformer::~forceTransformer()
{
  //Empty Destruction
}

void forceTransformer::loadParams()
{
  ros::NodeHandle nh("~"); //using private node handle
  nh.param<std::string>("aliengo/base_frame", base_frame_, "base_link");

  //reading all the topics
  nh.getParam("aliengo/foot_topics", foot_topics_);
  
  if(foot_topics_.size()!=4)
  {
    ROS_WARN("Only 4 topics are supported currently. Check the topics again. Shutting down now. Current size: %zu", foot_topics_.size());
    ros::shutdown();
  }
}

void forceTransformer::footSynchronizerCallback(const geometry_msgs::WrenchStampedConstPtr& foot1, const geometry_msgs::WrenchStampedConstPtr& foot2,
                                  const geometry_msgs::WrenchStampedConstPtr& foot3, const geometry_msgs::WrenchStampedConstPtr& foot4)
{
  aliengo_dynamics_computer::ReactionForce component_reaction_forces;
  aliengo_dynamics_computer::FootForces magnitude_forces;

  //transform force for every leg
  transformForce(*foot1, component_reaction_forces, magnitude_forces.FL_foot);
  transformForce(*foot2, component_reaction_forces, magnitude_forces.FR_foot);
  transformForce(*foot3, component_reaction_forces, magnitude_forces.RL_foot);
  transformForce(*foot4, component_reaction_forces, magnitude_forces.RR_foot);
  // ROS_INFO("Total Magnitude: %lf", (magnitude_forces.FL_foot+magnitude_forces.FR_foot+magnitude_forces.RL_foot+magnitude_forces.RR_foot));
  // ROS_INFO("--------------------------");

  //publish force
	component_reaction_forces.header.stamp = ros::Time::now();
	magnitude_forces.header.stamp = ros::Time::now();
  reaction_force_pub_.publish(component_reaction_forces);
  foot_force_pub_.publish(magnitude_forces);
}

geometry_msgs::TransformStamped forceTransformer::getTransformation(std::string source_frame, std::string target_frame)
{
  geometry_msgs::TransformStamped transform;

  try{
    transform = tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
  }

  return transform;
}

tf2::Vector3 forceTransformer::getTransformedForce (geometry_msgs::TransformStamped transform, geometry_msgs::WrenchStamped foot_force)
{
  //create rotation matix
  tf2::Quaternion q;
  tf2::Matrix3x3 rot_matrix;
  
  tf2::convert (transform.transform.rotation, q);
  rot_matrix.setRotation(q);

  //create force vector
  tf2::Vector3 force_vector(foot_force.wrench.force.x, foot_force.wrench.force.y, foot_force.wrench.force.z);

  tf2::Vector3 tf_force = rot_matrix*force_vector;
  // ROS_INFO("X: %f Y: %f Z: %f", tf_force.getX(),tf_force.getY(), tf_force.getZ());

  return tf_force;
}

void forceTransformer::transformForce(geometry_msgs::WrenchStamped foot_force, aliengo_dynamics_computer::ReactionForce& component_force, float& magnitude)
{
  geometry_msgs::WrenchStamped transformed_force;

  geometry_msgs::TransformStamped curr_transform;
  tf2::Vector3 force_vector;
  
  curr_transform = getTransformation(foot_force.header.frame_id, base_frame_);
  force_vector = getTransformedForce(curr_transform, foot_force);

  transformed_force.header.stamp = curr_transform.header.stamp;
  transformed_force.header.frame_id = foot_force.header.frame_id;

  transformed_force.wrench.force.x = force_vector.getX();
  transformed_force.wrench.force.y = force_vector.getY();
  transformed_force.wrench.force.z = force_vector.getZ();

  magnitude = std::sqrt(pow(force_vector.getX(),2)+pow(force_vector.getY(),2)+pow(force_vector.getZ(),2));
  component_force.reaction_forces.push_back(transformed_force);

  // ROS_INFO("Mag %s : %lf", foot_force.header.frame_id.c_str(), magnitude);
}