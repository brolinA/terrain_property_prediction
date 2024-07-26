
#include "aliengo_dynamics_computer/force_transformer.h"

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
  reaction_force_pub_ = nh_.advertise<aliengo_dynamics_computer::ReactionForce>("leg_reaction_forces", 2);

  ROS_INFO("Done initializing %s", ros::this_node::getName().c_str());

}

forceTransformer::~forceTransformer()
{
  //Empty Destruction
}

void forceTransformer::loadParams()
{
  nh_.param<std::string>("aliengo/base_frame", base_frame_, "base_link");

  //reading all the topics
  nh_.getParam("aliengo/foot_topics", foot_topics_);
  
  if(foot_topics_.size()!=4)
  {
    ROS_WARN("Only 4 topics are supported currently. Check the topics again. Shutting down now.");
    ros::shutdown();
  }
}

void forceTransformer::footSynchronizerCallback(const geometry_msgs::WrenchStampedConstPtr& foot1, const geometry_msgs::WrenchStampedConstPtr& foot2,
                                  const geometry_msgs::WrenchStampedConstPtr& foot3, const geometry_msgs::WrenchStampedConstPtr& foot4)
{
  // ROS_INFO("SYNC SUB --> F1: %f F3: %f" ,foot1->wrench.force.x, foot3->wrench.force.x);

  aliengo_dynamics_computer::ReactionForce leg_forces;

  //transform force for every leg
  leg_forces.reaction_forces.push_back(transformForce(*foot1));
  leg_forces.reaction_forces.push_back(transformForce(*foot2));
  leg_forces.reaction_forces.push_back(transformForce(*foot3));
  leg_forces.reaction_forces.push_back(transformForce(*foot4));
  ROS_INFO("--------------------------");
  //publish force
  reaction_force_pub_.publish(leg_forces);
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

geometry_msgs::WrenchStamped forceTransformer::transformForce(geometry_msgs::WrenchStamped foot_force)
{
  geometry_msgs::WrenchStamped transformed_force;

  geometry_msgs::TransformStamped curr_transform;
  tf2::Vector3 force_vector;
  
  //foot1
  curr_transform = getTransformation(foot_force.header.frame_id, base_frame_);
  force_vector = getTransformedForce(curr_transform, foot_force);

  transformed_force.header.stamp = curr_transform.header.stamp;
  transformed_force.header.frame_id = foot_force.header.frame_id;

  transformed_force.wrench.force.x = force_vector.getX();
  transformed_force.wrench.force.y = force_vector.getY();
  transformed_force.wrench.force.z = force_vector.getZ();

  ROS_INFO("Mag %s : %lf", foot_force.header.frame_id.c_str(), 
            std::sqrt(pow(force_vector.getX(),2)+pow(force_vector.getY(),2)+pow(force_vector.getZ(),2)));
  return transformed_force;
  // leg_forces.reaction_forces.push_back(forceToWrenchStamped(force_vector, foot_force->header.frame_id, curr_transform.header.frame_id));
}


/*int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  /*ros::Rate rate(10.0);
  while (node.ok()){
    geometry_msgs::TransformStamped transform;
    try{
      transform = tfBuffer.lookupTransform("base", "FL_calf",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    ROS_INFO("X: %f Y: %f Z: %f", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);

    rate.sleep();
  }*

 tf2::Quaternion q;
 q.setRPY(0.0, -0.785, 0.0);
 q.setRPY(0.0, 0.785, 0.0);

 tf2::Matrix3x3 m;
 m.setRotation(q);

//  tf2::Vector3 force_(0, 0, 4.8);
 tf2::Vector3 force_(-3.39276, 0, 3.3954);

 tf2::Vector3 tf_force = m*force_;

//  std::cout << m.getRow(0) <<std::endl;
    ROS_INFO("X: %f Y: %f Z: %f", tf_force.getX(),tf_force.getY(), tf_force.getZ());


  return 0;
}*/