
#include "aliengo_dynamics_computer/force_transformer.h"

forceTransformer::forceTransformer(/* args */)
{
  //load all the variables
  loadParams();

  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
  //initialize the synchronizer
  foot_1 = new message_filters::Subscriber<geometry_msgs::WrenchStamped>(nh_, foot_topics_[0], 1);
  foot_2 = new message_filters::Subscriber<geometry_msgs::WrenchStamped>(nh_, foot_topics_[1], 1);
  foot_3 = new message_filters::Subscriber<geometry_msgs::WrenchStamped>(nh_, foot_topics_[2], 1);
  foot_4 = new message_filters::Subscriber<geometry_msgs::WrenchStamped>(nh_, foot_topics_[3], 1);
  footSynchronizer = new message_filters::Synchronizer<forceTransformer::footSyncPolicy>(forceTransformer::footSyncPolicy(10),
                         *foot_1, *foot_2, *foot_3, *foot_4);

  footSynchronizer->registerCallback(boost::bind(&forceTransformer::footSynchronizerCallback, this, _1, _2, _3, _4));

  ROS_INFO("Done initializing %s", ros::this_node::getName().c_str());

}

forceTransformer::~forceTransformer()
{
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
  ROS_INFO("SYNC SUB --> F1: %f F3: %f" ,foot1->wrench.force.x, foot3->wrench.force.x);

  geometry_msgs::TransformStamped curr_transform = getTransformation(foot1->header.frame_id, base_frame_);

  tf2::Vector3 f = getTransformedForce(curr_transform, *foot1);
  
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