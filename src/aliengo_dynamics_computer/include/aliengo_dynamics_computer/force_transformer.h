#ifndef FORCE_TRANSFORMER_H
#define FORCE_TRANSFORMER_H

//ROS includes
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Vector3.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class forceTransformer
{
private:

    //create variables

    /**
     * \brief Node handle to communicate with ros master
     */

    ros::NodeHandle nh_;

    std::string base_frame_;

    std::vector<std::string> foot_topics_;

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::WrenchStamped, geometry_msgs::WrenchStamped,
	    geometry_msgs::WrenchStamped, geometry_msgs::WrenchStamped> footSyncPolicy;

	message_filters::Subscriber<geometry_msgs::WrenchStamped> *foot_1;
	message_filters::Subscriber<geometry_msgs::WrenchStamped> *foot_2;
	message_filters::Subscriber<geometry_msgs::WrenchStamped> *foot_3;
	message_filters::Subscriber<geometry_msgs::WrenchStamped> *foot_4;
	message_filters::Synchronizer<footSyncPolicy> *footSynchronizer;

    tf2_ros::Buffer tf_buffer_;
    // tf2_ros::TransformListener tfListener(tf_buffer_, nh_);
    tf2_ros::TransformListener *tf_listener_;

    //create functions

    /**
     * \brief Function to get the transformation between source frame and target frame
     * 
     * @param source_frame - frame that needs to be transformed 
     * @param target_frame - frame to which we want to tranform to 
     * 
     * @return transformation from source to target.
     */
    geometry_msgs::TransformStamped getTransformation(std::string source_frame, std::string target_frame);

    tf2::Vector3 getTransformedForce (geometry_msgs::TransformStamped transform, geometry_msgs::WrenchStamped foot_force);

    void loadParams();

    void footSynchronizerCallback(const geometry_msgs::WrenchStampedConstPtr& foot1, const geometry_msgs::WrenchStampedConstPtr& foot2,
                                  const geometry_msgs::WrenchStampedConstPtr& foot3, const geometry_msgs::WrenchStampedConstPtr& foot4);

public:
    forceTransformer(/* args */);
    ~forceTransformer();

}; //end of class forceTransformer
#endif