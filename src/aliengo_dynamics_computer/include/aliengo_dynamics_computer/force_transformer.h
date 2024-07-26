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
//custom message
#include <aliengo_dynamics_computer/ReactionForce.h>
class forceTransformer
{
private:

    //create variables

    /*!
     * \brief Node handle to communicate with ros master
     */

    ros::NodeHandle nh_;

    /**
     * @brief Frame to which all the force needs to be transformed
     * 
     */
    std::string base_frame_;

    /**
     * @brief List of topics on which the force values on a foot is published
     * 
     */
    std::vector<std::string> foot_topics_;

    /**
     * @brief Typedef for the ApproximateTime sync policy we are using.
     * 
     */
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::WrenchStamped, geometry_msgs::WrenchStamped,
	    geometry_msgs::WrenchStamped, geometry_msgs::WrenchStamped> footSyncPolicy;

    /**
     * @brief Message filter subscribing to 1st topic in the #foot_topics_ list
     * 
     */
	message_filters::Subscriber<geometry_msgs::WrenchStamped> *foot_1_;

    /**
     * @brief Message filter subscribing to 2nd topic in the #foot_topics_ list
     * 
     */
	message_filters::Subscriber<geometry_msgs::WrenchStamped> *foot_2_;

    /**
     * @brief Message filter subscribing to 3rd topic in the #foot_topics_ list
     * 
     */
	message_filters::Subscriber<geometry_msgs::WrenchStamped> *foot_3_;

    /**
     * @brief Message filter subscribing to 4th topic in the #foot_topics_ list
     * 
     */
	message_filters::Subscriber<geometry_msgs::WrenchStamped> *foot_4_;

    /**
     * @brief Message synchronizer
     * 
     */
	message_filters::Synchronizer<footSyncPolicy> *footSynchronizer;

    /**
     * @brief Tf buffer to store the transformation data
     * 
     */
    tf2_ros::Buffer tf_buffer_;

    /**
     * @brief Tf listerner to compute the transformation
     * 
     */
    tf2_ros::TransformListener *tf_listener_;

    /**
     * @brief ROS publisher for reaction force
     * 
     */
    ros::Publisher reaction_force_pub_;

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

    /*!
     * @brief Get the Transformed Force object
     * 
     * @param transform - Transformation used to transform the force
     * @param foot_force - Force acting on the feet
     * @return tf2::Vector3 - Transformed Force vector 
     */
    tf2::Vector3 getTransformedForce(geometry_msgs::TransformStamped transform, geometry_msgs::WrenchStamped foot_force);

    /**
     * @brief Function to load ROS params
     * 
     */
    void loadParams();

    /**
     * @brief Callback function for the message synchroniser
     * 
     * @param foot1 - Force on foot 1
     * @param foot2 - Force of foot 2
     * @param foot3 - Force of foot 3
     * @param foot4 - Force of foot 4
     */
    void footSynchronizerCallback(const geometry_msgs::WrenchStampedConstPtr& foot1, const geometry_msgs::WrenchStampedConstPtr& foot2,
                                  const geometry_msgs::WrenchStampedConstPtr& foot3, const geometry_msgs::WrenchStampedConstPtr& foot4);

    /**
     * @brief Function to transform the force from force frame to #base_frame_
     * 
     * @param foot_force 
     * @return geometry_msgs::WrenchStamped 
     */
    geometry_msgs::WrenchStamped transformForce(geometry_msgs::WrenchStamped foot_force);

public:
    /**
     * @brief Create the force Transformer object
     * 
     */
    forceTransformer(/* args */);

    /**
     * @brief Destroy the force Transformer object
     * 
     */
    ~forceTransformer();

}; //end of class forceTransformer
#endif