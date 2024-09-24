
#include "aliengo_dynamics_computer/inverse_dynamics_computer.hpp"

int main(int argc, char ** argv)
{

  using namespace pinocchio;

  // const std::string urdf_filename = "/home/isaac/terrain_property_prediction/src/aliengo_wild_visual_navigation/xacro/aliengo.champ.urdf";
  const std::string urdf_filename = "/home/isaac/terrain_property_prediction/src/aliengo_wild_visual_navigation/xacro/pinnochio_aliengo.champ.urdf";
  // const std::string urdf_filename = "/home/isaac/pinocchio/models/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf";
 
  // Load the urdf model
  std::cout <<"Urdf path: " << urdf_filename << std::endl;
  Model model;
  pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(), model);
  
  std::cout << "model name: " << model.name << std::endl;
 
  // Create data required by the algorithms
  Data data(model);
  
  // Sample a random configuration
  Eigen::VectorXd q = randomConfiguration(model);
  q[0] = 0.0; q[1] = 0.0; q[2] = 0.0;

  std::cout << "Joint Dimension: " << model.nq <<std::endl;
  std::cout << "Velocity Dimension: " << model.nv <<std::endl;
  std::cout << "q: " << q.transpose() << std::endl;

  //velocity initialization
  Eigen::VectorXd q_dot = Eigen::VectorXd::Random(model.nv);
  q_dot = (q_dot + Eigen::VectorXd::Ones(model.nv))/2; //normalizing between [0, 1]
  std::cout << "q_dot: " << q_dot.transpose() << std::endl;
 
  // Perform the forward kinematics over the kinematic tree
  pinocchio::computeAllTerms(model, data, q, q_dot);

  //Mass matrix
  Eigen::MatrixXd M_ = data.M;
  // std::cout << "Mass matrix: \n" << M_ << std::endl;

  //generalized gravity vector
  Eigen::VectorXd g_ = data.g;

  //corriollis matrix
  pinocchio::computeCoriolisMatrix(model, data, q, q_dot);
  Eigen::MatrixXd C_ = data.C;

  // std::vector<double> mass_ = data.mass;
  // std::cout << "mass vector: " << mass_[0] << " size: " << mass_.size() << std::endl;

  //getting all the available frames
  // for (auto frame:model.frames)
  //   std::cout << "Name: " << frame.name << " Type: " << frame.type << std::endl;

  //compute Jacobian
  //Step1: Define the contact points and gets its frame ID
  std::vector<std::string> contact_points {"FL_foot_fixed", "FR_foot_fixed", "RL_foot_fixed", "RR_foot_fixed"}; //using suffix fixed here to indicate joint type
  std::vector<FrameIndex> contactIds;

  for(auto cp:contact_points){
    // std::cout << cp << " exists? " << model.existFrame(cp, FIXED_JOINT) << std::endl; 
    contactIds.push_back(model.getFrameId(cp, FIXED_JOINT));
    // std::cout << contactIds.back() << std::endl;
  }
  
  //Step2: Compute forward kinematics
  forwardKinematics(model, data, q); //updates joint placement according to current joint configuration
  updateFramePlacements(model, data);

  //step3: compute jacobian for each frame
  std::vector<Data::Matrix6x> allJacobian;
  for (auto cId:contactIds){
    Data::Matrix6x J(6, model.nv);
    J.setZero();

    computeFrameJacobian(model, data, q, cId, LOCAL_WORLD_ALIGNED, J);
    allJacobian.push_back(J);
  }

  std::cout << "Jacbian shape: \n" << allJacobian[0].rows() << "x" << allJacobian[0].cols()
            << " vector size: " << allJacobian.size() << std::endl;

  std::cout << "Jacbian 0: \n" << allJacobian[0] << std::endl;
  std::cout << "Jacbian 1: \n" << allJacobian[1] << std::endl;
  
  Data::Matrix6x FJ(6, model.nv);

  FJ = computeJointJacobians(model, data, q);

  std::cout << "Full Jacboian: \n" << FJ << std::endl;
  // Print out the placement of each joint of the kinematic tree
  // std::cout << "\nJoint placements:" << std::endl;
  // for (JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
  //   std::cout << std::setw(24) << std::left << model.names[joint_id] << ": " << std::fixed
  //             << std::setprecision(2) << data.oMi[joint_id].translation().transpose() << std::endl;
 
  return 0;

}
