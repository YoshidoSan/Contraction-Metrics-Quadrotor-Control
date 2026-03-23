#ifndef CCMCONTROLLER_H
#define CCMCONTROLLER_H

#include "ros/ros.h"
#include "mavros_msgs/ActuatorControl.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <CCM/Geodesic.h>
//#include <CCM/Metric.h>
#include <CCM/ccmutils.h>
#include <cmath>
#include <deque>
#include <ccm_tracking_controller/Debug_Readings.h>
#include <ccm_tracking_controller/Debug_Calc.h>

#include <fstream>
#include <iostream>


class CCMController {
private:
  // ros 
  ros::NodeHandle nh;
  ros::Publisher DebugPubUpdate_; // debug readings publisher
  ros::Publisher DebugPubMeasured_; // debug measured state publisher
  ros::Publisher DebugPubMeasured_dot_; // debug measured dot state publisher
	ros::Publisher DebugPubNominal_; // debug nominal state publisher
  ros::Publisher DebugPubNominal_dot_; // debug nominal dot state publisher
  ros::Publisher DebugPubNominal_u_; // debug nominal state u publisher
  ros::Publisher DebugPubData_; // debug ccm data publisher

  // show debug data
  bool verbose_ = true;

  //debug logs
	std::ofstream log_file;

  // mode
  bool active_;

  // measured states
  Eigen::Vector3d mea_pos_;
  Eigen::Vector3d mea_vel_;
  Eigen::Vector3d mea_eul_;
  Eigen::Vector3d mea_wb_;
  Eigen::Matrix3d mea_Rot_;
  double mea_ft_;

  // nominal (target) state
  Eigen::VectorXd xc_nom_;
  Eigen::Vector4d uc_nom_;
  Eigen::VectorXd xc_nom_dot_;
  Eigen::MatrixXd W_nom_;
  Eigen::MatrixXd M_nom_;
  // current state
  Eigen::VectorXd xc_;
  Eigen::VectorXd xc_dot_;
  Eigen::MatrixXd W_;
  Eigen::MatrixXd M_;
  // state shift and straight-line appproximation of geodesic 
  Eigen::VectorXd delta_x_g_;
  // middle state, for integral calculation
  Eigen::VectorXd xc_mid_;
  Eigen::MatrixXd W_mid_;
  Eigen::MatrixXd M_mid_;
  // for Riemannian energy calculation
  Eigen::VectorXd h_;

  double M_yaw_; // yaw bounds
  Eigen::Matrix4d A_; // config matrix
  Eigen::Matrix3d J_; // inertia

  // CCM outputs
  double E_; // Riemannian energy
  Eigen::Vector4d delta_u_; // change of control signal
  double ft_dot_; // CCM computed thrust_dot
  Eigen::Vector3d euler_dot_; // CCM computed angular velocitioes in global intertia
  Eigen::Vector3d euler_dot_prev_;
  Eigen::Matrix3d R_wb_; // matrix co convert euler dot into angular velocity
  Eigen::Matrix3d R_euler_; // matrix co convert angular velocity into euler dot
  Eigen::Vector3d r_wb_; // CCM computed angular velocities in body frame
  double ft_cmd_; // thrust command

  // for simple thrust filtering
  double ft_dot_sum_;
  double filter_N_;
  // use for integration
  double dt_;
  // constants
  double g_;
  double lambda_;
  Eigen::MatrixXd B_ctrl_;

  // CCM controller funcs
  void calc_CCM_dyn(const Eigen::VectorXd &xc,const Eigen::Vector4d &uc,
                    Eigen::VectorXd &dyn); // compute dynamics
  void calc_xc_uc_nom(const Eigen::Vector3d &ref_pos,
                      const Eigen::Vector3d &ref_vel,
                      const Eigen::Vector3d &ref_acc,
                      const Eigen::Vector3d &ref_jer,
                      const double yaw_des,
                      const double yaw_dot_des); // compute xc nom, uc nom


public:
  CCMController(const double N=2.0);

  // copy in updated state into controller class
  void updateState(const Eigen::Vector3d &pos, const Eigen::Matrix3d &Rot,
                   const Eigen::Vector3d &vel, const Eigen::Vector3d &wb,
                   const double ft, const double dt, const bool pose_up, const bool vel_up, const bool ang_up, const bool thr_up);

  // compute control
  void calcCCM(
          const double yaw_des,
          const double yaw_dot_des,
          const Eigen::Vector3d &ref_pos,
          const Eigen::Vector3d &ref_vel,
          const Eigen::Vector3d &ref_acc,
          const Eigen::Vector3d &ref_jer);

  // return functions
  double getE();
  double getfz();
  Eigen::Vector3d getEr();
  Eigen::Vector3d getOm();
  Eigen::Vector3d getEuler();
  double getYawNom();
  // arm controller
  void setMode(const bool b);

};




#endif
