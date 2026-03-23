#include <CCM/CCMcontroller.h>

/********* Constructor ***********/
CCMController::CCMController(const double N):
  // fast initalize varables
   active_(false),
   mea_pos_(0.0,0.0,0.0), mea_vel_(0.0,0.0,0.0), mea_eul_(0.0,0.0,0.0), mea_wb_(0.0,0.0,0.0), 
   mea_ft_(9.8066),
   xc_nom_(10),uc_nom_(0.0,0.0,0.0,0.0),xc_nom_dot_(10),
   xc_(10),xc_dot_(10), 
   delta_x_g_(10),
   xc_mid_(10),
   h_(10),
   E_(0.0),
   delta_u_(0.0,0.0,0.0,0.0),
   ft_dot_(0.0),euler_dot_(0.0,0.0,0.0),euler_dot_prev_(0.0,0.0,0.0),r_wb_(0.0,0.0,0.0),
   ft_cmd_(9.8066),
   ft_dot_sum_(0.0),filter_N_(N), dt_(0.004), g_(9.8066), lambda_(1.28){
  // iniatlize rest
  //this->log_file.open("/home/kacper/Mgr/dron/straight_line_logs_ccm_unconstrained_no_ang_extrap_for_compare.txt", std::ios::out | std::ios::app);
  this->log_file.open("/home/kacper/Mgr/dron/straight_line_logs_ccm_temp.txt", std::ios::out | std::ios::app);

  this->mea_Rot_ = Eigen::Matrix3d::Identity();
  this->R_euler_ = Eigen::Matrix3d::Identity();
  this->R_wb_ = Eigen::Matrix3d::Identity();

  // inital states values
  this->xc_nom_.setZero(); this->xc_nom_(6) = g_;
  this->xc_.setZero(); this->xc_(6) = g_;
  this->xc_mid_.setZero(); this->xc_mid_(6) = g_;
  this->h_.setZero();

  // inital dynamics
  this->xc_nom_dot_.setZero();
  this->xc_dot_.setZero();

  // inital W matrices
  this->W_nom_ = Eigen::MatrixXd::Identity(9,9);
  this->W_mid_ = W_nom_;
  this->W_ = W_nom_;
  this->M_nom_ = Eigen::MatrixXd::Identity(10,10);
  this->M_mid_ = M_nom_;
  this->M_ = M_nom_;


  // inital M matrices
  double d_bar = 0.0420; // IMPORTANT -> from disruption
  double yaw_bound = 10.0*M_PI/180.0; // IMPORTANT -> yaw bound (0,1744 -> 10 deg) [-10, 10] deg 
  M_yaw_ = std::pow((d_bar/yaw_bound),2.0);
  this->M_nom_(9,9) = M_yaw_;
  this->M_mid_(9,9) = M_yaw_;
  this->M_(9,9) = M_yaw_;

  // inital control matrix
  this->B_ctrl_ = Eigen::MatrixXd(10,4);
  this->B_ctrl_ << Eigen::MatrixXd::Zero(6,4),
             Eigen::MatrixXd::Identity(4,4);

  // initialize controller
  setMode(false);

  // debug publishers
  #pragma region Debug_publishers
  this->DebugPubUpdate_ = this->nh.advertise<ccm_tracking_controller::Debug_Readings>("/debug_cccm_updates", 1000);
  this->DebugPubMeasured_ = this->nh.advertise<ccm_tracking_controller::Debug_Readings>("/debug_cccm_measured_state", 1000);
  this->DebugPubMeasured_dot_ = this->nh.advertise<ccm_tracking_controller::Debug_Readings>("/debug_cccm_measured_dot_state", 1000);
	this->DebugPubNominal_ = this->nh.advertise<ccm_tracking_controller::Debug_Readings>("/debug_cccm_nominal_state", 1000);
	this->DebugPubNominal_dot_ = this->nh.advertise<ccm_tracking_controller::Debug_Readings>("/debug_cccm_nominal_dot_state", 1000);
	this->DebugPubNominal_u_ = this->nh.advertise<ccm_tracking_controller::Debug_Readings>("/debug_cccm_nominal_u_state", 1000);
  this->DebugPubData_ = this->nh.advertise<ccm_tracking_controller::Debug_Readings>("/debug_cccm_calc", 1000);
  #pragma endregion Debug_publishers

  // show info
  std::cout << "[CCMcontroller]: CCM initialized " << std::endl;
}

void CCMController::setMode(const bool b){
  this->active_ = b;
  if (this->active_==false){
    // reset
    this->ft_cmd_ = this->g_;
    this->ft_dot_ = 0.0;
    this->delta_u_.setZero();
    this->euler_dot_.setZero();
    this->euler_dot_prev_.setZero();
  }
}

/********* State update ***********/
void CCMController::updateState(const Eigen::Vector3d &pos, const Eigen::Matrix3d &Rot,
                                const Eigen::Vector3d &vel, const Eigen::Vector3d &wb,
                                const double ft, const double dt, const bool pose_up, const bool vel_up, const bool ang_up, const bool thr_up) {
  // set timestamp
  this->dt_ = dt;

  // ====== update data from sources ======
  // position and orientation (map frame) from source
  if (pose_up) { 
    this->mea_pos_ = pos;
    this->mea_Rot_ = Rot;
    ccmutils::R2euler_123(this->mea_Rot_, this->mea_eul_);
  }
  // thrust (linear acceleration) (body frame) from source
  if (thr_up) {
    this->mea_ft_ = ft;
  }
  // linear velocity (map frame) from source
  if (vel_up) {
    this->mea_vel_ = vel;
  }
  // angular velocity (body frame) (and euler dot in map frame) from source
  if (ang_up) {  
    this->mea_wb_ = wb;
  }

  // ====== extrapolate data from control signal ======
  // extrapolate thrust 
  if (!thr_up) {
    this->mea_ft_ = this->ft_cmd_;
  } 
  // extrapolate orientation
  if (!pose_up && this->active_) {
    Eigen::Vector3d euler_accel = (this->euler_dot_ - this->euler_dot_prev_)/this->dt_;
    this->mea_eul_ += this->euler_dot_*dt + 0.5*euler_accel*std::pow(this->dt_,2);
    for (int i = 0; i<3; i++){
      this->mea_eul_(i) = std::atan2(std::sin(mea_eul_(i)),std::cos(mea_eul_(i)));
    }
    ccmutils::Euler2R_123(this->mea_eul_, this->mea_Rot_);
  } 
  // extrapolate angular velocity
  if (!ang_up && this->active_) {
    this->mea_wb_ = this->r_wb_;
  }
  //====================================NED==================================================
  
  // extrapolate velocity
  if (!vel_up && this->active_) {
    Eigen::Vector3d accel = -this->mea_ft_*this->mea_Rot_.col(2);
    // add grav effect (remove compensation)
    accel(2) += this->g_;
    this->mea_vel_ += accel*this->dt_;
  } 
  // extrapolate position
  if (!pose_up && this->active_) {
    Eigen::Vector3d accel = -this->mea_ft_*this->mea_Rot_.col(2);
    // add grav effect (remove compensation)
    accel(2) += this->g_;
    this->mea_pos_ += this->mea_vel_*this->dt_ + 0.5*accel*std::pow(this->dt_,2);
  }
  
  //====================================ENU==================================================
  /*
  // extrapolate velocity
  if (!vel_up && this->active_) {
    Eigen::Vector3d accel = this->mea_ft_*this->mea_Rot_.col(2);
    // add grav effect
    accel(2) += -this->g_;
    this->mea_vel_ += accel*this->dt_;
  } 
  // extrapolate position
  if (!pose_up && this->active_) {
    Eigen::Vector3d accel = this->mea_ft_*this->mea_Rot_.col(2);
    // add grav effect
    accel(2) += -this->g_;
    this->mea_pos_ += this->mea_vel_*this->dt_ + 0.5*accel*std::pow(this->dt_,2);
  }
  */
  //====================================END==================================================

  // show info
  if (this->verbose_){
    /*
    std::cout << "[ccmcontroller]: Updated state!" <<"Type: "<< this->active_<<", pose: " << pose_up<<", vel: " << vel_up<<", ang: " << ang_up<<", time: " << this->dt_ <<std::endl;
    std::cout << "[ccmcontroller]: Measured force: " << this->ft_ << std::endl;
    std::cout << "[ccmcontroller]: Measured pose: " << this->mea_pos_(0) <<", "<< this->mea_pos_(1) <<", "<< this->mea_pos_(2) << std::endl;
    std::cout << "[ccmcontroller]: Measured rpy: " << this->mea_eul_(0) <<", "<< this->mea_eul_(1) <<", "<< this->mea_eul_(2) << std::endl;
    std::cout << "[ccmcontroller]: Measured velocity: " << this->mea_vel_(0) <<", "<< this->mea_vel_(1) <<", "<< this->mea_vel_(2) << std::endl;    
    */
  }
  
  // send data to debug
  #pragma region Debug_data_state
  ccm_tracking_controller::Debug_Readings updateMsg;
  updateMsg.header.stamp = ros::Time::now();
  updateMsg.p_x = this->mea_pos_(0);
  updateMsg.p_y = this->mea_pos_(1);
  updateMsg.p_z = this->mea_pos_(2);

  updateMsg.v_x = this->mea_vel_(0);
  updateMsg.v_y = this->mea_vel_(1);
  updateMsg.v_z = this->mea_vel_(2);

  updateMsg.thr = this->mea_ft_;

  updateMsg.eul_1 = this->mea_eul_(0);
  updateMsg.eul_2 = this->mea_eul_(1);
  updateMsg.eul_3 = this->mea_eul_(2);

  updateMsg.rwb_1 = this->mea_wb_(0);
  updateMsg.rwb_2 = this->mea_wb_(1);
  updateMsg.rwb_3 = this->mea_wb_(2);

  updateMsg.eul_dot_1 = this->euler_dot_(0);
  updateMsg.eul_dot_2 = this->euler_dot_(1);
  updateMsg.eul_dot_3 = this->euler_dot_(2);

  this->DebugPubUpdate_.publish(updateMsg);
  #pragma endregion Debug_data_state
}

/********* CCM fncs ***********/
void CCMController::calc_xc_uc_nom(const Eigen::Vector3d &ref_pos,
                const Eigen::Vector3d &ref_vel,
                const Eigen::Vector3d &ref_acc,
                const Eigen::Vector3d &ref_jer,
                const double yaw_des,
                const double yaw_dot_des){

  // postion -> from given
  this->xc_nom_(0) = ref_pos(0); this->xc_nom_(1) = ref_pos(1); this->xc_nom_(2) = ref_pos(2);
  // velocity -> from given
  this->xc_nom_(3) = ref_vel(0); this->xc_nom_(4) = ref_vel(1); this->xc_nom_(5) = ref_vel(2);

  //====================================NED==================================================
  
  // thrust -> norm of acceleration
  // calculate thrust force (add compensation for grav element)
  Eigen::Vector3d th_vec(ref_acc(0), ref_acc(1), ref_acc(2)-this->g_);
  this->xc_nom_(6) = th_vec.norm(); // czy to nie powinno się przekonwertować na układ body? bo teraz jest chyba global -> jest ok, chemy żeby z_B sie pokrywało z 3D wektorem przyspieszenia
  // z axis direction from thrust
  Eigen::Vector3d zb = -th_vec.normalized();
  // thrust change 
  this->uc_nom_(0) = th_vec.normalized().dot(ref_jer);   
  
  //====================================ENU==================================================
  /*
  // thrust -> norm of acceleration
  // calculate thrust force (add compensation for grav element)
  Eigen::Vector3d th_vec(ref_acc(0), ref_acc(1), ref_acc(2)+this->g_);
  this->xc_nom_(6) = th_vec.norm();
  // z axis direction from thrust
  Eigen::Vector3d zb = th_vec.normalized();
  // thrust change 
  this->uc_nom_(0) = th_vec.normalized().dot(ref_jer); 
  */
  //====================================END==================================================

  // rpy -> directions (yc is from R_z column)
  Eigen::Vector3d yc(-std::sin(yaw_des),std::cos(yaw_des),0.0);
  Eigen::Vector3d xb_des = yc.cross(zb);
  Eigen::Vector3d xb = xb_des.normalized();
  Eigen::Vector3d yb = zb.cross(xb);
  Eigen::Matrix3d R_ref;   
  R_ref.col(0) = xb;
  R_ref.col(1) = yb;
  R_ref.col(2) = zb;

  // get euler angles
  Eigen::Vector3d desired_euler;        
  ccmutils::R2euler_123(R_ref, desired_euler);
  this->xc_nom_(7) = desired_euler(0);
  this->xc_nom_(8) = desired_euler(1);
  this->xc_nom_(9) = desired_euler(2);
  // get matrices        
  Eigen::Matrix3d R_euler_ref; 
  Eigen::Matrix3d R_wb_ref; 
  ccmutils::R_eul(desired_euler, R_euler_ref);
  ccmutils::R_wb(desired_euler, R_wb_ref);

  // calculate angular velocity                      
  double om_x = 0.0, om_y= 0.0, om_z = 0.0;
  if (this->xc_nom_(6) > 0.0) {
    om_x = (1.0/this->xc_nom_(6)) * ( ref_jer.dot(R_ref.col(1)) ); // why not:  om_x = -(1.0/this->xc_nom_(6)) * ( ref_jer.dot(R_ref.col(1)) );
    om_y = -(1.0/this->xc_nom_(6)) * ( ref_jer.dot(R_ref.col(0)) );  // why not: om_y = (1.0/this->xc_nom_(6)) * ( ref_jer.dot(R_ref.col(0)) );
  }

  // from R_eul
  double roll_d = om_x*(std::cos(this->xc_nom_(9))/std::cos(this->xc_nom_(8))) - om_y*(std::sin(this->xc_nom_(9))/std::cos(this->xc_nom_(8)));
  // from R_om
  om_z = yaw_dot_des + roll_d*std::sin(this->xc_nom_(8));

  // combine
  Eigen::Vector3d om_desired(om_x, om_y, om_z);
  
  // calculate euler_dot
  Eigen::Vector3d euler_dot_desired = R_euler_ref * om_desired;
  this->uc_nom_(1) = euler_dot_desired(0);
  this->uc_nom_(2) = euler_dot_desired(1);
  this->uc_nom_(3) = euler_dot_desired(2);


  #pragma region Debug_nominal_state
  ccm_tracking_controller::Debug_Readings nominalMsg;
  nominalMsg.header.stamp = ros::Time::now();
  nominalMsg.p_x = this->xc_nom_(0);
  nominalMsg.p_y = this->xc_nom_(1);
  nominalMsg.p_z = this->xc_nom_(2);

  nominalMsg.v_x = this->xc_nom_(3);
  nominalMsg.v_y = this->xc_nom_(4);
  nominalMsg.v_z = this->xc_nom_(5);

  nominalMsg.thr = this->xc_nom_(6);

  nominalMsg.eul_1 = this->xc_nom_(7);
  nominalMsg.eul_2 = this->xc_nom_(8);
  nominalMsg.eul_3 = this->xc_nom_(9);

  nominalMsg.yaw_nom = yaw_des;

  nominalMsg.rwb_1 = om_x;
  nominalMsg.rwb_2 = om_y;
  nominalMsg.rwb_3 = om_z;

  nominalMsg.eul_dot_1 = euler_dot_desired(0);
  nominalMsg.eul_dot_2 = euler_dot_desired(1);
  nominalMsg.eul_dot_3 = euler_dot_desired(2);

  this->DebugPubNominal_.publish(nominalMsg);
  #pragma endregion Debug_nominal_state

}

void CCMController::calc_CCM_dyn(const Eigen::VectorXd &xc,const Eigen::Vector4d &uc,
                                  Eigen::VectorXd &dyn){ 
  // calculate dynamics from equation (and R.col(2) matrix)
  Eigen::VectorXd f(10);
  //Eigen::Vector3d b_T(sin(p), -cos(p)*sin(r), cos(p)*cos(r));
  //====================================NED==================================================
  
  f << xc(3), xc(4), xc(5),                                                                                             // so state velocity is in inertial frame
      -xc(6)*std::sin(xc(8)), xc(6)*std::cos(xc(8))*std::sin(xc(7)), this->g_-(xc(6)*std::cos(xc(8))*std::cos(xc(7))),  // so state thrust force is in body frame
      0.0,0.0,0.0,0.0; 
  
  //====================================ENU==================================================
  /*
  f << xc(3), xc(4), xc(5),                                                                                             // so state velocity is in inertial frame
      xc(6)*std::sin(xc(8)), -xc(6)*std::cos(xc(8))*std::sin(xc(7)), (xc(6)*std::cos(xc(8))*std::cos(xc(7)))-this->g_,  // so state thrust force is in body frame
      0.0,0.0,0.0,0.0; 
  */
      //====================================END==================================================
  dyn = f + this->B_ctrl_*uc;
}

/********* CCM compute ***********/
void CCMController::calcCCM(const double yaw_des, const double yaw_dot_des, const Eigen::Vector3d &ref_pos, const Eigen::Vector3d &ref_vel,
          const Eigen::Vector3d &ref_acc, const Eigen::Vector3d &ref_jer) {
  double a = 0.0; Eigen::Vector4d b(0.0,0.0,0.0,0.0);
  // perform calculations          
  if (this->active_){
    // compute nominal xc and uc
    calc_xc_uc_nom(ref_pos, ref_vel, ref_acc, ref_jer, yaw_des, yaw_dot_des);

    // compute actual xc
    this->xc_ << this->mea_pos_, this->mea_vel_, this->mea_ft_, this->mea_eul_;   // ft_cmd_ or mea_ft_ ?

    // normalize yaw to [-pi, pi] to be sure 
    this->xc_nom_(9) = std::atan2(std::sin(xc_nom_(9)),std::cos(xc_nom_(9)));
    this->xc_(9) = std::atan2(std::sin(xc_(9)),std::cos(xc_(9)));

    // Straight-line appproximation of geodesic
    this->delta_x_g_ = this->xc_ - this->xc_nom_;

    // state between current and nominal
    this->xc_mid_ = 0.5 * (this->xc_nom_ + this->xc_);

    // Compute M for ccm
    geodesic::compute_W(this->xc_nom_, this->W_nom_);
    geodesic::compute_W(this->xc_mid_, this->W_mid_);
    geodesic::compute_W(this->xc_, this->W_);
    this->M_nom_.block(0,0,9,9) = this->W_nom_.inverse();
    this->M_mid_.block(0,0,9,9) = this->W_mid_.inverse();
    this->M_.block(0,0,9,9) = this->W_.inverse();

    // calculate dynamics for states
    calc_CCM_dyn(this->xc_nom_, this->uc_nom_, this->xc_nom_dot_);
    calc_CCM_dyn(this->xc_, this->uc_nom_, this->xc_dot_);

    // Riemannian energy -> simpson method h=0.5*(1-0) because ds from 0 to 1
    this->E_ = 0.5*((1.0/3.0)*this->delta_x_g_.dot(this->M_nom_*this->delta_x_g_)+
              (4.0/3.0)*this->delta_x_g_.dot(this->M_mid_*this->delta_x_g_)+
              (1.0/3.0)*this->delta_x_g_.dot(this->M_*this->delta_x_g_));

    // QP inequality solution
    a = 2.0*this->lambda_*this->E_ - 2.0*this->delta_x_g_.dot(this->M_nom_*this->xc_nom_dot_)+
                              2.0*this->delta_x_g_.dot(this->M_*this->xc_dot_);
    b = 2.0*this->B_ctrl_.transpose()*this->M_*this->delta_x_g_; 

    // analytical QP solution
    //1.0e-8
    if ((a > 0.0) && (b.norm() > 0.0)){
      this->delta_u_ = (this->delta_u_/2.0) - ( (b*(a+(b.dot(this->delta_u_)/2.0)))/((b.norm()*b.norm())+0.0) ); 
    } else {
      this->delta_u_.setZero();
    }

    // calculate thrust

    // set constraints on delta_u thrust
    double delta_constraint_thrust = 2*this->g_; // less than 0.2 causes jumps in matlab
    this->delta_u_(0) = std::max(std::min(this->delta_u_(0), delta_constraint_thrust),-delta_constraint_thrust);
    // avg filter
    this->ft_dot_ = (1.0 - (1.0/this->filter_N_))*this->ft_dot_ + (1.0/this->filter_N_)*(this->uc_nom_(0)+this->delta_u_(0));
    // update thrust and set constraints (0.5, 2)g
    this->ft_cmd_ = this->mea_ft_ + this->ft_dot_*this->dt_;                    // ft_cmd_ or mea_ft_ ?
    this->ft_cmd_ = std::max(std::min(this->ft_cmd_, 2.0*9.8066),0.5*9.8066);

    // calculate euler rates

    // set constraints on delta_u rates
    double delta_constraint_rates = (1.0/6.0)*M_PI; 
    for (int i = 1; i<4; i++){
      this->delta_u_(i) = std::max(std::min(this->delta_u_(i), delta_constraint_rates),-delta_constraint_rates);
    }
    // update desired euler dot and set constraint (-60deg/s, 60deg/s as px4 suggest in docs)
    double constraint_rates = (1.0/3.0)*M_PI;
    this->euler_dot_prev_= this->euler_dot_;
    for (int i = 0; i<3; i++){
      this->euler_dot_(i) = std::max(std::min(this->uc_nom_(i+1)+this->delta_u_(i+1),constraint_rates),-constraint_rates);
    }

  } else {
    this->ft_cmd_ = this->g_;
    this->ft_dot_ = 0.0;
    this->delta_u_.setZero();
    this->euler_dot_.setZero();
    this->euler_dot_prev_.setZero();
  }

  // convert to desired angular velocity in body frame
  ccmutils::R_wb(this->mea_eul_, this->R_wb_);
  this->r_wb_ = this->R_wb_*this->euler_dot_;


  // show info
  if(this->verbose_){
    this->log_file << "[CCMcontroller]: Nominal state info: " << ros::Time::now() << std::endl;
    this->log_file << "[CCMcontroller]: Nominal state position: " << this->xc_nom_(0) <<", "<< this->xc_nom_(1) <<", "<< this->xc_nom_(2) << std::endl;
    this->log_file << "[CCMcontroller]: Nominal state velocity: " << this->xc_nom_(3) <<", "<< this->xc_nom_(4) <<", "<< this->xc_nom_(5) << std::endl;
    this->log_file << "[CCMcontroller]: Nominal state thrust force: " << this->xc_nom_(6) << std::endl;
    this->log_file << "[CCMcontroller]: Nominal state rpy: " << this->xc_nom_(7) <<", "<< this->xc_nom_(8) <<", "<< this->xc_nom_(9) << std::endl;
    this->log_file << "[CCMcontroller]: Nominal state commands (thrust, rates): " << this->uc_nom_(0) <<", "<< this->uc_nom_(1) <<", "<< this->uc_nom_(2)<< ", "<< this->uc_nom_(3)  << std::endl;
    this->log_file << "[CCMcontroller]: Nominal state dynamics: " << this->xc_nom_dot_(0) <<", "<< this->xc_nom_dot_(1) <<", "<< this->xc_nom_dot_(2)<< ", "<< this->xc_nom_dot_(3)<<", "<< this->xc_nom_dot_(4) <<", "<< this->xc_nom_dot_(5) <<", "<< this->xc_nom_dot_(6)<< ", "<< this->xc_nom_dot_(7)<<", "<< this->xc_nom_dot_(8) <<", "<< this->xc_nom_dot_(9) << std::endl;
    this->log_file << "[CCMcontroller]: Measured state info" << std::endl;
    this->log_file << "[CCMcontroller]: Measured state position: " << this->xc_(0) <<", "<< this->xc_(1) <<", "<< this->xc_(2) << std::endl;
    this->log_file << "[CCMcontroller]: Measured state velocity: " << this->xc_(3) <<", "<< this->xc_(4) <<", "<< this->xc_(5) << std::endl;
    this->log_file << "[CCMcontroller]: Measured state thrust force: " << this->xc_(6) << std::endl;
    this->log_file << "[CCMcontroller]: Measured state rpy: " << this->xc_(7) <<", "<< this->xc_(8) <<", "<< this->xc_(9) << std::endl;
    this->log_file << "[CCMcontroller]: Measured state dynamics: " << this->xc_dot_(0) <<", "<< this->xc_dot_(1) <<", "<< this->xc_dot_(2)<<", "<< this->xc_dot_(3)<<", "<< this->xc_dot_(4) <<", "<< this->xc_dot_(5) <<", "<< this->xc_dot_(6)<< ", "<< this->xc_dot_(7)<<", "<< this->xc_dot_(8) <<", "<< this->xc_dot_(9) << std::endl;
    this->log_file << "[CCMcontroller]: Calculated command info" << std::endl;
    this->log_file << "[CCMcontroller]: Calculated commands delta (thrust, rates): " << this->delta_u_(0) <<", "<< this->delta_u_(1) <<", "<< this->delta_u_(2)<< ", "<< this->delta_u_(3)  << std::endl;
    this->log_file << "[CCMcontroller]: Calculated euler rates: " << this->euler_dot_(0) <<", "<< this->euler_dot_(1) <<", "<< this->euler_dot_(2) << std::endl;
    this->log_file << "[CCMcontroller]: Calculated body rates: " << this->r_wb_(0) <<", "<< this->r_wb_(1) <<", "<< this->r_wb_(2) << std::endl;
    this->log_file << "[CCMcontroller]: Calculated d_f_thrust, thrust, thrust percent: "<< this->ft_dot_ <<", "<< this->ft_cmd_ <<", "<< std::min(1.0, std::max(0.0, 0.7 * ((this->ft_cmd_) / 9.8066))) << std::endl;
    this->log_file << "==== " << std::endl;
    this->log_file.flush();
  }

  // send data to debug
  #pragma region Debug_data_calculate
  ccm_tracking_controller::Debug_Readings dataMsg;
  dataMsg.header.stamp = ros::Time::now();

  dataMsg.dfc = this->delta_u_(0);
  dataMsg.a = a;
  dataMsg.b_norm = b.norm();
  dataMsg.b_1 = b(0);
  dataMsg.b_2 = b(1);
  dataMsg.b_3 = b(2);
  dataMsg.b_4 = b(3);
  dataMsg.filtered_f_dot = this->ft_dot_;

  dataMsg.E = this->E_;

  dataMsg.thr = this->ft_cmd_;

  dataMsg.d_eul_dot_1 = this->delta_u_(1);
  dataMsg.d_eul_dot_2 = this->delta_u_(2);
  dataMsg.d_eul_dot_3 = this->delta_u_(3);

  dataMsg.eul_dot_1 = this->euler_dot_(0);
  dataMsg.eul_dot_2 = this->euler_dot_(1);
  dataMsg.eul_dot_3 = this->euler_dot_(2);

  dataMsg.rwb_1 = this->r_wb_(0);
  dataMsg.rwb_2 = this->r_wb_(1);
  dataMsg.rwb_3 = this->r_wb_(2);

  
  this->DebugPubData_.publish(dataMsg);


  ccm_tracking_controller::Debug_Readings nominaldotMsg;
  nominaldotMsg.header.stamp = ros::Time::now();
  nominaldotMsg.p_x = this->xc_nom_dot_(0);
  nominaldotMsg.p_y = this->xc_nom_dot_(1);
  nominaldotMsg.p_z = this->xc_nom_dot_(2);

  nominaldotMsg.v_x = this->xc_nom_dot_(3);
  nominaldotMsg.v_y = this->xc_nom_dot_(4);
  nominaldotMsg.v_z = this->xc_nom_dot_(5);

  nominaldotMsg.dfc = this->xc_nom_dot_(6);

  nominaldotMsg.eul_1 = this->xc_nom_dot_(7);
  nominaldotMsg.eul_2 = this->xc_nom_dot_(8);
  nominaldotMsg.eul_3 = this->xc_nom_dot_(9);
  this->DebugPubNominal_dot_.publish(nominaldotMsg);

  ccm_tracking_controller::Debug_Readings nominaluMsg;
  nominaluMsg.header.stamp = ros::Time::now();
  nominaluMsg.dfc = this->uc_nom_(0);

  nominaluMsg.eul_dot_1 = this->uc_nom_(1);
  nominaluMsg.eul_dot_2 = this->uc_nom_(2);
  nominaluMsg.eul_dot_3 = this->uc_nom_(3);

  this->DebugPubNominal_u_.publish(nominaluMsg);

  ccm_tracking_controller::Debug_Readings measuredMsg;
  measuredMsg.header.stamp = ros::Time::now();
  measuredMsg.p_x = this->xc_(0);
  measuredMsg.p_y = this->xc_(1);
  measuredMsg.p_z = this->xc_(2);

  measuredMsg.v_x = this->xc_(3);
  measuredMsg.v_y = this->xc_(4);
  measuredMsg.v_z = this->xc_(5);

  measuredMsg.thr = this->xc_(6);

  measuredMsg.eul_1 = this->xc_(7);
  measuredMsg.eul_2 = this->xc_(8);
  measuredMsg.eul_3 = this->xc_(9);

  this->DebugPubMeasured_.publish(measuredMsg);

  ccm_tracking_controller::Debug_Readings measureddotMsg;
  measureddotMsg.header.stamp = ros::Time::now();
  measureddotMsg.p_x = this->xc_dot_(0);
  measureddotMsg.p_y = this->xc_dot_(1);
  measureddotMsg.p_z = this->xc_dot_(2);

  measureddotMsg.v_x = this->xc_dot_(3);
  measureddotMsg.v_y = this->xc_dot_(4);
  measureddotMsg.v_z = this->xc_dot_(5);

  measureddotMsg.thr = this->xc_dot_(6);

  measureddotMsg.eul_1 = this->xc_dot_(7);
  measureddotMsg.eul_2 = this->xc_dot_(8);
  measureddotMsg.eul_3 = this->xc_dot_(9);
  this->DebugPubMeasured_dot_.publish(measureddotMsg);
  #pragma endregion Debug_data_calculate
}

/********* Return fncs ***********/

double CCMController::getE(){
  // contraction energy along geodesis
  return this->E_;
}

double CCMController::getfz(){
  // commanded thrust force
  return this->ft_cmd_;
}

Eigen::Vector3d CCMController::getEr(){
  // euler angles change in global frame
  return this->euler_dot_;
}

Eigen::Vector3d CCMController::getOm(){
  // angular velocity in body frame
  return this->r_wb_;
}

Eigen::Vector3d CCMController::getEuler(){
  // measured orientation in euler angles (inertial frame)
  return this->mea_eul_;
}

double CCMController::getYawNom(){
  // target yaw angle
  return this->xc_nom_(9);
}
