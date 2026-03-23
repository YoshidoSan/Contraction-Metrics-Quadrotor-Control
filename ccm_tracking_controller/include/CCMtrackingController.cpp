/*
	FILE: CCMtrackingController.cpp
	-------------------------------
	function implementation of px4 CCM tracking controller
*/

#include <CCMtrackingController.h>

namespace ccmtracking{
	CCMtrackingController::CCMtrackingController(const ros::NodeHandle& nh) : nh_(nh){
		this->initParamModules();
		this->registerPub();
		this->registerCallback();
	}

	void CCMtrackingController::initParamModules(){
		
		// Get params
		//this->log_file.open("/home/kacper/Mgr/dron/straight_line_commands_ccm_unconstrained_no_ang_extrap_for_compare.txt", std::ios::out | std::ios::app);
		this->log_file.open("/home/kacper/Mgr/dron/straight_line_commands_ccm_temp.txt", std::ios::out | std::ios::app);

		// state estimator filter value
		if (not this->nh_.getParam("ccm_controller/FZ_EST_N", this->FZ_EST_N_)){
			this->FZ_EST_N_ = 8.0;
			std::cout << "[CCMtrackingController]: FZ_EST_N use default: 8.0." << std::endl;
		}
		else{
			std::cout << "[CCMtrackingController]: FZ_EST_N is set to: " << this->FZ_EST_N_  << std::endl;
		}
		// controller filter value
		if (not this->nh_.getParam("ccm_controller/FZ_CTRL_N", this->FZ_CTRL_N_)){
			this->FZ_CTRL_N_ = 2.0;
			std::cout << "[CCMtrackingController]: FZ_CTRL_N use default: 2.0." << std::endl;
		}
		else{
			std::cout << "[CCMtrackingController]: FZ_CTRL_N is set to: " << this->FZ_CTRL_N_  << std::endl;
		}
		// hover thrust value
		if (not this->nh_.getParam("ccm_controller/hover_thrust", this->hoverThrust_)){
			this->hoverThrust_ = 0.4;
			std::cout << "[CCMtrackingController]: No hover thrust param. Use default: 0.4." << std::endl;
		}
		else{
			std::cout << "[CCMtrackingController]: Hover thrust is set to: " << this->hoverThrust_  << std::endl;
		}
		// hover thrust value
		if (not this->nh_.getParam("ccm_controller/K_THR", this->K_THR_)){
			this->K_THR_ = 1.0;
			std::cout << "[CCMtrackingController]: No thrust gain param. Use default: 1.0." << std::endl;
		}
		else{
			std::cout << "[CCMtrackingController]: Thrust gain is set to: " << this->K_THR_  << std::endl;
		}
		// display messages
		if (not this->nh_.getParam("ccm_controller/verbose", this->verbose_)){
			this->verbose_ = false;
			std::cout << "[CCMtrackingController]: No display message param. Use default: false." << std::endl;
		}
		else{
			std::cout << "[CCMtrackingController]: Display message is set to: " << this->verbose_  << std::endl;
		}
		
		// ccmcontroller
		this->ctrl_ = std::make_unique<CCMController>(this->FZ_CTRL_N_);
		this->ctrl_->setMode(false);

		// sensor readings initial values
		this->mea_q_.setZero();
		this->mea_R_ = Eigen::Matrix3d::Identity();
		this->mea_wb_.setZero();
		this->mea_pos_.setZero();
		this->mea_vel_.setZero();
		this->mea_accel_.setZero();
		this->vel_prev_.setZero();
		this->vel_prev_t_ = 0.0;
		this->fz_est_ = 9.8066;
		// ekf reading initial values
		this->mea_q_ekf_.setZero();
		this->mea_R_ekf_ = Eigen::Matrix3d::Identity();
		this->mea_wb_ekf_.setZero();
		this->mea_pos_ekf_.setZero();
		this->mea_vel_ekf_.setZero();
		this->mea_accel_ekf_.setZero();
		this->vel_prev_ekf_.setZero();
		this->vel_prev_t_ekf_ = 0.0;
		this->fz_est_ekf_ = 9.8066;
	}

	void CCMtrackingController::registerPub(){
		// command publisher
		this->cmdPub_ = this->nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 100);
		
		// visualization - current pose publisher
		this->poseVisPub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/tracking_controller/robot_pose", 1);

		// visualization - trajectory history publisher
		this->histTrajVisPub_ = this->nh_.advertise<nav_msgs::Path>("/tracking_controller/trajectory_history", 1);

		// visualization - target pose publisherher
		this->targetVisPub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/tracking_controller/target_pose", 1);
	
		// visualization - target trajectory history publisher
		this->targetHistTrajVisPub_ = this->nh_.advertise<nav_msgs::Path>("/tracking_controller/target_trajectory_history", 1); 

		// visualization - velocity and acceleration  publisher
		this->velAndAccVisPub_ = this->nh_.advertise<visualization_msgs::Marker>("/tracking_controller/vel_and_acc_info", 1);

		//debug info
		#pragma region Debug_publishers
		this->DebugPubTarget_ = this->nh_.advertise<ccm_tracking_controller::Debug_Readings>("/debug_target", 1000);
		this->DebugPubTargetUnconv_ = this->nh_.advertise<ccm_tracking_controller::Debug_Readings>("/debug_target_unconverted", 1000);
		this->DebugPubReadings_ = this->nh_.advertise<ccm_tracking_controller::Debug_Readings>("/debug_readings", 1000);
		this->DebugPubReadingsUnconv_ = this->nh_.advertise<ccm_tracking_controller::Debug_Readings>("/debug_readings_unconverted", 1000);
		this->DebugPubCommand_ = this->nh_.advertise<ccm_tracking_controller::Debug_Readings>("/debug_command", 1000);
		this->DebugEkfReadings_ = this->nh_.advertise<ccm_tracking_controller::Debug_Readings>("/debug_ekf", 1000);
		#pragma endregion Debug_publishers
	}

	void CCMtrackingController::registerCallback(){
		// odom subscriber -> ground truth odom of the quadcopter (position, orientation, velocities)
		this->odomSub_ = this->nh_.subscribe("mavros/local_position/odom", 1, &CCMtrackingController::odomCB, this);

		// ekf subscriber -> faster state extrapolation (pose, orientation, body linear velocity, body angular velocity)
		this->ekfSub_ = this->nh_.subscribe("ekf/state", 1, &CCMtrackingController::ekfCB, this);

		// ekf acceleration subscriber -> faster state extrapolation (body linear acceleration -> thrust)
		this->ekfaccelSub_ = this->nh_.subscribe("ekf/accel", 1, &CCMtrackingController::ekfaccelCB, this);

		// pose subscriber -> ground truth pose of the quadcopter (position, orientation)
		this->poseSub_ = this->nh_.subscribe("mavros/local_position/pose", 1, &CCMtrackingController::poseCB, this);

		// vel subscriber -> ground truth pose of the quadcopter (linear velocity, angular velocity) (global frame)
		this->velSub_ = this->nh_.subscribe("mavros/local_position/velocity_local", 1, &CCMtrackingController::velCB, this);

		// ang vel subscriber -> body frame linear velocity, angular velocity
		this->angSub_ = this->nh_.subscribe("mavros/local_position/velocity_body", 1, &CCMtrackingController::angCB, this);

		// imu subscriber
		this->imuSub_ = this->nh_.subscribe("mavros/imu/data", 1, &CCMtrackingController::imuCB, this);

		// target setpoint subscriber
		this->targetSub_ = this->nh_.subscribe("/autonomous_flight/target_state", 1, &CCMtrackingController::targetCB, this);
		
		// controller publisher timer
		this->cmdTimer_ = this->nh_.createTimer(ros::Duration(0.004), &CCMtrackingController::cmdCB, this);

		// auto thrust esimator timer -> zmiana estymaty 'hover thrust' gdy ostatnie 10 pomiarów jest w różnicy < 0.05 -> too fast and it will fail (or elongate list)
		this->thrustEstimatorTimer_ = this->nh_.createTimer(ros::Duration(0.01), &CCMtrackingController::thrustEstimateCB, this);

		// visualization timer
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.033), &CCMtrackingController::visCB, this);
	}

	void CCMtrackingController::ekfCB(const nav_msgs::OdometryConstPtr& ekf){
		this->ekf_ = *ekf;
		// gives pose and orientation in inertial and velocities in body frame
		//====================================NED==================================================
		
		// position and orientation in global NED frame
		this->mea_pos_ekf_(0) = this->ekf_.pose.pose.position.y;
		this->mea_pos_ekf_(1) = this->ekf_.pose.pose.position.x;
		this->mea_pos_ekf_(2) = -this->ekf_.pose.pose.position.z;
		// ROS quaternion
		this->mea_q_ekf_(0) = this->ekf_.pose.pose.orientation.w;
		this->mea_q_ekf_(1) = this->ekf_.pose.pose.orientation.x;
		this->mea_q_ekf_(2) = this->ekf_.pose.pose.orientation.y;
		this->mea_q_ekf_(3) = this->ekf_.pose.pose.orientation.z;
		// convert to matrix
		ccmutils::quat2rotM(this->mea_q_ekf_, this->mea_R_ekf_);
		// adjust around z by -90 deg ENU -> partial from body FLU to body FRD so axis are parrarel (x'=y, y'=-x, z'=z)
		this->mea_R_ekf_ = this->mea_R_ekf_ * this->Rz_T_;
		// extract ENU encoding of PX4 quat
		ccmutils::rotM2quat(this->mea_q_ekf_, this->mea_R_ekf_);
		// convert to NED -> switch axis -> now -90 ENU is 90 NED
		double q_w = this->mea_q_ekf_(0);
		double q_x = this->mea_q_ekf_(2);
		double q_y = this->mea_q_ekf_(1);
		double q_z = -this->mea_q_ekf_(3);
		this->mea_q_ekf_ << q_w, q_x, q_y, q_z;
		// final conversion to rot matrix (body to inertia) (v_global = R * v_local)
		ccmutils::quat2rotM(this->mea_q_ekf_, this->mea_R_ekf_);
		// set recevied
		this->pose_up_ekf_ = true;
		// angular velocity body in ned frame
		// set angular velocities
		this->mea_wb_ekf_(0) = this->ekf_.twist.twist.angular.x;
		this->mea_wb_ekf_(1) = -this->ekf_.twist.twist.angular.y;
		this->mea_wb_ekf_(2) = -this->ekf_.twist.twist.angular.z;
		// set recevied
		this->ang_up_ekf_ = true;
		// transform to local NED
		this->mea_vel_ekf_(0) = this->ekf_.twist.twist.linear.x;
		this->mea_vel_ekf_(1) = -this->ekf_.twist.twist.linear.y;
		this->mea_vel_ekf_(2) = -this->ekf_.twist.twist.linear.z;
		// transform to global NED
		this->mea_vel_ekf_ = this->mea_R_ekf_ * this->mea_vel_ekf_;
		// set recevied
		this->vel_up_ekf_ = true;
		//====================================ENU==================================================
		/*
		this->mea_pos_ekf_(0) = this->ekf_.pose.pose.position.x;
		this->mea_pos_ekf_(1) = this->ekf_.pose.pose.position.y;
		this->mea_pos_ekf_(2) = this->ekf_.pose.pose.position.z;
		// ROS quaternion
		this->mea_q_ekf_(0) = this->ekf_.pose.pose.orientation.w;
		this->mea_q_ekf_(1) = this->ekf_.pose.pose.orientation.x;
		this->mea_q_ekf_(2) = this->ekf_.pose.pose.orientation.y;
		this->mea_q_ekf_(3) = this->ekf_.pose.pose.orientation.z;
		// convert to matrix
		ccmutils::quat2rotM(this->mea_q_ekf_, this->mea_R_ekf_);
		// set recevied
		this->pose_up_ekf_ = true;
		// set angular velocities
		this->mea_wb_ekf_(0) = this->ekf_.twist.twist.angular.x;
		this->mea_wb_ekf_(1) = this->ekf_.twist.twist.angular.y;
		this->mea_wb_ekf_(2) = this->ekf_.twist.twist.angular.z;
		// set recevied
		this->ang_up_ekf_ = true;
		this->mea_vel_ekf_(0) = this->ekf_.twist.twist.linear.x;
		this->mea_vel_ekf_(1) = this->ekf_.twist.twist.linear.y;
		this->mea_vel_ekf_(2) = this->ekf_.twist.twist.linear.z;
		// transform to global 
		this->mea_vel_ekf_ = this->mea_R_ekf_ * this->mea_vel_ekf_;
		// set recevied
		this->vel_up_ekf_ = true;
		*/
		//====================================END==================================================
		// set recevied
		//this->thr_up_ekf_ = true;
		this->velmapReceived_ = true;
		this->velbodyReceived_ = true;
		this->poseReceived_ = true;

	}

	void CCMtrackingController::ekfaccelCB(const geometry_msgs::AccelWithCovarianceStampedConstPtr& accel){
		this->accel_ = *accel;
		//====================================NED==================================================

		// transform to local NED
		this->mea_accel_ekf_(0) = this->accel_.accel.accel.linear.x;
		this->mea_accel_ekf_(1) = -this->accel_.accel.accel.linear.y;
		this->mea_accel_ekf_(2) = -this->accel_.accel.accel.linear.z;
		// transform to global NED
		this->mea_accel_ekf_ = this->mea_R_ekf_ * this->mea_accel_ekf_;
		// add grav element in z axis 
		this->mea_accel_ekf_(2) += -9.8066;
		// get in body frame and negate as NED axis in z is opposite
		this->fz_est_raw_ekf_ = -this->mea_accel_ekf_.dot(this->mea_R_ekf_.inverse().col(2));
		// moving average update
		this->fz_est_ekf_ = (1.0 - (1.0/this->FZ_EST_N_)) * this->fz_est_ekf_ + (1.0/this->FZ_EST_N_) * this->fz_est_raw_ekf_;
		
		//====================================ENU==================================================
		/*
		// transform to local
		this->mea_accel_ekf_(0) = this->accel_.accel.accel.linear.x;
		this->mea_accel_ekf_(1) = this->accel_.accel.accel.linear.y;
		this->mea_accel_ekf_(2) = this->accel_.accel.accel.linear.z;
		// transform to global 
		this->mea_accel_ekf_ = this->mea_R_ekf_ * this->mea_accel_ekf_;
		// add grav element in z axis 
		this->mea_accel_ekf_(2) += 9.8066;
		// get in body frame
		this->fz_est_raw_ekf_ = this->mea_accel_ekf_.dot(this->mea_R_ekf_.inverse().col(2));
		// moving average update
		this->fz_est_ekf_ = (1.0 - (1.0/this->FZ_EST_N_)) * this->fz_est_ekf_ + (1.0/this->FZ_EST_N_) * this->fz_est_raw_ekf_;
		*/
		//====================================END==================================================
		// set recevied
		this->thr_up_ekf_ = true;
	}

	void CCMtrackingController::odomCB(const nav_msgs::OdometryConstPtr& odom){
		this->odom_ = *odom;
		/*
		//======== NED ========
		// convert from ENU to NED (mavros converts to ros ENU)
		// position and orientation
		this->mea_pos_(0) = this->odom_.pose.position.y;
		this->mea_pos_(1) = this->odom_.pose.position.x;
		this->mea_pos_(2) = -this->odom_.pose.position.z;
		
		// ROS quaternion
		this->mea_q_(0) = this->odom_.pose.orientation.w;
		this->mea_q_(1) = this->odom_.pose.orientation.x;
		this->mea_q_(2) = this->odom_.pose.orientation.y;
		this->mea_q_(3) = this->odom_.pose.orientation.z;
		// convert to matrix
		ccmutils::quat2rotM(this->mea_q_, this->mea_R_);
		// an adjust by -90 deg around z -> from body FLU to body FRD so axis are parrarel
		this->mea_R_ = this->mea_R_ * this->Rz_T_;
		// extract ENU encoding of PX4 quat
		ccmutils::rotM2quat(this->mea_q_, this->mea_R_);
		// convert to NED -> switch axis -> now -90 is 90
		double q_w = this->mea_q_(0);
		double q_x = this->mea_q_(2);
		double q_y = this->mea_q_(1);
		double q_z = -this->mea_q_(3);
		this->mea_q_ << q_w, q_x, q_y, q_z;
		// final conversion to rot matrix 
		ccmutils::quat2rotM(this->mea_q_, this->mea_R_);

		// set recevied
		this->pose_up_ = true;
		this->odomReceived_ = true;
		*/
		this->odomReceived_ = true;
	  }
	
	void CCMtrackingController::poseCB(const geometry_msgs::PoseStampedConstPtr& pose){
		this->pose_ = *pose;
		//====================================NED==================================================
		
		// convert from ENU to NED (mavros converts to ros ENU)
		// position and orientation
		this->mea_pos_(0) = this->pose_.pose.position.y;
		this->mea_pos_(1) = this->pose_.pose.position.x;
		this->mea_pos_(2) = -this->pose_.pose.position.z;
		// ROS quaternion
		this->mea_q_(0) = this->pose_.pose.orientation.w;
		this->mea_q_(1) = this->pose_.pose.orientation.x;
		this->mea_q_(2) = this->pose_.pose.orientation.y;
		this->mea_q_(3) = this->pose_.pose.orientation.z;
		// convert to matrix
		ccmutils::quat2rotM(this->mea_q_, this->mea_R_);
		// an adjust by -90 deg around z -> from body FLU to body FRD so axis are parrarel
		this->mea_R_ = this->mea_R_ * this->Rz_T_;
		// extract ENU encoding of PX4 quat
		ccmutils::rotM2quat(this->mea_q_, this->mea_R_);
		// convert to NED -> switch axis -> now -90 is 90
		double q_w = this->mea_q_(0);
		double q_x = this->mea_q_(2);
		double q_y = this->mea_q_(1);
		double q_z = -this->mea_q_(3);
		this->mea_q_ << q_w, q_x, q_y, q_z;
		// final conversion to rot matrix 
		ccmutils::quat2rotM(this->mea_q_, this->mea_R_);
		
		//====================================ENU==================================================
		/*
		// position
		this->mea_pos_(0) = this->pose_.pose.position.x;
		this->mea_pos_(1) = this->pose_.pose.position.y;
		this->mea_pos_(2) = this->pose_.pose.position.z;
		// ROS quaternion
		this->mea_q_(0) = this->pose_.pose.orientation.w;
		this->mea_q_(1) = this->pose_.pose.orientation.x;
		this->mea_q_(2) = this->pose_.pose.orientation.y;
		this->mea_q_(3) = this->pose_.pose.orientation.z;
		// convert to matrix
		ccmutils::quat2rotM(this->mea_q_, this->mea_R_);
		*/
		//====================================END==================================================
		// set recevied
		this->pose_up_ = true;
		//this->poseReceived_ = true;
		
	};
	  
	void CCMtrackingController::velCB(const geometry_msgs::TwistStampedConstPtr& vel_map) {
		this->vel_map_ = *vel_map;
		// calculate dt
		this->vel_prev_ = this->mea_vel_;
		double current_time = this->vel_map_.header.stamp.sec;
		if (this->vel_prev_t_ == 0) {
			this->vel_prev_t_ = current_time;
			return;
		}
		double vel_dt = (current_time + this->vel_map_.header.stamp.nsec*(1.0e-9)) - this->vel_prev_t_;
		this->vel_prev_t_ = current_time + this->vel_map_.header.stamp.nsec*(1.0e-9);
		//====================================NED==================================================
		
		// convert from ENU to NED (mavros converts to ros ENU)
		// transform to global NED
		this->mea_vel_(0) = this->vel_map_.twist.linear.y;
		this->mea_vel_(1) = this->vel_map_.twist.linear.x;
		this->mea_vel_(2) = -this->vel_map_.twist.linear.z;
		// get force in z axis from acceleration
		this->mea_accel_ = (this->mea_vel_ - this->vel_prev_)/vel_dt;
		this->mea_accel_(2) += -9.8066;
		// negate as NED axis in z is opposite
		this->fz_est_raw_ = -this->mea_accel_.dot(this->mea_R_.inverse().col(2));
		// moving average update
		this->fz_est_ = (1.0 - (1.0/this->FZ_EST_N_)) * this->fz_est_ + (1.0/this->FZ_EST_N_) * this->fz_est_raw_;
		
		//====================================ENU==================================================
		/*
		// get global velocity
		this->mea_vel_(0) = this->vel_map_.twist.linear.x;
		this->mea_vel_(1) = this->vel_map_.twist.linear.y;
		this->mea_vel_(2) = this->vel_map_.twist.linear.z;
		// get force in z axis from acceleration
		this->mea_accel_ = (this->mea_vel_ - this->vel_prev_)/vel_dt;
		this->mea_accel_(2) += 9.8066;
		// get in body frame in z axis
		this->fz_est_raw_ = this->mea_accel_.dot(this->mea_R_.inverse().col(2));
		// moving average update
		this->fz_est_ = (1.0 - (1.0/this->FZ_EST_N_)) * this->fz_est_ + (1.0/this->FZ_EST_N_) * this->fz_est_raw_;
		*/
		//====================================END==================================================
		// set recevied
		this->vel_up_ = true;
		this->thr_up_ = true;
		//this->velmapReceived_ = true;
		
	}

	void CCMtrackingController::angCB(const geometry_msgs::TwistStampedConstPtr& vel_body) {
		this->vel_body_ = *vel_body;
		//====================================NED==================================================
		
		// convert from ENU body (FLU) to NED body (FRD) (mavros converts to ros ENU)
		// set angular velocities
		this->mea_wb_(0) = this->vel_body_.twist.angular.x;
		this->mea_wb_(1) = -this->vel_body_.twist.angular.y;
		this->mea_wb_(2) = -this->vel_body_.twist.angular.z;
		
		//====================================ENU==================================================
		/*
		this->mea_wb_(0) = this->vel_body_.twist.angular.x;
		this->mea_wb_(1) = this->vel_body_.twist.angular.y;
		this->mea_wb_(2) = this->vel_body_.twist.angular.z;
		*/
		//====================================END==================================================
		// set recevied
		this->ang_up_ = true;
		//this->velbodyReceived_ = true;
		
	}

	void CCMtrackingController::imuCB(const sensor_msgs::ImuConstPtr& imu){
		this->imu_ = *imu;
		this->imuReceived_ = true;
	}

	void CCMtrackingController::targetCB(const tracking_controller::TargetConstPtr& target){
		this->target_ = *target;
		//====================================NED==================================================
		
		// convert target to NED
		
		this->position_recevied_conv_= Eigen::Vector3d(this->target_.position.y, this->target_.position.x, -this->target_.position.z);
		this->velocity_recevied_conv_=  Eigen::Vector3d(this->target_.velocity.y, this->target_.velocity.x, -this->target_.velocity.z);
		this->acceleration_recevied_conv_= Eigen::Vector3d(this->target_.acceleration.y, this->target_.acceleration.x, -this->target_.acceleration.z);
		this->jerk_recevied_conv_ = Eigen::Vector3d(this->target_.jerk.y, this->target_.jerk.x, -this->target_.jerk.z);
		/*
		this->position_recevied_conv_= Eigen::Vector3d(0.0, 0.0, -1.0);
		this->velocity_recevied_conv_= Eigen::Vector3d(0.0, 0.0, 0.0);
		this->acceleration_recevied_conv_ = Eigen::Vector3d(0.0, 0.0, 0.0);
		this->jerk_recevied_conv_= Eigen::Vector3d(0.0, 0.0, 0.0);*/
		
		// calculate yaw
		//this->yaw_recevied_conv_ = atan2(this->velocity_recevied_conv_(1), this->velocity_recevied_conv_(0));
		this->yaw_recevied_conv_ = 3.14159265358979323846/2;
		// calculate yaw dot
		//if (this->velocity_recevied_conv_(0)==0 and this->velocity_recevied_conv_(1)==0){this->yaw_dot_recevied_conv_  = 0;}
		//else {this->yaw_dot_recevied_conv_  = (this->velocity_recevied_conv_(0)*this->acceleration_recevied_conv_(1) - this->velocity_recevied_conv_(1)*this->acceleration_recevied_conv_(0))/(pow(this->velocity_recevied_conv_(0),2) + pow(this->velocity_recevied_conv_(1),2));}	
		this->yaw_dot_recevied_conv_ = 0;
		
		//====================================ENU==================================================
		/*
		this->position_recevied_conv_= Eigen::Vector3d(this->target_.position.x, this->target_.position.y, this->target_.position.z);
		this->velocity_recevied_conv_=  Eigen::Vector3d(this->target_.velocity.x, this->target_.velocity.y, this->target_.velocity.z);
		this->acceleration_recevied_conv_= Eigen::Vector3d(this->target_.acceleration.x, this->target_.acceleration.y, this->target_.acceleration.z);
		this->jerk_recevied_conv_ = Eigen::Vector3d(this->target_.jerk.x, this->target_.jerk.y, this->target_.jerk.z);
		this->yaw_recevied_conv_ = this->target_.yaw;
		this->yaw_dot_recevied_conv_ = 0;
		*/
		//====================================END==================================================
		// set recevied
		this->firstTargetReceived_ = true;
		this->targetReceived_ = true;
		// show info
		if(this->verbose_){
			/*
			std::cout << "[TargetUnconverted]: Position: " << this->target_.position.x<<", "<<this->target_.position.y<<", "<<this->target_.position.z << std::endl;
			std::cout << "[TargetUnconverted]: Velocity: " << this->target_.velocity.x<<", "<<this->target_.velocity.y<<", "<<this->target_.velocity.z << std::endl;
			std::cout << "[TargetUnconverted]: Acceleration: " << this->target_.acceleration.x<<", "<<this->target_.acceleration.y<<", "<<this->target_.acceleration.z << std::endl;
			std::cout << "[TargetUnconverted]: Jerk: " << this->target_.jerk.x<<", "<<this->target_.jerk.y<<", "<<this->target_.jerk.z << std::endl;
			std::cout << "[TargetUnconverted]: Yaw, Yaw_dot: " << this->target_.yaw<<", "<<this->target_.yaw_dot << std::endl;
			std::cout << "[TargetConverted]: Position: " << this->position_recevied_conv_(0)<<", "<<this->position_recevied_conv_(1)<<", "<<this->position_recevied_conv_(2)<< std::endl;
			std::cout << "[TargetConverted]: Velocity: " << this->velocity_recevied_conv_(0)<<", "<<this->velocity_recevied_conv_(1)<<", "<<this->velocity_recevied_conv_(2)<< std::endl;
			std::cout << "[TargetConverted]: Acceleration: " << this->acceleration_recevied_conv_(0)<<", "<<this->acceleration_recevied_conv_(1)<<", "<<this->acceleration_recevied_conv_(2)<< std::endl;
			std::cout << "[TargetConverted]: Jerk: " << this->jerk_recevied_conv_(0)<<", "<<this->jerk_recevied_conv_(1)<<", "<<this->jerk_recevied_conv_(2)<< std::endl;
			std::cout << "[TargetConverted]: Yaw, Yaw_dot: " << this->yaw_recevied_conv_<<", "<<this->yaw_dot_recevied_conv_ << std::endl;
			*/
		}
		// debug info
		#pragma region Debug_target
		ccm_tracking_controller::Debug_Readings targMsg;
		targMsg.header.stamp = ros::Time::now();
		targMsg.p_x = this->position_recevied_conv_(0);
		targMsg.p_y = this->position_recevied_conv_(1);
		targMsg.p_z = this->position_recevied_conv_(2);

		targMsg.v_x = this->velocity_recevied_conv_(0);
		targMsg.v_y = this->velocity_recevied_conv_(1);
		targMsg.v_z = this->velocity_recevied_conv_(2);

		targMsg.a_x = this->acceleration_recevied_conv_(0);
		targMsg.a_y = this->acceleration_recevied_conv_(1);
		targMsg.a_z = this->acceleration_recevied_conv_(2);

		targMsg.j_x = this->jerk_recevied_conv_(0);
		targMsg.j_y = this->jerk_recevied_conv_(1);
		targMsg.j_z = this->jerk_recevied_conv_(2);

		targMsg.yaw_nom = this->yaw_recevied_conv_;
		targMsg.yaw_d = this->yaw_dot_recevied_conv_;

		this->DebugPubTarget_.publish(targMsg);

		ccm_tracking_controller::Debug_Readings targMsgUnconv;
		targMsgUnconv.header.stamp = ros::Time::now();
		targMsgUnconv.p_x = this->target_.position.x;
		targMsgUnconv.p_y = this->target_.position.y;
		targMsgUnconv.p_z = this->target_.position.z;

		targMsgUnconv.v_x = this->target_.velocity.x;
		targMsgUnconv.v_y = this->target_.velocity.y;
		targMsgUnconv.v_z = this->target_.velocity.z;

		targMsgUnconv.a_x = this->target_.acceleration.x;
		targMsgUnconv.a_y = this->target_.acceleration.y;
		targMsgUnconv.a_z = this->target_.acceleration.z;

		targMsgUnconv.j_x = this->target_.jerk.x;
		targMsgUnconv.j_y = this->target_.jerk.y;
		targMsgUnconv.j_z = this->target_.jerk.z;

		targMsgUnconv.yaw_nom = this->target_.yaw;
		targMsgUnconv.yaw_d = this->target_.yaw_dot;

		this->DebugPubTargetUnconv_.publish(targMsgUnconv);
		#pragma endregion Debug_target
	}

	// moved into cmdCB
	/*
	void CCMtrackingController::updCB(const ros::TimerEvent&){
		if (not (this->odomReceived_ and this->imuReceived_ and this->velmapReceived_ and this->velbodyReceived_)){return;}
		
		// calculate dt
		this->ctrl_->setMode(true);
		double currTime = ros::Time::now().toSec();
		if (this->first_time_dt_){
			this->time_prev_ = ros::Time::now().toSec();
			this->first_time_dt_ = false;
			return;
			//this->dt_=0.004;
		}
		else{
			this->dt_ = (currTime - this->time_prev_);
			this->time_prev_ = currTime;
		}
		// update controller
		this->ctrl_->updateState(this->mea_pos_, this->mea_R_, this->mea_vel_, this->mea_wb_, this->fz_est_, this->dt_, this->pose_up_, this->vel_up_, this->ang_up_);
		
		//reset recevied
		this->pose_up_ = 0; 
		this->vel_up_ = 0;
		
		// debug info
		#pragma region Debug_state_update
		ccm_tracking_controller::Debug_Readings readingMsg;
		readingMsg.header.stamp = ros::Time::now();
		readingMsg.p_x = this->mea_pos_(0);
		readingMsg.p_y = this->mea_pos_(1);
		readingMsg.p_z = this->mea_pos_(2);

		readingMsg.v_x = this->mea_vel_(0);
		readingMsg.v_y = this->mea_vel_(1);
		readingMsg.v_z = this->mea_vel_(2);

		readingMsg.thr = this->fz_est_;

		readingMsg.rat_r = this->mea_wb_(0);
		readingMsg.rat_p = this->mea_wb_(1);
		readingMsg.rat_y = this->mea_wb_(2);

		readingMsg.q_w = this->mea_q_(0);
		readingMsg.q_x = this->mea_q_(1);
		readingMsg.q_y = this->mea_q_(2);
		readingMsg.q_z = this->mea_q_(3);

		this->DebugPubReadings_.publish(readingMsg);
		
		ccm_tracking_controller::Debug_Readings readingMsgUnconv;
		readingMsgUnconv.header.stamp = ros::Time::now();
		readingMsgUnconv.p_x = this->odom_.pose.pose.position.x;
		readingMsgUnconv.p_y = this->odom_.pose.pose.position.y;
		readingMsgUnconv.p_z = this->odom_.pose.pose.position.z;

		readingMsgUnconv.v_x = this->vel_map_.twist.linear.x;
		readingMsgUnconv.v_y = this->vel_map_.twist.linear.y;
		readingMsgUnconv.v_z = this->vel_map_.twist.linear.z;

		readingMsgUnconv.thr = this->fz_est_;

		readingMsgUnconv.rat_r = this->vel_body_.twist.angular.x;
		readingMsgUnconv.rat_p = this->vel_body_.twist.angular.y;
		readingMsgUnconv.rat_y = this->vel_body_.twist.angular.z;

		readingMsgUnconv.q_w = this->odom_.pose.pose.orientation.w;
		readingMsgUnconv.q_x = this->odom_.pose.pose.orientation.x;
		readingMsgUnconv.q_y = this->odom_.pose.pose.orientation.y;
		readingMsgUnconv.q_z = this->odom_.pose.pose.orientation.z;

		this->DebugPubReadingsUnconv_.publish(readingMsgUnconv);
		#pragma endregion Debug_state_update
	}
	*/

	void CCMtrackingController::cmdCB(const ros::TimerEvent&){
		//====== Update Controller State ======
		if (not (this->poseReceived_ and this->odomReceived_ and this->imuReceived_ and this->velmapReceived_ and this->velbodyReceived_ and this->poseReceived_)){return;}
		
		// calculate dt
		this->ctrl_->setMode(true);
		double currTime = ros::Time::now().toSec();
		if (this->first_time_dt_){
			this->time_prev_ = ros::Time::now().toSec();
			this->first_time_dt_ = false;
			return;
		}
		else{
			this->dt_ = (currTime - this->time_prev_);
			this->time_prev_ = currTime;
		}
		if (this->dt_==0){return;}

		// update controller
		this->ctrl_->updateState(this->mea_pos_ekf_, this->mea_R_ekf_, this->mea_vel_ekf_, this->mea_wb_ekf_, this->fz_est_ekf_, this->dt_, this->pose_up_ekf_, this->vel_up_ekf_, this->ang_up_ekf_, this->thr_up_ekf_);
		
		//reset recevied data -> test if extrapolation isnt messing things up
		this->pose_up_ = false; 
		this->vel_up_ = false;
		this->ang_up_ = false;
		this->thr_up_ = false;
		this->pose_up_ekf_ = false; 
		this->vel_up_ekf_ = false;
		this->ang_up_ekf_ = false;
		this->thr_up_ekf_ = false;
		
		// debug info
		#pragma region Debug_state_update
		Eigen::Vector3d measured_angles(0.0,0.0,0.0);
		ccmutils::R2euler_123(this->mea_R_, measured_angles);
		ccm_tracking_controller::Debug_Readings readingMsg;
		readingMsg.header.stamp = ros::Time::now();
		readingMsg.p_x = this->mea_pos_(0);
		readingMsg.p_y = this->mea_pos_(1);
		readingMsg.p_z = this->mea_pos_(2);
		readingMsg.v_x = this->mea_vel_(0);
		readingMsg.v_y = this->mea_vel_(1);
		readingMsg.v_z = this->mea_vel_(2);
		readingMsg.a_x = this->mea_accel_(0);
		readingMsg.a_y = this->mea_accel_(1);
		readingMsg.a_z = this->mea_accel_(2);
		readingMsg.thr = this->fz_est_;
		readingMsg.rwb_1 = this->mea_wb_(0);
		readingMsg.rwb_2 = this->mea_wb_(1);
		readingMsg.rwb_3 = this->mea_wb_(2);
		readingMsg.q_w = this->mea_q_(0);
		readingMsg.q_x = this->mea_q_(1);
		readingMsg.q_y = this->mea_q_(2);
		readingMsg.q_z = this->mea_q_(3);
		readingMsg.eul_1 = measured_angles(0);
		readingMsg.eul_2 = measured_angles(1);
		readingMsg.eul_3 = measured_angles(2);
		this->DebugPubReadings_.publish(readingMsg);
		
		Eigen::Vector3d measured_angles_ekf(0.0,0.0,0.0);
		ccmutils::R2euler_123(this->mea_R_ekf_, measured_angles_ekf);
		ccm_tracking_controller::Debug_Readings ekfMsg;
		ekfMsg.header.stamp = ros::Time::now();
		ekfMsg.p_x = this->mea_pos_ekf_(0);
		ekfMsg.p_y = this->mea_pos_ekf_(1);
		ekfMsg.p_z = this->mea_pos_ekf_(2);
		ekfMsg.v_x = this->mea_vel_ekf_(0);
		ekfMsg.v_y = this->mea_vel_ekf_(1);
		ekfMsg.v_z = this->mea_vel_ekf_(2);
		ekfMsg.a_x = this->mea_accel_ekf_(0);
		ekfMsg.a_y = this->mea_accel_ekf_(1);
		ekfMsg.a_z = this->mea_accel_ekf_(2);
		ekfMsg.thr = this->fz_est_ekf_;
		ekfMsg.rwb_1 = this->mea_wb_ekf_(0);
		ekfMsg.rwb_2 = this->mea_wb_ekf_(1);
		ekfMsg.rwb_3 = this->mea_wb_ekf_(2);
		ekfMsg.q_w = this->mea_q_ekf_(0);
		ekfMsg.q_x = this->mea_q_ekf_(1);
		ekfMsg.q_y = this->mea_q_ekf_(2);
		ekfMsg.q_z = this->mea_q_ekf_(3);
		ekfMsg.eul_1 = measured_angles_ekf(0);
		ekfMsg.eul_2 = measured_angles_ekf(1);
		ekfMsg.eul_3 = measured_angles_ekf(2);
		this->DebugEkfReadings_.publish(ekfMsg);

		//ccm_tracking_controller::Debug_Readings readingMsgUnconv;
		//readingMsgUnconv.header.stamp = ros::Time::now();
		//readingMsgUnconv.p_x = this->odom_.pose.pose.position.x;
		//readingMsgUnconv.p_y = this->odom_.pose.pose.position.y;
		//readingMsgUnconv.p_z = this->odom_.pose.pose.position.z;
		//readingMsgUnconv.v_x = this->vel_map_.twist.linear.x;
		//readingMsgUnconv.v_y = this->vel_map_.twist.linear.y;
		//readingMsgUnconv.v_z = this->vel_map_.twist.linear.z;
		//readingMsgUnconv.thr = this->fz_est_;
		//readingMsgUnconv.rwb_1 = this->vel_body_.twist.angular.x;
		//readingMsgUnconv.rwb_2 = this->vel_body_.twist.angular.y;
		//readingMsgUnconv.rwb_3 = this->vel_body_.twist.angular.z;
		//readingMsgUnconv.q_w = this->odom_.pose.pose.orientation.w;
		//readingMsgUnconv.q_x = this->odom_.pose.pose.orientation.x;
		//readingMsgUnconv.q_y = this->odom_.pose.pose.orientation.y;
		//readingMsgUnconv.q_z = this->odom_.pose.pose.orientation.z;
		//this->DebugPubReadingsUnconv_.publish(readingMsgUnconv);
		#pragma endregion Debug_state_update

		//====== Calculate Command ======
		if (not this->targetReceived_){return;}

		// calculate ccm
		this->ctrl_->calcCCM(this->yaw_recevied_conv_, this->yaw_dot_recevied_conv_, this->position_recevied_conv_, this->velocity_recevied_conv_, this->acceleration_recevied_conv_, this->jerk_recevied_conv_);

		// ==== Return values ====
		// zadana prędkość kątowa body frame
		Eigen::Vector3d ref_om(0.0,0.0,0.0); ref_om = this->ctrl_->getOm();
		// obecna orientacja euler
		Eigen::Vector3d euler = this->ctrl_->getEuler();
		// zadana prędkość euler dot global frame
		Eigen::Vector3d ref_er(0.0,0.0,0.0); ref_er = this->ctrl_->getEr();
		// obliczona energia kontrakcji
		double E = this->ctrl_->getE();
		// zadany ciąg
		double force = this->ctrl_->getfz();
		// zadany yaw
		double yaw_nom = this->ctrl_->getYawNom();
		// thrust as percent value
		double thrust_percent = std::min(1.0, std::max(0.0, this->hoverThrust_ * ((force) / 9.8066)));
		this->cmdThrust_ = thrust_percent;
		this->thrustReady_ = true;	
		
		// set commands (otrzymwane w NED, wysyłane ENU) body rates in body frame, thrust
		// convert from NED body (FRD) to ENU body (FLU)
		Eigen::Vector4d cmd;
		//====================================NED==================================================
		
		cmd(0) = ref_om(0);
		cmd(1) = -ref_om(1); 
		cmd(2) = -ref_om(2);
		
		//====================================ENU==================================================
		/*
		cmd(0) = ref_om(0);
		cmd(1) = ref_om(1); 
		cmd(2) = ref_om(2);
		*/
		//====================================END==================================================
		// v1
		cmd(3) = this->cmdThrust_;
		// v2
		/*
		double e = force - this->fz_est_ekf_;
		if (this->first_time_cmd_){
			cmd(3) = this->hoverThrust_ + this->K_THR_ * e;
			this->first_time_cmd_ = false;
		}
		else {
			cmd(3) = this->last_force_cmd_ + this->K_THR_ * e;
		}
		cmd(3) = std::min(1.0, std::max(0.0, cmd(3)));
		this->last_force_cmd_ = cmd(3);
		*/
		// v3
		//cmd(3) = std::min(1.0, std::max(0.0, this->hoverThrust_ * ((this->fz_est_ekf_ + this->K_THR_ * e) / 9.8066)));

		// publish command
		this->publishCommand(cmd);

		// reset update - this will cause drone to 'follow given target once' and wait for another one
		//this->targetReceived_ = false;

		// show info
		if (this->verbose_){
			this->log_file <<"[CCMtrackingController]: Output info: " << ros::Time::now() << std::endl;
			this->log_file <<"[CCMtrackingController]: Commands: "<< cmd(0)<<", "<<cmd(1)<<", "<<cmd(2)<<", "<<cmd(3)<<" From: "<<ref_om(0)<<", "<<ref_om(1)<<", "<<ref_om(2)<<", "<<force<< std::endl;
			this->log_file <<"[CCMtrackingController]: Target position: "<< this->position_recevied_conv_(0)<< this->position_recevied_conv_(1)<<", "<<this->position_recevied_conv_(2) << std::endl;
			this->log_file <<"[CCMtrackingController]: Target velocity: "<< this->velocity_recevied_conv_(0)<<", "<<this->velocity_recevied_conv_(1)<<", "<<this->velocity_recevied_conv_(2) << std::endl;
			this->log_file <<"[CCMtrackingController]: Target acceleration: "<< this->acceleration_recevied_conv_(0)<<", "<<this->acceleration_recevied_conv_(1)<<", "<<this->acceleration_recevied_conv_(2) << std::endl;
			this->log_file <<"[CCMtrackingController]: Target jerk: "<< this->jerk_recevied_conv_(0)<<", "<<this->jerk_recevied_conv_(1)<<", "<<this->jerk_recevied_conv_(2) << std::endl;
			this->log_file <<"[CCMtrackingController]: Target yaw, yaw_dot: "<< this->yaw_recevied_conv_<<", "<<this->yaw_dot_recevied_conv_ << std::endl;
			this->log_file <<"===="<< std::endl;
			this->log_file.flush();
			/*
			std::cout << "[CCMtrackingController]: Euler_dot, global: " << ref_er(0)<<", "<<ref_er(1)<<", "<<ref_er(2) << std::endl;
			std::cout << "[CCMtrackingController]: R_om, body: " << ref_om(0)<<", "<<ref_om(1)<<", "<<ref_om(2) << std::endl;
			std::cout << "[CCMtrackingController]: Commanded thrust force: " << force << std::endl;
			std::cout << "[CCMtrackingController]: Contraction energy: " << E << std::endl;
			std::cout << "[CCMtrackingController]: Required yaw: " << yaw_nom << std::endl;
			std::cout << "[CCMtrackingController]: Target yaw: " << this->yaw_recevied_conv_ << std::endl;
			std::cout << "[CCMtrackingController]: Current rpy: " << euler(0)<<", "<<euler(1)<<", "<<euler(2) << std::endl;
			std::cout << "[CCMtrackingController]: Hover thrust current estimate: " << this->hoverThrust_ << std::endl;			
			std::cout << "[CCMtrackingController]: Commanded angular velocities: " << cmd(0)<<", "<<cmd(1)<<", "<<cmd(2) << std::endl;
			std::cout << "[CCMtrackingController]: Commanded thrust percent command: " << cmd(3) << std::endl;
			*/
			std::cout << "[CCMtrackingController]: Commanded thrust percent command: " << cmd(3) << std::endl;
		}

		// debug info
		#pragma region Debug_command
		ccm_tracking_controller::Debug_Readings cmdMsg;
		cmdMsg.header.stamp = ros::Time::now();
		cmdMsg.cmd_x = cmd(0);
		cmdMsg.cmd_y = cmd(1);
		cmdMsg.cmd_z = cmd(2);
		cmdMsg.thr = cmd(3);
		cmdMsg.yaw_nom = yaw_nom;
		cmdMsg.eul_1 = euler(0);
		cmdMsg.eul_2 = euler(1);
		cmdMsg.eul_3 = euler(2);
		cmdMsg.rwb_1 = ref_om(0);
		cmdMsg.rwb_2 = ref_om(1);
		cmdMsg.rwb_3 = ref_om(2);
		cmdMsg.eul_dot_1 = ref_er(0);
		cmdMsg.eul_dot_2 = ref_er(1);
		cmdMsg.eul_dot_3 = ref_er(2);
		cmdMsg.E = E;
		cmdMsg.fc = force;
		this->DebugPubCommand_.publish(cmdMsg);
		#pragma endregion Debug_command

	}

	void CCMtrackingController::thrustEstimateCB(const ros::TimerEvent&){
		// collects last 10 thrust estimates, if difference between min and max within those is less than eps, then last value can be taken as hover thrust
		if (not this->thrustReady_ or not this->imuReceived_){return;}
		if (this->kfFirstTime_){
			this->kfFirstTime_ = false;
			this->kfStartTime_ = ros::Time::now();
			return;
		}

		// run estimator when the command thrust is available
		// sync IMU and command thrust (?)
		double hoverThrust = this->hoverThrust_;
		double cmdThrust = this->cmdThrust_;
		Eigen::Vector3d currAccBody (this->imu_.linear_acceleration.x, this->imu_.linear_acceleration.y, this->imu_.linear_acceleration.z);
		Eigen::Vector4d currQuat (this->odom_.pose.pose.orientation.w, this->odom_.pose.pose.orientation.x, this->odom_.pose.pose.orientation.y, this->odom_.pose.pose.orientation.z);
		Eigen::Matrix3d currRot = ccmtracking::quat2RotMatrix(currQuat);
		Eigen::Vector3d currAcc = currRot * currAccBody;	

		// states: Hover thrust
		double states = hoverThrust;
		double A = 1;
		double H = -(cmdThrust * 9.8) * (1.0/pow(hoverThrust, 2));
		double z = (currAcc(2) - 9.8); // acceleratoin

		// Kalman filter predict (predict states and propagate var)
		states = A * states;
		this->stateVar_ += this->processNoiseVar_;

		// Kalman filter correction
		double Ivar = std::max(H * this->stateVar_ * H + this->measureNoiseVar_, this->measureNoiseVar_);
		double K = this->stateVar_ * H/Ivar;
		double I = z - (cmdThrust/hoverThrust - 1.0) * 9.8;

		// update hoverThrust 
		double newHoverThrust = hoverThrust + K * I;
		this->stateVar_ = (1.0 - K * H) * this->stateVar_;

		if (this->verbose_){
			//std::cout << "[Kalman]: Thrust estimation variance: " << this->stateVar_ << std::endl;
			//std::cout << "test ratio: " << I*I/Ivar << std::endl;
			//std::cout << "[Kalman]: New thrust estimate is: " << newHoverThrust << std::endl;
		}

		double prevMinThrust = 0.0;
		double prevMaxThrust = 1.0;
		if (this->prevEstimateThrusts_.size() < 10){
			this->prevEstimateThrusts_.push_back(newHoverThrust);
		}
		else{
			this->prevEstimateThrusts_.pop_front();
			this->prevEstimateThrusts_.push_back(newHoverThrust);
			std::deque<double>::iterator itMin = std::min_element(this->prevEstimateThrusts_.begin(), this->prevEstimateThrusts_.end());
			std::deque<double>::iterator itMax = std::max_element(this->prevEstimateThrusts_.begin(), this->prevEstimateThrusts_.end());
			prevMinThrust = *itMin;
			prevMaxThrust = *itMax;
		}

		// if the state variance is smaller enough, update the hover thrust
		if (std::abs(prevMinThrust - prevMaxThrust) < 0.005){
			if (newHoverThrust > 0 and newHoverThrust < 1.0){
				this->hoverThrust_ = newHoverThrust;
				//ros::Time currTime = ros::Time::now(); 							// unused variable
				//double estimatedTime = (currTime - this->kfStartTime_).toSec(); 	// unused variable
				if (this->verbose_){
					//std::cout << "[CCMtrackingController]: New estimate at " << estimatedTime  << "s, and Estimated thrust is: " << newHoverThrust << ". Variance: " << this->stateVar_ << std::endl; 
				}
			}
			else{
				std::cout << "[CCMtrackingController]: !!!!!!!!!!AUTO TRHUST ESTIMATION FAILS!!!!!!!!!" << std::endl;
			}
		}
		//std::cout << "current commnd thrust is: " << cmdThrust << std::endl;
		//std::cout << "current z: " << z << std::endl;
		//std::cout << "world acc: " << currAcc.transpose() << std::endl;
		//std::cout << "[trackingController]: Estimated thrust is: " << newHoverThrust << std::endl; 
		//std::cout << "variance: " << this->stateVar_ << std::endl;
		//std::cout << "hover thrust from acc: " << (9.8 * cmdThrust)/currAcc(2) << std::endl;
		//std::cout << "Current hover thrust is set to: " << this->hoverThrust_ << std::endl;
	}

	void CCMtrackingController::visCB(const ros::TimerEvent&){
		this->publishPoseVis();
		this->publishHistTraj();
		this->publishTargetVis();
		this->publishTargetHistTraj();
		this->publishVelAndAccVis();
	}

	void CCMtrackingController::publishCommand(const Eigen::Vector4d& cmd){
		mavros_msgs::AttitudeTarget cmdMsg;
		cmdMsg.header.stamp = ros::Time::now();
		cmdMsg.header.frame_id = "map";
		cmdMsg.body_rate.x = cmd(0);
		cmdMsg.body_rate.y = cmd(1);
		cmdMsg.body_rate.z = cmd(2);
		cmdMsg.thrust = cmd(3);
		// mask for angulat velocity control instead of quaternion
		cmdMsg.type_mask = cmdMsg.IGNORE_ATTITUDE;
		this->cmdPub_.publish(cmdMsg);
	}

	void CCMtrackingController::publishPoseVis(){
		if (not this->odomReceived_) return;
		geometry_msgs::PoseStamped ps;
		ps.header.frame_id = "map";
		ps.header.stamp = ros::Time::now();
		ps.pose.position.x = this->odom_.pose.pose.position.x;
		ps.pose.position.y = this->odom_.pose.pose.position.y;
		ps.pose.position.z = this->odom_.pose.pose.position.z;
		ps.pose.orientation = this->odom_.pose.pose.orientation;
		if (this->histTraj_.size() <= 100){
			this->histTraj_.push_back(ps);
		}
		else{
			this->histTraj_.push_back(ps);
			this->histTraj_.pop_front();
		}
		this->poseVis_ = ps;
		this->poseVisPub_.publish(ps);
	}

	void CCMtrackingController::publishHistTraj(){
		if (not this->odomReceived_) return;
		nav_msgs::Path histTrajMsg;
		histTrajMsg.header.frame_id = "map";
		histTrajMsg.header.stamp = ros::Time::now();
		for (size_t i=0; i<this->histTraj_.size(); ++i){
			histTrajMsg.poses.push_back(this->histTraj_[i]);
		}
		
		this->histTrajVisPub_.publish(histTrajMsg);
	}

	void CCMtrackingController::publishTargetVis(){
		if (not this->firstTargetReceived_) return;
		geometry_msgs::PoseStamped ps;
		ps.header.frame_id = "map";
		ps.header.stamp = ros::Time::now();
		ps.pose.position.x = this->target_.position.x;
		ps.pose.position.y = this->target_.position.y;
		ps.pose.position.z = this->target_.position.z;
		ps.pose.orientation = ccmtracking::quaternion_from_rpy(0, 0, this->target_.yaw);
		if (this->targetHistTraj_.size() <= 100){
			this->targetHistTraj_.push_back(ps);
		}
		else{
			this->targetHistTraj_.push_back(ps);
			this->targetHistTraj_.pop_front();
		}

		this->targetPoseVis_ = ps;
		this->targetVisPub_.publish(ps);
		
	}

	void CCMtrackingController::publishTargetHistTraj(){
		if (not this->firstTargetReceived_) return;
		nav_msgs::Path targetHistTrajMsg;
		targetHistTrajMsg.header.frame_id = "map";
		targetHistTrajMsg.header.stamp = ros::Time::now();
		for (size_t i=0; i<this->targetHistTraj_.size(); ++i){
			targetHistTrajMsg.poses.push_back(this->targetHistTraj_[i]);
		}
		
		this->targetHistTrajVisPub_.publish(targetHistTrajMsg);
	}

	void CCMtrackingController::publishVelAndAccVis(){
		if (not this->odomReceived_) return;
		// current velocity
		Eigen::Vector3d currPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d currVelBody (this->odom_.twist.twist.linear.x, this->odom_.twist.twist.linear.y, this->odom_.twist.twist.linear.z);
		Eigen::Vector4d currQuat (this->odom_.pose.pose.orientation.w, this->odom_.pose.pose.orientation.x, this->odom_.pose.pose.orientation.y, this->odom_.pose.pose.orientation.z);
		Eigen::Matrix3d currRot = ccmtracking::quat2RotMatrix(currQuat);
		Eigen::Vector3d currVel = currRot * currVelBody;	

		// current acceleration	
		Eigen::Vector3d currAcc;
		ros::Time currTime = ros::Time::now();
		if (this->velFirstTime_){
			this->velPrevTime_ = ros::Time::now();
			currAcc = Eigen::Vector3d (0.0, 0.0, 0.0);
			this->velFirstTime_ = false;
		}
		else{
			double dt = (currTime - this->velPrevTime_).toSec();
			currAcc = (currVel - this->prevVel_)/dt;
			// std::cout << "dt: " << dt << std::endl;
			// std::cout << "current velocity: " << currVel(0) << " " << currVel(1) << " " << currVel(2) << std::endl;
			// std::cout << "prev velocity: " << this->prevVel_(0) << " " << this->prevVel_(1) << " " << this->prevVel_(2) << std::endl;
		}
		this->prevVel_ = currVel;
		this->velPrevTime_ = currTime;

		// target velocity
		Eigen::Vector3d targetVel (this->target_.velocity.x, this->target_.velocity.y, this->target_.velocity.z);

		// target acceleration
		Eigen::Vector3d targetAcc (this->target_.acceleration.x, this->target_.acceleration.y, this->target_.acceleration.z);


		visualization_msgs::Marker velAndAccVisMsg;
        velAndAccVisMsg.header.frame_id = "map";
        velAndAccVisMsg.header.stamp = ros::Time::now();
        velAndAccVisMsg.ns = "tracking_controller";
        // velAndAccVisMsg.id = 0;
        velAndAccVisMsg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        velAndAccVisMsg.pose.position.x = this->odom_.pose.pose.position.x;
        velAndAccVisMsg.pose.position.y = this->odom_.pose.pose.position.y;
        velAndAccVisMsg.pose.position.z = this->odom_.pose.pose.position.z + 0.4;
        velAndAccVisMsg.scale.x = 0.15;
        velAndAccVisMsg.scale.y = 0.15;
        velAndAccVisMsg.scale.z = 0.15;
        velAndAccVisMsg.color.a = 1.0;
        velAndAccVisMsg.color.r = 1.0;
        velAndAccVisMsg.color.g = 1.0;
        velAndAccVisMsg.color.b = 1.0;
        velAndAccVisMsg.lifetime = ros::Duration(0.05);

        double vNorm = currVel.norm();
        double aNorm = currAcc.norm();
        double vNormTgt = targetVel.norm();
        double aNormTgt = targetAcc.norm();

        std::string velText = "|V|=" + std::to_string(vNorm) + ", |VT|=" + std::to_string(vNormTgt) + "\n|A|=" + std::to_string(aNorm) + ", |AT|=" + std::to_string(aNormTgt) ;
        velAndAccVisMsg.text = velText;
        this->velAndAccVisPub_.publish(velAndAccVisMsg);
	}

}