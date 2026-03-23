/*
	FILE: CCMtrackingController.h
	-------------------------------
	function definition of px4 CCM controller
*/
#ifndef CCM_TRACKING_CONTROLLER_H
#define CCM_TRACKING_CONTROLLER_H
#include <ros/ros.h>
#include <Eigen/Dense>
#include <queue>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <iostream>
#include <std_msgs/Float64.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <utils.h>
#include <math.h>
#include <tracking_controller/Target.h>
#include <ccm_tracking_controller/Debug_Readings.h>
#include <CCM/CCMcontroller.h>
#include <CCM/ccmutils.h>
#include <CCM/Geodesic.h>
//#include <CCM/Metric.h>

#include <fstream>

namespace ccmtracking{
	class CCMtrackingController {
		private:
			ros::NodeHandle nh_;
			ros::Subscriber odomSub_; // Subscribe to odometry
			ros::Subscriber ekfSub_; // Subscribe to ekf
			ros::Subscriber ekfaccelSub_; // Subscribe to ekf acel
			ros::Subscriber poseSub_; // Subscribe to pose
			ros::Subscriber imuSub_; // Subscribe to imu
			ros::Subscriber velSub_; // Subscribe to vel in map frame
			ros::Subscriber angSub_; // Subscribe to vel in body frame
			ros::Subscriber targetSub_; // subscriber for the tracking target states
			ros::Publisher cmdPub_; // command publisher
			ros::Publisher poseVisPub_; // current pose publisher
			ros::Publisher targetVisPub_; // target pose publisher
			ros::Publisher histTrajVisPub_; // history trajectory publisher
			ros::Publisher targetHistTrajVisPub_; // target trajectory publisher
			ros::Publisher velAndAccVisPub_; // velocity and acceleration visualization publisher
			ros::Timer thrustEstimatorTimer_; // thrust estimator timer
			ros::Timer updTimer_; // update timer
			ros::Timer cmdTimer_; // command timer
			ros::Timer visTimer_; // visualization timer
			// debug publishers
			ros::Publisher DebugPubTarget_; // debug target publisher
			ros::Publisher DebugPubTargetUnconv_; // debug target unconverted publisher
			ros::Publisher DebugPubReadings_; // debug readings publisher
			ros::Publisher DebugPubReadingsUnconv_;
			ros::Publisher DebugPubCommand_; // debug command publisher
			ros::Publisher DebugEkfReadings_; // debug ekf data

			//debug logs
			std::ofstream log_file;


			// parameters
			double FZ_EST_N_; // rozmiar okna do estymacji siły ciągu (moving average)
			double FZ_CTRL_N_; // Stała czasowa filtru pochodnej ciągu
			bool verbose_;
			double hoverThrust_;
			double K_THR_;

			// CCM
			std::unique_ptr<CCMController> ctrl_;
			//std::shared_ptr<CCMController> ctrl_;

			//recevied data
			tracking_controller::Target target_;
			Eigen::Vector3d position_recevied_conv_;
			Eigen::Vector3d velocity_recevied_conv_;
			Eigen::Vector3d acceleration_recevied_conv_;
			Eigen::Vector3d jerk_recevied_conv_;
			double yaw_recevied_conv_;
			double yaw_dot_recevied_conv_;
			bool firstTargetReceived_ = false;
			bool targetReceived_ = false;
			bool poseReceived_ = false;
			bool odomReceived_ = false;
			bool velmapReceived_ = false;
			bool velbodyReceived_ = false;
			bool imuReceived_ = false;
			bool thrustReady_ = false;
			nav_msgs::Odometry odom_;
			nav_msgs::Odometry ekf_;
			geometry_msgs::AccelWithCovarianceStamped accel_;
			geometry_msgs::PoseStamped pose_;
			sensor_msgs::Imu imu_;
			geometry_msgs::TwistStamped vel_map_;
			geometry_msgs::TwistStamped vel_body_;

			// kalman filter
			bool kfFirstTime_ = true;
			ros::Time kfStartTime_;  
			double stateVar_ = 0.01;
			double processNoiseVar_ = 0.01;
			double measureNoiseVar_ = 0.02;
			std::deque<double> prevEstimateThrusts_;

			double cmdThrust_;
			ros::Time cmdThrustTime_;

			// Measured states
            Eigen::Vector4d mea_q_;
            Eigen::Matrix3d mea_R_;
            Eigen::Vector3d mea_wb_;
            Eigen::Vector3d mea_pos_;
            Eigen::Vector3d mea_vel_;
			Eigen::Vector3d mea_accel_;
            Eigen::Vector3d vel_prev_;
			Eigen::Vector3d euler_;
            double vel_prev_t_=0;
			// matrix -90 deg in z axis
            const Eigen::Matrix<double,3,3> Rz_T_= 
			(Eigen::Matrix<double,3,3>() 
			<< 0.0, 1.0, 0.0,
			-1.0, 0.0, 0.0,
			 0.0, 0.0, 1.0).finished();
			// enu to ned for debug
			const Eigen::Matrix<double,3,3> R_enu_to_ned_= 
			(Eigen::Matrix<double,3,3>() 
			<< 0.0, 1.0, 0.0,
			1.0, 0.0, 0.0,
			0.0, 0.0, -1.0).finished();
            // Thrust estimation MA
            double fz_est_raw_;
            double fz_est_;

			//ekf test
			Eigen::Vector4d mea_q_ekf_;
            Eigen::Matrix3d mea_R_ekf_;
            Eigen::Vector3d mea_wb_ekf_;
            Eigen::Vector3d mea_pos_ekf_;
            Eigen::Vector3d mea_vel_ekf_;
			Eigen::Vector3d mea_accel_ekf_;
            Eigen::Vector3d vel_prev_ekf_;
			double fz_est_raw_ekf_;
            double fz_est_ekf_;
			double vel_prev_t_ekf_ = 0.0;
			bool pose_up_ekf_ = false;
            bool vel_up_ekf_ = false;
			bool thr_up_ekf_ = false;
			bool ang_up_ekf_ = false;
			

            // Update variables
            bool pose_up_ = false;
            bool vel_up_ = false;
			bool ang_up_ = false;
			bool thr_up_ = false;

			//first time cmd
			bool first_time_cmd_ = true;
			double last_force_cmd_;
			
			//time for state update
			bool first_time_dt_ = true;
			double dt_;
			double time_prev_;

			// visualization
			geometry_msgs::PoseStamped poseVis_;
			std::deque<geometry_msgs::PoseStamped> histTraj_;
			geometry_msgs::PoseStamped targetPoseVis_;
			std::deque<geometry_msgs::PoseStamped> targetHistTraj_;
			bool velFirstTime_ = true;
			Eigen::Vector3d prevVel_;
			ros::Time velPrevTime_;
		
		public:
			CCMtrackingController(const ros::NodeHandle& nh);
			void initParamModules();
			void registerPub();
			void registerCallback();

			// callback functions
			void odomCB(const nav_msgs::OdometryConstPtr& odom);
			void ekfCB(const nav_msgs::OdometryConstPtr& ekf);
			void ekfaccelCB(const geometry_msgs::AccelWithCovarianceStampedConstPtr& accel);
			void poseCB(const geometry_msgs::PoseStampedConstPtr& pose);
			void velCB(const geometry_msgs::TwistStampedConstPtr& vel_map);
			void angCB(const geometry_msgs::TwistStampedConstPtr& vel_body);
			void imuCB(const sensor_msgs::ImuConstPtr& imu);
			void targetCB(const tracking_controller::TargetConstPtr& target);
			void thrustEstimateCB(const ros::TimerEvent&);
			void cmdCB(const ros::TimerEvent&);
			void visCB(const ros::TimerEvent&);

			void publishCommand(const Eigen::Vector4d& cmd);

			// visualization
			void publishPoseVis();
			void publishHistTraj();
			void publishTargetVis();
			void publishTargetHistTraj();
			void publishVelAndAccVis();


	};
}

#endif