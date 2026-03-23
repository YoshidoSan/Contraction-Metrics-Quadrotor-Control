# Contraction Metrics-based Quadrotor Control
Code for master thesis focusing on quadrotor control in dynamic environments when disruption is present (wind). My work focuses on design and implementation of tracking controller based on contraction metrics.
All the used code is one repo for easier control and deployment on physical quadrotor.
## Further description - work in progress
PX4-based quadrotor is used (1.17 version)...

### Used frameworks/packages
Elements of the excellent UAV-Autonomy framework by CERLAB where used - https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy
List of used packages and changes:
1. autonomous_flight
    - Based on commit: 82168298d9cae3b612af390156ec8712d852fdb6  
        - Changes: Modified "trajExeCB()" for jerk and yaw_dot target calulation, python includes
2. global_planner
    - Based on commit: 2afc6e3cf907d6ed25405ffdd989dae25510bf18  
        - Changes: none
3. map_manager
    - Based on commit: d8112b85f6ad4e2519b0f34740ff0f6c466bca24  
        - Changes: python includes
4. onboard_detector
    - Based on commit: d754d151869170b66b7f9e2059c8fc223da449eb  
        - Changes: python includes
5. remote_control
    - Based on commit: 8eed5515fea57d32c7f7eaa97b2b287006d9613d  
        - Changes: none
6. time_optimizer
    - Based on commit: 0854be3099be7a29fa2f3428a1c8e66da4b1f8ab  
        - Changes: none
7. tracking_controller
    - Based on commit: 7f8f5d77556877169f31c978b3f3bdba0fcb62d2  
        - Changes: Modified Target.msg to contain jerk and yaw_dot
8. trajectory_planner
    - Based on commit: a84b491456e4f97d400627e1e2cd07392ac91333  
        - Changes: Modified b-spline degree (hard-coded param) for jerk

For odometry source VINS-Fuison by HUSK will be used - https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
