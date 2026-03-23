std_msgs/Header header
# position
float32 p_x
float32 p_y
float32 p_z

# velocity
float32 v_x
float32 v_y
float32 v_z

# acceleration
float32 a_x
float32 a_y
float32 a_z

# jerk
float32 j_x
float32 j_y
float32 j_z

# angles
float32 eul_1
float32 eul_2
float32 eul_3
float32 yaw_nom

# dot euler angles
float32 eul_dot_1
float32 eul_dot_2
float32 eul_dot_3

# angle rates body
float32 rwb_1
float32 rwb_2
float32 rwb_3

# thrust %
float32 thr

# desured yaw
float32 yaw_d

# quaternion
float32 q_w
float32 q_x
float32 q_y
float32 q_z

# thrust force
float32 fc

# thrust force delta
float32 dfc

# debug values
float32 filtered_f_dot

# commands
float32 cmd_x
float32 cmd_y
float32 cmd_z

# energy 
float32 E

# delta dot euler angles
float32 d_eul_dot_1
float32 d_eul_dot_2
float32 d_eul_dot_3

# ccm calcluations
float32 a
float32 b_norm
float32 b_1
float32 b_2
float32 b_3
float32 b_4


