#include <CCM/ccmutils.h>

void ccmutils::quat2rotM(const Eigen::Vector4d& q, Eigen::Matrix3d &R){
  // quaternion to rotation matrix
  double q1 = q(0);
  double q2 = q(1);
  double q3 = q(2);
  double q4 = q(3);

  double q1s = q1*q1;
  double q2s = q2*q2;
  double q3s = q3*q3;
  double q4s = q4*q4;

  R <<
    q1s+q2s-q3s-q4s, 2.0*(q2*q3-q1*q4) ,2.0*(q2*q4+q1*q3),
    2.0*(q2*q3+q1*q4), q1s-q2s+q3s-q4s, 2.0*(q3*q4-q1*q2),
    2.0*(q2*q4-q1*q3), 2.0*(q3*q4+q1*q2), q1s-q2s-q3s+q4s;
}

void ccmutils::rotM2quat(Eigen::Vector4d& q, const Eigen::Matrix3d &R){
  // rotation matrix to quaternion
  double T = R.trace();
  Eigen::Vector4d case_vec(T, R(0,0), R(1,1), R(2,2));

  std::ptrdiff_t i;
  case_vec.maxCoeff(&i);

  double den = 0.0;
  switch (i)   {
    case 0:
      den = std::sqrt(1.0+T);
      q(0) = 0.5*den;
      q(1) = 0.5*(R(2,1)-R(1,2))/den;
      q(2) = 0.5*(R(0,2)-R(2,0))/den;
      q(3) = 0.5*(R(1,0)-R(0,1))/den;
      break;

    case 1:
      den = std::sqrt(1.0+R(0,0)-R(1,1)-R(2,2));
      q(0) = 0.5*(R(2,1)-R(1,2))/den;
      q(1) = 0.5*den;
      q(2) = 0.5*(R(0,1)+R(1,0))/den;
      q(3) = 0.5*(R(2,0)+R(0,2))/den;
      break;

    case 2:
      den = std::sqrt(1.0-R(0,0)+R(1,1)-R(2,2));
      q(0) = 0.5*(R(0,2)-R(2,0))/den;
      q(1) = 0.5*(R(0,1)+R(1,0))/den;
      q(2) = 0.5*den;
      q(3) = 0.5*(R(1,2)+R(2,1))/den;
      break;
      
    default:
      den = std::sqrt(1.0-R(0,0)-R(1,1)+R(2,2));
      q(0) = 0.5*(R(1,0)-R(0,1))/den;
      q(1) = 0.5*(R(2,0)+R(0,2))/den;
      q(2) = 0.5*(R(2,1)+R(1,2))/den;
      q(3) = 0.5*den;
  }

}

void ccmutils::R2euler_123(const Eigen::Matrix3d &R, Eigen::Vector3d &ang){
  // rotation matrix to euler angles
  ang(0) = std::atan2(-R(1,2),R(2,2));
  ang(1) = std::asin(R(0,2));
  ang(2) = std::atan2(-R(0,1),R(0,0));
}

void ccmutils::Euler2R_123(const Eigen::Vector3d &ang, Eigen::Matrix3d &R){
  // euler angles to rotation matrix (operatory złożone od lewej do prawej, kolejność 123 (xyz)) R = Rx*Ry*Rz
  // body frame to inertia frame
  R <<  std::cos(ang(1))*std::cos(ang(2)),-std::cos(ang(1))*std::sin(ang(2)), std::sin(ang(1)),
        std::cos(ang(0))*std::sin(ang(2))+std::cos(ang(2))*std::sin(ang(1))*std::sin(ang(0)), std::cos(ang(0))*std::cos(ang(2))-std::sin(ang(1))*std::sin(ang(0))*std::sin(ang(2)), -std::cos(ang(1))*std::sin(ang(0)),
        std::sin(ang(0))*std::sin(ang(2))-std::cos(ang(0))*std::cos(ang(2))*std::sin(ang(1)), std::cos(ang(2))*std::sin(ang(0))+std::cos(ang(0))*std::sin(ang(1))*std::sin(ang(2)),  std::cos(ang(1))*std::cos(ang(0));
}

void ccmutils::R_wb(const Eigen::Vector3d &ang, Eigen::Matrix3d &R){
  // euler rates to body rates
  // z q (euler) macierz, quat_dot -> prędkości kątowe body 
  //Eigen::Matrix3d R_om;
  // tutaj znaki w R są zmienione z jakiego spowodu (inne niż w Euler2R_123!) -> w tej wersji dron jest w FRD (90 wg ENU)
  R << std::cos(ang(1))*std::cos(ang(2)), std::sin(ang(2)), 0.0,
         -std::cos(ang(1))*std::sin(ang(2)), std::cos(ang(2)), 0.0,
          std::sin(ang(1)), 0.0, 1.0;
  // mi na kartce takie znaki wyszły -> te 'znane' znaki w R
  // w tej wersji dron jest w -90 wg ENU (90 wg NED) -> na razie zle
  //R <<  std::cos(ang(1))*std::cos(ang(2)), -std::sin(ang(2)), 0.0,
  //      std::cos(ang(1))*std::sin(ang(2)), std::cos(ang(2)), 0.0,
  //      -std::sin(ang(1)), 0.0, 1.0;
  //r_wb = R_om * q_dot;
}

void ccmutils::R_eul(const Eigen::Vector3d &ang, Eigen::Matrix3d &R){
  //body rates to Euler rates
  //Eigen::Matrix3d R_eul;  
  // tutaj znaki w R są zmienione z jakiego spowodu (inne niż w Euler2R_123!)  -> w tej wersji dron jest w FRD (90 wg ENU)
  R <<  std::cos(ang(2))/std::cos(ang(1)), -std::sin(ang(2))/std::cos(ang(1)), 0.0,
          std::sin(ang(2)), std::cos(ang(2)), 0.0,
         -std::cos(ang(2))*std::tan(ang(1)), std::sin(ang(2))*std::tan(ang(1)), 1.0; 
  //q_dot = R_eul * r_wb;
  // w tej wersji dron jest w -90 wg ENU (90 wg NED) -> na razie zle
  //R <<  std::cos(ang(2))/std::cos(ang(1)), std::sin(ang(2))/std::cos(ang(1)), 0.0,
  //      -std::sin(ang(2)), std::cos(ang(2)), 0.0,
  //      std::cos(ang(2))*std::tan(ang(1)), std::sin(ang(2))*std::tan(ang(1)), 1.0;    
}
