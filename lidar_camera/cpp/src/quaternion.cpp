#include <iostream>
#include <Eigen/Core>

#include "quaternion.hpp"


using namespace std;
using namespace transforms;


//Default
Quaternion::Quaternion(){
    q << 0.0, 0.0, 0.0, 1.0;
}

Quaternion::~Quaternion(){
   
}


Quaternion::Quaternion(const Quaternion& src):Quaternion(src.q, src.point){


}
// Quaternion 
Quaternion::Quaternion(const Eigen::Vector3d& v):point(true){


    q(0) = v(0);
    q(1) = v(1);
    q(2) = v(2);
    q(3) = 0;

    magnitude = q.norm();
   
}

// Quaternion 
Quaternion::Quaternion(const Eigen::Vector4d& v, bool p ):point(p), q(v){


    
    // converting to unit quat, if not point
    if (!point) {
            q.normalize();
        }
    
    // can be avoided 
    magnitude = q.norm();
   
}

// Euler angles or Rvec
Quaternion::Quaternion(const Eigen::Vector3d& v, bool deg = false, bool rvec = false):point(false){

    if (rvec){

        double angle = sqrt(pow(v(0),2) + pow(v(1),2) + pow(v(2),2));
        q(0) = v(0)/angle * sin(angle/2);
        q(1) = v(1)/angle * sin(angle/2);
        q(2) = v(2)/angle * sin(angle/2);
        q(3) = cos(angle/2);

    }else{

        double cy = cos(v(0) * 0.5);
        double sy = sin(v(0) * 0.5);
        double cp = cos(v(1) * 0.5);
        double sp = sin(v(1) * 0.5);
        double cr = cos(v(2) * 0.5);
        double sr = sin(v(2) * 0.5);

        q(3)= cr * cp * cy + sr * sp * sy;
        q(0)= sr * cp * cy - cr * sp * sy;
        q(1)= cr * sp * cy + sr * cp * sy;
        q(2)= cr * cp * sy - sr * sp * cy;
        
    }
    q.normalize();

    // can be avoided 
    magnitude = q.norm();

}

// Rotation matrix
Quaternion::Quaternion(const Eigen::Matrix3d& m):point(false){

    double tr = m.trace(); 

    if (tr > 0){

        double S = sqrt(tr+1.0) * 2;
        
        q(0) = (m(2,1) - m(1,2)) / S;
        q(1) = (m(0,2) - m(2,0)) / S;
        q(2) = (m(1,0) - m(0,1)) / S; 
        q(3) = 0.25 * S;

    }else if((q(0,0) > q(1,1)) && (q(0,0) > q(2,2))){

        double S = sqrt(1.0 + m(0,0) - m(1,1) - m(2,2)) * 2;

        q(0) = 0.25 * S;
        q(1) = (m(0,1) + m(1,0)) / S; 
        q(2) = (m(0,2) + m(2,0)) / S;
        q(3) = (m(2,1) - m(1,2)) / S;
        
    }else if(q(1,1) > q(2,2)){

        double S = sqrt(1.0 + m(1,1) - m(0,0) - m(2,2)) * 2;

        q(0) = (m(0,1) + m(1,0)) / S;
        q(1) = 0.25 * S;
        q(2) = (m(1,2) + m(2,1)) / S;
        q(3) = (m(0,2) - m(2,0)) / S;


    }else{

        double S = sqrt(1.0 + m(2,2) - m(0,0) - m(1,1)) * 2;

        q(0) = (m(0,2) + m(2,0)) / S;
        q(1) = (m(1,2) + m(2,1)) / S;
        q(2) = 0.25 * S;
        q(3) = (m(1,0) - m(0,1)) / S;

    }

    q.normalize();

    // can be avoided 
    magnitude = q.norm(); 

}

// Operator overloading ------

Quaternion Quaternion::operator+(const Quaternion& rhs){

    // May not be memory efficient but done to prevent changing teh original quat value
    
    Eigen::Vector4d result_q  = q + rhs.q;

    return Quaternion(result_q, rhs.point);
}
Quaternion Quaternion::operator-(const Quaternion& rhs){
    // May not be memory efficient but done to prevent changing teh original quat values

    Eigen::Vector4d result_q = q - rhs.q;

    return Quaternion(result_q, rhs.point);
}

Quaternion Quaternion::operator*(const Quaternion& rhs){

    Eigen::Vector4d result_q ;
    result_q(0) =  q(0)* rhs.q(3) + q(1) * rhs.q(2) - q(2) * rhs.q(1) + q(3) * rhs.q(0);
    result_q(1) = -q(0)* rhs.q(2) + q(1) * rhs.q(3) + q(2) * rhs.q(0)+ q(3) * rhs.q(1);
    result_q(2) =  q(0)* rhs.q(1) - q(1) * rhs.q(0)+ q(2) * rhs.q(3) + q(3) * rhs.q(2);
    result_q(3) = -q(0)* rhs.q(0)- q(1) * rhs.q(1) - q(2) * rhs.q(2) + q(3) * rhs.q(3);

    return Quaternion(result_q, rhs.point);
}


Quaternion Quaternion::operator/(const double& scalar){
    q /= scalar;
    return Quaternion(q, point);
}

// Getters ---------------

void Quaternion::getRotationMatrix(Eigen::Matrix3d& m){


}

Eigen::Vector4d Quaternion::getQuaternion() const{
    return q;
}

bool Quaternion::getPoint() const{
    return point;
}

void Quaternion::getEulerAngles(Eigen::Vector3d& v){

    //ZYX - current case if xyz needed then : z = x, x=z
    // Roll (x-axis rotation)
        double sinr_cosp = 2 * (q(3)* q(0) + q(1) * q(2));
        double cosr_cosp = 1 - 2 * (q(0) * q(0) + q(1) * q(1));
        double roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
        double pitch;
        double sinp = 2 * (q(3)* q(1) - q(2) * q(0));
        if (fabs(sinp) >= 1){
            pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range

        }else{
            pitch = asin(sinp);
        }

    // Yaw (z-axis rotation)
        double siny_cosp = 2 * (q(3)* q(2) + q(0) * q(1));
        double cosy_cosp = 1 - 2 * (q(1) * q(1) + q(2) * q(2));
        double yaw = atan2(siny_cosp, cosy_cosp);

        v(0) = roll;
        v(1) = pitch;
        v(2) = yaw;
}



namespace transforms {
    ostream& operator<<(ostream& os, const Quaternion& quat){

        os << "x: " << quat.q(0) << " y: " << quat.q(1) << " z: "<< quat.q(2) << " w: " << quat.q(3) ;

        return os;
    }
}



