#ifndef _RL_ALTITUDE_CONTROL_H_
#define _RL_ALTITUDE_CONTROL_H_
#include <mavros_msgs/AttitudeTarget.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/TwistStamped.h>
#include <iostream>
#include <math.h>
#include <fstream>

#define minimum_snap_Row 9
#define minimum_snap_Col 24
#define R_circle 2
#define mass 1.5
#define g 9.8
#define attctrl_tau_ 0.1
#define traj_T  2.5
double traj_omega_ = 2*M_PI/traj_T;

Eigen::Vector3d traj_radial_(1,0,0);
Eigen::Vector3d traj_axis_(0,0,1);
Eigen::Vector3d traj_origin_(0,0,1);
Eigen::Matrix<double,minimum_snap_Row,minimum_snap_Col> _polyCoeff;
Eigen::Matrix<double,minimum_snap_Row,1> _polyTime;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped local_pos_position;
Eigen::Vector3d pos_current;
geometry_msgs::TwistStamped local_vel_current;
Eigen::Vector3d ver_current;
mavros_msgs::AttitudeTarget local_attitude_target;

class rl_altitude_control{
    public:
        double time;
        Eigen::Vector3d getPosition(double time);
        Eigen::Vector3d getVelocity(double time);
        Eigen::Vector3d getAcceleration(double time);
        Eigen::Vector3d getPosPoly( Eigen::MatrixXd polyCoeff, int k, double t );
        Eigen::Vector3d getVelPoly( Eigen::MatrixXd polyCoeff, int k, double t );
        Eigen::Vector3d getAccPoly( Eigen::MatrixXd polyCoeff, int k, double t );
        void visWayPointTraj(const Eigen::Matrix<double,minimum_snap_Row,minimum_snap_Col> &polyCoeff, 
                    const Eigen::Matrix<double,minimum_snap_Row,1> &timer, double time,
                    Eigen::Vector3d &Trajectory_pos,Eigen::Vector3d &Trajectory_vel,
                    Eigen::Vector3d &Trajectory_acc);
};

inline Eigen::Vector3d ToEigen(const geometry_msgs::Point &point)
{
    Eigen::Vector3d tmp;
    tmp << point.x, point.y, point.z;
    return tmp;
}
inline Eigen::Vector3d ToEigen(const geometry_msgs::Vector3 &ev)
{
    Eigen::Vector3d tmp;
    tmp << ev.x, ev.y, ev.z;
    return tmp;
}

Eigen::Vector3d attcontroller(const Eigen::Vector4f &att_ref, const Eigen::Vector4f &curr_att)
{
    Eigen::Vector4f  qe, q_inv, inverse;
    Eigen::Vector3d ratecmd;
    
    inverse<< 1.0, -1.0, -1.0, -1.0;
    q_inv = inverse.asDiagonal() * curr_att;   //四元数 求逆，虚部相反数
    qe = quatMultiplication(q_inv,att_ref);
    ratecmd(0) = (2.0 / attctrl_tau_) *  qe(1);
    ratecmd(1) = (2.0 / attctrl_tau_) *  qe(2);
    ratecmd(2) = (2.0 / attctrl_tau_) *  qe(3);
    return ratecmd;
}

void Circle_trajectory(const geometry_msgs::PoseStamped &pos, const ros::Time &t1)
{
    Eigen::Vector3d Trajectory_pos,Trajectory_vel,Trajectory_acc;
    ros::Duration t = ros::Time::now() - t1;
    rl_altitude_control rl;
    rl.time = t.toSec();
    // rl.visWayPointTraj(_polyCoeff,_polyTime,rl.time,Trajectory_pos,Trajectory_vel,Trajectory_acc);
    Trajectory_pos = rl.getPosition(rl.time);
    Trajectory_vel = rl.getVelocity(rl.time);
    Trajectory_acc = rl.getAcceleration(rl.time);
    // std::cout<<Trajectory_pos.transpose()<<std::endl;
    Eigen::Vector3d pos_err = pos_current - Trajectory_pos;
    Eigen::Vector3d vel_err = ver_current - Trajectory_vel;


    Eigen::Vector3d k_pos,k_vel;
    k_pos << -8 ,-8 , -10;
    k_vel << -1.5, -3.3,  -3.3;

    Eigen::Vector4f quat_current;
    quat_current << pos.pose.orientation.w, pos.pose.orientation.x, 
                    pos.pose.orientation.y, pos.pose.orientation.z;
    Eigen::Matrix3d rotation_current;
    rotation_current = quat2RotMatrix(quat_current);
    double thrust;

    double psi = 0;
    Eigen::Vector3d a_fb = k_pos.asDiagonal()*pos_err + k_vel.asDiagonal()*vel_err;
    
    if(a_fb.norm() > 8) 
        a_fb = (8 / a_fb.norm()) * a_fb;
    
    const Eigen::Vector3d a_des = a_fb + g*Eigen::Vector3d(0,0,1) + Trajectory_acc;
    thrust = 0.05*a_des.dot(rotation_current*Eigen::Vector3d(0,0,1));

    Eigen::Vector3d proj_xb_des(cos(psi),sin(psi),0);
    Eigen::Vector3d zb_des = a_des/a_des.norm(); 
    Eigen::Vector3d yb_des = zb_des.cross(proj_xb_des)/(zb_des.cross(proj_xb_des).norm());
    Eigen::Vector3d xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();
    Eigen::Matrix3d R_des;
    R_des << xb_des(0), yb_des(0), zb_des(0),
            xb_des(1), yb_des(1), zb_des(1),
            xb_des(2), yb_des(2), zb_des(2);
    // std::cout<<thrust<<"\t"<<pos.pose.position.z<<std::endl;

    Eigen::Vector4f quat_des = rot2Quaternion(R_des);
    Eigen::Vector4f quat_curr(quat_current);

    Eigen::Vector3d ratecmd = attcontroller(quat_des,quat_curr);
    local_attitude_target.body_rate.x = ratecmd(0);
    local_attitude_target.body_rate.y = ratecmd(1);
    local_attitude_target.body_rate.z = ratecmd(2);
    local_attitude_target.thrust = std::max(0.0,std::min(1.0, thrust+0.1));
    local_attitude_target.type_mask = 0b10111000;

}

Eigen::Vector3d rl_altitude_control::getPosition(double time){
    Eigen::Vector3d position;
    double theta;

    theta = traj_omega_* time;
    // std::cout<<theta<<std::endl;
    position = std::cos(theta) * traj_radial_
                + std::sin(theta) * traj_axis_.cross(traj_radial_)
                + (1 - std::cos(theta)) * traj_axis_.dot(traj_radial_) * traj_axis_
                + traj_origin_;
    return position;
}

Eigen::Vector3d rl_altitude_control::getVelocity(double time){
    Eigen::Vector3d velocity;
    velocity = traj_omega_ * traj_axis_.cross(getPosition(time));
    return velocity;
}

Eigen::Vector3d rl_altitude_control::getAcceleration(double time){
    Eigen::Vector3d acceleration;
    acceleration = traj_omega_ * traj_axis_.cross(getVelocity(time));
    return acceleration;
}                                               // this method only useful for circle trajectory
 
Eigen::Vector3d rl_altitude_control::getPosPoly( Eigen::MatrixXd polyCoeff, int k, double t )
{
    Eigen::Vector3d ret;
    uint8_t _poly_num1D = 8;
    for ( int dim = 0; dim < 3; dim++ )
    {
        Eigen::VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        Eigen::VectorXd time  = Eigen::VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
        {
            if(j==0)
                time(j) = 1.0;
            else
                time(j) = pow(t, j);
        }
        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }
    return ret;
}

Eigen::Vector3d rl_altitude_control::getVelPoly( Eigen::MatrixXd polyCoeff, int k, double t )
{
    Eigen::Vector3d ret;
    uint8_t _poly_num1D = 8;
    for ( int dim = 0; dim < 3; dim++ )
    {
        Eigen::VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        Eigen::VectorXd time  = Eigen::VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
        {
            if(j==0)
                time(j) = 0.0;
            else if(j==1)
                time(j) = 1.0;
            else
                time(j) = j*pow(t, j-1);
        }
        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }
    return ret;
}

Eigen::Vector3d rl_altitude_control::getAccPoly( Eigen::MatrixXd polyCoeff, int k, double t )
{
    Eigen::Vector3d ret;
    uint8_t _poly_num1D = 8;
    for ( int dim = 0; dim < 3; dim++ )
    {
        Eigen::VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        Eigen::VectorXd time  = Eigen::VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
        {
            if(j==0||j==1)
                time(j) = 0.0;
            else if(j==2)
                time(j) = 2.0;
            else
                time(j) = j*(j-1)*pow(t, j-2);
        }
        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }
    return ret;
}

void rl_altitude_control::visWayPointTraj(const Eigen::Matrix<double,minimum_snap_Row,minimum_snap_Col> &polyCoeff, 
                    const Eigen::Matrix<double,minimum_snap_Row,1> &timer, double time,
                    Eigen::Vector3d &Trajectory_pos,Eigen::Vector3d &Trajectory_vel,
                    Eigen::Vector3d &Trajectory_acc)
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Matrix<double,minimum_snap_Row,1> time_cut;
    rl_altitude_control rl;
    for(uint8_t i=0;i<timer.size();i++)
    {
        if(i==0){
            time_cut(i) = 0;
        }
        else{
            time_cut(i) = time_cut(i-1)+timer(i);
        }
    }
    // std::cout<<time_cut<<std::endl;
    if(time>time_cut(minimum_snap_Row-1))
    {
        Trajectory_pos = Eigen::Vector3d(0,0,2);
        Trajectory_vel = Eigen::Vector3d::Zero();
        Trajectory_acc = Eigen::Vector3d::Zero();
    }
    else
    {
        for(uint8_t i=0;i<minimum_snap_Row-1;i++)
        {
            if(time>=time_cut(i) && time < time_cut(i+1))
            {
                Trajectory_pos = getPosPoly(polyCoeff, i, time - time_cut(i));
                Trajectory_vel = getVelPoly(polyCoeff, i, time - time_cut(i));
                Trajectory_acc = getAccPoly(polyCoeff, i, time - time_cut(i));
            }
        }
    }

    // std::cout<<Trajectory_pos.transpose()<<std::endl;
}

#endif