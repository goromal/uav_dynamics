#include "uav_dynamics/uav_dynamics_ros.h"

namespace uav_dynamics {

QuadrotorDynamicsROS::QuadrotorDynamicsROS() : nh_(), nh_private_("~"), br_(), uav_(),
    input_((Vector4d() << 0., 0., 0., 0.).finished()),
    ext_w_((Vector6d() << 0., 0., 0., 0., 0., 0.).finished()), prev_time_(0.0)
{
    std::string mav_name = "NO_MAV_NAME_GIVEN";
    if (nh_private_.hasParam("mav_name"))
        nh_private_.getParam("mav_name", mav_name);
    std::string truth_msg_string = "/truth/NED";

    // Set up ROS objects
    motor_wrench_sub_ = nh_.subscribe("uav_motor_wrench", 1, &QuadrotorDynamicsROS::wrenchCallback, this);
    ext_wrench_sub_ = nh_.subscribe("uav_ext_wrench", 1, &QuadrotorDynamicsROS::extWrenchCallback, this);
    truth_pub_      = nh_.advertise<nav_msgs::Odometry>(mav_name + truth_msg_string, 1);
    sim_state_pub_  = nh_.advertise<rosflight_sil::ROSflightSimState>("uav_truth_NED", 1);
    uav_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("uav_marker", 1);

    // Load parameters into dynamic model
    grav_ = nh_private_.param<double>("gravity", 9.80665);
    std::vector<double> x0_vec;
    if (nh_private_.hasParam("truth_x0"))
    {
        nh_private_.getParam("truth_x0", x0_vec);
    }
    else
    {
        x0_vec.push_back(0.0); x0_vec.push_back(0.0); x0_vec.push_back(0.0); // pos
        x0_vec.push_back(1.0); x0_vec.push_back(0.0); x0_vec.push_back(0.0); x0_vec.push_back(0.0); // att
        x0_vec.push_back(0.0); x0_vec.push_back(0.0); x0_vec.push_back(0.0); // vel
        x0_vec.push_back(0.0); x0_vec.push_back(0.0); x0_vec.push_back(0.0); // omega
    }
    Matrix<double, 13, 1> x0(x0_vec.data());
    mass_ = nh_private_.param<double>("mass", 2.0);
    Matrix3d inertia;

    double uav_ixx = 0.07; // default
    if (nh_private_.hasParam("uav_ixx"))
        nh_private_.getParam("uav_ixx", uav_ixx);

    double uav_iyy = 0.07;
    if (nh_private_.hasParam("uav_iyy"))
        nh_private_.getParam("uav_iyy", uav_iyy);

    double uav_izz = 0.12;
    if (nh_private_.hasParam("uav_izz"))
        nh_private_.getParam("uav_izz", uav_izz);

    inertia = (Vector3d() << uav_ixx, uav_iyy, uav_izz).finished().asDiagonal();
//    double drag_const = nh_.param<double>("uav_linear_mu", 0.05);
//    double angular_drag = nh_.param<double>("uav_angular_mu", 0.0005);
    Vector3d gravity = Vector3d(0., 0., grav_);
    uav_.loadParameters(x0, mass_, inertia, gravity); //, drag_const, angular_drag);
    eq_thrust_ = mass_ * grav_;

    // Set up infinite loop
    timer_ = nh_.createTimer(ros::Duration(ros::Rate(1000)), &QuadrotorDynamicsROS::run, this);
}

void QuadrotorDynamicsROS::wrenchCallback(const geometry_msgs::Wrench &msg)
{
    input_(THRUST) = -msg.force.z;
    input_(TAUX) = msg.torque.x;
    input_(TAUY) = msg.torque.y;
    input_(TAUZ) = msg.torque.z;
}

void QuadrotorDynamicsROS::extWrenchCallback(const geometry_msgs::Wrench &msg)
{
    ext_w_(0) = msg.force.x;
    ext_w_(1) = msg.force.y;
    ext_w_(2) = msg.force.z;
    ext_w_(3) = msg.torque.x;
    ext_w_(4) = msg.torque.y;
    ext_w_(5) = msg.torque.z;
}

void QuadrotorDynamicsROS::run(const ros::TimerEvent &)
{
    // handle time calculation
    double now_time = ros::Time::now().toSec();
    double dt = now_time - prev_time_;
    prev_time_ = now_time;

    // integrate dynamics with most recent command IF we're not gonna fall through the floor
    if (uav_.get_state().p(2) >= 0.0  && input_(THRUST) < mass_ * grav_) // grounded
    {
        ext_w_.setZero();
        ext_w_(2) = -mass_ * grav_; // for IMU calibration; needs to look like there's a normal force acting on the UAV!
        input_.setZero();
    }
    // else // <<<< FOR TESTING
    //     ext_w_.setZero();
    uav_.run(dt, input_, ext_w_);
//    if (!(uav_.get_state().p(2) < 0.0) && !(input_(THRUST) > 0.5)) // eq_thrust_)
    if (uav_.get_state().p(2) > 0.0 /*&& input_(THRUST) < 0.4*/) // grounded
    {
        State grounded_state;
        (grounded_state.arr << uav_.get_state().p(0), uav_.get_state().p(0), 0.0,
                1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0).finished();
        uav_.set_state(grounded_state);
    }

    // publish full truth state and transform
    State state = uav_.get_state();
    Vector3d imu_accel = uav_.get_imu_accel();
    Vector3d imu_gyros = uav_.get_imu_gyro();
    rosflight_sil::ROSflightSimState state_msg;
    state_msg.header.stamp = ros::Time::now();
    state_msg.pos.x = state.p(0);
    state_msg.pos.y = state.p(1);
    state_msg.pos.z = state.p(2);
    state_msg.att.w = state.q.w();
    state_msg.att.x = state.q.x();
    state_msg.att.y = state.q.y();
    state_msg.att.z = state.q.z();
    state_msg.vel.x = state.v(0);
    state_msg.vel.y = state.v(1);
    state_msg.vel.z = state.v(2);
    state_msg.w.x = state.w(0);
    state_msg.w.y = state.w(1);
    state_msg.w.z = state.w(2);
    state_msg.imu_accel.x = imu_accel(0);
    state_msg.imu_accel.y = imu_accel(1);
    state_msg.imu_accel.z = imu_accel(2);
    state_msg.imu_gyro.x = imu_gyros(0);
    state_msg.imu_gyro.y = imu_gyros(1);
    state_msg.imu_gyro.z = imu_gyros(2);
    sim_state_pub_.publish(state_msg);
    geometry_msgs::TransformStamped NED2UAV;
    NED2UAV.header.stamp = ros::Time::now();
    NED2UAV.header.frame_id = "NED";
    NED2UAV.child_frame_id = "UAV";
    NED2UAV.transform.translation = state_msg.pos;
    NED2UAV.transform.rotation = state_msg.att;
    br_.sendTransform(NED2UAV);
    nav_msgs::Odometry truth_msg;
    truth_msg.header.stamp = ros::Time::now();
    truth_msg.pose.pose.position.x = state.p(0);
    truth_msg.pose.pose.position.y = state.p(1);
    truth_msg.pose.pose.position.z = state.p(2);
    truth_msg.pose.pose.orientation = state_msg.att;
    truth_msg.twist.twist.linear = state_msg.vel;
    truth_msg.twist.twist.angular = state_msg.w;
    truth_pub_.publish(truth_msg);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "NED";
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.mesh_resource = "package://uav_dynamics/mesh/quadrotor_bodyaxes.dae";
    marker.pose.position.x = state_msg.pos.x;
    marker.pose.position.y = state_msg.pos.y;
    marker.pose.position.z = state_msg.pos.z;
    marker.pose.orientation = state_msg.att;
    marker.scale.x = 1.25;
    marker.scale.y = 1.25;
    marker.scale.z = 1.25;
    marker.color.b = 0.0f;
    marker.color.g = 1.0f;
    marker.color.r = 1.0f;
    marker.color.a = 1.0f;
    uav_marker_pub_.publish(marker);
}

} // end namespace uav_dynamics
