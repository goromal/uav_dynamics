#include <ros/ros.h>
#include "uav_dynamics/uav_dynamics.h"
#include "geometry-utils-lib/xform.h"
#include "logging-utils-lib/logging.h"
#include "rosflight_sil/ROSflightSimState.h"
#include "geometry_msgs/Wrench.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <iostream>

namespace uav_dynamics {

class QuadrotorDynamicsROS
{
public:
    QuadrotorDynamicsROS();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    tf2_ros::TransformBroadcaster br_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    ros::Publisher sim_state_pub_;
    ros::Publisher rel_truth_pub_;
    ros::Publisher truth_pub_;
    ros::Publisher uav_marker_pub_;
    ros::Subscriber motor_wrench_sub_;
    ros::Subscriber ext_wrench_sub_;
    ros::Timer timer_;

    geometry_msgs::TransformStamped tf_BOAT_UAV_;
    Xformd T_BOAT_UAV_;
    Vector3d v_BOAT_NED_;

    QuadrotorDynamics uav_;
    double grav_;
    double mass_;
    Vector4d input_;
    Vector6d ext_w_;
    double prev_time_;
    double eq_thrust_;

    void wrenchCallback(const geometry_msgs::Wrench &msg);
    void extWrenchCallback(const geometry_msgs::Wrench &msg);
    void run(const ros::TimerEvent&);
};

} // end namespace uav_dynamics
