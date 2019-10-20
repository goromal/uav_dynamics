#include <ros/ros.h>
#include "uav_dynamics/uav_dynamics.h"
#include "utils/xform.h"
#include "utils/logging.h"
#include "rosflight_msgs/ROSflightSimState.h"
#include "geometry_msgs/Wrench.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <iostream>

namespace uav_dynamics {

class QuadrotorDynamicsROS
{
public:
    QuadrotorDynamicsROS();
private:
    ros::NodeHandle nh_;
    tf2_ros::TransformBroadcaster br_;
    ros::Publisher sim_state_pub_;
    ros::Publisher uav_marker_pub_;
    ros::Subscriber motor_wrench_sub_;
    ros::Subscriber ext_wrench_sub_;
    ros::Timer timer_;

    QuadrotorDynamics uav_;
    double grav_;
    Vector4d input_;
    Vector6d ext_w_;
    double prev_time_;
    double eq_thrust_;

    void wrenchCallback(const geometry_msgs::Wrench &msg);
    void extWrenchCallback(const geometry_msgs::Wrench &msg);
    void run(const ros::TimerEvent&);
};

} // end namespace uav_dynamics
