#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <cmath>

class CubicPolyPlanner {
public:
    CubicPolyPlanner();
    Eigen::Vector3d get_position();
    void update();
    void set_target(ros::Duration target_time, Eigen::Vector3d target_pos,
        Eigen::Vector3d start_pos);
private:
    ros::Duration t;
    ros::Duration target_time;
    ros::Time start_time;
    Eigen::Vector3d start_pos;
    Eigen::Vector3d target_pos; 
};