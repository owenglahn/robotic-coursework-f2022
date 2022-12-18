#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <math.h>
#include <cmath>
#include <limits>

namespace base_controller {

enum State {
    START, MOVING, ARRIVED, WALL, COLLISION, COMPLETE
};

struct Limit {
    double speed;
    double omega;
};

class BaseController {
public:
    BaseController(ros::NodeHandle nodeHandle);
    ~BaseController();
    bool readParams();

    void updateLaserScan(const sensor_msgs::LaserScan laserScan);
    void updateFbk(const gazebo_msgs::ModelStates modelStates);
    bool collisionDetected();
    void update_function();
    bool objectInRange(const int fromStart, const int fromEnd);
    bool objectAtRight();
    double getTargetAngle();
    int getUpdateFrequency();
    geometry_msgs::Twist getTwist();

private:
    ros::NodeHandle node_handle;
    std::string modelName;
    std::string commandTopicName;
    std::string sensorTopicName;
    std::string fbkTopicName;
    ros::Publisher publisher;
    ros::Subscriber subscriber;
    ros::Subscriber poseSub;
    Eigen::Vector3d initial;
    double initPhi;
    Eigen::Vector3d target;
    double targetPhi;
    Limit limit;
    int updateFrequency;
    double minDistance;
    sensor_msgs::LaserScan laserScan;
    Eigen::Vector3d fbkPos;
    double fbkYaw;
    State state;
};


}