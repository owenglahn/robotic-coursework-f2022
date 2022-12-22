#include "base_controller/BaseController.hpp"

namespace base_controller {

BaseController::BaseController(ros::NodeHandle nodeHandle) {
    std::cout << "Calling constructor" << std::endl;
    this -> state = State::START;
    this -> node_handle = nodeHandle;
    this -> started = false;
    if (!readParams()) {
        ROS_ERROR("Unable to read params.");
        ros::requestShutdown();
    }
    this -> poseSub = node_handle.subscribe<gazebo_msgs::ModelStates>(fbkTopicName, 2, 
        &BaseController::updateFbk, this);
    this -> subscriber = node_handle.subscribe<sensor_msgs::LaserScan>(sensorTopicName, 2, 
        &BaseController::updateLaserScan, this); 
    this -> publisher = node_handle
        .advertise<geometry_msgs::Twist>(commandTopicName, 2);
    this -> fbkPos = this -> initial;
    this -> laserScan.ranges.insert(this -> laserScan.ranges.end(), 720, 
        std::numeric_limits<double>::max());
    this -> server = node_handle.advertiseService("/base_controller/start", 
        &BaseController::serviceCallBack, this);
}
BaseController::~BaseController() {
}

bool BaseController::readParams() {
    if (!node_handle.getParam("model_name", modelName) ||
        !node_handle.getParam("cmd_topic_name", commandTopicName) || 
        !node_handle.getParam("sensor_topic_name", sensorTopicName) || 
        !node_handle.getParam("fbk_topic_name", fbkTopicName) || 
        !node_handle.getParam("initial/x", initial[0]) ||
        !node_handle.getParam("initial/y", initial[1]) ||
        !node_handle.getParam("initial/phi", initPhi) ||
        !node_handle.getParam("target/x", target[0]) || 
        !node_handle.getParam("target/y", target[1]) || 
        !node_handle.getParam("target/phi", targetPhi) || 
        !node_handle.getParam("update_frequency", updateFrequency) ||
        !node_handle.getParam("min_distance_to_obstacles", minDistance) ||
        !node_handle.getParam("limit/speed", limit.speed) ||
        !node_handle.getParam("limit/omega", limit.omega)) { 
        return false;
    }
    return true;
}

void BaseController::updateLaserScan(const sensor_msgs::LaserScan laserScan) {
    this -> laserScan = laserScan;
}

void BaseController::updateFbk(const gazebo_msgs::ModelStates modelStates) {
    int i = 0;
    for (; i < modelStates.name.size(); i++) {
        if (modelStates.name[i].compare("husky") == 0) {
            break;
        }
    }
    this -> fbkPos[0] = modelStates.pose[i].position.x;
    this -> fbkPos[1] = modelStates.pose[i].position.y;
    this -> fbkPos[2] = modelStates.pose[i].position.z;
    std::cout << "X FBK: " << std::endl << fbkPos[0] << std::endl;
    std::cout << "Y FBK: " << std::endl << fbkPos[1] << std::endl;
    std::cout << "Z FBK: " << std::endl << fbkPos[2] << std::endl;

    this -> fbkYaw = tf::getYaw(modelStates.pose[i].orientation);
    std::cout << "FBK YAW: " << this -> fbkYaw << std::endl;

    std::cout << "DISTANCE TO TARGET: " << (this -> fbkPos - this -> target).norm() << std::endl;
}
bool BaseController::serviceCallBack(base_controller::start::Request &req, 
    base_controller::start::Response &res) {
        this -> started = true;
    return this -> started;
}

bool BaseController::objectInRange(const int fromStart, const int fromEnd) {
    std::vector<float> ranges(720 - fromStart - fromEnd);
    std::copy(this -> laserScan.ranges.begin() + fromStart, 
        this -> laserScan.ranges.end() - fromEnd, ranges.begin());
    float minInRange = *(std::min_element(ranges.begin(), ranges.end()));
    std::cout << "Min in range: " << minInRange << std::endl;
    return minInRange <= this -> minDistance;
}

bool BaseController::collisionDetected() {
    return this -> objectInRange(180, 180);
}

bool BaseController::objectAtRight() {
    return this -> objectInRange(450, 0);
}

double BaseController::getTargetAngle() {
    Eigen::Vector3d difference = this -> target - this -> initial;
    return std::atan2(difference[1], difference[0]);
}

int BaseController::getUpdateFrequency() {
    return this -> updateFrequency;
}

geometry_msgs::Twist BaseController::getTwist() {
    std::cout << "State: " << this -> state << std::endl;
    geometry_msgs::Twist toReturn;
    switch (this -> state) {
    case State::START: {
        double angleDiff = this -> fbkYaw - this -> getTargetAngle();
        toReturn.linear.x = 0;
        if (std::abs(angleDiff) <= .1) {
            this -> state = State::MOVING;
        } else if (angleDiff > 0) {
            toReturn.angular.z = -1 * this -> limit.omega;
        } else {
            toReturn.angular.z = this -> limit.omega;
        }
        break;
    }
    case State::MOVING: {
        if (this -> collisionDetected()) {
            this -> state = State::COLLISION;
        } else if ((this -> fbkPos - this -> target).norm() < 1) {
            this -> state = State::ARRIVED;
        } else {
            toReturn.linear.x = this -> limit.speed;
            toReturn.angular.z = 0;
        }
        break;
    }
    case State::COLLISION: {
        if (this -> collisionDetected()) {
            toReturn.linear.x =0;
            toReturn.angular.z = this -> limit.omega;
            break;
        }
        this -> state = State::WALL;
        break;
    }
    case State::WALL: {
        double D = (this -> fbkPos - this -> initial)
            .cross(this -> target - this -> initial).norm() 
            / (this -> target - this -> initial).norm();
        std::cout << "D: " << D << std::endl;
        if (D < 0.5) {
            this -> state = State::START;
            break;
        }
        if (this -> collisionDetected()) {
            this -> state = State::COLLISION;
            break;
        }
        if (!this -> objectAtRight()) {
            toReturn.linear.x = .1;
            toReturn.angular.z = -.3;
            break;
        }
        toReturn.linear.x = this -> limit.speed;
        toReturn.angular.z = 0;
        break;
    }
    case State::ARRIVED: {
        if (std::abs(this -> fbkYaw - this -> initPhi) < .1) {
            this -> state = State::COMPLETE;
            break;
        }
        toReturn.angular.z = this -> fbkYaw - this -> initPhi > 0? -1 * this -> limit.omega: 
            this -> limit.omega;
        toReturn.linear.x = 0;
        break;
    }
    default:
        toReturn.linear.x = 0;
        toReturn.angular.z = 0;
        break;
    }
    return toReturn;
}

void BaseController::update_function() {
    std::cout << "Calling update function" << std::endl;
    if (this -> started) {
        this -> publisher.publish(this -> getTwist());
    }
}
}