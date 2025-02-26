#include <thread>
#include <ros/init.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/ModelInterface.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <std_srvs/Empty.h>
#include <xbot_msgs/JointCommand.h>
#include <std_msgs/Bool.h>

void printMessage(const std::string& message) {
  std::cout << message << std::endl;
}

int main() {

    // const std::string robotName = "centauro";
    // // Initialize ros node
    // ros::init(argc, argv, robotName);
    // ros::NodeHandle nodeHandle("");

    // std_srvs::Empty srv;
    // // Create a Buffer and a TransformListener
    // tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener tfListener(tfBuffer);

    // auto cfg = XBot::ConfigOptionsFromParamServer();
    // // and we can make the model class
    // auto model = XBot::ModelInterface::getModel(cfg);
    // auto robot = XBot::RobotInterface::getRobot(cfg);
    // // initialize to a homing configuration
    // Eigen::VectorXd qhome;
    // model->getRobotState("home", qhome);
    // qhome[44] = 0.33;
    // model->setJointPosition(qhome);
    // model->update();
    // XBot::Cartesian::Utils::RobotStatePublisher rspub (model);
    // robot->setControlMode(
    //     {
    //         {"j_wheel_1", XBot::ControlMode::Velocity()},
    //         {"j_wheel_2", XBot::ControlMode::Velocity()},
    //         {"j_wheel_3", XBot::ControlMode::Velocity()},
    //         {"j_wheel_4", XBot::ControlMode::Velocity()}
    //     }
    // );

    // auto ctx = std::make_shared<XBot::Cartesian::Context>(
    //             std::make_shared<XBot::Cartesian::Parameters>(dt),
    //             model
    //         );

    // // load the ik problem given a yaml file
    // std::string problem_description_string;
    // nodeHandle.getParam("problem_description_wheel", problem_description_string);
    // auto ik_pb_yaml = YAML::Load(problem_description_string);
    // XBot::Cartesian::ProblemDescription ik_pb(ik_pb_yaml, ctx);


    printMessage("Hello, world!");
    return 0;
}
