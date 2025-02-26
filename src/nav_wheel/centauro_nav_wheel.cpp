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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>

bool start_nav_bool = false;
geometry_msgs::PoseStamped current_goal;  // Store the current goal
// Callback function for receiving the goal pose from RViz
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_goal = *msg;  // Update the goal pose
}

void printMessage(const std::string& message) {
  std::cout << message << std::endl;
}

bool start_nav(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    start_nav_bool = !start_nav_bool;
    return true;
};

int main(int argc, char **argv)
{

    const double dt = 0.1; double time_ = 0.0 ;
    const std::string robotName = "centauro";
    // Initialize ros node
    ros::init(argc, argv, robotName);
    ros::NodeHandle nodeHandle("");

    std_srvs::Empty srv;
    // Create a Buffer and a TransformListener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    auto cfg = XBot::ConfigOptionsFromParamServer();
    // and we can make the model class
    auto model = XBot::ModelInterface::getModel(cfg);
    auto robot = XBot::RobotInterface::getRobot(cfg);
    // initialize to a homing configuration
    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    qhome[44] = 0.33;
    model->setJointPosition(qhome);
    model->update();
    XBot::Cartesian::Utils::RobotStatePublisher rspub (model);
    robot->setControlMode(
        {
            {"j_wheel_1", XBot::ControlMode::Velocity()},
            {"j_wheel_2", XBot::ControlMode::Velocity()},
            {"j_wheel_3", XBot::ControlMode::Velocity()},
            {"j_wheel_4", XBot::ControlMode::Velocity()}
        }
    );

    auto ctx = std::make_shared<XBot::Cartesian::Context>(
                std::make_shared<XBot::Cartesian::Parameters>(dt),
                model
            );

    // load the ik problem given a yaml file
    std::string problem_description_string;
    nodeHandle.getParam("problem_description_wheel", problem_description_string);
    auto ik_pb_yaml = YAML::Load(problem_description_string);
    XBot::Cartesian::ProblemDescription ik_pb(ik_pb_yaml, ctx);

    auto solver = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                       ik_pb, ctx
                                                       );
    ros::ServiceServer service = nodeHandle.advertiseService("start_nav", start_nav);

    // Subscribe to the 2D Nav Goal topic
    ros::Subscriber goal_subscriber = nodeHandle.subscribe("/move_base_simple/goal", 1, goalCallback);

    ros::Rate r(10);

    double roll_e, pitch_e, yaw_e;
    double K_x = 0.1, K_y = 0.2, K_yaw = 0.1;
    Eigen::VectorXd q, qdot, qddot;
    auto car_task = solver->getTask("base_link");
    auto car_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(car_task);
    Eigen::Vector6d E;
    while (ros::ok())
    {
        // Calculate the error between current position and goal position
        double x_e = current_goal.pose.position.x;
        double y_e = current_goal.pose.position.y;


        tf2::Quaternion q_;
        tf2::fromMsg(current_goal.pose.orientation, q_);

    }
        ros::spinOnce();
        r.sleep();
    return 0;
}
