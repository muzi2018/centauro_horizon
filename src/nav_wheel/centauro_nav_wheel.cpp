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
tf2_ros::Buffer tfBuffer;  // Define tfBuffer globally so it can be accessed from callbacks
// Callback function for receiving the goal pose from RViz
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Store the original goal
    current_goal = *msg;
    
    // Transform the goal to world frame
    geometry_msgs::PoseStamped goal_in_world;
    try {
        // Transform the goal pose to world frame
        tfBuffer.transform(*msg, goal_in_world, "world");
        
        // Update current_goal with the transformed pose
        current_goal = goal_in_world;
        
        // Print debug information
        ROS_INFO_STREAM("Transformed goal to world frame:");
        ROS_INFO_STREAM("Position: (" << current_goal.pose.position.x << ", " 
                  << current_goal.pose.position.y << ", " 
                  << current_goal.pose.position.z << ")");
        ROS_INFO_STREAM("Orientation: (" << current_goal.pose.orientation.x << ", " 
                  << current_goal.pose.orientation.y << ", " 
                  << current_goal.pose.orientation.z << ", " 
                  << current_goal.pose.orientation.w << ")");
    }
    catch(tf2::TransformException &ex) {
        ROS_ERROR_STREAM("Failed to transform goal pose to world frame: " << ex.what());
    }
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
    // Create a TransformListener for the global tfBuffer
    tf2_ros::TransformListener tfListener(tfBuffer);

    auto cfg = XBot::ConfigOptionsFromParamServer();
    // and we can make the model class
    auto model = XBot::ModelInterface::getModel(cfg);
    auto robot = XBot::RobotInterface::getRobot(cfg);
    // initialize to a homing configuration
    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    // qhome[44] = 0.33;
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
        if (!current_goal.header.stamp.isZero()){
            // Get current robot pose using TF2
            geometry_msgs::PoseStamped current_pose;
            try {
                    // Get the transform as TransformStamped
                    geometry_msgs::TransformStamped transformStamped =
                        tfBuffer.lookupTransform("world", "base_link", ros::Time(0), ros::Duration(1.0));
                    // Convert TransformStamped to PoseStamped
                    current_pose.header = transformStamped.header;
                    current_pose.pose.position.x = transformStamped.transform.translation.x;
                    current_pose.pose.position.y = transformStamped.transform.translation.y;
                    current_pose.pose.position.z = transformStamped.transform.translation.z;
                    current_pose.pose.orientation = transformStamped.transform.rotation;
            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                continue;
            }
            
            // Calculate the error between current position and goal position
            double x_e = current_goal.pose.position.x - current_pose.pose.position.x;
            double y_e = current_goal.pose.position.y - current_pose.pose.position.y;

            // std::cout << "x_e: " ;
            // std::cout << x_e << std::endl;
            // std::cout << "y_e: " ;
            // std::cout << y_e << std::endl;

            // Calculate yaw errors
            tf2::Quaternion q_goal, q_current;
            tf2::fromMsg(current_goal.pose.orientation, q_goal);
            tf2::fromMsg(current_pose.pose.orientation, q_current);

            double roll_goal, pitch_goal, yaw_goal;
            double roll_current, pitch_current, yaw_current;

            tf2::Matrix3x3(q_goal).getRPY(roll_goal, pitch_goal, yaw_goal);
            tf2::Matrix3x3(q_current).getRPY(roll_current, pitch_current, yaw_current);

            // Calculate yaw error (difference between goal and current yaw)
            double yaw_error = yaw_goal - yaw_current;

            // Normalize the yaw error to [-π, π] range
            yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));
            // Calculate the error in position and orientation
            E[0] = K_x * (x_e - 0);  // Assuming current x = 0 for simplicity
            E[1] = K_y * (y_e - 0);  // Assuming current y = 0 for simplicity
            E[2] = 0;
            E[3] = 0;
            E[4] = 0;
            E[5] = K_yaw * yaw_error;

            // Set velocity references to control the robot towards the goal
            car_cartesian->setVelocityReference(E);

            solver->update(time_, dt);
            model->getJointPosition(q);
            model->getJointVelocity(qdot);
            model->getJointAcceleration(qddot);
            q += dt * qdot + 0.5 * std::pow(dt, 2) * qddot;
            qdot += dt * qddot;
            model->setJointPosition(q);
            model->setJointVelocity(qdot);
            model->update();
            robot->setPositionReference(q.tail(robot->getJointNum()));
            robot->setVelocityReference(qdot.tail(robot->getJointNum()));
            robot->move();
        }
        time_ += dt;
        rspub.publishTransforms(ros::Time::now(), "");

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
