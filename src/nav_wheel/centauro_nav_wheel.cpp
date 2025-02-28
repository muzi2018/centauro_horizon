#include <thread>
#include <ros/init.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
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
#include <termios.h>
#include <unistd.h>
#include <signal.h>

bool start_nav_bool = false;
// Store the last published transform
tf2::Transform last_published_transform;
ros::Time last_publish_time;
bool first_publish = true;
std::string latest_keyboard_input = "";


// Function to calculate the difference between two transforms
bool hasTransformChanged(const tf2::Transform& current_transform, const tf2::Transform& last_transform, double position_threshold, double rotation_threshold)
{
    // Calculate position (translation) difference
    double position_diff = (current_transform.getOrigin() - last_transform.getOrigin()).length();

    // Calculate rotation (orientation) difference (angle between quaternions)
    double rotation_diff = current_transform.getRotation().angleShortestPath(last_transform.getRotation());

    // Check if the position or rotation change exceeds the thresholds
    return position_diff > position_threshold || rotation_diff > rotation_threshold;
}



// Keyboard callback function
void keyboardCallback(const std_msgs::String::ConstPtr& msg)
{
    latest_keyboard_input = msg->data;
    ROS_INFO("Received keyboard input: %s", latest_keyboard_input.c_str());
}

int main(int argc, char **argv)
{
    const double dt = 0.1; double time_ = 0.0;
    const std::string robotName = "centauro";
    // Initialize ros node
    ros::init(argc, argv, robotName);
    ros::NodeHandle nodeHandle("");
    
    // Create TF buffer and listener
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    
    // Define thresholds for transform change detection
    const double position_threshold = 0.001; // meters
    const double rotation_threshold = 0.01; // radians
    
    // Subscribe to keyboard input
    ros::Subscriber key_sub = nodeHandle.subscribe("/keyboard_input", 10, keyboardCallback);

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
    XBot::Cartesian::Utils::RobotStatePublisher rspub(model);
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
    ros::Rate r(10);

    double x_e = 1, y_e = 1, yaw_e = 1;
    Eigen::VectorXd q, qdot, qddot;
    double K_x = 0.1, K_y = 0.1, K_yaw = 0.1;
    auto car_task = solver->getTask("base_link");
    auto car_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(car_task);
    Eigen::Vector6d E;


    while (ros::ok())
    {
        // Get the current transform between 'base_link' and 'world' (or any other frames you're interested in)
        tf2::Transform current_transform;
        try {
            // Look up the transform (replace "base_link" and "world" with the actual frames)
            geometry_msgs::TransformStamped transform_stamped = tf_buffer.lookupTransform("world", "base_link", ros::Time(0), ros::Duration(1.0));
            
            // Convert to tf2::Transform
            tf2::fromMsg(transform_stamped.transform, current_transform);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            continue;
        }


        if ( 0 ){
            E[0] = K_x * (x_e - 0);  
            E[1] = K_y * (y_e - 0);  
            E[2] = 0;
            E[3] = 0;
            E[4] = 0;
            E[5] = K_yaw * yaw_e;

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

        // Check if the transform has changed significantly
        if (first_publish || hasTransformChanged(current_transform, last_published_transform, position_threshold, rotation_threshold)) {
            // Publish transform only if the change exceeds the thresholds
            rspub.publishTransforms(ros::Time::now(), "");

            // Update last published transform and time
            last_published_transform = current_transform;
            last_publish_time = ros::Time::now();
            first_publish = false;
        } else {
            ROS_WARN("Skipping transform publish due to small change in transform.");
        }

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
