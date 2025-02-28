﻿#include <thread>
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
#include <termios.h>
#include <unistd.h>

bool start_nav_bool = false;

// Global variables for keyboard control
bool keys_pressed[256] = {false};
bool keyboard_control_active = true;

// Store terminal settings globally
struct termios oldt;
struct termios newt;  // Declare both structures globally

void reset_terminal_mode(void) {
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

char getch(void) {
    char ch;
    
    // Modify terminal settings for non-blocking input
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    
    // Read character
    ch = getchar();
    
    // Restore original settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}


void keyboardControlThread(void* arg) {
    char c;
    
    // Print control instructions
    ROS_INFO_STREAM("\nKeyboard Controls:");
    ROS_INFO_STREAM("W/S: Forward/Backward");
    ROS_INFO_STREAM("A/D: Left/Right");
    ROS_INFO_STREAM("Q/E: Rotate Left/Right");
    ROS_INFO_STREAM("X: Exit\n");

    while (ros::ok()) {
        // Read keyboard input without blocking
        if (read(STDIN_FILENO, &c, 1) > 0) {
            switch(c) {
                case 'w':
                    keys_pressed['w'] = true;
                    break;
                case 's':
                    keys_pressed['s'] = true;
                    break;
                case 'a':
                    keys_pressed['a'] = true;
                    break;
                case 'd':
                    keys_pressed['d'] = true;
                    break;
                case 'q':
                    keys_pressed['q'] = true;
                    break;
                case 'e':
                    keys_pressed['e'] = true;
                    break;
                case '\n':  // Handle Enter key
                    break;
                default:
                    keys_pressed[c] = true;
            }
        }
        
        // Reset pressed keys
        usleep(50000);  // 50ms delay
        
        for(int i = 0; i < 256; ++i) {
            keys_pressed[i] = false;
        }
    }
}



int main(int argc, char **argv)
{

    const double dt = 0.1; double time_ = 0.0 ;
    const std::string robotName = "centauro";
    // Initialize ros node
    ros::init(argc, argv, robotName);
    ros::NodeHandle nodeHandle("");


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
    ros::Rate r(10);

    double x_e = 1, y_e = 1, yaw_e = 1;
    Eigen::VectorXd q, qdot, qddot;
    double K_x = 0.1, K_y = 0.1, K_yaw = 0.1;
    auto car_task = solver->getTask("base_link");
    auto car_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(car_task);
    Eigen::Vector6d E;

    // Create keyboard control thread
    std::thread keyboard_thread(keyboardControlThread, nullptr);
    keyboard_thread.detach();


    while (ros::ok())
    {
        if ( keys_pressed['w'] == true ){
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
        rspub.publishTransforms(ros::Time::now(), "");

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
