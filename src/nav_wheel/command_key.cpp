#include <ros/ros.h>
#include <std_msgs/String.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// Global variable to store terminal settings
struct termios oldt;

// Function to configure terminal for non-blocking input
void setupTerminal() {
    struct termios newt;
    tcgetattr(STDIN_FILENO, &oldt);  // Save old terminal settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);  // Disable canonical mode and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
}

// Function to restore terminal settings when exiting
void restoreTerminal() {
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

// Function to check if a key is pressed (non-blocking)
bool kbhit() {
    struct timeval tv = {0L, 0L};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
}

// Function to get a single character (non-blocking)
char getch() {
    char ch = 0;
    if (read(STDIN_FILENO, &ch, 1) < 0) {
        return 0;
    }
    return ch;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_input_node");
    ros::NodeHandle nh;
    ros::Publisher key_pub = nh.advertise<std_msgs::String>("/keyboard_input", 10);

    ros::Rate loop_rate(10);  // 10 Hz loop rate
    setupTerminal();  // Configure terminal for non-blocking input

    ROS_INFO("Keyboard Input Node Started. Press keys to publish. Press 'q' to quit.");

    while (ros::ok()) {
        if (kbhit()) {  // Check if a key is pressed
            char key = getch();

            if (key == 'q') {
                ROS_INFO("Exiting keyboard input node.");
                break;
            }

            std_msgs::String msg;
            msg.data = std::string(1, key);  // Convert char to string
            key_pub.publish(msg);
            ROS_INFO("Published key: %c", key);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    restoreTerminal();  // Restore terminal settings before exiting
    return 0;
}
