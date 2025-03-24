# centauro_horizon

## Use

### Detection

start simulation: roslaunch centauro_horizon centauro_startup.launch
start control: xbot2-gui
start detection: roslaunch centauro_horizon centauro_perception.launch

### Localization

catkin_localization file
	roslaunch hdl_localization hdl_localization.launch

### Navigation

centauro_ros_nav file
	roslaunch centauro_ros_nav centauro_nav_exploration.launch
	clear && python navigation.py
