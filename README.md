# centauro_horizon

## Perception

using YOLO

1. Set Up Your RGB-D Camera in ROS
   /camera/color/image_raw → RGB image
   /camera/depth/image_raw → Depth image
   /camera/depth/points → Point cloud
2. Object Detection (RGB-Based)
   sensor_msgs::Image

/D435_head_camera/aligned_depth_to_color/image_raw -> msg is sensor_msgs/Image -> RGB image

/D435_head_camera/color/image_raw -> Depth image

## Localization
