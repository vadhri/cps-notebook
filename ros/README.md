The following is the list of examples that are in the repo.

|Item|Description|Path|
|-|-|-|
|Create package and subscribe|A simple ros package that communciates with the nav_msg to get the moving robot position|./basic_nav/|
|Wall following robot|A simple program run that guides a robot based on distance calcuated from lightrays. | ./wall-follwing/ |

Commands

cd ~/ros2_ws
colcon build --packages-select topic_subscriber_pkg
source install/setup.bash