The following is the list of examples that are in the repo.

|Item|Description|Path|
|-|-|-|
|Create package and subscribe|A simple ros package that communciates with the nav_msg to get the moving robot position|./basic_nav/|
|Wall following robot|A simple program run that guides a robot based on distance calcuated from lightrays. | ./wall-follwing/ |
|Services|Use of interfaces, services and clients in Ros2 | ./executors_exercises_pkg/ |
|Executors|Demonstration of differennt types of executors | ./services_quiz* |
|Find wall and follow|Find wall based on simple obstruction detection, align and move along. | ./wall-following-with-wall-detection |
|Actions|Odometer distance calc with example usage of actions| ./actions/|
|URDF|Examples of various robot models| <ul><li>Fixed joint robot</li><ul> | ./simple |


Commands

cd ~/ros2_ws
colcon build --packages-select topic_subscriber_pkg
source install/setup.bash