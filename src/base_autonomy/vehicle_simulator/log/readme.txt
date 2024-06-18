Exploration metrics and vehicle trajectory are saved in this folder.

'metric_xxx.txt' contains four columns, respectively explored volume (m^3), traveling distance (m), algorithm runtime (second), and time duration from start of the run (second). Note that the runtime is recorded by receiving numbers as 'std_msgs::Float32' typed messages on ROS topic '/runtime'.

'trajectory_xxx.txt' contains seven columns, respectively x (m), y (m), z (m), roll (rad), pitch (rad), yaw (rad), and time duration from start of the run (second).
