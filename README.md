Take robot off dock 

Terminal 1: ros2 run newNav_pkg get_obs
 - returns locations of obstacles in robot frame

Terminal 2: ros2 run lab2_pkg lab2
 - caps velocity and stops if obstacle too close

Terminal 3: ros2 run newNav_pkg go_to_goal
 - action server for going to goal
 - upon running, sets current robot location and orientation to 0, 0, 0 rad
 - something isn't right with it

Terminal 4: ros2 run newNav_pkg goal_client
 - sends robot goal
