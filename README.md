# FleetManagerSwisscat
This script generates goals for robots, computes non-colliding paths, and sends them to the robots.
## Libraries
- `pathlib.Path`: Used to work with file paths.
- `single_agent_planner`: A module for computing heuristics and A* algorithm.
- `random`: Used for generating random numbers.
- `numpy`: Used for numerical computations.
- `munkres.Munkres`: A module for the Hungarian algorithm.
- `cbs.CBSSolver`: A module for solving the Conflict-Based Search (CBS) problem.
- `prioritized.PrioritizedPlanningSolver`: A module for solving the Prioritized Planning problem.
- `nav_msgs.msg.Odometry`: A module for getting robot position from Gazebo.
- `geometry_msgs.msg`: A module for defining geometry messages.
- `tf.msg`: A module for handling transformations between coordinate frames.
- `configparser`: Used for parsing configuration files.
- `rospy`: A Python client library for ROS.
- `actionlib`: A module for implementing simple action clients.
- `move_base_msgs.msg`: A module for sending goals to robots using the move_base package.
## Variables
- `time_begin`: A float variable representing the start time.
- `checkpoint_thres`: A threshold value used to determine if a robot has reached a checkpoint.
- `scale_var`: A scale factor used to convert coordinates from the MAPF frame to the GAZEBO frame.
## Classes
### Robot
A class representing a robot.
- `__init__(self, nameRobot, initX, initY, number)`: Initializes the robot with a name, initial position, and ID.
- `Position(self, odom_data)`: Updates the robot’s position from Gazebo simulation data.
- `base_MAPF2Gazebo(self)`: Converts the robot’s path from the MAPF frame to the GAZEBO frame.
- `base_Gazebo2MAPF(self)`: Converts the robot’s position from the GAZEBO frame to the MAPF frame.
- `send_goal(self)`: Sends a goal to the robot using the move_base package.
### Station
A class representing a station.
- `__init__(self, name, x, y)`: Initializes the station with a name and position.
## Functions
### import_mapf_instance(filename)
Imports a map from a text file.
- `filename`: A string representing the file name.
### task_generator(stations)
Generates tasks to be performed by the robots.
- `stations`: A list of Station objects.
### task_allocation(stations, tasks_to_do, next_tasks_to_do)
Allocates tasks to the robots.
- `stations`: A list of Station objects.
- `tasks_to_do`: A list of collecting stations for each task.
- `next_tasks_to_do`: A list of delivery stations for each task.
### update_goal(robot, goals)
Updates the robot’s goal.
- `robot`: A Robot object.
- `goals`: A list of goals.
### check_checkpoint(robots)
Checks if the robots have reached their checkpoints.
- `robots`: A list of Robot objects.
### send_checkpoint(robots)
Sends the next checkpoint to the robots.
- `robots`: A list of Robot objects.
### state_task(robots)
Checks if all robots have completed their tasks.
- `robots`: A list of Robot objects.
## Usage
1. Install the required libraries and packages.
2. Set up the configuration file with the necessary parameters.
3. Run main.launch
4. Run navigation.lauch  
5. Run the Fleet Manager script.

