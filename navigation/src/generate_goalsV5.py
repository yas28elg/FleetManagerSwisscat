# Difference dans la generation des taches, on ne peut pas avoir deux fois le meme goal
# On envoie les goals aux robots
# Structure finale
# On effectue les missions de A à Z 

# python3 run_experiments.py --disjoint --instance instances/circuit.txt --solver Prioritized


# Librairie
from pathlib import Path
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
import random
import numpy as np
from hungarian_algorithm import hungarian_algorithm, ans_calculation
from cbs import CBSSolver
#from prioritized import PrioritizedPlanningSolver

from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.msg import *


import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal




stations = { 'discovery' : (5, 2), 'omni' : (5, 10), 'synt' : (9, 10), 'sfc' : (0, 10) }
robots = { '0' : (17, 1), '1' : (19, 1), '2' : (3, 5), '3' : (17, 10),'4' : (19, 10)}
number_robot = 5
number_station = 4

class Robot():

    def __init__(self, nameRobot, initX, initY, number):
        self.name = nameRobot
        self.id = number
        self.posX = initX
        self.posY = initY
        self.goalX = [initX]
        self.goalY = [initY]
        self. path = []
        self.produit = np.array([[0, 1, -3.6],[-1, 0, 4.5], [0, 0, 1]]) @ (self.posX/2.1, self.posY/2.1, 1)
        self.posX_gazebo = round(self.produit[0], 2 )
        self.posY_gazebo = round(self.produit[1], 2 )
        self.path_gazebo = []
        self.checkpoint = []

    # in MAPF frame
    #posX 
    #posY 
    #goalX= posX
    #goalY= posY

    #path = []

    # in Gazebo frame

    #posX_gazebo = 0 
    #posY_gazebo = 0
    
    #path_gazebo = []


    def Position(self, odom_data):
        curr_time = odom_data.header.stamp
        self.posX_gazebo = round(odom_data.pose.pose.position.x, 2)
        self.posY_gazebo = round(odom_data.pose.pose.position.y, 2)

    # Changement de base MAPF 2 GAZEBO
    def base_MAPF2Gazebo(self):
        # input: paths in the MAPF frame
        # output: paths in the GAZEBO frame
        self.path_gazebo.clear()
        matrix_chg_base = np.array([[0, 1, -3.6],[-1, 0, 4.5], [0, 0, 1]])
        #print(self.name, len(self.path), self.path)
        for i in range(0, len(self.path)): #We start at 1 so we don't take into account the initial position NOT the case anymore
            path_temp = (self.path[i][0]/2.1, self.path[i][1]/2.1, 1)
            produit = matrix_chg_base @ path_temp
            self.path_gazebo.append((round(produit[0], 2), round(produit[1], 2)))
        #print('Path in Gazebo coordinate', self.name, self.path_gazebo)

    # Changement de base GAZEBO 2 MAPF
    def base_Gazebo2MAPF(self):
        # input: paths in the MAPF frame
        # output: paths in the GAZEBO frame
        matrix_chg_base = np.array([[0, -1, 4.5],[1, 0, 3.6], [0, 0, 1]])
        path_temp = (self.posX_gazebo, self.posY_gazebo, 1)
        produit = matrix_chg_base @ path_temp
        self.posX = int(round(produit[0]*2.1, 0))
        self.posY = int(round(produit[1]*2.1, 0))
        if self.posX > 19:
            self.posX = 19
            #print('X value higher than 19, in gazebo: ', self.posX_gazebo)
        if self.posY > 12:
            self.posX = 12
            #print('Y value higher than 13, in gazebo: ', self.posY_gazebo)
        #print('Pos in MAPF coordinate', self.name, self.posX, self.posY)

    # Envoyer position au robot
    def send_goal(self):
        # Create a move_base client
        name = '/'+ self.name + '/move_base'
        client = actionlib.SimpleActionClient(name, MoveBaseAction)
        # Wait for the action server to start up
        client.wait_for_server()

        #print('Goal send for robot', self.name )

        # Set up the goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.orientation.w = 1.0

        
        if len(self.path_gazebo)> 1:
            self.path_gazebo.pop(0)
            goal.target_pose.pose.position.x = self.path_gazebo[0][0]
            goal.target_pose.pose.position.y = self.path_gazebo[0][1]
            #print(self.path_gazebo[0][0], self.path_gazebo[0][1])
            
        else:
            goal.target_pose.pose.position.x = self.path_gazebo[0][0]
            goal.target_pose.pose.position.y = self.path_gazebo[0][1]
            #print(self.path_gazebo[0][0], self.path_gazebo[0][1])
        '''
        goal.target_pose.pose.position.x = self.path_gazebo[-1][0]
        goal.target_pose.pose.position.y = self.path_gazebo[-1][1]
        '''
        print('Goal send for robot', self.name, 'position', goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
        # Send the goal and wait for completion
        client.send_goal(goal)
        #client.wait_for_result()
        


        # find where the direction change from self.path
        def change_direction(self):
            for i in range(0, len(self.path)-1):
                if self.path[i][0] != self.path[i+1][0] or self.path[i][1] != self.path[i+1][1]:
                    self.checkpoint.append(self.path[i])



# MAP GENERATION (FOR A*)
def import_mapf_instance(filename):
    # input: text file representing the map
    # output: table containing free and occupied cells

    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    print('Map generated')
    return my_map


#TASK GENERATION
def task_generator(number_station, stations):
    # input: number of station and name of stations
    # output: list containing the tasks to perform
    number_task = random.randint(1, number_station)
    #number_task = 1
    tasks_to_do = random.sample(list(stations), k=number_task)
    #tasks_to_do = ['discovery'] #, 'omni', 'synt']
    #next_tasks_to_do =[ 'synt'] #, 'sfc', 'omni']
    print('Tasks to do', tasks_to_do)
    next_tasks_to_do = random.sample(list(stations), k=number_task)
    print('Next tasks to do', next_tasks_to_do)

    #tasks_to_do = ['discovery', 'omni', 'synt', 'sfc']
    #next_tasks_to_do = ['discovery', 'synt', 'sfc', 'omni']

    # check element wise if next_tasks_to_do is identical to tasks_to_do and change next_tasks_to_do if its the case 
    for i in range(0, number_task):
        if tasks_to_do[i] == next_tasks_to_do[i]:
            print('Same task ')
            station_list =  list(stations)
            station_list.remove(tasks_to_do[i])
            print('Station list', station_list)
            next_tasks_to_do[i] = random.choice(station_list)
   
    return tasks_to_do, next_tasks_to_do


# TASK ALLOCATION
# input: robots
# goal: update the robot goals if needed
def task_allocation(robot1, robot2, robot3, robot4, robot5, stations, tasks_to_do, next_tasks_to_do):
    
    starts_robots = [(robot1.posX, robot1.posY), (robot2.posX, robot2.posY), (robot3.posX, robot3.posY), (robot4.posX, robot4.posY), (robot5.posX, robot5.posY)]
    goals_allocation = []
    cost_matrix = np.zeros((number_robot,number_robot))

    for i in range(0, len(tasks_to_do)):
        goals_allocation.append(stations[tasks_to_do[i]])
    
    heuristics = []
    for goal in goals_allocation:
        heuristics.append(compute_heuristics(my_map, goal))

    # compute the cost matrix
    for i in range(0, len(robots)):
        for j in range(0, len(tasks_to_do)):
            path = a_star(my_map, starts_robots[i], goals_allocation[j], heuristics[j], 0, [])
            cost_matrix[i][j]=len(path)

    
    # Hungarian algorithm
    ans_pos = hungarian_algorithm(cost_matrix.copy())#Get the element position.
    ans, ans_mat = ans_calculation(cost_matrix, ans_pos)#Get the minimum or maximum value and corresponding matrix.

    result = np.nonzero(ans_mat)
    # result[0]: contains the index (lines) of the nonzero element - it corresponds to which robots should perform the task
    # result[1]: contains the index (column) of the nonzero element - it corresponds to which task should be performed
    goals_allocation = [(robot1.goalX, robot1.goalY), (robot2.goalX, robot2.goalY), (robot3.goalX, robot3.goalY), (robot4.goalX, robot4.goalY), (robot5.goalX, robot5.goalY)]

    # Update goals
    
    for j in range(0, len(result[0])):
        if result[0][j] == 0:
            robot1.goalX.append(np.array(stations[next_tasks_to_do[result[1][j]]])[0])
            robot1.goalY.append(np.array(stations[next_tasks_to_do[result[1][j]]])[1])
            robot1.goalX.append(np.array(stations[tasks_to_do[result[1][j]]])[0])
            robot1.goalY.append(np.array(stations[tasks_to_do[result[1][j]]])[1])
            print('robot 1 is taking the task', j)
        elif result[0][j] == 1:
            robot2.goalX.append(np.array(stations[next_tasks_to_do[result[1][j]]])[0])
            robot2.goalY.append(np.array(stations[next_tasks_to_do[result[1][j]]])[1])
            robot2.goalX.append(np.array(stations[tasks_to_do[result[1][j]]])[0])
            robot2.goalY.append(np.array(stations[tasks_to_do[result[1][j]]])[1])
            print('robot 2 is taking the task', j)
        elif result[0][j] == 2:
            robot3.goalX.append(np.array(stations[next_tasks_to_do[result[1][j]]])[0])
            robot3.goalY.append(np.array(stations[next_tasks_to_do[result[1][j]]])[1])
            robot3.goalX.append(np.array(stations[tasks_to_do[result[1][j]]])[0])
            robot3.goalY.append(np.array(stations[tasks_to_do[result[1][j]]])[1])
            print('robot 3 is taking the task', j)
        elif result[0][j] == 3:
            robot4.goalX.append(np.array(stations[next_tasks_to_do[result[1][j]]])[0])
            robot4.goalY.append(np.array(stations[next_tasks_to_do[result[1][j]]])[1])
            robot4.goalX.append(np.array(stations[tasks_to_do[result[1][j]]])[0])
            robot4.goalY.append(np.array(stations[tasks_to_do[result[1][j]]])[1])
            print('robot 4 is taking the task', j)
        elif result[0][j] == 4:
            robot5.goalX.append(np.array(stations[next_tasks_to_do[result[1][j]]])[0])
            robot5.goalY.append(np.array(stations[next_tasks_to_do[result[1][j]]])[1])
            robot5.goalX.append(np.array(stations[tasks_to_do[result[1][j]]])[0])
            robot5.goalY.append(np.array(stations[tasks_to_do[result[1][j]]])[1])
            print('robot 5 is taking the task', j)
            
    
    '''
    robot5.goalX.append(np.array(stations[next_tasks_to_do[result[1][j]]])[0])
    robot5.goalY.append(np.array(stations[next_tasks_to_do[result[1][j]]])[1])
    robot5.goalX.append(np.array(stations[tasks_to_do[result[1][j]]])[0])
    robot5.goalY.append(np.array(stations[tasks_to_do[result[1][j]]])[1])
    print('robot 5 is taking the task', j)
    '''
    goals_allocation = [(robot1.goalX[-1], robot1.goalY[-1]), (robot2.goalX[-1], robot2.goalY[-1]), (robot3.goalX[-1], robot3.goalY[-1]), (robot4.goalX[-1], robot4.goalY[-1]), (robot5.goalX[-1], robot5.goalY[-1])]
    
    return goals_allocation


# Envoyer position au robot
def send_goal(robot):
    # Create a move_base client
    name = '/'+ robot.name + '/move_base'
    client = actionlib.SimpleActionClient(name, MoveBaseAction)
    # Wait for the action server to start up
    client.wait_for_server()

    print('Goal send for robot', robot.name )

    # Set up the goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    
    if len(robot.path_gazebo)> 1:
        goal.target_pose.pose.position.x = robot.path_gazebo[-1][0]
        goal.target_pose.pose.position.y = robot.path_gazebo[-1][1]
        goal.target_pose.pose.orientation.w = 1.0

        print(robot.path_gazebo[-1][0], robot.path_gazebo[-1][1])
        robot.path_gazebo.pop(1)
        
    else:
        goal.target_pose.pose.position.x = robot.path_gazebo[0][0]
        goal.target_pose.pose.position.y = robot.path_gazebo[0][1]
        print(robot.path_gazebo[0][0], robot.path_gazebo[0][1])

    #goal.target_pose.pose.orientation.w = 1.0

    # Send the goal and wait for completion
    client.send_goal(goal)
    client.wait_for_result()

# Goal reached
# Check if goal is reached, if it's the case, remove from goal list
def check_goal(robot, goals):
    goal_temp = goals.copy()
    if(robot.posX == robot.goalX[-1] and robot.posY == robot.goalY[-1] and len(robot.goalX) > 1):
        print(robot.name, 'reached its goal')
        goal_temp.pop(robot.id)
        print(goal_temp)
        if (robot.goalX[-2], robot.goalY[-2]) in goal_temp:
            print('robot', robot.name, 'waits')
        else:
            # remove the goal from the robot
            robot.goalX.pop(-1)
            robot.goalY.pop(-1)
            # add the new goal in goals_allocation
            goals[robot.id]=(robot.goalX[-1], robot.goalY[-1])
            print('new goal for', robot.name, 'is', robot.goalX[-1], robot.goalY[-1])


if __name__ == '__main__':
    try:
        goals = []
        starts = []

        rospy.init_node('odometry', anonymous=True) #make node 
        #rospy.init_node('send_goal', anonymous=True)


        # init robots with  initial position and name
        robot1 = Robot('robot1', 17, 1, 0)
        robot2 = Robot('robot2', 19, 1, 1)
        robot3 = Robot('robot3', 3, 5, 2)
        robot4 = Robot('robot4', 17, 10, 3)
        robot5 = Robot('robot5', 19, 10, 4)

        
        # init map
        my_map = import_mapf_instance('circuit_MAPF.txt')
        
        # generate the tasks
        tasks_to_do, next_tasks_to_do = task_generator(number_station, stations)
        print('task generated')
        print(tasks_to_do, next_tasks_to_do)
        
        # task allocation
        goals = task_allocation(robot1, robot2, robot3, robot4, robot5, stations, tasks_to_do, next_tasks_to_do )
        print('Goals allocated', goals)
        print('robot1', robot1.goalX, robot1.goalY)
        print('robot2', robot2.goalX, robot2.goalY)
        print('robot3', robot3.goalX, robot3.goalY)
        print('robot4', robot4.goalX, robot4.goalY)
        print('robot5', robot5.goalX, robot5.goalY)
        
        '''
        starts = [(robot1.posX, robot1.posY), (robot2.posX, robot2.posY), (robot3.posX, robot3.posY), (robot4.posX, robot4.posY), (robot5.posX, robot5.posY)]
        print('Current position: ', starts)

        # Find paths
        cbs = CBSSolver(my_map, starts, goals)
        paths = cbs.find_solution('--disjoint')
        #print(paths)
        robot1.path = paths[0] 
        robot2.path = paths[1] 
        robot3.path = paths[2] 
        robot4.path = paths[3] 
        robot5.path = paths[4]
        print('Paths updated') 

        # Change frame MAPF 2 Gazebo (for path)
        
        robot1.base_MAPF2Gazebo()
        robot2.base_MAPF2Gazebo()
        robot3.base_MAPF2Gazebo()
        robot4.base_MAPF2Gazebo()
        robot5.base_MAPF2Gazebo()
        '''

        # Get position (Gazebo frame)
        rospy.Subscriber('/robot1/odom',Odometry,robot1.Position)
        rospy.Subscriber('/robot2/odom',Odometry,robot2.Position)
        rospy.Subscriber('/robot3/odom',Odometry,robot3.Position)
        rospy.Subscriber('/robot4/odom',Odometry,robot4.Position)
        rospy.Subscriber('/robot5/odom',Odometry,robot5.Position)

        while True:
            print('')
            print('---------------------------------------')

            print('Pos Gazebo robot 1:', robot1.posX_gazebo, robot1.posY_gazebo)
            print('Pos Gazebo robot 2:', robot2.posX_gazebo, robot2.posY_gazebo)
            print('Pos Gazebo robot 3:', robot3.posX_gazebo, robot3.posY_gazebo)
            print('Pos Gazebo robot 4:', robot4.posX_gazebo, robot4.posY_gazebo)
            print('Pos Gazebo robot 5:', robot5.posX_gazebo, robot5.posY_gazebo)

            # Change frame Gazebo 2 MAPF (position)
            robot1.base_Gazebo2MAPF()
            robot2.base_Gazebo2MAPF()
            robot3.base_Gazebo2MAPF()
            robot4.base_Gazebo2MAPF()
            robot5.base_Gazebo2MAPF()
            print('position updated - MAPF')


            starts = [(robot1.posX, robot1.posY), (robot2.posX, robot2.posY), (robot3.posX, robot3.posY), (robot4.posX, robot4.posY), (robot5.posX, robot5.posY)]
            print('Current position: ', starts)


            #check if goal is reached
            check_goal(robot1, goals)
            check_goal(robot2, goals)
            check_goal(robot3, goals)
            check_goal(robot4, goals)
            check_goal(robot5, goals)

            print('goals', goals)  


            print('finding new paths')
            # Find paths
            cbs = CBSSolver(my_map, starts, goals)
            paths = cbs.find_solution('--disjoint')

            #print(paths)
            robot1.path = paths[0] 
            robot2.path = paths[1] 
            robot3.path = paths[2] 
            robot4.path = paths[3] 
            robot5.path = paths[4]
            
            print('Paths updated')
            #print(robot1.name, robot1.path )
            #print(robot2.name, robot2.path )
            #print(robot3.name, robot3.path )
            #print(robot4.name, robot4.path )
            print(robot5.name, robot5.path )
            
            # Change frame MAPF 2 Gazebo (for path)
            robot1.base_MAPF2Gazebo()
            robot2.base_MAPF2Gazebo()
            robot3.base_MAPF2Gazebo()
            robot4.base_MAPF2Gazebo()
            robot5.base_MAPF2Gazebo()
            

            # Send goals
            
            robot1.send_goal()
            robot2.send_goal()
            robot3.send_goal()
            robot4.send_goal()
            robot5.send_goal()
            
            # Wait 1 sec
            rospy.sleep(0.1)
            
            
        rospy.spin()
       
            

        



        '''
        # Initialize ROS node
        rospy.init_node('send_goal', anonymous=True)
        coordinate = paths[2][-1]
        x = coordinate[0]
        y = coordinate[1]
        # Send some goals
        send_goal(x, y)  # send goal to (2, 0)
        #send_goal(1, 1)  # send goal to (1, 1)
        '''
    except rospy.ROSInterruptException:
        pass



