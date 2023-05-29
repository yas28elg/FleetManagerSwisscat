# Difference dans la generation des taches, on ne peut pas avoir deux fois le meme goal
# On envoie les goals aux robots

# python3 run_experiments.py --disjoint --instance instances/circuit.txt --solver Prioritized

from pathlib import Path
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
import random
import numpy as np
from hungarian_algorithm import hungarian_algorithm, ans_calculation
#from visualize import Animation
from cbs import CBSSolver
#from prioritized import PrioritizedPlanningSolver

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal




stations = { 'discovery' : (5, 2), 'omni' : (5, 10), 'synt' : (9, 10), 'sfc' : (0, 10) }
robots = { '0' : (17, 1), '1' : (19, 1), '2' : (3, 5), '3' : (17, 10),'4' : (19, 10)}


# MAP GENERATION (FOR A*)

def import_mapf_instance(filename):
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
    return my_map

my_map = import_mapf_instance('circuit_MAPF.txt')


#TASK GENERATION
number_robot = len(robots)
number_station = len(stations)

def nb_task(number_station):
    nb = random.randint(1, number_station)
    return nb

number_task = nb_task(number_station)

tasks_to_do = random.sample(list(stations), k=number_task)
    
print(tasks_to_do)


# TASK ALLOCATION
starts_robots = list(robots.values())
goals_allocation = []
for i in range(0, number_task):
    goals_allocation.append(stations[tasks_to_do[i]])

print(goals_allocation)
cost_matrix = np.zeros((number_robot,number_robot))

heuristics = []
for goal in goals_allocation:
    heuristics.append(compute_heuristics(my_map, goal))

for i in range(0, number_robot):
    for j in range(0, number_task):
        path = a_star(my_map, starts_robots[i], goals_allocation[j], heuristics[j], 0, [])
        cost_matrix[i][j]=len(path)

#print(cost_matrix)

ans_pos = hungarian_algorithm(cost_matrix.copy())#Get the element position.
ans, ans_mat = ans_calculation(cost_matrix, ans_pos)#Get the minimum or maximum value and corresponding matrix.

#Show the result
#print(f"Linear Assignment problem result: {ans:.0f}\n{ans_mat}")

print('output hungarian', ans_mat)
result = np.nonzero(ans_mat) 
# result[0]: contains the index (lines) of the nonzero element - it corresponds to which robots should perform the task
# result[1]: contains the index (column) of the nonzero element - it corresponds to which task should be performed
#np.where( ans_mat > 0)
print('result: ', result[0])
#print(np.array(stations[tasks_to_do[0][0]])[0])
#print(len(result[0]))
#print(np.array(robots['0'])[0])


# Ecrire correctement qui fait quoi
mission = np.zeros((number_robot,4))
starts = []
goals = []

for i in range(0, number_robot):
    starts.append(robots[str(i)])
    mission[i][0]= np.array(robots[str(i)])[0]
    mission[i][1]= np.array(robots[str(i)])[1]
    mission[i][2]= np.array(robots[str(i)])[0]
    mission[i][3]= np.array(robots[str(i)])[1] 
    for j in range(0, len(result[0])):
        if result[0][j] == i:
            mission[i][2] = np.array(stations[tasks_to_do[result[1][j]]])[0]
            mission[i][3] = np.array(stations[tasks_to_do[result[1][j]]])[1]
    goals.append((int(mission[i][2]), int(mission[i][3])))


"""
solver = PrioritizedPlanningSolver(my_map, starts, goals)
paths = solver.find_solution()
print(paths)
"""

# Trouver les chemins
cbs = CBSSolver(my_map, starts, goals)
paths = cbs.find_solution('--disjoint')
print('avant  ', paths)
#print(paths[0][0][0])

# Changement de base
def changement_coodonnees(paths):
    matrix_chg_base = np.array([[0, 1, -3.6],[-1, 0, 4.5], [0, 0, 1]])
    #path_temp = []
    for i in range(0, len(paths)):
        for j in range(0, len(paths[i])):
            path_temp = (paths[i][j][0]/2.1, paths[i][j][1]/2.1, 1)
            #produit = np.dot(matrix_chg_base, path_temp.T)
            produit = matrix_chg_base @ path_temp
            #produit.pop(-1)
            paths[i][j] = (round(produit[0], 2), round(produit[1], 2))
    #print(path_temp)
    return paths

paths = changement_coodonnees(paths)
#print('apres  ', paths)

'''
paths_frame = paths.copy()
matrix_chg_base = [[-3.6, -3.2],[4.05, 4.55]]
for i in range(0, len(paths)):
    for j in range(0, len(paths[i])):
        produit = np.dot(matrix_chg_base, paths[i][j])
        produit[0]= round(produit[0], 3)
        produit[1]= round(produit[1], 3)
        paths_frame[i][j] = produit.tolist()
        #paths_frame.append()

#print(paths_frame)
#print(paths)
'''
#paths = changement_coodonnees(paths)
#print(paths)


# Envoyer position au robot
def send_goal(x, y):
    # Create a move_base client
    client = actionlib.SimpleActionClient('/robot3/move_base', MoveBaseAction)
    # Wait for the action server to start up
    client.wait_for_server()

    # Set up the goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0


    # Send the goal and wait for completion
    client.send_goal(goal)
    client.wait_for_result()


if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node('send_goal', anonymous=True)
        coordinate = paths[2][-1]
        x = coordinate[0]
        y = coordinate[1]
        # Send some goals
        send_goal(x, y)  # send goal to (2, 0)
        #send_goal(1, 1)  # send goal to (1, 1)
    except rospy.ROSInterruptException:
        pass



'''
test = []
with open('paths.txt', 'r') as file:
    # read a list of lines into data
    test = file.readlines()
    test[0] = str(paths[0]) + '\n'
    test[1] = str(paths[1]) + '\n'
    test[2] = str(paths[2]) + '\n'
    test[3] = str(paths[3]) + '\n'
    test[4] = str(paths[4]) + '\n'
with open('paths.txt', 'w') as file: 
    file.writelines( test )
file.close()
'''