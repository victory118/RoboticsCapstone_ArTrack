#!/usr/bin/python

import numpy as np
import yaml

def dijkstras(occupancy_map,x_spacing,y_spacing,start,goal):
    """
    Implements Dijkstra's shortest path algorithm
    Input:
    occupancy_map - an N by M numpy array of boolean values (represented
        as integers 0 and 1) that represents the locations of the obstacles
        in the world
    x_spacing - parameter representing spacing between adjacent columns
    y_spacing - parameter representing spacing between adjacent rows
    start - a 3 by 1 numpy array of (x,y,theta) for the starting position 
    goal - a 3 by 1 numpy array of (x,y,theta) for the finishing position 
    Output: 
    path: list of the indices of the nodes on the shortest path found
        starting with "start" and ending with "end" (each node is in
        metric coordinates)
    """
    # pass
    
    # Map that keeps track of the state of each grid cell
    state_map = np.zeros(occupancy_map.shape)
    state_map[occupancy_map == 0] = 1 # Mark free cells
    state_map[occupancy_map == 1] = 2 # Mark obstacle cells
    
    # Translate starting position to node index
    # iStart = int(round(start[1]/y_spacing - 0.5))
    # jStart = int(round(start[0]/x_spacing - 0.5))
    x_grid = np.arange(state_map.shape[1])*x_spacing
    y_grid = np.arange(state_map.shape[0])*y_spacing
    
    tol = 1e-6
    startBtw2xNodes = False
    startBtw2yNodes = False
    if abs(round(start[0]/x_spacing) - start[0]/x_spacing) < tol:
        jStart = np.searchsorted(x_grid, start[0])
        startBtw2xNodes = True
    else:
        jStart = np.searchsorted(x_grid, start[0]) - 1
        
    if abs(round(start[1]/y_spacing) - start[1]/y_spacing) < tol:
        iStart = np.searchsorted(y_grid, start[1])
        startBtw2yNodes = True
    else:
        iStart = np.searchsorted(y_grid, start[1]) - 1
      
    if startBtw2xNodes and startBtw2yNodes:
        possibleStartNodes = [(iStart,jStart),(iStart,jStart-1),(iStart-1,jStart),(iStart-1,jStart-1)]
    elif startBtw2xNodes:
        possibleStartNodes = [(iStart,jStart),(iStart,jStart-1)]
    elif startBtw2yNodes:
        possibleStartNodes = [(iStart,jStart),(iStart-1,jStart)]
    else:
        possibleStartNodes = [(iStart,jStart)]
    
        
    # print(jStart)
    start_node = (iStart.item(), jStart.item())
    start_linInd = np.ravel_multi_index(start_node, state_map.shape)
    
    # Translate goal position to node index
    # iGoal = int(round(goal[1]/y_spacing - 0.5))
    # jGoal = int(round(goal[0]/x_spacing - 0.5))
    
    goalBtw2xNodes = False
    goalBtw2yNodes = False
    
    if abs(round(goal[0]/x_spacing) - goal[0]/x_spacing) < tol:
        jGoal = np.searchsorted(x_grid, goal[0])
        goalBtw2xNodes = True
    else:
        jGoal = np.searchsorted(x_grid, goal[0]) - 1
        
    if abs(round(goal[1]/y_spacing) - goal[1]/y_spacing) < tol:
        iGoal = np.searchsorted(y_grid, goal[1])
        goalBtw2yNodes = True
    else:
        iGoal = np.searchsorted(y_grid, goal[1]) - 1
        
    if goalBtw2xNodes and goalBtw2yNodes:
        possibleGoalNodes = [(iGoal,jGoal),(iGoal,jGoal-1),(iGoal-1,jGoal),(iGoal-1,jGoal-1)]
    elif goalBtw2xNodes:
        possibleGoalNodes = [(iGoal,jGoal),(iGoal,jGoal-1)]
    elif goalBtw2yNodes:
        possibleGoalNodes = [(iGoal,jGoal),(iGoal-1,jGoal)]
    else:
        possibleGoalNodes = [(iGoal,jGoal)]
   
    # goal_node = (iGoal.item(), jGoal.item())
    # goal_linInd = np.ravel_multi_index(goal_node, state_map.shape)
    
    # Initialize distance array
    distanceFromStart = np.full(state_map.shape, np.inf)
    
    # For each grid cell this array holds the index of its parent
    parent = np.zeros(state_map.shape)
    
    distanceFromStart[start_node] = 0
    
    # Main Loop
    
    # linInd = np.ravel_multi_index((0,1), test_mat.shape)
    # np.unravel_index(3, test_mat.shape)
    
    while True:
        
        state_map[start_node] = 5
        # state_map[goal_node] = 6
        # print(state_map)
       
        # Find the node with the minimum distance from start
       
        min_dist = np.min(distanceFromStart)
        current_linInd = np.argmin(distanceFromStart) # linear index of new current node
        current_node = np.unravel_index(np.argmin(distanceFromStart), distanceFromStart.shape) # (i,j) tuple
       
        if (current_node in possibleGoalNodes):
            goal_node = current_node
            break
            
        # Update map
        state_map[current_node] = 3 # Mark current node as visited
        # print(state_map)
      
        # Visit each neighbor of the current node and update the map, distances, and parent tables
        iVec = np.array([current_node[0] - 1, current_node[0], current_node[0] + 1, current_node[0]])
        jVec = np.array([current_node[1], current_node[1] + 1, current_node[1], current_node[1] - 1])
        
        neighbor_nodes = [tuple([iVec[i],jVec[i]]) for i in range(4)]
        # print(neighbor_nodes)
        
        for neighbor in neighbor_nodes:
            
            # Skip node if out of range
            if neighbor[0] < 0 or neighbor[0] > state_map.shape[0] - 1:
                continue
                
            if neighbor[1] < 0 or neighbor[1] > state_map.shape[1] - 1:
                continue
                
            # Skip node if obstacle, already visited, or start node
            if state_map[neighbor] == 2 or state_map[neighbor] == 3 or neighbor == start_node:
                continue
                
            if state_map[neighbor] == 4:
                if distanceFromStart[neighbor] > distanceFromStart[current_node] + 1:
                    distanceFromStart[neighbor] = distanceFromStart[current_node] + 1
                    parent[neighbor] = current_linInd
            else:
                state_map[neighbor] = 4 # Add node to be considered
                distanceFromStart[neighbor] = distanceFromStart[current_node] + 1
                parent[neighbor] = current_linInd
            
            # print(state_map)
            # print(parent)
                
        
        distanceFromStart[current_node] = np.inf # Remove this node from further consideration
        
    # Construct route from start to goal by following the parent links
    #print('out of the loop')
    #print(iStart.item())
    #print(jStart)
    #print(iGoal)
    #print(jGoal)
    #print(state_map)
    state_map[goal_node] = 6
    parent = parent.astype(int)
    if (distanceFromStart[goal_node] == np.inf):
        path = np.array([])
    else:
        path = np.array([goal_node])
        #print(path)
        #print(path[0])
        #print(parent)
        parent_linInd = parent[tuple(path[0])]
        #print(parent_linInd)
        #print(path)
        
        while (parent_linInd != 0):
            parent_node = np.unravel_index(parent_linInd, parent.shape)
            #print(parent_node)
            path = np.concatenate((np.array([parent_node]), path))
            
            if parent_node in possibleStartNodes:
                start_node = parent_node
                break
                
            parent_linInd = parent[tuple(path[0])]
    
    # print(path)        
    path_xy = path.astype(float)
    path_xy[:,0] = (path[:,1] + 0.5)*x_spacing
    path_xy[:,1] = (path[:,0] + 0.5)*y_spacing
    
    # print(path_xy)
    
    startDist = np.sqrt((start[0][0]-path_xy[0][0])**2 + (start[1][0]-path_xy[0][1])**2)
    
    if startDist > tol:
        start_xy = np.array([start[[0,1]].flatten()])
        path_xy = np.concatenate((start_xy,path_xy))
        # print(start_xy)
    
    goalDist = np.sqrt((goal[0][0]-path_xy[-1][0])**2 + (goal[1][0]-path_xy[-1][1])**2)
    
    if goalDist > tol:
        goal_xy = np.array([goal[[0,1]].flatten()])
        path_xy = np.concatenate((path_xy,goal_xy))
        # print(goal_xy)
        
    # print(path_xy)
    
    return path_xy
        

def test():
    """
    Function that provides a few examples of maps and their solution paths
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 0.13
    y_spacing1 = 0.2
    start1 = np.array([[0.3], [0.3], [0]])
    goal1 = np.array([[0.6], [1], [0]])
    #path1, path1_xy = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    path1_xy = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    #print path1_xy
    true_path1 = np.array([
        [ 0.3  ,  0.3  ],
        [ 0.325,  0.3  ],
        [ 0.325,  0.5  ],
        [ 0.325,  0.7  ],
        [ 0.455,  0.7  ],
        [ 0.455,  0.9  ],
        [ 0.585,  0.9  ],
        [ 0.600,  1.0  ]
        ])
    print path1_xy
    print true_path1
    if np.array_equal(path1_xy,true_path1):
      print("Path 1 passes")

    test_map2 = np.array([
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [1, 1, 1, 1, 1, 1, 1, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.5], [1.0], [1.5707963267948966]])
    goal2 = np.array([[1.1], [0.9], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    #path2, path2_xy = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    path2_xy = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    # print path2_xy
    true_path2 = np.array([[ 0.5,  1.0],
                           [ 0.5,  1.1],
                           [ 0.5,  1.3],
                           [ 0.5,  1.5],
                           [ 0.7,  1.5],
                           [ 0.9,  1.5],
                           [ 1.1,  1.5],
                           [ 1.1,  1.3],
                           [ 1.1,  1.1],
                           [ 1.1,  0.9]])
    print path2_xy
    print true_path2
    if np.array_equal(path2_xy,true_path2):
      print("Path 2 passes")
      
    # return path2, path2_xy

def test_for_grader():
    """
    Function that provides the test paths for submission
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 1, 0, 0, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 0, 0, 1, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 1
    y_spacing1 = 1
    start1 = np.array([[1.5], [1.5], [0]])
    goal1 = np.array([[7.5], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    s = 0
    for i in range(len(path1)-1):
      s += np.sqrt((path1[i][0]-path1[i+1][0])**2 + (path1[i][1]-path1[i+1][1])**2)
    print("Path 1 length:")
    print(s)


    test_map2 = np.array([
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.4], [0.4], [1.5707963267948966]])
    goal2 = np.array([[0.4], [1.8], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    s = 0
    for i in range(len(path2)-1):
      s += np.sqrt((path2[i][0]-path2[i+1][0])**2 + (path2[i][1]-path2[i+1][1])**2)
    print("Path 2 length:")
    print(s)



def main():
    # Load parameters from yaml
    param_path = 'params.yaml' # rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    # Get params we need
    occupancy_map = np.array(params['occupancy_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']
    test()
    #test_for_grader()
    #path, path_xy = dijkstras(occupancy_map,x_spacing,y_spacing,pos_init,pos_goal)
    #print(path)
    #return occupancy_map, pos_init, pos_goal, x_spacing, y_spacing, path, path_xy
    #path1, path1_xy = test()
    #print(path1_xy)
    #return occupancy_map, pos_init, pos_goal, x_spacing, y_spacing, path1, path1_xy

if __name__ == '__main__':
    main()

