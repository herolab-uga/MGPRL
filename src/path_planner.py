import heapq
import numpy as np
from numba import njit

@njit(cache=True)
def A_STAR(grid, dist_cost_map, p1, p2, k=1):
    """Input grid shall be a m x n Numpy matrix, with 1 for grid cells representing obstacles, and 0 else.
    The algorithm also requires a cost_map, of the same size of the grid. Finally, p1 and p2 are the starting and goal point, respectively.
    """
 
    x_max, y_max = grid.shape
    
    # Initialize value function as inf, except for first point.
    value = np.full_like(grid, np.inf)
    value[p1[0],p1[1]] = grid[p1[0],p1[1]]

    if grid[p1[0],p1[1]] or grid[p2[0],p2[1]]: raise Exception("Start/goal is not in clear area.")

    # Matrices to store if point has been visited and which transition to be stored to retrieve path in the end.
    visited = np.zeros(grid.shape, dtype=np.bool_)
    transitions = np.zeros((x_max, y_max, 2), dtype=np.int32)
  
    # Priority queue, explore from most promising points (lower value function)
    priority_queue = []
    priority_queue.append((0., (p1[0],p1[1])))

    # Possible grid transitions
    dxy = [[0,1], [1,0], [-1,0], [0,-1], [1,1], [1,-1], [-1,1], [-1,-1]]
    
    while priority_queue:
        # Analyze grid point with lower value function --> take current pixels
        cur_cost, (cur_x, cur_y) = heapq.heappop(priority_queue)
        
        # Break if reached goal
        if cur_x == p2[0] and cur_y == p2[1]: break
        
        # Do not consider point if it has already been visited.
        if visited[cur_x, cur_y]: continue
        
        # Analyze all possible transitions
        for dx, dy in dxy:

            # New position in grid
            x, y = cur_x + dx, cur_y + dy
            # Travelling cost different for horiz/vert transitions and diagonal ones
            if abs(dx)+abs(dy)==2: L=np.sqrt(2)
            else: L=1
            # Don't consider if out of bounds
            if x < 0 or x >= x_max or y < 0 or y >= y_max: continue    
            # Don't consider if obstacle            
            if grid[x,y]: continue

            # Don't consider if already visited
            if ~visited[x,y]:
                dist_cost = k*dist_cost_map[x,y]   # activate to get smoothed A*, else 0

                # Only consider this point if it decreases the value function from the original point
                if value[cur_x, cur_y] + L + dist_cost < value[x,y]:       
                    
                    # Definition of the heuristic: first consider points which get closer to the end goal and stay far from obstacles
                    value[x,y] = value[cur_x,cur_y] + L + dist_cost               # Value function, keeps track of total cost of the path
                    cart_distance = np.sqrt( (p2[0]-x)**2 + (p2[1]-y)**2 )        # activate to get A*
                    heuristic = value[x,y]  + cart_distance                       # Heuristic to get closer to the goal faster
                    # Insert this new point in the priority queue
                    heapq.heappush(priority_queue, (heuristic, (x, y)))
                    # Store the transition to retrieve the path 
                    transitions[x,y,0] = cur_x
                    transitions[x,y,1] = cur_y

        # Mark this point as visited
        visited[cur_x,cur_y] = 1
        
    # retrieve the path. Go backwards from end goal to starting point, using the stored transitions 
    cur_x, cur_y = p2[0], p2[1]
    path = []
    path.append((cur_x,cur_y))

    step = 0
    while (cur_x, cur_y) != (p1[0], p1[1]):
        cur_x, cur_y = transitions[(cur_x, cur_y)]
        path.append((cur_x,cur_y))
        step+=1

        # To avoid getting stuck in the loop for any reason
        if step > 1000:
            return None

    return path

