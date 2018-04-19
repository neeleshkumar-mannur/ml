import numpy as np
from config import *

class Robot(object):
    def __init__(self, maze_dim):
        
        self.location = [0, 0]
        self.heading = 'up'

        # Navigation specific variables
        self.backwards = 0  # Initial value of Down direction.
        self.steps = 0 # Number of steps taken in each trial
        self.trial = 0 # Exploration or Optimization Trial
        self.centre_found = False

        # Maze specific variables
        self.maze_dimension = maze_dim
        self.maze_area = maze_dim ** 2. # Example 12 X 12 = 144 squares
      
        # Grid specific variables
        self.maze_grid = np.zeros((maze_dim, maze_dim), dtype=np.int) 
        self.path_grid = np.zeros((maze_dim, maze_dim), dtype=np.int) # Grid for path
        self.policy_grid = [[' ' for row in range(maze_dim)] for col in range(maze_dim)] # Grid to map optimal route.
        
        self.path_value = [[1000 for row in range(maze_dim)] for col in range(maze_dim)] # Value score starting from goal and working way back.
        self.located_goal = [maze_dim/2 - 1, maze_dim/2] or [maze_dim/2, maze_dim/2] or [maze_dim/2, maze_dim/2 - 1] or [maze_dim/2 - 1, maze_dim/2 - 1]  # Goal area in maze center

        # Optimisation specific approach
        self.heuristic = [[min(abs(row-maze_dim/2+1), abs(row-maze_dim/2))+min(abs(col-maze_dim/2+1), abs(col-maze_dim/2)) for row in range(maze_dim)] for col in range(maze_dim)] # Heuristic Grid
        
        
    def next_move(self, sensors):
        
        if self.trial == 0 :
             rotation, movement = self.exploration_trial(sensors)          
        elif self.trial == 1:
            rotation, movement = self.optimisation_trial(sensors)
        
        return rotation, movement
    
    '''
    Method for exploration trial
    '''
    def exploration_trial(self,sensors):
        
        # Count number of steps taken
        print "Exploration Step Count: ", self.steps, sensors
        self.steps +=1

        # Print the robot's location         
        x1 = self.location[0]
        y1 = self.location[1]
        print 'Current Position: ', self.location
        # Add 1 to path_grid
        self.path_grid[x1][y1] = self.path_grid[x1][y1] + 1
        
        # Calculate the percentage of the maze the robot has visited
        uncovered_count = 0
        for x in range(self.maze_dimension):
            for y in range(self.maze_dimension):
                if self.path_grid[x][y] > 0:
                    uncovered_count = uncovered_count + 1

        uncovered = (uncovered_count/self.maze_area) * 100

        print "Robot has discovered %.2f%% of the maze.\n" % uncovered
        
        # Draw the map of the maze from the sensor readings     
        val = self.wall_locations(sensors, self.backwards) 
        self.maze_grid[x1][y1] = val
        
        # Determine the robot's next move
        rotation, movement = self.get_next_move(x1, y1, sensors)
 
        # Update the self.backwards value
        if movement == 0:
            self.backwards = 0
        elif movement == 1:
            self.backwards = 1
        elif movement == -1 or movement == -2: # Robot hit a dead end
            for move in range(2):
                if self.heading == 'l' or self.heading == 'left': # If robot is facing left, 9, 12, & 13 have a wall on right side
                    if self.maze_grid[x1+move][y1] == 9 or self.maze_grid[x1+move][y1] == 12 or self.maze_grid[x1+move][y1] == 13:
                        self.backwards = 0
                    else:
                        self.backwards = 1
                elif self.heading == 'r' or self.heading == 'right': # If robot is facing right, 3, 6, & 7 have a wall on left side
                    if self.maze_grid[x1-move][y1] == 3 or self.maze_grid[x1-move][y1] == 6 or self.maze_grid[x1-move][y1] == 7:
                        self.backwards = 0
                    else:
                        self.backwards = 1
                elif self.heading == 'u' or self.heading == 'up': # If robot is facing up, 3, 9, & 11 have a wall on bottom side
                    if self.maze_grid[x1][y1-move] == 3 or self.maze_grid[x1][y1-move] == 9 or self.maze_grid[x1][y1-move] == 11:
                        self.backwards = 0
                    else:
                        self.backwards = 1
                if self.heading == 'd' or self.heading == 'down': # If robot is facing down, 6, 12, & 14 have a wall on top side
                    if self.maze_grid[x1][y1+move] == 6 or self.maze_grid[x1][y1+move] == 12 or self.maze_grid[x1][y1+move] == 14:
                        self.backwards = 0
                    else:
                        self.backwards = 1

        # Update the robot's direction it is facing & new location
        self.update_direction(rotation, movement)
        
        # Get the new location
        x2 = self.location[0]
        y2 = self.location[1]
        
        # See if new location is in the goal.
        if x2 in self.located_goal and y2 in self.located_goal:
            if self.path_grid[x2][y2] == 0: # Don't repeat this statement
                print '---- Goal found ----\n+++ Continuing Exploration +++'
                self.centre_found = True
            
        elif self.centre_found == True and uncovered >= 75:
            print 'Exploration Completed. Starting Optimisation Trial.'
            goal = self.located_goal
            self.compute_value(goal) # Compute the value function and find optimal path
            print "\n ##### Maze Grid #####\n", self.maze_grid
            print "\n ##### Path Grid #####\n", self.path_grid
            print "\n ##### Path Value #####\n", self.path_value
            print "\n ##### Policy Grid #####\n", self.policy_grid
            
            # Reset to original settings before beginning optimization trial
            rotation = 'Reset'
            movement = 'Reset'
            self.trial = 1
            self.location = [0,0]
            self.heading = 'up'
            self.steps = 0
            self.centre_found = False
            
        return rotation, movement

    '''
    Optimisation Trial method definition
    '''
    def optimisation_trial(self,sensors):
        print "Optimization Trial Step #: ", self.steps, sensors, self.location
        self.steps +=1
           
        movement = 1
        
        # Get current location 
        x1 = self.location[0]
        y1 = self.location[1]
        
        # Rotate to the optimal path
        heading_angle = delta_degrees[self.heading]
        optimal_heading_angle = delta_degrees[self.policy_grid[x1][y1]]
        rotation = optimal_heading_angle - heading_angle
        
        # Correct for 270 degrees
        if rotation == -270:
            rotation = 90
        elif rotation == 270:
            rotation = -90
            
        # Change the angles to an index [0,1,2]    
        rotation_key = rotation/90 + 1  
        direction = dir_sensors[self.heading][rotation_key]  # Change direction
        
        # Move up to 3 consecutive steps
        while movement < 3: # Limit movement to 3 spaces
            location = self.policy_grid[x1][y1]
            x1 = x1 + dir_move[direction][0]
            y1 = y1 + dir_move[direction][1] 

            if self.policy_grid[x1][y1] == location:
                movement = movement + 1      
            else: 
                break
        
        # Update direction robot is facing & location
        self.update_direction(rotation, movement)
        
        # Retrieve new location
        x2 = self.location[0]
        y2 = self.location[1]
       
        return rotation, movement

    '''
    Updates the direction the robot is facing and its corresponding location in the maze
    '''
    def update_direction(self, rotation, movement):

        # Convert rotation angles to an index key [0,1,2]
        if rotation == -90 or rotation == 270:
            rotation = 0
        elif rotation == 0 or rotation == 360:
            rotation = 1
        elif rotation == 90 or rotation == -270:
            rotation = 2
        
        # Compute new direction based upon robot's rotation
        self.heading  = dir_sensors[self.heading][rotation]
        
        # Update Location 
        self.location[0] += dir_move[self.heading][0]*movement
        self.location[1] += dir_move[self.heading][1]*movement

    '''
    Generates a binary number that represents a description of walls surrounding robot            
    Also scales sensor readings as 1 for open and 0 for closed. 
    Sensor readings are [left, front, right]. If a wall is sensed, the value is set to 0.
    '''
    def wall_locations(self, sensors, backwards):
        
        for sensor in range(len(sensors)):
            if sensors[sensor] > 0:
                sensors[sensor] = 1
                
        # 1 = North, 2 = East, 4 = South, 8 = West. Each sensor will give a reading of 1 (open) or 0 (closed)       
        if self.heading == 'u' or self.heading == 'up':
            num = (sensors[0]*8) + (backwards*4) + (sensors[2]*2) + sensors[1]
        elif self.heading =='d' or self.heading == 'down':
            num = (sensors[2]*8) + (sensors[1]*4) + (sensors[0]*2) + backwards
        elif self.heading == 'l' or self.heading == 'left':
            num = (sensors[1]*8) + (sensors[0]*4) + (backwards*2) + sensors[2]
        elif self.heading == 'r' or self.heading == 'right':
            num = (backwards*8) + (sensors[2]*4) + (sensors[1]*2) + sensors[0]
        
        return num

    '''
    Determines the next move for robot
    Adapted few portions of the code from Udacity's Artificial Intelligence for Robotics course
    Reference: https://www.udacity.com/course/artificial-intelligence-for-robotics--cs373
    '''
    def get_next_move(self,x1,y1,sensors):
        
        # If robot runs into a dead-end, back up.
        if sensors == [0,0,0] and self.heading == 'u' and self.maze_grid[x1][y1 - 1] == 5:
            movement = -2
            rotation = 0
            print "Robot hit a deep dead end. Reversing."
        elif sensors == [0,0,0] and self.heading == 'r' and self.maze_grid[x1 - 1][y1] == 10:
            movement = -2
            rotation = 0
            print "Robot hit a deep dead end. Reversing."
        elif sensors == [0,0,0] and self.heading == 'd' and self.maze_grid[x1][y1 + 1] == 5:
            movement = -2
            rotation = 0
            print "Robot hit a deep dead end. Reversing."
        elif sensors == [0,0,0] and self.heading == 'l' and self.maze_grid[x1 + 1][y1] == 10:
            movement = -2
            rotation = 0
            print "Robot hit a deep dead end. Reversing."
        elif sensors == [0,0,0]:
            movement = -1
            rotation = 0
            print "Robot hit a dead end. Reversing."

        else: # Move to open space. Implement A*
            possible_moves = [] 
            rotate = [-90, 0, 90]
            for sensor in range(len(sensors)): # Look at all sensor readings
                if sensors[sensor] > 0: # If it has a number, that means it's open.
                    sensors[sensor] = 1  # Convert to 1 (open) or 2 (closed)
                    
                    # Move to next open space
                    x2 = x1 + dir_move[dir_sensors[self.heading][sensor]][0]   
                    y2 = y1 + dir_move[dir_sensors[self.heading][sensor]][1] 
                    
                    if x2>=0 and x2<self.maze_dimension and y2>=0 and y2<self.maze_dimension:    # Make sure robot next space is in maze
                        t2 = self.path_grid[x2][y2]  # Pull the number of times this cell has been visited.
                        h2 = self.heuristic[x2][y2]  # Small number means close to the goal area.
                        possible_moves.append([t2,h2,x2,y2,sensor]) # Create a list of the possible moves
                        
            possible_moves.sort()
            possible_moves.reverse()
            min_move = possible_moves.pop()  # Move to cell that is closest to the center and/or hasn't been explored.
            
            t,h,x,y,sensor = min_move # Break it up into separate variables
            rotation,movement = rotate[sensor], 1  # Rotate to opening and move 1 space.

        return rotation, movement                    
         
    '''
    Dynamic Programming Method using a Value Function
    Adapted few portions of code from Udacity's Artificial Intelligence for Robotics course
    Reference: https://www.udacity.com/course/artificial-intelligence-for-robotics--cs373
    '''
    def compute_value(self, goal):
        change = True   # Initialize boolean to start while-loop  
        
        while change:
            change = False  # Stop if nothing has changed
            
            for x in range(self.maze_dimension):
                for y in range(self.maze_dimension):                                       
                    if goal[0] == x and goal[1] == y:                        
                        if self.path_value[x][y] > 0: # Prevent endless loop
                            self.path_value[x][y] = 0 # Assign 0 to goal position
                            self.policy_grid[x][y] = '*' # Assign * to goal position
                            print "Goal location: {}\n".format(goal)
                            
                            change = True
                    else:                        
                        # Convert the wall value into a 4-bit number 
                        wall = self.maze_grid[x][y]
                        binary = int(bin(wall)[2:])
                        four_bit = '%0*d' % (4,binary)

                        # Start at goal and increase incrementally by 1 in open spaces
                        for direction in range(len(delta)): 
                            if (four_bit[0] == '1' and direction == 0) or (four_bit[1] == '1' and direction == 1) or (four_bit[2] == '1' and direction == 2) or (four_bit[3] == '1' and direction == 3):
                            # four_bit = 0000 = left,down,right,up Direction = left,down,right,up
                                x2 = x + delta[direction][0]
                                y2 = y + delta[direction][1]

                                if x2 >= 0 and x2 < self.maze_dimension and y2 >= 0 and y2 < self.maze_dimension:    # Make sure inside maze
                                    v2 = self.path_value[x2][y2] + 1   # Add 1 to path value
                                    
                                    if v2 < self.path_value[x][y]:
                                        change = True
                                        self.path_value[x][y] = v2 # Update path_value with new count number
                                        self.policy_grid[x][y] = delta_name[direction]  # Add movement symbol to policy_grid (<, v, >, or ^)
              