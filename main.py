# -------------------------------------------------------------------
# 	Project:      Robotics Task 1: Maze Escape
#	Author:       Liam Hammond C1022456
#	Created:      20/01/2025
# -------------------------------------------------------------------

# Imports
#####################################################################  
import math
import random
from vexcode_vr import *

# Constants
#####################################################################
VELOCITY = 100
MOVE_DISTANCE = 250 # mm
MARKER_DISTANCE = MOVE_DISTANCE / 2
GRID_SIZE = 8

# Cell markings
# 0 = blank
# 1 = mark
# 2 = 2 x marks
# 3 = wall

# Robot Configuration
#####################################################################
brain = Brain()
drivetrain = Drivetrain("drivetrain", 0)
drivetrain.set_rotation(0, DEGREES)
drivetrain.set_drive_velocity(VELOCITY, PERCENT)
drivetrain.set_turn_velocity(VELOCITY, PERCENT)

pen            = Pen("pen", 8)
left_bumper    = Bumper("leftBumper", 2)
right_bumper   = Bumper("rightBumper", 3)
front_eye      = EyeSensor("frontEye", 4)
down_eye       = EyeSensor("downEye", 5)
front_distance = Distance("frontdistance", 6)
magnet         = Electromagnet("magnet", 7)
location       = Location("location", 9)

# Start and Finish Detection
#####################################################################
def is_start():                                   
    global start_row
    global start_col
    if row == 7 and col == 4: 
        brain.print(f"At the start of the maze\n")
        start_row = row
        start_col = col
        return True
    else:
        return False

def is_finish():                                                       # Scan the floor for red exit 
    global finish_row
    global finish_col
    if down_eye.detect(RED):
        brain.print(f"\nAt the end of the maze: {row},{col}\n")
        finish_row = row
        finish_col = col
        return True  
    else:
        return False 

# Robot Movement
#####################################################################
# move = [0, 0, 0, 0]
# [up, right, down, left]
#   |0|
# |3| |1|
#   |2|

def turn_right(angle=90):
    drivetrain.turn_for(RIGHT, angle, DEGREES, wait=True)  

def turn_left(angle=90):
    drivetrain.turn_for(LEFT, angle, DEGREES, wait=True)

def drive_forward(distance=MOVE_DISTANCE):
    drivetrain.drive_for(FORWARD, distance, MM)

def move(direction, mark=0):
    print_direction(direction)
    move_robot(direction, mark)
    move_location(direction)

def centre_robot():                                                    # Calibrate robot to face north
    wait(30,MSEC)
    robot_facing = location.position_angle(DEGREES)
    turn_angle = robot_facing % 360
    if turn_angle > 180:
        turn_angle = 360 - turn_angle
        turn_right(turn_angle)  
    else:
        turn_left(turn_angle)  
    wait(30,MSEC)

def move_robot(move, mark): 
    if move[1] == 1:
        turn_right()
    if move[2] == 1:
        turn_right(180)
    if move[3] == 1:
        turn_left()
    wait(300,MSEC)

    if mark > 0:                                                       # Move half step and drop marker
        drive_forward(MARKER_DISTANCE)
        if mark == 1:
            pen.set_pen_width(MEDIUM)
            pen.set_pen_color(BLUE)
        if mark == 2:
            pen.set_pen_width(WIDE)
            pen.set_pen_color(RED)
        pen.move(DOWN)
        pen.move(UP)
        drive_forward(MARKER_DISTANCE)
    else:
        drive_forward()

# Wall Detection
#####################################################################
# directions = [0, 0, 0, 0]
# [north, east, south, west]
#   |n|
# |w| |e|
#   |s|

def is_wall():
    return 3 if front_distance.get_distance(MM) < 100 else 0

def scan_walls():
    directions = [0, 0, 0, 0]
    for i in range(len(directions)):  
        directions[i] = is_wall()  
        turn_right()  
    return directions

def combine_cell_and_scan(cell, scan):
    return [max(cell[i], scan[i]) for i in range(len(cell))]

# Maze and Grid Location
#####################################################################
# Grid location is 0 indexed
# 2D array also is inverse vertically to visual maze, so moving up is row - 1

maze = [[[0,0,0,0] for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]

start_row = 0
start_col = 0
finish_row = 0
finish_col = 0

row = 7                                                                # Robots current location
col = 4

last_move = None

def get_current_cell():
    global maze
    return maze[row][col]

def save_cell(scan):                                                   # Add markings and walls to robots current cell location
    global maze
    maze[row][col] = scan
    map_opposite_cells(scan)

def move_location(move):                                               # Update virtual grid location
    global row, col
    if move[0] == 1:
        row -= 1
    elif move[1] == 1:
        col += 1
    if move[2] == 1:
        row += 1
    elif move[3] == 1:
        col -= 1

def map_opposite_cells(scan):                                          # Apply same markings in inverse cell
    global maze                                                        # E.g. a wall on the right of current cell is mapped to
    if scan[0] != 0 and row > 0:                                       #      the left of the East cell
        maze[row - 1][col][2] = scan[0]
    if scan[1] != 0 and col < len(maze[0]) - 1: 
        maze[row][col + 1][3] = scan[1]
    if scan[2] != 0 and row < len(maze) - 1:    
        maze[row + 1][col][0] = scan[2]
    if scan[3] != 0 and col > 0:                
        maze[row][col - 1][1] = scan[3]

def create_opposite_last_move(move=None):                              # Creates opposite move array to prevent back tracking
    if move is None:  
        move = last_move
    if move is not None:
        opposite_move = [move[2], move[3], move[0], move[1]]
        return opposite_move
    else:
        return None

def check_where_to_move(scan):                                         # Check path to take using Trémaux's algorithm
    global last_move
    direction_to_move = [0, 0, 0, 0]
    open_cells = sum(1 for cell in scan if cell == 0)                  # Count open cells
    for i in range(len(scan)):                                         # Check each direction for a free path (0)
        direction_to_check = [0, 0, 0, 0]
        direction_to_check[i] = 1
        if scan[i] == 0:
            if direction_to_check != create_opposite_last_move():      # Check not going back on itself 
                direction_to_move[i] = 1
                break  
            elif open_cells == 1:                                      # Only go back if its the only option
                direction_to_move[i] = 1
                break
        elif scan[i] == 1 and open_cells == 0:                         # If no other option but to go down a marked path, take it
            direction_to_move[i] = 1
            break
    last_move = direction_to_move
    return direction_to_move

def update_dead_end(current_cell, direction):                         # Drop a 2 marker at a dead end (onlny 1 path option)
    if current_cell.count(0) + current_cell.count(1) == 1: 
        try:
            index_to_update = direction.index(1)
            if current_cell[index_to_update] != 3 and current_cell[index_to_update] != 2: 
                current_cell[index_to_update] = 2
        except ValueError:
            pass 
    return current_cell

# Returning back to start
#####################################################################
def return_home(cell, last_direction):
    direction_to_move = [0, 0, 0, 0]                                  # Initialise arr and create last move
    if last_direction is None:
        last_direction = [0, 0, 0, 0]
    return_direction = create_opposite_last_move(last_direction)

    for i in range(4):                                                # Follow dropped marker if robot is not going back on itself              
        if cell[i] == 1:
            direction_to_move[i] = 1
    for i in range(4):
        if direction_to_move[i] + return_direction[i] == 2:
            direction_to_move[i] = 0

    if sum(direction_to_move) == 0:                                   # If no valid move remains, check for cells with value 0
        for i in range(4):
            if cell[i] == 0 and return_direction[i] == 0:
                direction_to_move[i] = 1
                break

    return direction_to_move

# Console prints
#####################################################################
def print_cell(cell):
    brain.print(f"   {cell[0]}\n")
    brain.print(f"   ^\n")
    brain.print(f"{cell[3]}<   >{cell[1]}\n")
    brain.print(f"   v\n")
    brain.print(f"   {cell[2]}\n")

def print_maze():
    global maze
    brain.print(f"Saved Virual Maze:\n")
    for row in maze: brain.print(f"{row}\n")

def print_direction(move):
    directions = ['North ^', "East >", 'South v', 'West <']
    brain.print(f"Moving {directions[move.index(1)]}\n")

# Main run file
##################################################################### #####################################################################
def main():
    global maze, row, col
    at_start = True
    at_finish = False

    # Exploring the maze - Trémaux's algorithm
    #####################################################################
    mark = 0
    turns = 0
    last_direction = None

    # Loop searching cells until finish is found
    # -------------------------------------------------------------------
    while not at_finish:
        wait(300, MSEC)
        turns += 1 
        brain.print(f"\n{turns} ----- {row},{col} \n") 

        # Search cell and determine a direction
        # -------------------------------------------------------------------
        centre_robot()                                                        # Calibrate robot direction                  
        wait(300,MSEC)

        scan = scan_walls()                                                   # New wall scan
        current_cell = get_current_cell()                                     # Load any saved markers
        current_cell = combine_cell_and_scan(current_cell, scan)              # Apply saved markings to scan

        at_start = is_start()                                                 # Check if at start, place temp wall so cant exit map
        if at_start: current_cell[2] = 3                     
        direction = check_where_to_move(current_cell)                         # Apply Trémaux's algorithm to the path options
        if at_start: current_cell[2] = 0                                      # Remove temp wall

        # Checking if floor marker needs to be placed 
        # -------------------------------------------------------------------
        if current_cell.count(0) + current_cell.count(1) > 2:                 # If at a junction (more than 1 path), marker needs to drop
            brain.print(f"At a junction\n")
            current_cell[direction.index(1)] += 1                             # Increment marking by 1
            if not at_start:                                                  # Drop a marker in the last visited cell as is the end of a path
                pre_marker_location = create_opposite_last_move(last_direction)
                if current_cell[pre_marker_location.index(1)] == 0:           
                    current_cell[pre_marker_location.index(1)] += 1  

        if current_cell.count(0) + current_cell.count(1) == 1:                # If only 1 exit path, mark as dead end
            brain.print(f"At a dead end\n")
            current_cell = update_dead_end(current_cell, direction)

        mark = current_cell[direction.index(1)]                               # Apply floor marking needed to cell

        # Move robot and save state
        # -------------------------------------------------------------------
        save_cell(current_cell)
        move(direction, mark)
        print_cell(current_cell)

        at_finish = is_finish()                                               # Check if reached the finish
        last_direction = direction                                            # Save last move
        mark = 0                                                              # Reset mark state

    # Found the end of the maze
    # ------------------------------------------------------------------- 
    brain.print(f"Turns taken to find finish: {turns}\n")
    print_maze()                                                          # Show the virtuall grid mapped of maze

    # Finding the quickest return path
    #####################################################################
    pen.set_pen_color(GREEN)
    pen.set_pen_width(MEDIUM)
    pen.move(DOWN)
    last_direction = None
    turns = 0
    wait(1,SECONDS)

    # Move robot until back at start
    # -------------------------------------------------------------------
    brain.print(f"Drawing route back to the start\n")
    while not at_start:
        wait(300,MSEC)
        turns += 1 
        brain.print(f"\n{turns} ----- {row},{col} \n") 

        centre_robot()                                                        # Calibrate robot direction                  
        wait(300,MSEC)

        # Determine route to take
        # -------------------------------------------------------------------
        current_cell = get_current_cell()                                     # Load cell markers and wall scans
        at_finish = is_finish()                                               # Check if at finish, place temp wall so cant exit map
        if at_finish: current_cell[0] = 3

        direction = return_home(current_cell, last_direction)                 # Follow the dropped markers in the cell
        move(direction)
        last_direction = direction
        print_cell(current_cell)

        at_start = is_start()                                                 # Check if back at the start
        
    # Back at the start of the maze and path is drawn
    # ------------------------------------------------------------------- 
    brain.print(f"\nWe have found the fastest route in the maze, it takes {turns} moves to complete") 

# Run the file in VEXVR
#####################################################################
vr_thread(main)