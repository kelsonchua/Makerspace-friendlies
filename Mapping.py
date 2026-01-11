import serial

ser = serial.Serial('/dev/serial0', 115200, timeout=1)

import RPi.GPIO as GPIO

# Button 
BUTTON_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

class Cell:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        # Wall status: True means a wall exists
        # self.walls = {'N': False, 'E': False, 'S': False, 'W': False}
        self.paths = {'N': False, 'E': False, 'S': False, 'W': False} # Is it connected to neighbour cells
        self.visited = False
        self.distance = float('inf')  # Default "infinity" for Flood Fill

# Create the 14x19 grid (1 based index)
MAZE_SIZE_X = 14
MAZE_SIZE_Y = 19
maze = [[Cell(x, y) for x in range(MAZE_SIZE_X+1)] for y in range(MAZE_SIZE_Y+1)]

# Block free track zone
for y in range(11, 20):
    for x in range(8, 15):
        maze[y][x].visited = True

def get_absolute_direction(direction):
    # move can be 'F' (Forward), 'R' (Right), 'L' (Left), 'B' (Back/U-turn)
    
    mapping = {
        'F': 0, # No change
        'R': 1, # +90 degrees
        'B': 2, # +180 degrees
        'L': 3  # +270 degrees (or -1)
    }
    
    # Calculate new numerical orientation
    new_orient = (orientation + mapping[direction]) % 4
    
    # Convert number back to letter
    directions = ['N', 'E', 'S', 'W']
    return directions[new_orient]

def get_flood_fill_path(start_x, start_y, target_x, target_y, maze_width, maze_height):
    # 1. Initialize distances with a high value
    distances = [[float('inf')] * maze_width for _ in range(maze_height)]
    distances[target_y][target_x] = 0
    
    queue = [(target_x, target_y)]
    
    # 2. Fill the grid with distances from the target
    while queue:
        cx, cy = queue.pop(0)
        current_dist = distances[cy][cx]
        
        # Check all 4 directions
        for direction, dx, dy in [('N', 0, 1), ('S', 0, -1), ('E', 1, 0), ('W', -1, 0)]:
            # Check if our path map says a path exists in this direction
            if maze[cy][cx].paths[direction]:
                nx, ny = cx + dx, cy + dy
                
                # If within bounds and not visited by flood fill yet
                if 0 < nx <= maze_width and 0 < ny <= maze_height:
                    if distances[ny][nx] == float('inf'):
                        distances[ny][nx] = current_dist + 1
                        queue.append((nx, ny))

    # 3. Trace the path from Start back to Target
    full_path_coords = []
    curr_x, curr_y = start_x, start_y
    
    while (curr_x, curr_y) != (target_x, target_y):
        for direction, dx, dy in [('N', 0, 1), ('S', 0, -1), ('E', 1, 0), ('W', -1, 0)]:
            # Can we move there?
            if maze[curr_y][curr_x].paths[direction]:
                nx, ny = curr_x + dx, curr_y + dy
                # Is it the shortest step?
                if distances[ny][nx] == distances[curr_y][curr_x] - 1:
                    full_path_coords.append(direction)
                    curr_x, curr_y = nx, ny
                    break
    path_steps = translate_dir_to_steps(full_path_coords)
    return path_steps # Returns list of moves like ['S', 'W', 'W']

def translate_dir_to_steps(path_steps):
    # path_steps looks like ['N', 'W', 'S']

    # Directions ordered N, E, S, W
    directions = ['N', 'E', 'S', 'W']

    execution_steps = []
    local_orientation = orientation
    for target_dir in path_steps:
        # Convert absolute target_dir to relative turn (L, R, F, B)
        # Then call explore(turn) and forward()
        if directions[local_orientation] == target_dir:
            execution_steps.append('F')
        elif directions[(local_orientation - 1) % 4] == target_dir:
            execution_steps.append('L')
            local_orientation = (local_orientation - 1) % 4
        elif directions[(local_orientation + 1) % 4] == target_dir:
            execution_steps.append('R')
            local_orientation = (local_orientation + 1) % 4
        elif directions[(local_orientation + 2) % 4] == target_dir:
            execution_steps.append('B')
            local_orientation = (local_orientation + 2) % 4

    return execution_steps

def forward(): # Update new postion after move forward
    global current_x, current_y, orientation
    if orientation == 0:   # North
        current_y += 1
    elif orientation == 1: # East
        current_x += 1
    elif orientation == 2: # South
        current_y -= 1
    elif orientation == 3: # West
        current_x -= 1

def update_map(L, F, R):
    global current_x, current_y, orientation
    cell = maze[current_y][current_x]

    cell = maze[current_y][current_x]
    cell.visited = True
    
    # The absolute direction the robot is facing
    front = get_absolute_direction('F')
    left = get_absolute_direction('L')
    right = get_absolute_direction('R')
    
    def mark_path(direction): # Update the path availability in current cell and neigbour cell
        cell.paths[direction] = True
        dx, dy = {'N':(0,1), 'S':(0,-1), 'E':(1,0), 'W':(-1,0)}[direction]
        nx, ny = current_x + dx, current_y + dy
        if 0 < nx <= MAZE_SIZE_X and 0 < ny <= MAZE_SIZE_Y:
            maze[ny][nx].paths[{'N':'S', 'S':'N', 'E':'W', 'W':'E'}[direction]] = True
    
    # Update the cell walls based on sensor input
    if F: mark_path(front)
    if L: mark_path(left)
    if R: mark_path(right)

def is_visited(relative_dir):
    abs_dir = get_absolute_direction(relative_dir)
    dx, dy = {'N':(0,1), 'E':(1,0), 'S':(0,-1), 'W':(-1,0)}[abs_dir]
    nx, ny = current_x + dx, current_y + dy
    
    # If it's out of bounds, treat as visited so we don't go there
    if not (0 < nx <= MAZE_SIZE_X and 0 < ny <= MAZE_SIZE_Y):
        return True
    return maze[ny][nx].visited
    
def explore(path): # e.g: F
    global orientation
    send(path) # Tell ESP to explore the direction

    # Update orientation
    if path == 'L':
        orientation = (orientation - 1) % 4
    elif path == 'R':
        orientation = (orientation + 1) % 4
    elif path == 'B':
        orientation = (orientation + 2) % 4

def send(message):
    ser.write((message + "\n").encode('utf-8')) # translate the message into byte-stream

def receive():
    try:
        while True:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').strip()
                return data
    except KeyboardInterrupt:
        ser.close()

def start_explore(): # Send message for ESP to initiate exploration
    send("START")    

def end_explore():
    send("END")

# Set initial location
start_x = 1
start_y = 19
current_x = start_x
current_y = start_y
orientation = 2 # 0=North, 1=East, 2=South, 3=West
target_x = None
target_y = None
target_orientation = None
entrance_x = None
entrance_y = None
path_stack = [] # (direction, x, y)

update_map(0, 1, 0) # update starting location
start_explore() 
while True:
    resp = receive()
    if resp == "UPDATE" or resp.startswith("JUNCTION"):
        forward()

        if resp == "UPDATE":
            L, F, R = 0, 1, 0
        else:
            # Remove the 'JUNCTION:' part
            payload = resp.replace("JUNCTION:", "")
            # Split '1,0,1' into ['1', '0', '1']
            paths = payload.split(",")
            L = int(paths[0])
            F = int(paths[1])
            R = int(paths[2])
        
        update_map(L, F, R)

        choices = []
        # Exploration priority
        if L and not is_visited('L'):
            choices.append('L')
        if F and not is_visited('F'):
            choices.append('F')
        if R and not is_visited('R'):
            choices.append('R')
        
        if choices != []:
            if len(choices) > 1:
                for path in choices[1:]:
                    path_stack.append((get_absolute_direction(path), current_x, current_y))
            explore(choices[0])

        else: # dead end
            if path_stack:
                path, x, y = path_stack.pop()

                orien, dx, dy = {'N':(0,0,1), 'E':(1,1,0), 'S':(2,0,-1), 'W':(3,-1,0)}[path]
                target_orientation, target_x, target_y = orien, x + dx, y + dy

                back_path = get_flood_fill_path(current_x, current_y, target_x, target_y, MAZE_SIZE_X, MAZE_SIZE_Y) # Get command from current loc to target loc
                send("".join(back_path))
            else:
                # end of exploration
                break

    elif resp == "ARRIVE":
        current_x, current_y, orientation = target_x, target_y, target_orientation
        update_map(0, 1, 0)

    elif resp == "FOUND":
        forward()
        update_map(0, 1, 0)
        entrance_x, entrance_y = current_x, current_y

end_explore()
shortest_path = get_flood_fill_path(start_x, start_y, entrance_x, entrance_y, MAZE_SIZE_X, MAZE_SIZE_Y)

print("Waiting for Button Press to start Speed Run...")
# This line stops the code until the pin is grounded (button pressed)
GPIO.wait_for_edge(BUTTON_PIN, GPIO.FALLING)

print("Button pressed! Starting Speed Run...")
# Calculate and send path...
command_string = "".join(shortest_path)
send(f"FINAL:{command_string}")

while True:
    resp = receive()
    if resp == "FINISH":
        print("Speed run done.")
        break