from threading import Thread, Lock
from queue import PriorityQueue
from enum import Enum
from StarveSafeReadWriteLock import StarveSafeReadWriteLock

# index to data map
names_map = ['waypoints', 'motor speed', 'state coordinate', 'state color', 'obstacles', 'targets']

# starve safe locks
waypoints_public_lock = StarveSafeReadWriteLock()
public_waypoints = None
waypoints_public_dirty = False

motor_speed_public_lock = StarveSafeReadWriteLock()
public_motor_speed = None
motor_speed_public_dirty = False

state_coordinate_public_lock = StarveSafeReadWriteLock()
public_state_coordinate = None
state_coordinate_public_dirty = False

state_color_public_lock = StarveSafeReadWriteLock()
public_state_color = None
state_color_public_dirty = False

obstacles_public_lock = StarveSafeReadWriteLock() #location, objects to avoid
public_obstacles = None
obstacles_public_dirty = False

targets_public_lock = StarveSafeReadWriteLock()
public_targets = None
targets_public_dirty = False

public_locks = [waypoints_public_lock, motor_speed_public_lock, state_coordinate_public_lock, state_color_public_lock, obstacles_public_lock, targets_public_lock]
public_data = [public_waypoints, public_motor_speed, public_state_coordinate, public_state_color, public_obstacles, public_targets]
public_dirty = [waypoints_public_dirty, motor_speed_public_dirty, state_coordinate_public_dirty, state_color_public_dirty, obstacles_public_dirty, targets_public_dirty]

# agent-to-system normal locks and their respective dirty bits
waypoints_private_lock = Lock()
private_waypoints = None
waypoints_private_dirty = False

motor_speed_private_lock = Lock()
private_motor_speed = 0
motor_speed_private_dirty = False

state_coordinate_private_lock = Lock()
private_state_coordinate = None
state_coordinate_private_dirty = False

state_color_private_lock = Lock()
private_state_color = None
state_color_private_dirty = False

obstacles_private_lock = Lock()
private_obstacles = None
obstacles_private_dirty = False

targets_private_lock = Lock()
private_targets = None
targets_private_dirty = False

private_locks = [waypoints_private_lock, motor_speed_private_lock, state_coordinate_private_lock, state_color_private_lock, obstacles_private_lock, targets_private_lock]
private_data = [private_waypoints, private_motor_speed, private_state_coordinate, private_state_color, private_obstacles, private_targets]
private_dirty = [waypoints_private_dirty, motor_speed_private_dirty, state_coordinate_private_dirty, state_color_private_dirty, obstacles_private_dirty, targets_private_dirty]

# in from: list of tuples representing objects, etc.
def vision(obstacles_lock, obstacles_dirty, obstacles, \
           targets_lock, targets_dirty, targets):
    pass
# out to: motor speed, sensor data
# in from: our position
def localization(motor_speed_lock, motor_speed_dirty, motor_speed, \
                 state_coordinate_lock, state_coordinate_dirty, state_coordinate, \
                 state_color_lock, state_color_dirty, state_color):
    pass
# out to: locations, objects we see, bases, obstacles (tuples)
# in from: list of waypoints
def pathfinding(obstacles_lock, obstacles_dirty, obstacles, \
                targets_lock, targets_dirty, targets, \
                waypoints_lock, waypoints_dirty, waypoints):
    pass
# out to: list of waypoints
# in from: motor speed
def motor_control(waypoints_lock, waypoints_dirty, waypoints, \
                  motor_speed_lock, motor_speed_dirty, motor_speed):
    # while(True):
    #     if (new_waypoints):
    #         pass
    pass

if __name__ == '__main__':

    # initialize and start processes
    vision_proc = Thread(target=vision, args=((obstacles_private_lock),(obstacles_private_dirty),(private_obstacles), \
                                              (targets_private_lock),(targets_private_dirty),(private_targets),))
    
    localization_proc = Thread(target=localization, args=((motor_speed_public_lock),(motor_speed_public_dirty),(public_motor_speed), \
                                                          (state_coordinate_private_lock),(state_coordinate_private_dirty),(private_state_coordinate), \
                                                          (state_color_private_lock),(state_color_private_dirty),(private_state_color),))
    
    pathfinding_proc = Thread(target=pathfinding, args=((obstacles_public_lock),(obstacles_public_dirty),(public_obstacles), \
                                                        (targets_public_lock),(targets_public_dirty),(public_targets), \
                                                        (waypoints_private_lock),(waypoints_private_dirty),(private_waypoints),))
    
    motor_control_proc = Thread(target=motor_control, args=((waypoints_public_lock),(waypoints_public_dirty),(public_waypoints), \
                                                            (motor_speed_private_lock),(motor_speed_private_dirty),(private_motor_speed),))

    vision_proc.daemon = True
    localization_proc.daemon = True
    pathfinding_proc.daemon = True
    motor_control_proc.daemon = True

    print("Starting threads and Initializing Agent 007... ")
    vision_proc.start()
    localization_proc.start()
    pathfinding_proc.start()
    motor_control_proc.start()

    # main loop
    # acquire, modify, post data

    # loop copies of locked data
    agent_waypoints = None
    agent_waypoints_dirty = False
    agent_motor_speed = 0
    agent_motor_speed_dirty = False
    agent_state_coordinates = None
    agent_state_coordinates_dirty = False
    agent_state_color = None
    agent_state_color_dirty = False
    agent_obstacles = None
    agent_obstacles_dirty = False
    agent_targets = None
    agent_targets_dirty = False

    agent_data = [agent_waypoints, agent_motor_speed, agent_state_coordinates, agent_state_color, agent_obstacles, agent_targets]
    agent_dirty = [agent_waypoints_dirty, agent_motor_speed_dirty, agent_state_coordinates_dirty, agent_state_color_dirty, agent_obstacles_dirty, \
        agent_targets_dirty]

    num_data_vectors = len(names_map)
    print("Done... Shaken not stirred\n")

    while(True):
        # grab all dirty data from local locks
        # TODO - we assume we can read the dirty bit without it being corrupt (it doesn't matter that much though)
        for i in range(num_data_vectors):
            if (private_dirty[i]):
                private_locks[i].acquire()
                agent_data[i] = private_data[i]
                agent_dirty[i] = True

                private_dirty[i] = False
                private_locks[i].release()

        # process data
        for i in range(num_data_vectors):
            if (agent_dirty[i]):
                agent_dirty[i] = False
                #do whatever - use switch statement to determine what to do
        
        # post data to public
        for i in range(num_data_vectors):
            if (agent_dirty[i]):
                public_locks[i].acquire_write()
                public_data[i] = agent_data[i]
                public_locks[i].release_write()