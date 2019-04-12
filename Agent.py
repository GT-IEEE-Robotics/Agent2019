# Georgia Tech IEEE Robotics Club
# SoutheastCon 2019

# Agent.py
# Integrates software subsystems of the robot by making fresh, formatted data cleanly available
# as often as possible

# TODO - Handle initial conditions
# TODO - include team updates as they're made
# TODO - switch to notifying threads instead of waiting in a loop

from threading import Thread, Lock, Event
import time

from StarveSafeReadWriteLock import StarveSafeReadWriteLock
from MotorControllerAbstraction.MotorControl import MotorController
import PathPlanning.Planning
import OrEvent

# index to data map
# state coordinate - our location
names_map = ['waypoints', 'motor speed', 'state coordinate', 'state color', 'obstacles', 'targets']

# starve safe locks
waypoints_public_lock = StarveSafeReadWriteLock()
public_waypoints = None
waypoints_public_dirty = Event()

motor_speed_public_lock = StarveSafeReadWriteLock()
public_motor_speed = None
motor_speed_public_dirty = Event()

state_coordinate_public_lock = StarveSafeReadWriteLock()
public_state_coordinate = None
state_coordinate_public_dirty = Event()

state_color_public_lock = StarveSafeReadWriteLock()
public_state_color = None
state_color_public_dirty = Event()

obstacles_public_lock = StarveSafeReadWriteLock() #location, objects to avoid
public_obstacles = None
obstacles_public_dirty = Event()

targets_public_lock = StarveSafeReadWriteLock()
public_targets = None
targets_public_dirty = Event()

public_locks = [waypoints_public_lock, motor_speed_public_lock, state_coordinate_public_lock, state_color_public_lock, obstacles_public_lock, targets_public_lock]
public_data = [public_waypoints, public_motor_speed, public_state_coordinate, public_state_color, public_obstacles, public_targets]
# TODO - make sure there aren't any collisions with this dirty bit - could result in a system not getting new data if another one reads the same data before it
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
    my_motor_speed = None

    my_state_coordinate = None
    my_state_color = None

    while True:
        #my_state_coordinate = polar_ransac.ransac()
        
        # get state color somehow
        state_coordinate_lock.acquire()

        state_coordinate = my_state_coordinate
        state_coordinate_dirty = True
        state_coordinate_lock.release()

        state_color_lock.acquire()
        state_color = my_state_color
        state_color_dirty = True
        state_color_lock.release()

        #time.sleep(0.0005)
# out to: locations, objects we see, bases, obstacles (tuples)
# in from: list of waypoints
def path_planning(obstacles_lock, obstacles_dirty, obstacles, \
                targets_lock, targets_dirty, targets, \
                state_coordinate_lock, state_coordinate_dirty, state_coordinate, \
                state_color_lock, state_color_dirty, state_color, \
                waypoints_lock, waypoints_dirty, waypoints):

    my_obstacles = None
    my_targets = None
    my_state_coordinate = None
    my_state_color = None

    my_waypoints = None

    new_data = OrEvent.OrEvent(obstacles_dirty, targets_dirty, state_coordinate_dirty, state_color_dirty)

    while True:

        new_data.wait()

        # collect any new data
        if (obstacles_dirty.is_set()):
            obstacles_lock.acquire()
            my_obstacles = obstacles
            obstacles_dirty.clear()
            obstacles_lock.release()
        if (targets_dirty.is_set()):
            targets_lock.acquire()
            my_targets = targets
            targets_dirty.clear()
            targets_lock.release()

        if (state_coordinate_dirty.is_set()):
            state_coordinate_lock.acquire()
            my_state_coordinate = state_coordinate
            state_coordinate_dirty.clear()
            state_coordinate_lock.release()

        if (state_color_dirty.is_set()):
            state_color_lock.acquire()
            my_state_color = state_color
            state_color_dirty.clear()
            state_color_lock.release()

        # operate on new data if it exists and make it available to the Agent
        my_waypoints = Planning.main(my_state_coordinate, my_state_color, my_obstacles, my_targets)
        
        waypoints_lock.acquire()
        waypoints = my_waypoints
        waypoints_dirty = True
        waypoints_lock.release()

# out to: list of waypoints
# in from: motor speed
def motor_control(waypoints_lock, waypoints_dirty, waypoints, \
                  motor_speed_lock, motor_speed_dirty, motor_speed):
 
    mc = MotorController(5, 10.0, 0.1, 3)
    my_waypoints = None
    
    while True:
        waypoints_dirty.wait()
        # acquire and save dirty waypoints, then update the dirty bit
        waypoints_lock.acquire()
        my_waypoints = waypoints # TODO - This is really important!!!!! Convert waypoints into a list of Point objects - do this in agent loop?
        waypoints_dirty.clear()
        waypoints_lock.release()

        # do work
        mc.run(my_waypoints)
        
        # acqure and write new speed
        motor_speed_lock.acquire()
        motor_speed = mc.getSpeeds() # left and right motor speed TODO - also times? how do we incorporate that?
        motor_speed_dirty = True
        motor_speed_lock.release()

if __name__ == '__main__':

    print("Starting threads and Initializing Agent 007...")

    # initialize and start processes
    vision_proc = Thread(target=vision, args=((obstacles_private_lock),(obstacles_private_dirty),(private_obstacles), \
                                              (targets_private_lock),(targets_private_dirty),(private_targets),))
    
    localization_proc = Thread(target=localization, args=((motor_speed_public_lock),(motor_speed_public_dirty),(public_motor_speed), \
                                                          (state_coordinate_private_lock),(state_coordinate_private_dirty),(private_state_coordinate), \
                                                          (state_color_private_lock),(state_color_private_dirty),(private_state_color),))
    
    path_planning_proc = Thread(target=path_planning, args=((obstacles_public_lock),(obstacles_public_dirty),(public_obstacles), \
                                                        (targets_public_lock),(targets_public_dirty),(public_targets), \
                                                        (state_coordinate_public_lock),(state_coordinate_public_dirty),(public_state_coordinate), \
                                                        (state_color_public_lock),(state_color_public_dirty),(public_state_color), \
                                                        (waypoints_private_lock),(waypoints_private_dirty),(private_waypoints),))
    
    motor_control_proc = Thread(target=motor_control, args=((waypoints_public_lock),(waypoints_public_dirty),(public_waypoints), \
                                                            (motor_speed_private_lock),(motor_speed_private_dirty),(private_motor_speed),))

    vision_proc.setDaemon(True)
    localization_proc.setDaemon(True)
    path_planning_proc.setDaemon(True)
    motor_control_proc.setDaemon(True)

    vision_proc.start()
    localization_proc.start()
    path_planning_proc.start()
    motor_control_proc.start()

    # main loop
    # acquire, modify, post data

    # loop copies of locked data
    agent_waypoints = None
    agent_waypoints_dirty = False
    agent_motor_speed = 0
    agent_motor_speed_dirty = False
    agent_state_coordinate = None
    agent_state_coordinate_dirty = False
    agent_state_color = None
    agent_state_color_dirty = False
    agent_obstacles = None
    agent_obstacles_dirty = False
    agent_targets = None
    agent_targets_dirty = False

    agent_data = [agent_waypoints, agent_motor_speed, agent_state_coordinate, agent_state_color, agent_obstacles, agent_targets]
    agent_dirty = [agent_waypoints_dirty, agent_motor_speed_dirty, agent_state_coordinate_dirty, agent_state_color_dirty, agent_obstacles_dirty, \
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
                pass
                #do whatever - use switch statement to determine what to do
        
        # post data to public
        for i in range(num_data_vectors):
            if (agent_dirty[i]):
                public_locks[i].acquire_write()
                public_data[i] = agent_data[i]
                public_dirty[i].set()
                agent_dirty[i] = False
                public_locks[i].release_write()