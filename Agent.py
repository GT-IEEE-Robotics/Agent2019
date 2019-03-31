from threading import Thread
from queue import Queue
from enum import Enum

vision_in_queue = PriorityQueue()
#motor_control_in_queue = PriorityQueue()
#motor_control_out_queue = PriorityQueue()
waypoints_lock = Thread.Lock()
pathfinding_in_queue = PriorityQueue()
pathfinding_out_queue = PriorityQueue()
localization_in_queue = PriorityQueue()
localization_out_queue = PriorityQueue()

# class Command(Enum):
#     REQUEST_DATA = 1
#     PUT_DATA = 2

# in from: list of tuples representing objects, etc.
def vision(inQ):
    pass
# out to: motor speed, sensor data
# in from: our position
def localization(inQ, outQ):
    lastTimestamp = 0
    while(True):
        outQ.get
    pass
# out to: locations, objects we see, bases, obstacles (tuples)
# in from: list of waypoints
def pathfinding(inQ, outQ):
    pass
# out to: list of waypoints
# in from: motor speed
def motor_control(waypoints, new_waypoints, new_waypoints_lock):
    while(True):
        if (new_waypoints):

    pass

if __name__ == '__main__':

    # initialize and start processes
    vision_proc = Thread(target=vision, args=((vision_in_queue),))
    localization_proc = Thread(target=localization, args=((localization_in_queue),(localization_out_queue),))
    pathfinding_proc = Thread(target=pathfinding, args=((pathfinding_in_queue),(pathfinding_out_queue),))
    motor_control_proc = Thread(target=motor_control, args=((waypoints_lock), (new_waypoints),))

    vision_proc.daemon = True
    localization_proc.daemon = True
    pathfinding_proc.daemon = True
    motor_control_proc.daemon = True

    print("Starting threads... ")
    vision_proc.start()
    localization_proc.start()
    pathfinding_proc.start()
    motor_control_proc.start()
    print("Done\n")

    #motor control
    new_waypoints = False

