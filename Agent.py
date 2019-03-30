from multiprocessing import Process, Queue
from enum import Enum

vision_in_queue = Queue()
motor_control_in_queue = Queue()
motor_control_out_queue = Queue()
pathfinding_in_queue = Queue()
pathfinding_out_queue = Queue()
localization_in_queue = Queue()
localization_out_queue = Queue()

# class Command(Enum):
#     REQUEST_DATA = 1
#     PUT_DATA = 2

def vision(inQ):
    pass
def localization(inQ, outQ):
    pass
def pathfinding(inQ, outQ):
    pass
def motor_control(inQ, outQ):
    pass

if __name__ == '__main__':

    # initialize and start processes
    vision_proc = Process(target=vision, args=((vision_in_queue),))
    localization_proc = Process(target=localization, args=((localization_in_queue),(localization_out_queue),))
    pathfinding_proc = Process(target=pathfinding, args=((pathfinding_in_queue),(pathfinding_out_queue),))
    motor_control_proc = Process(target=motor_control, args=((motor_control_in_queue),(motor_control_out_queue),))

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

