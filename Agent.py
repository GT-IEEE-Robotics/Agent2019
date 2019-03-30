from multiprocessing import Process, Queue
from enum import Enum
vision_in_queue = Queue()
motor_control_out_queue = Queue()
pathfinding_in_queue = Queue()
pathfinding_out_queue = Queue()
localization_in_queue = Queue()

# class Command(Enum):
#     REQUEST_DATA = 1
#     PUT_DATA = 2

def vision_reader(inQ):
    pass
def localization_reader(inQ):
    pass
def pathfinding_reader(inQ):
    pass
def pathfinding_writer(outQ):
    pass
def motor_control_writer(outQ):
    pass

if __name__ == '__main__':

    # initialize and start processes
    vision_reader_proc = Process(target=vision_reader, args=((vision_in_queue),))
    localization_reader_proc = Process(target=localization_reader, args=((localization_in_queue),))
    pathfinding_reader_proc = Process(target=pathfinding_reader, args=((pathfinding_in_queue),))
    pathfinding_writer_proc = Process(target=pathfinding_writer, args=((pathfinding_out_queue),))
    motor_control_writer_proc = Process(target=motor_control_writer, args=((motor_control_out_queue),))

    vision_reader_proc.daemon = True
    localization_reader_proc.daemon = True
    pathfinding_reader_proc.daemon = True
    pathfinding_writer_proc.daemon = True
    motor_control_writer_proc.daemon = True

    print("Starting threads... ")
    vision_reader_proc.start()
    localization_reader_proc.start()
    pathfinding_reader_proc.start()
    pathfinding_writer_proc.start()
    motor_control_writer_proc.start()
    print("Done\n")

