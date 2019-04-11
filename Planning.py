"""
Decision making part of path planning implemented with priority queue
"""
import queue
import sys
import re
from frenet import frenet

Round = 1
numOfObjects = 12

"""
Goal class containing location of goal and reward for completion
Comparison operators are implemented to allow
"""
class Goal:

    inGoal = False

    def __init__(self, color, location=(), priority = 1):
        self.color = color
        self.location = location
        self.priority = priority # Number representing the priority of goals

    def __gt__(self, other):
        if not isInstance(other, Goal):
            raise ValueError("Attempted to compare incorrect types")
        return self.priority > other.priority

    def __lt__(self, other):
        if not isInstance(other, Goal):
            raise ValueError("Attempted to compare incorrect types")
        return self.priority < other.priority

    # priority = how close to the next section * distance to the next obj
    def generateRound1Priority(self, state = None):
        if self.color  == state.nextSections[0]:
            self.priority = 4 * state.distToObj(self.location)
        elif self.color  == state.nextSections[1]:
            self.priority = 3 * state.distToObj(self.location)
        elif self.color  == state.nextSections[2]:
            self.priority = 2 * state.distToObj(self.location)
        else:
            self.priority = 1 * state.distToObj(self.location)

    def generateRound2Priority(self, state = None):
        if self.color  == state.nextSections[0]:
            self.priority = 4 * state.distToObj(self.location)
        elif self.color  == state.nextSections[1]:
            self.priority = 3 * state.distToObj(self.location)
        elif self.color  == state.nextSections[2]:
            self.priority = 2 * state.distToObj(self.location)
        else:
            self.priority = 1 * state.distToObj(self.location)

    def updateLoc(self):
        if (self.location == section):
            inGoal = True

"""
The state of the map with current color, loc coordinates, list of goal priorities
list of objects we can go for and immediate next goal
"""
class State:

    isHome = True # Robot always starts at home
    def __init__(self, location = (0,0), currSection = "red", listofSections = {"yellow", "blue", "green", "red"},
                    listOfObj = [], goal = (0,0)):
        self.currState = currSection
        self.location = location
        self.nextSections = listofSections
        self.objsInView = listOfObj
        self.next_goal = goal

    # @property
    # def next_goal(self, goal):
    #     self.next_goal = goal
    #
    # @property
    # def goal(self):
    #     return self.next_goal
    #
    # @property
    # def objects(self):
    #     return self.objsInView
    #
    # @property
    # def getSections(self):
    #     return self.nextSections
    #
    # @property
    # def setlocation(self, loc):
    #     self.location = loc

    """
    Helps to organize goals based on what comes up next
    """
    def checkSensors(self):
        if self.currState is "red":
            self.nextSections = {"yellow", "blue", "green", "red"}
        elif self.currState is "yellow":
            self.nextSections = {"blue", "green", "red", "yellow"}
        elif self.currState is "blue":
            self.nextSections = {"green", "red", "yellow", "blue"}
        elif self.currState is "green":
            self.nextSections = {"red", "yellow", "blue", "green"}

    def distToObj(self, block):
        return sqrt((block[0]-currState.location[0])**2 + (block[1]-currState.location[1])**2)


"""
Priority Queue class
- has update method and takes in tuples of data and priority
"""
class PriorityQueue(object):
    def __init__(self):
        self.queue = []

    def __str__(self):
        return ' '.join([str(i) for i in self.queue])

    def isEmpty(self):
        return len(self.queue) == []

    # for inserting an element in the queue
    def enqueue(self, data):
        self.queue.append(data)

    # picks a goal to go for and removes it from queue
    def dequeue(self):
        if (not isEmpty()):
            max = 0
            for i in range(len(self.queue)):
                if self.queue[i].priority > self.queue[max].priority:
                    max = i
            item = self.queue[max]
            del self.queue[max]
            return item
        else:
            print("No goals")
            return 0

    # updates an existing goals priority
    def update(self, data):
        for x in self.queue:
            if (x.location == data.location and x.location == data.location):
                x.priority = data.priority
                return True
        return False

queue = PriorityQueue()
currState = State()

# stateCoord - tuple of x,y location
# stateColor - string of color
#
def main(start, color, obj, avoid):
    #print("Hello World")
    grid = ()
    # args will have the following information
    StateCoord = start
    StateColor = color
    objectsToGoFor = obj
    objectsToAvoid = avoid
    currState = State(location = StateCoord, currSection = StateColor, listOfObj = objectsToGoFor)
    currState.checkSensors()
    # currently objects only have coordinates, no color
    for goal in objectsToGoFor:
        obj = Goal(color = goal[0], location = goal[1])
        if (Round == 1):
            obj.generateRound1Priority()
        else:
            obj.generateRound2Priority()
        if (not queue.update(obj)):
            queue.enqueue(obj)
    newGoal = queue.dequeue()
    if (newGoal == 0):
        # tell to explore
        # flip coordinates around centerpiece and set path planning on it
        newX = 0;
        newY = 0;
        if (4.5-currState.location[0] > 0):
            newX = 4.5 + abs(4.5-currState.location[0])
        else:
            newX = 4.5 - abs(4.5-currState.location[0])
        if (4.5-currState.location[1] > 0):
            newY = 4.5 + abs(4.5-currState.location[1])
        else:
            newY = 4.5 - abs(4.5-currState.location[1])
        #aim = ((newX, newY), objectsToAvoid)
        return frenet(((newX, newY), objectsToAvoid))
    else:
        # call frenet on this tuple
        #aim = (newGoal.location, objectsToAvoid)
        return frenet(newGoal.location, objectsToAvoid)

if __name__=="__main__":
    main()
