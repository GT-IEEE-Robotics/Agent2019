import math

class Point:
    def __init__(self, x_temp, y_temp):
        self.x = float(x_temp)
        self.y = float(y_temp)
    def getX(self):
        return float(self.x)
    def getY(self):
        return float(self.y)
    def getDistance(self, two):
        return float(math.sqrt((two.getX() - self.x)**2 + ((two.getY() - self.y)**2)))
    def getTheta(self, two):
        return math.atan((two.getY() - self.y)/(two.getX() - self.x))