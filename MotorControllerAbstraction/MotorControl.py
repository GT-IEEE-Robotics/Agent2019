import numpy as np
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
    def getString(self):
        return "Point(x=" + str(self.x) + ", y=" + str(self.y) + ")"

class Spline:
    def __init__(self, n, begin):
        self.points = n
        self.calculate(self.points, False, begin, 0)

    def calculate(self, n, constant, begin, end):
        self.z = [0] * len(n)

        estEnd = float(float((n[len(n) - 1].getY() - n[len(n) - 2].getY()))/(n[len(n) - 1].getX() - n[len(n) - 2].getX()))

        d = [[0 for j in range(1)] for i in range(len(n))]
        a = [[0 for i in range(len(n))] for j in range(len(n))]

        for i in range(0, len(n)):
            if i == 0:
                h = float(n[i + 1].getX() - n[i].getX())
                d[i][0] = (6.0/h)*(n[i + 1].getY() - n[i].getY()) - 6.0*float(begin)
                a[i][i] = h*2.0
                a[i+1][i] = h
            elif i == (len(n) - 1):
                h = float(n[i].getX() - n[i - 1].getX())
                d[i][0] = 6.0*estEnd - (6.0/h)*(n[i].getY() - n[i - 1].getY())
                a[i][i] = 2.0*h
                a[i-1][i] = h
            else:
                h_upper = float(n[i + 1].getX() - n[i].getX())
                h_lower = float(n[i].getX() - n[i - 1].getX())

                d[i][0] = (6.0/h_upper)*(n[i + 1].getY() - n[i].getY()) - (6/h_lower)*(n[i].getY() - n[i - 1].getY())
                a[i][i] = 2*(h_upper+h_lower)
                a[i+1][i] = h_upper
                a[i-1][i] = h_lower


        try:
            z_inv = self.matmult(np.linalg.inv(a), d)
        except numpy.linalg.LinAlgError:
            print("Oh no")
            pass
        else:
            for i in range(0, len(z_inv)):
                self.z[i] = float(z_inv[i][0])


    def matmult(self, m1,m2):
        r=[]
        m=[]
        for i in range(len(m1)):
            for j in range(len(m2[0])):
                sums=0
                for k in range(len(m2)):
                    sums=sums+(m1[i][k]*m2[k][j])
                r.append(sums)
            m.append(r)
            r=[]
        return m

    def getY(self, x, n):
        x = float(x)
        h = float(self.points[n + 1].getX() - self.points[n].getX())
        t_upper = float(self.points[n + 1].getX() - x)
        t_lower = float(x - self.points[n].getX())
        value = float(self.getA(n)*t_upper**3 + self.getB(n)*t_lower**3 + self.getC(n)*t_lower + self.getD(n)*t_upper)
        return value

    def getA(self, n):
        h = float(self.points[n + 1].getX() - self.points[n].getX())
        # print("HI: %d"%self.z[n])
        return float(self.z[n]/(6.0*h))

    def getB(self, n):
        h = float(self.points[n + 1].getX() - self.points[n].getX())
        return float(self.z[n + 1]/(6*h))

    def getC(self, n):
        h = float(self.points[n + 1].getX() - self.points[n].getX())
        return float((self.points[n + 1].getY()/h) - ((self.z[n + 1]*h)/6))

    def getD(self, n):
        h = float(self.points[n + 1].getX() - self.points[n].getX())
        return float((self.points[n].getY()/h) - ((self.z[n]*h)/6))

    def getXYSet(self):
        fullSet = []
        count = 0
        for i in range(1, len(self.points)):
            diffx = float(self.points[i].getX() - self.points[i - 1].getX())
            step = float(diffx/20.0)
            j = float(self.points[i - 1].getX())
            while (j < self.points[i].getX()):
                y = self.getY(j,i - 1)
                #print("HEY: %d"%y)
                fullSet.append(Point(j, y))
                count = count + 1
                j = j + step

        return fullSet




class Trajectory:
    theta_arr = [0, 0]
    velocity = [0, 0]
    wheelbase_inv = [[0, 0], [0, 0]]
    wheelSpeeds = [[0, 0], [0, 0]]

    def __init__(self, old_x, old_y, init_theta_input, r_wheel, l_wheelbase, ratio_input):
        self.new_x = old_x
        self.new_y = old_y

        self.init_theta = init_theta_input
        self.theta_arr[1] = self.init_theta
        self.ratio = ratio_input
        self.r = r_wheel
        self.l = l_wheelbase

    def getNextPoint(self, new_x_input, new_y_input):
        self.curr_x = self.new_x
        self.curr_y = self.new_y
        self.new_x = new_x_input
        self.new_y = new_y_input

        self.dx = self.new_x - self.curr_x
        self.dy = self.new_y - self.curr_y
        if (self.dx == 0):
            if (self.dy == 0):
                self.theta = 0
            elif (self.dy > 0):
                self.theta = math.pi/2
            else:
                self.theta = -math.pi/2
        else:
            self.theta = math.atan(self.dy/self.dx)

        # print("theta: ", self.theta)

    def getTheta(self):
        return self.theta

    def setLinVel(self):
        v1 = 0
        v2 = 0
        if (self.theta == math.pi/2 or self.theta == -math.pi/2):
            v1 = 0
            v2 = self.dy / math.sin(self.theta)

        elif (self.theta == 0 or self.theta == math.pi):
            v1 = self.dx / math.cos(self.theta)
            v2 = 0
        else:
            v1 = self.dx / math.cos(self.theta)
            v2 = self.dy / math.sin(self.theta)

        self.lin_vel = math.sqrt(v1*v1 + v2*v2)

    def setAngVel(self):
        self.ang_vel = self.dtheta
        # print("ange_vel: ", self.ang_vel)

    def setThetaArr(self):
        self.theta_arr[0] = self.theta_arr[1]
        self.theta_arr[1] = self.theta

    def setDtheta(self):
        self.dtheta = self.theta_arr[1] - self.theta_arr[0]

    def makeArrays(self):
        self.velocity[0] = self.lin_vel
        self.velocity[1] = self.ang_vel
        self.wheelbase_inv[0][0] = -self.r / (2 * self.l)
        self.wheelbase_inv[0][1] = -self.r / 2
        self.wheelbase_inv[1][0] = -self.r / (2 * self.l)
        self.wheelbase_inv[1][1] = self.r / 2

    def setDet(self):
        self.det = -(self.r*self.r)/(2*self.l)
        # print("det: ", self.det)

    def getRatio(self):
        self.ratio = 2.5 / (abs(self.velocity[1]) + 0.05)
        # print("ratio: ", self.ratio)

    def getWheelSpeed(self):
        self.wheelSpeeds[0] = self.ratio * (1/self.det) * (self.wheelbase_inv[0][0] * self.velocity[0] + self.wheelbase_inv[0][1] * self.velocity[1])
        self.wheelSpeeds[1] = self.ratio * (1/self.det) * (self.wheelbase_inv[1][0] * self.velocity[0] + self.wheelbase_inv[1][1] * self.velocity[1])
        self.wheelSpeeds[0] = abs(self.wheelSpeeds[0])
        self.wheelSpeeds[1] = abs(self.wheelSpeeds[1])
        # print(self.wheelSpeeds)
        return self.wheelSpeeds





#coordinates: are the next set of waypoints from path planning, type: double

#subsection: number of points from spline wanted to sample for path, type double

#constantVelocity: max velocity that the robot should go over subsection, type double
#
#endRatio: determines when to slow down robot over path
#       Ex. To slow robot down for last tenth of subsection, endRatio = .1

#l_wheelbase: size of wheelbase of the robot, needs to be measured, type double

#OUTPUTS:

#speeds: an array of wheel speeds split into left and right wheels, type double
#times: an array of times, how long to each respective speed, type double



class MotorController:
    def __init__(self, subsection, constantVelocity, endRatio, l_wheelbase):
        self.subsection = subsection
        self.constantVelocity = constantVelocity
        self.endRatio = endRatio
        self.l_wheelbase = l_wheelbase

        self.speeds = None
        self.times = None

    def run(self, coordinates):
        theta_prev = 0.0
        lastRatio = len(coordinates)*20.0*self.endRatio
        trajectory = Trajectory(coordinates[0].getX(), coordinates[0].getY(), 0, 5.0, 10.0, 1)
        i = 0

        self.xValues = [coordinates[0].getX()]
        yValues = [coordinates[0].getY()]
        self.speeds = []

        self.times = [0]

        endPoint = coordinates[len(coordinates) - 1]
        distance = coordinates[0].getDistance(endPoint)
        endDistance = self.endRatio*distance

        deriv = 0
        end = False
        finish = False

        prev = coordinates[0]

        while (not finish) and ((i <= (len(coordinates) - self.subsection)) or end):
            if end:
                sub = coordinates[i:]
                finish = True
            else:
                sub = coordinates[i:(i + self.subsection)]

            spline = Spline(sub, deriv)
            subPoints = spline.getXYSet()

            for j in range(0, len(subPoints)):
                ratio = 1
                # tempDistance = subPoints[j].getDistance(endPoint)
                # if (tempDistance < endDistance):
                #     ratio = tempDistance / endDistance
                
                # print(endPoint.getString())
                # print(ratio)
                x = subPoints[j].getX()
                y = subPoints[j].getY()
                trajectory.getNextPoint(x, y)

                delta_d = subPoints[j].getDistance(prev)

                self.xValues.append(float(x))
                yValues.append(float(y))
                # trajectory.setLinVel()
                # trajectory.setThetaArr()
                # trajectory.setDtheta()
                # trajectory.setAngVel()
                # trajectory.makeArrays()
                # trajectory.setDet()
                # trajectory.getRatio()
                # wheelSpeeds = trajectory.getWheelSpeed()
                wheelSpeeds = [0, 0]
                theta_ratio = 1 - (abs(trajectory.getTheta()) - theta_prev) / math.pi
                if (theta_ratio < .4):
                    theta_ratio = .4
                if (theta_ratio > 1):
                    theta_ratio = 1
                # print(theta_ratio)
                wheelSpeeds[0] = theta_ratio*(2*self.constantVelocity - self.l_wheelbase*trajectory.getTheta())/2
                wheelSpeeds[1] = theta_ratio*(2*self.constantVelocity + self.l_wheelbase*trajectory.getTheta())/2

                avg_v = (wheelSpeeds[0] + wheelSpeeds[1]) / 2
                if (avg_v != 0):
                    self.times.append(delta_d / avg_v)
                else:
                    self.times.append(0)

                self.speeds.append([wheelSpeeds[0], wheelSpeeds[1]])

                prev = subPoints[j]
                theta_prev = trajectory.getTheta()

            i += self.subsection - 1

            lastX = self.xValues[len(self.xValues) - 1]
            lastY = yValues[len(yValues) - 1]
            secondLastX = self.xValues[len(self.xValues) - 5]
            secondLastY = yValues[len(yValues) - 5]
            deriv = (lastY - secondLastY)/(lastX - secondLastX)

            if (i > (len(coordinates) - self.subsection) and not end):
                end = True

    def getSpeeds(self):
        return self.speeds

    def getTimes(self):
        return self.times

    def getXVals(self):
        return self.xValues
    



# The situation:
# You're going to only call this here Velocity class
# Stuff you're gonna need to give it
#   your list of coordinates
#   how frequently you want spline to happen (the subsection of spline coordinates)
#   the velocity (central, assuming the robot is going straight and nowhere near its destinayion)
#   When you want the robot to start slowing down (like before it reaches the target... the last tenth? fifth? you decide.. )
#   wheel to wheel length of robot
# Stuff you're gonna get
#   an array of speeds, separated into left and right sides of the robot!
#   an array of times correlating to these speeds
#       i.e. go bla bla speed for bla bla seconds








# Tester class!!!
def main():
    x_data = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    y_data = [10.02, 10.04, 10.08, 10.16, 10.32, 10.64, 11.28, 12.56, 15.12, 20.24]
    test_points = []
    for i in range(len(x_data)):
        test_points.append(Point(x_data[i], y_data[i]))

    mc = MotorController(5, 5.0, 0.1, 3)
    mc.run(test_points)
    
    print("times")
    times = mc.getTimes()
    print(np.matrix(times))
    
    print("speed")
    speeds = mc.getSpeeds()
    xVals = mc.getXVals()
    for i in range(0, len(speeds)):
        print("%f, %f, %f"%(speeds[i][0], speeds[i][1], xVals[i]))

    # p = Point(0,0)
    # test = [
    #     Point(0.0, 5.0),
    #     Point(1.0, 4.0),
    #     Point(5.0, 6.0),
    #     Point(7.0, 10.5),
    #     Point(8.0, 11.0),
    #     Point(9.0, 8.9),
    #     Point(11.0, 14.6),
    #     Point(15.0, 10.8),
    #     Point(17.0, 17.0),
    #     Point(18.0, 12.1)
    # ]
    # vel = Velocity(test, 5, 10.0, .1, 3)
    print("here!")

if __name__ == "__main__":
    main()