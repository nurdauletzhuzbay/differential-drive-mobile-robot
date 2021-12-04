import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle
from scipy.interpolate import CubicSpline
import math
from matplotlib.widgets import Slider, Button

x_obstacles = [20, 30, 70, 90]
y_obstacles = [90, 40, 35, 10]
radius = 6
sensor_radius = 4

x_start = 8
y_start = 8
theta_start = np.pi/4

dt = 0.1
animation = True

length = 8
width = 4
bcw = 4
lenWheel = 1
widWheel = 1
tread = 3

#class to plot circles
class Config():

    def __init__(self):
        self.circle = 0
        # 0 is circle, 1 is rectangle

def move(x_goal, y_goal, theta_goal):

    config = Config() #set circle obstacles
    x = x_start       #starting position x
    y = y_start       #starting position y
    theta = theta_start  #starting angle

    x_diff = x_goal - x    #distance that is left to the goal
    y_diff = y_goal - y
    vl = 5  #velocity of the left wheel
    vr = 5  #velocity of the right wheel

    dis = np.hypot(x_diff, y_diff)  #calculate the distance using hypotenuse of
                                    #triangle
    l = 4.8 + (2*widWheel)

    while dis > 0.5: #continue until we reach goal

        if vr == vl:
            vr = vl + 0.0000001   # to avoid division by zero

        w = (vr - vl)/l        #angular velocity
        R = (l/2)*(vr+vl)/(vr-vl)   #radius of curvature

        #forward kinematics of mobile robot
        #instantaneous center of curvature
        ICCx = x - R*np.sin(theta)
        ICCy = y + R*np.cos(theta)

        [x_robt, y_robt] = [np.cos(w*dt)*(x-ICCx) - np.sin(w*dt)*(y-ICCy) + ICCx, np.sin(w*dt)*(x-ICCx) + np.cos(w*dt)*(y-ICCy) + ICCy]
        theta_robt = theta + w*dt

        x = x_robt
        y = y_robt
        theta = theta_robt
        x_diff = x_goal - x_robt
        y_diff = y_goal - y_robt
        dis = np.hypot(x_diff, y_diff)

        leftSensor1 = [x + width/2, y + length/2]
        rightSensor1 = [x + width/2, y - length/2]

        leftSensor2 = [x - 2.5/np.sqrt(2), y + 2.5/np.sqrt(2)]
        rightSensor2 = [x + 2.5/np.sqrt(2), y - 2.5/np.sqrt(2)]

        # lightSensor_left = [x + 3.16, y + 3.16]
        # lightSensor_right = [x + 2, y - 2]
        #
        # dis_x_left = x_goal - lightSensor_left[0]
        # dis_y_left = y_goal - lightSensor_left[1]
        # dis_x_right = x_goal - lightSensor_right[0]
        # dis_y_right = x_goal - lightSensor_right[1]
        #
        # dis_goal_left = np.hypot(dis_x_left, dis_y_left)
        # dis_goal_right = np.hypot(dis_x_right, dis_y_right)
        #
        # if dis_goal_left > dis_goal_right:
        #     vl = vl - 0.05
        # elif dis_goal_left < dis_goal_right:
        #     vr = vr - 0.05
        # else:
        #     vr = vl + 0.000001

        for i in range(len(x_obstacles)):
            leftX = x_obstacles[i] - leftSensor1[0]
            leftY = y_obstacles[i] - leftSensor1[1]
            rightX = x_obstacles[i] - rightSensor1[0]
            rightY = y_obstacles[i] - rightSensor1[1]

            leftX2 = x_obstacles[i] - leftSensor2[0]
            leftY2 = y_obstacles[i] - leftSensor2[1]
            rightX2 = x_obstacles[i] - rightSensor2[0]
            rightY2 = y_obstacles[i] - rightSensor2[1]

            LEFT = np.hypot(leftX, leftY)
            RIGHT = np.hypot(rightX, rightY)
            LEFT2 = np.hypot(leftX2, leftY2)
            RIGHT2 = np.hypot(rightX2, rightY2)

            if LEFT <= radius:
                vr = -vr
            if RIGHT <= radius:
                vl = -vl
            if LEFT2 <= radius:
                vl = vl + 1
            if RIGHT2 <= radius:
                vr = vr + 1




        #border collision check
        if leftSensor1[0] >= 100 or leftSensor1[1] >= 100 or rightSensor1[0] <= 0 or rightSensor1[1] <= 0:
            vr = -vr
            vl = -vl


        if animation:
            plt.cla()
            obstacles()
            plt.plot([x, x_goal],[y, y_goal], 'go')
            robot(x, y, theta)



def obstacles():
    #obstacles
    o1 = circ(x_obstacles[0], y_obstacles[0], radius)
    o2 = circ(x_obstacles[1], y_obstacles[1], radius)
    o3 = circ(x_obstacles[2], y_obstacles[2], radius)
    o4 = circ(x_obstacles[3], y_obstacles[3], radius)

    plt.gca().add_patch(o1)
    plt.gca().add_patch(o2)
    plt.gca().add_patch(o3)
    plt.gca().add_patch(o4)

def robot(x, y, theta):
    chassis = np.matrix([[-bcw, (length - bcw), (length - bcw), -bcw, -bcw],
                         [width / 2, width / 2, - width / 2, -width / 2, width / 2]])

    r_wheel = np.matrix([[-lenWheel, lenWheel, lenWheel, -lenWheel, -lenWheel],
                        [-widWheel - tread, -widWheel - tread, widWheel - tread, widWheel - tread, -widWheel - tread]])
    l_wheel = np.copy(r_wheel)
    l_wheel[1, :] *= -1

    r_wheel = (r_wheel.T * rotation(theta)).T
    l_wheel = (l_wheel.T * rotation(theta)).T
    chassis = (chassis.T * rotation(theta)).T

    chassis[0, :] += x
    chassis[1, :] += y
    r_wheel[0, :] += x
    r_wheel[1, :] += y
    l_wheel[0, :] += x
    l_wheel[1, :] += y

    plt.plot(np.array(chassis[0, :]).flatten(), np.array(chassis[1, :]).flatten(), "-k")
    plt.plot(np.array(r_wheel[0, :]).flatten(), np.array(r_wheel[1, :]).flatten(), "-k")
    plt.plot(np.array(l_wheel[0, :]).flatten(), np.array(l_wheel[1, :]).flatten(), "-k")
    plt.xlim(0,100)
    plt.ylim(0,100)
    plt.plot(x, y, "*")
    plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
    plt.pause(dt)

def rotation(theta):
    rot = np.matrix([[math.cos(theta), math.sin(theta)],
                    [-math.sin(theta), math.cos(theta)]])
    return rot

def circ(x, y, rad):
    return Circle((x, y), rad)

def main():
    x_goal = 90
    y_goal = 90
    theta_goal = np.pi/4

    move(x_goal, y_goal, theta_goal)

if __name__ == '__main__':
    main()
