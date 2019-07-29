#!/usr/bin/env python

import math
import sys
import rospy
import tf.transformations as transform

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from goal_publisher.msg import PointArray
from location import Location, necessary_heading
from dist import Dist
from gazebo_msgs.msg import ModelStates

current_location = Location()
current_dists = Dist()
delta = .2
WALL_PADDING = .3

STRAIGHT = 0
LEFT = 1
RIGHT = 2
MSG_STOP = 3

global x_pos
global y_pos
global reward_list

x_pos = []
y_pos = []
x_goal_pos = []
y_goal_pos = []
goal = []
dist = []
reward_list = []
reward_sum=[]
f_dir = 0
sum_reward = 0

def retinitialize():                #Initialise the list variables and the corresponding X and Y coordinates indexes.
    global goal_pos_curr,d,curr_x,curr_y,x_pos_iterator,goal_pos_iterator,x_pos_curee
    x_pos_curee = 0
    goal_pos_curr = 0
    d = 0
    curr_x = 0
    curr_y = 0
    x_pos_iterator = 0
    goal_pos_iterator = 0
    del x_goal_pos[:]
    del y_goal_pos[:]
    del dist[:]
    del goal[:]

def init():                                                     #Launch the final project and subscribe to various topics
    rospy.init_node('final_project')
    rospy.Subscriber("/gazebo/model_states", ModelStates, location_callback)
    rospy.Subscriber('/scan', LaserScan, sensor_callback)
    rospy.Subscriber("/goals",PointArray, goal_callback)

def goal_callback(data):                                        #Add dynamics goals to the list using callback.
    global goal
    global x_pos
    global y_pos
    global reward_list
    x_pos=[]
    y_pos=[]
    reward_list =[]
    count=0
    while count<=(len(data.goals)-1):
        x_pos.append(data.goals[count].x)
        y_pos.append(data.goals[count].y)
        reward_list.append(data.goals[count].reward)
        count=count+1



def location_callback(data):                                    #Obtain the actual angle of the turlt bot with respect to ground
    p = data.pose[1].position
    q = (
            data.pose[1].orientation.x,
            data.pose[1].orientation.y,
            data.pose[1].orientation.z,
            data.pose[1].orientation.w)
    t = transform.euler_from_quaternion(q)[2] # in [-pi, pi]
    current_location.update_location(p.x, p.y, t)

def sensor_callback(data):
    current_dists.update(data)

class Bug:
    def __init__(self, tx, ty):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.tx = tx
        self.ty = ty

    def go(self, direction):                                        #Move the turtle bot towards the goal
        cmd = Twist()
        if direction == STRAIGHT:
            cmd.linear.x = 0.1
        elif direction == LEFT:
            cmd.angular.z = 0.25
        elif direction == RIGHT:
            cmd.angular.z = -0.25
        elif direction == MSG_STOP:
            cmd.linear.x = 0
            cmd.angular.z = 0
        self.pub.publish(cmd)

    def go_until_obstacle(self):                                    #Move towards the obstacle followng the MLine
        print "Going until destination or obstacle"
        while current_location.distance(tx, ty) > delta:
            (frontdist, _, _) = current_dists.get()
            if current_location.facing_point(tx, ty):
                self.go(STRAIGHT)
            elif current_location.faster_left(tx, ty):
                self.go(LEFT)
            else:
                self.go(RIGHT)
            if frontdist <= WALL_PADDING:
                return True
            rospy.sleep(.01)
        return False

    def follow_wall_right(self):                                 #Follow the right wall when obstacle is encountered while following the MLine
        print "Following Right wall"
        while not self.should_leave_wall():
            (front, left, right) = current_dists.get()
            if front <= WALL_PADDING:
                self.go(LEFT)
            elif WALL_PADDING - .1 <= right <= WALL_PADDING + .05:
                self.go(STRAIGHT)
            elif right > WALL_PADDING + .05:
                self.go(RIGHT)
            elif left < 1 and right < 1:
                self.go(STRAIGHT)
            else:
                self.go(LEFT)
            rospy.sleep(.01)

    def follow_wall_left(self):                                #Follow the left wall when obstacle is encountered while following the MLine
        print "Following Left wall"
        while current_dists.get()[0] <= WALL_PADDING:
            self.go(RIGHT)
            rospy.sleep(.01)
        while not self.should_leave_wall():
            (front, left, right) = current_dists.get()
            if front <= WALL_PADDING:
                self.go(RIGHT)
            elif WALL_PADDING - .17 <= left <= WALL_PADDING + .1:
                self.go(STRAIGHT)
            elif left > WALL_PADDING + .1:
                self.go(LEFT)
            elif left < 1 and right < 1:
                self.go(STRAIGHT)
            else:
                self.go(RIGHT)
            rospy.sleep(.01)

class Bug2(Bug):
    def __init__(self, tx, ty):
        Bug.__init__(self, tx, ty)
        self.lh = None
        self.encountered_wall_at = (None, None)

    def face_goal(self):                                        #Check if the turtle bot is facing the goal point
        while not current_location.facing_point(self.tx, self.ty):  #used to turn robot in goal direction
            self.go(RIGHT)
            rospy.sleep(.01)

    def follow_wall(self):                                          #Follows the wall depending on left or right when a hit point is encountered
        if f_dir == 1:
            Bug.follow_wall_right(self)
        else:
            Bug.follow_wall_left(self)
        self.face_goal()

    def should_leave_wall(self):                                #Follows the goal when a leave point is encountered
        (x, y, _) = current_location.current_location()
        if None in self.encountered_wall_at:
            self.encountered_wall_at = (x, y)               #store current location parameter when robot meets obstacle
            self.lh = necessary_heading(x, y, self.tx, self.ty)
            return False
        t_angle = necessary_heading(x, y, self.tx, self.ty)
        (ox, oy) = self.encountered_wall_at
        od = math.sqrt((ox-self.tx)**2 + (oy-self.ty)**2)
        cd = math.sqrt( (x-self.tx)**2 +  (y-self.ty)**2)
        dt = 0.01

        if self.lh - dt <= t_angle <= self.lh + dt and not near(x, y, ox, oy):
            if cd < od:
                print "Leaving wall"
                return True
        return False

    def face_goal_one(self):
        while not current_location.facing_point(self.x, self.y):
            self.go_one(LEFT)
            rospy.sleep(.01)

def near(cx, cy, x, y):
    nearx = x - .3 <= cx <= x + .3
    neary = y - .3 <= cy <= y + .3
    return nearx and neary


def bug_algorithm(bug):                                     #The bug 2 algorithm is invoked
    print "Calibrating sensors..."
    rospy.sleep(1)
    print "Calibrated"
    while current_location.distance(tx, ty) > delta:
        hit_wall = bug.go_until_obstacle()
        if hit_wall:
            if f_dir == 1:
                print "Follow Right Wall"
                bug.follow_wall_right()
            else:
                print "Follow Left Wall"
                bug.follow_wall_left()
        bug.face_goal()
        bug.go(MSG_STOP)
    print "Arrived at", (tx, ty)
    rospy.sleep(2)

init()
goals_count = 0
rospy.sleep(1)
retinitialize()
print "Calculating Minimum Distances from Initial Location"

while goal_pos_curr < len(x_pos):                  #Sorting all the goal position with negative X cordinates and calculating the distances of all the goals from intial positon
    if x_pos[goal_pos_curr] <= 0:
        x_goal_pos.append(x_pos[goal_pos_curr])
        y_goal_pos.append(y_pos[goal_pos_curr])
        reward_sum.append(reward_list[goal_pos_curr])
        (curr_x,curr_y,_) = current_location.current_location()
        dist.append(math.sqrt((x_goal_pos[x_pos_curee]-curr_x)**2 + (y_goal_pos[x_pos_curee]-curr_y)**2))
        x_pos_curee = x_pos_curee + 1
    goal_pos_curr = goal_pos_curr + 1

print "Get List of goals in ascending order of their distances"
while x_pos_iterator < len(x_goal_pos):            #Getting the coordinates of the nearest goal position with negative x coordinate
    d = dist.index(min(dist))
    dist[d] = 100
    goal.append(x_goal_pos[d])
    goal.append(y_goal_pos[d])
    goal.append(reward_sum[d])
    x_pos_iterator = x_pos_iterator + 1
print "It starts now in negative x direction"
while goal_pos_iterator < len(goal):                #obtaining the X and Y cordinates based on minimum distances
    f_dir = 1
    tx = goal[goal_pos_iterator]
    ty = goal[goal_pos_iterator+1]
    print "Going to goal at location", (tx, ty)
    bug = Bug2(tx,ty)
    bug_algorithm(bug)
    print "Got reward:",goal[goal_pos_iterator+2]
    sum_reward = sum_reward + goal[goal_pos_iterator+2]
    print "Total reward:", sum_reward
    goal_pos_iterator = goal_pos_iterator + 3

retinitialize()
print "Calculating Minimum Distances from Last x positive cordinate Location"
while goal_pos_curr < len(x_pos):                   #sorting all the goal position with Positive X cordinates and calculating the distances of all the goals from current positon
    if x_pos[goal_pos_curr] > 0:
        x_goal_pos.append(x_pos[goal_pos_curr])
        y_goal_pos.append(y_pos[goal_pos_curr])
        reward_sum.append(reward_list[goal_pos_curr])
        (curr_x,curr_y,_) = current_location.current_location()
        dist.append(math.sqrt((x_goal_pos[x_pos_curee]-curr_x)**2 + (y_goal_pos[x_pos_curee]-curr_y)**2))
        x_pos_curee = x_pos_curee + 1
    goal_pos_curr = goal_pos_curr + 1
print "Get List of +x goals in ascending order of their distances"
while x_pos_iterator < len(x_goal_pos):                 #Getting the coordinates of the nearest goal position with positive x coordinates

    d = dist.index(min(dist))
    dist[d] = 100
    goal.append(x_goal_pos[d])
    goal.append(y_goal_pos[d])
    goal.append(reward_sum[d])
    x_pos_iterator = x_pos_iterator + 1

print "It starts now in postive x direction"
while goal_pos_iterator < len(goal):                #obtaining the X and Y cordinates based on minimum distances
    f_dir = 2
    tx = goal[goal_pos_iterator]
    ty = goal[goal_pos_iterator+1]
    print "Going to goal at location", (tx, ty)
    bug = Bug2(tx,ty)
    bug_algorithm(bug)
    print "Got reward:",goal[goal_pos_iterator+2]
    sum_reward = sum_reward + goal[goal_pos_iterator+2]
    print "Total reward:", sum_reward
    goal_pos_iterator = goal_pos_iterator + 3
