import threading
import math
import sys

class Location:
    def __init__(self):
        self.m = threading.Lock()
        self.l = threading.Lock()
        self.x = None
        self.y = None
        self.t = None
        self.deltaT = 0.05


    def update_location(self, x, y, t):                         #Automatically updates the current location of turtle bot.
        self.m.acquire()
        self.x = x
        self.y = y
        self.t = t
        self.m.release()

    def current_location(self):                                 #Used to get the current location of the turtle bot.
        self.m.acquire()
        x = self.x
        y = self.y
        t = self.t
        self.m.release()
        return (x, y, t)

    def distance(self, x, y):                                 #Calculates the distance to the goal from the current turtle bot position
        (x0, y0, _) = self.current_location()
        if x0 == None or y0 == None:
            return sys.maxint
        return math.sqrt((x-x0)**2 + (y-y0)**2)

    def facing_point(self, x, y):                           #the function checks if the turtle bot is facing the goal or not
        (cx, cy, current_heading) = self.current_location()
        if None in (cx, cy, current_heading):
            return False
        n = necessary_heading(cx, cy, x, y)

        return n - self.deltaT <= current_heading <= n + self.deltaT

    def faster_left(self, x, y):                                                    #it is evoked when the above function returns false.The turtle bot rotates in the left direction
        (cx, cy, current_heading) = self.current_location()                         # till it faces the goal position
        if None in (cx, cy, current_heading):
            return False
        return current_heading - necessary_heading(cx, cy, x, y) < 0



def necessary_heading(cx, cy, tx, ty):                                              #Calculates the tangent angel btween the goal and current turtle bot position
    return math.atan2(ty-cy, tx-cx)
