import threading
import sys

class Dist:
    def __init__(self):                                         #Initialise the lazer data for specified directions
        self.m = threading.Lock()
        self.left = 0
        self.front = 0
        self.right = 0
        self.raw = []

    def update(self, data):                                                     #It is used to obtain the minimum of laser data continously

        def getmin(a, b):
            in_rng = lambda x: data.range_min <= x <= data.range_max
            vsp = filter(in_rng, data.ranges[a:b])
            if len(vsp) > 0:
                return min(vsp)
            else:
                return sys.maxint

        newfront = getmin(330, 360)                               #Front Laser Data
        newleft = getmin(30, 80)                                  #Left Laser Data
        newright = getmin(300,330)                                #Front Laser Data
        self.m.acquire()
        self.left = newleft
        self.front = newfront
        self.right = newright
        self.raw = data
        self.m.release()

    def get(self):                                  #obtain the laser data
        self.m.acquire()
        l = self.left
        f = self.front
        r = self.right
        self.m.release()
        return (f, l, r)

    def angle_to_index(self, angle):                            #Theoretical calculating the angle between turtle bot positon and the goal position
        return int((angle - self.raw.angle_min)/self.raw.angle_increment)
