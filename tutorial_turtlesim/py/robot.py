from turtlesim_wrapper import Turtlesim

import time,math

# a robot is a turtle +
# a battery that get depleted
# when swimming (and recharged when
# at spawning place)


class Robot(Turtlesim):

    DECHARGE = 1.0 # battery unit per seconde
    BATTERY_CAPACITY = 100.0
    CHARGING_THRESHOLD = 1.0
    
    def __init__(self,x,y,theta):

        Turtlesim.__init__(self,x,y,theta)
        self._battery_level = self.BATTERY_CAPACITY
        self._center = [x,y]
        self._last_update = time.time()
        
        
    def distance_to_charger(self):

        position = self.get_position()
        distance = math.sqrt(sum([(c-p)**2 for c,p in zip(self._center,position)]))

        return distance


    # override of Turtlesim set_velocity:
    # not moving if battery depleted
    def set_velocity(self,x,y,theta):

        if not self.has_charge():
            super(Turtlesim,self).set_velocity(0,0,0)
            return

        Turtlesim.set_velocity(self,x,y,theta)

        
    def is_charging(self):

        return self.distance_to_charger() < self.CHARGING_THRESHOLD

    
    def update_battery(self):

        t = time.time()
        time_diff = t-self._last_update
        decharge = - time_diff * self.DECHARGE

        if  self.is_charging():
            decharge = -decharge

        self._battery_level += decharge

        if self._battery_level < 0:
            self._battery_level = 0
            
        elif self._battery_level > self.BATTERY_CAPACITY:        
            self._battery_level = self.BATTERY_CAPACITY


    def get_battery_level(self):

        return self._battery_level
    
            
    def has_charge(self):

        return self._battery_level > 0


    def in_robot_frame(self,absolute_position):

        if not absolute_position :
            return None

        robot_position = self.get_position()

        if not robot_position :
            return None

        theta = robot_position[2]
        
        gamma = math.atan2(absolute_position[1],
                           absolute_position[0])

        beta = theta-gamma

        d = math.sqrt(sum([(a-r)**2 for a,r in zip(absolute_position,robot_position)]))

        return [d*math.cos(beta),d*math.sin(beta),beta]
