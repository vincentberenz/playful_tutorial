# Copyright 2018 Max Planck Society. All rights reserved.
# Author: Vincent Berenz

# This file is part of Playful Tutorial.
 
# Playful Tutorial is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# Playful Tutorial is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with Playful Tutorial.  If not, see <http://www.gnu.org/licenses/>.

##############################################################################


import playful,time,math
from playful_tutorial.py.robot import Playful_tutorial_robot


# ###################################################### #
#                                                        #
# For more information, visit the wiki at :              #
# https://github.com/vincentberenz/playful_tutorial/wiki #
#                                                        #
# ###################################################### #




##############
# TUTORIAL 1 #
##############

# This is a playful leaf node. It asks for access to the "console" resource and
# (only when having it) displays its value
class display(playful.Node):

    # allowing the playful script to pass the "value" argument
    def __init__(self,value="no value"):
        self.value = value

    # execution code
    def execute(self):

        # looping for as long the engine does not require this
        # node to stop
        while not self.should_pause():

            # asking to use the "console" resource
            if self.ask_for_resource("console"):

                # access to resource was granted: displaying "value".
                # the console function prints strings in terminal during runtime.
                # it takes an arbitrary string id as first argument
                # and the string to print as second argument. 

                playful.console(str(id(self)),"\n"+str(self.value)+"\n")

            else :

                # 'ask_for_resource' returned false:
                # access to the "console" resource was revoked, so we should
                # free it

                # removing what this node was displaying (if anything)
                playful.unconsole(str(id(self)))

                # freeing the resource (if taken).
                # not doing so would
                # prevent other nodes to access the resource
                self.release_all_resources()

                # note: code here may be called even if the resource has not
                # been taken by this node or when the node already
                # released the resource. This is fine.
                
            # operating at 10Hz
            self.spin(10)

        # leaving execution of the node,
        # removing what the node was displaying
        # (if anything)
        playful.unconsole(str(id(self)))

        # note: when a node exits, it automatically releases any
        #       resource it was using 
        

####################
# TUTORIAL 2 and 3 #
####################

# any python function can be used as evaluation.
# this function returns true when time.time()%5 is 0, so
# 1 second every 5 seconds

def time_modulo_5():

    t = int(time.time())
    modulo = t%5

    if modulo == 0 : 
        return True

    return False


##############
# TUTORIAL 4 #
##############

def time_sin():

    t = time.time()
    return 1.0 + math.sin(t)


def time_cos():

    t = time.time()
    return 1.0 + math.cos(t)


##############
# TUTORIAL 5 #
##############

# mood manager lead node:
# manages a virtual "mood" which
# alternates sequentially between "happy", "do not care" and
# "amused". This node shares the current mood
# in a shared memory using the "mood" memory key
class mood_manager(playful.Node):

    def execute(self):

        # returns the last digit of current time,
        # i.e. if current time is 162363.7
        #      returns 3
        def _time_digit():
            t = int(time.time())
            last_number = str(t)[-1]
            return int(last_number)

        # evaluates if value between
        # min and max
        def _in_range(value,min_,max_):
            if value < min_ :
                return False
            if value > max_ :
                return False
            return True
        
        while not self.should_pause():

            # alternates sequentially the mood between "happy",
            # "do not care" and "amused"

            digit = _time_digit()
            current_mood = None
            
            if _in_range(digit,0,3): current_mood = "happy"
            elif _in_range(digit,4,6): current_mood = "do_not_care"
            else : current_mood = "amused"

            # setting value of the "mood" memory key
            playful.Memory.set("mood",current_mood)

            # displaying to user current mood
            playful.console("mood_manager","current mood: "+current_mood)
            
            self.spin(10)


# nodes which set the facial expression of the robot.
# See "robot.py" in the same folder as this file
class set_expression(playful.Node):

    def __init__(self,expression=":-|"):
        self.expression = expression

    def execute(self):

        while not self.should_pause():

            # the 'face' resource is defined in ../config/resources.txt
            if self.ask_for_resource("face"):
                Playful_tutorial_robot.set_expression(self.expression)

            else :
                self.release_all_resources()

            self.spin(10)

            
class smile(set_expression):
    
    def __init__(self):
        set_expression.__init__(self,expression=":-)")

        
class stay_put(set_expression):

    def __init__(self):
        set_expression.__init__(self,expression=":-|")

        
class laugh(set_expression):

    def __init__(self):
        set_expression.__init__(self,expression=":-p")


# various evaluations reading the mood from shared memory
# and returning True or False 

def is_of_mood(target_mood=None):

    # reading current mood from the shared memory
    mood = playful.Memory.get("mood") 

    if mood is None : # may happen if mood not set yet
        return False 

    if mood == target_mood :
        return True    

    return False
    

def is_happy():
    r = is_of_mood(target_mood="happy")
    return r

def do_not_care():
    r = is_of_mood(target_mood="do_not_care")
    return r

def is_amused():
    r = is_of_mood(target_mood="amused")
    return r




# node which displays the robot
# Playful_tutorial_robot defined in the
# robot.py file (same folder as this file)
class display_robot(playful.Node):

    def execute(self):

        while not self.should_pause():

            playful.console("robot",Playful_tutorial_robot.get_representation())

            self.spin(15)

            
##############            
# TUTORIAL 6 #
##############


# defines a ball, allowing the keyworkd "targeting"
# to be used toward ball, e.g. targeting ball: ball_display

class Ball:

    def __init__( self,
                  position=None,
                  time_stamp=None,
                  color=None ):
        
        self.position = position
        self.time_stamp = time_stamp
        self.color = color


# Code of nodes
# -------------
#
class virtual_balls_detection(playful.Node):

    def execute(self):

        # simulating bouncing balls in 1D
        class _Position_manager:

            def __init__(self,initial_position,speed,direction=1,max_x=30):
                self.position = initial_position
                self.speed = speed
                self.direction = direction
                self.max_x = max_x

            def update(self):
                self.position += self.direction * self.speed
                if self.position > self.max_x :
                    self.position = self.max_x
                    self.direction = -1 * self.direction
                if self.position < 0 :
                    self.position = 0
                    self.direction = -1 * self.direction
                return self.position
                
        time_start = time.time()

        blue = _Position_manager(0,0.2)
        green = _Position_manager(0,0.1)
        #you may uncomment for fun
        #red = _Position_manager(10,0.15)

        while not self.should_pause():

            t = time.time()

            # creating a ball objects from properties, and sending it to the memory
            # "fuse" means the memory compares the incoming scheme with schemes already
            # maintained in the memory (similarity functions of the properties) and either maintain a new
            # scheme or fuse with an existing scheme (fuse functions of the properties)

            # the blue ball is created from start
            blue_ball = Ball( position=blue.update(),
                              time_stamp=t,
                              color="BLUE" )
            playful.Memory.set(blue_ball,"BLUE")

            # the green ball is created 4 seconds after start
            if(t-time_start > 4):
                green_ball = Ball( position=green.update(),
                                   time_stamp=t,
                                   color="GREEN" )
                playful.Memory.set(green_ball,"GREEN")

            # uncomment for fun
            #red_ball = Ball( position=red.update(),
            #                 time_stamp=t,
            #                 color="RED" )
            #playful.Memory.set(red_ball,"RED")

            self.spin(20)


# class for displaying a ball in the console
# (note that the code is not specific to ball, though:
# it is specific to scheme with a color and position properties)

class ball_display(playful.Node):

    def execute(self):

        while not self.should_pause():
        
            # the targeting keyword in the playful script associated this instance of
            # 'ball_display' with one of the ball, that is retrieved here with the get_target
            # method
            ball = self.get_target()

            if ball is not None:
            
                position = ball.position
                color = ball.color

                if position and color :
                    playful.console(str(id(self))," "*int(position)+str(color))
            
            self.spin(10)

            
##############
# TUTORIAL 7 #
##############
            
            
# Distance is an evaluation based on the position property.
# Similarly to the ball_display node, it needs to be used
# along the "targeting" keyword.
# The engine will pass the correct scheme_id parameter automatically

def distance(target=None):
    
    if not target or not target.position :
        return float("+inf")
    return abs(Playful_tutorial_robot.position-target.position)


##############
# TUTORIAL 8 #
##############

# evaluation that returns a number higher as the robot gets closer
def inv_distance(target=None,base_score=None):
    d = distance(target=target)
    if d<1 : d = 1
    return base_score + 1.0/d

# node that moves the virtual robot toward any scheme with a 'position' property
class follow(playful.Node):

    def __init__(self,speed=0.1):
        self.speed = speed
    
    def execute(self):

        while not self.should_pause():

            if self.ask_for_resource("wheels"):
            
                # position of the targeted ball
                target = self.get_target()
                
                if target:

                    # current position of the robot
                    position_robot = Playful_tutorial_robot.position

                    # moving toward the ball
                    diff = abs(target.position - position_robot)
                    move = self.speed
                    if move>diff : move = diff
                    if target.position > position_robot : Playful_tutorial_robot.position += move
                    else: Playful_tutorial_robot.position -= move
                
            else :

                self.release_all_resources()

                    
            self.spin(20)
