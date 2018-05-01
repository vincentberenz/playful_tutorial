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


### PLAYFUL TUTORIAL: python leaf nodes and evaluations ###

# This file shows the code of the leaf nodes and evaluations used in all the tutorials.

# FILE NAME
# Note that the name of this file is <something>_play.py.
# Python files names must end with "_play.py" to be found by the playful interpreter.
# Once a python file found by playful interpreter, all functions and nodes it contains
# are made available to the interpreter, except private ones (functions/classes which name starts with "_")

# DIRECTORY NAME
# Note that this file is in a "py" directory. Only python files in "py" directories
# will be found by the playful interpreter (with the exception of 'on_start.py' and 'on_stop.py'
# files which should be in "config" directories)

# FOLDER STRUCTURE
# Notice the __playful__ empty file in the folder above the py directory in which this file is located.
# this empty file is required for the python interpreter to understand it is supposed to search for "*_play.py"
# files in the py directory.

# PYTHON PATH
# At the top of this file, the playful python module is imported and used.
# ("import playful")
# yet, you did not install any playful module.
# This is normal: the playful module is created on the fly by the playful executable

# Also, the "top" directory (playful_tutorial) is always added to the python path at run time.
# Therefore the module: <>/playful_tutorial/py/robot.py was succesfully imported by using: 
# "import playful_tutorial.py.robot"

# To start this tutorial, visit ../tutorial1/README.txt
# You will be invited to come back to this file to check the python code used in the playful script

###################################################################################################################


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
            # note: this resource is defined in the "resources.txt" file in
            # the "../config/" folder. Good idea to read this file.
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
            playful.memory.set("mood",current_mood)

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
    mood = playful.memory.get_value("mood") 

    if mood is None : # may happen if mood not set yet
        return False 

    if mood == target_mood :
        return True    

    return False
    

def is_happy():

    return is_of_mood(target_mood="happy")


def do_not_care():

    return is_of_mood(target_mood="do_not_care")


def is_amused():

    return is_of_mood(target_mood="amused")




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

# When you read the IEEE robotics and automation article, you saw some playful commands of the like:
#
#     targeting ball: walk, while far, priority of inv_distance
#
# and this command was valid even when there were in the environment
# several balls, e.g. a red and green one. In such case, playful would
# instantiate a walk branch for each of the detected ball, and then activate the walk 
# action regarding the closest ball (i.e. the robot always walks to the closest ball)
# In tutorial 6, we do something similar with virtual balls, and the python code
# is there.

# Using Playful terminology, 'ball' is a scheme. A scheme is
# a collection of properties. Properties are declared in Python.
# In this tutorial, a ball consists of 3 properties: position, time_stamp and color.
# The ball scheme is declared in ../play/schemes.play (check this file)
# and the code corresponding to these properties is below.

# Code of properties
# ------------------
#
# A property is basically a holder for an arbitrary chunck of data (i.e. any pytho 
# object would do) (to be held in the "self._value" member of the property)
# e.g. for 'position', the value is a 1D position; for 'color' a string (e.g. "BLUE");
#      and for 'time_stamp' a time value (as returned by time.time())
# Code is trivial, but there is a suble point to pay attention to:
# the similarity function returns "None" for position and time_stamp, and "True" or "False"
# for color. This indicate to the playful engine that
# 1) seeing a red ball
# followed by
# 2) seeing a blue ball
# corresponds to seeing two different balls,
# (and not seeing a single ball which changed color)

# This may sound obvious, but this has to be encoded somehow. Also note this
# is arbitrary and application specific. In some case, it is known there can
# be only one ball in the environment (e.g. by convention, as in robot soccer). In other cases, you may want to encode
# that if the robot 1) sees a red ball at position p1, followed by 2) sees a red ball
# at position p2 (which is very far from p1), then the robot saw 2 balls, and not a single ball which
# moved fast. These considerations may sound weird, but will grantly gain in importance as service robotics advances:
# a waiter robot will have to reason all day long about the number and motions of surrounding coffee cups. Object permanence
# is trivial for now only because we ask trivial things to our robots. These considerations are overkill for
# this tutorial, but we have to get prepared for the future
# (note: this is overkill, but you may notice that if explanation is long, code below is short)

# If you wish more information on the fuse and similarity functions used below, you may read:
# https://am.is.tuebingen.mpg.de/publications/conf-humanoids-berenztsh11
# (only section 3.A, rest of the paper is deprecated. This paper is about TDM, which is Playful's grandpa.
#  Both systems are similar in regards of schemes and properties).

class position(playful.Property):

    def fuse(self,value):
        self._value=value

    def similarity(self,value):
        return None

class time_stamp(playful.Property):

    def fuse(self,value):
        self._value=value

    def similarity(self,value):
        return None

class color(playful.Property):

    def fuse(self,value):
        self._value=value

    def similarity(self,value):
        if self._value == value :
            return True
        return False


# Code of nodes
# -------------
#
# Leaf node simulating the perception by robot of virtual balls
# (in real applications, this could be a node using for example opencv over a video
# stream to detect balls, determining their color, computing their absolute position,
# and pushing related ball schemes to the memory)
#
# This node simulates the follows;
# From the start, a blue ball is detected; and a green ball is detected
# 4s after this.
#
# This node encodes info into a ball schemes that it pushes to the memory.
# The memory uses the similarity and fusing functions to update
# existing schemes and create new onces.
#
# When running the tutorial, you may see the program expanding during runtime
# after 4 seconds : the targeting keyword (in the playful script) commands
# a new leaf node corresponding to the green ball to be created.
#
# If you find all this confusing, this is normal at first. Just notice
# that the playful script in tutorial 6 is short and elegant.
#
# Also, feel free to uncomment code in the node below to simulate a 3rd red ball, and
# notice the playful script does not need to be updated:
#     'targeting ball: display' (in the playful script)
# will work for any arbitrary number of ball perceived by the robot
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
            playful.memory.fuse(playful.create("ball",position=blue.update(),time_stamp=t,color="BLUE"))

            # the green ball is created 4 seconds after start
            if(t-time_start > 4):
                playful.memory.fuse(playful.create("ball",position=green.update(),time_stamp=t,color="GREEN"))

            # uncomment for fun
            #playful.memory.fuse(playful.create("ball",position=red.update(),time_stamp=t,color="RED"))

            self.spin(20)


# class for displaying a ball in the console
# (note that the code is not specific to ball, though:
# it is specific to scheme with a color and position properties)

class ball_display(playful.Node):

    def execute(self):

        while not self.should_pause():
        
            # the targeting keyword in the playful script associated this instance of
            # 'ball_display' with one of the ball scheme. 'self.get_scheme_id()'
            # returns the unique id of the ball scheme this instance is associated to,
            # and the get_propery_value function allows to access the properties of this scheme.
            position = playful.memory.get_property_value("position",scheme_id=self.get_scheme_id())
            color = playful.memory.get_property_value("color",scheme_id=self.get_scheme_id())

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

def distance(scheme_id=None):

    position = playful.memory.get_property_value("position",scheme_id=scheme_id)
    if not position :
        return float("+inf")

    return abs(Playful_tutorial_robot.position-position)


##############
# TUTORIAL 8 #
##############

# evaluation that returns a number higher as the robot gets closer
def inv_distance(scheme_id=None,base_score=None):
    d = distance(scheme_id=scheme_id)
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
                position_ball = playful.memory.get_property_value("position",scheme_id=self.get_scheme_id())

                if position_ball:

                    # current position of the robot
                    position_robot = Playful_tutorial_robot.position

                    # moving toward the ball
                    diff = abs(position_ball - position_robot)
                    move = self.speed
                    if move>diff : move = diff
                    if position_ball > position_robot : Playful_tutorial_robot.position += move
                    else: Playful_tutorial_robot.position -= move
                
            else :

                self.release_all_resources()

                    
            self.spin(20)
