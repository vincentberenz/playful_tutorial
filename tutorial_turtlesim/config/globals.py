from playful_tutorial.tutorial_turtlesim.py.world import ROS_world
from playful_tutorial.tutorial_turtlesim.py.robot import Robot


# initializing a simulated world
# with 3 turtles swimming randomly
# in it. This simulated world is started
# in on_start.py, and stopped in on_stop.py

ROS_WORLD = ROS_world(3)


# adding a fourth turtle
# this one is the "robot" we control

ROBOT = Robot(ROS_WORLD.get_center()[0],ROS_WORLD.get_center()[1],0)

