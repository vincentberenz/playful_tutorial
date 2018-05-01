import playful


print "STARTING PLAYFUL TURTLESIM"


# ROS_WORLD was set in globals.py (same folder)

playful.get_global("ROS_WORLD").start()


# setting a charger scheme in the middle of the world
# charger scheme defined in ../play/schemes.play

center = playful.get_global("ROS_WORLD").get_center()
charger = playful.create("charger",
                         position=center)
playful.memory.fuse(charger)

