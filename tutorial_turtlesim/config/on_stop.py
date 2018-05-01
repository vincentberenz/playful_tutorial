import playful

print "STOPPING PLAYFUL TURTLESIM"

# ROS_WORLD and ROBOT were set in globals.py (same folder)

playful.get_global("ROS_WORLD").stop()

playful.get_global("ROBOT").clear()

