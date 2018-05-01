from world import ROS_world
import time

if __name__ == "__main__":

    rospy.init_node('test_world', anonymous=True)
    
    world = ROS_world(4)

    world.start()

    time.sleep(20)

    world.stop()
