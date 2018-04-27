#!/usr/bin/env python

# This class and related functions are an abstraction of the turtlesim above ROS,
# i.e. it hides calls to services and publishers in a convenient python API


import math,time

try:
    import rospy
    from geometry_msgs.msg import Twist
except:
    raise Exception("ROS needs to be installed. Please visit http://www.ros.org/")
    
try :
    from turtlesim.srv import Spawn
    from turtlesim.srv import TeleportAbsolute
    from turtlesim.srv import Kill
except :
    raise Exception("turtlesim.py requires turtlesim to be installed. Please visit http://wiki.ros.org/turtlesim")


_SPAWN_SERVICE = "spawn"
_VELOCITY_TOPIC = "cmd_vel"
_TELEPORT_SERVICE = "teleport_absolute"
_KILL_SERVICE = "kill"
_PERMANENT_TURTLE_NAME = "turtle1"


class Turtlesim:

    CALLED_AT_LEAST_ONCE = False


    def _spawn(self,x,y,theta):

        global _SPAWN_SERVICE
        
        rospy.wait_for_service(_SPAWN_SERVICE)
        service = rospy.ServiceProxy(_SPAWN_SERVICE, Spawn)
        response = service(x, y, theta, None)

        return response.name
    
    
    def __init__(self,x,y,theta):

        global _VELOCITY_TOPIC
        global _PERMANENT_TURTLE_NAME

        # if this is not the first turtlesim to be initialized, it requires to
        # be spawned
        if self.__class__.CALLED_AT_LEAST_ONCE is True:
            self.name = self._spawn(x,y,theta)

        # otherwise the new turtle is created at 0,0,0 ; not a x,y,theta
        else:
            self.name = _PERMANENT_TURTLE_NAME
            self.teleport(x,y,theta)
            self.__class__.CALLED_AT_LEAST_ONCE = True
            
        self.current_velocity = [0,0,0] # [linear x, linear y, angular z]

        # for setting the desired velocity of this turtle
        self.velocity_publisher = rospy.Publisher(self.name+"/"+_VELOCITY_TOPIC,
                                                  Twist, queue_size=10)
        

        
    def get_name(self):

        return self.name

    
    def _init_msg(self):

        msg = Twist()
        msg.linear.x = self.current_velocity[0]
        msg.linear.y = self.current_velocity[1]
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = self.current_velocity[2]

        return msg
        
        
    def set_velocity(self,x=None,y=None,angular=None):

        msg = self._init_msg()

        if x is not None:
            msg.linear.x = x

        if y is not None:
            msg.linear.y = y

        if angular is not None:
            msg.angular.z = angular

        self.velocity_publisher.publish(msg)

        
    def teleport(self,x,y,theta):

        global _TELEPORT_SERVICE
        
        rospy.wait_for_service(self.name+"/"+_TELEPORT_SERVICE)
        service = rospy.ServiceProxy(self.name+"/"+_TELEPORT_SERVICE, TeleportAbsolute)
        service(x, y, theta)

        
    def stop(self):

        self.set_linear_velocity(x=0,y=0,angular=0)

        
    def __del__(self):

        global _PERMANENT_TURTLE_NAME

        if(self.name == _PERMANENT_TURTLE_NAME):
            return
        
        global _KILL_SERVICE
        
        rospy.wait_for_service(_KILL_SERVICE)
        service = rospy.ServiceProxy(_KILL_SERVICE, Kill)
        service(self.name)


        
def spawn_turtle(x,y,theta):

    return Turtlesim(x,y,theta)



if __name__ == "__main__":        

    rospy.init_node('turtlesim_test', anonymous=True)
    
    turtle1 = spawn_turtle(20,0,0)
    turtle2 = spawn_turtle(20,0,math.pi)

    for _ in range(200):
        turtle1.set_velocity(0.5,0,0.1)
        turtle2.set_velocity(0.5,0,0.1)
        time.sleep(0.01)
