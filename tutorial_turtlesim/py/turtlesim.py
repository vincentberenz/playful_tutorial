import rospy
from geometry_msgs.msg import Twist

class Turtlesim:
    
    VELOCITY_TOPIC = "cmd_vel"
    VELOCITY_PUBLISHER = None
    ROBOT_NAME = None
    CURRENT_VELOCITY = [0,0,0] # [linear x, linear y, angular z]
    
    @classmethod
    def init(cls,robot_name="turtle1"):

        try:
            rospy.init_node(robot_name, anonymous=True)
        except :
            pass
            
        cls.ROBOT_NAME = robot_name
        cls.VELOCITY_PUBLISHER = rospy.Publisher(robot_name+"/"+cls.VELOCITY_TOPIC,
                                                 Twist, queue_size=10)
    


    @classmethod
    def _init_msg(cls):

        msg = Twist()
        msg.linear.x = cls.CURRENT_VELOCITY[0]
        msg.linear.y = cls.CURRENT_VELOCITY[1]
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = cls.CURRENT_VELOCITY[2]

        return msg
        
        
    @classmethod
    def set_velocity(cls,x=None,y=None,angular=None):

        msg = cls._init_msg()

        if x is not None:
            msg.linear.x = x

        if y is not None:
            msg.linear.y = y

        if angular is not None:
            msg.angular.z = angular

        cls.VELOCITY_PUBLISHER.publish(msg)

        
    @classmethod
    def stop(cls):

        cls.set_linear_velocity(x=0,y=0,angular=0)

        
