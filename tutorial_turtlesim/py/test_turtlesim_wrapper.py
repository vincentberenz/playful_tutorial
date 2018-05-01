from turtlesim_wrapper import Turtlesim
import rospy,math,time


if __name__ == "__main__":        

    rospy.init_node('test_turtlesim_wrapper', anonymous=True)
    
    turtle1 = Turtlesim(4,5,0)
    turtle2 = Turtlesim(6,5,math.pi)

    def _print(turtle):

        position = turtle.get_position()
        velocity = turtle.get_velocity()

        if position and velocity:

            d = {"x":position[0],
                 "y":position[1],
                 "theta":position[2],
                 "linear":velocity[0],
                 "angular":velocity[1]}

            s = turtle.get_name()+" "+" ".join( [k+": "+str(d[k])
                           for k in ["x","y","theta","linear","angular"]] )
            print s
    
    for _ in range(500):

        turtle1.set_velocity(0.4,0,0.4)
        turtle2.set_velocity(0.4,0,0.4)

        _print(turtle1)
        _print(turtle2)
        print ""

        time.sleep(0.01)
    
    
    turtle1.clear()
    turtle2.clear()

