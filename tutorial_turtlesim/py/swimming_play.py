import playful


# turtle moving the legs to control
# forward and lateral velocity

class slide(playful.Node):

    
    def __init__(self,kp=1,kd=1):

        self.kp = kp
        self.kd = kd

        
    def execute(self):

        robot = playful.get_global("ROBOT")
        
        while not self.should_pause():

            if self.ask_for_resource("legs"):
            
                # in absolute frame
                target = self.get_property("position")
                
                # in robot frame
                target = robot.in_robot_frame(target)
                
                # robot current velocity
                velocity = robot.get_velocity()
                
                if target and velocity:
                    
                    control_x = self.kp * target[0] - self.kd * velocity[0]
                    control_y = self.kp * target[1] - self.kd * velocity[1]
                    
                    robot.set_velocity(control_x,control_y,None)

            else :

                self.release_all_resources()

            self.spin(10)

    
                    
# turtle moving the tail to control
# orientation

class steer(playful.Node):

    def __init__(self,kp=1,kd=1):

        self.kp = kp
        self.kd = kd

    def execute(self):

        robot = playful.get_global("ROBOT")
        
        while not self.should_pause():

            if self.ask_for_resource("tail"):
            
                # in absolute frame
                target = self.get_property("position")
                
                # in robot frame
                target = robot.in_robot_frame(target)
                
                # robot current velocity
                velocity = robot.get_velocity()
                
                if target and velocity:

                    angle = math.atan2(target[1],target[0])

                    control_angle = self.kp * angle - self.kd * velocity[2]
                    
                    robot.set_velocity(None,None,control_angle)

            else :

                self.release_all_resources()

            self.spin(10)


# breaking to stop

class stop(playful.Node):

    def execute(self):

        robot = playful.get_global("ROBOT")
        
        while not self.should_pause():

            if self.ask_for_resource("tail"):

                robot.set_velocity(0,0,None)

            else:

                self.release_resource("tail")
                
            if self.ask_for_resource("legs"):

                    robot.set_velocity(0,0,0)

            else:

                self.release_resource("legs")

            self.spin(10)

            
                                



