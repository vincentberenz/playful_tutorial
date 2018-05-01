import playful


# displaying the current charge of the robot.
# robot defined in robot.py (same folder)
# and declared in globals.py (../config)

class display_charge(playful.Node):

    def execute(self):

        robot = playful.get_global("ROBOT")

        while not self.should_pause():

            charging_str = ""
            if robot.is_charging():
                charging_str = "[CHARGING]"
            
            charge = robot.get_battery_level()

            playful.console(str(id(self)),charging_str+" battery_level: "+str(charge))

            self.spin(10)
            
