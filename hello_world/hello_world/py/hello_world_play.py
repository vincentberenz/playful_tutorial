import playful
import datetime


class print_hello(playful.Node):

    # getting hello_to from playful script
    def __init__(self,hello_to=None):

        self._hello_to = hello_to


    def execute(self):

        # spinning while the node execution is required
        while not self.should_pause():
            
            # printing on the console
            playful.console("hello","hello "+str(self._hello_to)+" !")

            # running at 5Hz
            self.spin(5)

        # removing text from console
        playful.unconsole("hello")



class print_time(playful.Node):

    def execute(self):
        
        while not self.should_pause():

            current  = datetime.datetime.now()
            
            playful.console("current_time","current date and time: "+str(current))

            self.spin(50)

        playful.unconsole("current_time")


class print_exit_instructions(playful.Node):

    def execute(self):
        
        while not self.should_pause():

            playful.console("exit_instructions","to exit press 'q'")

            self.spin(5)

        playful.unconsole("exit_instructions")

