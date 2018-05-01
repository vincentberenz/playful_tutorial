import playful,time


# we assume perfect detection of the world by
# the robot. For each turtle in the world,
# a target scheme is pushed


class radar(playful.Node):

    def execute(self):

        # instance of ROS_world, declared in
        # world.py (same folder) and defined
        # in ../config/globals.py
        world = playful.get_global("ROS_WORLD")

        while not self.should_pause():

            index_position = world.scan()

            if index_position:

                for index,position in index_position.iteritems():

                    target = playful.create("target",
                                            position=position,
                                            time_stamp=time.time(),
                                            index=index)
                    
                    playful.memory.fuse(target)
            
            self.spin(15)
