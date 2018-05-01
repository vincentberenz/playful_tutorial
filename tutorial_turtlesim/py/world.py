from turtlesim_wrapper import Turtlesim
import math,time,random,threading,rospy

# simulation of a "world" in which some turtles go around
# randomly


# a turtle that moves randomly in
# straight lines

class _Free_turtle:

    def __init__(self,
                 min_x,min_y,
                 max_x,max_y,
                 min_speed,max_speed):

        self._min = [min_x,min_y]
        self._max = [max_x,max_y]
        self._speed_limit = [min_speed,max_speed]
        
        self._position,_ = self._get_next_position()

        self._last_time_update = time.time()

        self._set_next_move()
        
        
    def _distance(self,p1,p2):

        return math.sqrt(sum([(a-b)**2 for a,b in zip(p1,p2)]))
        

    def _get_next_position(self):

        x = random.uniform(self._min[0],self._max[0])
        y = random.uniform(self._min[1],self._max[1])

        speed = random.uniform(self._speed_limit[0],
                           self._speed_limit[1])

        return [x,y],speed


    def _set_next_move(self):

        position,self._speed = self._get_next_position()
        self._target_distance = self._distance(position,self._position)
        vector = [p2-p1 for p1,p2 in zip(self._position,position)]
        norm = math.sqrt(sum([v**2 for v in vector]))
        self._vector = [v/norm for v in vector]
        self._started_position = [p for p in self._position]
        
        
    def _arrived(self):

        d = self._distance(self._position,self._started_position)

        if d > self._target_distance:
            return True

        return False
    

    def iterate(self):

        if (self._arrived()):
            self._set_next_move()
        
        t = time.time()
        time_diff = t-self._last_time_update
        traveled = self._speed * time_diff
        self._last_time_update = t
        
        self._position = [p + v*traveled for p,v in zip(self._position,self._vector)]

    def get_position(self):

        return [p for p in self._position]
        
        

# A collection of free turtles
    
class World:

    def __init__(self,nb_turtles,min_x,min_y,max_x,max_y,min_speed,max_speed):

        self._turtles = {i:_Free_turtle(min_x,min_y,max_x,max_y,min_speed,max_speed)
                         for i in range(nb_turtles)}

        self._lock = threading.Lock()
        
        
    def iterate(self):

        with self._lock:
            for turtle in self._turtles.values():
                turtle.iterate()

            
    def size(self):

        return len(self._turtles)

    
    def scan(self):

        with self._lock:
            positions = {i:t.get_position() for i,t in self._turtles.iteritems()}
            return positions


# A collection of free turtle, connected
# to turtlesmim for visualization
# note: this teleports the turtles to move them

class ROS_world:

    MIN_X = 1
    MIN_Y = 1
    MAX_X = 9
    MAX_Y = 9
    MIN_SPEED = 0.1
    MAX_SPEED = 0.8
    FREQUENCY = 15
    
    def __init__(self,nb_turtles):

        rospy.init_node('playful_turtlesim', anonymous=True)
        
        self._world = World(nb_turtles,
                            self.MIN_X,self.MIN_Y,
                            self.MAX_X,self.MAX_Y,
                            self.MIN_SPEED,self.MAX_SPEED)

        positions = self._world.scan()

        self._turtles = {}
        
        for i in range(self._world.size()):

            position = positions[i]
            ros_turtle = Turtlesim(position[0],position[1],0,pen_off=True)
            self._turtles[i] = ros_turtle

        self._center = [(self.MAX_X-self.MIN_X)/2.0,(self.MAX_Y-self.MIN_Y)/2.0]
            
        self._running = False

        
    def scan(self):

        return self._world.scan()
        
        
    def _iterate(self):

        self._world.iterate()

        positions = self._world.scan()
        
        for i in range(self._world.size()):

            position = positions[i]
            self._turtles[i].teleport(position[0],position[1],0)

            
    def get_center(self):

        return [p for p in self._center]
        
            
    def _run(self):

        self._running = True

        while(self._running):

            self._iterate()
            time.sleep(1.0/self.FREQUENCY)

            
    def start(self):

        self._thread = threading.Thread(target=self._run)
        self._thread.setDaemon(True)
        self._thread.start()

        
    def stop(self):

        if(self._running):

            self._running = False
            self._thread.join()

        for turtle in self._turtles.values():
            turtle.clear()
            

