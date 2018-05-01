import playful,time,math


# distance is already defined in playful_tutorial/py/tutorial_play.py
def gap(scheme_id=None):

    target_position = playful.memory.get_property_value("position",scheme_id=scheme_id)

    if not target_position :
        return float("+inf")

    robot = playful.get_global("ROBOT")
    robot_position = robot.get_position()

    if not robot_position :
        return float("+inf")

    d = math.sqrt(sum([(r-t)**2 for r,t in
                       zip(robot_position[:2],target_position[:2])]))
    
    return d


def inv_gap(scheme_id=None,base_score=0):

    gap_ = gap(scheme_id=scheme_id)
    return base_score + 1.0/d


def abs_angle(scheme_id=None):

    target_position = playful.memory.get_property_value("position",scheme_id=scheme_id)

    if not target_position :
        return float("+inf")

    robot = playful.get_global("ROBOT")
    
    relative_position = robot.in_robot_frame(target_position)
    
    if not relative_position :
        return float("+inf")

    return abs( relative_position[2] )


def low_battery(battery_threshold=20):

    robot = playful.get_global("ROBOT")
    charge = robot.get_battery_level()

    if charge < battery_threshold:
        return True

    return False


def high_battery(battery_threshold=80):

    robot = playful.get_global("ROBOT")
    charge = robot.get_battery_level()

    if charge > battery_threshold:
        return True

    return False

