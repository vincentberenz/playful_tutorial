# Copyright 2018 Max Planck Society. All rights reserved.
# Author: Vincent Berenz

# This file is part of Playful Tutorial.
 
# Playful Tutorial is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# Playful Tutorial is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
#along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

##############################################################################



# Class representing our simple virtual robot:
# A facial expression (emoticon) and a 1D position
# This interface to this simple robot
# is the equivalent of the robot middleware in real non-trivial applications.

class Playful_tutorial_robot:

    expression = ":-|"
    position = 0

    @classmethod
    def start(cls):
        print "robot says: hello !"
        cls.set_expression(cls.expression)

    @classmethod
    def stop(cls):
        print "robot says: goodbye !"

    @classmethod
    def set_expression(cls,expression):
        cls.expression = expression

    @classmethod
    def set_position(cls,position):
        cls.position = position

    @classmethod
    def get_position(cls):
        return cls.position

    @classmethod
    def get_representation(cls):
        return " "*int(cls.position)+cls.expression
