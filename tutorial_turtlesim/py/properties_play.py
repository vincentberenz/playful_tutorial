import playful


class index(playful.Property):

    
    def fuse(self,value):

        self._value=value


    # if two turtles have the same index,
    # then they are the same turtle
    def similarity(self,value):

        if self._value == value:
            return True

        return False


                      

