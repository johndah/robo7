# Object message containing the type(class) and position of an object

int8 obj_class            # class of the object

geometry_msgs/Point pos   # object coordinates in map frame

int32[] weights            # number of votes for each class,
                          # eg. weights[2] = 21 means that class 2 has 21 votes.
                          # every vote has been used for positioning average


int8 total_votes          # total number of votes for the objects position.
