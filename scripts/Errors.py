def RaiseNotReachable():
    print "Locations not reachable! No valid joint solution found!"
    return 0

def RaisePickFailed():
    print "Pick action failed!"
    return 0

def RaisePlaceFailed():
    print "Place action failed!"
    return 0

def RaiseGoToFailed(task, destination, height, offset_x, offset_y, offset_z):
    print "GoTo action failed!"
    print ("Task: {0}".format(task))
    print ("Destination: {0}".format(destination))
    print ("Hight: {0}".format(height))
    print ("x: {0}, y: {1}, z: {2}".format(offset_x, offset_y, offset_z))
    return 0

def RaisePickAndPlaceFailed():
    print "Pick and place action failed!"
    return 0
