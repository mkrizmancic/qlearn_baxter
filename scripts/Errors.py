from Util import *

def RaiseNotReachable():
    user_print("Locations not reachable! No valid joint solution found!", 'error')
    return 0

def RaisePickFailed():
    user_print("Pick action failed!", 'error')
    return 0

def RaisePlaceFailed():
    user_print("Place action failed!", 'error')
    return 0

def RaiseGoToFailed(task, destination, height, offset_x, offset_y, offset_z):
    user_print("GoTo action failed!", 'error')
    print ("Task: {0}".format(task))
    print ("Destination: {0}".format(destination))
    print ("Hight: {0}".format(height))
    print ("x: {0}, y: {1}, z: {2}".format(offset_x, offset_y, offset_z))
    return 0

def RaisePickAndPlaceFailed():
    user_print("Pick and place action failed!", 'error')
    return 0
