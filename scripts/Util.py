"""
This module is used for utility and helper functions used elsewhere in the program.

Functions:
    get_pick_height: Returns the height from which to pick the disk.
    get_place_height: Returns the height where to place the disk.
    index2state: Converts state indexes to tuples.
    printGame: Visualizes moves from start to goal state.
    game2robot: Converts moves to format readable to robot-moving function.
"""
thickness = 0.06

def get_pick_height(disk):
    """Calculate and return the height from which to pick the disk."""
    return disk*thickness - 0.05 - 0.02 - 0.005
    
def get_place_height(disk):
    """Calculate and return the height where to place the disk."""
    return disk*thickness + 0.05 -0.04 -0.025 + 0.007 

def index2state (lookup, actions, nStates):
    """
    Converts state indexes back to tuples.

    Args:
        lookup (dict): Dictionary containing state-index pairs
        actions (list): List of actions (state indexes) to convert
        nStates (int): Number of states

    Returns
        sequence (list): List of converted tuples
    """
    keys = [0 for i in range (nStates)]
    for item in lookup.items():
        keys[item[1]] = item[0]
    sequence = []
    for action in actions:
        sequence.append(keys[action])
    
    return sequence

def printGame (sequence, height):
    """
    Visualize moves from start to goal state.

    Args:
        sequence (list): List containing tuples representing states
        height (int): Height of the rod, equal to the number of disks
    """
    move = 0
    for state in sequence:
        print move, ". potez:"
        lista = list(state)
        for i in range (3):
            for j in range(height-len(state[i])):
                lista[i] = ('|',) + lista[i]
        for i in range (height):
            print ("     {0}       {1}       {2}".format(lista[0][i], lista[1][i], lista[2][i]))
        print ("============================")
        print ("============================")
        print
        move += 1

def game2robot (sequence):
    """
    Convert moves to format readable to robot-moving function.

    Args:
        sequence (list): List containing tuples representing states
    """
    commands = []
    for i in range(len(sequence)-1):
        fromState = sequence[i]
        toState = sequence[i+1]
        for j in range (3): # For each rod...
            if len(fromState[j]) > len(toState[j]): # ...if there were more disks before...
                pick_destination = j # ...this is the rod to pick from
                pick_height = len(fromState[j]) # Number of disks BEFORE picking
            elif len(fromState[j]) < len(toState[j]): # ... if there are more disks after...
                place_destination = j # ...this is the rod to place to
                place_height = len(fromState[j]) # Number of disks BEFORE placing
        commands.append( (pick_destination, pick_height, place_destination, place_height) )

    return commands