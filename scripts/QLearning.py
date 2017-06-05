#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import pow
from random import randint

from Util import *


# Comments beginning with "noinspection" are PyCharm auto-generated comments
# noinspection PyUnboundLocalVariable,PyShadowingNames

class QLearn:
    """
    This class is an implementation of a Q-Learning algorithm for solving Tower of Hanoi problem.

    Understanding class implementation requires knowledge about the algorithm and the problem.
    You can find out more about Q-Learning here: https://en.wikipedia.org/wiki/Q-learning
    If you are interested in Tower of Hanoi, please see: https://en.wikipedia.org/wiki/Tower_of_Hanoi
    You can also compare this class to a similar one written in C# here:
    https://github.com/Kenandeen/machine-learning/tree/master/TowersOfHanoi

    Attributes:
        nStates (int): Number of all possible states
        gama (float): Discount factor (see qlearning)
        Q (2D float array): Q matrix
        R (2D int array): R matrix
        lookup (dict): Stores relation between states and matrix indexes
    """

    # noinspection PyUnusedLocal
    def __init__(self, numberOfDisks, discountFactor=0.95, learningRate=0.5):
        """
        Initialize Q and R matrices, set local helper variables and start state generation.

        Args:
            numberOfDisks (int): Number of disks in Tower of Hanoi problem
            discountFactor (float): Discount factor (see qlearning)
        """
        # Game related variables
        self.nStates = int(pow(3, numberOfDisks))

        # Variables related to Q-Learning algorithm
        self.gama = discountFactor
        self.alpha = learningRate
        self.Q = [[0 for i in range(self.nStates)] for j in range(self.nStates)]
        self.R = [[-1 for i in range(self.nStates)] for j in range(self.nStates)]
        self.lookup = {}

        # Start state generation
        StateGenerator(self.R, self.nStates, numberOfDisks, self.lookup)

    def learn(self):
        """
        Train Q matrix (learning process). Find out more by studying Q-Learning.

        'goal', 'current' and 'next' are indexes of goal, current and next states respectively.

        Args: none
        """
        goal = self.nStates - 1  # Goal state index
        episodes = 3 * self.nStates
        for i in range(episodes):  # Repeat learning process
            sys.stdout.write("  {:.0f}% \r".format(i * 100.0 / episodes))  # Display current progress
            sys.stdout.flush()
            current = randint(0, goal)  # Select random starting state
            while current != goal:
                temp = []
                for possible in range(self.nStates):  # Find possible next states
                    if self.R[current][possible] != -1:
                        temp.append(possible)
                next = temp[randint(0, len(temp) - 1)]  # Select one of the possible states at random
                maxQ = max(self.Q[next])  # Find maximum Q value for next state

                # Update the Q matrix
                self.Q[current][next] += self.alpha * (self.R[current][next] + self.gama * maxQ - self.Q[current][next])
                current = next  # Start from the new state in next iteration
        return

    def play(self, start):
        """
        Utilize gained knowledge to solve the problem.

        'goal', 'current' and 'maxState' are indexes of goal, current and state with max value respectively.

        Args:
            start (int): Index of a start state

        Returns:
            actions (list of ints): list containing optimal sequence of states from start to goal
        """
        goal = self.nStates - 1
        current = start
        actions = [start]
        while current != goal:
            maximum = 0  # Maximum value in a row of a Q matrix
            for i in range(self.nStates):
                if self.Q[current][i] > maximum:
                    maximum = self.Q[current][i]
                    maxState = i
            current = maxState
            actions.append(current)
        return actions


# noinspection PyPep8Naming,PyUnusedLocal,PyMethodMayBeStatic,PyUnboundLocalVariable,PyTypeChecker
class StateGenerator:
    """
    Class for generating states for Tower of Hanoi problem and filling out R matrix.

    State is in form of a tuple containing three other tuples representing three rods.
    In each of the three tuples are int values representing disks.
    The smaller the number, the smaller the disk. Left (first, index 0) element is on top.
    States are generated using iterative recursion. New (unexpanded) states are placed on a stack
    from where they are popped one by one until all states have been generated.
    A visited list makes sure no state is expanded more than once.

    Attributes:
        startState (tuple): Starting state
        finalState (tuple): Goal state
    """

    # noinspection PyShadowingNames
    def __init__(self, R, nStates, numberOfDisks, lookup):
        """
        Create helper variables: list of generated states, dictionary and start and end positions.
        Start generation process.

        Args:
            R (2D array): Reference to the R-matrix from QLearn (lists in python are immutable)
            nStates (int): Total number of states
            numberOfDisks (int): Number of disks in game
            lookup (dict): Stores relation between states and matrix indexes
        """
        self.R = R

        # Helper variables
        self.visited = [[0 for i in range(nStates)] for j in range(nStates)]
        generated = []
        self.lookup = lookup
        self.freeIndex = 1  # Next available index for storing storing states in dictionary
        ordered = tuple()  # Tuple containing disks in correct starting and final order
        for i in range(numberOfDisks):
            ordered += (i + 1,)

        # Set initial values
        self.startState = (ordered, (), ())
        self.finalState = ((), (), ordered)
        self.lookup[self.startState] = 0
        goal = nStates - 1
        self.lookup[self.finalState] = goal

        toGenerate = [self.startState]  # Stack containing states for expansion

        while len(toGenerate) > 0:
            fromState = toGenerate.pop()
            while fromState in generated:
                if len(toGenerate) <= 0:
                    self.R[goal][goal] = 100
                    return
                fromState = toGenerate.pop()
            generated.append(fromState)
            fromIndex = self.lookup[fromState]

            for i in range(3):  # For each rod...
                if len(fromState[i]):  # ... if it is not empty...
                    moveDisk = (fromState[i][0],)  # ... remove the top disk

                    nextState = self.getNextState(fromState, moveDisk, 2 * i)  # Place disk on another rod
                    nextIndex = self.updateRMatrix(nextState, fromIndex)
                    if nextIndex:  # If this was unvisited state, generate another
                        toGenerate.append(nextState)

                    nextState = self.getNextState(fromState, moveDisk, 2 * i + 1)
                    nextIndex = self.updateRMatrix(nextState, fromIndex)
                    if nextIndex:
                        toGenerate.append(nextState)

    def getNextState(self, state, moveDisk, step):
        """
        Create new states from origin state.

        Simulate physical moving of top disk from one rod to the other. There are 6 possible moves.
        Use tuple manipulation to create a new state tuple.

        Args:
            state (tuple): Current state
            moveDisk (int): Number representing size of the disk to be moved
            step (int): Which of the possible moves should be simulated

        Returns:
            nextState (tuple): Generated state
        """
        if step == 0:
            nextState = (state[0][1:], moveDisk + state[1], state[2])
        elif step == 1:
            nextState = (state[0][1:], state[1], moveDisk + state[2])
        elif step == 2:
            nextState = (moveDisk + state[0], state[1][1:], state[2])
        elif step == 3:
            nextState = (state[0], state[1][1:], moveDisk + state[2])
        elif step == 4:
            nextState = (moveDisk + state[0], state[1], state[2][1:])
        elif step == 5:
            nextState = (state[0], moveDisk + state[1], state[2][1:])
        return nextState

    def updateRMatrix(self, nextState, fromIndex):
        """
        Update R matrix for given transition from state to state.

        Args:
            nextState (tuple): State we are transitioning to
            fromIndex (int): Index of a state we are transitioning from

        Returns:
            Index of next state if this transition has not yet been noted in R matrix
            0 (False) otherwise
        """
        if self.isValid(nextState):
            if nextState not in self.lookup:  # If this state doesn't yet have an index in dictionary...
                self.lookup[nextState] = self.freeIndex  # ... create a new one
                nextIndex = self.freeIndex
                self.freeIndex += 1  # Update the next available (not used) index
            else:
                nextIndex = self.lookup.get(nextState)

            if self.visited[fromIndex][nextIndex] == 0:  # If transition has not yet been evaluated...
                if nextState == self.finalState:  # ... if it is a final state, add big reward
                    self.R[fromIndex][nextIndex] = 100
                else:
                    self.R[fromIndex][nextIndex] = 0  # ... if it is a valid transition
                self.visited[fromIndex][nextIndex] = 1
                return nextIndex
        return 0

    def isValid(self, state):
        """Check validity of the state. Larger disk must not be on top of the smaller."""
        for i in range(3):
            if len(state[i]):
                minimum = state[i][0]
            for disk in state[i]:
                if disk < minimum:
                    return False
        return True


if __name__ == '__main__':
    # Get user input
    n = int(user_input("Unesite broj kolutova:"))
    gama = float(user_input("Unesite discount faktor:") or 0.95)
    while gama <= 0 or gama > 1:
        user_print("Neispravan unos. Vrijednost mora biti u intervalu <0, 1]", 'warn')
        gama = float(user_input("Unesite discount faktor:  ") or 0.95)

    alpha = float(user_input("Unesite brzinu ucenja:") or 0.5)
    while alpha <= 0 or alpha > 1:
        user_print("Neispravan unos. Vrijednost mora biti u intervalu <0, 1]", 'warn')
        alpha = float(user_input("Unesite brzinu ucenja:") or 0.5)

    # Make new QLearn object
    user_print("Inicijaliziranje algoritma...", 'info')
    alg = QLearn(n, gama, alpha)
    user_print("GOTOVO", 'info')
    print

    # Start learning process
    user_print("Pocinje proces ucenja...", 'info')
    alg.learn()
    user_print("GOTOVO", 'info')
    print

    # Get solution from starting state
    user_print("Unesite pocetno stanje >> ", 'input', False)
    start = input()
    lookup = alg.lookup.copy()  # Get local copy of lookup dictionary
    if type(start) is tuple:  # Allows users to input state both as tuple and index
        if start in lookup:
            start = lookup[start]
        else:
            user_print("Nepostojece stanje! Krecem od pocetnog stanja!", 'warn')
            start = 0
    actions = alg.play(start)

    # Visualize moves by printing in console
    print
    sequence = index2state(lookup, actions, alg.nStates)
    printGame(sequence, n)

    # Get commands for the robot - USED FOR DEBUGGING
    print
    commands = game2robot(sequence)
    print commands
