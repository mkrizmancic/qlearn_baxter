#!/usr/bin/env python
import rospy
import time

from QLearning import QLearn
from BaxterArmClient import BaxterArmClient
from Util import *


# Comments beginning with "noinspection" are PyCharm auto-generated comments

class BaxterMain:
    """
    A top-level class for calling both Q-Learning algorithm (class QLearn)
    and controls for the robot (class BaxterArmClient).

    Attributes:
        client: Instance of the BaxterArmClient class
    """
    def __init__(self):
        """Start up arm client, ask for calibration and start the whole playing procedure."""
        # Create class instances
        self.client = BaxterArmClient()

        # Option to calibrate
        query = user_input("Zelite li provesti kalibraciju? d/n")
        if query == 'd':
            self.client.calibration()

        self.startProcedure()
        while user_input("Zelite li ponoviti igru? d/n") == 'd':
            self.startProcedure()

    def startProcedure(self):
        """
        Get user input for Q-Learning hyper-parameters, create a class instance,
        start learning process, find the solution and run that solution on the robot.
        """
        # Get user input
        n = int(user_input("Unesite broj kolutova:"))
        gama = float(user_input("Unesite discount faktor <0, 1]:") or 0.95)
        while gama <= 0 or gama > 1:
            user_print("Neispravan unos. Vrijednost mora biti u intervalu <0, 1]", 'warn')
            gama = float(user_input("Unesite discount faktor:") or 0.95)

        alpha = float(user_input("Unesite brzinu ucenja <0, 1]:") or 0.5)
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

        # Get commands for the robot
        print
        commands = game2robot(sequence)

        # Start the robot and send commands step-by-step
        for step in commands:
            print self.client.start(step[0], step[1], step[2], step[3])


if __name__ == '__main__':
    time.sleep(5)
    rospy.init_node('BaxterMain')
    try:
        node = BaxterMain()
    except rospy.ROSInterruptException:
        rospy.loginfo('Terminating baxter client and qlearn!')
