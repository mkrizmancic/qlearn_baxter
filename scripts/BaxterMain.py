#!/usr/bin/env python
import rospy
import time

from QLearning import QLearn
from BaxterArmClient import BaxterArmClient
from Util import *

class BaxterMain:
    def __init__(self):
        # Create class instances
        self.client = BaxterArmClient()

        # Option to calibrate
        query = user_input("Zelite li provesti kalibraciju? d/n")
        if query == 'd':
            self.calibration()
        
        query = user_input("Test? d/n")
        if query == 'd':
            self.client.test()
        self.startProcedure()

    def calibration (self):
        query = user_input("Kalibracija kolutova? d/n")
        if query == 'd':
            self.client.kalibracija_kolutovi()

        query = user_input("Kalibracija stupova? d/n")
        if query == 'd':
            self.client.kalibracija_stupovi()

    def startProcedure(self):
        # Get user input
        n = int(user_input("Unesite broj kolutova:"))
        gama = float(user_input("Unesite discount faktor:") or 0.95)
        while gama <= 0 or gama > 1:
            user_print("Neispravan unos. Vrijednost mora biti u intervalu <0, 1]", 'warn')
            gama = float(user_input("Unesite discount faktor:") or 0.95)

    # Make new QLearn object
        user_print("Inicijaliziranje algoritma...", 'info')
        alg = QLearn(n, gama)
        user_print("GOTOVO", 'info')
        print

        # Start learning proccess
        user_print("Pocinje proces ucenja...", 'info')
        alg.learn()
        user_print("GOTOVO", 'info')
        print

        # Get solution from starting state
        user_print("Unesite pocetno stanje >> ", 'input', False)
        start = input()
        lookup = alg.lookup.copy()  # Get local copy of lookup dictionary
        if type(start) is tuple:    # Allows users to inpute state both as tuple and index
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

        # Start the robot and send commands step-by-step
        for step in commands:
            self.client.start(step[0], step[1], step[2], step[3])

if __name__ == '__main__':
    time.sleep(35)
    rospy.init_node('BaxterMain')
    try:
        node = BaxterMain()
    except rospy.ROSInterruptException:
        rospy.loginfo('Terminating baxter client and qlearn!')