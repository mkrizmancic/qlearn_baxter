from QLearning import QLearn
from BaxterArmClient import BaxterArmClient
from Util import *

class topLevel:
    def __init__(self):
        # Create class instances
        self.alg = QLearn()
        self.client = BaxterArmClient()

        # Option to calibrate
        query = raw_input('Zelite li provesti kalibraciju? d/n ---> ')
        if query == 'd':
            self.calibration()

    def calibration (self):
        query = raw_input('Kalibracija kolutova? d/n ---> ')
        if query == 'd':
            self.client.kalibracija_kolutovi()

        query = raw_input('Kalibracija stupova? d/n ---> ')
        if query == 'd':
        self.client.kalibracija_stupovi()

    def startProcedure(self)
        # Get user input
        n = input ("Unesite broj kolutova:  ")
        gama = float (raw_input ("Unesite discount faktor:  ") or 0.95)
        while gama <= 0 and gama > 1:
            print ("Neispravan unos. Vrijednost mora biti u intervalu <0, 1]")
            gama = float (raw_input ("Unesite discount faktor:  ") or 0.95)

        # Make new QLearn object
        print "Inicijaliziranje algoritma..."
        self.alg = QLearn(n, gama)
        print "GOTOVO"
        print

        # Start learning proccess
        print "Pocinje proces ucenja..."
        self.alg.learn()
        print "GOTOVO"
        print

        # Get solution from starting state
        start = input ("Unesite pocetno stanje:  ")
        lookup = self.alg.lookup.copy()  # Get local copy of lookup dictionary
        if type(start) is tuple:    # Allows users to inpute state both as tuple and index
            if start in lookup:
                start = lookup[start]
            else:
                print "Nepostojece stanje! Krecem od pocetnog stanja!"
                start = 0
        actions = self.alg.play(start)

        # Visualize moves by printing in console
        print
        sequence = index2state(lookup, actions, self.alg.nStates)
        printGame(sequence)

        # Get commands for the robot - USED FOR DEBUGGING
        print
        commands = game2robot(sequence)
        print commands

        # Start the robot and send commands step-by-step
        for step in commands:
            self.client.start(step[0], step[1], step[2], step[3])