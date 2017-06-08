#!/usr/bin/env python
import time

from BaxterArmClient import BaxterArmClient
from Util import *


# Comments beginning with "noinspection" are PyCharm auto-generated comments

class BaxterTest:
    """Class for running tests on Baxter without having to start other scripts."""
    def __init__(self):
        # Create class instances
        self.client = BaxterArmClient()

        prompt = 'p'
        while prompt == 'p':
            user_print("Unesite broj testa koji zelite izvesti:", 'input')
            user_print("1 - test apsolutnih pozicija", 'input')
            user_print("2 - test relativnih pozicija", 'input')
            user_print("Bilo koja druga tipka - kraj", 'input')
            prompt = user_input("")

            if prompt == '1':
                self.client.test_absolute()
                prompt = user_input("Unesite 'p' za promjenu ili 'n' za zavrsetak")
            elif prompt == '2':
                self.client.test_relative()
                prompt = user_input("Unesite 'p' za promjenu ili 'n' za zavrsetak")

if __name__ == '__main__':
    time.sleep(5)
    rospy.init_node('BaxterTesting')
    try:
        node = BaxterTest()
    except rospy.ROSInterruptException:
        rospy.loginfo('Terminating baxter client and test!')
