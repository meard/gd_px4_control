#!/usr/bin/env python3

import sys
import os
import math
import time
import subprocess
import asyncio

import cv2
import numpy as np

from flymambo import *


class parrotControl:

    def __init__(self):
        # super().__init__('parrot_control_node')

        # Initialize the FlyMambo object
        mamboAddr = "D0:3A:96:D0:E6:3B"
        self.fly_parrot = FlyMambo(mamboAddr)

        # Print the battery level
        print('Current Battery Level: ', self.fly_parrot.get_battery())

        self.results = []
        self.command_seq = True
        self.gesture_result = None

        self.gesture_old = None
        self.gesture = None

        self.timer_period = 10     # seconds
        self.counter = 0       # counter

    def gesture_callback(self):
        process = subprocess.Popen(
            ['python3', 'test_user_in.py'], stdout=subprocess.PIPE)
        while process.poll() is None:
            result = process.stdout.readline().decode()
            self.gesture_result = result.split()

            self.gesture = self.gesture_result

    def mission_callback(self):

        if self.gesture is not None:

            # Current gesture result
            if self.gesture == ['GO'] and self.gesture_old != self.gesture:
                match self.gesture:
                    case ['UP']:
                        self.fly_parrot.move_mambo(move="UP", amount=50)
                    case ['DOWN']:
                        self.fly_parrot.move_mambo(move="DWN", amount=50)
                    case ['LEFT']:
                        self.fly_parrot.move_mambo(move="LFT", amount=50)
                    case ['RIGHT']:
                        self.fly_parrot.move_mambo(move="RGT", amount=50)
                    case ['BOX']:
                        self.fly_parrot.move_mambo(move="BOX", amount=50)
                    case ['HOLD']:
                        self.fly_parrot.move_mambo(move="STOP", amount=50)
                    case _:
                        # IDEAL STATE - Hover
                        self.fly_parrot.move_mambo(move="STOP", amount=50)

                self.gesture_old = self.gesture

            else:
                self.fly_parrot.move_mambo(move="STOP", amount=50)


            
    def command_seq_callback(self):

        input_command = input('Ready for Mission (Y/N): ')

        if input_command == 'Y':
            command_seq = True
        else:
            command_seq = False

        return command_seq

    def parrot_main(self):
        try:

            command_seq = self.command_seq_callback()

            if command_seq is True:

                self.fly_parrot.takeoff(self.timer_period)

                self.mission_callback()

                exit(0)

            else:
                self.counter += 1
                if self.counter == 100:
                    self.counter = 0
                    print('Current Battery Level: ',
                          self.fly_parrot.get_battery())

        except KeyboardInterrupt:
            exit(0)

        except SystemExit:
            self.fly_parrot.emergency()


def main(args=None) -> None:
    pC = parrotControl()
    pC.parrot_main()

    return 0


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
