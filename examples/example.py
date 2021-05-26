import random
from argparse import ArgumentParser

import numpy as np


from frankpiv import Controller

if __name__ == '__main__':
    arg_parser = ArgumentParser()
    arg_parser.add_argument("--config", "-c", type=str, default=None)
    args = arg_parser.parse_args()

    # move the roboter to the initial position before starting the controller
    with Controller(args.config) as controller:
        # move to random points using pitch, yaw, roll, z-translation values
        for _ in range(10):
            pitch = (random.random() - 0.5) * 0.7 * np.pi
            yaw = (random.random() - 0.5) * 0.7 * np.pi
            roll = (random.random() - 0.5) * 0.8 * np.pi
            z_translation = (random.random() - 0.5) * 0.05
            controller.move_pyrz([pitch, yaw, roll, z_translation])
        # TODO part below is broken. Needs to be fixed in the binding
        # move to some point in the pivot point frame (reference frame of the controller)
        controller.move_to_point([0.05, -0.05, 0.1], roll=np.pi)
        # move to some point in the global frame
        controller.move_to_point([0.2, 0.1, 0.15], frame=[0, 0, 0, 0, 0, 0], roll=np.pi)
