#!/usr/bin/env python3


import argparse, time
from functools import partial
from pilot import Pilot
import multiprocessing


if __name__ == '__main__':
    # get mission parameters from user setup
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", required=True,
                        help="name of mission file formatted as explained in user_help.txt")
    # extract command line arguments
    cli_args = parser.parse_args()

    ti = time.time()
    crazypilot = Pilot(cli_args.input).skills_test() # warning: crazypilot = None

    tf = time.time()
    print(f"Done. Total mission time was {int(tf-ti)} s.")


# EOF