#!/usr/bin/env python

import sys
import copy
import rospy
import argparse

from panda_interface.arm import PandaArmInterface


if __name__ == '__main__':
    parser = argparse.ArgumentParser("Stop current gripper action; Open or close gripper.")

    parser.add_argument("--width", type=float, default=None, help="gripper width")
    parser.add_argument("--force", type=float, default=None, help="gripper force")
    parser.add_argument("-o", "--open", dest="open",
        action='store_true', default=False, help="open gripper")
    parser.add_argument("-c", "--close", dest="close",
        action='store_true', default=False, help="close gripper")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('panda_client')
    panda = PandaArmInterface()

    if args.width:
        panda.exec_gripper_cmd(args.width, args.force)
    else:
        if args.open:
            panda.gripper.open()
        if args.close:
            panda.gripper.close()

