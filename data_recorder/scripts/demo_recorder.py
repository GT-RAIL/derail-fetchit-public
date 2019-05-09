#!/usr/bin/env python

import argparse

import rospy

from data_recorder.recorder import StateRecorder

def main():

    epilog = """
Related examples:
  joint_position_file_playback.py; joint_trajectory_file_playback.py.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f', '--file', dest='filename', required=True,
        help='the file name to record to'
    )
    parser.add_argument(
        '-r', '--record-rate', type=int, default=100, metavar='RECORDRATE',
        help='rate at which to record (default: 100)'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("demo_recorder")
    print("Getting robot state... ")

    recorder = StateRecorder(args.filename, args.record_rate)
    rospy.on_shutdown(recorder.stop)

    print("Recording. Press Ctrl-C to stop.")
    recorder.record()

    print("\nDone.")

if __name__ == '__main__':
    main()

