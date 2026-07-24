#!/usr/bin/env python3
"""Extract an exact record-time window into an uncompressed ROS bag.

This is useful for deterministic regression playback of a window taken from a
large compressed bag: all message stamps and record times are preserved, while
chunk decompression can no longer make rosbag play catch up in multi-second
bursts.
"""

import argparse
import os
import sys
from collections import Counter

import rosbag
import rospy


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_bag")
    parser.add_argument("output_bag")
    parser.add_argument("--skip", type=float, required=True,
                        help="seconds after the input bag record-time start")
    parser.add_argument("--duration", type=float, required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.skip < 0.0 or args.duration <= 0.0:
        raise ValueError("--skip must be >= 0 and --duration must be > 0")
    if not os.path.isfile(args.input_bag):
        raise FileNotFoundError(args.input_bag)
    if os.path.exists(args.output_bag):
        raise FileExistsError(args.output_bag)

    output_dir = os.path.dirname(os.path.abspath(args.output_bag))
    os.makedirs(output_dir, exist_ok=True)

    counts = Counter()
    with rosbag.Bag(args.input_bag, "r") as source:
        start_sec = source.get_start_time() + args.skip
        end_sec = start_sec + args.duration
        start = rospy.Time.from_sec(start_sec)
        end = rospy.Time.from_sec(end_sec)

        with rosbag.Bag(args.output_bag, "w", compression=rosbag.Compression.NONE) as target:
            for topic, msg, stamp, connection_header in source.read_messages(
                    start_time=start,
                    end_time=end,
                    return_connection_header=True):
                target.write(topic, msg, stamp, connection_header=connection_header)
                counts[topic] += 1

    total = sum(counts.values())
    if total == 0:
        os.unlink(args.output_bag)
        raise RuntimeError("selected bag window contains no messages")

    print("Extracted {:.3f}s..{:.3f}s ({} messages) to {}".format(
        start_sec, end_sec, total, args.output_bag))
    for topic, count in counts.most_common():
        print("{:8d} {}".format(count, topic))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # Keep command-line failures concise.
        print("[ERROR] {}".format(exc), file=sys.stderr)
        sys.exit(1)
