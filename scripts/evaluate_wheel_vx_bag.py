#!/usr/bin/env python3
"""Compare old/new Yangpu EKF outputs against the independent CHC speed."""

import argparse
import bisect
import math
import statistics
import sys
from collections import defaultdict

import rosbag


def percentile(values, probability):
    values = sorted(values)
    position = (len(values) - 1) * probability
    lower = int(position)
    upper = min(lower + 1, len(values) - 1)
    return values[lower] + (values[upper] - values[lower]) * (position - lower)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def interpolate_scalar(series, stamp, max_gap=0.1):
    times, values = series
    index = bisect.bisect_left(times, stamp)
    if index == 0 or index == len(times):
        return None
    t0, t1 = times[index - 1], times[index]
    if t1 <= t0 or t1 - t0 > max_gap:
        return None
    ratio = (stamp - t0) / (t1 - t0)
    return values[index - 1] + ratio * (values[index] - values[index - 1])


def interpolate_pose(series, stamp, max_gap=0.1):
    times, values = series
    index = bisect.bisect_left(times, stamp)
    if index == 0 or index == len(times):
        return None
    t0, t1 = times[index - 1], times[index]
    if t1 <= t0 or t1 - t0 > max_gap:
        return None
    ratio = (stamp - t0) / (t1 - t0)
    x0, y0, yaw0 = values[index - 1]
    x1, y1, yaw1 = values[index]
    return (
        x0 + ratio * (x1 - x0),
        y0 + ratio * (y1 - y0),
        normalize_angle(yaw0 + ratio * normalize_angle(yaw1 - yaw0)),
    )


def error_metrics(errors):
    absolute = [abs(value) for value in errors]
    return {
        "n": len(errors),
        "bias": statistics.fmean(errors),
        "rmse": math.sqrt(statistics.fmean(value * value for value in errors)),
        "mae": statistics.fmean(absolute),
        "p95": percentile(absolute, 0.95),
        "max": max(absolute),
    }


def speed_errors(reference_times, reference_values, source, selector=None):
    errors = []
    for index, (stamp, reference) in enumerate(zip(reference_times, reference_values)):
        if selector is not None and not selector(index, stamp):
            continue
        value = interpolate_scalar(source, stamp)
        if value is not None:
            errors.append(value - reference)
    return error_metrics(errors)


def paired_speed_errors(reference_times, reference_values, old_source, new_source, selector=None):
    old_errors = []
    new_errors = []
    for index, (stamp, reference) in enumerate(zip(reference_times, reference_values)):
        if selector is not None and not selector(index, stamp):
            continue
        old_value = interpolate_scalar(old_source, stamp)
        new_value = interpolate_scalar(new_source, stamp)
        if old_value is None or new_value is None:
            continue
        old_errors.append(old_value - reference)
        new_errors.append(new_value - reference)
    return error_metrics(old_errors), error_metrics(new_errors)


def paired_pose_errors(reference_times, ins_pose, old_pose, new_pose, selector=None):
    old_distances = []
    new_distances = []
    old_yaw_errors_deg = []
    new_yaw_errors_deg = []
    for index, stamp in enumerate(reference_times):
        if selector is not None and not selector(index, stamp):
            continue
        ins_value = interpolate_pose(ins_pose, stamp)
        old_value = interpolate_pose(old_pose, stamp)
        new_value = interpolate_pose(new_pose, stamp)
        if ins_value is None or old_value is None or new_value is None:
            continue
        old_distances.append(math.hypot(old_value[0] - ins_value[0], old_value[1] - ins_value[1]))
        new_distances.append(math.hypot(new_value[0] - ins_value[0], new_value[1] - ins_value[1]))
        old_yaw_errors_deg.append(math.degrees(normalize_angle(old_value[2] - ins_value[2])))
        new_yaw_errors_deg.append(math.degrees(normalize_angle(new_value[2] - ins_value[2])))
    return (
        error_metrics(old_distances), error_metrics(new_distances),
        error_metrics(old_yaw_errors_deg), error_metrics(new_yaw_errors_deg),
    )


def find_gap_exclusions(times, threshold, recovery):
    return [
        (previous, current + recovery)
        for previous, current in zip(times, times[1:])
        if current - previous > threshold
    ]


def is_excluded(stamp, intervals):
    return any(begin <= stamp <= end for begin, end in intervals)


def timing_metrics(samples, begin, end):
    selected = [(record, stamp) for record, stamp in samples if begin <= stamp <= end]
    records = [value[0] for value in selected]
    stamps = [value[1] for value in selected]
    gaps = [current - previous for previous, current in zip(stamps, stamps[1:])]
    positive_gaps = [value for value in gaps if value > 0.0]
    delays = [record - stamp for record, stamp in selected]
    return {
        "n": len(selected),
        "rate": (len(stamps) - 1) / (stamps[-1] - stamps[0]),
        "gap_p95": percentile(positive_gaps, 0.95),
        "gap_max": max(positive_gaps),
        "regressions": sum(value < 0.0 for value in gaps),
        "delay_mean": statistics.fmean(delays),
        "delay_p95": percentile(delays, 0.95),
        "delay_max": max(delays),
    }


def print_error_row(label, metrics):
    print(
        f"{label:16s} n={metrics['n']:5d} RMSE={metrics['rmse']:.9f} "
        f"MAE={metrics['mae']:.9f} bias={metrics['bias']:+.9f} "
        f"p95={metrics['p95']:.9f} max={metrics['max']:.9f}"
    )


def improvement(old, new, key):
    return 100.0 * (old[key] - new[key]) / old[key]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="Result bag containing old and new EKF topics")
    parser.add_argument("--trim-start", type=float, default=5.0, help="Cold-start seconds to exclude")
    parser.add_argument(
        "--ignore-output-gaps", action="store_true",
        help="exclude new-output gaps and their recovery transients from accuracy metrics")
    parser.add_argument("--gap-threshold", type=float, default=0.1)
    parser.add_argument("--gap-recovery", type=float, default=0.5)
    parser.add_argument(
        "--new-time-offset", type=float, default=0.0,
        help="diagnostic-only offset added to new output header stamps")
    parser.add_argument("--show-worst", type=int, default=0)
    parser.add_argument("--strict", action="store_true", help="Return non-zero unless acceptance checks pass")
    args = parser.parse_args()

    scalar = defaultdict(lambda: [[], []])
    pose = defaultdict(lambda: [[], []])
    timing = defaultdict(list)
    covariance = defaultdict(list)
    topic_names = {
        "/localization/final_odom": "old",
        "/localization/final_odom1": "new",
        "/wheel_odom": "wheel",
        "/lio_loc_result": "lio",
        "/localization/ins": "ins",
    }

    with rosbag.Bag(args.bag) as bag:
        for topic, message, bag_stamp in bag.read_messages(
            topics=list(topic_names) + ["/chcnav/devpvt"]
        ):
            if topic == "/chcnav/devpvt":
                stamp = message.header.stamp.to_sec()
                scalar["chc"][0].append(stamp)
                scalar["chc"][1].append(float(message.speed))
                continue
            name = topic_names[topic]
            stamp = message.header.stamp.to_sec()
            if name == "new":
                stamp += args.new_time_offset
            scalar[name][0].append(stamp)
            scalar[name][1].append(message.twist.twist.linear.x)
            orientation = message.pose.pose.orientation
            pose[name][0].append(stamp)
            pose[name][1].append(
                (message.pose.pose.position.x, message.pose.pose.position.y,
                 yaw_from_quaternion(orientation))
            )
            if name in ("old", "new"):
                timing[name].append((bag_stamp.to_sec(), stamp))
                covariance[name].append(message.twist.covariance[0])

    required = ("chc", "old", "new", "wheel", "lio", "ins")
    missing = [name for name in required if not scalar[name][0]]
    if missing:
        print("Missing required series: " + ", ".join(missing), file=sys.stderr)
        return 2

    # rosbag record order can contain small bursts; all comparisons use header time.
    for name in scalar:
        ordered = sorted(zip(*scalar[name]))
        scalar[name] = [[item[0] for item in ordered], [item[1] for item in ordered]]
    for name in pose:
        ordered = sorted(zip(*pose[name]))
        pose[name] = [[item[0] for item in ordered], [item[1] for item in ordered]]

    raw_start = scalar["chc"][0][0]
    begin = max(
        raw_start + args.trim_start,
        scalar["old"][0][0], scalar["new"][0][0], scalar["ins"][0][0],
    )
    end = min(
        scalar["chc"][0][-1], scalar["old"][0][-1],
        scalar["new"][0][-1], scalar["ins"][0][-1],
    )
    indices = [index for index, stamp in enumerate(scalar["chc"][0]) if begin <= stamp <= end]
    reference_times = [scalar["chc"][0][index] for index in indices]
    reference_values = [scalar["chc"][1][index] for index in indices]

    excluded_intervals = []
    if args.ignore_output_gaps:
        excluded_intervals = find_gap_exclusions(
            scalar["new"][0], args.gap_threshold, args.gap_recovery)
    valid_selector = lambda _index, stamp: not is_excluded(stamp, excluded_intervals)

    # Dynamic points use a 0.5 s centered derivative of CHC speed.
    dynamic_indices = set()
    for local_index, stamp in enumerate(reference_times):
        before = interpolate_scalar(scalar["chc"], stamp - 0.25)
        after = interpolate_scalar(scalar["chc"], stamp + 0.25)
        if before is not None and after is not None and abs((after - before) / 0.5) > 0.08:
            dynamic_indices.add(local_index)
    dynamic_selector = lambda index, stamp: (
        index in dynamic_indices and valid_selector(index, stamp))

    old_speed, new_speed = paired_speed_errors(
        reference_times, reference_values, scalar["old"], scalar["new"], valid_selector)
    wheel_speed = speed_errors(
        reference_times, reference_values, scalar["wheel"], valid_selector)
    lio_speed = speed_errors(
        reference_times, reference_values, scalar["lio"], valid_selector)
    old_dynamic, new_dynamic = paired_speed_errors(
        reference_times, reference_values, scalar["old"], scalar["new"], dynamic_selector)
    wheel_dynamic = speed_errors(
        reference_times, reference_values, scalar["wheel"], dynamic_selector)
    lio_dynamic = speed_errors(
        reference_times, reference_values, scalar["lio"], dynamic_selector)

    old_position, new_position, old_yaw, new_yaw = paired_pose_errors(
        reference_times, pose["ins"], pose["old"], pose["new"], valid_selector)
    old_timing = timing_metrics(timing["old"], begin, end)
    new_timing = timing_metrics(timing["new"], begin, end)

    print(f"evaluation header window: {begin:.6f} .. {end:.6f} ({end - begin:.3f} s)")
    if args.ignore_output_gaps:
        excluded_seconds = sum(end_time - begin_time for begin_time, end_time in excluded_intervals)
        print(
            f"accuracy mask: excluded {len(excluded_intervals)} output gaps plus "
            f"{args.gap_recovery:.3f}s recovery ({excluded_seconds:.3f}s total interval)")
    print(f"paired accuracy samples: old/new n={old_speed['n']}")
    print("\nforward speed vs /chcnav/devpvt.speed [m/s]")
    print_error_row("old final", old_speed)
    print_error_row("new final", new_speed)
    print_error_row("wheel", wheel_speed)
    print_error_row("LIO", lio_speed)
    print(
        "new improvement: "
        f"RMSE={improvement(old_speed, new_speed, 'rmse'):+.3f}% "
        f"MAE={improvement(old_speed, new_speed, 'mae'):+.3f}% "
        f"|bias|={100.0 * (abs(old_speed['bias']) - abs(new_speed['bias'])) / abs(old_speed['bias']):+.3f}%"
    )

    print("\ndynamic forward speed vs CHC [m/s]")
    print_error_row("old dynamic", old_dynamic)
    print_error_row("new dynamic", new_dynamic)
    print_error_row("wheel dynamic", wheel_dynamic)
    print_error_row("LIO dynamic", lio_dynamic)
    if args.show_worst > 0:
        worst = []
        for index, (stamp, reference) in enumerate(zip(reference_times, reference_values)):
            if not dynamic_selector(index, stamp):
                continue
            old_value = interpolate_scalar(scalar["old"], stamp)
            new_value = interpolate_scalar(scalar["new"], stamp)
            wheel_value = interpolate_scalar(scalar["wheel"], stamp)
            lio_value = interpolate_scalar(scalar["lio"], stamp)
            if None in (old_value, new_value, wheel_value, lio_value):
                continue
            old_error = old_value - reference
            new_error = new_value - reference
            worst.append((abs(new_error) - abs(old_error), stamp, reference,
                          old_value, new_value, wheel_value, lio_value))
        print("\nworst dynamic regressions [m/s]")
        for delta, stamp, reference, old_value, new_value, wheel_value, lio_value in sorted(
                worst, reverse=True)[:args.show_worst]:
            print(
                f"stamp={stamp:.6f} abs_error_delta={delta:+.9f} ref={reference:.9f} "
                f"old={old_value:.9f} new={new_value:.9f} "
                f"wheel={wheel_value:.9f} lio={lio_value:.9f}"
            )

    print("\nplanar pose vs /localization/ins [m and deg]")
    print_error_row("old position", old_position)
    print_error_row("new position", new_position)
    print_error_row("old yaw deg", old_yaw)
    print_error_row("new yaw deg", new_yaw)

    print("\noutput timing [s]")
    for label, values in (("old", old_timing), ("new", new_timing)):
        print(
            f"{label:4s} n={values['n']} rate={values['rate']:.3f}Hz "
            f"gap_p95/max={values['gap_p95']:.6f}/{values['gap_max']:.6f} "
            f"regressions={values['regressions']} "
            f"record-header mean/p95/max={values['delay_mean']:.6f}/"
            f"{values['delay_p95']:.6f}/{values['delay_max']:.6f}"
        )

    print("\nvx covariance")
    for label in ("old", "new"):
        values = covariance[label]
        print(
            f"{label:4s} min/median/p95/max={min(values):.9f}/"
            f"{percentile(values, 0.5):.9f}/{percentile(values, 0.95):.9f}/"
            f"{max(values):.9f}"
        )

    checks = {
        "speed RMSE improved": new_speed["rmse"] < old_speed["rmse"],
        "speed MAE improved": new_speed["mae"] < old_speed["mae"],
        "speed |bias| improved": abs(new_speed["bias"]) < abs(old_speed["bias"]),
        "dynamic RMSE non-regression": new_dynamic["rmse"] <= old_dynamic["rmse"],
        "dynamic MAE non-regression": new_dynamic["mae"] <= old_dynamic["mae"],
        "dynamic p95 non-regression": new_dynamic["p95"] <= old_dynamic["p95"],
        "position RMSE within 2%": new_position["rmse"] <= 1.02 * old_position["rmse"],
        "yaw RMSE within 2%": new_yaw["rmse"] <= 1.02 * old_yaw["rmse"],
        "no timestamp regression": new_timing["regressions"] == 0,
    }
    if args.ignore_output_gaps:
        checks["output gaps excluded by request"] = True
    else:
        checks["max output gap below 0.1s"] = new_timing["gap_max"] < 0.1
    print("\nacceptance")
    for label, passed in checks.items():
        print(f"[{'PASS' if passed else 'FAIL'}] {label}")
    passed = all(checks.values())
    print("OVERALL:", "PASS" if passed else "FAIL")
    return 0 if passed or not args.strict else 1


if __name__ == "__main__":
    sys.exit(main())
