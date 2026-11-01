"""
Calculate ROS IMU covariance matrices from a stationary rosbag2 recording.
The bag should contain /imu/data and may also contain /imu/mag and /imu/calibration.
"""
import argparse
import math
from pathlib import Path
import sys

import numpy as np
import yaml

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:
    raise SystemExit(
        "ROS 2 Python modules were not found. Run:\n"
        "  source /opt/ros/jazzy/setup.bash\n"
        "and try again."
    ) from exc


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Calculate orientation, gyro, acceleration, and magnetic-field "
            "covariances from a stationary ROS 2 bag."
        )
    )
    parser.add_argument("bag", type=Path, help="Path to the rosbag2 directory")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--mag-topic", default="/imu/mag")
    parser.add_argument("--calibration-topic", default="/imu/calibration")
    parser.add_argument(
        "--skip-start",
        type=float,
        default=0.0,
        metavar="SECONDS",
        help="Ignore this many seconds at the beginning of the bag",
    )
    parser.add_argument(
        "--skip-end",
        type=float,
        default=0.0,
        metavar="SECONDS",
        help="Ignore this many seconds at the end of the bag",
    )
    parser.add_argument(
        "--safety-factor",
        type=float,
        default=1.0,
        help="Multiply measured covariance matrices by this factor (default 1)",
    )
    parser.add_argument(
        "--diagonal-only",
        action="store_true",
        help="Set cross-axis covariance values to zero in recommendations",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("imu_covariance_report.yaml"),
        help="YAML report path (default: imu_covariance_report.yaml)",
    )
    args = parser.parse_args()
    if args.skip_start < 0.0 or args.skip_end < 0.0:
        parser.error("--skip-start and --skip-end must be nonnegative")
    if args.safety_factor <= 0.0:
        parser.error("--safety-factor must be greater than zero")
    return args


def storage_identifier(bag_path):
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"Missing rosbag metadata: {metadata_path}")
    with metadata_path.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    info = metadata.get("rosbag2_bagfile_information", {})
    storage_id = info.get("storage_identifier")
    if not storage_id:
        raise ValueError(f"Cannot find storage_identifier in {metadata_path}")
    return storage_id


def quaternion_to_euler(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1.0e-12:
        raise ValueError("invalid zero or non-finite quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return roll, pitch, yaw


def valid_vector(values):
    return all(math.isfinite(value) for value in values)


def covariance_statistics(samples, labels, units):
    array = np.asarray(samples, dtype=np.float64)
    if array.ndim != 2 or array.shape[0] < 2 or array.shape[1] != 3:
        raise ValueError("at least two three-axis samples are required")
    covariance = np.cov(array, rowvar=False, ddof=1)
    return {
        "axes": labels,
        "units": units,
        "sample_count": int(array.shape[0]),
        "mean": np.mean(array, axis=0).tolist(),
        "standard_deviation": np.std(array, axis=0, ddof=1).tolist(),
        "covariance_matrix": covariance.tolist(),
        "covariance_row_major": covariance.reshape(-1).tolist(),
    }


def adjusted_covariance(statistics, factor, diagonal_only):
    matrix = np.asarray(statistics["covariance_matrix"], dtype=np.float64)
    matrix *= factor
    if diagonal_only:
        matrix = np.diag(np.diag(matrix))
    return {
        "safety_factor": factor,
        "diagonal_only": diagonal_only,
        "matrix": matrix.tolist(),
        "ros_row_major": matrix.reshape(-1).tolist(),
    }


def read_bag(args):
    storage_id = storage_identifier(args.bag)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )

    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    if args.imu_topic not in topic_types:
        available = ", ".join(sorted(topic_types))
        raise ValueError(
            f"IMU topic {args.imu_topic!r} is not in the bag. Available: {available}"
        )

    selected_topics = {
        args.imu_topic,
        args.mag_topic,
        args.calibration_topic,
    }
    message_classes = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
        if topic in selected_topics
    }
    records = []
    first_time_ns = None
    last_time_ns = None

    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if first_time_ns is None:
            first_time_ns = timestamp_ns
        last_time_ns = timestamp_ns
        if topic not in selected_topics:
            continue
        message = deserialize_message(serialized, message_classes[topic])
        records.append((topic, timestamp_ns, message))

    if first_time_ns is None or last_time_ns is None:
        raise ValueError("The bag contains no messages")

    start_ns = first_time_ns + int(args.skip_start * 1.0e9)
    end_ns = last_time_ns - int(args.skip_end * 1.0e9)
    if start_ns >= end_ns:
        raise ValueError("The selected time interval is empty")

    orientation = []
    gyro = []
    acceleration = []
    magnetic = []
    calibration = []
    invalid_imu = 0

    for topic, timestamp_ns, message in records:
        if not start_ns <= timestamp_ns <= end_ns:
            continue
        if topic == args.imu_topic:
            try:
                angles = quaternion_to_euler(
                    message.orientation.x,
                    message.orientation.y,
                    message.orientation.z,
                    message.orientation.w,
                )
            except ValueError:
                invalid_imu += 1
                continue
            gyro_vector = (
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            )
            accel_vector = (
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            )
            if not valid_vector(angles + gyro_vector + accel_vector):
                invalid_imu += 1
                continue
            orientation.append(angles)
            gyro.append(gyro_vector)
            acceleration.append(accel_vector)
        elif topic == args.mag_topic:
            vector = (
                message.magnetic_field.x,
                message.magnetic_field.y,
                message.magnetic_field.z,
            )
            if valid_vector(vector):
                magnetic.append(vector)
        elif topic == args.calibration_topic:
            values = list(message.data)
            if len(values) >= 4:
                calibration.append(values[:4])

    return {
        "storage_identifier": storage_id,
        "duration_seconds": (end_ns - start_ns) / 1.0e9,
        "orientation": orientation,
        "gyro": gyro,
        "acceleration": acceleration,
        "magnetic": magnetic,
        "calibration": calibration,
        "invalid_imu": invalid_imu,
    }


def build_report(args, data):
    orientation = np.asarray(data["orientation"], dtype=np.float64)
    if orientation.shape[0] < 2:
        raise ValueError("Fewer than two valid /imu/data messages were selected")

    # Remove the artificial discontinuity at +/-pi before calculating variance.
    orientation[:, 0] = np.unwrap(orientation[:, 0])
    orientation[:, 2] = np.unwrap(orientation[:, 2])

    measurements = {
        "orientation": covariance_statistics(
            orientation, ["roll", "pitch", "yaw"], "rad and rad^2"
        ),
        "angular_velocity": covariance_statistics(
            data["gyro"], ["x", "y", "z"], "rad/s and (rad/s)^2"
        ),
        "linear_acceleration": covariance_statistics(
            data["acceleration"],
            ["x", "y", "z"],
            "m/s^2 and (m/s^2)^2",
        ),
    }
    if len(data["magnetic"]) >= 2:
        measurements["magnetic_field"] = covariance_statistics(
            data["magnetic"], ["x", "y", "z"], "T and T^2"
        )

    recommendations = {
        name: adjusted_covariance(stats, args.safety_factor, args.diagonal_only)
        for name, stats in measurements.items()
    }

    calibration_summary = {"message_count": len(data["calibration"])}
    if data["calibration"]:
        values = np.asarray(data["calibration"], dtype=np.int64)
        calibration_summary.update(
            {
                "order": ["system", "gyroscope", "accelerometer", "magnetometer"],
                "minimum": np.min(values, axis=0).tolist(),
                "maximum": np.max(values, axis=0).tolist(),
                "all_samples_fully_calibrated": bool(np.all(values == 3)),
            }
        )

    return {
        "source": {
            "bag": str(args.bag.resolve()),
            "storage_identifier": data["storage_identifier"],
            "selected_duration_seconds": data["duration_seconds"],
            "imu_topic": args.imu_topic,
            "mag_topic": args.mag_topic,
            "invalid_imu_messages_skipped": data["invalid_imu"],
            "stationary_data_required": True,
        },
        "calibration": calibration_summary,
        "measured": measurements,
        "recommended": recommendations,
    }


def print_summary(report):
    print("\nIMU covariance results")
    print("=" * 72)
    print(f"Bag: {report['source']['bag']}")
    print(
        "Selected duration: "
        f"{report['source']['selected_duration_seconds']:.2f} seconds"
    )

    for name, stats in report["measured"].items():
        print(f"\n{name} ({stats['units']})")
        print(f"Samples: {stats['sample_count']}")
        print("Mean:", np.array(stats["mean"]))
        print("Standard deviation:", np.array(stats["standard_deviation"]))
        print("Measured covariance matrix:")
        print(np.array(stats["covariance_matrix"]))
        print("Recommended ROS row-major covariance:")
        print(report["recommended"][name]["ros_row_major"])

    calibration = report["calibration"]
    if calibration["message_count"]:
        print("\nCalibration order: system, gyro, accel, mag")
        print("Minimum calibration:", calibration["minimum"])
        print("Fully calibrated for entire selection:",
              calibration["all_samples_fully_calibrated"])
    else:
        print("\nNo calibration messages were found in the selected interval.")


def main():
    args = parse_args()
    try:
        data = read_bag(args)
        report = build_report(args, data)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(report, stream, sort_keys=False)
        print_summary(report)
        print(f"\nYAML report written to: {args.output.resolve()}")
        print(
            "Use stationary recordings. Repeat with electronics/motors active "
            "and prefer the largest representative covariance."
        )
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
