#!/usr/bin/env python3
"""Compare waypoint missions flown with different position controllers.

Reads rosbag2 recordings of /fmu/out/vehicle_local_position_v1 and
/fmu/out/vehicle_status_v1, keeps the part of each flight spent in OFFBOARD
mode, prints tracking metrics and saves a plot.

    python3 compare_flights.py --mission square_mission.yaml \
        --flight px4=~/flights/px4 --flight own=~/flights/own --plot comparison.png
"""

import argparse
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rosbag2_py  # noqa: E402
import yaml  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

POSITION_TOPIC = '/fmu/out/vehicle_local_position_v1'
STATUS_TOPIC = '/fmu/out/vehicle_status_v1'
NAV_STATE_OFFBOARD = 14


@dataclass
class Flight:
    name: str
    t: np.ndarray  # s since offboard start
    north: np.ndarray  # m, relative to home
    east: np.ndarray
    altitude: np.ndarray  # m above home (positive up)


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path)), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    positions, statuses = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == POSITION_TOPIC:
            m = deserialize_message(data, types[topic])
            positions.append((m.timestamp, m.x, m.y, m.z))
        elif topic == STATUS_TOPIC:
            m = deserialize_message(data, types[topic])
            statuses.append((m.timestamp, m.nav_state))
    return np.array(positions), np.array(statuses)


def load_flight(name, path):
    positions, statuses = read_bag(Path(path).expanduser())
    # First continuous OFFBOARD period: from entering offboard until the first
    # status in another mode (the land command switches PX4 to AUTO_LAND).
    is_offboard = statuses[:, 1] == NAV_STATE_OFFBOARD
    if not is_offboard.any():
        raise SystemExit(f'{name}: no OFFBOARD period in {path}')
    first = int(np.argmax(is_offboard))
    after = np.flatnonzero(~is_offboard[first:])
    start = statuses[first, 0]
    end = statuses[first + after[0], 0] if after.size else statuses[-1, 0]

    home = positions[positions[:, 0] <= start][-1, 1:]  # last sample before take-off
    flying = positions[(positions[:, 0] >= start) & (positions[:, 0] <= end)]
    rel = flying[:, 1:] - home
    return Flight(name=name, t=(flying[:, 0] - start) * 1e-6,
                  north=rel[:, 0], east=rel[:, 1], altitude=-rel[:, 2])


def load_waypoints(mission_file):
    params = yaml.safe_load(Path(mission_file).read_text())['mission_node']['ros__parameters']
    flat = params['waypoints']
    return params['takeoff_altitude'], np.array(flat).reshape(-1, 3)


def metrics(flight, takeoff_altitude, waypoints):
    # Altitude tracking only after the vehicle first reached the cruise altitude.
    cruise = np.argmax(flight.altitude >= takeoff_altitude - 0.1)
    alt_err = flight.altitude[cruise:] - waypoints[:, 2].mean()

    # Horizontal path error: distance from the route (home -> waypoints) seen from above.
    # Covers both corner overshoot and sideways drift along a leg.
    xy = np.column_stack([flight.north, flight.east])
    route = np.vstack([[0.0, 0.0], waypoints[:, :2]])
    path_err = np.min([distance_to_segment(xy, a, b) for a, b in zip(route[:-1], route[1:])],
                      axis=0)
    return {
        'duration_s': flight.t[-1],
        'alt_rms_m': float(np.sqrt(np.mean(alt_err ** 2))),
        'alt_max_m': float(np.abs(alt_err).max()),
        'path_rms_m': float(np.sqrt(np.mean(path_err ** 2))),
        'path_max_m': float(path_err.max()),
    }


def distance_to_segment(points, a, b):
    ab = b - a
    t = np.clip((points - a) @ ab / max(ab @ ab, 1e-12), 0.0, 1.0)
    return np.linalg.norm(points - (a + t[:, None] * ab), axis=1)


def plot(flights, waypoints, out):
    fig, (ax_xy, ax_alt) = plt.subplots(1, 2, figsize=(12, 5))
    route = np.vstack([[0, 0, 0], waypoints])
    ax_xy.plot(route[:, 1], route[:, 0], 'k--', lw=1, label='route')
    for f in flights:
        ax_xy.plot(f.east, f.north, label=f.name)
        ax_alt.plot(f.t, f.altitude, label=f.name)
    ax_xy.set(xlabel='east [m]', ylabel='north [m]', title='Path seen from above')
    ax_xy.set_aspect('equal')
    ax_alt.set(xlabel='time in offboard [s]', ylabel='altitude [m]', title='Altitude')
    for ax in (ax_xy, ax_alt):
        ax.grid(True, alpha=0.3)
        ax.legend()
    fig.tight_layout()
    fig.savefig(out, dpi=120)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--mission', required=True, help='mission YAML used for the flights')
    parser.add_argument('--flight', action='append', required=True, metavar='NAME=BAG_DIR')
    parser.add_argument('--plot', default='comparison.png')
    args = parser.parse_args()

    takeoff_altitude, waypoints = load_waypoints(args.mission)
    flights = [load_flight(*spec.split('=', 1)) for spec in args.flight]

    print(f"{'controller':<12}{'time [s]':>10}{'alt RMS [m]':>13}{'alt max [m]':>13}"
          f"{'path RMS [m]':>14}{'path max [m]':>14}")
    for f in flights:
        m = metrics(f, takeoff_altitude, waypoints)
        print(f"{f.name:<12}{m['duration_s']:>10.1f}{m['alt_rms_m']:>13.2f}"
              f"{m['alt_max_m']:>13.2f}{m['path_rms_m']:>14.2f}{m['path_max_m']:>14.2f}")

    plot(flights, waypoints, args.plot)
    print(f'plot saved to {args.plot}')


if __name__ == '__main__':
    main()
