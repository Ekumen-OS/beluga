#!/usr/bin/env python3

# Copyright 2024 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This script allows conversions from ROS occupancy grid maps to NDT maps."""
import argparse
from pathlib import Path

from beluga_ros.conversion_utils import (
    grid_to_point_cloud,
    OccupancyGrid,
    point_cloud_to_ndt,
    NDTMap,
)

import matplotlib.pyplot as plt


def _parse_args() -> argparse.Namespace:
    arg_parser = argparse.ArgumentParser(usage=__doc__)
    arg_parser.add_argument(
        '--input',
        '-i',
        type=Path,
        help="Path to the map's .yaml file.",
        required=True,
    )
    arg_parser.add_argument(
        '--output_dir',
        '-o',
        type=Path,
        help="Path to the directory where outputs will be dumped to. \
          Will be created if it doesn't exist.",
        required=True,
    )
    arg_parser.add_argument(
        '--cell_size',
        '-c',
        type=float,
        default=1.0,
        help="Cell side length in meters of the final NDT map.",
    )
    return arg_parser.parse_args()


def main():
    """Script entrypoint."""
    args = _parse_args()

    yaml_file: Path = args.input
    out_dir: Path = args.output_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    grid = OccupancyGrid.load_from_file(args.input)
    pc = grid_to_point_cloud(grid)

    print(
        f"Extracted a pointcloud of {pc.shape[1]} points from the occupancy grid... \n"
    )
    ndt = point_cloud_to_ndt(pc, args.cell_size)
    print(
        f"Constructed a NDT representation of the point cloud with {len(ndt.grid)} cells.\n"
    )

    ndt.plot()
    output_plot_name = out_dir / f"{yaml_file.stem}.png"
    plt.savefig(output_plot_name.absolute())
    output_hdf5_name = out_dir / f"{yaml_file.stem}.hdf5"
    ndt.to_hdf5(output_hdf5_name)

    print(f"Saved output artifacts to {out_dir.absolute()}\n\n")
    print(
        f"Loading the map from its serialized form ({output_hdf5_name}) "
        "and verifying its integrity...\n"
    )
    assert ndt.is_close(
        NDTMap.from_hdf5(output_hdf5_name)
    ), "Reading the NDT map from disk produced a different map from the serialized one,\
          this is a bug."

    print("Integrity check OK!")


if __name__ == "__main__":
    main()
