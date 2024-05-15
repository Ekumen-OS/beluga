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

"""This script allows conversions from PLY files to NDT maps."""

from argparse import ArgumentParser
from pathlib import Path

from plyfile import PlyData
import numpy as np
import matplotlib.pyplot as plt

from beluga_ros.conversion_utils import point_cloud_to_ndt, NDTMap


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        '-i', '--input', type=Path, help='PLY file path.', required=True
    )
    parser.add_argument(
        '-o',
        '--output_dir',
        type=Path,
        help="Path to the directory where outputs will be dumped to. "
        "Will be created if it doesn't exist.",
        required=True,
    )
    parser.add_argument(
        '-c',
        '--cell_size',
        type=float,
        default=0.1,
        help="Cell side length in meters of the final NDT map.",
    )

    args = parser.parse_args()

    data = PlyData.read(args.input)

    # Assumes that the PLY file contains a 2D point cloud.
    point_cloud = np.vstack([data['vertex']['x'], data['vertex']['y']])

    print(f"Extracted a pointcloud of {point_cloud.shape[1]} points from PLY file...")

    ndt = point_cloud_to_ndt(point_cloud, cell_size=args.cell_size)

    print(
        f"Constructed a NDT representation of the point cloud with {len(ndt.grid)} cells."
    )

    ndt.plot()

    plt.savefig(args.output_dir / f"{args.input.stem}.png")

    hdf5_path = args.output_dir / f"{args.input.stem}.hdf5"
    ndt.to_hdf5(hdf5_path)

    print(f"Saved output artifacts to `{args.output_dir}`")

    print(
        "Loading the map from its serialized form (.hdf5) and verifying its integrity..."
    )
    assert ndt.is_close(
        NDTMap.from_hdf5(hdf5_path)
    ), "Reading the NDT map from disk produced a different map from the serialized one."

    print("Integrity check OK!")


if __name__ == "__main__":
    main()
