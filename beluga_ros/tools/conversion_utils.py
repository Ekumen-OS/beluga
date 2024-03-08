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

import numpy as np
import h5py
import yaml
from typing import Dict, Optional
from pathlib import Path
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal

from dataclasses import dataclass


@dataclass(frozen=True)
class DiscreteCell:
    x: int
    y: int


@dataclass(frozen=True)
class NormalDistribution:
    """
    Wrapper around scipy's multivariate normal distribution implementation.

    Allows for equality checks and typing hints.
    """

    mean: np.ndarray
    covariance: np.ndarray

    def get_scipy_representation(self):
        """Get its scipy representation."""
        return multivariate_normal(mean=self.mean, cov=self.covariance)

    def __eq__(self, other: "NormalDistribution") -> bool:
        """Element-wise equality."""
        return np.allclose(self.mean, other.mean) and np.allclose(
            self.covariance, other.covariance
        )


class NDTMap:
    def __init__(self, resolution: float) -> None:
        """Create a new NDT map with cell size of 'resolution'."""
        self._resolution = resolution
        self._grid: Dict[DiscreteCell, NormalDistribution] = {}

    def add_distribution(self, cell: DiscreteCell, ndt: NormalDistribution):
        """Add a new cell with its distribution."""
        self._grid[cell] = ndt

    def _get_plot_bounds(self):
        min_x = float("inf")
        min_y = float("inf")

        max_x = -float("inf")
        max_y = -float("inf")
        for discrete_cell in self._grid.keys():
            x = discrete_cell.x * self._resolution + self._resolution / 2
            y = discrete_cell.y * self._resolution + self._resolution / 2
            min_x = min(min_x, x)
            max_x = max(max_x, x)

            min_y = min(min_y, y)
            max_y = max(max_y, y)

        PADDING = 1  # [m]

        min_x -= PADDING
        max_x += PADDING
        min_y -= PADDING
        max_y += PADDING

        step_x = (max_x - min_x) / 100.0
        step_y = (max_y - min_y) / 100.0

        xx, yy = np.mgrid[min_x:max_x:step_x, min_y:max_y:step_y]
        return (xx, yy, np.dstack((xx, yy)))

    def __eq__(self, other: "NDTMap") -> bool:
        """Equality for two NDT maps."""
        return (
            np.allclose(self._resolution, other._resolution)
            and self._grid == other._grid
        )

    def plot(self, show: bool = False) -> plt.figure:
        """
        Plot the map using contour plots into the current figure and returns it.

        Optionally show the plot based on the 'show' parameter.
        """
        xx, yy, pos = self._get_plot_bounds()

        for ndt in self._grid.values():
            plt.contour(
                xx,
                yy,
                ndt.get_scipy_representation().pdf(pos),
                [0.1, 0.5],
                alpha=0.2,
                linewidths=3,
                extend="max",
            )
        if show:
            plt.show()
        return plt.gcf()

    def to_hdf5(self, output_file_path: Path):
        """
        Serialize the NDT map into an HDF5 format.

        See https://docs.hdfgroup.org/hdf5/develop/_intro_h_d_f5.html for details on the format.
        It'll create 4 datasets:
            - "resolution": () resolution for the discrete grid (cells are resolution x
              resolution m^2 squares).
            - "discrete_cells": (NUM_CELLS, 2) that contains the discrete cell coordinates.
            - "means": (NUM_CELLS, 2) that contains the 2d mean of the normal distribution
              of the cell.
            - "covariances": (NUM_CELLS, 2, 2) Contains the covariance for each cell.
        """
        assert output_file_path.suffix in (".hdf5", ".h5")
        output_file_path.parent.mkdir(parents=True, exist_ok=True)

        num_cells = len(self._grid.keys())

        with h5py.File(output_file_path.absolute(), "w") as fp:
            discrete_cells_dataset = fp.create_dataset(
                "discrete_cells", (num_cells, 2), chunks=True
            )
            for idx, cell in enumerate(self._grid.keys()):
                discrete_cells_dataset[idx] = np.asarray([cell.x, cell.y])

            means_dataset = fp.create_dataset("means", (num_cells, 2), chunks=True)
            covariances_dataset = fp.create_dataset("covariances", (num_cells, 2, 2))

            for idx, distribution in enumerate(self._grid.values()):
                means_dataset[idx] = distribution.mean
                covariances_dataset[idx] = distribution.covariance
            fp.create_dataset("resolution", data=np.asarray(self._resolution))

    @classmethod
    def from_hdf5(cls, hdf5_file: Path):
        """
        Create an NDTMap instance from a path to a hdf5 file.

        See 'serialize_to' docstring for details on the hdf5 format.
        """
        with h5py.File(hdf5_file.absolute(), "r") as fp:
            resolution: float = fp["resolution"][()]
            means: np.ndarray = fp["means"]
            discrete_cells: np.ndarray = fp["discrete_cells"]
            covs: np.ndarray = fp["covariances"]
            total_cells = covs.shape[0]

            assert covs.shape[1] == 2
            assert covs.shape[2] == 2
            assert means.shape[1] == 2
            assert discrete_cells.shape[0] == total_cells
            assert discrete_cells.shape[1] == 2
            assert means.shape[0] == total_cells

            ret = NDTMap(resolution=resolution)
            for cell_num in range(total_cells):
                cell = DiscreteCell(*discrete_cells[cell_num].tolist())
                mean = means[cell_num]
                cov = covs[cell_num]
                ret.add_distribution(
                    cell=cell, ndt=NormalDistribution(mean=mean, covariance=cov)
                )
            return ret


@dataclass
class OccupancyGrid:
    resolution: float
    origin: np.ndarray
    grid: np.ndarray

    @staticmethod
    def load_from_file(yaml_path: Path) -> "OccupancyGrid":
        """Create an grid from a yaml file describing a ROS style occupancy grid."""
        assert yaml_path.is_file(), f"file {yaml_path} doesn't exist."
        assert yaml_path.suffix == ".yaml"

        with open(yaml_path, "rb") as fp:
            data = yaml.safe_load(fp)

        pgm_path: Path = yaml_path.parent / data["image"]
        assert pgm_path.is_file(), f".pgm file at {pgm_path} does not exist"

        with open(yaml_path.parent / data["image"], "rb") as pgmf:
            grid = plt.imread(pgmf)

        return OccupancyGrid(
            resolution=data["resolution"], origin=np.asarray(data["origin"]), grid=grid
        )


def grid_to_point_cloud(occupancy_grid: OccupancyGrid) -> np.ndarray:
    """
    Convert an OccupancyGrid's occupied cells to a 2D point cloud.

    Uses the center of the cell for the conversion to reduce max error.
    """
    occupied_cells_indices = np.asarray(
        np.where(occupancy_grid.grid == 0)
    )  # in ROS maps 0 == occupied.
    res = occupancy_grid.resolution

    # Discretized occupied cells using the center of the cell.
    points = occupied_cells_indices * res + np.ones_like(occupied_cells_indices) * (
        res / 2
    )
    # Compensate for origin
    points[0] += occupancy_grid.origin[0]
    points[1] += occupancy_grid.origin[1]
    return points


def fit_normal_distribution(points: np.ndarray) -> Optional[NormalDistribution]:
    """Fit a normal distribution to a set of 2D points."""
    assert points.shape[0] == 2
    # Literature suggests doing this check to avoid singularities.
    # See The Three-Dimensional Normal-Distributions Transform– an Efficient
    # Representation for Registration, Surface Analysis, and Loop Detection
    # by Martin Magnusson, 2009 chapter 6.
    if points.shape[1] <= 5:
        return None
    mean = points.mean(axis=1)
    covariance = np.cov(points)

    # avoid singularities by enforcing a minimum variance in both dimensions.
    covariance[0, 0] = max(covariance[0, 0], 5e-3)
    covariance[1, 1] = max(covariance[1, 1], 5e-3)

    return NormalDistribution(mean=mean, covariance=covariance)


def point_cloud_to_ndt(pc: np.ndarray, cell_size=1.0) -> NDTMap:
    """
    Convert a 2D point cloud into a NDT map representation.

    Does so by clustering points in 2D cells of {cell_size} * {cell_size} meters^2
    and fitting a normal distribution when applicable.
    """
    assert pc.shape[0] == 2
    points_clusters: Dict[DiscreteCell, np.ndarray] = {}
    discretized_points = np.floor(pc / cell_size).astype(np.int64)
    cells = np.unique(discretized_points, axis=-1).T

    for cell in cells:
        pts_in_cell = np.all(discretized_points.T == cell, axis=1)
        points_clusters[DiscreteCell(x=cell[0], y=cell[1])] = pc[:, pts_in_cell]

    ret = NDTMap(resolution=cell_size)

    for cell, points in points_clusters.items():
        if (dist := fit_normal_distribution(points)) is not None:
            ret.add_distribution(cell=cell, ndt=dist)
    return ret
