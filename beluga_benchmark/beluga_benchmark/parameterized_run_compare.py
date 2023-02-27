# Copyright 2023 Ekumen, Inc.
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

import argparse
from pathlib import Path
import re

from evo import main_ape
from evo.tools import file_interface
import matplotlib.pyplot as plt
from rosbag2_py._info import Info as Bag2Info
import pandas as pd

from . import timem
from .exceptions import ScriptError

DIR_NAME_PATTERN = 'benchmark_([0-9]+)_particles_output'
SUPPORTED_TOPIC_NAMES = ('/pose', '/amcl_pose')
EVO_RESULTS_FILE_NAME = 'evo_results.zip'


def get_bag_est_topic(bag_path: Path):
    metadata = Bag2Info().read_metadata(str(bag_path), '')
    for topic_name in SUPPORTED_TOPIC_NAMES:
        if any(
            topic_info.topic_metadata.name == topic_name
            for topic_info in metadata.topics_with_message_count
        ):
            return topic_name
    raise ScriptError(
        f'Estimate pose topic was not found in bag file, expected names: {SUPPORTED_TOPIC_NAMES}'
    )


def run_evo_ape(dir_path: Path):
    evo_results_path = dir_path / EVO_RESULTS_FILE_NAME
    if evo_results_path.exists():
        return
    bag_path = dir_path / 'rosbag'
    est_topic = get_bag_est_topic(bag_path)
    arg_parser = main_ape.parser()
    args = arg_parser.parse_args(
        [
            'bag2',
            str(bag_path),
            '/odometry/ground_truth',
            est_topic,
            '--save_results',
            str(evo_results_path.resolve()),
        ]
    )
    main_ape.run(args)


def read_evo_stats(zip_file_path):
    # keys: "rmse", "mean", "median", "std", "min", "max", "sse"
    return file_interface.load_res_file(zip_file_path).stats


def create_parameterized_series(results_path: Path):
    particles = []
    peak_rss = []
    cpu_usage = []
    ape_rmse = []
    ape_mean = []
    ape_median = []
    ape_max = []

    for dir in results_path.iterdir():
        if not dir.is_dir():
            continue
        m = re.match(DIR_NAME_PATTERN, dir.name)
        if not m:
            continue
        particles.append(int(m[1]))
        timem_output = timem.read_timem_output(dir)
        rss, cpu = timem.get_timem_metrics_values(timem_output)
        peak_rss.append(rss)
        cpu_usage.append(cpu)
        run_evo_ape(dir)
        evo_results_path = dir / EVO_RESULTS_FILE_NAME
        stats = read_evo_stats(evo_results_path)
        ape_rmse.append(stats["rmse"])
        ape_max.append(stats["max"])
        ape_mean.append(stats["mean"])
        ape_median.append(stats["median"])
    return (
        pd.DataFrame(
            {
                'particles_n': particles,
                'peak_rss': peak_rss,
                'cpu_usage': cpu_usage,
                'ape_rmse': ape_rmse,
                'ape_mean': ape_mean,
                'ape_median': ape_median,
                'ape_max': ape_max,
            }
        )
        .set_index('particles_n')
        .sort_index()
    )


def main():
    arg_parser = argparse.ArgumentParser(
        description='Script to compare parameterized run results of beluga with another'
        ' implementation (e.g. nav2_amcl, another beluga version, etc).'
    )
    arg_parser.add_argument(
        '--beluga-results',
        '-b',
        type=Path,
        help='Folder with parameterized beluga benchmark results',
    )
    arg_parser.add_argument(
        '--other-results',
        '-o',
        type=Path,
        help='Folder with parameterized benchmark results for the other implementation',
    )
    args = arg_parser.parse_args()
    beluga_series = create_parameterized_series(args.beluga_results).add_prefix(
        'beluga_'
    )
    other_series = create_parameterized_series(args.other_results).add_prefix('other_')

    ax = beluga_series.plot(subplots=True, color='red')
    other_series.plot(ax=ax, subplots=True, color='blue')
    for ax in plt.gcf().axes:
        ax.legend(fontsize='small', loc='upper left', bbox_to_anchor=(1.01, 1))
        current_bounds = ax.get_position().bounds
        new_bounds = (0.05, *current_bounds[1:])
        ax.set_position(new_bounds)
    plt.show()
