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
from itertools import cycle

from . import timem_results
from .exceptions import ScriptError

DIR_NAME_PATTERN = 'benchmark_([0-9]+)_particles_output'
SUPPORTED_TOPIC_NAMES = ('/pose', '/amcl_pose')
EVO_RESULTS_FILE_NAME = 'evo_results.zip'
TERMINAL_OUTPUT_LOG_FILE_NAME = 'output.log'


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


def parse_latency_data(output_file_path):
    import re

    # Example line:
    pattern = re.compile(
        r'\[.*\] \[INFO\] \[([0-9]*\.[0-9]*)\] \[amcl\]: Particle filter update iteration stats: '
        + r'[0-9]* particles [0-9]* points - ([0-9]*\.[0-9]*)ms'
    )
    latencies_seq = []
    with open(output_file_path, 'r') as f:
        for line in f:
            if not pattern.match(line):
                continue
            # first match is the whole line, second match the timestamp, and third the latency
            latency = pattern.match(line).group(2)
            latencies_seq.append(float(latency) * 1e-3)  # convert to seconds
    return (
        {
            'latency_min': min(latencies_seq),
            'latency_max': max(latencies_seq),
            'latency_mean': sum(latencies_seq) / len(latencies_seq),
            'latency_median': sorted(latencies_seq)[len(latencies_seq) // 2],
        }
        if latencies_seq
        else {
            'latency_min': 0.0,
            'latency_max': 0.0,
            'latency_mean': 0.0,
            'latency_median': 0.0,
        }
    )


def create_parameterized_series(results_path: Path):
    particles = []
    peak_rss = []
    cpu_usage = []
    ape_rmse = []
    ape_mean = []
    ape_std = []
    ape_median = []
    ape_max = []
    latency_min = []
    latency_max = []
    latency_mean = []
    latency_median = []

    for dir in results_path.iterdir():
        if not dir.is_dir():
            continue
        m = re.match(DIR_NAME_PATTERN, dir.name)
        if not m:
            continue
        particles.append(int(m[1]))
        timem_results_output = timem_results.read_timem_output(dir)
        rss, cpu = (
            timem_results.get_timem_metrics_values(timem_results_output)
            if timem_results_output
            else (None, None)
        )
        peak_rss.append(rss)
        cpu_usage.append(cpu)
        run_evo_ape(dir)
        evo_results_path = dir / EVO_RESULTS_FILE_NAME
        stats = read_evo_stats(evo_results_path)
        ape_rmse.append(stats["rmse"])
        ape_max.append(stats["max"])
        ape_mean.append(stats["mean"])
        ape_std.append(stats["std"])
        ape_median.append(stats["median"])
        terminal_output_path = dir / TERMINAL_OUTPUT_LOG_FILE_NAME
        latency_stats = parse_latency_data(terminal_output_path)
        latency_min.append(latency_stats['latency_min'])
        latency_max.append(latency_stats['latency_max'])
        latency_mean.append(latency_stats['latency_mean'])
        latency_median.append(latency_stats['latency_median'])

    return (
        pd.DataFrame(
            {
                'particles_n': particles,
                'peak_rss': peak_rss,
                'cpu_usage': cpu_usage,
                'ape_rmse': ape_rmse,
                'ape_mean': ape_mean,
                'ape_std': ape_std,
                'ape_median': ape_median,
                'ape_max': ape_max,
                'latency_min': latency_min,
                'latency_max': latency_max,
                'latency_mean': latency_mean,
                'latency_median': latency_median,
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
        '--series',
        '-s',
        type=Path,
        action='append',
        help='Folder with parameterized benchmark results',
        required=True,
    )
    arg_parser.add_argument(
        '--label',
        '-l',
        type=str,
        action='append',
        help='Label for a series',
        required=True,
    )

    arg_parser.add_argument(
        '--use-ylog',
        action='store_true',
        help='Use log scale on y axis',
    )

    default_plots = ['cpu_usage', 'peak_rss', 'ape_rmse', 'latency_median']
    arg_parser.add_argument(
        '--plot-names',
        type=str,
        nargs='*',  # 0 or more values expected => creates a list
        default=default_plots,
        help='List of plots to generate. Default: ' + ', '.join(default_plots),
    )

    arg_parser.add_argument(
        '--save-csv',
        type=str,
        action='store',
        help='Instead of plotting, save all the results to a csv file',
    )

    args = arg_parser.parse_args()

    assert len(args.series) == len(args.label), 'Number of series and labels must match'

    color_gen = cycle(
        [
            'red',
            'green',
            'blue',
            'brown',
            'purple',
            'gray',
            'orange',
        ]
    )

    if args.save_csv:
        # create list of dataframes with no filtered columns
        series = [
            create_parameterized_series(series).add_prefix(label + '_')
            for label, series in zip(args.label, args.series)
        ]
        # leave only the index
        full_table = series[0].filter(items=[])
        # concatenate data in a single table
        for s in series:
            full_table = pd.concat([full_table, s], axis=1)
        print('Saving to CSV file: ', args.save_csv)
        full_table.to_csv(args.save_csv)
        return 0

    series = [
        create_parameterized_series(series)
        .filter(items=args.plot_names)
        .add_prefix(label + '_')
        for label, series in zip(args.label, args.series)
    ]

    print('Plotting data...')

    marker_gen = cycle('o^sDvP*')

    ax = series[0].plot(
        subplots=True,
        color=next(color_gen),
        marker=next(marker_gen),
        linestyle='dashed',
    )

    for other_series in series[1:]:
        other_series.plot(
            ax=ax,
            subplots=True,
            color=next(color_gen),
            marker=next(marker_gen),
            linestyle='dashed',
        )

    for ax in plt.gcf().axes:
        ax.set_xscale('log')
        if args.use_ylog:
            ax.set_yscale('log')
        ax.grid(True, which="both", ls="-")
        ax.legend(fontsize='small', loc='upper left', bbox_to_anchor=(1.01, 1))
        current_bounds = ax.get_position().bounds
        new_bounds = (0.05, *current_bounds[1:])
        ax.set_position(new_bounds)
        current_ylimits = ax.get_ylim()
        new_limits = (
            min(0.0, current_ylimits[0]),  # include zero below
            max(0.0, current_ylimits[1]),  # include zero above
        )
        ax.set_ylim(new_limits)

    plt.show()
