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
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from .exceptions import ScriptError

DEFAULT_FILE_NAME = "timem-output.json"


def read_timem_output(dir_path: Path):
    file = dir_path / DEFAULT_FILE_NAME
    try:
        with open(file, mode='r') as f:
            output = json.load(f)
    except (FileExistsError, FileNotFoundError) as ex:
        raise ScriptError(f"Failed to open file '{file.absolute()}': {ex}")
    except json.decoder.JSONDecodeError:
        return None
    return output['timemory']['timem'][0]


def get_value_with_unit(data):
    return f"{data['value']:.2f}{data['unit_repr']}"


def get_timem_metrics(output):
    return output['peak_rss'], output['cpu_util']


def get_timem_metrics_values(output):
    return tuple(data['value'] for data in get_timem_metrics(output))


def get_timem_metrics_with_units(output):
    return tuple(get_value_with_unit(data) for data in get_timem_metrics(output))


def create_timeseries(output):
    data = output['wall_clock']
    elapsed_time = data['value']
    time = np.arange(0.0, elapsed_time, 0.2)
    wall_clock_to_time_offset = output['history'][0]['wall_clock']['value']
    wall_clock_to_time_scale = elapsed_time / (
        output['history'][-1]['wall_clock']['value'] - wall_clock_to_time_offset
    )
    time = []
    rss = []
    virtual_memory = []
    for sample in output['history']:
        time.append(
            (sample['wall_clock']['value'] - wall_clock_to_time_offset)
            * wall_clock_to_time_scale
        )
        rss.append(sample['page_rss']['value'])
        virtual_memory.append(sample['virtual_memory']['value'])

    series = pd.DataFrame(
        {'time': time, 'rss': rss, 'virtual_memory': virtual_memory}
    ).set_index('time')
    return series


def main():
    arg_parser = argparse.ArgumentParser(
        description='Script to postprocess timem results.'
    )
    arg_parser.add_argument(
        'dir', help='Directory with timem-output.json file', type=Path
    )
    args = arg_parser.parse_args()
    dir_path = Path(args.dir)
    name = dir_path.name
    output = read_timem_output(dir_path)
    peak_rss_str, cpu_usage_str = get_timem_metrics_with_units(output)
    series = create_timeseries(output)
    elapsed_time_str = get_value_with_unit(output['wall_clock'])
    print(f"timem metrics for run '{name}':")
    print(f"\telapsed_time: {elapsed_time_str}")
    print(f"\tcpu_usage: {cpu_usage_str}")
    print(f"\tpeak_rss: {peak_rss_str}")
    series.plot(subplots=True)
    plt.show()
