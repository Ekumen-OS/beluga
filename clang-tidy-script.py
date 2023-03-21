#!/usr/bin/env python3

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

from argparse import ArgumentParser
import glob
from pathlib import Path
import subprocess
import sys


def create_argument_parser():
    arg_parser = ArgumentParser(description='run clang-tidy')
    arg_parser.add_argument(
        '--all',
        '-a',
        action='store_true',
        help=(
            'If using this option, the script must be run in the top level directory of one '
            'package.'
        ),
    )
    arg_parser.add_argument(
        'files', nargs='*', type=Path, help='Files to be analyzed by clang-tidy.'
    )
    arg_parser.add_argument(
        '--build-directory',
        '-b',
        type=Path,
        required=True,
        help='Build directory containing compile_commands.json of package being analyzed',
    )
    return arg_parser


def main(args):
    if args.all is True:
        files = glob.glob('**/*.hpp', recursive=True)
        files.extend(glob.glob('**/*.cpp', recursive=True))
        files = [Path(file) for file in files]
    else:
        files = args.files
    if not files:
        print('WARN: No files to check', file=sys.stderr)
        return 0
    compile_commands = args.build_directory / 'compile_commands.json'
    if not compile_commands.exists() or not compile_commands.is_file():
        print(
            'WARN: no compile_commands.json in built directory you may have not built the '
            'package or forgot to pass CMAKE_EXPORT_COMPILE_COMMANDS=ON',
            file=sys.stderr,
        )
        return 0
    result = subprocess.run(
        [
            'clang-tidy',
            f'-p={args.build_directory}',
            '--fix',
            "-header-filter=.*",
            "--quiet",
            "-extra-arg=-std=c++17",
            *files,
        ],
        capture_output=True,
    )
    if result.returncode != 0:
        print(
            '-------------------------------------------------------------------------------'
        )
        print('clang-tidy failed, output:')
        print(
            '-------------------------------------------------------------------------------'
        )
        print(result.stdout)
        print(
            '-------------------------------------------------------------------------------'
        )
        print('clang-tidy stderr:')
        print(
            '-------------------------------------------------------------------------------'
        )
        print(result.stderr)
        print(
            '-------------------------------------------------------------------------------'
        )
        return result.returncode
    return 0


if __name__ == '__main__':
    arg_parser = create_argument_parser()
    args = arg_parser.parse_args()
    main(args)
