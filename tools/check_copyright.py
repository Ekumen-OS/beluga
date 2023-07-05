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

"""Check that files have a copyright notice.

This tool checks that a copyright notice matching a specific template exists in the
provided files. It can also fix the file by adding a copyright notice or replacing
an existing one.
"""

import argparse
import datetime
import re
import textwrap

from pathlib import Path

DEFAULT_COPYRIGHT_HOLDER = 'Ekumen, Inc.'

# Template for the copyright notice.
# This should not contain any regex patterns since special characters will be escaped.
DEFAULT_COPYRIGHT_TEMPLATE = '''\
Copyright {year} {name}

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.\
'''


class FileContentChecker:
    """Helper class to check the contents of a file and apply fixes."""

    # Regex pattern that matches a year (YYYY) and a year range (YYYY-ZZZZ).
    # It has capturing groups for both years (the second one is optional).
    _YEAR_PATTERN = r'([0-9]{4})(?:-([0-9]{4}))?'

    # Regex pattern that matches the name of a copyright holder.
    _NAME_PATTERN = r'.*'

    # Regex pattern that matches anything that looks like a copyright block.
    # Specifically, a comment beginning with the word `copyright` (case insensitive)
    # followed by single line comments that form a block.
    _COPYRIGHT_PATTERN = r'(?:[;#%|*]|//)\s*[Cc]opyright.*(?:\n(?:[;#%|*]|//).*)*'

    def __init__(self, path: Path):
        """Initialize a file content checker."""
        self._path = path
        with open(path) as file:
            self._content = file.read()

    def _comment_lines(self, text: str) -> str:
        """Comment text lines using the syntax associated with the file extension.

        :param text: Input text.
        :return: The modified text.
        """
        comment_prefix = {
            '.py': '#',
            '.cpp': '//',
            '.hpp': '//',
        }

        base = textwrap.indent(text, ' ', None)
        return textwrap.indent(
            base, comment_prefix.get(self._path.suffix, '#'), lambda line: True
        )

    def has_copyright(self, template: str = DEFAULT_COPYRIGHT_TEMPLATE) -> bool:
        """Verify that the file has a valid copyright notice.

        :param template: Template copyright notice.
        :return: True if a copyright notice was found. False otherwise.
        """
        pattern = (
            # Comment lines and escape special characters in the template...
            re.escape(self._comment_lines(template.format(year='YYYY', name='NAME')))
            # Then insert year and name patterns.
            .replace('YYYY', self._YEAR_PATTERN).replace('NAME', self._NAME_PATTERN)
        )
        return bool(re.search(pattern, self._content))

    def fix_copyright(
        self,
        template: str = DEFAULT_COPYRIGHT_TEMPLATE,
        name: str = DEFAULT_COPYRIGHT_HOLDER,
        year: int = datetime.date.today().year,
    ) -> None:
        """Fix copyright notice.

        Searches for a copyright notice block and replaces it with the template.
        If no copyright block is found, adds the notice at the top of the
        file preserving the shebang line if it exists.

        The new copyright notice will be preceded by an empty line if not at
        the beginning of the file, and will always be followed by an empty line.

        :param template: Template copyright notice.
        :param name: Name of the copyright holder.
        :param year: Copyright year.
        """

        def fix_content() -> str:
            """Return the fixed file content."""
            copyright_notice = self._comment_lines(
                template.format(year=year, name=name)
            )

            match = re.search(self._COPYRIGHT_PATTERN, self._content)
            if match:
                # Found something that looks like a copyright block...
                # Replace it with the template.
                return self._content.replace(match.group(0), copyright_notice)

            lines = self._content.splitlines()
            if lines and lines[0].startswith('#!'):
                # Keep shebang line at the top.
                lines.insert(1, '\n' + copyright_notice + '\n')
                return '\n'.join(lines)

            return copyright_notice + '\n\n' + self._content

        with open(self._path, 'w') as file:
            self._content = fix_content()
            file.write(self._content)


def main(argv=None) -> int:
    """Run the entry point of the program."""
    parser = argparse.ArgumentParser(description=globals()['__doc__'])
    parser.add_argument(
        'paths',
        type=Path,
        nargs='+',
        help='files to check',
    )
    parser.add_argument(
        '-f',
        '--fix',
        action='store_true',
        help='whether to apply the detected fixes',
    )
    parser.add_argument(
        '-n',
        '--copyright-holder-name',
        type=str,
        default=DEFAULT_COPYRIGHT_HOLDER,
        help='copyright holder name',
        metavar='NAME',
    )

    args = parser.parse_args(argv)

    # Whether any of the files need to be fixed.
    need_fix: bool = False

    for path in args.paths:
        checker = FileContentChecker(path)
        if not checker.has_copyright():
            need_fix = True
            print(f'File at `{path}` does not have a valid copyright notice')
            if args.fix:
                checker.fix_copyright(name=args.copyright_holder_name)

    return 0 if not need_fix else 1


if __name__ == '__main__':
    exit(main())
