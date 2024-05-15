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

"""Library that provides utilities to read PLY file contents into Numpy arrays.

See: https://paulbourke.net/dataformats/ply/ for details on the PLY format.
"""

from dataclasses import dataclass
from typing import Optional, List, BinaryIO

import numpy as np


# Map to convert from PLY valid property types to numpy dtypes.
DTYPE_MAP = {
    'int8': 'i1',
    'char': 'i1',
    'uint8': 'u1',
    'uchar': 'u1',
    'int16': 'i2',
    'short': 'i2',
    'uint16': 'u2',
    'ushort': 'u2',
    'int32': 'i4',
    'int': 'i4',
    'uint32': 'u4',
    'uint': 'u4',
    'float32': 'f4',
    'float': 'f4',
    'float64': 'f8',
    'double': 'f8',
}

# Map to convert from PLY formats to numpy byte order prefixes.
DTYPE_BYTE_ORDER_MAP = {
    'binary_big_endian': '>',
    'binary_little_endian': '<',
}


@dataclass
class Element:
    name: str
    count: int
    property_names: List[str]
    property_types: List[str]


def read_ply_header(file: BinaryIO):
    """Read and validate the contents of the header of a PLY file."""
    assert file.readline().rstrip() == b'ply', 'Missing ply at the start of the file'

    keyword, format, version = file.readline().decode().split()

    assert keyword == "format", 'Missing format keyword'
    assert format in ['ascii', 'binary_little_endian', 'binary_big_endian']
    assert version == '1.0'

    elements: List[Element] = []
    element: Optional[Element] = None

    for line in file:
        if line.rstrip() == b'end_header':
            break

        keyword, *values = line.decode().split()

        if keyword == 'comment':
            continue  # Ignore comments

        if keyword == 'element':
            name, count = values
            assert name in ['vertex', 'face', 'edge']

            element = Element(
                name=name,
                count=int(count),
                property_names=[],
                property_types=[],
            )
            elements.append(element)

        if keyword == 'property':
            assert element is not None
            assert values[0] != "list", '`list` property not supported'

            type, value = values
            element.property_names.append(value)
            element.property_types.append(type)

    return format, elements


def read_ply(file: BinaryIO):
    """Generate tuples containing element names and data from a PLY file.

    Element names could be: vertex, face, edge.
    Element data will be numpy arrays with field names.
    """
    format, elements = read_ply_header(file)

    if 'ascii' == format:
        for element in elements:
            yield element.name, np.genfromtxt(
                file,
                delimiter=' ',
                names=element.property_names,
                max_rows=element.count,
            )

    if 'binary' in format:
        for element in elements:
            yield element.name, np.fromfile(
                file,
                dtype=list(
                    zip(
                        element.property_names,
                        [
                            DTYPE_BYTE_ORDER_MAP[format] + DTYPE_MAP[type]
                            for type in element.property_types
                        ],
                    )
                ),
                count=element.count,
            )
