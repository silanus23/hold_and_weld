# Copyright 2025 Berkan Tali
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""URDF processor - handles package paths, xacro processing, and URDF parsing.

Loads a URDF or xacro file, resolving package:// URIs and expanding xacro
where needed, and exposes the parsed model as `robot`. Placing that model in
the world is the caller's job: it hands its own world transform to whichever
generator builds the geometry.
"""

import logging
from pathlib import Path

from urdf_parser_py.urdf import URDF

from ..utils.path_utils import resolve_package_path

logger = logging.getLogger(__name__)


class URDFProcessor:
    """Process URDF and xacro files.

    Handles package:// path resolution and xacro processing.

    Attributes:
        robot: The parsed urdf_parser_py URDF model.
    """

    def __init__(self, urdf_path: str | Path) -> None:
        """Initialize URDF processor with URDF or xacro file.

        Args:
            urdf_path: Path to URDF or xacro file (accepts package:// URIs).

        Raises:
            FileNotFoundError: If file doesn't exist.
            ValueError: If URDF/xacro parsing fails.
        """
        resolved_path = resolve_package_path(urdf_path)
        logger.info(f'Loading URDF from: {resolved_path}')

        urdf_string = self._process_xacro(resolved_path)

        try:
            self.robot = URDF.from_xml_string(urdf_string)
            logger.info(f'Parsed URDF: {self.robot.name}')
        except Exception as e:
            logger.error(f'Failed to parse URDF: {e}')
            raise ValueError(f'Failed to parse URDF: {e}')

    def _process_xacro(self, xacro_path: Path) -> str:
        """Process xacro files; read plain .urdf files directly without xacro."""
        if xacro_path.suffix.lower() == '.urdf':
            logger.debug(f'Reading plain URDF directly (no xacro): {xacro_path}')
            return xacro_path.read_text()

        logger.debug(f'Processing file through xacro: {xacro_path}')
        try:
            import xacro

            doc = xacro.process_file(str(xacro_path))
            return doc.toxml()
        except ModuleNotFoundError:
            raise ValueError(
                'xacro module not installed - cannot process xacro files'
            )
        except Exception as e:
            logger.error(f'Xacro processing failed: {e}')
            raise ValueError(f"Failed to process xacro '{xacro_path}': {e}")
