# Copyright 2026 Berkan Tali
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

"""Shared helpers for hold_and_weld_bringup launch files."""

import os

from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml_file(absolute_file_path):
    """
    Load a YAML file by absolute path.

    Raise RuntimeError naming the path if the file is missing, unreadable,
    not valid YAML, or empty.
    """
    try:
        with open(absolute_file_path, 'r') as file:
            data = yaml.safe_load(file)
    except EnvironmentError as exc:
        raise RuntimeError(f"Could not read YAML file '{absolute_file_path}'") from exc
    except yaml.YAMLError as exc:
        raise RuntimeError(f"Invalid YAML in '{absolute_file_path}': {exc}") from exc
    if data is None:
        raise RuntimeError(f"YAML file '{absolute_file_path}' is empty")
    return data


def load_yaml(package_name, file_path):
    """
    Load a YAML file from a package share directory.

    Raise RuntimeError on the same conditions as load_yaml_file.
    """
    package_share = get_package_share_directory(package_name)
    return load_yaml_file(os.path.join(package_share, file_path))
