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

"""Tests for launch/launch_utils.py."""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'launch'))
from launch_utils import load_yaml_file  # noqa: E402, I100


def test_valid_yaml(tmp_path):
    path = tmp_path / 'ok.yaml'
    path.write_text('a: 1\nb: [2, 3]\n')
    assert load_yaml_file(str(path)) == {'a': 1, 'b': [2, 3]}


def test_missing_file(tmp_path):
    path = tmp_path / 'missing.yaml'
    with pytest.raises(RuntimeError, match='missing.yaml'):
        load_yaml_file(str(path))


def test_empty_file(tmp_path):
    path = tmp_path / 'empty.yaml'
    path.write_text('')
    with pytest.raises(RuntimeError, match='empty'):
        load_yaml_file(str(path))


def test_invalid_yaml(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text('a: [1, 2\n')
    with pytest.raises(RuntimeError, match='Invalid YAML'):
        load_yaml_file(str(path))
