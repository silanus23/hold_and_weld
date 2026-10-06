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

"""Base for the validated parameter dataclasses every pipeline stage builds from the config."""

from dataclasses import fields
from typing import Any

import numpy as np


class ParamsBase:
    """Build a params dataclass from a loosely typed config dict."""

    @classmethod
    def from_dict(cls, params: dict[str, Any] | None):
        """Build from a config dict, coercing types and ignoring foreign keys.

        Foreign keys are ignored rather than rejected because the pipeline
        hands ONE dict to both the extractor and PathCreator, so each
        necessarily sees the other's keys. Values arrive from YAML, so the
        declared field type does the coercion.

        Args:
            params: Config dict, or None for all defaults.

        Returns:
            An instance with declared defaults for everything not supplied.

        Raises:
            ValueError: If params is not a dict, a value cannot be coerced,
                or a value fails a constraint.
        """
        if params is not None and not isinstance(params, dict):
            raise ValueError(
                f'params must be a dict, got {type(params).__name__}')
        given = params or {}
        taken = {}
        for spec in fields(cls):
            if spec.name not in given:
                continue
            taken[spec.name] = cls._coerce(spec.name, spec.type,
                                           given[spec.name])
        return cls(**taken)

    @staticmethod
    def _coerce(name: str, declared: Any, value: Any) -> Any:
        """Coerce one config value to a finite number of its declared type."""
        # float(True) is 1.0, so a YAML `yes` would otherwise pass as a number.
        if isinstance(value, bool):
            raise ValueError(f'{name} must be a number, got {value!r}')
        try:
            number = float(value)
        except (TypeError, ValueError) as e:
            raise ValueError(
                f'{name} must be a number, got {value!r}') from e
        # YAML's .nan and .inf coerce cleanly, and nan passes every `< 0` style check, so they
        # would otherwise switch features off silently.
        if not np.isfinite(number):
            raise ValueError(f'{name} must be finite, got {value!r}')
        if declared is int or declared == 'int':
            if not number.is_integer():
                raise ValueError(
                    f'{name} must be a whole number, got {value!r}')
            return int(number)
        return number
