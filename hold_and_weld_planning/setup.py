# Copyright 2025 Berkan Tali
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

from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'hold_and_weld_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/generated', glob('generated/*.json')),
        ('share/' + package_name + '/config/examples', []),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'pyyaml'
    ],
    zip_safe=True,
    maintainer='Berkan Tali',
    maintainer_email='berkantali23@outlook.com',
    description='Weld seam trajectory generation and geometry processing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'seam_generator = hold_and_weld_planning.scripts.seam_generator:main',
        ],
    },
)
