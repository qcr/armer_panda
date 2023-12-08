#!/usr/bin/env python3
"""
.. codeauthor:: Gavin Suddreys
.. codeauthor:: Dasun Gunasinghe
"""
import os
from glob import glob
from typing import List
from setuptools import setup, find_packages

here = os.path.abspath(os.path.dirname(__file__))

req = [
    'armer'
]

# list all data folders here, to ensure they get packaged
data_folders = [
]

def package_files(directory: List[str]) -> List[str]:
    """[summary]

    :param directory: [description]
    :type directory: [type]
    :return: [description]
    :rtype: [type]
    """
    paths: List[str] = []
    for (pathhere, _, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join('..', pathhere, filename))
    return paths


extra_files = []
for data_folder in data_folders:
    extra_files += package_files(data_folder)

setup(
    name='armer_panda',
    version='0.2.0',
    description='Armer Panda - The ROS Arm drivEr for the Franka-Emika Panda',
    long_description_content_type='text/markdown',
    url='https://github.com/qcr/armer',
    author='Gavin Suddrey',
    maintainer='Dasun Gunasinghe, Timothy Morris',
    maintainer_email='dasun.gunasinghe@qut.edu.au, timothy.morris@qut.edu.au',
    license='BSD3',
    classifiers=[
        #   3 - Alpha
        #   4 - Beta
        #   5 - Production/Stable
        'Development Status :: 3 - Alpha',

        # Indicate who your project is intended for
        'Intended Audience :: Developers',
        # Pick your license as you wish (should match "license" above)
        'License :: OSI Approved :: MIT License',

        # Specify the Python versions you support here. In particular, ensure
        # that you indicate whether you support Python 2, Python 3 or both.
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
    ],
    python_requires='>=3.6',
    project_urls={
        # 'Documentation': 'https://petercorke.github.io/roboticstoolbox-python',
        'Source': 'https://github.com/qcr/armer',
        'Tracker': 'https://github.com/qcr/armer/issues'#,
        # 'Coverage': 'https://codecov.io/gh/petercorke/roboticstoolbox-python'
    },
    keywords='python robotics robotics-toolbox kinematics dynamics' \
             ' motion-planning trajectory-generation jacobian hessian' \
             ' control simulation robot-manipulator mobile-robot ros',

    packages=find_packages(exclude=['tests']),
    package_data={'armer_panda': extra_files},
    include_package_data=True,
    install_requires=req,
    extras_require={},
    entry_points={
        'console_scripts': [
          'entry_point = armer_panda.entry_point:main'
        ],
    },
    data_files=[
        (os.path.join('share', 'armer_panda'), ['package.xml']),
        (os.path.join('share', 'armer_panda'), glob('launch/*.py')),
        (os.path.join('share', 'armer_panda', 'cfg'), glob(os.path.join('cfg', '*'))),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'armer_panda']),
    ],
)
