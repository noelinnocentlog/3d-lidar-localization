from setuptools import setup
from glob import glob
import os
package_name = 'opensource_2d_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/opensource_2d_mapping']
        ),
        (
            'share/opensource_2d_mapping',
            ['package.xml']
        ),
        (
            os.path.join(
                'share',
                'opensource_2d_mapping',
                'launch'
            ),
            glob('launch/*.py')
        ),
        (
            os.path.join(
                'share',
                'opensource_2d_mapping',
                'config'
            ),
            glob('config/*')
        ),
    ],
    scripts=[
        'scripts/save_map.py',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Noel',
    maintainer_email='your@email.com',
    description='Open Source 2D Mapping Package',
    license='Apache-2.0',
    tests_require=['pytest'],
)