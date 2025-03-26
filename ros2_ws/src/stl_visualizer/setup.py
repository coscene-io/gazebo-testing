import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'stl_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        (os.path.join('share', package_name), ['package.xml'])
    ],
    install_requires=[
        'setuptools',
        'numpy-stl',
        'numpy'
        ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stl_publisher = stl_visualizer.stl_publisher:main'
        ],
    },
)
