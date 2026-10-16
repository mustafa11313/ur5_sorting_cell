from glob import glob

from setuptools import setup

package_name = 'sorting_cell_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/scenarios', glob('config/scenarios/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ali',
    maintainer_email='mustafa11313@users.noreply.github.com',
    description='Launch files and configuration for the UR5 sorting cell.',
    license='AGPL-3.0-only',
    tests_require=['pytest'],
)
