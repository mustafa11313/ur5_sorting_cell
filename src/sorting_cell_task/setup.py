from glob import glob

from setuptools import setup

package_name = 'sorting_cell_task'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mustafa Almohamad',
    maintainer_email='mustafa11313@users.noreply.github.com',
    description='Behavior-tree task logic, planning scene and metrics for the UR5 sorting cell.',
    license='AGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sorting_task = sorting_cell_task.sorting_task:main',
            'planning_scene = sorting_cell_task.planning_scene:main',
            'metrics_node = sorting_cell_task.metrics_node:main',
        ],
    },
)
