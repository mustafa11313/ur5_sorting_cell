from glob import glob

from setuptools import setup

package_name = 'sorting_cell_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models', glob('models/*.pt') + glob('models/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ali',
    maintainer_email='ali@example.com',
    description='Detection, classification and pose estimation for the UR5 sorting cell.',
    license='AGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = sorting_cell_perception.detector_node:main',
            'dataset_recorder = sorting_cell_perception.dataset_recorder:main',
            'train_yolo = sorting_cell_perception.train_yolo:main',
            'calibrate_camera = sorting_cell_perception.calibrate_camera:main',
        ],
    },
)
