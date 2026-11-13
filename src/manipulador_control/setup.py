import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'manipulador_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['yolo/pesos/*.pt', 'calibracao/*.npz'],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joao_pedro',
    maintainer_email='joao_pedro@todo.todo',
    description='Controle do manipulador Arctos via GRBL, planejamento Bezier e interface ROS 2 (RViz2).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ros2_node = manipulador_control.ros2_node:main',
        ],
    },
)
