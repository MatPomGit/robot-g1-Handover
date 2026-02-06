from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'g1_pick_and_handover'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot G1 Handover Team',
    maintainer_email='contact@robotg1handover.org',
    description='System przekazywania obiektów (handover) dla robota Unitree G1',
    license='MIT',
    entry_points={
        'console_scripts': [
            'object_detector = perception.object_detector:main',
            'pose_estimator_6d = perception.pose_estimator_6d:main',
            'human_hand_detector = perception.human_hand_detector:main',
            'static_tf_camera = perception.static_tf_camera:main',
            'execute_grasp = manipulation.execute_grasp:main',
            'execute_handover_wma = manipulation.execute_handover:main',
        ],
    },
)
