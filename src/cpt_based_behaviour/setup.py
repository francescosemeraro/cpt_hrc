from setuptools import find_packages, setup

package_name = 'cpt_based_behaviour'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='francesco',
    maintainer_email='francesco.semeraro@manchester.ac.uk',
    description='This package detects the markers in the scene and predicts the next action to perform on the collaborative assembly',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fiducial_marker_detector = cpt_based_behaviour.fiducial_marker_detector:main',
            'next_step_predictor = cpt_based_behaviour.next_step_predictor:main',
            'panda_motion_controller = cpt_based_behaviour.panda_motion_controller:main'
        ],
    },
)
