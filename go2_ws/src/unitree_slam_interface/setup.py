from setuptools import setup

package_name = 'unitree_slam_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],  # Leave empty since we're not packaging additional Python modules
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Interface for Unitree SLAM services',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_mapping_client = unitree_slam_interface.slam_mapping_client:main',  # Registers the executable
        ],
    },
)
