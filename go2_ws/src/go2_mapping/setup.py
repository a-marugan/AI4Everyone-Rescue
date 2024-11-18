from setuptools import setup

package_name = 'go2_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),  # Explicitly include package.xml
        ('share/' + package_name + '/launch', ['launch/go2_mapping.launch.py']),
        ('share/' + package_name + '/config', ['config/mapper_params_online_sync.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Mapping setup for the Go2 robot using LiDAR data.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
