from setuptools import find_packages, setup

package_name = 'bno055_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/imu.launch.py']),
        ('share/' + package_name + '/config',
            ['config/imu.yaml']),

        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ydlidar',
    maintainer_email='ydlidar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'imu_serial_node = bno055_imu.imu_serial_node:main',
        ],
    },
)
