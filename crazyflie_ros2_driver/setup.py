from xml.etree.ElementPath import find
from setuptools import setup, find_packages

package_name = 'crazyflie_ros2_driver'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/crazyflie_webots_driver.launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/crazyflie_hw_driver.launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/crazyflie_arena.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/complete_apartment.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/crazyflie.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/ModifiedCrazyflie.proto']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alboalby00',
    maintainer_email='ge87cuf@mytum.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crazyflie_webots_driver = crazyflie_ros2_driver.crazyflie_webots_driver:main',
            "esp32_driver = crazyflie_ros2_driver.esp32_driver:main",
            'crazyflie_hw_attitude_driver = crazyflie_ros2_driver.crazyflie_hw_attitude_driver:main',
            'crazyflie_hw_motor_vel_driver = crazyflie_ros2_driver.crazyflie_hw_motor_vel_driver:main'
            
        ],
    },
)
