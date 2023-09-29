from setuptools import setup

package_name = 'crazyflie_ros2_driver'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/crazyflie_ros2_driver.launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/crazyflie_arena.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/crazyflie.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
            'crazyflie_driver = crazyflie_ros2_driver.crazyflie_driver:main'
        ],
    },
)
